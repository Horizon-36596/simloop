package org.horizon36596.simloop.fit;

/**
 * Linear least squares, written out: find the {@code x} that makes {@code A x} as close as possible to
 * {@code b}, in the sense of the smallest sum of squared differences. Horizon (FTC 36596) SimLoop.
 *
 * <p><b>How.</b> By a QR factorisation built from Householder reflections, rather than by the normal
 * equations {@code (A^T A) x = A^T b}. Both give the same answer on well-behaved data; QR stays accurate
 * when two columns of {@code A} are nearly alike (a robot that barely changed speed, say), where the normal
 * equations square the problem's sensitivity and can lose half the digits. The method, in three steps:
 * <ol>
 *   <li>For each column in turn, a reflection {@code H = I - 2 v v^T / (v^T v)} is chosen that zeroes
 *       everything below the diagonal in that column. Applying the reflections to {@code A} leaves an
 *       upper-triangular {@code R}; applying the same reflections to {@code b} gives {@code Q^T b}.</li>
 *   <li>Reflections do not change lengths, so minimising {@code |A x - b|} is the same as minimising
 *       {@code |R x - Q^T b|}. Its first {@code n} rows can be made exactly zero.</li>
 *   <li>Back-substitution solves those {@code n} triangular rows for {@code x}.</li>
 * </ol>
 *
 * <p><b>How sure.</b> Each column is first scaled to unit length, so the answer does not depend on the
 * columns' units, and the diagonal of the scaled {@code R} then gives a {@link Solution#conditionNumber}: how
 * nearly one column is a combination of the others. Near 1, every coefficient is pinned down independently;
 * large, the data cannot tell some coefficients apart and small noise moves them a lot even while the
 * residual stays small. {@link Solution#standardErrors} put a number on that, assuming the rows' errors are
 * independent and equal in size: {@code sqrt(diag(s^2 (R^T R)^-1))}, with {@code s^2 = RSS / (rows - columns)}.
 *
 * <p>Pure and deterministic (domain R5): no clock, no randomness, and the arithmetic runs in a fixed order,
 * so the same data gives the same bits. No library: the repo takes no new dependency for this.
 */
public final class LeastSquares {

    /**
     * A column whose remaining length, after the earlier columns have been taken out, is below this fraction
     * of its own original length is treated as a copy of the others, and the fit refused. Measured against
     * its own length (every column is scaled to length 1 first), not the largest column's, because columns
     * come in different units. This catches only EXACT copies; a column that is merely NEARLY a combination
     * of the others passes, and shows up instead as a large {@link Solution#conditionNumber} and large
     * {@link Solution#standardErrors}, which the caller must check. SET BY HAND:
     * well above double rounding (about 1e-16) and well below anything a real regressor produces.
     */
    static final double RANK_TOLERANCE = 1e-10;

    private LeastSquares() {
    }

    /** The answer to one least-squares problem. */
    public static final class Solution {
        /** The best-fit {@code x}, one entry per column of {@code A}. */
        public final double[] coefficients;
        /** {@code b - A x} for each row, in row order. */
        public final double[] residuals;
        /** Root-mean-square of {@link #residuals}, in the units of {@code b}. */
        public final double rmsResidual;
        /**
         * {@code 1 - (residual sum of squares) / (spread of b about its mean)}: 1 is perfect. For a fit with
         * no constant column this can fall below 0, because the fit is not compared with its own best
         * constant. {@code NaN} when {@code b} is constant.
         */
        public final double rSquared;
        /**
         * One standard error per coefficient, in that coefficient's units: how far it would typically move if
         * the data were collected again with fresh, independent noise. Assumes the rows' errors are
         * independent and equal in size; rows that share noise make the true spread larger than this.
         * {@code NaN} when there are exactly as many rows as columns (no residual left to measure noise from).
         */
        public final double[] standardErrors;
        /**
         * The coefficients' covariance matrix, {@code s^2 (R^T R)^-1} in the coefficients' own units:
         * {@code covariance[i][i]} is {@code standardErrors[i]} squared, and the off-diagonal entries say how
         * the errors in two coefficients move together - needed to put an error on a quantity computed from
         * several of them. Same assumptions as {@link #standardErrors}.
         */
        public final double[][] covariance;
        /**
         * How nearly the columns are combinations of each other, after each was scaled to unit length:
         * the largest over the smallest {@code |R[j][j]|}. 1 means the columns are at right angles; 100 means
         * noise in {@code b} can be amplified about a hundredfold into the coefficients. This ratio is a
         * cheap estimate (it can read lower than the true condition number, never higher), which is enough to
         * catch data that barely separates two parameters.
         */
        public final double conditionNumber;

        Solution(double[] coefficients, double[] residuals, double rmsResidual, double rSquared,
                double[] standardErrors, double[][] covariance, double conditionNumber) {
            this.coefficients = coefficients;
            this.residuals = residuals;
            this.rmsResidual = rmsResidual;
            this.rSquared = rSquared;
            this.standardErrors = standardErrors;
            this.covariance = covariance;
            this.conditionNumber = conditionNumber;
        }
    }

    /**
     * Solve {@code min |A x - b|}.
     *
     * @param a       the design matrix, {@code a[row][column]}; not modified; at least as many rows as columns
     * @param b       the observations, one per row of {@code a}; not modified
     * @param columns a name for each column, used only to say which one is at fault if the fit is refused
     * @return the solution
     * @throws IllegalArgumentException if the shapes disagree, a value is not finite, there are fewer rows
     *                                  than columns, or a column is (numerically) a combination of the
     *                                  others - the data then cannot tell those parameters apart
     */
    public static Solution solve(double[][] a, double[] b, String... columns) {
        int rows = a.length;
        if (rows == 0 || b.length != rows) {
            throw new IllegalArgumentException("A has " + rows + " rows and b has " + b.length + "; they must match and not be zero");
        }
        int cols = a[0].length;
        if (columns.length != cols) {
            throw new IllegalArgumentException("A has " + cols + " columns but " + columns.length + " names were given");
        }
        if (rows < cols) {
            throw new IllegalArgumentException("need at least " + cols + " rows to fit " + cols + " parameters, got " + rows);
        }

        // Work on copies: r becomes R (upper triangle), qtb becomes Q^T b.
        double[][] r = new double[rows][cols];
        double[] qtb = new double[rows];
        double largestColumn = 0.0;
        for (int i = 0; i < rows; i++) {
            if (a[i].length != cols) {
                throw new IllegalArgumentException("row " + i + " of A has " + a[i].length + " entries, not " + cols);
            }
            for (int j = 0; j < cols; j++) {
                requireFinite(a[i][j], "A[" + i + "][" + j + "]");
                r[i][j] = a[i][j];
            }
            requireFinite(b[i], "b[" + i + "]");
            qtb[i] = b[i];
        }
        double[] originalLength = new double[cols];
        for (int j = 0; j < cols; j++) {
            originalLength[j] = columnLength(r, j, 0);
            largestColumn = Math.max(largestColumn, originalLength[j]);
        }
        if (largestColumn == 0.0) {
            throw new IllegalArgumentException("every column of A is zero, so there is nothing to fit");
        }
        for (int j = 0; j < cols; j++) {
            if (originalLength[j] == 0.0) {
                throw new IllegalArgumentException("column '" + columns[j] + "' is zero throughout, so there is"
                        + " nothing to fit its coefficient from. Typically the log never exercised whatever that"
                        + " column measures.");
            }
            // Scale every column to unit length. The solve below finds the coefficients of the scaled
            // columns; each is divided by its column's length at the end to get back to the caller's units.
            for (int i = 0; i < rows; i++) {
                r[i][j] /= originalLength[j];
            }
        }

        // Step 1: one Householder reflection per column.
        for (int j = 0; j < cols; j++) {
            double length = columnLength(r, j, j);
            // Every column started at length 1, so what is left of it is already a share of its own length.
            if (length <= RANK_TOLERANCE) {
                throw new IllegalArgumentException("column '" + columns[j] + "' is (numerically) a combination of the"
                        + " columns before it, so the data cannot tell it apart from them. Typically the log never"
                        + " exercised it - e.g. the mechanism never moved, or the power never changed.");
            }
            // v = x + sign(x0) |x| e0, chosen so the subtraction below never cancels.
            double alpha = r[j][j] >= 0 ? -length : length;
            double[] v = new double[rows];
            for (int i = j; i < rows; i++) {
                v[i] = r[i][j];
            }
            v[j] -= alpha;
            double vv = 0.0;
            for (int i = j; i < rows; i++) {
                vv += v[i] * v[i];
            }
            // Apply H = I - 2 v v^T / (v^T v) to every remaining column of r, and to qtb.
            for (int k = j; k < cols; k++) {
                double dot = 0.0;
                for (int i = j; i < rows; i++) {
                    dot += v[i] * r[i][k];
                }
                double scale = 2.0 * dot / vv;
                for (int i = j; i < rows; i++) {
                    r[i][k] -= scale * v[i];
                }
            }
            double dotB = 0.0;
            for (int i = j; i < rows; i++) {
                dotB += v[i] * qtb[i];
            }
            double scaleB = 2.0 * dotB / vv;
            for (int i = j; i < rows; i++) {
                qtb[i] -= scaleB * v[i];
            }
        }

        // Step 3: back-substitution on the top cols x cols triangle, for the scaled columns' coefficients,
        // then back to the caller's units.
        double[] scaledX = new double[cols];
        for (int j = cols - 1; j >= 0; j--) {
            double sum = qtb[j];
            for (int k = j + 1; k < cols; k++) {
                sum -= r[j][k] * scaledX[k];
            }
            scaledX[j] = sum / r[j][j];
        }
        double[] x = new double[cols];
        for (int j = 0; j < cols; j++) {
            x[j] = scaledX[j] / originalLength[j];
        }

        double largestDiagonal = 0.0;
        double smallestDiagonal = Double.POSITIVE_INFINITY;
        for (int j = 0; j < cols; j++) {
            largestDiagonal = Math.max(largestDiagonal, Math.abs(r[j][j]));
            smallestDiagonal = Math.min(smallestDiagonal, Math.abs(r[j][j]));
        }
        double conditionNumber = largestDiagonal / smallestDiagonal;

        // Residuals from the ORIGINAL data, so they mean exactly what they say.
        double[] residuals = new double[rows];
        double sumSquares = 0.0;
        double mean = 0.0;
        for (int i = 0; i < rows; i++) {
            double predicted = 0.0;
            for (int j = 0; j < cols; j++) {
                predicted += a[i][j] * x[j];
            }
            residuals[i] = b[i] - predicted;
            sumSquares += residuals[i] * residuals[i];
            mean += b[i];
        }
        mean /= rows;
        double spread = 0.0;
        for (int i = 0; i < rows; i++) {
            spread += (b[i] - mean) * (b[i] - mean);
        }
        double rSquared = spread == 0.0 ? Double.NaN : 1.0 - sumSquares / spread;

        // Covariance: s^2 (R^T R)^-1 = s^2 R^-1 R^-T for the scaled coefficients, where R is the top triangle.
        // R^-1 is upper-triangular too, found column by column by back-substitution against the identity.
        double noiseVariance = rows > cols ? sumSquares / (rows - cols) : Double.NaN;
        double[][] rInverse = new double[cols][cols];
        for (int column = 0; column < cols; column++) {
            for (int j = column; j >= 0; j--) {
                double sum = j == column ? 1.0 : 0.0;
                for (int k = j + 1; k <= column; k++) {
                    sum -= r[j][k] * rInverse[k][column];
                }
                rInverse[j][column] = sum / r[j][j];
            }
        }
        double[][] covariance = new double[cols][cols];
        double[] standardErrors = new double[cols];
        for (int i = 0; i < cols; i++) {
            for (int j = 0; j < cols; j++) {
                double sum = 0.0;
                for (int k = 0; k < cols; k++) {
                    sum += rInverse[i][k] * rInverse[j][k];
                }
                // Undo the column scaling: x_i = scaledX_i / length_i.
                covariance[i][j] = noiseVariance * sum / (originalLength[i] * originalLength[j]);
            }
            standardErrors[i] = Math.sqrt(covariance[i][i]);
        }
        return new Solution(x, residuals, Math.sqrt(sumSquares / rows), rSquared, standardErrors, covariance,
                conditionNumber);
    }

    /** {@return the Euclidean length of column {@code j} of {@code m}, from row {@code fromRow} down} */
    private static double columnLength(double[][] m, int j, int fromRow) {
        // Scaled sum so a column of large numbers cannot overflow before its square root is taken.
        double largest = 0.0;
        for (int i = fromRow; i < m.length; i++) {
            largest = Math.max(largest, Math.abs(m[i][j]));
        }
        if (largest == 0.0) {
            return 0.0;
        }
        double sum = 0.0;
        for (int i = fromRow; i < m.length; i++) {
            double scaled = m[i][j] / largest;
            sum += scaled * scaled;
        }
        return largest * Math.sqrt(sum);
    }

    private static void requireFinite(double value, String where) {
        if (!Double.isFinite(value)) {
            throw new IllegalArgumentException(where + " is " + value + "; least squares needs finite numbers");
        }
    }
}
