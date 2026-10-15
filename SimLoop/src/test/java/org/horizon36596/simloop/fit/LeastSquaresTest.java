package org.horizon36596.simloop.fit;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.horizon36596.simloop.field.SeededRandom;
import org.junit.jupiter.api.Test;

/**
 * Pins the solver on problems whose answer is known without it. INVARIANT tier: any correct least-squares
 * solver passes these, and a wrong one cannot.
 */
class LeastSquaresTest {

    @Test
    void invariant_anExactSystemIsSolvedExactly() {
        // y = 2 + 3 t - 0.5 t^2, sampled without noise.
        double[][] a = new double[12][];
        double[] b = new double[12];
        for (int i = 0; i < 12; i++) {
            double t = i * 0.25;
            a[i] = new double[] {1.0, t, t * t};
            b[i] = 2.0 + 3.0 * t - 0.5 * t * t;
        }

        LeastSquares.Solution fit = LeastSquares.solve(a, b, "one", "t", "t^2");

        assertEquals(2.0, fit.coefficients[0], 1e-10);
        assertEquals(3.0, fit.coefficients[1], 1e-10);
        assertEquals(-0.5, fit.coefficients[2], 1e-10);
        assertEquals(0.0, fit.rmsResidual, 1e-10);
        assertEquals(1.0, fit.rSquared, 1e-12);
    }

    /**
     * The defining property of a least-squares answer: the residual is orthogonal to every column, so no
     * nudge to any coefficient can shrink it. Checked on noisy data where the answer is not exact.
     */
    @Test
    void invariant_theResidualIsOrthogonalToEveryColumn() {
        SeededRandom random = new SeededRandom(7);
        int rows = 200;
        double[][] a = new double[rows][];
        double[] b = new double[rows];
        for (int i = 0; i < rows; i++) {
            double x1 = random.nextBetween(-1.0, 1.0);
            double x2 = random.nextBetween(0.0, 5.0);
            a[i] = new double[] {x1, x2, 1.0};
            b[i] = 4.0 * x1 - 1.5 * x2 + 0.3 + random.nextBetween(-0.2, 0.2);
        }

        LeastSquares.Solution fit = LeastSquares.solve(a, b, "x1", "x2", "one");

        for (int j = 0; j < 3; j++) {
            double dot = 0.0;
            double scale = 0.0;
            for (int i = 0; i < rows; i++) {
                dot += a[i][j] * fit.residuals[i];
                scale += Math.abs(a[i][j] * b[i]);
            }
            assertEquals(0.0, dot / scale, 1e-12, "residual . column " + j);
        }
        assertEquals(4.0, fit.coefficients[0], 0.05);
        assertEquals(-1.5, fit.coefficients[1], 0.02);
    }

    @Test
    void invariant_agreesWithTheNormalEquationsOnAWellConditionedProblem() {
        SeededRandom random = new SeededRandom(11);
        int rows = 50;
        double[][] a = new double[rows][];
        double[] b = new double[rows];
        for (int i = 0; i < rows; i++) {
            a[i] = new double[] {random.nextBetween(-2, 2), random.nextBetween(-2, 2)};
            b[i] = random.nextBetween(-3, 3);
        }
        // Normal equations for two unknowns, solved by Cramer's rule: independent of the QR code.
        double s11 = 0, s12 = 0, s22 = 0, r1 = 0, r2 = 0;
        for (int i = 0; i < rows; i++) {
            s11 += a[i][0] * a[i][0];
            s12 += a[i][0] * a[i][1];
            s22 += a[i][1] * a[i][1];
            r1 += a[i][0] * b[i];
            r2 += a[i][1] * b[i];
        }
        double det = s11 * s22 - s12 * s12;
        double x0 = (r1 * s22 - r2 * s12) / det;
        double x1 = (s11 * r2 - s12 * r1) / det;

        LeastSquares.Solution fit = LeastSquares.solve(a, b, "p", "q");

        assertEquals(x0, fit.coefficients[0], 1e-10);
        assertEquals(x1, fit.coefficients[1], 1e-10);

        // Standard errors from the textbook formula, s^2 (A^T A)^-1, inverted by hand for two unknowns.
        double rss = 0.0;
        for (int i = 0; i < rows; i++) {
            double residual = b[i] - x0 * a[i][0] - x1 * a[i][1];
            rss += residual * residual;
        }
        double noiseVariance = rss / (rows - 2);
        assertEquals(Math.sqrt(noiseVariance * s22 / det), fit.standardErrors[0], 1e-10);
        assertEquals(Math.sqrt(noiseVariance * s11 / det), fit.standardErrors[1], 1e-10);
        assertEquals(-noiseVariance * s12 / det, fit.covariance[0][1], 1e-10);
    }

    /**
     * Two columns that are nearly, but not exactly, the same pass the exact-copy check - and must then show
     * up as a large condition number and large standard errors, which is how a caller knows not to trust
     * the coefficients.
     */
    @Test
    void invariant_nearlyAlikeColumnsGiveALargeConditionNumberAndLargeErrors() {
        SeededRandom random = new SeededRandom(3);
        int rows = 200;
        double[][] apart = new double[rows][];
        double[][] alike = new double[rows][];
        double[] b = new double[rows];
        for (int i = 0; i < rows; i++) {
            double x = random.nextBetween(-1.0, 1.0);
            apart[i] = new double[] {x, random.nextBetween(-1.0, 1.0)};
            alike[i] = new double[] {x, x + random.nextBetween(-0.001, 0.001)};
            b[i] = 2.0 * x + random.nextBetween(-0.1, 0.1);
        }

        LeastSquares.Solution separated = LeastSquares.solve(apart, b, "x", "y");
        LeastSquares.Solution tangled = LeastSquares.solve(alike, b, "x", "x again");

        assertTrue(separated.conditionNumber < 2.0, "independent columns: " + separated.conditionNumber);
        assertTrue(tangled.conditionNumber > 100.0, "nearly equal columns: " + tangled.conditionNumber);
        assertTrue(tangled.standardErrors[0] > 100.0 * separated.standardErrors[0],
                separated.standardErrors[0] + " vs " + tangled.standardErrors[0]);
    }

    /** Scaling a column (changing its units) scales its coefficient and error, and changes nothing else. */
    @Test
    void invariant_theAnswerDoesNotDependOnAColumnsUnits() {
        SeededRandom random = new SeededRandom(5);
        int rows = 60;
        double[][] a = new double[rows][];
        double[][] scaled = new double[rows][];
        double[] b = new double[rows];
        for (int i = 0; i < rows; i++) {
            double p = random.nextBetween(-1, 1);
            double q = random.nextBetween(-1, 1);
            a[i] = new double[] {p, q};
            scaled[i] = new double[] {p, 1000.0 * q};
            b[i] = p - q + random.nextBetween(-0.1, 0.1);
        }

        LeastSquares.Solution plain = LeastSquares.solve(a, b, "p", "q");
        LeastSquares.Solution inMilli = LeastSquares.solve(scaled, b, "p", "q in thousandths");

        assertEquals(plain.coefficients[1], 1000.0 * inMilli.coefficients[1], 1e-10);
        assertEquals(plain.standardErrors[1], 1000.0 * inMilli.standardErrors[1], 1e-10);
        assertEquals(plain.conditionNumber, inMilli.conditionNumber, 1e-9);
    }

    @Test
    void aColumnThatCopiesAnotherIsRefusedByName() {
        double[][] a = new double[10][];
        double[] b = new double[10];
        for (int i = 0; i < 10; i++) {
            a[i] = new double[] {i, 2.0 * i};
            b[i] = i;
        }
        IllegalArgumentException refused = assertThrows(IllegalArgumentException.class,
                () -> LeastSquares.solve(a, b, "velocity", "twice velocity"));
        assertTrue(refused.getMessage().contains("twice velocity"), refused.getMessage());
    }

    @Test
    void badShapesAndNonFiniteValuesAreRefused() {
        assertThrows(IllegalArgumentException.class,
                () -> LeastSquares.solve(new double[][] {{1, 2}}, new double[] {1}, "a", "b"));
        assertThrows(IllegalArgumentException.class,
                () -> LeastSquares.solve(new double[][] {{1}, {2}}, new double[] {1, Double.NaN}, "a"));
        assertThrows(IllegalArgumentException.class,
                () -> LeastSquares.solve(new double[][] {{1}, {2}}, new double[] {1}, "a"));
    }
}
