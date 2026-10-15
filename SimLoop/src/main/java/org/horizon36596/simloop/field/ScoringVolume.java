package org.horizon36596.simloop.field;

/**
 * A region of the field that an in-flight game piece scores by entering: the mouth of a goal. Defined in
 * the field frame, inches (FTC-Cartesian, centre origin, conventions §7), as an axis-aligned box plus two
 * optional entry conditions that make a real goal mouth different from a box in the air:
 * <ul>
 *   <li><b>descending only</b> — a goal with a top opening is only entered by a piece on its way down, so
 *       a piece passing through the box on the way up does not score;</li>
 *   <li><b>approach direction</b> — a goal open on one face only is entered by a piece moving
 *       <em>into</em> that face, so a piece crossing the box the other way does not score.</li>
 * </ul>
 * Both conditions describe a raised goal that is open on one face; a season whose goal is a simple hole
 * leaves both off.
 *
 * <p><b>What happens to a scored piece.</b> It is taken out of play and parked at a resting slot inside
 * the goal so a viewer can still draw it: the first piece at the rest point, the next one a rest-spacing
 * step further (along X for a row of balls in a wide cell, along Z for a stack in a tube), and so on. The
 * slots are for the picture, not the rules — nothing in the core counts them against a capacity.
 *
 * <p>Immutable and season-agnostic (domain R7). Build one with {@link #builder(String)}.
 */
public final class ScoringVolume {

    private final String name;
    private final double minXInches;
    private final double maxXInches;
    private final double minYInches;
    private final double maxYInches;
    private final double minZInches;
    private final double maxZInches;
    private final boolean descendingOnly;
    private final double approachDirectionX;
    private final double approachDirectionY;
    private final double restXInches;
    private final double restYInches;
    private final double restZInches;
    private final double restSpacingXInches;
    private final double restSpacingYInches;
    private final double restSpacingZInches;
    private final int restPiecesPerRow;
    private final double restRowStepXInches;
    private final double restRowStepYInches;
    private final double restRowStepZInches;
    private final int restRowsPerLayer;
    private final double restLayerStepXInches;
    private final double restLayerStepYInches;
    private final double restLayerStepZInches;

    private ScoringVolume(Builder builder) {
        this.name = builder.name;
        this.minXInches = builder.minXInches;
        this.maxXInches = builder.maxXInches;
        this.minYInches = builder.minYInches;
        this.maxYInches = builder.maxYInches;
        this.minZInches = builder.minZInches;
        this.maxZInches = builder.maxZInches;
        this.descendingOnly = builder.descendingOnly;
        this.approachDirectionX = builder.approachDirectionX;
        this.approachDirectionY = builder.approachDirectionY;
        this.restXInches = builder.restXInches;
        this.restYInches = builder.restYInches;
        this.restZInches = builder.restZInches;
        this.restSpacingXInches = builder.restSpacingXInches;
        this.restSpacingYInches = builder.restSpacingYInches;
        this.restSpacingZInches = builder.restSpacingZInches;
        this.restPiecesPerRow = builder.restPiecesPerRow;
        this.restRowStepXInches = builder.restRowStepXInches;
        this.restRowStepYInches = builder.restRowStepYInches;
        this.restRowStepZInches = builder.restRowStepZInches;
        this.restRowsPerLayer = builder.restRowsPerLayer;
        this.restLayerStepXInches = builder.restLayerStepXInches;
        this.restLayerStepYInches = builder.restLayerStepYInches;
        this.restLayerStepZInches = builder.restLayerStepZInches;
    }

    /**
     * Starts a scoring volume.
     *
     * @param name a short label, non-blank; it becomes the cause in the possession log
     *             ({@code "3 IN_FLIGHT->SCORED goalNorth"})
     * @return a builder; {@link Builder#box} and {@link Builder#restPoint} must be called before build
     */
    public static Builder builder(String name) {
        return new Builder(name);
    }

    /**
     * Whether a piece at this position, moving with this velocity, is inside the goal the right way: its
     * centre is inside the box and every entry condition holds. A test of one point; the physics uses
     * {@link #acceptsPath}, which also catches a fast piece that crosses a thin box between two sub-steps.
     *
     * @param xInches         piece centre, field frame, inches
     * @param yInches         piece centre, field frame, inches
     * @param zInches         piece centre height above the floor, inches
     * @param velocityXInchesPerSecond inches per second along field +X
     * @param velocityYInchesPerSecond inches per second along field +Y
     * @param velocityZInchesPerSecond inches per second, positive upward
     * @return true when the centre is inside the box and every entry condition holds
     */
    public boolean accepts(double xInches, double yInches, double zInches, double velocityXInchesPerSecond,
            double velocityYInchesPerSecond, double velocityZInchesPerSecond) {
        return containsCentre(xInches, yInches, zInches)
                && entryConditionsHold(velocityXInchesPerSecond, velocityYInchesPerSecond,
                        velocityZInchesPerSecond);
    }

    /**
     * Whether a piece whose centre moved in a straight line from {@code (x0, y0, z0)} to
     * {@code (x1, y1, z1)} during one physics sub-step, at this velocity, entered the goal the right way.
     *
     * <p><b>Why a path and not a point.</b> Testing only where the centre ended up misses a fast piece that
     * crosses a thin box in one sub-step: a 1 in deep mouth crossed at 260 in/s is entered and left inside a
     * single 5 ms step (1.3 in of travel). So the whole segment is tested against the box, the standard
     * slab test: along each axis the segment is inside that axis's range for some fraction of the step,
     * and it touched the box if those three fractions overlap. A segment that ends inside the box always
     * passes, so this accepts everything {@link #accepts} does, and more.
     *
     * <p><b>The entry conditions</b> (descending only, approach direction) are tested on the velocity
     * given. The physics passes the segment and velocity as they stand <em>after</em> this sub-step's wall
     * and obstacle pushes. For a piece that touched nothing that is exact: the integrator moves a flying
     * piece along a straight line at one velocity per sub-step. For a piece that a wall or obstacle pushed
     * earlier in the same sub-step it is an approximation: the push moves the end point and can reverse
     * the horizontal velocity, so the segment is not quite the path flown and the velocity is the one after
     * the bounce, not the one at the goal's face (a known limit, listed in {@code GamePiecePhysics}).
     *
     * @param x0Inches piece centre at the start of the sub-step, field frame, inches
     * @param y0Inches ditto, Y
     * @param z0Inches ditto, height above the floor
     * @param x1Inches piece centre at the end of the sub-step, field frame, inches
     * @param y1Inches ditto, Y
     * @param z1Inches ditto, height above the floor
     * @param velocityXInchesPerSecond inches per second along field +X during the sub-step
     * @param velocityYInchesPerSecond inches per second along field +Y
     * @param velocityZInchesPerSecond inches per second, positive upward
     * @return true when the segment touches the box and every entry condition holds
     */
    boolean acceptsPath(double x0Inches, double y0Inches, double z0Inches, double x1Inches, double y1Inches,
            double z1Inches, double velocityXInchesPerSecond, double velocityYInchesPerSecond,
            double velocityZInchesPerSecond) {
        return segmentTouchesBox(x0Inches, y0Inches, z0Inches, x1Inches, y1Inches, z1Inches)
                && entryConditionsHold(velocityXInchesPerSecond, velocityYInchesPerSecond,
                        velocityZInchesPerSecond);
    }

    /**
     * Whether a point is inside the box, faces included, with no entry condition applied.
     *
     * @param xInches field frame, inches
     * @param yInches field frame, inches
     * @param zInches height above the floor, inches
     * @return true when the point is inside the box or on one of its faces
     */
    boolean containsCentre(double xInches, double yInches, double zInches) {
        return xInches >= minXInches && xInches <= maxXInches
                && yInches >= minYInches && yInches <= maxYInches
                && zInches >= minZInches && zInches <= maxZInches;
    }

    // Descending only and approach direction, the two conditions that make a mouth different from a box.
    private boolean entryConditionsHold(double velocityXInchesPerSecond, double velocityYInchesPerSecond,
            double velocityZInchesPerSecond) {
        if (descendingOnly && velocityZInchesPerSecond >= 0.0) {
            return false;
        }
        if (approachDirectionX != 0.0 || approachDirectionY != 0.0) {
            double towardTheOpening = velocityXInchesPerSecond * approachDirectionX
                    + velocityYInchesPerSecond * approachDirectionY;
            if (towardTheOpening <= 0.0) {
                return false;
            }
        }
        return true;
    }

    // The slab test. "Fraction" runs from 0 at the segment's start to 1 at its end; window holds the part
    // of the segment that is inside every axis range tested so far, and it shrinks one axis at a time.
    private boolean segmentTouchesBox(double x0, double y0, double z0, double x1, double y1, double z1) {
        double[] window = {0.0, 1.0};
        return clipToRange(x0, x1, minXInches, maxXInches, window)
                && clipToRange(y0, y1, minYInches, maxYInches, window)
                && clipToRange(z0, z1, minZInches, maxZInches, window);
    }

    // Narrow window to the fractions of the segment where this one coordinate is inside [min, max].
    // Returns false as soon as the window is empty: the segment is never inside every range at once.
    private static boolean clipToRange(double start, double end, double min, double max, double[] window) {
        double change = end - start;
        if (change == 0.0) {
            return start >= min && start <= max;       // not moving along this axis: always in, or never
        }
        double fractionAtMin = (min - start) / change;
        double fractionAtMax = (max - start) / change;
        window[0] = Math.max(window[0], Math.min(fractionAtMin, fractionAtMax));
        window[1] = Math.min(window[1], Math.max(fractionAtMin, fractionAtMax));
        return window[0] <= window[1];
    }

    /** {@return the label given at construction; also the cause logged for a score} */
    public String getName() {
        return name;
    }

    /** {@return the low X face, field frame, inches} */
    public double getMinXInches() {
        return minXInches;
    }

    /** {@return the high X face, field frame, inches} */
    public double getMaxXInches() {
        return maxXInches;
    }

    /** {@return the low Y face, field frame, inches} */
    public double getMinYInches() {
        return minYInches;
    }

    /** {@return the high Y face, field frame, inches} */
    public double getMaxYInches() {
        return maxYInches;
    }

    /** {@return the bottom of the box, inches above the floor} */
    public double getMinZInches() {
        return minZInches;
    }

    /** {@return the top of the box, inches above the floor} */
    public double getMaxZInches() {
        return maxZInches;
    }

    /** {@return true when only a piece moving downward can score} */
    public boolean isDescendingOnly() {
        return descendingOnly;
    }

    /** {@return the resting X of the first scored piece, field frame, inches} */
    public double getRestXInches() {
        return restXInches;
    }

    /** {@return the resting Y of every scored piece, field frame, inches} */
    public double getRestYInches() {
        return restYInches;
    }

    /** {@return the resting centre height of every scored piece, inches above the floor} */
    public double getRestZInches() {
        return restZInches;
    }

    /**
     * Where the {@code indexInVolume}-th scored piece (counting from zero) is parked. Pieces fill a row of
     * {@link Builder#restRows piecesPerRow} rest-spacing steps from the rest point, then start the next row
     * one row step on, and after {@link Builder#restLayers rowsPerLayer} rows the next layer one layer step
     * on. With no rows or layers set, every piece is one more rest-spacing step along a single line, as
     * before rows existed. Package-private: the tracker parks pieces; a caller reads where they ended up
     * from the pieces themselves.
     *
     * @return {@code {x, y, z}} in field-frame inches
     */
    double[] restPositionFor(int indexInVolume) {
        int column = indexInVolume % restPiecesPerRow;
        int rowIndex = indexInVolume / restPiecesPerRow;
        int row = rowIndex % restRowsPerLayer;
        int layer = rowIndex / restRowsPerLayer;
        return new double[] {
                restXInches + column * restSpacingXInches + row * restRowStepXInches
                        + layer * restLayerStepXInches,
                restYInches + column * restSpacingYInches + row * restRowStepYInches
                        + layer * restLayerStepYInches,
                restZInches + column * restSpacingZInches + row * restRowStepZInches
                        + layer * restLayerStepZInches};
    }

    @Override
    public String toString() {
        return "ScoringVolume(" + name + ")";
    }

    /** One named setter per fact; {@link #box} and {@link #restPoint} are mandatory. */
    public static final class Builder {

        private final String name;
        private double minXInches = Double.NaN;
        private double maxXInches;
        private double minYInches;
        private double maxYInches;
        private double minZInches;
        private double maxZInches;
        private boolean descendingOnly = false;
        private double approachDirectionX = 0.0;
        private double approachDirectionY = 0.0;
        private double restXInches = Double.NaN;
        private double restYInches;
        private double restZInches;
        private double restSpacingXInches = 0.0;
        private double restSpacingYInches = 0.0;
        private double restSpacingZInches = 0.0;
        private int restPiecesPerRow = Integer.MAX_VALUE;
        private double restRowStepXInches = 0.0;
        private double restRowStepYInches = 0.0;
        private double restRowStepZInches = 0.0;
        private int restRowsPerLayer = Integer.MAX_VALUE;
        private double restLayerStepXInches = 0.0;
        private double restLayerStepYInches = 0.0;
        private double restLayerStepZInches = 0.0;

        private Builder(String name) {
            if (name == null || name.trim().isEmpty()) {
                throw new IllegalArgumentException("a scoring volume needs a non-blank name");
            }
            this.name = name.trim();
        }

        /**
         * The axis-aligned box a piece's centre must enter, field-frame inches: a piece scores on the
         * sub-step its centre's path touches the box (see {@link ScoringVolume#acceptsPath}). Every max
         * must exceed its min.
         *
         * @param minXInches low X face, field frame, inches
         * @param maxXInches high X face, field frame, inches
         * @param minYInches low Y face, field frame, inches
         * @param maxYInches high Y face, field frame, inches
         * @param minZInches bottom of the box, inches above the floor
         * @param maxZInches top of the box, inches above the floor
         * @return this builder
         */
        public Builder box(double minXInches, double maxXInches, double minYInches, double maxYInches,
                double minZInches, double maxZInches) {
            requireRange(minXInches, maxXInches, "X");
            requireRange(minYInches, maxYInches, "Y");
            requireRange(minZInches, maxZInches, "Z");
            this.minXInches = minXInches;
            this.maxXInches = maxXInches;
            this.minYInches = minYInches;
            this.maxYInches = maxYInches;
            this.minZInches = minZInches;
            this.maxZInches = maxZInches;
            return this;
        }

        /**
         * Only a piece moving downward ({@code vz < 0}) scores. Off by default.
         *
         * @param value true to score only pieces moving downward
         * @return this builder
         */
        public Builder descendingOnly(boolean value) {
            this.descendingOnly = value;
            return this;
        }

        /**
         * Only a piece with a positive velocity component along this field-frame direction scores — the
         * direction a piece travels when it flies <em>into</em> the goal's open face. Zero-length (the
         * default) means any direction.
         *
         * @param fieldX direction, field X component; any length, normalised here
         * @param fieldY direction, field Y component
         * @return this builder
         */
        public Builder approachDirection(double fieldX, double fieldY) {
            if (!Double.isFinite(fieldX) || !Double.isFinite(fieldY)) {
                throw new IllegalArgumentException("approach direction must be finite");
            }
            double length = StrictMath.sqrt(fieldX * fieldX + fieldY * fieldY);
            if (length == 0.0) {
                this.approachDirectionX = 0.0;
                this.approachDirectionY = 0.0;
            } else {
                this.approachDirectionX = fieldX / length;
                this.approachDirectionY = fieldY / length;
            }
            return this;
        }

        /**
         * Where the first scored piece is parked for the viewer, field-frame inches.
         *
         * @param xInches rest point, field frame X, inches
         * @param yInches rest point, field frame Y, inches
         * @param zInches rest point centre height, inches above the floor
         * @return this builder
         */
        public Builder restPoint(double xInches, double yInches, double zInches) {
            if (!Double.isFinite(xInches) || !Double.isFinite(yInches) || !Double.isFinite(zInches)) {
                throw new IllegalArgumentException("rest point must be finite");
            }
            this.restXInches = xInches;
            this.restYInches = yInches;
            this.restZInches = zInches;
            return this;
        }

        /**
         * The step from each parked piece to the next, field-frame inches: e.g. {@code (3.6, 0, 0)} for a
         * row of 3.6 in balls across a cell, {@code (0, 0, 2.8)} for a stack of 2.8 in balls in a tube.
         * Default: no step, every piece parks on the rest point.
         *
         * @param xInches step along field X, inches
         * @param yInches step along field Y, inches
         * @param zInches step upward, inches
         * @return this builder
         */
        public Builder restSpacing(double xInches, double yInches, double zInches) {
            if (!Double.isFinite(xInches) || !Double.isFinite(yInches) || !Double.isFinite(zInches)) {
                throw new IllegalArgumentException("rest spacing must be finite");
            }
            this.restSpacingXInches = xInches;
            this.restSpacingYInches = yInches;
            this.restSpacingZInches = zInches;
            return this;
        }

        /**
         * Wrap the parked pieces into rows, so a goal that fills up keeps them inside it: after
         * {@code piecesPerRow} pieces the next starts a new row, offset from the first row by this step,
         * field-frame inches. Default: one endless row.
         *
         * @param piecesPerRow pieces in each row, at least 1
         * @param stepXInches  offset from one row to the next along field X, inches
         * @param stepYInches  offset from one row to the next along field Y, inches
         * @param stepZInches  offset from one row to the next upward, inches
         * @return this builder
         * @throws IllegalArgumentException if {@code piecesPerRow} is below 1 or a step is not finite
         */
        public Builder restRows(int piecesPerRow, double stepXInches, double stepYInches, double stepZInches) {
            requireWrap(piecesPerRow, stepXInches, stepYInches, stepZInches, "rows");
            this.restPiecesPerRow = piecesPerRow;
            this.restRowStepXInches = stepXInches;
            this.restRowStepYInches = stepYInches;
            this.restRowStepZInches = stepZInches;
            return this;
        }

        /**
         * Stack the rows into layers: after {@code rowsPerLayer} rows the next row starts a new layer, offset
         * from the first by this step, field-frame inches. Default: one endless layer.
         *
         * @param rowsPerLayer rows in each layer, at least 1
         * @param stepXInches  offset from one layer to the next along field X, inches
         * @param stepYInches  offset from one layer to the next along field Y, inches
         * @param stepZInches  offset from one layer to the next upward, inches
         * @return this builder
         * @throws IllegalArgumentException if {@code rowsPerLayer} is below 1 or a step is not finite
         */
        public Builder restLayers(int rowsPerLayer, double stepXInches, double stepYInches, double stepZInches) {
            requireWrap(rowsPerLayer, stepXInches, stepYInches, stepZInches, "layers");
            this.restRowsPerLayer = rowsPerLayer;
            this.restLayerStepXInches = stepXInches;
            this.restLayerStepYInches = stepYInches;
            this.restLayerStepZInches = stepZInches;
            return this;
        }

        private static void requireWrap(int count, double x, double y, double z, String what) {
            if (count < 1) {
                throw new IllegalArgumentException("rest " + what + " need a count of at least 1, got " + count);
            }
            if (!Double.isFinite(x) || !Double.isFinite(y) || !Double.isFinite(z)) {
                throw new IllegalArgumentException("rest " + what + " step must be finite");
            }
        }

        /**
         * Builds the scoring volume.
         *
         * @return the immutable volume
         * @throws IllegalStateException if {@link #box} or {@link #restPoint} was not called
         */
        public ScoringVolume build() {
            if (Double.isNaN(minXInches)) {
                throw new IllegalStateException("scoring volume " + name + " needs box(...)");
            }
            if (Double.isNaN(restXInches)) {
                throw new IllegalStateException("scoring volume " + name + " needs restPoint(...)");
            }
            return new ScoringVolume(this);
        }

        private static void requireRange(double min, double max, String axis) {
            if (!Double.isFinite(min) || !Double.isFinite(max)) {
                throw new IllegalArgumentException("scoring " + axis + " range must be finite, got [" + min
                        + ", " + max + "]");
            }
            if (max <= min) {
                throw new IllegalArgumentException("scoring " + axis + " range must have max > min, got ["
                        + min + ", " + max + "]");
            }
        }
    }
}
