package org.horizon36596.simloop.field;

/**
 * Whether the robot's rectangular footprint overlaps a field structure, and the shortest move that would
 * take it back out. This is what lets a season stop a simulated chassis at a structure the way a real one
 * is stopped: the drive plant is kinematic and knows nothing about the field, so after it moves, the
 * season asks this class about each structure and moves the robot back out of anything it drove into.
 *
 * <p><b>The test</b> is the separating-axis test for two rectangles in the plane: the footprint, rotated
 * by the robot's heading, and the structure's box, which is axis-aligned. Two convex shapes overlap if and
 * only if their shadows overlap on every axis that is perpendicular to one of their edges, and for two
 * rectangles those are four axes: field X, field Y, the robot's forward and the robot's left. Along each
 * axis the overlap depth is how far the two shadows overlap; the push is along the axis with the smallest
 * depth, by that depth, in the direction that moves the robot away from the box's centre. That is the
 * "minimum translation": the smallest move that separates them.
 *
 * <p><b>What it does not do.</b> Height is the caller's business: a structure the robot drives under (a
 * goal hanging above the tiles) is simply not asked about. Heading and velocity are not touched - the
 * robot is not rigid-body (domain R6), so a push moves it and nothing else, the same "stop and slide" a
 * perimeter clamp gives.
 *
 * <p>Frame and units: FTC-Cartesian field frame, inches, heading radians CCW-positive from field +X
 * (conventions §7). Every trigonometric call is {@link StrictMath}, so a push is bit-identical on every
 * machine (domain R5). Season-agnostic (domain R7).
 */
public final class FootprintContact {

    /** The shortest move that takes the footprint out of one box. */
    public static final class Push {
        private final double dxInches;
        private final double dyInches;
        private final double depthInches;

        Push(double dxInches, double dyInches, double depthInches) {
            this.dxInches = dxInches;
            this.dyInches = dyInches;
            this.depthInches = depthInches;
        }

        /** {@return how far to move the robot centre along field X, inches} */
        public double getDxInches() {
            return dxInches;
        }

        /** {@return how far to move the robot centre along field Y, inches} */
        public double getDyInches() {
            return dyInches;
        }

        /** {@return how deep the footprint was inside the box along the push direction, inches (&gt; 0)} */
        public double getDepthInches() {
            return depthInches;
        }
    }

    private FootprintContact() {
    }

    /**
     * The push that takes a rotated robot footprint out of a box, or {@code null} when they do not overlap.
     * Touching exactly (zero overlap) counts as not overlapping.
     *
     * @param centreXInches robot centre, field X, inches
     * @param centreYInches robot centre, field Y, inches
     * @param headingRad    robot heading, radians CCW from field +X
     * @param lengthInches  footprint along the robot's forward axis, inches
     * @param widthInches   footprint along the robot's left axis, inches
     * @param box           the structure; only its X and Y ranges are used
     * @return the minimum push, or {@code null} if the footprint is clear of the box
     * @throws IllegalArgumentException if a position or the heading is non-finite, the length or width is
     *                                  not a positive finite number, or the box is null
     */
    public static Push pushOutOf(double centreXInches, double centreYInches, double headingRad,
            double lengthInches, double widthInches, ObstacleBox box) {
        requireFinite(centreXInches, "centreXInches");
        requireFinite(centreYInches, "centreYInches");
        requireFinite(headingRad, "headingRad");
        requirePositive(lengthInches, "lengthInches");
        requirePositive(widthInches, "widthInches");
        if (box == null) {
            throw new IllegalArgumentException("pushOutOf needs a box to push the footprint out of");
        }
        double forwardX = StrictMath.cos(headingRad);
        double forwardY = StrictMath.sin(headingRad);
        double leftX = -forwardY;
        double leftY = forwardX;

        double boxCentreX = (box.getMinXInches() + box.getMaxXInches()) / 2.0;
        double boxCentreY = (box.getMinYInches() + box.getMaxYInches()) / 2.0;
        double boxHalfX = (box.getMaxXInches() - box.getMinXInches()) / 2.0;
        double boxHalfY = (box.getMaxYInches() - box.getMinYInches()) / 2.0;

        double[][] axes = {{1.0, 0.0}, {0.0, 1.0}, {forwardX, forwardY}, {leftX, leftY}};
        double bestDepth = Double.POSITIVE_INFINITY;
        double bestAxisX = 0.0;
        double bestAxisY = 0.0;
        for (double[] axis : axes) {
            // Half the length of each shape's shadow on this axis.
            double footprintShadow = lengthInches / 2.0 * Math.abs(forwardX * axis[0] + forwardY * axis[1])
                    + widthInches / 2.0 * Math.abs(leftX * axis[0] + leftY * axis[1]);
            double boxShadow = boxHalfX * Math.abs(axis[0]) + boxHalfY * Math.abs(axis[1]);
            double centreGap = (centreXInches - boxCentreX) * axis[0] + (centreYInches - boxCentreY) * axis[1];
            double depth = footprintShadow + boxShadow - Math.abs(centreGap);
            if (depth <= 0.0) {
                return null;                       // a separating axis: no overlap at all
            }
            if (depth < bestDepth) {
                bestDepth = depth;
                double awayFromBox = centreGap >= 0.0 ? 1.0 : -1.0;
                bestAxisX = axis[0] * awayFromBox;
                bestAxisY = axis[1] * awayFromBox;
            }
        }
        return new Push(bestAxisX * bestDepth, bestAxisY * bestDepth, bestDepth);
    }

    private static void requireFinite(double value, String name) {
        if (!Double.isFinite(value)) {
            throw new IllegalArgumentException(name + " must be finite, got " + value);
        }
    }

    private static void requirePositive(double value, String name) {
        if (!Double.isFinite(value) || value <= 0.0) {
            throw new IllegalArgumentException(name + " must be a positive finite number, got " + value);
        }
    }
}
