package org.horizon36596.simloop.field;

/**
 * One part of a robot touching one thing on the field: which part, which structure, how deep, and the shortest
 * horizontal move of the robot that would take that part back out. A season finds these each tick for its
 * {@link RobotShape} against the perimeter, the field's obstacles and any {@link StaticRobot}, then decides what
 * to do with them: push the robot and stop its motion into the contact (a normal run), or only record them (a
 * replay of a real robot's poses, which must be drawn as it happened).
 *
 * <p>Frame and units: field frame, inches. Season-agnostic (domain R7); deterministic (domain R5).
 */
public final class RobotContact {

    /** The structure name a contact with a perimeter wall carries. */
    public static final String PERIMETER = "perimeter";

    private final String partName;
    private final String structureName;
    private final double pushXInches;
    private final double pushYInches;
    private final double depthInches;

    /**
     * Creates a contact from the part, the structure and the move that frees it.
     *
     * @param partName      the robot part, e.g. a box name from the season's shape
     * @param structureName the thing it touched, e.g. an obstacle's name or {@link #PERIMETER}
     * @param pushXInches   field X of the shortest move that frees the part, inches
     * @param pushYInches   field Y of that move, inches
     * @param depthInches   how deep the part is in, inches (&gt; 0): the length of the move
     */
    public RobotContact(String partName, String structureName, double pushXInches, double pushYInches,
            double depthInches) {
        this.partName = partName;
        this.structureName = structureName;
        this.pushXInches = pushXInches;
        this.pushYInches = pushYInches;
        this.depthInches = depthInches;
    }

    /**
     * A placed part against a fixed box, or {@code null} when they do not overlap.
     *
     * @param part      a robot part, field frame
     * @param structure a structure, field frame
     * @return the contact, its push horizontal ({@link OrientedBox#horizontalPushOutOf})
     */
    public static RobotContact between(OrientedBox part, OrientedBox structure) {
        FootprintContact.Push push = part.horizontalPushOutOf(structure);
        if (push == null) {
            return null;
        }
        return new RobotContact(part.getName(), structure.getName(), push.getDxInches(), push.getDyInches(),
                push.getDepthInches());
    }

    /**
     * A placed part against the perimeter walls at {@code |x| = halfWidthInches}, {@code |y| = halfLengthInches}
     * (full height), or {@code null} when every corner is inside. The push brings the corner furthest past each
     * wall back to it, along X and Y separately, the "stop and slide" a wall gives.
     *
     * @param part             a robot part, field frame
     * @param halfWidthInches  the walls' distance from centre along X, inches
     * @param halfLengthInches the walls' distance from centre along Y, inches
     * @return the contact, or null
     */
    public static RobotContact withPerimeter(OrientedBox part, double halfWidthInches, double halfLengthInches) {
        double pushX = 0.0;
        double pushY = 0.0;
        for (double[] corner : part.corners()) {
            pushX = deeper(pushX, wallPush(corner[0], halfWidthInches));
            pushY = deeper(pushY, wallPush(corner[1], halfLengthInches));
        }
        if (pushX == 0.0 && pushY == 0.0) {
            return null;
        }
        return new RobotContact(part.getName(), PERIMETER, pushX, pushY, StrictMath.hypot(pushX, pushY));
    }

    /** {@return the robot part's name} */
    public String getPartName() {
        return partName;
    }

    /** {@return the structure's name, or {@link #PERIMETER}} */
    public String getStructureName() {
        return structureName;
    }

    /** {@return field X of the move that frees the part, inches} */
    public double getPushXInches() {
        return pushXInches;
    }

    /** {@return field Y of the move that frees the part, inches} */
    public double getPushYInches() {
        return pushYInches;
    }

    /** {@return how deep the part was in, inches} */
    public double getDepthInches() {
        return depthInches;
    }

    /** {@return {@code part@structure}, the form contact logs use} */
    public String label() {
        return partName + "@" + structureName;
    }

    @Override
    public String toString() {
        return label() + " " + depthInches + " in";
    }

    // The move along one axis that brings a coordinate back inside +-half (negative past the high wall).
    private static double wallPush(double coordinate, double half) {
        if (coordinate > half) {
            return half - coordinate;
        }
        if (coordinate < -half) {
            return -half - coordinate;
        }
        return 0.0;
    }

    private static double deeper(double current, double candidate) {
        return Math.abs(candidate) > Math.abs(current) ? candidate : current;
    }
}
