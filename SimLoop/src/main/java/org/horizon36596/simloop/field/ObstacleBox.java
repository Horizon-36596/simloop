package org.horizon36596.simloop.field;

/**
 * A fixed, solid, axis-aligned box on the field that game pieces bounce off: a goal structure's frame, a
 * flower's foot, a wall segment. Defined in the field frame, inches (FTC-Cartesian, centre origin,
 * conventions §7), with a height range so that an in-flight piece flying <em>over</em> a low box is left
 * alone and one flying <em>into</em> a tall one is deflected.
 *
 * <p><b>Contact model, stated plainly.</b> A piece whose sphere overlaps the box in all three axes is
 * pushed out sideways along whichever of X or Y is the shorter way out, and the velocity component along
 * that axis is reversed and scaled by {@link GamePiecePhysicsConfig#getPieceToObstacleRestitution()}.
 * The box has no top to land on: a piece dropping onto it from above is pushed out sideways too, which is
 * wrong for a piece that should have come to rest on top, and right for everything the season has asked
 * for so far. If a season needs pieces resting on structures, that is a new feature, not a tweak here.
 *
 * <p>Immutable and season-agnostic (domain R7): the core knows a box, not what it is part of.
 */
public final class ObstacleBox {

    private final String name;
    private final double minXInches;
    private final double maxXInches;
    private final double minYInches;
    private final double maxYInches;
    private final double minZInches;
    private final double maxZInches;

    /**
     * Creates a box from its six faces, field frame, inches.
     *
     * @param name       a short label for logs and error messages, non-blank
     * @param minXInches the box's low X face, field frame, inches
     * @param maxXInches the box's high X face, field frame, inches (&gt; min)
     * @param minYInches the box's low Y face, field frame, inches
     * @param maxYInches the box's high Y face, field frame, inches (&gt; min)
     * @param minZInches the box's bottom, inches above the floor (0 for something standing on the tiles)
     * @param maxZInches the box's top, inches above the floor (&gt; min)
     * @throws IllegalArgumentException on a blank name, a non-finite number, or an empty range
     */
    public ObstacleBox(String name, double minXInches, double maxXInches, double minYInches,
            double maxYInches, double minZInches, double maxZInches) {
        if (name == null || name.trim().isEmpty()) {
            throw new IllegalArgumentException("an obstacle box needs a non-blank name");
        }
        requireRange(minXInches, maxXInches, "X");
        requireRange(minYInches, maxYInches, "Y");
        requireRange(minZInches, maxZInches, "Z");
        this.name = name.trim();
        this.minXInches = minXInches;
        this.maxXInches = maxXInches;
        this.minYInches = minYInches;
        this.maxYInches = maxYInches;
        this.minZInches = minZInches;
        this.maxZInches = maxZInches;
    }

    /** {@return the label given at construction} */
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

    /** {@return the bottom, inches above the floor} */
    public double getMinZInches() {
        return minZInches;
    }

    /** {@return the top, inches above the floor} */
    public double getMaxZInches() {
        return maxZInches;
    }

    @Override
    public String toString() {
        return "ObstacleBox(" + name + ")";
    }

    private static void requireRange(double min, double max, String axis) {
        if (!Double.isFinite(min) || !Double.isFinite(max)) {
            throw new IllegalArgumentException("obstacle " + axis + " range must be finite, got [" + min
                    + ", " + max + "]");
        }
        if (max <= min) {
            throw new IllegalArgumentException("obstacle " + axis + " range must have max > min, got ["
                    + min + ", " + max + "]");
        }
    }
}
