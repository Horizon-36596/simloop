package org.horizon36596.simloop.field;

/**
 * A robot that stands still: an alliance partner parked for a whole period, or a no-show's empty spot made
 * solid. It is an oriented footprint with a height, a solid box from the floor up, that balls bounce off and
 * that a driven robot cannot pass through ({@link RobotContact}).
 *
 * <p>Frame and units: field frame, FTC-Cartesian, centre origin, inches; heading radians CCW from field +X, the
 * direction the robot's front faces (conventions §7). Length is along that heading, width across it.
 *
 * <p>Immutable and season-agnostic (domain R7).
 */
public final class StaticRobot {

    private final String name;
    private final double xInches;
    private final double yInches;
    private final double headingRad;
    private final double lengthInches;
    private final double widthInches;
    private final double heightInches;

    /**
     * Creates a standing robot from its centre, heading and size.
     *
     * @param name         a label for contact logs, non-blank
     * @param xInches      centre, field X, inches
     * @param yInches      centre, field Y, inches
     * @param headingRad   heading, radians CCW from field +X
     * @param lengthInches full extent along the heading, inches, &gt; 0
     * @param widthInches  full extent across it, inches, &gt; 0
     * @param heightInches height above the floor, inches, &gt; 0
     * @throws IllegalArgumentException on a blank name, a non-finite number or a non-positive size
     */
    public StaticRobot(String name, double xInches, double yInches, double headingRad, double lengthInches,
            double widthInches, double heightInches) {
        if (name == null || name.trim().isEmpty()) {
            throw new IllegalArgumentException("a static robot needs a non-blank name");
        }
        for (double value : new double[] {xInches, yInches, headingRad}) {
            if (!Double.isFinite(value)) {
                throw new IllegalArgumentException("a static robot's pose must be finite");
            }
        }
        for (double value : new double[] {lengthInches, widthInches, heightInches}) {
            if (!Double.isFinite(value) || value <= 0.0) {
                throw new IllegalArgumentException("a static robot's size must be positive and finite");
            }
        }
        this.name = name.trim();
        this.xInches = xInches;
        this.yInches = yInches;
        this.headingRad = headingRad;
        this.lengthInches = lengthInches;
        this.widthInches = widthInches;
        this.heightInches = heightInches;
    }

    /** {@return the label given at construction} */
    public String getName() {
        return name;
    }

    /** {@return the centre's field X, inches} */
    public double getXInches() {
        return xInches;
    }

    /** {@return the centre's field Y, inches} */
    public double getYInches() {
        return yInches;
    }

    /** {@return the heading, radians CCW from field +X} */
    public double getHeadingRad() {
        return headingRad;
    }

    /** {@return the full extent along the heading, inches} */
    public double getLengthInches() {
        return lengthInches;
    }

    /** {@return the full extent across the heading, inches} */
    public double getWidthInches() {
        return widthInches;
    }

    /** {@return the height above the floor, inches} */
    public double getHeightInches() {
        return heightInches;
    }

    /** {@return the robot as a solid box in the field frame, floor to its height, named like the robot} */
    public OrientedBox toBox() {
        return OrientedBox.yawed(name, xInches, yInches, heightInches / 2.0, headingRad,
                lengthInches / 2.0, widthInches / 2.0, heightInches / 2.0);
    }
}
