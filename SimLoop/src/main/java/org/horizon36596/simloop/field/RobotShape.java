package org.horizon36596.simloop.field;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * The robot as a compound of solid {@link OrientedBox boxes} in its own frame: a few for the chassis, a few for
 * each mechanism, posed for the mechanisms' current positions. Balls and field structures meet these boxes, not
 * a single rectangle, so an open intake mouth stays open and an arm out in front is as solid as the chassis.
 *
 * <p><b>Frame and units.</b> Robot frame, inches: +X forward, +Y left, +Z up, origin on the floor under the robot
 * centre (the point a drive plant's pose names). The season builds one each tick from its joint states and hands
 * it to {@link GamePieceTracker#setRobotShape}; {@link #placedOnRobot} turns it into the field frame.
 *
 * <p>Immutable and season-agnostic (domain R7): the core knows boxes and names, not what they belong to.
 */
public final class RobotShape {

    private final List<OrientedBox> parts;

    /**
     * Creates a robot shape from its boxes.
     *
     * @param parts the boxes, robot frame, in a fixed order (contacts are reported in it); at least one, names
     *              unique
     * @throws IllegalArgumentException when empty, holding null, or two parts share a name
     */
    public RobotShape(List<OrientedBox> parts) {
        if (parts == null || parts.isEmpty()) {
            throw new IllegalArgumentException("a robot shape needs at least one box");
        }
        List<String> names = new ArrayList<>();
        for (OrientedBox part : parts) {
            if (part == null) {
                throw new IllegalArgumentException("a robot shape cannot hold a null box");
            }
            if (names.contains(part.getName())) {
                throw new IllegalArgumentException("two robot parts named " + part.getName());
            }
            names.add(part.getName());
        }
        this.parts = Collections.unmodifiableList(new ArrayList<>(parts));
    }

    /** {@return the boxes, robot frame, in the order given} */
    public List<OrientedBox> getParts() {
        return parts;
    }

    /**
     * {@return every box placed in the field for a robot at this pose, in the same order}
     *
     * @param robotXInches    robot centre, field X, inches
     * @param robotYInches    robot centre, field Y, inches
     * @param robotHeadingRad robot heading, radians CCW from field +X
     */
    public List<OrientedBox> placedOnRobot(double robotXInches, double robotYInches, double robotHeadingRad) {
        List<OrientedBox> placed = new ArrayList<>(parts.size());
        for (OrientedBox part : parts) {
            placed.add(part.placedOnRobot(robotXInches, robotYInches, robotHeadingRad));
        }
        return placed;
    }

    /** {@return the highest point of any part, inches above the floor} */
    public double getMaxZ() {
        double top = Double.NEGATIVE_INFINITY;
        for (OrientedBox part : parts) {
            top = Math.max(top, part.getMaxZ());
        }
        return top;
    }
}
