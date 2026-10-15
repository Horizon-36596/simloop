package org.horizon36596.simloop.sensor;

import org.horizon36596.simloop.fakehardware.FakeDigitalChannel;
import org.horizon36596.simloop.fakehardware.FakeTouchSensor;
import org.horizon36596.simloop.fakehardware.SwitchPolarity;
import org.horizon36596.simloop.field.FieldWorld;
import org.horizon36596.simloop.field.GamePiece;
import org.horizon36596.simloop.field.ObstacleBox;

import java.util.List;

/**
 * Binds a touch sensor or limit switch to the simulated field: the switch's actuator is a small rectangle
 * bolted to the robot, and it is pressed while that rectangle touches the perimeter wall, an
 * {@link ObstacleBox}, or a game piece that is not held. The device fake stays a boolean; this class
 * decides what the boolean is.
 *
 * <pre>{@code
 * // A bumper switch whose lever sticks 0.25 in out of an 18 in robot's front face:
 * ContactSwitch bumper = new ContactSwitch("bumper", 9.125, 0.0, 0.25, 4.0, 2.0);
 * // each tick:
 * bumper.apply(bumperTouchSensor, tracker.getWorld(), tracker.getPieces(), x, y, heading);
 * }</pre>
 *
 * <p><b>Make the actuator stick out by the switch's travel.</b> A drive plant that stops a robot at a wall
 * stops it flush, with the bumper exactly on the wall. A switch whose actuator is flush with the bumper would
 * then be exactly touching, which is a knife-edge; one whose lever sticks out past the bumper, as a real
 * lever does, overlaps the wall and is pressed. Touching exactly does count as pressed.
 *
 * <p><b>The geometry.</b> Robot frame and field frame as {@code TriggerVolume} defines them: the actuator is
 * a rectangle centred {@code forwardOffsetInches} ahead of and {@code leftOffsetInches} left of the robot's
 * centre, {@code depthInches} along forward and {@code widthInches} along left, at {@code heightInches}
 * above the floor. It presses against the wall when any corner is on or past the perimeter of a bounded
 * world; against an obstacle whose height range includes {@code heightInches} when the two rectangles
 * overlap or touch (the separating-axis test {@code FootprintContact} uses, with touching counted); against
 * a piece when the piece's horizontal slice at that height touches the rectangle.
 *
 * <p><b>A piece the robot holds is not here</b>: the tracker parks it at the robot's centre. A switch a
 * held piece rests on is driven from the tracker instead:
 * {@code touch.setPressedSource(() -> tracker.getHeldCount() > 0)}.
 *
 * <p>Immutable and pure; deterministic and clock-free, with all trigonometry in {@link StrictMath}.
 */
public final class ContactSwitch {

    /** What the actuator is touching. */
    public enum Contact {
        /** Nothing: the switch is released. */
        NONE,
        /** The field perimeter wall. */
        WALL,
        /** One of the world's {@code ObstacleBox}es. */
        OBSTACLE,
        /** A game piece that is not held. */
        PIECE
    }

    private final String name;
    private final double forwardOffsetInches;
    private final double leftOffsetInches;
    private final double depthInches;
    private final double widthInches;
    private final double heightInches;

    /**
     * Validates and stores one actuator.
     *
     * @param name                a short label for messages, non-blank
     * @param forwardOffsetInches the actuator's centre along robot forward, inches (+ toward the front)
     * @param leftOffsetInches    the actuator's centre along robot left, inches (+ toward the left side)
     * @param depthInches         the actuator's full extent along robot forward, inches (&gt; 0)
     * @param widthInches         the actuator's full extent along robot left, inches (&gt; 0)
     * @param heightInches        the actuator's height above the floor, inches (&ge; 0)
     * @throws IllegalArgumentException on a blank name, a non-finite number, a non-positive extent or a
     *                                  negative height
     */
    public ContactSwitch(String name, double forwardOffsetInches, double leftOffsetInches, double depthInches,
            double widthInches, double heightInches) {
        if (name == null || name.trim().isEmpty()) {
            throw new IllegalArgumentException("a contact switch needs a non-blank name");
        }
        SensorMount.requireFinite(forwardOffsetInches, "forwardOffsetInches");
        SensorMount.requireFinite(leftOffsetInches, "leftOffsetInches");
        SensorMount.requireFinite(depthInches, "depthInches");
        SensorMount.requireFinite(widthInches, "widthInches");
        SensorMount.requireFinite(heightInches, "heightInches");
        if (depthInches <= 0.0 || widthInches <= 0.0) {
            throw new IllegalArgumentException("an actuator needs a positive depth and width, got " + depthInches
                    + " x " + widthInches);
        }
        if (heightInches < 0.0) {
            throw new IllegalArgumentException("an actuator cannot be below the floor, got " + heightInches);
        }
        this.name = name.trim();
        this.forwardOffsetInches = forwardOffsetInches;
        this.leftOffsetInches = leftOffsetInches;
        this.depthInches = depthInches;
        this.widthInches = widthInches;
        this.heightInches = heightInches;
    }

    /**
     * What the actuator is touching for a robot at the given pose, checked in the order wall, obstacles
     * (in declaration order), pieces (in id order); the first contact found is returned.
     *
     * @param world           the fixed field: perimeter and obstacles
     * @param pieces          every piece, from {@code GamePieceTracker.getPieces()}; held pieces are skipped
     * @param robotXInches    robot centre, field frame, inches
     * @param robotYInches    robot centre, field frame, inches
     * @param robotHeadingRad robot heading, radians CCW from field +X
     * @return the contact, or {@link Contact#NONE}
     * @throws IllegalArgumentException if {@code world} or {@code pieces} is null, or the pose is not finite
     */
    public Contact check(FieldWorld world, List<GamePiece> pieces, double robotXInches, double robotYInches,
            double robotHeadingRad) {
        if (world == null || pieces == null) {
            throw new IllegalArgumentException("a contact switch needs the world and the piece list");
        }
        SensorMount.requireFinite(robotXInches, "robotXInches");
        SensorMount.requireFinite(robotYInches, "robotYInches");
        SensorMount.requireFinite(robotHeadingRad, "robotHeadingRad");

        double cos = StrictMath.cos(robotHeadingRad);
        double sin = StrictMath.sin(robotHeadingRad);
        double centreX = robotXInches + forwardOffsetInches * cos - leftOffsetInches * sin;
        double centreY = robotYInches + forwardOffsetInches * sin + leftOffsetInches * cos;

        if (world.isBounded() && touchesPerimeter(centreX, centreY, cos, sin, world.getHalfWidthInches(),
                world.getHalfLengthInches())) {
            return Contact.WALL;
        }
        for (ObstacleBox box : world.getObstacles()) {
            if (heightInches >= box.getMinZInches() && heightInches <= box.getMaxZInches()
                    && touchesBox(centreX, centreY, cos, sin, box)) {
                return Contact.OBSTACLE;
            }
        }
        for (GamePiece piece : pieces) {
            if (piece.isVisibleOnField() && touchesPiece(centreX, centreY, cos, sin, piece)) {
                return Contact.PIECE;
            }
        }
        return Contact.NONE;
    }

    /**
     * True when the actuator touches anything, for a robot at the given pose.
     *
     * @param world           the fixed field: perimeter and obstacles
     * @param pieces          every piece; held pieces are skipped
     * @param robotXInches    robot centre, field frame, inches
     * @param robotYInches    robot centre, field frame, inches
     * @param robotHeadingRad robot heading, radians CCW from field +X
     * @return true when {@link #check} finds a contact
     * @throws IllegalArgumentException if {@code world} or {@code pieces} is null, or the pose is not finite
     */
    public boolean isPressed(FieldWorld world, List<GamePiece> pieces, double robotXInches, double robotYInches,
            double robotHeadingRad) {
        return check(world, pieces, robotXInches, robotYInches, robotHeadingRad) != Contact.NONE;
    }

    /**
     * Checks, then presses or releases a fake touch sensor to match.
     *
     * @param sensor          the fake to drive
     * @param world           the fixed field: perimeter and obstacles
     * @param pieces          every piece; held pieces are skipped
     * @param robotXInches    robot centre, field frame, inches
     * @param robotYInches    robot centre, field frame, inches
     * @param robotHeadingRad robot heading, radians CCW from field +X
     * @return the contact, so a test can check what pressed it
     * @throws IllegalArgumentException if {@code sensor}, {@code world} or {@code pieces} is null, or the
     *                                  pose is not finite
     */
    public Contact apply(FakeTouchSensor sensor, FieldWorld world, List<GamePiece> pieces, double robotXInches,
            double robotYInches, double robotHeadingRad) {
        if (sensor == null) {
            throw new IllegalArgumentException("a contact switch needs a sensor to drive");
        }
        Contact contact = check(world, pieces, robotXInches, robotYInches, robotHeadingRad);
        sensor.setPressed(contact != Contact.NONE);
        return contact;
    }

    /**
     * Checks, then puts the matching line level on a fake digital channel, for a robot that reads its switch
     * as a {@code DigitalChannel}.
     *
     * @param channel         the fake the switch is wired to
     * @param polarity        how it is wired; {@link SwitchPolarity#PRESSED_IS_LOW} for a REV Touch Sensor
     * @param world           the fixed field: perimeter and obstacles
     * @param pieces          every piece; held pieces are skipped
     * @param robotXInches    robot centre, field frame, inches
     * @param robotYInches    robot centre, field frame, inches
     * @param robotHeadingRad robot heading, radians CCW from field +X
     * @return the contact, so a test can check what pressed it
     * @throws IllegalArgumentException if {@code channel}, {@code polarity}, {@code world} or {@code pieces}
     *                                  is null, or the pose is not finite
     */
    public Contact apply(FakeDigitalChannel channel, SwitchPolarity polarity, FieldWorld world,
            List<GamePiece> pieces, double robotXInches, double robotYInches, double robotHeadingRad) {
        if (channel == null || polarity == null) {
            throw new IllegalArgumentException("a contact switch needs a channel and its wiring to drive");
        }
        Contact contact = check(world, pieces, robotXInches, robotYInches, robotHeadingRad);
        polarity.drive(channel, contact != Contact.NONE);
        return contact;
    }

    /** {@return the label given at construction} */
    public String getName() {
        return name;
    }

    /** {@return the actuator's centre along robot forward, inches} */
    public double getForwardOffsetInches() {
        return forwardOffsetInches;
    }

    /** {@return the actuator's centre along robot left, inches} */
    public double getLeftOffsetInches() {
        return leftOffsetInches;
    }

    /** {@return the actuator's full extent along robot forward, inches} */
    public double getDepthInches() {
        return depthInches;
    }

    /** {@return the actuator's full extent along robot left, inches} */
    public double getWidthInches() {
        return widthInches;
    }

    /** {@return the actuator's height above the floor, inches} */
    public double getHeightInches() {
        return heightInches;
    }

    /** {@return the switch, for test messages} */
    @Override
    public String toString() {
        return "ContactSwitch(" + name + ")";
    }

    /** Any corner of the actuator on or past the wall. */
    private boolean touchesPerimeter(double centreX, double centreY, double cos, double sin, double halfWidth,
            double halfLength) {
        double halfDepth = depthInches / 2.0;
        double halfWidthOfPad = widthInches / 2.0;
        // The corners' reach along each field axis from the centre: the rectangle's shadow half-lengths.
        double reachX = halfDepth * Math.abs(cos) + halfWidthOfPad * Math.abs(sin);
        double reachY = halfDepth * Math.abs(sin) + halfWidthOfPad * Math.abs(cos);
        return Math.abs(centreX) + reachX >= halfWidth || Math.abs(centreY) + reachY >= halfLength;
    }

    /** Separating-axis test of the rotated actuator against an axis-aligned box, touching counted. */
    private boolean touchesBox(double centreX, double centreY, double cos, double sin, ObstacleBox box) {
        double forwardX = cos;
        double forwardY = sin;
        double leftX = -sin;
        double leftY = cos;
        double boxCentreX = (box.getMinXInches() + box.getMaxXInches()) / 2.0;
        double boxCentreY = (box.getMinYInches() + box.getMaxYInches()) / 2.0;
        double boxHalfX = (box.getMaxXInches() - box.getMinXInches()) / 2.0;
        double boxHalfY = (box.getMaxYInches() - box.getMinYInches()) / 2.0;
        double[][] axes = {{1.0, 0.0}, {0.0, 1.0}, {forwardX, forwardY}, {leftX, leftY}};
        for (double[] axis : axes) {
            double padShadow = depthInches / 2.0 * Math.abs(forwardX * axis[0] + forwardY * axis[1])
                    + widthInches / 2.0 * Math.abs(leftX * axis[0] + leftY * axis[1]);
            double boxShadow = boxHalfX * Math.abs(axis[0]) + boxHalfY * Math.abs(axis[1]);
            double centreGap = (centreX - boxCentreX) * axis[0] + (centreY - boxCentreY) * axis[1];
            if (Math.abs(centreGap) > padShadow + boxShadow) {
                return false;                      // a separating axis
            }
        }
        return true;
    }

    /** The piece's horizontal slice at the actuator's height, as a circle, against the rectangle. */
    private boolean touchesPiece(double centreX, double centreY, double cos, double sin, GamePiece piece) {
        double dz = heightInches - piece.getFieldZInches();
        double radius = piece.getRadiusInches();
        if (Math.abs(dz) > radius) {
            return false;                          // the piece is wholly above or below the actuator
        }
        double sliceRadius = StrictMath.sqrt(radius * radius - dz * dz);
        double dx = piece.getFieldXInches() - centreX;
        double dy = piece.getFieldYInches() - centreY;
        double along = dx * cos + dy * sin;
        double across = -dx * sin + dy * cos;
        double gapAlong = Math.max(0.0, Math.abs(along) - depthInches / 2.0);
        double gapAcross = Math.max(0.0, Math.abs(across) - widthInches / 2.0);
        return gapAlong * gapAlong + gapAcross * gapAcross <= sliceRadius * sliceRadius;
    }
}
