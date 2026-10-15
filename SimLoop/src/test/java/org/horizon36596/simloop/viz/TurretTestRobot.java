package org.horizon36596.simloop.viz;

/**
 * <b>SimLoop's turret test robot</b>: a hand-made fixture that exercises {@link TurretSketch} end to end. It is
 * <b>not</b> any season's robot and is not linked to one; it exists so the turret renderer can be run, logged
 * and looked at in AdvantageScope from inside SimLoop alone. A season robot draws its own turret from its own
 * CAD import.
 *
 * <p>Mirrors {@code src/test/resources/org/horizon36596/simloop/viz/turret-test-robot.mechanism.json}
 * (schema version 4, {@code source.tool: "hand"}). Both are written by hand; change that file first, then this one:
 * {@link TurretTestRobotTest} fails if they disagree.
 *
 * <p>The robot: a turret on a vertical axis 2 in behind the robot's centre, its bearing 8 in up. The hood's
 * hinge sits 6 in straight above the bearing, and the hood is 5 in long, swinging from 30° to 70° above
 * horizontal. Because the hood's hinge is directly above the axis, the yaw arm is vertical, which exercises the
 * rule that a turret faces the way the joint it carries swings.
 *
 * <p>Robot frame ({@code +X} forward, {@code +Y} left, {@code +Z} up), inches, radians. Every number here is
 * {@code SET BY HAND}: it is the definition of the test robot, not an estimate of a real one.
 */
final class TurretTestRobot {

    /** Root of the AdvantageScope log keys: {@code TestRobot/Turret3d/Ghost}, {@code .../Mechanism}, {@code .../Aim}. */
    static final String TURRET_LOG_KEY = "TestRobot/Turret3d";

    /**
     * The yaw joint's {@code stateKey}: the turret's reported angle, radians from the zero pose. Must equal the
     * yaw's {@code "stateKey"} in the JSON; {@link TurretTestRobotTest} checks it.
     */
    static final String YAW_STATE_KEY = "TestRobot/Turret/measuredRad";

    /**
     * The hood joint's {@code stateKey}: its joint angle, radians from the drawn pose (every REVOLUTE
     * {@code stateKey} is radians from the CAD pose). Must equal the hood's {@code "stateKey"} in the JSON.
     */
    static final String HOOD_STATE_KEY = "TestRobot/Hood/jointRad";

    /** Name of the hood joint. */
    static final String HOOD_JOINT = "hood";

    /** Yaw position {@code forward}, radians from the zero pose. SET BY HAND. */
    static final double YAW_FORWARD_RADIANS = 0.0;

    /** Yaw position {@code limitCw}, radians from the zero pose. SET BY HAND. */
    static final double YAW_LIMIT_CW_RADIANS = -3.0;

    /** Yaw position {@code limitCcw}, radians from the zero pose. SET BY HAND. */
    static final double YAW_LIMIT_CCW_RADIANS = 3.0;

    /** Hood position {@code low}: the drawn pose, radians from it. SET BY HAND. */
    static final double HOOD_LOW_RADIANS = 0.0;

    /** Hood position {@code high}: 40° above the drawn pose, radians from it. SET BY HAND. */
    static final double HOOD_HIGH_RADIANS = 0.6981317007977318;

    /** The hood angle the drawing is made at, degrees from horizontal: the {@code low} position. SET BY HAND. */
    static final double HOOD_DRAWN_POSE_DEG = 30.0;

    /** The turret bearing, a point on the yaw axis, robot frame, inches. SET BY HAND. */
    private static final RobotVector3 YAW_PIVOT_INCHES = new RobotVector3(-2.0, 0.0, 8.0);

    /** Yaw arm: straight up from the bearing to the hood hinge. SET BY HAND. */
    private static final RobotVector3 YAW_ARM_DIRECTION = new RobotVector3(0.0, 0.0, 1.0);

    /** Yaw arm length, bearing to hood hinge, inches. SET BY HAND. */
    private static final double YAW_ARM_LENGTH_INCHES = 6.0;

    /** Yaw axis: +Z, so a positive angle turns counter-clockwise seen from above. SET BY HAND. */
    private static final RobotVector3 YAW_ROTATION_AXIS = new RobotVector3(0.0, 0.0, 1.0);

    /** Hood direction in the drawn pose: 30° above horizontal, straight ahead. SET BY HAND. */
    private static final RobotVector3 HOOD_DRAWN_DIRECTION =
            new RobotVector3(0.8660254037844387, 0.0, 0.49999999999999994);

    /** Hood axis: -Y (robot-right), so a positive angle raises the exit. SET BY HAND. */
    private static final RobotVector3 HOOD_ROTATION_AXIS = new RobotVector3(0.0, -1.0, 0.0);

    /** Hood length, hinge to ball exit, inches. SET BY HAND. */
    private static final double HOOD_ARM_LENGTH_INCHES = 5.0;

    private TurretTestRobot() {
        // Static factory only.
    }

    /**
     * The hood's angle as a shooter states it, converted to the drawing's joint angle.
     *
     * @param hoodAngleDegFromHorizontal the hood's exit angle, degrees from horizontal
     * @return the hood joint angle, radians from the drawn pose
     */
    static double hoodJointRadiansFor(double hoodAngleDegFromHorizontal) {
        return Math.toRadians(hoodAngleDegFromHorizontal - HOOD_DRAWN_POSE_DEG);
    }

    /** {@return the test robot's turret, ready to hand to {@link TurretSketch}} */
    static TurretSketchDefinition create() {
        return TurretSketchDefinition.builder("TurretTestRobot")
                .canvasSizeInches(16.0, 24.0)
                .backgroundColorHex("#202020")
                .yaw(YAW_PIVOT_INCHES, new RevoluteJointDefinition(
                        "yaw", YAW_ARM_DIRECTION, YAW_ROTATION_AXIS, YAW_ARM_LENGTH_INCHES,
                        YAW_LIMIT_CW_RADIANS, YAW_LIMIT_CCW_RADIANS, "#FFD24C", 8.0))
                .addJoint(new RevoluteJointDefinition(
                        HOOD_JOINT, HOOD_DRAWN_DIRECTION, HOOD_ROTATION_AXIS, HOOD_ARM_LENGTH_INCHES,
                        HOOD_LOW_RADIANS, HOOD_HIGH_RADIANS, "#FF9F4C", 6.0))
                .build();
    }
}
