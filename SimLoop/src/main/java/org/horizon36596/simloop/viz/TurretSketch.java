package org.horizon36596.simloop.viz;

import org.psilynx.psikit.core.Logger;
import org.psilynx.psikit.core.wpi.math.Pose3d;
import org.psilynx.psikit.core.wpi.math.Rotation3d;

/**
 * Draws a {@link TurretSketchDefinition} in AdvantageScope's <b>3D Field</b> tab: a turret that yaws on the
 * robot, the chain riding on it (a hood), and a cone at the chain's tip pointing along the shot.
 *
 * <p><b>How a yaw gets drawn at all</b> ({@code docs/briefs/b13-stacked-dof-viz.md} §1.2, options a and b).
 * A {@code Mechanism2d} has one fixed plane, so it cannot turn. Three things are logged instead, all under
 * one key:
 * <ul>
 *   <li>{@code <key>/Ghost}, a {@code Pose3d}: the robot's field pose, turned by the turret's yaw about the
 *       turret's own axis. In the tab it is a <b>Ghost</b> object, and it draws a second robot body there.</li>
 *   <li>{@code <key>/Mechanism}, a {@code Mechanism2d}: the chain above the turret, in the turret's own plane.
 *       Attached to the ghost (on its XZ plane), it swings round with the turret.</li>
 *   <li>{@code <key>/Aim}, a {@code Pose3d} at the chain's tip whose +X points along the last segment: yaw
 *       plus pitch. In the tab it is a <b>Cone</b> object, with its position option set to Back so it grows
 *       out of the exit.</li>
 *   <li>{@code <key>/yawRadians}, {@code <key>/aimHeadingRadians} and {@code <key>/aimElevationRadians}: the
 *       drawn yaw (after clamping) and the aim direction as plain numbers, for a Line Graph beside the
 *       turret's own keys.</li>
 * </ul>
 *
 * <p><b>Frames.</b> Everything a caller passes in is the robot frame ({@code +X} forward, {@code +Y} left,
 * {@code +Z} up) or the field frame the robot's own pose is logged in (the one {@code Drive/pose2d} uses),
 * in inches and radians. The turret frame is internal: its origin is the yaw axis at floor level, its
 * {@code +X} is the way the turret faces ({@link TurretSketchDefinition#getFacingHeadingRadians()}), turned by
 * the current yaw. Poses are written in <b>metres</b>,
 * at the PsiKit boundary and nowhere else, through the same inches-to-metres conversion {@link MechanismSketch} uses.
 *
 * <p><b>Clamping.</b> The yaw is clamped to the yaw joint's limits, exactly as {@link MechanismSketch} clamps
 * every joint, so the drawing never shows the turret past its own hard stop. A continuous or multi-turn turret
 * (one on a slip ring, with no hard stop) is therefore not drawn past its declared range: wrap its angle into
 * that range before calling {@link #setYawRadians}.
 *
 * <p><b>Determinism and separability.</b> No wall-clock read; the drawn state is a pure function of the values
 * set. Trigonometry is {@link StrictMath}. No {@code teamcode} import.
 */
public final class TurretSketch {

    private final TurretSketchDefinition definition;

    /** The chain above the turret, drawn in the turret's plane; {@code null} for a bare turret. */
    private final MechanismSketch rideAlong;

    /**
     * Heading of the turret's {@code +X} at yaw zero, robot frame, radians CCW from robot forward: the plane the
     * chain above swings in ({@link TurretSketchDefinition#getFacingHeadingRadians()}).
     */
    private final double zeroHeadingRadians;

    /** {@code +1} if the yaw axis is {@code +Z} (a positive yaw turns CCW seen from above), else {@code -1}. */
    private final double yawSign;

    /** The yaw axis at floor level, robot frame, inches. The turret frame's origin. */
    private final double axisXInches;
    private final double axisYInches;

    /**
     * The yaw joint's tip in the turret frame, inches: {@code [0]} forward, {@code [1]} left, {@code [2]} up.
     * The chain above the turret mounts here.
     */
    private final double[] yawTipInTurretFrame;

    private double yawRadians;
    private double robotFieldXInches;
    private double robotFieldYInches;
    private double robotHeadingRadians;

    /**
     * Builds the drawing at the zero pose, with the robot at the field origin.
     *
     * @param definition what to draw
     * @throws IllegalArgumentException if {@code definition} is null, or if a joint above the turret does not
     *                                  lie in the turret's own vertical plane (which {@link MechanismSketch}
     *                                  would draw foreshortened)
     */
    public TurretSketch(TurretSketchDefinition definition) {
        if (definition == null) {
            throw new IllegalArgumentException("TurretSketch needs a definition to draw");
        }
        this.definition = definition;
        RevoluteJointDefinition yaw = definition.getYaw();
        RobotVector3 arm = yaw.getZeroPoseDirection();
        this.zeroHeadingRadians = definition.getFacingHeadingRadians();
        this.yawSign = yaw.getRotationAxis().z > 0.0 ? 1.0 : -1.0;

        RobotVector3 pivot = definition.getYawPivotInches();
        this.axisXInches = pivot.x;
        this.axisYInches = pivot.y;
        RobotVector3 tip = new RobotVector3(
                pivot.x + yaw.getArmLengthInches() * arm.x,
                pivot.y + yaw.getArmLengthInches() * arm.y,
                pivot.z + yaw.getArmLengthInches() * arm.z);
        RobotVector3 tipInTurret = toTurretFrame(
                new RobotVector3(tip.x - axisXInches, tip.y - axisYInches, tip.z), zeroHeadingRadians);
        this.yawTipInTurretFrame = new double[] {tipInTurret.x, tipInTurret.y, tipInTurret.z};

        this.rideAlong = definition.getJoints().isEmpty() ? null : new MechanismSketch(rideAlongDefinition());
        this.yawRadians = 0.0;
    }

    /**
     * Set the turret's yaw, radians from the zero pose, positive by the right-hand rule about the yaw axis.
     * Clamped to the yaw joint's limits; a non-finite value draws the turret at zero rather than throwing.
     * A turret with a slip ring (no hard stop, more than one turn of travel) must wrap its angle into the yaw
     * joint's range before calling, or the drawing stops at the limit while the real turret keeps going.
     *
     * @param angleRadians the turret's angle, radians from the zero pose
     */
    public void setYawRadians(double angleRadians) {
        this.yawRadians = definition.getYaw().clampAngle(angleRadians);
    }

    /**
     * Set a pivot riding on the turret, exactly as {@link MechanismSketch#setJointAngleRadians} does.
     *
     * @param jointName    the pivot's name
     * @param angleRadians angle from the CAD pose (rad)
     * @throws IllegalArgumentException if no joint above the turret has that name, or it is a stage
     */
    public void setJointAngleRadians(String jointName, double angleRadians) {
        requireRideAlong("setJointAngleRadians", jointName).setJointAngleRadians(jointName, angleRadians);
    }

    /**
     * Set a stage riding on the turret, exactly as {@link MechanismSketch#setJointExtensionInches} does.
     *
     * @param jointName       the stage's name
     * @param extensionInches extension from retracted (in)
     * @throws IllegalArgumentException if no joint above the turret has that name, or it is a pivot
     */
    public void setJointExtensionInches(String jointName, double extensionInches) {
        requireRideAlong("setJointExtensionInches", jointName).setJointExtensionInches(jointName, extensionInches);
    }

    /**
     * Where the robot is, in the field frame its own pose is logged in: inches and radians CCW, with the
     * robot's forward axis at the heading. A pose with any non-finite part is ignored, so the drawing stays
     * where it was rather than vanishing to NaN.
     *
     * @param xInches        robot centre, field X, inches
     * @param yInches        robot centre, field Y, inches
     * @param headingRadians robot heading, radians CCW
     */
    public void setRobotFieldPose(double xInches, double yInches, double headingRadians) {
        if (!Double.isFinite(xInches) || !Double.isFinite(yInches) || !Double.isFinite(headingRadians)) {
            return;
        }
        this.robotFieldXInches = xInches;
        this.robotFieldYInches = yInches;
        this.robotHeadingRadians = headingRadians;
    }

    /**
     * Publish the ghost pose, the ride-along mechanism and the aim pose under {@code key}. Call once per loop,
     * after everything above has been set.
     *
     * @param key the AdvantageScope log key, e.g. {@code "Turret3d"}
     */
    public void record(String key) {
        if (key == null || key.trim().isEmpty()) {
            throw new IllegalArgumentException("turret log key must be a non-blank string");
        }
        Logger.recordOutput(key + "/Ghost", ghostPose3d());
        Logger.recordOutput(key + "/Aim", aimPose3d());
        Logger.recordOutput(key + "/yawRadians", yawRadians);
        Logger.recordOutput(key + "/aimHeadingRadians", getAimHeadingRadians());
        Logger.recordOutput(key + "/aimElevationRadians", getAimElevationRadians());
        if (rideAlong != null) {
            rideAlong.record(key + "/Mechanism");
        }
    }

    /*
     * The two poses exactly as record() logs them, in METRES. Package-private on purpose: they are the test seam
     * for the one thing the named getters below cannot show, which is the logged rotation. A Cone points along
     * its pose's +X, so a test rotates +X by this pose's rotation and checks it rises with the shot, which is how
     * a pitch sign flipped the WPILib way round (positive nose-down) gets caught.
     */

    /** The ghost pose exactly as {@link #record} logs it: field frame, <b>metres</b>. */
    Pose3d ghostPose3d() {
        return new Pose3d(MechanismSketch.toMetres(getGhostFieldXInches()),
                MechanismSketch.toMetres(getGhostFieldYInches()), 0.0, new Rotation3d(0.0, 0.0, getGhostFieldHeadingRadians()));
    }

    /** The aim pose exactly as {@link #record} logs it: field frame, <b>metres</b>, +X along the shot. */
    Pose3d aimPose3d() {
        // WPILib's pitch is positive NOSE-DOWN, so a shot rising at +elevation has pitch -elevation.
        return new Pose3d(MechanismSketch.toMetres(getAimFieldXInches()),
                MechanismSketch.toMetres(getAimFieldYInches()), MechanismSketch.toMetres(getAimFieldZInches()),
                new Rotation3d(0.0, -getAimElevationRadians(), getAimHeadingRadians()));
    }

    /** {@return the turret's current clamped yaw, radians from the zero pose} */
    public double getYawRadians() {
        return yawRadians;
    }

    /**
     * The ghost's field X: the yaw axis, moved sideways by the chain's own sideways offset (a Mechanism2d drawn
     * on the ghost's XZ plane has no way to carry one), carried by the robot's pose.
     *
     * @return the ghost's field X, in inches
     */
    public double getGhostFieldXInches() {
        return ghostInField()[0];
    }

    /** {@return the ghost's field Y, in inches; see {@link #getGhostFieldXInches()}} */
    public double getGhostFieldYInches() {
        return ghostInField()[1];
    }

    /**
     * The ghost's heading: the robot's heading plus the way the turret faces now.
     *
     * @return the ghost's field heading, radians CCW
     */
    public double getGhostFieldHeadingRadians() {
        return robotHeadingRadians + turretHeadingInRobotRadians();
    }

    /**
     * The aim point's field X: the tip of the chain above the turret (the yaw arm's own tip for a bare turret).
     *
     * @return the aim point's field X, in inches
     */
    public double getAimFieldXInches() {
        return aimInField()[0];
    }

    /** {@return the aim point's field Y, in inches; see {@link #getAimFieldXInches()}} */
    public double getAimFieldYInches() {
        return aimInField()[1];
    }

    /** {@return the aim point's height above the floor, in inches} */
    public double getAimFieldZInches() {
        return rideAlong == null ? yawTipInTurretFrame[2] : rideAlong.getTipUpInches();
    }

    /** {@return the aim direction's heading: the ghost's, radians CCW in the field frame} */
    public double getAimHeadingRadians() {
        return getGhostFieldHeadingRadians();
    }

    /**
     * The aim direction's elevation: the direction the last joint above the turret points, measured in the
     * turret's own vertical plane from the turret's <b>facing</b> ({@link TurretSketchDefinition#getFacingHeadingRadians()},
     * turned by the yaw) toward straight up. Zero for a bare turret.
     *
     * <p>It is <b>not</b> folded into (-π/2, π/2]: a joint swung back past vertical, so it points behind the
     * turret, reads more than π/2 (and one pointing down and back, less than -π/2). {@link #getAimHeadingRadians()}
     * stays the turret's facing either way, and {@link #aimPose3d()} is still correct, because a pitch past
     * vertical turns the cone over the top. Do not read this number as "above horizontal" without that caveat.
     *
     * @return the aim elevation, radians from the turret's facing toward up
     */
    public double getAimElevationRadians() {
        return rideAlong == null ? 0.0 : rideAlong.getTipAngleRadians();
    }

    /**
     * The chain above the turret, as drawn in the turret's plane: canvas right is the turret's forward, canvas
     * up is world up. {@code null} for a bare turret.
     *
     * @return the ride-along sketch, or {@code null}
     */
    public MechanismSketch getRideAlongSketch() {
        return rideAlong;
    }

    /** {@return what this sketch was built from} */
    public TurretSketchDefinition getDefinition() {
        return definition;
    }

    /** The ghost's field position, inches: the yaw axis plus the chain's sideways offset, turned with it. */
    private double[] ghostInField() {
        double[] inRobot = turretToRobot(0.0, yawTipInTurretFrame[1], turretHeadingInRobotRadians());
        return robotToField(inRobot[0], inRobot[1]);
    }

    /** The aim point's field position (x, y), inches. */
    private double[] aimInField() {
        double forward = rideAlong == null ? yawTipInTurretFrame[0] : rideAlong.getTipAcrossInches();
        double[] inRobot = turretToRobot(forward, yawTipInTurretFrame[1], turretHeadingInRobotRadians());
        return robotToField(inRobot[0], inRobot[1]);
    }

    private double turretHeadingInRobotRadians() {
        return zeroHeadingRadians + yawSign * yawRadians;
    }

    /** A turret-frame (forward, left) offset from the yaw axis, as a robot-frame point, inches. */
    private double[] turretToRobot(double forward, double left, double turretHeadingInRobot) {
        double cos = StrictMath.cos(turretHeadingInRobot);
        double sin = StrictMath.sin(turretHeadingInRobot);
        return new double[] {
            axisXInches + forward * cos - left * sin,
            axisYInches + forward * sin + left * cos};
    }

    /** A robot-frame point as a field-frame point, inches. */
    private double[] robotToField(double robotX, double robotY) {
        double cos = StrictMath.cos(robotHeadingRadians);
        double sin = StrictMath.sin(robotHeadingRadians);
        return new double[] {
            robotFieldXInches + robotX * cos - robotY * sin,
            robotFieldYInches + robotX * sin + robotY * cos};
    }

    /**
     * The chain above the turret, re-stated in the turret frame: every direction and axis turned by the zero
     * heading, and the mount at the yaw tip. Canvas right is turret forward, canvas up is world up.
     */
    private MechanismSketchDefinition rideAlongDefinition() {
        MechanismSketchDefinition.Builder builder = MechanismSketchDefinition.builder(definition.getName())
                .canvasSizeInches(definition.getCanvasWidthInches(), definition.getCanvasHeightInches())
                .backgroundColorHex(definition.getBackgroundColorHex())
                .canvasAxes(new RobotVector3(1.0, 0.0, 0.0), new RobotVector3(0.0, 0.0, 1.0))
                .mountPointInches(new RobotVector3(yawTipInTurretFrame[0], 0.0, yawTipInTurretFrame[2]));
        for (JointDefinition joint : definition.getJoints()) {
            builder.addJoint(inTurretFrame(joint));
        }
        return builder.build();
    }

    private JointDefinition inTurretFrame(JointDefinition joint) {
        RobotVector3 direction = toTurretFrame(joint.getZeroPoseDirection(), zeroHeadingRadians).normalized();
        if (joint.getKind() == JointDefinition.Kind.LINEAR) {
            LinearJointDefinition stage = (LinearJointDefinition) joint;
            return new LinearJointDefinition(stage.getName(), direction, stage.getRetractedLengthInches(),
                    stage.getMinTravelInches(), stage.getMaxTravelInches(), stage.getColorHex(),
                    stage.getLineWeightPixels());
        }
        RevoluteJointDefinition pivot = (RevoluteJointDefinition) joint;
        return new RevoluteJointDefinition(pivot.getName(), direction,
                toTurretFrame(pivot.getRotationAxis(), zeroHeadingRadians).normalized(),
                pivot.getArmLengthInches(), pivot.getMinAngleRadians(), pivot.getMaxAngleRadians(),
                pivot.getColorHex(), pivot.getLineWeightPixels());
    }

    /** A robot-frame vector, turned about {@code +Z} by {@code -headingRadians}. */
    private static RobotVector3 toTurretFrame(RobotVector3 v, double headingRadians) {
        double cos = StrictMath.cos(headingRadians);
        double sin = StrictMath.sin(headingRadians);
        return new RobotVector3(v.x * cos + v.y * sin, -v.x * sin + v.y * cos, v.z);
    }

    private MechanismSketch requireRideAlong(String method, String jointName) {
        if (rideAlong == null) {
            throw new IllegalArgumentException(
                    "turret mechanism \"" + definition.getName() + "\" has nothing riding on it, so "
                            + method + " has no joint \"" + jointName + "\" to move; its only joint is the yaw \""
                            + definition.getYaw().getName() + "\" (use setYawRadians)");
        }
        return rideAlong;
    }
}
