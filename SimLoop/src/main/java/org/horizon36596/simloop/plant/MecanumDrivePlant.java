package org.horizon36596.simloop.plant;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.horizon36596.simloop.config.DrivetrainSimConfig;
import org.horizon36596.simloop.fakehardware.FakeMotor;

/**
 * Closes the sim odometry loop for a mecanum drivetrain: each tick it reads the four drive
 * {@link FakeMotor}s' modelled wheel velocities and integrates the pose through
 * {@link MecanumPoseIntegrator}. Because the season drivetrain's inverse kinematics (which commanded the
 * motor powers) and this forward-kinematics integrator share the {@code [frontLeft, frontRight, backLeft,
 * backRight]} wheel order and convention (the IK is the exact inverse of this FK), a forward chassis
 * command drives the pose forward: the sim odometry is self-consistent with what was commanded.
 *
 * <p>Season-agnostic core: it depends only on {@link FakeMotor} + {@link DrivetrainSimConfig}. The glue
 * that exposes this pose through the season's odometry device (e.g. an OctoQuad fake) lives in the season
 * layer. Deterministic: state changes only in {@link #update(double)}; getters are pure reads.
 *
 * <p><b>The pose is in the localizer frame</b> ({@code +x} forward at heading 0, {@code +y} left), which
 * is ninety degrees from the field frame that {@code org.horizon36596.simloop.field} places game pieces
 * and trigger volumes in. Both are deliberate: the localizer frame is what an odometry device reports,
 * the field frame is what a human reads off a field drawing. Rotate with
 * {@link org.horizon36596.simloop.field.FieldFrameTransform} at the boundary; a pose passed across
 * unrotated puts the robot somewhere plausible, facing the wrong way.
 */
public final class MecanumDrivePlant {

    private final FakeMotor frontLeft;
    private final FakeMotor frontRight;
    private final FakeMotor backLeft;
    private final FakeMotor backRight;
    private final double ticksPerInch;
    private final MecanumPoseIntegrator integrator;

    /**
     * Per-wheel physical mounting signs, {@code [FL, FR, BL, BR]} — see
     * {@link DrivetrainSimConfig#wheelMountingSigns()}. Applied to each motor's modelled shaft velocity on
     * the way into the forward kinematics, because the integrator's convention is "positive means this
     * corner is driving the chassis forward" and a mirrored motor's shaft turns the other way to do that.
     */
    private final double[] mountingSigns;

    /**
     * Wires the plant to the four fake motors the robot code already drives.
     *
     * @param motors drive motors in {@code [frontLeft, frontRight, backLeft, backRight]} order — the same
     *               order the season drivetrain's IK writes and {@link MecanumPoseIntegrator} reads.
     * @param config the drivetrain's geometry, ticks-per-inch and per-wheel mounting signs
     * @throws IllegalArgumentException if there are not exactly four motors, {@code ticksPerInch} is not a
     *                                  finite positive number, the mounting signs are not four values of
     *                                  {@code +1}/{@code -1}, or {@link MecanumPoseIntegrator} rejects the
     *                                  geometry
     */
    public MecanumDrivePlant(FakeMotor[] motors, DrivetrainSimConfig config) {
        if (motors == null || motors.length != 4) {
            throw new IllegalArgumentException("mecanum drive needs exactly 4 motors [FL, FR, BL, BR]");
        }
        double tpi = config.ticksPerInch();
        // isFinite first: NaN fails every comparison, so `tpi <= 0` alone would let it through and every
        // wheel speed divided by it, and so the whole pose, would be NaN.
        if (!Double.isFinite(tpi) || tpi <= 0) {
            throw new IllegalArgumentException("ticksPerInch must be a finite positive number, got " + tpi);
        }
        this.frontLeft = motors[0];
        this.frontRight = motors[1];
        this.backLeft = motors[2];
        this.backRight = motors[3];
        this.ticksPerInch = tpi;
        double[] signs = config.wheelMountingSigns();
        if (signs == null || signs.length != 4) {
            throw new IllegalArgumentException(
                    "wheelMountingSigns must return exactly 4 values [FL, FR, BL, BR], got "
                            + (signs == null ? "null" : String.valueOf(signs.length)));
        }
        for (int wheel = 0; wheel < 4; wheel++) {
            // Only +1 and -1 mean anything: a motor is either mounted this way round or the other way.
            // A 0.9 here would silently become a per-wheel gain — a scaling fudge factor hiding in a field
            // whose javadoc says "a fact about the metal" — so it is rejected rather than interpreted.
            if (signs[wheel] != 1.0 && signs[wheel] != -1.0) {
                throw new IllegalArgumentException(
                        "wheelMountingSigns must be exactly +1 or -1 (which way the motor is bolted in);"
                                + " wheel " + wheel + " was " + signs[wheel]);
            }
        }
        this.mountingSigns = signs.clone();
        this.integrator = new MecanumPoseIntegrator(config);
    }

    /**
     * Integrate the pose one tick from the motors' current modelled PHYSICAL wheel velocities (ticks/s -&gt;
     * in/s), each corrected for how that motor is physically mounted. Reads {@code getPhysicalVelocityTicksPerSecond()}
     * rather than the SDK-reported {@code getVelocityTicksPerSecond()}: this plant needs the
     * shaft's true physical rotation to combine with {@link #mountingSigns}, not the software-direction
     * abstraction a real robot's own code would read.
     *
     * <p>It also tells the integrator whether the drivetrain is <b>floating</b> this tick — every motor
     * commanded (near) zero power with {@code FLOAT} as its zero-power behaviour — because that is when a
     * real chassis coasts on its own drag rather than being driven or braked
     * ({@link DrivetrainSimConfig#naturalForwardDecelerationInchesPerSecondSquared()}).
     *
     * @param deltaTime the tick length, in seconds
     */
    public void update(double deltaTime) {
        integrator.integrate(
                mountingSigns[0] * frontLeft.getPhysicalVelocityTicksPerSecond() / ticksPerInch,
                mountingSigns[1] * frontRight.getPhysicalVelocityTicksPerSecond() / ticksPerInch,
                mountingSigns[2] * backLeft.getPhysicalVelocityTicksPerSecond() / ticksPerInch,
                mountingSigns[3] * backRight.getPhysicalVelocityTicksPerSecond() / ticksPerInch,
                isFloating(frontLeft) && isFloating(frontRight) && isFloating(backLeft) && isFloating(backRight),
                deltaTime);
    }

    /**
     * Below this commanded power, as a fraction of full power, a drive motor is taken as off: unitless.
     * SET BY HAND to the fake motor's own stopped-shaft threshold (2% of full power, {@code FakeMotor}'s
     * {@code EPS}), so "off" means the same thing to the motor model and to this plant. Its job is to stop a
     * follower's rounding residue — a power of {@code 1e-17} — from reading as a drive command.
     */
    private static final double FLOATING_POWER_THRESHOLD = 0.02;

    /** {@return true when this motor is commanded (near) zero power and set to {@code FLOAT} at zero power} */
    private static boolean isFloating(FakeMotor motor) {
        return Math.abs(motor.getPower()) < FLOATING_POWER_THRESHOLD
                && motor.getZeroPowerBehavior() == DcMotor.ZeroPowerBehavior.FLOAT;
    }

    /** {@return the localizer-frame X position, in inches — {@code +x} is forward at heading 0} */
    public double getX() { return integrator.getX(); }

    /** {@return the localizer-frame Y position, in inches — {@code +y} is left at heading 0} */
    public double getY() { return integrator.getY(); }

    /**
     * {@return the heading in radians, CCW-positive, measured from localizer {@code +x}; not wrapped, so it
     * grows without bound while the robot keeps turning}
     */
    public double getHeading() { return integrator.getHeading(); }

    /** {@return the chassis's forward velocity, robot frame ({@code +} forward), in inches/second} */
    public double getForwardVelocity() { return integrator.getForwardVelocity(); }

    /** {@return the chassis's sideways velocity, robot frame ({@code +} left), in inches/second} */
    public double getLateralVelocity() { return integrator.getLateralVelocity(); }

    /**
     * Teleport the integrated pose (e.g. to seed a routine's start pose).
     *
     * @param x       localizer-frame X position (forward at heading 0), in inches
     * @param y       localizer-frame Y position (left at heading 0), in inches
     * @param heading heading in radians, CCW-positive from localizer {@code +x}
     */
    public void setPose(double x, double y, double heading) {
        integrator.setPose(x, y, heading);
    }

    /**
     * Remove the part of the chassis velocity that points into a contact; see
     * {@link MecanumPoseIntegrator#removeVelocityInto}.
     *
     * @param normalX localizer-frame x of the contact normal, pointing from the obstacle toward the robot
     * @param normalY localizer-frame y of the contact normal
     */
    public void removeVelocityInto(double normalX, double normalY) {
        integrator.removeVelocityInto(normalX, normalY);
    }
}
