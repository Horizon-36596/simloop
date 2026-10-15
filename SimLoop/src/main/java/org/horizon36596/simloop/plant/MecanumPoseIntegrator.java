package org.horizon36596.simloop.plant;

import org.horizon36596.simloop.config.DrivetrainSimConfig;

/**
 * Kinematic mecanum forward-kinematics pose integrator. Given the four wheel linear speeds (in/s) it
 * computes chassis velocity, integrates the pose, and clamps it to the field bounds.
 *
 * <p>Kinematic only — no contact forces or rigid-body physics: the chassis goes exactly where its wheels
 * say, and the field wall is a clamp, not a collision. Geometry comes from the season
 * {@link DrivetrainSimConfig}; the core hard-codes no season numbers.
 *
 * <p><b>The chassis has a velocity of its own.</b> It chases what the wheels ask for, but no faster per
 * axis than the config's traction limits, and while the motors float at zero power it coasts down at the
 * config's natural decelerations instead of following the wheels' much faster decay. All four numbers are
 * fixed constants — ceilings and rates, the same kind of thing as the top speeds — not a force model: nothing
 * here computes a normal load or a friction coefficient. Left at their defaults they do nothing, and the
 * chassis goes exactly where its wheels say, as it did before they existed. See
 * {@link #integrate(double, double, double, double, boolean, double)}.
 *
 * <p>Wheel order is [frontLeft, frontRight, backLeft, backRight].
 *
 * <p><b>The frame is the localizer frame, not the field frame</b>: {@code +x} forward
 * at heading 0, {@code +y} left, heading CCW-positive, radians, inches. That is the frame a dead-wheel
 * odometry device reports, and this class integrates in it because in a real robot such a device is what
 * feeds the same numbers. It is <b>ninety degrees from the field frame</b> that game pieces and trigger
 * volumes are placed in ({@code +X} right, {@code +Y} forward). Rotate with
 * {@link org.horizon36596.simloop.field.FieldFrameTransform} before asking a field question of a pose
 * from here; passing one straight across is wrong in a way that still looks plausible.
 */
public final class MecanumPoseIntegrator {

    /** (wheelBase/2 + trackWidth/2), in inches: the mecanum lever arm that turns wheel speed into turn rate. */
    private final double halfSum;
    /** Half the field's right-left extent (field X), in inches. Bounds localizer y, which is field -X. */
    private final double fieldHalfWidth;
    /** Half the field's forward-back extent (field Y), in inches. Bounds localizer x, which is field +Y. */
    private final double fieldHalfHeight;

    /**
     * How much of the wheels' sideways contribution actually reaches the floor, as a fraction in
     * {@code (0, 1]} — {@link DrivetrainSimConfig#maxLateralVelocityTicksPerSecond()} divided by
     * {@link DrivetrainSimConfig#maxVelocityTicksPerSecond()}.
     *
     * <p>This is the mecanum's roller slip, and it is the one place this integrator stops being pure
     * geometry. The forward-kinematics sum below says what the wheels are <i>asking</i> the chassis to do
     * sideways; this factor says how much of that ask the rollers deliver. 1.0 means a robot that strafes
     * exactly as fast as it drives, which is what this class assumed before the factor existed.
     *
     * <p>It multiplies <b>only</b> the lateral velocity. Forward speed and rotation are unaffected, because
     * both are produced by the wheels rolling in their primary direction, where nothing slips.
     *
     * <p><b>This is not a friction model, and the drivetrain stays kinematic.</b> Worth saying
     * plainly, because the words "slip" and "rollers" invite the opposite reading. Nothing here computes a
     * force, a normal load, or a contact patch: it is one measured constant, fixed for the life of the
     * object, multiplied once per tick — the same kind of thing as the top-speed ceiling the config already
     * supplies. If a future change wants slip that varies with load, speed or surface, that is rigid-body
     * physics, which this library deliberately does not model: stop and flag it rather than write it here.
     */
    private final double lateralEfficiency;

    /** Forward coast deceleration while the motors float, in/s²; 0 means not modelled (see the config). */
    private final double naturalForwardDeceleration;
    /** Sideways coast deceleration while the motors float, in/s²; 0 means not modelled (see the config). */
    private final double naturalLateralDeceleration;
    /** Most the forward velocity may change per second, in/s² (traction); may be infinite. */
    private final double maxForwardAcceleration;
    /** Most the sideways velocity may change per second, in/s² (traction); may be infinite. */
    private final double maxLateralAcceleration;

    private double x = 0.0;
    private double y = 0.0;
    private double heading = 0.0;

    /**
     * The chassis's own velocity, robot frame, in/s: forward and left. This is the chassis's momentum, which
     * is not the same thing as what its wheels are asking for: it chases the wheels' request no faster than
     * the traction limits allow, and while the motors float it ignores them and coasts down at the natural
     * decelerations. With every one of those four set to "not modelled" it equals the wheels' request every
     * tick, which is exactly how this class behaved before it had a velocity.
     */
    private double forwardVelocity = 0.0;
    private double lateralVelocity = 0.0;

    /**
     * Builds an integrator sitting at the field origin, facing localizer {@code +x}.
     *
     * @param config the drivetrain's geometry and speed ceilings; wheel base and track width set the
     *               rotation arm, the two field half-extents set the clamp, and the two velocity ceilings
     *               set the roller-slip factor
     * @throws IllegalArgumentException if wheel base, track width, either field half-extent or either
     *                                  velocity ceiling is not a finite positive number, or the lateral
     *                                  ceiling exceeds the forward one. Each of these would otherwise
     *                                  poison the pose silently: a zero lever arm makes the turn rate
     *                                  {@code 0/0 = NaN}, a negative field bound makes the clamp teleport
     *                                  the robot every tick, and a NaN slips past a plain {@code <= 0}
     *                                  check. A NaN pose can never be clamped back onto the field. Also
     *                                  if a natural deceleration is negative or not finite, or a traction
     *                                  limit is NaN or not positive (infinite is allowed: no limit).
     */
    public MecanumPoseIntegrator(DrivetrainSimConfig config) {
        double wheelBase = requireFinitePositive(config.wheelBase(), "wheelBase");
        double trackWidth = requireFinitePositive(config.trackWidth(), "trackWidth");
        this.halfSum = (wheelBase / 2.0) + (trackWidth / 2.0);
        this.fieldHalfWidth = requireFinitePositive(config.fieldHalfWidth(), "fieldHalfWidth");
        this.fieldHalfHeight = requireFinitePositive(config.fieldHalfHeight(), "fieldHalfHeight");

        double forward = requireFinitePositive(config.maxVelocityTicksPerSecond(), "maxVelocityTicksPerSecond");
        double lateral = requireFinitePositive(
                config.maxLateralVelocityTicksPerSecond(), "maxLateralVelocityTicksPerSecond");
        if (lateral > forward) {
            throw new IllegalArgumentException(
                    "maxLateralVelocityTicksPerSecond must be no greater than the forward"
                            + " figure (a mecanum is never faster sideways than forwards); forward was "
                            + forward + ", lateral was " + lateral);
        }
        this.lateralEfficiency = lateral / forward;

        this.naturalForwardDeceleration = requireFiniteNonNegative(
                config.naturalForwardDecelerationInchesPerSecondSquared(),
                "naturalForwardDecelerationInchesPerSecondSquared");
        this.naturalLateralDeceleration = requireFiniteNonNegative(
                config.naturalLateralDecelerationInchesPerSecondSquared(),
                "naturalLateralDecelerationInchesPerSecondSquared");
        this.maxForwardAcceleration = requirePositiveLimit(
                config.maxForwardAccelerationInchesPerSecondSquared(),
                "maxForwardAccelerationInchesPerSecondSquared");
        this.maxLateralAcceleration = requirePositiveLimit(
                config.maxLateralAccelerationInchesPerSecondSquared(),
                "maxLateralAccelerationInchesPerSecondSquared");
    }

    /**
     * Advance the pose by one tick from the four wheel linear speeds, with the motors driving (not floating):
     * the same as {@link #integrate(double, double, double, double, boolean, double)} with
     * {@code motorsFloating} false.
     *
     * <p><b>How the tick is integrated.</b> The chassis velocity is constant across the tick, but the
     * heading turns during it, so the direction the robot travels in turns too. The robot-frame velocity
     * is rotated into the localizer frame at the heading <b>halfway through the tick</b> (the midpoint
     * rule), which points the step along the chord of the arc the robot really drives. Using the heading
     * at the start of the tick instead would aim every step a little behind the turn, so a robot driving
     * an arc would drift to the outside of it by an amount that grows with the tick size.
     *
     * <p>A tick that is not forward in time — zero, negative, NaN or infinite {@code deltaTime} — is
     * ignored, the same rule the mechanism plants follow: the sim clock only moves forward, so such a
     * tick is not a real tick, and letting it through would run the robot backwards or poison the pose.
     *
     * @param frontLeft  front-left wheel's linear speed at the floor, in inches/second
     * @param frontRight front-right wheel's linear speed at the floor, in inches/second
     * @param backLeft   back-left wheel's linear speed at the floor, in inches/second
     * @param backRight  back-right wheel's linear speed at the floor, in inches/second
     * @param deltaTime  the tick length, in seconds
     * @throws IllegalArgumentException if any wheel speed is NaN or infinite. A wheel speed comes from a
     *                                  motor model that should never produce one, so it is a caller bug:
     *                                  it is refused loudly rather than zeroed, and the pose is left as it
     *                                  was, because a single NaN would otherwise poison the pose forever.
     */
    public void integrate(double frontLeft, double frontRight, double backLeft, double backRight, double deltaTime) {
        integrate(frontLeft, frontRight, backLeft, backRight, false, deltaTime);
    }

    /**
     * Advance the pose by one tick from the four wheel linear speeds, saying whether the drive motors are
     * floating at zero power this tick.
     *
     * <p><b>What the chassis velocity does.</b> The wheels' forward kinematics give the velocity the wheels
     * are <i>asking</i> for. The chassis's own velocity then moves toward that request, each axis by no more
     * than its traction limit times {@code deltaTime} — that is the most the floor can push. While
     * {@code motorsFloating} is true, an axis whose natural deceleration is configured ignores the wheels and
     * slows toward zero at that deceleration instead, which is how a real floating drivetrain coasts. An
     * axis left at "not modelled" follows the wheels as before. Rotation is not limited: the turn rate is
     * the wheels' geometry exactly, as it always was.
     *
     * <p>Before the velocity moves, it is turned with the chassis: momentum belongs to the field, so when the
     * robot turns by {@code omega × deltaTime} its robot-frame velocity turns the other way by the same
     * angle. Without that, a robot sliding sideways while turning would carry its slide round with it.
     *
     * <p><b>A known limit.</b> While the chassis coasts, the motor model's wheels keep decaying at their own
     * rate, so when power comes back the wheels ask for less than the chassis is doing and the chassis is
     * pulled down toward them, at no more than the traction limit. A real floating wheel rolls with the
     * chassis; this one does not, because the motor model has no way to be told the floor is turning it.
     *
     * @param frontLeft      front-left wheel's linear speed at the floor, in inches/second
     * @param frontRight     front-right wheel's linear speed at the floor, in inches/second
     * @param backLeft       back-left wheel's linear speed at the floor, in inches/second
     * @param backRight      back-right wheel's linear speed at the floor, in inches/second
     * @param motorsFloating true when every drive motor is commanded zero power with {@code FLOAT} as its
     *                       zero-power behaviour, so the chassis coasts rather than being driven or braked
     * @param deltaTime      the tick length, in seconds; a non-positive or non-finite tick is ignored
     * @throws IllegalArgumentException if any wheel speed is NaN or infinite (see the five-argument form)
     */
    public void integrate(double frontLeft, double frontRight, double backLeft, double backRight,
            boolean motorsFloating, double deltaTime) {
        if (!Double.isFinite(deltaTime) || deltaTime <= 0.0) {
            return;
        }
        requireFiniteWheelSpeed(frontLeft, "frontLeft");
        requireFiniteWheelSpeed(frontRight, "frontRight");
        requireFiniteWheelSpeed(backLeft, "backLeft");
        requireFiniteWheelSpeed(backRight, "backRight");

        // Mecanum forward kinematics (robot frame): what the wheels ask for, forward and left, and CCW omega.
        double requestedForward = (frontLeft + frontRight + backLeft + backRight) / 4.0;
        double requestedLateral = (-frontLeft + frontRight + backLeft - backRight) / 4.0;
        double omega = (-frontLeft + frontRight - backLeft + backRight) / (4.0 * halfSum);

        // Roller slip: the wheels ask for vy sideways, the rollers deliver rather less of it. Applied here
        // and only to vy, so forward motion and rotation stay exact geometry. See lateralEfficiency.
        requestedLateral *= lateralEfficiency;

        // Momentum stays put in the field while the robot turns under it, so its robot-frame components
        // turn the other way (see the javadoc). With no traction limit and no coast the next two steps
        // overwrite the velocity with the request exactly, so this changes nothing for such a config.
        double turnThisTick = omega * deltaTime;
        if (turnThisTick != 0.0) {
            double turnCos = StrictMath.cos(turnThisTick);
            double turnSin = StrictMath.sin(turnThisTick);
            double turnedForward = forwardVelocity * turnCos + lateralVelocity * turnSin;
            double turnedLateral = -forwardVelocity * turnSin + lateralVelocity * turnCos;
            forwardVelocity = turnedForward;
            lateralVelocity = turnedLateral;
        }

        forwardVelocity = stepAxisVelocity(forwardVelocity, requestedForward, motorsFloating,
                naturalForwardDeceleration, maxForwardAcceleration, deltaTime);
        lateralVelocity = stepAxisVelocity(lateralVelocity, requestedLateral, motorsFloating,
                naturalLateralDeceleration, maxLateralAcceleration, deltaTime);
        double vx = forwardVelocity;
        double vy = lateralVelocity;

        // Rotate robot-frame velocity into the localizer frame at the MIDPOINT heading of the tick (see
        // the javadoc above), then integrate.
        double midpointHeading = heading + 0.5 * omega * deltaTime;
        double cos = Math.cos(midpointHeading);
        double sin = Math.sin(midpointHeading);
        x += (vx * cos - vy * sin) * deltaTime;
        y += (vx * sin + vy * cos) * deltaTime;
        heading += omega * deltaTime;

        // Kinematic field clamp. The config names its bounds in the FIELD frame (width is field X,
        // right-left; height is field Y, forward-back), and this pose is in the LOCALIZER frame, where
        // localizer x is field +Y and localizer y is field -X. So localizer x is bounded by the
        // half-HEIGHT and localizer y by the half-WIDTH. On a square field the two are equal.
        x = clamp(x, -fieldHalfHeight, fieldHalfHeight);
        y = clamp(y, -fieldHalfWidth, fieldHalfWidth);
    }

    /** {@return the localizer-frame X position (forward at heading 0, field +Y), in inches, within +-fieldHalfHeight} */
    public double getX() { return x; }

    /** {@return the localizer-frame Y position (left at heading 0, field -X), in inches, within +-fieldHalfWidth} */
    public double getY() { return y; }

    /**
     * {@return the heading in radians, CCW-positive, measured from localizer {@code +x}; not wrapped, so it
     * grows without bound while the robot keeps turning}
     */
    public double getHeading() { return heading; }

    /** {@return the chassis's forward velocity, robot frame ({@code +} forward), in inches/second} */
    public double getForwardVelocity() { return forwardVelocity; }

    /** {@return the chassis's sideways velocity, robot frame ({@code +} left), in inches/second} */
    public double getLateralVelocity() { return lateralVelocity; }

    /**
     * Teleports the pose, for a test that needs to start somewhere other than the origin.
     *
     * <p>The chassis velocity is <b>kept</b>: a season's contact code moves the pose with this to push the
     * robot out of a wall, and a robot sliding along a wall keeps sliding. Use {@link #reset()} to stop it.
     *
     * @param x       localizer-frame X position (forward at heading 0), in inches
     * @param y       localizer-frame Y position (left at heading 0), in inches
     * @param heading heading in radians, CCW-positive from localizer {@code +x}
     */
    public void setPose(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.heading = heading;
    }

    /**
     * Stops the chassis moving into something it has hit: the part of its velocity that points against
     * {@code (normalX, normalY)} is removed, and the rest is kept, so a robot driven into a wall at an angle
     * slides along it instead of keeping its speed into it. A velocity already pointing away from the contact
     * is left alone.
     *
     * <p>This is the velocity half of a contact; the season's contact code moves the pose out with
     * {@link #setPose}. Without it the chassis's own velocity (which the traction limit only changes slowly)
     * would keep carrying the robot into the wall at full speed every tick, to be pushed back out every tick.
     * The wheels are not told: the next tick they still ask for their speed, and the chassis chases it again at
     * no more than the traction limit, which is how a driven robot scrubs against a wall.
     *
     * @param normalX localizer-frame x of the contact normal, pointing out of the obstacle toward the robot;
     *                any length, need not be unit
     * @param normalY localizer-frame y of the normal
     */
    public void removeVelocityInto(double normalX, double normalY) {
        double length = StrictMath.sqrt(normalX * normalX + normalY * normalY);
        if (!(length > 0.0) || !Double.isFinite(length)) {
            return;
        }
        // The normal in the robot frame (forward, left), at the current heading.
        double cos = StrictMath.cos(heading);
        double sin = StrictMath.sin(heading);
        double normalForward = (normalX * cos + normalY * sin) / length;
        double normalLeft = (-normalX * sin + normalY * cos) / length;
        double along = forwardVelocity * normalForward + lateralVelocity * normalLeft;
        if (along < 0.0) {
            forwardVelocity -= along * normalForward;
            lateralVelocity -= along * normalLeft;
        }
    }

    /** Returns the pose to the origin, facing localizer {@code +x} (robot forward), with the chassis at rest. */
    public void reset() {
        setPose(0.0, 0.0, 0.0);
        forwardVelocity = 0.0;
        lateralVelocity = 0.0;
    }

    /**
     * One axis of the chassis velocity, one tick on: coasting toward zero at the natural deceleration when
     * the motors float and that deceleration is configured, otherwise moving toward the wheels' request by
     * no more than the traction limit. With an infinite limit the request is returned exactly.
     *
     * @param velocity          the axis velocity now, in/s
     * @param requested         what the wheels ask for on this axis, in/s
     * @param motorsFloating    whether the drive motors float at zero power this tick
     * @param naturalDeceleration the axis's coast deceleration, in/s², 0 for not modelled
     * @param maxAcceleration   the axis's traction limit, in/s², possibly infinite
     * @param deltaTime         the tick, seconds
     * @return the axis velocity after the tick, in/s
     */
    private static double stepAxisVelocity(double velocity, double requested, boolean motorsFloating,
            double naturalDeceleration, double maxAcceleration, double deltaTime) {
        if (motorsFloating && naturalDeceleration > 0.0) {
            return moveToward(velocity, 0.0, naturalDeceleration * deltaTime);
        }
        return moveToward(velocity, requested, maxAcceleration * deltaTime);
    }

    /** {@code value} moved toward {@code target} by at most {@code maxStep}; {@code target} itself if within reach. */
    private static double moveToward(double value, double target, double maxStep) {
        double gap = target - value;
        if (Math.abs(gap) <= maxStep) {
            return target;
        }
        return value + Math.copySign(maxStep, gap);
    }

    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    // A deceleration: zero (not modelled) or a real positive size. NaN fails every comparison, hence isFinite.
    private static double requireFiniteNonNegative(double value, String name) {
        if (!Double.isFinite(value) || value < 0.0) {
            throw new IllegalArgumentException(name + " must be a finite number >= 0 (0 = not modelled), got "
                    + value);
        }
        return value;
    }

    // A traction limit: positive, and infinity is the legitimate "no limit".
    private static double requirePositiveLimit(double value, String name) {
        if (Double.isNaN(value) || value <= 0.0) {
            throw new IllegalArgumentException(name + " must be positive (infinity for no limit), got " + value);
        }
        return value;
    }

    // A config value that must be a real, positive size. NaN is caught here too: it fails every
    // comparison, so a plain `value <= 0` check would let it straight through.
    private static double requireFinitePositive(double value, String name) {
        if (!Double.isFinite(value) || value <= 0.0) {
            throw new IllegalArgumentException(name + " must be a finite positive number, got " + value);
        }
        return value;
    }

    private static void requireFiniteWheelSpeed(double inchesPerSecond, String wheel) {
        if (!Double.isFinite(inchesPerSecond)) {
            throw new IllegalArgumentException(
                    wheel + " wheel speed must be finite, got " + inchesPerSecond + " in/s");
        }
    }
}
