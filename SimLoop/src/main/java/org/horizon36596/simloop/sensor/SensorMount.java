package org.horizon36596.simloop.sensor;

/**
 * Where a sensor sits on the robot and which way it looks. The frame is the one {@code CameraMount} and
 * {@code TriggerVolume} use, so one robot description serves the camera, the intake and the sensors:
 *
 * <ul>
 *   <li><b>Robot frame</b>: origin at the robot's centre projected onto the floor, {@code +forward} out the
 *       nose, {@code +left} out the left side, {@code +up} off the floor. Inches.</li>
 *   <li><b>Yaw</b>: radians, counter-clockwise about {@code +up} seen from above; 0 looks forward,
 *       {@code π/2} looks out the left side, {@code π} looks out the back.</li>
 *   <li><b>Pitch</b>: radians, <b>positive tilts the sensor down</b> toward the floor, as for a camera;
 *       {@code π/2} looks straight down, which is how a colour sensor over an intake slot is often mounted.</li>
 * </ul>
 *
 * <p><b>Field frame it is placed in.</b> {@link #fieldPosition} and {@link #fieldDirection} take the robot's
 * pose in the FTC-Cartesian field frame (+X right, +Y away from the audience, centre origin, inches, heading
 * radians CCW from +X, the robot's forward being {@code (cos heading, sin heading)}). A drivetrain plant
 * reports the localizer frame instead; rotate it first with {@code FieldFrameTransform}, as
 * {@code TriggerVolume} explains.
 *
 * <p>Immutable and pure; all trigonometry in {@link StrictMath} so a replay is bit-identical.
 */
public final class SensorMount {

    private final double forwardInches;
    private final double leftInches;
    private final double upInches;
    private final double yawRad;
    private final double pitchRad;

    /**
     * Validates and stores one mount.
     *
     * @param forwardInches the sensor face's position along robot forward, inches
     * @param leftInches    the sensor face's position along robot left, inches
     * @param upInches      the sensor face's height above the floor, inches (&ge; 0)
     * @param yawRad        radians CCW about up, 0 = looking forward
     * @param pitchRad      radians, positive = tilted down; must be in {@code [-π/2, π/2]}
     * @throws IllegalArgumentException on a non-finite number, a negative height, or a pitch past
     *                                  straight up or straight down
     */
    public SensorMount(double forwardInches, double leftInches, double upInches, double yawRad,
            double pitchRad) {
        requireFinite(forwardInches, "forwardInches");
        requireFinite(leftInches, "leftInches");
        requireFinite(upInches, "upInches");
        requireFinite(yawRad, "yawRad");
        requireFinite(pitchRad, "pitchRad");
        if (upInches < 0.0) {
            throw new IllegalArgumentException("a sensor cannot be below the floor, got upInches=" + upInches);
        }
        if (Math.abs(pitchRad) > Math.PI / 2.0) {
            throw new IllegalArgumentException("pitch must be between straight up and straight down, got "
                    + pitchRad + " rad");
        }
        this.forwardInches = forwardInches;
        this.leftInches = leftInches;
        this.upInches = upInches;
        this.yawRad = yawRad;
        this.pitchRad = pitchRad;
    }

    /**
     * A level sensor looking straight out from the robot: the common mount for a distance sensor.
     *
     * @param forwardInches the sensor face's position along robot forward, inches
     * @param leftInches    the sensor face's position along robot left, inches
     * @param upInches      the sensor face's height above the floor, inches (&ge; 0)
     * @param yawRad        radians CCW about up, 0 = looking forward
     * @return a mount with zero pitch
     */
    public static SensorMount level(double forwardInches, double leftInches, double upInches, double yawRad) {
        return new SensorMount(forwardInches, leftInches, upInches, yawRad, 0.0);
    }

    /**
     * Where the sensor face is on the field, for a robot at the given pose.
     *
     * @param robotXInches    robot centre, field frame, inches
     * @param robotYInches    robot centre, field frame, inches
     * @param robotHeadingRad robot heading, radians CCW from field +X
     * @return {@code {x, y, z}} in the field frame, inches, z above the floor
     */
    double[] fieldPosition(double robotXInches, double robotYInches, double robotHeadingRad) {
        double cos = StrictMath.cos(robotHeadingRad);
        double sin = StrictMath.sin(robotHeadingRad);
        return new double[] {
                robotXInches + forwardInches * cos - leftInches * sin,
                robotYInches + forwardInches * sin + leftInches * cos,
                upInches,
        };
    }

    /**
     * The unit vector the sensor looks along on the field, for a robot at the given heading.
     *
     * @param robotHeadingRad robot heading, radians CCW from field +X
     * @return {@code {dx, dy, dz}} in the field frame, unit length, dz negative when tilted down
     */
    double[] fieldDirection(double robotHeadingRad) {
        double heading = robotHeadingRad + yawRad;
        double level = StrictMath.cos(pitchRad);
        return new double[] {
                level * StrictMath.cos(heading),
                level * StrictMath.sin(heading),
                -StrictMath.sin(pitchRad),
        };
    }

    /** {@return the sensor face's position along robot forward, inches} */
    public double getForwardInches() {
        return forwardInches;
    }

    /** {@return the sensor face's position along robot left, inches} */
    public double getLeftInches() {
        return leftInches;
    }

    /** {@return the sensor face's height above the floor, inches} */
    public double getUpInches() {
        return upInches;
    }

    /** {@return yaw, radians CCW about up, 0 = forward} */
    public double getYawRad() {
        return yawRad;
    }

    /** {@return pitch, radians, positive = tilted down} */
    public double getPitchRad() {
        return pitchRad;
    }

    /** {@return the mount, for test messages} */
    @Override
    public String toString() {
        return "SensorMount(fwd=" + forwardInches + " left=" + leftInches + " up=" + upInches
                + " yaw=" + yawRad + " pitch=" + pitchRad + ")";
    }

    static void requireFinite(double value, String name) {
        if (!Double.isFinite(value)) {
            throw new IllegalArgumentException(name + " must be finite, got " + value);
        }
    }
}
