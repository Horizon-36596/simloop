package org.horizon36596.simloop.vision;

/**
 * Where a camera sits on the robot and which way it points: the extrinsics. The frame and the sign
 * conventions are the usual ones for a ground-plane vision pipeline, so one description can serve both the
 * sim camera and a real pipeline:
 *
 * <ul>
 *   <li><b>Robot frame</b>: origin at the robot's centre of rotation <em>projected onto the floor</em>,
 *       {@code +forward} out the nose, {@code +left} out the left side, {@code +up} off the floor. Inches.</li>
 *   <li><b>Yaw</b>: radians, counter-clockwise about {@code +up} seen from above; 0 looks forward, {@code π}
 *       is a rear-facing camera.</li>
 *   <li><b>Pitch</b>: radians, <b>positive tilts the camera down</b> toward the floor. A camera at zero
 *       pitch never sees the floor directly in front of it.</li>
 *   <li><b>Optical frame</b> (the camera's own): the OpenCV convention, {@code +x} image right,
 *       {@code +y} image down, {@code +z} out along the optical axis. Roll is not modelled: the image's
 *       horizontal stays level with the floor.</li>
 * </ul>
 *
 * <p>The two frame conversions, {@link #robotToOptical} and {@link #opticalToRobot}, are each other's
 * exact inverse, and a test pins that. They are the whole of what a sim camera needs to project a piece
 * into pixels, and the whole of what a vision pipeline needs to put a pixel back on the floor.
 *
 * <p>Immutable and pure; all trigonometry in {@link StrictMath} for replay-identical results (domain R5).
 */
public final class CameraMount {

    private final double forwardInches;
    private final double leftInches;
    private final double upInches;
    private final double yawRad;
    private final double pitchRad;

    // The camera's three axes expressed in the robot frame, computed once. Optical +z (the boresight) is
    // "forward" rotated by yaw about up and then tilted down by pitch; optical +x (image right) is the
    // robot's right after the yaw; optical +y (image down) completes the right-handed set.
    private final double boresightForward;
    private final double boresightLeft;
    private final double boresightUp;
    private final double imageRightForward;
    private final double imageRightLeft;
    private final double imageDownForward;
    private final double imageDownLeft;
    private final double imageDownUp;

    /**
     * A camera fixed to the robot at the given position and angle, robot frame.
     *
     * @param forwardInches camera lens position along robot forward, inches
     * @param leftInches    camera lens position along robot left, inches
     * @param upInches      camera lens height above the floor, inches (&gt; 0)
     * @param yawRad        radians CCW about up, 0 = looking forward
     * @param pitchRad      radians, positive = tilted down; must be in {@code (-π/2, π/2)}
     * @throws IllegalArgumentException on a non-finite number, a non-positive height, or a pitch at or
     *                                  beyond straight up/down
     */
    public CameraMount(double forwardInches, double leftInches, double upInches, double yawRad,
            double pitchRad) {
        requireFinite(forwardInches, "forwardInches");
        requireFinite(leftInches, "leftInches");
        requireFinite(upInches, "upInches");
        requireFinite(yawRad, "yawRad");
        requireFinite(pitchRad, "pitchRad");
        if (upInches <= 0.0) {
            throw new IllegalArgumentException("a camera must be above the floor, got upInches=" + upInches);
        }
        if (Math.abs(pitchRad) >= Math.PI / 2.0) {
            throw new IllegalArgumentException("pitch must be strictly between straight down and straight"
                    + " up, got " + pitchRad + " rad");
        }
        this.forwardInches = forwardInches;
        this.leftInches = leftInches;
        this.upInches = upInches;
        this.yawRad = yawRad;
        this.pitchRad = pitchRad;

        double cosYaw = StrictMath.cos(yawRad);
        double sinYaw = StrictMath.sin(yawRad);
        double cosPitch = StrictMath.cos(pitchRad);
        double sinPitch = StrictMath.sin(pitchRad);

        // Level boresight after yaw, then tilt it down by pitch (down = -up).
        this.boresightForward = cosYaw * cosPitch;
        this.boresightLeft = sinYaw * cosPitch;
        this.boresightUp = -sinPitch;

        // Image right = robot right after yaw = -(left after yaw). Unaffected by pitch (no roll).
        this.imageRightForward = sinYaw;
        this.imageRightLeft = -cosYaw;

        // Image down = boresight tilted a further 90 degrees down: forward component -sinPitch along the
        // yawed forward, and up component -cosPitch.
        this.imageDownForward = -cosYaw * sinPitch;
        this.imageDownLeft = -sinYaw * sinPitch;
        this.imageDownUp = -cosPitch;
    }

    /**
     * A forward-facing camera on the robot's centreline: a common mount, one call to describe.
     *
     * @param forwardInches    camera lens position along robot forward, inches
     * @param upInches         camera lens height above the floor, inches (&gt; 0)
     * @param pitchDownDegrees degrees below horizontal, 0 = level
     * @return the mount, with yaw 0 (looking forward)
     */
    public static CameraMount front(double forwardInches, double upInches, double pitchDownDegrees) {
        return new CameraMount(forwardInches, 0.0, upInches, 0.0, StrictMath.toRadians(pitchDownDegrees));
    }

    /**
     * A point in the robot frame, seen from the camera: returns it in the optical frame.
     *
     * @param forwardInches the point along robot forward, inches
     * @param leftInches    the point along robot left, inches
     * @param upInches      the point above the floor, inches
     * @return {@code {x, y, z}} in the optical frame, inches: x image-right, y image-down, z along the
     *         boresight (positive = in front of the lens)
     */
    double[] robotToOptical(double forwardInches, double leftInches, double upInches) {
        double dForward = forwardInches - this.forwardInches;
        double dLeft = leftInches - this.leftInches;
        double dUp = upInches - this.upInches;
        double x = dForward * imageRightForward + dLeft * imageRightLeft;
        double y = dForward * imageDownForward + dLeft * imageDownLeft + dUp * imageDownUp;
        double z = dForward * boresightForward + dLeft * boresightLeft + dUp * boresightUp;
        return new double[] {x, y, z};
    }

    /**
     * A point in the optical frame, placed back in the robot frame. Exact inverse of
     * {@link #robotToOptical}.
     *
     * @param xInches optical x (image right), inches
     * @param yInches optical y (image down), inches
     * @param zInches optical z (along the boresight), inches
     * @return {@code {forward, left, up}} in the robot frame, inches
     */
    double[] opticalToRobot(double xInches, double yInches, double zInches) {
        // The three optical axes are orthonormal in the robot frame, so the inverse rotation is the
        // transpose: each robot component is the dot with that axis's column.
        double forward = xInches * imageRightForward + yInches * imageDownForward + zInches * boresightForward;
        double left = xInches * imageRightLeft + yInches * imageDownLeft + zInches * boresightLeft;
        double up = yInches * imageDownUp + zInches * boresightUp;
        return new double[] {forward + this.forwardInches, left + this.leftInches, up + this.upInches};
    }

    /** {@return the lens position along robot forward, inches} */
    public double getForwardInches() {
        return forwardInches;
    }

    /** {@return the lens position along robot left, inches} */
    public double getLeftInches() {
        return leftInches;
    }

    /** {@return the lens height above the floor, inches} */
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

    @Override
    public String toString() {
        return "CameraMount(fwd=" + forwardInches + " left=" + leftInches + " up=" + upInches
                + " yaw=" + yawRad + " pitch=" + pitchRad + ")";
    }

    private static void requireFinite(double value, String name) {
        if (!Double.isFinite(value)) {
            throw new IllegalArgumentException(name + " must be finite, got " + value);
        }
    }
}
