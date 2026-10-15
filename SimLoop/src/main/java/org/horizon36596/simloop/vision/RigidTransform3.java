package org.horizon36596.simloop.vision;

/**
 * A rigid transform in 3D: a rotation and then a translation, the pose of one frame expressed in another.
 * Read {@code T_AB} as "frame B's pose in frame A": {@link #apply} takes a point written in B and returns
 * it written in A, and {@code T_AB.compose(T_BC)} is {@code T_AC}.
 *
 * <p>Translation is in <b>inches</b>, the unit every SimLoop geometry type uses. The rotation is a proper
 * 3x3 rotation matrix whose columns are frame B's axes written in frame A.
 *
 * <p><b>Roll, pitch, yaw.</b> {@link #getRollRad()}, {@link #getPitchRad()} and {@link #getYawRad()}
 * decompose the rotation as {@code R = Rz(yaw) * Ry(pitch) * Rx(roll)}: roll about A's {@code x}, then pitch
 * about A's {@code y}, then yaw about A's {@code z}, each counter-clockwise when seen from the positive end
 * of its axis. That is WPILib's {@code Rotation3d(roll, pitch, yaw)} order, the order Limelight's six-number
 * pose arrays are read in. When pitch is within about 1e-9 of plus or minus 90 degrees roll is reported as
 * 0 and the whole remaining rotation as yaw.
 *
 * <p>Immutable and pure; all trigonometry in {@link StrictMath} for replay-identical results (domain R5).
 */
public final class RigidTransform3 {

    private static final RigidTransform3 IDENTITY =
            new RigidTransform3(new double[] {1, 0, 0, 0, 1, 0, 0, 0, 1}, 0.0, 0.0, 0.0);

    /** Row-major 3x3 rotation: {@code r[3 * row + col]}. */
    private final double[] r;
    private final double xInches;
    private final double yInches;
    private final double zInches;

    private RigidTransform3(double[] rowMajorRotation, double xInches, double yInches, double zInches) {
        this.r = rowMajorRotation;
        this.xInches = xInches;
        this.yInches = yInches;
        this.zInches = zInches;
    }

    /** {@return the transform that changes nothing} */
    public static RigidTransform3 identity() {
        return IDENTITY;
    }

    /**
     * A transform from a rotation given as the three axes of the child frame, written in the parent frame,
     * and a translation. The axes must be unit length and mutually perpendicular, and must form a
     * right-handed set; nothing else is a rotation.
     *
     * @param xAxis   the child frame's {@code x} axis in the parent frame, {@code {x, y, z}}
     * @param yAxis   the child frame's {@code y} axis in the parent frame
     * @param zAxis   the child frame's {@code z} axis in the parent frame
     * @param xInches the child frame's origin along the parent's {@code x}, inches
     * @param yInches the child frame's origin along the parent's {@code y}, inches
     * @param zInches the child frame's origin along the parent's {@code z}, inches
     * @return the transform
     * @throws IllegalArgumentException if a number is not finite, or the axes are not a right-handed
     *                                  orthonormal set to within 1e-9
     */
    public static RigidTransform3 fromAxes(double[] xAxis, double[] yAxis, double[] zAxis, double xInches,
            double yInches, double zInches) {
        double[] m = {
                xAxis[0], yAxis[0], zAxis[0],
                xAxis[1], yAxis[1], zAxis[1],
                xAxis[2], yAxis[2], zAxis[2],
        };
        for (double v : m) {
            requireFinite(v, "rotation entry");
        }
        requireFinite(xInches, "xInches");
        requireFinite(yInches, "yInches");
        requireFinite(zInches, "zInches");
        double[][] axes = {xAxis, yAxis, zAxis};
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                double dot = axes[i][0] * axes[j][0] + axes[i][1] * axes[j][1] + axes[i][2] * axes[j][2];
                if (Math.abs(dot - (i == j ? 1.0 : 0.0)) > 1e-9) {
                    throw new IllegalArgumentException("axes are not orthonormal (axis " + i + " . axis " + j
                            + " = " + dot + ")");
                }
            }
        }
        double det = m[0] * (m[4] * m[8] - m[5] * m[7]) - m[1] * (m[3] * m[8] - m[5] * m[6])
                + m[2] * (m[3] * m[7] - m[4] * m[6]);
        if (det < 0.0) {
            throw new IllegalArgumentException("axes are left-handed (a reflection, not a rotation)");
        }
        return new RigidTransform3(m, xInches, yInches, zInches);
    }

    /**
     * A transform from a translation and roll, pitch, yaw in the order this class decomposes them
     * ({@code R = Rz(yaw) * Ry(pitch) * Rx(roll)}).
     *
     * @param xInches  translation along the parent's {@code x}, inches
     * @param yInches  translation along the parent's {@code y}, inches
     * @param zInches  translation along the parent's {@code z}, inches
     * @param rollRad  rotation about the parent's {@code x}, radians
     * @param pitchRad rotation about the parent's {@code y}, radians
     * @param yawRad   rotation about the parent's {@code z}, radians
     * @return the transform
     */
    public static RigidTransform3 fromRollPitchYaw(double xInches, double yInches, double zInches,
            double rollRad, double pitchRad, double yawRad) {
        requireFinite(xInches, "xInches");
        requireFinite(yInches, "yInches");
        requireFinite(zInches, "zInches");
        requireFinite(rollRad, "rollRad");
        requireFinite(pitchRad, "pitchRad");
        requireFinite(yawRad, "yawRad");
        double cr = StrictMath.cos(rollRad);
        double sr = StrictMath.sin(rollRad);
        double cp = StrictMath.cos(pitchRad);
        double sp = StrictMath.sin(pitchRad);
        double cy = StrictMath.cos(yawRad);
        double sy = StrictMath.sin(yawRad);
        double[] m = {
                cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr,
                sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr,
                -sp, cp * sr, cp * cr,
        };
        return new RigidTransform3(m, xInches, yInches, zInches);
    }

    /**
     * A robot on the floor: a translation in the floor plane and a turn about {@code +z}. This is how a 2D
     * field pose ({@code x, y, heading}) becomes a 3D frame.
     *
     * @param xInches    along the parent's {@code x}, inches
     * @param yInches    along the parent's {@code y}, inches
     * @param headingRad counter-clockwise about {@code +z}, radians
     * @return the transform
     */
    public static RigidTransform3 planar(double xInches, double yInches, double headingRad) {
        return fromRollPitchYaw(xInches, yInches, 0.0, 0.0, 0.0, headingRad);
    }

    /**
     * {@code this * other}: if this is {@code T_AB} and {@code other} is {@code T_BC}, the result is
     * {@code T_AC}.
     *
     * @param other the transform to apply first
     * @return the composed transform
     */
    public RigidTransform3 compose(RigidTransform3 other) {
        double[] o = other.r;
        double[] m = new double[9];
        for (int row = 0; row < 3; row++) {
            for (int col = 0; col < 3; col++) {
                m[3 * row + col] = r[3 * row] * o[col] + r[3 * row + 1] * o[3 + col] + r[3 * row + 2] * o[6 + col];
            }
        }
        double[] t = apply(other.xInches, other.yInches, other.zInches);
        return new RigidTransform3(m, t[0], t[1], t[2]);
    }

    /** {@return the inverse: if this is {@code T_AB}, the result is {@code T_BA}} */
    public RigidTransform3 inverse() {
        double[] m = {
                r[0], r[3], r[6],
                r[1], r[4], r[7],
                r[2], r[5], r[8],
        };
        double x = -(m[0] * xInches + m[1] * yInches + m[2] * zInches);
        double y = -(m[3] * xInches + m[4] * yInches + m[5] * zInches);
        double z = -(m[6] * xInches + m[7] * yInches + m[8] * zInches);
        return new RigidTransform3(m, x, y, z);
    }

    /**
     * A point written in the child frame, written in the parent frame.
     *
     * @param x the point along the child's {@code x}, inches
     * @param y the point along the child's {@code y}, inches
     * @param z the point along the child's {@code z}, inches
     * @return {@code {x, y, z}} in the parent frame, inches
     */
    public double[] apply(double x, double y, double z) {
        return new double[] {
                r[0] * x + r[1] * y + r[2] * z + xInches,
                r[3] * x + r[4] * y + r[5] * z + yInches,
                r[6] * x + r[7] * y + r[8] * z + zInches,
        };
    }

    /**
     * One entry of the rotation matrix.
     *
     * @param row 0, 1 or 2: the parent axis
     * @param col 0, 1 or 2: the child axis
     * @return the component of the child's axis {@code col} along the parent's axis {@code row}, unitless
     */
    public double rotation(int row, int col) {
        return r[3 * row + col];
    }

    /** {@return the child origin along the parent's {@code x}, inches} */
    public double getXInches() {
        return xInches;
    }

    /** {@return the child origin along the parent's {@code y}, inches} */
    public double getYInches() {
        return yInches;
    }

    /** {@return the child origin along the parent's {@code z}, inches} */
    public double getZInches() {
        return zInches;
    }

    /** {@return the distance from the parent origin to the child origin, inches} */
    public double getDistanceInches() {
        return StrictMath.sqrt(xInches * xInches + yInches * yInches + zInches * zInches);
    }

    /** {@return pitch, radians in [-pi/2, pi/2], rotation about the parent's {@code y}} */
    public double getPitchRad() {
        return StrictMath.asin(Math.max(-1.0, Math.min(1.0, -r[6])));
    }

    /** {@return roll, radians in (-pi, pi], rotation about the parent's {@code x}; 0 at gimbal lock} */
    public double getRollRad() {
        if (isGimbalLocked()) {
            return 0.0;
        }
        return StrictMath.atan2(r[7], r[8]);
    }

    /** {@return yaw, radians in (-pi, pi], rotation about the parent's {@code z}} */
    public double getYawRad() {
        if (isGimbalLocked()) {
            return StrictMath.atan2(-r[1], r[4]);
        }
        return StrictMath.atan2(r[3], r[0]);
    }

    private boolean isGimbalLocked() {
        return Math.abs(r[6]) > 1.0 - 1e-9;
    }

    @Override
    public String toString() {
        return "RigidTransform3(t=(" + xInches + ", " + yInches + ", " + zInches + ") in, rpy=("
                + StrictMath.toDegrees(getRollRad()) + ", " + StrictMath.toDegrees(getPitchRad()) + ", "
                + StrictMath.toDegrees(getYawRad()) + ") deg)";
    }

    private static void requireFinite(double value, String name) {
        if (!Double.isFinite(value)) {
            throw new IllegalArgumentException(name + " must be finite, got " + value);
        }
    }
}
