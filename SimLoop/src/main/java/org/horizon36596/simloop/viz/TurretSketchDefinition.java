package org.horizon36596.simloop.viz;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * A mechanism whose base joint is a <b>turret</b>: a pivot that yaws about a vertical axis, carrying a planar
 * chain (a hood, an arm, a slide) that swings round with it. {@link TurretSketch} draws it in AdvantageScope's
 * <b>3D Field</b> tab.
 *
 * <p><b>Why this is not a {@link MechanismSketchDefinition}.</b> A {@code Mechanism2d} lives in one fixed
 * plane, and a yaw turns that plane, so {@link MechanismSketch} rejects a yaw joint
 * ({@code docs/briefs/b13-stacked-dof-viz.md} §1). This class is the other half of that design (§1.2): the yaw
 * is carried by a <b>ghost pose</b> that turns with the turret, and the chain above it is an ordinary
 * {@code Mechanism2d} drawn in the turret's own plane and attached to that ghost.
 *
 * <p><b>Same shape as the IR.</b> In {@code mechanism.json} this is one chain whose root is a
 * {@code REVOLUTE} joint with a vertical {@code rotationAxis}: the yaw. Every other joint rides on the yaw
 * joint's tip exactly as on any chain. So a generated class builds one of these from the same numbers it would
 * give a {@link MechanismSketchDefinition}, with nothing converted.
 *
 * <p><b>Frame and units.</b> The robot frame ({@code +X} forward, {@code +Y} left, {@code +Z} up, origin at
 * floor level in the middle of the drivetrain), inches, and radians, all stated at the <b>zero pose</b>: yaw at
 * zero and every joint as CAD drew it. {@link TurretSketch} does the turning.
 */
public final class TurretSketchDefinition {

    /**
     * How close to vertical the yaw axis must be, and how far from vertical its arm must point. Matches
     * {@link MechanismSketch}'s in-plane gate, for the same reason: it sits above the unit-length slop of the
     * inputs, and is still far tighter than any real modelling error.
     */
    static final double VERTICAL_TOLERANCE = 1.0e-5;

    private final String name;
    private final RobotVector3 yawPivotInches;
    private final RevoluteJointDefinition yaw;
    private final double canvasWidthInches;
    private final double canvasHeightInches;
    private final String backgroundColorHex;
    private final List<JointDefinition> joints;

    private TurretSketchDefinition(Builder builder) {
        this.name = builder.name;
        this.yawPivotInches = builder.yawPivotInches;
        this.yaw = builder.yaw;
        this.canvasWidthInches = builder.canvasWidthInches;
        this.canvasHeightInches = builder.canvasHeightInches;
        this.backgroundColorHex = builder.backgroundColorHex;
        this.joints = Collections.unmodifiableList(new ArrayList<JointDefinition>(builder.joints));
    }

    /** {@return the mechanism name; the root of its AdvantageScope log keys} */
    public String getName() {
        return name;
    }

    /**
     * A point on the turret's yaw axis, robot frame, inches. It is where the yaw joint's arm starts. Any point
     * on the axis describes the same turret; the conventional one is where the axis meets the turret bearing.
     *
     * @return a point on the yaw axis, in inches, in the robot frame
     */
    public RobotVector3 getYawPivotInches() {
        return yawPivotInches;
    }

    /**
     * The yaw joint. Its {@code rotationAxis} is vertical ({@code +Z} for counter-clockwise-positive seen from
     * above), its {@code zeroPoseDirection} points from {@link #getYawPivotInches()} toward the point the rest
     * of the chain mounts on (straight up is fine, for a hood directly above the axis), and its angle limits
     * are the turret's travel. Which way the turret faces is {@link #getFacingHeadingRadians()}.
     *
     * @return the yaw joint
     */
    public RevoluteJointDefinition getYaw() {
        return yaw;
    }

    /** {@return the ride-along canvas width, in inches; it must hold the chain above the turret} */
    public double getCanvasWidthInches() {
        return canvasWidthInches;
    }

    /** {@return the ride-along canvas height, in inches} */
    public double getCanvasHeightInches() {
        return canvasHeightInches;
    }

    /** {@return the ride-along canvas background, {@code "#RRGGBB"}} */
    public String getBackgroundColorHex() {
        return backgroundColorHex;
    }

    /**
     * The joints riding on the turret, base first, stated in the robot frame at yaw zero. Joint 0 mounts on the
     * yaw joint's tip. Empty means a bare turret: {@link TurretSketch} then draws the ghost and the aim marker
     * only.
     *
     * @return an unmodifiable list of the joints above the yaw, base first; possibly empty
     */
    public List<JointDefinition> getJoints() {
        return joints;
    }

    /**
     * Which way the turret faces at yaw zero, robot frame, radians CCW from robot forward.
     *
     * <p>The chain riding on a turret swings in one vertical plane, and that plane is the turret's facing, so
     * the facing is the horizontal direction of the <b>first joint above the yaw that is not vertical</b>. A bare
     * turret (or one whose joints are all vertical) falls back to the yaw arm's horizontal direction. The yaw
     * arm is not used when a joint is available, because an arm that leans sideways toward an off-axis mount
     * says where the chain is, not which way it faces.
     *
     * @return the facing heading in radians, robot frame
     */
    public double getFacingHeadingRadians() {
        return facingHeadingRadians(yaw, joints);
    }

    /** The facing rule of {@link #getFacingHeadingRadians()}; {@code NaN} when nothing points sideways. */
    static double facingHeadingRadians(RevoluteJointDefinition yaw, List<JointDefinition> joints) {
        for (JointDefinition joint : joints) {
            RobotVector3 direction = joint.getZeroPoseDirection();
            if (StrictMath.sqrt(direction.x * direction.x + direction.y * direction.y) > VERTICAL_TOLERANCE) {
                return StrictMath.atan2(direction.y, direction.x);
            }
        }
        RobotVector3 arm = yaw.getZeroPoseDirection();
        if (StrictMath.sqrt(arm.x * arm.x + arm.y * arm.y) > VERTICAL_TOLERANCE) {
            return StrictMath.atan2(arm.y, arm.x);
        }
        return Double.NaN;
    }

    /**
     * Start building. See {@link Builder} for what is required.
     *
     * @param name the mechanism name, which becomes the root of its AdvantageScope log keys
     * @return a builder with nothing set but the name
     */
    public static Builder builder(String name) {
        return new Builder(name);
    }

    /**
     * Step-by-step builder. Everything is required except the background colour and the joints above the yaw;
     * {@link #build()} fails loudly rather than defaulting anything that would change what is drawn.
     */
    public static final class Builder {

        private final String name;
        private final List<JointDefinition> joints = new ArrayList<JointDefinition>();
        private RobotVector3 yawPivotInches;
        private RevoluteJointDefinition yaw;
        private double canvasWidthInches = Double.NaN;
        private double canvasHeightInches = Double.NaN;
        private String backgroundColorHex = "#000000";

        private Builder(String name) {
            if (name == null || name.trim().isEmpty()) {
                throw new IllegalArgumentException("turret mechanism name must be a non-blank string");
            }
            if (name.contains("/")) {
                throw new IllegalArgumentException(
                        "turret mechanism name must not contain '/', got \"" + name + "\"");
            }
            this.name = name;
        }

        /**
         * The yaw: a point on its axis and the joint itself.
         *
         * @param pivotInches a point on the yaw axis, robot frame, inches
         * @param yawJoint    the yaw joint; its rotation axis must be vertical
         * @return this builder
         */
        public Builder yaw(RobotVector3 pivotInches, RevoluteJointDefinition yawJoint) {
            this.yawPivotInches = pivotInches;
            this.yaw = yawJoint;
            return this;
        }

        /**
         * The canvas the chain above the turret is drawn on. It must hold that chain at full reach, measured
         * from the turret, and be at least twice its furthest horizontal reach (the {@link MechanismSketch}
         * canvas-origin rule).
         *
         * @param widthInches  canvas width in inches
         * @param heightInches canvas height in inches
         * @return this builder
         */
        public Builder canvasSizeInches(double widthInches, double heightInches) {
            this.canvasWidthInches = widthInches;
            this.canvasHeightInches = heightInches;
            return this;
        }

        /**
         * Canvas background. Defaults to black.
         *
         * @param backgroundColorHex the colour as {@code "#RRGGBB"}
         * @return this builder
         */
        public Builder backgroundColorHex(String backgroundColorHex) {
            this.backgroundColorHex = backgroundColorHex;
            return this;
        }

        /**
         * Append a joint riding on the turret, stated in the robot frame at yaw zero. The first one mounts on
         * the yaw joint's tip, and each later one on the tip of the one before.
         *
         * @param joint the joint; its name must be unique within this mechanism, and differ from the yaw's
         * @return this builder
         */
        public Builder addJoint(JointDefinition joint) {
            if (joint == null) {
                throw new IllegalArgumentException("cannot add a null joint to \"" + name + "\"");
            }
            for (JointDefinition existing : joints) {
                if (existing.getName().equals(joint.getName())) {
                    // Rejected here, as MechanismSketchDefinition does: duplicate names collide as log keys.
                    throw new IllegalArgumentException(
                            "turret mechanism \"" + name + "\" already has a joint named \"" + joint.getName()
                                    + "\"");
                }
            }
            joints.add(joint);
            return this;
        }

        /**
         * Validates the turret-specific rules and produces the definition. The joints above the yaw are
         * validated again, in the turret's own plane, when {@link TurretSketch} builds their drawing.
         *
         * @return the definition, ready to hand to {@link TurretSketch}
         * @throws IllegalArgumentException if the yaw or its pivot is missing, the yaw axis is not vertical,
         *                                  nothing points sideways to say which way the turret faces, the
         *                                  canvas size or colour is invalid, or two joints share a name.
         */
        public TurretSketchDefinition build() {
            if (yaw == null || yawPivotInches == null) {
                throw new IllegalArgumentException(
                        "turret mechanism \"" + name + "\" needs a yaw joint and a point on its axis — call"
                                + " yaw(pivotInches, yawJoint)");
            }
            RobotVector3 axis = yaw.getRotationAxis();
            if (Math.abs(Math.abs(axis.z) - 1.0) > VERTICAL_TOLERANCE) {
                throw new IllegalArgumentException(
                        "turret mechanism \"" + name + "\" yaw joint \"" + yaw.getName() + "\" turns about "
                                + axis + ", which is not vertical. A turret yaws about +Z or -Z; a pivot about"
                                + " a horizontal axis belongs in a MechanismSketchDefinition.");
            }
            if (Double.isNaN(facingHeadingRadians(yaw, joints))) {
                throw new IllegalArgumentException(
                        "turret mechanism \"" + name + "\" has nothing that points sideways: the yaw arm and"
                                + " every joint on it are vertical, so nothing says which way the turret faces"
                                + " at zero. Add the joint the turret carries (its swing plane is the facing),"
                                + " or point the yaw arm toward the barrel.");
            }
            if (!Double.isFinite(canvasWidthInches) || !Double.isFinite(canvasHeightInches)
                    || canvasWidthInches <= 0.0 || canvasHeightInches <= 0.0) {
                throw new IllegalArgumentException(
                        "turret mechanism \"" + name + "\" needs a positive canvas size, got "
                                + canvasWidthInches + " x " + canvasHeightInches
                                + " — call canvasSizeInches(width, height)");
            }
            if (backgroundColorHex == null || !backgroundColorHex.matches("#[0-9a-fA-F]{6}")) {
                throw new IllegalArgumentException(
                        "turret mechanism \"" + name + "\" backgroundColor must be \"#RRGGBB\", got "
                                + backgroundColorHex);
            }
            for (JointDefinition joint : joints) {
                // Checked here rather than in addJoint, because yaw(...) may be called after the joints.
                if (joint.getName().equals(yaw.getName())) {
                    throw new IllegalArgumentException(
                            "turret mechanism \"" + name + "\" has a joint named \"" + joint.getName()
                                    + "\", which is also the yaw joint's name");
                }
            }
            return new TurretSketchDefinition(this);
        }
    }
}
