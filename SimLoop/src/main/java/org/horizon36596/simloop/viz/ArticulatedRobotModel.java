package org.horizon36596.simloop.viz;

import org.psilynx.psikit.core.wpi.math.Pose3d;
import org.psilynx.psikit.core.wpi.math.Quaternion;
import org.psilynx.psikit.core.wpi.math.Rotation3d;
import org.psilynx.psikit.core.wpi.math.Translation3d;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * A robot's moving parts as a tree of joints, turned into the {@code Pose3d[]} AdvantageScope's 3D Field tab
 * draws as <i>Component (Robot)</i> objects. Every component is a rigid body hanging off the robot or off
 * another component through one joint: a <b>revolute</b> joint (an axle), a <b>prismatic</b> joint (a slide)
 * or a <b>fixed</b> one (a part that only rides along, such as a camera on a turret).
 *
 * <h2>The one convention the asset and this class share</h2>
 * Each component has a <b>joint origin</b>: a point on its axle (revolute), on its line of travel (prismatic),
 * or any chosen point (fixed), given in the robot frame with every joint at zero. The component's mesh in the
 * robot asset is authored <b>relative to that point, with the robot frame's axes</b>: take the part where it
 * sits in the CAD at zero, subtract the joint origin from every vertex, and leave the axes alone. With the
 * asset's {@code zeroedPosition}/{@code zeroedRotations} left as identity, AdvantageScope draws the mesh at
 * {@code robot pose ∘ logged pose}, so:
 * <ul>
 *   <li>at zero the logged pose of a component is its joint origin with no rotation, and the part is drawn
 *       exactly where the CAD put it;</li>
 *   <li>away from zero it is the joint origin carried along by every joint above it, rotated (or slid) by its
 *       own joint about (along) its own axis.</li>
 * </ul>
 * The translation of a logged pose is therefore always "where this joint is now", which is what a human
 * reading the log wants, and the asset never has to be regenerated when a joint value changes.
 *
 * <h2>Frames and units</h2>
 * Inputs are the robot frame ({@code +X} forward, {@code +Y} left, {@code +Z} up, origin on the floor under
 * the drivetrain centre; {@link RobotVector3}), inches and radians, as the rest of this package. A revolute
 * value is radians, right-handed about the joint's axis; a prismatic value is inches along its direction.
 * Output poses are metres, converted here and nowhere else.
 *
 * <h2>Determinism</h2>
 * A pure function of the joint values; {@link StrictMath} trigonometry; no clock. No {@code teamcode} import.
 */
public final class ArticulatedRobotModel {

    /** What kind of joint attaches a component to its parent. */
    public enum JointType {
        /** Rotates about an axis through the joint origin. Value: radians, right-handed about the axis. */
        REVOLUTE,
        /** Slides along a direction through the joint origin. Value: inches along the direction. */
        PRISMATIC,
        /** Rides along with its parent and never moves relative to it. Its value is ignored. */
        FIXED
    }

    /** Index meaning "attached straight to the robot body". */
    public static final int ROBOT = -1;

    private static final double METRES_PER_INCH = 0.0254;

    private final List<String> names;
    private final int[] parents;
    private final JointType[] types;
    private final double[][] origins;
    private final double[][] axes;

    private ArticulatedRobotModel(Builder builder) {
        int count = builder.names.size();
        this.names = Collections.unmodifiableList(new ArrayList<>(builder.names));
        this.parents = new int[count];
        this.types = new JointType[count];
        this.origins = new double[count][];
        this.axes = new double[count][];
        for (int i = 0; i < count; i++) {
            parents[i] = builder.parents.get(i);
            types[i] = builder.types.get(i);
            origins[i] = builder.origins.get(i);
            axes[i] = builder.axes.get(i);
        }
    }

    /** {@return a builder with no components} */
    public static Builder builder() {
        return new Builder();
    }

    /** {@return how many components, which is the length of every pose array this model produces} */
    public int size() {
        return names.size();
    }

    /** {@return the component names, in index order} */
    public List<String> getNames() {
        return names;
    }

    /**
     * {@return the index of the named component}
     *
     * @param name the component's name
     * @throws IllegalArgumentException if there is none
     */
    public int indexOf(String name) {
        int index = names.indexOf(name);
        if (index < 0) {
            throw new IllegalArgumentException("no component named '" + name + "'");
        }
        return index;
    }

    /**
     * {@return the component's parent index, or {@link #ROBOT}}
     *
     * @param component the component's index
     */
    public int getParent(int component) {
        return parents[component];
    }

    /**
     * {@return the component's joint type}
     *
     * @param component the component's index
     */
    public JointType getJointType(int component) {
        return types[component];
    }

    /**
     * {@return the component's joint origin with every joint at zero, robot frame, inches}
     *
     * @param component the component's index
     */
    public RobotVector3 getJointOrigin(int component) {
        double[] o = origins[component];
        return new RobotVector3(o[0], o[1], o[2]);
    }

    /**
     * {@return the component's joint axis (unit) with every joint at zero, robot frame; zero for FIXED}
     *
     * @param component the component's index
     */
    public RobotVector3 getJointAxis(int component) {
        double[] a = axes[component];
        return new RobotVector3(a[0], a[1], a[2]);
    }

    /**
     * Every component's pose for these joint values.
     *
     * @param jointValues one value per component, in index order: radians for REVOLUTE, inches for PRISMATIC,
     *                    ignored for FIXED
     * @return robot-frame poses in metres, one per component, in index order
     * @throws IllegalArgumentException if the array is the wrong length or holds a non-finite value
     */
    public Pose3d[] poses(double... jointValues) {
        int count = names.size();
        if (jointValues == null || jointValues.length != count) {
            throw new IllegalArgumentException("expected " + count + " joint values, got "
                    + (jointValues == null ? "null" : jointValues.length));
        }
        // Each component's frame as a rotation quaternion (w, x, y, z) and a translation in inches.
        double[][] rotation = new double[count][];
        double[][] translation = new double[count][];
        Pose3d[] poses = new Pose3d[count];
        for (int i = 0; i < count; i++) {
            double value = jointValues[i];
            if (!Double.isFinite(value)) {
                throw new IllegalArgumentException("joint value for '" + names.get(i) + "' must be finite, got "
                        + value);
            }
            double[] parentRotation;
            double[] parentTranslation;
            double[] parentOrigin;
            int parent = parents[i];
            if (parent == ROBOT) {
                parentRotation = new double[] {1.0, 0.0, 0.0, 0.0};
                parentTranslation = new double[] {0.0, 0.0, 0.0};
                parentOrigin = new double[] {0.0, 0.0, 0.0};
            } else {
                parentRotation = rotation[parent];
                parentTranslation = translation[parent];
                parentOrigin = origins[parent];
            }
            // The joint origin's offset from the parent's, in the parent's frame (axes agree at zero).
            double[] offset = {origins[i][0] - parentOrigin[0], origins[i][1] - parentOrigin[1],
                    origins[i][2] - parentOrigin[2]};
            double[] localRotation = {1.0, 0.0, 0.0, 0.0};
            if (types[i] == JointType.REVOLUTE) {
                double half = value / 2.0;
                double s = StrictMath.sin(half);
                localRotation = new double[] {StrictMath.cos(half), axes[i][0] * s, axes[i][1] * s, axes[i][2] * s};
            } else if (types[i] == JointType.PRISMATIC) {
                offset[0] += axes[i][0] * value;
                offset[1] += axes[i][1] * value;
                offset[2] += axes[i][2] * value;
            }
            double[] moved = rotate(parentRotation, offset);
            translation[i] = new double[] {parentTranslation[0] + moved[0], parentTranslation[1] + moved[1],
                    parentTranslation[2] + moved[2]};
            rotation[i] = multiply(parentRotation, localRotation);
            double[] q = rotation[i];
            poses[i] = new Pose3d(
                    new Translation3d(translation[i][0] * METRES_PER_INCH, translation[i][1] * METRES_PER_INCH,
                            translation[i][2] * METRES_PER_INCH),
                    new Rotation3d(new Quaternion(q[0], q[1], q[2], q[3])));
        }
        return poses;
    }

    // Hamilton product a * b, quaternions as (w, x, y, z).
    private static double[] multiply(double[] a, double[] b) {
        return new double[] {
                a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
                a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
                a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
                a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0]};
    }

    // q * v * q^-1 for a unit quaternion q.
    private static double[] rotate(double[] q, double[] v) {
        double[] p = multiply(multiply(q, new double[] {0.0, v[0], v[1], v[2]}),
                new double[] {q[0], -q[1], -q[2], -q[3]});
        return new double[] {p[1], p[2], p[3]};
    }

    /** Builds an {@link ArticulatedRobotModel} one component at a time, parents before children. */
    public static final class Builder {
        private final List<String> names = new ArrayList<>();
        private final List<Integer> parents = new ArrayList<>();
        private final List<JointType> types = new ArrayList<>();
        private final List<double[]> origins = new ArrayList<>();
        private final List<double[]> axes = new ArrayList<>();

        private Builder() {
        }

        /**
         * Adds a component that turns about {@code axis} through {@code origin}.
         *
         * @param name   unique component name
         * @param parent {@link #ROBOT} or the index of a component already added
         * @param origin a point on the axle, robot frame, inches, every joint at zero
         * @param axis   the axle direction, robot frame, unit length; a positive value turns right-handed
         *               about it
         * @return this builder
         */
        public Builder revolute(String name, int parent, RobotVector3 origin, RobotVector3 axis) {
            return add(name, parent, JointType.REVOLUTE, origin, requireUnit(name, axis));
        }

        /**
         * Adds a component that slides along {@code direction} from {@code origin}.
         *
         * @param name      unique component name
         * @param parent    {@link #ROBOT} or the index of a component already added
         * @param origin    the component's joint origin at zero travel, robot frame, inches
         * @param direction the travel direction, robot frame, unit length
         * @return this builder
         */
        public Builder prismatic(String name, int parent, RobotVector3 origin, RobotVector3 direction) {
            return add(name, parent, JointType.PRISMATIC, origin, requireUnit(name, direction));
        }

        /**
         * Adds a component that rides on its parent without moving relative to it.
         *
         * @param name   unique component name
         * @param parent {@link #ROBOT} or the index of a component already added
         * @param origin the component's origin, robot frame, inches, every joint at zero
         * @return this builder
         */
        public Builder fixed(String name, int parent, RobotVector3 origin) {
            return add(name, parent, JointType.FIXED, origin, RobotVector3.origin());
        }

        /** {@return the model} @throws IllegalStateException if no component was added */
        public ArticulatedRobotModel build() {
            if (names.isEmpty()) {
                throw new IllegalStateException("an articulated model needs at least one component");
            }
            return new ArticulatedRobotModel(this);
        }

        private Builder add(String name, int parent, JointType type, RobotVector3 origin, RobotVector3 axis) {
            if (name == null || name.isEmpty()) {
                throw new IllegalArgumentException("a component needs a name");
            }
            if (names.contains(name)) {
                throw new IllegalArgumentException("duplicate component name '" + name + "'");
            }
            if (parent != ROBOT && (parent < 0 || parent >= names.size())) {
                throw new IllegalArgumentException("component '" + name + "' has parent " + parent
                        + ", which is neither ROBOT nor a component added before it");
            }
            if (origin == null) {
                throw new IllegalArgumentException("component '" + name + "' needs a joint origin");
            }
            names.add(name);
            parents.add(parent);
            types.add(type);
            origins.add(new double[] {origin.x, origin.y, origin.z});
            axes.add(new double[] {axis.x, axis.y, axis.z});
            return this;
        }

        private static RobotVector3 requireUnit(String name, RobotVector3 axis) {
            if (axis == null || !axis.isUnitLength()) {
                throw new IllegalArgumentException("component '" + name + "' needs a unit-length axis, got " + axis);
            }
            return axis;
        }
    }
}
