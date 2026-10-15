package org.horizon36596.simloop.field;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

/**
 * A rigid structure that game pieces collide with and can rest in, and that may turn about one fixed axis: a
 * goal that tips, a flap, or a fixed frame that never moves. It is <b>kinematic</b>: its angle is whatever the
 * season says it is ({@link GamePieceTracker#setBodyMotion}), and pieces never push it. The pieces feel its
 * motion - a turning surface carries a resting ball along and can tip it out.
 *
 * <p><b>Frames and units.</b> The body is made of {@link ConvexShape}s and holds {@link BodyCell}s, all in its
 * own <b>local frame</b>, inches. At angle {@code theta} (radians, right-handed about the axis) a local point
 * {@code p} is at field position {@code pivot + R(axis, theta) p}, where {@code pivot} is a field-frame point,
 * inches, and {@code axis} a field-frame unit vector. A body that never turns can simply be built with its
 * shapes in field coordinates and its pivot at the origin.
 *
 * <p><b>Contact numbers.</b> Each body carries the restitution and friction coefficient a piece meets on it,
 * because a padded tray and a bare steel frame differ. One shape can differ from the rest of its body
 * ({@link #withShapeMaterial}): a padded back wall that a ball dies against, a floor with more grip than the
 * walls. They are the numbers for the <i>pair</i>, piece against this surface: a season that copies an engine
 * which combines a piece's own coefficient with the surface's (an average, the smaller, the product) does that
 * combining itself and hands over the result. A piece's own spin is part of the contact (see
 * {@link GamePiecePhysicsConfig#getPieceInertiaFactor()}).
 *
 * <p><b>Or dynamic.</b> A body given {@link HingeDynamics} ({@link #withHinge}) is no longer kinematic: it is a
 * one-degree-of-freedom hinged body whose angle the physics integrates from gravity on its own mass, the pieces'
 * contacts and its hard limits. The season then sets only where it starts ({@link GamePieceTracker#setBodyMotion})
 * and reads where it is. The class keeps its name because every other part of it - shapes, cells, frames - is
 * the same.
 *
 * <p>The geometry is immutable; the angle lives in the {@link GamePieceTracker}, which is what keeps a
 * {@link FieldWorld} immutable. Season-agnostic (domain R7).
 */
public final class KinematicBody {

    private final String name;
    private final double pivotX;
    private final double pivotY;
    private final double pivotZ;
    private final double axisX;
    private final double axisY;
    private final double axisZ;
    private final List<ConvexShape> shapes;
    private final List<BodyCell> cells;
    private final double restitution;
    private final double friction;
    // Per shape, indexed like shapes: the pair's restitution and friction on that shape. The body's own numbers
    // unless withShapeMaterial gave the shape its own.
    private final double[] shapeRestitution;
    private final double[] shapeFriction;
    private final double boundingRadius;
    private final HingeDynamics hinge;

    /**
     * Creates a body from its turning axis, its solid parts, its cells and its contact material.
     *
     * @param name        a label for logs and errors, unique in the world
     * @param pivot       a point on the turning axis, field frame, inches: {x, y, z}
     * @param axis        the turning axis, field frame; any non-zero length (it is normalised)
     * @param shapes      the solid parts, body-local inches; at least one
     * @param cells       the compartments pieces are counted in, body-local; may be empty
     * @param restitution the coefficient of restitution a piece meets on this body, 0 to 1
     * @param friction    the Coulomb friction coefficient between a piece and this body, 0 or more
     * @throws IllegalArgumentException on a blank name, a bad vector, no shapes, or an out-of-range coefficient
     */
    public KinematicBody(String name, double[] pivot, double[] axis, List<ConvexShape> shapes, List<BodyCell> cells,
            double restitution, double friction) {
        if (name == null || name.trim().isEmpty()) {
            throw new IllegalArgumentException("a kinematic body needs a name");
        }
        if (pivot == null || pivot.length != 3 || axis == null || axis.length != 3) {
            throw new IllegalArgumentException(name + ": pivot and axis must each be three numbers");
        }
        for (int i = 0; i < 3; i++) {
            if (!Double.isFinite(pivot[i]) || !Double.isFinite(axis[i])) {
                throw new IllegalArgumentException(name + ": pivot and axis must be finite");
            }
        }
        double length = StrictMath.sqrt(axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2]);
        if (!(length > 0.0)) {
            throw new IllegalArgumentException(name + ": the axis must not be zero");
        }
        if (shapes == null || shapes.isEmpty()) {
            throw new IllegalArgumentException(name + ": a body needs at least one shape");
        }
        if (!Double.isFinite(restitution) || restitution < 0.0 || restitution > 1.0) {
            throw new IllegalArgumentException(name + ": restitution must be in [0, 1], got " + restitution);
        }
        if (!Double.isFinite(friction) || friction < 0.0) {
            throw new IllegalArgumentException(name + ": friction must be finite and >= 0, got " + friction);
        }
        this.name = name;
        this.pivotX = pivot[0];
        this.pivotY = pivot[1];
        this.pivotZ = pivot[2];
        this.axisX = axis[0] / length;
        this.axisY = axis[1] / length;
        this.axisZ = axis[2] / length;
        this.shapes = Collections.unmodifiableList(new ArrayList<>(shapes));
        List<BodyCell> cellCopy = new ArrayList<>(cells == null ? Collections.<BodyCell>emptyList() : cells);
        for (int i = 0; i < cellCopy.size(); i++) {
            for (int j = i + 1; j < cellCopy.size(); j++) {
                if (cellCopy.get(i).getName().equals(cellCopy.get(j).getName())) {
                    throw new IllegalArgumentException(name + ": two cells named " + cellCopy.get(i).getName());
                }
            }
        }
        this.cells = Collections.unmodifiableList(cellCopy);
        this.restitution = restitution;
        this.friction = friction;
        this.shapeRestitution = new double[this.shapes.size()];
        this.shapeFriction = new double[this.shapes.size()];
        Arrays.fill(this.shapeRestitution, restitution);
        Arrays.fill(this.shapeFriction, friction);
        double radius = 0.0;
        for (ConvexShape shape : this.shapes) {
            double[] c = shape.getBoundingCentre();
            radius = Math.max(radius,
                    StrictMath.sqrt(c[0] * c[0] + c[1] * c[1] + c[2] * c[2]) + shape.getBoundingRadiusInches());
        }
        this.boundingRadius = radius;
        this.hinge = null;
    }

    private KinematicBody(KinematicBody body, HingeDynamics hinge, double[] shapeRestitution, double[] shapeFriction) {
        this.name = body.name;
        this.pivotX = body.pivotX;
        this.pivotY = body.pivotY;
        this.pivotZ = body.pivotZ;
        this.axisX = body.axisX;
        this.axisY = body.axisY;
        this.axisZ = body.axisZ;
        this.shapes = body.shapes;
        this.cells = body.cells;
        this.restitution = body.restitution;
        this.friction = body.friction;
        this.shapeRestitution = shapeRestitution;
        this.shapeFriction = shapeFriction;
        this.boundingRadius = body.boundingRadius;
        this.hinge = hinge;
    }

    /**
     * This body, the same shapes, cells and contact numbers, made dynamic: its angle is integrated by the
     * physics from {@code hinge} and the pieces' contacts instead of set by the season.
     *
     * @param hinge the body's mass, centre of mass, inertia, damping and limits
     * @return a new body; this one is unchanged
     * @throws IllegalArgumentException if {@code hinge} is null
     */
    public KinematicBody withHinge(HingeDynamics hinge) {
        if (hinge == null) {
            throw new IllegalArgumentException(name + ": the hinge dynamics must not be null");
        }
        return new KinematicBody(this, hinge, shapeRestitution, shapeFriction);
    }

    /**
     * This body with every shape named {@code shapeName} given its own contact numbers, in place of the body's.
     * Everything else - the other shapes, the cells, the hinge - is unchanged.
     *
     * @param shapeName   the {@link ConvexShape#getName() name} of the shape (every shape of that name, if several)
     * @param restitution the coefficient of restitution a piece meets on that shape, 0 to 1
     * @param friction    the Coulomb friction coefficient between a piece and that shape, 0 or more
     * @return a new body; this one is unchanged
     * @throws IllegalArgumentException if no shape has that name, or a coefficient is out of range
     */
    public KinematicBody withShapeMaterial(String shapeName, double restitution, double friction) {
        if (!Double.isFinite(restitution) || restitution < 0.0 || restitution > 1.0) {
            throw new IllegalArgumentException(name + ": restitution must be in [0, 1], got " + restitution);
        }
        if (!Double.isFinite(friction) || friction < 0.0) {
            throw new IllegalArgumentException(name + ": friction must be finite and >= 0, got " + friction);
        }
        double[] restitutions = shapeRestitution.clone();
        double[] frictions = shapeFriction.clone();
        boolean found = false;
        for (int i = 0; i < shapes.size(); i++) {
            if (shapes.get(i).getName().equals(shapeName)) {
                restitutions[i] = restitution;
                frictions[i] = friction;
                found = true;
            }
        }
        if (!found) {
            throw new IllegalArgumentException(name + ": no shape named " + shapeName);
        }
        return new KinematicBody(this, hinge, restitutions, frictions);
    }

    /** {@return the body's hinge dynamics, or null for a kinematic body the season moves} */
    public HingeDynamics getHingeDynamics() {
        return hinge;
    }

    /** {@return true when the physics moves this body ({@link #withHinge}), false when the season does} */
    public boolean isDynamic() {
        return hinge != null;
    }

    /** {@return the body's name} */
    public String getName() {
        return name;
    }

    /** {@return the pivot, field frame, inches, {x, y, z}, a copy} */
    public double[] getPivot() {
        return new double[] {pivotX, pivotY, pivotZ};
    }

    /** {@return the unit turning axis, field frame, {x, y, z}, a copy} */
    public double[] getAxis() {
        return new double[] {axisX, axisY, axisZ};
    }

    /** {@return the solid parts, body-local, in declaration order} */
    public List<ConvexShape> getShapes() {
        return shapes;
    }

    /** {@return the cells, body-local, in declaration order} */
    public List<BodyCell> getCells() {
        return cells;
    }

    /**
     * {@return the coefficient of restitution a piece meets on this body, 0 to 1: on every shape not given its
     * own by {@link #withShapeMaterial}}
     */
    public double getRestitution() {
        return restitution;
    }

    /**
     * {@return the Coulomb friction coefficient between a piece and this body: on every shape not given its own
     * by {@link #withShapeMaterial}}
     */
    public double getFriction() {
        return friction;
    }

    /**
     * {@return the coefficient of restitution a piece meets on one shape, 0 to 1}
     *
     * @param shapeIndex the shape's index in {@link #getShapes()}
     */
    public double getShapeRestitution(int shapeIndex) {
        return shapeRestitution[shapeIndex];
    }

    /**
     * {@return the Coulomb friction coefficient between a piece and one shape}
     *
     * @param shapeIndex the shape's index in {@link #getShapes()}
     */
    public double getShapeFriction(int shapeIndex) {
        return shapeFriction[shapeIndex];
    }

    /**
     * {@return the radius, inches, of a sphere about the pivot that holds every shape at any angle} A piece
     * whose centre is further than this plus its radius from the pivot cannot touch the body.
     */
    public double getBoundingRadiusInches() {
        return boundingRadius;
    }

    /**
     * Turn a field-frame point into this body's local frame at an angle.
     *
     * @param angleRad the body's angle, radians
     * @param x        the point, field frame, inches
     * @param y        ditto
     * @param z        ditto
     * @param out      receives the local point, inches
     */
    public void toLocal(double angleRad, double x, double y, double z, double[] out) {
        rotate(-angleRad, x - pivotX, y - pivotY, z - pivotZ, out);
    }

    /**
     * Turn a local point into the field frame at an angle.
     *
     * @param angleRad the body's angle, radians
     * @param x        the point, body-local, inches
     * @param y        ditto
     * @param z        ditto
     * @param out      receives the field-frame point, inches
     */
    public void toField(double angleRad, double x, double y, double z, double[] out) {
        rotate(angleRad, x, y, z, out);
        out[0] += pivotX;
        out[1] += pivotY;
        out[2] += pivotZ;
    }

    /**
     * Rotate a vector about this body's axis (Rodrigues' formula): from local to field for {@code +angle}, from
     * field to local for {@code -angle}.
     *
     * @param angleRad the rotation, radians, right-handed about the axis
     * @param x        the vector
     * @param y        ditto
     * @param z        ditto
     * @param out      receives the rotated vector
     */
    public void rotate(double angleRad, double x, double y, double z, double[] out) {
        double c = StrictMath.cos(angleRad);
        double s = StrictMath.sin(angleRad);
        rotate(c, s, x, y, z, out);
    }

    /** {@link #rotate(double, double, double, double, double[])} with the cosine and sine already worked out. */
    void rotate(double c, double s, double x, double y, double z, double[] out) {
        double dot = axisX * x + axisY * y + axisZ * z;
        double crossX = axisY * z - axisZ * y;
        double crossY = axisZ * x - axisX * z;
        double crossZ = axisX * y - axisY * x;
        out[0] = x * c + crossX * s + axisX * dot * (1.0 - c);
        out[1] = y * c + crossY * s + axisY * dot * (1.0 - c);
        out[2] = z * c + crossZ * s + axisZ * dot * (1.0 - c);
    }

    /**
     * How fast the body's surface moves at a field point while it turns: {@code omega * axis x (p - pivot)}.
     *
     * @param angularVelocityRadPerS the body's turning rate, radians per second
     * @param x                      the point, field frame, inches
     * @param y                      ditto
     * @param z                      ditto
     * @param out                    receives the velocity, field frame, inches per second
     */
    public void surfaceVelocity(double angularVelocityRadPerS, double x, double y, double z, double[] out) {
        double dx = x - pivotX;
        double dy = y - pivotY;
        double dz = z - pivotZ;
        out[0] = angularVelocityRadPerS * (axisY * dz - axisZ * dy);
        out[1] = angularVelocityRadPerS * (axisZ * dx - axisX * dz);
        out[2] = angularVelocityRadPerS * (axisX * dy - axisY * dx);
    }

    /** {@return the squared distance, inches squared, from a field point to the pivot} */
    double distanceSquaredToPivot(double x, double y, double z) {
        double dx = x - pivotX;
        double dy = y - pivotY;
        double dz = z - pivotZ;
        return dx * dx + dy * dy + dz * dz;
    }
}
