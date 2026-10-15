package org.horizon36596.simloop.field;

/**
 * What makes a {@link KinematicBody} a <b>dynamic</b> hinged body: one degree of freedom, its angle about the
 * body's fixed axis, moved by gravity on its own mass and by every contact the game pieces make with it. A
 * see-saw goal that tips when enough weight sits on one side is the case this is for; nothing about it is
 * season-specific. Give a body these numbers with {@link KinematicBody#withHinge(HingeDynamics)}.
 *
 * <p><b>The model</b>, per physics sub-step ({@link GamePiecePhysics}), in this order:
 * <ol>
 *   <li>Gravity on the body's own mass at its centre of mass, plus the weight of every piece asleep on it - in
 *       one of its cells, on any of its shapes, or on pieces that rest on it (a sleeping piece is part of the body
 *       until it wakes) - gives a torque about the axis; the
 *       angular velocity gains {@code torque / momentOfInertia * dt}.</li>
 *   <li>Angular damping: {@code omega *= 1 / (1 + angularDampingPerSecond * dt)}, the form a rigid-body
 *       engine's angular damping takes.</li>
 *   <li>The angle advances by {@code omega * dt}, and a hard limit stops it at {@link #getMinAngleRad()} or
 *       {@link #getMaxAngleRad()}: the angle is held at the limit and the velocity into it is reversed and scaled
 *       by {@link #getLimitRestitution()} (0, the usual joint limit, simply stops it).</li>
 *   <li>Every contact impulse between a piece and the body's shapes is a two-body impulse: the piece's mass
 *       against the body's inertia about the axis at the contact's lever arm, and the body takes the equal and
 *       opposite angular impulse {@code (r x -J) . axis / I}. That is how the pieces' weight and impacts move
 *       it: nothing tells it to tip.</li>
 * </ol>
 *
 * <p><b>Units.</b> Inches, seconds and radians, like the rest of the {@code field} package. Mass is in whatever
 * unit the season gives its pieces ({@link GamePiecePhysicsConfig.Builder#pieceMass}); the inertia is that unit
 * times inches squared. Only ratios matter, so pounds, kilograms or "one piece" all work, so long as the body and
 * the pieces use the same one.
 *
 * <p>Immutable.
 */
public final class HingeDynamics {

    private final double mass;
    private final double centreOfMassX;
    private final double centreOfMassY;
    private final double centreOfMassZ;
    private final double momentOfInertia;
    private final double angularDampingPerSecond;
    private final double minAngleRad;
    private final double maxAngleRad;
    private final double limitRestitution;

    /**
     * Creates a hinge from its mass, inertia, damping and angle limits.
     *
     * @param mass                    the body's own mass, in the pieces' mass unit; 0 or more
     * @param centreOfMassLocal       where that mass acts, body-local frame, inches: {x, y, z}
     * @param momentOfInertia         the body's moment of inertia about its axis, mass unit times square inches;
     *                                more than 0
     * @param angularDampingPerSecond angular damping, per second; 0 or more
     * @param minAngleRad             the lower hard limit, radians
     * @param maxAngleRad             the upper hard limit, radians; more than {@code minAngleRad}
     * @param limitRestitution        how much of its velocity into a limit the body keeps, reversed, 0 to 1
     * @throws IllegalArgumentException on a non-finite or out-of-range number
     */
    public HingeDynamics(double mass, double[] centreOfMassLocal, double momentOfInertia,
            double angularDampingPerSecond, double minAngleRad, double maxAngleRad, double limitRestitution) {
        if (!Double.isFinite(mass) || mass < 0.0) {
            throw new IllegalArgumentException("mass must be finite and >= 0, got " + mass);
        }
        if (centreOfMassLocal == null || centreOfMassLocal.length != 3) {
            throw new IllegalArgumentException("the centre of mass must be three numbers");
        }
        for (double c : centreOfMassLocal) {
            if (!Double.isFinite(c)) {
                throw new IllegalArgumentException("the centre of mass must be finite");
            }
        }
        if (!Double.isFinite(momentOfInertia) || momentOfInertia <= 0.0) {
            throw new IllegalArgumentException("momentOfInertia must be finite and > 0, got " + momentOfInertia);
        }
        if (!Double.isFinite(angularDampingPerSecond) || angularDampingPerSecond < 0.0) {
            throw new IllegalArgumentException("angularDampingPerSecond must be finite and >= 0, got "
                    + angularDampingPerSecond);
        }
        if (!Double.isFinite(minAngleRad) || !Double.isFinite(maxAngleRad) || !(maxAngleRad > minAngleRad)) {
            throw new IllegalArgumentException("the limits must be finite with max > min, got " + minAngleRad
                    + " to " + maxAngleRad);
        }
        if (!Double.isFinite(limitRestitution) || limitRestitution < 0.0 || limitRestitution > 1.0) {
            throw new IllegalArgumentException("limitRestitution must be in [0, 1], got " + limitRestitution);
        }
        this.mass = mass;
        this.centreOfMassX = centreOfMassLocal[0];
        this.centreOfMassY = centreOfMassLocal[1];
        this.centreOfMassZ = centreOfMassLocal[2];
        this.momentOfInertia = momentOfInertia;
        this.angularDampingPerSecond = angularDampingPerSecond;
        this.minAngleRad = minAngleRad;
        this.maxAngleRad = maxAngleRad;
        this.limitRestitution = limitRestitution;
    }

    /** {@return the body's own mass, in the pieces' mass unit} */
    public double getMass() {
        return mass;
    }

    /** {@return the body's centre of mass, body-local frame, inches, {x, y, z}, a copy} */
    public double[] getCentreOfMassLocal() {
        return new double[] {centreOfMassX, centreOfMassY, centreOfMassZ};
    }

    /** {@return the body's moment of inertia about its axis, mass unit times square inches} */
    public double getMomentOfInertia() {
        return momentOfInertia;
    }

    /** {@return the angular damping, per second} */
    public double getAngularDampingPerSecond() {
        return angularDampingPerSecond;
    }

    /** {@return the lower hard limit, radians} */
    public double getMinAngleRad() {
        return minAngleRad;
    }

    /** {@return the upper hard limit, radians} */
    public double getMaxAngleRad() {
        return maxAngleRad;
    }

    /** {@return the fraction of its velocity into a limit the body keeps, reversed, 0 to 1} */
    public double getLimitRestitution() {
        return limitRestitution;
    }
}
