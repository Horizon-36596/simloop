package org.horizon36596.simloop.field;

import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.Map;

/**
 * The numbers that govern how loose and in-flight game pieces move: gravity, how bouncy each kind of
 * contact is, how quickly a rolling piece slows down, and how finely a sim tick is sub-stepped.
 *
 * <p><b>Every number is a physical parameter the season can calibrate</b>, in the spirit of domain R6's
 * "calibration-ready": the defaults below are plausible for a foam ball a few inches across, carried over
 * rather than measured, and are labelled with where they came from, but a
 * season that measures its own balls bouncing off its own field wall replaces them through
 * {@link FieldWorld.Builder#physics}. A season that wants its provenance on record builds the config
 * explicitly with {@link #builder()} and comments each number there. Units are inches and seconds
 * throughout, matching the rest of the {@code field} package.
 *
 * <p><b>What is deliberately not here.</b> No Magnus lift, and no spin except where a piece touches a
 * {@link KinematicBody} or another piece near one (the 3-D contact numbers below). <b>Mass</b> matters only in the
 * 3-D model: a mass per piece kind ({@link Builder#pieceMass}, 1 for a kind not named) shares a collision between
 * two pieces, and sets how hard a piece pushes a {@link HingeDynamics dynamic body}. Loose pieces on the floor
 * still collide as equal masses. The robot is treated as immovable, so a ball never pushes the robot back. Air drag is available but defaults
 * to <em>off</em> so that a launched piece follows exactly the drag-free arc the robot's own shot solver
 * assumes — turning it on is a deliberate act of making the sim harder than the solver's model.
 *
 * <p>Immutable. Build one with {@link #defaults()} or {@link Builder}.
 */
public final class GamePiecePhysicsConfig {

    /** Standard gravity in inches per second squared (9.80665 m/s² ÷ 0.0254). SPEC. */
    public static final double STANDARD_GRAVITY_INCHES_PER_S2 = 9.80665 / 0.0254;

    /**
     * The sim tick, in seconds, that the old {@link Builder#substepsPerTick(int)} setting was written
     * against: 20 ms, the tick every scenario in the repo runs at. Used only to turn that old setting into
     * a longest sub-step.
     */
    public static final double NOMINAL_TICK_SECONDS = 0.020;

    // How close a tick has to be to a whole number of sub-steps to count as exactly that many. Floating
    // point makes 0.02 / (0.02 / 27) come out as 27.000000000000004, which must still mean 27 sub-steps.
    private static final double WHOLE_SUBSTEP_TOLERANCE = 1e-9;

    /**
     * The shortest {@link Builder#maxSubstepSeconds(double) maxSubstepSeconds} allowed, seconds: one
     * microsecond. Far below anything useful, and it keeps the sub-step count of any sane tick far from
     * the largest {@code int}.
     */
    public static final double MIN_MAX_SUBSTEP_SECONDS = 1e-6;

    private final double gravityInchesPerS2;
    private final double pieceToPieceRestitution;
    private final double pieceToWallRestitution;
    private final double pieceToObstacleRestitution;
    private final double pieceToRobotRestitution;
    private final double pieceToFloorRestitution;
    private final double pieceToFloorTangentialRetention;
    // The 3-D model's own contact numbers; NaN means "the same as the plain one" (see each getter).
    private final double pieceToPieceRestitution3d;
    private final double pieceToWallRestitution3d;
    private final double pieceToObstacleRestitution3d;
    private final double pieceToFloorFriction3d;
    private final double rollingDecelerationInchesPerS2;
    private final double restSpeedInchesPerS;
    private final double airDragPerInch;
    private final double maxSubstepSeconds;
    private final double pieceToPieceFriction;
    private final double pieceInertiaFactor;
    private final double spinDampingPerSecond;
    private final double sleepSpeedInchesPerS;
    private final double sleepSeconds;
    private final double wakeSpeedInchesPerS;
    private final int contactIterations;
    private final double maxContactTravelInches;
    private final double defaultPieceMass;
    private final Map<String, Double> pieceMassByKind;

    private GamePiecePhysicsConfig(Builder builder) {
        this.gravityInchesPerS2 = builder.gravityInchesPerS2;
        this.pieceToPieceRestitution = builder.pieceToPieceRestitution;
        this.pieceToWallRestitution = builder.pieceToWallRestitution;
        this.pieceToObstacleRestitution = builder.pieceToObstacleRestitution;
        this.pieceToRobotRestitution = builder.pieceToRobotRestitution;
        this.pieceToFloorRestitution = builder.pieceToFloorRestitution;
        this.pieceToFloorTangentialRetention = builder.pieceToFloorTangentialRetention;
        this.pieceToPieceRestitution3d = builder.pieceToPieceRestitution3d;
        this.pieceToWallRestitution3d = builder.pieceToWallRestitution3d;
        this.pieceToObstacleRestitution3d = builder.pieceToObstacleRestitution3d;
        this.pieceToFloorFriction3d = builder.pieceToFloorFriction3d;
        this.rollingDecelerationInchesPerS2 = builder.rollingDecelerationInchesPerS2;
        this.restSpeedInchesPerS = builder.restSpeedInchesPerS;
        this.airDragPerInch = builder.airDragPerInch;
        this.maxSubstepSeconds = builder.maxSubstepSeconds;
        this.pieceToPieceFriction = builder.pieceToPieceFriction;
        this.pieceInertiaFactor = builder.pieceInertiaFactor;
        this.spinDampingPerSecond = builder.spinDampingPerSecond;
        this.sleepSpeedInchesPerS = builder.sleepSpeedInchesPerS;
        this.sleepSeconds = builder.sleepSeconds;
        this.wakeSpeedInchesPerS = builder.wakeSpeedInchesPerS;
        this.contactIterations = builder.contactIterations;
        this.maxContactTravelInches = builder.maxContactTravelInches;
        this.defaultPieceMass = builder.defaultPieceMass;
        this.pieceMassByKind = Collections.unmodifiableMap(new LinkedHashMap<>(builder.pieceMassByKind));
    }

    /**
     * {@return the mass of a piece of this kind, in the season's mass unit} A kind the season did not name has
     * {@link Builder#defaultPieceMass the default mass}, 1 unless set.
     *
     * @param kind the piece's {@link GamePiece#getKind() kind}
     */
    public double getPieceMass(String kind) {
        Double mass = pieceMassByKind.get(kind);
        return mass == null ? defaultPieceMass : mass;
    }

    /** {@return the mass of a piece whose kind has no mass of its own, in the season's mass unit} */
    public double getDefaultPieceMass() {
        return defaultPieceMass;
    }

    /** {@return the foam-ball defaults documented on each builder setter} */
    public static GamePiecePhysicsConfig defaults() {
        return new Builder().build();
    }

    /** {@return a builder pre-filled with {@link #defaults()}} */
    public static Builder builder() {
        return new Builder();
    }

    /** {@return downward acceleration of an in-flight piece, inches per second squared} */
    public double getGravityInchesPerS2() {
        return gravityInchesPerS2;
    }

    /** {@return coefficient of restitution when two loose pieces collide, 0 = stick, 1 = perfectly elastic} */
    public double getPieceToPieceRestitution() {
        return pieceToPieceRestitution;
    }

    /** {@return coefficient of restitution against the field perimeter wall} */
    public double getPieceToWallRestitution() {
        return pieceToWallRestitution;
    }

    /** {@return coefficient of restitution against an {@link ObstacleBox}} */
    public double getPieceToObstacleRestitution() {
        return pieceToObstacleRestitution;
    }

    /** {@return coefficient of restitution when the robot's footprint pushes a loose piece} */
    public double getPieceToRobotRestitution() {
        return pieceToRobotRestitution;
    }

    /** {@return coefficient of restitution when an in-flight piece hits the floor} */
    public double getPieceToFloorRestitution() {
        return pieceToFloorRestitution;
    }

    /**
     * {@return the fraction of its horizontal speed an in-flight piece keeps on each floor bounce, 0 to 1;
     * 1 keeps it all} See {@link Builder#pieceToFloorTangentialRetention(double)}.
     */
    public double getPieceToFloorTangentialRetention() {
        return pieceToFloorTangentialRetention;
    }

    /**
     * {@return the coefficient of restitution when two pieces in the 3-D model collide (in flight, or resting in
     * a {@link KinematicBody body}'s cell)} {@link #getPieceToPieceRestitution()} unless set apart with
     * {@link Builder#pieceToPieceRestitution3d}.
     */
    public double getPieceToPieceRestitution3d() {
        return Double.isNaN(pieceToPieceRestitution3d) ? pieceToPieceRestitution : pieceToPieceRestitution3d;
    }

    /**
     * {@return the coefficient of restitution against the perimeter wall for a piece in the 3-D model}
     * {@link #getPieceToWallRestitution()} unless set apart with {@link Builder#pieceToWallRestitution3d}.
     */
    public double getPieceToWallRestitution3d() {
        return Double.isNaN(pieceToWallRestitution3d) ? pieceToWallRestitution : pieceToWallRestitution3d;
    }

    /**
     * {@return the coefficient of restitution against an {@link ObstacleBox} for a piece in the 3-D model}
     * {@link #getPieceToObstacleRestitution()} unless set apart with {@link Builder#pieceToObstacleRestitution3d}.
     */
    public double getPieceToObstacleRestitution3d() {
        return Double.isNaN(pieceToObstacleRestitution3d) ? pieceToObstacleRestitution : pieceToObstacleRestitution3d;
    }

    /**
     * {@return whether a piece in the 3-D model meets the floor with Coulomb friction and spin
     * ({@link #getPieceToFloorFriction3d()}) rather than by {@link #getPieceToFloorTangentialRetention()}}
     */
    public boolean hasPieceToFloorFriction3d() {
        return !Double.isNaN(pieceToFloorFriction3d);
    }

    /**
     * {@return the Coulomb friction coefficient between a piece in the 3-D model and the floor, or NaN when not
     * set} See {@link Builder#pieceToFloorFriction3d(double)}.
     */
    public double getPieceToFloorFriction3d() {
        return pieceToFloorFriction3d;
    }

    /** {@return how fast a rolling piece loses speed, inches per second squared} */
    public double getRollingDecelerationInchesPerS2() {
        return rollingDecelerationInchesPerS2;
    }

    /** {@return the speed below which a rolling or bouncing piece is snapped to rest, inches per second} */
    public double getRestSpeedInchesPerS() {
        return restSpeedInchesPerS;
    }

    /**
     * Quadratic air-drag coefficient {@code k} in {@code 1/inch}: an in-flight piece decelerates by
     * {@code k * speed^2} along its direction of travel. Zero (the default) means no drag.
     *
     * @return the drag coefficient, per inch, zero for none
     */
    public double getAirDragPerInch() {
        return airDragPerInch;
    }

    /**
     * The longest physics sub-step, seconds. {@link GamePieceTracker#advance} splits whatever tick it is
     * handed into equal sub-steps no longer than this, so the physics runs at the same resolution whether
     * the caller ticks every 10 ms or every 40 ms — every stability and tunnelling bound in
     * {@code GamePiecePhysics} is stated per sub-step and so holds at any tick length.
     *
     * <p>Provenance: the default (0.005 s) is SET BY HAND - the 20 ms nominal tick divided by 4, enough for
     * 260 in/s pieces - not a measured value.
     *
     * @return the longest sub-step, seconds (&gt; 0)
     */
    public double getMaxSubstepSeconds() {
        return maxSubstepSeconds;
    }

    /**
     * {@return the Coulomb friction coefficient between two pieces touching in 3-D, among pieces resting in or
     * tumbling off a {@link KinematicBody}} Unused in a world without bodies.
     */
    public double getPieceToPieceFriction() {
        return pieceToPieceFriction;
    }

    /**
     * {@return a piece's moment of inertia as a fraction of {@code m r^2}: 2/3 for a thin hollow shell, 2/5
     * for a solid ball} It decides how much of a sliding contact turns into spin, which is what makes a ball
     * roll rather than slide down a slope. Used only in contacts with a {@link KinematicBody} and between
     * pieces near one.
     */
    public double getPieceInertiaFactor() {
        return pieceInertiaFactor;
    }

    /** {@return how fast a piece's spin decays on its own, per second: {@code w <- w / (1 + k dt)} each sub-step} */
    public double getSpinDampingPerSecond() {
        return spinDampingPerSecond;
    }

    /**
     * {@return the speed, inches per second, under which a piece resting on a {@link KinematicBody} or on
     * other pieces counts as still} Measured as how far its centre moved over a sub-step. A piece still for
     * {@link #getSleepSeconds()} is put to sleep: the physics stops moving it until something disturbs it, which
     * is what lets a pile of pieces sit exactly still.
     */
    public double getSleepSpeedInchesPerS() {
        return sleepSpeedInchesPerS;
    }

    /** {@return how long, seconds, a piece must be still before it is put to sleep} */
    public double getSleepSeconds() {
        return sleepSeconds;
    }

    /**
     * {@return how fast, inches per second, another piece must arrive to wake a sleeping one} Slower than this a
     * sleeping piece is a fixed surface to it.
     */
    public double getWakeSpeedInchesPerS() {
        return wakeSpeedInchesPerS;
    }

    /** {@return how many times each sub-step's 3-D contacts are resolved, at least 1} */
    public int getContactIterations() {
        return contactIterations;
    }

    /**
     * {@return the furthest a piece moves between two tests against a {@link KinematicBody}, inches} A fast
     * piece near a body is moved in steps no longer than this. Keep it below the smallest piece radius: then a
     * piece's centre can never step past a surface it has not been tested against, however thin the solid.
     */
    public double getMaxContactTravelInches() {
        return maxContactTravelInches;
    }

    /**
     * How many equal sub-steps a tick of {@code deltaTimeSeconds} is split into: the fewest that keep each
     * one no longer than {@link #getMaxSubstepSeconds()}, to within 1e-9 of a sub-step, i.e.
     * {@code ceil(deltaTime / maxSubstep - 1e-9)}. The tolerance is there so floating-point noise in a tick
     * that is a whole number of sub-steps does not add one more. At the default 5 ms and a 20 ms tick that
     * is exactly 4.
     *
     * @param deltaTimeSeconds the tick, seconds (&gt; 0)
     * @return the number of sub-steps, at least 1
     * @throws IllegalArgumentException if {@code deltaTimeSeconds} is not a positive finite number
     */
    public int substepsFor(double deltaTimeSeconds) {
        if (!Double.isFinite(deltaTimeSeconds) || deltaTimeSeconds <= 0.0) {
            throw new IllegalArgumentException("deltaTimeSeconds must be a positive finite number, got "
                    + deltaTimeSeconds);
        }
        double exact = deltaTimeSeconds / maxSubstepSeconds;
        return Math.max(1, (int) Math.ceil(exact - WHOLE_SUBSTEP_TOLERANCE));
    }

    /**
     * How many sub-steps a {@link #NOMINAL_TICK_SECONDS 20 ms} tick is split into.
     *
     * @return {@code substepsFor(NOMINAL_TICK_SECONDS)}
     * @deprecated the sub-step count now depends on the tick the caller hands to
     *             {@link GamePieceTracker#advance}; use {@link #substepsFor(double)} with that tick, or
     *             {@link #getMaxSubstepSeconds()}. Kept so existing callers still compile.
     */
    @Deprecated
    public int getSubstepsPerTick() {
        return substepsFor(NOMINAL_TICK_SECONDS);
    }

    /** Builder with one named setter per number, each documented with its default's provenance. */
    public static final class Builder {

        private double gravityInchesPerS2 = STANDARD_GRAVITY_INCHES_PER_S2;
        private double pieceToPieceRestitution = 0.68;
        private double pieceToWallRestitution = 0.35;
        private double pieceToObstacleRestitution = 0.30;
        private double pieceToRobotRestitution = 0.05;
        private double pieceToFloorRestitution = 0.30;
        private double pieceToFloorTangentialRetention = 1.0;
        private double pieceToPieceRestitution3d = Double.NaN;
        private double pieceToWallRestitution3d = Double.NaN;
        private double pieceToObstacleRestitution3d = Double.NaN;
        private double pieceToFloorFriction3d = Double.NaN;
        private double rollingDecelerationInchesPerS2 = 28.0;
        private double restSpeedInchesPerS = 2.0;
        private double airDragPerInch = 0.0;
        // SET BY HAND: the 20 ms nominal tick / 4, which keeps the tunnelling bound (see GamePiecePhysics) safe at
        // 260 in/s. Not measured; shorten it if a faster piece is ever modelled.
        private double maxSubstepSeconds = 0.005;
        private double pieceToPieceFriction = 0.6;
        private double pieceInertiaFactor = 2.0 / 3.0;
        private double spinDampingPerSecond = 0.4;
        private double sleepSpeedInchesPerS = 1.0;
        private double sleepSeconds = 0.25;
        private double wakeSpeedInchesPerS = 10.0;
        private int contactIterations = 3;
        private double maxContactTravelInches = 1.0;
        private double defaultPieceMass = 1.0;
        private final Map<String, Double> pieceMassByKind = new LinkedHashMap<>();

        private Builder() {
        }

        /**
         * The mass of one kind of piece, in whatever unit the season uses (pounds, kilograms, or "one piece"),
         * the same unit as any {@link HingeDynamics#getMass() dynamic body's}. Used in the 3-D model only: it
         * shares a piece-to-piece collision by reduced mass, and sets the push a piece gives a dynamic body.
         * Default: every kind has {@link #defaultPieceMass(double) the default mass}, so equal masses.
         *
         * @param kind the piece kind, as given to the tracker
         * @param mass more than 0
         * @return this builder
         */
        public Builder pieceMass(String kind, double mass) {
            if (kind == null || kind.trim().isEmpty()) {
                throw new IllegalArgumentException("pieceMass needs a kind");
            }
            pieceMassByKind.put(kind, requirePositive(mass, "pieceMass(" + kind + ")"));
            return this;
        }

        /**
         * Default 1: the mass of any kind not given one with {@link #pieceMass}, in the season's unit.
         *
         * @param value the default mass, in the season's mass unit, more than 0
         * @return this builder
         */
        public Builder defaultPieceMass(double value) {
            this.defaultPieceMass = requirePositive(value, "defaultPieceMass");
            return this;
        }

        /**
         * Default: standard gravity, 386.09 in/s². SPEC.
         *
         * @param value downward acceleration, inches per second squared, more than 0
         * @return this builder
         */
        public Builder gravityInchesPerS2(double value) {
            this.gravityInchesPerS2 = requirePositive(value, "gravityInchesPerS2");
            return this;
        }

        /**
         * Default 0.68. CARRIED OVER, not measured on a real ball.
         *
         * @param value coefficient of restitution, 0 to 1
         * @return this builder
         */
        public Builder pieceToPieceRestitution(double value) {
            this.pieceToPieceRestitution = requireRestitution(value, "pieceToPieceRestitution");
            return this;
        }

        /**
         * Default 0.35. CARRIED OVER, not measured.
         *
         * @param value coefficient of restitution, 0 to 1
         * @return this builder
         */
        public Builder pieceToWallRestitution(double value) {
            this.pieceToWallRestitution = requireRestitution(value, "pieceToWallRestitution");
            return this;
        }

        /**
         * Default 0.30. CARRIED OVER, not measured.
         *
         * @param value coefficient of restitution, 0 to 1
         * @return this builder
         */
        public Builder pieceToObstacleRestitution(double value) {
            this.pieceToObstacleRestitution = requireRestitution(value, "pieceToObstacleRestitution");
            return this;
        }

        /**
         * Default 0.05: a pushed ball barely rebounds. CARRIED OVER, not measured.
         *
         * @param value coefficient of restitution, 0 to 1
         * @return this builder
         */
        public Builder pieceToRobotRestitution(double value) {
            this.pieceToRobotRestitution = requireRestitution(value, "pieceToRobotRestitution");
            return this;
        }

        /**
         * Default 0.30: SET BY HAND. A flight that lands dead-stopped looks wrong: a wiffle ball on foam tile does
         * bounce a little, and a small value lets the arc end in a visible skip rather than a freeze.
         *
         * @param value coefficient of restitution, 0 to 1
         * @return this builder
         */
        public Builder pieceToFloorRestitution(double value) {
            this.pieceToFloorRestitution = requireRestitution(value, "pieceToFloorRestitution");
            return this;
        }

        /**
         * The fraction of its horizontal speed an in-flight piece keeps each time it hits the floor and
         * bounces, 0 to 1. Applied to both horizontal components on every floor impact that reverses the
         * vertical speed, including the last one that sets the piece down rolling.
         *
         * <p><b>APPROX physics.</b> A ball that arrives at the floor sliding is brought to rolling without
         * slipping by friction during the contact, which leaves it {@code 1 / (1 + I / (m r²))} of its
         * horizontal speed: 0.6 for a thin spherical shell ({@code I = 2/3 m r²}), 5/7 for a solid sphere
         * ({@code I = 2/5 m r²}). It assumes the ball arrives without spin and the contact lasts long enough to
         * reach rolling, which a hollow ball on foam tile does.
         *
         * <p>Default 1.0: the floor takes no horizontal speed, which is how the physics behaved before this
         * setting existed. A season sets its piece's value.
         *
         * @param value fraction of horizontal speed kept per bounce, 0 to 1
         * @return this builder
         */
        public Builder pieceToFloorTangentialRetention(double value) {
            this.pieceToFloorTangentialRetention = requireRestitution(value, "pieceToFloorTangentialRetention");
            return this;
        }

        /**
         * The piece-to-piece restitution for pieces in the 3-D model (in a world with {@link KinematicBody bodies}:
         * in flight, or resting in a cell), when it differs from the loose pieces' on the floor
         * ({@link #pieceToPieceRestitution}). A season whose reference has two ball models, a flat one and a 3-D
         * one, with different numbers, sets both.
         *
         * <p>Default: not set, so the 3-D model uses {@link #pieceToPieceRestitution}, as it always has.
         *
         * @param value coefficient of restitution, 0 to 1
         * @return this builder
         */
        public Builder pieceToPieceRestitution3d(double value) {
            this.pieceToPieceRestitution3d = requireRestitution(value, "pieceToPieceRestitution3d");
            return this;
        }

        /**
         * The perimeter wall's restitution for a piece in the 3-D model, when it differs from a loose piece's
         * ({@link #pieceToWallRestitution}). Default: not set, the same as {@link #pieceToWallRestitution}.
         *
         * @param value coefficient of restitution, 0 to 1
         * @return this builder
         */
        public Builder pieceToWallRestitution3d(double value) {
            this.pieceToWallRestitution3d = requireRestitution(value, "pieceToWallRestitution3d");
            return this;
        }

        /**
         * An {@link ObstacleBox}'s restitution for a piece in the 3-D model, when it differs from a loose piece's
         * ({@link #pieceToObstacleRestitution}). Default: not set, the same as {@link #pieceToObstacleRestitution}.
         *
         * @param value coefficient of restitution, 0 to 1
         * @return this builder
         */
        public Builder pieceToObstacleRestitution3d(double value) {
            this.pieceToObstacleRestitution3d = requireRestitution(value, "pieceToObstacleRestitution3d");
            return this;
        }

        /**
         * Coulomb friction between the floor and a piece in the 3-D model when it lands, with spin: the floor's
         * grip on the sliding contact point both slows the piece and spins it, as against a {@link KinematicBody}.
         * A ball that arrives sliding leaves rolling (a thin shell then keeps 0.6 of its horizontal speed, a solid
         * ball 5/7), and one that arrives already rolling, as on its second bounce, keeps its speed. Set, it
         * replaces {@link #pieceToFloorTangentialRetention} for those pieces; that one takes the same share on
         * every bounce.
         *
         * <p>Default: not set, so every landing uses {@link #pieceToFloorTangentialRetention}, as it always has.
         *
         * @param value 0 or more
         * @return this builder
         */
        public Builder pieceToFloorFriction3d(double value) {
            this.pieceToFloorFriction3d = requireNonNegative(value, "pieceToFloorFriction3d");
            return this;
        }

        /**
         * Default 28 in/s² (about 0.073 g). CARRIED OVER, not measured.
         *
         * @param value rolling deceleration, inches per second squared, 0 or more
         * @return this builder
         */
        public Builder rollingDecelerationInchesPerS2(double value) {
            this.rollingDecelerationInchesPerS2 = requireNonNegative(value, "rollingDecelerationInchesPerS2");
            return this;
        }

        /**
         * Default 2 in/s: below this a rolling piece snaps to rest. CARRIED OVER.
         *
         * @param value rest speed, inches per second, 0 or more
         * @return this builder
         */
        public Builder restSpeedInchesPerS(double value) {
            this.restSpeedInchesPerS = requireNonNegative(value, "restSpeedInchesPerS");
            return this;
        }

        /**
         * Default 0 (no drag), so a launched piece flies the drag-free arc the shot solver assumes.
         *
         * @param value quadratic drag coefficient, per inch, 0 or more
         * @return this builder
         */
        public Builder airDragPerInch(double value) {
            this.airDragPerInch = requireNonNegative(value, "airDragPerInch");
            return this;
        }

        /**
         * Default 0.005 s: the longest physics sub-step. A 20 ms sim tick becomes four 5 ms steps, a
         * 40 ms tick eight, a 10 ms tick two - enough for 260 in/s pieces at any tick length (see
         * {@link GamePieceTracker#advance}).
         *
         * @param value the longest sub-step, seconds, at least {@link #MIN_MAX_SUBSTEP_SECONDS}
         * @return this builder
         */
        public Builder maxSubstepSeconds(double value) {
            requirePositive(value, "maxSubstepSeconds");
            if (value < MIN_MAX_SUBSTEP_SECONDS) {
                throw new IllegalArgumentException("maxSubstepSeconds must be at least "
                        + MIN_MAX_SUBSTEP_SECONDS + " s (one microsecond), got " + value
                        + "; a shorter step would split a tick into an absurd number of sub-steps");
            }
            this.maxSubstepSeconds = value;
            return this;
        }

        /**
         * Default 0.6. CARRIED OVER, not measured on a real ball.
         *
         * @param value Coulomb friction coefficient, 0 or more
         * @return this builder
         */
        public Builder pieceToPieceFriction(double value) {
            this.pieceToPieceFriction = requireNonNegative(value, "pieceToPieceFriction");
            return this;
        }

        /**
         * Default 2/3: a thin hollow sphere, {@code I = 2/3 m r^2}. APPROX: a hollow plastic ball with a wall
         * much thinner than its radius; a solid ball is 2/5.
         *
         * @param value moment of inertia as a fraction of {@code m r^2}, more than 0 and at most 1
         * @return this builder
         */
        public Builder pieceInertiaFactor(double value) {
            requirePositive(value, "pieceInertiaFactor");
            if (value > 1.0) {
                throw new IllegalArgumentException("pieceInertiaFactor is at most 1 (all mass at the radius), got " + value);
            }
            this.pieceInertiaFactor = value;
            return this;
        }

        /**
         * Default 0.4 per second. CARRIED OVER, not measured.
         *
         * @param value spin damping, per second, 0 or more
         * @return this builder
         */
        public Builder spinDampingPerSecond(double value) {
            this.spinDampingPerSecond = requireNonNegative(value, "spinDampingPerSecond");
            return this;
        }

        /**
         * Default 1 in/s: SET BY HAND. A centre that moves less than 0.005 in in a 5 ms sub-step; a ball rolling
         * or falling at any speed worth drawing moves more.
         *
         * @param value sleep speed, inches per second, 0 or more
         * @return this builder
         */
        public Builder sleepSpeedInchesPerS(double value) {
            this.sleepSpeedInchesPerS = requireNonNegative(value, "sleepSpeedInchesPerS");
            return this;
        }

        /**
         * Default 0.25 s: SET BY HAND. Fifty 5 ms sub-steps of stillness before a piece is put to sleep.
         *
         * @param value time still before sleeping, seconds, more than 0
         * @return this builder
         */
        public Builder sleepSeconds(double value) {
            this.sleepSeconds = requirePositive(value, "sleepSeconds");
            return this;
        }

        /**
         * Default 10 in/s: SET BY HAND. Well above the few inches per second the contacts of a resting pile carry
         * from one sub-step to the next, and below a piece dropped from half an inch (20 in/s), so a pile does not
         * wake itself and anything that actually falls or flies onto it does.
         *
         * @param value wake speed, inches per second, 0 or more
         * @return this builder
         */
        public Builder wakeSpeedInchesPerS(double value) {
            this.wakeSpeedInchesPerS = requireNonNegative(value, "wakeSpeedInchesPerS");
            return this;
        }

        /**
         * Default 3: SET BY HAND. Each pass resolves every 3-D contact once in turn; a stack of pieces passes
         * its weight down one piece per pass, so three keeps a short pile from sagging within one sub-step.
         *
         * @param value contact passes per sub-step, at least 1
         * @return this builder
         */
        public Builder contactIterations(int value) {
            if (value < 1) {
                throw new IllegalArgumentException("contactIterations must be >= 1, got " + value);
            }
            this.contactIterations = value;
            return this;
        }

        /**
         * Default 1.0 in: SET BY HAND, below the radius of any piece the sim has carried (1.4 in and up), so a
         * centre never crosses a surface between two tests (see the getter).
         *
         * @param value longest move between two contact tests, inches, more than 0
         * @return this builder
         */
        public Builder maxContactTravelInches(double value) {
            this.maxContactTravelInches = requirePositive(value, "maxContactTravelInches");
            return this;
        }

        /**
         * The old way to set the sub-step: split every tick into {@code value} equal steps. That made the
         * sub-step length depend on the caller's tick, so it now means "the sub-step a
         * {@link #NOMINAL_TICK_SECONDS 20 ms} tick split {@code value} ways would have":
         * {@code maxSubstepSeconds(0.020 / value)}. At a 20 ms tick the behaviour is exactly what it was.
         *
         * @param value sub-steps per 20 ms tick, at least 1
         * @return this builder
         * @deprecated use {@link #maxSubstepSeconds(double)}, which says what it means at any tick.
         */
        @Deprecated
        public Builder substepsPerTick(int value) {
            if (value < 1) {
                throw new IllegalArgumentException("substepsPerTick must be >= 1, got " + value);
            }
            return maxSubstepSeconds(NOMINAL_TICK_SECONDS / value);   // same floor check as the new setter
        }

        /**
         * Builds the config from the numbers set so far.
         *
         * @return a new immutable config
         */
        public GamePiecePhysicsConfig build() {
            return new GamePiecePhysicsConfig(this);
        }

        private static double requirePositive(double value, String name) {
            if (!Double.isFinite(value) || value <= 0.0) {
                throw new IllegalArgumentException(name + " must be a positive finite number, got " + value);
            }
            return value;
        }

        private static double requireNonNegative(double value, String name) {
            if (!Double.isFinite(value) || value < 0.0) {
                throw new IllegalArgumentException(name + " must be a finite number >= 0, got " + value);
            }
            return value;
        }

        private static double requireRestitution(double value, String name) {
            if (!Double.isFinite(value) || value < 0.0 || value > 1.0) {
                throw new IllegalArgumentException(name + " must be in [0, 1], got " + value);
            }
            return value;
        }
    }
}
