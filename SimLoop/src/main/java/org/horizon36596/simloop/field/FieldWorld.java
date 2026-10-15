package org.horizon36596.simloop.field;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * Everything fixed about the field that a game piece can touch: the perimeter wall, the solid
 * {@link ObstacleBox obstacles}, the {@link ScoringVolume goals}, the {@link KinematicBody rigid structures}
 * pieces collide with and rest in, the robot's own footprint, and the
 * {@link GamePiecePhysicsConfig physics numbers}. The season layer builds one and hands it to
 * {@link GamePieceTracker}; the core never knows what game the boxes belong to (domain R7).
 *
 * <p><b>Frame and units.</b> Field frame, FTC-Cartesian, centre origin, inches (conventions §7). The
 * perimeter is the square {@code |x| <= halfWidth, |y| <= halfLength}; a world built with
 * {@link #unbounded()} has no perimeter, no obstacles, no goals and no robot footprint or height — pieces still roll,
 * fly and collide with each other, they just never hit anything fixed. That is the world the original
 * one-argument {@link GamePieceTracker#GamePieceTracker(int)} constructor gives you, so every scenario
 * written before the physics existed behaves as it did.
 *
 * <p>Immutable. Build one with {@link #builder()}.
 */
public final class FieldWorld {

    private final boolean bounded;
    private final double halfWidthInches;
    private final double halfLengthInches;
    private final List<ObstacleBox> obstacles;
    private final List<ScoringVolume> scoringVolumes;
    private final List<KinematicBody> bodies;
    private final double robotLengthInches;
    private final double robotWidthInches;
    private final double robotHeightInches;
    private final GamePiecePhysicsConfig physics;

    private FieldWorld(Builder builder) {
        this.bounded = builder.bounded;
        this.halfWidthInches = builder.halfWidthInches;
        this.halfLengthInches = builder.halfLengthInches;
        this.obstacles = Collections.unmodifiableList(new ArrayList<>(builder.obstacles));
        this.scoringVolumes = Collections.unmodifiableList(new ArrayList<>(builder.scoringVolumes));
        this.bodies = Collections.unmodifiableList(new ArrayList<>(builder.bodies));
        this.robotLengthInches = builder.robotLengthInches;
        this.robotWidthInches = builder.robotWidthInches;
        this.robotHeightInches = builder.robotHeightInches;
        this.physics = builder.physics;
    }

    /** {@return a world with default physics and nothing fixed in it — see the class javadoc} */
    public static FieldWorld unbounded() {
        return new Builder().build();
    }

    /** {@return an empty builder; with nothing set it builds the same world as {@link #unbounded()}} */
    public static Builder builder() {
        return new Builder();
    }

    /** {@return true when a perimeter wall was declared} */
    public boolean isBounded() {
        return bounded;
    }

    /** {@return half the field's extent along X, inches (72 for a 144 in FTC field); meaningless if unbounded} */
    public double getHalfWidthInches() {
        return halfWidthInches;
    }

    /** {@return half the field's extent along Y, inches; meaningless if unbounded} */
    public double getHalfLengthInches() {
        return halfLengthInches;
    }

    /** {@return the solid boxes, in declaration order} */
    public List<ObstacleBox> getObstacles() {
        return obstacles;
    }

    /** {@return the goals, in declaration order — the order they are tested in, which matters if two overlap} */
    public List<ScoringVolume> getScoringVolumes() {
        return scoringVolumes;
    }

    /**
     * A goal by name.
     *
     * @param name the goal's name, as given to its {@link ScoringVolume}
     * @return the goal of that name
     * @throws IllegalArgumentException if no goal has that name
     */
    public ScoringVolume getScoringVolume(String name) {
        for (ScoringVolume volume : scoringVolumes) {
            if (volume.getName().equals(name)) {
                return volume;
            }
        }
        throw new IllegalArgumentException("no scoring volume named " + name);
    }

    /**
     * {@return the rigid structures pieces collide with and rest in, in declaration order} Empty for a world
     * that has none, which then behaves exactly as it did before bodies existed.
     */
    public List<KinematicBody> getKinematicBodies() {
        return bodies;
    }

    /** {@return true when the world has at least one {@link KinematicBody}} */
    public boolean hasKinematicBodies() {
        return !bodies.isEmpty();
    }

    /**
     * A body by name.
     *
     * @param name the body's name, as given to its {@link KinematicBody}
     * @return the body of that name
     * @throws IllegalArgumentException if no body has that name
     */
    public KinematicBody getKinematicBody(String name) {
        for (KinematicBody body : bodies) {
            if (body.getName().equals(name)) {
                return body;
            }
        }
        throw new IllegalArgumentException("no kinematic body named " + name);
    }

    /**
     * {@return the body cell of that name, or null when no body has one}
     *
     * @param name the cell's name, as given to its {@link BodyCell}
     */
    public BodyCell findCell(String name) {
        for (KinematicBody body : bodies) {
            for (BodyCell cell : body.getCells()) {
                if (cell.getName().equals(name)) {
                    return cell;
                }
            }
        }
        return null;
    }

    /**
     * {@return the body that owns the cell of that name, or null when no body has one}
     *
     * @param name the cell's name, as given to its {@link BodyCell}
     */
    public KinematicBody findCellOwner(String name) {
        for (KinematicBody body : bodies) {
            for (BodyCell cell : body.getCells()) {
                if (cell.getName().equals(name)) {
                    return body;
                }
            }
        }
        return null;
    }

    /** {@return true when the robot has a footprint that pushes loose pieces} */
    public boolean hasRobotFootprint() {
        return robotLengthInches > 0.0 && robotWidthInches > 0.0;
    }

    /** {@return the robot's full extent along its own forward axis, inches; 0 when no footprint} */
    public double getRobotLengthInches() {
        return robotLengthInches;
    }

    /** {@return the robot's full extent along its own left axis, inches; 0 when no footprint} */
    public double getRobotWidthInches() {
        return robotWidthInches;
    }

    /**
     * {@return true when the robot is a solid box that in-flight pieces bounce off: it has a footprint and a
     * height} Without a height only loose pieces on the floor feel the robot, and a piece in the air passes
     * through it, which is how every world built before the height existed behaves.
     */
    public boolean hasRobotBody() {
        return hasRobotFootprint() && robotHeightInches > 0.0;
    }

    /** {@return the robot's height above the floor, inches: the top of the box in-flight pieces hit; 0 when not declared} */
    public double getRobotHeightInches() {
        return robotHeightInches;
    }

    /** {@return the physics numbers} */
    public GamePiecePhysicsConfig getPhysics() {
        return physics;
    }

    /** One named setter per fact. Everything is optional; the empty builder gives {@link #unbounded()}. */
    public static final class Builder {

        private boolean bounded = false;
        private double halfWidthInches = 0.0;
        private double halfLengthInches = 0.0;
        private final List<ObstacleBox> obstacles = new ArrayList<>();
        private final List<ScoringVolume> scoringVolumes = new ArrayList<>();
        private final List<KinematicBody> bodies = new ArrayList<>();
        private double robotLengthInches = 0.0;
        private double robotWidthInches = 0.0;
        private double robotHeightInches = 0.0;
        private GamePiecePhysicsConfig physics = GamePiecePhysicsConfig.defaults();

        private Builder() {
        }

        /**
         * A perimeter wall at {@code |x| == halfWidthInches} and {@code |y| == halfLengthInches}. Pieces
         * bounce off it with {@link GamePiecePhysicsConfig#getPieceToWallRestitution()}.
         *
         * @param halfWidthInches  half the field's extent along X, inches; finite and positive
         * @param halfLengthInches half the field's extent along Y, inches; finite and positive
         * @return this builder
         */
        public Builder perimeter(double halfWidthInches, double halfLengthInches) {
            if (!Double.isFinite(halfWidthInches) || halfWidthInches <= 0.0
                    || !Double.isFinite(halfLengthInches) || halfLengthInches <= 0.0) {
                throw new IllegalArgumentException("perimeter half-extents must be positive and finite");
            }
            this.bounded = true;
            this.halfWidthInches = halfWidthInches;
            this.halfLengthInches = halfLengthInches;
            return this;
        }

        /**
         * A fixed solid box pieces bounce off (see {@link ObstacleBox}).
         *
         * @param box the box, field frame, inches
         * @return this builder
         */
        public Builder obstacle(ObstacleBox box) {
            if (box == null) {
                throw new IllegalArgumentException("obstacle must not be null");
            }
            obstacles.add(box);
            return this;
        }

        /**
         * A goal pieces are counted in (see {@link ScoringVolume}). Its name must be new among every goal and
         * cell.
         *
         * @param volume the goal
         * @return this builder
         */
        public Builder scoringVolume(ScoringVolume volume) {
            if (volume == null) {
                throw new IllegalArgumentException("scoring volume must not be null");
            }
            for (ScoringVolume existing : scoringVolumes) {
                if (existing.getName().equals(volume.getName())) {
                    throw new IllegalArgumentException("two scoring volumes named " + volume.getName());
                }
            }
            requireUnusedName(volume.getName());
            scoringVolumes.add(volume);
            return this;
        }

        /**
         * A rigid structure pieces collide with and can rest in (see {@link KinematicBody}). Its name must be new
         * among the bodies, and each of its cells' names new among every cell and scoring volume, because a cell
         * is counted by name exactly like a goal.
         *
         * @param body the body; its angle starts at zero and is set through
         *             {@link GamePieceTracker#setBodyMotion}
         * @return this builder
         */
        public Builder kinematicBody(KinematicBody body) {
            if (body == null) {
                throw new IllegalArgumentException("kinematic body must not be null");
            }
            for (KinematicBody existing : bodies) {
                if (existing.getName().equals(body.getName())) {
                    throw new IllegalArgumentException("two kinematic bodies named " + body.getName());
                }
            }
            for (BodyCell cell : body.getCells()) {
                requireUnusedName(cell.getName());
                for (ScoringVolume volume : scoringVolumes) {
                    if (volume.getName().equals(cell.getName())) {
                        throw new IllegalArgumentException("a cell and a scoring volume are both named " + cell.getName());
                    }
                }
            }
            bodies.add(body);
            return this;
        }

        // A cell and a goal are both counted by name, so no two of either may share one.
        private void requireUnusedName(String name) {
            for (KinematicBody body : bodies) {
                for (BodyCell cell : body.getCells()) {
                    if (cell.getName().equals(name)) {
                        throw new IllegalArgumentException("two cells or goals named " + name);
                    }
                }
            }
        }

        /**
         * The robot as a rectangle in its own frame, centred on its pose, that pushes loose pieces out of
         * its way. Full extents, inches: {@code lengthInches} along forward, {@code widthInches} along left.
         *
         * @param lengthInches the robot's full extent along its forward axis, inches; finite and positive
         * @param widthInches  the robot's full extent along its left axis, inches; finite and positive
         * @return this builder
         */
        public Builder robotFootprint(double lengthInches, double widthInches) {
            if (!Double.isFinite(lengthInches) || lengthInches <= 0.0
                    || !Double.isFinite(widthInches) || widthInches <= 0.0) {
                throw new IllegalArgumentException("robot footprint extents must be positive and finite");
            }
            this.robotLengthInches = lengthInches;
            this.robotWidthInches = widthInches;
            return this;
        }

        /**
         * How tall the robot is, inches above the floor. With a {@link #robotFootprint footprint} this makes
         * the robot a solid box, floor to {@code heightInches}, that <b>in-flight</b> pieces bounce off — its
         * sides and its top — with {@link GamePiecePhysicsConfig#getPieceToRobotRestitution()}. Without it an
         * airborne piece passes straight through the robot.
         *
         * @param heightInches the robot's height, inches; finite and positive
         * @return this builder
         */
        public Builder robotHeight(double heightInches) {
            if (!Double.isFinite(heightInches) || heightInches <= 0.0) {
                throw new IllegalArgumentException("robot height must be positive and finite, got " + heightInches);
            }
            this.robotHeightInches = heightInches;
            return this;
        }

        /**
         * The physics numbers; {@link GamePiecePhysicsConfig#defaults()} when not set.
         *
         * @param physics the physics numbers
         * @return this builder
         */
        public Builder physics(GamePiecePhysicsConfig physics) {
            if (physics == null) {
                throw new IllegalArgumentException("physics must not be null");
            }
            this.physics = physics;
            return this;
        }

        /** {@return a world holding everything set so far} */
        public FieldWorld build() {
            return new FieldWorld(this);
        }
    }
}
