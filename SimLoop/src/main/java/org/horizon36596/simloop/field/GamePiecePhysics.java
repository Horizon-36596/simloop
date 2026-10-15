package org.horizon36596.simloop.field;

import java.util.Collections;
import java.util.List;

/**
 * The numbers behind {@link GamePieceTracker#advance}: how loose pieces roll, get pushed and collide, and
 * how in-flight pieces fly, bounce, and enter goals. Package-private on purpose — the tracker is the only
 * caller, and it alone turns the {@link Event events} this class reports into logged possession
 * transitions, so "every transition is logged" stays true by construction.
 *
 * <p><b>Read this before changing a formula.</b> Everything here is the simplest model that produces the
 * behaviour the human asked for on 2026-09-20 (pushed, rolling, colliding, flying balls), written so a
 * student can check each step against a first mechanics course:
 * <ul>
 *   <li><b>Integration</b> is semi-implicit Euler at sub-steps no longer than the config's
 *       {@link GamePiecePhysicsConfig#getMaxSubstepSeconds() maxSubstepSeconds} (5 ms by default),
 *       whatever tick the caller advances by: velocity first, then position from the new velocity.
 *       Stable for the speeds and steps in play (a 260 in/s piece moves 1.3 in per 5 ms sub-step, under
 *       one radius of a 2.8 in ball).</li>
 *   <li><b>Rolling</b> is a constant deceleration along the direction of travel, snapped to rest below a
 *       small speed, with no distinction between rolling and sliding.</li>
 *   <li><b>Flight</b> is gravity plus optional quadratic drag. No spin, no lift.</li>
 *   <li><b>Every contact</b> is resolved the same way: push the piece out of whatever it overlaps along
 *       the contact normal, then reverse and scale the velocity component along that normal by the
 *       contact's restitution. Against a rectangle (the robot, an obstacle) the normal is the shortest
 *       axis when the centre is beside a face, and the line from the nearest corner to the centre when it
 *       is off a corner - a ball is round, so it only touches a corner when its centre is within one
 *       radius of that corner. Loose pieces have equal mass; walls, obstacles and the robot are immovable.</li>
 *   <li><b>Goals</b> are entered along the path: a flying piece scores on the sub-step the straight line
 *       its centre moved along touches the goal's box, so a fast piece cannot skip a thin goal between
 *       two sub-steps ({@link ScoringVolume#acceptsPath}).</li>
 *   <li><b>Only loose pieces touch each other</b> (in a world without bodies; see the 3-D model below). An in-flight piece interacts with gravity, drag, the
 *       floor, the perimeter, obstacles, goals and — when the world gives the robot a
 *       {@link FieldWorld#hasRobotBody() height} — the robot's sides and top, but not with other pieces.
 *       Two balls colliding in mid-air are outside what the sim claims to model, and saying so here beats
 *       pretending. The robot contact is swept along the piece's path relative to the robot, so a piece
 *       is pushed back out of the face it flew in through ({@link #bounceInFlightOffRobot}).</li>
 *   <li><b>The 3-D model</b>, only in a world with {@link KinematicBody bodies}. A piece in flight, or resting
 *       in a body's {@link BodyCell cell}, collides with the bodies' convex shapes and with every other such
 *       piece as a sphere in 3-D: pushed out along the contact normal, its velocity into the surface (relative
 *       to the surface's own motion, so a turning body carries a piece along) reversed and scaled by the
 *       restitution - zero below the rest speed - and Coulomb friction on the sliding of its contact point,
 *       which spins it, so a ball rolls down a slope rather than sliding. Contacts are resolved one after
 *       another, {@link GamePiecePhysicsConfig#getContactIterations() a few passes} per sub-step. A piece near
 *       a body moves in steps shorter than its radius, so it cannot pass through a thin wall. A piece that has
 *       been still for a while on what it rests on is put to sleep until a body near it moves or another piece
 *       arrives fast. A flying piece that comes down into a cell is SCORED in it, and a piece the physics
 *       carries out of its cell is IN_FLIGHT again, logged {@code "<cell>Out"}: what a cell holds is wherever
 *       the physics put the pieces, never a parked list. In this model in-flight pieces also hit each other,
 *       sharing each impulse by their masses ({@link GamePiecePhysicsConfig#getPieceMass}); loose pieces on the
 *       floor still do not meet the bodies, nor pieces in the air.</li>
 *   <li><b>Dynamic bodies.</b> A body with {@link HingeDynamics} turns by itself: each sub-step gravity on its own
 *       mass and on the pieces asleep on it (in a cell, on any of its shapes, or on pieces resting on it) turns
 *       it, damping slows it and its hard limits stop it; and every contact a piece makes with it is a two-body
 *       impulse, the piece's mass against the body's inertia at the contact's lever arm, the body taking the
 *       equal and opposite angular impulse. A piece asleep on it is part of it: an awake piece landing on one
 *       pushes the body through it. Each shape can carry its own restitution and friction
 *       ({@link KinematicBody#withShapeMaterial}). A dynamic
 *       body the season {@linkplain #setBodyHeld holds} is a kinematic body again until it is let go.</li>
 *   <li><b>A robot made of parts.</b> When the tracker has a {@link RobotShape}, the robot is that compound
 *       of {@link OrientedBox boxes} instead of one footprint box. A loose piece meets each part by the
 *       part's point nearest its centre, horizontally only, so a part whose underside is above the ball
 *       (an intake mouth) lets it roll under; an in-flight piece is swept against every face of every
 *       part. Robots standing still ({@link StaticRobot}) are solid boxes from the floor up for both.</li>
 * </ul>
 *
 * <p><b>Determinism (domain R5).</b> No clock, no allocation of anything whose order matters, every
 * transcendental through {@link StrictMath}, and every loop over pieces in id order — so the same state
 * and the same {@code deltaTime} give bit-identical results on any JVM.
 *
 * <p><b>Known limits, on purpose.</b>
 * <ul>
 *   <li><b>Obstacle contact is not swept.</b> It is tested where the piece is at the end of each
 *       sub-step, and an overlap is pushed out of the <em>nearer</em> face. So a piece of radius
 *       {@code r} moving {@code d = v * dt_sub} per sub-step can come out the far side of an obstacle of
 *       thickness {@code t} when it lands past the obstacle's middle, which is possible only if
 *       {@code t < 2 * (d - r)}. Worked example at the default 5 ms sub-step and 260 in/s
 *       ({@code d = 1.3 in}): a 2.8 in ball ({@code r = 1.4}) gives {@code 2 * (1.3 - 1.4) < 0}, so it
 *       cannot tunnel through anything, however thin; a 1 in ball ({@code r = 0.5}) can pass through an
 *       obstacle thinner than {@code 2 * (1.3 - 0.5) = 1.6 in}; a point-sized piece through anything
 *       thinner than 2.6 in. Goals are swept (above) and do not have this limit.</li>
 *   <li><b>A squeezed piece ends up inside the robot.</b> Contacts are resolved one after another, not
 *       together: the robot pushes a piece out first, then a wall or obstacle pushes it back. A piece
 *       pinned between the robot and a wall is therefore left where the wall put it, which can be inside
 *       the robot's footprint. Nothing breaks - the next sub-step pushes it again - but a viewer can see a
 *       ball drawn inside the robot while it drives into a wall with a ball in front of it.</li>
 *   <li><b>A goal next to a wall judges a bounced piece after the bounce.</b> The goal test runs after the
 *       perimeter and obstacle pushes of the same sub-step, on the segment from the start point to the
 *       pushed end point and on the pushed velocity. A piece that hits a wall and a goal mouth in one
 *       sub-step can therefore be judged with a reversed horizontal velocity and a slightly shifted path
 *       (see {@link ScoringVolume#acceptsPath}).</li>
 *   <li>The robot pushes with its centre's velocity only; a robot spinning in place does not fling a
 *       ball with its corner.</li>
 *   <li>A piece in flight meets goals, the floor, the perimeter and obstacles, but not other pieces: a low
 *       shot passes through a ball on the tiles instead of knocking it.</li>
 * </ul>
 * None of this matters for a foam ball on a field of thick walls, and they are the first things to revisit
 * if it ever does.
 */
final class GamePiecePhysics {

    /** What a physics step wants the tracker to log: a piece changed state for a stated reason. */
    static final class Event {
        final GamePiece piece;
        final PossessionState to;
        final String cause;
        /** The goal the piece entered, when {@code to} is SCORED into a goal; null otherwise. */
        final ScoringVolume volume;
        /** The body cell the piece came to rest in, when {@code to} is SCORED into a cell; null otherwise. */
        final BodyCell cell;

        Event(GamePiece piece, PossessionState to, String cause, ScoringVolume volume) {
            this(piece, to, cause, volume, null);
        }

        Event(GamePiece piece, PossessionState to, String cause, ScoringVolume volume, BodyCell cell) {
            this.piece = piece;
            this.to = to;
            this.cause = cause;
            this.volume = volume;
            this.cell = cell;
        }
    }

    /** The robot for one tick: where it is and how fast its centre is moving, field frame. */
    static final class Robot {
        final double xInches;
        final double yInches;
        final double headingRad;
        final double velocityXInchesPerS;
        final double velocityYInchesPerS;
        /**
         * The robot's solid parts placed in the field this tick ({@link RobotShape#placedOnRobot}), or null to
         * use the world's single footprint box instead ({@link FieldWorld#hasRobotFootprint()}).
         */
        final List<OrientedBox> parts;

        Robot(double xInches, double yInches, double headingRad, double velocityXInchesPerS,
                double velocityYInchesPerS) {
            this(xInches, yInches, headingRad, velocityXInchesPerS, velocityYInchesPerS, null);
        }

        Robot(double xInches, double yInches, double headingRad, double velocityXInchesPerS,
                double velocityYInchesPerS, List<OrientedBox> parts) {
            this.xInches = xInches;
            this.yInches = yInches;
            this.headingRad = headingRad;
            this.velocityXInchesPerS = velocityXInchesPerS;
            this.velocityYInchesPerS = velocityYInchesPerS;
            this.parts = parts;
        }
    }

    private final FieldWorld world;
    private final GamePiecePhysicsConfig config;
    private final List<KinematicBody> bodies;

    // Each body's angle (radians) and turning rate (radians per second), indexed like the world's bodies. The
    // season sets them through the tracker; each sub-step advances the angle by rate * dt.
    private final double[] bodyAngleRad;
    private final double[] bodyAngularVelocityRadPerS;
    // True when the season moved a body since the last sub-step, so pieces asleep on it must wake.
    private final boolean[] bodyDisturbed;
    // Per sub-step: is the body moving (turning, or just moved by the season)?
    private final boolean[] bodyMoving;
    // A dynamic body the season holds still (a latch, a detent): it does not integrate and takes no impulse.
    private final boolean[] bodyHeld;

    // Scratch vectors, reused so a sub-step allocates nothing per contact.
    private final double[] local = new double[3];
    private final double[] contact = new double[4];
    private final double[] normal = new double[3];
    private final double[] surface = new double[3];
    private final double[] frictionImpulse = new double[3];
    private final double[] leverArm = new double[3];

    /** Robots that stand still ({@link StaticRobot#toBox()}), field frame; set by the tracker before each tick. */
    private List<OrientedBox> staticRobotBoxes = Collections.emptyList();

    GamePiecePhysics(FieldWorld world) {
        this.world = world;
        this.config = world.getPhysics();
        this.bodies = world.getKinematicBodies();
        this.bodyAngleRad = new double[bodies.size()];
        this.bodyAngularVelocityRadPerS = new double[bodies.size()];
        this.bodyDisturbed = new boolean[bodies.size()];
        this.bodyMoving = new boolean[bodies.size()];
        this.bodyHeld = new boolean[bodies.size()];
    }

    /**
     * Set one body's angle and turning rate, as the season's model of it says (see
     * {@link GamePieceTracker#setBodyMotion}). For a dynamic body this is where it is now; the physics moves it on.
     *
     * @param index                  the body's index in the world's list
     * @param angleRad               its angle now, radians
     * @param angularVelocityRadPerS how fast it is turning, radians per second; the angle advances by this
     *                               within each sub-step until the next call
     */
    void setBodyMotion(int index, double angleRad, double angularVelocityRadPerS) {
        if (angleRad != bodyAngleRad[index] || angularVelocityRadPerS != 0.0) {
            bodyDisturbed[index] = true;
        }
        bodyAngleRad[index] = angleRad;
        bodyAngularVelocityRadPerS[index] = angularVelocityRadPerS;
    }

    /** {@return a body's angle now, radians} */
    double getBodyAngleRad(int index) {
        return bodyAngleRad[index];
    }

    /** {@return a body's turning rate now, radians per second: as set for a kinematic body, as integrated for a dynamic one} */
    double getBodyAngularVelocityRadPerS(int index) {
        return bodyAngularVelocityRadPerS[index];
    }

    /**
     * Hold a dynamic body still where it is, or let it go (see {@link GamePieceTracker#setBodyHeld}). Holding it
     * stops it - its turning rate becomes 0 - and while held it is moved like a kinematic body, by
     * {@link #setBodyMotion} only. A kinematic body ignores this.
     *
     * @param index the body's index in the world's list
     * @param held  true to hold it, false to let the physics move it
     */
    void setBodyHeld(int index, boolean held) {
        if (held) {
            bodyAngularVelocityRadPerS[index] = 0.0;
        }
        bodyHeld[index] = held;
    }

    /** {@return whether a dynamic body is held still by the season} */
    boolean isBodyHeld(int index) {
        return bodyHeld[index];
    }

    /** A dynamic body the physics moves this sub-step: dynamic, and not held. */
    private boolean isFreeHinge(int b) {
        return bodies.get(b).isDynamic() && !bodyHeld[b];
    }

    /**
     * The cell a piece whose centre is at this point counts as having entered (seat depth included), or null.
     *
     * @param x field frame, inches
     * @param y ditto
     * @param z ditto
     * @return the first such cell in world order, or null
     */
    BodyCell cellAt(double x, double y, double z) {
        for (int b = 0; b < bodies.size(); b++) {
            KinematicBody body = bodies.get(b);
            if (body.getCells().isEmpty()) {
                continue;
            }
            body.toLocal(bodyAngleRad[b], x, y, z, local);
            for (BodyCell cell : body.getCells()) {
                if (cell.contains(local[0], local[1], local[2], false)) {
                    return cell;
                }
            }
        }
        return null;
    }

    /** Hand over the robots standing still this tick, field frame; an empty list for none. */
    void setStaticRobotBoxes(List<OrientedBox> boxes) {
        this.staticRobotBoxes = boxes;
    }

    /**
     * Advance every non-held piece by one sub-step. The caller splits its tick into sub-steps no longer
     * than the config's {@code maxSubstepSeconds} and calls this once for each.
     *
     * @param pieces  every piece, in id order; held pieces are skipped
     * @param robot   the robot this tick, or null when its pose is not known yet (then nothing is pushed)
     * @param dt      the sub-step, seconds
     * @param events  state changes to log are appended here, in the order they happened
     */
    void step(List<GamePiece> pieces, Robot robot, double dt, List<Event> events) {
        boolean threeD = !bodies.isEmpty();
        boolean[] touched = threeD ? new boolean[pieces.size()] : null;
        if (threeD) {
            advanceBodies(pieces, dt);
        }

        // 0. Where each piece's centre starts this sub-step, so step 5 can test the whole path it moved
        //    along against the goals, not only where it ended up. Indexed like the piece list.
        double[] startX = new double[pieces.size()];
        double[] startY = new double[pieces.size()];
        double[] startZ = new double[pieces.size()];
        for (int i = 0; i < pieces.size(); i++) {
            GamePiece piece = pieces.get(i);
            startX[i] = piece.getFieldXInches();
            startY[i] = piece.getFieldYInches();
            startZ[i] = piece.getFieldZInches();
        }

        // 1. Free motion: gravity/drag for flight, rolling deceleration on the floor, then integrate.
        //    In a world with bodies, a piece in the 3-D model is moved in steps short enough to be tested
        //    against every surface on the way (moveAmongBodies); one asleep is not moved at all.
        for (int i = 0; i < pieces.size(); i++) {
            GamePiece piece = pieces.get(i);
            if (threeD && isFree3d(piece)) {
                if (!piece.isSleeping()) {
                    moveAmongBodies(piece, dt, touched, i);
                }
            } else if (piece.isInFlight()) {
                integrateFlight(piece, dt);
            } else if (piece.getState() == PossessionState.LOOSE) {
                integrateRolling(piece, dt);
            }
        }

        // 2. The robot shoves loose pieces out of its footprint, and, when it has a height, in-flight pieces
        //    bounce off its sides and top.
        //    A robot built from parts (a RobotShape) uses those instead of the world's one box.
        if (robot != null && robot.parts != null) {
            for (int i = 0; i < pieces.size(); i++) {
                GamePiece piece = pieces.get(i);
                if (piece.getState() == PossessionState.LOOSE) {
                    pushOutOfRobotParts(piece, robot);
                } else if (piece.isInFlight()) {
                    bounceInFlightOffRobotParts(piece, robot, startX[i], startY[i], startZ[i], dt);
                }
            }
        } else if (robot != null && world.hasRobotFootprint()) {
            for (int i = 0; i < pieces.size(); i++) {
                GamePiece piece = pieces.get(i);
                if (piece.getState() == PossessionState.LOOSE) {
                    pushOutOfRobot(piece, robot);
                } else if (piece.isInFlight() && world.hasRobotBody()) {
                    bounceInFlightOffRobot(piece, robot, startX[i], startY[i], startZ[i], dt);
                }
            }
        }

        // 3. Loose pieces bounce off each other. Each unordered pair once, lower id first.
        for (int i = 0; i < pieces.size(); i++) {
            GamePiece a = pieces.get(i);
            if (a.getState() != PossessionState.LOOSE) {
                continue;
            }
            for (int j = i + 1; j < pieces.size(); j++) {
                GamePiece b = pieces.get(j);
                if (b.getState() == PossessionState.LOOSE) {
                    collidePieces(a, b);
                }
            }
        }

        // 3b. The 3-D contacts: pieces in the 3-D model against each other and against the bodies, a few
        //     passes so a pile settles within one sub-step.
        if (threeD) {
            for (int pass = 0; pass < config.getContactIterations(); pass++) {
                for (int i = 0; i < pieces.size(); i++) {
                    GamePiece a = pieces.get(i);
                    if (!isFree3d(a)) {
                        continue;
                    }
                    for (int j = i + 1; j < pieces.size(); j++) {
                        GamePiece b = pieces.get(j);
                        if (isFree3d(b) && collidePieces3d(a, b)) {
                            touched[i] = true;
                            touched[j] = true;
                        }
                    }
                }
                for (int i = 0; i < pieces.size(); i++) {
                    GamePiece piece = pieces.get(i);
                    if (isFree3d(piece) && !piece.isSleeping() && resolveBodyContacts(piece)) {
                        touched[i] = true;
                    }
                }
            }
        }

        // 4. Fixed things: the perimeter and the obstacles, for anything loose or flying.
        for (GamePiece piece : pieces) {
            if (piece.isSleeping()) {
                continue;                          // asleep: it has not moved, so it cannot have hit anything
            }
            if (piece.getState() == PossessionState.LOOSE || piece.isInFlight()) {
                boolean inThreeD = threeD && piece.isInFlight();
                bounceOffPerimeter(piece, inThreeD);
                for (ObstacleBox box : world.getObstacles()) {
                    bounceOffObstacle(piece, box, inThreeD);
                }
                for (OrientedBox standing : staticRobotBoxes) {
                    bounceOffStaticRobot(piece, standing);
                }
            }
        }

        // 5. Flight endings: a goal entered, or the floor reached. Goals are tested first so a mouth at
        //    floor height still scores a piece rather than landing it.
        //    In a world with bodies, first: a piece resting in a cell that the physics has carried out of it
        //    leaves the cell, and a flying piece that has come down deep enough into a cell is in it.
        for (int i = 0; i < pieces.size(); i++) {
            GamePiece piece = pieces.get(i);
            if (threeD && piece.getSeatedCellName() != null) {
                if (!piece.isSleeping() && !stillInSeatedCell(piece)) {
                    events.add(new Event(piece, PossessionState.IN_FLIGHT, piece.getSeatedCellName() + "Out", null));
                }
                continue;
            }
            if (!piece.isInFlight() || piece.isSleeping()) {
                continue;
            }
            if (threeD) {
                BodyCell cell = cellEntered(piece);
                if (cell != null) {
                    events.add(new Event(piece, PossessionState.SCORED, cell.getName(), null, cell));
                    continue;
                }
            }
            ScoringVolume entered = goalEntered(piece, startX[i], startY[i], startZ[i]);
            if (entered != null) {
                events.add(new Event(piece, PossessionState.SCORED, entered.getName(), entered));
                continue;
            }
            if (landed(piece, threeD)) {
                events.add(new Event(piece, PossessionState.LOOSE, "landed", null));
            }
        }

        // 6. Sleep: a piece in the 3-D model that has been still, touching something, for long enough stops
        //    being moved until something disturbs it.
        if (threeD) {
            for (int i = 0; i < pieces.size(); i++) {
                GamePiece piece = pieces.get(i);
                if (isFree3d(piece) && !piece.isSleeping()) {
                    if (!touched[i]) {
                        piece.setSupportBodyIndex(-1);   // resting on nothing this sub-step
                    }
                    updateSleep(piece, touched[i], startX[i], startY[i], startZ[i], dt);
                }
            }
        }
    }

    // ------------------------------------------------------------------------------------ the 3-D model

    /**
     * Whether a piece is in the 3-D model: in flight, or SCORED and resting in a body's cell. Only consulted in a
     * world with bodies; everything else keeps the floor-and-flight model.
     */
    private static boolean isFree3d(GamePiece piece) {
        return piece.isInFlight()
                || (piece.getState() == PossessionState.SCORED && piece.getSeatedCellName() != null);
    }

    /**
     * Turn each body through this sub-step, and wake every sleeping piece within reach of one that moved: a
     * piece asleep on a surface that starts to turn must feel it.
     */
    private void advanceBodies(List<GamePiece> pieces, double dt) {
        for (int b = 0; b < bodies.size(); b++) {
            if (isFreeHinge(b)) {
                integrateHinge(b, pieces, dt);    // the rate after the limits: 0 for a body resting on a stop
                bodyMoving[b] = bodyDisturbed[b] || bodyAngularVelocityRadPerS[b] != 0.0;
            } else {
                bodyMoving[b] = bodyDisturbed[b] || bodyAngularVelocityRadPerS[b] != 0.0;
                bodyAngleRad[b] += bodyAngularVelocityRadPerS[b] * dt;
            }
            bodyDisturbed[b] = false;
            if (!bodyMoving[b]) {
                continue;
            }
            KinematicBody body = bodies.get(b);
            for (GamePiece piece : pieces) {
                if (piece.isSleeping() && isFree3d(piece) && withinReach(body, piece, 0.0)) {
                    piece.setSleeping(false);
                }
            }
        }
    }

    /**
     * One sub-step of a dynamic body ({@link HingeDynamics}): gravity on its own mass and on every piece asleep on it
     * ({@link #carrierOf}: in one of its cells, on any of its shapes, or on other pieces that rest on it) gives a
     * torque about the axis; then damping, the move, and the hard limits. The contact impulses of the sub-step
     * before have already changed its turning rate; an awake piece's weight reaches the body that way.
     */
    private void integrateHinge(int b, List<GamePiece> pieces, double dt) {
        KinematicBody body = bodies.get(b);
        HingeDynamics hinge = body.getHingeDynamics();
        double g = config.getGravityInchesPerS2();
        double angle = bodyAngleRad[b];
        double[] com = hinge.getCentreOfMassLocal();
        body.rotate(angle, com[0], com[1], com[2], leverArm);
        double torque = hinge.getMass() * g * weightLever(body, leverArm[0], leverArm[1]);
        double[] pivot = body.getPivot();
        for (GamePiece piece : pieces) {
            if (piece.isSleeping() && isFree3d(piece) && carrierOf(piece) == b) {
                torque += config.getPieceMass(piece.getKind()) * g
                        * weightLever(body, piece.getFieldXInches() - pivot[0], piece.getFieldYInches() - pivot[1]);
            }
        }
        double omega = bodyAngularVelocityRadPerS[b] + torque / hinge.getMomentOfInertia() * dt;
        omega /= 1.0 + hinge.getAngularDampingPerSecond() * dt;
        angle += omega * dt;
        if (angle > hinge.getMaxAngleRad()) {
            angle = hinge.getMaxAngleRad();
            if (omega > 0.0) {
                omega = -hinge.getLimitRestitution() * omega;
            }
        } else if (angle < hinge.getMinAngleRad()) {
            angle = hinge.getMinAngleRad();
            if (omega < 0.0) {
                omega = -hinge.getLimitRestitution() * omega;
            }
        }
        bodyAngleRad[b] = angle;
        bodyAngularVelocityRadPerS[b] = omega;
    }

    /**
     * The lever about a body's axis of a unit downward force (field -Z) at a point {@code (rx, ry, *)} from the
     * pivot: {@code axis . (r x (0, 0, -1))}. Times a weight, its torque.
     */
    private static double weightLever(KinematicBody body, double rx, double ry) {
        double[] axis = body.getAxis();
        return -axis[0] * ry + axis[1] * rx;
    }

    /**
     * The body a piece rests on, by index, or -1: the body it last touched from above, directly or through pieces
     * resting on it ({@link GamePiece#getSupportBodyIndex()}, kept by the contacts); failing that, the body whose
     * cell it is seated in.
     */
    private int supportOf(GamePiece piece) {
        if (piece.getSupportBodyIndex() >= 0) {
            return piece.getSupportBodyIndex();
        }
        String cellName = piece.getSeatedCellName();
        if (cellName == null) {
            return -1;
        }
        for (int b = 0; b < bodies.size(); b++) {
            for (BodyCell cell : bodies.get(b).getCells()) {
                if (cell.getName().equals(cellName)) {
                    return b;
                }
            }
        }
        return -1;
    }

    /**
     * The free dynamic body ({@link #isFreeHinge}) a sleeping piece is part of, by index, or -1 when it rests on
     * nothing, or on a body that is kinematic or held. Its weight turns that body, and what hits it pushes that
     * body.
     */
    private int carrierOf(GamePiece piece) {
        int b = supportOf(piece);
        return b >= 0 && isFreeHinge(b) ? b : -1;
    }

    /**
     * Give a dynamic body the reaction to a piece's velocity change {@code (dvx, dvy, dvz)} at a field point: the
     * body takes the impulse {@code -m dv} there, so its rate changes by {@code axis . (r x (-m dv)) / I}.
     */
    private void pushBody(int b, double pieceMass, double pointX, double pointY, double pointZ, double dvx,
            double dvy, double dvz) {
        KinematicBody body = bodies.get(b);
        double lever = leverAbout(body, pointX, pointY, pointZ, dvx, dvy, dvz);
        bodyAngularVelocityRadPerS[b] -= pieceMass * lever / body.getHingeDynamics().getMomentOfInertia();
    }

    /**
     * {@code axis . (r x u)} for the vector {@code r} from a body's pivot to a field point and a vector {@code u}:
     * the torque about the axis of a force {@code u} at that point.
     */
    private static double leverAbout(KinematicBody body, double pointX, double pointY, double pointZ, double ux,
            double uy, double uz) {
        double[] pivot = body.getPivot();
        double[] axis = body.getAxis();
        double rx = pointX - pivot[0];
        double ry = pointY - pivot[1];
        double rz = pointZ - pivot[2];
        return axis[0] * (ry * uz - rz * uy) + axis[1] * (rz * ux - rx * uz) + axis[2] * (rx * uy - ry * ux);
    }

    /** Whether a piece that may move {@code travel} inches could touch a body, by its bounding sphere. */
    private static boolean withinReach(KinematicBody body, GamePiece piece, double travel) {
        double reach = body.getBoundingRadiusInches() + piece.getRadiusInches() + travel;
        return body.distanceSquaredToPivot(piece.getFieldXInches(), piece.getFieldYInches(),
                piece.getFieldZInches()) < reach * reach;
    }

    /**
     * Free motion for a piece in the 3-D model: gravity, drag and spin decay on the velocity, then the move, cut
     * into steps no longer than {@link GamePiecePhysicsConfig#getMaxContactTravelInches()} while a body is
     * within reach, with the body contacts resolved after each step. A step shorter than the piece's radius means
     * its centre never passes a surface between two tests, however thin the solid.
     */
    private void moveAmongBodies(GamePiece piece, double dt, boolean[] touched, int index) {
        double vx = piece.getVelocityXInchesPerSecond();
        double vy = piece.getVelocityYInchesPerSecond();
        double vz = piece.getVelocityZInchesPerSecond() - config.getGravityInchesPerS2() * dt;
        double drag = config.getAirDragPerInch();
        if (drag > 0.0) {
            double speed = StrictMath.sqrt(vx * vx + vy * vy + vz * vz);
            double scale = Math.max(0.0, 1.0 - drag * speed * dt);
            vx *= scale;
            vy *= scale;
            vz *= scale;
        }
        piece.setVelocityInchesPerSecond(vx, vy, vz);
        double spinScale = 1.0 / (1.0 + config.getSpinDampingPerSecond() * dt);
        piece.setSpinRadPerSecond(piece.getSpinXRadPerSecond() * spinScale, piece.getSpinYRadPerSecond() * spinScale,
                piece.getSpinZRadPerSecond() * spinScale);

        double travel = StrictMath.sqrt(vx * vx + vy * vy + vz * vz) * dt;
        boolean nearBody = false;
        for (KinematicBody body : bodies) {
            if (withinReach(body, piece, travel)) {
                nearBody = true;
                break;
            }
        }
        int steps = 1;
        if (nearBody && travel > config.getMaxContactTravelInches()) {
            steps = (int) Math.ceil(travel / config.getMaxContactTravelInches());
        }
        double h = dt / steps;
        for (int k = 0; k < steps; k++) {
            piece.setFieldPositionInches(piece.getFieldXInches() + piece.getVelocityXInchesPerSecond() * h,
                    piece.getFieldYInches() + piece.getVelocityYInchesPerSecond() * h,
                    piece.getFieldZInches() + piece.getVelocityZInchesPerSecond() * h);
            if (nearBody && resolveBodyContacts(piece)) {
                touched[index] = true;
            }
        }
    }

    /**
     * Push a piece out of every body shape it overlaps and resolve each contact's velocity: the part of the
     * piece's velocity into the surface, relative to the surface's own motion, is reversed and scaled by the
     * body's restitution - zero below the rest speed, so a piece settles rather than buzzing - and friction acts
     * on the sliding of the piece's contact point, spin included ({@link #applyFriction}).
     *
     * @return true when the piece touched any body
     */
    private boolean resolveBodyContacts(GamePiece piece) {
        boolean any = false;
        double radius = piece.getRadiusInches();
        for (int b = 0; b < bodies.size(); b++) {
            KinematicBody body = bodies.get(b);
            if (!withinReach(body, piece, 0.0)) {
                continue;
            }
            double angle = bodyAngleRad[b];
            double c = StrictMath.cos(angle);
            double s = StrictMath.sin(angle);
            List<ConvexShape> shapes = body.getShapes();
            for (int si = 0; si < shapes.size(); si++) {
                ConvexShape shape = shapes.get(si);
                body.toLocal(angle, piece.getFieldXInches(), piece.getFieldYInches(), piece.getFieldZInches(), local);
                if (!shape.contact(local[0], local[1], local[2], radius, contact)) {
                    continue;
                }
                any = true;
                body.rotate(c, s, contact[0], contact[1], contact[2], normal);
                if (normal[2] > 0.0) {
                    piece.setSupportBodyIndex(b);      // touching it from above: resting on it
                }
                double depth = contact[3];
                double px = piece.getFieldXInches() + normal[0] * depth;
                double py = piece.getFieldYInches() + normal[1] * depth;
                double pz = piece.getFieldZInches() + normal[2] * depth;
                piece.setFieldPositionInches(px, py, pz);
                body.surfaceVelocity(bodyAngularVelocityRadPerS[b], px - normal[0] * radius,
                        py - normal[1] * radius, pz - normal[2] * radius, surface);
                double intoSurface = (piece.getVelocityXInchesPerSecond() - surface[0]) * normal[0]
                        + (piece.getVelocityYInchesPerSecond() - surface[1]) * normal[1]
                        + (piece.getVelocityZInchesPerSecond() - surface[2]) * normal[2];
                if (intoSurface >= 0.0) {
                    continue;                      // already leaving the surface: no impulse, no friction
                }
                double e = -intoSurface > config.getRestSpeedInchesPerS() ? body.getShapeRestitution(si) : 0.0;
                double friction = body.getShapeFriction(si);
                double normalImpulse = -(1.0 + e) * intoSurface;
                if (isFreeHinge(b)) {
                    // Two bodies: the piece's mass against the body's inertia at this contact's lever arm.
                    double cx = px - normal[0] * radius;
                    double cy = py - normal[1] * radius;
                    double cz = pz - normal[2] * radius;
                    double mass = config.getPieceMass(piece.getKind());
                    double lever = leverAbout(body, cx, cy, cz, normal[0], normal[1], normal[2]);
                    normalImpulse /= 1.0 + mass * lever * lever / body.getHingeDynamics().getMomentOfInertia();
                    pushBody(b, mass, cx, cy, cz, normal[0] * normalImpulse, normal[1] * normalImpulse,
                            normal[2] * normalImpulse);
                    piece.setVelocityInchesPerSecond(piece.getVelocityXInchesPerSecond() + normal[0] * normalImpulse,
                            piece.getVelocityYInchesPerSecond() + normal[1] * normalImpulse,
                            piece.getVelocityZInchesPerSecond() + normal[2] * normalImpulse);
                    applyFrictionAgainstBody(piece, normal[0], normal[1], normal[2], normalImpulse, friction, b, cx, cy,
                            cz);
                    continue;
                }
                piece.setVelocityInchesPerSecond(piece.getVelocityXInchesPerSecond() + normal[0] * normalImpulse,
                        piece.getVelocityYInchesPerSecond() + normal[1] * normalImpulse,
                        piece.getVelocityZInchesPerSecond() + normal[2] * normalImpulse);
                applyFriction(piece, normal[0], normal[1], normal[2], normalImpulse, friction,
                        surface[0], surface[1], surface[2], 0.0);
            }
        }
        return any;
    }

    /**
     * Coulomb friction at one contact, per unit mass, written into {@link #frictionImpulse} and applied to the
     * piece. The piece touches along unit normal {@code n}, pointing from the other thing toward the piece, so
     * its contact point is {@code -r n} from its centre and moves at {@code v + spin x (-r n)}. A tangential
     * impulse {@code P} changes that point's sliding speed by {@code P (1 + 1/k)} for inertia factor {@code k}
     * (the {@code 1/k} is the spin it gains), plus {@code otherShare * P} from what the other thing does with
     * {@code -P}. The impulse that would stop the sliding is used, capped at {@code friction * normalImpulse}:
     * past that the point slides.
     *
     * @param otherPointVx the velocity of the other thing's surface at the contact, field frame, inches per second
     * @param otherShare   0 for an immovable surface; {@code (m / m_other)(1 + 1/k)} for another piece, which the
     *                     caller then gives {@code -P m / m_other}; a free dynamic body's share for one
     */
    private void applyFriction(GamePiece piece, double nx, double ny, double nz, double normalImpulse,
            double friction, double otherPointVx, double otherPointVy, double otherPointVz, double otherShare) {
        frictionImpulse[0] = 0.0;
        frictionImpulse[1] = 0.0;
        frictionImpulse[2] = 0.0;
        if (normalImpulse <= 0.0 || friction <= 0.0) {
            return;
        }
        double r = piece.getRadiusInches();
        double k = config.getPieceInertiaFactor();
        double wx = piece.getSpinXRadPerSecond();
        double wy = piece.getSpinYRadPerSecond();
        double wz = piece.getSpinZRadPerSecond();
        double ox = -r * nx;
        double oy = -r * ny;
        double oz = -r * nz;
        double cx = piece.getVelocityXInchesPerSecond() + (wy * oz - wz * oy) - otherPointVx;
        double cy = piece.getVelocityYInchesPerSecond() + (wz * ox - wx * oz) - otherPointVy;
        double cz = piece.getVelocityZInchesPerSecond() + (wx * oy - wy * ox) - otherPointVz;
        double along = cx * nx + cy * ny + cz * nz;
        double tx = cx - along * nx;
        double ty = cy - along * ny;
        double tz = cz - along * nz;
        double slip = StrictMath.sqrt(tx * tx + ty * ty + tz * tz);
        if (slip < 1e-12) {
            return;
        }
        double spinShare = r > 0.0 ? 1.0 / k : 0.0;
        double toStop = slip / (1.0 + spinShare + otherShare);
        double size = Math.min(toStop, friction * normalImpulse);
        double px = -tx / slip * size;
        double py = -ty / slip * size;
        double pz = -tz / slip * size;
        piece.setVelocityInchesPerSecond(piece.getVelocityXInchesPerSecond() + px,
                piece.getVelocityYInchesPerSecond() + py, piece.getVelocityZInchesPerSecond() + pz);
        if (r > 0.0) {
            addSpinFromImpulse(piece, nx, ny, nz, px, py, pz, -1.0);
        }
        frictionImpulse[0] = px;
        frictionImpulse[1] = py;
        frictionImpulse[2] = pz;
    }

    /**
     * {@link #applyFriction} against a free dynamic body: the body's surface at the contact gives way as well, by
     * {@code m (axis . (r x t))^2 / I} per unit of the piece's tangential velocity change along the sliding
     * direction {@code t}, and the body takes the equal and opposite angular impulse.
     *
     * @param friction the Coulomb friction coefficient of this contact
     * @param b        the body's index
     * @param contactX the contact point, field frame, inches
     * @param contactY ditto
     * @param contactZ ditto
     */
    private void applyFrictionAgainstBody(GamePiece piece, double nx, double ny, double nz, double normalImpulse,
            double friction, int b, double contactX, double contactY, double contactZ) {
        KinematicBody body = bodies.get(b);
        double mass = config.getPieceMass(piece.getKind());
        double inertia = body.getHingeDynamics().getMomentOfInertia();
        body.surfaceVelocity(bodyAngularVelocityRadPerS[b], contactX, contactY, contactZ, surface);
        // The sliding direction, as applyFriction will find it, so the body's share is known before the impulse.
        double r = piece.getRadiusInches();
        double wx = piece.getSpinXRadPerSecond();
        double wy = piece.getSpinYRadPerSecond();
        double wz = piece.getSpinZRadPerSecond();
        double ox = -r * nx;
        double oy = -r * ny;
        double oz = -r * nz;
        double sx = piece.getVelocityXInchesPerSecond() + (wy * oz - wz * oy) - surface[0];
        double sy = piece.getVelocityYInchesPerSecond() + (wz * ox - wx * oz) - surface[1];
        double sz = piece.getVelocityZInchesPerSecond() + (wx * oy - wy * ox) - surface[2];
        double along = sx * nx + sy * ny + sz * nz;
        double tx = sx - along * nx;
        double ty = sy - along * ny;
        double tz = sz - along * nz;
        double slip = StrictMath.sqrt(tx * tx + ty * ty + tz * tz);
        double share = 0.0;
        if (slip >= 1e-12) {
            double lever = leverAbout(body, contactX, contactY, contactZ, tx / slip, ty / slip, tz / slip);
            share = mass * lever * lever / inertia;
        }
        applyFriction(piece, nx, ny, nz, normalImpulse, friction, surface[0], surface[1], surface[2], share);
        if (frictionImpulse[0] != 0.0 || frictionImpulse[1] != 0.0 || frictionImpulse[2] != 0.0) {
            pushBody(b, mass, contactX, contactY, contactZ, frictionImpulse[0], frictionImpulse[1], frictionImpulse[2]);
        }
    }

    /**
     * The spin a tangential impulse {@code P} gives a piece through its contact point {@code side * r * n} from
     * its centre: {@code (side r n) x P / (k r^2) = side (n x P) / (k r)}.
     */
    private void addSpinFromImpulse(GamePiece piece, double nx, double ny, double nz, double px, double py,
            double pz, double side) {
        double f = side / (config.getPieceInertiaFactor() * piece.getRadiusInches());
        piece.setSpinRadPerSecond(piece.getSpinXRadPerSecond() + f * (ny * pz - nz * py),
                piece.getSpinYRadPerSecond() + f * (nz * px - nx * pz),
                piece.getSpinZRadPerSecond() + f * (nx * py - ny * px));
    }

    /**
     * Two pieces in the 3-D model touching: pushed apart along the line between their centres, the closing
     * speed along it reversed and scaled by {@link GamePiecePhysicsConfig#getPieceToPieceRestitution3d()} (zero
     * below the rest speed), and friction between their contact points. The push and the impulse are shared by
     * the two masses ({@link GamePiecePhysicsConfig#getPieceMass}): the lighter piece moves more.
     *
     * <p><b>A sleeping piece is a fixed surface</b> to an awake one, unless the awake one arrives faster than
     * {@link GamePiecePhysicsConfig#getWakeSpeedInchesPerS() the wake speed}, which wakes it. That is what lets a pile go to sleep from the bottom up: a piece settling
     * onto a sleeping one does not keep waking it with gravity's small push. A sleeping piece seated in a free
     * {@link HingeDynamics dynamic body} ({@link #carrierOf}) is part of that body, so what the awake one takes,
     * the body takes back at the contact point.
     *
     * <p>A piece on top of another (the normal from the lower one points up) rests on whatever the lower one rests
     * on ({@link #supportOf}), which is how a whole pile on a body is carried by it.
     *
     * @return true when the two touch
     */
    private boolean collidePieces3d(GamePiece a, GamePiece b) {
        if (a.isSleeping() && b.isSleeping()) {
            return false;
        }
        double dx = a.getFieldXInches() - b.getFieldXInches();
        double dy = a.getFieldYInches() - b.getFieldYInches();
        double dz = a.getFieldZInches() - b.getFieldZInches();
        double touching = a.getRadiusInches() + b.getRadiusInches();
        double distanceSquared = dx * dx + dy * dy + dz * dz;
        if (distanceSquared >= touching * touching) {
            return false;
        }
        double distance = StrictMath.sqrt(distanceSquared);
        // Unit normal from b toward a; straight up when the centres coincide, so the answer is not NaN.
        double nx = 0.0;
        double ny = 0.0;
        double nz = 1.0;
        if (distance > 0.0) {
            nx = dx / distance;
            ny = dy / distance;
            nz = dz / distance;
        }
        // Closing speed, positive when approaching.
        double closing = -((a.getVelocityXInchesPerSecond() - b.getVelocityXInchesPerSecond()) * nx
                + (a.getVelocityYInchesPerSecond() - b.getVelocityYInchesPerSecond()) * ny
                + (a.getVelocityZInchesPerSecond() - b.getVelocityZInchesPerSecond()) * nz);
        if (closing > config.getWakeSpeedInchesPerS()) {
            a.setSleeping(false);
            b.setSleeping(false);
        }
        if (nz > 0.0) {
            inheritSupport(a, b);                  // a is above b
        } else if (nz < 0.0) {
            inheritSupport(b, a);
        }
        double overlap = touching - distance;
        double e = closing > config.getRestSpeedInchesPerS() ? config.getPieceToPieceRestitution3d() : 0.0;
        double friction = config.getPieceToPieceFriction();
        if (a.isSleeping() || b.isSleeping()) {
            // One is a fixed surface: the awake one takes the whole push and the whole impulse.
            GamePiece moving = a.isSleeping() ? b : a;
            GamePiece still = a.isSleeping() ? a : b;
            double sign = moving == a ? 1.0 : -1.0;   // the normal must point toward the moving piece
            double mx = nx * sign;
            double my = ny * sign;
            double mz = nz * sign;
            moving.setFieldPositionInches(moving.getFieldXInches() + mx * overlap,
                    moving.getFieldYInches() + my * overlap, moving.getFieldZInches() + mz * overlap);
            if (closing > 0.0) {
                double impulse = (1.0 + e) * closing;
                moving.setVelocityInchesPerSecond(moving.getVelocityXInchesPerSecond() + mx * impulse,
                        moving.getVelocityYInchesPerSecond() + my * impulse,
                        moving.getVelocityZInchesPerSecond() + mz * impulse);
                applyFriction(moving, mx, my, mz, impulse, friction, still.getVelocityXInchesPerSecond(),
                        still.getVelocityYInchesPerSecond(), still.getVelocityZInchesPerSecond(), 0.0);
                int carrier = carrierOf(still);
                if (carrier >= 0) {
                    double r = still.getRadiusInches();
                    pushBody(carrier, config.getPieceMass(moving.getKind()), still.getFieldXInches() + mx * r,
                            still.getFieldYInches() + my * r, still.getFieldZInches() + mz * r,
                            mx * impulse + frictionImpulse[0], my * impulse + frictionImpulse[1],
                            mz * impulse + frictionImpulse[2]);
                }
            }
            return true;
        }
        // Each piece takes the share of the push and of the impulse that the OTHER's mass is of the two: equal
        // masses share both half and half (multiplying by 0.5 is exactly dividing by 2).
        double massA = config.getPieceMass(a.getKind());
        double massB = config.getPieceMass(b.getKind());
        double shareA = massB / (massA + massB);
        double shareB = massA / (massA + massB);
        a.setFieldPositionInches(a.getFieldXInches() + nx * overlap * shareA,
                a.getFieldYInches() + ny * overlap * shareA, a.getFieldZInches() + nz * overlap * shareA);
        b.setFieldPositionInches(b.getFieldXInches() - nx * overlap * shareB,
                b.getFieldYInches() - ny * overlap * shareB, b.getFieldZInches() - nz * overlap * shareB);
        if (closing <= 0.0) {
            return true;                           // touching but already separating
        }
        double impulseA = (1.0 + e) * closing * shareA;   // each one's velocity change: the momentum over its mass
        double impulseB = (1.0 + e) * closing * shareB;
        a.setVelocityInchesPerSecond(a.getVelocityXInchesPerSecond() + nx * impulseA,
                a.getVelocityYInchesPerSecond() + ny * impulseA, a.getVelocityZInchesPerSecond() + nz * impulseA);
        b.setVelocityInchesPerSecond(b.getVelocityXInchesPerSecond() - nx * impulseB,
                b.getVelocityYInchesPerSecond() - ny * impulseB, b.getVelocityZInchesPerSecond() - nz * impulseB);
        // b's contact point is +rb n from its centre; its velocity is what a's contact point slides against.
        double rb = b.getRadiusInches();
        double bwx = b.getSpinXRadPerSecond();
        double bwy = b.getSpinYRadPerSecond();
        double bwz = b.getSpinZRadPerSecond();
        double bpx = b.getVelocityXInchesPerSecond() + (bwy * nz - bwz * ny) * rb;
        double bpy = b.getVelocityYInchesPerSecond() + (bwz * nx - bwx * nz) * rb;
        double bpz = b.getVelocityZInchesPerSecond() + (bwx * ny - bwy * nx) * rb;
        double massRatio = massA / massB;          // b's velocity change per unit of a's, for one impulse
        double otherShare = massRatio * (1.0 + (rb > 0.0 ? 1.0 / config.getPieceInertiaFactor() : 0.0));
        applyFriction(a, nx, ny, nz, impulseA, friction, bpx, bpy, bpz, otherShare);
        double px = frictionImpulse[0] * massRatio;
        double py = frictionImpulse[1] * massRatio;
        double pz = frictionImpulse[2] * massRatio;
        if (px != 0.0 || py != 0.0 || pz != 0.0) {
            b.setVelocityInchesPerSecond(b.getVelocityXInchesPerSecond() - px, b.getVelocityYInchesPerSecond() - py,
                    b.getVelocityZInchesPerSecond() - pz);
            if (rb > 0.0) {
                // -P through b's contact point +rb n: (rb n) x (-P) / (k rb^2) = -(n x P) / (k rb).
                addSpinFromImpulse(b, nx, ny, nz, px, py, pz, -1.0);
            }
        }
        return true;
    }

    /** A piece resting on another rests on what that one rests on, when it rests on a body at all. */
    private void inheritSupport(GamePiece upper, GamePiece lower) {
        int support = supportOf(lower);
        if (support >= 0) {
            upper.setSupportBodyIndex(support);
        }
    }

    /** Whether a piece resting in a cell is still inside it (the whole box counts: no seat depth to stay in). */
    private boolean stillInSeatedCell(GamePiece piece) {
        String name = piece.getSeatedCellName();
        for (int b = 0; b < bodies.size(); b++) {
            KinematicBody body = bodies.get(b);
            for (BodyCell cell : body.getCells()) {
                if (cell.getName().equals(name)) {
                    body.toLocal(bodyAngleRad[b], piece.getFieldXInches(), piece.getFieldYInches(),
                            piece.getFieldZInches(), local);
                    return cell.contains(local[0], local[1], local[2], true);
                }
            }
        }
        return false;                              // the world no longer has that cell
    }

    /**
     * The cell a flying piece has come down into this sub-step, by the entry rule (centre inside the box and
     * at least the seat depth below its top), or null. The cell a piece was just released from does not count
     * until the piece's centre has been out of it, the same rule {@link #goalEntered} applies to a goal.
     */
    private BodyCell cellEntered(GamePiece piece) {
        String releasedFrom = piece.getReleasedFromGoalName();
        for (int b = 0; b < bodies.size(); b++) {
            KinematicBody body = bodies.get(b);
            if (body.getCells().isEmpty()) {
                continue;
            }
            body.toLocal(bodyAngleRad[b], piece.getFieldXInches(), piece.getFieldYInches(),
                    piece.getFieldZInches(), local);
            for (BodyCell cell : body.getCells()) {
                if (cell.getName().equals(releasedFrom)) {
                    if (!cell.contains(local[0], local[1], local[2], true)) {
                        piece.setReleasedFromGoalName(null);
                    }
                    continue;
                }
                if (cell.contains(local[0], local[1], local[2], false)) {
                    return cell;
                }
            }
        }
        return null;
    }

    /**
     * Count a piece's still time up while it is touching something and its centre moved slower than the sleep
     * speed over this sub-step, and put it to sleep once that has lasted the sleep time. Near a moving body
     * nothing sleeps.
     *
     * <p><b>Why displacement and not velocity.</b> In a pile resolved one contact at a time, a few passes do not
     * cancel gravity exactly: a resting piece is left with a small downward velocity that the next sub-step's
     * push-out undoes, so its velocity never reads zero although it does not move. Where it is, over a whole
     * sub-step, is the honest measure of still.
     */
    private void updateSleep(GamePiece piece, boolean touchedSomething, double startX, double startY,
            double startZ, double dt) {
        boolean nearMovingBody = false;
        for (int b = 0; b < bodies.size(); b++) {
            if (bodyMoving[b] && withinReach(bodies.get(b), piece, 0.0)) {
                nearMovingBody = true;
                break;
            }
        }
        double dx = piece.getFieldXInches() - startX;
        double dy = piece.getFieldYInches() - startY;
        double dz = piece.getFieldZInches() - startZ;
        double limit = config.getSleepSpeedInchesPerS() * dt;
        if (!touchedSomething || nearMovingBody || dx * dx + dy * dy + dz * dz >= limit * limit) {
            piece.setStillSeconds(0.0);
            return;
        }
        double still = piece.getStillSeconds() + dt;
        if (still >= config.getSleepSeconds()) {
            piece.setVelocityInchesPerSecond(0.0, 0.0, 0.0);
            piece.setSpinRadPerSecond(0.0, 0.0, 0.0);
            piece.setSleeping(true);
        } else {
            piece.setStillSeconds(still);
        }
    }

    // ------------------------------------------------------------------------------------- free motion

    private void integrateFlight(GamePiece piece, double dt) {
        double vx = piece.getVelocityXInchesPerSecond();
        double vy = piece.getVelocityYInchesPerSecond();
        double vz = piece.getVelocityZInchesPerSecond();

        // Gravity, then drag opposing the velocity (quadratic in speed, applied to each component).
        vz -= config.getGravityInchesPerS2() * dt;
        double drag = config.getAirDragPerInch();
        if (drag > 0.0) {
            double speed = StrictMath.sqrt(vx * vx + vy * vy + vz * vz);
            double scale = Math.max(0.0, 1.0 - drag * speed * dt);
            vx *= scale;
            vy *= scale;
            vz *= scale;
        }
        piece.setVelocityInchesPerSecond(vx, vy, vz);
        piece.setFieldPositionInches(piece.getFieldXInches() + vx * dt,
                piece.getFieldYInches() + vy * dt, piece.getFieldZInches() + vz * dt);
    }

    private void integrateRolling(GamePiece piece, double dt) {
        double vx = piece.getVelocityXInchesPerSecond();
        double vy = piece.getVelocityYInchesPerSecond();
        double speed = StrictMath.sqrt(vx * vx + vy * vy);
        if (speed > 0.0) {
            double slowerBy = config.getRollingDecelerationInchesPerS2() * dt;
            double newSpeed = speed - slowerBy;
            if (newSpeed <= config.getRestSpeedInchesPerS()) {
                vx = 0.0;
                vy = 0.0;
            } else {
                vx *= newSpeed / speed;
                vy *= newSpeed / speed;
            }
        }
        piece.setVelocityInchesPerSecond(vx, vy, 0.0);
        // A loose piece is always on the floor: z is pinned to its radius.
        piece.setFieldPositionInches(piece.getFieldXInches() + vx * dt,
                piece.getFieldYInches() + vy * dt, piece.getRadiusInches());
    }

    // ---------------------------------------------------------------------------------------- contacts

    /**
     * Circle-versus-rectangle in the robot's frame. If the piece overlaps the footprint it is moved out
     * along the contact normal, and if it was moving into the robot slower than the robot moves into it,
     * it takes the robot's speed along that normal (plus a little rebound).
     *
     * <p>The normal depends on where the piece's centre is. Beside a face (or inside the footprint) it is
     * that face's axis, and the move is along whichever axis needs the smaller one. Off a corner - past
     * the footprint along both axes - the ball can only be touching the corner itself, so it touches only
     * if its centre is within one radius of that corner, and the normal is the line from the corner to the
     * centre. Testing a corner like a face would treat the ball as a square and push it from up to
     * {@code 0.41 * r} too far away.
     */
    private void pushOutOfRobot(GamePiece piece, Robot robot) {
        double halfLength = world.getRobotLengthInches() / 2.0;
        double halfWidth = world.getRobotWidthInches() / 2.0;
        double radius = piece.getRadiusInches();

        double cos = StrictMath.cos(robot.headingRad);
        double sin = StrictMath.sin(robot.headingRad);
        double dx = piece.getFieldXInches() - robot.xInches;
        double dy = piece.getFieldYInches() - robot.yInches;
        double forward = dx * cos + dy * sin;      // piece along robot forward
        double left = -dx * sin + dy * cos;        // piece along robot left

        double normalForward;
        double normalLeft;
        double pushDistance;
        boolean offACorner = Math.abs(forward) > halfLength && Math.abs(left) > halfWidth;
        if (offACorner) {
            // From the nearest corner to the piece's centre, robot frame.
            double fromCornerForward = forward - Math.copySign(halfLength, forward);
            double fromCornerLeft = left - Math.copySign(halfWidth, left);
            double distanceToCorner = StrictMath.sqrt(fromCornerForward * fromCornerForward
                    + fromCornerLeft * fromCornerLeft);
            if (distanceToCorner >= radius) {
                return;                            // off the corner by more than a radius: not touching
            }
            normalForward = fromCornerForward / distanceToCorner;
            normalLeft = fromCornerLeft / distanceToCorner;
            pushDistance = radius - distanceToCorner;
        } else {
            double forwardOverlap = (halfLength + radius) - Math.abs(forward);
            double leftOverlap = (halfWidth + radius) - Math.abs(left);
            if (forwardOverlap <= 0.0 || leftOverlap <= 0.0) {
                return;                            // not touching
            }
            // Push along the axis with less overlap. The normal is the robot-frame axis, signed toward
            // the side the piece is on.
            if (forwardOverlap < leftOverlap) {
                normalForward = forward >= 0.0 ? 1.0 : -1.0;
                normalLeft = 0.0;
                pushDistance = forwardOverlap;
            } else {
                normalForward = 0.0;
                normalLeft = left >= 0.0 ? 1.0 : -1.0;
                pushDistance = leftOverlap;
            }
        }
        // Rotate the robot-frame normal back into the field frame.
        double normalX = normalForward * cos - normalLeft * sin;
        double normalY = normalForward * sin + normalLeft * cos;

        piece.setFieldPositionInches(piece.getFieldXInches() + normalX * pushDistance,
                piece.getFieldYInches() + normalY * pushDistance, piece.getRadiusInches());

        double pieceAlongNormal = piece.getVelocityXInchesPerSecond() * normalX
                + piece.getVelocityYInchesPerSecond() * normalY;
        double robotAlongNormal = robot.velocityXInchesPerS * normalX
                + robot.velocityYInchesPerS * normalY;
        if (pieceAlongNormal < robotAlongNormal) {
            double change = (robotAlongNormal - pieceAlongNormal)
                    * (1.0 + config.getPieceToRobotRestitution());
            piece.setVelocityInchesPerSecond(
                    piece.getVelocityXInchesPerSecond() + normalX * change,
                    piece.getVelocityYInchesPerSecond() + normalY * change, 0.0);
        }
    }

    /**
     * An in-flight piece against the robot's solid box — its footprint, from the floor up to
     * {@link FieldWorld#getRobotHeightInches()} — tested along the path the piece moved this sub-step
     * <b>relative to the robot</b>, so the face it is pushed back out of is the face it came in through.
     *
     * <p><b>Why swept, and why not the loose-piece rule.</b> {@link #pushOutOfRobot} pushes along the axis of
     * least overlap, which is right for a ball on the floor that the robot creeps into but wrong for a ball
     * that has flown a few inches into a moving robot between two ticks: once its centre is past the box's
     * middle on that axis, least overlap is the <i>far</i> side, and the ball is shoved out of the back of the
     * robot as if it had flown through it. Here the piece's path — its own motion minus the robot's, both in
     * the robot's frame — is traced back to where it entered the box, and that face is the contact.
     *
     * <p>The box is the robot's footprint and height grown by the piece's radius on every side (the
     * Minkowski sum of box and ball, taken square at the edges, so a ball passing within a radius of an edge
     * diagonally is caught up to {@code 0.41 r} early). The bottom is open: a piece cannot come up through the
     * floor. On contact the piece is put on the entered face, keeping its position along the face, and the
     * part of its velocity into that face, relative to the robot, is reversed and scaled by
     * {@link GamePiecePhysicsConfig#getPieceToRobotRestitution()}. On the top face that is a bounce off the
     * robot's roof, and the piece stays IN_FLIGHT: a piece that comes to rest on the roof stays up there, in
     * flight, until the robot drives out from under it.
     *
     * <p>A piece the robot just {@link GamePieceTracker#launch launched} is skipped until its centre is first
     * clear of the box, because a launcher's exit point is normally inside the robot.
     *
     * @param piece  an in-flight piece, already moved for this sub-step
     * @param robot  the robot this tick
     * @param startX the piece's field X at the start of the sub-step, inches
     * @param startY the piece's field Y at the start of the sub-step, inches
     * @param startZ the piece's height at the start of the sub-step, inches
     * @param dt     the sub-step, seconds
     */
    private void bounceInFlightOffRobot(GamePiece piece, Robot robot, double startX, double startY,
            double startZ, double dt) {
        double radius = piece.getRadiusInches();
        double halfLength = world.getRobotLengthInches() / 2.0 + radius;
        double halfWidth = world.getRobotWidthInches() / 2.0 + radius;
        double top = world.getRobotHeightInches() + radius;

        double cos = StrictMath.cos(robot.headingRad);
        double sin = StrictMath.sin(robot.headingRad);
        // Where the piece ended this sub-step, robot frame.
        double dx = piece.getFieldXInches() - robot.xInches;
        double dy = piece.getFieldYInches() - robot.yInches;
        double endForward = dx * cos + dy * sin;
        double endLeft = -dx * sin + dy * cos;
        double endZ = piece.getFieldZInches();
        boolean endInside = Math.abs(endForward) < halfLength && Math.abs(endLeft) < halfWidth && endZ < top;

        if (piece.isLeavingRobot()) {
            if (!endInside) {
                piece.setLeavingRobot(false);      // clear of the robot now: from here on it can hit it
            }
            return;
        }

        // How far the piece moved relative to the robot this sub-step, robot frame.
        double relativeX = piece.getFieldXInches() - startX - robot.velocityXInchesPerS * dt;
        double relativeY = piece.getFieldYInches() - startY - robot.velocityYInchesPerS * dt;
        double moveForward = relativeX * cos + relativeY * sin;
        double moveLeft = -relativeX * sin + relativeY * cos;
        double moveZ = endZ - startZ;
        double startForward = endForward - moveForward;
        double startLeft = endLeft - moveLeft;
        double startZRelative = endZ - moveZ;

        // Slab test on the line start + t * move: the latest entry over the three axes is where the line
        // enters the box, the earliest exit is where it leaves. t = 1 is the end of the sub-step.
        double tEnter = Double.NEGATIVE_INFINITY;
        double tExit = Double.POSITIVE_INFINITY;
        int enteredAxis = NO_AXIS;
        double enteredSign = 0.0;
        double[] near = new double[2];

        if (!slab(startForward, moveForward, halfLength, near)) {
            return;
        }
        if (near[0] > tEnter) {
            tEnter = near[0];
            enteredAxis = FORWARD_AXIS;
            enteredSign = moveForward > 0.0 ? -1.0 : 1.0;
        }
        tExit = Math.min(tExit, near[1]);

        if (!slab(startLeft, moveLeft, halfWidth, near)) {
            return;
        }
        if (near[0] > tEnter) {
            tEnter = near[0];
            enteredAxis = LEFT_AXIS;
            enteredSign = moveLeft > 0.0 ? -1.0 : 1.0;
        }
        tExit = Math.min(tExit, near[1]);

        // Vertically only the roof is a face: the box runs down through the floor.
        if (moveZ == 0.0) {
            if (startZRelative >= top) {
                return;
            }
        } else if (moveZ < 0.0) {
            double tTop = (top - startZRelative) / moveZ;
            if (tTop > tEnter) {
                tEnter = tTop;
                enteredAxis = TOP_AXIS;
                enteredSign = 1.0;
            }
        } else {
            tExit = Math.min(tExit, (top - startZRelative) / moveZ);
        }

        // Touching means the line meets the box (enter before exit) and either the sub-step ends inside it
        // or crosses it. A piece that started inside and has already left (tExit < 1, tEnter < 0) is on its
        // way out and is left alone.
        boolean meets = tEnter < tExit && tEnter <= 1.0;
        boolean endsInsideOrCrosses = tExit >= 1.0 || tEnter >= 0.0;
        if (!meets || !endsInsideOrCrosses || (!endInside && tEnter < 0.0)) {
            return;
        }
        if (enteredAxis == NO_AXIS) {
            // No relative motion at all and somehow inside: nothing to trace back, so leave by the nearest face.
            double forwardGap = halfLength - Math.abs(endForward);
            double leftGap = halfWidth - Math.abs(endLeft);
            double topGap = top - endZ;
            if (topGap <= forwardGap && topGap <= leftGap) {
                enteredAxis = TOP_AXIS;
                enteredSign = 1.0;
            } else if (forwardGap <= leftGap) {
                enteredAxis = FORWARD_AXIS;
                enteredSign = endForward >= 0.0 ? 1.0 : -1.0;
            } else {
                enteredAxis = LEFT_AXIS;
                enteredSign = endLeft >= 0.0 ? 1.0 : -1.0;
            }
        }

        // Put the piece on the face it entered by, keeping where it is along that face.
        double newForward = endForward;
        double newLeft = endLeft;
        double newZ = endZ;
        double normalForward = 0.0;
        double normalLeft = 0.0;
        double normalZ = 0.0;
        if (enteredAxis == FORWARD_AXIS) {
            newForward = enteredSign * halfLength;
            normalForward = enteredSign;
        } else if (enteredAxis == LEFT_AXIS) {
            newLeft = enteredSign * halfWidth;
            normalLeft = enteredSign;
        } else {
            newZ = top;
            normalZ = 1.0;
        }
        piece.setFieldPositionInches(robot.xInches + newForward * cos - newLeft * sin,
                robot.yInches + newForward * sin + newLeft * cos, newZ);

        // Reverse the part of the velocity, relative to the robot, that goes into the face.
        double normalX = normalForward * cos - normalLeft * sin;
        double normalY = normalForward * sin + normalLeft * cos;
        double vx = piece.getVelocityXInchesPerSecond();
        double vy = piece.getVelocityYInchesPerSecond();
        double vz = piece.getVelocityZInchesPerSecond();
        double intoFace = (vx - robot.velocityXInchesPerS) * normalX + (vy - robot.velocityYInchesPerS) * normalY
                + vz * normalZ;
        if (intoFace < 0.0) {
            double change = -(1.0 + config.getPieceToRobotRestitution()) * intoFace;
            piece.setVelocityInchesPerSecond(vx + normalX * change, vy + normalY * change, vz + normalZ * change);
        }
    }

    // ------------------------------------------------------------------- the robot as parts (RobotShape)

    /**
     * A loose piece against every part of the robot, one after another, in the shape's order: the part's point
     * nearest the ball's centre gives the contact ({@link #sphereContact}), and the ball is moved out sideways
     * and given the robot's speed along the contact the same way {@link #pushOutOfRobot} does. A ball on the
     * floor can only be pushed sideways, so only the horizontal part of the contact counts; a part whose
     * contact is almost straight down onto the ball (the ball has rolled under it) is left to the floor. That is
     * what keeps an intake mouth open: a part whose underside is above the ball's top never touches it.
     */
    private void pushOutOfRobotParts(GamePiece piece, Robot robot) {
        for (OrientedBox part : robot.parts) {
            double[] contact = sphereContact(part, piece.getFieldXInches(), piece.getFieldYInches(),
                    piece.getFieldZInches(), piece.getRadiusInches());
            if (contact == null) {
                continue;
            }
            double horizontal = StrictMath.sqrt(contact[0] * contact[0] + contact[1] * contact[1]);
            if (horizontal < MIN_HORIZONTAL_NORMAL) {
                continue;
            }
            double normalX = contact[0] / horizontal;
            double normalY = contact[1] / horizontal;
            double push = contact[3] / horizontal;
            piece.setFieldPositionInches(piece.getFieldXInches() + normalX * push,
                    piece.getFieldYInches() + normalY * push, piece.getRadiusInches());

            double pieceAlongNormal = piece.getVelocityXInchesPerSecond() * normalX
                    + piece.getVelocityYInchesPerSecond() * normalY;
            double robotAlongNormal = robot.velocityXInchesPerS * normalX + robot.velocityYInchesPerS * normalY;
            if (pieceAlongNormal < robotAlongNormal) {
                double change = (robotAlongNormal - pieceAlongNormal) * (1.0 + config.getPieceToRobotRestitution());
                piece.setVelocityInchesPerSecond(
                        piece.getVelocityXInchesPerSecond() + normalX * change,
                        piece.getVelocityYInchesPerSecond() + normalY * change, 0.0);
            }
        }
    }

    /**
     * An in-flight piece against the robot's parts: {@link #bounceInFlightOffRobot}'s swept test, done in each
     * part's own frame, with every face of the part solid. The first part the piece entered this sub-step is
     * the contact; one bounce per sub-step. A piece the robot just launched is skipped until its centre is clear
     * of every part (grown by its radius), because the launcher's exit point is inside the robot.
     */
    private void bounceInFlightOffRobotParts(GamePiece piece, Robot robot, double startX, double startY,
            double startZ, double dt) {
        double radius = piece.getRadiusInches();
        if (piece.isLeavingRobot()) {
            for (OrientedBox part : robot.parts) {
                if (insideGrown(part, piece.getFieldXInches(), piece.getFieldYInches(), piece.getFieldZInches(),
                        radius)) {
                    return;
                }
            }
            piece.setLeavingRobot(false);          // clear of the robot now: from here on it can hit it
            return;
        }
        // How far the piece moved relative to the robot this sub-step, field frame.
        double relativeX = piece.getFieldXInches() - startX - robot.velocityXInchesPerS * dt;
        double relativeY = piece.getFieldYInches() - startY - robot.velocityYInchesPerS * dt;
        double relativeZ = piece.getFieldZInches() - startZ;
        for (OrientedBox part : robot.parts) {
            if (sweptBounce(piece, part, relativeX, relativeY, relativeZ, robot.velocityXInchesPerS,
                    robot.velocityYInchesPerS)) {
                return;
            }
        }
    }

    /**
     * One part, in its own frame: trace the piece's relative path back to the face of the part (grown by the
     * radius) it came in through, put the piece on that face, and reverse its velocity into the face relative to
     * the robot, scaled by {@link GamePiecePhysicsConfig#getPieceToRobotRestitution()}.
     *
     * @return true when the piece touched this part
     */
    private boolean sweptBounce(GamePiece piece, OrientedBox part, double relativeX, double relativeY,
            double relativeZ, double robotVelocityX, double robotVelocityY) {
        double radius = piece.getRadiusInches();
        double[] end = part.toLocal(piece.getFieldXInches(), piece.getFieldYInches(), piece.getFieldZInches());
        double[] move = new double[3];
        double[] grown = new double[3];
        boolean endInside = true;
        for (int i = 0; i < 3; i++) {
            double[] axis = part.axis(i);
            move[i] = relativeX * axis[0] + relativeY * axis[1] + relativeZ * axis[2];
            grown[i] = part.halfExtent(i) + radius;
            endInside &= Math.abs(end[i]) < grown[i];
        }
        double tEnter = Double.NEGATIVE_INFINITY;
        double tExit = Double.POSITIVE_INFINITY;
        int enteredAxis = -1;
        double enteredSign = 0.0;
        double[] near = new double[2];
        for (int i = 0; i < 3; i++) {
            if (!slab(end[i] - move[i], move[i], grown[i], near)) {
                return false;
            }
            if (near[0] > tEnter) {
                tEnter = near[0];
                enteredAxis = i;
                enteredSign = move[i] > 0.0 ? -1.0 : 1.0;
            }
            tExit = Math.min(tExit, near[1]);
        }
        boolean meets = tEnter < tExit && tEnter <= 1.0;
        boolean endsInsideOrCrosses = tExit >= 1.0 || tEnter >= 0.0;
        if (!meets || !endsInsideOrCrosses || (!endInside && tEnter < 0.0)) {
            return false;
        }
        if (enteredAxis < 0) {
            // No relative motion at all and somehow inside: nothing to trace back, so leave by the nearest face.
            double smallest = Double.POSITIVE_INFINITY;
            for (int i = 0; i < 3; i++) {
                double gap = grown[i] - Math.abs(end[i]);
                if (gap < smallest) {
                    smallest = gap;
                    enteredAxis = i;
                    enteredSign = end[i] >= 0.0 ? 1.0 : -1.0;
                }
            }
        }
        end[enteredAxis] = enteredSign * grown[enteredAxis];
        double[] offset = part.fromLocalDirection(end[0], end[1], end[2]);
        piece.setFieldPositionInches(part.getCentreX() + offset[0], part.getCentreY() + offset[1],
                part.getCentreZ() + offset[2]);
        double[] normal = part.axis(enteredAxis);
        double normalX = normal[0] * enteredSign;
        double normalY = normal[1] * enteredSign;
        double normalZ = normal[2] * enteredSign;
        double vx = piece.getVelocityXInchesPerSecond();
        double vy = piece.getVelocityYInchesPerSecond();
        double vz = piece.getVelocityZInchesPerSecond();
        double intoFace = (vx - robotVelocityX) * normalX + (vy - robotVelocityY) * normalY + vz * normalZ;
        if (intoFace < 0.0) {
            double change = -(1.0 + config.getPieceToRobotRestitution()) * intoFace;
            piece.setVelocityInchesPerSecond(vx + normalX * change, vy + normalY * change, vz + normalZ * change);
        }
        return true;
    }

    /**
     * A piece against a robot that stands still ({@link StaticRobot}): the box's point nearest the ball gives the
     * contact. A loose piece is moved out sideways, like {@link #pushOutOfRobotParts} with the robot at rest; a
     * flying one is moved out along the full contact normal, so it can land on the robot's top and bounce off
     * it. Either way the velocity into the robot is reversed and scaled by the robot restitution.
     */
    private void bounceOffStaticRobot(GamePiece piece, OrientedBox standing) {
        double[] contact = sphereContact(standing, piece.getFieldXInches(), piece.getFieldYInches(),
                piece.getFieldZInches(), piece.getRadiusInches());
        if (contact == null) {
            return;
        }
        double normalX = contact[0];
        double normalY = contact[1];
        double normalZ = contact[2];
        double push = contact[3];
        if (!piece.isInFlight()) {
            double horizontal = StrictMath.sqrt(normalX * normalX + normalY * normalY);
            if (horizontal < MIN_HORIZONTAL_NORMAL) {
                return;
            }
            normalX /= horizontal;
            normalY /= horizontal;
            normalZ = 0.0;
            push /= horizontal;
        }
        piece.setFieldPositionInches(piece.getFieldXInches() + normalX * push,
                piece.getFieldYInches() + normalY * push, piece.getFieldZInches() + normalZ * push);
        double vx = piece.getVelocityXInchesPerSecond();
        double vy = piece.getVelocityYInchesPerSecond();
        double vz = piece.getVelocityZInchesPerSecond();
        double into = vx * normalX + vy * normalY + vz * normalZ;
        if (into < 0.0) {
            double change = -(1.0 + config.getPieceToRobotRestitution()) * into;
            piece.setVelocityInchesPerSecond(vx + normalX * change, vy + normalY * change, vz + normalZ * change);
        }
    }

    /**
     * A sphere against a box: {normal x, y, z (unit, out of the box toward the centre), how deep}, or null when
     * they do not touch. Outside the box the normal runs from the box's nearest point to the centre; with the
     * centre inside, it is the nearest face's, and the depth includes the way back out to that face.
     */
    private static double[] sphereContact(OrientedBox box, double x, double y, double z, double radius) {
        double[] local = box.toLocal(x, y, z);
        double[] fromSurface = new double[3];
        double distanceSquared = 0.0;
        for (int i = 0; i < 3; i++) {
            double clamped = Math.max(-box.halfExtent(i), Math.min(box.halfExtent(i), local[i]));
            fromSurface[i] = local[i] - clamped;
            distanceSquared += fromSurface[i] * fromSurface[i];
        }
        if (distanceSquared > 0.0) {
            double distance = StrictMath.sqrt(distanceSquared);
            if (distance >= radius) {
                return null;
            }
            double[] normal = box.fromLocalDirection(fromSurface[0] / distance, fromSurface[1] / distance,
                    fromSurface[2] / distance);
            return new double[] {normal[0], normal[1], normal[2], radius - distance};
        }
        int nearest = 0;
        double smallest = Double.POSITIVE_INFINITY;
        for (int i = 0; i < 3; i++) {
            double gap = box.halfExtent(i) - Math.abs(local[i]);
            if (gap < smallest) {
                smallest = gap;
                nearest = i;
            }
        }
        double[] direction = new double[3];
        direction[nearest] = local[nearest] >= 0.0 ? 1.0 : -1.0;
        double[] normal = box.fromLocalDirection(direction[0], direction[1], direction[2]);
        return new double[] {normal[0], normal[1], normal[2], smallest + radius};
    }

    /** {@return true when the point is inside the box grown by {@code radius} on every side (square corners)} */
    private static boolean insideGrown(OrientedBox box, double x, double y, double z, double radius) {
        double[] local = box.toLocal(x, y, z);
        for (int i = 0; i < 3; i++) {
            if (Math.abs(local[i]) >= box.halfExtent(i) + radius) {
                return false;
            }
        }
        return true;
    }

    /**
     * The smallest horizontal share of a contact normal that still pushes a ball on the floor sideways. Below it
     * the part is above (or below) the ball rather than beside it, and the floor, not the robot, carries the
     * ball: pushing it sideways by {@code depth / share} would throw it a long way for a graze. SET BY HAND: 0.2,
     * a normal within 78 degrees of vertical.
     */
    private static final double MIN_HORIZONTAL_NORMAL = 0.2;

    /** {@link #bounceInFlightOffRobot}'s record of which face a piece entered by. */
    private static final int NO_AXIS = 0;
    private static final int FORWARD_AXIS = 1;
    private static final int LEFT_AXIS = 2;
    private static final int TOP_AXIS = 3;

    /**
     * One axis of a slab test: the parameters {@code t} at which the line {@code start + t * move} enters
     * and leaves the slab {@code [-half, +half]}, written into {@code out[0]} and {@code out[1]}.
     *
     * @return false when the line never meets the slab (no motion on this axis and outside it)
     */
    private static boolean slab(double start, double move, double half, double[] out) {
        if (move == 0.0) {
            if (Math.abs(start) >= half) {
                return false;
            }
            out[0] = Double.NEGATIVE_INFINITY;
            out[1] = Double.POSITIVE_INFINITY;
            return true;
        }
        double t1 = (-half - start) / move;
        double t2 = (half - start) / move;
        out[0] = Math.min(t1, t2);
        out[1] = Math.max(t1, t2);
        return true;
    }

    /** Equal-mass sphere collision on the floor: separate the overlap half each, exchange normal velocity. */
    private void collidePieces(GamePiece a, GamePiece b) {
        double dx = b.getFieldXInches() - a.getFieldXInches();
        double dy = b.getFieldYInches() - a.getFieldYInches();
        double distance = StrictMath.sqrt(dx * dx + dy * dy);
        double touching = a.getRadiusInches() + b.getRadiusInches();
        if (distance >= touching) {
            return;
        }
        double normalX;
        double normalY;
        if (distance == 0.0) {
            // Exactly coincident centres (only possible if a scenario placed them so): pick +X so the
            // answer is still deterministic rather than NaN.
            normalX = 1.0;
            normalY = 0.0;
        } else {
            normalX = dx / distance;
            normalY = dy / distance;
        }
        double overlap = touching - distance;
        a.setFieldPositionInches(a.getFieldXInches() - normalX * overlap / 2.0,
                a.getFieldYInches() - normalY * overlap / 2.0, a.getRadiusInches());
        b.setFieldPositionInches(b.getFieldXInches() + normalX * overlap / 2.0,
                b.getFieldYInches() + normalY * overlap / 2.0, b.getRadiusInches());

        // Closing speed along the normal, positive when the two are approaching.
        double closing = (a.getVelocityXInchesPerSecond() - b.getVelocityXInchesPerSecond()) * normalX
                + (a.getVelocityYInchesPerSecond() - b.getVelocityYInchesPerSecond()) * normalY;
        if (closing <= 0.0) {
            return;                                // already separating
        }
        double impulse = (1.0 + config.getPieceToPieceRestitution()) * closing / 2.0;
        a.setVelocityInchesPerSecond(a.getVelocityXInchesPerSecond() - normalX * impulse,
                a.getVelocityYInchesPerSecond() - normalY * impulse, 0.0);
        b.setVelocityInchesPerSecond(b.getVelocityXInchesPerSecond() + normalX * impulse,
                b.getVelocityYInchesPerSecond() + normalY * impulse, 0.0);
    }

    private void bounceOffPerimeter(GamePiece piece, boolean inThreeD) {
        if (!world.isBounded()) {
            return;
        }
        double e = inThreeD ? config.getPieceToWallRestitution3d() : config.getPieceToWallRestitution();
        double r = piece.getRadiusInches();
        double x = piece.getFieldXInches();
        double y = piece.getFieldYInches();
        double vx = piece.getVelocityXInchesPerSecond();
        double vy = piece.getVelocityYInchesPerSecond();
        double halfWidth = world.getHalfWidthInches();
        double halfLength = world.getHalfLengthInches();

        if (x - r < -halfWidth) {
            x = -halfWidth + r;
            if (vx < 0.0) {
                vx = -vx * e;
            }
        } else if (x + r > halfWidth) {
            x = halfWidth - r;
            if (vx > 0.0) {
                vx = -vx * e;
            }
        }
        if (y - r < -halfLength) {
            y = -halfLength + r;
            if (vy < 0.0) {
                vy = -vy * e;
            }
        } else if (y + r > halfLength) {
            y = halfLength - r;
            if (vy > 0.0) {
                vy = -vy * e;
            }
        }
        piece.setFieldPositionInches(x, y, piece.getFieldZInches());
        piece.setVelocityInchesPerSecond(vx, vy, piece.getVelocityZInchesPerSecond());
    }

    /**
     * A piece against a fixed box, in plan view: the same face-or-corner rule as
     * {@link #pushOutOfRobot}, with the obstacle immovable. A piece entirely above or below the box's
     * height range is left alone, which is how a shot flies over a low bar.
     */
    private void bounceOffObstacle(GamePiece piece, ObstacleBox box, boolean inThreeD) {
        double r = piece.getRadiusInches();
        double z = piece.getFieldZInches();
        if (z - r >= box.getMaxZInches() || z + r <= box.getMinZInches()) {
            return;                                // clear above or below the box
        }
        double x = piece.getFieldXInches();
        double y = piece.getFieldYInches();
        double e = inThreeD ? config.getPieceToObstacleRestitution3d() : config.getPieceToObstacleRestitution();
        double vx = piece.getVelocityXInchesPerSecond();
        double vy = piece.getVelocityYInchesPerSecond();
        double centreX = (box.getMinXInches() + box.getMaxXInches()) / 2.0;
        double centreY = (box.getMinYInches() + box.getMaxYInches()) / 2.0;
        double halfX = (box.getMaxXInches() - box.getMinXInches()) / 2.0;
        double halfY = (box.getMaxYInches() - box.getMinYInches()) / 2.0;

        boolean offACorner = Math.abs(x - centreX) > halfX && Math.abs(y - centreY) > halfY;
        if (offACorner) {
            // From the nearest corner to the piece's centre; touching only within one radius of it.
            double fromCornerX = x - (centreX + Math.copySign(halfX, x - centreX));
            double fromCornerY = y - (centreY + Math.copySign(halfY, y - centreY));
            double distanceToCorner = StrictMath.sqrt(fromCornerX * fromCornerX + fromCornerY * fromCornerY);
            if (distanceToCorner >= r) {
                return;
            }
            double normalX = fromCornerX / distanceToCorner;
            double normalY = fromCornerY / distanceToCorner;
            double push = r - distanceToCorner;
            x += normalX * push;
            y += normalY * push;
            double alongNormal = vx * normalX + vy * normalY;
            if (alongNormal < 0.0) {               // moving into the corner: reflect that part, scaled by e
                vx -= (1.0 + e) * alongNormal * normalX;
                vy -= (1.0 + e) * alongNormal * normalY;
            }
            piece.setFieldPositionInches(x, y, z);
            piece.setVelocityInchesPerSecond(vx, vy, piece.getVelocityZInchesPerSecond());
            return;
        }

        // Beside a face (or inside the box): the overlap along each axis measured from the nearest face
        // the centre projects onto, and the push along the axis with less of it.
        double overlapX = halfX + r - Math.abs(x - centreX);
        double overlapY = halfY + r - Math.abs(y - centreY);
        if (overlapX <= 0.0 || overlapY <= 0.0) {
            return;
        }
        if (overlapX < overlapY) {
            double sign = x >= centreX ? 1.0 : -1.0;
            x += sign * overlapX;
            if (vx * sign < 0.0) {
                vx = -vx * e;
            }
        } else {
            double sign = y >= centreY ? 1.0 : -1.0;
            y += sign * overlapY;
            if (vy * sign < 0.0) {
                vy = -vy * e;
            }
        }
        piece.setFieldPositionInches(x, y, z);
        piece.setVelocityInchesPerSecond(vx, vy, piece.getVelocityZInchesPerSecond());
    }

    // ------------------------------------------------------------------------------------ flight ends

    /**
     * The goal a flying piece entered during this sub-step, or null. The piece's centre moved from
     * {@code (startX, startY, startZ)} to where it is now; a goal whose box that path touches, entered the
     * right way, is the one (the first in the world's order, if two).
     *
     * <p><b>The goal a piece was just released from does not count</b> until the piece's centre has been
     * outside that goal's box at the end of a sub-step. A goal that empties itself can let a piece go from
     * a point inside its own box, and without this the piece would score straight back into it on the next
     * sub-step: counted twice, and logged SCORED -> IN_FLIGHT -> SCORED for a piece that never left.
     */
    private ScoringVolume goalEntered(GamePiece piece, double startX, double startY, double startZ) {
        double x = piece.getFieldXInches();
        double y = piece.getFieldYInches();
        double z = piece.getFieldZInches();
        for (ScoringVolume volume : world.getScoringVolumes()) {
            if (volume.getName().equals(piece.getReleasedFromGoalName())) {
                if (!volume.containsCentre(x, y, z)) {
                    piece.setReleasedFromGoalName(null);   // out of it now: it may score there again later
                }
                continue;
            }
            if (volume.acceptsPath(startX, startY, startZ, x, y, z, piece.getVelocityXInchesPerSecond(),
                    piece.getVelocityYInchesPerSecond(), piece.getVelocityZInchesPerSecond())) {
                return volume;
            }
        }
        return null;
    }

    /**
     * Floor contact for a flying piece. Bounces with the floor restitution, keeping
     * {@link GamePiecePhysicsConfig#getPieceToFloorTangentialRetention()} of its horizontal velocity - or, in the
     * 3-D model with {@link GamePiecePhysicsConfig#hasPieceToFloorFriction3d() floor friction} set, whatever the
     * floor's Coulomb friction on its spinning contact point leaves it ({@link #landedWithFriction}); once the
     * rebound would be slower than the rest speed the piece is set down rolling with that reduced horizontal
     * velocity, and the caller logs IN_FLIGHT to LOOSE.
     *
     * @param threeD true in a world with bodies, where every flying piece is in the 3-D model
     * @return true when the piece has settled onto the floor this step
     */
    private boolean landed(GamePiece piece, boolean threeD) {
        double r = piece.getRadiusInches();
        if (piece.getFieldZInches() - r > 0.0 || piece.getVelocityZInchesPerSecond() > 0.0) {
            return false;
        }
        if (threeD && config.hasPieceToFloorFriction3d()) {
            return landedWithFriction(piece);
        }
        double rebound = -piece.getVelocityZInchesPerSecond() * config.getPieceToFloorRestitution();
        // Floor friction on the impact: the piece keeps this share of its horizontal speed (1.0 keeps it all).
        double retention = config.getPieceToFloorTangentialRetention();
        double vx = piece.getVelocityXInchesPerSecond() * retention;
        double vy = piece.getVelocityYInchesPerSecond() * retention;
        piece.setFieldPositionInches(piece.getFieldXInches(), piece.getFieldYInches(), r);
        if (rebound <= config.getRestSpeedInchesPerS()) {
            piece.setVelocityInchesPerSecond(vx, vy, 0.0);
            return true;
        }
        piece.setVelocityInchesPerSecond(vx, vy, rebound);
        return false;
    }

    /**
     * {@link #landed} in the 3-D model with floor friction: the normal impulse is the arrival speed times
     * {@code 1 + e} (just the arrival speed when the rebound would be under the rest speed), and the floor's grip
     * acts on the contact point's sliding with that impulse, spinning the piece, as a body's surface does.
     */
    private boolean landedWithFriction(GamePiece piece) {
        double arrival = -piece.getVelocityZInchesPerSecond();
        double rebound = arrival * config.getPieceToFloorRestitution();
        boolean settles = rebound <= config.getRestSpeedInchesPerS();
        double normalImpulse = settles ? arrival : arrival + rebound;
        piece.setFieldPositionInches(piece.getFieldXInches(), piece.getFieldYInches(), piece.getRadiusInches());
        piece.setVelocityInchesPerSecond(piece.getVelocityXInchesPerSecond(), piece.getVelocityYInchesPerSecond(),
                settles ? 0.0 : rebound);
        applyFriction(piece, 0.0, 0.0, 1.0, normalImpulse, config.getPieceToFloorFriction3d(), 0.0, 0.0, 0.0, 0.0);
        return settles;
    }
}
