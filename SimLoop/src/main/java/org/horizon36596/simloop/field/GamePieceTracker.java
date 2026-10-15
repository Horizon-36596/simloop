package org.horizon36596.simloop.field;

import org.psilynx.psikit.core.Logger;
import org.psilynx.psikit.core.wpi.math.Translation3d;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

/**
 * The one place a game piece changes hands or moves. Holds every piece in the sim, decides when a loose
 * piece becomes held, enforces how many the robot can carry, launches held pieces into a ballistic arc,
 * advances the physics of everything that is not held, puts ejected pieces back where the caller says, and
 * logs every one of those transitions so a replay shows the same sequence (domain R5).
 *
 * <p><b>What this models, and what it refuses to.</b> Since 2026-09-20 (a human decision reversing the
 * 2026-09-03 "possession events only" ruling for game pieces) a piece is a sphere with a position,
 * velocity and state. Loose pieces roll, are pushed by the robot's footprint, and bounce off walls,
 * obstacles and each other; launched pieces fly a gravity arc into a goal or onto the floor. The
 * <em>robot</em> is still not rigid-body: it is an immovable rectangle to the pieces, and its mechanisms are
 * first-order plants. Acquisition is still a "teleport" — a running mechanism with a piece in its mouth
 * and room to carry it takes the piece, with no roller-ball contact simulated. The season layer decides
 * where a launched piece leaves the robot and how fast, because that depends on its own mechanisms.
 *
 * <p><b>Moving goals.</b> A world may hold {@link KinematicBody kinematic bodies}: convex solids that turn
 * about a fixed axis at an angle and rate the season sets each tick ({@link #setBodyMotion}). Flying pieces
 * and pieces seated in a body's {@link BodyCell cells} then run a 3-D contact model against the bodies and
 * each other, and a cell's contents are whatever is physically inside it: a piece that settles in is SCORED
 * there, and one that rolls out goes back IN_FLIGHT. A body made {@linkplain KinematicBody#withHinge dynamic}
 * turns by itself instead, from gravity and the pieces' pushes, between hard limits; the season can only hold it
 * still ({@link #setBodyHeld}) or put it somewhere.
 *
 * <p><b>The loop, once per sim tick.</b> Five calls make up a tick; the first, the last two and the order
 * are required, and the middle two happen whenever the robot does them:
 * <pre>
 *   tracker.updateRobotPose(fieldX, fieldY, fieldHeading);  // top: carry held pieces along
 *   tracker.tryAcquire(intakeVolume, intake.isRunning());   // whenever a mechanism could pick up
 *   ... launch(...) / eject(...) as the robot releases pieces ...
 *   tracker.advance(deltaTime);                             // physics for everything not held
 *   tracker.record("GamePieces");                           // bottom: flush the log
 * </pre>
 * {@link #record(String)} is what writes the accumulated transitions into the RLOG and clears them, so a
 * tick without it would carry its transitions into the next tick's log entry. Skipping it entirely is a
 * caller bug this class refuses to let slide: it throws once the unlogged backlog is implausibly large,
 * rather than growing forever and quietly losing the replay record.
 *
 * <p><b>Frame and units.</b> Every position in and out of this class is in the repo's FTC-Cartesian field
 * frame — +X right, +Y forward, origin at the centre of the field, +Z up from the floor — in inches, and
 * headings are radians CCW-positive (conventions §7). Velocities are inches per second. Where the field
 * ends is the {@link FieldWorld}'s business: a tracker built without one lets pieces roll forever.
 *
 * <p><b>Determinism (domain R5).</b> There is no clock in this class at all. Pieces are scanned in id
 * order, so a mouth over two pieces with only one slot left always takes the lower-numbered one; the
 * transition log is built from ids, state names and cause names with no formatted numbers in it, so no
 * locale can change a logged string; and every bit of trigonometry is {@link StrictMath}. Two identical
 * runs produce identical logs.
 *
 * <p><b>Season-agnostic (domain R7).</b> Nothing here names a game, a piece type, or a subsystem. The
 * piece kinds, sizes, the carrying capacity and the world geometry are what the season layer supplies.
 */
public final class GamePieceTracker {

    /**
     * How many transitions may pile up unlogged before {@link #record(String)} being missing is treated as
     * a defect. A real tick produces at most a handful (one per piece acquired, launched, landed or
     * scored), so reaching this many means nobody is calling {@code record} and the replay record is being
     * lost.
     *
     * <p><b>A backstop, not a per-tick check.</b> The tracker has no notion of a tick — that is what keeps
     * it clock-free (domain R5) — so it cannot tell "record was not called this tick" from "nothing
     * changed hands this tick". A short scenario that produces only a few transitions and never logs them
     * will therefore finish without tripping this. What catches that case is a test asserting the backlog
     * is empty at the end of a run; this constant only stops a long run from growing the list forever.
     */
    private static final int MAX_UNLOGGED_TRANSITIONS = 256;

    /**
     * Inches to metres, applied only where a WPI geometry struct is handed to PsiKit.
     *
     * <p>AdvantageScope's 2-D and 3-D Field tabs draw in the field's own coordinate space, which is
     * <b>metres</b> — the same space a robot pose is logged in for the same reason (this module's
     * {@code MechanismSketch} converts at its log boundary too). A geometry struct handed over in inches
     * renders about 39x too far out, i.e. off the field entirely, with no error anywhere.
     *
     * <p>So this class's whole API stays in inches, and the conversion happens here at the last possible
     * moment and nowhere else.
     */
    private static final double METRES_PER_INCH = 0.0254;

    /** The kind a piece gets when the caller does not name one. */
    private static final String DEFAULT_KIND = "piece";

    private final int heldCapacity;
    // The world and its physics are replaced together by replaceWorld(), which is how a season models a
    // goal that moves during a match (a tipping goal that swaps sides). Everything else about the tracker
    // survives a swap.
    private FieldWorld world;
    private GamePiecePhysics physics;
    private final List<GamePiece> pieces = new ArrayList<>();
    private final List<String> unloggedTransitions = new ArrayList<>();
    private final List<GamePiecePhysics.Event> pendingEvents = new ArrayList<>();
    // Which goal each parked piece is in, by piece id - the one record of what is in which goal; the
    // per-goal counts and the parking slots are both worked out from it. A LinkedHashMap, and a piece's
    // entry is added when it parks and removed when it leaves, so iterating it gives park order: bottom of
    // a stack first. A piece scored with the label-only score(int, String) is in no goal and has no entry.
    private final Map<Integer, String> scoredVolumeByPieceId = new LinkedHashMap<>();
    // Every kind ever added, in the order first added. record() writes one positions key per kind in this
    // set on every tick - an empty array when none of that kind is visible - so a kind whose pieces are all
    // held stops being drawn instead of leaving its last positions on screen.
    private final Set<String> kindsEverAdded = new LinkedHashSet<>();

    private double robotXInches;
    private double robotYInches;
    private double robotHeadingRad;
    private boolean robotPoseKnown;

    // The robot's pose at the previous advance(), for its velocity. The physics needs how fast the robot is
    // moving to give a pushed ball the robot's speed; the tracker has no clock, so it takes the difference
    // over the deltaTime the caller hands to advance().
    private double lastAdvancedRobotXInches;
    private double lastAdvancedRobotYInches;
    private boolean advancedBefore;

    // The robot's solid parts for the next advance(), robot frame, or null for the world's one footprint box;
    // and the robots standing still on the field. Both belong to the tracker, not the world, so a world swap
    // (replaceWorld) keeps them.
    private RobotShape robotShape;
    private List<StaticRobot> staticRobots = Collections.emptyList();

    /**
     * Builds a tracker over an {@link FieldWorld#unbounded() unbounded world}: no walls, no obstacles,
     * no goals, no robot footprint, default physics. Pieces still roll, fly and collide with each other.
     * This is the constructor every scenario written before the physics existed uses, and it behaves the
     * same for them because a piece that is never launched and never given a velocity never moves.
     *
     * @param heldCapacity the most pieces the robot may hold at once — the season's number (domain R7).
     *                     {@link #tryAcquire} will not take a piece that would push the held count past
     *                     it, which is the sim's stand-in for a full intake refusing another ball.
     * @throws IllegalArgumentException if {@code heldCapacity} is negative. Zero is allowed and means a
     *                                  robot that cannot carry anything, which is a legitimate scenario
     *                                  (drive-only tests) rather than a mistake.
     */
    public GamePieceTracker(int heldCapacity) {
        this(heldCapacity, FieldWorld.unbounded());
    }

    /**
     * Builds a tracker over the given world.
     *
     * @param heldCapacity as for {@link #GamePieceTracker(int)}
     * @param world        the field's fixed geometry and physics numbers, from the season layer
     * @throws IllegalArgumentException if {@code heldCapacity} is negative or {@code world} is null
     */
    public GamePieceTracker(int heldCapacity, FieldWorld world) {
        if (heldCapacity < 0) {
            throw new IllegalArgumentException("heldCapacity must be >= 0, got " + heldCapacity);
        }
        if (world == null) {
            throw new IllegalArgumentException("world must not be null; use FieldWorld.unbounded()");
        }
        this.heldCapacity = heldCapacity;
        this.world = world;
        this.physics = new GamePiecePhysics(world);
    }

    /**
     * Put a new piece of the default kind on the field, {@link PossessionState#LOOSE}, and return its id.
     * Same as {@link #addLoosePiece(String, double, double, double)} with kind {@code "piece"}.
     *
     * @param fieldXInches field position, inches (+X right)
     * @param fieldYInches field position, inches (+Y forward)
     * @param radiusInches the piece's radius, inches; 0 or more
     * @return the new piece's id
     */
    public int addLoosePiece(double fieldXInches, double fieldYInches, double radiusInches) {
        return addLoosePiece(DEFAULT_KIND, fieldXInches, fieldYInches, radiusInches);
    }

    /**
     * Put a new piece on the field at rest on the floor, {@link PossessionState#LOOSE}, and return its id.
     *
     * <p>Ids are handed out in call order starting at {@code 0}, and acquisition scans in that order, so
     * the order pieces are added is part of what makes a scenario reproducible (domain R5).
     *
     * @param kind          the season's label for the piece, e.g. {@code "ball"}; non-blank
     * @param fieldXInches  field position, inches (+X right)
     * @param fieldYInches  field position, inches (+Y forward)
     * @param radiusInches  the piece's radius, inches — half its diameter (1.4 for a 2.8 in ball). May be
     *                      0 for a point-sized piece, in which case only its centre can trigger a volume.
     * @return the new piece's id
     * @throws IllegalArgumentException if the kind is blank, any number is non-finite, or the radius is
     *                                  negative
     */
    public int addLoosePiece(String kind, double fieldXInches, double fieldYInches, double radiusInches) {
        String cleanKind = requireKind(kind);
        requireFinite(fieldXInches, "fieldXInches");
        requireFinite(fieldYInches, "fieldYInches");
        requireRadius(radiusInches);
        int id = pieces.size();
        pieces.add(new GamePiece(id, cleanKind, fieldXInches, fieldYInches, radiusInches,
                PossessionState.LOOSE));
        kindsEverAdded.add(cleanKind);
        return id;
    }

    /**
     * Put a new piece straight into the robot, {@link PossessionState#HELD}, and return its id — a preload.
     * Counts against the capacity like any other held piece.
     *
     * @param kind         the season's label for the piece; non-blank
     * @param radiusInches the piece's radius, inches; 0 or more
     * @return the new piece's id
     * @throws IllegalStateException if the robot is already at capacity
     */
    public int addHeldPiece(String kind, double radiusInches) {
        String cleanKind = requireKind(kind);
        requireRadius(radiusInches);
        if (getHeldCount() >= heldCapacity) {
            throw new IllegalStateException("cannot preload a piece: the robot already holds "
                    + getHeldCount() + " of " + heldCapacity);
        }
        int id = pieces.size();
        GamePiece piece = new GamePiece(id, cleanKind, robotXInches, robotYInches, radiusInches,
                PossessionState.HELD);
        pieces.add(piece);
        kindsEverAdded.add(cleanKind);
        return id;
    }

    /**
     * Put a new piece straight into a goal, {@link PossessionState#SCORED}, parked at that goal's next
     * resting slot — the pieces a match starts with already in the goal.
     *
     * @param kind         the season's label for the piece
     * @param radiusInches the piece's radius, inches
     * @param volumeName   which of the world's scoring volumes it starts in
     * @return the new piece's id
     * @throws IllegalArgumentException if the world has no scoring volume of that name
     */
    public int addScoredPiece(String kind, double radiusInches, String volumeName) {
        String cleanKind = requireKind(kind);
        requireRadius(radiusInches);
        ScoringVolume volume = world.getScoringVolume(volumeName);
        int id = pieces.size();
        GamePiece piece = new GamePiece(id, cleanKind, 0.0, 0.0, radiusInches, PossessionState.SCORED);
        pieces.add(piece);
        kindsEverAdded.add(cleanKind);
        parkInVolume(piece, volume);
        return id;
    }

    /**
     * Put a new piece at a point in space and let the physics have it - how a match's starting pieces are set
     * into a {@link KinematicBody}'s cell, where they then settle under gravity rather than being parked.
     *
     * <ul>
     *   <li>A centre inside one of the world's {@link BodyCell cells} (deep enough by its entry rule) gives a
     *       piece {@link PossessionState#SCORED} in that cell, resting there.</li>
     *   <li>Otherwise a centre at or below its own radius gives a {@link PossessionState#LOOSE} piece on the
     *       floor at that spot, as {@link #addLoosePiece} would.</li>
     *   <li>Otherwise it is {@link PossessionState#IN_FLIGHT} at rest, and falls.</li>
     * </ul>
     * Like the other {@code add} methods it logs no transition: the piece starts in its state.
     *
     * @param kind         the season's label for the piece
     * @param fieldXInches the piece's centre, field frame, inches
     * @param fieldYInches ditto
     * @param fieldZInches ditto, height above the floor; at least the radius
     * @param radiusInches the piece's radius, inches
     * @return the new piece's id
     * @throws IllegalArgumentException if the kind is blank, a number is non-finite, the radius is negative or
     *                                  the centre is below its own radius
     */
    public int addPieceAt(String kind, double fieldXInches, double fieldYInches, double fieldZInches,
            double radiusInches) {
        String cleanKind = requireKind(kind);
        requireFinite(fieldXInches, "fieldXInches");
        requireFinite(fieldYInches, "fieldYInches");
        requireFinite(fieldZInches, "fieldZInches");
        requireRadius(radiusInches);
        if (fieldZInches < radiusInches) {
            throw new IllegalArgumentException("a piece's centre must be at least its radius above the floor, got z "
                    + fieldZInches + " for radius " + radiusInches);
        }
        BodyCell cell = physics.cellAt(fieldXInches, fieldYInches, fieldZInches);
        PossessionState state;
        if (cell != null) {
            state = PossessionState.SCORED;
        } else if (fieldZInches <= radiusInches) {
            state = PossessionState.LOOSE;
        } else {
            state = PossessionState.IN_FLIGHT;
        }
        int id = pieces.size();
        GamePiece piece = new GamePiece(id, cleanKind, fieldXInches, fieldYInches, radiusInches, state);
        piece.setFieldPositionInches(fieldXInches, fieldYInches, state == PossessionState.LOOSE ? radiusInches
                : fieldZInches);
        if (cell != null) {
            piece.setSeatedCellName(cell.getName());
        }
        pieces.add(piece);
        kindsEverAdded.add(cleanKind);
        return id;
    }

    /**
     * Set where one of the world's {@link KinematicBody bodies} is and how fast it is turning, as the season's
     * model of it says. Call it every tick a body moves, before {@link #advance}; within the tick the body turns
     * on at {@code angularVelocityRadPerS}, so the next call should agree with that. A body never set sits at
     * angle zero. Pieces asleep on a body that moves are woken.
     *
     * <p>A <b>dynamic</b> body ({@link KinematicBody#withHinge}) moves itself: this puts it somewhere, at a rate,
     * and the physics carries on from there. Set it once where it starts, and again only to override it.
     *
     * @param bodyName               one of the world's bodies
     * @param angleRad               its angle now, radians, right-handed about its axis
     * @param angularVelocityRadPerS how fast it is turning, radians per second; 0 when still
     * @throws IllegalArgumentException if the world has no body of that name, or a number is non-finite
     */
    public void setBodyMotion(String bodyName, double angleRad, double angularVelocityRadPerS) {
        requireFinite(angleRad, "angleRad");
        requireFinite(angularVelocityRadPerS, "angularVelocityRadPerS");
        physics.setBodyMotion(bodyIndex(bodyName), angleRad, angularVelocityRadPerS);
    }

    /**
     * {@return a body's angle now, radians} After an {@link #advance} it has turned on by the rate times the
     * tick since the last {@link #setBodyMotion}.
     *
     * @param bodyName one of the world's bodies
     * @throws IllegalArgumentException if the world has no body of that name
     */
    public double getBodyAngleRad(String bodyName) {
        return physics.getBodyAngleRad(bodyIndex(bodyName));
    }

    /**
     * {@return a body's turning rate, radians per second: as last set for a kinematic body, as the physics has it
     * for a dynamic one}
     *
     * @param bodyName one of the world's bodies
     * @throws IllegalArgumentException if the world has no body of that name
     */
    public double getBodyAngularVelocityRadPerS(String bodyName) {
        return physics.getBodyAngularVelocityRadPerS(bodyIndex(bodyName));
    }

    /**
     * Hold a {@link KinematicBody#isDynamic() dynamic} body still where it is, or let it go: a latch, a detent, a
     * pin a rule lifts. Holding it stops it (its rate becomes 0), and while held it is kinematic again: it takes no
     * push from the pieces and moves only as {@link #setBodyMotion} says, so a season can also drive a dynamic body
     * along a profile of its own by holding it. Let go, the physics moves it on from where it is, at the rate it
     * has. A body starts not held. Holding a kinematic body does nothing.
     *
     * @param bodyName one of the world's bodies
     * @param held     true to hold it, false to let it go
     * @throws IllegalArgumentException if the world has no body of that name
     */
    public void setBodyHeld(String bodyName, boolean held) {
        physics.setBodyHeld(bodyIndex(bodyName), held);
    }

    /**
     * {@return whether a body is held still ({@link #setBodyHeld})}
     *
     * @param bodyName one of the world's bodies
     * @throws IllegalArgumentException if the world has no body of that name
     */
    public boolean isBodyHeld(String bodyName) {
        return physics.isBodyHeld(bodyIndex(bodyName));
    }

    /**
     * Put the robot somewhere else without it having driven there. Same as {@link #updateRobotPose} except
     * that {@link #advance} will not read the jump as a velocity: a pose change through this method gives
     * a pushed piece no speed from the jump.
     *
     * <p>It does not make the robot a ghost. If the robot lands on top of a loose piece, the next
     * {@link #advance} still pushes that piece out of the footprint - it just does so with the robot's
     * velocity taken as zero, so the piece is moved clear but not thrown.
     *
     * @param robotXInches    robot centre, field frame, inches (+X right)
     * @param robotYInches    robot centre, field frame, inches (+Y forward)
     * @param robotHeadingRad robot heading, radians CCW-positive
     * @throws IllegalArgumentException if any argument is non-finite
     */
    public void teleportRobot(double robotXInches, double robotYInches, double robotHeadingRad) {
        updateRobotPose(robotXInches, robotYInches, robotHeadingRad);
        advancedBefore = false;
    }

    /**
     * Record where the robot is for this tick and carry every held piece to it.
     *
     * <p>A held piece's field position is simply the robot's field position: the model does not know where
     * inside the robot a piece sits. That is also why {@link #eject} and {@link #launch} make the caller
     * state a position — the tracker cannot work out where a released piece leaves the robot.
     *
     * <p>Call this once per tick before {@link #tryAcquire} and {@link #advance}, which use the pose
     * stored here.
     *
     * @param robotXInches    robot centre, field frame, inches (+X right)
     * @param robotYInches    robot centre, field frame, inches (+Y forward)
     * @param robotHeadingRad robot heading, radians CCW-positive (conventions §7)
     * @throws IllegalArgumentException if any argument is non-finite
     */
    public void updateRobotPose(double robotXInches, double robotYInches, double robotHeadingRad) {
        requireFinite(robotXInches, "robotXInches");
        requireFinite(robotYInches, "robotYInches");
        requireFinite(robotHeadingRad, "robotHeadingRad");
        this.robotXInches = robotXInches;
        this.robotYInches = robotYInches;
        this.robotHeadingRad = robotHeadingRad;
        this.robotPoseKnown = true;
        for (GamePiece piece : pieces) {
            if (piece.isHeld()) {
                piece.setFieldPositionInches(robotXInches, robotYInches, piece.getRadiusInches());
            }
        }
    }

    /**
     * Take every loose piece that {@code volume} overlaps, up to the remaining capacity — but only while
     * {@code mechanismRunning} is true.
     *
     * <p>Those three conditions together are the whole acquisition rule: <b>a loose piece becomes held
     * only when a trigger volume attached to the robot overlaps it while the acquiring mechanism reports
     * itself running, and only if there is room.</b> A mouth parked over a ball with the roller stopped
     * takes nothing, which is what makes this a model of an intake rather than a magnet. A piece in
     * flight is not loose and is never taken.
     *
     * @param volume           the mouth to test, in robot-frame inches; its name goes into the log
     * @param mechanismRunning what the acquiring mechanism reports about itself this tick (e.g. the
     *                         subsystem's own {@code isRunning()}); false means nothing is acquired
     * @return how many pieces were newly acquired by this call (0 when the mechanism is off, the mouth is
     *         empty, or the robot is already full)
     * @throws IllegalStateException    if {@link #updateRobotPose} has never been called — testing a
     *                                  robot-mounted volume against an unknown pose would silently use
     *                                  field centre and acquire the wrong pieces
     * @throws IllegalArgumentException if {@code volume} is null
     */
    public int tryAcquire(TriggerVolume volume, boolean mechanismRunning) {
        return tryAcquire(volume, mechanismRunning, null);
    }

    /**
     * {@link #tryAcquire(TriggerVolume, boolean)} for an intake that only takes some kinds of piece: a
     * mouth sized for the small ball rolls over the big one without taking it.
     *
     * @param volume           the mechanism's trigger volume, in the robot frame
     * @param mechanismRunning whether the mechanism is actually running this tick
     * @param acceptedKinds    the kinds this intake takes, or {@code null} for every kind. A piece of any
     *                         other kind is left LOOSE where it is, even inside the running mouth.
     * @return how many pieces were acquired this call
     * @throws IllegalArgumentException if {@code volume} is null
     * @throws IllegalStateException    if {@link #updateRobotPose} has never been called
     */
    public int tryAcquire(TriggerVolume volume, boolean mechanismRunning, Collection<String> acceptedKinds) {
        return tryAcquire(volume, mechanismRunning, acceptedKinds, Integer.MAX_VALUE);
    }

    /**
     * {@link #tryAcquire(TriggerVolume, boolean, Collection)} for an intake that takes at most
     * {@code maxPieces} this call. A real roller passes one piece at a time: a mouth that overlaps a whole
     * row should not swallow the row in one tick. The caller decides how often it may take again (its own
     * per-piece interval); this only caps one call. Pieces are tried in id order, as always.
     *
     * @param volume           the mechanism's trigger volume, in the robot frame
     * @param mechanismRunning whether the mechanism is actually running this tick
     * @param acceptedKinds    the kinds this intake takes, or {@code null} for every kind
     * @param maxPieces        the most pieces this call may acquire; must be at least 1
     * @return how many pieces were acquired this call, never more than {@code maxPieces}
     * @throws IllegalArgumentException if {@code volume} is null or {@code maxPieces} is below 1
     * @throws IllegalStateException    if {@link #updateRobotPose} has never been called
     */
    public int tryAcquire(TriggerVolume volume, boolean mechanismRunning, Collection<String> acceptedKinds,
            int maxPieces) {
        if (maxPieces < 1) {
            throw new IllegalArgumentException("tryAcquire needs maxPieces >= 1, got " + maxPieces);
        }
        if (volume == null) {
            throw new IllegalArgumentException("tryAcquire needs a trigger volume");
        }
        requireRobotPose("tryAcquire");
        if (!mechanismRunning) {
            return 0;
        }

        int acquired = 0;
        for (GamePiece piece : pieces) {      // id order: reproducible fill order (domain R5)
            if (getHeldCount() >= heldCapacity) {
                break;                        // full — the capacity invariant, enforced before each take
            }
            if (acquired >= maxPieces) {
                break;                        // the caller's per-call cap: one roller, one piece at a time
            }
            if (piece.getState() != PossessionState.LOOSE) {
                continue;
            }
            if (acceptedKinds != null && !acceptedKinds.contains(piece.getKind())) {
                continue;                     // the wrong kind of piece: the mouth does not fit it
            }
            boolean overlapping = volume.overlapsPiece(robotXInches, robotYInches, robotHeadingRad,
                    piece.getFieldXInches(), piece.getFieldYInches(), piece.getRadiusInches());
            if (!overlapping) {
                continue;
            }
            transition(piece, PossessionState.HELD, volume.getName());
            piece.setFieldPositionInches(robotXInches, robotYInches, piece.getRadiusInches());
            piece.setVelocityInchesPerSecond(0.0, 0.0, 0.0);
            acquired++;
        }
        return acquired;
    }

    /**
     * Take the <b>bottom</b> piece out of a goal that has an opening at its base - a column a robot can
     * pull pieces out of from underneath - when a running mouth overlaps that piece.
     *
     * <p>The same three conditions as {@link #tryAcquire} (a running mechanism, a mouth overlapping the
     * piece, room to carry it) plus two that belong to a stacked goal: only the piece that has been parked
     * in the goal longest is reachable, because it is the one at the bottom; and if that piece is not of
     * an accepted kind, <b>nothing</b> comes out, because the piece that blocks the opening is the bottom
     * one and the ones above it cannot get past it. What is left is re-parked one slot lower each, so the
     * column drops by one piece.
     *
     * <p>Which goals have such an opening, and which kinds fit through it, are the season's facts: this
     * method takes any goal it is given. A goal filled from the top only must simply never be passed here.
     * The log line is {@code "<id> SCORED->HELD <mouth name>"}.
     *
     * @param volumeName       the goal to take from; one of the world's scoring volumes
     * @param mouth            the mechanism's trigger volume, in the robot frame
     * @param mechanismRunning whether the mechanism is actually running this tick
     * @param acceptedKinds    the kinds that fit through the opening and into the mouth; never null. Unlike
     *                         {@link #tryAcquire}, there is no "every kind" default: which kinds fit
     *                         through a goal's opening is always a fact the caller has to state
     * @return the id of the piece taken, or {@code -1} when nothing came out
     * @throws IllegalArgumentException if {@code mouth} or {@code acceptedKinds} is null, or the world has no
     *                                  scoring volume of that name
     * @throws IllegalStateException    if {@link #updateRobotPose} has never been called
     */
    public int tryRetrieveBottomPiece(String volumeName, TriggerVolume mouth, boolean mechanismRunning,
            Collection<String> acceptedKinds) {
        if (mouth == null) {
            throw new IllegalArgumentException("tryRetrieveBottomPiece needs a trigger volume");
        }
        if (acceptedKinds == null) {
            throw new IllegalArgumentException("tryRetrieveBottomPiece needs the kinds that fit the opening");
        }
        ScoringVolume volume = world.getScoringVolume(volumeName);
        requireRobotPose("tryRetrieveBottomPiece");
        if (!mechanismRunning || getHeldCount() >= heldCapacity) {
            return -1;
        }
        List<Integer> parkOrder = parkedInOrder(volume.getName());
        if (parkOrder.isEmpty()) {
            return -1;
        }
        GamePiece bottom = pieces.get(parkOrder.get(0));
        if (!acceptedKinds.contains(bottom.getKind())) {
            return -1;                        // the bottom piece does not fit the opening: the column is locked
        }
        boolean overlapping = mouth.overlapsPiece(robotXInches, robotYInches, robotHeadingRad,
                bottom.getFieldXInches(), bottom.getFieldYInches(), bottom.getRadiusInches());
        if (!overlapping) {
            return -1;
        }
        scoredVolumeByPieceId.remove(bottom.getId());
        transition(bottom, PossessionState.HELD, mouth.getName());
        bottom.setFieldPositionInches(robotXInches, robotYInches, bottom.getRadiusInches());
        bottom.setVelocityInchesPerSecond(0.0, 0.0, 0.0);
        reparkRemainingPieces(volume);             // everything above drops one slot
        return bottom.getId();
    }

    /**
     * Release a held piece back onto the field floor at a position the caller states, at rest, leaving it
     * {@link PossessionState#LOOSE} and acquirable again.
     *
     * <p>The caller states the position because the tracker cannot know it: where a released piece comes
     * out depends on the mechanism that released it. A subsystem that drops out of the back of the robot
     * computes the spot it wants (usually the robot pose offset along its backward direction) and passes
     * it here. For a piece thrown rather than dropped, use {@link #launch}.
     *
     * @param pieceId      the piece to release, from {@link #addLoosePiece}
     * @param causeName    a short non-blank label for the log, e.g. {@code "intake"} or {@code "reject"}
     * @param fieldXInches where it lands, field frame, inches
     * @param fieldYInches where it lands, field frame, inches
     * @throws IllegalArgumentException if the id is unknown, the label is blank, or a position is
     *                                  non-finite
     * @throws IllegalStateException    if that piece is not currently held — ejecting something the robot
     *                                  does not have is a caller bug, not a no-op
     */
    public void eject(int pieceId, String causeName, double fieldXInches, double fieldYInches) {
        GamePiece piece = requirePiece(pieceId);
        String cause = requireCauseName(causeName);
        requireFinite(fieldXInches, "fieldXInches");
        requireFinite(fieldYInches, "fieldYInches");
        requireHeld(piece, "eject");
        transition(piece, PossessionState.LOOSE, cause);
        piece.restOnFloorAt(fieldXInches, fieldYInches);
    }

    /**
     * Throw a held piece: it leaves the robot at the stated exit point with the stated velocity and is
     * {@link PossessionState#IN_FLIGHT} from now until {@link #advance} lands it or scores it.
     *
     * <p>The caller states the exit point and velocity because both belong to the season's launcher: the
     * exit point is the robot pose plus the turret's offset and height, and the velocity is the flywheel's
     * speed turned into a ball speed along the turret's heading at the hood's angle. The tracker only
     * flies what it is given. Velocity is in field-frame inches per second, +Z up.
     *
     * @param pieceId       the piece to throw
     * @param causeName     a short non-blank label for the log, e.g. {@code "launcher"}
     * @param exitXInches   where the piece's centre is as it leaves the robot, field frame, inches
     * @param exitYInches   ditto, Y
     * @param exitZInches   ditto, height above the floor
     * @param velocityXInchesPerSecond     inches per second along field +X
     * @param velocityYInchesPerSecond     inches per second along field +Y
     * @param velocityZInchesPerSecond     inches per second upward
     * @throws IllegalArgumentException if the id is unknown, the label is blank, a number is non-finite, or
     *                                  the exit height is below the floor
     * @throws IllegalStateException    if that piece is not currently held
     */
    public void launch(int pieceId, String causeName, double exitXInches, double exitYInches,
            double exitZInches, double velocityXInchesPerSecond, double velocityYInchesPerSecond,
            double velocityZInchesPerSecond) {
        GamePiece piece = requirePiece(pieceId);
        String cause = requireCauseName(causeName);
        requireExitPointAndVelocity(exitXInches, exitYInches, exitZInches, velocityXInchesPerSecond,
                velocityYInchesPerSecond, velocityZInchesPerSecond);
        requireHeld(piece, "launch");
        transition(piece, PossessionState.IN_FLIGHT, cause);
        piece.setFieldPositionInches(exitXInches, exitYInches, exitZInches);
        piece.setVelocityInchesPerSecond(velocityXInchesPerSecond, velocityYInchesPerSecond, velocityZInchesPerSecond);
        piece.setLeavingRobot(true);  // after transition(), which clears it: a shot does not hit its own robot
    }

    /**
     * Place a held piece directly into a scoring location — a deposit mechanism that sets a piece down
     * rather than throwing it — leaving it {@link PossessionState#SCORED} and out of play.
     *
     * <p><b>The piece leaves the field.</b> It is in none of the world's goals, so only the total scored
     * count sees it: it is not {@linkplain GamePiece#isVisibleOnField() visible}, so it is not logged in
     * any positions key and a sim camera does not see it. (Its last position, the robot's, is kept only
     * so the piece has one.) To park it inside one of the world's goals, where it stays visible, use
     * {@link #scoreInto(int, String)}.
     *
     * @param pieceId   the piece to score
     * @param causeName a short non-blank label for the log, e.g. {@code "deposit"}; only a label — to park
     *                  the piece in one of the world's goals use {@link #scoreInto(int, String)}
     * @throws IllegalArgumentException if the id is unknown or the label is blank
     * @throws IllegalStateException    if that piece is not currently held
     */
    public void score(int pieceId, String causeName) {
        GamePiece piece = requirePiece(pieceId);
        String cause = requireCauseName(causeName);
        requireHeld(piece, "score");
        transition(piece, PossessionState.SCORED, cause);
        piece.setVelocityInchesPerSecond(0.0, 0.0, 0.0);
    }

    /**
     * A held piece is scored straight into one of the world's goals — a deposit by hand into a goal the
     * robot is touching, rather than a flight. The piece is parked at that goal's next resting slot and
     * the goal's name is the cause in the log ({@code "3 HELD->SCORED goalNorth"}).
     *
     * <p>Distinct from {@link #score(int, String)} on purpose: there the second argument is only a label,
     * here it is looked up, so a mistyped goal name fails loudly instead of quietly scoring nowhere.
     *
     * @param pieceId    the piece to score
     * @param volumeName the name of one of the world's scoring volumes
     * @throws IllegalArgumentException if the id is unknown or the world has no scoring volume of that name
     * @throws IllegalStateException    if that piece is not currently held
     */
    public void scoreInto(int pieceId, String volumeName) {
        GamePiece piece = requirePiece(pieceId);
        ScoringVolume volume = world.getScoringVolume(volumeName);
        requireHeld(piece, "scoreInto");
        transition(piece, PossessionState.SCORED, volume.getName());
        parkInVolume(piece, volume);
    }

    /**
     * The pieces in one goal right now, in id order: for a scoring volume, the ones {@link #addScoredPiece},
     * {@link #scoreInto} and a scoring flight parked there and nothing has released since; for a
     * {@link BodyCell}, the ones resting in it now, wherever the physics has them.
     *
     * @param volumeName one of the world's scoring volumes or body cells
     * @return the ids of the SCORED pieces in it, lowest first; empty if none
     * @throws IllegalArgumentException if the world has no scoring volume or cell of that name
     */
    public List<Integer> getScoredPieceIdsIn(String volumeName) {
        if (world.findCell(volumeName) != null) {
            List<Integer> seated = new ArrayList<>();
            for (GamePiece piece : pieces) {
                if (volumeName.equals(piece.getSeatedCellName())) {
                    seated.add(piece.getId());
                }
            }
            return seated;
        }
        world.getScoringVolume(volumeName);
        List<Integer> ids = new ArrayList<>();
        for (Map.Entry<Integer, String> entry : scoredVolumeByPieceId.entrySet()) {
            if (entry.getValue().equals(volumeName)) {
                ids.add(entry.getKey());
            }
        }
        Collections.sort(ids);
        return ids;
    }

    /**
     * A goal lets go of a piece it was holding: the piece leaves the goal at the stated point with the
     * stated velocity and is {@link PossessionState#IN_FLIGHT} until {@link #advance} lands it — the
     * season's model of a goal that empties itself (a tipping goal dumps its contents back onto the floor).
     *
     * <p>The caller states the exit point and velocity for the same reason {@link #launch} does: where and
     * how fast a piece leaves a goal belongs to the season's model of that goal. Once released the piece is
     * no longer counted by {@link #getScoredCountIn} for that goal, and every piece still parked there is
     * re-parked at slots {@code 0..n-1} in the order it arrived, so releasing a piece from the middle of a
     * stack closes the gap and a goal that empties completely fills from its first slot again.
     *
     * <p><b>The exit point may be inside the goal's own box.</b> The released piece cannot score back into
     * the goal it just left until its centre has been outside that goal's box once; then it can, like any
     * other piece. It can score into any other goal straight away.
     *
     * @param pieceId                  a piece that is currently SCORED in one of the world's goals
     * @param causeName                a short non-blank label for the log, e.g. {@code "goalTip"}
     * @param exitXInches              where the piece's centre is as it leaves, field frame, inches
     * @param exitYInches              ditto, Y
     * @param exitZInches              ditto, height above the floor
     * @param velocityXInchesPerSecond inches per second along field +X
     * @param velocityYInchesPerSecond inches per second along field +Y
     * @param velocityZInchesPerSecond inches per second upward
     * @throws IllegalArgumentException if the id is unknown, the label is blank, a number is non-finite, or
     *                                  the exit height is below the floor
     * @throws IllegalStateException    if that piece is not SCORED in one of the world's goals or body cells (a
     *                                  piece scored with the label-only {@link #score} is in no goal)
     */
    public void releaseScoredPiece(int pieceId, String causeName, double exitXInches, double exitYInches,
            double exitZInches, double velocityXInchesPerSecond, double velocityYInchesPerSecond,
            double velocityZInchesPerSecond) {
        GamePiece piece = requirePiece(pieceId);
        String cause = requireCauseName(causeName);
        requireExitPointAndVelocity(exitXInches, exitYInches, exitZInches, velocityXInchesPerSecond,
                velocityYInchesPerSecond, velocityZInchesPerSecond);
        String volumeName = scoredVolumeByPieceId.get(pieceId);
        String cellName = piece.getSeatedCellName();
        if (piece.getState() != PossessionState.SCORED || (volumeName == null && cellName == null)) {
            throw new IllegalStateException("cannot release game piece " + pieceId
                    + ": it is not scored in one of the world's goals (state is " + piece.getState() + ")");
        }
        if (volumeName != null) {
            scoredVolumeByPieceId.remove(pieceId);
            reparkRemainingPieces(world.getScoringVolume(volumeName));
        } else {
            volumeName = cellName;
        }
        transition(piece, PossessionState.IN_FLIGHT, cause);
        piece.setFieldPositionInches(exitXInches, exitYInches, exitZInches);
        piece.setVelocityInchesPerSecond(velocityXInchesPerSecond, velocityYInchesPerSecond, velocityZInchesPerSecond);
        piece.setReleasedFromGoalName(volumeName);  // after transition(), which clears it
    }

    /**
     * Swap the world this tracker's physics runs in, keeping every piece where it is and in the state it
     * is in. This is how a season models a field whose fixed geometry changes during a match: a tipping
     * goal whose open side becomes its closed side gets a new world with the scoring volume on the other side.
     *
     * <p>Scoring volumes are matched to the old world by <b>name</b>: a goal that keeps its name keeps
     * its parked pieces and its slot count even if its box moved, and a name the new world does not have
     * loses both — so release (or accept losing track of) what is parked in a goal before taking it away.
     * A piece left parked in a goal that is taken away stays SCORED but is in no goal any more, so, like a
     * piece scored with the label-only {@link #score}, it is no longer {@linkplain
     * GamePiece#isVisibleOnField() visible}. The robot's pose and its last-advance velocity are untouched.
     *
     * <p>{@link KinematicBody Bodies} and their cells are matched by name too: a body the new world keeps keeps
     * its angle, turning rate and whether it is held, and a piece resting in a cell the new world does not have becomes
     * IN_FLIGHT, logged {@code "<cell>Out"}, and falls.
     *
     * @param newWorld the world to run in from now on
     * @throws IllegalArgumentException if {@code newWorld} is null
     */
    public void replaceWorld(FieldWorld newWorld) {
        if (newWorld == null) {
            throw new IllegalArgumentException("newWorld must not be null; use FieldWorld.unbounded()");
        }
        List<String> keptNames = new ArrayList<>();
        for (ScoringVolume volume : newWorld.getScoringVolumes()) {
            keptNames.add(volume.getName());
        }
        List<Integer> orphaned = new ArrayList<>();
        for (Map.Entry<Integer, String> entry : scoredVolumeByPieceId.entrySet()) {
            if (!keptNames.contains(entry.getValue())) {
                orphaned.add(entry.getKey());
            }
        }
        for (Integer id : orphaned) {
            scoredVolumeByPieceId.remove(id);
            pieces.get(id).setParkedInGoal(false);
        }
        GamePiecePhysics oldPhysics = physics;
        FieldWorld oldWorld = world;
        this.world = newWorld;
        this.physics = new GamePiecePhysics(newWorld);
        // A body that keeps its name keeps its angle and turning rate.
        List<KinematicBody> newBodies = newWorld.getKinematicBodies();
        List<KinematicBody> oldBodies = oldWorld.getKinematicBodies();
        for (int i = 0; i < newBodies.size(); i++) {
            for (int j = 0; j < oldBodies.size(); j++) {
                if (oldBodies.get(j).getName().equals(newBodies.get(i).getName())) {
                    physics.setBodyMotion(i, oldPhysics.getBodyAngleRad(j), oldPhysics.getBodyAngularVelocityRadPerS(j));
                    physics.setBodyHeld(i, oldPhysics.isBodyHeld(j));
                }
            }
        }
        // A piece resting in a cell the new world does not have is in nothing now: it falls.
        for (GamePiece piece : pieces) {
            String cellName = piece.getSeatedCellName();
            if (cellName != null && newWorld.findCell(cellName) == null) {
                transition(piece, PossessionState.IN_FLIGHT, cellName + "Out");
            }
        }
    }

    /**
     * The one held piece with the lowest id, or {@code -1} when the robot holds nothing. This is the piece
     * a launcher fires next: pieces leave in the order they arrived, which is what a hopper does and what
     * keeps a replay identical (domain R5).
     *
     * @return the id of the next piece to leave the robot, or -1
     */
    public int nextHeldPieceId() {
        for (GamePiece piece : pieces) {
            if (piece.isHeld()) {
                return piece.getId();
            }
        }
        return -1;
    }

    /**
     * Advance the physics of every piece that is not held by {@code deltaTime} seconds: loose pieces roll,
     * are pushed by the robot and bounce; in-flight pieces fly, and land or score. Call once per sim
     * tick after the robot pose has been updated and after any launches.
     *
     * <p>The robot's velocity, which a pushed piece inherits, is the change in the robot pose since the
     * previous call divided by {@code deltaTime}; on the first call it is zero.
     *
     * <p><b>Any tick length gives the same physics.</b> The tick is split into the fewest equal sub-steps
     * no longer than the world's {@link GamePiecePhysicsConfig#getMaxSubstepSeconds() maxSubstepSeconds}
     * (5 ms by default: four sub-steps for a 20 ms tick, eight for 40 ms), so every stability and
     * tunnelling bound, which is stated per sub-step, holds whatever {@code deltaTime} the caller uses.
     *
     * @param deltaTime the sim tick, seconds (&gt; 0)
     * @throws IllegalArgumentException if {@code deltaTime} is not a positive finite number
     */
    public void advance(double deltaTime) {
        if (!Double.isFinite(deltaTime) || deltaTime <= 0.0) {
            throw new IllegalArgumentException("deltaTime must be a positive finite number, got " + deltaTime);
        }
        GamePiecePhysics.Robot robot = null;
        if (robotPoseKnown) {
            double velocityX = 0.0;
            double velocityY = 0.0;
            if (advancedBefore) {
                velocityX = (robotXInches - lastAdvancedRobotXInches) / deltaTime;
                velocityY = (robotYInches - lastAdvancedRobotYInches) / deltaTime;
            }
            robot = new GamePiecePhysics.Robot(robotXInches, robotYInches, robotHeadingRad,
                    velocityX, velocityY,
                    robotShape == null ? null : robotShape.placedOnRobot(robotXInches, robotYInches, robotHeadingRad));
            lastAdvancedRobotXInches = robotXInches;
            lastAdvancedRobotYInches = robotYInches;
            advancedBefore = true;
        }

        List<OrientedBox> standing = new ArrayList<>(staticRobots.size());
        for (StaticRobot staticRobot : staticRobots) {
            standing.add(staticRobot.toBox());
        }
        physics.setStaticRobotBoxes(standing);

        int substeps = world.getPhysics().substepsFor(deltaTime);
        double substep = deltaTime / substeps;
        for (int step = 0; step < substeps; step++) {
            pendingEvents.clear();
            physics.step(pieces, robot, substep, pendingEvents);
            for (GamePiecePhysics.Event event : pendingEvents) {
                transition(event.piece, event.to, event.cause);
                if (event.to == PossessionState.SCORED) {
                    if (event.cell != null) {
                        event.piece.setSeatedCellName(event.cell.getName());   // resting in it, still moving
                    } else {
                        parkInVolume(event.piece, event.volume);
                    }
                }
            }
        }
        pendingEvents.clear();
    }

    /**
     * Publish this tick's possession state to the RLOG under {@code keyPrefix}, then clear the transition
     * backlog. Call once per tick, after every {@code tryAcquire} / {@code launch} / {@code advance}.
     *
     * <p>Keys written (a season layer typically passes {@code "GamePieces"} and lists the resulting keys in
     * its own logging contract):
     * <ul>
     *   <li>{@code <prefix>/heldCount}, {@code <prefix>/looseCount}, {@code <prefix>/inFlightCount},
     *       {@code <prefix>/scoredCount} — ints; {@code scoredCount} counts every SCORED piece, parked in a
     *       goal or not</li>
     *   <li>{@code <prefix>/atCapacity} — boolean, true while the robot cannot take another piece</li>
     *   <li>{@code <prefix>/transitions} — one string per transition since the last call, in the order they
     *       happened, e.g. {@code "3 LOOSE->HELD intake"}. Empty on a tick where nothing changed hands,
     *       which is most ticks.</li>
     *   <li>{@code <prefix>/loosePositions3d}, {@code <prefix>/inFlightPositions3d},
     *       {@code <prefix>/scoredPositions3d} — the pieces in each of those states as field-frame points
     *       for AdvantageScope's <b>3D Field</b> tab, in <b>metres</b> like every other geometry struct this
     *       repo logs (see {@link #METRES_PER_INCH}; the keys have no unit suffix for the same reason
     *       a robot pose does not). Held pieces are left out because they are all at the robot pose, which
     *       the season already logs; {@code scoredPositions3d} holds only the pieces parked in a goal, so a
     *       piece scored with the label-only {@link #score} is in none of these keys.</li>
     *   <li>{@code <prefix>/kinds/<kind>/positions3d} — every {@linkplain GamePiece#isVisibleOnField()
     *       visible} piece (loose, in flight or parked in a goal) of that kind, for drawing each kind with
     *       its own model: drag it onto the 3D Field as a <i>Game Piece</i> object and pick that kind's
     *       variant. One key per kind ever added, kinds in the order they were first added, <b>written on
     *       every call</b> — an empty array when no piece of that kind is visible. That matters: AdvantageScope
     *       keeps drawing a key's last value until the key is written again, so a kind whose pieces were
     *       all picked up would otherwise stay on screen as ghosts.
     *
     *       <p><b>Why {@code Translation3d} and not {@code Translation2d}.</b> Only the 3D Field tab can
     *       draw a game piece as the game's actual piece model, and its <i>Game Piece</i> object accepts
     *       3-D types only — verified against AdvantageScope v26's own source, where
     *       {@code Field3dController_Config.ts} declares
     *       {@code sourceTypes: ["Pose3d", "Pose3d[]", "Transform3d", "Transform3d[]", "Translation3d",
     *       "Translation3d[]"]}. A {@code Translation2d[]} is accepted on both Field tabs only by
     *       <i>Trajectory</i> (which joins the points into one polyline) and <i>Heatmap</i>, so it can
     *       never render as separate balls. The z of each point is the piece's centre height, which for a
     *       piece on the floor is its own radius, so the model rests on the tiles.</li>
     * </ul>
     *
     * @param keyPrefix the log key prefix, e.g. {@code "GamePieces"} — must be non-blank
     * @throws IllegalArgumentException if {@code keyPrefix} is null or blank
     */
    public void record(String keyPrefix) {
        if (keyPrefix == null || keyPrefix.trim().isEmpty()) {
            throw new IllegalArgumentException("game-piece log key prefix must be a non-blank string");
        }
        String prefix = keyPrefix.trim();

        int held = countInState(PossessionState.HELD);
        int loose = countInState(PossessionState.LOOSE);
        int inFlight = countInState(PossessionState.IN_FLIGHT);
        int scored = countInState(PossessionState.SCORED);

        Logger.recordOutput(prefix + "/heldCount", held);
        Logger.recordOutput(prefix + "/looseCount", loose);
        Logger.recordOutput(prefix + "/inFlightCount", inFlight);
        Logger.recordOutput(prefix + "/scoredCount", scored);
        Logger.recordOutput(prefix + "/atCapacity", held >= heldCapacity);
        Logger.recordOutput(prefix + "/transitions",
                unloggedTransitions.toArray(new String[0]));
        Logger.recordOutput(prefix + "/loosePositions3d", visiblePositionsInState(PossessionState.LOOSE));
        Logger.recordOutput(prefix + "/inFlightPositions3d",
                visiblePositionsInState(PossessionState.IN_FLIGHT));
        Logger.recordOutput(prefix + "/scoredPositions3d", visiblePositionsInState(PossessionState.SCORED));

        // One key per kind ever added, every tick, in first-added order so the key set is the same on
        // every replay. Written even when empty: see the javadoc above on ghosts.
        for (String kind : kindsEverAdded) {
            List<Translation3d> positions = new ArrayList<>();
            for (GamePiece piece : pieces) {
                if (piece.isVisibleOnField() && piece.getKind().equals(kind)) {
                    positions.add(toMetres(piece));
                }
            }
            Logger.recordOutput(prefix + "/kinds/" + kind + "/positions3d",
                    positions.toArray(new Translation3d[0]));
        }

        unloggedTransitions.clear();
    }

    /** {@return the most pieces the robot may hold at once, as declared at construction} */
    public int getHeldCapacity() {
        return heldCapacity;
    }

    /**
     * Give the robot solid parts for the next {@link #advance}, in place of the world's single footprint box:
     * a compound of {@link OrientedBox boxes} in the robot frame, posed by the season for where its mechanisms
     * are this tick. Loose pieces are pushed by each part they touch and pass under a part whose underside is
     * above them; in-flight pieces bounce off every face of every part. Call it each tick a mechanism moves (or
     * every tick); the last shape given stays until replaced. A world swap keeps it.
     *
     * @param shape the robot's parts, robot frame inches, or null to go back to the world's footprint
     */
    public void setRobotShape(RobotShape shape) {
        this.robotShape = shape;
    }

    /** {@return the shape given to {@link #setRobotShape}, or null when the world's footprint is used} */
    public RobotShape getRobotShape() {
        return robotShape;
    }

    /**
     * Put robots that stand still on the field (an alliance partner parked for the period). Every loose and
     * in-flight piece meets each one as a solid box from the floor up. They stay until replaced, and a world
     * swap keeps them.
     *
     * @param robots the standing robots, field frame; an empty list for none
     */
    public void setStaticRobots(List<StaticRobot> robots) {
        if (robots == null) {
            throw new IllegalArgumentException("pass an empty list for no standing robots, not null");
        }
        for (StaticRobot robot : robots) {
            if (robot == null) {
                throw new IllegalArgumentException("a standing robot must not be null");
            }
        }
        this.staticRobots = Collections.unmodifiableList(new ArrayList<>(robots));
    }

    /** {@return the robots standing still, in the order given; empty for none} */
    public List<StaticRobot> getStaticRobots() {
        return staticRobots;
    }

    /** {@return the world this tracker's physics runs in right now (see {@link #replaceWorld})} */
    public FieldWorld getWorld() {
        return world;
    }

    /**
     * How many pieces the robot is carrying right now. Never greater than {@link #getHeldCapacity()}.
     *
     * @return how many pieces the robot is carrying right now
     */
    public int getHeldCount() {
        return countInState(PossessionState.HELD);
    }

    /** {@return how many pieces are on the field floor, available to acquire} */
    public int getLooseCount() {
        return countInState(PossessionState.LOOSE);
    }

    /** {@return how many pieces are airborne right now} */
    public int getInFlightCount() {
        return countInState(PossessionState.IN_FLIGHT);
    }

    /** {@return how many pieces have been scored and are out of play} */
    public int getScoredCount() {
        return countInState(PossessionState.SCORED);
    }

    /**
     * How many pieces are parked in one particular goal right now: the ones the match started with, scored
     * by hand or by a flight, minus any that have since left it ({@link #tryRetrieveBottomPiece},
     * {@link #releaseScoredPiece}). Not a running total of everything ever scored there.
     *
     * <p>For a {@link BodyCell} it is the pieces resting in the cell now.
     *
     * @param volumeName one of the world's scoring volumes or body cells
     * @return the number of pieces in it now, same as {@code getScoredPieceIdsIn(name).size()}
     * @throws IllegalArgumentException if the world has no scoring volume or cell of that name
     */
    public int getScoredCountIn(String volumeName) {
        if (world.findCell(volumeName) != null) {
            return getScoredPieceIdsIn(volumeName).size();
        }
        world.getScoringVolume(volumeName);
        return parkedInOrder(volumeName).size();
    }

    /** {@return true while the robot cannot take another piece} */
    public boolean isAtCapacity() {
        return getHeldCount() >= heldCapacity;
    }

    /**
     * Every piece added so far, in id order.
     *
     * <p><b>Live views, not snapshots.</b> A caller cannot change a piece — {@link GamePiece}'s setters
     * are package-private, and the list itself is unmodifiable — but a piece it holds on to will change
     * position and state underneath it as the tracker updates them. That is deliberate: a scenario asking
     * "where is piece 3 now" wants now, not the answer from the tick it first looked. Read what you need
     * off a piece in the same tick you got it; do not cache a position across ticks and assume it is still
     * true.
     *
     * @return an unmodifiable live view of every piece on the field, in registration order
     */
    public List<GamePiece> getPieces() {
        return Collections.unmodifiableList(pieces);
    }

    /**
     * One piece by id.
     *
     * @param pieceId the id the piece was registered under
     * @return the piece, as a live view (see {@link #getPieces()})
     * @throws IllegalArgumentException if no piece has that id
     */
    public GamePiece getPiece(int pieceId) {
        return requirePiece(pieceId);
    }

    /**
     * The transitions that have happened since the last {@link #record(String)}, in order — the exact
     * strings that call will log.
     *
     * <p><b>A diagnostic surface, not part of the possession model.</b> Nothing a robot does should depend
     * on it: the log is the channel this data is meant to travel on. It is public because a test wants to
     * assert on the transition sequence without decoding an RLOG, and because a tool driving the sim may
     * want to read the same sequence live rather than after the fact. Calling {@link #record(String)} empties
     * it, so read it before you log, not after.
     *
     * @return an unmodifiable view of the pending transition strings, in the order they happened
     */
    public List<String> getUnloggedTransitions() {
        return Collections.unmodifiableList(unloggedTransitions);
    }

    // Apply a state change and queue its log line. Every state change in this class goes through here,
    // which is what makes "every transition is logged" true by construction rather than by discipline.
    private void transition(GamePiece piece, PossessionState to, String causeName) {
        if (unloggedTransitions.size() >= MAX_UNLOGGED_TRANSITIONS) {
            throw new IllegalStateException(MAX_UNLOGGED_TRANSITIONS + " possession transitions have piled"
                    + " up unlogged — record(keyPrefix) is not being called once per tick, so the replay"
                    + " record of who picked up what is being lost");
        }
        // No formatted numbers in this string on purpose: String.format would apply a default locale and a
        // machine with a comma decimal separator would then log different text for the same run (domain
        // R5). Positions land in the positions3d keys instead.
        unloggedTransitions.add(piece.getId() + " " + piece.getState() + "->" + to + " " + causeName);
        piece.setState(to);
        // A state change ends whatever the piece's goal bookkeeping said: parkInVolume and
        // releaseScoredPiece set these again, after this call, when they apply.
        piece.setParkedInGoal(false);
        piece.setReleasedFromGoalName(null);
        piece.setLeavingRobot(false);
        piece.setSeatedCellName(null);
        piece.setSleeping(false);
        piece.setSupportBodyIndex(-1);
        if (to == PossessionState.LOOSE || to == PossessionState.HELD) {
            piece.setSpinRadPerSecond(0.0, 0.0, 0.0);   // spin is only modelled in the air and in a body's cell
        }
    }

    private int bodyIndex(String bodyName) {
        List<KinematicBody> bodies = world.getKinematicBodies();
        for (int i = 0; i < bodies.size(); i++) {
            if (bodies.get(i).getName().equals(bodyName)) {
                return i;
            }
        }
        throw new IllegalArgumentException("no kinematic body named " + bodyName);
    }

    // Park a newly scored piece at the goal's next resting slot (one past the pieces already there), at
    // rest, and record it as in that goal.
    private void parkInVolume(GamePiece piece, ScoringVolume volume) {
        int slot = parkedInOrder(volume.getName()).size();
        double[] rest = volume.restPositionFor(slot);
        piece.setFieldPositionInches(rest[0], rest[1], rest[2]);
        piece.setVelocityInchesPerSecond(0.0, 0.0, 0.0);
        piece.setParkedInGoal(true);
        scoredVolumeByPieceId.put(piece.getId(), volume.getName());
    }

    // After a piece leaves a goal, put every piece still parked there back at slots 0..n-1 in the order
    // they arrived, so the stack has no gap and no two pieces share a slot.
    private void reparkRemainingPieces(ScoringVolume volume) {
        List<Integer> parkOrder = parkedInOrder(volume.getName());
        for (int slot = 0; slot < parkOrder.size(); slot++) {
            double[] rest = volume.restPositionFor(slot);
            pieces.get(parkOrder.get(slot)).setFieldPositionInches(rest[0], rest[1], rest[2]);
        }
    }

    // The pieces parked in one goal in the order they were parked, bottom of the stack first. The map is
    // a LinkedHashMap and a piece's entry is added when it parks and removed when it leaves, so its
    // iteration order is exactly park order.
    private List<Integer> parkedInOrder(String volumeName) {
        List<Integer> ids = new ArrayList<>();
        for (Map.Entry<Integer, String> entry : scoredVolumeByPieceId.entrySet()) {
            if (entry.getValue().equals(volumeName)) {
                ids.add(entry.getKey());
            }
        }
        return ids;
    }

    // The visible pieces in one state, in id order, in metres. For LOOSE and IN_FLIGHT that is every piece
    // in the state; for SCORED it is the ones parked in a goal.
    private Translation3d[] visiblePositionsInState(PossessionState state) {
        List<Translation3d> positions = new ArrayList<>();
        for (GamePiece piece : pieces) {
            if (piece.getState() == state && piece.isVisibleOnField()) {
                positions.add(toMetres(piece));
            }
        }
        return positions.toArray(new Translation3d[0]);
    }

    // Metres, not inches — see METRES_PER_INCH. This is the only place the conversion happens.
    private static Translation3d toMetres(GamePiece piece) {
        return new Translation3d(
                piece.getFieldXInches() * METRES_PER_INCH,
                piece.getFieldYInches() * METRES_PER_INCH,
                piece.getFieldZInches() * METRES_PER_INCH);
    }

    private int countInState(PossessionState state) {
        int count = 0;
        for (GamePiece piece : pieces) {
            if (piece.getState() == state) {
                count++;
            }
        }
        return count;
    }

    private void requireRobotPose(String action) {
        if (!robotPoseKnown) {
            throw new IllegalStateException(
                    "call updateRobotPose(...) before " + action + "(...) — a trigger volume is attached to"
                            + " the robot, so without a pose there is nothing to attach it to");
        }
    }

    private GamePiece requirePiece(int pieceId) {
        if (pieceId < 0 || pieceId >= pieces.size()) {
            throw new IllegalArgumentException("no game piece with id " + pieceId + "; ids run 0.."
                    + (pieces.size() - 1));
        }
        return pieces.get(pieceId);
    }

    private static void requireHeld(GamePiece piece, String action) {
        if (!piece.isHeld()) {
            throw new IllegalStateException("cannot " + action + " game piece " + piece.getId()
                    + ": the robot is not holding it (state is " + piece.getState() + ")");
        }
    }

    private static String requireCauseName(String causeName) {
        if (causeName == null || causeName.trim().isEmpty()) {
            throw new IllegalArgumentException(
                    "a possession transition needs a non-blank cause name for the log");
        }
        return causeName.trim();
    }

    private static String requireKind(String kind) {
        if (kind == null || kind.trim().isEmpty()) {
            throw new IllegalArgumentException("a game piece needs a non-blank kind, e.g. \"ball\"");
        }
        String clean = kind.trim();
        if (clean.indexOf('/') >= 0 || clean.indexOf(' ') >= 0) {
            throw new IllegalArgumentException("a kind becomes part of a log key, so it may not contain"
                    + " '/' or a space: \"" + clean + "\"");
        }
        return clean;
    }

    private static void requireRadius(double radiusInches) {
        requireFinite(radiusInches, "radiusInches");
        if (radiusInches < 0.0) {
            throw new IllegalArgumentException("radiusInches must be >= 0, got " + radiusInches);
        }
    }

    // The six numbers launch() and releaseScoredPiece() both take: all finite, and the exit point not
    // below the floor (a centre height under zero would start the piece underground).
    private static void requireExitPointAndVelocity(double exitXInches, double exitYInches, double exitZInches,
            double velocityXInchesPerSecond, double velocityYInchesPerSecond, double velocityZInchesPerSecond) {
        requireFinite(exitXInches, "exitXInches");
        requireFinite(exitYInches, "exitYInches");
        requireFinite(exitZInches, "exitZInches");
        requireFinite(velocityXInchesPerSecond, "velocityXInchesPerSecond");
        requireFinite(velocityYInchesPerSecond, "velocityYInchesPerSecond");
        requireFinite(velocityZInchesPerSecond, "velocityZInchesPerSecond");
        if (exitZInches < 0.0) {
            throw new IllegalArgumentException("exitZInches is a height above the floor and must be >= 0, got "
                    + exitZInches);
        }
    }

    private static void requireFinite(double value, String name) {
        if (!Double.isFinite(value)) {
            throw new IllegalArgumentException(name + " must be finite, got " + value);
        }
    }
}
