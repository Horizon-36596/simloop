package org.horizon36596.simloop.field;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;

import org.junit.jupiter.api.Test;

/**
 * INVARIANT-tier (domain R9) — the game-piece physics Contract, clause by clause, as the human stated it
 * on 2026-09-20: pieces can be intaken by teleport, launched pieces follow a proper physics arc, and loose
 * pieces have contact physics with the robot, the field and each other. These hold for any
 * reimplementation of the physics, because each pins a number a first mechanics course gives, not the
 * integrator that produced it:
 *
 * <ol>
 *   <li>a launched piece follows the drag-free ballistic arc, lands where the range equation says, and is
 *       loose again when it does;</li>
 *   <li>a piece that enters a scoring volume the right way is scored; the wrong way (rising, or from the
 *       closed side) it is not;</li>
 *   <li>a rolling piece slows at the configured deceleration and stops;</li>
 *   <li>the robot's footprint pushes a loose piece out of its way and gives it the robot's speed;</li>
 *   <li>two loose pieces never overlap after a step, and a moving one hands momentum to a resting one;</li>
 *   <li>the perimeter and obstacles turn a piece back; a low obstacle is flown over;</li>
 *   <li>a held piece is never moved by physics, and the same scenario twice gives bit-identical
 *       results (domain R5).</li>
 * </ol>
 *
 * <p>Inches, seconds, FTC-Cartesian field frame throughout. No clock anywhere (domain R5).
 */
class GamePiecePhysicsTest {

    private static final double DT = 0.02;
    private static final double RADIUS = 1.4;
    private static final double G = GamePiecePhysicsConfig.STANDARD_GRAVITY_INCHES_PER_S2;

    /** A tracker over a bare world, with one preloaded piece held by a robot parked at the origin. */
    private static GamePieceTracker trackerWithOneHeldPiece(FieldWorld world) {
        GamePieceTracker tracker = new GamePieceTracker(4, world);
        tracker.updateRobotPose(0.0, 0.0, 0.0);
        tracker.addHeldPiece("ball", RADIUS);
        return tracker;
    }

    private static void advanceFor(GamePieceTracker tracker, double seconds) {
        int ticks = (int) Math.round(seconds / DT);
        for (int i = 0; i < ticks; i++) {
            tracker.advance(DT);
        }
    }

    // ---------------------------------------------------------------- clause 1: the arc

    @Test
    void aLaunchedPieceFollowsTheDragFreeBallisticArc() {
        GamePieceTracker tracker = trackerWithOneHeldPiece(FieldWorld.unbounded());
        double speed = 200.0;                                    // in/s, a plausible shot
        double angle = Math.toRadians(60.0);
        double exitZ = 16.0;
        tracker.launch(0, "launcher", 0.0, 0.0, exitZ, speed * Math.cos(angle), 0.0, speed * Math.sin(angle));
        assertEquals(PossessionState.IN_FLIGHT, tracker.getPiece(0).getState());

        double t = 0.5;                                          // well before the apex-to-floor return
        advanceFor(tracker, t);

        GamePiece piece = tracker.getPiece(0);
        double expectedX = speed * Math.cos(angle) * t;
        double expectedZ = exitZ + speed * Math.sin(angle) * t - 0.5 * G * t * t;
        assertEquals(expectedX, piece.getFieldXInches(), 1e-6, "no horizontal force, so x is exact");
        assertEquals(0.0, piece.getFieldYInches(), 1e-9, "launched along +X, y never moves");
        // Semi-implicit Euler at 5 ms sub-steps sits a hair under the exact parabola: g*dt*t/2 = 0.5 in
        // at t = 0.5 s. That is the integrator's known bias, pinned as a bound rather than hidden.
        assertEquals(expectedZ, piece.getFieldZInches(), 0.6, "z follows z0 + vz t - g t^2 / 2");
        assertTrue(piece.getFieldZInches() < expectedZ, "the Euler step lags the exact arc, never leads it");
        assertEquals(speed * Math.sin(angle) - G * t, piece.getVelocityZInchesPerSecond(), 1e-6,
                "vertical speed loses g per second");
    }

    @Test
    void aLaunchedPieceLandsAtTheRangeTheRangeEquationGivesAndIsLooseAgain() {
        GamePieceTracker tracker = trackerWithOneHeldPiece(FieldWorld.unbounded());
        double speed = 150.0;
        double angle = Math.toRadians(45.0);
        // Launch from centre height = radius, so the flat-ground range equation applies exactly.
        tracker.launch(0, "launcher", 0.0, 0.0, RADIUS, speed * Math.cos(angle), 0.0, speed * Math.sin(angle));

        double flightTime = 2.0 * speed * Math.sin(angle) / G;      // 0.55 s
        double range = speed * speed * Math.sin(2.0 * angle) / G;   // 58.3 in
        advanceFor(tracker, flightTime + 5.0);                       // land, then roll to a stop (~3.8 s)

        GamePiece piece = tracker.getPiece(0);
        assertEquals(PossessionState.LOOSE, piece.getState(), "back on the floor and acquirable");
        assertEquals(RADIUS, piece.getFieldZInches(), 0.0, "resting on the floor");
        assertTrue(tracker.getUnloggedTransitions().contains("0 IN_FLIGHT->LOOSE landed"),
                "the landing is a logged transition: " + tracker.getUnloggedTransitions());
        // It reaches the floor within a sub-step of the exact range. Then it skips: each bounce keeps
        // 30% of the vertical speed and all of the horizontal, so the skips add at most
        // v_x * (2 e v_z / g) / (1 - e) of range (a geometric series of ever-shorter hops). Then it
        // rolls on with its horizontal speed until friction stops it, v_x^2 / (2 a) further at most.
        double horizontalSpeed = speed * Math.cos(angle);
        double verticalSpeed = speed * Math.sin(angle);
        double skipOut = horizontalSpeed * (2.0 * 0.3 * verticalSpeed / G) / (1.0 - 0.3);
        double rollOut = horizontalSpeed * horizontalSpeed / (2.0 * 28.0);
        assertTrue(piece.getFieldXInches() >= range - 1.5,
                "landed no earlier than the range equation minus a sub-step, got " + piece.getFieldXInches());
        assertTrue(piece.getFieldXInches() <= range + skipOut + rollOut + 1.5,
                "skipped and rolled no further than the model allows, got " + piece.getFieldXInches());
        int landings = 0;
        for (String transition : tracker.getUnloggedTransitions()) {
            if (transition.equals("0 IN_FLIGHT->LOOSE landed")) {
                landings++;
            }
        }
        assertEquals(1, landings, "the skips are not separate landings; it is logged loose once, when it settles");
        assertEquals(0.0, piece.getVelocityXInchesPerSecond(), 0.0, "and it has come to rest");
    }

    @Test
    void airDragWhenTurnedOnShortensTheFlight() {
        FieldWorld draggy = FieldWorld.builder()
                .physics(GamePiecePhysicsConfig.builder().airDragPerInch(0.002).build()).build();
        GamePieceTracker withDrag = trackerWithOneHeldPiece(draggy);
        GamePieceTracker without = trackerWithOneHeldPiece(FieldWorld.unbounded());
        for (GamePieceTracker tracker : new GamePieceTracker[] {withDrag, without}) {
            tracker.launch(0, "launcher", 0.0, 0.0, 16.0, 120.0, 0.0, 120.0);
            advanceFor(tracker, 0.4);
        }
        assertTrue(withDrag.getPiece(0).getFieldXInches() < without.getPiece(0).getFieldXInches(),
                "drag costs range");
        assertTrue(withDrag.getPiece(0).getFieldZInches() < without.getPiece(0).getFieldZInches(),
                "drag costs height");
    }

    // ---------------------------------------------------------------- clause 2: scoring

    private static ScoringVolume goalAbove(double xMin, double xMax) {
        return ScoringVolume.builder("goal")
                .box(xMin, xMax, -10.0, 10.0, 50.0, 70.0)
                .descendingOnly(true)
                .restPoint(xMin + 3.0, 0.0, 60.0).restSpacing(3.0, 0.0, 0.0)
                .build();
    }

    @Test
    void aPieceDescendingIntoAScoringVolumeIsScoredAndParkedAtItsRestSlot() {
        // 260 in/s at 60 deg from 16 in: apex 66 in above exit (82 in), so on the way down it crosses
        // the goal's 50..70 in band between x = 112 and x = 130 in.
        FieldWorld world = FieldWorld.builder().scoringVolume(goalAbove(90.0, 130.0)).build();
        GamePieceTracker tracker = trackerWithOneHeldPiece(world);
        double angle = Math.toRadians(60.0);
        tracker.launch(0, "launcher", 0.0, 0.0, 16.0, 260.0 * Math.cos(angle), 0.0, 260.0 * Math.sin(angle));
        advanceFor(tracker, 2.0);

        GamePiece piece = tracker.getPiece(0);
        assertEquals(PossessionState.SCORED, piece.getState());
        assertEquals(1, tracker.getScoredCount());
        assertEquals(1, tracker.getScoredCountIn("goal"));
        assertTrue(tracker.getUnloggedTransitions().contains("0 IN_FLIGHT->SCORED goal"),
                tracker.getUnloggedTransitions().toString());
        assertEquals(93.0, piece.getFieldXInches(), 0.0, "parked at the first rest slot");
        assertEquals(60.0, piece.getFieldZInches(), 0.0, "at the rest height");
        assertEquals(0.0, piece.getVelocityZInchesPerSecond(), 0.0, "and at rest");
    }

    @Test
    void aPieceRisingThroughADescendingOnlyVolumeIsNotScored() {
        // The same arc, but the volume sits over the rising half (x in [20, 60], crossed going up between
        // x = 25 and x = 43) and only takes descenders.
        FieldWorld world = FieldWorld.builder().scoringVolume(goalAbove(20.0, 60.0)).build();
        GamePieceTracker tracker = trackerWithOneHeldPiece(world);
        double angle = Math.toRadians(60.0);
        tracker.launch(0, "launcher", 0.0, 0.0, 16.0, 260.0 * Math.cos(angle), 0.0, 260.0 * Math.sin(angle));
        advanceFor(tracker, 3.0);

        assertEquals(0, tracker.getScoredCount(), "went up through it, never down into it");
        assertEquals(PossessionState.LOOSE, tracker.getPiece(0).getState(), "so it landed instead");
    }

    @Test
    void aPieceEnteringFromTheClosedSideIsNotScored() {
        ScoringVolume oneWay = ScoringVolume.builder("goal")
                .box(90.0, 130.0, -10.0, 10.0, 50.0, 70.0)
                .approachDirection(-1.0, 0.0)               // open face is on the +X side: enter moving -X
                .restPoint(100.0, 0.0, 60.0)
                .build();
        GamePieceTracker tracker = trackerWithOneHeldPiece(FieldWorld.builder().scoringVolume(oneWay).build());
        double angle = Math.toRadians(60.0);
        tracker.launch(0, "launcher", 0.0, 0.0, 16.0, 260.0 * Math.cos(angle), 0.0, 260.0 * Math.sin(angle));
        advanceFor(tracker, 3.0);

        assertEquals(0, tracker.getScoredCount(), "arrived moving +X, into the closed face");
    }

    @Test
    void piecesTheMatchStartsWithInAGoalCountAndOccupyTheFirstSlots() {
        FieldWorld world = FieldWorld.builder().scoringVolume(goalAbove(90.0, 130.0)).build();
        GamePieceTracker tracker = new GamePieceTracker(4, world);
        tracker.addScoredPiece("ball", RADIUS, "goal");
        tracker.addScoredPiece("ball", RADIUS, "goal");
        assertEquals(2, tracker.getScoredCountIn("goal"));
        assertEquals(93.0, tracker.getPiece(0).getFieldXInches(), 0.0);
        assertEquals(96.0, tracker.getPiece(1).getFieldXInches(), 0.0);
        assertThrows(IllegalArgumentException.class, () -> tracker.addScoredPiece("ball", RADIUS, "nope"));
    }

    // ---------------------------------------------------------------- clause 3: rolling

    @Test
    void aRollingPieceSlowsAtTheConfiguredDecelerationAndStops() {
        GamePieceTracker tracker = trackerWithOneHeldPiece(FieldWorld.unbounded());
        // "Launch" horizontally from floor height: it lands on the first sub-step and rolls from there.
        double speed = 40.0;
        tracker.launch(0, "launcher", 0.0, 0.0, RADIUS, speed, 0.0, 0.0);
        advanceFor(tracker, 0.5);
        GamePiece piece = tracker.getPiece(0);
        assertEquals(PossessionState.LOOSE, piece.getState());
        // It spends exactly one 5 ms sub-step in flight before the floor catches it, and flight has no
        // rolling friction, so the roll started one sub-step late.
        double subStep = DT / GamePiecePhysicsConfig.defaults().substepsFor(DT);
        assertEquals(speed - 28.0 * (0.5 - subStep), piece.getVelocityXInchesPerSecond(), 1e-9,
                "constant deceleration of 28 in/s^2 from the moment it landed");

        advanceFor(tracker, 2.0);
        assertEquals(0.0, piece.getVelocityXInchesPerSecond(), 0.0, "stopped");
        double stoppingDistance = speed * speed / (2.0 * 28.0);    // 28.6 in
        assertEquals(stoppingDistance, piece.getFieldXInches(), 1.0,
                "v^2 / 2a, give or take the rest-speed snap");
    }

    // ---------------------------------------------------------------- clause 4: the robot pushes

    @Test
    void theRobotFootprintPushesALoosePieceOutOfItsWayAtTheRobotsSpeed() {
        FieldWorld world = FieldWorld.builder().robotFootprint(18.0, 18.0).build();
        GamePieceTracker tracker = new GamePieceTracker(4, world);
        tracker.addLoosePiece("ball", 12.0, 0.0, RADIUS);
        // Drive +X at 50 in/s: 1 in per tick. The front bumper is at x = 9, and it reaches the ball's
        // near edge (12 - 1.4 = 10.6) on the second tick.
        for (int tick = 0; tick < 10; tick++) {
            tracker.updateRobotPose(tick * 1.0, 0.0, 0.0);
            tracker.advance(DT);
        }
        GamePiece piece = tracker.getPiece(0);
        double bumperX = 9.0 + 9.0;                                 // robot at x = 9 after the loop
        assertTrue(piece.getFieldXInches() >= bumperX + RADIUS - 1e-9,
                "never inside the robot: " + piece.getFieldXInches());
        // Each shove gives it the robot's 50 in/s plus 5% rebound, it pulls ahead, friction bleeds a little
        // off, the bumper catches it and shoves again. So at any instant it is within a few in/s of 50.
        assertTrue(piece.getVelocityXInchesPerSecond() >= 45.0 && piece.getVelocityXInchesPerSecond() <= 52.5,
                "carries the robot's speed, give or take a shove: " + piece.getVelocityXInchesPerSecond());
        assertEquals(0.0, piece.getVelocityYInchesPerSecond(), 0.0, "pushed straight, not sideways");
    }

    @Test
    void aStationaryRobotDoesNotMoveAPieceItIsNotTouching() {
        FieldWorld world = FieldWorld.builder().robotFootprint(18.0, 18.0).build();
        GamePieceTracker tracker = new GamePieceTracker(4, world);
        tracker.addLoosePiece("ball", 30.0, 0.0, RADIUS);
        tracker.updateRobotPose(0.0, 0.0, 0.0);
        advanceFor(tracker, 1.0);
        assertEquals(30.0, tracker.getPiece(0).getFieldXInches(), 0.0);
    }

    // ---------------------------------------------------------------- clause 5: piece on piece

    @Test
    void twoOverlappingLoosePiecesAreSeparatedToTouching() {
        GamePieceTracker tracker = new GamePieceTracker(4);
        tracker.addLoosePiece("ball", 0.0, 0.0, RADIUS);
        tracker.addLoosePiece("ball", 2.0, 0.0, RADIUS);        // 2.0 apart, 2.8 is touching
        tracker.advance(DT);
        double gap = tracker.getPiece(1).getFieldXInches() - tracker.getPiece(0).getFieldXInches();
        assertEquals(2.0 * RADIUS, gap, 1e-9, "pushed apart to exactly touching");
        assertEquals(-0.4, tracker.getPiece(0).getFieldXInches(), 1e-9, "half the overlap each way");
    }

    @Test
    void aMovingPieceHandsItsMomentumToARestingOneItHits() {
        GamePieceTracker tracker = new GamePieceTracker(4);
        tracker.updateRobotPose(0.0, 0.0, 0.0);
        tracker.addHeldPiece("ball", RADIUS);                      // id 0, the mover
        tracker.addLoosePiece("ball", 20.0, 0.0, RADIUS);          // id 1, the target
        tracker.launch(0, "launcher", 0.0, 0.0, RADIUS, 60.0, 0.0, 0.0);
        advanceFor(tracker, 1.0);
        GamePiece mover = tracker.getPiece(0);
        GamePiece target = tracker.getPiece(1);
        assertTrue(target.getFieldXInches() > 20.0, "the target was knocked forward: " + target.getFieldXInches());
        assertTrue(mover.getFieldXInches() < target.getFieldXInches() - 2.0 * RADIUS + 1e-9,
                "the mover stayed behind it");
        assertEquals(0.0, target.getFieldYInches(), 1e-9, "a head-on hit stays on the line");
    }

    @Test
    void aHeadOnHitBetweenEqualBallsSharesMomentumByTheRestitution() {
        // No rolling friction, so the velocities right after the hit are still there to read a second later.
        // The rest speed stays at its default: it is also the floor's "has landed" threshold, and a mover
        // that never lands never becomes LOOSE, and only LOOSE pieces collide with each other.
        FieldWorld frictionless = FieldWorld.builder()
                .physics(GamePiecePhysicsConfig.builder()
                        .rollingDecelerationInchesPerS2(0.0)
                        .build())
                .build();
        GamePieceTracker tracker = new GamePieceTracker(4, frictionless);
        tracker.updateRobotPose(0.0, 0.0, 0.0);
        tracker.addHeldPiece("ball", RADIUS);                      // id 0, the mover
        tracker.addLoosePiece("ball", 20.0, 0.0, RADIUS);          // id 1, the target
        double speed = 60.0;
        tracker.launch(0, "launcher", 0.0, 0.0, RADIUS, speed, 0.0, 0.0);
        advanceFor(tracker, 1.0);
        double mover = tracker.getPiece(0).getVelocityXInchesPerSecond();
        double target = tracker.getPiece(1).getVelocityXInchesPerSecond();
        double e = GamePiecePhysicsConfig.defaults().getPieceToPieceRestitution();
        // Equal masses: momentum is conserved and the separation speed is e times the approach speed.
        assertEquals(speed, mover + target, 1e-6, "momentum conserved");
        assertEquals(e * speed, target - mover, 1e-6, "separation speed = e * approach speed");
    }

    @Test
    void scoredPiecesStackUpwardWhenTheRestSpacingIsVertical() {
        ScoringVolume tube = ScoringVolume.builder("tube")
                .box(-2.0, 2.0, -2.0, 2.0, 19.5, 23.5)
                .descendingOnly(true)
                .restPoint(0.0, 0.0, 3.98 + RADIUS)
                .restSpacing(0.0, 0.0, 2.0 * RADIUS)
                .build();
        GamePieceTracker tracker = new GamePieceTracker(4, FieldWorld.builder().scoringVolume(tube).build());
        for (int i = 0; i < 3; i++) {
            tracker.addScoredPiece("ball", RADIUS, "tube");
        }
        for (int i = 0; i < 3; i++) {
            GamePiece piece = tracker.getPiece(i);
            assertEquals(0.0, piece.getFieldXInches(), 0.0, "a vertical stack does not drift in x");
            assertEquals(0.0, piece.getFieldYInches(), 0.0, "or in y");
            assertEquals(3.98 + RADIUS + i * 2.0 * RADIUS, piece.getFieldZInches(), 1e-12,
                    "piece " + i + " sits one diameter above the one below");
        }
    }

    // ---------------------------------------------------------------- clause 6: walls and obstacles

    @Test
    void thePerimeterTurnsALoosePieceBack() {
        FieldWorld world = FieldWorld.builder().perimeter(72.0, 72.0).build();
        GamePieceTracker tracker = trackerWithOneHeldPiece(world);
        // Shot flat along the floor: it is in flight for one sub-step, then lands and rolls LOOSE into the
        // +X wall 12 in away. The tracker has no call that sets a loose piece moving, so this is how a test
        // gets one.
        tracker.launch(0, "launcher", 60.0, 0.0, RADIUS, 100.0, 0.0, 0.0);
        tracker.advance(DT);
        assertEquals(PossessionState.LOOSE, tracker.getPiece(0).getState(), "rolling, not flying, before the wall");
        advanceFor(tracker, 0.3);
        GamePiece piece = tracker.getPiece(0);
        assertEquals(PossessionState.LOOSE, piece.getState());
        assertTrue(piece.getFieldXInches() <= 72.0 - RADIUS + 1e-9, "never through the wall");
        assertTrue(piece.getVelocityXInchesPerSecond() < 0.0, "coming back off it");
    }

    @Test
    void aTallObstacleDeflectsAndALowOneIsFlownOver() {
        ObstacleBox tall = new ObstacleBox("post", 40.0, 44.0, -5.0, 5.0, 0.0, 60.0);
        ObstacleBox low = new ObstacleBox("bar", 40.0, 44.0, -5.0, 5.0, 0.0, 2.0);
        double angle = Math.toRadians(45.0);
        double speed = 150.0;                                     // apex ~ 22 in above exit

        GamePieceTracker blocked = trackerWithOneHeldPiece(FieldWorld.builder().obstacle(tall).build());
        blocked.launch(0, "launcher", 0.0, 0.0, 16.0, speed * Math.cos(angle), 0.0, speed * Math.sin(angle));
        advanceFor(blocked, 2.0);
        assertTrue(blocked.getPiece(0).getFieldXInches() < 40.0, "bounced off the post: "
                + blocked.getPiece(0).getFieldXInches());

        GamePieceTracker clear = trackerWithOneHeldPiece(FieldWorld.builder().obstacle(low).build());
        clear.launch(0, "launcher", 0.0, 0.0, 16.0, speed * Math.cos(angle), 0.0, speed * Math.sin(angle));
        advanceFor(clear, 2.0);
        assertTrue(clear.getPiece(0).getFieldXInches() > 44.0, "sailed over the bar: "
                + clear.getPiece(0).getFieldXInches());
    }

    // ---------------------------------------------------------------- the tick length does not matter

    @Test
    void theTickIsSplitIntoSubStepsNoLongerThanTheConfiguredMaximum() {
        GamePiecePhysicsConfig defaults = GamePiecePhysicsConfig.defaults();
        assertEquals(0.005, defaults.getMaxSubstepSeconds(), 0.0);
        assertEquals(4, defaults.substepsFor(0.020), "a 20 ms tick is exactly four 5 ms steps, as always");
        assertEquals(2, defaults.substepsFor(0.010));
        assertEquals(8, defaults.substepsFor(0.040));
        assertEquals(5, defaults.substepsFor(0.021), "a tick a hair over 20 ms needs a fifth step");
        assertEquals(1, defaults.substepsFor(0.001), "a tick shorter than one step is one step");
        @SuppressWarnings("deprecation")
        GamePiecePhysicsConfig twentySevenPerTick = GamePiecePhysicsConfig.builder().substepsPerTick(27).build();
        // 0.02 / (0.02 / 27) is 27.000000000000004 in doubles, so a bare ceil would say 28.
        assertEquals(27, twentySevenPerTick.substepsFor(0.020),
                "the old setting still means what it did at a 20 ms tick, despite 0.02 / (0.02 / 27) rounding");
        assertThrows(IllegalArgumentException.class,
                () -> GamePiecePhysicsConfig.builder().maxSubstepSeconds(1e-7),
                "a sub-step below a microsecond is refused, so the count cannot overflow an int");
    }

    @Test
    void aShotFliesTheSameArcWhetherTheTickIsTenTwentyOrFortyMilliseconds() {
        double[] tickSeconds = {0.010, 0.020, 0.040};
        double[][] positionAtPointFourSeconds = new double[3][];
        for (int i = 0; i < tickSeconds.length; i++) {
            GamePieceTracker tracker = trackerWithOneHeldPiece(FieldWorld.unbounded());
            tracker.launch(0, "launcher", 0.0, 0.0, 16.0, 150.0, 0.0, 150.0);
            int ticks = (int) Math.round(0.4 / tickSeconds[i]);
            for (int tick = 0; tick < ticks; tick++) {
                tracker.advance(tickSeconds[i]);
            }
            GamePiece piece = tracker.getPiece(0);
            positionAtPointFourSeconds[i] = new double[] {piece.getFieldXInches(), piece.getFieldZInches()};
        }
        // Every tick length is split into 5 ms steps, so the integrator takes the same 80 steps each time.
        // Before, a 40 ms tick took four 10 ms steps and flew about 0.4 in lower at t = 0.4 s.
        for (int i = 1; i < tickSeconds.length; i++) {
            assertEquals(positionAtPointFourSeconds[0][0], positionAtPointFourSeconds[i][0], 1e-9,
                    "x at a " + tickSeconds[i] + " s tick");
            assertEquals(positionAtPointFourSeconds[0][1], positionAtPointFourSeconds[i][1], 1e-9,
                    "z at a " + tickSeconds[i] + " s tick");
        }
    }

    // ---------------------------------------------------------------- goals: thin mouths, released pieces

    @Test
    void aFastPieceCannotSkipAThinGoalBetweenTwoSubSteps() {
        // A 0.6 in deep mouth at x = 100.2..100.8. At 260 in/s the centre moves 1.3 in per sub-step, from
        // x = 100.1 straight to 101.4: it never ends a sub-step inside the box, but its path crosses it.
        ScoringVolume thin = ScoringVolume.builder("slot")
                .box(100.2, 100.8, -10.0, 10.0, 0.0, 100.0)
                .restPoint(100.5, 0.0, 50.0)
                .build();
        GamePieceTracker tracker = trackerWithOneHeldPiece(FieldWorld.builder().scoringVolume(thin).build());
        tracker.launch(0, "launcher", 0.0, 0.0, 40.0, 260.0, 0.0, 0.0);
        advanceFor(tracker, 1.0);
        assertEquals(PossessionState.SCORED, tracker.getPiece(0).getState(),
                "the path went through the mouth: " + tracker.getUnloggedTransitions());
        assertEquals(1, tracker.getScoredCountIn("slot"));
    }

    /** A goal whose box is x -10..10, y 10..20, z 50..70, entered only by a descending piece. */
    private static FieldWorld worldWithOneRaisedGoal() {
        return FieldWorld.builder()
                .scoringVolume(ScoringVolume.builder("goal")
                        .box(-10.0, 10.0, 10.0, 20.0, 50.0, 70.0)
                        .descendingOnly(true)
                        .restPoint(-6.0, 15.0, 60.0).restSpacing(3.0, 0.0, 0.0)
                        .build())
                .build();
    }

    @Test
    void aPieceReleasedFromInsideItsGoalIsNotScoredBackIntoIt() {
        GamePieceTracker tracker = new GamePieceTracker(4, worldWithOneRaisedGoal());
        int piece = tracker.addScoredPiece("ball", RADIUS, "goal");
        // Let go from the middle of the goal's own box, already moving down: the exact case a goal that
        // dumps its contents produces. Every entry condition holds, and it must still not re-score.
        tracker.releaseScoredPiece(piece, "dump", 0.0, 15.0, 60.0, 0.0, 0.0, -10.0);
        advanceFor(tracker, 3.0);

        assertEquals(PossessionState.LOOSE, tracker.getPiece(piece).getState(), "it fell out and landed");
        assertEquals(0, tracker.getScoredCountIn("goal"), "the goal no longer counts it");
        for (String transition : tracker.getUnloggedTransitions()) {
            assertFalse(transition.contains("->SCORED"), "never re-scored: " + tracker.getUnloggedTransitions());
        }
    }

    @Test
    void aReleasedPieceThatLeavesItsGoalAndComesBackDownIntoItScoresAgain() {
        GamePieceTracker tracker = new GamePieceTracker(4, worldWithOneRaisedGoal());
        int piece = tracker.addScoredPiece("ball", RADIUS, "goal");
        // Thrown straight up out of the top (z = 70) from inside: apex about 60 + 200^2 / (2 g) = 112 in.
        tracker.releaseScoredPiece(piece, "pop", 0.0, 15.0, 60.0, 0.0, 0.0, 200.0);
        advanceFor(tracker, 2.0);

        assertEquals(PossessionState.SCORED, tracker.getPiece(piece).getState(),
                "once out of the box it is an ordinary piece, and it fell back in: "
                        + tracker.getUnloggedTransitions());
        assertEquals(1, tracker.getScoredCountIn("goal"));
    }

    // ---------------------------------------------------------------- a ball is round at a corner

    @Test
    void aPieceOffAnObstacleCornerByLessThanASquareButMoreThanARadiusIsNotTouched() {
        ObstacleBox box = new ObstacleBox("block", 0.0, 10.0, 0.0, 10.0, 0.0, 20.0);
        GamePieceTracker tracker = new GamePieceTracker(4, FieldWorld.builder().obstacle(box).build());
        // 1.2 in past the corner along both axes: 1.70 in from the corner, more than the 1.4 in radius.
        int clear = tracker.addLoosePiece("ball", 11.2, 11.2, RADIUS);
        tracker.advance(DT);
        assertEquals(11.2, tracker.getPiece(clear).getFieldXInches(), 0.0, "a round ball does not reach the corner");
        assertEquals(11.2, tracker.getPiece(clear).getFieldYInches(), 0.0);
    }

    @Test
    void aPieceOverlappingAnObstacleCornerIsPushedStraightAwayFromTheCorner() {
        ObstacleBox box = new ObstacleBox("block", 0.0, 10.0, 0.0, 10.0, 0.0, 20.0);
        GamePieceTracker tracker = new GamePieceTracker(4, FieldWorld.builder().obstacle(box).build());
        int touching = tracker.addLoosePiece("ball", 10.8, 10.8, RADIUS);     // 1.13 in from the corner
        tracker.advance(DT);
        GamePiece piece = tracker.getPiece(touching);
        double fromCornerX = piece.getFieldXInches() - 10.0;
        double fromCornerY = piece.getFieldYInches() - 10.0;
        assertEquals(fromCornerX, fromCornerY, 1e-12, "pushed along the diagonal, the corner's normal");
        assertEquals(RADIUS, Math.hypot(fromCornerX, fromCornerY), 1e-9, "to exactly touching the corner");
    }

    @Test
    void aPieceOffARobotCornerByMoreThanARadiusIsNotPushed() {
        GamePieceTracker tracker = new GamePieceTracker(4, FieldWorld.builder().robotFootprint(18.0, 18.0).build());
        tracker.updateRobotPose(0.0, 0.0, 0.0);
        int clear = tracker.addLoosePiece("ball", 10.2, 10.2, RADIUS);      // 1.70 in off the (9, 9) corner
        int touching = tracker.addLoosePiece("ball", -9.8, 9.8, RADIUS);    // 1.13 in off the (-9, 9) corner
        tracker.advance(DT);
        assertEquals(10.2, tracker.getPiece(clear).getFieldXInches(), 0.0, "not touching, not pushed");
        assertEquals(10.2, tracker.getPiece(clear).getFieldYInches(), 0.0);
        double fromCornerX = tracker.getPiece(touching).getFieldXInches() + 9.0;
        double fromCornerY = tracker.getPiece(touching).getFieldYInches() - 9.0;
        assertEquals(-fromCornerX, fromCornerY, 1e-9, "pushed out along the corner's diagonal");
        assertEquals(RADIUS, Math.hypot(fromCornerX, fromCornerY), 1e-9, "to exactly touching");
    }

    // ---------------------------------------------------------------- clause 7: held stays put; R5

    @Test
    void physicsNeverMovesAHeldPieceAndLaunchNeedsAHeldOne() {
        GamePieceTracker tracker = new GamePieceTracker(4, FieldWorld.builder().perimeter(72, 72).build());
        tracker.updateRobotPose(10.0, 20.0, 1.0);
        tracker.addHeldPiece("ball", RADIUS);
        tracker.addLoosePiece("ball", -30.0, 0.0, RADIUS);
        advanceFor(tracker, 1.0);
        assertEquals(10.0, tracker.getPiece(0).getFieldXInches(), 0.0);
        assertEquals(20.0, tracker.getPiece(0).getFieldYInches(), 0.0);
        assertThrows(IllegalStateException.class,
                () -> tracker.launch(1, "launcher", 0, 0, 16, 100, 0, 100), "piece 1 is loose, not held");
        assertEquals(0, tracker.nextHeldPieceId());
        tracker.launch(0, "launcher", 10.0, 20.0, 16.0, 0.0, 100.0, 100.0);
        assertEquals(-1, tracker.nextHeldPieceId(), "nothing left to fire");
        assertFalse(tracker.getPiece(0).isHeld());
    }

    @Test
    void theSameScenarioTwiceIsBitIdentical() {
        List<String> firstTransitions;
        double[] firstFinal;
        GamePieceTracker first = busyScenario();
        firstTransitions = first.getUnloggedTransitions();
        firstFinal = finalPositions(first);
        GamePieceTracker second = busyScenario();
        assertEquals(firstTransitions, second.getUnloggedTransitions());
        double[] secondFinal = finalPositions(second);
        for (int i = 0; i < firstFinal.length; i++) {
            assertEquals(Double.doubleToLongBits(firstFinal[i]), Double.doubleToLongBits(secondFinal[i]),
                    "component " + i + " differs between two runs of the same scenario");
        }
    }

    /** Everything at once: a pushed heap, a wall, a goal, two shots, one landing. */
    private static GamePieceTracker busyScenario() {
        FieldWorld world = FieldWorld.builder()
                .perimeter(72.0, 72.0)
                .robotFootprint(18.0, 18.0)
                .obstacle(new ObstacleBox("post", 30.0, 34.0, 20.0, 24.0, 0.0, 30.0))
                .scoringVolume(goalAbove(50.0, 72.0))
                .build();
        GamePieceTracker tracker = new GamePieceTracker(4, world);
        tracker.updateRobotPose(-40.0, 0.0, 0.0);
        tracker.addHeldPiece("ball", RADIUS);
        tracker.addHeldPiece("ball", RADIUS);
        for (int i = 0; i < 6; i++) {
            tracker.addLoosePiece("ball", -20.0 + i * 2.5, 0.3 * i, RADIUS);
        }
        for (int tick = 0; tick < 150; tick++) {
            tracker.updateRobotPose(-40.0 + tick * 0.4, 0.0, 0.0);
            if (tick == 10) {
                double angle = Math.toRadians(60.0);
                tracker.launch(0, "launcher", -36.0, 0.0, 16.0, 200.0 * Math.cos(angle), 0.0,
                        200.0 * Math.sin(angle));
            }
            if (tick == 40) {
                tracker.launch(1, "launcher", -24.0, 0.0, 16.0, 100.0, 30.0, 60.0);
            }
            tracker.advance(DT);
        }
        return tracker;
    }

    private static double[] finalPositions(GamePieceTracker tracker) {
        List<GamePiece> pieces = tracker.getPieces();
        double[] out = new double[pieces.size() * 6];
        for (int i = 0; i < pieces.size(); i++) {
            GamePiece p = pieces.get(i);
            out[i * 6] = p.getFieldXInches();
            out[i * 6 + 1] = p.getFieldYInches();
            out[i * 6 + 2] = p.getFieldZInches();
            out[i * 6 + 3] = p.getVelocityXInchesPerSecond();
            out[i * 6 + 4] = p.getVelocityYInchesPerSecond();
            out[i * 6 + 5] = p.getVelocityZInchesPerSecond();
        }
        return out;
    }
}
