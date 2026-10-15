package org.horizon36596.simloop.field;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.junit.jupiter.api.Test;

/**
 * INVARIANT-tier (domain R9) — the possession model's Contract, clause by clause. These hold for any
 * reimplementation of {@link GamePieceTracker}, because none of them names how the bookkeeping is stored:
 *
 * <ol>
 *   <li>a game piece has a field position in inches and exactly one possession state from a fixed set;</li>
 *   <li>a loose piece becomes held only when a declared trigger volume attached to the robot overlaps it
 *       while the acquiring mechanism reports itself running;</li>
 *   <li>the number of held pieces never exceeds the declared capacity;</li>
 *   <li>an ejected piece returns to a field position the caller states;</li>
 *   <li>every transition is recorded, in order (the RLOG half of clause 5 — that two runs decode
 *       identically — is pinned by {@code GamePieceSimTest} in {@code TeamCode}, where PsiKit's logger is
 *       actually running).</li>
 * </ol>
 *
 * <p><b>Two tests in this file are IMPLEMENTATION-tier, not invariant-tier</b>, and say so at their own
 * declaration: the one pinning the ascending-id fill order and the one pinning the literal transition
 * string. Both name a choice this design made where the Contract only demanded *some* stable answer, so
 * both are legitimately retireable by a reviewed redesign (domain R9) — unlike everything else here,
 * which any correct reimplementation must satisfy.
 *
 * <p>Positions are FTC-Cartesian inches and headings CCW-positive radians throughout (conventions §7).
 * There is no clock anywhere in this class under test, so nothing here needs a {@code FakeTimer} to be
 * deterministic (domain R5).
 */
class GamePieceTrackerTest {

    private static final double FACING_PLUS_X = 0.0;             // robot forward = field +X
    private static final double FACING_PLUS_Y = Math.PI / 2.0;   // robot forward = field +Y
    private static final double PIECE_RADIUS_IN = 1.4;           // any round piece; the season supplies the real one

    /** A mouth across the front of the robot: 6 in deep, 16 in wide, centred 8 in ahead of robot centre. */
    private static TriggerVolume frontMouth() {
        return new TriggerVolume("intake", 8.0, 0.0, 6.0, 16.0);
    }

    // ---------------------------------------------------------------- clause 1

    @Test
    void aNewPieceIsLooseAtThePositionItWasPlaced() {
        GamePieceTracker tracker = new GamePieceTracker(5);
        int id = tracker.addLoosePiece(12.0, -34.5, PIECE_RADIUS_IN);

        GamePiece piece = tracker.getPiece(id);
        assertEquals(PossessionState.LOOSE, piece.getState(), "a piece starts loose on the field");
        assertEquals(12.0, piece.getFieldXInches(), 0.0, "field x in inches, as placed");
        assertEquals(-34.5, piece.getFieldYInches(), 0.0, "field y in inches, as placed");
        assertEquals(PIECE_RADIUS_IN, piece.getRadiusInches(), 0.0, "radius in inches, as placed");
    }

    @Test
    void everyPieceIsAlwaysInExactlyOneStateAndTheStatesAlwaysAddUp() {
        GamePieceTracker tracker = new GamePieceTracker(2);
        tracker.addLoosePiece(0.0, 8.0, PIECE_RADIUS_IN);      // will be acquired
        tracker.addLoosePiece(0.0, 9.0, PIECE_RADIUS_IN);      // will be acquired then scored
        tracker.addLoosePiece(60.0, 60.0, PIECE_RADIUS_IN);    // stays across the field, untouched

        // Walk the piece through every transition the model has, checking after each step that the three
        // counts still partition the pieces — no piece is in two states, none has fallen out of all three.
        assertCountsPartitionEveryPiece(tracker);

        tracker.updateRobotPose(0.0, 0.0, FACING_PLUS_Y);
        tracker.tryAcquire(frontMouth(), true);
        assertCountsPartitionEveryPiece(tracker);

        tracker.score(1, "deposit");
        assertCountsPartitionEveryPiece(tracker);

        tracker.eject(0, "intake", 20.0, 20.0);
        assertCountsPartitionEveryPiece(tracker);

        // And every state a piece ends up in is one of the fixed, named set — nothing invents a fourth.
        List<PossessionState> allowed = Arrays.asList(PossessionState.values());
        for (GamePiece piece : tracker.getPieces()) {
            assertTrue(allowed.contains(piece.getState()),
                    "piece " + piece.getId() + " is in state " + piece.getState()
                            + ", which is not one of the declared possession states");
        }
    }

    @Test
    void aHeldPieceRidesAlongWithTheRobot() {
        GamePieceTracker tracker = new GamePieceTracker(5);
        int id = tracker.addLoosePiece(0.0, 8.0, PIECE_RADIUS_IN);

        tracker.updateRobotPose(0.0, 0.0, FACING_PLUS_Y);
        tracker.tryAcquire(frontMouth(), true);
        assertTrue(tracker.getPiece(id).isHeld(), "the piece in the running mouth is picked up");

        tracker.updateRobotPose(40.0, -20.0, FACING_PLUS_X);
        assertEquals(40.0, tracker.getPiece(id).getFieldXInches(), 0.0,
                "a held piece's field position follows the robot");
        assertEquals(-20.0, tracker.getPiece(id).getFieldYInches(), 0.0,
                "a held piece's field position follows the robot");
    }

    // ---------------------------------------------------------------- clause 2

    @Test
    void aPieceInTheMouthIsOnlyAcquiredWhileTheMechanismReportsItselfRunning() {
        // The truth table of clause 2: overlap AND running. Anything less takes nothing.
        boolean[][] cases = {
                // {pieceIsInTheMouth, mechanismRunning, expectedAcquired}
                {false, false, false},
                {false, true, false},
                {true, false, false},
                {true, true, true},
        };

        for (boolean[] testCase : cases) {
            boolean inMouth = testCase[0];
            boolean running = testCase[1];
            boolean expected = testCase[2];

            GamePieceTracker tracker = new GamePieceTracker(5);
            // 8 in straight ahead is the middle of the mouth; 40 in to the side is nowhere near it.
            int id = inMouth
                    ? tracker.addLoosePiece(0.0, 8.0, PIECE_RADIUS_IN)
                    : tracker.addLoosePiece(40.0, 8.0, PIECE_RADIUS_IN);
            tracker.updateRobotPose(0.0, 0.0, FACING_PLUS_Y);

            int acquired = tracker.tryAcquire(frontMouth(), running);

            String label = "inMouth=" + inMouth + " running=" + running;
            assertEquals(expected ? 1 : 0, acquired, label + ": acquired count");
            assertEquals(expected ? PossessionState.HELD : PossessionState.LOOSE,
                    tracker.getPiece(id).getState(), label + ": resulting state");
        }
    }

    @Test
    void aStoppedMechanismDoesNotEvenTouchAPieceItIsSittingOnTop() {
        // Spelled out separately because it is the behaviour that makes this a model of an intake rather
        // than of a magnet, and it is the one a future refactor is most likely to lose.
        GamePieceTracker tracker = new GamePieceTracker(5);
        tracker.addLoosePiece(0.0, 8.0, PIECE_RADIUS_IN);
        tracker.updateRobotPose(0.0, 0.0, FACING_PLUS_Y);

        assertEquals(0, tracker.tryAcquire(frontMouth(), false), "roller off: nothing is picked up");
        assertTrue(tracker.getUnloggedTransitions().isEmpty(), "and nothing is logged either");

        assertEquals(1, tracker.tryAcquire(frontMouth(), true), "roller on: now it is picked up");
    }

    @Test
    void aScoredPieceIsNeverPickedUpAgain() {
        GamePieceTracker tracker = new GamePieceTracker(5);
        int id = tracker.addLoosePiece(0.0, 8.0, PIECE_RADIUS_IN);
        tracker.updateRobotPose(0.0, 0.0, FACING_PLUS_Y);
        tracker.tryAcquire(frontMouth(), true);
        tracker.score(id, "deposit");

        // The piece's last position is the robot's, so the mouth is not over it any more — park the robot
        // right on top of that spot to be sure the mouth genuinely overlaps it, then try again.
        tracker.updateRobotPose(0.0, -8.0, FACING_PLUS_Y);
        assertEquals(0, tracker.tryAcquire(frontMouth(), true),
                "SCORED is terminal — a mouth sweeping over a scored piece takes nothing");
        assertEquals(PossessionState.SCORED, tracker.getPiece(id).getState(), "and it stays scored");
    }

    @Test
    void acquiringBeforeTheRobotPoseIsKnownIsRefusedRatherThanGuessed() {
        GamePieceTracker tracker = new GamePieceTracker(5);
        tracker.addLoosePiece(0.0, 8.0, PIECE_RADIUS_IN);

        // Defaulting to field centre would silently acquire whatever happened to be near (0, 0).
        assertThrows(IllegalStateException.class, () -> tracker.tryAcquire(frontMouth(), true),
                "a robot-mounted volume with no robot pose has nothing to be attached to");
    }

    // ---------------------------------------------------------------- clause 3

    @Test
    void theHeldCountNeverExceedsTheDeclaredCapacity() {
        // Ten pieces heaped in the mouth, capacity 5. Sweep repeatedly with the roller running.
        int capacity = 5;
        GamePieceTracker tracker = new GamePieceTracker(capacity);
        for (int i = 0; i < 10; i++) {
            tracker.addLoosePiece(0.0, 8.0, PIECE_RADIUS_IN);
        }
        tracker.updateRobotPose(0.0, 0.0, FACING_PLUS_Y);

        for (int sweep = 0; sweep < 4; sweep++) {
            tracker.tryAcquire(frontMouth(), true);
            assertTrue(tracker.getHeldCount() <= capacity,
                    "held count " + tracker.getHeldCount() + " exceeded capacity " + capacity);
        }
        assertEquals(capacity, tracker.getHeldCount(), "a full sweep of a heap fills the robot exactly");
        assertTrue(tracker.isAtCapacity(), "and it reports itself full");
        assertEquals(5, tracker.getLooseCount(), "the other five are still on the floor");

        // Room appears the moment one leaves, and exactly one slot's worth of room.
        tracker.eject(0, "intake", 60.0, 60.0);
        assertFalse(tracker.isAtCapacity(), "ejecting one leaves room for one");
        assertEquals(1, tracker.tryAcquire(frontMouth(), true), "so the next sweep takes exactly one");
        assertEquals(capacity, tracker.getHeldCount(), "back to full, never over");
    }

    @Test
    void aPerCallCapTakesNoMoreThanItsShareOfAHeapAndTheRestOnLaterCalls() {
        // A roller passes pieces one at a time: a mouth over a row of three, capped at 1, takes one per call.
        GamePieceTracker tracker = new GamePieceTracker(5);
        for (int i = 0; i < 3; i++) {
            tracker.addLoosePiece(-3.0 + 3.0 * i, 8.0, PIECE_RADIUS_IN);
        }
        tracker.updateRobotPose(0.0, 0.0, FACING_PLUS_Y);

        assertEquals(1, tracker.tryAcquire(frontMouth(), true, null, 1), "capped at one: takes one");
        assertEquals(1, tracker.getHeldCount(), "and only one is held");
        assertEquals(1, tracker.tryAcquire(frontMouth(), true, null, 1), "the next call takes the next");
        assertEquals(1, tracker.tryAcquire(frontMouth(), true, null, 1), "and the last");
        assertEquals(0, tracker.tryAcquire(frontMouth(), true, null, 1), "then the mouth is empty");
        assertThrows(IllegalArgumentException.class, () -> tracker.tryAcquire(frontMouth(), true, null, 0),
                "a cap below one is a caller bug, not a silent no-op");
    }

    /**
     * IMPLEMENTATION-tier (domain R9). With more pieces in the mouth than slots left, *something* has to
     * break the tie, and clause 5 requires the tie-break be the same on replay — but it does not require
     * it to be ascending id. A redesign that took the nearest piece first would satisfy the Contract and
     * fail this test, which is exactly what makes this implementation-tier and retireable by a reviewed
     * redesign rather than part of the safety envelope. It is still worth pinning: ascending id is what
     * {@code addLoosePiece}'s javadoc promises callers, so changing it silently would break that promise.
     */
    @Test
    void aFullSweepTakesPiecesInIdOrderSoAReplayFillsUpTheSameWay() {
        GamePieceTracker tracker = new GamePieceTracker(2);
        int first = tracker.addLoosePiece(0.0, 8.0, PIECE_RADIUS_IN);
        int second = tracker.addLoosePiece(0.0, 9.0, PIECE_RADIUS_IN);
        int third = tracker.addLoosePiece(0.0, 10.0, PIECE_RADIUS_IN);
        tracker.updateRobotPose(0.0, 0.0, FACING_PLUS_Y);

        assertEquals(2, tracker.tryAcquire(frontMouth(), true), "two slots, three pieces in the mouth");
        assertTrue(tracker.getPiece(first).isHeld(), "the lowest-id piece goes first");
        assertTrue(tracker.getPiece(second).isHeld(), "then the next");
        assertEquals(PossessionState.LOOSE, tracker.getPiece(third).getState(), "the last one is left");
    }

    @Test
    void aZeroCapacityRobotCarriesNothing() {
        GamePieceTracker tracker = new GamePieceTracker(0);
        tracker.addLoosePiece(0.0, 8.0, PIECE_RADIUS_IN);
        tracker.updateRobotPose(0.0, 0.0, FACING_PLUS_Y);

        assertEquals(0, tracker.tryAcquire(frontMouth(), true), "no slots, so nothing is acquired");
        assertEquals(0, tracker.getHeldCount(), "and the held count stays at zero");
    }

    // ---------------------------------------------------------------- clause 4

    @Test
    void anEjectedPieceGoesBackOnTheFieldWhereTheCallerSaid() {
        GamePieceTracker tracker = new GamePieceTracker(5);
        int id = tracker.addLoosePiece(0.0, 8.0, PIECE_RADIUS_IN);
        tracker.updateRobotPose(0.0, 0.0, FACING_PLUS_Y);
        tracker.tryAcquire(frontMouth(), true);

        tracker.eject(id, "deposit", -30.25, 44.5);

        GamePiece piece = tracker.getPiece(id);
        assertEquals(PossessionState.LOOSE, piece.getState(), "an ejected piece is loose again");
        assertEquals(-30.25, piece.getFieldXInches(), 0.0, "at exactly the x the caller stated");
        assertEquals(44.5, piece.getFieldYInches(), 0.0, "at exactly the y the caller stated");
        assertEquals(0, tracker.getHeldCount(), "and the robot is not holding it any more");

        // Loose again means acquirable again: park the mouth over where it landed.
        tracker.updateRobotPose(-30.25, 36.5, FACING_PLUS_Y);
        assertEquals(1, tracker.tryAcquire(frontMouth(), true), "an ejected piece can be picked back up");
    }

    @Test
    void releasingSomethingTheRobotIsNotHoldingIsRefusedRatherThanIgnored() {
        GamePieceTracker tracker = new GamePieceTracker(5);
        int id = tracker.addLoosePiece(0.0, 8.0, PIECE_RADIUS_IN);

        assertThrows(IllegalStateException.class, () -> tracker.eject(id, "deposit", 0.0, 0.0),
                "ejecting a piece the robot never picked up is a caller bug, not a no-op");
        assertThrows(IllegalStateException.class, () -> tracker.score(id, "deposit"),
                "scoring a piece the robot never picked up is a caller bug, not a no-op");
        assertThrows(IllegalArgumentException.class, () -> tracker.eject(99, "deposit", 0.0, 0.0),
                "there is no piece 99");
        assertThrows(IllegalArgumentException.class, () -> tracker.eject(id, " ", 0.0, 0.0),
                "a transition with no cause name would log as an anonymous state change");
    }

    // ---------------------------------------------------------------- clause 5

    /**
     * IMPLEMENTATION-tier (domain R9) for the string <i>format</i>, invariant-tier for the facts inside it.
     * What the Contract requires is that every transition is recorded and that a reader can tell which
     * piece went from which state to which, and why. This asserts the exact text
     * {@code <id> <FROM>-><TO> <cause>} because that text is the published format in
     * {@code docs/process/logging-contract.md} and a human or the Phase-3 scorer parses it out of an RLOG
     * — so changing the format is a breaking change to a log consumer, not a free refactor. A redesign
     * that logged the same four facts as structured fields would satisfy the Contract, fail this test, and
     * be a legitimate reviewed redesign: retire this test then, and update that doc in the same change.
     */
    @Test
    void everyTransitionIsRecordedInTheOrderItHappened() {
        GamePieceTracker tracker = new GamePieceTracker(2);
        tracker.addLoosePiece(0.0, 8.0, PIECE_RADIUS_IN);
        tracker.addLoosePiece(0.0, 9.0, PIECE_RADIUS_IN);
        tracker.updateRobotPose(0.0, 0.0, FACING_PLUS_Y);

        tracker.tryAcquire(frontMouth(), true);
        tracker.score(1, "deposit");
        tracker.eject(0, "intake", 20.0, 20.0);

        assertEquals(
                Arrays.asList(
                        "0 LOOSE->HELD intake",
                        "1 LOOSE->HELD intake",
                        "1 HELD->SCORED deposit",
                        "0 HELD->LOOSE intake"),
                new ArrayList<>(tracker.getUnloggedTransitions()),
                "all four transitions, each naming the piece, both states and what caused it, in order");
    }

    @Test
    void twoIdenticalScenariosProduceIdenticalTransitionSequences() {
        // The determinism clause at the model level: same inputs, same transition log, no clock involved.
        // The RLOG-level version of this (two runs decoding identically) lives in GamePieceSimTest.
        assertEquals(runScriptedScenario(), runScriptedScenario(),
                "two identical runs must produce the same transitions in the same order (domain R5)");
    }

    @Test
    void aRunThatNeverFlushesItsLogFailsLoudRatherThanLosingTheRecord() {
        // Left unbounded, a caller that forgot record() would silently drop the replay record of who
        // picked up what. The tracker refuses instead.
        GamePieceTracker tracker = new GamePieceTracker(1);
        int id = tracker.addLoosePiece(0.0, 8.0, PIECE_RADIUS_IN);
        tracker.updateRobotPose(0.0, 0.0, FACING_PLUS_Y);
        tracker.tryAcquire(frontMouth(), true);

        assertThrows(IllegalStateException.class, () -> {
            // Cycle one piece in and out forever; without a record() call the backlog only grows.
            for (int i = 0; i < 1000; i++) {
                tracker.eject(id, "intake", 0.0, 8.0);
                tracker.tryAcquire(frontMouth(), true);
            }
        }, "an unflushed transition backlog is a missing record() call, and is reported as one");
    }

    // ---------------------------------------------------------------- input guards

    @Test
    void nonsenseInputIsRejectedAtTheDoor() {
        assertThrows(IllegalArgumentException.class, () -> new GamePieceTracker(-1),
                "a negative capacity is a typo");

        GamePieceTracker tracker = new GamePieceTracker(5);
        assertThrows(IllegalArgumentException.class,
                () -> tracker.addLoosePiece(Double.NaN, 0.0, PIECE_RADIUS_IN), "NaN position");
        assertThrows(IllegalArgumentException.class,
                () -> tracker.addLoosePiece(0.0, 0.0, -1.0), "negative radius");
        assertThrows(IllegalArgumentException.class,
                () -> tracker.updateRobotPose(0.0, 0.0, Double.POSITIVE_INFINITY), "infinite heading");
        assertThrows(IllegalArgumentException.class, () -> tracker.tryAcquire(null, true),
                "no volume to test against");
        assertThrows(IllegalArgumentException.class, () -> tracker.record("  "),
                "a blank log key prefix would write keys with no name");
    }

    @Test
    void theListOfPiecesCannotBeChangedFromOutsideTheTracker() {
        // Every transition is logged only because the tracker is the sole route to a state change; a
        // caller that could add or drop pieces behind its back would break clause 5 quietly.
        GamePieceTracker tracker = new GamePieceTracker(5);
        tracker.addLoosePiece(0.0, 8.0, PIECE_RADIUS_IN);
        List<GamePiece> pieces = tracker.getPieces();

        assertThrows(UnsupportedOperationException.class, () -> pieces.remove(0));
        assertThrows(UnsupportedOperationException.class, () -> tracker.getUnloggedTransitions().clear());
    }

    // ---------------------------------------------------------------- helpers

    /** Run a fixed script of robot moves and mechanism states; return the transitions it produced. */
    private static List<String> runScriptedScenario() {
        GamePieceTracker tracker = new GamePieceTracker(3);
        tracker.addLoosePiece(0.0, 24.0, PIECE_RADIUS_IN);
        tracker.addLoosePiece(4.0, 24.0, PIECE_RADIUS_IN);
        tracker.addLoosePiece(-4.0, 24.0, PIECE_RADIUS_IN);
        tracker.addLoosePiece(0.0, 48.0, PIECE_RADIUS_IN);
        TriggerVolume mouth = frontMouth();

        // Drive up the field, roller running, then score what got picked up and eject the rest.
        for (int tick = 0; tick < 20; tick++) {
            tracker.updateRobotPose(0.0, tick, FACING_PLUS_Y);
            tracker.tryAcquire(mouth, tick >= 5);       // roller spins up after five ticks
        }
        for (GamePiece piece : tracker.getPieces()) {
            if (piece.isHeld()) {
                if (piece.getId() % 2 == 0) {
                    tracker.score(piece.getId(), "deposit");
                } else {
                    tracker.eject(piece.getId(), "deposit", 12.0, 12.0);
                }
            }
        }
        return new ArrayList<>(tracker.getUnloggedTransitions());
    }

    // ---------------------------------------------------------------- a goal that moves, a goal that empties

    @Test
    void parkedPiecesWrapIntoRowsAndLayersSoAFullGoalKeepsThemInsideIt() {
        // Three across, two rows 4 in apart along y, then a layer 3 in up: slot 7 is layer 1, row 0, column 1.
        ScoringVolume goal = ScoringVolume.builder("cell")
                .box(-10.0, 10.0, -10.0, 10.0, 50.0, 70.0)
                .restPoint(-3.0, 0.0, 52.0).restSpacing(3.0, 0.0, 0.0)
                .restRows(3, 0.0, 4.0, 0.0)
                .restLayers(2, 0.0, 0.0, 3.0)
                .build();

        assertArrayEquals(new double[] {-3.0, 0.0, 52.0}, goal.restPositionFor(0), 1e-12, "slot 0: the rest point");
        assertArrayEquals(new double[] {3.0, 0.0, 52.0}, goal.restPositionFor(2), 1e-12, "slot 2: end of row 0");
        assertArrayEquals(new double[] {-3.0, 4.0, 52.0}, goal.restPositionFor(3), 1e-12, "slot 3: row 1 starts");
        assertArrayEquals(new double[] {0.0, 0.0, 55.0}, goal.restPositionFor(7), 1e-12, "slot 7: layer 1");
        for (int slot = 0; slot < 12; slot++) {
            double[] rest = goal.restPositionFor(slot);
            assertTrue(Math.abs(rest[0]) <= 3.0 && rest[1] <= 4.0 && rest[2] <= 55.0,
                    "slot " + slot + " stays inside the three-by-two-by-two grid");
        }
        assertThrows(IllegalArgumentException.class,
                () -> ScoringVolume.builder("bad").restRows(0, 0.0, 1.0, 0.0), "a row of zero is a caller bug");
    }

    /** A goal box named {@code goal} spanning the given field-x band, 50..70 in up, resting rows along x. */
    private static ScoringVolume goalNamedOver(String name, double yMin, double yMax) {
        return ScoringVolume.builder(name)
                .box(-10.0, 10.0, yMin, yMax, 50.0, 70.0)
                .descendingOnly(true)
                .restPoint(-6.0, (yMin + yMax) / 2.0, 60.0).restSpacing(3.0, 0.0, 0.0)
                .build();
    }

    @Test
    void anIntakeWithAcceptedKindsLeavesTheOtherKindOnTheFloor() {
        GamePieceTracker tracker = new GamePieceTracker(4);
        int small = tracker.addLoosePiece("small", 0.0, 8.0, PIECE_RADIUS_IN);
        int big = tracker.addLoosePiece("big", 2.0, 8.0, 1.8);
        tracker.updateRobotPose(0.0, 0.0, FACING_PLUS_Y);

        int taken = tracker.tryAcquire(frontMouth(), true, Arrays.asList("small"));

        assertEquals(1, taken, "only the accepted kind is taken");
        assertTrue(tracker.getPiece(small).isHeld(), "the small piece is in the robot");
        assertEquals(PossessionState.LOOSE, tracker.getPiece(big).getState(),
                "the big piece is left on the floor inside the mouth");
        assertEquals(1, tracker.tryAcquire(frontMouth(), true), "with no filter the big piece is taken too");
    }

    @Test
    void replaceWorldKeepsPiecesAndMovesTheGoalItIsNamedAfter() {
        FieldWorld before = FieldWorld.builder().scoringVolume(goalNamedOver("goal", -20.0, -10.0)).build();
        FieldWorld after = FieldWorld.builder().scoringVolume(goalNamedOver("goal", 10.0, 20.0)).build();
        GamePieceTracker tracker = new GamePieceTracker(2, before);
        int parked = tracker.addScoredPiece("ball", PIECE_RADIUS_IN, "goal");
        int loose = tracker.addLoosePiece("ball", 30.0, 30.0, PIECE_RADIUS_IN);

        tracker.replaceWorld(after);

        assertEquals(after, tracker.getWorld(), "the new world is the one physics runs in");
        assertEquals(PossessionState.SCORED, tracker.getPiece(parked).getState(), "a parked piece stays parked");
        assertEquals(1, tracker.getScoredCountIn("goal"), "the goal keeps its count because it kept its name");
        assertEquals(Arrays.asList(parked), tracker.getScoredPieceIdsIn("goal"), "and its parked piece");
        assertEquals(30.0, tracker.getPiece(loose).getFieldXInches(), 0.0, "a loose piece does not move");
        assertEquals(10.0, tracker.getWorld().getScoringVolume("goal").getMinYInches(), 0.0,
                "the goal's box is the new world's box");
    }

    @Test
    void replaceWorldForgetsAGoalTheNewWorldDoesNotHave() {
        FieldWorld before = FieldWorld.builder().scoringVolume(goalNamedOver("goal", -20.0, -10.0)).build();
        GamePieceTracker tracker = new GamePieceTracker(2, before);
        int parked = tracker.addScoredPiece("ball", PIECE_RADIUS_IN, "goal");

        tracker.replaceWorld(FieldWorld.unbounded());

        assertEquals(PossessionState.SCORED, tracker.getPiece(parked).getState(),
                "the piece is still scored, just no longer in a goal the tracker knows");
        assertFalse(tracker.getPiece(parked).isVisibleOnField(),
                "and, in no goal, it is off the field like a piece scored with the label-only score()");
        assertThrows(IllegalArgumentException.class, () -> tracker.getScoredCountIn("goal"),
                "the goal is gone from the world");
        assertThrows(IllegalStateException.class,
                () -> tracker.releaseScoredPiece(parked, "spill", 0.0, 0.0, 40.0, 0.0, 0.0, 0.0),
                "a piece in no goal cannot be released from one");
    }

    @Test
    void releaseScoredPieceSpillsTheGoalAndTheGoalRefillsFromItsFirstSlot() {
        FieldWorld world = FieldWorld.builder()
                .perimeter(72.0, 72.0)
                .scoringVolume(goalNamedOver("goal", 10.0, 20.0))
                .build();
        GamePieceTracker tracker = new GamePieceTracker(2, world);
        int first = tracker.addScoredPiece("ball", PIECE_RADIUS_IN, "goal");
        int second = tracker.addScoredPiece("ball", PIECE_RADIUS_IN, "goal");
        assertEquals(2, tracker.getScoredCountIn("goal"));

        // The goal empties: both pieces leave its outer face 40 in up, drifting outboard.
        tracker.releaseScoredPiece(first, "spill", -3.0, 22.0, 40.0, 0.0, 20.0, 0.0);
        tracker.releaseScoredPiece(second, "spill", 3.0, 22.0, 40.0, 0.0, 20.0, 0.0);

        assertEquals(0, tracker.getScoredCountIn("goal"), "the goal counts nothing once spilled");
        assertTrue(tracker.getScoredPieceIdsIn("goal").isEmpty(), "and holds no piece ids");
        assertEquals(PossessionState.IN_FLIGHT, tracker.getPiece(first).getState(), "a spilled piece falls");
        assertTrue(tracker.getUnloggedTransitions().contains(first + " SCORED->IN_FLIGHT spill"),
                "the spill is logged as a transition: " + tracker.getUnloggedTransitions());

        // Let them fall: with nothing to score into on the way down they land LOOSE outboard of the goal.
        for (int tick = 0; tick < 200; tick++) {
            tracker.advance(0.02);
        }
        assertEquals(PossessionState.LOOSE, tracker.getPiece(first).getState(), "a spilled piece lands");
        assertTrue(tracker.getPiece(first).getFieldYInches() > 22.0,
                "and lands outboard of where it left: y = " + tracker.getPiece(first).getFieldYInches());

        // A new piece scored into the emptied goal takes the first rest slot again, not the third.
        int third = tracker.addScoredPiece("ball", PIECE_RADIUS_IN, "goal");
        assertEquals(-6.0, tracker.getPiece(third).getFieldXInches(), 1e-9,
                "the emptied goal refills from its first slot");
    }

    // ---------------------------------------------------------------- where a scored piece is

    @Test
    void aPieceScoredIntoNoGoalLeavesTheFieldAndOneScoredIntoAGoalStaysVisible() {
        GamePieceTracker tracker = new GamePieceTracker(4, FieldWorld.builder()
                .scoringVolume(goalNamedOver("goal", 10.0, 20.0)).build());
        tracker.updateRobotPose(5.0, 5.0, 0.0);
        int labelOnly = tracker.addHeldPiece("ball", PIECE_RADIUS_IN);
        int parked = tracker.addHeldPiece("ball", PIECE_RADIUS_IN);

        tracker.score(labelOnly, "deposit");
        tracker.scoreInto(parked, "goal");

        assertEquals(PossessionState.SCORED, tracker.getPiece(labelOnly).getState());
        assertFalse(tracker.getPiece(labelOnly).isVisibleOnField(),
                "scored into no goal: not drawn on the tiles at the robot's last spot, not seen by a camera");
        assertFalse(tracker.getPiece(labelOnly).isParkedInGoal());
        assertTrue(tracker.getPiece(parked).isVisibleOnField(), "parked in a goal: still drawn there");
        assertTrue(tracker.getPiece(parked).isParkedInGoal());
        assertEquals(2, tracker.getScoredCount(), "both count as scored");
    }

    @Test
    void releasingAPieceFromTheMiddleOfAGoalClosesTheGapAndTheNextPieceTakesTheNextFreeSlot() {
        GamePieceTracker tracker = new GamePieceTracker(4, FieldWorld.builder()
                .scoringVolume(goalNamedOver("goal", 10.0, 20.0)).build());
        int bottom = tracker.addScoredPiece("ball", PIECE_RADIUS_IN, "goal");   // slot 0, x = -6
        int middle = tracker.addScoredPiece("ball", PIECE_RADIUS_IN, "goal");   // slot 1, x = -3
        int top = tracker.addScoredPiece("ball", PIECE_RADIUS_IN, "goal");      // slot 2, x = 0

        tracker.releaseScoredPiece(middle, "knocked", 0.0, 30.0, 40.0, 0.0, 10.0, 0.0);

        assertEquals(2, tracker.getScoredCountIn("goal"));
        assertEquals(Arrays.asList(bottom, top), tracker.getScoredPieceIdsIn("goal"));
        assertEquals(-6.0, tracker.getPiece(bottom).getFieldXInches(), 1e-12, "the bottom piece stays put");
        assertEquals(-3.0, tracker.getPiece(top).getFieldXInches(), 1e-12, "the top piece drops into the gap");

        int next = tracker.addScoredPiece("ball", PIECE_RADIUS_IN, "goal");
        assertEquals(0.0, tracker.getPiece(next).getFieldXInches(), 1e-12,
                "the next piece parks in slot 2, not on top of the piece that dropped into slot 1");
    }

    @Test
    void anExitPointBelowTheFloorIsRefused() {
        GamePieceTracker tracker = new GamePieceTracker(4, FieldWorld.builder()
                .scoringVolume(goalNamedOver("goal", 10.0, 20.0)).build());
        tracker.updateRobotPose(0.0, 0.0, 0.0);
        int held = tracker.addHeldPiece("ball", PIECE_RADIUS_IN);
        int parked = tracker.addScoredPiece("ball", PIECE_RADIUS_IN, "goal");
        assertThrows(IllegalArgumentException.class,
                () -> tracker.launch(held, "launcher", 0.0, 0.0, -1.0, 100.0, 0.0, 100.0),
                "a launch that starts underground is a caller bug");
        assertThrows(IllegalArgumentException.class,
                () -> tracker.releaseScoredPiece(parked, "spill", 0.0, 0.0, -0.5, 0.0, 0.0, 0.0),
                "so is a release that starts underground");
        assertTrue(tracker.getPiece(held).isHeld(), "and nothing changed");
        assertEquals(PossessionState.SCORED, tracker.getPiece(parked).getState());
    }

    /** The three counts must always add up to every piece in the tracker, with none double-counted. */
    private static void assertCountsPartitionEveryPiece(GamePieceTracker tracker) {
        int total = tracker.getPieces().size();
        int sum = tracker.getHeldCount() + tracker.getLooseCount() + tracker.getScoredCount();
        assertEquals(total, sum,
                "held + loose + scored must equal the " + total + " pieces in the tracker, got " + sum);
    }

    @Test
    public void askingForTheScoredPiecesOfAGoalTheWorldDoesNotHaveIsACallerBug() {
        GamePieceTracker tracker = new GamePieceTracker(4, FieldWorld.builder()
                .scoringVolume(goalNamedOver("goalA", 60.0, 70.0)).build());
        tracker.addScoredPiece("ball", 1.0, "goalA");
        assertEquals(1, tracker.getScoredPieceIdsIn("goalA").size());
        assertThrows(IllegalArgumentException.class, () -> tracker.getScoredPieceIdsIn("goalB"),
                "a goal name the world does not know is a typo, and a typo must not read as an empty goal");
    }

    // ---------------------------------------------------------------- a column emptied from underneath

    /** A column 4 in across standing at (0, 12), pieces stacked upward from 1.4 in, 2.8 in apart. */
    private static FieldWorld columnWorld() {
        return FieldWorld.builder()
                .scoringVolume(ScoringVolume.builder("column")
                        .box(-2.0, 2.0, 10.0, 14.0, 20.0, 22.0)
                        .descendingOnly(true)
                        .restPoint(0.0, 12.0, 1.4).restSpacing(0.0, 0.0, 2.8)
                        .build())
                .build();
    }

    @Test
    void aRunningMouthOverTheColumnTakesTheBottomPieceAndTheRestDropOneSlot() {
        GamePieceTracker tracker = new GamePieceTracker(4, columnWorld());
        int bottom = tracker.addScoredPiece("small", PIECE_RADIUS_IN, "column");
        int middle = tracker.addScoredPiece("small", PIECE_RADIUS_IN, "column");
        int top = tracker.addScoredPiece("small", PIECE_RADIUS_IN, "column");
        tracker.updateRobotPose(0.0, 0.0, FACING_PLUS_Y);   // mouth spans y = 5..11, the column's piece is at 12

        int taken = tracker.tryRetrieveBottomPiece("column", frontMouth(), true, Arrays.asList("small"));

        assertEquals(bottom, taken, "the piece parked first is the one at the bottom");
        assertEquals(PossessionState.HELD, tracker.getPiece(bottom).getState());
        assertTrue(tracker.getUnloggedTransitions().contains(bottom + " SCORED->HELD intake"),
                "logged like any pickup: " + tracker.getUnloggedTransitions());
        assertEquals(Arrays.asList(middle, top), tracker.getScoredPieceIdsIn("column"));
        assertEquals(1.4, tracker.getPiece(middle).getFieldZInches(), 1e-9, "the column drops by one piece");
        assertEquals(4.2, tracker.getPiece(top).getFieldZInches(), 1e-9);
        assertEquals(2, tracker.getScoredCountIn("column"));
    }

    @Test
    void aBottomPieceThatDoesNotFitTheOpeningLocksTheColumn() {
        GamePieceTracker tracker = new GamePieceTracker(4, columnWorld());
        tracker.addScoredPiece("big", 1.8, "column");
        tracker.addScoredPiece("small", PIECE_RADIUS_IN, "column");
        tracker.updateRobotPose(0.0, 0.0, FACING_PLUS_Y);

        assertEquals(-1, tracker.tryRetrieveBottomPiece("column", frontMouth(), true, Arrays.asList("small")),
                "the small piece above cannot get past the big one at the bottom");
        assertEquals(2, tracker.getScoredCountIn("column"));
        assertTrue(tracker.getUnloggedTransitions().isEmpty());
    }

    @Test
    void retrievalNeedsTheMechanismRunningTheMouthOverThePieceAndRoomToCarryIt() {
        GamePieceTracker tracker = new GamePieceTracker(1, columnWorld());
        tracker.addScoredPiece("small", PIECE_RADIUS_IN, "column");
        tracker.addScoredPiece("small", PIECE_RADIUS_IN, "column");
        tracker.updateRobotPose(0.0, -10.0, FACING_PLUS_Y);
        assertEquals(-1, tracker.tryRetrieveBottomPiece("column", frontMouth(), true, Arrays.asList("small")),
                "a mouth that does not reach the piece takes nothing");
        tracker.updateRobotPose(0.0, 0.0, FACING_PLUS_Y);
        assertEquals(-1, tracker.tryRetrieveBottomPiece("column", frontMouth(), false, Arrays.asList("small")),
                "a stopped roller takes nothing");
        assertTrue(tracker.tryRetrieveBottomPiece("column", frontMouth(), true, Arrays.asList("small")) >= 0);
        assertEquals(-1, tracker.tryRetrieveBottomPiece("column", frontMouth(), true, Arrays.asList("small")),
                "a full robot takes nothing");
    }
}
