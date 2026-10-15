package org.horizon36596.simloop.field;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

/**
 * INVARIANT-tier (domain R9) — the Contract: <b>a robot with a height is solid to pieces in the air. A piece
 * that flies into its side bounces back off the side it came in by, a piece dropped on it bounces off its
 * top, and a piece the robot launches is not stopped by the robot it left.</b> Plus the floor's grip: a
 * piece that bounces on the floor keeps the configured share of its horizontal speed.
 *
 * <p>Why it is pinned. Before the height existed only loose pieces felt the robot, so a ball spilled from a
 * goal flew straight through the chassis, and the first tick it touched the floor inside the footprint it
 * was shoved out of the <i>far</i> side, because the loose-piece push uses the axis of least overlap. Both
 * the pass-through and the wrong-side exit are what these tests rule out.
 *
 * <p>Robot: 18 in square and 18 in tall, at the origin facing +X unless stated. Inches, seconds, field frame.
 * No clock (domain R5).
 */
class GamePieceRobotBodyTest {

    private static final double DT = 0.02;
    private static final double RADIUS = 1.4;
    private static final double HALF = 9.0;
    private static final double HEIGHT = 18.0;

    private static FieldWorld robotWorld(GamePiecePhysicsConfig physics) {
        return FieldWorld.builder().robotFootprint(2 * HALF, 2 * HALF).robotHeight(HEIGHT).physics(physics).build();
    }

    /** A tracker whose robot sits at the origin holding one piece, ready to throw it from anywhere. */
    private static GamePieceTracker trackerHoldingOne(FieldWorld world) {
        GamePieceTracker tracker = new GamePieceTracker(4, world);
        tracker.updateRobotPose(0.0, 0.0, 0.0);
        tracker.addHeldPiece("ball", RADIUS);
        return tracker;
    }

    @Test
    void aPieceFlyingIntoTheRobotsSideBouncesBackOffTheNearSide() {
        GamePiecePhysicsConfig physics = GamePiecePhysicsConfig.builder().pieceToRobotRestitution(0.5).build();
        GamePieceTracker tracker = trackerHoldingOne(robotWorld(physics));
        // From 30 in behind the robot, level at 10 in, flying +X at 150 in/s straight at the back face.
        tracker.launch(0, "spill", -30.0, 0.0, 10.0, 150.0, 0.0, 0.0);
        boolean bounced = false;
        for (int tick = 0; tick < 25; tick++) {
            tracker.advance(DT);
            GamePiece piece = tracker.getPiece(0);
            assertTrue(piece.getFieldXInches() <= -HALF - RADIUS + 1e-9,
                    "tick " + tick + ": never inside or past the back face, x = " + piece.getFieldXInches());
            if (piece.getVelocityXInchesPerSecond() < 0.0) {
                bounced = true;
            }
        }
        assertTrue(bounced, "it came back off the face it hit");
        assertTrue(tracker.getPiece(0).getFieldXInches() < -HALF - RADIUS - 1.0, "and moved away from it");
    }

    @Test
    void theBounceOffTheSideUsesTheRobotRestitution() {
        GamePiecePhysicsConfig physics = GamePiecePhysicsConfig.builder().pieceToRobotRestitution(0.5).build();
        GamePieceTracker tracker = trackerHoldingOne(robotWorld(physics));
        tracker.launch(0, "spill", -12.0, 0.0, 10.0, 150.0, 0.0, 0.0);   // reaches the face in the first tick
        tracker.advance(DT);
        assertEquals(-75.0, tracker.getPiece(0).getVelocityXInchesPerSecond(), 1e-9,
                "half of the 150 in/s comes back, against a stationary robot");
    }

    @Test
    void aPieceDroppedOnTheRobotBouncesOffItsTop() {
        GamePiecePhysicsConfig physics = GamePiecePhysicsConfig.builder().pieceToRobotRestitution(0.5).build();
        GamePieceTracker tracker = trackerHoldingOne(robotWorld(physics));
        double dropHeight = 40.0;
        tracker.launch(0, "spill", 2.0, -3.0, dropHeight, 0.0, 0.0, 0.0);
        double impactSpeed = Math.sqrt(2.0 * GamePiecePhysicsConfig.STANDARD_GRAVITY_INCHES_PER_S2
                * (dropHeight - HEIGHT - RADIUS));
        double firstRebound = Double.NaN;
        for (int tick = 0; tick < 30; tick++) {
            tracker.advance(DT);
            GamePiece piece = tracker.getPiece(0);
            assertTrue(piece.getFieldZInches() >= HEIGHT + RADIUS - 1e-9,
                    "tick " + tick + ": never sinks into the robot, z = " + piece.getFieldZInches());
            if (Double.isNaN(firstRebound) && piece.getVelocityZInchesPerSecond() > 0.0) {
                firstRebound = piece.getVelocityZInchesPerSecond();
            }
        }
        assertTrue(!Double.isNaN(firstRebound), "it bounced up off the roof");
        // Half the impact speed comes back, less at most one tick of gravity (the sub-step it was read on).
        assertEquals(0.5 * impactSpeed, firstRebound,
                GamePiecePhysicsConfig.STANDARD_GRAVITY_INCHES_PER_S2 * DT + 0.02 * impactSpeed);
        assertEquals(PossessionState.IN_FLIGHT, tracker.getPiece(0).getState(), "the roof is not the floor");
    }

    @Test
    void aRobotDrivingIntoAPieceInTheAirPushesItOutOfItsFrontNotItsBack() {
        GamePieceTracker tracker = trackerHoldingOne(robotWorld(GamePiecePhysicsConfig.defaults()));
        tracker.launch(0, "spill", 13.0, 0.0, 17.0, 0.0, 0.0, 0.0);      // 13 in ahead, falling from 17 in
        for (int tick = 1; tick <= 15; tick++) {
            double robotX = 2.0 * tick;                                   // 100 in/s at it
            tracker.updateRobotPose(robotX, 0.0, 0.0);
            tracker.advance(DT);
            GamePiece piece = tracker.getPiece(0);
            if (piece.getFieldZInches() < HEIGHT + RADIUS - 1e-9) {
                assertTrue(piece.getFieldXInches() >= robotX + HALF + RADIUS - 1e-9,
                        "tick " + tick + ": in front of the bumper, x = " + piece.getFieldXInches()
                                + " with the robot at " + robotX);
            }
        }
        assertTrue(tracker.getPiece(0).getVelocityXInchesPerSecond() > 0.0, "carried forward by the bumper");
    }

    @Test
    void aPieceTheRobotLaunchesFromInsideItselfIsNotStoppedByIt() {
        GamePieceTracker tracker = trackerHoldingOne(robotWorld(GamePiecePhysicsConfig.defaults()));
        tracker.launch(0, "launcher", 0.0, 0.0, 12.0, 200.0, 0.0, 100.0);
        for (int tick = 0; tick < 10; tick++) {
            tracker.advance(DT);
        }
        assertEquals(200.0, tracker.getPiece(0).getVelocityXInchesPerSecond(), 0.0, "flew out untouched");
    }

    @Test
    void withoutAHeightAPieceInTheAirStillPassesThroughTheRobot() {
        FieldWorld flat = FieldWorld.builder().robotFootprint(2 * HALF, 2 * HALF).build();
        GamePieceTracker tracker = trackerHoldingOne(flat);
        // A lob that stays between 1.4 and 14.7 in high for the 0.42 s it takes to cross the robot's square.
        tracker.launch(0, "spill", -30.0, 0.0, 10.0, 150.0, 0.0, 60.0);
        for (int tick = 0; tick < 21; tick++) {
            tracker.advance(DT);
        }
        assertTrue(tracker.getPiece(0).getFieldXInches() > HALF + RADIUS,
                "it came out of the far side: the behaviour every older world keeps");
    }

    @Test
    void aRobotHeightMustBePositive() {
        assertThrows(IllegalArgumentException.class, () -> FieldWorld.builder().robotHeight(0.0));
        assertThrows(IllegalArgumentException.class, () -> FieldWorld.builder().robotHeight(Double.NaN));
    }

    // ---------------------------------------------------------------------------- floor grip

    @Test
    void aPieceBouncingOnTheFloorKeepsTheConfiguredShareOfItsHorizontalSpeed() {
        GamePiecePhysicsConfig physics = GamePiecePhysicsConfig.builder()
                .pieceToFloorTangentialRetention(0.6).build();
        GamePieceTracker tracker = trackerHoldingOne(FieldWorld.builder().physics(physics).build());
        tracker.launch(0, "spill", 0.0, 0.0, 30.0, 50.0, -20.0, 0.0);
        for (int tick = 0; tick < 50; tick++) {
            tracker.advance(DT);
            GamePiece piece = tracker.getPiece(0);
            if (piece.getVelocityZInchesPerSecond() > 0.0) {
                assertEquals(30.0, piece.getVelocityXInchesPerSecond(), 1e-9, "0.6 of 50 in/s");
                assertEquals(-12.0, piece.getVelocityYInchesPerSecond(), 1e-9, "0.6 of -20 in/s");
                return;
            }
        }
        throw new AssertionError("the piece never bounced");
    }

    @Test
    void theFloorKeepsAllHorizontalSpeedByDefault() {
        assertEquals(1.0, GamePiecePhysicsConfig.defaults().getPieceToFloorTangentialRetention(), 0.0);
        assertThrows(IllegalArgumentException.class,
                () -> GamePiecePhysicsConfig.builder().pieceToFloorTangentialRetention(1.2));
    }
}
