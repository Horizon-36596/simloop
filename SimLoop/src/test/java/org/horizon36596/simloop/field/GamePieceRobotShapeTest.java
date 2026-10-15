package org.horizon36596.simloop.field;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Arrays;
import java.util.Collections;

import org.junit.jupiter.api.Test;

/**
 * INVARIANT-tier (domain R9) — the Contract: <b>a robot built from parts is solid where its parts are and open
 * where they are not. A ball on the floor rolls under a part whose underside is above the ball's top and stops
 * against the part behind it; a ball in the air bounces off the face of the part it flies into; and a robot
 * standing still is a solid box from the floor up for both.</b>
 *
 * <p>The test robot: a low core 12 in long and 16 in wide up to 2.83 in, and an upper deck 18 in long over it
 * from 2.83 to 5 in, so each end has a 3 in deep slot under the deck: an intake mouth. Inches, field frame,
 * no clock (domain R5).
 */
class GamePieceRobotShapeTest {

    private static final double DT = 0.02;
    private static final double RADIUS = 1.4;

    private static final RobotShape MOUTHED = new RobotShape(Arrays.asList(
            OrientedBox.axisAligned("core", 0, 0, 1.415, 6, 8, 1.415),
            OrientedBox.axisAligned("deck", 0, 0, 3.915, 9, 8, 1.085)));

    private static GamePieceTracker trackerWithShape() {
        GamePieceTracker tracker = new GamePieceTracker(4, FieldWorld.builder()
                .physics(GamePiecePhysicsConfig.builder().pieceToRobotRestitution(0.0).build()).build());
        tracker.setRobotShape(MOUTHED);
        tracker.updateRobotPose(0.0, 0.0, 0.0);
        return tracker;
    }

    @Test
    void aBallRollsUnderTheDeckAndStopsAgainstTheCore() {
        GamePieceTracker tracker = trackerWithShape();
        int ball = tracker.addLoosePiece(20.0, 0.0, RADIUS);
        tracker.getPiece(ball).setVelocityInchesPerSecond(-60.0, 0.0, 0.0);
        for (int tick = 0; tick < 50; tick++) {
            tracker.advance(DT);
        }
        assertEquals(6.0 + RADIUS, tracker.getPiece(ball).getFieldXInches(), 1e-6,
                "the deck's underside (2.83 in) is above the ball's top (2.8 in): it reaches the core face");
    }

    @Test
    void aBallAtTheSideRestsAgainstTheRealSide() {
        GamePieceTracker tracker = trackerWithShape();
        int ball = tracker.addLoosePiece(0.0, 20.0, RADIUS);
        tracker.getPiece(ball).setVelocityInchesPerSecond(0.0, -60.0, 0.0);
        for (int tick = 0; tick < 50; tick++) {
            tracker.advance(DT);
        }
        assertEquals(8.0 + RADIUS, tracker.getPiece(ball).getFieldYInches(), 1e-6);
    }

    @Test
    void aRobotDrivingAtABallPushesItWithItsCoreNotItsDeck() {
        GamePieceTracker tracker = trackerWithShape();
        int ball = tracker.addLoosePiece(10.0, 0.0, RADIUS);   // under the deck already, ahead of the core
        double x = 0.0;
        for (int tick = 0; tick < 25; tick++) {
            x += 30.0 * DT;
            tracker.updateRobotPose(x, 0.0, 0.0);
            tracker.advance(DT);
            assertTrue(tracker.getPiece(ball).getFieldXInches() >= x + 6.0 + RADIUS - 1e-6,
                    "tick " + tick + ": never inside the core");
        }
        assertTrue(tracker.getPiece(ball).getFieldXInches() > 10.0 + 5.0, "it was pushed ahead");
    }

    @Test
    void aBallInTheAirBouncesOffTheDecksFace() {
        GamePieceTracker tracker = new GamePieceTracker(4, FieldWorld.builder()
                .physics(GamePiecePhysicsConfig.builder().pieceToRobotRestitution(0.5)
                        .gravityInchesPerS2(1.0).build()).build());
        tracker.setRobotShape(MOUTHED);
        tracker.updateRobotPose(0.0, 0.0, 0.0);
        tracker.addHeldPiece("ball", RADIUS);
        tracker.launch(0, "test", 30.0, 0.0, 4.0, -150.0, 0.0, 0.0);
        tracker.getPiece(0).setLeavingRobot(false);
        boolean bounced = false;
        for (int tick = 0; tick < 25; tick++) {
            tracker.advance(DT);
            GamePiece piece = tracker.getPiece(0);
            assertTrue(piece.getFieldXInches() >= 9.0 + RADIUS - 1e-9,
                    "tick " + tick + ": never inside the deck, x = " + piece.getFieldXInches());
            bounced |= piece.getVelocityXInchesPerSecond() > 0.0;
        }
        assertTrue(bounced);
    }

    @Test
    void aStandingRobotStopsRollingAndFlyingBalls() {
        GamePieceTracker tracker = new GamePieceTracker(4, FieldWorld.builder()
                .physics(GamePiecePhysicsConfig.builder().pieceToRobotRestitution(0.0).build()).build());
        tracker.setStaticRobots(Collections.singletonList(new StaticRobot("partner", 0, 0, 0.3, 17.8, 16.96, 15)));
        int rolling = tracker.addLoosePiece(-30.0, 0.0, RADIUS);
        tracker.getPiece(rolling).setVelocityInchesPerSecond(80.0, 0.0, 0.0);
        tracker.updateRobotPose(100.0, 100.0, 0.0);
        tracker.addHeldPiece("ball", RADIUS);
        int dropped = tracker.getPieces().size() - 1;
        tracker.launch(dropped, "test", 1.0, 2.0, 40.0, 0.0, 0.0, 0.0);
        double lowest = Double.POSITIVE_INFINITY;
        for (int tick = 0; tick < 60; tick++) {
            tracker.advance(DT);
            OrientedBox box = tracker.getStaticRobots().get(0).toBox();
            double[] near = box.closestPoint(tracker.getPiece(rolling).getFieldXInches(),
                    tracker.getPiece(rolling).getFieldYInches(), tracker.getPiece(rolling).getFieldZInches());
            double gap = StrictMath.hypot(near[0] - tracker.getPiece(rolling).getFieldXInches(),
                    near[1] - tracker.getPiece(rolling).getFieldYInches());
            assertTrue(gap >= RADIUS - 1e-6, "tick " + tick + ": the rolling ball never enters the partner");
            lowest = Math.min(lowest, tracker.getPiece(dropped).getFieldZInches());
        }
        assertTrue(lowest >= 15.0 + RADIUS - 1e-6, "the dropped ball stays on the partner's top, z " + lowest);
    }

    @Test
    void aWorldSwapKeepsTheShapeAndTheStandingRobots() {
        GamePieceTracker tracker = trackerWithShape();
        tracker.setStaticRobots(Collections.singletonList(new StaticRobot("partner", 40, 0, 0, 17.8, 16.96, 15)));
        tracker.replaceWorld(FieldWorld.unbounded());
        assertEquals(MOUTHED, tracker.getRobotShape());
        assertEquals(1, tracker.getStaticRobots().size());
    }
}
