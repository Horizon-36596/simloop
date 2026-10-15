package org.horizon36596.simloop.field;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Arrays;
import java.util.List;

import org.junit.jupiter.api.Test;

/**
 * A {@link KinematicBody} made dynamic with {@link HingeDynamics}, and pieces with a mass per kind: a see-saw
 * turned by gravity on its own mass and on the pieces in its cups, stopped by its hard limits.
 *
 * <p>The see-saw: a plank 40 in long along local Y, hinged on field +X at {@link #PIVOT}, with a closed cup at
 * each end (local Y 8 to 20 either side, walls 8 in high). Its own mass is {@link #PLANK_MASS} at
 * {@link #COM_HEIGHT} above the hinge, so it is bi-stable: at either 30 degree stop its own weight holds it there,
 * with a torque of {@code M g d sin 30}. At -30 degrees the {@code -Y} cup is up, and a ball in it rolls to its
 * inner wall, about 7.4 in out from the hinge horizontally.
 *
 * <p>Worked numbers: holding torque {@code 10 g 1 0.5 = 5 g}. A ball against the up cup's inner wall
 * ({@code (0, -9.4, 1.4)} local, at -30 degrees {@code 7.44} in out) weighs {@code 7.44 m g} about the hinge: a
 * {@code 0.4} piece ({@code 3.0 g}) cannot tip it, a {@code 1.0} piece ({@code 7.4 g}) can, and two {@code 0.4}
 * pieces ({@code 6.0 g}) can. The margin is not the whole story: as the see-saw comes off its stop its own hold
 * shrinks with {@code sin(angle)} while a piece's lever grows, so a piece dropped in hard enough can kick it past
 * the balance point. The pieces here are set down gently, just above the cup's floor.
 */
class HingeDynamicsPhysicsTest {

    private static final double RADIUS = 1.4;
    private static final double[] PIVOT = {0.0, 0.0, 30.0};
    private static final double[] X_AXIS = {1.0, 0.0, 0.0};
    private static final String SEESAW = "seesaw";
    private static final String UP_CUP = "cupMinusY";
    private static final double TICK = 0.02;
    private static final double THIRTY_DEG = StrictMath.toRadians(30.0);
    private static final double PLANK_MASS = 10.0;
    private static final double COM_HEIGHT = 1.0;
    private static final double INERTIA = 1000.0;
    private static final String LIGHT = "light";
    private static final String HEAVY = "heavy";

    // --------------------------------------------------------------------------------------- the body

    @Test
    void anEmptySeesawRestsOnTheStopItsOwnWeightPushesItTo() {
        GamePieceTracker tracker = seesawTracker(seesaw(0.0));
        tracker.setBodyMotion(SEESAW, -THIRTY_DEG, 0.0);
        run(tracker, 2.0);
        assertEquals(-THIRTY_DEG, tracker.getBodyAngleRad(SEESAW), 0.0, "held at the stop, exactly");
        assertEquals(0.0, tracker.getBodyAngularVelocityRadPerS(SEESAW), 0.0, "and still");

        GamePieceTracker nearlyLevel = seesawTracker(seesaw(0.0));
        nearlyLevel.setBodyMotion(SEESAW, StrictMath.toRadians(1.0), 0.0);
        run(nearlyLevel, 3.0);
        assertEquals(THIRTY_DEG, nearlyLevel.getBodyAngleRad(SEESAW), 0.0,
                "a centre of mass above the hinge falls to the near stop and stays: a limit of restitution 0");
        assertEquals(0.0, nearlyLevel.getBodyAngularVelocityRadPerS(SEESAW), 0.0);
    }

    /** A centre of mass below the hinge is a pendulum: period {@code 2 pi sqrt(I / (M g d))}. */
    @Test
    void aBodyHangingBelowItsHingeSwingsAtThePendulumPeriod() {
        double mass = 5.0;
        double drop = 2.0;
        double inertia = 50.0;
        KinematicBody pendulum = plank().withHinge(new HingeDynamics(mass, new double[] {0.0, 0.0, -drop}, inertia,
                0.0, -StrictMath.toRadians(80.0), StrictMath.toRadians(80.0), 0.0));
        GamePieceTracker tracker = seesawTracker(pendulum);
        tracker.setBodyMotion(SEESAW, 0.1, 0.0);

        double tick = 0.005;
        double previous = tracker.getBodyAngleRad(SEESAW);
        double time = 0.0;
        double firstDownCrossing = -1.0;
        double lastDownCrossing = -1.0;
        int downCrossings = 0;
        for (int i = 0; i < 1000; i++) {
            tracker.advance(tick);
            time += tick;
            double angle = tracker.getBodyAngleRad(SEESAW);
            if (previous > 0.0 && angle <= 0.0) {
                double crossing = time - tick + tick * previous / (previous - angle);
                if (downCrossings == 0) {
                    firstDownCrossing = crossing;
                }
                lastDownCrossing = crossing;
                downCrossings++;
            }
            previous = angle;
        }
        double measured = (lastDownCrossing - firstDownCrossing) / (downCrossings - 1);
        double expected = 2.0 * Math.PI * StrictMath.sqrt(inertia
                / (mass * GamePiecePhysicsConfig.STANDARD_GRAVITY_INCHES_PER_S2 * drop));
        assertTrue(downCrossings >= 5, "several periods: " + downCrossings);
        assertEquals(expected, measured, expected * 0.01, "small-angle period, within 1 %");
    }

    @Test
    void theHingeRefusesNonsense() {
        double[] com = {0.0, 0.0, 1.0};
        assertThrows(IllegalArgumentException.class, () -> new HingeDynamics(-1.0, com, 1.0, 0.0, -1.0, 1.0, 0.0));
        assertThrows(IllegalArgumentException.class, () -> new HingeDynamics(1.0, com, 0.0, 0.0, -1.0, 1.0, 0.0));
        assertThrows(IllegalArgumentException.class, () -> new HingeDynamics(1.0, com, 1.0, -1.0, -1.0, 1.0, 0.0));
        assertThrows(IllegalArgumentException.class, () -> new HingeDynamics(1.0, com, 1.0, 0.0, 1.0, -1.0, 0.0));
        assertThrows(IllegalArgumentException.class, () -> new HingeDynamics(1.0, com, 1.0, 0.0, -1.0, 1.0, 1.5));
        assertThrows(IllegalArgumentException.class, () -> plank().withHinge(null));
        assertFalse(plank().isDynamic());
        assertTrue(seesaw(0.0).isDynamic());
    }

    // ------------------------------------------------------------------------- what the pieces do to it

    @Test
    void aLightPieceInTheUpCupDoesNotTipIt() {
        GamePieceTracker tracker = loadedSeesaw(LIGHT);
        run(tracker, 4.0);
        assertEquals(-THIRTY_DEG, tracker.getBodyAngleRad(SEESAW), 1e-9, "3.0 g against a 5 g hold");
        assertEquals(1, tracker.getScoredCountIn(UP_CUP));
    }

    @Test
    void aHeavyPieceTipsItToTheOtherStopWhereItStays() {
        GamePieceTracker tracker = loadedSeesaw(HEAVY);
        double reachedAt = runUntilAngle(tracker, THIRTY_DEG, 8.0);
        assertTrue(reachedAt > 0.0, "7.4 g against a 5 g hold tips it");
        run(tracker, 2.0);
        assertEquals(THIRTY_DEG, tracker.getBodyAngleRad(SEESAW), 0.0, "and it rests on the far stop");
        assertEquals(1, tracker.getScoredCountIn(UP_CUP), "the closed cup kept its ball");
    }

    @Test
    void twoLightPiecesTipItWhereOneDoesNot() {
        GamePieceTracker tracker = loadedSeesaw(LIGHT);
        run(tracker, 2.0);
        assertEquals(-THIRTY_DEG, tracker.getBodyAngleRad(SEESAW), 1e-9);
        dropIntoUpCup(tracker, LIGHT, 3.0);
        assertTrue(runUntilAngle(tracker, THIRTY_DEG, 8.0) > 0.0, "two light pieces outweigh the hold");
    }

    @Test
    void aHeldBodyStaysPutUnderAnyLoadAndTipsWhenLetGo() {
        GamePieceTracker tracker = seesawTracker(seesaw(0.0));
        tracker.setBodyMotion(SEESAW, -THIRTY_DEG, 0.0);
        tracker.setBodyHeld(SEESAW, true);
        assertTrue(tracker.isBodyHeld(SEESAW));
        dropIntoUpCup(tracker, HEAVY, 0.0);
        dropIntoUpCup(tracker, HEAVY, 3.0);
        run(tracker, 3.0);
        assertEquals(-THIRTY_DEG, tracker.getBodyAngleRad(SEESAW), 0.0, "held");
        assertEquals(0.0, tracker.getBodyAngularVelocityRadPerS(SEESAW), 0.0);
        assertTrue(tracker.getPiece(0).isSleeping(), "the load has settled in the held cup");

        tracker.setBodyHeld(SEESAW, false);
        assertTrue(runUntilAngle(tracker, THIRTY_DEG, 8.0) > 0.0, "let go, the sleeping load's weight tips it");
    }

    @Test
    void damperSlowsTheSwing() {
        double undamped = runUntilAngle(loadedSeesaw(HEAVY, seesaw(0.0)), THIRTY_DEG, 20.0);
        double damped = runUntilAngle(loadedSeesaw(HEAVY, seesaw(2.0)), THIRTY_DEG, 20.0);
        assertTrue(undamped > 0.0 && damped > undamped * 1.2,
                "angular damping 2/s lengthens the swing: " + undamped + " s then " + damped + " s");
    }

    @Test
    void twoRunsAreBitIdentical() {
        assertEquals(fingerprint(), fingerprint());
    }

    // ------------------------------------------------------------------------------- mass per kind

    @Test
    void aHeadOnCollisionKeepsMomentumAndTheLighterPieceBouncesHarder() {
        GamePiecePhysicsConfig physics = GamePiecePhysicsConfig.builder()
                .pieceMass(LIGHT, 1.0).pieceMass(HEAVY, 3.0).gravityInchesPerS2(1e-9).build();
        // A body far off, so the 3-D model (which gives flying pieces each other) is on.
        FieldWorld world = FieldWorld.builder().physics(physics).kinematicBody(new KinematicBody("far",
                new double[] {500.0, 500.0, 0.0}, X_AXIS, plankShapes(), null, 0.1, 0.5)).build();
        GamePieceTracker tracker = new GamePieceTracker(2, world);
        int light = tracker.addHeldPiece(LIGHT, RADIUS);
        int heavy = tracker.addHeldPiece(HEAVY, RADIUS);
        tracker.launch(light, "test", -10.0, 0.0, 50.0, 100.0, 0.0, 0.0);
        tracker.launch(heavy, "test", 10.0, 0.0, 50.0, -100.0, 0.0, 0.0);
        run(tracker, 0.2);

        double vLight = tracker.getPiece(light).getVelocityXInchesPerSecond();
        double vHeavy = tracker.getPiece(heavy).getVelocityXInchesPerSecond();
        assertEquals(1.0 * 100.0 + 3.0 * -100.0, 1.0 * vLight + 3.0 * vHeavy, 1e-6, "momentum along X kept");
        assertTrue(vLight < 0.0, "the light one bounced back: " + vLight);
        assertEquals(-3.0, (vLight - 100.0) / (vHeavy + 100.0), 1e-9,
                "and changed speed three times as much, the other way");
        assertEquals(1.0, physics.getPieceMass("unnamed"), 0.0, "a kind with no mass of its own weighs 1");
    }

    // ------------------------------------------------------------------- a piece outside every cell

    /**
     * A piece asleep on the body but in none of its cells weighs on it all the same: its weight goes to what it
     * rests on, not to a cell. A see-saw balanced on its hinge (centre of mass on the axis) is held level while a
     * heavy piece settles on the plank between the cups, 3 in out along {@code +Y}, and goes to sleep there. Let
     * go, the piece's weight turns it {@code +Y} down, to the {@code -30} degree stop.
     */
    @Test
    void aPieceAsleepOnTheBodyOutsideEveryCellStillWeighsOnIt() {
        KinematicBody balanced = plank().withHinge(new HingeDynamics(PLANK_MASS, new double[] {0.0, 0.0, 0.0},
                INERTIA, 0.0, -THIRTY_DEG, THIRTY_DEG, 0.0));
        GamePieceTracker tracker = seesawTracker(balanced);
        tracker.setBodyMotion(SEESAW, 0.0, 0.0);
        tracker.setBodyHeld(SEESAW, true);
        double[] field = new double[3];
        balanced.toField(0.0, 0.0, 3.0, RADIUS + 0.05, field);
        int id = tracker.addPieceAt(HEAVY, field[0], field[1], field[2], RADIUS);
        run(tracker, 1.0);
        GamePiece piece = tracker.getPiece(id);
        assertTrue(piece.isSleeping(), "settled on the plank");
        assertEquals(null, piece.getSeatedCellName(), "in no cup");
        assertEquals(PossessionState.IN_FLIGHT, piece.getState());

        tracker.setBodyHeld(SEESAW, false);
        run(tracker, 3.0);
        assertEquals(-THIRTY_DEG, tracker.getBodyAngleRad(SEESAW), 0.0, "its weight tipped the balanced plank");
    }

    // ---------------------------------------------------------------------------------------- helpers

    private static List<ConvexShape> plankShapes() {
        return Arrays.asList(
                ConvexShape.box("plank", -6, 6, -21, 21, -1, 0),
                ConvexShape.box("endMinus", -6, 6, -21, -20, 0, 8),
                ConvexShape.box("innerMinus", -6, 6, -8, -7, 0, 8),
                ConvexShape.box("innerPlus", -6, 6, 7, 8, 0, 8),
                ConvexShape.box("endPlus", -6, 6, 20, 21, 0, 8),
                ConvexShape.box("sideNeg", -6, -5, -20, 20, 0, 8),
                ConvexShape.box("sidePos", 5, 6, -20, 20, 0, 8));
    }

    private static KinematicBody plank() {
        List<BodyCell> cups = Arrays.asList(
                new BodyCell(UP_CUP, -5, 5, -20, -8, 0, 8, 1.0),
                new BodyCell("cupPlusY", -5, 5, 8, 20, 0, 8, 1.0));
        return new KinematicBody(SEESAW, PIVOT, X_AXIS, plankShapes(), cups, 0.15, 0.6);
    }

    private static KinematicBody seesaw(double damping) {
        return plank().withHinge(new HingeDynamics(PLANK_MASS, new double[] {0.0, 0.0, COM_HEIGHT}, INERTIA, damping,
                -THIRTY_DEG, THIRTY_DEG, 0.0));
    }

    private static GamePieceTracker seesawTracker(KinematicBody body) {
        GamePiecePhysicsConfig physics = GamePiecePhysicsConfig.builder().pieceMass(LIGHT, 0.4).pieceMass(HEAVY, 1.0)
                .build();
        return new GamePieceTracker(1, FieldWorld.builder().physics(physics).kinematicBody(body).build());
    }

    private static GamePieceTracker loadedSeesaw(String kind) {
        return loadedSeesaw(kind, seesaw(0.0));
    }

    /** The see-saw at -30 degrees ({@code -Y} cup up), with one piece of {@code kind} dropped into the up cup. */
    private static GamePieceTracker loadedSeesaw(String kind, KinematicBody body) {
        GamePieceTracker tracker = seesawTracker(body);
        tracker.setBodyMotion(SEESAW, -THIRTY_DEG, 0.0);
        dropIntoUpCup(tracker, kind, 0.0);
        return tracker;
    }

    /** A piece just inside the up cup, {@code across} inches along X from its middle, which rolls to the inner wall. */
    private static void dropIntoUpCup(GamePieceTracker tracker, String kind, double across) {
        double[] field = new double[3];
        tracker.getWorld().getKinematicBody(SEESAW).toField(tracker.getBodyAngleRad(SEESAW), across, -14.0, 2.2,
                field);
        tracker.addPieceAt(kind, field[0], field[1], field[2], RADIUS);
    }

    private static void run(GamePieceTracker tracker, double seconds) {
        int ticks = (int) StrictMath.round(seconds / TICK);
        for (int tick = 0; tick < ticks; tick++) {
            tracker.advance(TICK);
            tracker.record("Test");
        }
    }

    /** Seconds until the body's angle first reaches {@code target}, or -1 if it does not within {@code limit}. */
    private static double runUntilAngle(GamePieceTracker tracker, double target, double limit) {
        int ticks = (int) StrictMath.round(limit / TICK);
        for (int tick = 1; tick <= ticks; tick++) {
            tracker.advance(TICK);
            tracker.record("Test");
            if (tracker.getBodyAngleRad(SEESAW) >= target) {
                return tick * TICK;
            }
        }
        return -1.0;
    }

    private static String fingerprint() {
        GamePieceTracker tracker = loadedSeesaw(HEAVY);
        dropIntoUpCup(tracker, LIGHT, 3.0);
        StringBuilder out = new StringBuilder();
        for (int tick = 0; tick < 200; tick++) {
            tracker.advance(TICK);
            out.append(Double.doubleToLongBits(tracker.getBodyAngleRad(SEESAW))).append(',');
            out.append(tracker.getUnloggedTransitions());
            tracker.record("Test");
        }
        for (GamePiece piece : tracker.getPieces()) {
            out.append('|').append(Double.doubleToLongBits(piece.getFieldXInches()))
                    .append(',').append(Double.doubleToLongBits(piece.getFieldYInches()))
                    .append(',').append(Double.doubleToLongBits(piece.getFieldZInches()));
        }
        return out.toString();
    }
}
