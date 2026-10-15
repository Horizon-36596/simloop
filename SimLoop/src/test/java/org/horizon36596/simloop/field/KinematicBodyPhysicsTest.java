package org.horizon36596.simloop.field;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

import org.junit.jupiter.api.Test;

/**
 * The 3-D contact model: pieces resting in, sliding out of and stacking inside a {@link KinematicBody}.
 *
 * <p>The body most of these use is an open-topped tray hinged on field +X at {@link #PIVOT}: a floor, a back
 * wall at local {@code -Y}, two side walls, and no wall at local {@code +Y} - the mouth. At a positive angle
 * the mouth end is raised, so a ball rolls to the back wall and stays; at a negative angle the mouth end is the
 * low end and the ball rolls out. Its one cell is the space above the floor between the walls.
 */
class KinematicBodyPhysicsTest {

    private static final double RADIUS = 1.4;
    private static final double[] PIVOT = {0.0, 0.0, 20.0};
    private static final double[] X_AXIS = {1.0, 0.0, 0.0};
    private static final String TRAY = "tray";
    private static final String CELL = "trayCell";
    private static final double TICK = 0.02;
    private static final double THIRTY_DEG = StrictMath.toRadians(30.0);

    // ------------------------------------------------------------------------------------------ shapes

    @Test
    void aBoxTouchesABallOnAFaceAnEdgeAndACorner() {
        ConvexShape box = ConvexShape.box("b", -1, 1, -1, 1, -1, 1);
        double[] out = new double[4];

        assertTrue(box.contact(0, 0, 2.0, 1.5, out), "a ball 1 in above the top face, radius 1.5");
        assertEquals(0.0, out[0], 1e-12);
        assertEquals(1.0, out[2], 1e-12);
        assertEquals(0.5, out[3], 1e-12);

        assertFalse(box.contact(0, 0, 2.6, 1.5, out), "1.6 in above the face is clear of a 1.5 in ball");

        // Off an edge (x and z past the box): the normal is diagonal and the gap is the distance to the edge.
        double off = 1.0 + 1.0 / StrictMath.sqrt(2.0);
        assertTrue(box.contact(off, 0, off, 1.2, out));
        assertEquals(1.0 / StrictMath.sqrt(2.0), out[0], 1e-9);
        assertEquals(1.0 / StrictMath.sqrt(2.0), out[2], 1e-9);
        assertEquals(0.2, out[3], 1e-9);

        // Off a corner by 1 in along the diagonal: touched only by a ball bigger than 1 in.
        double corner = 1.0 + 1.0 / StrictMath.sqrt(3.0);
        assertFalse(box.contact(corner, corner, corner, 0.99, out));
        assertTrue(box.contact(corner, corner, corner, 1.01, out));

        // A centre inside goes out through the nearest face.
        assertTrue(box.contact(0.9, 0, 0, 0.5, out));
        assertEquals(1.0, out[0], 1e-12);
        assertEquals(0.6, out[3], 1e-12);
    }

    @Test
    void aShapeRefusesInwardFacesAndNonConvexCorners() {
        double[][] corners = {{0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
        int[][] outward = {{0, 2, 1}, {0, 1, 3}, {0, 3, 2}, {1, 2, 3}};
        new ConvexShape("tetra", corners, outward);
        int[][] inward = {{0, 1, 2}, {0, 1, 3}, {0, 3, 2}, {1, 2, 3}};
        assertThrows(IllegalArgumentException.class, () -> new ConvexShape("tetra", corners, inward));
    }

    @Test
    void aBodyTurnsPointsAboutItsAxisAndBack() {
        KinematicBody body = tray();
        double[] field = new double[3];
        double[] back = new double[3];
        body.toField(THIRTY_DEG, 0.0, 10.0, 0.0, field);
        assertEquals(10.0 * StrictMath.cos(THIRTY_DEG), field[1], 1e-12, "local +Y turns toward +Z");
        assertEquals(20.0 + 10.0 * StrictMath.sin(THIRTY_DEG), field[2], 1e-12);
        body.toLocal(THIRTY_DEG, field[0], field[1], field[2], back);
        assertEquals(0.0, back[0], 1e-12);
        assertEquals(10.0, back[1], 1e-12);
        assertEquals(0.0, back[2], 1e-12);

        double[] velocity = new double[3];
        body.surfaceVelocity(2.0, 0.0, 10.0, 20.0, velocity);
        assertEquals(20.0, velocity[2], 1e-12, "omega x r: 2 rad/s at 10 in moves 20 in/s, +Y end going up");
    }

    // ------------------------------------------------------------------------------------- resting in

    @Test
    void aBallDroppedIntoATiltedTrayIsScoredInItsCellAndRestsAgainstTheBackWall() {
        GamePieceTracker tracker = trackerWithTray();
        tracker.setBodyMotion(TRAY, THIRTY_DEG, 0.0);
        double[] above = new double[3];
        tracker.getWorld().getKinematicBody(TRAY).toField(THIRTY_DEG, 0.0, 0.0, 12.0, above);
        int id = tracker.addPieceAt("ball", above[0], above[1], above[2], RADIUS);
        assertEquals(PossessionState.IN_FLIGHT, tracker.getPiece(id).getState(), "starts above the cell, falling");

        List<String> log = run(tracker, 3.0);

        assertTrue(log.contains("0 IN_FLIGHT->SCORED " + CELL), "came down into the cell: " + log);
        GamePiece ball = tracker.getPiece(id);
        assertEquals(PossessionState.SCORED, ball.getState());
        assertEquals(CELL, ball.getSeatedCellName());
        assertTrue(ball.isVisibleOnField(), "a ball resting in a cell is drawn");
        assertTrue(ball.isSleeping(), "settled and asleep after 3 s");
        assertEquals(Collections.singletonList(id), tracker.getScoredPieceIdsIn(CELL));
        assertEquals(1, tracker.getScoredCountIn(CELL));

        double[] local = localOf(tracker, ball);
        assertEquals(-6.0 + RADIUS, local[1], 0.05, "rolled down to the back wall");
        assertEquals(RADIUS, local[2], 0.05, "on the floor");
    }

    @Test
    void aPieceAddedInsideACellStartsScoredThereAndSettles() {
        GamePieceTracker tracker = trackerWithTray();
        tracker.setBodyMotion(TRAY, THIRTY_DEG, 0.0);
        double[] inside = new double[3];
        tracker.getWorld().getKinematicBody(TRAY).toField(THIRTY_DEG, 0.0, 3.0, 2.0, inside);
        int id = tracker.addPieceAt("ball", inside[0], inside[1], inside[2], RADIUS);

        assertEquals(PossessionState.SCORED, tracker.getPiece(id).getState());
        assertEquals(1, tracker.getScoredCountIn(CELL));
        List<String> log = run(tracker, 3.0);
        assertTrue(log.isEmpty(), "it never left: " + log);
        assertEquals(-6.0 + RADIUS, localOf(tracker, tracker.getPiece(id))[1], 0.05);
    }

    @Test
    void aBallSlidesOutOfTheMouthWhenTheTrayTipsPastLevelAndLandsOnTheFloor() {
        GamePieceTracker tracker = trackerWithTray();
        int id = settledBall(tracker);

        List<String> log = tipTo(tracker, -THIRTY_DEG, 2.0);
        log.addAll(run(tracker, 3.0));

        assertTrue(log.contains(id + " SCORED->IN_FLIGHT " + CELL + "Out"), "rolled out of the mouth: " + log);
        assertTrue(log.contains(id + " IN_FLIGHT->LOOSE landed"), "and fell to the floor: " + log);
        GamePiece ball = tracker.getPiece(id);
        assertEquals(PossessionState.LOOSE, ball.getState());
        assertTrue(ball.getFieldYInches() > 4.0, "left by the mouth, at local +Y, not through the floor or back");
        assertEquals(0, tracker.getScoredCountIn(CELL));
        assertEquals(0.0, ball.getSpinXRadPerSecond(), 0.0, "spin is not modelled on the floor");
    }

    @Test
    void aBallStaysInWhileTheTrayTipsOnlyPartWay() {
        GamePieceTracker tracker = trackerWithTray();
        int id = settledBall(tracker);
        List<String> log = tipTo(tracker, StrictMath.toRadians(10.0), 1.0);
        log.addAll(run(tracker, 2.0));
        assertTrue(log.isEmpty(), "still uphill of the mouth at +10 degrees: " + log);
        assertEquals(CELL, tracker.getPiece(id).getSeatedCellName());
    }

    @Test
    void aStackOfBallsInAColumnSettlesAndStaysStill() {
        GamePieceTracker tracker = new GamePieceTracker(0, FieldWorld.builder().kinematicBody(column()).build());
        int count = 5;
        for (int i = 0; i < count; i++) {
            // Dropped one above the other, each nudged a little off-centre so the stack has to settle.
            tracker.addPieceAt("ball", 0.05 * (i % 2 == 0 ? 1 : -1), 0.03 * i, 2.0 + 3.2 * i, RADIUS);
        }
        run(tracker, 4.0);

        double[][] settled = positions(tracker);
        List<Double> heights = new ArrayList<>();
        for (GamePiece piece : tracker.getPieces()) {
            assertTrue(piece.isSleeping(), piece + " asleep");
            assertEquals("column", piece.getSeatedCellName());
            heights.add(piece.getFieldZInches());
        }
        Collections.sort(heights);
        assertEquals(RADIUS, heights.get(0), 0.05, "bottom ball on the floor");
        for (int i = 1; i < count; i++) {
            double gap = heights.get(i) - heights.get(i - 1);
            assertTrue(gap > 2.0 * RADIUS * 0.9 && gap < 2.0 * RADIUS + 0.05, "stacked, not sunk: gap " + gap);
        }
        for (int i = 0; i < count; i++) {
            for (int j = i + 1; j < count; j++) {
                double[] a = settled[i];
                double[] b = settled[j];
                double d = StrictMath.sqrt((a[0] - b[0]) * (a[0] - b[0]) + (a[1] - b[1]) * (a[1] - b[1])
                        + (a[2] - b[2]) * (a[2] - b[2]));
                assertTrue(d > 2.0 * RADIUS - 0.05, "balls " + i + " and " + j + " overlap by " + (2 * RADIUS - d));
            }
        }

        run(tracker, 3.0);
        double[][] later = positions(tracker);
        for (int i = 0; i < count; i++) {
            assertTrue(Arrays.equals(settled[i], later[i]), "ball " + i + " moved after settling");
        }
    }

    @Test
    void energyNeverGrowsAsBallsPileIntoATray() {
        GamePieceTracker tracker = trackerWithTray();
        tracker.setBodyMotion(TRAY, THIRTY_DEG, 0.0);
        double[] at = new double[3];
        for (int i = 0; i < 8; i++) {
            tracker.getWorld().getKinematicBody(TRAY).toField(THIRTY_DEG, -3.3 + 3.0 * (i % 3), -2.0 + 3.0 * (i / 3),
                    3.0 + 1.0 * i, at);
            tracker.addPieceAt("ball", at[0], at[1], at[2], RADIUS);
        }
        GamePiecePhysicsConfig config = tracker.getWorld().getPhysics();
        double start = energy(tracker, config);
        double highest = start;
        for (int tick = 0; tick < 250; tick++) {
            tracker.advance(TICK);
            tracker.record("Test");
            highest = Math.max(highest, energy(tracker, config));
        }
        assertTrue(highest <= start * 1.005, "energy rose from " + start + " to " + highest);
        for (GamePiece piece : tracker.getPieces()) {
            assertEquals(CELL, piece.getSeatedCellName(), piece + " came to rest in the tray");
        }
    }

    @Test
    void aFastBallDoesNotPassThroughAThinWall() {
        ConvexShape plate = ConvexShape.box("plate", 9.95, 10.05, -10, 10, 0, 20);
        KinematicBody wall = new KinematicBody("wall", new double[] {0, 0, 0}, X_AXIS,
                Collections.singletonList(plate), Collections.<BodyCell>emptyList(), 0.3, 0.5);
        GamePieceTracker tracker = new GamePieceTracker(1, FieldWorld.builder().kinematicBody(wall).build());
        tracker.addHeldPiece("ball", RADIUS);
        tracker.launch(0, "test", 0.0, 0.0, 10.0, 400.0, 0.0, 0.0);
        for (int tick = 0; tick < 10; tick++) {
            tracker.advance(TICK);
            assertTrue(tracker.getPiece(0).getFieldXInches() < 10.0, "crossed the 0.1 in plate");
        }
        assertTrue(tracker.getPiece(0).getVelocityXInchesPerSecond() < 0.0, "bounced back");
    }

    @Test
    void twoIdenticalRunsAreBitIdentical() {
        String first = scenarioFingerprint();
        String second = scenarioFingerprint();
        assertEquals(first, second);
    }

    @Test
    void aWorldRejectsACellNamedLikeAGoalAndAnUnknownBody() {
        ScoringVolume goal = ScoringVolume.builder(CELL).box(-1, 1, -1, 1, 0, 1).restPoint(0, 0, 0.5).build();
        assertThrows(IllegalArgumentException.class,
                () -> FieldWorld.builder().scoringVolume(goal).kinematicBody(tray()).build());
        GamePieceTracker tracker = trackerWithTray();
        assertThrows(IllegalArgumentException.class, () -> tracker.setBodyMotion("nope", 0.0, 0.0));
        assertNull(tracker.getWorld().findCell("nope"));
    }

    @Test
    void releasingASeatedPieceThrowsItOutOfItsCell() {
        GamePieceTracker tracker = trackerWithTray();
        int id = settledBall(tracker);
        tracker.releaseScoredPiece(id, "byHand", 0.0, 30.0, 40.0, 0.0, 0.0, 0.0);
        assertEquals(PossessionState.IN_FLIGHT, tracker.getPiece(id).getState());
        assertNull(tracker.getPiece(id).getSeatedCellName());
        assertEquals(0, tracker.getScoredCountIn(CELL));
    }

    // ------------------------------------------------------------------------------- materials

    /**
     * A shape given its own contact numbers ({@link KinematicBody#withShapeMaterial}) uses them; the body's other
     * shapes keep the body's. Two plates on one fixed body, restitution 0.5; the {@code +X} one is made dead (0).
     */
    @Test
    void aShapeWithItsOwnMaterialBouncesByItAndTheRestByTheBody() {
        KinematicBody plates = new KinematicBody("plates", new double[] {0, 0, 0}, X_AXIS, Arrays.asList(
                ConvexShape.box("dead", 9.95, 10.05, -10, 10, 0, 40),
                ConvexShape.box("lively", -10.05, -9.95, -10, 10, 0, 40)), Collections.<BodyCell>emptyList(),
                0.5, 0.0).withShapeMaterial("dead", 0.0, 0.0);
        assertEquals(0.0, plates.getShapeRestitution(0), 0.0);
        assertEquals(0.5, plates.getShapeRestitution(1), 0.0);
        assertEquals(0.5, plates.getRestitution(), 0.0, "the body's own number is unchanged");
        GamePiecePhysicsConfig floating = GamePiecePhysicsConfig.builder().gravityInchesPerS2(1e-9).build();
        GamePieceTracker tracker = new GamePieceTracker(2,
                FieldWorld.builder().physics(floating).kinematicBody(plates).build());
        int toDead = tracker.addHeldPiece("ball", RADIUS);
        int toLively = tracker.addHeldPiece("ball", RADIUS);
        tracker.launch(toDead, "test", 0.0, -5.0, 20.0, 100.0, 0.0, 0.0);
        tracker.launch(toLively, "test", 0.0, 5.0, 20.0, -100.0, 0.0, 0.0);
        for (int tick = 0; tick < 10; tick++) {
            tracker.advance(TICK);
        }
        assertEquals(0.0, tracker.getPiece(toDead).getVelocityXInchesPerSecond(), 1e-9, "stopped dead");
        assertEquals(50.0, tracker.getPiece(toLively).getVelocityXInchesPerSecond(), 1e-9, "came back at half speed");
        assertThrows(IllegalArgumentException.class, () -> plates.withShapeMaterial("nope", 0.1, 0.1));
        assertThrows(IllegalArgumentException.class, () -> plates.withShapeMaterial("dead", 1.1, 0.1));
        assertThrows(IllegalArgumentException.class, () -> plates.withShapeMaterial("dead", 0.1, -0.1));
    }

    /** The 3-D model's own contact numbers follow the flat model's until a season sets them apart. */
    @Test
    void theThreeDContactNumbersFollowThePlainOnesUntilSet() {
        GamePiecePhysicsConfig plain = GamePiecePhysicsConfig.builder().pieceToPieceRestitution(0.7)
                .pieceToWallRestitution(0.4).pieceToObstacleRestitution(0.3).build();
        assertEquals(0.7, plain.getPieceToPieceRestitution3d(), 0.0);
        assertEquals(0.4, plain.getPieceToWallRestitution3d(), 0.0);
        assertEquals(0.3, plain.getPieceToObstacleRestitution3d(), 0.0);
        assertFalse(plain.hasPieceToFloorFriction3d());
        GamePiecePhysicsConfig apart = GamePiecePhysicsConfig.builder().pieceToPieceRestitution(0.7)
                .pieceToPieceRestitution3d(0.55).pieceToWallRestitution3d(0.275).pieceToObstacleRestitution3d(0.2)
                .pieceToFloorFriction3d(0.6).build();
        assertEquals(0.7, apart.getPieceToPieceRestitution(), 0.0);
        assertEquals(0.55, apart.getPieceToPieceRestitution3d(), 0.0);
        assertEquals(0.275, apart.getPieceToWallRestitution3d(), 0.0);
        assertEquals(0.2, apart.getPieceToObstacleRestitution3d(), 0.0);
        assertTrue(apart.hasPieceToFloorFriction3d());
        assertEquals(0.6, apart.getPieceToFloorFriction3d(), 0.0);
        assertThrows(IllegalArgumentException.class, () -> GamePiecePhysicsConfig.builder().pieceToFloorFriction3d(-1));
        assertThrows(IllegalArgumentException.class,
                () -> GamePiecePhysicsConfig.builder().pieceToPieceRestitution3d(1.5));
    }

    /**
     * With floor friction in the 3-D model, a ball that lands sliding leaves rolling: a thin shell ({@code k = 2/3})
     * keeps {@code 1 / (1 + 1/k) = 0.6} of its horizontal speed on the first bounce, and then rolls, so its later
     * bounces cost it almost nothing (only the spin its damping took in the air). The flat model's retention takes
     * the same share on every bounce.
     */
    @Test
    void floorFrictionLeavesALandedBallRollingAndItsLaterBouncesKeepItsSpeed() {
        double withFriction = speedOnceLanded(GamePiecePhysicsConfig.builder().pieceToFloorRestitution(0.5)
                .pieceToFloorFriction3d(0.6).build());
        double withRetention = speedOnceLanded(GamePiecePhysicsConfig.builder().pieceToFloorRestitution(0.5)
                .pieceToFloorTangentialRetention(0.6).build());
        assertTrue(withFriction > 50.0 && withFriction <= 60.0 + 1e-9,
                "the first bounce leaves 60 in/s, the rest keep nearly all of it: " + withFriction);
        assertTrue(withRetention < 0.6 * 0.6 * 100.0, "retention takes 40 % every bounce: " + withRetention);
    }

    /** A ball thrown down at the floor at 100 in/s each way, in a world with a body far off; its speed once LOOSE. */
    private static double speedOnceLanded(GamePiecePhysicsConfig physics) {
        KinematicBody far = new KinematicBody("far", new double[] {500.0, 500.0, 0.0}, X_AXIS,
                Collections.singletonList(ConvexShape.box("b", -1, 1, -1, 1, -1, 1)), Collections.<BodyCell>emptyList(),
                0.1, 0.5);
        GamePieceTracker tracker = new GamePieceTracker(1, FieldWorld.builder().physics(physics).kinematicBody(far)
                .build());
        int id = tracker.addHeldPiece("ball", RADIUS);
        tracker.launch(id, "test", -60.0, 0.0, RADIUS + 1.0, 100.0, 0.0, -100.0);
        for (int tick = 0; tick < 200 && tracker.getPiece(id).getState() != PossessionState.LOOSE; tick++) {
            tracker.advance(TICK);
        }
        assertEquals(PossessionState.LOOSE, tracker.getPiece(id).getState(), "it settled on the floor");
        return tracker.getPiece(id).getVelocityXInchesPerSecond();
    }

    // ---------------------------------------------------------------------------------------- helpers

    /** The open tray described in the class javadoc; cell seat depth 1 in. */
    private static KinematicBody tray() {
        List<ConvexShape> shapes = Arrays.asList(
                ConvexShape.box("floor", -6, 6, -7, 6, -1, 0),
                ConvexShape.box("back", -6, 6, -7, -6, 0, 8),
                ConvexShape.box("sideNeg", -6, -5, -6, 6, 0, 8),
                ConvexShape.box("sidePos", 5, 6, -6, 6, 0, 8));
        BodyCell cell = new BodyCell(CELL, -5, 5, -6, 6, 0, 8, 1.0);
        return new KinematicBody(TRAY, PIVOT, X_AXIS, shapes, Collections.singletonList(cell), 0.15, 0.6);
    }

    /** A fixed square column 3.0 in across, a little wider than a 2.8 in ball, on the floor. */
    private static KinematicBody column() {
        List<ConvexShape> shapes = Arrays.asList(
                ConvexShape.box("floor", -3, 3, -3, 3, -1, 0),
                ConvexShape.box("wallXNeg", -3, -1.5, -3, 3, 0, 20),
                ConvexShape.box("wallXPos", 1.5, 3, -3, 3, 0, 20),
                ConvexShape.box("wallYNeg", -1.5, 1.5, -3, -1.5, 0, 20),
                ConvexShape.box("wallYPos", -1.5, 1.5, 1.5, 3, 0, 20));
        BodyCell cell = new BodyCell("column", -1.5, 1.5, -1.5, 1.5, 0, 20, 0.0);
        return new KinematicBody("column", new double[] {0, 0, 0}, X_AXIS, shapes, Collections.singletonList(cell),
                0.15, 0.6);
    }

    private static GamePieceTracker trackerWithTray() {
        return new GamePieceTracker(1, FieldWorld.builder().kinematicBody(tray()).build());
    }

    /** One ball dropped into the tray at +30 degrees and left to settle against the back wall. */
    private static int settledBall(GamePieceTracker tracker) {
        tracker.setBodyMotion(TRAY, THIRTY_DEG, 0.0);
        double[] inside = new double[3];
        tracker.getWorld().getKinematicBody(TRAY).toField(THIRTY_DEG, 0.0, 0.0, 3.0, inside);
        int id = tracker.addPieceAt("ball", inside[0], inside[1], inside[2], RADIUS);
        run(tracker, 2.0);
        assertTrue(tracker.getPiece(id).isSleeping());
        return id;
    }

    /** Turn the tray at a steady rate from where it is to {@code target} over {@code seconds}; the transitions. */
    private static List<String> tipTo(GamePieceTracker tracker, double target, double seconds) {
        double start = tracker.getBodyAngleRad(TRAY);
        double rate = (target - start) / seconds;
        int ticks = (int) StrictMath.round(seconds / TICK);
        List<String> log = new ArrayList<>();
        for (int tick = 0; tick < ticks; tick++) {
            tracker.setBodyMotion(TRAY, start + rate * tick * TICK, rate);
            tracker.advance(TICK);
            log.addAll(tracker.getUnloggedTransitions());
            tracker.record("Test");
        }
        tracker.setBodyMotion(TRAY, target, 0.0);
        return log;
    }

    private static List<String> run(GamePieceTracker tracker, double seconds) {
        List<String> log = new ArrayList<>();
        int ticks = (int) StrictMath.round(seconds / TICK);
        for (int tick = 0; tick < ticks; tick++) {
            tracker.advance(TICK);
            log.addAll(tracker.getUnloggedTransitions());
            tracker.record("Test");
        }
        return log;
    }

    private static double[] localOf(GamePieceTracker tracker, GamePiece piece) {
        double[] local = new double[3];
        tracker.getWorld().getKinematicBody(TRAY).toLocal(tracker.getBodyAngleRad(TRAY), piece.getFieldXInches(),
                piece.getFieldYInches(), piece.getFieldZInches(), local);
        return local;
    }

    private static double[][] positions(GamePieceTracker tracker) {
        double[][] out = new double[tracker.getPieces().size()][];
        for (GamePiece piece : tracker.getPieces()) {
            out[piece.getId()] = new double[] {piece.getFieldXInches(), piece.getFieldYInches(), piece.getFieldZInches()};
        }
        return out;
    }

    /** Kinetic (moving and spinning) plus potential energy per unit mass, summed over every piece, in in^2/s^2. */
    private static double energy(GamePieceTracker tracker, GamePiecePhysicsConfig config) {
        double total = 0.0;
        double k = config.getPieceInertiaFactor();
        for (GamePiece p : tracker.getPieces()) {
            double v2 = p.getVelocityXInchesPerSecond() * p.getVelocityXInchesPerSecond()
                    + p.getVelocityYInchesPerSecond() * p.getVelocityYInchesPerSecond()
                    + p.getVelocityZInchesPerSecond() * p.getVelocityZInchesPerSecond();
            double w2 = p.getSpinXRadPerSecond() * p.getSpinXRadPerSecond()
                    + p.getSpinYRadPerSecond() * p.getSpinYRadPerSecond()
                    + p.getSpinZRadPerSecond() * p.getSpinZRadPerSecond();
            total += 0.5 * v2 + 0.5 * k * p.getRadiusInches() * p.getRadiusInches() * w2
                    + config.getGravityInchesPerS2() * p.getFieldZInches();
        }
        return total;
    }

    /** Six balls into the tray, settle, tip it out, and print every transition and the exact final state. */
    private static String scenarioFingerprint() {
        GamePieceTracker tracker = trackerWithTray();
        tracker.setBodyMotion(TRAY, THIRTY_DEG, 0.0);
        double[] at = new double[3];
        for (int i = 0; i < 6; i++) {
            tracker.getWorld().getKinematicBody(TRAY).toField(THIRTY_DEG, -3.0 + 1.2 * i, 2.0, 6.0 + 1.5 * i, at);
            tracker.addPieceAt("ball", at[0], at[1], at[2], RADIUS);
        }
        StringBuilder out = new StringBuilder();
        out.append(run(tracker, 2.0));
        out.append(tipTo(tracker, -THIRTY_DEG, 2.0));
        out.append(run(tracker, 2.0));
        for (GamePiece p : tracker.getPieces()) {
            out.append('|').append(p.getState()).append(' ')
                    .append(Double.doubleToLongBits(p.getFieldXInches())).append(' ')
                    .append(Double.doubleToLongBits(p.getFieldYInches())).append(' ')
                    .append(Double.doubleToLongBits(p.getFieldZInches()));
        }
        return out.toString();
    }
}
