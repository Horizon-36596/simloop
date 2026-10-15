package org.horizon36596.simloop.sensor;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.horizon36596.simloop.fakehardware.FakeDistanceSensor;
import org.horizon36596.simloop.fakehardware.FakeRevColorSensorV3;
import org.horizon36596.simloop.field.FieldWorld;
import org.horizon36596.simloop.field.GamePieceTracker;
import org.horizon36596.simloop.field.ObstacleBox;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * Pins {@link RangeFinder}: a ray from a sensor mount stops at the first wall, obstacle, loose piece or
 * floor, at the distance plane geometry gives by hand. INVARIANT tier (domain R9) for the geometry and the
 * determinism; the sensor hand-off tests are implementation tier.
 *
 * <p>The field is the FTC one: a 144 in square, so the walls are at {@code |x| = 72} and {@code |y| = 72}.
 */
class RangeFinderTest {

    private static final double EPS = 1e-9;
    private static final double BALL_RADIUS = 2.5;
    private static final FieldWorld FIELD = FieldWorld.builder().perimeter(72.0, 72.0).build();

    private static RangeHit castFrom(SensorMount mount, FieldWorld world, GamePieceTracker tracker, double x,
            double y, double heading) {
        return new RangeFinder(mount).cast(world, tracker.getPieces(), x, y, heading);
    }

    private static GamePieceTracker empty(FieldWorld world) {
        return new GamePieceTracker(3, world);
    }

    // ---- walls, at known distances and angles ----

    @Test
    void invariant_aLevelRayFromTheCentreHitsTheWall72InchesAway() {
        RangeHit hit = castFrom(SensorMount.level(0.0, 0.0, 3.0, 0.0), FIELD, empty(FIELD), 0.0, 0.0, 0.0);

        assertEquals(RangeHit.Target.WALL, hit.getTarget());
        assertEquals(72.0, hit.getDistanceInches(), EPS);
        assertEquals("perimeter", hit.getTargetName());
        assertEquals(-1, hit.getPieceId());
    }

    @Test
    void invariant_theRangeIsMeasuredFromTheSensorFaceNotTheRobotCentre() {
        RangeHit hit = castFrom(SensorMount.level(9.0, 0.0, 3.0, 0.0), FIELD, empty(FIELD), 0.0, 0.0, 0.0);

        assertEquals(63.0, hit.getDistanceInches(), EPS);
    }

    @Test
    void invariant_aRayAtThirtyDegreesHitsTheNearWallAtItsSlantDistance() {
        RangeHit hit = castFrom(SensorMount.level(0.0, 0.0, 3.0, 0.0), FIELD, empty(FIELD), 0.0, 0.0,
                Math.PI / 6.0);

        assertEquals(72.0 / Math.cos(Math.PI / 6.0), hit.getDistanceInches(), EPS,
                "the +X wall at 72 / cos 30 = 83.1 in, before the +Y wall at 72 / sin 30 = 144 in");
    }

    @Test
    void invariant_atFortyFiveDegreesTheRayMeetsTheCorner() {
        RangeHit hit = castFrom(SensorMount.level(0.0, 0.0, 3.0, 0.0), FIELD, empty(FIELD), 0.0, 0.0,
                Math.PI / 4.0);

        assertEquals(72.0 * Math.sqrt(2.0), hit.getDistanceInches(), 1e-9);
    }

    @Test
    void theMountsYawAndTheRobotsPoseBothTurnTheRay() {
        RangeHit sideways = castFrom(SensorMount.level(0.0, 0.0, 3.0, Math.PI / 2.0), FIELD, empty(FIELD),
                0.0, 10.0, 0.0);
        assertEquals(62.0, sideways.getDistanceInches(), EPS, "yaw pi/2 at heading 0 looks at +Y, from y = 10");

        RangeHit backwards = castFrom(SensorMount.level(9.0, 0.0, 3.0, 0.0), FIELD, empty(FIELD), 40.0, -10.0,
                Math.PI);
        assertEquals(103.0, backwards.getDistanceInches(), EPS, "facing -X from x = 31: 31 + 72");
    }

    @Test
    void aSensorFaceOnOrPastTheWallReadsZero() {
        RangeHit hit = castFrom(SensorMount.level(9.0, 0.0, 3.0, 0.0), FIELD, empty(FIELD), 63.0, 0.0, 0.0);

        assertEquals(0.0, hit.getDistanceInches(), 0.0);
        assertEquals(RangeHit.Target.WALL, hit.getTarget());
    }

    @Test
    void aRayTiltedUpIsStillStoppedByTheWall() {
        // PLACEHOLDER geometry, pinned so a change to it is a decision: the world has no wall height.
        RangeHit hit = castFrom(new SensorMount(0.0, 0.0, 3.0, 0.0, -Math.PI / 6.0), FIELD, empty(FIELD),
                0.0, 0.0, 0.0);

        assertEquals(RangeHit.Target.WALL, hit.getTarget());
        assertEquals(72.0 / Math.cos(Math.PI / 6.0), hit.getDistanceInches(), EPS);
    }

    @Test
    void aRayTiltedDownHitsTheFloor() {
        RangeHit hit = castFrom(new SensorMount(0.0, 0.0, 4.0, 0.0, Math.PI / 6.0), FIELD, empty(FIELD),
                0.0, 0.0, 0.0);

        assertEquals(RangeHit.Target.FLOOR, hit.getTarget());
        assertEquals(8.0, hit.getDistanceInches(), EPS, "4 in up, 30 degrees down: 4 / sin 30");
    }

    @Test
    void anUnboundedWorldWithNothingInItIsAMiss() {
        FieldWorld open = FieldWorld.unbounded();
        RangeHit hit = castFrom(SensorMount.level(0.0, 0.0, 3.0, 0.0), open, empty(open), 0.0, 0.0, 0.0);

        assertFalse(hit.isHit());
        assertEquals(RangeHit.Target.NOTHING, hit.getTarget());
        assertEquals(Double.POSITIVE_INFINITY, hit.getDistanceInches(), 0.0);
    }

    // ---- obstacles ----

    @Test
    void invariant_anObstacleInTheWayStopsTheRayAtItsNearFace() {
        FieldWorld world = FieldWorld.builder().perimeter(72.0, 72.0)
                .obstacle(new ObstacleBox("goal", 30.0, 40.0, -5.0, 5.0, 0.0, 10.0)).build();
        RangeHit hit = castFrom(SensorMount.level(0.0, 0.0, 3.0, 0.0), world, empty(world), 0.0, 0.0, 0.0);

        assertEquals(RangeHit.Target.OBSTACLE, hit.getTarget());
        assertEquals("goal", hit.getTargetName());
        assertEquals(30.0, hit.getDistanceInches(), EPS);
    }

    @Test
    void aLevelRayPassesUnderAHighObstacleAndBesideAnOffsetOne() {
        FieldWorld world = FieldWorld.builder().perimeter(72.0, 72.0)
                .obstacle(new ObstacleBox("hanging", 30.0, 40.0, -5.0, 5.0, 5.0, 20.0))
                .obstacle(new ObstacleBox("beside", 20.0, 25.0, 3.1, 10.0, 0.0, 10.0)).build();
        RangeHit hit = castFrom(SensorMount.level(0.0, 3.0, 3.0, 0.0), world, empty(world), 0.0, 0.0, 0.0);

        assertEquals(RangeHit.Target.WALL, hit.getTarget(), "under the first box, just beside the second");
        assertEquals(72.0, hit.getDistanceInches(), EPS);
    }

    @Test
    void anObstacleAtAnAngleIsHitAtTheSlantDistanceToItsFace() {
        FieldWorld world = FieldWorld.builder().perimeter(72.0, 72.0)
                .obstacle(new ObstacleBox("wall segment", 20.0, 22.0, -50.0, 50.0, 0.0, 10.0)).build();
        RangeHit hit = castFrom(SensorMount.level(0.0, 0.0, 3.0, 0.0), world, empty(world), 0.0, 0.0,
                Math.toRadians(60.0));

        assertEquals(20.0 / Math.cos(Math.toRadians(60.0)), hit.getDistanceInches(), 1e-9, "40 in");
    }

    // ---- pieces ----

    @Test
    void invariant_aBallOnTheRayIsHitAtItsNearSurface() {
        GamePieceTracker tracker = empty(FIELD);
        int id = tracker.addLoosePiece("ball", 20.0, 0.0, BALL_RADIUS);

        RangeHit atCentreHeight = castFrom(SensorMount.level(0.0, 0.0, BALL_RADIUS, 0.0), FIELD, tracker,
                0.0, 0.0, 0.0);
        assertEquals(RangeHit.Target.PIECE, atCentreHeight.getTarget());
        assertEquals(id, atCentreHeight.getPieceId());
        assertEquals("ball", atCentreHeight.getTargetName());
        assertEquals(17.5, atCentreHeight.getDistanceInches(), EPS);

        // 1.5 in above the ball's centre the slice through the ball is a circle of radius 2.
        RangeHit higher = castFrom(SensorMount.level(0.0, 0.0, 4.0, 0.0), FIELD, tracker, 0.0, 0.0, 0.0);
        assertEquals(18.0, higher.getDistanceInches(), EPS);

        RangeHit over = castFrom(SensorMount.level(0.0, 0.0, 5.5, 0.0), FIELD, tracker, 0.0, 0.0, 0.0);
        assertEquals(RangeHit.Target.WALL, over.getTarget(), "a ray above the ball's top misses it");
    }

    @Test
    void aBallJustBesideTheRayIsMissedAndOneJustInsideIsGrazed() {
        GamePieceTracker beside = empty(FIELD);
        beside.addLoosePiece("ball", 20.0, 2.6, BALL_RADIUS);
        assertEquals(RangeHit.Target.WALL,
                castFrom(SensorMount.level(0.0, 0.0, BALL_RADIUS, 0.0), FIELD, beside, 0.0, 0.0, 0.0).getTarget());

        GamePieceTracker grazing = empty(FIELD);
        grazing.addLoosePiece("ball", 20.0, 2.4, BALL_RADIUS);
        RangeHit hit = castFrom(SensorMount.level(0.0, 0.0, BALL_RADIUS, 0.0), FIELD, grazing, 0.0, 0.0, 0.0);
        assertEquals(20.0 - Math.sqrt(2.5 * 2.5 - 2.4 * 2.4), hit.getDistanceInches(), 1e-9);
    }

    @Test
    void theNearestThingWinsAndAnExactTieGoesToTheLowestId() {
        GamePieceTracker tracker = empty(FIELD);
        tracker.addLoosePiece("far", 30.0, 0.0, BALL_RADIUS);
        int left = tracker.addLoosePiece("left", 10.0, 1.0, BALL_RADIUS);
        tracker.addLoosePiece("right", 10.0, -1.0, BALL_RADIUS);

        RangeHit hit = castFrom(SensorMount.level(0.0, 0.0, BALL_RADIUS, 0.0), FIELD, tracker, 0.0, 0.0, 0.0);
        assertEquals(left, hit.getPieceId(), "two balls mirror-image about the ray: the first added wins");
    }

    @Test
    void aHeldPieceIsInsideTheRobotAndIsNeverHit() {
        GamePieceTracker tracker = empty(FIELD);
        tracker.teleportRobot(0.0, 0.0, 0.0);
        tracker.addHeldPiece("ball", BALL_RADIUS);

        RangeHit hit = castFrom(SensorMount.level(0.0, 0.0, BALL_RADIUS, 0.0), FIELD, tracker, 0.0, 0.0, 0.0);
        assertEquals(RangeHit.Target.WALL, hit.getTarget());
    }

    @Test
    void aPieceInTheAirIsHitWhereItIs() {
        GamePieceTracker tracker = empty(FIELD);
        tracker.teleportRobot(0.0, 0.0, 0.0);
        int id = tracker.addHeldPiece("ball", BALL_RADIUS);
        tracker.launch(id, "launcher", 30.0, 0.0, 12.0, 0.0, 0.0, 0.0);

        RangeHit low = castFrom(SensorMount.level(0.0, 0.0, 3.0, 0.0), FIELD, tracker, 0.0, 0.0, 0.0);
        assertEquals(RangeHit.Target.WALL, low.getTarget(), "a low sensor sees under it");
        RangeHit high = castFrom(SensorMount.level(0.0, 0.0, 12.0, 0.0), FIELD, tracker, 0.0, 0.0, 0.0);
        assertEquals(id, high.getPieceId());
        assertEquals(27.5, high.getDistanceInches(), EPS);
    }

    // ---- handing the result to a sensor ----

    @Test
    void theDistanceSensorReadsTheRayInWholeMillimetresAndOutOfRangePastTwoMetres() {
        RangeFinder front = new RangeFinder(SensorMount.level(9.0, 0.0, 3.0, 0.0));
        FakeDistanceSensor sensor = new FakeDistanceSensor();

        front.apply(sensor, FIELD, Collections.emptyList(), 0.0, 0.0, 0.0);
        assertEquals(1600.0, sensor.getDistance(DistanceUnit.MM), 0.0, "63 in is 1600.2 mm");

        RangeHit far = front.apply(sensor, FIELD, Collections.emptyList(), -60.0, 0.0, 0.0);
        assertEquals(123.0, far.getDistanceInches(), EPS);
        assertFalse(sensor.isTargetInRange(), "123 in is past the REV 2m's 2 m");
        assertEquals(FakeDistanceSensor.REV_2M_NO_TARGET_MM, sensor.getDistance(DistanceUnit.MM), 0.0);

        front.apply(sensor, FieldWorld.unbounded(), Collections.emptyList(), 0.0, 0.0, 0.0);
        assertFalse(sensor.isTargetInRange(), "a miss reads as nothing in range");
    }

    @Test
    void aColourSensorUsedAsAProximitySensorReadsTheRayToo() {
        GamePieceTracker tracker = empty(FIELD);
        tracker.addLoosePiece("ball", 4.5, 0.0, BALL_RADIUS);
        RangeFinder slot = new RangeFinder(SensorMount.level(0.0, 0.0, BALL_RADIUS, 0.0));
        FakeRevColorSensorV3 sensor = new FakeRevColorSensorV3();

        slot.apply(sensor, FIELD, tracker.getPieces(), 0.0, 0.0, 0.0);
        assertEquals(2.0, sensor.getDistance(DistanceUnit.INCH), 0.01);

        slot.apply(sensor, FIELD, Collections.emptyList(), 0.0, 0.0, 0.0);
        assertEquals(6.0, sensor.getDistance(DistanceUnit.INCH), 0.0, "the wall 72 in away is past its reach");
    }

    @Test
    void badArgumentsAreRefused() {
        RangeFinder finder = new RangeFinder(SensorMount.level(0.0, 0.0, 3.0, 0.0));
        List<org.horizon36596.simloop.field.GamePiece> none = Collections.emptyList();

        assertThrows(IllegalArgumentException.class, () -> new RangeFinder(null));
        assertThrows(IllegalArgumentException.class, () -> finder.cast(null, none, 0.0, 0.0, 0.0));
        assertThrows(IllegalArgumentException.class, () -> finder.cast(FIELD, null, 0.0, 0.0, 0.0));
        assertThrows(IllegalArgumentException.class, () -> finder.cast(FIELD, none, Double.NaN, 0.0, 0.0));
        assertThrows(IllegalArgumentException.class,
                () -> finder.apply((FakeDistanceSensor) null, FIELD, none, 0.0, 0.0, 0.0));
    }

    // ---- determinism ----

    @Test
    void invariant_theSameWorldAndPoseGiveTheSameHitEveryRun() {
        assertEquals(sweep(), sweep());
    }

    private static List<String> sweep() {
        FieldWorld world = FieldWorld.builder().perimeter(72.0, 72.0)
                .obstacle(new ObstacleBox("goal", 30.0, 40.0, -5.0, 5.0, 0.0, 10.0)).build();
        GamePieceTracker tracker = new GamePieceTracker(3, world);
        for (int i = 0; i < 12; i++) {
            tracker.addLoosePiece("ball", -50.0 + 9.0 * i, 30.0 * StrictMath.sin(i), BALL_RADIUS);
        }
        RangeFinder finder = new RangeFinder(new SensorMount(8.0, 1.0, 2.0, 0.3, 0.05));
        List<String> hits = new ArrayList<>();
        for (int step = 0; step < 360; step++) {
            double heading = Math.toRadians(step);
            RangeHit hit = finder.cast(world, tracker.getPieces(), 5.0 * StrictMath.cos(heading), -3.0, heading);
            hits.add(hit.getTarget() + " " + hit.getPieceId() + " " + Double.doubleToLongBits(hit.getDistanceInches()));
        }
        assertTrue(hits.stream().anyMatch(h -> h.startsWith("PIECE")), "the sweep should see at least one ball");
        return hits;
    }
}
