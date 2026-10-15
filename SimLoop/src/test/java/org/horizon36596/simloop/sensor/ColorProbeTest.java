package org.horizon36596.simloop.sensor;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.horizon36596.simloop.fakehardware.ColorCounts;
import org.horizon36596.simloop.fakehardware.FakeRevColorSensorV3;
import org.horizon36596.simloop.field.GamePiece;
import org.horizon36596.simloop.field.GamePieceTracker;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/**
 * Pins {@link ColorProbe}: a colour sensor sees the nearest piece inside its cone and range, reads that
 * piece's counts at that piece's gap, and sees the background with nothing in view. INVARIANT tier (domain
 * R9) for the in-view rule and determinism; the hand-off details are implementation tier.
 *
 * <p>The counts below are made up for the test. Real ones are measured on the robot.
 */
class ColorProbeTest {

    private static final double BALL_RADIUS = 2.5;
    private static final ColorCounts PURPLE = new ColorCounts(900, 700, 1800);
    private static final ColorCounts GREEN = new ColorCounts(300, 2100, 900);
    private static final ColorCounts BACKGROUND = new ColorCounts(40, 50, 60);

    private static Map<String, ColorCounts> palette() {
        Map<String, ColorCounts> colours = new LinkedHashMap<>();
        colours.put("purple", PURPLE);
        colours.put("green", GREEN);
        return colours;
    }

    /** A level sensor at the robot's centre, at a ball's centre height, looking forward: 2 in range, 30 degrees each side. */
    private static ColorProbe forwardProbe() {
        return new ColorProbe(SensorMount.level(0.0, 0.0, BALL_RADIUS, 0.0), 2.0, Math.toRadians(30.0), palette(),
                BACKGROUND);
    }

    // ---- in view or not ----

    @Test
    void invariant_aBallInFrontAndWithinRangeIsSeenAtItsGap() {
        GamePieceTracker tracker = new GamePieceTracker(3);
        int id = tracker.addLoosePiece("purple", 3.0, 0.0, BALL_RADIUS);

        ColorProbe.Sighting sighting = forwardProbe().look(tracker.getPieces(), 0.0, 0.0, 0.0);

        assertTrue(sighting.isPieceInView());
        assertEquals(id, sighting.getPiece().getId());
        assertEquals(0.5, sighting.getGapInches(), 1e-12, "3 in to the centre, less the 2.5 in radius");
    }

    @Test
    void invariant_aBallPastTheRangeIsNotSeen() {
        GamePieceTracker tracker = new GamePieceTracker(3);
        tracker.addLoosePiece("purple", 4.6, 0.0, BALL_RADIUS);    // gap 2.1

        assertFalse(forwardProbe().look(tracker.getPieces(), 0.0, 0.0, 0.0).isPieceInView());

        GamePieceTracker atTheEdge = new GamePieceTracker(3);
        atTheEdge.addLoosePiece("purple", 4.5, 0.0, BALL_RADIUS);  // gap exactly 2
        assertTrue(forwardProbe().look(atTheEdge.getPieces(), 0.0, 0.0, 0.0).isPieceInView(), "the range is inclusive");
    }

    @Test
    void invariant_aBallOutsideTheConeIsNotSeenEvenWhenClose() {
        GamePieceTracker tracker = new GamePieceTracker(3);
        tracker.addLoosePiece("purple", 2.5, 2.5, BALL_RADIUS);    // 45 degrees off the axis, gap 1.04

        assertFalse(forwardProbe().look(tracker.getPieces(), 0.0, 0.0, 0.0).isPieceInView(), "45 > 30 degrees");

        ColorProbe wide = new ColorProbe(SensorMount.level(0.0, 0.0, BALL_RADIUS, 0.0), 2.0, Math.toRadians(60.0),
                palette(), BACKGROUND);
        assertTrue(wide.look(tracker.getPieces(), 0.0, 0.0, 0.0).isPieceInView(), "45 < 60 degrees");
    }

    @Test
    void aBallBehindTheSensorIsNotSeen() {
        GamePieceTracker tracker = new GamePieceTracker(3);
        tracker.addLoosePiece("purple", -3.0, 0.0, BALL_RADIUS);

        assertFalse(forwardProbe().look(tracker.getPieces(), 0.0, 0.0, 0.0).isPieceInView());
    }

    @Test
    void theNearestBallInViewIsTheOneSeen() {
        GamePieceTracker tracker = new GamePieceTracker(3);
        tracker.addLoosePiece("purple", 4.0, 0.0, BALL_RADIUS);    // gap 1.5
        int green = tracker.addLoosePiece("green", 3.0, 0.5, BALL_RADIUS);   // gap 0.54, 9.5 degrees off

        ColorProbe.Sighting sighting = forwardProbe().look(tracker.getPieces(), 0.0, 0.0, 0.0);
        assertEquals(green, sighting.getPiece().getId());
        assertEquals(Math.sqrt(9.25) - BALL_RADIUS, sighting.getGapInches(), 1e-12);
    }

    @Test
    void theMountFollowsTheRobotPose() {
        // Robot at (10, 20) facing +Y; a sensor 3 in ahead of centre is at (10, 23), and a ball centred at
        // (10, 26) is 0.5 in from its face.
        GamePieceTracker tracker = new GamePieceTracker(3);
        tracker.addLoosePiece("green", 10.0, 26.0, BALL_RADIUS);
        ColorProbe probe = new ColorProbe(SensorMount.level(3.0, 0.0, BALL_RADIUS, 0.0), 2.0, Math.toRadians(30.0),
                palette(), BACKGROUND);

        ColorProbe.Sighting sighting = probe.look(tracker.getPieces(), 10.0, 20.0, Math.PI / 2.0);
        assertTrue(sighting.isPieceInView());
        assertEquals(0.5, sighting.getGapInches(), 1e-9);
        assertFalse(probe.look(tracker.getPieces(), 10.0, 20.0, 0.0).isPieceInView(), "turned away, it is gone");
    }

    @Test
    void aSensorLookingStraightDownSeesTheBallUnderIt() {
        GamePieceTracker tracker = new GamePieceTracker(3);
        tracker.addLoosePiece("purple", 3.0, 0.0, BALL_RADIUS);
        ColorProbe overSlot = new ColorProbe(new SensorMount(3.0, 0.0, 6.0, 0.0, Math.PI / 2.0), 2.0,
                Math.toRadians(20.0), palette(), BACKGROUND);

        ColorProbe.Sighting sighting = overSlot.look(tracker.getPieces(), 0.0, 0.0, 0.0);
        assertTrue(sighting.isPieceInView());
        assertEquals(1.0, sighting.getGapInches(), 1e-12, "6 in up, ball top at 5 in");
    }

    @Test
    void aHeldPieceIsNotInViewButCanBeShown() {
        GamePieceTracker tracker = new GamePieceTracker(3);
        tracker.teleportRobot(0.0, 0.0, 0.0);
        int held = tracker.addHeldPiece("green", BALL_RADIUS);
        ColorProbe probe = new ColorProbe(SensorMount.level(0.0, 0.0, BALL_RADIUS, 0.0), 5.0, Math.PI, palette(),
                BACKGROUND);
        assertFalse(probe.look(tracker.getPieces(), 0.0, 0.0, 0.0).isPieceInView(),
                "held pieces are parked at the robot's centre and never seen");

        FakeRevColorSensorV3 sensor = new FakeRevColorSensorV3();
        probe.show(sensor, tracker.getPiece(held), 0.3);
        assertEquals(GREEN, sensor.getRawCounts());
        assertEquals(0.3, sensor.getDistance(DistanceUnit.INCH), 0.001);
    }

    // ---- what the sensor reads ----

    @Test
    void invariant_applyShowsThePiecesCountsAtItsGapAndTheBackgroundWhenItLeaves() {
        GamePieceTracker tracker = new GamePieceTracker(3);
        tracker.addLoosePiece("purple", 3.5, 0.0, BALL_RADIUS);
        FakeRevColorSensorV3 sensor = new FakeRevColorSensorV3();
        ColorProbe probe = forwardProbe();

        ColorProbe.Sighting sighting = probe.apply(sensor, tracker.getPieces(), 0.0, 0.0, 0.0);
        assertTrue(sighting.isPieceInView());
        assertEquals(PURPLE, sensor.getRawCounts());
        assertEquals(963, sensor.red(), "and robot code reads the SDK's arithmetic on those counts: 1.07 * 900");
        assertEquals(1.0, sensor.getDistance(DistanceUnit.INCH), 0.002);

        ColorProbe.Sighting away = probe.apply(sensor, tracker.getPieces(), -20.0, 0.0, 0.0);
        assertFalse(away.isPieceInView());
        assertEquals(BACKGROUND, sensor.getRawCounts());
        assertEquals(6.0, sensor.getDistance(DistanceUnit.INCH), 0.0, "nothing in range");
    }

    @Test
    void aKindWithNoCountsIsALoudFailureNotADarkSensor() {
        GamePieceTracker tracker = new GamePieceTracker(3);
        tracker.addLoosePiece("orange", 3.0, 0.0, BALL_RADIUS);
        FakeRevColorSensorV3 sensor = new FakeRevColorSensorV3();

        IllegalStateException refused = assertThrows(IllegalStateException.class,
                () -> forwardProbe().apply(sensor, tracker.getPieces(), 0.0, 0.0, 0.0));
        assertTrue(refused.getMessage().contains("orange"), refused.getMessage());
    }

    @Test
    void thePaletteIsCopiedSoALaterEditDoesNotLeakIn() {
        Map<String, ColorCounts> colours = palette();
        ColorProbe probe = new ColorProbe(SensorMount.level(0.0, 0.0, 1.0, 0.0), 2.0, 0.5, colours, BACKGROUND);
        colours.put("purple", GREEN);

        assertSame(PURPLE, probe.countsFor("purple"));
    }

    @Test
    void badArgumentsAreRefused() {
        SensorMount mount = SensorMount.level(0.0, 0.0, 1.0, 0.0);
        assertThrows(IllegalArgumentException.class, () -> new ColorProbe(null, 2.0, 0.5, palette(), BACKGROUND));
        assertThrows(IllegalArgumentException.class, () -> new ColorProbe(mount, 0.0, 0.5, palette(), BACKGROUND));
        assertThrows(IllegalArgumentException.class, () -> new ColorProbe(mount, 2.0, 0.0, palette(), BACKGROUND));
        assertThrows(IllegalArgumentException.class, () -> new ColorProbe(mount, 2.0, 3.2, palette(), BACKGROUND));
        assertThrows(IllegalArgumentException.class, () -> new ColorProbe(mount, 2.0, 0.5, null, BACKGROUND));
        assertThrows(IllegalArgumentException.class, () -> new ColorProbe(mount, 2.0, 0.5, palette(), null));
        Map<String, ColorCounts> withNull = palette();
        withNull.put("ghost", null);
        assertThrows(IllegalArgumentException.class, () -> new ColorProbe(mount, 2.0, 0.5, withNull, BACKGROUND));

        ColorProbe probe = forwardProbe();
        assertThrows(IllegalArgumentException.class, () -> probe.look(null, 0.0, 0.0, 0.0));
        assertThrows(IllegalArgumentException.class,
                () -> probe.apply(null, Collections.<GamePiece>emptyList(), 0.0, 0.0, 0.0));
        assertThrows(IllegalArgumentException.class,
                () -> probe.look(Collections.<GamePiece>emptyList(), 0.0, Double.NaN, 0.0));
    }

    @Test
    void invariant_theSameSceneGivesTheSameSightingsEveryRun() {
        assertEquals(sweep(), sweep());
    }

    private static List<String> sweep() {
        GamePieceTracker tracker = new GamePieceTracker(3);
        for (int i = 0; i < 8; i++) {
            tracker.addLoosePiece(i % 2 == 0 ? "purple" : "green", 4.0 * StrictMath.cos(i), 4.0 * StrictMath.sin(i),
                    BALL_RADIUS);
        }
        ColorProbe probe = new ColorProbe(new SensorMount(1.0, 0.0, 3.0, 0.0, 0.2), 3.0, Math.toRadians(25.0),
                palette(), BACKGROUND);
        FakeRevColorSensorV3 sensor = new FakeRevColorSensorV3();
        List<String> seen = new ArrayList<>();
        for (int step = 0; step < 360; step += 3) {
            ColorProbe.Sighting sighting = probe.apply(sensor, tracker.getPieces(), 0.0, 0.0, Math.toRadians(step));
            seen.add((sighting.isPieceInView() ? sighting.getPiece().getId() : -1) + " "
                    + Double.doubleToLongBits(sighting.getGapInches()) + " " + sensor.argb());
        }
        return seen;
    }
}
