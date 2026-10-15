package org.horizon36596.simloop.viz;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class ShotArcTest {

    private static final double PITCH = Math.toRadians(70.0);

    /** A shot from the origin, 14 in up, at a point {@code distance} in along +X; 70° comes down INTO a 59.5-in cell, the way a scoring shot must. */
    private static ShotArc.Launch aimedAt(double distance, double targetZ, double scale) {
        double speed = ShotArc.speedToReach(distance, targetZ - 14.0, PITCH);
        return new ShotArc.Launch(0.0, 0.0, 14.0, speed * scale, 0.0, PITCH);
    }

    @Test
    void aDragFreeArcSolvedForATargetEndsOnIt() {
        ShotArc arc = ShotArc.ballistic(aimedAt(48.0, 59.5, 1.0), 0.0, 59.5, 0.02);
        int last = arc.size() - 1;
        assertEquals(48.0, arc.x(last), 0.3, "the solved arc should come down through the target plane at the target");
        assertEquals(59.5, arc.z(last), 1e-9, "the last point is placed exactly on the stop plane");
        assertTrue(arc.closestApproachInches(48.0, 0.0, 59.5) < 0.3);
    }

    @Test
    void theApexMatchesTheTextbookHeight() {
        ShotArc.Launch launch = aimedAt(48.0, 59.5, 1.0);
        double vz = launch.speedInchesPerSecond * Math.sin(PITCH);
        double expected = 14.0 + vz * vz / (2.0 * ShotArc.GRAVITY_IN_PER_S2);
        assertEquals(expected, ShotArc.ballistic(launch, 0.0, 0.0, 0.005).apexZInches(), 0.2);
    }

    @Test
    void dragAndALowFlywheelBothMakeTheShotShort() {
        ShotArc ideal = ShotArc.ballistic(aimedAt(48.0, 59.5, 1.0), 0.0, 59.5, 0.02);
        ShotArc dragged = ShotArc.ballistic(aimedAt(48.0, 59.5, 1.0), 0.0015, 59.5, 0.02);
        ShotArc slow = ShotArc.ballistic(aimedAt(48.0, 59.5, 0.96), 0.0, 59.5, 0.02);
        double idealEnd = ideal.x(ideal.size() - 1);
        assertTrue(dragged.x(dragged.size() - 1) < idealEnd - 1.0, "drag should cost more than an inch at 4 ft");
        assertTrue(slow.x(slow.size() - 1) < idealEnd - 1.0, "4% less exit speed should cost more than an inch");
    }

    @Test
    void theSameLaunchGivesTheSamePoints() {
        ShotArc a = ShotArc.ballistic(aimedAt(60.0, 59.5, 0.98), 0.0015, 0.0, 0.02);
        ShotArc b = ShotArc.ballistic(aimedAt(60.0, 59.5, 0.98), 0.0015, 0.0, 0.02);
        assertEquals(a.size(), b.size());
        for (int i = 0; i < a.size(); i++) {
            assertEquals(a.x(i), b.x(i), 0.0);
            assertEquals(a.z(i), b.z(i), 0.0);
        }
    }

    @Test
    void oneSampleIsOneTickOfFlight() {
        ShotArc.Launch launch = aimedAt(48.0, 59.5, 1.0);
        ShotArc arc = ShotArc.ballistic(launch, 0.0, 0.0, 0.02);
        double vx = launch.speedInchesPerSecond * Math.cos(PITCH);
        assertEquals(vx * 0.02 * 5, arc.x(5), 1e-6, "with no drag, x after five ticks is vx times 0.1 s");
    }

    @Test
    void aShotThatNeverReachesTheStopPlaneFallsToTheFloor() {
        ShotArc weak = ShotArc.ballistic(aimedAt(48.0, 59.5, 0.5), 0.0, 59.5, 0.02);
        assertTrue(weak.apexZInches() < 59.5, "half the speed cannot reach the cell");
        assertEquals(0.0, weak.z(weak.size() - 1), 1e-9, "it ends on the floor, not at its apex");
    }

    @Test
    void aTargetAboveThePitchLineIsUnreachable() {
        assertTrue(Double.isNaN(ShotArc.speedToReach(10.0, 40.0, Math.toRadians(30.0))));
        assertTrue(Double.isNaN(ShotArc.speedToReach(0.0, 10.0, PITCH)));
    }

    @Test
    void closestApproachMeasuresToTheSegmentNotJustTheVertices() {
        ShotArc line = ShotArc.ofPoints(0, 0, 0, 10, 0, 0);
        assertEquals(3.0, line.closestApproachInches(5, 3, 0), 1e-12);
        assertEquals(5.0, line.closestApproachInches(-3, 4, 0), 1e-12, "beyond the end, it is the distance to the end");
    }

    @Test
    void prefixClampsToTheArc() {
        ShotArc arc = ShotArc.ofPoints(0, 0, 0, 1, 0, 0, 2, 0, 0);
        assertEquals(1, arc.prefix(0).size());
        assertEquals(2, arc.prefix(2).size());
        assertEquals(3, arc.prefix(99).size());
    }

    @Test
    void translationsAreInMetres() {
        assertEquals(0.0254, ShotArc.ofPoints(1, 2, 3).toTranslations()[0].getX(), 1e-12);
    }

    @Test
    void badInputIsRefused() {
        assertThrows(IllegalArgumentException.class, () -> ShotArc.ofPoints(1, 2));
        assertThrows(IllegalArgumentException.class, () -> ShotArc.ofPoints(1, 2, Double.NaN));
        assertThrows(IllegalArgumentException.class,
                () -> ShotArc.ballistic(aimedAt(48.0, 59.5, 1.0), -1.0, 0.0, 0.02));
        assertThrows(IllegalArgumentException.class,
                () -> ShotArc.ballistic(aimedAt(48.0, 59.5, 1.0), 0.0, 0.0, 0.0));
        assertThrows(IllegalArgumentException.class, () -> new ShotArc.Launch(0, 0, 0, -1, 0, 0));
    }

    @Test
    void theRecorderCyclesItsSlotsAndReportsNoMissUntilThereIsAnArc() {
        ShotArcRecorder recorder = new ShotArcRecorder(2);
        assertTrue(Double.isNaN(recorder.lastMissInches()));
        assertThrows(IllegalStateException.class, () -> recorder.setActual(ShotArc.ofPoints(0, 0, 0)));

        ShotArc arc = ShotArc.ofPoints(0, 0, 0, 10, 0, 0);
        assertEquals(0, recorder.fire(arc, 5, 3, 0));
        assertTrue(Double.isNaN(recorder.lastMissInches()), "no actual arc yet");
        recorder.setActual(arc);
        assertEquals(3.0, recorder.lastMissInches(), 1e-12);
        assertEquals(1, recorder.fire(arc, 0, 0, 0));
        assertEquals(0, recorder.fire(arc, 0, 0, 0), "the third shot overwrites the first slot");
        assertThrows(IllegalArgumentException.class, () -> new ShotArcRecorder(0));
    }
}
