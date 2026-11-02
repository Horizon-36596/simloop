package org.horizon36596.simloop.fakehardware;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import java.util.ArrayList;
import java.util.List;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.horizon36596.simloop.vision.AprilTagLayout;
import org.horizon36596.simloop.vision.CameraMount;
import org.horizon36596.simloop.vision.VisionFaults;
import org.junit.jupiter.api.Test;

/**
 * INVARIANT-tier (domain R9) — the fake Limelight under stress: heavy noise and {@link VisionFaults}.
 *
 * <ol>
 *   <li>Noise never reports a tag outside the image (regression: before the fix, 60 degrees of tx/ty noise
 *       reported tx = -188 degrees and a centre pixel of -11718).</li>
 *   <li>{@code faults(VisionFaults.none())} changes nothing.</li>
 *   <li>With faults on, botpose is solved from the tags seen: correct tags still give the true pose, wrong
 *       and false ones do not.</li>
 *   <li>Out-of-order delivery makes the capture stamp go backwards; extra latency makes it lag; a blackout
 *       empties results.</li>
 * </ol>
 *
 * <p>Same made-up rig as {@link FakeLimelight3ATest}: 54.5 x 42 degrees, lens 10 in up looking forward.
 */
class FakeLimelight3AFaultsTest {

    private static final double M = 0.0254;
    private static final double TICK = 0.02;
    private static final CameraMount LEVEL = new CameraMount(0.0, 0.0, 10.0, 0.0, 0.0);
    private static final AprilTagLayout WALL = AprilTagLayout.builder()
            .addTag(1, 72.0, -12.0, 10.0, Math.PI, 6.0)
            .addTag(2, 72.0, 0.0, 10.0, Math.PI, 6.0)
            .addTag(3, 72.0, 12.0, 10.0, Math.PI, 6.0)
            .build();

    private static double metres(Pose3D pose, char axis) {
        double v = axis == 'x' ? pose.getPosition().toUnit(DistanceUnit.METER).x
                : pose.getPosition().toUnit(DistanceUnit.METER).y;
        return v;
    }

    private static FakeLimelight3A.Builder builder() {
        return FakeLimelight3A.builder(WALL, LEVEL, 54.5, 42.0);
    }

    @Test
    void heavyNoiseNeverReportsATagOutsideTheImage() {
        FakeLimelight3A ll = builder().noise(3, 60.0, 0.0, 0.0).build();
        ll.start();
        int reported = 0;
        for (int i = 0; i < 500; i++) {
            ll.update(TICK);
            for (LLResultTypes.FiducialResult f : ll.getLatestResult().getFiducialResults()) {
                reported++;
                assertTrue(Math.abs(f.getTargetXDegrees()) <= 54.5 / 2.0 + 1e-9, "tx " + f.getTargetXDegrees());
                assertTrue(Math.abs(f.getTargetYDegrees()) <= 42.0 / 2.0 + 1e-9, "ty " + f.getTargetYDegrees());
                assertTrue(f.getTargetXPixels() >= 0.0 && f.getTargetXPixels() <= 640.0, "txp " + f.getTargetXPixels());
                assertTrue(f.getTargetYPixels() >= 0.0 && f.getTargetYPixels() <= 480.0, "typ " + f.getTargetYPixels());
            }
        }
        assertTrue(reported > 0 && reported < 1500, "some tags stay in the image and some leave it: " + reported);
    }

    @Test
    void moderateNoiseIsUnchangedByTheImageEdgeRule() {
        // 0.5 degree noise cannot push a tag 12 in off the axis at 72 in out of the image: nothing drops.
        FakeLimelight3A ll = builder().noise(3, 0.5, 1.0, 1.0).build();
        ll.start();
        for (int i = 0; i < 200; i++) {
            ll.update(TICK);
            assertEquals(3, ll.getLatestResult().getFiducialResults().size());
        }
    }

    private static String trace(FakeLimelight3A ll, int ticks) {
        ll.start();
        StringBuilder s = new StringBuilder();
        for (int i = 0; i < ticks; i++) {
            ll.setRobotPose(-10.0 + i * 0.2, 2.0, 0.05);
            ll.update(TICK);
            LLResult r = ll.getLatestResult();
            s.append(r.getTimestamp()).append(' ').append(metres(r.getBotpose(), 'x')).append(' ')
                    .append(metres(r.getBotpose_MT2(), 'y')).append(' ');
            for (LLResultTypes.FiducialResult f : r.getFiducialResults()) {
                s.append(f.getFiducialId()).append(':').append(f.getTargetXDegrees()).append(',');
            }
            s.append('\n');
        }
        return s.toString();
    }

    @Test
    void noFaultsChangesNothing() {
        String plain = trace(builder().noise(4, 0.3, 1.0, 1.0).dropout(5, 0.2).latencyMs(20, 10).build(), 150);
        String none = trace(builder().noise(4, 0.3, 1.0, 1.0).dropout(5, 0.2).latencyMs(20, 10)
                .faults(VisionFaults.none()).build(), 150);
        assertEquals(plain, none);
        assertThrows(IllegalArgumentException.class, () -> builder().faults(null));
    }

    @Test
    void withFaultsOnCorrectTagsStillSolveTheTruePose() {
        FakeLimelight3A ll = builder().faults(VisionFaults.builder(1).latency(0.0, 0.0).dropFrames(0.0)
                .missDetections(0.3).build()).build();
        ll.start();
        ll.setRobotPose(5.0, -3.0, 0.1);
        for (int i = 0; i < 50; i++) {
            ll.update(TICK);
            LLResult r = ll.getLatestResult();
            if (r.getFiducialResults().isEmpty()) {
                continue;
            }
            assertEquals(5.0 * M, metres(r.getBotpose(), 'x'), 1e-9);
            assertEquals(-3.0 * M, metres(r.getBotpose(), 'y'), 1e-9);
        }
    }

    @Test
    void wrongIdsMoveBothBotposes() {
        FakeLimelight3A ll = builder().faults(VisionFaults.builder(1).wrongIds(1.0).build()).build();
        ll.start();
        ll.updateRobotOrientation(0.0);
        ll.update(TICK);
        LLResult r = ll.getLatestResult();
        List<Integer> ids = new ArrayList<Integer>();
        for (LLResultTypes.FiducialResult f : r.getFiducialResults()) {
            ids.add(f.getFiducialId());
        }
        assertEquals(3, ids.size());
        double mt1Error = Math.hypot(metres(r.getBotpose(), 'x'), metres(r.getBotpose(), 'y')) / M;
        double mt2Error = Math.hypot(metres(r.getBotpose_MT2(), 'x'), metres(r.getBotpose_MT2(), 'y')) / M;
        assertTrue(mt1Error > 1.0, "every tag misread: botpose must be off the true origin, was " + mt1Error + " in");
        assertTrue(mt2Error > 1.0, "botpose_orb too, was " + mt2Error + " in");
    }

    @Test
    void falseTagsAddAFiducialAndPullBotpose() {
        FakeLimelight3A ll = builder().faults(VisionFaults.builder(2)
                .falseDetections(1.0, "tag", 1.0, 1.0, 0).build()).build();
        ll.start();
        int extra = 0;
        double worst = 0.0;
        for (int i = 0; i < 40; i++) {
            ll.update(TICK);
            LLResult r = ll.getLatestResult();
            if (r.getFiducialResults().size() == 4) {
                extra++;
                worst = Math.max(worst, Math.hypot(metres(r.getBotpose(), 'x'), metres(r.getBotpose(), 'y')) / M);
            }
        }
        assertTrue(extra > 30, "frames with a false tag: " + extra);
        assertTrue(worst > 1.0, "a false tag must pull botpose off the truth, worst " + worst + " in");
    }

    @Test
    void outOfOrderFramesMakeTheCaptureStampGoBackwards() {
        // Frames every other tick, so a tick with no newer frame shows the late one.
        FakeLimelight3A ll = builder().frameRateHz(25.0)
                .faults(VisionFaults.builder(3).reorderFrames(0.3).build()).build();
        ll.start();
        double last = Double.NEGATIVE_INFINITY;
        int backwards = 0;
        for (int i = 0; i < 300; i++) {
            ll.update(TICK);
            double ts = ll.getLatestResult().getTimestamp();
            if (ts < last) {
                backwards++;
            }
            last = ts;
        }
        assertTrue(backwards > 15, "backward steps in ts: " + backwards);
        assertTrue(backwards <= ll.getFaultStream().getReordered());
    }

    @Test
    void extraLatencyMakesTheResultLagAndIsNotReportedAsTl() {
        FakeLimelight3A ll = builder().latencyMs(10.0, 5.0)
                .faults(VisionFaults.builder(3).latency(0.200, 0.0).build()).build();
        ll.start();
        for (int i = 0; i < 100; i++) {
            ll.update(TICK);
        }
        LLResult r = ll.getLatestResult();
        double ageMs = ll.getSimTimeNanos() / 1e6 - r.getTimestamp();
        assertEquals(215.0, ageMs, 20.0 + 1e-9, "capture is 15 ms device latency plus 200 ms extra old");
        assertEquals(5.0, r.getTargetingLatency(), 0.0, "tl reports only the device's own latency");
    }

    @Test
    void aBlackoutEmptiesResults() {
        FakeLimelight3A ll = builder().faults(VisionFaults.builder(4).blackout(0.05, 10).build()).build();
        ll.start();
        int empty = 0;
        for (int i = 0; i < 400; i++) {
            ll.update(TICK);
            LLResult r = ll.getLatestResult();
            if (!r.isValid()) {
                empty++;
                assertEquals(0, r.getFiducialResults().size());
            }
        }
        assertTrue(empty >= 10, "blacked-out results: " + empty);
        assertNotEquals(400, empty);
    }

    private static boolean finite(Pose3D pose) {
        return Double.isFinite(pose.getPosition().x) && Double.isFinite(pose.getPosition().y)
                && Double.isFinite(pose.getPosition().z) && Double.isFinite(pose.getOrientation().getRoll())
                && Double.isFinite(pose.getOrientation().getPitch()) && Double.isFinite(pose.getOrientation().getYaw());
    }

    @Test
    void aResultWithNoTagsIsInvalidAndItsBotposesAreFiniteUnderEveryFaultAtOnce() {
        FakeLimelight3A ll = builder().frameRateHz(25.0).faults(VisionFaults.builder(5).blackout(0.05, 10)
                .reorderFrames(0.3).missDetections(0.9).build()).build();
        ll.start();
        int noTags = 0;
        int withTags = 0;
        for (int i = 0; i < 600; i++) {
            ll.setRobotPose(-10.0 + i * 0.03, 2.0, 0.05);
            ll.update(TICK);
            LLResult r = ll.getLatestResult();
            assertTrue(finite(r.getBotpose()) && finite(r.getBotpose_MT2()), "tick " + i);
            if (r.getFiducialResults().isEmpty()) {
                noTags++;
                assertFalse(r.isValid(), "tick " + i + ": a result with no tags is not valid");
            } else {
                withTags++;
            }
        }
        assertTrue(noTags > 100 && withTags > 20, noTags + " results with no tags, " + withTags + " with");
        assertTrue(ll.getFaultStream().getReordered() > 0);
    }
}
