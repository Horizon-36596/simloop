package org.horizon36596.simloop.fakehardware;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.horizon36596.simloop.field.ObstacleBox;
import org.horizon36596.simloop.vision.AprilTagLayout;
import org.horizon36596.simloop.vision.CameraMount;
import org.junit.jupiter.api.Test;

/**
 * INVARIANT-tier (domain R9) — the fake Limelight's fidelity options, read through the SDK's
 * {@link LLResult}:
 *
 * <ol>
 *   <li>dropouts miss tags at about the asked-for rate, the same tags for the same seed, and none when
 *       off;</li>
 *   <li>{@code getStddevMt1/Mt2} report the noise model's spread, and zeros with noise off;</li>
 *   <li>the robot's own body hides a tag;</li>
 *   <li>distortion moves the corner and centre pixels and leaves {@code tx}, {@code ty} and the poses
 *       alone;</li>
 *   <li>a tilted tag is seen from where a plumb one is not.</li>
 * </ol>
 *
 * <p>Same made-up rig as {@link FakeLimelight3ATest}: 54.5 x 42 degrees, lens 10 in up looking forward.
 */
class FakeLimelight3AFidelityTest {

    private static final double M = 0.0254;
    private static final double TICK = 0.02;
    private static final CameraMount LEVEL = new CameraMount(0.0, 0.0, 10.0, 0.0, 0.0);
    private static final AprilTagLayout WALL = AprilTagLayout.builder()
            .addTag(1, 72.0, -12.0, 10.0, Math.PI, 6.0)
            .addTag(2, 72.0, 0.0, 10.0, Math.PI, 6.0)
            .addTag(3, 72.0, 12.0, 10.0, Math.PI, 6.0)
            .build();

    private static FakeLimelight3A.Builder builder(AprilTagLayout layout) {
        return FakeLimelight3A.builder(layout, LEVEL, 54.5, 42.0);
    }

    private static List<Integer> tagCounts(FakeLimelight3A ll, int frames) {
        ll.start();
        List<Integer> counts = new ArrayList<Integer>();
        for (int i = 0; i < frames; i++) {
            ll.update(TICK);
            counts.add(ll.getLatestResult().getFiducialResults().size());
        }
        return counts;
    }

    @Test
    void dropoutsMissTagsAtAboutTheRateAndReplayForTheSameSeed() {
        List<Integer> a = tagCounts(builder(WALL).dropout(11, 0.3).build(), 400);
        List<Integer> b = tagCounts(builder(WALL).dropout(11, 0.3).build(), 400);
        List<Integer> c = tagCounts(builder(WALL).dropout(12, 0.3).build(), 400);
        List<Integer> off = tagCounts(builder(WALL).build(), 400);
        assertEquals(a, b);
        assertTrue(!a.equals(c), "another seed misses other tags");
        int seen = 0;
        for (int n : a) {
            seen += n;
        }
        double missed = 1.0 - seen / (3.0 * 400);
        assertEquals(0.3, missed, 0.05, "missed share " + missed);
        for (int n : off) {
            assertEquals(3, n);
        }
        assertThrows(IllegalArgumentException.class, () -> builder(WALL).dropout(1, 1.0));
        assertThrows(IllegalArgumentException.class, () -> builder(WALL).dropout(1, -0.1));
    }

    @Test
    void stddevsReportTheNoiseModelAndZerosWithNoiseOff() {
        FakeLimelight3A noisy = builder(WALL).noise(5, 0.2, 1.5, 2.0).build();
        noisy.start();
        noisy.update(TICK);
        LLResult r = noisy.getLatestResult();
        assertArrayEquals(new double[] {1.5 * M, 1.5 * M, 0.0, 0.0, 0.0, 2.0}, r.getStddevMt1(), 1e-12);
        assertArrayEquals(new double[] {1.5 * M, 1.5 * M, 0.0, 0.0, 0.0, 0.0}, r.getStddevMt2(), 1e-12);

        FakeLimelight3A clean = builder(WALL).build();
        clean.start();
        clean.update(TICK);
        assertArrayEquals(new double[6], clean.getLatestResult().getStddevMt1(), 0.0);
        assertArrayEquals(new double[6], clean.getLatestResult().getStddevMt2(), 0.0);
    }

    @Test
    void theRobotsOwnBodyHidesATag() {
        // A post 4-5 in ahead of the robot centre, 2 in wide, on the centreline: hides the middle tag only.
        ObstacleBox post = new ObstacleBox("post", 4.0, 5.0, -0.5, 0.5, 0.0, 20.0);
        FakeLimelight3A ll = builder(WALL).robotBody(Arrays.asList(post)).build();
        ll.start();
        ll.update(TICK);
        List<Integer> ids = new ArrayList<Integer>();
        for (LLResultTypes.FiducialResult f : ll.getLatestResult().getFiducialResults()) {
            ids.add(f.getFiducialId());
        }
        assertEquals(Arrays.asList(1, 3), ids);
    }

    @Test
    void distortionMovesPixelsAndLeavesAnglesAndPosesAlone() {
        // Well off the axis, where barrel distortion moves a pixel by several pixels.
        AprilTagLayout offAxis = AprilTagLayout.builder().addTag(1, 40.0, -16.0, 10.0, Math.PI, 6.0).build();
        FakeLimelight3A pinhole = builder(offAxis).build();
        FakeLimelight3A barrel = builder(offAxis).distortion(-0.3, 0.08, 0.0, 0.0, 0.0).build();
        pinhole.start();
        barrel.start();
        pinhole.update(TICK);
        barrel.update(TICK);
        LLResultTypes.FiducialResult a = pinhole.getLatestResult().getFiducialResults().get(0);
        LLResultTypes.FiducialResult b = barrel.getLatestResult().getFiducialResults().get(0);
        assertEquals(a.getTargetXDegrees(), b.getTargetXDegrees(), 0.0);
        assertEquals(a.getTargetYDegrees(), b.getTargetYDegrees(), 0.0);
        assertEquals(a.getTargetPoseCameraSpace().getPosition().x, b.getTargetPoseCameraSpace().getPosition().x, 0.0);
        assertTrue(Math.abs(a.getTargetXPixels() - b.getTargetXPixels()) > 1.0, "the centre pixel moves");
        double cx = 320.0;
        double pinholeCorner = Math.abs(a.getTargetCorners().get(0).get(0) - cx);
        double barrelCorner = Math.abs(b.getTargetCorners().get(0).get(0) - cx);
        assertTrue(barrelCorner < pinholeCorner - 1.0, "barrel pulls the corner in: " + barrelCorner + " vs "
                + pinholeCorner);
    }

    @Test
    void aTiltedTagIsSeenFromWhereAPlumbOneIsNot() {
        // A tag on the floor ahead, face up; the level camera on a tall mast looking down sees it.
        CameraMount mast = new CameraMount(0.0, 0.0, 30.0, 0.0, Math.toRadians(45.0));
        AprilTagLayout flat = AprilTagLayout.builder()
                .addTiltedTag(8, 30.0, 0.0, 0.0, Math.PI, Math.PI / 2.0, 6.0).build();
        FakeLimelight3A ll = FakeLimelight3A.builder(flat, mast, 54.5, 42.0).build();
        ll.start();
        ll.update(TICK);
        assertEquals(1, ll.getLatestResult().getFiducialResults().size());
        AprilTagLayout plumbFacingAway = AprilTagLayout.builder().addTag(8, 30.0, 0.0, 0.0, 0.0, 6.0).build();
        FakeLimelight3A away = FakeLimelight3A.builder(plumbFacingAway, mast, 54.5, 42.0).build();
        away.start();
        away.update(TICK);
        assertEquals(0, away.getLatestResult().getFiducialResults().size());
    }
}
