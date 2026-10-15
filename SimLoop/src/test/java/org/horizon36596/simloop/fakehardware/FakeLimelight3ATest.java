package org.horizon36596.simloop.fakehardware;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import java.lang.reflect.Method;
import java.lang.reflect.Modifier;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.horizon36596.simloop.field.FieldFrameTransform;
import org.horizon36596.simloop.vision.AprilTagLayout;
import org.horizon36596.simloop.vision.CameraMount;
import org.junit.jupiter.api.Test;

/**
 * INVARIANT-tier (domain R9) — the fake Limelight 3A, read only through the SDK 12.0.0 surface robot code
 * uses ({@link Limelight3A}, {@link LLResult}, {@link LLResultTypes.FiducialResult}):
 *
 * <ol>
 *   <li>lifecycle as the SDK driver's bytecode has it: before a poll the result is empty and invalid, never
 *       null; {@code start()} polls, {@code stop()}/{@code pause()} stop polling and keep the last result;</li>
 *   <li>time is sim time only: staleness, timestamps, latency and connection all follow {@code update},
 *       and none of them move with the wall clock;</li>
 *   <li>MegaTag1 {@code botpose} returns the sim pose; MegaTag2 {@code botpose_orb} uses the yaw sent with
 *       {@code updateRobotOrientation};</li>
 *   <li>per-tag results (id, tx/ty, poses in camera, robot, target and field space) match the geometry;</li>
 *   <li>{@code pipelineSwitch} shows in the result; noise is off by default, seeded and deterministic;</li>
 *   <li>every network method of the SDK driver is overridden, so nothing can reach a socket.</li>
 * </ol>
 *
 * <p>Test rig, made up: 54.5 x 42 degree field of view, lens 10 in up on the robot centre looking
 * forward. Every layout is invented for the test; SimLoop has no field geometry.
 */
class FakeLimelight3ATest {

    private static final double M = 0.0254;       // metres per inch
    private static final double TICK = 0.02;      // 20 ms, the harness's usual step
    private static final CameraMount LEVEL = new CameraMount(0.0, 0.0, 10.0, 0.0, 0.0);

    /** One tag, 60 in straight ahead of a robot at the origin facing +X. */
    private static final AprilTagLayout ONE_TAG = AprilTagLayout.builder()
            .addTag(7, 60.0, 0.0, 10.0, Math.PI, 6.0)
            .build();

    /** Three tags on a made-up +X wall. */
    private static final AprilTagLayout WALL = AprilTagLayout.builder()
            .addTag(3, 72.0, 12.0, 10.0, Math.PI, 6.0)
            .addTag(1, 72.0, -12.0, 10.0, Math.PI, 6.0)
            .addTag(2, 48.0, 0.0, 12.0, Math.PI, 6.0)
            .build();

    private static FakeLimelight3A.Builder builder(AprilTagLayout layout) {
        return FakeLimelight3A.builder(layout, LEVEL, 54.5, 42.0);
    }

    private static FakeLimelight3A started(AprilTagLayout layout) {
        FakeLimelight3A ll = builder(layout).build();
        ll.start();
        return ll;
    }

    private static double metres(Position p, char axis) {
        Position m = p.toUnit(DistanceUnit.METER);
        return axis == 'x' ? m.x : axis == 'y' ? m.y : m.z;
    }

    // ------------------------------------------------------------------ clause 1: lifecycle

    @Test
    void itIsALimelight3AInTheHardwareMap() {
        FakeLimelight3A ll = builder(ONE_TAG).build();
        FakeHardwareMap map = new FakeHardwareMap();
        map.register("limelight", ll);
        assertSame(ll, map.get(Limelight3A.class, "limelight"));
        assertEquals("limelight", ll.getDeviceName());
        assertEquals(Limelight3A.Manufacturer.LimelightVision, ll.getManufacturer());
        assertEquals("sim", ll.getConnectionInfo());
    }

    @Test
    void beforeStartTheResultIsEmptyAndInvalidButNeverNull() {
        FakeLimelight3A ll = builder(ONE_TAG).build();
        LLResult first = ll.getLatestResult();
        assertNotNull(first);
        assertFalse(first.isValid());
        assertTrue(first.getFiducialResults().isEmpty());
        assertFalse(ll.isRunning());
        // The camera is powered and capturing, but nothing polls it: still the same empty result.
        for (int i = 0; i < 10; i++) {
            ll.update(TICK);
        }
        assertSame(first, ll.getLatestResult());
        assertFalse(ll.getLatestResult().isValid());
        assertFalse(ll.isConnected());
    }

    @Test
    void afterStartTheNextTickDeliversAValidResult() {
        FakeLimelight3A ll = builder(ONE_TAG).build();
        ll.start();
        assertTrue(ll.isRunning());
        assertFalse(ll.getLatestResult().isValid(), "start() alone does not poll; the next update does");
        ll.update(TICK);
        LLResult r = ll.getLatestResult();
        assertTrue(r.isValid());
        assertEquals(1, r.getFiducialResults().size());
        assertTrue(ll.isConnected());
    }

    @Test
    void stopAndPauseKeepTheLastResultWhichGoesStaleAndDisconnects() {
        for (boolean usePause : new boolean[] {false, true}) {
            FakeLimelight3A ll = started(ONE_TAG);
            ll.update(TICK);
            LLResult last = ll.getLatestResult();
            if (usePause) {
                ll.pause();
            } else {
                ll.stop();
            }
            assertFalse(ll.isRunning());
            ll.setRobotPose(10.0, 5.0, 0.3);          // the robot moves; nobody polls
            for (int i = 0; i < 13; i++) {
                ll.update(TICK);
            }
            assertSame(last, ll.getLatestResult(), "no fresh result while stopped");
            assertEquals(260L, ll.getLatestResult().getStaleness());
            assertFalse(ll.isConnected(), "the SDK's isConnected: more than 250 ms since the last poll");

            ll.start();
            ll.update(TICK);
            assertNotEquals(last, ll.getLatestResult());
            assertEquals(0L, ll.getLatestResult().getStaleness());
            assertEquals(10.0 * M, metres(ll.getLatestResult().getBotpose().getPosition(), 'x'), 1e-12);
            assertTrue(ll.isConnected());
        }
    }

    @Test
    void startWhileRunningIsANoOpAndCloseStops() {
        FakeLimelight3A ll = started(ONE_TAG);
        ll.update(TICK);
        ll.start();
        assertTrue(ll.isRunning());
        ll.close();
        assertFalse(ll.isRunning());
    }

    // ------------------------------------------------------------------ clause 2: sim time only

    @Test
    void stalenessAndTimestampsFollowSimTimeAndNotTheWallClock() throws InterruptedException {
        FakeLimelight3A ll = started(ONE_TAG);
        for (int i = 0; i < 5; i++) {
            ll.update(TICK);
        }
        LLResult r = ll.getLatestResult();
        assertEquals(100L, r.getControlHubTimeStamp());
        assertEquals(100L * 1_000_000L, r.getControlHubTimeStampNanos());
        assertEquals(100.0, r.getTimestamp(), 1e-9, "ts: capture time on the device, sim ms");
        assertEquals(0.0, r.getParseLatency(), 0.0);
        ll.stop();
        long before = r.getStaleness();
        Thread.sleep(30);                         // wall time passes; sim time does not
        assertEquals(before, r.getStaleness());
        assertEquals(before, ll.getTimeSinceLastUpdate());
        ll.update(0.05);
        assertEquals(before + 50L, r.getStaleness());
        assertEquals(before + 50L, ll.getTimeSinceLastUpdate());
    }

    @Test
    void aNonPositiveOrNonFiniteTickAdvancesNothing() {
        FakeLimelight3A ll = started(ONE_TAG);
        ll.update(TICK);
        ll.update(0.0);
        ll.update(-1.0);
        ll.update(Double.NaN);
        ll.update(Double.POSITIVE_INFINITY);
        assertEquals(20_000_000L, ll.getSimTimeNanos());
    }

    @Test
    void thePollRateSetsHowOftenAFreshResultArrives() {
        FakeLimelight3A ll = builder(ONE_TAG).build();
        ll.setPollRateHz(10);                     // 100 ms; allowed only while stopped (SDK rule)
        ll.start();
        ll.setPollRateHz(250);                    // ignored while running
        List<Long> staleness = new ArrayList<>();
        for (int i = 0; i < 10; i++) {
            ll.update(TICK);
            staleness.add(ll.getLatestResult().getStaleness());
        }
        // Fixed-rate from start() at t = 0: polls are due at 0, 100 and 200 ms and happen on the first tick
        // at or after each (20, 100, 200 ms).
        assertEquals(Arrays.asList(0L, 20L, 40L, 60L, 0L, 20L, 40L, 60L, 80L, 0L), staleness);
    }

    @Test
    void atTheDefault100HzPollEveryTwentyMsTickIsFresh() {
        FakeLimelight3A ll = started(ONE_TAG);
        for (int i = 0; i < 5; i++) {
            ll.update(TICK);
            assertEquals(0L, ll.getLatestResult().getStaleness());
        }
    }

    @Test
    void latencyDelaysResultsAndTheyDescribeWhereTheRobotWas() {
        FakeLimelight3A ll = builder(ONE_TAG).latencyMs(30.0, 20.0).build();
        ll.start();
        // Robot drives +Y at 1 in per 10 ms tick; 50 ms of latency is 5 ticks. The first frame is captured
        // on tick 1 (t = 10 ms) and clears its latency on tick 6.
        for (int i = 1; i <= 5; i++) {
            ll.setRobotPose(0.0, i, 0.0);
            ll.update(0.01);
            assertFalse(ll.getLatestResult().isValid(), "tick " + i + ": nothing has cleared the latency yet");
        }
        for (int i = 6; i <= 12; i++) {
            ll.setRobotPose(0.0, i, 0.0);
            ll.update(0.01);
            LLResult r = ll.getLatestResult();
            assertTrue(r.isValid());
            assertEquals(30.0, r.getCaptureLatency(), 0.0);
            assertEquals(20.0, r.getTargetingLatency(), 0.0);
            double capturedAtMs = r.getTimestamp();
            assertEquals(i * 10.0 - 50.0, capturedAtMs, 1e-9);
            // The frame was captured when the robot was at y = capture tick.
            assertEquals((i - 5) * M, metres(r.getBotpose().getPosition(), 'y'), 1e-12);
        }
    }

    @Test
    void aSetFrameRateCapturesOnItsOwnScheduleNotEveryTick() {
        FakeLimelight3A ll = builder(ONE_TAG).frameRateHz(25.0).build();   // 40 ms per frame
        ll.start();
        List<Double> ts = new ArrayList<>();
        for (int i = 0; i < 8; i++) {
            ll.update(0.01);
            ts.add(ll.getLatestResult().getTimestamp());
        }
        assertEquals(Arrays.asList(10.0, 10.0, 10.0, 10.0, 50.0, 50.0, 50.0, 50.0), ts);
        assertEquals(25.0, ll.getStatus().getFps(), 0.0);
    }

    // ------------------------------------------------------------------ clause 3: botpose, MegaTag2

    @Test
    void megaTag1BotposeRoundTripsTheSimPoseExactlyWithNoNoise() {
        FakeLimelight3A ll = started(WALL);
        double[][] poses = {{0, 0, 0}, {10, -6, 0.2}, {-20, 8, -0.35}, {5, 5, 0.1}, {-30, -10, 0.25}};
        for (double[] p : poses) {
            ll.setRobotPose(p[0], p[1], p[2]);
            ll.update(TICK);
            LLResult r = ll.getLatestResult();
            assertTrue(r.isValid(), "pose " + Arrays.toString(p) + " should see the wall");
            Pose3D bot = r.getBotpose();
            // Stated tolerance: 1e-9 m and 1e-9 deg (double rounding only).
            assertEquals(DistanceUnit.METER, bot.getPosition().unit);
            assertEquals(p[0] * M, metres(bot.getPosition(), 'x'), 1e-9);
            assertEquals(p[1] * M, metres(bot.getPosition(), 'y'), 1e-9);
            assertEquals(0.0, metres(bot.getPosition(), 'z'), 1e-9);
            assertEquals(StrictMath.toDegrees(p[2]), bot.getOrientation().getYaw(AngleUnit.DEGREES), 1e-9);
            // MegaTag2 with the true yaw also lands on the true pose.
            ll.updateRobotOrientation(StrictMath.toDegrees(p[2]));
            ll.update(TICK);
            Pose3D mt2 = ll.getLatestResult().getBotpose_MT2();
            assertEquals(p[0] * M, metres(mt2.getPosition(), 'x'), 1e-9);
            assertEquals(p[1] * M, metres(mt2.getPosition(), 'y'), 1e-9);
            assertEquals(0.0, metres(mt2.getPosition(), 'z'), 1e-9);
        }
    }

    @Test
    void botposeYawIsWrappedToPlusMinus180() {
        FakeLimelight3A ll = started(ONE_TAG);
        ll.setRobotPose(0.0, 0.0, 2.0 * Math.PI + 0.1);
        ll.update(TICK);
        assertEquals(StrictMath.toDegrees(0.1), ll.getLatestResult().getBotpose().getOrientation().getYaw(), 1e-9);
    }

    @Test
    void megaTag2UsesTheSuppliedYawAndAWrongYawSwingsThePositionAroundTheTag() {
        FakeLimelight3A ll = started(ONE_TAG);
        ll.update(TICK);
        assertEquals(0.0, ll.getLatestResult().getBotpose_MT2().getOrientation().getYaw(), 0.0,
                "before any updateRobotOrientation the device's yaw is 0");

        assertTrue(ll.updateRobotOrientation(10.0));       // the robot really faces 0
        ll.update(TICK);
        Pose3D mt2 = ll.getLatestResult().getBotpose_MT2();
        assertEquals(10.0, mt2.getOrientation().getYaw(AngleUnit.DEGREES), 0.0);
        // The tag is (60, 0, 0) in from the robot (at camera height, so z cancels). Turned by 10 degrees,
        // that offset puts the robot at tag - Rz(10) * (60, 0).
        double yaw = StrictMath.toRadians(10.0);
        assertEquals((60.0 - 60.0 * StrictMath.cos(yaw)) * M, metres(mt2.getPosition(), 'x'), 1e-9);
        assertEquals((-60.0 * StrictMath.sin(yaw)) * M, metres(mt2.getPosition(), 'y'), 1e-9);
        // MegaTag1 ignores the supplied yaw.
        assertEquals(0.0, ll.getLatestResult().getBotpose().getOrientation().getYaw(), 1e-9);
        assertEquals(10.0, ll.getStatus().getFinalYaw(), 0.0);
    }

    @Test
    void aNonFiniteYawIsRefusedAndChangesNothing() {
        FakeLimelight3A ll = started(ONE_TAG);
        ll.updateRobotOrientation(5.0);
        assertFalse(ll.updateRobotOrientation(Double.NaN));
        assertEquals(5.0, ll.getSuppliedYawDegrees(), 0.0);
    }

    @Test
    void theLocalizerFramePoseIsRotatedIntoTheFieldFrame() {
        FakeLimelight3A viaLocalizer = started(WALL);
        FakeLimelight3A viaField = started(WALL);
        double lx = 4.0;
        double ly = 12.0;
        double lh = -Math.PI / 2 + 0.1;
        viaLocalizer.setRobotPoseFromLocalizer(lx, ly, lh);
        viaField.setRobotPose(FieldFrameTransform.fieldXFromLocalizer(lx, ly),
                FieldFrameTransform.fieldYFromLocalizer(lx, ly), FieldFrameTransform.fieldHeadingFromLocalizer(lh));
        viaLocalizer.update(TICK);
        viaField.update(TICK);
        assertTrue(viaField.getLatestResult().isValid());
        assertEquals(viaField.getLatestResult().toString(), viaLocalizer.getLatestResult().toString());
    }

    // ------------------------------------------------------------------ clause 4: per-tag results

    @Test
    void eachFiducialCarriesItsIdAnglesAndPosesInEverySpace() {
        FakeLimelight3A ll = started(ONE_TAG);
        ll.setRobotPose(0.0, 10.0, 0.0);                   // tag is 10 in to the robot's right
        ll.update(TICK);
        LLResult r = ll.getLatestResult();
        LLResultTypes.FiducialResult f = r.getFiducialResults().get(0);
        assertEquals(7, f.getFiducialId());
        assertEquals("36h11", f.getFamily());
        double tx = StrictMath.toDegrees(StrictMath.atan(10.0 / 60.0));
        assertEquals(tx, f.getTargetXDegrees(), 1e-9);
        assertEquals(tx, f.getTargetXDegreesNoCrosshair(), 1e-9);
        assertEquals(0.0, f.getTargetYDegrees(), 1e-9);
        assertEquals(tx, r.getTx(), 1e-9);
        assertEquals(tx, r.getTxNC(), 1e-9);
        assertTrue(f.getTargetXPixels() > 320.0);
        assertEquals(240.0, f.getTargetYPixels(), 1e-9);
        assertEquals(4, f.getTargetCorners().size());
        assertTrue(f.getTargetArea() > 0.0);
        assertEquals(f.getTargetArea(), r.getTa(), 0.0);

        // Camera space: +x right, +y down, +z out of the lens. Metres.
        Position cs = f.getTargetPoseCameraSpace().getPosition();
        assertEquals(10.0 * M, metres(cs, 'x'), 1e-9);
        assertEquals(0.0, metres(cs, 'y'), 1e-9);
        assertEquals(60.0 * M, metres(cs, 'z'), 1e-9);
        // Robot space: +x forward, +y left, +z up. The tag is ahead, to the right, at lens height.
        Position rs = f.getTargetPoseRobotSpace().getPosition();
        assertEquals(60.0 * M, metres(rs, 'x'), 1e-9);
        assertEquals(-10.0 * M, metres(rs, 'y'), 1e-9);
        assertEquals(10.0 * M, metres(rs, 'z'), 1e-9);
        // Target space: the camera is in front of the tag, which is -z, and to the tag viewer's left.
        Position ts = f.getCameraPoseTargetSpace().getPosition();
        assertEquals(-60.0 * M, metres(ts, 'z'), 1e-9);
        assertEquals(-10.0 * M, metres(ts, 'x'), 1e-9);
        Position rts = f.getRobotPoseTargetSpace().getPosition();
        assertEquals(-60.0 * M, metres(rts, 'z'), 1e-9);
        assertEquals(10.0 * M, metres(rts, 'y'), 1e-9, "the robot origin is on the floor, 10 in below the tag");
        // Field space: the single-tag robot pose.
        Position fs = f.getRobotPoseFieldSpace().getPosition();
        assertEquals(0.0, metres(fs, 'x'), 1e-9);
        assertEquals(10.0 * M, metres(fs, 'y'), 1e-9);
    }

    @Test
    void severalTagsAreAllReportedInIdOrderAndThePrimaryIsTheLargest() {
        FakeLimelight3A ll = started(WALL);
        ll.update(TICK);
        LLResult r = ll.getLatestResult();
        List<Integer> ids = new ArrayList<>();
        for (LLResultTypes.FiducialResult f : r.getFiducialResults()) {
            ids.add(f.getFiducialId());
        }
        assertEquals(Arrays.asList(1, 2, 3), ids);
        assertEquals(3, r.getBotposeTagCount());
        // Tag 2 is nearest (48 in), so largest: it is the primary target.
        LLResultTypes.FiducialResult nearest = r.getFiducialResults().get(1);
        assertEquals(nearest.getTargetArea(), r.getTa(), 0.0);
        assertEquals(nearest.getTargetXDegrees(), r.getTx(), 0.0);
        assertEquals(nearest.getTargetYDegrees(), r.getTy(), 0.0);
        double d1 = StrictMath.sqrt(72 * 72 + 12 * 12);
        double d2 = StrictMath.sqrt(48 * 48 + 2 * 2);
        assertEquals((2 * d1 + d2) / 3.0 * M, r.getBotposeAvgDist(), 1e-9);
        // Span: the largest distance between two seen tags' centres, here tag 1 to tag 2.
        assertEquals(StrictMath.sqrt(24 * 24 + 12 * 12 + 2 * 2) * M, r.getBotposeSpan(), 1e-9);
    }

    @Test
    void anEmptyLayoutNeverProducesAValidResult() {
        FakeLimelight3A ll = started(AprilTagLayout.empty());
        for (int i = 0; i < 5; i++) {
            ll.setRobotPose(i, -i, 0.1 * i);
            ll.update(TICK);
            LLResult r = ll.getLatestResult();
            assertFalse(r.isValid());
            assertTrue(r.getFiducialResults().isEmpty());
            assertEquals(0, r.getBotposeTagCount());
            assertEquals(0.0, metres(r.getBotpose().getPosition(), 'x'), 0.0);
            assertEquals(0.0, metres(r.getBotpose_MT2().getPosition(), 'x'), 0.0);
        }
    }

    @Test
    void turningAwayLosesTheTagAndTheResultGoesInvalid() {
        FakeLimelight3A ll = started(ONE_TAG);
        ll.update(TICK);
        assertTrue(ll.getLatestResult().isValid());
        ll.setRobotPose(0.0, 0.0, Math.PI);
        ll.update(TICK);
        assertFalse(ll.getLatestResult().isValid());
        assertEquals(0L, ll.getLatestResult().getStaleness(), "still polled: a fresh result with no target");
    }

    @Test
    void theRangeCutoffAndOccludersFromTheBuilderReachTheCamera() {
        FakeLimelight3A near = builder(ONE_TAG).rangeInches(0.0, 50.0).build();
        near.start();
        near.update(TICK);
        assertFalse(near.getLatestResult().isValid(), "the tag is 60 in away, past the 50 in cutoff");
        near.setRobotPose(15.0, 0.0, 0.0);
        near.update(TICK);
        assertTrue(near.getLatestResult().isValid());

        FakeLimelight3A blocked = builder(ONE_TAG).occluders(Arrays.asList(
                new org.horizon36596.simloop.field.ObstacleBox("post", 30, 34, -3, 3, 0, 40))).build();
        blocked.start();
        blocked.update(TICK);
        assertFalse(blocked.getLatestResult().isValid());
    }

    // ------------------------------------------------------------------ clause 5: pipelines and noise

    @Test
    void pipelineSwitchShowsInTheNextResultAndInTheStatus() {
        FakeLimelight3A ll = started(ONE_TAG);
        ll.update(TICK);
        assertEquals(0, ll.getLatestResult().getPipelineIndex());
        assertTrue(ll.pipelineSwitch(3));
        assertEquals(0, ll.getLatestResult().getPipelineIndex(), "the result already polled does not change");
        ll.update(TICK);
        assertEquals(3, ll.getLatestResult().getPipelineIndex());
        assertEquals(FakeLimelight3A.PIPELINE_TYPE, ll.getLatestResult().getPipelineType());
        LLStatus status = ll.getStatus();
        assertEquals(3, status.getPipelineIndex());
        assertEquals("limelight", status.getName());

        assertFalse(ll.pipelineSwitch(10));
        assertFalse(ll.pipelineSwitch(-1));
        ll.update(TICK);
        assertEquals(3, ll.getLatestResult().getPipelineIndex());
        assertTrue(ll.reloadPipeline());
    }

    @Test
    void pipelineSwitchWorksBeforeStartAsTheRestCallDoes() {
        FakeLimelight3A ll = builder(ONE_TAG).build();
        assertTrue(ll.pipelineSwitch(2));
        ll.start();
        ll.update(TICK);
        assertEquals(2, ll.getLatestResult().getPipelineIndex());
    }

    /** The JSON text of every result over a scripted drive. */
    private static List<String> scriptedRun(FakeLimelight3A ll) {
        ll.start();
        List<String> results = new ArrayList<>();
        for (int i = 0; i < 60; i++) {
            ll.setRobotPose(-20.0 + i * 0.5, 5.0 * StrictMath.sin(i * 0.1), 0.2 * StrictMath.cos(i * 0.07));
            ll.updateRobotOrientation(StrictMath.toDegrees(0.2 * StrictMath.cos(i * 0.07)));
            if (i == 30) {
                ll.pipelineSwitch(1);
            }
            ll.update(TICK);
            results.add(ll.getLatestResult().toString());
        }
        return results;
    }

    @Test
    void theSameSeedGivesByteIdenticalResultsAndADifferentSeedDoesNot() {
        List<String> a = scriptedRun(builder(WALL).noise(1234, 0.3, 0.5, 1.0).build());
        List<String> b = scriptedRun(builder(WALL).noise(1234, 0.3, 0.5, 1.0).build());
        List<String> c = scriptedRun(builder(WALL).noise(99, 0.3, 0.5, 1.0).build());
        assertEquals(a, b);
        assertNotEquals(a, c);
        for (int i = 0; i < a.size(); i++) {
            assertNotEquals(a.get(i), c.get(i), "every noisy frame differs between seeds, frame " + i);
        }
    }

    @Test
    void noiseIsOffByDefaultAndTwoNoiselessRunsAreIdentical() {
        List<String> a = scriptedRun(builder(WALL).build());
        List<String> b = scriptedRun(builder(WALL).build());
        List<String> zeroNoise = scriptedRun(builder(WALL).noise(5, 0.0, 0.0, 0.0).build());
        assertEquals(a, b);
        assertEquals(a, zeroNoise, "zero standard deviations draw nothing and change nothing");
    }

    @Test
    void noiseHasTheConfiguredSpreadAndNoBias() {
        double sigmaDeg = 0.5;
        double sigmaIn = 2.0;
        FakeLimelight3A ll = builder(ONE_TAG).noise(42, sigmaDeg, sigmaIn, 1.5).build();
        ll.start();
        int n = 4000;
        double[] txErr = new double[n];
        double[] xErr = new double[n];
        double[] mt2YErr = new double[n];
        double[] yawErr = new double[n];
        for (int i = 0; i < n; i++) {
            ll.update(TICK);
            LLResult r = ll.getLatestResult();
            txErr[i] = r.getFiducialResults().get(0).getTargetXDegrees();          // truth is 0
            xErr[i] = metres(r.getBotpose().getPosition(), 'x') / M;              // truth is 0 in
            mt2YErr[i] = metres(r.getBotpose_MT2().getPosition(), 'y') / M;
            yawErr[i] = r.getBotpose().getOrientation().getYaw(AngleUnit.DEGREES);
        }
        // Statistical bound: the sample mean is within 4 standard errors of 0, and the sample standard
        // deviation within 8% of the configured one (its own standard error at n = 4000 is about 1.1%).
        assertSpread(txErr, sigmaDeg);
        assertSpread(xErr, sigmaIn);
        assertSpread(mt2YErr, sigmaIn);
        assertSpread(yawErr, 1.5);
    }

    private static void assertSpread(double[] samples, double sigma) {
        double mean = 0.0;
        for (double s : samples) {
            mean += s;
        }
        mean /= samples.length;
        double var = 0.0;
        for (double s : samples) {
            var += (s - mean) * (s - mean);
        }
        double sd = StrictMath.sqrt(var / (samples.length - 1));
        assertEquals(0.0, mean, 4.0 * sigma / StrictMath.sqrt(samples.length), "mean");
        assertEquals(sigma, sd, 0.08 * sigma, "standard deviation");
    }

    // ------------------------------------------------------------------ clause 6: no network, no thread

    /** SDK methods that touch no network and no clock, so the fake keeps the SDK's own code. */
    private static final Set<String> SDK_METHODS_KEPT = new HashSet<>(Arrays.asList(
            "getManufacturer", "getDeviceName", "getVersion", "resetDeviceConfigurationForOpMode", "close",
            "shutdown", "isConnected"));

    @Test
    void everyPublicSdkMethodIsEitherOverriddenOrKnownToBeLocal() throws NoSuchMethodException {
        List<String> missing = new ArrayList<>();
        for (Method m : Limelight3A.class.getDeclaredMethods()) {
            if (!Modifier.isPublic(m.getModifiers()) || Modifier.isStatic(m.getModifiers())) {
                continue;
            }
            Method resolved = FakeLimelight3A.class.getMethod(m.getName(), m.getParameterTypes());
            boolean overridden = resolved.getDeclaringClass() == FakeLimelight3A.class;
            if (!overridden && !SDK_METHODS_KEPT.contains(m.getName())) {
                missing.add(m.toString());
            }
        }
        assertTrue(missing.isEmpty(), "SDK methods that would reach the network or wall clock: " + missing);
    }

    @Test
    void theNetworkOnlyMethodsFailLoud() {
        FakeLimelight3A ll = builder(ONE_TAG).build();
        assertThrows(UnsupportedOperationException.class, () -> ll.uploadFieldmap(null, 0));
        assertThrows(UnsupportedOperationException.class, () -> ll.uploadPipeline("{}", 0));
        assertThrows(UnsupportedOperationException.class, () -> ll.uploadPython("", 0));
        assertThrows(UnsupportedOperationException.class, () -> ll.updatePythonInputs(new double[8]));
        assertThrows(UnsupportedOperationException.class, () -> ll.updatePythonInputs(1, 2, 3, 4, 5, 6, 7, 8));
        assertThrows(UnsupportedOperationException.class, () -> ll.captureSnapshot("x"));
        assertThrows(UnsupportedOperationException.class, ll::deleteSnapshots);
        assertThrows(UnsupportedOperationException.class, () -> ll.deleteSnapshot("x"));
        assertThrows(UnsupportedOperationException.class, ll::getCalDefault);
        assertThrows(UnsupportedOperationException.class, ll::getCalFile);
        assertThrows(UnsupportedOperationException.class, ll::getCalEEPROM);
        assertThrows(UnsupportedOperationException.class, ll::getCalLatest);
    }

    @Test
    void buildingStartingAndPollingStartsNoThread() {
        Set<Thread> before = Thread.getAllStackTraces().keySet();
        FakeLimelight3A ll = started(WALL);
        for (int i = 0; i < 20; i++) {
            ll.update(TICK);
        }
        ll.stop();
        Set<Thread> after = new HashSet<>(Thread.getAllStackTraces().keySet());
        after.removeAll(before);
        assertTrue(after.isEmpty(), "new threads: " + after);
    }

    @Test
    void theBuilderRejectsNonsense() {
        assertThrows(IllegalArgumentException.class, () -> FakeLimelight3A.builder(null, LEVEL, 50, 40));
        assertThrows(IllegalArgumentException.class, () -> FakeLimelight3A.builder(ONE_TAG, null, 50, 40));
        assertThrows(IllegalArgumentException.class, () -> FakeLimelight3A.builder(ONE_TAG, LEVEL, 0, 40));
        assertThrows(IllegalArgumentException.class, () -> FakeLimelight3A.builder(ONE_TAG, LEVEL, 50, 180));
        assertThrows(IllegalArgumentException.class, () -> builder(ONE_TAG).frameRateHz(-1));
        assertThrows(IllegalArgumentException.class, () -> builder(ONE_TAG).latencyMs(-1, 0));
        assertThrows(IllegalArgumentException.class, () -> builder(ONE_TAG).noise(1, -0.1, 0, 0));
        assertThrows(IllegalArgumentException.class, () -> builder(ONE_TAG).imageSizePx(0, 480));
        assertThrows(IllegalArgumentException.class, () -> builder(ONE_TAG).rangeInches(10, 5).build());
        assertThrows(IllegalArgumentException.class, () -> builder(ONE_TAG).build().setRobotPose(Double.NaN, 0, 0));
    }
}
