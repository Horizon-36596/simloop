package org.horizon36596.simloop.vision;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import org.horizon36596.simloop.field.FieldWorld;
import org.horizon36596.simloop.field.GamePiece;
import org.horizon36596.simloop.field.GamePieceTracker;
import org.horizon36596.simloop.field.ObstacleBox;
import org.junit.jupiter.api.Test;

/**
 * INVARIANT-tier (domain R9) — the vision fault injection: {@link VisionFaults}, {@link VisionFaultInjector}
 * and {@link FaultedFrameStream}.
 *
 * <ol>
 *   <li>No faults changes nothing.</li>
 *   <li>Every fault replays for the same seed, and turning one fault on does not move another's draws.</li>
 *   <li>Each fault does what it says, at about the asked-for rate.</li>
 *   <li>No fault produces a non-finite number, a non-positive radius or a sighting outside the image, at any
 *       level: a NaN reaching a consumer must be the consumer's own doing.</li>
 * </ol>
 *
 * <p>A made-up rig: 70 degrees, 640 x 400, 6 in forward, 10 in up, 22 degrees down; balls of 1.4 in radius.
 */
class VisionFaultsTest {

    private static final CameraLens LENS = CameraLens.fromHorizontalFov(70.0, 640, 400);
    private static final CameraMount FRONT = CameraMount.front(6.0, 10.0, 22.0);
    private static final long MS = 1_000_000L;

    /** Six balls in front of the robot, some near the image edges. */
    private static List<CameraSighting> sightings() {
        GamePieceTracker tracker = new GamePieceTracker(8, FieldWorld.unbounded());
        tracker.updateRobotPose(0.0, 0.0, 0.0);
        double[][] xy = {{24, 0}, {30, 8}, {30, -8}, {40, 14}, {20, -6}, {50, 0}};
        for (double[] p : xy) {
            tracker.addLoosePiece(p[1] > 0 ? "red" : "blue", p[0], p[1], 1.4);
        }
        List<CameraSighting> seen = new SimCamera("t", LENS, FRONT).observe(tracker.getPieces(), 0.0, 0.0, 0.0);
        assertEquals(6, seen.size(), "the rig must see every ball: " + seen);
        return seen;
    }

    private static final AprilTagLayout WALL = AprilTagLayout.builder()
            .addTag(1, 72.0, -12.0, 10.0, Math.PI, 6.0)
            .addTag(2, 72.0, 0.0, 10.0, Math.PI, 6.0)
            .addTag(3, 72.0, 12.0, 10.0, Math.PI, 6.0)
            .build();

    private static SimTagCamera tagCamera() {
        return new SimTagCamera(LENS, new CameraMount(0.0, 0.0, 10.0, 0.0, 0.0), 0.0, Double.POSITIVE_INFINITY,
                90.0, Collections.<ObstacleBox>emptyList());
    }

    private static String describe(List<CameraSighting> list) {
        StringBuilder s = new StringBuilder();
        for (CameraSighting c : list) {
            s.append(c.getPieceId()).append(c.getKind()).append('@').append(c.getCentreXPx()).append(',')
                    .append(c.getCentreYPx()).append('r').append(c.getRadiusPx()).append(';');
        }
        return s.toString();
    }

    // ------------------------------------------------------------------------------------- none

    @Test
    void noFaultsPassesSightingsAndFramesThroughUnchanged() {
        List<CameraSighting> in = sightings();
        VisionFaultInjector injector = new VisionFaultInjector(VisionFaults.none());
        assertEquals(describe(in), describe(injector.applyToSightings(in, LENS)));
        assertTrue(VisionFaults.none().isNone());
        assertEquals("VisionFaults(none)", VisionFaults.none().toString());

        FaultedFrameStream<Integer> stream = new FaultedFrameStream<Integer>(VisionFaults.none());
        List<Integer> got = new ArrayList<Integer>();
        for (int i = 0; i < 50; i++) {
            stream.offer(i * 20 * MS, i * 20 * MS + 5 * MS, i);
            for (FaultedFrameStream.Delivery<Integer> d : stream.pollDue(i * 20 * MS + 5 * MS)) {
                assertEquals(d.getFrame() * 20 * MS, d.getCaptureNanos());
                assertEquals(d.getFrame() * 20 * MS + 5 * MS, d.getDeliveredNanos());
                got.add(d.getFrame());
            }
        }
        for (int i = 0; i < 50; i++) {
            assertEquals(Integer.valueOf(i), got.get(i));
        }
        assertEquals(50, got.size());
    }

    // ---------------------------------------------------------------------------- determinism

    private static String sightingRun(VisionFaults faults, int frames) {
        VisionFaultInjector injector = new VisionFaultInjector(faults);
        List<CameraSighting> in = sightings();
        StringBuilder s = new StringBuilder();
        for (int i = 0; i < frames; i++) {
            s.append(describe(injector.applyToSightings(in, LENS))).append('|');
        }
        return s.toString();
    }

    @Test
    void theSameSeedReplaysAndAnotherSeedDoesNot() {
        VisionFaults a = VisionFaults.builder(7).pixelNoise(5.0, 2.0).missDetections(0.2).blackout(0.05, 3)
                .falseDetections(0.3, "false", 3.0, 20.0, 0xFFFF00).wrongIds(0.2, "red", "blue").build();
        VisionFaults b = VisionFaults.builder(8).pixelNoise(5.0, 2.0).missDetections(0.2).blackout(0.05, 3)
                .falseDetections(0.3, "false", 3.0, 20.0, 0xFFFF00).wrongIds(0.2, "red", "blue").build();
        assertEquals(sightingRun(a, 200), sightingRun(a, 200));
        assertNotEquals(sightingRun(a, 200), sightingRun(b, 200));
    }

    @Test
    void turningOneFaultOnDoesNotMoveAnotherFaultsDraws() {
        // The misses alone, and the misses with heavy noise on top: the same sightings must go missing.
        VisionFaults missOnly = VisionFaults.builder(3).missDetections(0.3).build();
        VisionFaults missAndNoise = VisionFaults.builder(3).missDetections(0.3).pixelNoise(0.001, 0.0).build();
        VisionFaultInjector x = new VisionFaultInjector(missOnly);
        VisionFaultInjector y = new VisionFaultInjector(missAndNoise);
        List<CameraSighting> in = sightings();
        for (int i = 0; i < 200; i++) {
            List<Integer> idsX = ids(x.applyToSightings(in, LENS));
            List<Integer> idsY = ids(y.applyToSightings(in, LENS));
            assertEquals(idsX, idsY, "frame " + i);
        }

        // The drops alone, and the drops with jitter: the same frames must be dropped.
        FaultedFrameStream<Integer> dropOnly = new FaultedFrameStream<Integer>(
                VisionFaults.builder(3).dropFrames(0.3).build());
        FaultedFrameStream<Integer> dropAndJitter = new FaultedFrameStream<Integer>(
                VisionFaults.builder(3).dropFrames(0.3).latency(0.0, 0.004).build());
        assertEquals(deliveredFrames(dropOnly, 300), deliveredFrames(dropAndJitter, 300));
    }

    private static List<Integer> ids(List<CameraSighting> list) {
        List<Integer> out = new ArrayList<Integer>();
        for (CameraSighting s : list) {
            out.add(s.getPieceId());
        }
        return out;
    }

    /** Offers a frame every 20 ms for {@code frames} frames and returns what came out, in delivery order. */
    private static List<Integer> deliveredFrames(FaultedFrameStream<Integer> stream, int frames) {
        List<Integer> out = new ArrayList<Integer>();
        for (int i = 0; i < frames + 50; i++) {
            if (i < frames) {
                stream.offer(i * 20 * MS, i * 20 * MS, i);
            }
            for (FaultedFrameStream.Delivery<Integer> d : stream.pollDue(i * 20 * MS)) {
                out.add(d.getFrame());
            }
        }
        return out;
    }

    // ------------------------------------------------------------------------------- content faults

    @Test
    void extremeNoiseNeverProducesANonFiniteNumberANonPositiveRadiusOrADiscOffTheImage() {
        VisionFaultInjector injector = new VisionFaultInjector(
                VisionFaults.builder(11).pixelNoise(400.0, 60.0).build());
        List<CameraSighting> in = sightings();
        int kept = 0;
        for (int i = 0; i < 2000; i++) {
            for (CameraSighting s : injector.applyToSightings(in, LENS)) {
                kept++;
                assertTrue(Double.isFinite(s.getCentreXPx()) && Double.isFinite(s.getCentreYPx())
                        && Double.isFinite(s.getRadiusPx()) && Double.isFinite(s.getRangeInches()), s.toString());
                assertTrue(s.getRadiusPx() > 0.0, s.toString());
                double nearX = Math.max(0.0, Math.min(640.0, s.getCentreXPx()));
                double nearY = Math.max(0.0, Math.min(400.0, s.getCentreYPx()));
                assertTrue(Math.hypot(s.getCentreXPx() - nearX, s.getCentreYPx() - nearY) < s.getRadiusPx(),
                        "the disc overlaps the image: " + s);
            }
        }
        assertTrue(kept > 0 && kept < 2000 * 6, "noise this heavy must drop some sightings and keep some: " + kept);
    }

    @Test
    void noiseHasAboutTheAskedForSpread() {
        VisionFaultInjector injector = new VisionFaultInjector(VisionFaults.builder(5).pixelNoise(3.0, 0.0).build());
        CameraSighting centreBall = sightings().get(0);
        double sum = 0.0;
        double sumSq = 0.0;
        int n = 4000;
        for (int i = 0; i < n; i++) {
            double dx = injector.applyToSightings(Collections.singletonList(centreBall), LENS).get(0).getCentreXPx()
                    - centreBall.getCentreXPx();
            sum += dx;
            sumSq += dx * dx;
        }
        double mean = sum / n;
        double sd = Math.sqrt(sumSq / n - mean * mean);
        assertEquals(0.0, mean, 0.2);
        assertEquals(3.0, sd, 0.15);
    }

    @Test
    void missesAndBlackoutsRemoveSightingsAtAboutTheRate() {
        VisionFaultInjector miss = new VisionFaultInjector(VisionFaults.builder(2).missDetections(0.25).build());
        List<CameraSighting> in = sightings();
        int seen = 0;
        for (int i = 0; i < 1000; i++) {
            seen += miss.applyToSightings(in, LENS).size();
        }
        assertEquals(0.25, 1.0 - seen / 6000.0, 0.03);

        VisionFaultInjector blackout = new VisionFaultInjector(VisionFaults.builder(2).blackout(0.05, 5).build());
        int run = 0;
        List<Integer> runs = new ArrayList<Integer>();
        for (int i = 0; i < 2000; i++) {
            if (blackout.applyToSightings(in, LENS).isEmpty()) {
                run++;
            } else if (run > 0) {
                runs.add(run);
                run = 0;
            }
        }
        assertFalse(runs.isEmpty(), "a 5% blackout chance over 2000 frames must black some out");
        for (int r : runs) {
            assertEquals(0, r % 5, "a blackout lasts whole multiples of 5 frames, back to back at most: " + runs);
        }
        assertEquals(blackout.getBlackedOutFrames(), sum(runs) + run);
        assertEquals(2000, blackout.getFramesSeen());
    }

    private static int sum(List<Integer> values) {
        int total = 0;
        for (int v : values) {
            total += v;
        }
        return total;
    }

    @Test
    void falseDetectionsAreInTheImageFiniteAndCarryNoPieceId() {
        VisionFaultInjector injector = new VisionFaultInjector(
                VisionFaults.builder(4).falseDetections(0.5, "phantom", 3.0, 25.0, 0xFFFFFF).build());
        List<CameraSighting> in = sightings();
        int phantoms = 0;
        for (int i = 0; i < 1000; i++) {
            for (CameraSighting s : injector.applyToSightings(in, LENS)) {
                if (s.getPieceId() == VisionFaultInjector.FALSE_PIECE_ID) {
                    phantoms++;
                    assertEquals("phantom", s.getKind());
                    assertTrue(s.getRadiusPx() >= 3.0 && s.getRadiusPx() <= 25.0, s.toString());
                    assertTrue(s.getCentreXPx() >= 0.0 && s.getCentreXPx() <= 640.0, s.toString());
                    assertTrue(Double.isFinite(s.getRangeInches()) && s.getRangeInches() > 0.0, s.toString());
                }
            }
        }
        assertEquals(500, phantoms, 50);
    }

    @Test
    void wrongIdsRelabelPiecesAsAnotherKindAndKeepThePieceId() {
        VisionFaultInjector injector = new VisionFaultInjector(
                VisionFaults.builder(9).wrongIds(1.0, "red", "blue").build());
        List<CameraSighting> in = sightings();
        List<CameraSighting> out = injector.applyToSightings(in, LENS);
        assertEquals(in.size(), out.size());
        for (int i = 0; i < in.size(); i++) {
            assertEquals(in.get(i).getPieceId(), out.get(i).getPieceId());
            assertNotEquals(in.get(i).getKind(), out.get(i).getKind());
        }
    }

    @Test
    void tagFaultsMislabelMissAndInventTagsWithWrongGeometry() {
        SimTagCamera camera = tagCamera();
        List<TagSighting> truth = camera.observe(WALL, 0.0, 0.0, 0.0);
        assertEquals(3, truth.size());

        VisionFaultInjector wrong = new VisionFaultInjector(VisionFaults.builder(1).wrongIds(1.0).build());
        List<TagSighting> relabelled = wrong.applyToTags(truth, WALL, camera, 0.0, 0.0, 0.0);
        for (int i = 0; i < truth.size(); i++) {
            assertNotEquals(truth.get(i).getId(), relabelled.get(i).getId());
            assertSame(WALL.getTag(relabelled.get(i).getId()), relabelled.get(i).getTag());
            assertEquals(truth.get(i).getRobotToTag().getXInches(), relabelled.get(i).getRobotToTag().getXInches());
        }

        VisionFaultInjector phantom = new VisionFaultInjector(
                VisionFaults.builder(1).falseDetections(1.0, "tag", 1.0, 1.0, 0).build());
        int found = 0;
        for (int i = 0; i < 50; i++) {
            List<TagSighting> out = phantom.applyToTags(truth, WALL, camera, 0.0, 0.0, 0.0);
            if (out.size() == 4) {
                found++;
                TagSighting p = out.get(3);
                assertSame(WALL.getTag(p.getId()), p.getTag(), "a false tag claims a layout tag");
                // Where the robot would be if the tag really were the layout tag: not where it is.
                RigidTransform3 solved = p.getTag().getFieldToTag().compose(p.getRobotToTag().inverse());
                assertTrue(Math.hypot(solved.getXInches(), solved.getYInches()) > 1.0,
                        "a false tag must not solve to the true pose: " + solved);
                assertTrue(p.getCentreXPx() >= 0.0 && p.getCentreXPx() <= 640.0);
            }
        }
        assertTrue(found > 40, "false tags placed: " + found);

        VisionFaultInjector noisy = new VisionFaultInjector(VisionFaults.builder(1).tagAngleNoise(80.0).build());
        for (int i = 0; i < 500; i++) {
            for (TagSighting t : noisy.applyToTags(truth, WALL, camera, 0.0, 0.0, 0.0)) {
                assertTrue(Math.abs(t.getTxDegrees()) <= 35.0 + 1e-9, "tx stays inside the 70 degree image: " + t);
                assertTrue(t.getCentreXPx() >= 0.0 && t.getCentreXPx() <= 640.0, t.toString());
                assertTrue(t.getCentreYPx() >= 0.0 && t.getCentreYPx() <= 400.0, t.toString());
            }
        }
    }

    @Test
    void frameFaultsPaintBlackoutsPatchesBlobsAndNoiseAndRelabelWhatTheyCover() {
        GamePieceTracker tracker = new GamePieceTracker(4, FieldWorld.unbounded());
        tracker.updateRobotPose(0.0, 0.0, 0.0);
        int id = tracker.addLoosePiece("red", 30.0, 0.0, 1.4);
        List<GamePiece> pieces = tracker.getPieces();
        SimFrameRenderer renderer = SimFrameRenderer.builder(CameraLens.fromHorizontalFov(70.0, 160, 100), FRONT)
                .build();
        SimFrame frame = renderer.render(pieces, 0.0, 0.0, 0.0, CameraOccluders.none(), AprilTagLayout.empty());
        assertTrue(frame.pixelCountOfPiece(id) > 0);

        SimFrame black = new VisionFaultInjector(VisionFaults.builder(1).blackout(1.0, 1).build()).applyToFrame(frame);
        for (byte b : black.getRgbBytes()) {
            assertEquals(0, b);
        }
        assertEquals(0, black.pixelCountOfPiece(id));
        assertTrue(frame.pixelCountOfPiece(id) > 0, "the input frame is unchanged");

        SimFrame patched = new VisionFaultInjector(VisionFaults.builder(1).occlusionPatch(1.0, 1.0).build())
                .applyToFrame(frame);
        assertEquals(0, patched.pixelCountOfPiece(id), "a full-frame patch hides the piece");
        assertEquals(0x303030, patched.getRgb(80, 50));

        SimFrame blob = new VisionFaultInjector(
                VisionFaults.builder(1).falseDetections(1.0, "x", 10.0, 10.0, 0x00FF00).build()).applyToFrame(frame);
        int green = 0;
        for (int y = 0; y < 100; y++) {
            for (int x = 0; x < 160; x++) {
                if (blob.getRgb(x, y) == 0x00FF00) {
                    green++;
                    assertEquals(-1, blob.getPieceIdAt(x, y), "a false blob is no piece");
                }
            }
        }
        assertTrue(green > 100, "a 10 px blob: " + green + " pixels");

        SimFrame noisy = new VisionFaultInjector(VisionFaults.builder(1).frameNoise(500.0).build())
                .applyToFrame(frame);
        assertNotEquals(java.util.Arrays.toString(frame.getRgbBytes()), java.util.Arrays.toString(noisy.getRgbBytes()));
        assertEquals(frame.pixelCountOfPiece(id), noisy.pixelCountOfPiece(id), "noise does not move labels");
    }

    // -------------------------------------------------------------------------------- timing faults

    @Test
    void latencyAndJitterDelayFramesWithoutReorderingThemOrDeliveringBeforeCapture() {
        FaultedFrameStream<Integer> stream = new FaultedFrameStream<Integer>(
                VisionFaults.builder(6).latency(0.100, 0.060).build());
        long lastCapture = Long.MIN_VALUE;
        double sumDelay = 0.0;
        int n = 0;
        for (int i = 0; i < 2000; i++) {
            long now = i * 20 * MS;
            stream.offer(now, now, i);
            for (FaultedFrameStream.Delivery<Integer> d : stream.pollDue(now)) {
                assertTrue(d.getCaptureNanos() > lastCapture, "in order");
                lastCapture = d.getCaptureNanos();
                assertTrue(d.getDeliveredNanos() >= d.getCaptureNanos());
                assertTrue(now >= d.getDeliveredNanos());
                sumDelay += (d.getDeliveredNanos() - d.getCaptureNanos()) / 1e9;
                n++;
            }
        }
        double meanDelay = sumDelay / n;
        // 100 ms plus jitter that is symmetric before the in-order rule pushes some frames later.
        assertTrue(meanDelay >= 0.095 && meanDelay <= 0.130, "mean delay " + meanDelay);
    }

    @Test
    void droppedFramesAreNeverDeliveredAtAboutTheRate() {
        FaultedFrameStream<Integer> stream = new FaultedFrameStream<Integer>(
                VisionFaults.builder(6).dropFrames(0.2).build());
        List<Integer> out = deliveredFrames(stream, 2000);
        assertEquals(0.2, 1.0 - out.size() / 2000.0, 0.03);
        assertEquals(2000, stream.getOffered());
        assertEquals(2000 - out.size(), stream.getDropped());
    }

    @Test
    void aRepeatedFrameComesAgainWithTheSameStampOnTheNextPoll() {
        FaultedFrameStream<Integer> stream = new FaultedFrameStream<Integer>(
                VisionFaults.builder(6).repeatFrames(0.3).build());
        List<FaultedFrameStream.Delivery<Integer>> all = new ArrayList<FaultedFrameStream.Delivery<Integer>>();
        for (int i = 0; i < 500; i++) {
            stream.offer(i * 20 * MS, i * 20 * MS, i);
            all.addAll(stream.pollDue(i * 20 * MS));
        }
        int repeats = 0;
        for (int i = 1; i < all.size(); i++) {
            if (all.get(i).isRepeat()) {
                repeats++;
                assertEquals(all.get(i - 1).getCaptureNanos(), all.get(i).getCaptureNanos(),
                        "a repeat follows its original, same stamp");
                assertSame(all.get(i - 1).getFrame(), all.get(i).getFrame());
            }
        }
        assertEquals(stream.getRepeated(), repeats);
        assertEquals(0.3, repeats / 500.0, 0.06);
    }

    @Test
    void aReorderedFrameArrivesAfterTheNextOneSoStampsGoBackwards() {
        FaultedFrameStream<Integer> stream = new FaultedFrameStream<Integer>(
                VisionFaults.builder(6).reorderFrames(0.2).build());
        List<FaultedFrameStream.Delivery<Integer>> all = new ArrayList<FaultedFrameStream.Delivery<Integer>>();
        for (int i = 0; i < 1050; i++) {
            if (i < 1000) {
                stream.offer(i * 20 * MS, i * 20 * MS, i);
            }
            all.addAll(stream.pollDue(i * 20 * MS));
        }
        // A frame flagged out of order is older than one already delivered; every other frame is newer than
        // everything before it. (Two held frames released on one poll can come out oldest first, so counting
        // backward steps between neighbours would undercount.)
        long newest = Long.MIN_VALUE;
        int outOfOrder = 0;
        List<Integer> out = new ArrayList<Integer>();
        for (FaultedFrameStream.Delivery<Integer> d : all) {
            out.add(d.getFrame());
            if (d.isOutOfOrder()) {
                outOfOrder++;
                assertTrue(d.getCaptureNanos() < newest, "frame " + d.getFrame() + " came after a newer one");
            } else {
                assertTrue(d.getCaptureNanos() > newest, "frame " + d.getFrame() + " is in order");
            }
            newest = Math.max(newest, d.getCaptureNanos());
        }
        assertTrue(outOfOrder > 100, "out-of-order deliveries: " + outOfOrder);
        // Every frame comes out once, except a run of held frames at the very end, each waiting on a next
        // frame that is never offered: those are the last frames, and they are still in flight.
        int neverDelivered = 1000 - out.size();
        List<Integer> sorted = new ArrayList<Integer>(out);
        Collections.sort(sorted);
        for (int i = 0; i < sorted.size(); i++) {
            assertEquals(i, (int) sorted.get(i));
        }
        assertTrue(neverDelivered <= 10, "frames never delivered: " + neverDelivered);
        assertEquals(neverDelivered, stream.inFlight(), "the undelivered frames are still held");
        assertTrue(outOfOrder <= stream.getReordered() && outOfOrder >= stream.getReordered() - neverDelivered,
                outOfOrder + " out of order, " + stream.getReordered() + " reordered, " + neverDelivered + " held");
    }

    @Test
    void badArgumentsAreRefused() {
        assertThrows(IllegalArgumentException.class, () -> VisionFaults.builder(1).dropFrames(1.5));
        assertThrows(IllegalArgumentException.class, () -> VisionFaults.builder(1).latency(-0.1, 0.0));
        assertThrows(IllegalArgumentException.class, () -> VisionFaults.builder(1).pixelNoise(Double.NaN, 0.0));
        assertThrows(IllegalArgumentException.class, () -> VisionFaults.builder(1).blackout(0.5, 0));
        assertThrows(IllegalArgumentException.class, () -> VisionFaults.builder(1).occlusionPatch(0.5, 0.0));
        assertThrows(IllegalArgumentException.class,
                () -> VisionFaults.builder(1).falseDetections(0.5, "x", 5.0, 2.0, 0));
        assertThrows(IllegalArgumentException.class, () -> new VisionFaultInjector(null));
        FaultedFrameStream<Integer> stream = new FaultedFrameStream<Integer>(VisionFaults.none());
        stream.offer(10 * MS, 10 * MS, 1);
        assertThrows(IllegalArgumentException.class, () -> stream.offer(5 * MS, 5 * MS, 2),
                "frames are offered in capture order");
        assertThrows(IllegalArgumentException.class, () -> stream.offer(20 * MS, 19 * MS, 3),
                "ready before captured");
    }
}
