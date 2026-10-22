package org.horizon36596.simloop.vision;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.OutputStream;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.zip.CRC32;
import java.util.zip.DataFormatException;
import java.util.zip.Inflater;

import org.horizon36596.simloop.field.FieldWorld;
import org.horizon36596.simloop.field.GamePiece;
import org.horizon36596.simloop.field.GamePieceTracker;
import org.horizon36596.simloop.field.ObstacleBox;
import org.junit.jupiter.api.Test;

/**
 * INVARIANT-tier (domain R9) — the synthetic frame renderer agrees with the geometric cameras, so a pixel
 * pipeline run on its frames is being tested against the same scene the rest of the sim describes:
 *
 * <ol>
 *   <li>a piece is drawn where {@link SimCamera} says it is, about the size it says, through a pinhole or a
 *       distorted lens;</li>
 *   <li>what {@link SimCamera}'s occluded {@code observe} hides (behind a field box, behind the robot's own
 *       body) has no pixels in the frame;</li>
 *   <li>a tag is drawn inside the corners {@link SimTagCamera} reports, white border outside them;</li>
 *   <li>frames are deterministic: same inputs, same bytes; sensor noise is seeded;</li>
 *   <li>the PNG is a valid PNG with the pixels in it, and says it is synthetic.</li>
 * </ol>
 *
 * <p>The scene, colours and boxes are all invented for the test. Nothing here measures a real camera.
 */
class SimFrameRendererTest {

    private static final CameraLens LENS = CameraLens.fromHorizontalFov(70.0, 320, 240);
    private static final CameraMount FRONT = CameraMount.front(6.0, 10.0, 22.0);
    private static final double RADIUS = 2.5;
    private static final List<ObstacleBox> NONE = Collections.emptyList();
    private static final FrameStyle STYLE = FrameStyle.builder()
            .pieceColour("red", 0xC02020)
            .pieceColour("blue", 0x2040C0)
            .build();

    private static GamePieceTracker pieces(Object... kindXY) {
        GamePieceTracker tracker = new GamePieceTracker(4, FieldWorld.unbounded());
        tracker.updateRobotPose(0.0, 0.0, 0.0);
        for (int i = 0; i < kindXY.length; i += 3) {
            tracker.addLoosePiece((String) kindXY[i], (Double) kindXY[i + 1], (Double) kindXY[i + 2], RADIUS);
        }
        return tracker;
    }

    private static SimFrameRenderer renderer(CameraLens lens) {
        return SimFrameRenderer.builder(lens, FRONT).style(STYLE).build();
    }

    @Test
    void aPieceIsDrawnWhereSimCameraSaysItIs() {
        GamePieceTracker tracker = pieces("red", 30.0, 4.0, "blue", 40.0, -12.0);
        SimFrame frame = renderer(LENS).render(tracker.getPieces(), 0.0, 0.0, 0.0, CameraOccluders.none(),
                AprilTagLayout.empty());
        List<CameraSighting> seen = new SimCamera("front", LENS, FRONT).observe(tracker.getPieces(), 0.0, 0.0, 0.0);
        assertEquals(2, seen.size());
        for (CameraSighting s : seen) {
            double[] centroid = frame.centroidOfPiece(s.getPieceId());
            assertNotNull(centroid);
            // A sphere's image is an ellipse a little larger than the projected centre's disc and stretched
            // away from the image centre, so allow a pixel; the sighting is the projected centre.
            assertEquals(s.getCentreXPx(), centroid[0], 1.0, "piece " + s.getPieceId() + " x");
            assertEquals(s.getCentreYPx(), centroid[1], 1.0, "piece " + s.getPieceId() + " y");
            double area = frame.pixelCountOfPiece(s.getPieceId());
            double disc = Math.PI * s.getRadiusPx() * s.getRadiusPx();
            assertTrue(area > 0.85 * disc && area < 1.35 * disc, "area " + area + " vs disc " + disc);
        }
        // The kind's colour, shaded: red pixels are red.
        double[] c = frame.centroidOfPiece(seen.get(0).getPieceId());
        int rgb = frame.getRgb((int) c[0], (int) c[1]);
        assertTrue(((rgb >> 16) & 0xFF) > 2 * ((rgb >> 8) & 0xFF), "red piece pixel 0x" + Integer.toHexString(rgb));
    }

    @Test
    void aDistortedLensDrawsThePieceWhereTheDistortedCameraSaysItIs() {
        CameraLens barrel = LENS.withDistortion(-0.30, 0.08, 0.0, 0.0, 0.0);
        GamePieceTracker tracker = pieces("red", 26.0, 13.0);
        SimFrame frame = renderer(barrel).render(tracker.getPieces(), 0.0, 0.0, 0.0, CameraOccluders.none(),
                AprilTagLayout.empty());
        CameraSighting distorted = new SimCamera("front", barrel, FRONT).observe(tracker.getPieces(), 0.0, 0.0, 0.0).get(0);
        CameraSighting pinhole = new SimCamera("front", LENS, FRONT).observe(tracker.getPieces(), 0.0, 0.0, 0.0).get(0);
        double[] centroid = frame.centroidOfPiece(distorted.getPieceId());
        assertEquals(distorted.getCentreXPx(), centroid[0], 1.0);
        assertEquals(distorted.getCentreYPx(), centroid[1], 1.0);
        assertTrue(Math.abs(pinhole.getCentreXPx() - centroid[0]) > 3.0,
                "the distortion moved the piece by more than the tolerance, or this test proves nothing");
    }

    @Test
    void whatTheOccludedObserveHidesHasNoPixels() {
        GamePieceTracker tracker = pieces("red", 34.0, 0.0, "blue", 30.0, 14.0);
        ObstacleBox wall = new ObstacleBox("wall", 22.0, 23.0, -5.0, 5.0, 0.0, 14.0);
        CameraOccluders boxes = CameraOccluders.field(Arrays.asList(wall));
        SimFrame frame = renderer(LENS).render(tracker.getPieces(), 0.0, 0.0, 0.0, boxes, AprilTagLayout.empty());
        List<CameraSighting> seen = new SimCamera("front", LENS, FRONT).observe(tracker.getPieces(), 0.0, 0.0, 0.0,
                boxes);
        assertEquals(1, seen.size(), "only the blue piece, beside the wall, is seen");
        assertEquals(0, frame.pixelCountOfPiece(0), "the red piece behind the wall");
        assertTrue(frame.pixelCountOfPiece(seen.get(0).getPieceId()) > 0);
        // The wall itself is drawn, in the box colour, in front of where the red piece would be.
        CameraSighting hidden = new SimCamera("front", LENS, FRONT).observe(tracker.getPieces(), 0.0, 0.0, 0.0).get(0);
        int rgb = frame.getRgb((int) hidden.getCentreXPx(), (int) hidden.getCentreYPx());
        assertEquals(-1, frame.getPieceIdAt((int) hidden.getCentreXPx(), (int) hidden.getCentreYPx()));
        assertTrue(Math.abs(((rgb >> 16) & 0xFF) - (rgb & 0xFF)) < 40, "box grey, got 0x" + Integer.toHexString(rgb));
    }

    @Test
    void theRobotsOwnBodyHidesAPieceAndTurnsWithTheRobot() {
        // A raised arm 8-9 in ahead of the robot centre, across the camera's view.
        ObstacleBox arm = new ObstacleBox("arm", 8.0, 9.0, -6.0, 6.0, 2.0, 12.0);
        CameraOccluders body = CameraOccluders.of(NONE, Arrays.asList(arm));
        GamePieceTracker tracker = new GamePieceTracker(4, FieldWorld.unbounded());
        tracker.addLoosePiece("red", -30.0, 0.0, RADIUS);   // ahead of a robot facing -X
        double heading = Math.PI;
        SimFrame open = renderer(LENS).render(tracker.getPieces(), 0.0, 0.0, heading, CameraOccluders.none(),
                AprilTagLayout.empty());
        SimFrame blocked = renderer(LENS).render(tracker.getPieces(), 0.0, 0.0, heading, body, AprilTagLayout.empty());
        assertTrue(open.pixelCountOfPiece(0) > 0);
        assertEquals(0, blocked.pixelCountOfPiece(0));
        assertTrue(new SimCamera("front", LENS, FRONT).observe(tracker.getPieces(), 0.0, 0.0, heading, body).isEmpty());
    }

    @Test
    void aTagIsDrawnInsideTheCornersSimTagCameraReports() {
        AprilTagLayout layout = AprilTagLayout.builder()
                .addTag(5, 40.0, 0.0, 6.0, Math.PI, 6.0)
                .addTiltedTag(9, 30.0, 8.0, 0.05, Math.PI, Math.PI / 2.0, 6.0)   // lying face up on the floor
                .build();
        SimFrame frame = renderer(LENS).render(Collections.<GamePiece>emptyList(), 0.0, 0.0, 0.0,
                CameraOccluders.none(), layout);
        SimTagCamera tags = new SimTagCamera(LENS, FRONT, 0.0, Double.POSITIVE_INFINITY, 90.0, NONE);
        List<TagSighting> seen = tags.observe(layout, 0.0, 0.0, 0.0);
        assertEquals(2, seen.size());
        for (TagSighting s : seen) {
            double[] c = {s.getCentreXPx(), s.getCentreYPx()};
            assertEquals(s.getId(), frame.getTagIdAt((int) c[0], (int) c[1]), "tag " + s.getId() + " centre");
            // Every pixel inside the black square's corners belongs to the tag.
            double[][] corners = s.getCornersPx();
            double minX = Double.MAX_VALUE;
            double maxX = -Double.MAX_VALUE;
            double minY = Double.MAX_VALUE;
            double maxY = -Double.MAX_VALUE;
            for (double[] corner : corners) {
                minX = Math.min(minX, corner[0]);
                maxX = Math.max(maxX, corner[0]);
                minY = Math.min(minY, corner[1]);
                maxY = Math.max(maxY, corner[1]);
            }
            int tagPixels = 0;
            int outside = 0;
            for (int y = 0; y < frame.getHeightPx(); y++) {
                for (int x = 0; x < frame.getWidthPx(); x++) {
                    if (frame.getTagIdAt(x, y) == s.getId()) {
                        tagPixels++;
                        // The white border is a cell wide; nothing of the tag lies beyond 1.5 cells out.
                        double margin = 0.25 * Math.max(maxX - minX, maxY - minY) + 2.0;
                        if (x < minX - margin || x > maxX + margin || y < minY - margin || y > maxY + margin) {
                            outside++;
                        }
                    }
                }
            }
            assertTrue(tagPixels > 20, "tag " + s.getId() + " has " + tagPixels + " pixels");
            assertEquals(0, outside, "tag " + s.getId() + " pixels far outside its corners");
        }
    }

    @Test
    void framesAreDeterministicAndNoiseIsSeeded() {
        GamePieceTracker tracker = pieces("red", 30.0, 4.0, "blue", 40.0, -12.0);
        ObstacleBox wall = new ObstacleBox("wall", 60.0, 62.0, -40.0, 40.0, 0.0, 12.0);
        CameraOccluders boxes = CameraOccluders.field(Arrays.asList(wall));
        byte[] a = renderer(LENS).render(tracker.getPieces(), 1.0, 2.0, 0.1, boxes, AprilTagLayout.empty()).getRgbBytes();
        byte[] b = renderer(LENS).render(tracker.getPieces(), 1.0, 2.0, 0.1, boxes, AprilTagLayout.empty()).getRgbBytes();
        assertArrayEquals(a, b);

        SimFrameRenderer noisyA = SimFrameRenderer.builder(LENS, FRONT).style(STYLE).sensorNoise(7, 3.0).build();
        SimFrameRenderer noisyB = SimFrameRenderer.builder(LENS, FRONT).style(STYLE).sensorNoise(7, 3.0).build();
        SimFrameRenderer otherSeed = SimFrameRenderer.builder(LENS, FRONT).style(STYLE).sensorNoise(8, 3.0).build();
        SimFrame first = noisyA.render(tracker.getPieces(), 1.0, 2.0, 0.1, boxes, AprilTagLayout.empty());
        SimFrame second = noisyA.render(tracker.getPieces(), 1.0, 2.0, 0.1, boxes, AprilTagLayout.empty());
        assertArrayEquals(first.getRgbBytes(),
                noisyB.render(tracker.getPieces(), 1.0, 2.0, 0.1, boxes, AprilTagLayout.empty()).getRgbBytes());
        assertFalse(Arrays.equals(first.getRgbBytes(), second.getRgbBytes()), "frame 1's noise is not frame 0's");
        assertFalse(Arrays.equals(first.getRgbBytes(),
                otherSeed.render(tracker.getPieces(), 1.0, 2.0, 0.1, boxes, AprilTagLayout.empty()).getRgbBytes()));
        assertFalse(Arrays.equals(a, first.getRgbBytes()));
        assertEquals(0L, first.getFrameNumber());
        assertEquals(1L, second.getFrameNumber());

        // The noise has about the asked-for spread: compare against the noise-free frame.
        double sum = 0.0;
        double sumSq = 0.0;
        byte[] clean = a;
        byte[] noisy = first.getRgbBytes();
        int n = 0;
        for (int i = 0; i < clean.length; i++) {
            int c = clean[i] & 0xFF;
            if (c < 20 || c > 235) {
                continue;                                   // clamped values would bias the spread
            }
            double d = (noisy[i] & 0xFF) - c;
            sum += d;
            sumSq += d * d;
            n++;
        }
        double mean = sum / n;
        double sd = Math.sqrt(sumSq / n - mean * mean);
        // Flat surfaces have one colour each, so rounding both frames to whole counts can shift the mean
        // by up to half a count; the noise itself is zero-mean.
        assertEquals(0.0, mean, 0.5);
        assertEquals(3.0, sd, 0.3, "rounding to whole counts adds about 0.3 in quadrature");
    }

    @Test
    void supersamplingKeepsLabelsOnPixelCentresAndSmoothsEdges() {
        GamePieceTracker tracker = pieces("red", 30.0, 4.0);
        SimFrame one = renderer(LENS).render(tracker.getPieces(), 0.0, 0.0, 0.0, CameraOccluders.none(),
                AprilTagLayout.empty());
        SimFrame four = SimFrameRenderer.builder(LENS, FRONT).style(STYLE).supersampling(2).build()
                .render(tracker.getPieces(), 0.0, 0.0, 0.0, CameraOccluders.none(), AprilTagLayout.empty());
        assertEquals(one.pixelCountOfPiece(0), four.pixelCountOfPiece(0));
        // Some edge pixel is a blend: neither the floor's colour nor the piece's.
        int blends = 0;
        for (int y = 0; y < four.getHeightPx(); y++) {
            for (int x = 0; x < four.getWidthPx(); x++) {
                if (four.getRgb(x, y) != one.getRgb(x, y)) {
                    blends++;
                }
            }
        }
        assertTrue(blends > 0);
    }

    @Test
    void thePngIsValidHoldsThePixelsAndSaysItIsSynthetic() throws IOException {
        GamePieceTracker tracker = pieces("red", 30.0, 4.0, "blue", 40.0, -12.0);
        AprilTagLayout layout = AprilTagLayout.builder().addTag(3, 70.0, 10.0, 8.0, Math.PI, 6.0).build();
        ObstacleBox wall = new ObstacleBox("wall", 72.0, 74.0, -72.0, 72.0, 0.0, 12.0);
        SimFrame frame = SimFrameRenderer.builder(LENS, FRONT).style(STYLE).supersampling(2).build()
                .render(tracker.getPieces(), 0.0, 0.0, 0.0, CameraOccluders.field(Arrays.asList(wall)), layout);
        byte[] png = frame.toPng();
        byte[] pixels = decodePng(png, frame.getWidthPx(), frame.getHeightPx());
        assertArrayEquals(frame.getRgbBytes(), pixels);
        assertTrue(new String(png, StandardCharsets.ISO_8859_1).contains(SimFrame.PNG_COMMENT));

        // Kept for the docs: build/sim/vision/sim-frame-example.png.
        Path dir = Paths.get("build", "sim", "vision");
        Files.createDirectories(dir);
        try (OutputStream out = Files.newOutputStream(dir.resolve("sim-frame-example.png"))) {
            frame.writePng(out);
        }
    }

    /**
     * The test's own reader for the one PNG layout {@link SimFrame} writes (8-bit RGB, filter 0 rows, one
     * IDAT), checking the signature, the header and every chunk's CRC: the unit-test classpath has no
     * {@code javax.imageio}.
     */
    private static byte[] decodePng(byte[] png, int width, int height) throws IOException {
        byte[] signature = {(byte) 0x89, 0x50, 0x4E, 0x47, 0x0D, 0x0A, 0x1A, 0x0A};
        assertArrayEquals(signature, Arrays.copyOf(png, 8));
        ByteArrayOutputStream idat = new ByteArrayOutputStream();
        int at = 8;
        boolean sawEnd = false;
        while (at < png.length) {
            int length = readInt(png, at);
            String type = new String(png, at + 4, 4, StandardCharsets.ISO_8859_1);
            CRC32 crc = new CRC32();
            crc.update(png, at + 4, 4 + length);
            assertEquals((int) crc.getValue(), readInt(png, at + 8 + length), type + " CRC");
            if (type.equals("IHDR")) {
                assertEquals(width, readInt(png, at + 8));
                assertEquals(height, readInt(png, at + 12));
                assertEquals(8, png[at + 16]);
                assertEquals(2, png[at + 17]);
            } else if (type.equals("IDAT")) {
                idat.write(png, at + 8, length);
            } else if (type.equals("IEND")) {
                sawEnd = true;
            }
            at += 12 + length;
        }
        assertTrue(sawEnd);
        Inflater inflater = new Inflater();
        inflater.setInput(idat.toByteArray());
        byte[] raw = new byte[(width * 3 + 1) * height];
        try {
            assertEquals(raw.length, inflater.inflate(raw));
        } catch (DataFormatException e) {
            throw new AssertionError("IDAT does not inflate", e);
        } finally {
            inflater.end();
        }
        byte[] pixels = new byte[width * height * 3];
        for (int y = 0; y < height; y++) {
            assertEquals(0, raw[y * (width * 3 + 1)], "row " + y + " filter");
            System.arraycopy(raw, y * (width * 3 + 1) + 1, pixels, y * width * 3, width * 3);
        }
        return pixels;
    }

    private static int readInt(byte[] b, int at) {
        return ((b[at] & 0xFF) << 24) | ((b[at + 1] & 0xFF) << 16) | ((b[at + 2] & 0xFF) << 8) | (b[at + 3] & 0xFF);
    }

    @Test
    void rgbaAndAccessorsAgree() {
        SimFrame frame = renderer(LENS).render(pieces("red", 30.0, 4.0).getPieces(), 0.0, 0.0, 0.0,
                CameraOccluders.none(), AprilTagLayout.empty());
        byte[] rgb = frame.getRgbBytes();
        byte[] rgba = frame.getRgbaBytes();
        assertEquals(rgb.length / 3 * 4, rgba.length);
        assertEquals(rgb[3 * 1000 + 1], rgba[4 * 1000 + 1]);
        assertEquals((byte) 0xFF, rgba[4 * 1000 + 3]);
        assertThrows(IndexOutOfBoundsException.class, () -> frame.getRgb(320, 0));
        assertNull(frame.centroidOfPiece(99));
        assertTrue(frame.toString().contains("synthetic"));
    }
}
