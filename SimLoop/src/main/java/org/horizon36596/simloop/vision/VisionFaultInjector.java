package org.horizon36596.simloop.vision;

import org.horizon36596.simloop.field.ObstacleBox;
import org.horizon36596.simloop.field.SeededRandom;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

/**
 * Applies the content faults of a {@link VisionFaults} to one frame at a time: noise beyond the default,
 * missed sightings, blackouts, occluding patches, false detections and wrong ids. The timing faults
 * (latency, jitter, drops, repeats, reordering) are {@link FaultedFrameStream}'s.
 *
 * <p><b>One call is one frame.</b> Call exactly one {@code apply} method per captured frame, in capture order;
 * a blackout counts frames by those calls. The output is a new list or frame; the input is never changed.
 *
 * <p><b>What a fault never produces.</b> Every number in an output is finite. A piece sighting whose noisy
 * radius is zero or less, or whose noisy disc no longer overlaps the image, and a tag whose noisy centre
 * leaves the image, are dropped, because a detector cannot report them; a frame's pixels stay in 0..255. A test that wants a NaN in its pipeline must put it there itself.
 *
 * <p><b>Ground truth stays truthful.</b> A false piece sighting carries {@link #FALSE_PIECE_ID}, the id
 * {@link SimFrame#getPieceIdAt} uses for "no piece". A mislabelled piece keeps its true piece id; only its
 * kind changes. A mislabelled or false tag sighting names the layout tag it claims to be, but its transforms
 * are the geometry actually seen, so a pose solved from it is wrong exactly as it would be on a real robot.
 * In a frame, pixels a patch or a blackout covers are relabelled "no piece" and "no tag", and a false blob
 * is labelled "no piece".
 *
 * <p>Deterministic: the same {@link VisionFaults} and the same sequence of calls give the same outputs.
 * Not thread-safe.
 */
public final class VisionFaultInjector {

    /** The piece id a false piece sighting carries: no piece, as in {@link SimFrame#getPieceIdAt}. */
    public static final int FALSE_PIECE_ID = -1;

    /**
     * Range window a false tag is placed in, lens to tag centre, inches. SET BY HAND: 12 to 120 in, the span
     * an FTC field gives a robot-mounted camera.
     */
    private static final double FALSE_TAG_MIN_RANGE_IN = 12.0;
    private static final double FALSE_TAG_MAX_RANGE_IN = 120.0;

    /** Placements tried for a false tag before giving up on it for this frame. */
    private static final int FALSE_TAG_ATTEMPTS = 16;

    /** The flat colour an occluding patch is painted, {@code 0xRRGGBB}: a dark grey, like a robot's side. */
    private static final int PATCH_RGB = 0x303030;

    private final VisionFaults faults;
    private final SeededRandom noiseRandom;
    private final SeededRandom missRandom;
    private final SeededRandom blackoutRandom;
    private final SeededRandom patchRandom;
    private final SeededRandom falseRandom;
    private final SeededRandom wrongIdRandom;
    private int blackoutFramesLeft;
    private long framesSeen;
    private long blackedOutFrames;

    /**
     * An injector for these faults, at the start of its random streams.
     *
     * @param faults what to inject; {@link VisionFaults#none()} passes every frame through unchanged
     */
    public VisionFaultInjector(VisionFaults faults) {
        if (faults == null) {
            throw new IllegalArgumentException("faults must not be null; use VisionFaults.none()");
        }
        this.faults = faults;
        this.noiseRandom = stream(faults.getSeed(), 1);
        this.missRandom = stream(faults.getSeed(), 2);
        this.blackoutRandom = stream(faults.getSeed(), 3);
        this.patchRandom = stream(faults.getSeed(), 4);
        this.falseRandom = stream(faults.getSeed(), 5);
        this.wrongIdRandom = stream(faults.getSeed(), 6);
    }

    /**
     * One independent generator per fault, so turning one fault on cannot move another's draws. The salt is
     * scattered through a fixed odd multiplier so neighbouring seeds do not give neighbouring streams.
     */
    static SeededRandom stream(int seed, int salt) {
        return new SeededRandom(seed ^ (salt * 0x9E3779B9));
    }

    /** {@return the faults this injector applies} */
    public VisionFaults getFaults() {
        return faults;
    }

    /** {@return frames passed through any {@code apply} method so far} */
    public long getFramesSeen() {
        return framesSeen;
    }

    /** {@return frames so far that fell inside a blackout} */
    public long getBlackedOutFrames() {
        return blackedOutFrames;
    }

    // ------------------------------------------------------------------------------------- pieces

    /**
     * One frame of piece sightings, faulted. In this order: a blackout empties the frame; each sighting may
     * be missed; noise moves the centre and radius (a radius at or below zero, or a disc moved wholly off the
     * image, drops the sighting); a sighting may be mislabelled; one false sighting may be added at the end.
     *
     * @param sightings what the camera saw this frame, e.g. {@link SimCamera#observe}'s result
     * @param lens      the camera's lens, for the image size
     * @return the faulted sightings, unmodifiable; the input order, a false sighting last
     */
    public List<CameraSighting> applyToSightings(List<CameraSighting> sightings, CameraLens lens) {
        if (sightings == null || lens == null) {
            throw new IllegalArgumentException("sightings and lens must not be null");
        }
        if (startFrame()) {
            return Collections.emptyList();
        }
        List<CameraSighting> out = new ArrayList<CameraSighting>(sightings.size() + 1);
        for (CameraSighting s : sightings) {
            if (missed()) {
                continue;
            }
            double x = s.getCentreXPx() + gaussian(noiseRandom, faults.getCentreNoiseStdDevPx());
            double y = s.getCentreYPx() + gaussian(noiseRandom, faults.getCentreNoiseStdDevPx());
            double r = s.getRadiusPx() + gaussian(noiseRandom, faults.getRadiusNoiseStdDevPx());
            if (!(r > 0.0) || !discOverlapsImage(x, y, r, lens)) {
                continue;
            }
            String kind = s.getKind();
            String[] kinds = faults.getWrongKinds();
            if (faults.getWrongIdProbability() > 0.0 && kinds.length > 0
                    && wrongIdRandom.nextFraction() < faults.getWrongIdProbability()) {
                String other = pickOther(kinds, kind);
                if (other != null) {
                    kind = other;
                }
            }
            out.add(new CameraSighting(s.getPieceId(), kind, x, y, r, s.getRangeInches(),
                    clipped(x, y, r, lens), s.getVisibleFraction()));
        }
        if (faults.getFalseDetectionProbability() > 0.0
                && falseRandom.nextFraction() < faults.getFalseDetectionProbability()) {
            double x = falseRandom.nextBetween(0.0, lens.getWidthPx());
            double y = falseRandom.nextBetween(0.0, lens.getHeightPx());
            double r = falseRandom.nextBetween(faults.getFalseDetectionMinRadiusPx(),
                    faults.getFalseDetectionMaxRadiusPx());
            // A range is part of a sighting; give the false one the range a 1 in radius piece of that apparent
            // size would be at, so it is finite and plausible rather than a NaN that hides the real question.
            double range = lens.getFocalXPx() * 1.0 / r;
            out.add(new CameraSighting(FALSE_PIECE_ID, faults.getFalseDetectionKind(), x, y, r, range,
                    clipped(x, y, r, lens), 1.0));
        }
        return Collections.unmodifiableList(out);
    }

    // ---------------------------------------------------------------------------------------- tags

    /**
     * One frame of tag sightings, faulted. In this order: a blackout empties the frame; each sighting may be
     * missed; noise moves {@code tx}, {@code ty}, the centre and the corners together (a centre pushed out of
     * the image drops the sighting); a sighting may report another layout tag's id; one false tag may be
     * added: a random layout tag's id, seen at a random place in front of the camera.
     *
     * @param tags            what the camera saw this frame, e.g. {@link SimTagCamera#observe}'s result
     * @param layout          the layout the ids come from
     * @param camera          the camera the tags were seen with, for its lens and mount
     * @param robotXInches    robot centre, field X, inches, where the frame was captured
     * @param robotYInches    robot centre, field Y, inches
     * @param robotHeadingRad robot heading, radians CCW from field {@code +X}
     * @return the faulted sightings, unmodifiable; ascending id order of the input, a false tag last
     */
    public List<TagSighting> applyToTags(List<TagSighting> tags, AprilTagLayout layout, SimTagCamera camera,
            double robotXInches, double robotYInches, double robotHeadingRad) {
        if (tags == null || layout == null || camera == null) {
            throw new IllegalArgumentException("tags, layout and camera must not be null");
        }
        if (startFrame()) {
            return Collections.emptyList();
        }
        CameraLens lens = camera.getLens();
        List<AprilTagLayout.Tag> all = layout.getTags();
        List<TagSighting> out = new ArrayList<TagSighting>(tags.size() + 1);
        for (TagSighting s : tags) {
            if (missed()) {
                continue;
            }
            TagSighting t = s;
            double sigma = faults.getTagAngleNoiseStdDevDegrees();
            if (sigma > 0.0) {
                t = withAngles(s, s.getTxDegrees() + gaussian(noiseRandom, sigma),
                        s.getTyDegrees() + gaussian(noiseRandom, sigma), lens);
                if (t == null) {
                    continue;
                }
            }
            if (faults.getWrongIdProbability() > 0.0 && all.size() > 1
                    && wrongIdRandom.nextFraction() < faults.getWrongIdProbability()) {
                AprilTagLayout.Tag claimed = pickOtherTag(all, t.getId());
                t = new TagSighting(claimed, t.getCameraToTag(), t.getRobotToTag(), t.getCentreXPx(),
                        t.getCentreYPx(), t.getCornersPx(), t.getTxDegrees(), t.getTyDegrees(), t.getAreaPercent());
            }
            out.add(t);
        }
        if (!all.isEmpty() && faults.getFalseDetectionProbability() > 0.0
                && falseRandom.nextFraction() < faults.getFalseDetectionProbability()) {
            TagSighting phantom = falseTag(all, camera, robotXInches, robotYInches, robotHeadingRad);
            if (phantom != null) {
                out.add(phantom);
            }
        }
        return Collections.unmodifiableList(out);
    }

    /**
     * A tag sighting with new angles: the centre pixel is re-projected from them and the corners move with
     * it. Null when the new centre is not in the image, or the angles point behind the lens.
     */
    private static TagSighting withAngles(TagSighting s, double txDegrees, double tyDegrees, CameraLens lens) {
        if (!(Math.abs(txDegrees) < 90.0) || !(Math.abs(tyDegrees) < 90.0)) {
            return null;
        }
        double[] centre = lens.project(StrictMath.tan(StrictMath.toRadians(txDegrees)),
                -StrictMath.tan(StrictMath.toRadians(tyDegrees)), 1.0);
        if (centre == null || !insideImage(centre[0], centre[1], lens)) {
            return null;
        }
        double dx = centre[0] - s.getCentreXPx();
        double dy = centre[1] - s.getCentreYPx();
        double[][] corners = s.getCornersPx();
        double[][] moved = new double[corners.length][];
        for (int i = 0; i < corners.length; i++) {
            moved[i] = new double[] {corners[i][0] + dx, corners[i][1] + dy};
        }
        return new TagSighting(s.getTag(), s.getCameraToTag(), s.getRobotToTag(), centre[0], centre[1], moved,
                txDegrees, tyDegrees, s.getAreaPercent());
    }

    /**
     * A layout tag's id seen somewhere it is not: a copy of a random tag placed at a random range and bearing
     * in front of the camera, facing it, seen through the camera's own rules, then relabelled as the layout
     * tag so that its transforms are the phantom's geometry. Null when no placement tried was visible.
     */
    private TagSighting falseTag(List<AprilTagLayout.Tag> all, SimTagCamera camera, double robotXInches,
            double robotYInches, double robotHeadingRad) {
        AprilTagLayout.Tag claimed = all.get(Math.min(all.size() - 1, (int) (falseRandom.nextFraction() * all.size())));
        CameraLens lens = camera.getLens();
        RigidTransform3 fieldToCamera = RigidTransform3.planar(robotXInches, robotYInches, robotHeadingRad)
                .compose(camera.getRobotToCamera());
        for (int attempt = 0; attempt < FALSE_TAG_ATTEMPTS; attempt++) {
            double range = falseRandom.nextBetween(FALSE_TAG_MIN_RANGE_IN, FALSE_TAG_MAX_RANGE_IN);
            double px = falseRandom.nextBetween(0.2, 0.8) * lens.getWidthPx();
            double py = falseRandom.nextBetween(0.2, 0.8) * lens.getHeightPx();
            double[] ray = lens.undistortPixel(px, py);
            double norm = StrictMath.sqrt(ray[0] * ray[0] + ray[1] * ray[1] + 1.0);
            double[] centre = fieldToCamera.apply(range * ray[0] / norm, range * ray[1] / norm, range / norm);
            double[] lensField = fieldToCamera.apply(0.0, 0.0, 0.0);
            double facing = StrictMath.atan2(lensField[1] - centre[1], lensField[0] - centre[0]);
            AprilTagLayout phantomLayout = AprilTagLayout.builder()
                    .addTag(claimed.getId(), centre[0], centre[1], centre[2], facing, claimed.getSizeInches())
                    .build();
            List<TagSighting> seen = new SimTagCamera(lens, camera.getMount(), 0.0, Double.POSITIVE_INFINITY,
                    90.0, Collections.<ObstacleBox>emptyList())
                    .observe(phantomLayout, robotXInches, robotYInches, robotHeadingRad);
            if (!seen.isEmpty()) {
                TagSighting p = seen.get(0);
                return new TagSighting(claimed, p.getCameraToTag(), p.getRobotToTag(), p.getCentreXPx(),
                        p.getCentreYPx(), p.getCornersPx(), p.getTxDegrees(), p.getTyDegrees(), p.getAreaPercent());
            }
        }
        return null;
    }

    // -------------------------------------------------------------------------------------- frames

    /**
     * One synthetic frame, faulted. In this order: a blackout paints it black; one occluding patch may be
     * painted; one false blob may be painted; extra noise goes on every channel. Pixels a blackout or a patch
     * covers lose their piece and tag labels.
     *
     * @param frame the rendered frame
     * @return a new frame with the same size and frame number; the input is unchanged
     */
    public SimFrame applyToFrame(SimFrame frame) {
        if (frame == null) {
            throw new IllegalArgumentException("frame must not be null");
        }
        int w = frame.getWidthPx();
        int h = frame.getHeightPx();
        byte[] rgb = frame.getRgbBytes();
        int[] pieces = new int[w * h];
        int[] tagIds = new int[w * h];
        for (int y = 0; y < h; y++) {
            for (int x = 0; x < w; x++) {
                pieces[y * w + x] = frame.getPieceIdAt(x, y);
                tagIds[y * w + x] = frame.getTagIdAt(x, y);
            }
        }
        if (startFrame()) {
            Arrays.fill(rgb, (byte) 0);
            Arrays.fill(pieces, -1);
            Arrays.fill(tagIds, -1);
            return new SimFrame(w, h, frame.getFrameNumber(), rgb, pieces, tagIds);
        }
        if (faults.getOcclusionPatchProbability() > 0.0
                && patchRandom.nextFraction() < faults.getOcclusionPatchProbability()) {
            int pw = Math.max(1, (int) Math.round(faults.getOcclusionPatchFraction() * w));
            int ph = Math.max(1, (int) Math.round(faults.getOcclusionPatchFraction() * h));
            int x0 = (int) (patchRandom.nextFraction() * (w - pw + 1));
            int y0 = (int) (patchRandom.nextFraction() * (h - ph + 1));
            for (int y = y0; y < Math.min(h, y0 + ph); y++) {
                for (int x = x0; x < Math.min(w, x0 + pw); x++) {
                    paint(rgb, y * w + x, PATCH_RGB);
                    pieces[y * w + x] = -1;
                    tagIds[y * w + x] = -1;
                }
            }
        }
        if (faults.getFalseDetectionProbability() > 0.0
                && falseRandom.nextFraction() < faults.getFalseDetectionProbability()) {
            double cx = falseRandom.nextBetween(0.0, w);
            double cy = falseRandom.nextBetween(0.0, h);
            double r = falseRandom.nextBetween(faults.getFalseDetectionMinRadiusPx(),
                    faults.getFalseDetectionMaxRadiusPx());
            for (int y = Math.max(0, (int) (cy - r)); y <= Math.min(h - 1, (int) (cy + r)); y++) {
                for (int x = Math.max(0, (int) (cx - r)); x <= Math.min(w - 1, (int) (cx + r)); x++) {
                    double dx = x + 0.5 - cx;
                    double dy = y + 0.5 - cy;
                    if (dx * dx + dy * dy <= r * r) {
                        paint(rgb, y * w + x, faults.getFalseDetectionColourRgb());
                        pieces[y * w + x] = -1;
                    }
                }
            }
        }
        double sigma = faults.getPixelNoiseStdDevCounts();
        if (sigma > 0.0) {
            for (int i = 0; i < rgb.length; i++) {
                long v = Math.round((rgb[i] & 0xFF) + gaussian(noiseRandom, sigma));
                rgb[i] = (byte) Math.max(0L, Math.min(255L, v));
            }
        }
        return new SimFrame(w, h, frame.getFrameNumber(), rgb, pieces, tagIds);
    }

    // ------------------------------------------------------------------------------------- helpers

    /** Counts the frame and reports whether it is blacked out, starting a blackout when one is drawn. */
    private boolean startFrame() {
        framesSeen++;
        if (blackoutFramesLeft == 0 && faults.getBlackoutProbability() > 0.0
                && blackoutRandom.nextFraction() < faults.getBlackoutProbability()) {
            blackoutFramesLeft = faults.getBlackoutFrames();
        }
        if (blackoutFramesLeft > 0) {
            blackoutFramesLeft--;
            blackedOutFrames++;
            return true;
        }
        return false;
    }

    private boolean missed() {
        return faults.getMissProbability() > 0.0 && missRandom.nextFraction() < faults.getMissProbability();
    }

    private String pickOther(String[] kinds, String own) {
        List<String> others = new ArrayList<String>(kinds.length);
        for (String k : kinds) {
            if (!k.equals(own)) {
                others.add(k);
            }
        }
        if (others.isEmpty()) {
            return null;
        }
        return others.get(Math.min(others.size() - 1, (int) (wrongIdRandom.nextFraction() * others.size())));
    }

    private AprilTagLayout.Tag pickOtherTag(List<AprilTagLayout.Tag> all, int ownId) {
        int pick = Math.min(all.size() - 2, (int) (wrongIdRandom.nextFraction() * (all.size() - 1)));
        for (AprilTagLayout.Tag tag : all) {
            if (tag.getId() == ownId) {
                continue;
            }
            if (pick-- == 0) {
                return tag;
            }
        }
        throw new IllegalStateException("a layout of " + all.size() + " tags has another tag; this is a bug");
    }

    private static boolean insideImage(double x, double y, CameraLens lens) {
        return x >= 0.0 && x <= lens.getWidthPx() && y >= 0.0 && y <= lens.getHeightPx();
    }

    /** The rule {@link SimCamera} sights a piece by: the image point nearest the disc's centre is in the disc. */
    private static boolean discOverlapsImage(double x, double y, double r, CameraLens lens) {
        double nearestX = Math.max(0.0, Math.min(lens.getWidthPx(), x));
        double nearestY = Math.max(0.0, Math.min(lens.getHeightPx(), y));
        double offX = x - nearestX;
        double offY = y - nearestY;
        return offX * offX + offY * offY < r * r;
    }

    private static boolean clipped(double x, double y, double r, CameraLens lens) {
        return x - r < 0.0 || x + r > lens.getWidthPx() || y - r < 0.0 || y + r > lens.getHeightPx();
    }

    private static void paint(byte[] rgb, int pixel, int colour) {
        rgb[3 * pixel] = (byte) (colour >>> 16);
        rgb[3 * pixel + 1] = (byte) (colour >>> 8);
        rgb[3 * pixel + 2] = (byte) colour;
    }

    /** One zero-mean Gaussian draw, or exactly 0 with no draw when the spread is 0. Box-Muller, as elsewhere. */
    static double gaussian(SeededRandom random, double stdDev) {
        if (stdDev == 0.0) {
            return 0.0;
        }
        double u1 = 1.0 - random.nextFraction();
        double u2 = random.nextFraction();
        return stdDev * StrictMath.sqrt(-2.0 * StrictMath.log(u1)) * StrictMath.cos(2.0 * Math.PI * u2);
    }
}
