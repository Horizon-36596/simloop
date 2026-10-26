package org.horizon36596.simloop.vision;

import org.horizon36596.simloop.field.GamePiece;
import org.horizon36596.simloop.field.ObstacleBox;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.IntStream;

/**
 * Draws what a {@link CameraLens} on a {@link CameraMount} would see, as a {@link SimFrame} of RGB pixels,
 * so a team's real pixel pipeline (an OpenCV {@code VisionProcessor}, a colour-blob detector, a contour
 * splitter) can run unchanged on frames from the sim. It is a ray caster: one ray per pixel (or a grid of
 * {@code n x n} per pixel when supersampling), through the same lens model, distortion included, that
 * {@link SimCamera} and {@link SimTagCamera} use, so a piece is drawn exactly where those cameras say it is.
 *
 * <p><b>What it draws.</b> The floor, as tiles with seams; field boxes and robot boxes from
 * {@link CameraOccluders}, flat-coloured; game pieces as spheres in their kind's colour; AprilTags from an
 * {@link AprilTagLayout} as black-and-white squares with a white border, lying in their own plane (tilted
 * tags included). Colours come from a {@link FrameStyle}; shading is Lambert with one light and ambient.
 * Optional extras: supersampling for smooth edges, a gain, and seeded Gaussian sensor noise.
 *
 * <p><b>What it does not draw</b>, stated so nothing depends on it: shadows, specular highlights,
 * reflections, textures, motion blur, rolling shutter, exposure time, white balance, lens blur, vignetting,
 * compression artefacts, anything not given as a box, piece or tag. Tag interiors are a deterministic
 * pattern made from the id, <b>not</b> a real tag family's code, so a real AprilTag decoder will not decode
 * them; use {@link SimTagCamera} or {@code FakeLimelight3A} to test tag pose code.
 *
 * <p><b>Synthetic frames are never real frames.</b> Use them to check that a pipeline's code paths run,
 * that its geometry puts a blob back where the piece is, that edge-clipped and touching pieces are handled,
 * and that occluded pieces are not reported. Never tune a colour threshold, an exposure or a minimum blob
 * area on them: a threshold fitted to these flat colours says nothing about real lighting. {@link SimFrame}
 * carries a per-pixel ground-truth label for checking, and its PNG says it is synthetic.
 *
 * <p><b>Frames.</b> Pieces, field boxes and tags in the field frame; robot boxes in the robot frame; the
 * robot pose as {@link SimCamera#observe(List, double, double, double)} takes it. All inches and radians.
 *
 * <p><b>Determinism.</b> No clock and no shared random state: the sensor noise of frame {@code k} is a pure
 * function of the seed, {@code k} and the pixel, so the same inputs in the same order give byte-identical
 * frames. StrictMath for all trigonometry. Rows of tiles are drawn in parallel on the common fork-join
 * pool, but every pixel is a pure function of the scene and writes only its own slots, so the bytes do not
 * depend on how many threads ran or in what order.
 *
 * <p><b>Cost.</b> Pixels are grouped in 16 x 16 tiles, and each tile only tests the objects whose image
 * bounds overlap it, so the cost is roughly one floor test per ray plus the objects actually near that
 * ray. A 640 x 480 frame of a few dozen objects takes tens of milliseconds of CPU on a laptop at
 * {@code supersampling(1)}, spread over its cores; keep rendered frames to the tests that need pixels.
 */
public final class SimFrameRenderer {

    private static final int TILE = 16;

    /**
     * Where shape bounds are cut, inches in front of the lens. Anything closer than this cannot draw; a box
     * holding the lens is left out anyway.
     */
    private static final double NEAR_PLANE_INCHES = 1e-3;
    /** A tag is pulled this far toward the camera so one mounted on a box face is drawn, not the face. */
    private static final double TAG_DEPTH_BIAS_INCHES = 0.02;

    private final CameraLens lens;
    private final CameraMount mount;
    private final FrameStyle style;
    private final int supersampling;
    private final int noiseSeed;
    private final double noiseStdDevCounts;
    private final double gain;

    private final int width;
    private final int height;
    /** Undistorted normalised ray through each pixel's centre. */
    private final double[] centreRayX;
    private final double[] centreRayY;
    /** Undistorted normalised rays for the supersampling grid, {@code n * n} per pixel; null when n = 1. */
    private final double[] subRayX;
    private final double[] subRayY;
    private final int tilesX;
    private final int tilesY;
    /** Per tile: min x, max x, min y, max y of its rays, normalised. */
    private final double[] tileBounds;

    private final double[] lensRobot;
    private final double[] axisXRobot;
    private final double[] axisYRobot;
    private final double[] axisZRobot;

    private long framesRendered;

    private SimFrameRenderer(Builder b) {
        this.lens = b.lens;
        this.mount = b.mount;
        this.style = b.style;
        this.supersampling = b.supersampling;
        this.noiseSeed = b.noiseSeed;
        this.noiseStdDevCounts = b.noiseStdDevCounts;
        this.gain = b.gain;
        this.width = lens.getWidthPx();
        this.height = lens.getHeightPx();

        int pixels = width * height;
        centreRayX = new double[pixels];
        centreRayY = new double[pixels];
        int n = supersampling;
        subRayX = n > 1 ? new double[pixels * n * n] : null;
        subRayY = n > 1 ? new double[pixels * n * n] : null;
        tilesX = (width + TILE - 1) / TILE;
        tilesY = (height + TILE - 1) / TILE;
        tileBounds = new double[tilesX * tilesY * 4];
        for (int t = 0; t < tilesX * tilesY; t++) {
            tileBounds[4 * t] = Double.POSITIVE_INFINITY;
            tileBounds[4 * t + 1] = Double.NEGATIVE_INFINITY;
            tileBounds[4 * t + 2] = Double.POSITIVE_INFINITY;
            tileBounds[4 * t + 3] = Double.NEGATIVE_INFINITY;
        }
        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++) {
                int p = y * width + x;
                int tile = (y / TILE) * tilesX + x / TILE;
                double[] c = lens.undistortPixel(x + 0.5, y + 0.5);
                centreRayX[p] = c[0];
                centreRayY[p] = c[1];
                grow(tile, c[0], c[1]);
                if (n > 1) {
                    for (int j = 0; j < n; j++) {
                        for (int i = 0; i < n; i++) {
                            double[] r = lens.undistortPixel(x + (i + 0.5) / n, y + (j + 0.5) / n);
                            int s = p * n * n + j * n + i;
                            subRayX[s] = r[0];
                            subRayY[s] = r[1];
                            grow(tile, r[0], r[1]);
                        }
                    }
                }
            }
        }

        lensRobot = mount.opticalToRobot(0.0, 0.0, 0.0);
        axisXRobot = minus(mount.opticalToRobot(1.0, 0.0, 0.0), lensRobot);
        axisYRobot = minus(mount.opticalToRobot(0.0, 1.0, 0.0), lensRobot);
        axisZRobot = minus(mount.opticalToRobot(0.0, 0.0, 1.0), lensRobot);
    }

    private void grow(int tile, double x, double y) {
        tileBounds[4 * tile] = Math.min(tileBounds[4 * tile], x);
        tileBounds[4 * tile + 1] = Math.max(tileBounds[4 * tile + 1], x);
        tileBounds[4 * tile + 2] = Math.min(tileBounds[4 * tile + 2], y);
        tileBounds[4 * tile + 3] = Math.max(tileBounds[4 * tile + 3], y);
    }

    /**
     * A builder for a renderer with this lens and mount, the default {@link FrameStyle}, no supersampling,
     * gain 1 and no noise.
     *
     * @param lens  the intrinsics, distortion included; its size is the frame size
     * @param mount the extrinsics
     * @return the builder
     */
    public static Builder builder(CameraLens lens, CameraMount mount) {
        return new Builder(lens, mount);
    }

    /**
     * Draws one frame. Advances the frame counter the sensor noise is keyed on.
     *
     * @param pieces          the game pieces; only those {@linkplain GamePiece#isVisibleOnField() on the
     *                        field} are drawn
     * @param robotXInches    robot centre, field X, inches
     * @param robotYInches    robot centre, field Y, inches
     * @param robotHeadingRad robot heading, radians CCW from field {@code +X}
     * @param boxes           field and robot boxes to draw; a box containing the lens is left out
     * @param tags            the tags to draw; {@link AprilTagLayout#empty()} for none
     * @return the frame
     */
    public SimFrame render(List<GamePiece> pieces, double robotXInches, double robotYInches,
            double robotHeadingRad, CameraOccluders boxes, AprilTagLayout tags) {
        if (pieces == null || boxes == null || tags == null) {
            throw new IllegalArgumentException("pieces, boxes and tags must not be null; pass empty ones");
        }
        requireFinite(robotXInches, "robotXInches");
        requireFinite(robotYInches, "robotYInches");
        requireFinite(robotHeadingRad, "robotHeadingRad");
        double cos = StrictMath.cos(robotHeadingRad);
        double sin = StrictMath.sin(robotHeadingRad);
        Scene scene = new Scene(robotXInches, robotYInches, cos, sin);
        buildShapes(scene, pieces, boxes, tags);

        int pixels = width * height;
        byte[] rgb = new byte[pixels * 3];
        int[] pieceIds = new int[pixels];
        int[] tagIds = new int[pixels];
        long frame = framesRendered++;
        int frameKey = noiseSeed ^ mix((int) frame ^ mix((int) (frame >>> 32)));
        // One row of tiles per task. Each task traces with its own copy of the scene's per-ray state.
        IntStream.range(0, tilesY).parallel().forEach(ty -> renderTileRow(new Scene(scene), ty, frameKey,
                rgb, pieceIds, tagIds));
        return new SimFrame(width, height, frame, rgb, pieceIds, tagIds);
    }

    /** Draws one row of tiles into the frame's arrays; touches only that row's pixels. */
    private void renderTileRow(Scene scene, int ty, int frameKey, byte[] rgb, int[] pieceIds, int[] tagIds) {
        int n = supersampling;
        Hit hit = new Hit();
        double[] colour = new double[3];
        for (int tx = 0; tx < tilesX; tx++) {
            int tile = ty * tilesX + tx;
            Shape[] candidates = scene.candidatesFor(tileBounds, tile);
            int x1 = Math.min(width, (tx + 1) * TILE);
            int y1 = Math.min(height, (ty + 1) * TILE);
            for (int y = ty * TILE; y < y1; y++) {
                for (int x = tx * TILE; x < x1; x++) {
                    int p = y * width + x;
                    trace(scene, candidates, centreRayX[p], centreRayY[p], hit);
                    pieceIds[p] = hit.pieceId;
                    tagIds[p] = hit.tagId;
                    double r;
                    double g;
                    double bl;
                    if (n == 1) {
                        shade(scene, hit, colour);
                        r = colour[0];
                        g = colour[1];
                        bl = colour[2];
                    } else {
                        r = 0.0;
                        g = 0.0;
                        bl = 0.0;
                        for (int s = p * n * n, end = s + n * n; s < end; s++) {
                            trace(scene, candidates, subRayX[s], subRayY[s], hit);
                            shade(scene, hit, colour);
                            r += colour[0];
                            g += colour[1];
                            bl += colour[2];
                        }
                        r /= n * n;
                        g /= n * n;
                        bl /= n * n;
                    }
                    rgb[3 * p] = toByte(r * gain + noise(frameKey, p, 0));
                    rgb[3 * p + 1] = toByte(g * gain + noise(frameKey, p, 1));
                    rgb[3 * p + 2] = toByte(bl * gain + noise(frameKey, p, 2));
                }
            }
        }
    }

    /** {@return how many frames this renderer has drawn} */
    public long getFramesRendered() {
        return framesRendered;
    }

    /** {@return the lens frames are drawn through} */
    public CameraLens getLens() {
        return lens;
    }

    /** {@return the mount frames are drawn from} */
    public CameraMount getMount() {
        return mount;
    }

    /** {@return the colours and light} */
    public FrameStyle getStyle() {
        return style;
    }

    // --------------------------------------------------------------------------------------------- scene

    /** The camera's pose for this frame and every shape to test, in both frames a shape may live in. */
    private final class Scene {
        final double robotX;
        final double robotY;
        final double cos;
        final double sin;
        final double[] lensField;
        final double[] axisXField;
        final double[] axisYField;
        final double[] axisZField;
        final List<Shape> shapes = new ArrayList<Shape>();
        /** The ray for the current sample, field frame and robot frame. */
        double fdx;
        double fdy;
        double fdz;
        double rdx;
        double rdy;
        double rdz;

        /** A copy for another thread: the same pose and shapes, its own per-ray state. */
        Scene(Scene shared) {
            this.robotX = shared.robotX;
            this.robotY = shared.robotY;
            this.cos = shared.cos;
            this.sin = shared.sin;
            this.lensField = shared.lensField;
            this.axisXField = shared.axisXField;
            this.axisYField = shared.axisYField;
            this.axisZField = shared.axisZField;
            this.shapes.addAll(shared.shapes);
        }

        Scene(double robotX, double robotY, double cos, double sin) {
            this.robotX = robotX;
            this.robotY = robotY;
            this.cos = cos;
            this.sin = sin;
            this.lensField = pointToField(lensRobot);
            this.axisXField = vectorToField(axisXRobot);
            this.axisYField = vectorToField(axisYRobot);
            this.axisZField = vectorToField(axisZRobot);
        }

        double[] pointToField(double[] robot) {
            return new double[] {robotX + robot[0] * cos - robot[1] * sin, robotY + robot[0] * sin + robot[1] * cos,
                    robot[2]};
        }

        double[] vectorToField(double[] robot) {
            return new double[] {robot[0] * cos - robot[1] * sin, robot[0] * sin + robot[1] * cos, robot[2]};
        }

        /**
         * Sets a shape's image bounds from field points whose convex hull holds it. The hull is cut at
         * {@link #NEAR_PLANE_INCHES} in front of the lens first: the points in front, plus where each segment
         * between a point in front and one behind crosses that plane (the segments between every pair
         * include the hull's edges, so the bound stays conservative). A shape wholly behind the plane gets
         * empty bounds and is never tested; a ray cannot hit it.
         */
        void bound(Shape shape, double[][] fieldPoints) {
            int n = fieldPoints.length;
            double[][] optical = new double[n][];
            for (int i = 0; i < n; i++) {
                optical[i] = toOptical(fieldPoints[i]);
            }
            double minX = Double.POSITIVE_INFINITY;
            double maxX = Double.NEGATIVE_INFINITY;
            double minY = Double.POSITIVE_INFINITY;
            double maxY = Double.NEGATIVE_INFINITY;
            for (int i = 0; i < n; i++) {
                double[] a = optical[i];
                if (a[2] < NEAR_PLANE_INCHES) {
                    continue;
                }
                minX = Math.min(minX, a[0] / a[2]);
                maxX = Math.max(maxX, a[0] / a[2]);
                minY = Math.min(minY, a[1] / a[2]);
                maxY = Math.max(maxY, a[1] / a[2]);
                for (int j = 0; j < n; j++) {
                    double[] b = optical[j];
                    if (b[2] >= NEAR_PLANE_INCHES) {
                        continue;
                    }
                    double s = (a[2] - NEAR_PLANE_INCHES) / (a[2] - b[2]);
                    double x = (a[0] + s * (b[0] - a[0])) / NEAR_PLANE_INCHES;
                    double y = (a[1] + s * (b[1] - a[1])) / NEAR_PLANE_INCHES;
                    minX = Math.min(minX, x);
                    maxX = Math.max(maxX, x);
                    minY = Math.min(minY, y);
                    maxY = Math.max(maxY, y);
                }
            }
            // Wholly behind the plane: the infinities left here overlap no tile.
            shape.minX = minX;
            shape.maxX = maxX;
            shape.minY = minY;
            shape.maxY = maxY;
        }

        /** A field point in the optical frame, inches: {@code +x} image right, {@code +y} down, {@code +z} ahead. */
        double[] toOptical(double[] p) {
            double dx = p[0] - lensField[0];
            double dy = p[1] - lensField[1];
            double dz = p[2] - lensField[2];
            return new double[] {
                    dx * axisXField[0] + dy * axisXField[1] + dz * axisXField[2],
                    dx * axisYField[0] + dy * axisYField[1] + dz * axisYField[2],
                    dx * axisZField[0] + dy * axisZField[1] + dz * axisZField[2]};
        }

        Shape[] candidatesFor(double[] bounds, int tile) {
            double minX = bounds[4 * tile];
            double maxX = bounds[4 * tile + 1];
            double minY = bounds[4 * tile + 2];
            double maxY = bounds[4 * tile + 3];
            List<Shape> list = new ArrayList<Shape>();
            for (Shape shape : shapes) {
                if (shape.maxX >= minX && shape.minX <= maxX && shape.maxY >= minY && shape.minY <= maxY) {
                    list.add(shape);
                }
            }
            return list.toArray(new Shape[0]);
        }
    }

    private void buildShapes(Scene scene, List<GamePiece> pieces, CameraOccluders boxes, AprilTagLayout tags) {
        for (ObstacleBox box : boxes.getFieldBoxes()) {
            if (CameraOccluders.contains(box, scene.lensField[0], scene.lensField[1], scene.lensField[2])) {
                continue;
            }
            Shape shape = new BoxShape(box, false, style.colourOfBox(box.getName(), false));
            scene.bound(shape, boxCorners(box, scene, false));
            scene.shapes.add(shape);
        }
        for (ObstacleBox box : boxes.getRobotBoxes()) {
            if (CameraOccluders.contains(box, lensRobot[0], lensRobot[1], lensRobot[2])) {
                continue;
            }
            Shape shape = new BoxShape(box, true, style.colourOfBox(box.getName(), true));
            scene.bound(shape, boxCorners(box, scene, true));
            scene.shapes.add(shape);
        }
        for (GamePiece piece : pieces) {
            if (!piece.isVisibleOnField()) {
                continue;
            }
            double r = piece.getRadiusInches();
            SphereShape shape = new SphereShape(piece.getId(), piece.getFieldXInches(), piece.getFieldYInches(),
                    piece.getFieldZInches(), r, style.colourOfPiece(piece.getKind()));
            double[][] corners = new double[8][];
            for (int i = 0; i < 8; i++) {
                corners[i] = new double[] {shape.cx + ((i & 1) == 0 ? -r : r), shape.cy + ((i & 2) == 0 ? -r : r),
                        shape.cz + ((i & 4) == 0 ? -r : r)};
            }
            scene.bound(shape, corners);
            scene.shapes.add(shape);
        }
        for (AprilTagLayout.Tag tag : tags.getTags()) {
            TagShape shape = new TagShape(tag);
            double h = shape.halfOuter;
            double[][] corners = new double[4][];
            int k = 0;
            for (double u : new double[] {-h, h}) {
                for (double v : new double[] {-h, h}) {
                    corners[k++] = tag.getFieldToTag().apply(u, v, 0.0);
                }
            }
            scene.bound(shape, corners);
            scene.shapes.add(shape);
        }
    }

    private static double[][] boxCorners(ObstacleBox box, Scene scene, boolean robotFrame) {
        double[][] corners = new double[8][];
        for (int i = 0; i < 8; i++) {
            double[] c = {
                    (i & 1) == 0 ? box.getMinXInches() : box.getMaxXInches(),
                    (i & 2) == 0 ? box.getMinYInches() : box.getMaxYInches(),
                    (i & 4) == 0 ? box.getMinZInches() : box.getMaxZInches()};
            corners[i] = robotFrame ? scene.pointToField(c) : c;
        }
        return corners;
    }

    // -------------------------------------------------------------------------------------- ray casting

    /** What the current ray hit: the shape (null for floor or background), where, and its labels. */
    private static final class Hit {
        Shape shape;
        boolean floor;
        double t;
        int pieceId;
        int tagId;
    }

    private void trace(Scene scene, Shape[] candidates, double xn, double yn, Hit hit) {
        double[] ax = scene.axisXField;
        double[] ay = scene.axisYField;
        double[] az = scene.axisZField;
        scene.fdx = xn * ax[0] + yn * ay[0] + az[0];
        scene.fdy = xn * ax[1] + yn * ay[1] + az[1];
        scene.fdz = xn * ax[2] + yn * ay[2] + az[2];
        scene.rdx = xn * axisXRobot[0] + yn * axisYRobot[0] + axisZRobot[0];
        scene.rdy = xn * axisXRobot[1] + yn * axisYRobot[1] + axisZRobot[1];
        scene.rdz = xn * axisXRobot[2] + yn * axisYRobot[2] + axisZRobot[2];

        double best = Double.POSITIVE_INFINITY;
        hit.shape = null;
        hit.floor = false;
        if (scene.fdz < -1e-12) {
            best = -scene.lensField[2] / scene.fdz;
            hit.floor = true;
        }
        for (Shape shape : candidates) {
            double t = shape.intersect(scene, best);
            if (t < best) {
                best = t;
                hit.shape = shape;
                hit.floor = false;
            }
        }
        hit.t = best;
        hit.pieceId = hit.shape instanceof SphereShape ? ((SphereShape) hit.shape).pieceId : -1;
        hit.tagId = hit.shape instanceof TagShape ? ((TagShape) hit.shape).tag.getId() : -1;
    }

    /** Colour, 0 to 255 per channel before gain, of what the current ray hit. */
    private void shade(Scene scene, Hit hit, double[] out) {
        int rgb;
        double[] normal;
        if (hit.shape != null) {
            double[] p = {scene.lensField[0] + hit.t * scene.fdx, scene.lensField[1] + hit.t * scene.fdy,
                    scene.lensField[2] + hit.t * scene.fdz};
            rgb = hit.shape.colourAt(scene, p, hit.t);
            normal = hit.shape.normalAt(scene, p, hit.t);
        } else if (hit.floor) {
            double x = scene.lensField[0] + hit.t * scene.fdx;
            double y = scene.lensField[1] + hit.t * scene.fdy;
            rgb = floorColour(x, y);
            normal = new double[] {0.0, 0.0, 1.0};
        } else {
            int bg = style.getBackgroundRgb();
            out[0] = (bg >> 16) & 0xFF;
            out[1] = (bg >> 8) & 0xFF;
            out[2] = bg & 0xFF;
            return;
        }
        double[] light = style.getLightDirection();
        double lambert = Math.max(0.0, normal[0] * light[0] + normal[1] * light[1] + normal[2] * light[2]);
        double k = style.getAmbient() + style.getDiffuse() * lambert;
        out[0] = ((rgb >> 16) & 0xFF) * k;
        out[1] = ((rgb >> 8) & 0xFF) * k;
        out[2] = (rgb & 0xFF) * k;
    }

    private int floorColour(double x, double y) {
        double tile = style.getTileSizeInches();
        if (tile > 0.0 && style.getSeamWidthInches() > 0.0) {
            double half = style.getSeamWidthInches() / 2.0;
            double fx = x - tile * StrictMath.floor(x / tile);
            double fy = y - tile * StrictMath.floor(y / tile);
            if (fx < half || fx > tile - half || fy < half || fy > tile - half) {
                return style.getSeamRgb();
            }
        }
        return style.getFloorRgb();
    }

    /** One thing a ray can hit, with its image bounds (normalised coordinates) for tile culling. */
    private abstract static class Shape {
        double minX = Double.NEGATIVE_INFINITY;
        double maxX = Double.POSITIVE_INFINITY;
        double minY = Double.NEGATIVE_INFINITY;
        double maxY = Double.POSITIVE_INFINITY;

        /** The ray parameter of the first hit below {@code limit}, or {@code +infinity}. */
        abstract double intersect(Scene scene, double limit);

        abstract int colourAt(Scene scene, double[] fieldPoint, double t);

        abstract double[] normalAt(Scene scene, double[] fieldPoint, double t);
    }

    private static final class SphereShape extends Shape {
        final int pieceId;
        final double cx;
        final double cy;
        final double cz;
        final double r;
        final int rgb;

        SphereShape(int pieceId, double cx, double cy, double cz, double r, int rgb) {
            this.pieceId = pieceId;
            this.cx = cx;
            this.cy = cy;
            this.cz = cz;
            this.r = r;
            this.rgb = rgb;
        }

        @Override
        double intersect(Scene scene, double limit) {
            double ox = scene.lensField[0] - cx;
            double oy = scene.lensField[1] - cy;
            double oz = scene.lensField[2] - cz;
            double a = scene.fdx * scene.fdx + scene.fdy * scene.fdy + scene.fdz * scene.fdz;
            double b = 2.0 * (ox * scene.fdx + oy * scene.fdy + oz * scene.fdz);
            double c = ox * ox + oy * oy + oz * oz - r * r;
            if (c <= 0.0) {
                return Double.POSITIVE_INFINITY;            // the lens is inside the piece: draw nothing
            }
            double disc = b * b - 4.0 * a * c;
            if (disc < 0.0) {
                return Double.POSITIVE_INFINITY;
            }
            double t = (-b - StrictMath.sqrt(disc)) / (2.0 * a);
            return t > 0.0 && t < limit ? t : Double.POSITIVE_INFINITY;
        }

        @Override
        int colourAt(Scene scene, double[] p, double t) {
            return rgb;
        }

        @Override
        double[] normalAt(Scene scene, double[] p, double t) {
            return new double[] {(p[0] - cx) / r, (p[1] - cy) / r, (p[2] - cz) / r};
        }
    }

    private final class BoxShape extends Shape {
        final ObstacleBox box;
        final boolean robotFrame;
        final int rgb;

        BoxShape(ObstacleBox box, boolean robotFrame, int rgb) {
            this.box = box;
            this.robotFrame = robotFrame;
            this.rgb = rgb;
        }

        @Override
        double intersect(Scene scene, double limit) {
            // The slab test, written out on primitives: this is the renderer's innermost loop.
            double[] o = robotFrame ? lensRobot : scene.lensField;
            double enter = 1e-9;
            double exit = limit;
            double d = robotFrame ? scene.rdx : scene.fdx;
            if (Math.abs(d) < 1e-12) {
                if (o[0] <= box.getMinXInches() || o[0] >= box.getMaxXInches()) {
                    return Double.POSITIVE_INFINITY;
                }
            } else {
                double t0 = (box.getMinXInches() - o[0]) / d;
                double t1 = (box.getMaxXInches() - o[0]) / d;
                enter = Math.max(enter, Math.min(t0, t1));
                exit = Math.min(exit, Math.max(t0, t1));
            }
            d = robotFrame ? scene.rdy : scene.fdy;
            if (Math.abs(d) < 1e-12) {
                if (o[1] <= box.getMinYInches() || o[1] >= box.getMaxYInches()) {
                    return Double.POSITIVE_INFINITY;
                }
            } else {
                double t0 = (box.getMinYInches() - o[1]) / d;
                double t1 = (box.getMaxYInches() - o[1]) / d;
                enter = Math.max(enter, Math.min(t0, t1));
                exit = Math.min(exit, Math.max(t0, t1));
            }
            d = robotFrame ? scene.rdz : scene.fdz;
            if (Math.abs(d) < 1e-12) {
                if (o[2] <= box.getMinZInches() || o[2] >= box.getMaxZInches()) {
                    return Double.POSITIVE_INFINITY;
                }
            } else {
                double t0 = (box.getMinZInches() - o[2]) / d;
                double t1 = (box.getMaxZInches() - o[2]) / d;
                enter = Math.max(enter, Math.min(t0, t1));
                exit = Math.min(exit, Math.max(t0, t1));
            }
            return enter < exit && enter < limit ? enter : Double.POSITIVE_INFINITY;
        }

        @Override
        int colourAt(Scene scene, double[] p, double t) {
            return rgb;
        }

        @Override
        double[] normalAt(Scene scene, double[] p, double t) {
            double[] q = robotFrame ? new double[] {lensRobot[0] + t * scene.rdx, lensRobot[1] + t * scene.rdy,
                    lensRobot[2] + t * scene.rdz} : p;
            double[] faces = {
                    Math.abs(q[0] - box.getMinXInches()), Math.abs(q[0] - box.getMaxXInches()),
                    Math.abs(q[1] - box.getMinYInches()), Math.abs(q[1] - box.getMaxYInches()),
                    Math.abs(q[2] - box.getMinZInches()), Math.abs(q[2] - box.getMaxZInches())};
            int nearest = 0;
            for (int i = 1; i < 6; i++) {
                if (faces[i] < faces[nearest]) {
                    nearest = i;
                }
            }
            double[] normal = new double[3];
            normal[nearest / 2] = nearest % 2 == 0 ? -1.0 : 1.0;
            return robotFrame ? scene.vectorToField(normal) : normal;
        }
    }

    private final class TagShape extends Shape {
        final AprilTagLayout.Tag tag;
        final double[] centre;
        final double[] right;
        final double[] down;
        final double[] normal;
        final double cell;
        final double halfOuter;

        TagShape(AprilTagLayout.Tag tag) {
            this.tag = tag;
            RigidTransform3 t = tag.getFieldToTag();
            this.centre = new double[] {tag.getXInches(), tag.getYInches(), tag.getZInches()};
            this.right = new double[] {t.rotation(0, 0), t.rotation(1, 0), t.rotation(2, 0)};
            this.down = new double[] {t.rotation(0, 1), t.rotation(1, 1), t.rotation(2, 1)};
            this.normal = tag.getFaceNormal();
            // The reported size is the black square: 8 of the 10 cells across, the outer ring being white.
            this.cell = tag.getSizeInches() / 8.0;
            this.halfOuter = 5.0 * cell;
        }

        @Override
        double intersect(Scene scene, double limit) {
            double denom = scene.fdx * normal[0] + scene.fdy * normal[1] + scene.fdz * normal[2];
            if (Math.abs(denom) < 1e-12) {
                return Double.POSITIVE_INFINITY;
            }
            double t = ((centre[0] - scene.lensField[0]) * normal[0] + (centre[1] - scene.lensField[1]) * normal[1]
                    + (centre[2] - scene.lensField[2]) * normal[2]) / denom;
            if (!(t > 0.0)) {
                return Double.POSITIVE_INFINITY;
            }
            double px = scene.lensField[0] + t * scene.fdx - centre[0];
            double py = scene.lensField[1] + t * scene.fdy - centre[1];
            double pz = scene.lensField[2] + t * scene.fdz - centre[2];
            double u = px * right[0] + py * right[1] + pz * right[2];
            double v = px * down[0] + py * down[1] + pz * down[2];
            if (Math.abs(u) > halfOuter || Math.abs(v) > halfOuter) {
                return Double.POSITIVE_INFINITY;
            }
            double length = StrictMath.sqrt(scene.fdx * scene.fdx + scene.fdy * scene.fdy + scene.fdz * scene.fdz);
            double biased = Math.max(1e-9, t - TAG_DEPTH_BIAS_INCHES / length);
            return biased < limit ? biased : Double.POSITIVE_INFINITY;
        }

        @Override
        int colourAt(Scene scene, double[] p, double t) {
            double towardLens = scene.fdx * normal[0] + scene.fdy * normal[1] + scene.fdz * normal[2];
            if (towardLens > 0.0) {
                return style.getTagBackRgb();
            }
            // Recompute on the tag's plane: the depth bias moved p a hair off it.
            double length = StrictMath.sqrt(scene.fdx * scene.fdx + scene.fdy * scene.fdy + scene.fdz * scene.fdz);
            double tPlane = t + TAG_DEPTH_BIAS_INCHES / length;
            double px = scene.lensField[0] + tPlane * scene.fdx - centre[0];
            double py = scene.lensField[1] + tPlane * scene.fdy - centre[1];
            double pz = scene.lensField[2] + tPlane * scene.fdz - centre[2];
            double u = px * right[0] + py * right[1] + pz * right[2];
            double v = px * down[0] + py * down[1] + pz * down[2];
            int i = Math.max(0, Math.min(9, (int) StrictMath.floor((u + halfOuter) / cell)));
            int j = Math.max(0, Math.min(9, (int) StrictMath.floor((v + halfOuter) / cell)));
            if (i == 0 || i == 9 || j == 0 || j == 9) {
                return style.getTagWhiteRgb();
            }
            if (i == 1 || i == 8 || j == 1 || j == 8) {
                return style.getTagBlackRgb();
            }
            int bit = mix(tag.getId() * 36 + (j - 2) * 6 + (i - 2)) & 1;
            return bit == 1 ? style.getTagBlackRgb() : style.getTagWhiteRgb();
        }

        @Override
        double[] normalAt(Scene scene, double[] p, double t) {
            double towardLens = scene.fdx * normal[0] + scene.fdy * normal[1] + scene.fdz * normal[2];
            return towardLens > 0.0 ? new double[] {-normal[0], -normal[1], -normal[2]} : normal;
        }
    }

    // ------------------------------------------------------------------------------------------- sensor

    /** Entries in {@link #STANDARD_NORMAL}: a power of two, so a hash's top bits index it. */
    private static final int NORMAL_TABLE_BITS = 16;

    /**
     * Standard-normal samples (mean 0, standard deviation 1), fixed at class load by Box-Muller over a hashed
     * counter. Looking one up per channel costs a hash and an array read; computing Box-Muller per channel with
     * {@link StrictMath} cost about 0.25 microseconds, which at 640 x 400 x 3 channels was most of a frame.
     */
    private static final float[] STANDARD_NORMAL = new float[1 << NORMAL_TABLE_BITS];

    static {
        for (int i = 0; i < STANDARD_NORMAL.length; i++) {
            double u1 = ((mix(i * 2 + 1) >>> 8) + 1) / 16777217.0;   // (0, 1]: log is finite
            double u2 = (mix(i * 2 + 2) >>> 8) / 16777216.0;
            STANDARD_NORMAL[i] = (float) (StrictMath.sqrt(-2.0 * StrictMath.log(u1))
                    * StrictMath.cos(2.0 * Math.PI * u2));
        }
    }

    /**
     * Zero-mean Gaussian sensor noise for one channel of one pixel of one frame, counts; 0 with noise off.
     *
     * @param frameKey the seed and frame number hashed together, once per frame
     */
    private double noise(int frameKey, int pixel, int channel) {
        if (noiseStdDevCounts == 0.0) {
            return 0.0;
        }
        int key = mix(frameKey ^ mix(pixel * 3 + channel));
        return noiseStdDevCounts * STANDARD_NORMAL[key >>> (32 - NORMAL_TABLE_BITS)];
    }

    /**
     * The 32-bit finaliser of MurmurHash3 (public domain): a fixed permutation that scatters every input
     * bit across the output, used as a stateless random source.
     */
    static int mix(int h) {
        h ^= h >>> 16;
        h *= 0x85EBCA6B;
        h ^= h >>> 13;
        h *= 0xC2B2AE35;
        h ^= h >>> 16;
        return h;
    }

    private static byte toByte(double value) {
        long rounded = Math.round(value);
        return (byte) Math.max(0L, Math.min(255L, rounded));
    }

    private static double[] minus(double[] a, double[] b) {
        return new double[] {a[0] - b[0], a[1] - b[1], a[2] - b[2]};
    }

    private static void requireFinite(double value, String name) {
        if (!Double.isFinite(value)) {
            throw new IllegalArgumentException(name + " must be finite, got " + value);
        }
    }

    /** Configures a {@link SimFrameRenderer}. */
    public static final class Builder {

        private final CameraLens lens;
        private final CameraMount mount;
        private FrameStyle style = FrameStyle.defaults();
        private int supersampling = 1;
        private int noiseSeed;
        private double noiseStdDevCounts;
        private double gain = 1.0;

        private Builder(CameraLens lens, CameraMount mount) {
            if (lens == null || mount == null) {
                throw new IllegalArgumentException("a renderer needs a lens and a mount");
            }
            this.lens = lens;
            this.mount = mount;
        }

        /**
         * The colours and light. Default {@link FrameStyle#defaults()}.
         *
         * @param frameStyle the style
         * @return this builder
         */
        public Builder style(FrameStyle frameStyle) {
            if (frameStyle == null) {
                throw new IllegalArgumentException("style must not be null");
            }
            this.style = frameStyle;
            return this;
        }

        /**
         * Rays per pixel along each axis, averaged: 2 casts 4 rays a pixel and smooths edges. Default 1.
         * The per-pixel labels always come from the ray through the pixel's centre.
         *
         * @param raysPerAxis 1 to 4
         * @return this builder
         */
        public Builder supersampling(int raysPerAxis) {
            if (raysPerAxis < 1 || raysPerAxis > 4) {
                throw new IllegalArgumentException("supersampling must be 1 to 4, got " + raysPerAxis);
            }
            this.supersampling = raysPerAxis;
            return this;
        }

        /**
         * Multiplies every pixel before noise and clamping: above 1 is a brighter exposure. Default 1.
         *
         * @param factor the gain, &gt; 0
         * @return this builder
         */
        public Builder gain(double factor) {
            if (!(factor > 0.0) || Double.isInfinite(factor)) {
                throw new IllegalArgumentException("gain must be positive and finite, got " + factor);
            }
            this.gain = factor;
            return this;
        }

        /**
         * Independent Gaussian noise on every channel of every pixel, seeded. Off by default. The value is a
         * PLACEHOLDER of your choosing, not a measured sensor's noise.
         *
         * @param seed            the seed; same seed, same frames
         * @param stdDevCounts    standard deviation, in 8-bit counts, &ge; 0
         * @return this builder
         */
        public Builder sensorNoise(int seed, double stdDevCounts) {
            if (!(stdDevCounts >= 0.0) || Double.isInfinite(stdDevCounts)) {
                throw new IllegalArgumentException("noise must be finite and >= 0, got " + stdDevCounts);
            }
            this.noiseSeed = seed;
            this.noiseStdDevCounts = stdDevCounts;
            return this;
        }

        /** {@return the renderer; precomputes one ray per pixel (and per supersample), so build it once} */
        public SimFrameRenderer build() {
            return new SimFrameRenderer(this);
        }
    }
}
