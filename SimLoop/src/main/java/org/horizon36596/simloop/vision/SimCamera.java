package org.horizon36596.simloop.vision;

import org.horizon36596.simloop.field.GamePiece;
import org.psilynx.psikit.core.Logger;
import org.psilynx.psikit.core.wpi.math.Pose3d;
import org.psilynx.psikit.core.wpi.math.Rotation3d;
import org.psilynx.psikit.core.wpi.math.Translation3d;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

/**
 * A camera on the robot that "sees" game pieces: given every piece's true position and the robot's pose, it
 * works out which pieces fall inside its image and where, as {@link CameraSighting}s. No image is rendered
 * — the output is the geometry a renderer would draw and a detector would have to recover.
 *
 * <p><b>What this is for.</b> A vision pipeline is two things: geometry (which pixel is which point on the
 * floor, tracking, velocity) and appearance (colour thresholds, blob splitting). The sim can only ever
 * test the first, and this class is the seam for that: a test hands the sightings to the pipeline's
 * projection and tracking code and checks the answers against the pieces' true positions. Colour and
 * thresholds need real frames from a real camera; a synthetic scene must never be used to tune them,
 * because a colour range fitted to a renderer's flat colours says nothing about real lighting.
 *
 * <p><b>Visibility rules, stated plainly.</b> A piece is sighted when it is
 * {@linkplain GamePiece#isVisibleOnField() visible on the field} (not held, and not scored into no goal),
 * its centre is in front of the lens by more than its
 * own radius, and its apparent disc (a circle of {@link CameraSighting#getRadiusPx()} about the centre
 * pixel) overlaps the image rectangle. A sighting is <em>clipped</em> when part of that disc is outside
 * the image. A piece whose centre pixel is covered by the disc of a nearer sighted piece is
 * <em>occluded</em> and not reported at all — the one bit of "what is in front of what" the geometry can
 * answer without rendering. Nothing else occludes: the robot's own body, the field structures and the far
 * wall are all transparent to this camera, which is wrong for a real camera and documented here so a test
 * does not depend on it.
 *
 * <p>A piece whose centre is no more than its own radius in front of the lens (behind the camera, or so
 * close that the ball would wrap around the lens) is dropped entirely: it is not sighted and it does not
 * occlude anything behind it either. A real camera would see such a piece as a blur filling the frame; the
 * pinhole projection has no sensible answer for it, so this camera pretends it is not there.
 *
 * <p><b>Frames.</b> Pieces and the robot pose are in the field frame (FTC-Cartesian, centre origin, inches,
 * heading CCW from +X, conventions §7). The mount is in the robot frame and the projection in the optical
 * frame, as {@link CameraMount} defines them. The one WPI struct written, the camera's field-frame
 * {@link Pose3d}, follows WPILib's camera convention — {@code +X} along the boresight, {@code +Z} up,
 * pitch positive nose-down — in metres, so AdvantageScope's 3D Field can use it as a <i>Camera Override</i>
 * to look through this camera.
 *
 * <p>Deterministic and clock-free (domain R5): the same pieces and pose give the same sightings, sorted
 * by piece id whatever order the pieces were handed in, with all trigonometry in {@link StrictMath}.
 */
public final class SimCamera {

    /** Inches to metres, applied only where a WPI geometry struct is handed to PsiKit. */
    private static final double METRES_PER_INCH = 0.0254;

    private final String name;
    private final CameraLens lens;
    private final CameraMount mount;

    private List<CameraSighting> lastSightings = Collections.emptyList();

    /** The camera's field pose at the last {@link #observe}; {@code null} until the first call. */
    private Pose3d lastFieldPose = null;

    /**
     * A simulated camera with the given lens and mount.
     *
     * @param name  a short label for log keys, e.g. {@code "front"}; letters, digits, {@code _} and
     *              {@code -} only, so it is always exactly one log-key segment
     * @param lens  the intrinsics
     * @param mount the extrinsics
     */
    public SimCamera(String name, CameraLens lens, CameraMount mount) {
        if (name == null || !name.matches("[A-Za-z0-9_-]+")) {
            throw new IllegalArgumentException("a camera name becomes one segment of a log key, so it must be"
                    + " non-empty and use only letters, digits, '_' and '-': \"" + name + "\"");
        }
        if (lens == null || mount == null) {
            throw new IllegalArgumentException("a sim camera needs a lens and a mount");
        }
        this.name = name;
        this.lens = lens;
        this.mount = mount;
    }

    /**
     * Work out what the camera sees this tick. Call once per tick after the pieces and the robot have
     * been moved; the result is also kept for {@link #record}.
     *
     * @param pieces          every piece, from {@code GamePieceTracker.getPieces()}
     * @param robotXInches    robot centre, field frame, inches
     * @param robotYInches    robot centre, field frame, inches
     * @param robotHeadingRad robot heading, radians CCW from field +X
     * @return the sightings, sorted by piece id (whatever order {@code pieces} is in), unmodifiable
     */
    public List<CameraSighting> observe(List<GamePiece> pieces, double robotXInches, double robotYInches,
            double robotHeadingRad) {
        if (pieces == null) {
            throw new IllegalArgumentException("pieces must not be null");
        }
        requireFinite(robotXInches, "robotXInches");
        requireFinite(robotYInches, "robotYInches");
        requireFinite(robotHeadingRad, "robotHeadingRad");

        double cos = StrictMath.cos(robotHeadingRad);
        double sin = StrictMath.sin(robotHeadingRad);

        List<CameraSighting> candidates = new ArrayList<>();
        for (GamePiece piece : pieces) {
            if (!piece.isVisibleOnField()) {
                continue;
            }
            // Field -> robot frame: translate to the robot, rotate by -heading. Height is already robot up.
            double dx = piece.getFieldXInches() - robotXInches;
            double dy = piece.getFieldYInches() - robotYInches;
            double forward = dx * cos + dy * sin;
            double left = -dx * sin + dy * cos;
            double[] optical = mount.robotToOptical(forward, left, piece.getFieldZInches());
            double depth = optical[2];
            if (depth <= piece.getRadiusInches()) {
                continue;                                   // behind, or wrapped around, the lens
            }
            double centreX = lens.getCentreXPx() + lens.getFocalXPx() * optical[0] / depth;
            double centreY = lens.getCentreYPx() + lens.getFocalYPx() * optical[1] / depth;
            double range = StrictMath.sqrt(optical[0] * optical[0] + optical[1] * optical[1] + depth * depth);
            // Apparent radius as focal length times real radius over distance to the lens -- an
            // approximation, documented on CameraSighting.getRadiusPx(), kept because it is what a simple
            // synthetic renderer draws.
            double radiusPx = lens.getFocalXPx() * piece.getRadiusInches() / range;

            // Disc-versus-rectangle overlap: the nearest image point to the disc's centre must lie inside
            // the disc. (Testing the disc's bounding square instead would wrongly count a disc sitting just
            // off a corner of the image.)
            double nearestImageX = Math.max(0.0, Math.min(lens.getWidthPx(), centreX));
            double nearestImageY = Math.max(0.0, Math.min(lens.getHeightPx(), centreY));
            double offX = centreX - nearestImageX;
            double offY = centreY - nearestImageY;
            boolean overlapsImage = offX * offX + offY * offY < radiusPx * radiusPx;
            if (!overlapsImage) {
                continue;
            }
            boolean clipped = centreX - radiusPx < 0.0 || centreX + radiusPx > lens.getWidthPx()
                    || centreY - radiusPx < 0.0 || centreY + radiusPx > lens.getHeightPx();
            candidates.add(new CameraSighting(piece.getId(), piece.getKind(), centreX, centreY, radiusPx,
                    range, clipped));
        }

        // Occlusion: drop a candidate whose centre pixel lies inside the disc of a nearer candidate.
        List<CameraSighting> visible = new ArrayList<>();
        for (CameraSighting candidate : candidates) {
            boolean hidden = false;
            for (CameraSighting other : candidates) {
                if (other == candidate || other.getRangeInches() >= candidate.getRangeInches()) {
                    continue;
                }
                double ddx = candidate.getCentreXPx() - other.getCentreXPx();
                double ddy = candidate.getCentreYPx() - other.getCentreYPx();
                if (ddx * ddx + ddy * ddy < other.getRadiusPx() * other.getRadiusPx()) {
                    hidden = true;
                    break;
                }
            }
            if (!hidden) {
                visible.add(candidate);
            }
        }

        // Piece id order, as documented. The list a tracker hands out happens to be in id order already, but
        // a caller may pass a filtered or reordered list, and the output order must not depend on that.
        Collections.sort(visible, new Comparator<CameraSighting>() {
            @Override
            public int compare(CameraSighting a, CameraSighting b) {
                return Integer.compare(a.getPieceId(), b.getPieceId());
            }
        });
        lastSightings = Collections.unmodifiableList(visible);
        lastFieldPose = fieldPose3d(robotXInches, robotYInches, robotHeadingRad);
        return lastSightings;
    }

    /** {@return what {@link #observe} last returned; empty before the first call} */
    public List<CameraSighting> getLastSightings() {
        return lastSightings;
    }

    /**
     * The point on a horizontal plane that a pixel looks at — the inverse of {@link #observe}'s projection
     * for a point at a known height, in the robot frame. This is the calculation a ground-plane vision
     * pipeline does to turn a detected ball centre into a floor position, so a test can round-trip a
     * sighting through it and check the result against the piece's true position.
     *
     * @param centreXPx           the pixel's x, from the left edge
     * @param centreYPx           the pixel's y, from the top edge
     * @param pointHeightInches   the height of the plane the ray is intersected with — the ball's centre
     *                            height, i.e. its radius, for a ball on the floor
     * @return the point in the robot frame, or {@code null} when the ray never reaches that plane (it
     *         points at or above the horizon relative to the plane)
     */
    public RobotFramePoint groundPointInRobotFrame(double centreXPx, double centreYPx, double pointHeightInches) {
        requireFinite(centreXPx, "centreXPx");
        requireFinite(centreYPx, "centreYPx");
        requireFinite(pointHeightInches, "pointHeightInches");
        // A ray through the pixel in the optical frame, at unit depth.
        double xOverZ = (centreXPx - lens.getCentreXPx()) / lens.getFocalXPx();
        double yOverZ = (centreYPx - lens.getCentreYPx()) / lens.getFocalYPx();
        double[] rayEnd = mount.opticalToRobot(xOverZ, yOverZ, 1.0);
        double[] rayStart = mount.opticalToRobot(0.0, 0.0, 0.0);
        double dForward = rayEnd[0] - rayStart[0];
        double dLeft = rayEnd[1] - rayStart[1];
        double dUp = rayEnd[2] - rayStart[2];
        double drop = rayStart[2] - pointHeightInches;      // how far down to the plane
        if (dUp >= -1e-12 || drop <= 0.0) {
            return null;                                   // looking level or up, or the plane is above the lens
        }
        double t = drop / -dUp;
        return new RobotFramePoint(rayStart[0] + t * dForward, rayStart[1] + t * dLeft);
    }

    /**
     * The camera's pose in the field, for AdvantageScope. WPILib camera convention: {@code +X} along the
     * boresight, {@code +Y} left, {@code +Z} up; positive pitch tilts the boresight down. Metres.
     *
     * @param robotXInches    robot centre x, inches, field frame
     * @param robotYInches    robot centre y, inches, field frame
     * @param robotHeadingRad robot heading, radians CCW from field +X
     * @return the camera's field-frame pose in metres
     */
    public Pose3d fieldPose3d(double robotXInches, double robotYInches, double robotHeadingRad) {
        double cos = StrictMath.cos(robotHeadingRad);
        double sin = StrictMath.sin(robotHeadingRad);
        double fieldX = robotXInches + mount.getForwardInches() * cos - mount.getLeftInches() * sin;
        double fieldY = robotYInches + mount.getForwardInches() * sin + mount.getLeftInches() * cos;
        Translation3d translation = new Translation3d(fieldX * METRES_PER_INCH, fieldY * METRES_PER_INCH,
                mount.getUpInches() * METRES_PER_INCH);
        // Rotation3d(roll, pitch, yaw), applied yaw then pitch about the yawed Y axis, is exactly a level
        // camera turned to face heading+yaw and then tilted down by pitch. Positive WPI pitch rotates +X
        // toward -Z (right-hand rule about +Y), which is "down", matching CameraMount's sign.
        Rotation3d rotation = new Rotation3d(0.0, mount.getPitchRad(), robotHeadingRad + mount.getYawRad());
        return new Pose3d(translation, rotation);
    }

    /**
     * Publish the last {@link #observe} result under {@code keyPrefix/<name>/...}. Keys:
     * <ul>
     *   <li>{@code pose3d} — the camera's field pose, metres ({@link #fieldPose3d}); drag it onto the 3D
     *       Field as <i>Camera Override</i> to look through the camera</li>
     *   <li>{@code sightingCount} — int</li>
     *   <li>{@code sightings/pieceIds} — int[]; {@code sightings/kinds} — String[];
     *       {@code sightings/centreXPx}, {@code centreYPx}, {@code radiusPx}, {@code rangeInches} —
     *       double[]; {@code sightings/clipped} — boolean[]; all index-aligned</li>
     *   <li>{@code seenPositions3d} — Translation3d[] of the sighted pieces' true positions, metres, for
     *       a <i>Vision Target</i> object on the 3D Field, so a viewer sees which balls the camera saw</li>
     * </ul>
     *
     * @param keyPrefix e.g. {@code "Vision"}; keys land under {@code Vision/<camera name>/}
     * @param pieces    the same list handed to {@link #observe}, to look up sighted pieces' positions
     * @throws IllegalStateException if {@link #observe} has never been called: there is no camera pose to
     *                               log yet, and logging one at the field origin would draw the camera
     *                               somewhere it never was
     */
    public void record(String keyPrefix, List<GamePiece> pieces) {
        if (keyPrefix == null || keyPrefix.trim().isEmpty()) {
            throw new IllegalArgumentException("camera log key prefix must be a non-blank string");
        }
        if (lastFieldPose == null) {
            throw new IllegalStateException("camera \"" + name + "\": record() was called before observe(),"
                    + " so there is no camera pose or sighting to log yet. Call observe() first each tick.");
        }
        String prefix = keyPrefix.trim() + "/" + name;
        int count = lastSightings.size();
        int[] pieceIds = new int[count];
        String[] kinds = new String[count];
        double[] centreX = new double[count];
        double[] centreY = new double[count];
        double[] radius = new double[count];
        double[] range = new double[count];
        boolean[] clipped = new boolean[count];
        Translation3d[] seen = new Translation3d[count];
        for (int i = 0; i < count; i++) {
            CameraSighting sighting = lastSightings.get(i);
            pieceIds[i] = sighting.getPieceId();
            kinds[i] = sighting.getKind();
            centreX[i] = sighting.getCentreXPx();
            centreY[i] = sighting.getCentreYPx();
            radius[i] = sighting.getRadiusPx();
            range[i] = sighting.getRangeInches();
            clipped[i] = sighting.isClipped();
            GamePiece piece = pieceWithId(pieces, sighting.getPieceId());
            seen[i] = new Translation3d(piece.getFieldXInches() * METRES_PER_INCH,
                    piece.getFieldYInches() * METRES_PER_INCH, piece.getFieldZInches() * METRES_PER_INCH);
        }
        Logger.recordOutput(prefix + "/pose3d", Pose3d.struct, lastFieldPose);
        Logger.recordOutput(prefix + "/sightingCount", count);
        Logger.recordOutput(prefix + "/sightings/pieceIds", pieceIds);
        Logger.recordOutput(prefix + "/sightings/kinds", kinds);
        Logger.recordOutput(prefix + "/sightings/centreXPx", centreX);
        Logger.recordOutput(prefix + "/sightings/centreYPx", centreY);
        Logger.recordOutput(prefix + "/sightings/radiusPx", radius);
        Logger.recordOutput(prefix + "/sightings/rangeInches", range);
        Logger.recordOutput(prefix + "/sightings/clipped", clipped);
        Logger.recordOutput(prefix + "/seenPositions3d", seen);
    }

    // The list a tracker hands out is in id order, but nothing here may rely on that: a caller may pass a
    // filtered list, and a wrong ball drawn as "seen" is exactly the kind of silent error a vision test
    // is meant to catch.
    private static GamePiece pieceWithId(List<GamePiece> pieces, int pieceId) {
        for (GamePiece piece : pieces) {
            if (piece.getId() == pieceId) {
                return piece;
            }
        }
        throw new IllegalArgumentException("sighting refers to piece " + pieceId
                + ", which is not in the list of pieces handed to record()");
    }

    /** {@return the camera's name, as used in its log keys} */
    public String getName() {
        return name;
    }

    /** {@return the intrinsics} */
    public CameraLens getLens() {
        return lens;
    }

    /** {@return the extrinsics} */
    public CameraMount getMount() {
        return mount;
    }

    private static void requireFinite(double value, String name) {
        if (!Double.isFinite(value)) {
            throw new IllegalArgumentException(name + " must be finite, got " + value);
        }
    }
}
