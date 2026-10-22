package org.horizon36596.simloop.vision;

import org.horizon36596.simloop.field.GamePiece;
import org.horizon36596.simloop.field.ObstacleBox;
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
 * here — the output is the geometry a renderer would draw and a detector would have to recover.
 * {@link SimFrameRenderer} draws the same scene as synthetic pixels when a pixel pipeline needs them.
 *
 * <p><b>What this is for.</b> A vision pipeline is two things: geometry (which pixel is which point on the
 * floor, tracking, velocity) and appearance (colour thresholds, blob splitting). This class is the
 * seam for the first: a test hands the sightings to the pipeline's projection and tracking code and checks
 * the answers against the pieces' true positions. Colour and thresholds need real frames from a real
 * camera; a synthetic scene (including {@link SimFrameRenderer}'s) must never be used to tune them, because
 * a colour range fitted to a renderer's flat colours says nothing about real lighting.
 *
 * <p><b>Visibility rules, stated plainly.</b> A piece is sighted when it is
 * {@linkplain GamePiece#isVisibleOnField() visible on the field} (not held, and not scored into no goal),
 * its centre is in front of the lens by more than its
 * own radius, and its apparent disc (a circle of {@link CameraSighting#getRadiusPx()} about the centre
 * pixel) overlaps the image rectangle. A sighting is <em>clipped</em> when part of that disc is outside
 * the image. What hides a piece depends on the overload. The four-argument
 * {@link #observe(List, double, double, double)} hides only a piece whose centre pixel is covered by the
 * disc of a nearer sighted piece; everything else is transparent. The overload taking
 * {@link CameraOccluders} casts {@value #OCCLUSION_SAMPLES} rays over each piece's disc against the field
 * boxes, the robot boxes (in the robot frame, turning with it) and every nearer piece, reports the share
 * that gets through as {@link CameraSighting#getVisibleFraction()}, and drops a piece below
 * {@link #getMinVisibleFraction()}. A box that holds the piece's centre or the lens does not hide it.
 * Anything not given as a box (curved structures, the far wall unless it is a box) stays transparent.
 *
 * <p>A piece whose centre is no more than its own radius in front of the lens (behind the camera, or so
 * close that the ball would wrap around the lens) is dropped entirely: it is not sighted and it does not
 * occlude anything behind it either. A real camera would see such a piece as a blur filling the frame; the
 * projection has no sensible answer for it, so this camera pretends it is not there.
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

    /**
     * Rays the occluded {@link #observe(List, double, double, double, CameraOccluders)} casts per piece: the
     * centre, six at 0.45 of the radius and twelve at 0.85, spread over the disc the camera sees.
     */
    public static final int OCCLUSION_SAMPLES = 19;

    /** The visible fraction a piece needs, by default, to be reported when occluders are given: half. */
    public static final double DEFAULT_MIN_VISIBLE_FRACTION = 0.5;

    /** Sample offsets across the disc, in radii: {@code SAMPLE_A} along the horizontal axis, {@code B} across. */
    private static final double[] SAMPLE_A = new double[OCCLUSION_SAMPLES];
    private static final double[] SAMPLE_B = new double[OCCLUSION_SAMPLES];

    static {
        int k = 1;
        for (int i = 0; i < 6; i++) {
            double angle = 2.0 * Math.PI * i / 6.0;
            SAMPLE_A[k] = 0.45 * StrictMath.cos(angle);
            SAMPLE_B[k] = 0.45 * StrictMath.sin(angle);
            k++;
        }
        for (int i = 0; i < 12; i++) {
            double angle = 2.0 * Math.PI * (i + 0.5) / 12.0;
            SAMPLE_A[k] = 0.85 * StrictMath.cos(angle);
            SAMPLE_B[k] = 0.85 * StrictMath.sin(angle);
            k++;
        }
    }

    private final double minVisibleFraction;

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
        this(name, lens, mount, DEFAULT_MIN_VISIBLE_FRACTION);
    }

    private SimCamera(String name, CameraLens lens, CameraMount mount, double minVisibleFraction) {
        if (!(minVisibleFraction > 0.0 && minVisibleFraction <= 1.0)) {
            throw new IllegalArgumentException("minimum visible fraction must be in (0, 1], got "
                    + minVisibleFraction);
        }
        this.minVisibleFraction = minVisibleFraction;
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
        List<Candidate> candidates = candidates(pieces, robotXInches, robotYInches, robotHeadingRad);

        // Occlusion: drop a candidate whose centre pixel lies inside the disc of a nearer candidate.
        List<CameraSighting> visible = new ArrayList<>();
        for (Candidate candidate : candidates) {
            boolean hidden = false;
            for (Candidate other : candidates) {
                if (other == candidate || other.sighting.getRangeInches() >= candidate.sighting.getRangeInches()) {
                    continue;
                }
                double ddx = candidate.sighting.getCentreXPx() - other.sighting.getCentreXPx();
                double ddy = candidate.sighting.getCentreYPx() - other.sighting.getCentreYPx();
                if (ddx * ddx + ddy * ddy < other.sighting.getRadiusPx() * other.sighting.getRadiusPx()) {
                    hidden = true;
                    break;
                }
            }
            if (!hidden) {
                visible.add(candidate.sighting);
            }
        }
        return keep(visible, robotXInches, robotYInches, robotHeadingRad);
    }

    /**
     * Work out what the camera sees this tick when solid things can block the view: the field's structures,
     * the robot's own body, and the other pieces. Call once per tick instead of
     * {@link #observe(List, double, double, double)}; the result is also kept for {@link #record}.
     *
     * <p><b>The occlusion rule.</b> For each piece that passes the image test, {@link #OCCLUSION_SAMPLES}
     * rays go from the lens to points spread over the piece's disc as the camera sees it (the centre, six
     * points at 0.45 of the radius and twelve at 0.85). A ray is blocked when, before it reaches the piece's
     * surface, it passes through a box in {@code occluders} or through another piece on the field. The
     * piece's {@link CameraSighting#getVisibleFraction() visible fraction} is the share of rays not blocked,
     * and the piece is reported only when that is at least {@link #getMinVisibleFraction()}. A box that
     * contains the piece's centre (a piece resting inside a structure) does not hide it, and a box that
     * contains the lens is skipped. The centre pixel and radius reported are the whole piece's, even when
     * part of it is hidden; a real detector's blob centre would move toward the visible part.
     *
     * @param pieces          every piece, from {@code GamePieceTracker.getPieces()}
     * @param robotXInches    robot centre, field frame, inches
     * @param robotYInches    robot centre, field frame, inches
     * @param robotHeadingRad robot heading, radians CCW from field +X
     * @param occluders       field and robot boxes that block the view; {@link CameraOccluders#none()} leaves
     *                        only piece-on-piece occlusion
     * @return the sightings, sorted by piece id, unmodifiable
     */
    public List<CameraSighting> observe(List<GamePiece> pieces, double robotXInches, double robotYInches,
            double robotHeadingRad, CameraOccluders occluders) {
        if (occluders == null) {
            throw new IllegalArgumentException("occluders must not be null; use CameraOccluders.none()");
        }
        List<Candidate> candidates = candidates(pieces, robotXInches, robotYInches, robotHeadingRad);
        double cos = StrictMath.cos(robotHeadingRad);
        double sin = StrictMath.sin(robotHeadingRad);
        double[] lensRobot = mount.opticalToRobot(0.0, 0.0, 0.0);
        double[] lensField = robotToField(lensRobot, robotXInches, robotYInches, cos, sin);

        // Every piece that can block a ray: on the field and in front of the lens by more than its radius
        // (a piece closer than that is dropped entirely, as observe() without occluders drops it).
        List<GamePiece> blockers = new ArrayList<>();
        for (GamePiece piece : pieces) {
            if (!piece.isVisibleOnField()) {
                continue;
            }
            double[] robot = fieldToRobot(piece, robotXInches, robotYInches, cos, sin);
            if (mount.robotToOptical(robot[0], robot[1], robot[2])[2] > piece.getRadiusInches()) {
                blockers.add(piece);
            }
        }
        List<ObstacleBox> robotBoxes = withoutBoxesContaining(occluders.getRobotBoxes(), lensRobot);
        List<ObstacleBox> fieldBoxes = withoutBoxesContaining(occluders.getFieldBoxes(), lensField);

        List<CameraSighting> visible = new ArrayList<>();
        for (Candidate candidate : candidates) {
            double fraction = visibleFraction(candidate.piece, candidate.robot, lensRobot, lensField, cos, sin,
                    robotBoxes, fieldBoxes, blockers);
            if (fraction > 0.0 && fraction >= minVisibleFraction) {
                CameraSighting s = candidate.sighting;
                visible.add(new CameraSighting(s.getPieceId(), s.getKind(), s.getCentreXPx(), s.getCentreYPx(),
                        s.getRadiusPx(), s.getRangeInches(), s.isClipped(), fraction));
            }
        }
        return keep(visible, robotXInches, robotYInches, robotHeadingRad);
    }

    /** One piece that passed the image test, with its centre in the robot frame. */
    private static final class Candidate {
        final GamePiece piece;
        final double[] robot;
        final CameraSighting sighting;

        Candidate(GamePiece piece, double[] robot, CameraSighting sighting) {
            this.piece = piece;
            this.robot = robot;
            this.sighting = sighting;
        }
    }

    /** Every piece in front of the lens whose disc overlaps the image, in the order given, unoccluded. */
    private List<Candidate> candidates(List<GamePiece> pieces, double robotXInches, double robotYInches,
            double robotHeadingRad) {
        if (pieces == null) {
            throw new IllegalArgumentException("pieces must not be null");
        }
        requireFinite(robotXInches, "robotXInches");
        requireFinite(robotYInches, "robotYInches");
        requireFinite(robotHeadingRad, "robotHeadingRad");

        double cos = StrictMath.cos(robotHeadingRad);
        double sin = StrictMath.sin(robotHeadingRad);

        List<Candidate> candidates = new ArrayList<>();
        for (GamePiece piece : pieces) {
            if (!piece.isVisibleOnField()) {
                continue;
            }
            // Field -> robot frame: translate to the robot, rotate by -heading. Height is already robot up.
            double[] robot = fieldToRobot(piece, robotXInches, robotYInches, cos, sin);
            double[] optical = mount.robotToOptical(robot[0], robot[1], robot[2]);
            double depth = optical[2];
            if (depth <= piece.getRadiusInches()) {
                continue;                                   // behind, or wrapped around, the lens
            }
            double[] pixel = lens.project(optical[0], optical[1], depth);
            if (pixel == null) {
                continue;                                   // past where the lens's distortion model holds
            }
            double centreX = pixel[0];
            double centreY = pixel[1];
            double range = StrictMath.sqrt(optical[0] * optical[0] + optical[1] * optical[1] + depth * depth);
            // Apparent radius as focal length times real radius over distance to the lens -- an
            // approximation, documented on CameraSighting.getRadiusPx(), kept because it is what a simple
            // synthetic renderer draws -- scaled by the lens's local magnification, exactly 1 without
            // distortion.
            double radiusPx = lens.getFocalXPx() * piece.getRadiusInches() / range
                    * lens.localMagnification(optical[0] / depth, optical[1] / depth);

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
            candidates.add(new Candidate(piece, robot, new CameraSighting(piece.getId(), piece.getKind(),
                    centreX, centreY, radiusPx, range, clipped)));
        }
        return candidates;
    }

    /** Sorts by piece id, keeps the list and the camera pose for {@link #record}, and returns the list. */
    private List<CameraSighting> keep(List<CameraSighting> visible, double robotXInches, double robotYInches,
            double robotHeadingRad) {
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

    /** A piece's centre in the robot frame: translate to the robot, rotate by -heading. */
    private static double[] fieldToRobot(GamePiece piece, double robotX, double robotY, double cos, double sin) {
        double dx = piece.getFieldXInches() - robotX;
        double dy = piece.getFieldYInches() - robotY;
        return new double[] {dx * cos + dy * sin, -dx * sin + dy * cos, piece.getFieldZInches()};
    }

    private static double[] robotToField(double[] robot, double robotX, double robotY, double cos, double sin) {
        return new double[] {robotX + robot[0] * cos - robot[1] * sin, robotY + robot[0] * sin + robot[1] * cos,
                robot[2]};
    }

    /** The share of the {@link #OCCLUSION_SAMPLES} rays from the lens to the piece's disc that are not blocked. */
    private static double visibleFraction(GamePiece piece, double[] centre, double[] lensRobot,
            double[] lensField, double cos, double sin, List<ObstacleBox> robotBoxes,
            List<ObstacleBox> fieldBoxes, List<GamePiece> blockers) {
        double radius = piece.getRadiusInches();
        double vx = centre[0] - lensRobot[0];
        double vy = centre[1] - lensRobot[1];
        double vz = centre[2] - lensRobot[2];
        double length = StrictMath.sqrt(vx * vx + vy * vy + vz * vz);
        // Two unit vectors across the line of sight: u1 horizontal, u2 = (line of sight) x u1.
        double u1x = vy;
        double u1y = -vx;
        double u1Length = StrictMath.sqrt(u1x * u1x + u1y * u1y);
        if (u1Length < 1e-9) {
            u1x = 1.0;
            u1y = 0.0;
            u1Length = 1.0;
        }
        u1x /= u1Length;
        u1y /= u1Length;
        double wx = vx / length;
        double wy = vy / length;
        double wz = vz / length;
        double u2x = -wz * u1y;
        double u2y = wz * u1x;
        double u2z = wx * u1y - wy * u1x;

        double[] centreField = {piece.getFieldXInches(), piece.getFieldYInches(), piece.getFieldZInches()};
        List<ObstacleBox> robotCandidates = withoutBoxesContaining(robotBoxes, centre);
        List<ObstacleBox> fieldCandidates = withoutBoxesContaining(fieldBoxes, centreField);

        int clear = 0;
        for (int k = 0; k < OCCLUSION_SAMPLES; k++) {
            double a = SAMPLE_A[k] * radius;
            double b = SAMPLE_B[k] * radius;
            double[] dirRobot = {vx + a * u1x + b * u2x, vy + a * u1y + b * u2y, vz + b * u2z};
            double tSurface = sphereEntry(lensRobot, dirRobot, centre, radius);
            if (!(tSurface > 0.0)) {
                continue;                                   // cannot happen for a point inside the disc
            }
            if (blockedByBoxes(lensRobot, dirRobot, tSurface, robotCandidates)) {
                continue;
            }
            double[] dirField = {dirRobot[0] * cos - dirRobot[1] * sin, dirRobot[0] * sin + dirRobot[1] * cos,
                    dirRobot[2]};
            if (blockedByBoxes(lensField, dirField, tSurface, fieldCandidates)) {
                continue;
            }
            boolean hitPiece = false;
            for (GamePiece other : blockers) {
                if (other.getId() == piece.getId()) {
                    continue;
                }
                double[] otherCentre = {other.getFieldXInches(), other.getFieldYInches(), other.getFieldZInches()};
                double t = sphereEntry(lensField, dirField, otherCentre, other.getRadiusInches());
                if (t > 0.0 && t < tSurface) {
                    hitPiece = true;
                    break;
                }
            }
            if (!hitPiece) {
                clear++;
            }
        }
        return clear / (double) OCCLUSION_SAMPLES;
    }

    private static List<ObstacleBox> withoutBoxesContaining(List<ObstacleBox> boxes, double[] point) {
        List<ObstacleBox> kept = new ArrayList<>(boxes.size());
        for (ObstacleBox box : boxes) {
            if (!CameraOccluders.contains(box, point[0], point[1], point[2])) {
                kept.add(box);
            }
        }
        return kept;
    }

    private static boolean blockedByBoxes(double[] origin, double[] dir, double tLimit, List<ObstacleBox> boxes) {
        for (ObstacleBox box : boxes) {
            if (CameraOccluders.enterParameter(origin, dir, box, 0.0, tLimit) < tLimit) {
                return true;
            }
        }
        return false;
    }

    /**
     * The smallest positive {@code t} at which {@code origin + t * dir} meets the sphere's surface from
     * outside, or {@code NaN} when the ray misses it, starts inside it, or the sphere is behind the origin.
     */
    static double sphereEntry(double[] origin, double[] dir, double[] centre, double radius) {
        double ox = origin[0] - centre[0];
        double oy = origin[1] - centre[1];
        double oz = origin[2] - centre[2];
        double a = dir[0] * dir[0] + dir[1] * dir[1] + dir[2] * dir[2];
        double b = 2.0 * (ox * dir[0] + oy * dir[1] + oz * dir[2]);
        double c = ox * ox + oy * oy + oz * oz - radius * radius;
        if (c <= 0.0) {
            return Double.NaN;
        }
        double disc = b * b - 4.0 * a * c;
        if (disc < 0.0) {
            return Double.NaN;
        }
        double t0 = (-b - StrictMath.sqrt(disc)) / (2.0 * a);
        return t0 > 0.0 ? t0 : Double.NaN;
    }

    /** {@return the visible fraction a piece needs to be reported by the occluded {@code observe}, 0 to 1} */
    public double getMinVisibleFraction() {
        return minVisibleFraction;
    }

    /**
     * This camera with a different minimum visible fraction for the occluded
     * {@link #observe(List, double, double, double, CameraOccluders)}: a detector that still finds a ball
     * when a quarter of it shows wants 0.25. The default is {@link #DEFAULT_MIN_VISIBLE_FRACTION}.
     *
     * @param fraction the minimum share of sample rays that must reach the piece, in {@code (0, 1]}
     * @return a new camera with the same name, lens and mount
     */
    public SimCamera withMinVisibleFraction(double fraction) {
        return new SimCamera(name, lens, mount, fraction);
    }

    /** {@return what {@link #observe} last returned; empty before the first call} */
    public List<CameraSighting> getLastSightings() {
        return lastSightings;
    }

    /**
     * The point on a horizontal plane that a pixel looks at — the inverse of {@link #observe}'s projection
     * for a point at a known height, in the robot frame. This is the calculation a ground-plane vision
     * pipeline does to turn a detected ball centre into a floor position, so a test can round-trip a
     * sighting through it and check the result against the piece's true position. A distorted lens is
     * undistorted first ({@link CameraLens#undistortPixel}), as a calibrated pipeline does.
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
        double[] normalized = lens.undistortPixel(centreXPx, centreYPx);
        double xOverZ = normalized[0];
        double yOverZ = normalized[1];
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
     *       double[]; {@code sightings/clipped} — boolean[]; {@code sightings/visibleFraction} — double[]
     *       (1 without occluders); all index-aligned</li>
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
        double[] visibleFraction = new double[count];
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
            visibleFraction[i] = sighting.getVisibleFraction();
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
        Logger.recordOutput(prefix + "/sightings/visibleFraction", visibleFraction);
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
