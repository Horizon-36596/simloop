package org.horizon36596.simloop.vision;

import org.horizon36596.simloop.field.ObstacleBox;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * A camera on the robot that "sees" AprilTags: given a team-supplied {@link AprilTagLayout} and the robot's
 * pose, it works out which tags a detector would find and where, as {@link TagSighting}s. No image is
 * rendered here ({@link SimFrameRenderer} draws tags as synthetic pixels, not decodable). It is the tag counterpart of {@link SimCamera}, and uses the same {@link CameraLens} and
 * {@link CameraMount}, so one camera description serves both.
 *
 * <p><b>Visibility rules, in the order they are checked</b> (the first one that fails is what
 * {@link #visibility} reports):
 * <ol>
 *   <li>{@link Visibility#OUT_OF_RANGE}: the lens-to-tag-centre distance is outside
 *       {@code [minRangeInches, maxRangeInches]}.</li>
 *   <li>{@link Visibility#FACING_AWAY}: the angle between the tag's face normal and the line from the tag
 *       to the lens is {@code maxIncidenceDegrees} or more. With the default of 90 this rejects exactly
 *       the tags seen from behind or edge-on. A tilted tag's normal is tilted with it.</li>
 *   <li>{@link Visibility#BEHIND_CAMERA}: any corner of the tag is not in front of the lens.</li>
 *   <li>{@link Visibility#OUTSIDE_IMAGE}: any corner projects outside the image rectangle, through the
 *       lens's distortion when it has one, or lies where the distortion model no longer holds. A detector
 *       needs the whole tag border, so a tag that is only partly in frame is not seen at all.</li>
 *   <li>{@link Visibility#OCCLUDED}: the straight line from the lens to any corner passes through the
 *       inside of one of the {@link ObstacleBox}es given at construction, or of a box in the
 *       {@link CameraOccluders} given to {@link #observe(AprilTagLayout, double, double, double,
 *       CameraOccluders)}: field boxes, and robot boxes such as the robot's own chassis. A tag mounted on a
 *       box's face is not hidden by that box. Nothing else occludes: other tags and anything not given as a
 *       box are transparent.</li>
 * </ol>
 *
 * <p><b>Frames.</b> The layout and the robot pose are in the field frame (FTC-Cartesian, centre origin,
 * inches, heading CCW from {@code +X}). The mount is in the robot frame, the projection in the optical
 * frame, both as {@link CameraMount} defines them. Pixels (the centre and the corners) go through the
 * lens's distortion model when it has one ({@link CameraLens#project}); the angles {@code tx} and
 * {@code ty} and the transforms are the undistorted geometry a calibrated detector recovers.
 *
 * <p>Deterministic and clock-free (domain R5): the same layout and pose give the same sightings, in
 * ascending tag id order, with all trigonometry in {@link StrictMath}.
 */
public final class SimTagCamera {

    /** Why a tag is or is not seen; see the class comment for the order these are checked in. */
    public enum Visibility {
        /** Seen: every check passed. */
        VISIBLE,
        /** The tag centre is nearer than the minimum or farther than the maximum range. */
        OUT_OF_RANGE,
        /** The camera is behind the tag's face, edge-on to it, or past the maximum incidence angle. */
        FACING_AWAY,
        /** At least one corner of the tag is not in front of the lens. */
        BEHIND_CAMERA,
        /** At least one corner projects outside the image. */
        OUTSIDE_IMAGE,
        /** An obstacle box blocks the line of sight to at least one corner. */
        OCCLUDED
    }

    /** Line-of-sight parameter margin, so a tag lying on a box face is not hidden by that same box. */
    private static final double SEGMENT_END_MARGIN = 1e-6;

    private final CameraLens lens;
    private final CameraMount mount;
    private final double minRangeInches;
    private final double maxRangeInches;
    private final double maxIncidenceDegrees;
    private final double cosMaxIncidence;
    private final List<ObstacleBox> occluders;
    private final RigidTransform3 robotToCamera;

    /**
     * A tag camera with these optics, this mount and these detection limits.
     *
     * @param lens                the intrinsics; its image size is the detector's image
     * @param mount               the extrinsics
     * @param minRangeInches      tags nearer than this (lens to tag centre) are not seen, inches (&ge; 0)
     * @param maxRangeInches      tags farther than this are not seen, inches (&gt; min; may be infinite)
     * @param maxIncidenceDegrees tags seen at this angle or more off their face normal are not seen,
     *                            degrees, in {@code (0, 90]}
     * @param occluders           boxes that block line of sight, field frame; may be empty, not null
     * @throws IllegalArgumentException on a null argument, a negative or NaN range, an empty range
     *                                  interval, or an incidence limit outside {@code (0, 90]}
     */
    public SimTagCamera(CameraLens lens, CameraMount mount, double minRangeInches, double maxRangeInches,
            double maxIncidenceDegrees, List<ObstacleBox> occluders) {
        if (lens == null || mount == null || occluders == null) {
            throw new IllegalArgumentException("a tag camera needs a lens, a mount and an occluder list");
        }
        if (!(minRangeInches >= 0.0) || Double.isInfinite(minRangeInches)) {
            throw new IllegalArgumentException("minimum range must be finite and >= 0, got " + minRangeInches);
        }
        if (!(maxRangeInches > minRangeInches)) {
            throw new IllegalArgumentException("maximum range must exceed the minimum, got [" + minRangeInches
                    + ", " + maxRangeInches + "]");
        }
        if (!(maxIncidenceDegrees > 0.0 && maxIncidenceDegrees <= 90.0)) {
            throw new IllegalArgumentException("maximum incidence must be in (0, 90] degrees, got "
                    + maxIncidenceDegrees);
        }
        this.lens = lens;
        this.mount = mount;
        this.minRangeInches = minRangeInches;
        this.maxRangeInches = maxRangeInches;
        this.maxIncidenceDegrees = maxIncidenceDegrees;
        this.cosMaxIncidence = StrictMath.cos(StrictMath.toRadians(maxIncidenceDegrees));
        this.occluders = Collections.unmodifiableList(new ArrayList<ObstacleBox>(occluders));
        this.robotToCamera = robotToCamera(mount);
    }

    /**
     * The optical frame in the robot frame, built from {@link CameraMount#opticalToRobot} so this class and
     * {@link SimCamera} share one definition of where the camera looks.
     */
    private static RigidTransform3 robotToCamera(CameraMount mount) {
        double[] origin = mount.opticalToRobot(0.0, 0.0, 0.0);
        double[] x = minus(mount.opticalToRobot(1.0, 0.0, 0.0), origin);
        double[] y = minus(mount.opticalToRobot(0.0, 1.0, 0.0), origin);
        double[] z = minus(mount.opticalToRobot(0.0, 0.0, 1.0), origin);
        return RigidTransform3.fromAxes(x, y, z, origin[0], origin[1], origin[2]);
    }

    private static double[] minus(double[] a, double[] b) {
        return new double[] {a[0] - b[0], a[1] - b[1], a[2] - b[2]};
    }

    /**
     * Every tag in the layout the camera sees from this robot pose.
     *
     * @param layout          the tags on the field
     * @param robotXInches    robot centre, field X, inches
     * @param robotYInches    robot centre, field Y, inches
     * @param robotHeadingRad robot heading, radians CCW from field {@code +X}
     * @return the sightings, in ascending tag id order, unmodifiable; empty when nothing is seen
     */
    public List<TagSighting> observe(AprilTagLayout layout, double robotXInches, double robotYInches,
            double robotHeadingRad) {
        return observe(layout, robotXInches, robotYInches, robotHeadingRad, CameraOccluders.none());
    }

    /**
     * Every tag in the layout the camera sees from this robot pose, with more boxes in the way than the
     * ones given at construction: boxes that move from tick to tick, and the robot's own body.
     *
     * @param layout          the tags on the field
     * @param robotXInches    robot centre, field X, inches
     * @param robotYInches    robot centre, field Y, inches
     * @param robotHeadingRad robot heading, radians CCW from field {@code +X}
     * @param extra           more occluders, checked after the constructor's boxes; field boxes in the field
     *                        frame, robot boxes in the robot frame ({@link CameraOccluders})
     * @return the sightings, in ascending tag id order, unmodifiable; empty when nothing is seen
     */
    public List<TagSighting> observe(AprilTagLayout layout, double robotXInches, double robotYInches,
            double robotHeadingRad, CameraOccluders extra) {
        if (layout == null) {
            throw new IllegalArgumentException("layout must not be null; use AprilTagLayout.empty()");
        }
        if (extra == null) {
            throw new IllegalArgumentException("extra occluders must not be null; use CameraOccluders.none()");
        }
        RigidTransform3 fieldToRobot = fieldToRobot(robotXInches, robotYInches, robotHeadingRad);
        List<TagSighting> seen = new ArrayList<TagSighting>();
        for (AprilTagLayout.Tag tag : layout.getTags()) {
            TagSighting sighting = sight(tag, fieldToRobot, extra, null);
            if (sighting != null) {
                seen.add(sighting);
            }
        }
        return Collections.unmodifiableList(seen);
    }

    /**
     * Whether one tag is seen from this robot pose, and if not, the first rule it fails.
     *
     * @param tag             the tag
     * @param robotXInches    robot centre, field X, inches
     * @param robotYInches    robot centre, field Y, inches
     * @param robotHeadingRad robot heading, radians CCW from field {@code +X}
     * @return {@link Visibility#VISIBLE}, or the reason it is not
     */
    public Visibility visibility(AprilTagLayout.Tag tag, double robotXInches, double robotYInches,
            double robotHeadingRad) {
        return visibility(tag, robotXInches, robotYInches, robotHeadingRad, CameraOccluders.none());
    }

    /**
     * Whether one tag is seen from this robot pose with more occluders in the way, and if not, the first
     * rule it fails.
     *
     * @param tag             the tag
     * @param robotXInches    robot centre, field X, inches
     * @param robotYInches    robot centre, field Y, inches
     * @param robotHeadingRad robot heading, radians CCW from field {@code +X}
     * @param extra           more occluders, as for {@link #observe(AprilTagLayout, double, double, double,
     *                        CameraOccluders)}
     * @return {@link Visibility#VISIBLE}, or the reason it is not
     */
    public Visibility visibility(AprilTagLayout.Tag tag, double robotXInches, double robotYInches,
            double robotHeadingRad, CameraOccluders extra) {
        if (tag == null) {
            throw new IllegalArgumentException("tag must not be null");
        }
        if (extra == null) {
            throw new IllegalArgumentException("extra occluders must not be null; use CameraOccluders.none()");
        }
        Visibility[] reason = new Visibility[1];
        sight(tag, fieldToRobot(robotXInches, robotYInches, robotHeadingRad), extra, reason);
        return reason[0];
    }

    private static RigidTransform3 fieldToRobot(double x, double y, double heading) {
        requireFinite(x, "robotXInches");
        requireFinite(y, "robotYInches");
        requireFinite(heading, "robotHeadingRad");
        return RigidTransform3.planar(x, y, heading);
    }

    /** One tag through every rule; null when unseen, with the reason written to {@code reason} if given. */
    private TagSighting sight(AprilTagLayout.Tag tag, RigidTransform3 fieldToRobot, CameraOccluders extra,
            Visibility[] reason) {
        RigidTransform3 fieldToCamera = fieldToRobot.compose(robotToCamera);
        RigidTransform3 fieldToTag = tag.getFieldToTag();
        RigidTransform3 cameraToTag = fieldToCamera.inverse().compose(fieldToTag);
        double[] lensInField = fieldToCamera.apply(0.0, 0.0, 0.0);

        double range = cameraToTag.getDistanceInches();
        if (range < minRangeInches || range > maxRangeInches) {
            return reject(reason, Visibility.OUT_OF_RANGE);
        }

        // The face normal is the tag frame's -Z (target space +Z points into the tag). For a plumb tag it is
        // horizontal and its z component is exactly 0.
        double toLensX = lensInField[0] - tag.getXInches();
        double toLensY = lensInField[1] - tag.getYInches();
        double toLensZ = lensInField[2] - tag.getZInches();
        double[] normal = tag.getFaceNormal();
        double dot = toLensX * normal[0] + toLensY * normal[1];
        if (normal[2] != 0.0) {
            dot += toLensZ * normal[2];
        }
        double cosIncidence = range == 0.0 ? 0.0 : dot / range;
        if (cosIncidence <= cosMaxIncidence) {
            return reject(reason, Visibility.FACING_AWAY);
        }

        double half = tag.getSizeInches() / 2.0;
        double[][] cornersInTag = {{-half, -half}, {half, -half}, {half, half}, {-half, half}};
        double[][] cornersPx = new double[4][];
        double[][] cornersField = new double[4][];
        for (int i = 0; i < 4; i++) {
            double[] optical = cameraToTag.apply(cornersInTag[i][0], cornersInTag[i][1], 0.0);
            if (optical[2] <= 1e-9) {
                return reject(reason, Visibility.BEHIND_CAMERA);
            }
            cornersPx[i] = lens.project(optical[0], optical[1], optical[2]);
            cornersField[i] = fieldToTag.apply(cornersInTag[i][0], cornersInTag[i][1], 0.0);
        }
        for (double[] px : cornersPx) {
            if (px == null || px[0] < 0.0 || px[0] > lens.getWidthPx() || px[1] < 0.0 || px[1] > lens.getHeightPx()) {
                return reject(reason, Visibility.OUTSIDE_IMAGE);
            }
        }
        for (double[] corner : cornersField) {
            for (ObstacleBox box : occluders) {
                if (segmentEntersBox(lensInField, corner, box)) {
                    return reject(reason, Visibility.OCCLUDED);
                }
            }
            for (ObstacleBox box : extra.getFieldBoxes()) {
                if (segmentEntersBox(lensInField, corner, box)) {
                    return reject(reason, Visibility.OCCLUDED);
                }
            }
        }
        if (!extra.getRobotBoxes().isEmpty()) {
            RigidTransform3 robotFromField = fieldToRobot.inverse();
            double[] lensInRobot = robotToCamera.apply(0.0, 0.0, 0.0);
            for (double[] corner : cornersField) {
                double[] cornerInRobot = robotFromField.apply(corner[0], corner[1], corner[2]);
                for (ObstacleBox box : extra.getRobotBoxes()) {
                    if (CameraOccluders.contains(box, lensInRobot[0], lensInRobot[1], lensInRobot[2])) {
                        continue;                           // the camera's own housing
                    }
                    if (segmentEntersBox(lensInRobot, cornerInRobot, box)) {
                        return reject(reason, Visibility.OCCLUDED);
                    }
                }
            }
        }

        double x = cameraToTag.getXInches();
        double y = cameraToTag.getYInches();
        double z = cameraToTag.getZInches();
        double[] centrePx = lens.project(x, y, z);
        if (centrePx == null) {
            return reject(reason, Visibility.OUTSIDE_IMAGE);
        }
        double tx = StrictMath.toDegrees(StrictMath.atan2(x, z));
        double ty = StrictMath.toDegrees(StrictMath.atan2(-y, z));
        double area = 100.0 * polygonArea(cornersPx) / ((double) lens.getWidthPx() * lens.getHeightPx());
        if (reason != null) {
            reason[0] = Visibility.VISIBLE;
        }
        return new TagSighting(tag, cameraToTag, fieldToRobot.inverse().compose(fieldToTag), centrePx[0], centrePx[1],
                cornersPx, tx, ty, area);
    }

    private static TagSighting reject(Visibility[] reason, Visibility why) {
        if (reason != null) {
            reason[0] = why;
        }
        return null;
    }

    /** Shoelace area of a simple polygon, square pixels. */
    private static double polygonArea(double[][] points) {
        double twice = 0.0;
        for (int i = 0; i < points.length; i++) {
            double[] a = points[i];
            double[] b = points[(i + 1) % points.length];
            twice += a[0] * b[1] - b[0] * a[1];
        }
        return Math.abs(twice) / 2.0;
    }

    /**
     * Whether the segment from {@code from} to {@code to} passes through the box's inside, ignoring a
     * sliver at each end so a point lying on the box's surface does not count (slab method).
     */
    private static boolean segmentEntersBox(double[] from, double[] to, ObstacleBox box) {
        double[] min = {box.getMinXInches(), box.getMinYInches(), box.getMinZInches()};
        double[] max = {box.getMaxXInches(), box.getMaxYInches(), box.getMaxZInches()};
        double enter = 0.0;
        double exit = 1.0;
        for (int axis = 0; axis < 3; axis++) {
            double d = to[axis] - from[axis];
            if (Math.abs(d) < 1e-12) {
                if (from[axis] <= min[axis] || from[axis] >= max[axis]) {
                    return false;
                }
                continue;
            }
            double t0 = (min[axis] - from[axis]) / d;
            double t1 = (max[axis] - from[axis]) / d;
            enter = Math.max(enter, Math.min(t0, t1));
            exit = Math.min(exit, Math.max(t0, t1));
        }
        return enter < exit && enter < 1.0 - SEGMENT_END_MARGIN && exit > SEGMENT_END_MARGIN;
    }

    /** {@return the intrinsics} */
    public CameraLens getLens() {
        return lens;
    }

    /** {@return the extrinsics} */
    public CameraMount getMount() {
        return mount;
    }

    /** {@return the lens in the robot frame as a transform: the optical frame's pose, inches} */
    public RigidTransform3 getRobotToCamera() {
        return robotToCamera;
    }

    /** {@return the minimum range, lens to tag centre, inches} */
    public double getMinRangeInches() {
        return minRangeInches;
    }

    /** {@return the maximum range, lens to tag centre, inches; may be infinite} */
    public double getMaxRangeInches() {
        return maxRangeInches;
    }

    /** {@return the maximum angle off a tag's face normal at which it is still seen, degrees} */
    public double getMaxIncidenceDegrees() {
        return maxIncidenceDegrees;
    }

    /** {@return the boxes that block line of sight, unmodifiable} */
    public List<ObstacleBox> getOccluders() {
        return occluders;
    }

    private static void requireFinite(double value, String name) {
        if (!Double.isFinite(value)) {
            throw new IllegalArgumentException(name + " must be finite, got " + value);
        }
    }
}
