package org.horizon36596.simloop.vision;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import org.horizon36596.simloop.field.ObstacleBox;
import org.horizon36596.simloop.vision.SimTagCamera.Visibility;
import org.junit.jupiter.api.Test;

/**
 * INVARIANT-tier (domain R9) — what the tag camera must get right for a fake Limelight to be trusted:
 *
 * <ol>
 *   <li>a tag is seen only when all four corners are in the image, in front of the lens, within range,
 *       facing the camera, and not behind an obstacle box — each rule tested at its edge;</li>
 *   <li>tx is positive to the right, ty positive up, and both equal the pinhole angles to the centre;</li>
 *   <li>the transforms agree with each other and with the robot pose, for any heading and mount yaw.</li>
 * </ol>
 *
 * <p>The rig is made up for this test: 60 x 45 degree field of view at 640 x 480, lens on the robot centre
 * 10 in up, looking straight forward. Every tag is a made-up 4 in tag; no real field geometry is used.
 */
class SimTagCameraTest {

    private static final double HFOV = 60.0;
    private static final double VFOV = 45.0;
    private static final CameraLens LENS = new CameraLens(
            320.0 / StrictMath.tan(StrictMath.toRadians(HFOV / 2)),
            240.0 / StrictMath.tan(StrictMath.toRadians(VFOV / 2)), 320.0, 240.0, 640, 480);
    private static final CameraMount LEVEL = new CameraMount(0.0, 0.0, 10.0, 0.0, 0.0);
    private static final double SIZE = 4.0;
    private static final double EPS = 1e-9;

    private static SimTagCamera camera() {
        return new SimTagCamera(LENS, LEVEL, 0.0, Double.POSITIVE_INFINITY, 90.0,
                Collections.<ObstacleBox>emptyList());
    }

    private static AprilTagLayout.Tag tag(double x, double y, double z, double facing) {
        return AprilTagLayout.builder().addTag(1, x, y, z, facing, SIZE).build().getTag(1);
    }

    /** Robot at the origin facing +X. */
    private static Visibility seenFromOrigin(SimTagCamera camera, AprilTagLayout.Tag tag) {
        return camera.visibility(tag, 0.0, 0.0, 0.0);
    }

    // ---------------------------------------------------------------- clause 1: visibility rules

    @Test
    void aTagStraightAheadFacingTheCameraIsSeen() {
        assertEquals(Visibility.VISIBLE, seenFromOrigin(camera(), tag(60, 0, 10, Math.PI)));
    }

    @Test
    void theHorizontalFieldOfViewEdgeIsWhereTheOuterCornerLeavesTheImage() {
        double edge = 60.0 * StrictMath.tan(StrictMath.toRadians(HFOV / 2));   // lateral offset of the edge
        // Right edge (field -Y is the robot's right): the outer corner sits 2 in beyond the centre.
        assertEquals(Visibility.VISIBLE, seenFromOrigin(camera(), tag(60, -(edge - 2.0 - 0.01), 10, Math.PI)));
        assertEquals(Visibility.OUTSIDE_IMAGE, seenFromOrigin(camera(), tag(60, -(edge - 2.0 + 0.01), 10, Math.PI)));
        // Left edge.
        assertEquals(Visibility.VISIBLE, seenFromOrigin(camera(), tag(60, edge - 2.0 - 0.01, 10, Math.PI)));
        assertEquals(Visibility.OUTSIDE_IMAGE, seenFromOrigin(camera(), tag(60, edge - 2.0 + 0.01, 10, Math.PI)));
        // A tag whose centre is in view but whose far corner is not is still not seen: a detector needs the
        // whole border.
        assertEquals(Visibility.OUTSIDE_IMAGE, seenFromOrigin(camera(), tag(60, edge - 1.0, 10, Math.PI)));
    }

    @Test
    void theVerticalFieldOfViewEdgeIsWhereTheTopOrBottomCornerLeavesTheImage() {
        double edge = 60.0 * StrictMath.tan(StrictMath.toRadians(VFOV / 2));
        assertEquals(Visibility.VISIBLE, seenFromOrigin(camera(), tag(60, 0, 10 + edge - 2.0 - 0.01, Math.PI)));
        assertEquals(Visibility.OUTSIDE_IMAGE, seenFromOrigin(camera(), tag(60, 0, 10 + edge - 2.0 + 0.01, Math.PI)));
        assertEquals(Visibility.VISIBLE, seenFromOrigin(camera(), tag(60, 0, 10 - edge + 2.0 + 0.01, Math.PI)));
        assertEquals(Visibility.OUTSIDE_IMAGE, seenFromOrigin(camera(), tag(60, 0, 10 - edge + 2.0 - 0.01, Math.PI)));
    }

    @Test
    void aTagBehindTheCameraIsNotSeenEvenWhenItFacesTheRobot() {
        assertEquals(Visibility.BEHIND_CAMERA, seenFromOrigin(camera(), tag(-60, 0, 10, 0.0)));
        // Straddling the lens plane: two corners in front, two behind.
        assertEquals(Visibility.BEHIND_CAMERA, seenFromOrigin(camera(), tag(0, 30, 10, -Math.PI / 2 - 0.3)));
    }

    @Test
    void aTagSeenFromBehindOrEdgeOnIsRejected() {
        assertEquals(Visibility.FACING_AWAY, seenFromOrigin(camera(), tag(60, 0, 10, 0.0)));        // back
        assertEquals(Visibility.FACING_AWAY, seenFromOrigin(camera(), tag(60, 0, 10, Math.PI / 2))); // edge-on
    }

    @Test
    void theIncidenceLimitRejectsTagsSeenTooObliquely() {
        SimTagCamera limited = new SimTagCamera(LENS, LEVEL, 0.0, Double.POSITIVE_INFINITY, 60.0,
                Collections.<ObstacleBox>emptyList());
        assertEquals(Visibility.VISIBLE, seenFromOrigin(limited, tag(60, 0, 10, Math.PI + StrictMath.toRadians(55))));
        assertEquals(Visibility.FACING_AWAY,
                seenFromOrigin(limited, tag(60, 0, 10, Math.PI + StrictMath.toRadians(65))));
        // The default limit (90) sees the same oblique tag.
        assertEquals(Visibility.VISIBLE, seenFromOrigin(camera(), tag(60, 0, 10, Math.PI + StrictMath.toRadians(65))));
    }

    @Test
    void theRangeCutoffAppliesAtBothEnds() {
        SimTagCamera ranged = new SimTagCamera(LENS, LEVEL, 24.0, 100.0, 90.0, Collections.<ObstacleBox>emptyList());
        assertEquals(Visibility.OUT_OF_RANGE, seenFromOrigin(ranged, tag(23.9, 0, 10, Math.PI)));
        assertEquals(Visibility.VISIBLE, seenFromOrigin(ranged, tag(24.1, 0, 10, Math.PI)));
        assertEquals(Visibility.VISIBLE, seenFromOrigin(ranged, tag(99.9, 0, 10, Math.PI)));
        assertEquals(Visibility.OUT_OF_RANGE, seenFromOrigin(ranged, tag(100.1, 0, 10, Math.PI)));
    }

    @Test
    void anObstacleBoxBetweenLensAndTagHidesIt_butNotTheBoxTheTagIsMountedOn() {
        List<ObstacleBox> boxes = new ArrayList<>();
        boxes.add(new ObstacleBox("backstop", 60, 64, -10, 10, 0, 30));    // the tag hangs on its -X face
        boxes.add(new ObstacleBox("beside", 20, 40, 20, 30, 0, 30));       // off the line of sight
        boxes.add(new ObstacleBox("low", 20, 40, -10, 10, 0, 5));          // under the line of sight
        SimTagCamera withBoxes = new SimTagCamera(LENS, LEVEL, 0.0, Double.POSITIVE_INFINITY, 90.0, boxes);
        assertEquals(Visibility.VISIBLE, seenFromOrigin(withBoxes, tag(60, 0, 10, Math.PI)));

        boxes.add(new ObstacleBox("wall", 30, 32, -10, 10, 0, 30));
        SimTagCamera blocked = new SimTagCamera(LENS, LEVEL, 0.0, Double.POSITIVE_INFINITY, 90.0, boxes);
        assertEquals(Visibility.OCCLUDED, seenFromOrigin(blocked, tag(60, 0, 10, Math.PI)));
        assertTrue(blocked.observe(layoutOf(tag(60, 0, 10, Math.PI)), 0, 0, 0).isEmpty());
    }

    @Test
    void anEmptyLayoutIsNeverSeen() {
        assertTrue(camera().observe(AprilTagLayout.empty(), 0, 0, 0).isEmpty());
    }

    @Test
    void severalTagsComeBackInIdOrderAndOnlyTheVisibleOnes() {
        AprilTagLayout layout = AprilTagLayout.builder()
                .addTag(30, 60, 10, 10, Math.PI, SIZE)
                .addTag(4, 80, -5, 10, Math.PI, SIZE)
                .addTag(12, -60, 0, 10, 0.0, SIZE)       // behind
                .addTag(7, 60, 0, 10, 0.0, SIZE)         // facing away
                .build();
        List<TagSighting> seen = camera().observe(layout, 0, 0, 0);
        assertEquals(2, seen.size());
        assertEquals(4, seen.get(0).getId());
        assertEquals(30, seen.get(1).getId());
    }

    // ---------------------------------------------------------------- clause 2: angles and pixels

    @Test
    void txIsPositiveToTheRightAndTyPositiveUp() {
        TagSighting right = only(tag(60, -10, 10, Math.PI));
        assertEquals(StrictMath.toDegrees(StrictMath.atan(10.0 / 60.0)), right.getTxDegrees(), EPS);
        assertEquals(0.0, right.getTyDegrees(), EPS);
        assertTrue(right.getCentreXPx() > 320.0);

        TagSighting upLeft = only(tag(60, 10, 18, Math.PI));
        assertTrue(upLeft.getTxDegrees() < 0.0);
        assertEquals(StrictMath.toDegrees(StrictMath.atan(8.0 / 60.0)), upLeft.getTyDegrees(), EPS);
        assertTrue(upLeft.getCentreYPx() < 240.0);
    }

    @Test
    void cornersAreTopLeftClockwiseAndAreaGrowsAsTheTagNears() {
        TagSighting far = only(tag(80, 0, 10, Math.PI));
        TagSighting near = only(tag(30, 0, 10, Math.PI));
        double[][] c = near.getCornersPx();
        assertTrue(c[0][0] < c[1][0] && c[0][1] < c[3][1], "top-left, top-right, bottom-right, bottom-left");
        assertEquals(c[1][0], c[2][0], EPS);
        assertTrue(near.getAreaPercent() > far.getAreaPercent());
        // Head-on, the image square's side is f * size / depth; area percent is its square over the image.
        double side = LENS.getFocalXPx() * SIZE / 30.0;
        double sideY = LENS.getFocalYPx() * SIZE / 30.0;
        assertEquals(100.0 * side * sideY / (640.0 * 480.0), near.getAreaPercent(), 1e-9);
    }

    // ---------------------------------------------------------------- clause 3: transforms

    @Test
    void transformsAgreeWithEachOtherForATurnedRobotAndAYawedMount() {
        CameraMount leftLooking = new CameraMount(3.0, 2.0, 9.0, Math.PI / 2, StrictMath.toRadians(-5));
        SimTagCamera cam = new SimTagCamera(LENS, leftLooking, 0.0, Double.POSITIVE_INFINITY, 90.0,
                Collections.<ObstacleBox>emptyList());
        // Robot at (10, -20) facing +Y; its camera looks left, i.e. toward field -X.
        double rx = 10.0;
        double ry = -20.0;
        double heading = Math.PI / 2;
        AprilTagLayout.Tag t = AprilTagLayout.builder().addTag(3, -50, -17, 14, 0.0, SIZE).build().getTag(3);
        List<TagSighting> seen = cam.observe(layoutOf(t), rx, ry, heading);
        assertEquals(1, seen.size());
        TagSighting s = seen.get(0);

        // robot->tag == robot->camera->tag
        RigidTransform3 viaCamera = cam.getRobotToCamera().compose(s.getCameraToTag());
        assertEquals(s.getRobotToTag().getXInches(), viaCamera.getXInches(), EPS);
        assertEquals(s.getRobotToTag().getYInches(), viaCamera.getYInches(), EPS);
        assertEquals(s.getRobotToTag().getZInches(), viaCamera.getZInches(), EPS);
        // field->robot->tag == the layout's field->tag
        RigidTransform3 fieldToTag = RigidTransform3.planar(rx, ry, heading).compose(s.getRobotToTag());
        assertEquals(-50.0, fieldToTag.getXInches(), EPS);
        assertEquals(-17.0, fieldToTag.getYInches(), EPS);
        assertEquals(14.0, fieldToTag.getZInches(), EPS);
        for (int row = 0; row < 3; row++) {
            for (int col = 0; col < 3; col++) {
                assertEquals(t.getFieldToTag().rotation(row, col), fieldToTag.rotation(row, col), EPS);
            }
        }
        // The lens is 3 in forward and 2 in left of the centre: on a robot facing +Y that is field
        // (10 - 2, -20 + 3, 9). The camera looks along field -X, and the tag is 58 in along that line and
        // 5 in higher, straight ahead of the lens, so tx is 0.
        assertEquals(0.0, s.getTxDegrees(), 1e-9);
        assertEquals(58.0 * 58.0 + 5.0 * 5.0, square(s.getRangeInches()), 1e-6);
    }

    @Test
    void aTagHeadOnHasTheIdentityRotationInCameraSpace() {
        TagSighting s = only(tag(60, 0, 10, Math.PI));
        RigidTransform3 t = s.getCameraToTag();
        assertEquals(60.0, t.getZInches(), EPS);
        assertEquals(0.0, t.getRollRad(), EPS);
        assertEquals(0.0, t.getPitchRad(), EPS);
        assertEquals(0.0, t.getYawRad(), EPS);
        // So the camera, in target space, sits on the tag's -Z axis: in front of the tag.
        assertEquals(-60.0, t.inverse().getZInches(), EPS);
    }

    @Test
    void theConstructorRejectsNonsenseLimits() {
        List<ObstacleBox> none = Collections.emptyList();
        assertThrows(IllegalArgumentException.class, () -> new SimTagCamera(LENS, LEVEL, -1.0, 10.0, 90.0, none));
        assertThrows(IllegalArgumentException.class, () -> new SimTagCamera(LENS, LEVEL, 10.0, 10.0, 90.0, none));
        assertThrows(IllegalArgumentException.class, () -> new SimTagCamera(LENS, LEVEL, 0.0, 10.0, 0.0, none));
        assertThrows(IllegalArgumentException.class, () -> new SimTagCamera(LENS, LEVEL, 0.0, 10.0, 91.0, none));
        assertThrows(IllegalArgumentException.class, () -> new SimTagCamera(null, LEVEL, 0.0, 10.0, 90.0, none));
    }

    private static TagSighting only(AprilTagLayout.Tag t) {
        List<TagSighting> seen = camera().observe(layoutOf(t), 0, 0, 0);
        assertEquals(1, seen.size(), "expected the tag to be seen: " + seenFromOrigin(camera(), t));
        return seen.get(0);
    }

    private static AprilTagLayout layoutOf(AprilTagLayout.Tag t) {
        return AprilTagLayout.builder().addTag(t.getId(), t.getXInches(), t.getYInches(), t.getZInches(),
                t.getFacingRad(), t.getSizeInches()).build();
    }

    private static double square(double v) {
        return v * v;
    }
}
