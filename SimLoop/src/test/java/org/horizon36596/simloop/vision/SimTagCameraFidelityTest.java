package org.horizon36596.simloop.vision;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Arrays;
import java.util.Collections;
import java.util.List;

import org.horizon36596.simloop.field.ObstacleBox;
import org.junit.jupiter.api.Test;

/**
 * INVARIANT-tier (domain R9) — the tag camera beyond plumb tags and a pinhole:
 *
 * <ol>
 *   <li>a tilted tag's frame is a rotation, its normal tilts with it, and tilt 0 is the plumb tag exactly;</li>
 *   <li>a tag lying face up is seen from above and not from below;</li>
 *   <li>the robot's own body hides a tag, and turns with the robot;</li>
 *   <li>a distorted lens moves the corner pixels and leaves the angles and transforms alone.</li>
 * </ol>
 *
 * <p>Tags, boxes and coefficients are invented for the test.
 */
class SimTagCameraFidelityTest {

    private static final CameraLens LENS = CameraLens.fromHorizontalFov(60.0, 640, 480);
    private static final CameraMount LEVEL = new CameraMount(0.0, 0.0, 10.0, 0.0, 0.0);
    private static final CameraMount DOWN = new CameraMount(0.0, 0.0, 20.0, 0.0, Math.toRadians(40.0));
    private static final List<ObstacleBox> NONE = Collections.emptyList();

    private static SimTagCamera camera(CameraLens lens, CameraMount mount) {
        return new SimTagCamera(lens, mount, 0.0, Double.POSITIVE_INFINITY, 90.0, NONE);
    }

    @Test
    void tiltZeroIsThePlumbTagExactly() {
        AprilTagLayout.Tag plumb = AprilTagLayout.builder().addTag(1, 50.0, 5.0, 8.0, 2.5, 6.0).build().getTag(1);
        AprilTagLayout.Tag zero = AprilTagLayout.builder()
                .addTiltedTag(1, 50.0, 5.0, 8.0, 2.5, 0.0, 6.0).build().getTag(1);
        for (int r = 0; r < 3; r++) {
            for (int c = 0; c < 3; c++) {
                assertEquals(plumb.getFieldToTag().rotation(r, c), zero.getFieldToTag().rotation(r, c), 0.0);
            }
        }
        assertEquals(0.0, plumb.getFaceNormal()[2], 0.0);
    }

    @Test
    void aTiltedTagsFrameIsARotationAndItsNormalTiltsWithIt() {
        double tilt = Math.toRadians(30.0);
        AprilTagLayout.Tag tag = AprilTagLayout.builder()
                .addTiltedTag(4, 60.0, 0.0, 20.0, Math.PI, tilt, 6.0).build().getTag(4);
        double[] n = tag.getFaceNormal();
        assertEquals(-Math.cos(tilt), n[0], 1e-12);
        assertEquals(0.0, n[1], 1e-12);
        assertEquals(Math.sin(tilt), n[2], 1e-12, "tilted up: the face looks up");
        // Viewer's right stays level.
        assertEquals(0.0, tag.getFieldToTag().rotation(2, 0), 1e-12);
        // Target +Z points into the tag: against the normal.
        assertEquals(-n[0], tag.getFieldToTag().rotation(0, 2), 1e-12);
        assertEquals(-n[2], tag.getFieldToTag().rotation(2, 2), 1e-12);
        assertThrows(IllegalArgumentException.class,
                () -> AprilTagLayout.builder().addTiltedTag(1, 0, 0, 0, 0, 2.0, 6.0));
    }

    @Test
    void aTagLyingFaceUpIsSeenFromAboveAndNotFromBelow() {
        AprilTagLayout layout = AprilTagLayout.builder()
                .addTiltedTag(2, 22.0, 0.0, 0.0, 0.0, Math.PI / 2.0, 6.0)
                .addTiltedTag(3, 22.0, 0.0, 0.0, 0.0, -Math.PI / 2.0, 6.0)
                .build();
        SimTagCamera down = camera(LENS, DOWN);
        assertEquals(SimTagCamera.Visibility.VISIBLE, down.visibility(layout.getTag(2), 0.0, 0.0, 0.0));
        assertEquals(SimTagCamera.Visibility.FACING_AWAY, down.visibility(layout.getTag(3), 0.0, 0.0, 0.0));
        // A plumb tag at the same place facing away from the camera's side would be seen edge-on or not at
        // all; the flat one is seen with its corners spread in both image axes.
        TagSighting seen = down.observe(layout, 0.0, 0.0, 0.0).get(0);
        double[][] corners = seen.getCornersPx();
        assertTrue(Math.abs(corners[0][1] - corners[3][1]) > 10.0, "a flat tag has height in the image");
    }

    @Test
    void theRobotsOwnBodyHidesATagAndTurnsWithTheRobot() {
        AprilTagLayout layout = AprilTagLayout.builder().addTag(7, 0.0, 60.0, 10.0, -Math.PI / 2.0, 6.0).build();
        ObstacleBox mast = new ObstacleBox("mast", 4.0, 5.0, -3.0, 3.0, 0.0, 30.0);
        CameraOccluders body = CameraOccluders.of(NONE, Arrays.asList(mast));
        SimTagCamera camera = camera(LENS, LEVEL);
        double facingTag = Math.PI / 2.0;                   // robot forward = field +Y, toward the tag
        assertEquals(SimTagCamera.Visibility.VISIBLE, camera.visibility(layout.getTag(7), 0.0, 0.0, facingTag));
        assertEquals(SimTagCamera.Visibility.OCCLUDED,
                camera.visibility(layout.getTag(7), 0.0, 0.0, facingTag, body));
        assertTrue(camera.observe(layout, 0.0, 0.0, facingTag, body).isEmpty());
        // The same box as a FIELD box stands at field x 4-5, off the line of sight.
        assertEquals(SimTagCamera.Visibility.VISIBLE, camera.visibility(layout.getTag(7), 0.0, 0.0, facingTag,
                CameraOccluders.field(Arrays.asList(mast))));
    }

    @Test
    void distortionMovesCornerPixelsAndLeavesAnglesAlone() {
        AprilTagLayout layout = AprilTagLayout.builder().addTag(1, 40.0, 14.0, 16.0, Math.PI, 6.0).build();
        CameraLens barrel = LENS.withDistortion(-0.25, 0.05, 0.0, 0.0, 0.0);
        TagSighting pinhole = camera(LENS, LEVEL).observe(layout, 0.0, 0.0, 0.0).get(0);
        TagSighting distorted = camera(barrel, LEVEL).observe(layout, 0.0, 0.0, 0.0).get(0);
        assertEquals(pinhole.getTxDegrees(), distorted.getTxDegrees(), 0.0);
        assertEquals(pinhole.getTyDegrees(), distorted.getTyDegrees(), 0.0);
        assertEquals(pinhole.getRangeInches(), distorted.getRangeInches(), 0.0);
        double[][] a = pinhole.getCornersPx();
        double[][] b = distorted.getCornersPx();
        for (int i = 0; i < 4; i++) {
            double pinholeOff = Math.hypot(a[i][0] - LENS.getCentreXPx(), a[i][1] - LENS.getCentreYPx());
            double barrelOff = Math.hypot(b[i][0] - LENS.getCentreXPx(), b[i][1] - LENS.getCentreYPx());
            assertTrue(barrelOff < pinholeOff - 1.0, "corner " + i + " pulled toward the centre");
        }
        // And each distorted corner is exactly the lens's projection of the corner's optical position.
        double[] optical = distorted.getCameraToTag().apply(-3.0, -3.0, 0.0);
        double[] expected = barrel.project(optical[0], optical[1], optical[2]);
        assertNotNull(expected);
        assertEquals(expected[0], b[0][0], 1e-9);
        assertEquals(expected[1], b[0][1], 1e-9);
    }
}
