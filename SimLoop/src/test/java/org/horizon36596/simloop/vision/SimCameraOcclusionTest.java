package org.horizon36596.simloop.vision;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Arrays;
import java.util.Collections;
import java.util.List;

import org.horizon36596.simloop.field.FieldWorld;
import org.horizon36596.simloop.field.GamePieceTracker;
import org.horizon36596.simloop.field.ObstacleBox;
import org.junit.jupiter.api.Test;

/**
 * INVARIANT-tier (domain R9) — what {@link SimCamera} hides when solid things are in the way, and how
 * distortion moves its sightings:
 *
 * <ol>
 *   <li>with no occluders the occluded {@code observe} reports what the plain one does, fully visible;</li>
 *   <li>a field box across the line of sight hides a piece, half a box half-hides it, and the
 *       minimum visible fraction decides whether a half-hidden piece is reported;</li>
 *   <li>a robot box turns with the robot and hides what is behind it;</li>
 *   <li>a box holding the piece, or the lens, does not hide it;</li>
 *   <li>a nearer piece hides a farther one by how much of it it covers;</li>
 *   <li>with a distorted lens, a sighting's pixel goes back to the floor point it came from.</li>
 * </ol>
 *
 * <p>Rig as in {@link SimCameraTest}: 70 degrees at 640 x 400, 6 in forward, 10 in up, 22 degrees down. The
 * boxes are invented for the test.
 */
class SimCameraOcclusionTest {

    private static final CameraLens LENS = CameraLens.fromHorizontalFov(70.0, 640, 400);
    private static final CameraMount FRONT = CameraMount.front(6.0, 10.0, 22.0);
    private static final double RADIUS = 1.4;
    private static final List<ObstacleBox> NO_BOXES = Collections.emptyList();

    private static SimCamera camera() {
        return new SimCamera("front", LENS, FRONT);
    }

    private static GamePieceTracker piecesAt(double[]... fieldXY) {
        GamePieceTracker tracker = new GamePieceTracker(4, FieldWorld.unbounded());
        tracker.updateRobotPose(0.0, 0.0, 0.0);
        for (double[] xy : fieldXY) {
            tracker.addLoosePiece("piece", xy[0], xy[1], RADIUS);
        }
        return tracker;
    }

    @Test
    void noOccludersMatchesThePlainObserveAndIsFullyVisible() {
        GamePieceTracker tracker = piecesAt(new double[] {30.0, 0.0}, new double[] {26.0, 8.0});
        List<CameraSighting> plain = camera().observe(tracker.getPieces(), 0.0, 0.0, 0.0);
        List<CameraSighting> occluded = camera().observe(tracker.getPieces(), 0.0, 0.0, 0.0, CameraOccluders.none());
        assertEquals(plain.size(), occluded.size());
        for (int i = 0; i < plain.size(); i++) {
            assertEquals(plain.get(i).getPieceId(), occluded.get(i).getPieceId());
            assertEquals(plain.get(i).getCentreXPx(), occluded.get(i).getCentreXPx(), 0.0);
            assertEquals(plain.get(i).getCentreYPx(), occluded.get(i).getCentreYPx(), 0.0);
            assertEquals(plain.get(i).getRadiusPx(), occluded.get(i).getRadiusPx(), 0.0);
            assertEquals(1.0, occluded.get(i).getVisibleFraction(), 0.0);
            assertEquals(1.0, plain.get(i).getVisibleFraction(), 0.0);
        }
    }

    @Test
    void aFieldBoxAcrossTheLineOfSightHidesThePiece() {
        GamePieceTracker tracker = piecesAt(new double[] {30.0, 0.0});
        ObstacleBox wall = new ObstacleBox("wall", 20.0, 21.0, -6.0, 6.0, 0.0, 12.0);
        assertEquals(1, camera().observe(tracker.getPieces(), 0.0, 0.0, 0.0).size(),
                "the plain observe does not know about boxes");
        assertTrue(camera().observe(tracker.getPieces(), 0.0, 0.0, 0.0,
                CameraOccluders.field(Arrays.asList(wall))).isEmpty());
    }

    @Test
    void halfABoxHalfHidesThePieceAndTheThresholdDecides() {
        GamePieceTracker tracker = piecesAt(new double[] {30.0, 0.0});
        // Covers field +Y (the robot's left, the image's left) from the piece's centre line outward.
        ObstacleBox half = new ObstacleBox("half", 20.0, 21.0, 0.0, 6.0, 0.0, 12.0);
        CameraOccluders boxes = CameraOccluders.field(Arrays.asList(half));
        List<CameraSighting> lenient = camera().withMinVisibleFraction(0.25)
                .observe(tracker.getPieces(), 0.0, 0.0, 0.0, boxes);
        assertEquals(1, lenient.size());
        double fraction = lenient.get(0).getVisibleFraction();
        assertTrue(fraction > 0.3 && fraction < 0.7, "about half visible, got " + fraction);
        assertTrue(camera().withMinVisibleFraction(0.9).observe(tracker.getPieces(), 0.0, 0.0, 0.0, boxes).isEmpty());
    }

    @Test
    void aRobotBoxTurnsWithTheRobot() {
        // A bumper 9-10 in ahead of the robot centre, 12 in tall: in front of the 6 in-forward lens.
        ObstacleBox bumper = new ObstacleBox("bumper", 9.0, 10.0, -8.0, 8.0, 0.0, 12.0);
        CameraOccluders body = CameraOccluders.of(NO_BOXES, Arrays.asList(bumper));
        // Robot facing +Y: its forward is field +Y, so the piece 30 in "ahead" is at field (0, 30).
        GamePieceTracker tracker = piecesAt(new double[] {0.0, 30.0});
        double heading = Math.PI / 2.0;
        assertEquals(1, camera().observe(tracker.getPieces(), 0.0, 0.0, heading, CameraOccluders.none()).size());
        assertTrue(camera().observe(tracker.getPieces(), 0.0, 0.0, heading, body).isEmpty());
        // The same box read in the FIELD frame sits at field x 9-10, nowhere near the line of sight.
        assertEquals(1, camera().observe(tracker.getPieces(), 0.0, 0.0, heading,
                CameraOccluders.field(Arrays.asList(bumper))).size());
    }

    @Test
    void aBoxHoldingThePieceOrTheLensDoesNotHideIt() {
        GamePieceTracker tracker = piecesAt(new double[] {30.0, 0.0});
        ObstacleBox tray = new ObstacleBox("tray", 27.0, 33.0, -3.0, 3.0, 0.0, 4.0);
        ObstacleBox housing = new ObstacleBox("housing", 5.0, 7.0, -1.0, 1.0, 9.0, 11.0);
        assertEquals(1, camera().observe(tracker.getPieces(), 0.0, 0.0, 0.0,
                CameraOccluders.of(Arrays.asList(tray), Arrays.asList(housing))).size());
    }

    @Test
    void aNearerPieceHidesAFartherOneByHowMuchItCovers() {
        // A low, level camera (2 in up), so a nearer floor piece can stand in front of a farther one.
        SimCamera low = new SimCamera("low", LENS, CameraMount.front(6.0, 2.0, 0.0));
        // Straight behind: the near piece covers the far one's whole disc as the camera sees it.
        GamePieceTracker behind = piecesAt(new double[] {24.0, 0.0}, new double[] {36.0, 0.0});
        List<CameraSighting> seen = low.observe(behind.getPieces(), 0.0, 0.0, 0.0, CameraOccluders.none());
        assertEquals(1, seen.size());
        assertEquals(24.0, nearestPieceX(behind, seen.get(0).getPieceId()), 1e-9);
        // Offset sideways by a radius and a bit: partly covered, reported with less than full visibility.
        GamePieceTracker offset = piecesAt(new double[] {24.0, 0.0}, new double[] {36.0, 2.2});
        List<CameraSighting> both = low.withMinVisibleFraction(0.05)
                .observe(offset.getPieces(), 0.0, 0.0, 0.0, CameraOccluders.none());
        assertEquals(2, both.size());
        double far = Math.min(both.get(0).getVisibleFraction(), both.get(1).getVisibleFraction());
        assertTrue(far > 0.0 && far < 1.0, "the far piece is partly hidden, got " + far);
    }

    private static double nearestPieceX(GamePieceTracker tracker, int id) {
        for (org.horizon36596.simloop.field.GamePiece piece : tracker.getPieces()) {
            if (piece.getId() == id) {
                return piece.getFieldXInches();
            }
        }
        throw new AssertionError("no piece " + id);
    }

    @Test
    void withADistortedLensASightingGoesBackToItsFloorPoint() {
        CameraLens barrel = LENS.withDistortion(-0.28, 0.07, 0.0005, -0.0004, -0.008);
        SimCamera distorted = new SimCamera("front", barrel, FRONT);
        GamePieceTracker tracker = piecesAt(new double[] {28.0, 14.0}, new double[] {40.0, -16.0},
                new double[] {22.0, 0.0});
        List<CameraSighting> pinholeSeen = camera().observe(tracker.getPieces(), 0.0, 0.0, 0.0);
        List<CameraSighting> seen = distorted.observe(tracker.getPieces(), 0.0, 0.0, 0.0);
        assertEquals(3, seen.size());
        for (CameraSighting s : seen) {
            RobotFramePoint p = distorted.groundPointInRobotFrame(s.getCentreXPx(), s.getCentreYPx(), RADIUS);
            boolean matched = false;
            for (org.horizon36596.simloop.field.GamePiece piece : tracker.getPieces()) {
                if (piece.getId() == s.getPieceId()) {
                    assertEquals(piece.getFieldXInches(), p.forwardInches, 1e-6);
                    assertEquals(piece.getFieldYInches(), p.leftInches, 1e-6);
                    matched = true;
                }
            }
            assertTrue(matched);
        }
        // The off-axis pieces move toward the image centre under barrel distortion.
        for (int i = 0; i < 2; i++) {
            double pinholeOff = Math.abs(pinholeSeen.get(i).getCentreXPx() - LENS.getCentreXPx());
            double barrelOff = Math.abs(seen.get(i).getCentreXPx() - LENS.getCentreXPx());
            assertTrue(barrelOff < pinholeOff, "piece " + i + ": " + barrelOff + " vs " + pinholeOff);
        }
    }

    @Test
    void nullOccludersAndABadThresholdAreRejected() {
        GamePieceTracker tracker = piecesAt(new double[] {30.0, 0.0});
        assertThrows(IllegalArgumentException.class,
                () -> camera().observe(tracker.getPieces(), 0.0, 0.0, 0.0, null));
        assertThrows(IllegalArgumentException.class, () -> camera().withMinVisibleFraction(0.0));
        assertThrows(IllegalArgumentException.class, () -> camera().withMinVisibleFraction(1.5));
    }
}
