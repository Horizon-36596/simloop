package org.horizon36596.simloop.vision;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Map;

import org.horizon36596.simloop.config.DrivetrainSimConfig;
import org.horizon36596.simloop.config.SimRobotConfig;
import org.horizon36596.simloop.field.FieldWorld;
import org.horizon36596.simloop.field.GamePiece;
import org.horizon36596.simloop.field.GamePieceTracker;
import org.horizon36596.simloop.sim.FakeTimer;
import org.horizon36596.simloop.sim.RlogDecodedCompare;
import org.horizon36596.simloop.sim.ScenarioRunner;
import org.junit.jupiter.api.Test;
import org.psilynx.psikit.core.wpi.math.Pose3d;

/**
 * INVARIANT-tier (domain R9) — what a sim camera must get right for a vision pipeline's geometry to be
 * testable against it:
 *
 * <ol>
 *   <li>the mount's two frame conversions are exact inverses, and the frame conventions are the ones
 *       {@link CameraMount} documents: +forward/+left/+up, yaw CCW, pitch positive down, OpenCV optical
 *       axes;</li>
 *   <li>a point on the boresight lands on the principal point, left in the world is left in the image,
 *       nearer floor points are lower in the image;</li>
 *   <li>a pixel projected back onto the plane it came from gives the point it came from — the
 *       ground-plane ray a ball detector relies on;</li>
 *   <li>only visible pieces are sighted: not held, not behind the lens, not outside the frame, not hidden
 *       behind a nearer piece; a piece at the edge is flagged clipped;</li>
 *   <li>the logged camera pose is field-relative in metres with WPILib's camera axes, and
 *       {@link SimCamera#record}'s arrays are index-aligned with the sightings.</li>
 * </ol>
 *
 * <p>The rig: 70 degree horizontal FOV at 640x400, mounted 6 in forward, 10 in high, 22 degrees down.
 */
class SimCameraTest {

    private static final CameraLens LENS = CameraLens.fromHorizontalFov(70.0, 640, 400);
    private static final CameraMount FRONT = CameraMount.front(6.0, 10.0, 22.0);
    private static final double RADIUS = 1.4;

    private static SimCamera camera() {
        return new SimCamera("front", LENS, FRONT);
    }

    /** A tracker holding the given loose pieces, robot at the origin facing +X. */
    private static GamePieceTracker piecesAt(double[][] fieldXY) {
        GamePieceTracker tracker = new GamePieceTracker(4, FieldWorld.unbounded());
        tracker.updateRobotPose(0.0, 0.0, 0.0);
        for (double[] xy : fieldXY) {
            tracker.addLoosePiece("piece", xy[0], xy[1], RADIUS);
        }
        return tracker;
    }

    // ---------------------------------------------------------------- clause 1: the mount

    @Test
    void robotToOpticalAndBackIsTheIdentityForAnyMount() {
        CameraMount[] mounts = {
                FRONT,
                new CameraMount(-5.0, 3.0, 12.0, Math.PI, Math.toRadians(30.0)),
                new CameraMount(2.0, -7.0, 8.0, Math.toRadians(-40.0), Math.toRadians(-10.0)),
        };
        double[][] points = {{30.0, 5.0, 1.4}, {-12.0, 20.0, 0.0}, {3.0, -3.0, 25.0}, {100.0, 0.0, 10.0}};
        for (CameraMount mount : mounts) {
            for (double[] p : points) {
                double[] optical = mount.robotToOptical(p[0], p[1], p[2]);
                double[] back = mount.opticalToRobot(optical[0], optical[1], optical[2]);
                assertEquals(p[0], back[0], 1e-9, mount + " forward");
                assertEquals(p[1], back[1], 1e-9, mount + " left");
                assertEquals(p[2], back[2], 1e-9, mount + " up");
            }
        }
    }

    @Test
    void theOpticalAxesFollowTheDocumentedConventions() {
        // Level, forward-facing camera at the origin: robot forward is optical +z, robot left is optical
        // -x (image right is robot right), robot up is optical -y (image down is robot down).
        CameraMount level = new CameraMount(0.0, 0.0, 1.0, 0.0, 0.0);
        double[] ahead = level.robotToOptical(10.0, 0.0, 1.0);
        assertEquals(10.0, ahead[2], 1e-12, "forward is depth");
        double[] left = level.robotToOptical(0.0, 10.0, 1.0);
        assertEquals(-10.0, left[0], 1e-12, "robot left is image left (negative x)");
        double[] up = level.robotToOptical(0.0, 0.0, 11.0);
        assertEquals(-10.0, up[1], 1e-12, "robot up is image up (negative y)");

        // Pitched down 90-ish: a point below the lens moves toward the boresight.
        CameraMount down = new CameraMount(0.0, 0.0, 10.0, 0.0, Math.toRadians(45.0));
        double[] floorAhead = down.robotToOptical(10.0, 0.0, 0.0);   // 10 fwd, 10 down: exactly on the boresight
        assertEquals(0.0, floorAhead[0], 1e-9);
        assertEquals(0.0, floorAhead[1], 1e-9);
        assertEquals(Math.sqrt(200.0), floorAhead[2], 1e-9, "range along the boresight");
    }

    // ---------------------------------------------------------------- clause 2: projection

    @Test
    void aPieceOnTheBoresightLandsOnThePrincipalPointAndLeftIsLeft() {
        // Boresight of FRONT pitched 22 deg down from 10 in high meets height 1.4 at
        // 6 + (10 - 1.4) / tan(22 deg) = 27.29 in ahead.
        double boresightFloorX = 6.0 + (10.0 - RADIUS) / Math.tan(Math.toRadians(22.0));
        GamePieceTracker tracker = piecesAt(new double[][] {
                {boresightFloorX, 0.0},          // id 0: dead centre
                {boresightFloorX, 8.0},          // id 1: to the robot's left (+Y with heading 0)
                {boresightFloorX - 10.0, 0.0},   // id 2: nearer
        });
        List<CameraSighting> seen = camera().observe(tracker.getPieces(), 0.0, 0.0, 0.0);
        assertEquals(3, seen.size());
        assertEquals(320.0, seen.get(0).getCentreXPx(), 1e-6);
        assertEquals(200.0, seen.get(0).getCentreYPx(), 1e-6);
        assertTrue(seen.get(1).getCentreXPx() < 320.0, "left in the world is left in the image");
        assertTrue(seen.get(2).getCentreYPx() > 200.0, "nearer on the floor is lower in the image");
        assertTrue(seen.get(2).getRadiusPx() > seen.get(0).getRadiusPx(), "and looks bigger");
        assertEquals("piece", seen.get(0).getKind());
        assertEquals(0, seen.get(0).getPieceId());
    }

    @Test
    void theRobotHeadingIsAccountedForInTheFieldFrame() {
        // Robot at (10, -20) facing +Y: a piece at field (10, -20 + d) is straight ahead.
        double d = 6.0 + (10.0 - RADIUS) / Math.tan(Math.toRadians(22.0));
        GamePieceTracker tracker = new GamePieceTracker(4, FieldWorld.unbounded());
        tracker.updateRobotPose(10.0, -20.0, Math.PI / 2.0);
        tracker.addLoosePiece("piece", 10.0, -20.0 + d, RADIUS);
        List<CameraSighting> seen = camera().observe(tracker.getPieces(), 10.0, -20.0, Math.PI / 2.0);
        assertEquals(1, seen.size());
        assertEquals(320.0, seen.get(0).getCentreXPx(), 1e-6);
        assertEquals(200.0, seen.get(0).getCentreYPx(), 1e-6);
    }

    // ---------------------------------------------------------------- clause 3: the ground ray

    @Test
    void aSightingProjectedBackOntoTheBallCentrePlaneRecoversTheBallPosition() {
        double[][] truth = {{30.0, 5.0}, {48.0, -12.0}, {20.0, 0.5}, {84.0, 20.0}};
        GamePieceTracker tracker = piecesAt(truth);
        SimCamera camera = camera();
        List<CameraSighting> seen = camera.observe(tracker.getPieces(), 0.0, 0.0, 0.0);
        assertEquals(truth.length, seen.size(), "all in frame");
        for (CameraSighting sighting : seen) {
            RobotFramePoint onPlane = camera.groundPointInRobotFrame(sighting.getCentreXPx(),
                    sighting.getCentreYPx(), RADIUS);
            assertNotNull(onPlane);
            double[] expected = truth[sighting.getPieceId()];
            assertEquals(expected[0], onPlane.forwardInches, 1e-6, "forward, piece " + sighting.getPieceId());
            assertEquals(expected[1], onPlane.leftInches, 1e-6, "left, piece " + sighting.getPieceId());
        }
    }

    @Test
    void solvingTheGroundRayAtTheFloorInsteadOfTheBallCentreOverRanges() {
        // The classic ground-plane mistake, made concrete: a ball centre 1.4 in up, projected to z = 0,
        // lands too far away by d * r / (h - r).
        GamePieceTracker tracker = piecesAt(new double[][] {{48.0, 0.0}});
        SimCamera camera = camera();
        CameraSighting sighting = camera.observe(tracker.getPieces(), 0.0, 0.0, 0.0).get(0);
        RobotFramePoint wrong = camera.groundPointInRobotFrame(sighting.getCentreXPx(), sighting.getCentreYPx(), 0.0);
        double horizontalFromLens = 48.0 - 6.0;
        double expectedOverRange = horizontalFromLens * RADIUS / (10.0 - RADIUS);
        assertEquals(48.0 + expectedOverRange, wrong.forwardInches, 1e-6);
    }

    @Test
    void aPixelAtOrAboveTheHorizonHasNoGroundPoint() {
        SimCamera camera = camera();
        // The horizon for a 22 deg down camera is 22 deg above the boresight: fy * tan(22 deg) above centre.
        double horizonY = 200.0 - LENS.getFocalYPx() * Math.tan(Math.toRadians(22.0));
        assertNull(camera.groundPointInRobotFrame(320.0, horizonY - 1.0, 0.0), "above the horizon");
        assertNotNull(camera.groundPointInRobotFrame(320.0, horizonY + 5.0, 0.0), "just below it");
    }

    // ---------------------------------------------------------------- clause 4: visibility

    @Test
    void heldPiecesPiecesBehindTheLensAndPiecesOutOfFrameAreNotSighted() {
        GamePieceTracker tracker = piecesAt(new double[][] {
                {30.0, 0.0},      // id 0: seen
                {-30.0, 0.0},     // id 1: behind the robot
                {30.0, 40.0},     // id 2: far to the left, outside a 70 deg FOV at 30 in
        });
        tracker.addLoosePiece("piece", 30.0, 2.0, RADIUS);   // id 3: about to be held
        // A 1 in mouth right on piece 3: piece 0, 2 in away with a 1.4 in radius, is 0.1 in clear of it.
        tracker.tryAcquire(new org.horizon36596.simloop.field.TriggerVolume("intake", 30.0, 2.0, 1.0, 1.0),
                true);
        assertTrue(tracker.getPiece(3).isHeld());

        List<CameraSighting> seen = camera().observe(tracker.getPieces(), 0.0, 0.0, 0.0);
        assertEquals(1, seen.size(), seen.toString());
        assertEquals(0, seen.get(0).getPieceId());
        assertFalse(seen.get(0).isClipped());
    }

    @Test
    void aNearerPieceHidesOneDirectlyBehindItAndAnEdgePieceIsClipped() {
        // Two pieces on the same floor line are NOT an occlusion: the farther one sits higher in the
        // image, outside the near one's disc, so both are seen.
        GamePieceTracker inLine = piecesAt(new double[][] {{30.0, 0.0}, {70.0, 0.0}});
        List<CameraSighting> inLineSeen = camera().observe(inLine.getPieces(), 0.0, 0.0, 0.0);
        assertEquals(2, inLineSeen.size(), "a farther piece higher in the frame is not occluded: " + inLineSeen);

        // A real occlusion: a piece parked in mid-air halfway along the ray from the lens to a floor
        // piece. Its centre pixel is the floor piece's centre pixel and it is nearer, so it hides it.
        double[] farOptical = FRONT.robotToOptical(50.0, 0.0, RADIUS);
        double[] halfway = FRONT.opticalToRobot(farOptical[0] / 2.0, farOptical[1] / 2.0, farOptical[2] / 2.0);
        assertTrue(halfway[2] > RADIUS, "sanity: halfway to the floor piece is above the floor");
        GamePieceTracker stacked = new GamePieceTracker(4, FieldWorld.builder()
                .scoringVolume(org.horizon36596.simloop.field.ScoringVolume.builder("goal")
                        .box(-1, 1, -1, 1, 0, 1).restPoint(halfway[0], halfway[1], halfway[2]).build())
                .build());
        stacked.updateRobotPose(0.0, 0.0, 0.0);
        stacked.addLoosePiece("piece", 50.0, 0.0, RADIUS);                            // id 0: on the floor
        stacked.addScoredPiece("piece", RADIUS, "goal");                             // id 1: on the ray, nearer
        List<CameraSighting> stackedSeen = camera().observe(stacked.getPieces(), 0.0, 0.0, 0.0);
        assertEquals(1, stackedSeen.size(), "the floor piece is hidden: " + stackedSeen);
        assertEquals(1, stackedSeen.get(0).getPieceId(), "and the near one is what is seen");

        // Clipping: a piece at the bottom edge of the frame.
        RobotFramePoint bottomEdge = camera().groundPointInRobotFrame(320.0, 399.0, RADIUS);
        GamePieceTracker edge = piecesAt(new double[][] {{bottomEdge.forwardInches, bottomEdge.leftInches}});
        List<CameraSighting> edgeSeen = camera().observe(edge.getPieces(), 0.0, 0.0, 0.0);
        assertEquals(1, edgeSeen.size());
        assertTrue(edgeSeen.get(0).isClipped(), "its disc runs off the bottom of the image");
    }

    @Test
    void sightingsComeBackInPieceIdOrderWhateverOrderThePiecesAreHandedIn() {
        GamePieceTracker tracker = piecesAt(new double[][] {{30.0, 0.0}, {40.0, 6.0}, {50.0, -6.0}});
        List<GamePiece> reversed = new ArrayList<>(tracker.getPieces());
        Collections.reverse(reversed);

        List<CameraSighting> seen = camera().observe(reversed, 0.0, 0.0, 0.0);

        assertEquals(3, seen.size(), seen.toString());
        assertEquals(0, seen.get(0).getPieceId());
        assertEquals(1, seen.get(1).getPieceId());
        assertEquals(2, seen.get(2).getPieceId());
    }

    @Test
    void aMountWithYawAndASideOffsetStillPutsItsBoresightOnThePrincipalPoint() {
        // Mounted 4 in forward, 5 in LEFT of centre, 12 in up, turned 30 deg to the left and tilted 25 deg
        // down. Its boresight meets the ball-centre plane (height RADIUS) this far out from the lens:
        double yaw = Math.toRadians(30.0);
        double pitch = Math.toRadians(25.0);
        CameraMount mount = new CameraMount(4.0, 5.0, 12.0, yaw, pitch);
        double reach = (12.0 - RADIUS) / Math.tan(pitch);
        double onBoresightForward = 4.0 + reach * Math.cos(yaw);
        double onBoresightLeft = 5.0 + reach * Math.sin(yaw);
        // A second piece further to the robot's left, which a camera turned left must see LEFT of centre.
        double leftOfBoresightForward = onBoresightForward - 6.0 * Math.sin(yaw);
        double leftOfBoresightLeft = onBoresightLeft + 6.0 * Math.cos(yaw);
        GamePieceTracker tracker = piecesAt(new double[][] {
                {onBoresightForward, onBoresightLeft},
                {leftOfBoresightForward, leftOfBoresightLeft},
        });

        List<CameraSighting> seen = new SimCamera("side", LENS, mount).observe(tracker.getPieces(), 0.0, 0.0,
                0.0);

        assertEquals(2, seen.size(), seen.toString());
        assertEquals(320.0, seen.get(0).getCentreXPx(), 1e-6, "on the boresight: principal point x");
        assertEquals(200.0, seen.get(0).getCentreYPx(), 1e-6, "on the boresight: principal point y");
        assertTrue(seen.get(1).getCentreXPx() < 320.0, "left of the boresight is left in the image");
    }

    @Test
    void aDiscJustOffACornerOfTheImageIsNotSighted() {
        // Place a piece whose centre pixel sits diagonally off the bottom-left corner by k pixels in x and y,
        // with k between r / sqrt(2) and r. Its bounding square overlaps the image; the disc itself does not
        // reach the corner. The radius depends on where the piece ends up, so settle it in two passes.
        SimCamera camera = camera();
        double k = 20.0;
        double radiusPx = 0.0;
        GamePieceTracker tracker = null;
        for (int pass = 0; pass < 3; pass++) {
            RobotFramePoint spot = camera.groundPointInRobotFrame(-k, 400.0 + k, RADIUS);
            tracker = piecesAt(new double[][] {{spot.forwardInches, spot.leftInches}});
            radiusPx = projectedRadiusPx(tracker);
            k = 0.85 * radiusPx;
        }
        RobotFramePoint spot = camera.groundPointInRobotFrame(-k, 400.0 + k, RADIUS);
        tracker = piecesAt(new double[][] {{spot.forwardInches, spot.leftInches}});
        radiusPx = projectedRadiusPx(tracker);
        assertTrue(k < radiusPx && k * Math.sqrt(2.0) > radiusPx,
                "sanity: the bounding square overlaps the image but the disc does not; k=" + k + " r=" + radiusPx);

        assertTrue(camera.observe(tracker.getPieces(), 0.0, 0.0, 0.0).isEmpty(),
                "a disc that misses the image rectangle is not sighted, even though its bounding square hits it");
    }

    /** The apparent radius the camera would give the tracker's one piece, computed the way it computes it. */
    private static double projectedRadiusPx(GamePieceTracker tracker) {
        GamePiece piece = tracker.getPiece(0);
        double[] optical = FRONT.robotToOptical(piece.getFieldXInches(), piece.getFieldYInches(),
                piece.getFieldZInches());
        double range = Math.sqrt(optical[0] * optical[0] + optical[1] * optical[1] + optical[2] * optical[2]);
        return LENS.getFocalXPx() * RADIUS / range;
    }

    // ---------------------------------------------------------------- clause 5: the logged pose

    @Test
    void theFieldPoseIsInMetresWithTheCameraAxesAdvantageScopeExpects() {
        SimCamera camera = camera();
        // Robot at (20, -40) in, facing +Y. Camera 6 in ahead of it => at (20, -34) in, 10 in up.
        Pose3d pose = camera.fieldPose3d(20.0, -40.0, Math.PI / 2.0);
        assertEquals(20.0 * 0.0254, pose.getX(), 1e-12);
        assertEquals(-34.0 * 0.0254, pose.getY(), 1e-12);
        assertEquals(10.0 * 0.0254, pose.getZ(), 1e-12);
        assertEquals(Math.toRadians(22.0), pose.getRotation().getY(), 1e-9, "WPI pitch = tilt down");
        assertEquals(Math.PI / 2.0, pose.getRotation().getZ(), 1e-9, "WPI yaw = heading + mount yaw");
        assertEquals(0.0, pose.getRotation().getX(), 1e-9, "no roll");
    }

    @Test
    void recordBeforeObserveIsRefused() {
        GamePieceTracker tracker = piecesAt(new double[][] {{30.0, 0.0}});
        assertThrows(IllegalStateException.class, () -> camera().record("Vision", tracker.getPieces()));
    }

    @Test
    void aCameraNameThatIsNotOneCleanKeySegmentIsRefused() {
        for (String bad : new String[] {"", " front", "front ", "fr ont", "front/left", "front\tcam", "front.cam"}) {
            assertThrows(IllegalArgumentException.class, () -> new SimCamera(bad, LENS, FRONT), "\"" + bad + "\"");
        }
        assertEquals("front_left-2", new SimCamera("front_left-2", LENS, FRONT).getName());
    }

    @Test
    void recordLogsIndexAlignedArraysAndTruePositionsInMetres() throws IOException {
        GamePieceTracker tracker = piecesAt(new double[][] {{30.0, 5.0}, {-30.0, 0.0}, {48.0, -12.0}});
        SimCamera camera = camera();
        Path rlog = Paths.get("build", "sim", "sim-camera-record-test.rlog");

        ScenarioRunner.run("SimCameraRecord", STUB_CONFIG, new FakeTimer(), 1, 0.02, rlog, deltaTimeSeconds -> {
            camera.observe(tracker.getPieces(), 0.0, 0.0, 0.0);
            camera.record("Vision", tracker.getPieces());
        });
        Map<String, String> frame = RlogDecodedCompare.decode(rlog).get(0);

        // Piece 1 is behind the robot, so the sightings are pieces 0 and 2, in that order, in every array.
        assertEquals("2", frame.get("RealOutputs/Vision/front/sightingCount"));
        assertEquals("[0, 2]", frame.get("RealOutputs/Vision/front/sightings/pieceIds"));
        assertEquals("[piece, piece]", frame.get("RealOutputs/Vision/front/sightings/kinds"));
        List<CameraSighting> seen = camera.getLastSightings();
        assertEquals("[" + seen.get(0).getRangeInches() + ", " + seen.get(1).getRangeInches() + "]",
                frame.get("RealOutputs/Vision/front/sightings/rangeInches"));
        assertEquals("[" + seen.get(0).getCentreXPx() + ", " + seen.get(1).getCentreXPx() + "]",
                frame.get("RealOutputs/Vision/front/sightings/centreXPx"));

        // seenPositions3d is a WPILib Translation3d struct array: three little-endian doubles per entry,
        // in metres, in the same order as the other arrays.
        double[] metres = littleEndianDoubles(frame.get("RealOutputs/Vision/front/seenPositions3d"));
        assertEquals(6, metres.length, "two Translation3d entries");
        assertEquals(30.0 * 0.0254, metres[0], 1e-12, "entry 0 is piece 0's x");
        assertEquals(5.0 * 0.0254, metres[1], 1e-12, "entry 0 is piece 0's y");
        assertEquals(RADIUS * 0.0254, metres[2], 1e-12, "entry 0 is piece 0's z");
        assertEquals(48.0 * 0.0254, metres[3], 1e-12, "entry 1 is piece 2's x");
        assertEquals(-12.0 * 0.0254, metres[4], 1e-12, "entry 1 is piece 2's y");
    }

    /** Parses PsiKit's rendering of a raw value ({@code "[1, -2, ...]"}) as little-endian doubles. */
    private static double[] littleEndianDoubles(String renderedBytes) {
        String inner = renderedBytes.substring(1, renderedBytes.length() - 1).trim();
        String[] parts = inner.isEmpty() ? new String[0] : inner.split(",\\s*");
        ByteBuffer bytes = ByteBuffer.allocate(parts.length).order(ByteOrder.LITTLE_ENDIAN);
        for (String part : parts) {
            bytes.put(Byte.parseByte(part));
        }
        bytes.flip();
        double[] values = new double[parts.length / 8];
        for (int i = 0; i < values.length; i++) {
            values[i] = bytes.getDouble();
        }
        return values;
    }

    private static final SimRobotConfig STUB_CONFIG = new SimRobotConfig() {
        @Override public String[] driveMotorNames() { return new String[] {"fl", "fr", "bl", "br"}; }
        @Override public String odometryName() { return "odo"; }
        @Override public DrivetrainSimConfig drivetrain() { return null; }
    };
}
