package org.horizon36596.simloop.viz;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;

import org.junit.jupiter.api.Test;
import org.psilynx.psikit.core.wpi.math.Pose3d;
import org.psilynx.psikit.core.wpi.math.Translation3d;

/**
 * Pins {@code docs/briefs/turret-3d-viz.md}'s Contract on {@link TurretSketch}: the turret yaws on the robot at
 * the angle it is given (clause 1), what rides on it swings round with it (clause 2), and the aim marker sits
 * at the chain's tip pointing along the shot (clause 3).
 *
 * <p><b>How the expected values are computed.</b> Never by re-running the class's own arithmetic. The turret
 * is a rigid body turning about a vertical line, so every expected point is the <b>zero-pose point, rotated
 * about that line</b> by the yaw and then carried into the field by the robot's pose — worked out here from
 * the definition's own numbers, in the robot frame, with no turret frame at all. The class decomposes the
 * same motion differently (a ghost, a turret-frame canvas, a tip), so agreement is evidence, not an echo.
 *
 * <p>Every test here is <b>invariant tier</b>: it holds for any correct turret renderer.
 */
class TurretSketchTest {

    private static final double TOLERANCE_INCHES = 1.0e-9;
    private static final double TOLERANCE_RADIANS = 1.0e-9;

    /** The yaw axis, robot frame: off-centre on purpose, so a renderer that forgets it is caught. */
    private static final RobotVector3 AXIS = new RobotVector3(3.0, -2.0, 10.0);

    /** Yaw arm from the axis point to the hood pivot: 2 in forward and 4 in up at yaw zero. */
    private static final double ARM_FORWARD = 2.0;
    private static final double ARM_UP = 4.0;

    private static final double HOOD_LENGTH = 4.0;
    private static final double HOOD_DRAWN_ELEVATION = Math.toRadians(45.0);

    private static final double YAW_LIMIT = 2.5;

    /** A turret whose barrel points out of the nose at yaw zero, carrying a 4 in hood drawn at 45 degrees. */
    private static TurretSketchDefinition turretWithHood() {
        double armLength = Math.hypot(ARM_FORWARD, ARM_UP);
        return TurretSketchDefinition.builder("Turret")
                .canvasSizeInches(20.0, 30.0)
                .yaw(AXIS, new RevoluteJointDefinition("yaw",
                        new RobotVector3(ARM_FORWARD / armLength, 0.0, ARM_UP / armLength),
                        new RobotVector3(0.0, 0.0, 1.0), armLength, -YAW_LIMIT, YAW_LIMIT, "#FFD24C", 8.0))
                .addJoint(new RevoluteJointDefinition("hood",
                        new RobotVector3(Math.cos(HOOD_DRAWN_ELEVATION), 0.0, Math.sin(HOOD_DRAWN_ELEVATION)),
                        // -Y is robot-right: by the right-hand rule a positive angle raises the hood.
                        new RobotVector3(0.0, -1.0, 0.0), HOOD_LENGTH, 0.0, Math.toRadians(30.0),
                        "#FF9F4C", 6.0))
                .build();
    }

    /**
     * The hood's tip at the given yaw and hood angle, field frame, inches: the zero-yaw tip in the robot frame,
     * rotated about the vertical line through {@link #AXIS}, then placed by the robot pose.
     */
    private static double[] expectedTipInField(double yaw, double hoodAngle,
                                               double robotX, double robotY, double robotHeading) {
        double elevation = HOOD_DRAWN_ELEVATION + hoodAngle;
        // Zero-yaw tip, robot frame: along +X from the axis, up from the axis point.
        double tipForwardOfAxis = ARM_FORWARD + HOOD_LENGTH * Math.cos(elevation);
        double tipZ = AXIS.z + ARM_UP + HOOD_LENGTH * Math.sin(elevation);
        // Rotate about the axis by the yaw, then by the robot heading into the field.
        double inRobotX = AXIS.x + tipForwardOfAxis * Math.cos(yaw);
        double inRobotY = AXIS.y + tipForwardOfAxis * Math.sin(yaw);
        return new double[] {
            robotX + inRobotX * Math.cos(robotHeading) - inRobotY * Math.sin(robotHeading),
            robotY + inRobotX * Math.sin(robotHeading) + inRobotY * Math.cos(robotHeading),
            tipZ};
    }

    // ---- Clause 1: the turret yaws on the robot, at the angle it is given ----------------------------------

    @Test
    void theGhostTurnsWithTheTurretAndRidesOnTheRobotAtTheTurretsOwnAxis() {
        double[] yaws = {-2.0, -0.7, 0.0, 0.4, 1.9};
        double[] headings = {0.0, 0.6, -2.4};
        for (double yaw : yaws) {
            for (double heading : headings) {
                TurretSketch sketch = new TurretSketch(turretWithHood());
                sketch.setRobotFieldPose(30.0, -12.0, heading);
                sketch.setYawRadians(yaw);
                double axisFieldX = 30.0 + AXIS.x * Math.cos(heading) - AXIS.y * Math.sin(heading);
                double axisFieldY = -12.0 + AXIS.x * Math.sin(heading) + AXIS.y * Math.cos(heading);
                String at = " at yaw " + yaw + ", heading " + heading;
                assertEquals(axisFieldX, sketch.getGhostFieldXInches(), TOLERANCE_INCHES, "ghost x must be the turret axis" + at);
                assertEquals(axisFieldY, sketch.getGhostFieldYInches(), TOLERANCE_INCHES, "ghost y must be the turret axis" + at);
                assertEquals(heading + yaw, sketch.getGhostFieldHeadingRadians(), TOLERANCE_RADIANS,
                        "ghost heading must be the robot's heading plus the turret's yaw" + at);
            }
        }
    }

    @Test
    void theYawIsClampedToTheTurretsTravelSoItIsNeverDrawnPastItsHardStop() {
        TurretSketch sketch = new TurretSketch(turretWithHood());
        sketch.setYawRadians(4.0);
        assertEquals(YAW_LIMIT, sketch.getYawRadians(), TOLERANCE_RADIANS, "past the CCW stop draws at the stop");
        sketch.setYawRadians(-4.0);
        assertEquals(-YAW_LIMIT, sketch.getYawRadians(), TOLERANCE_RADIANS, "past the CW stop draws at the stop");
        sketch.setYawRadians(Double.NaN);
        assertEquals(0.0, sketch.getYawRadians(), TOLERANCE_RADIANS, "a bad reading draws the turret at zero");
    }

    @Test
    void aClockwisePositiveTurretTurnsTheGhostTheOtherWay() {
        // Same turret, axis flipped to -Z: a positive angle now turns clockwise seen from above.
        double armLength = Math.hypot(ARM_FORWARD, ARM_UP);
        TurretSketch sketch = new TurretSketch(TurretSketchDefinition.builder("Turret")
                .canvasSizeInches(20.0, 30.0)
                .yaw(AXIS, new RevoluteJointDefinition("yaw",
                        new RobotVector3(ARM_FORWARD / armLength, 0.0, ARM_UP / armLength),
                        new RobotVector3(0.0, 0.0, -1.0), armLength, -YAW_LIMIT, YAW_LIMIT, "#FFD24C", 8.0))
                .build());
        sketch.setYawRadians(0.5);
        assertEquals(-0.5, sketch.getGhostFieldHeadingRadians(), TOLERANCE_RADIANS,
                "about -Z, +0.5 rad is 0.5 rad clockwise");
    }

    // ---- Clause 2: the hood rides on the turret -------------------------------------------------------------

    @Test
    void theHoodIsDrawnInTheTurretsOwnPlaneSoItsPictureDoesNotChangeAsTheTurretTurns() {
        TurretSketch sketch = new TurretSketch(turretWithHood());
        sketch.setJointAngleRadians("hood", Math.toRadians(20.0));
        double drawnAtZero = sketch.getRideAlongSketch().getDrawnAngleDegrees("hood");
        double tipAcrossAtZero = sketch.getRideAlongSketch().getTipAcrossInches();
        double tipUpAtZero = sketch.getRideAlongSketch().getTipUpInches();

        sketch.setYawRadians(1.7);
        sketch.setRobotFieldPose(-40.0, 25.0, 2.2);

        assertEquals(drawnAtZero, sketch.getRideAlongSketch().getDrawnAngleDegrees("hood"), 1.0e-9,
                "the ghost carries the yaw; the hood's own drawing must not move when the turret does");
        assertEquals(tipAcrossAtZero, sketch.getRideAlongSketch().getTipAcrossInches(), TOLERANCE_INCHES);
        assertEquals(tipUpAtZero, sketch.getRideAlongSketch().getTipUpInches(), TOLERANCE_INCHES);
    }

    @Test
    void theHoodIsDrawnAtItsElevationAboveHorizontal() {
        TurretSketch sketch = new TurretSketch(turretWithHood());
        for (double hoodDegrees : new double[] {0.0, 10.0, 30.0}) {
            sketch.setJointAngleRadians("hood", Math.toRadians(hoodDegrees));
            assertEquals(HOOD_DRAWN_ELEVATION + Math.toRadians(hoodDegrees), sketch.getAimElevationRadians(),
                    TOLERANCE_RADIANS, "a +" + hoodDegrees + " degree hood command raises the exit by that much");
        }
    }

    // ---- Clause 3: the aim marker is at the exit, along yaw plus pitch ---------------------------------------

    @Test
    void theAimMarkerSitsAtTheHoodsTipForAnyYawHoodAngleAndRobotPose() {
        double[][] cases = {
            // yaw, hood angle, robot x, robot y, robot heading
            {0.0, 0.0, 0.0, 0.0, 0.0},
            {1.2, 0.3, 30.0, -12.0, 0.6},
            {-2.1, 0.5, -55.0, 40.0, -2.4},
            {2.5, 0.1, 10.0, 10.0, 3.1},
        };
        for (double[] c : cases) {
            TurretSketch sketch = new TurretSketch(turretWithHood());
            sketch.setRobotFieldPose(c[2], c[3], c[4]);
            sketch.setYawRadians(c[0]);
            sketch.setJointAngleRadians("hood", c[1]);

            double[] expected = expectedTipInField(c[0], c[1], c[2], c[3], c[4]);
            String at = " for yaw " + c[0] + ", hood " + c[1] + ", robot (" + c[2] + ", " + c[3] + ", " + c[4] + ")";
            assertEquals(expected[0], sketch.getAimFieldXInches(), TOLERANCE_INCHES, "aim x" + at);
            assertEquals(expected[1], sketch.getAimFieldYInches(), TOLERANCE_INCHES, "aim y" + at);
            assertEquals(expected[2], sketch.getAimFieldZInches(), TOLERANCE_INCHES, "aim height" + at);
            assertEquals(c[4] + c[0], sketch.getAimHeadingRadians(), TOLERANCE_RADIANS, "aim heading is robot heading plus yaw" + at);
            assertEquals(HOOD_DRAWN_ELEVATION + c[1], sketch.getAimElevationRadians(), TOLERANCE_RADIANS, "aim elevation" + at);
        }
    }

    /**
     * The logged pose is what AdvantageScope reads, and a Cone points along its pose's +X. So rotate +X by the
     * logged rotation and check it points up the shot: this catches a pitch sign flipped the wrong way (WPILib's
     * pitch is positive nose-down), which the angle getters alone would not.
     */
    @Test
    void theLoggedAimPosePointsItsPlusXAlongTheShotInMetres() {
        TurretSketch sketch = new TurretSketch(turretWithHood());
        sketch.setRobotFieldPose(30.0, -12.0, 0.6);
        sketch.setYawRadians(1.2);
        sketch.setJointAngleRadians("hood", 0.3);

        Pose3d logged = sketch.aimPose3d();
        Translation3d pointing = new Translation3d(1.0, 0.0, 0.0).rotateBy(logged.getRotation());
        double elevation = HOOD_DRAWN_ELEVATION + 0.3;
        double heading = 0.6 + 1.2;
        assertEquals(Math.cos(elevation) * Math.cos(heading), pointing.getX(), 1.0e-9, "cone +X, field x");
        assertEquals(Math.cos(elevation) * Math.sin(heading), pointing.getY(), 1.0e-9, "cone +X, field y");
        assertEquals(Math.sin(elevation), pointing.getZ(), 1.0e-9, "cone +X must rise with the shot");

        double[] expected = expectedTipInField(1.2, 0.3, 30.0, -12.0, 0.6);
        assertEquals(expected[0] * 0.0254, logged.getX(), 1.0e-9, "logged in metres, like Drive/pose2d");
        assertEquals(expected[2] * 0.0254, logged.getZ(), 1.0e-9, "logged in metres, like Drive/pose2d");
    }

    @Test
    void aBareTurretAimsLevelAlongItsArmFromTheArmsTip() {
        double armLength = Math.hypot(ARM_FORWARD, ARM_UP);
        TurretSketch sketch = new TurretSketch(TurretSketchDefinition.builder("Bare")
                .canvasSizeInches(20.0, 30.0)
                .yaw(AXIS, new RevoluteJointDefinition("yaw",
                        new RobotVector3(ARM_FORWARD / armLength, 0.0, ARM_UP / armLength),
                        new RobotVector3(0.0, 0.0, 1.0), armLength, -YAW_LIMIT, YAW_LIMIT, "#FFD24C", 8.0))
                .build());
        sketch.setYawRadians(Math.PI / 2.0);
        assertEquals(AXIS.x, sketch.getAimFieldXInches(), TOLERANCE_INCHES, "turned a quarter left, the arm tip is straight left");
        assertEquals(AXIS.y + ARM_FORWARD, sketch.getAimFieldYInches(), TOLERANCE_INCHES);
        assertEquals(AXIS.z + ARM_UP, sketch.getAimFieldZInches(), TOLERANCE_INCHES);
        assertEquals(0.0, sketch.getAimElevationRadians(), TOLERANCE_RADIANS, "with nothing on the turret, the aim is level");
    }

    /**
     * A chain mounted off to the SIDE of the turret axis cannot be carried by a Mechanism2d, which has no
     * sideways coordinate. The ghost must take the offset instead, or the hood is drawn on the axis.
     */
    @Test
    void aChainMountedToTheSideOfTheAxisStaysOnItsOwnSideAsTheTurretTurns() {
        // Yaw arm: 2 in forward, 3 in to the LEFT, 4 in up.
        RobotVector3 arm = new RobotVector3(2.0, 3.0, 4.0);
        double armLength = arm.length();
        TurretSketch sketch = new TurretSketch(TurretSketchDefinition.builder("Offset")
                .canvasSizeInches(20.0, 30.0)
                .yaw(AXIS, new RevoluteJointDefinition("yaw", arm.normalized(), new RobotVector3(0.0, 0.0, 1.0),
                        armLength, -YAW_LIMIT, YAW_LIMIT, "#FFD24C", 8.0))
                .addJoint(new RevoluteJointDefinition("hood",
                        new RobotVector3(Math.cos(HOOD_DRAWN_ELEVATION), 0.0, Math.sin(HOOD_DRAWN_ELEVATION)),
                        new RobotVector3(0.0, -1.0, 0.0), HOOD_LENGTH, 0.0, Math.toRadians(30.0),
                        "#FF9F4C", 6.0))
                .build());
        double yaw = 0.9;
        sketch.setYawRadians(yaw);

        // Zero-yaw tip in the robot frame, relative to the axis: the arm, then the hood straight ahead.
        double relX = 2.0 + HOOD_LENGTH * Math.cos(HOOD_DRAWN_ELEVATION);
        double relY = 3.0;
        assertEquals(AXIS.x + relX * Math.cos(yaw) - relY * Math.sin(yaw), sketch.getAimFieldXInches(), TOLERANCE_INCHES,
                "the whole tip, offset included, turns about the axis");
        assertEquals(AXIS.y + relX * Math.sin(yaw) + relY * Math.cos(yaw), sketch.getAimFieldYInches(), TOLERANCE_INCHES,
                "the whole tip, offset included, turns about the axis");
    }

    // ---- What cannot be drawn is refused, not drawn wrong -----------------------------------------------------

    @Test
    void aYawAxisThatIsNotVerticalIsRejected() {
        assertThrows(IllegalArgumentException.class, () -> TurretSketchDefinition.builder("Tilted")
                .canvasSizeInches(20.0, 30.0)
                .yaw(AXIS, new RevoluteJointDefinition("yaw", new RobotVector3(1.0, 0.0, 0.0),
                        new RobotVector3(0.0, -1.0, 0.0), 2.0, -1.0, 1.0, "#FFD24C", 8.0))
                .build());
    }

    @Test
    void aTurretWithNothingPointingSidewaysIsRejectedBecauseNothingSaysWhereZeroFaces() {
        assertThrows(IllegalArgumentException.class, () -> TurretSketchDefinition.builder("Vertical")
                .canvasSizeInches(20.0, 30.0)
                .yaw(AXIS, new RevoluteJointDefinition("yaw", new RobotVector3(0.0, 0.0, 1.0),
                        new RobotVector3(0.0, 0.0, 1.0), 2.0, -1.0, 1.0, "#FFD24C", 8.0))
                .build());
    }

    /** The common layout: the hood directly above the turret axis. The hood's own plane says which way is zero. */
    @Test
    void aHoodDirectlyAboveTheAxisFacesTheWayTheHoodPoints() {
        TurretSketch sketch = new TurretSketch(TurretSketchDefinition.builder("Stacked")
                .canvasSizeInches(20.0, 30.0)
                .yaw(AXIS, new RevoluteJointDefinition("yaw", new RobotVector3(0.0, 0.0, 1.0),
                        new RobotVector3(0.0, 0.0, 1.0), ARM_UP, -YAW_LIMIT, YAW_LIMIT, "#FFD24C", 8.0))
                .addJoint(new RevoluteJointDefinition("hood",
                        new RobotVector3(Math.cos(HOOD_DRAWN_ELEVATION), 0.0, Math.sin(HOOD_DRAWN_ELEVATION)),
                        new RobotVector3(0.0, -1.0, 0.0), HOOD_LENGTH, 0.0, Math.toRadians(30.0),
                        "#FF9F4C", 6.0))
                .build());
        sketch.setYawRadians(0.8);
        double tipForward = HOOD_LENGTH * Math.cos(HOOD_DRAWN_ELEVATION);
        assertEquals(AXIS.x + tipForward * Math.cos(0.8), sketch.getAimFieldXInches(), TOLERANCE_INCHES);
        assertEquals(AXIS.y + tipForward * Math.sin(0.8), sketch.getAimFieldYInches(), TOLERANCE_INCHES);
        assertEquals(AXIS.z + ARM_UP + HOOD_LENGTH * Math.sin(HOOD_DRAWN_ELEVATION), sketch.getAimFieldZInches(), TOLERANCE_INCHES);
        assertEquals(0.8, sketch.getAimHeadingRadians(), TOLERANCE_RADIANS, "the aim heading is the yaw, from a forward-facing zero");
    }

    @Test
    void aJointThatSweepsOutOfTheTurretsPlaneIsRejectedRatherThanDrawnForeshortened() {
        double armLength = Math.hypot(ARM_FORWARD, ARM_UP);
        TurretSketchDefinition sideways = TurretSketchDefinition.builder("Sideways")
                .canvasSizeInches(20.0, 30.0)
                .yaw(AXIS, new RevoluteJointDefinition("yaw",
                        new RobotVector3(ARM_FORWARD / armLength, 0.0, ARM_UP / armLength),
                        new RobotVector3(0.0, 0.0, 1.0), armLength, -YAW_LIMIT, YAW_LIMIT, "#FFD24C", 8.0))
                // The hood sets the turret's plane (forward and up); a stage pointing LEFT leaves that plane.
                .addJoint(new RevoluteJointDefinition("hood",
                        new RobotVector3(Math.cos(HOOD_DRAWN_ELEVATION), 0.0, Math.sin(HOOD_DRAWN_ELEVATION)),
                        new RobotVector3(0.0, -1.0, 0.0), HOOD_LENGTH, 0.0, Math.toRadians(30.0),
                        "#FF9F4C", 6.0))
                .addJoint(new LinearJointDefinition("stage", new RobotVector3(0.0, 1.0, 0.0),
                        1.0, 0.0, 5.0, "#FFFFFF", 4.0))
                .build();
        assertThrows(IllegalArgumentException.class, () -> new TurretSketch(sideways));
    }

    @Test
    void aNonFiniteRobotPoseLeavesTheDrawingWhereItWas() {
        TurretSketch sketch = new TurretSketch(turretWithHood());
        sketch.setRobotFieldPose(10.0, 20.0, 0.5);
        double xBefore = sketch.getGhostFieldXInches();
        double yBefore = sketch.getGhostFieldYInches();
        double headingBefore = sketch.getGhostFieldHeadingRadians();
        sketch.setRobotFieldPose(Double.NaN, 20.0, 0.5);
        assertEquals(xBefore, sketch.getGhostFieldXInches(), 0.0);
        assertEquals(yBefore, sketch.getGhostFieldYInches(), 0.0);
        assertEquals(headingBefore, sketch.getGhostFieldHeadingRadians(), 0.0);
    }

    @Test
    void theGhostIsLoggedInMetresAtFloorLevel() {
        TurretSketch sketch = new TurretSketch(turretWithHood());
        sketch.setRobotFieldPose(40.0, -20.0, 0.0);
        Pose3d ghost = sketch.ghostPose3d();
        assertEquals((40.0 + AXIS.x) * 0.0254, ghost.getX(), 1.0e-12, "inches in, metres out");
        assertEquals(0.0, ghost.getZ(), 0.0, "the ghost's origin is on the floor, where the canvas bottom is");
    }
}
