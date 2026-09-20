package org.horizon36596.simloop.field;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.horizon36596.simloop.config.DrivetrainSimConfig;
import org.horizon36596.simloop.plant.MecanumPoseIntegrator;

import org.junit.jupiter.api.Test;

/**
 * The scenario the module never had: a drivetrain pose driven forward and then asked a field question
 * (BACKLOG B37). This is the only place the localizer frame and the field frame meet, so it is the only
 * place a missing rotation can be caught.
 *
 * <p>The robot drives 24 in straight forward from the origin. In the localizer frame that is
 * {@code (24, 0)} at heading 0. In the field frame it is {@code (0, 24)} at heading {@code +PI/2} — up
 * the field, facing up the field. A piece sits at field {@code (0, 30)}, six inches ahead of the robot's
 * nose, inside an intake mouth that reaches from 5 in to 11 in ahead of the robot's centre.
 *
 * <p>Both halves matter. The rotated pose acquires the piece; the raw pose does not, and that is the
 * defect written as a test rather than as a paragraph.
 */
class DrivetrainPoseMeetsFieldFrameTest {

    private static final double DELTA_TIME_SECONDS = 0.02;
    private static final double FORWARD_SPEED_INCHES_PER_SECOND = 24.0;
    private static final int TICKS = 50;                 // 50 * 0.02 s * 24 in/s = 24 in

    /** Mouth spanning the front of an 18 in robot, reaching from 5 in to 11 in ahead of centre. */
    private static final TriggerVolume INTAKE_MOUTH =
            new TriggerVolume("intake", 8.0, 0.0, 6.0, 16.0);

    private static final double PIECE_FIELD_X_INCHES = 0.0;
    private static final double PIECE_FIELD_Y_INCHES = 30.0;
    private static final double PIECE_RADIUS_INCHES = 2.5;

    private static DrivetrainSimConfig config() {
        return new DrivetrainSimConfig() {
            @Override public double trackWidth() { return 12.0; }
            @Override public double wheelBase() { return 12.0; }
            @Override public double wheelRadius() { return 2.0; }
            @Override public double ticksPerInch() { return 100.0; }
            @Override public double maxVelocityTicksPerSecond() { return 9300.0; }
            @Override public double maxAccel() { return 1.8; }
            @Override public double fieldHalfWidth() { return 72.0; }
            @Override public double fieldHalfHeight() { return 72.0; }
        };
    }

    /** Drives straight forward for {@link #TICKS} ticks and hands back the integrator. */
    private static MecanumPoseIntegrator drivenStraightForward() {
        MecanumPoseIntegrator integrator = new MecanumPoseIntegrator(config());
        for (int tick = 0; tick < TICKS; tick++) {
            integrator.integrate(FORWARD_SPEED_INCHES_PER_SECOND, FORWARD_SPEED_INCHES_PER_SECOND,
                    FORWARD_SPEED_INCHES_PER_SECOND, FORWARD_SPEED_INCHES_PER_SECOND,
                    DELTA_TIME_SECONDS);
        }
        return integrator;
    }

    @Test
    void drivingForwardMovesTheRobotUpTheFieldOnceRotated() {
        MecanumPoseIntegrator integrator = drivenStraightForward();

        // What the drivetrain reports: 24 in along its own +x, nothing along its own +y.
        assertEquals(24.0, integrator.getX(), 1e-9, "the localizer frame's +x is the robot's forward");
        assertEquals(0.0, integrator.getY(), 1e-9);

        // What the field sees: 24 in up the field, nothing across it.
        double fieldX = FieldFrameTransform.fieldXFromLocalizer(integrator.getX(), integrator.getY());
        double fieldY = FieldFrameTransform.fieldYFromLocalizer(integrator.getX(), integrator.getY());
        double fieldHeading = FieldFrameTransform.fieldHeadingFromLocalizer(integrator.getHeading());

        assertEquals(0.0, fieldX, 1e-9, "driving straight forward moves nothing across the field");
        assertEquals(24.0, fieldY, 1e-9, "driving straight forward moves 24 in up the field");
        assertEquals(Math.PI / 2.0, fieldHeading, 1e-9, "facing up the field is +PI/2 from field +X");
    }

    @Test
    void theRotatedPoseAcquiresThePieceInFrontOfTheRobot() {
        MecanumPoseIntegrator integrator = drivenStraightForward();

        double fieldX = FieldFrameTransform.fieldXFromLocalizer(integrator.getX(), integrator.getY());
        double fieldY = FieldFrameTransform.fieldYFromLocalizer(integrator.getX(), integrator.getY());
        double fieldHeading = FieldFrameTransform.fieldHeadingFromLocalizer(integrator.getHeading());

        assertTrue(INTAKE_MOUTH.overlapsPiece(fieldX, fieldY, fieldHeading,
                        PIECE_FIELD_X_INCHES, PIECE_FIELD_Y_INCHES, PIECE_RADIUS_INCHES),
                "a piece 6 in ahead of the nose is inside a mouth reaching 5 in to 11 in ahead");

        GamePieceTracker tracker = new GamePieceTracker(1);
        tracker.addLoosePiece(PIECE_FIELD_X_INCHES, PIECE_FIELD_Y_INCHES, PIECE_RADIUS_INCHES);
        tracker.updateRobotPose(fieldX, fieldY, fieldHeading);

        assertEquals(1, tracker.tryAcquire(INTAKE_MOUTH, true), "the piece is in the mouth");
        assertEquals(1, tracker.getHeldCount());
        assertEquals(0, tracker.getLooseCount());
    }

    @Test
    void theUnrotatedPoseMissesThePieceEntirelyAndThatIsTheDefect() {
        MecanumPoseIntegrator integrator = drivenStraightForward();

        // Exactly the mistake B37 describes: a localizer pose handed straight to a field question. It
        // does not throw and it does not return a nonsense number — it quietly reports an empty mouth,
        // and a season's intake test then fails as though the mouth were mis-measured.
        GamePieceTracker tracker = new GamePieceTracker(1);
        tracker.addLoosePiece(PIECE_FIELD_X_INCHES, PIECE_FIELD_Y_INCHES, PIECE_RADIUS_INCHES);
        tracker.updateRobotPose(integrator.getX(), integrator.getY(), integrator.getHeading());

        assertEquals(0, tracker.tryAcquire(INTAKE_MOUTH, true),
                "the unrotated pose puts the robot 24 in to the field's right, facing right");
        assertEquals(0, tracker.getHeldCount());
        assertEquals(1, tracker.getLooseCount());
    }

    @Test
    void aFieldPoseWrittenByAHumanGoesBackIntoTheDrivetrainsFrame() {
        // The other direction, which an autonomous start pose needs: a pose read off a field drawing,
        // pushed into the drivetrain that integrates in the localizer frame.
        double startFieldX = -36.0;          // left half of the field
        double startFieldY = 60.0;           // near the far wall
        double startFieldHeading = Math.PI;  // facing field -X

        double localizerX = FieldFrameTransform.localizerXFromField(startFieldX, startFieldY);
        double localizerY = FieldFrameTransform.localizerYFromField(startFieldX, startFieldY);
        double localizerHeading = FieldFrameTransform.localizerHeadingFromField(startFieldHeading);

        // Literal expectations on the intermediate values, not just the round trip: a field pose on the
        // left half of the field, near the far wall, facing field -X becomes a localizer pose 60 in
        // "forward" and 36 in "left", facing the robot's own +PI/2.
        assertEquals(60.0, localizerX, 1e-9);
        assertEquals(36.0, localizerY, 1e-9);
        assertEquals(Math.PI / 2.0, localizerHeading, 1e-9);

        MecanumPoseIntegrator integrator = new MecanumPoseIntegrator(config());
        integrator.setPose(localizerX, localizerY, localizerHeading);

        // Rotating back out reproduces the pose the human wrote, which is the only property that matters.
        assertEquals(startFieldX,
                FieldFrameTransform.fieldXFromLocalizer(integrator.getX(), integrator.getY()), 1e-9);
        assertEquals(startFieldY,
                FieldFrameTransform.fieldYFromLocalizer(integrator.getX(), integrator.getY()), 1e-9);
        assertEquals(startFieldHeading,
                FieldFrameTransform.fieldHeadingFromLocalizer(integrator.getHeading()), 1e-9);
    }
}
