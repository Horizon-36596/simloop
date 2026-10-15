package org.horizon36596.simloop.fakehardware;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS.Pose2D;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS.SignalProcessConfig;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS.Status;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS.Version;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.horizon36596.simloop.config.DrivetrainSimConfig;
import org.horizon36596.simloop.plant.MecanumDrivePlant;
import org.junit.jupiter.api.Test;

import java.io.ByteArrayOutputStream;
import java.io.DataOutputStream;
import java.io.IOException;
import java.lang.reflect.Method;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * Pins {@link FakeSparkFunOTOS}: a moving chassis read through the REAL SDK 12.0 {@code SparkFunOTOS} driver,
 * and the sensor behaviours that driver exposes - units, frame and signs, declared versus physical mounting,
 * scalars, reset, re-basing, velocity and acceleration, IMU calibration, status, self test, connection, and
 * determinism. SimLoop only: no season code. INVARIANT tier throughout, except where a test's name starts
 * with {@code placeholder_}: those pin one of this fake's own PLACEHOLDER choices.
 */
class FakeSparkFunOTOSTest {

    private static final double DT = 0.02;
    private static final double MM_PER_INCH = 25.4;
    /** One int16 count of the position registers, inches: 1 / 3276.8 m. */
    private static final double POSITION_COUNT_INCHES = 1.0 / 3276.8 / 0.0254;
    /** One int16 count of the heading register, degrees: 180 / 32768. */
    private static final double HEADING_COUNT_DEGREES = 180.0 / 32768.0;

    private static DrivetrainSimConfig chassisConfig() {
        return new DrivetrainSimConfig() {
            @Override public double trackWidth() { return 14.0; }
            @Override public double wheelBase() { return 12.0; }
            @Override public double wheelRadius() { return 1.89; }
            @Override public double ticksPerInch() { return 26.2; }
            @Override public double maxVelocityTicksPerSecond() { return 2000.0; }
            @Override public double maxAccel() { return 4.0; }
            @Override public double fieldHalfWidth() { return 72.0; }
            @Override public double fieldHalfHeight() { return 72.0; }
        };
    }

    /** A drive plant and the OTOS riding on it, attached at the plant's start. */
    private static final class Robot {
        final FakeMotor[] wheels = new FakeMotor[4];
        final MecanumDrivePlant chassis;
        final FakeSparkFunOTOS otos;

        Robot(FakeSparkFunOTOS otos) {
            DrivetrainSimConfig config = chassisConfig();
            for (int i = 0; i < 4; i++) {
                wheels[i] = new FakeMotor(config.maxVelocityTicksPerSecond(), config.maxAccel());
            }
            chassis = new MecanumDrivePlant(wheels, config);
            this.otos = otos;
            otos.attachChassisAt(chassis.getX() * MM_PER_INCH, chassis.getY() * MM_PER_INCH, chassis.getHeading());
        }

        /** One loop: set the wheel powers [FL, FR, BL, BR], let the world move, then tick the sensor. */
        void tick(double fl, double fr, double bl, double br) {
            double[] power = {fl, fr, bl, br};
            for (int i = 0; i < 4; i++) {
                wheels[i].setPower(power[i]);
                wheels[i].update(DT);
            }
            chassis.update(DT);
            otos.moveChassisTo(chassis.getX() * MM_PER_INCH, chassis.getY() * MM_PER_INCH, chassis.getHeading());
            otos.update(DT);
        }

        /** 50 ticks forward, 50 strafing left, 50 turning CCW. */
        void driveTheStandardRoute(Runnable afterEachTick) {
            for (int k = 0; k < 150; k++) {
                if (k < 50) {
                    tick(1, 1, 1, 1);
                } else if (k < 100) {
                    tick(-1, 1, 1, -1);
                } else {
                    tick(-0.5, 0.5, -0.5, 0.5);
                }
                afterEachTick.run();
            }
        }
    }

    /** Synthetic motion: the chassis is now at (x, y, h) mm/mm/rad, then the sensor ticks once. */
    private static void step(FakeSparkFunOTOS otos, double xMm, double yMm, double headingRad) {
        otos.moveChassisTo(xMm, yMm, headingRad);
        otos.update(DT);
    }

    /** Walks the chassis in a straight line to (xMm, yMm) at a fixed heading, in {@code steps} ticks. */
    private static void driveStraight(FakeSparkFunOTOS otos, double fromXMm, double fromYMm,
                                      double toXMm, double toYMm, double headingRad, int steps) {
        for (int k = 1; k <= steps; k++) {
            double f = (double) k / steps;
            step(otos, fromXMm + (toXMm - fromXMm) * f, fromYMm + (toYMm - fromYMm) * f, headingRad);
        }
    }

    /** Spins the chassis in place at (xMm, yMm) from one heading to another, in {@code steps} ticks. */
    private static void spinInPlace(FakeSparkFunOTOS otos, double xMm, double yMm,
                                    double fromRad, double toRad, int steps) {
        for (int k = 1; k <= steps; k++) {
            step(otos, xMm, yMm, fromRad + (toRad - fromRad) * k / steps);
        }
    }

    private static double wrapDegrees(double degrees) {
        double wrapped = degrees % 360.0;
        if (wrapped >= 180.0) {
            wrapped -= 360.0;
        } else if (wrapped < -180.0) {
            wrapped += 360.0;
        }
        return wrapped;
    }

    // =================================================================================================
    // Pose, units, frame
    // =================================================================================================

    @Test
    void invariant_aMovingChassisReadsBackThroughTheRealDriverInInchesAndDegrees() {
        Robot robot = new Robot(new FakeSparkFunOTOS());
        robot.driveTheStandardRoute(() -> {
            Pose2D pose = robot.otos.getPosition();
            assertEquals(robot.chassis.getX(), pose.x, 2 * POSITION_COUNT_INCHES, "x, inches");
            assertEquals(robot.chassis.getY(), pose.y, 2 * POSITION_COUNT_INCHES, "y, inches");
            assertEquals(wrapDegrees(Math.toDegrees(robot.chassis.getHeading())), pose.h,
                    2 * HEADING_COUNT_DEGREES, "heading, degrees");
        });
        assertTrue(robot.chassis.getX() > 10.0 && robot.chassis.getY() > 10.0 && robot.chassis.getHeading() > 0.5,
                "the chassis really went forward (+X), left (+Y) and counter-clockwise (+heading)");
    }

    @Test
    void invariant_unitsStartAtTheSdkDefaultsAndConvertThroughTheDriver() {
        FakeSparkFunOTOS otos = new FakeSparkFunOTOS();
        assertEquals(DistanceUnit.INCH, otos.getLinearUnit(), "the SDK driver's doInitialize default");
        assertEquals(AngleUnit.DEGREES, otos.getAngularUnit(), "the SDK driver's doInitialize default");

        step(otos, 1000.0, 500.0, Math.PI / 2);
        Pose2D inches = otos.getPosition();
        assertEquals(1000.0 / MM_PER_INCH, inches.x, POSITION_COUNT_INCHES);
        assertEquals(500.0 / MM_PER_INCH, inches.y, POSITION_COUNT_INCHES);
        assertEquals(90.0, inches.h, HEADING_COUNT_DEGREES);

        otos.setLinearUnit(DistanceUnit.METER);
        otos.setAngularUnit(AngleUnit.RADIANS);
        assertEquals(DistanceUnit.METER, otos.getLinearUnit());
        assertEquals(AngleUnit.RADIANS, otos.getAngularUnit());
        Pose2D metres = otos.getPosition();
        assertEquals(1.0, metres.x, 1.0 / 3276.8);
        assertEquals(0.5, metres.y, 1.0 / 3276.8);
        assertEquals(Math.PI / 2, metres.h, Math.PI / 32768);

        otos.setLinearUnit(DistanceUnit.MM);
        assertEquals(1000.0, otos.getPosition().x, 0.31, "millimetres too");
    }

    @Test
    void invariant_headingIsCounterClockwisePositiveAndWrapsIntoPlusMinus180() {
        FakeSparkFunOTOS otos = new FakeSparkFunOTOS();
        spinInPlace(otos, 0.0, 0.0, 0.0, Math.toRadians(30.0), 3);
        assertEquals(30.0, otos.getPosition().h, HEADING_COUNT_DEGREES, "a CCW turn is positive");
        spinInPlace(otos, 0.0, 0.0, Math.toRadians(30.0), Math.toRadians(200.0), 10);
        assertEquals(-160.0, otos.getPosition().h, HEADING_COUNT_DEGREES, "200 degrees reads as -160");
        spinInPlace(otos, 0.0, 0.0, Math.toRadians(200.0), Math.toRadians(-45.0), 20);
        assertEquals(-45.0, otos.getPosition().h, HEADING_COUNT_DEGREES, "a CW turn is negative");
    }

    // =================================================================================================
    // Declared (setOffset) versus physical (setPhysicalMount) mounting
    // =================================================================================================

    @Test
    void invariant_aDeclaredOffsetThatMatchesThePhysicalMountReadsTheRobotCentre() {
        FakeSparkFunOTOS otos = new FakeSparkFunOTOS();
        otos.setPhysicalMount(150.0, -80.0, Math.toRadians(90.0));
        otos.setOffset(new Pose2D(150.0 / MM_PER_INCH, -80.0 / MM_PER_INCH, 90.0));
        Robot robot = new Robot(otos);
        // The driver truncates the offset to whole register counts, so the declared mount is up to one count
        // (0.3 mm, 0.0055 degrees) off the physical one; over this route that is a few thousandths of an inch.
        robot.driveTheStandardRoute(() -> {
            Pose2D pose = otos.getPosition();
            assertEquals(robot.chassis.getX(), pose.x, 0.02, "x, inches");
            assertEquals(robot.chassis.getY(), pose.y, 0.02, "y, inches");
            assertEquals(wrapDegrees(Math.toDegrees(robot.chassis.getHeading())), pose.h, 0.02, "heading");
        });
    }

    @Test
    void invariant_anUndeclaredForwardOffsetMakesATurnInPlaceReadAsACircle() {
        FakeSparkFunOTOS misconfigured = new FakeSparkFunOTOS();
        misconfigured.setPhysicalMount(100.0, 0.0, 0.0);      // really 100 mm ahead of centre; never declared
        misconfigured.setLinearUnit(DistanceUnit.MM);
        spinInPlace(misconfigured, 0.0, 0.0, 0.0, Math.PI / 2, 10);
        Pose2D wrong = misconfigured.getPosition();
        // It reports the sensor's own path: from (100, 0) round to (0, 100), a move of (-100, +100).
        assertEquals(-100.0, wrong.x, 0.5, "the robot centre never moved, but it reads as having moved back");
        assertEquals(100.0, wrong.y, 0.5, "and left");
        assertEquals(90.0, wrong.h, 2 * HEADING_COUNT_DEGREES, "the heading is right: it comes from the gyro");

        FakeSparkFunOTOS configured = new FakeSparkFunOTOS();
        configured.setPhysicalMount(100.0, 0.0, 0.0);
        configured.setLinearUnit(DistanceUnit.MM);
        configured.setOffset(new Pose2D(100.0, 0.0, 0.0));
        spinInPlace(configured, 0.0, 0.0, 0.0, Math.PI / 2, 10);
        Pose2D right = configured.getPosition();
        assertEquals(0.0, right.x, 0.5, "declared, the same spin stays on the spot");
        assertEquals(0.0, right.y, 0.5);
    }

    @Test
    void invariant_anUndeclaredNinetyDegreeMountMakesDrivingForwardReadAsAStrafeRight() {
        FakeSparkFunOTOS misconfigured = new FakeSparkFunOTOS();
        misconfigured.setPhysicalMount(0.0, 0.0, Math.toRadians(90.0));   // its X axis really points left
        misconfigured.setLinearUnit(DistanceUnit.MM);
        driveStraight(misconfigured, 0.0, 0.0, 500.0, 0.0, 0.0, 25);
        Pose2D wrong = misconfigured.getPosition();
        assertEquals(0.0, wrong.x, 0.5, "no forward motion is seen");
        assertEquals(-500.0, wrong.y, 0.5, "forward travel lands on the sensor's -Y: a strafe to the right");

        FakeSparkFunOTOS configured = new FakeSparkFunOTOS();
        configured.setPhysicalMount(0.0, 0.0, Math.toRadians(90.0));
        configured.setLinearUnit(DistanceUnit.MM);
        configured.setOffset(new Pose2D(0.0, 0.0, 90.0));
        driveStraight(configured, 0.0, 0.0, 500.0, 0.0, 0.0, 25);
        Pose2D right = configured.getPosition();
        assertEquals(500.0, right.x, 0.5, "declared, forward reads forward");
        assertEquals(0.0, right.y, 0.5);
    }

    @Test
    void invariant_theOffsetReadsBackAsWrittenInTheCallersUnits() {
        FakeSparkFunOTOS otos = new FakeSparkFunOTOS();
        otos.setOffset(new Pose2D(3.0, -2.0, 45.0));
        Pose2D inches = otos.getOffset();
        // The SDK driver truncates toward zero to whole counts on the way in, so the read-back is at most one
        // count smaller in magnitude, never larger.
        assertEquals(3.0, inches.x, POSITION_COUNT_INCHES);
        assertEquals(-2.0, inches.y, POSITION_COUNT_INCHES);
        assertEquals(45.0, inches.h, HEADING_COUNT_DEGREES);
        assertTrue(Math.abs(inches.x) <= 3.0 && Math.abs(inches.y) <= 2.0, "truncated, not rounded up");

        otos.setLinearUnit(DistanceUnit.METER);
        otos.setAngularUnit(AngleUnit.RADIANS);
        Pose2D metres = otos.getOffset();
        assertEquals(3.0 * 0.0254, metres.x, 1.0 / 3276.8);
        assertEquals(Math.PI / 4, metres.h, Math.PI / 32768);
    }

    @Test
    void invariant_physicalMountArgumentsAreChecked() {
        FakeSparkFunOTOS otos = new FakeSparkFunOTOS();
        assertThrows(IllegalArgumentException.class, () -> otos.setPhysicalMount(Double.NaN, 0.0, 0.0));
        assertThrows(IllegalArgumentException.class, () -> otos.setPhysicalMount(0.0, 0.0, Double.POSITIVE_INFINITY));
        assertThrows(IllegalArgumentException.class, () -> otos.setPhysicalTrackingGains(0.0, 1.0));
        assertThrows(IllegalArgumentException.class, () -> otos.setPhysicalTrackingGains(1.0, -1.0));
        assertThrows(IllegalArgumentException.class, () -> otos.setPhysicalTrackingGains(Double.NaN, 1.0));
    }

    // =================================================================================================
    // Scalars
    // =================================================================================================

    @Test
    void invariant_theLinearScalarMultipliesEveryDistance() {
        FakeSparkFunOTOS otos = new FakeSparkFunOTOS();
        otos.setLinearUnit(DistanceUnit.MM);
        assertEquals(1.0, otos.getLinearScalar(), 1e-12, "power-on scalar");
        assertTrue(otos.setLinearScalar(1.05));
        assertEquals(1.05, otos.getLinearScalar(), 1e-12);
        driveStraight(otos, 0.0, 0.0, 600.0, 800.0, 0.0, 40);
        Pose2D pose = otos.getPosition();
        assertEquals(630.0, pose.x, 0.5, "1.05 x 600 mm");
        assertEquals(840.0, pose.y, 0.5, "1.05 x 800 mm");
        assertEquals(0.0, pose.h, 1e-9, "the linear scalar does not touch heading");
    }

    @Test
    void invariant_theAngularScalarMultipliesEveryTurn() {
        FakeSparkFunOTOS otos = new FakeSparkFunOTOS();
        assertTrue(otos.setAngularScalar(1.05));
        assertEquals(1.05, otos.getAngularScalar(), 1e-12);
        spinInPlace(otos, 0.0, 0.0, 0.0, Math.PI / 2, 10);
        assertEquals(94.5, otos.getPosition().h, 2 * HEADING_COUNT_DEGREES, "1.05 x 90 degrees");

        // Below 1 the SDK's (value - 1) * 1000 + 0.5 truncates toward zero, so 0.95 is stored as 0.951.
        // That is the real driver's rounding, reproduced.
        FakeSparkFunOTOS below = new FakeSparkFunOTOS();
        assertTrue(below.setAngularScalar(0.95));
        assertEquals(0.951, below.getAngularScalar(), 1e-12);
        spinInPlace(below, 0.0, 0.0, 0.0, Math.PI / 2, 10);
        assertEquals(0.951 * 90.0, below.getPosition().h, 2 * HEADING_COUNT_DEGREES);
    }

    @Test
    void invariant_scalarsOutsideTheSdkRangeAreRefusedAndChangeNothing() {
        FakeSparkFunOTOS otos = new FakeSparkFunOTOS();
        assertTrue(otos.setLinearScalar(1.1));
        assertFalse(otos.setLinearScalar(1.2), "above MAX_SCALAR");
        assertFalse(otos.setLinearScalar(0.8), "below MIN_SCALAR");
        assertEquals(1.1, otos.getLinearScalar(), 1e-12, "the refused writes never reached the sensor");
        assertFalse(otos.setAngularScalar(SparkFunOTOS.MAX_SCALAR + 0.001));
        assertEquals(1.0, otos.getAngularScalar(), 1e-12);

        // The ends of the range, through the driver's own (value - 1) * 1000 + 0.5, truncated to a byte.
        assertTrue(otos.setLinearScalar(SparkFunOTOS.MAX_SCALAR));
        assertEquals(1.127, otos.getLinearScalar(), 1e-12);
        assertTrue(otos.setLinearScalar(SparkFunOTOS.MIN_SCALAR));
        assertEquals(0.873, otos.getLinearScalar(), 1e-12, "-127.5 truncates to -127: the SDK's own rounding");
        assertTrue(otos.setAngularScalar(1.0504));
        assertEquals(1.050, otos.getAngularScalar(), 1e-12, "whole thousandths only");
    }

    @Test
    void invariant_aScalarCalibratesAwayAPhysicalTrackingError() {
        FakeSparkFunOTOS uncalibrated = new FakeSparkFunOTOS();
        uncalibrated.setPhysicalTrackingGains(0.97, 1.02);
        uncalibrated.setLinearUnit(DistanceUnit.MM);
        driveStraight(uncalibrated, 0.0, 0.0, 1000.0, 0.0, 0.0, 50);
        spinInPlace(uncalibrated, 1000.0, 0.0, 0.0, Math.PI / 2, 10);
        assertEquals(970.0, uncalibrated.getPosition().x, 0.5, "the sensor under-reads this floor by 3%");
        assertEquals(91.8, uncalibrated.getPosition().h, 2 * HEADING_COUNT_DEGREES, "and its gyro over-reads 2%");

        FakeSparkFunOTOS calibrated = new FakeSparkFunOTOS();
        calibrated.setPhysicalTrackingGains(0.97, 1.02);
        calibrated.setLinearUnit(DistanceUnit.MM);
        assertTrue(calibrated.setLinearScalar(1.0 / 0.97));
        assertTrue(calibrated.setAngularScalar(1.0 / 1.02));
        driveStraight(calibrated, 0.0, 0.0, 1000.0, 0.0, 0.0, 50);
        spinInPlace(calibrated, 1000.0, 0.0, 0.0, Math.PI / 2, 10);
        // The scalar register holds whole thousandths, so the correction is good to about 0.05%.
        assertEquals(1000.0, calibrated.getPosition().x, 1.0);
        assertEquals(90.0, calibrated.getPosition().h, 0.1);
    }

    // =================================================================================================
    // resetTracking and setPosition
    // =================================================================================================

    @Test
    void invariant_resetTrackingZeroesThePoseAndTrackingCarriesOnFromTheOrigin() {
        FakeSparkFunOTOS otos = new FakeSparkFunOTOS();
        otos.setLinearUnit(DistanceUnit.MM);
        otos.setOffset(new Pose2D(50.0, 0.0, 0.0));
        otos.setPhysicalMount(50.0, 0.0, 0.0);
        driveStraight(otos, 0.0, 0.0, 300.0, 0.0, 0.0, 15);
        spinInPlace(otos, 300.0, 0.0, 0.0, Math.PI / 2, 10);
        assertTrue(otos.getPosition().x > 290.0);

        otos.resetTracking();
        Pose2D pose = new Pose2D();
        Pose2D velocity = new Pose2D();
        Pose2D acceleration = new Pose2D();
        otos.getPosVelAcc(pose, velocity, acceleration);
        assertEquals(0.0, pose.x, 0.0);
        assertEquals(0.0, pose.y, 0.0);
        assertEquals(0.0, pose.h, 0.0);
        assertEquals(0.0, velocity.x, 0.0);
        assertEquals(0.0, acceleration.h, 0.0);
        assertEquals(50.0, otos.getOffset().x, 0.31, "reset keeps the offset");

        // The chassis now faces +Y in the plant, but the sensor's new frame starts at heading 0: forward is +X.
        driveStraight(otos, 300.0, 0.0, 300.0, 200.0, Math.PI / 2, 10);
        assertEquals(200.0, otos.getPosition().x, 0.5);
        assertEquals(0.0, otos.getPosition().y, 0.5);
    }

    @Test
    void invariant_setPositionReBasesTheFrameAndTrackingCarriesOnFromThere() {
        FakeSparkFunOTOS otos = new FakeSparkFunOTOS();
        otos.setPosition(new Pose2D(24.0, -12.0, 90.0));
        Pose2D written = otos.getPosition();
        assertEquals(24.0, written.x, POSITION_COUNT_INCHES);
        assertEquals(-12.0, written.y, POSITION_COUNT_INCHES);
        assertEquals(90.0, written.h, HEADING_COUNT_DEGREES);

        // The robot drives 10 inches forward; the sensor thinks it faces +Y, so the pose moves along +Y.
        driveStraight(otos, 0.0, 0.0, 10.0 * MM_PER_INCH, 0.0, 0.0, 10);
        Pose2D pose = otos.getPosition();
        assertEquals(24.0, pose.x, 2 * POSITION_COUNT_INCHES);
        assertEquals(-2.0, pose.y, 2 * POSITION_COUNT_INCHES);
        assertEquals(90.0, pose.h, 2 * HEADING_COUNT_DEGREES);
    }

    @Test
    void invariant_aWrittenPoseIsNotMotion() {
        FakeSparkFunOTOS otos = new FakeSparkFunOTOS();
        step(otos, 0.0, 0.0, 0.0);
        otos.setPosition(new Pose2D(40.0, 30.0, -135.0));
        step(otos, 0.0, 0.0, 0.0);                        // standing still across the write
        Pose2D velocity = otos.getVelocity();
        Pose2D acceleration = otos.getAcceleration();
        assertEquals(0.0, velocity.x, 1e-9);
        assertEquals(0.0, velocity.y, 1e-9);
        assertEquals(0.0, velocity.h, 1e-9);
        assertEquals(0.0, acceleration.x, 1e-9);
        assertEquals(0.0, acceleration.h, 1e-9);
    }

    // =================================================================================================
    // Velocity and acceleration
    // =================================================================================================

    @Test
    void invariant_velocityIsThePosesRateOfChangeFromAMovingPlant() {
        Robot robot = new Robot(new FakeSparkFunOTOS());
        double previousX = 0.0;
        for (int k = 0; k < 30; k++) {                     // short of the plant's field wall
            previousX = robot.chassis.getX();
            robot.tick(1, 1, 1, 1);
        }
        double expectedInchesPerSecond = (robot.chassis.getX() - previousX) / DT;
        assertTrue(expectedInchesPerSecond > 5.0, "the chassis really drove forward");
        Pose2D velocity = robot.otos.getVelocity();
        assertEquals(expectedInchesPerSecond, velocity.x, 0.01, "in/s, one velocity count is 0.006 in/s");
        assertEquals(0.0, velocity.y, 0.01, "no sideways motion");
        assertEquals(0.0, velocity.h, 1e-9, "no turn");
    }

    @Test
    void invariant_velocityAndAccelerationAreInTheFieldFrameOfThePose() {
        FakeSparkFunOTOS otos = new FakeSparkFunOTOS();
        otos.setLinearUnit(DistanceUnit.METER);
        otos.setPosition(new Pose2D(0.0, 0.0, 90.0));   // the sensor thinks the robot faces +Y

        // From rest, 10 mm forward per 20 ms tick: 0.5 m/s, reached in one tick.
        step(otos, 10.0, 0.0, 0.0);
        Pose2D velocity = otos.getVelocity();
        Pose2D acceleration = otos.getAcceleration();
        assertEquals(0.0, velocity.x, 2.0 / 6553.6, "the robot drives along field +Y, not +X");
        assertEquals(0.5, velocity.y, 2.0 / 6553.6, "m/s");
        assertEquals(0.5 / DT, acceleration.y, 2.0 / 208.8, "0 to 0.5 m/s in one tick: 25 m/s^2");

        step(otos, 20.0, 0.0, 0.0);
        assertEquals(0.5, otos.getVelocity().y, 2.0 / 6553.6);
        assertEquals(0.0, otos.getAcceleration().y, 2.0 / 208.8, "steady speed: no acceleration");
    }

    @Test
    void invariant_angularRatesReadThroughTheDriversNormalisingAngleUnit() {
        FakeSparkFunOTOS otos = new FakeSparkFunOTOS();
        double radiansPerTick = 1.0 * DT;                   // 1 rad/s
        for (int k = 1; k <= 5; k++) {
            step(otos, 0.0, 0.0, k * radiansPerTick);
        }
        assertEquals(Math.toDegrees(1.0), otos.getVelocity().h, 0.1, "deg/s, CCW positive");

        // 4 rad/s = 229 deg/s. The SDK driver converts through AngleUnit.fromRadians, which wraps into
        // [-180, 180): so a real OTOS spinning this fast reads -131 deg/s. Reproduced, not fixed.
        FakeSparkFunOTOS fast = new FakeSparkFunOTOS();
        for (int k = 1; k <= 5; k++) {
            step(fast, 0.0, 0.0, k * 4.0 * DT);
        }
        assertEquals(Math.toDegrees(4.0) - 360.0, fast.getVelocity().h, 0.1);
        fast.setAngularUnit(AngleUnit.RADIANS);
        assertEquals(4.0 - 2.0 * Math.PI, fast.getVelocity().h, 0.002, "and in radians it wraps at pi");
    }

    @Test
    void invariant_theCombinedReadsMatchTheSeparateOnesInOneBusRead() {
        FakeSparkFunOTOS otos = new FakeSparkFunOTOS();
        for (int k = 1; k <= 20; k++) {
            step(otos, 8.0 * k * k / 20.0, -3.0 * k, 0.02 * k);
        }
        Pose2D pose = new Pose2D();
        Pose2D velocity = new Pose2D();
        Pose2D acceleration = new Pose2D();
        int readsBefore = otos.busReadCount();
        otos.getPosVelAcc(pose, velocity, acceleration);
        assertEquals(readsBefore + 1, otos.busReadCount(), "one 18-byte read, as the driver makes it");
        assertPoseEquals(otos.getPosition(), pose);
        assertPoseEquals(otos.getVelocity(), velocity);
        assertPoseEquals(otos.getAcceleration(), acceleration);

        Pose2D[] all = {new Pose2D(), new Pose2D(), new Pose2D(), new Pose2D(), new Pose2D(), new Pose2D()};
        otos.getPosVelAccAndStdDev(all[0], all[1], all[2], all[3], all[4], all[5]);
        assertPoseEquals(pose, all[0]);
        assertPoseEquals(velocity, all[1]);
        assertPoseEquals(acceleration, all[2]);
        for (int i = 3; i < 6; i++) {
            assertPoseEquals(new Pose2D(0.0, 0.0, 0.0), all[i]);
        }
    }

    @Test
    void placeholder_theStandardDeviationsAreZeroBecauseTheSimHasNoNoise() {
        FakeSparkFunOTOS otos = new FakeSparkFunOTOS();
        driveStraight(otos, 0.0, 0.0, 500.0, 100.0, 0.0, 10);
        assertPoseEquals(new Pose2D(0.0, 0.0, 0.0), otos.getPositionStdDev());
        assertPoseEquals(new Pose2D(0.0, 0.0, 0.0), otos.getVelocityStdDev());
        assertPoseEquals(new Pose2D(0.0, 0.0, 0.0), otos.getAccelerationStdDev());
        Pose2D p = new Pose2D(1, 1, 1);
        Pose2D v = new Pose2D(1, 1, 1);
        Pose2D a = new Pose2D(1, 1, 1);
        otos.getPosVelAccStdDev(p, v, a);
        assertPoseEquals(new Pose2D(0.0, 0.0, 0.0), p);
        assertPoseEquals(new Pose2D(0.0, 0.0, 0.0), a);
    }

    private static void assertPoseEquals(Pose2D expected, Pose2D actual) {
        assertEquals(expected.x, actual.x, 0.0, "x");
        assertEquals(expected.y, actual.y, 0.0, "y");
        assertEquals(expected.h, actual.h, 0.0, "h");
    }

    // =================================================================================================
    // IMU calibration
    // =================================================================================================

    @Test
    void invariant_aBlockingCalibrationFinishesInsideTheCallAndTakesNoSimTime() {
        FakeSparkFunOTOS otos = new FakeSparkFunOTOS();
        assertTrue(otos.calibrateImu(), "255 samples, waiting: the SDK's default");
        assertEquals(0, otos.getImuCalibrationProgress());
        assertTrue(otos.calibrateImu(20, true));
        assertEquals(0, otos.getImuCalibrationProgress());
        // No update() was called: the whole calibration happened inside the driver's own poll loop.
        step(otos, 100.0, 0.0, 0.0);
        otos.setLinearUnit(DistanceUnit.MM);
        assertEquals(100.0, otos.getPosition().x, 0.5, "tracking straight after");
    }

    @Test
    void invariant_aNonBlockingCalibrationCountsDownInSimTime() {
        FakeSparkFunOTOS otos = new FakeSparkFunOTOS();
        assertTrue(otos.calibrateImu(100, false));
        // The driver slept 3 ms after the write: one 2.4 ms sample is done.
        assertEquals(99, otos.getImuCalibrationProgress());
        otos.update(0.1);                                   // 103 ms: 42 samples
        assertEquals(58, otos.getImuCalibrationProgress());
        assertEquals(58, otos.getImuCalibrationProgress(), "a loop's own polls take no device time");
        otos.update(0.1);                                   // 203 ms: 84 samples
        assertEquals(16, otos.getImuCalibrationProgress());
        otos.update(0.1);                                   // 303 ms: done
        assertEquals(0, otos.getImuCalibrationProgress());
    }

    @Test
    void invariant_calibrationSampleCountsOutsideTheSdkRangeAreRefused() {
        FakeSparkFunOTOS otos = new FakeSparkFunOTOS();
        assertFalse(otos.calibrateImu(0, false));
        assertFalse(otos.calibrateImu(256, false));
        assertEquals(0, otos.getImuCalibrationProgress(), "nothing started");
    }

    @Test
    void placeholder_thePoseIgnoresMotionWhileCalibrating() {
        FakeSparkFunOTOS otos = new FakeSparkFunOTOS();
        otos.setLinearUnit(DistanceUnit.MM);
        assertTrue(otos.calibrateImu(255, false));
        step(otos, 100.0, 0.0, 0.0);
        assertEquals(0.0, otos.getPosition().x, 0.0, "not tracking while calibrating");
        for (int k = 0; k < 40; k++) {
            otos.update(DT);                                // 0.8 s: done
        }
        assertEquals(0, otos.getImuCalibrationProgress());
        step(otos, 200.0, 0.0, 0.0);
        assertEquals(100.0, otos.getPosition().x, 0.5, "tracking again");
    }

    // =================================================================================================
    // Status, self test, connection, version, signal processing
    // =================================================================================================

    @Test
    void invariant_statusFlagsDecodeThroughTheSdksStatusType() {
        FakeSparkFunOTOS otos = new FakeSparkFunOTOS();
        Status clean = otos.getStatus();
        assertFalse(clean.warnTiltAngle || clean.warnOpticalTracking || clean.errorPaa || clean.errorLsm);

        Status fault = new Status();
        fault.warnOpticalTracking = true;
        fault.errorLsm = true;
        otos.forceStatus(fault);
        Status read = otos.getStatus();
        assertTrue(read.warnOpticalTracking);
        assertTrue(read.errorLsm);
        assertFalse(read.warnTiltAngle);
        assertFalse(read.errorPaa);

        otos.clearForcedStatus();
        assertEquals(0, otos.getStatus().get());
        assertThrows(IllegalArgumentException.class, () -> otos.forceStatus(null));
    }

    @Test
    void invariant_beginIsConnectedAndTheVersionComeFromTheBus() {
        FakeSparkFunOTOS otos = new FakeSparkFunOTOS();
        assertTrue(otos.begin());
        assertTrue(otos.isConnected());
        Version hardware = new Version();
        Version firmware = new Version();
        otos.getVersionInfo(hardware, firmware);
        assertEquals(1, hardware.major);
        assertEquals(0, hardware.minor);
        assertEquals(1, firmware.major);
        assertEquals(0, firmware.minor);
        assertEquals(HardwareDevice.Manufacturer.SparkFun, otos.getManufacturer());
        assertEquals("SparkFun Qwiic Optical Tracking Odometry Sensor", otos.getDeviceName());
    }

    @Test
    void invariant_anUnpluggedSensorIsNotConnectedAndLosesWrites() {
        FakeSparkFunOTOS otos = new FakeSparkFunOTOS();
        otos.setConnected(false);
        assertFalse(otos.isConnected());
        assertFalse(otos.begin());
        assertTrue(otos.setLinearScalar(1.1), "the driver does not know the write was lost");
        otos.setConnected(true);
        assertTrue(otos.isConnected());
        assertEquals(1.0, otos.getLinearScalar(), 1e-12, "the write never arrived");
    }

    @Test
    void invariant_selfTestReportsTheSensorsResult() {
        FakeSparkFunOTOS otos = new FakeSparkFunOTOS();
        assertTrue(otos.selfTest());
        otos.setSelfTestResult(false);
        assertFalse(otos.selfTest());
    }

    @Test
    void invariant_theSignalProcessConfigRoundTrips() {
        FakeSparkFunOTOS otos = new FakeSparkFunOTOS();
        SignalProcessConfig config = otos.getSignalProcessConfig();
        assertTrue(config.enLut && config.enAcc && config.enRot && config.enVar, "PLACEHOLDER default: all on");
        config.enAcc = false;
        otos.setSignalProcessConfig(config);
        SignalProcessConfig read = otos.getSignalProcessConfig();
        assertFalse(read.enAcc);
        assertTrue(read.enLut && read.enRot && read.enVar);
    }

    @Test
    void invariant_busTrafficTheSdkDriverNeverMakesFailsLoud() {
        FakeSparkFunOTOS otos = new FakeSparkFunOTOS();
        I2cDeviceSynch bus = (I2cDeviceSynch) otos.getDeviceClient();
        assertTrue(bus.isEngaged(), "the driver's constructor engages the bus");
        assertThrows(UnsupportedOperationException.class, () -> bus.read(3, 1), "register 3 is not in the map");
        assertThrows(UnsupportedOperationException.class, () -> bus.write8(REGISTER_VELOCITY_X, 0));
        assertThrows(UnsupportedOperationException.class, () -> bus.write8(7, 2), "reset bit 1 is not used");
        assertThrows(UnsupportedOperationException.class, () -> bus.write(38, new byte[6]));
        assertThrows(UnsupportedOperationException.class, () -> bus.getHeartbeatInterval());
    }

    /** {@code REG_VEL_XL} in the SDK driver: a read-only register. */
    private static final int REGISTER_VELOCITY_X = 38;

    // =================================================================================================
    // Determinism and the no-override guarantee
    // =================================================================================================

    @Test
    void invariant_twoIdenticalRunsAreByteIdentical() throws IOException {
        assertArrayEquals(scriptedRunTranscript(), scriptedRunTranscript());
    }

    /** A fixed scenario - plant driving, a mis-set scalar, a reset, a re-base - read every tick, as raw bits. */
    private static byte[] scriptedRunTranscript() throws IOException {
        FakeSparkFunOTOS otos = new FakeSparkFunOTOS();
        otos.setPhysicalMount(120.0, 40.0, Math.toRadians(-30.0));
        otos.setPhysicalTrackingGains(0.98, 1.01);
        otos.setOffset(new Pose2D(4.5, 1.5, -30.0));
        otos.setLinearScalar(1.02);
        otos.calibrateImu(40, false);
        Robot robot = new Robot(otos);
        ByteArrayOutputStream bytes = new ByteArrayOutputStream();
        DataOutputStream out = new DataOutputStream(bytes);
        Pose2D p = new Pose2D();
        Pose2D v = new Pose2D();
        Pose2D a = new Pose2D();
        for (int k = 0; k < 200; k++) {
            if (k == 90) {
                otos.resetTracking();
            }
            if (k == 140) {
                otos.setPosition(new Pose2D(-10.0, 20.0, 45.0));
            }
            robot.tick(0.8 - 0.004 * k, 0.3 + 0.003 * k, -0.2 + 0.005 * k, 0.9 - 0.006 * k);
            otos.getPosVelAcc(p, v, a);
            for (double value : new double[] {p.x, p.y, p.h, v.x, v.y, v.h, a.x, a.y, a.h}) {
                out.writeLong(Double.doubleToRawLongBits(value));
            }
            out.writeInt(otos.getImuCalibrationProgress());
        }
        out.flush();
        return bytes.toByteArray();
    }

    @Test
    void invariant_theFakeOverridesNoMethodOfTheRealDriver() {
        List<String> overridden = new ArrayList<>();
        for (Method mine : FakeSparkFunOTOS.class.getDeclaredMethods()) {
            for (Class<?> sdk = SparkFunOTOS.class; sdk != null && sdk != Object.class; sdk = sdk.getSuperclass()) {
                try {
                    sdk.getDeclaredMethod(mine.getName(), mine.getParameterTypes());
                    overridden.add(sdk.getSimpleName() + "." + mine.getName() + Arrays.toString(mine.getParameterTypes()));
                } catch (NoSuchMethodException notTheDrivers) {
                    // the fake's own sim-side method: fine
                }
            }
        }
        assertTrue(overridden.isEmpty(), "FakeSparkFunOTOS overrides the real driver's " + overridden);
    }
}
