package org.horizon36596.simloop.fakehardware;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.DeviceStatus;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.EncoderDirection;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.ErrorDetectionType;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.GoBildaOdometryPods;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.ReadData;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.Register;
import com.qualcomm.robotcore.hardware.HardwareDevice;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.horizon36596.simloop.config.DrivetrainSimConfig;
import org.horizon36596.simloop.plant.MecanumDrivePlant;
import org.junit.jupiter.api.Test;

import java.lang.reflect.Method;
import java.lang.reflect.Modifier;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * Pins {@link FakeGoBildaPinpoint}: a moving chassis read through the REAL SDK 12.0 driver's public API,
 * and the device behaviours that API exposes - units, frames, signs, status values, update semantics,
 * reset and IMU recalibration, pose setters, bulk-read formats and CRC. SimLoop only: no season code.
 * INVARIANT tier throughout, except where a test says it pins one of this fake's own PLACEHOLDER choices.
 */
class FakeGoBildaPinpointTest {

    private static final double DT = 0.02;
    private static final double MM_PER_INCH = 25.4;

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

    /** A drive plant and the Pinpoint riding on it, attached at the plant's start. */
    private static final class Robot {
        final FakeMotor[] wheels = new FakeMotor[4];
        final MecanumDrivePlant chassis;
        final FakeGoBildaPinpoint pinpoint;

        Robot(FakeGoBildaPinpoint pinpoint) {
            DrivetrainSimConfig config = chassisConfig();
            for (int i = 0; i < 4; i++) {
                wheels[i] = new FakeMotor(config.maxVelocityTicksPerSecond(), config.maxAccel());
            }
            chassis = new MecanumDrivePlant(wheels, config);
            this.pinpoint = pinpoint;
            pinpoint.attachChassisAt(chassis.getX() * MM_PER_INCH, chassis.getY() * MM_PER_INCH, chassis.getHeading());
        }

        /** One loop: set the wheel powers [FL, FR, BL, BR], let the world move, then the robot reads. */
        void tick(double fl, double fr, double bl, double br) {
            double[] power = {fl, fr, bl, br};
            for (int i = 0; i < 4; i++) {
                wheels[i].setPower(power[i]);
                wheels[i].update(DT);
            }
            chassis.update(DT);
            pinpoint.moveChassisTo(chassis.getX() * MM_PER_INCH, chassis.getY() * MM_PER_INCH, chassis.getHeading());
            pinpoint.update(DT);          // the device runs
            pinpoint.update();            // the robot code reads it, through the real driver
        }
    }

    @Test
    void invariant_aMovingChassisReadsBackThroughTheRealDriverInItsUnitsAndFrame() {
        Robot robot = new Robot(new FakeGoBildaPinpoint());
        for (int k = 0; k < 150; k++) {
            if (k < 50) {
                robot.tick(1, 1, 1, 1);           // forward
            } else if (k < 100) {
                robot.tick(-1, 1, 1, -1);         // strafe left
            } else {
                robot.tick(-0.5, 0.5, -0.5, 0.5); // turn counter-clockwise
            }
            // The device is exact to its float registers: well under a millimetre and a milliradian.
            assertEquals(robot.chassis.getX(), robot.pinpoint.getPosX(DistanceUnit.INCH), 1e-3, "x at tick " + k);
            assertEquals(robot.chassis.getY(), robot.pinpoint.getPosY(DistanceUnit.INCH), 1e-3, "y at tick " + k);
            assertEquals(robot.chassis.getHeading(),
                    robot.pinpoint.getHeading(UnnormalizedAngleUnit.RADIANS), 1e-5, "heading at tick " + k);
        }
        assertTrue(robot.chassis.getX() > 10.0 && robot.chassis.getY() > 10.0 && robot.chassis.getHeading() > 0.5,
                "the chassis really went forward (+X), left (+Y) and counter-clockwise (+heading)");
        Pose2D pose = robot.pinpoint.getPosition();
        assertEquals(robot.chassis.getX() * MM_PER_INCH, pose.getX(DistanceUnit.MM), 0.1);
        assertEquals(DeviceStatus.READY, robot.pinpoint.getDeviceStatus());
    }

    @Test
    void invariant_velocityIsThePosesRateOfChangeInThePosesFrame() {
        Robot robot = new Robot(new FakeGoBildaPinpoint());
        double previousX = 0.0;
        for (int k = 0; k < 60; k++) {
            previousX = robot.chassis.getX() * MM_PER_INCH;
            robot.tick(1, 1, 1, 1);
        }
        double expected = (robot.chassis.getX() * MM_PER_INCH - previousX) / DT;
        assertEquals(expected, robot.pinpoint.getVelX(DistanceUnit.MM), 0.1, "mm/s over the last tick");
        assertEquals(expected / MM_PER_INCH, robot.pinpoint.getVelX(DistanceUnit.INCH), 0.01, "and in inches");
        assertEquals(0.0, robot.pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS), 1e-9);
        assertEquals(0.0, robot.pinpoint.getVelY(DistanceUnit.MM), 1e-6, "no sideways motion");

        Robot strafing = new Robot(new FakeGoBildaPinpoint());
        double previousY = 0.0;
        for (int k = 0; k < 30; k++) {
            previousY = strafing.chassis.getY() * MM_PER_INCH;
            strafing.tick(-1, 1, 1, -1);          // strafe left
        }
        double expectedY = (strafing.chassis.getY() * MM_PER_INCH - previousY) / DT;
        assertTrue(expectedY > 100.0, "the chassis really strafed left (+Y), at " + expectedY + " mm/s");
        assertEquals(expectedY, strafing.pinpoint.getVelY(DistanceUnit.MM), 0.1, "Y velocity, mm/s, left positive");
    }

    /** A pose written by the robot is not motion: the next tick reports no velocity and no bad read. */
    @Test
    void invariant_writingAPoseIsNotReportedAsVelocity() {
        Robot robot = new Robot(new FakeGoBildaPinpoint());
        robot.tick(0, 0, 0, 0);
        robot.pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 20.0, -10.0, AngleUnit.DEGREES, 90.0));
        robot.pinpoint.setPosY(-300.0, DistanceUnit.MM);
        robot.tick(0, 0, 0, 0);                   // the robot stands still across the write

        assertEquals(0.0, robot.pinpoint.getVelX(DistanceUnit.MM), 1e-6);
        assertEquals(0.0, robot.pinpoint.getVelY(DistanceUnit.MM), 1e-6);
        assertEquals(0.0, robot.pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS), 1e-9);
        assertEquals(DeviceStatus.READY, robot.pinpoint.getDeviceStatus(), "the write is not a corrupt read");
        assertEquals(20.0 * MM_PER_INCH, robot.pinpoint.getPosX(DistanceUnit.MM), 1e-3);
        assertEquals(-300.0, robot.pinpoint.getPosY(DistanceUnit.MM), 1e-3, "setPosY, the last write, wins");
    }

    /** The device integrates motion onto the pose it was given, so a written pose sticks. */
    @Test
    void invariant_aWrittenPoseSticksAndMotionIsAddedOnTopOfIt() {
        Robot robot = new Robot(new FakeGoBildaPinpoint());
        robot.pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 20.0, -10.0, AngleUnit.DEGREES, 90.0));
        robot.pinpoint.update();
        assertEquals(20.0, robot.pinpoint.getPosX(DistanceUnit.INCH), 1e-4);
        assertEquals(90.0, robot.pinpoint.getHeading(AngleUnit.DEGREES), 1e-4);

        for (int k = 0; k < 40; k++) {
            robot.tick(1, 1, 1, 1);               // the chassis drives forward in its own frame...
        }
        double travelled = robot.chassis.getX();
        // ...and the device, facing +90 degrees, sees that as +Y travel from where it was told it was.
        assertEquals(20.0, robot.pinpoint.getPosX(DistanceUnit.INCH), 1e-3);
        assertEquals(-10.0 + travelled, robot.pinpoint.getPosY(DistanceUnit.INCH), 1e-3);
    }

    @Test
    void invariant_headingIsWrappedForAngleUnitAndContinuousForUnnormalizedAngleUnit() {
        FakeGoBildaPinpoint pinpoint = new FakeGoBildaPinpoint();
        for (int quarter = 1; quarter <= 3; quarter++) {             // three quarters of a turn, CCW
            pinpoint.moveChassisTo(0.0, 0.0, Math.toRadians(90.0 * quarter));
        }
        pinpoint.update();

        assertEquals(-90.0, pinpoint.getHeading(AngleUnit.DEGREES), 1e-3, "wrapped into [-180, 180)");
        assertEquals(270.0, pinpoint.getHeading(UnnormalizedAngleUnit.DEGREES), 1e-3, "not wrapped");
    }

    /** The SDK's {@code AngleUnit} setters wrap, so a heading written as 270 degrees is stored as -90. */
    @Test
    void invariant_aHeadingWrittenThroughAngleUnitIsWrappedBeforeTheDeviceSeesIt() {
        FakeGoBildaPinpoint pinpoint = new FakeGoBildaPinpoint();
        pinpoint.setHeading(270.0, AngleUnit.DEGREES);
        pinpoint.update();

        assertEquals(-90.0, pinpoint.getHeading(UnnormalizedAngleUnit.DEGREES), 1e-3);
    }

    @Test
    void invariant_recalibratingReportsCalibratingForItsTimeAndKeepsThePose() {
        Robot robot = new Robot(new FakeGoBildaPinpoint());
        for (int k = 0; k < 20; k++) {
            robot.tick(1, 1, 1, 1);
        }
        double before = robot.pinpoint.getPosX(DistanceUnit.MM);

        robot.pinpoint.recalibrateIMU();
        robot.pinpoint.update();
        assertEquals(DeviceStatus.CALIBRATING, robot.pinpoint.getDeviceStatus());
        assertEquals(before, robot.pinpoint.getPosX(DistanceUnit.MM), 1e-3, "recalibrating does not move the pose");

        int ticks = (int) Math.ceil(FakeGoBildaPinpoint.CALIBRATION_SECONDS / DT);
        for (int k = 0; k < ticks; k++) {
            robot.tick(0, 0, 0, 0);
        }
        assertEquals(DeviceStatus.READY, robot.pinpoint.getDeviceStatus());
    }

    /**
     * A calibration of 0.25 s is over after five 0.05 s ticks, not six: 0.25 - 5 * 0.05 is 1.4e-17 in
     * doubles, and that rounding must not keep the device calibrating for a whole extra tick.
     */
    @Test
    void invariant_aCalibrationEndsOnTheTickItsTimeRunsOutDespiteFloatingPointRounding() {
        FakeGoBildaPinpoint pinpoint = new FakeGoBildaPinpoint();
        pinpoint.recalibrateIMU();
        for (int k = 0; k < 5; k++) {
            pinpoint.update();
            assertEquals(DeviceStatus.CALIBRATING, pinpoint.getDeviceStatus(), "before tick " + (k + 1));
            pinpoint.update(0.05);
        }
        pinpoint.update();
        assertEquals(DeviceStatus.READY, pinpoint.getDeviceStatus(), "after 5 x 0.05 s = 0.25 s");
    }

    /**
     * A drive plant that wraps its heading jumps from about +pi to about -pi when it turns through the back.
     * That is a small turn, not most of a turn the other way: the device's heading keeps counting, and the
     * driver sees no velocity spike and no bad read.
     */
    @Test
    void invariant_aPlantHeadingThatWrapsThroughPiIsASmallTurnNotAJump() {
        FakeGoBildaPinpoint pinpoint = new FakeGoBildaPinpoint();
        pinpoint.attachChassisAt(0.0, 0.0, Math.PI - 0.01);
        pinpoint.update();
        pinpoint.moveChassisTo(0.0, 0.0, -Math.PI + 0.01);   // turned 0.02 rad CCW, through +-pi
        pinpoint.update(DT);
        pinpoint.update();

        assertEquals(0.02, pinpoint.getHeading(UnnormalizedAngleUnit.RADIANS), 1e-6);
        assertEquals(0.02 / DT, pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS), 1e-4);
        assertEquals(DeviceStatus.READY, pinpoint.getDeviceStatus());
    }

    @Test
    void invariant_resetZeroesThePoseAndIgnoresMotionWhileItCalibrates() {
        Robot robot = new Robot(new FakeGoBildaPinpoint());
        for (int k = 0; k < 20; k++) {
            robot.tick(1, 1, 1, 1);
        }

        assertTrue(robot.pinpoint.getVelX(DistanceUnit.MM) > 100.0, "it was moving before the reset");

        robot.pinpoint.resetPosAndIMU();
        robot.pinpoint.update();                  // read straight after the reset, before the device ticks
        assertEquals(0.0, robot.pinpoint.getPosX(DistanceUnit.MM), 1e-6);
        assertEquals(0.0, robot.pinpoint.getVelX(DistanceUnit.MM), 1e-6, "the reset zeroes the velocity too");

        int countsBefore = robot.pinpoint.getEncoderX();
        robot.tick(1, 1, 1, 1);                   // moving while it calibrates: the pose does not track it
        assertEquals(DeviceStatus.CALIBRATING, robot.pinpoint.getDeviceStatus());
        assertEquals(0.0, robot.pinpoint.getPosX(DistanceUnit.MM), 1e-6);
        assertEquals(0.0, robot.pinpoint.getHeading(AngleUnit.RADIANS), 1e-9);
        assertTrue(robot.pinpoint.getEncoderX() > countsBefore, "but the pod still counts");
    }

    /** The raw counts are what the PHYSICAL pods make: their resolution, their direction, where they sit. */
    @Test
    void invariant_encoderCountsFollowThePhysicalPodsResolutionDirectionAndOffset() {
        FakeGoBildaPinpoint pinpoint = new FakeGoBildaPinpoint();
        pinpoint.setPhysicalPodMount(0.0, 0.0, EncoderDirection.FORWARD, EncoderDirection.FORWARD, 13.26291192);
        pinpoint.setEncoderResolution(GoBildaOdometryPods.goBILDA_SWINGARM_POD);   // configured to match

        pinpoint.moveChassisTo(100.0, 0.0, 0.0);      // 100 mm forward
        pinpoint.update();
        assertEquals(Math.round(100.0 * 13.262912), pinpoint.getEncoderX(), 1.0, "swingarm pod: 13.26 ticks/mm");
        assertEquals(100.0, pinpoint.getPosX(DistanceUnit.MM), 1e-3, "configured to match: the chassis's pose");

        pinpoint.setEncoderDirections(EncoderDirection.REVERSED, EncoderDirection.FORWARD);
        pinpoint.moveChassisTo(200.0, 0.0, 0.0);      // another 100 mm forward, the X pod now configured reversed
        pinpoint.update();
        assertEquals(0.0, pinpoint.getEncoderX(), 1.0, "a pod configured reversed counts the other way");
        assertEquals(0.0, pinpoint.getPosX(DistanceUnit.MM), 1e-3,
                "and the device reads those 100 mm as backwards: 100 forward, then 100 back");

        FakeGoBildaPinpoint spinning = new FakeGoBildaPinpoint();
        // X pod 100 mm right of centre, Y pod 50 mm forward, 1 tick/mm; configured the same way.
        spinning.setPhysicalPodMount(-100.0, 50.0, EncoderDirection.FORWARD, EncoderDirection.FORWARD, 1.0);
        spinning.setEncoderResolution(1.0, DistanceUnit.MM);
        spinning.setOffsets(-100.0, 50.0, DistanceUnit.MM);
        spinning.moveChassisTo(0.0, 0.0, 0.5);        // turn in place, 0.5 rad CCW
        spinning.update();
        assertEquals(50, spinning.getEncoderX(), "a pod right of centre rolls forward in a CCW turn");
        assertEquals(25, spinning.getEncoderY(), "a pod ahead of centre rolls left in a CCW turn");
        assertEquals(0.0, spinning.getPosX(DistanceUnit.MM), 1e-3, "configured offsets take the turn back out");
        assertEquals(0.0, spinning.getPosY(DistanceUnit.MM), 1e-3);
        assertEquals(-100.0f, spinning.getXOffset(DistanceUnit.MM), 1e-4);
        assertEquals(50.0f, spinning.getYOffset(DistanceUnit.MM), 1e-4);
    }

    /**
     * Pods mounted off-centre with one counting backwards, and a robot that configures all of it correctly,
     * read the chassis exactly while it drives forward, strafes and turns - the arc steps included.
     */
    @Test
    void invariant_aCorrectlyConfiguredOffCentreMountTracksTheChassis() {
        FakeGoBildaPinpoint pinpoint = new FakeGoBildaPinpoint();
        pinpoint.setPhysicalPodMount(-80.0, 120.0, EncoderDirection.REVERSED, EncoderDirection.FORWARD, 13.26291192);
        pinpoint.setOffsets(-80.0, 120.0, DistanceUnit.MM);
        pinpoint.setEncoderDirections(EncoderDirection.REVERSED, EncoderDirection.FORWARD);
        pinpoint.setEncoderResolution(GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        Robot robot = new Robot(pinpoint);
        for (int k = 0; k < 150; k++) {
            if (k < 50) {
                robot.tick(1, 1, 1, 1);               // forward
            } else if (k < 100) {
                robot.tick(-1, 1, 1, -1);             // strafe left
            } else {
                robot.tick(0.2, 1, 0.2, 1);           // forward while turning CCW: arcs
            }
        }
        assertTrue(robot.chassis.getHeading() > 0.5, "the chassis really turned, to " + robot.chassis.getHeading());
        assertEquals(robot.chassis.getX(), pinpoint.getPosX(DistanceUnit.INCH), 1e-3);
        assertEquals(robot.chassis.getY(), pinpoint.getPosY(DistanceUnit.INCH), 1e-3);
        assertEquals(robot.chassis.getHeading(), pinpoint.getHeading(UnnormalizedAngleUnit.RADIANS), 1e-5);
    }

    /** The most common Pinpoint setup bug: an X pod that counts down, and a robot that did not reverse it. */
    @Test
    void invariant_aPodDirectionConfiguredWrongReadsTheMotionBackwards() {
        FakeGoBildaPinpoint pinpoint = new FakeGoBildaPinpoint();
        pinpoint.setPhysicalPodMount(0.0, 0.0, EncoderDirection.REVERSED, EncoderDirection.FORWARD,
                FakeGoBildaPinpoint.DEFAULT_TICKS_PER_MM);
        pinpoint.moveChassisTo(500.0, 0.0, 0.0);      // 500 mm forward; the robot left the direction FORWARD
        pinpoint.update();
        assertEquals(-500.0, pinpoint.getPosX(DistanceUnit.MM), 1e-3, "it reads 500 mm backwards");

        pinpoint.setEncoderDirections(EncoderDirection.REVERSED, EncoderDirection.FORWARD);
        pinpoint.moveChassisTo(1000.0, 0.0, 0.0);     // another 500 mm forward, now configured right
        pinpoint.update();
        assertEquals(0.0, pinpoint.getPosX(DistanceUnit.MM), 1e-3, "and from then on reads it forwards");
    }

    /**
     * An X pod really 100 mm left of centre, configured as on the centre: turning in place, the device
     * reads the pod's roll as driving, and the pose swings round a 100 mm circle.
     */
    @Test
    void invariant_aPodOffsetConfiguredWrongMakesTheRobotDriftWhileItTurns() {
        FakeGoBildaPinpoint pinpoint = new FakeGoBildaPinpoint();
        pinpoint.setPhysicalPodMount(100.0, 0.0, EncoderDirection.FORWARD, EncoderDirection.FORWARD,
                FakeGoBildaPinpoint.DEFAULT_TICKS_PER_MM);
        for (int step = 1; step <= 10; step++) {
            pinpoint.moveChassisTo(0.0, 0.0, step * (Math.PI / 2.0) / 10.0);   // a quarter turn in place
        }
        pinpoint.update();
        // The pod rolls backwards 100 mm per radian: x = -100 sin(h), y = -100 (1 - cos(h)), exact on arcs.
        assertEquals(-100.0, pinpoint.getPosX(DistanceUnit.MM), 1e-3);
        assertEquals(-100.0, pinpoint.getPosY(DistanceUnit.MM), 1e-3);
        assertEquals(Math.PI / 2.0, pinpoint.getHeading(UnnormalizedAngleUnit.RADIANS), 1e-5, "the IMU is right");

        pinpoint.setOffsets(100.0, 0.0, DistanceUnit.MM);
        pinpoint.moveChassisTo(0.0, 0.0, Math.PI / 2.0 + 1.0);   // another radian, now configured right
        pinpoint.update();
        assertEquals(-100.0, pinpoint.getPosX(DistanceUnit.MM), 1e-3, "no more drift");
        assertEquals(-100.0, pinpoint.getPosY(DistanceUnit.MM), 1e-3);
    }

    /** Swingarm pods, and a robot that left the resolution at the 4-bar value: every distance reads short. */
    @Test
    void invariant_aResolutionConfiguredWrongScalesEveryDistance() {
        FakeGoBildaPinpoint pinpoint = new FakeGoBildaPinpoint();
        pinpoint.setPhysicalPodMount(0.0, 0.0, EncoderDirection.FORWARD, EncoderDirection.FORWARD, 13.26291192);
        pinpoint.moveChassisTo(300.0, 0.0, 0.0);
        pinpoint.update();
        assertEquals(300.0 * 13.26291192 / 19.89436789, pinpoint.getPosX(DistanceUnit.MM), 1e-2, "two thirds");
    }

    @Test
    void invariant_theYawScalarScalesEveryHeadingChangeAsTheDeviceDoes() {
        FakeGoBildaPinpoint pinpoint = new FakeGoBildaPinpoint();
        pinpoint.setYawScalar(1.01);
        pinpoint.moveChassisTo(0.0, 0.0, 1.0);
        pinpoint.update();

        assertEquals(1.01, pinpoint.getHeading(UnnormalizedAngleUnit.RADIANS), 1e-5);
        assertEquals(1.01f, pinpoint.getYawScalar(), 1e-6);
    }

    /**
     * The real driver's own corruption filter runs: a reading that jumps more than 5000 mm since the last
     * one is refused, the old value kept, and the status set to FAULT_BAD_READ. That is what the SDK does
     * after a {@code setPosX} of more than five metres - a real quirk, reproduced because it is the real code.
     */
    @Test
    void invariant_theRealDriversCorruptionFilterRunsOnTheFakesReadings() {
        FakeGoBildaPinpoint pinpoint = new FakeGoBildaPinpoint();
        pinpoint.update();
        pinpoint.setPosX(6000.0, DistanceUnit.MM);
        pinpoint.update();

        assertEquals(0.0, pinpoint.getPosX(DistanceUnit.MM), 1e-6, "the 6 m jump is refused");
        assertEquals(DeviceStatus.FAULT_BAD_READ, pinpoint.getDeviceStatus());
    }

    @Test
    void invariant_firmwareOneAndTwoUseTheFixedBulkReadAndRefuseCrcScopesAndOrientation() {
        for (int version = 1; version <= 2; version++) {
            FakeGoBildaPinpoint pinpoint = new FakeGoBildaPinpoint(version);
            pinpoint.moveChassisTo(10.0, 0.0, 0.0);
            pinpoint.update();
            assertEquals(10.0, pinpoint.getPosX(DistanceUnit.MM), 1e-4, "firmware " + version);
            assertEquals(version, pinpoint.getDeviceVersion());

            assertThrows(RuntimeException.class, () -> pinpoint.setBulkReadScope(Register.X_POSITION),
                    "the driver refuses a bulk-read scope below firmware 3");
            assertThrows(RuntimeException.class, () -> pinpoint.getPitch(AngleUnit.DEGREES),
                    "and pitch, which firmware " + version + " does not have");
            pinpoint.setErrorDetectionType(ErrorDetectionType.CRC);
            RuntimeException crc = assertThrows(RuntimeException.class, pinpoint::update);
            assertTrue(crc.getMessage().contains("CRC"), crc.getMessage());
        }
    }

    /**
     * Pins this fake's NEEDS MEASUREMENT stand-in for the byte old firmware sends where a CRC would be: the
     * driver checks it on every single-register read, so on firmware 2 a register read marks the status bad
     * until the next bulk read. Whether a real v2 unit does this is not known.
     */
    @Test
    void placeholder_onFirmwareTwoASingleRegisterReadFailsTheDriversCrcCheck() {
        FakeGoBildaPinpoint pinpoint = new FakeGoBildaPinpoint(2);
        pinpoint.update();
        assertEquals(DeviceStatus.READY, pinpoint.getDeviceStatus());

        assertEquals(1.0f, pinpoint.getYawScalar(), 1e-6, "the value itself still gets through");
        assertEquals(DeviceStatus.FAULT_BAD_READ, pinpoint.getDeviceStatus());
        pinpoint.update();
        assertEquals(DeviceStatus.READY, pinpoint.getDeviceStatus(), "the next bulk read overwrites it");
    }

    /**
     * Pins this fake's PLACEHOLDER: firmware 1 and 2 are taken to have no quaternion, pitch or roll
     * registers. The real driver reads the register anyway when it has not yet learnt the version; what a
     * real old unit answers is not known, so the fake fails loud rather than invent an answer.
     */
    @Test
    void placeholder_oldFirmwareHasNoOrientationRegistersEvenBeforeTheDriverKnowsItsVersion() {
        FakeGoBildaPinpoint pinpoint = new FakeGoBildaPinpoint(2);
        assertThrows(UnsupportedOperationException.class, () -> pinpoint.getRoll(AngleUnit.DEGREES));
    }

    @Test
    void invariant_firmwareThreeAnswersCrcCheckedAndScopedBulkReads() {
        FakeGoBildaPinpoint pinpoint = new FakeGoBildaPinpoint(3);
        pinpoint.setErrorDetectionType(ErrorDetectionType.CRC);
        pinpoint.moveChassisTo(10.0, 5.0, 0.2);
        pinpoint.update(DT);
        pinpoint.update();
        assertEquals(10.0, pinpoint.getPosX(DistanceUnit.MM), 1e-4, "a CRC-checked bulk read gets through");
        assertEquals(DeviceStatus.READY, pinpoint.getDeviceStatus(), "and its CRC checks out");

        pinpoint.setBulkReadScope(Register.DEVICE_STATUS, Register.LOOP_TIME, Register.X_POSITION);
        pinpoint.moveChassisTo(20.0, 8.0, 0.2);    // the device and the plant share an origin here
        pinpoint.update();
        assertEquals(20.0, pinpoint.getPosX(DistanceUnit.MM), 1e-3, "a register in the scope is refreshed by update()");
        assertEquals(8.0, pinpoint.getPosY(DistanceUnit.MM), 1e-3, "one outside it is read on demand, so it is fresh too");
    }

    @Test
    void invariant_onlyUpdateHeadingRefreshesTheHeadingAlone() {
        FakeGoBildaPinpoint pinpoint = new FakeGoBildaPinpoint();
        pinpoint.update();
        pinpoint.moveChassisTo(50.0, 0.0, 0.3);
        pinpoint.update(ReadData.ONLY_UPDATE_HEADING);

        assertEquals(0.3, pinpoint.getHeading(UnnormalizedAngleUnit.RADIANS), 1e-6);
        assertEquals(0.0, pinpoint.getPosX(DistanceUnit.MM), 1e-6, "x is still the last full read's");
    }

    @Test
    void invariant_getterReadsAreCachedUntilUpdateAsOnTheRobot() {
        FakeGoBildaPinpoint pinpoint = new FakeGoBildaPinpoint();
        pinpoint.update();
        pinpoint.moveChassisTo(30.0, 0.0, 0.0);

        assertEquals(0.0, pinpoint.getPosX(DistanceUnit.MM), 1e-6, "nothing new until the driver reads");
        pinpoint.update();
        assertEquals(30.0, pinpoint.getPosX(DistanceUnit.MM), 1e-4);
    }

    @Test
    void invariant_aForcedFaultDecodesToThatStatus() {
        FakeGoBildaPinpoint pinpoint = new FakeGoBildaPinpoint();
        for (DeviceStatus status : DeviceStatus.values()) {
            if (status == DeviceStatus.FAULT_BAD_READ) {
                continue;                                // the driver's status, not the device's: see below
            }
            pinpoint.forceFault(status);
            pinpoint.update();
            assertEquals(status, pinpoint.getDeviceStatus(), "forced " + status);
        }
        pinpoint.clearFault();
        pinpoint.update();
        assertEquals(DeviceStatus.READY, pinpoint.getDeviceStatus());
        assertThrows(IllegalArgumentException.class, () -> pinpoint.forceFault(DeviceStatus.FAULT_BAD_READ),
                "the device never reports a bad read; the driver decides that");
    }

    /** A corrupt bulk read is caught by the real driver's default local test: NaN refused, status bad. */
    @Test
    void invariant_aCorruptBulkReadIsCaughtByTheRealDriversLocalTest() {
        FakeGoBildaPinpoint pinpoint = new FakeGoBildaPinpoint();
        pinpoint.moveChassisTo(40.0, 0.0, 0.0);
        pinpoint.update();
        pinpoint.moveChassisTo(80.0, 0.0, 0.0);
        pinpoint.corruptNextBulkRead();
        pinpoint.update();

        assertEquals(40.0, pinpoint.getPosX(DistanceUnit.MM), 1e-4, "the NaN is refused; the last good value stays");
        assertEquals(DeviceStatus.FAULT_BAD_READ, pinpoint.getDeviceStatus());
        pinpoint.update();
        assertEquals(80.0, pinpoint.getPosX(DistanceUnit.MM), 1e-4, "only the one read was corrupt");
        assertEquals(DeviceStatus.READY, pinpoint.getDeviceStatus());
    }

    /** With CRC detection the driver discards the whole corrupt read, integer registers too. */
    @Test
    void invariant_aCorruptBulkReadFailsTheRealDriversCrcCheck() {
        FakeGoBildaPinpoint pinpoint = new FakeGoBildaPinpoint(3);
        pinpoint.setErrorDetectionType(ErrorDetectionType.CRC);
        pinpoint.moveChassisTo(40.0, 0.0, 0.0);
        pinpoint.update();
        int countsBefore = pinpoint.getEncoderX();
        pinpoint.moveChassisTo(80.0, 0.0, 0.0);
        pinpoint.corruptNextBulkRead();
        pinpoint.update();

        assertEquals(40.0, pinpoint.getPosX(DistanceUnit.MM), 1e-4);
        assertEquals(countsBefore, pinpoint.getEncoderX(), "nothing from the bad read is kept");
        assertEquals(DeviceStatus.FAULT_BAD_READ, pinpoint.getDeviceStatus());
    }

    /** Firmware 1 and 2: the fixed bulk read, no CRC; the driver's local test still catches it. */
    @Test
    void invariant_aCorruptFixedBulkReadIsCaughtOnOldFirmware() {
        FakeGoBildaPinpoint pinpoint = new FakeGoBildaPinpoint(2);
        pinpoint.moveChassisTo(40.0, 0.0, 0.0);
        pinpoint.update();
        pinpoint.corruptNextBulkRead();
        pinpoint.update();

        assertEquals(40.0, pinpoint.getPosX(DistanceUnit.MM), 1e-4);
        assertEquals(DeviceStatus.FAULT_BAD_READ, pinpoint.getDeviceStatus());
    }

    @Test
    void placeholder_loopTimeFrequencyAndIdentityAreTheFakesStatedValues() {
        FakeGoBildaPinpoint pinpoint = new FakeGoBildaPinpoint();
        pinpoint.update();

        assertEquals(FakeGoBildaPinpoint.LOOP_TIME_MICROSECONDS, pinpoint.getLoopTime());
        assertEquals(1e6 / FakeGoBildaPinpoint.LOOP_TIME_MICROSECONDS, pinpoint.getFrequency(), 1e-6);
        assertEquals(FakeGoBildaPinpoint.DEVICE_ID, pinpoint.getDeviceID());
        assertEquals(FakeGoBildaPinpoint.DEFAULT_FIRMWARE_VERSION, pinpoint.getDeviceVersion());
        assertTrue(pinpoint.getDeviceName().contains("Pinpoint"), pinpoint.getDeviceName());
        assertEquals(HardwareDevice.Manufacturer.GoBilda, pinpoint.getManufacturer(), "a constant: no bus read");
    }

    @Test
    void invariant_aFlatRobotReportsNoPitchOrRollAndAYawQuaternion() {
        FakeGoBildaPinpoint pinpoint = new FakeGoBildaPinpoint();
        pinpoint.moveChassisTo(0.0, 0.0, 1.2);
        pinpoint.update();

        assertEquals(0.0, pinpoint.getPitch(AngleUnit.DEGREES), 1e-9);
        assertEquals(0.0, pinpoint.getRoll(AngleUnit.DEGREES), 1e-9);
        assertEquals(Math.cos(0.6), pinpoint.getQuaternion().w, 1e-6);
        assertEquals(Math.sin(0.6), pinpoint.getQuaternion().z, 1e-6);
    }

    @Test
    void invariant_oneBulkReadPerUpdate() {
        FakeGoBildaPinpoint pinpoint = new FakeGoBildaPinpoint();
        pinpoint.update();                           // the first update also reads the firmware version
        int before = pinpoint.busReadCount();
        pinpoint.update();
        assertEquals(1, pinpoint.busReadCount() - before);
    }

    /**
     * The structural guarantee: every public method a robot calls is the SDK driver's own, because this fake
     * overrides none of them. If someone "fixes" a behaviour by overriding a getter, this fails and says which.
     */
    @Test
    void invariant_theFakeOverridesNoPublicMethodOfTheRealDriver() {
        List<String> overridden = new ArrayList<>();
        for (Method mine : FakeGoBildaPinpoint.class.getDeclaredMethods()) {
            try {
                Method theirs = GoBildaPinpointDriver.class.getMethod(mine.getName(), mine.getParameterTypes());
                if (Modifier.isPublic(theirs.getModifiers())) {
                    overridden.add(mine.getName() + Arrays.toString(mine.getParameterTypes()));
                }
            } catch (NoSuchMethodException notTheDrivers) {
                // the fake's own sim-side method: fine
            }
        }
        assertTrue(overridden.isEmpty(), "FakeGoBildaPinpoint overrides the real driver's " + overridden);
    }
}
