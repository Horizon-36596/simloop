package org.horizon36596.simloop.fakehardware;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot.LogoFacingDirection;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot.UsbFacingDirection;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.horizon36596.simloop.config.DrivetrainSimConfig;
import org.horizon36596.simloop.plant.MecanumDrivePlant;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.List;

/**
 * INVARIANT-tier (domain R9): robot code that reads the SDK's universal {@link IMU} gets the simulated
 * chassis heading back in the SDK's own units, frames and wrapping, through whatever mounting it declares,
 * and deterministically. Pins {@link FakeImu}.
 *
 * <p>Tolerances are float-sized: the SDK's quaternion and orientation types are {@code float}, so a real hub
 * cannot do better than about 1e-5 degrees either.
 */
class FakeImuTest {

    private static final double DT = 0.02;
    private static final double DEGREES_TOLERANCE = 1e-3;
    private static final double RADIANS_TOLERANCE = Math.toRadians(DEGREES_TOLERANCE);

    private static IMU.Parameters declared(LogoFacingDirection logo, UsbFacingDirection usb) {
        return new IMU.Parameters(new RevHubOrientationOnRobot(logo, usb));
    }

    /** Heading in degrees wrapped into {@code [-180, 180)}, the SDK's range, computed independently. */
    private static double wrappedDegrees(double headingRadians) {
        double degrees = Math.toDegrees(headingRadians) % 360.0;
        if (degrees >= 180.0) degrees -= 360.0;
        if (degrees < -180.0) degrees += 360.0;
        return degrees;
    }

    private static double yawDegrees(IMU imu) {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    /** Injects each heading and ticks once, the way a harness does per loop. */
    private static void turnThrough(FakeImu imu, double... headingsRadians) {
        for (double heading : headingsRadians) {
            imu.setChassisHeading(heading);
            imu.update(DT);
        }
    }

    // ---- heading ----

    @Test
    void yawIsTheChassisHeadingInDegreesWithTheDefaultMounting() {
        FakeImu imu = new FakeImu();
        for (double heading : new double[] {0.0, 0.3, 1.2, -0.7, 2.9, -3.0}) {
            imu.setChassisHeading(heading);
            YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
            assertEquals(Math.toDegrees(heading), angles.getYaw(AngleUnit.DEGREES), DEGREES_TOLERANCE,
                    "yaw at heading " + heading + " rad");
            assertEquals(Math.toDegrees(heading), angles.getYaw(), DEGREES_TOLERANCE,
                    "getYaw() with no unit is degrees, as the SDK defines it");
            assertEquals(0.0, angles.getPitch(AngleUnit.DEGREES), DEGREES_TOLERANCE, "a flat chassis has no pitch");
            assertEquals(0.0, angles.getRoll(AngleUnit.DEGREES), DEGREES_TOLERANCE, "a flat chassis has no roll");
        }
    }

    @Test
    void yawTracksAMecanumPlantTurningInPlaceThroughTheHardwareMap() {
        FakeMotor[] motors = {
                new FakeMotor(2435.0, 2.0), new FakeMotor(2435.0, 2.0),
                new FakeMotor(2435.0, 2.0), new FakeMotor(2435.0, 2.0),
        };
        FakeHardwareMap map = new FakeHardwareMap();
        String[] names = {"frontLeft", "frontRight", "backLeft", "backRight"};
        for (int i = 0; i < 4; i++) {
            map.register(names[i], motors[i]);
        }
        FakeImu imu = new FakeImu();
        map.register("imu", imu);
        MecanumDrivePlant plant = new MecanumDrivePlant(motors, drivetrain());

        // Robot code: find the IMU the way a team's init does, then turn CCW (left side back, right forward).
        IMU robotImu = map.get(IMU.class, "imu");
        motors[0].setPower(-0.6);
        motors[1].setPower(0.6);
        motors[2].setPower(-0.6);
        motors[3].setPower(0.6);

        double previousHeading = 0.0;
        double headingBeforeThat = 0.0;
        for (int tick = 0; tick < 400; tick++) {
            // Harness order, as in a season's sim: the fakes tick, the plant integrates, the sensors are fed.
            map.updateAll(DT);
            plant.update(DT);
            imu.setChassisHeading(plant.getHeading());

            assertEquals(wrappedDegrees(plant.getHeading()), yawDegrees(robotImu), DEGREES_TOLERANCE,
                    "yaw is the plant heading, wrapped, at tick " + tick);
            if (tick >= 2) {
                // Injected after updateAll, so the rate is the turn over the previous tick (class javadoc).
                double expectedRate = (previousHeading - headingBeforeThat) / DT;
                assertEquals(expectedRate,
                        robotImu.getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate, 1e-4,
                        "rate is the finite-differenced plant heading at tick " + tick);
            }
            headingBeforeThat = previousHeading;
            previousHeading = plant.getHeading();
        }
        assertTrue(plant.getHeading() > 2.0 * Math.PI, "the plant must have turned more than once for the "
                + "wrap to be exercised, turned " + plant.getHeading() + " rad");
    }

    @Test
    void yawWrapsIntoTheSdkRangeWhateverTheUnwrappedPlantHeading() {
        FakeImu imu = new FakeImu();
        for (double degrees : new double[] {190.0, 359.0, 540.5, 721.0, -190.0, -725.0, 3600.0 + 45.0}) {
            imu.setChassisHeading(Math.toRadians(degrees));
            double yaw = yawDegrees(imu);
            assertTrue(yaw >= -180.0 && yaw < 180.0, "yaw " + yaw + " is outside [-180, 180)");
            assertEquals(wrappedDegrees(Math.toRadians(degrees)), yaw, DEGREES_TOLERANCE,
                    "heading " + degrees + " deg");
        }
    }

    @Test
    void radiansAreRadiansEverywhereTheCallerAsksForThem() {
        FakeImu imu = new FakeImu();
        imu.setChassisHeading(1.0);

        assertEquals(1.0, imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS), RADIANS_TOLERANCE);
        Orientation zyx = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        assertEquals(AngleUnit.RADIANS, zyx.angleUnit);
        assertEquals(1.0, zyx.firstAngle, RADIANS_TOLERANCE, "first ZYX angle is the heading");
        assertEquals(0.0, zyx.secondAngle, RADIANS_TOLERANCE);
        assertEquals(0.0, zyx.thirdAngle, RADIANS_TOLERANCE);
        Orientation degrees = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        assertEquals(Math.toDegrees(1.0), degrees.firstAngle, DEGREES_TOLERANCE);

        imu.setChassisHeading(Math.toRadians(200.0));
        double yawRadians = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        assertEquals(Math.toRadians(-160.0), yawRadians, RADIANS_TOLERANCE, "radians wrap into [-pi, pi) too");
    }

    @Test
    void theQuaternionIsThePureYawRotation() {
        FakeImu imu = new FakeImu();
        imu.setChassisHeading(0.8);
        Quaternion q = imu.getRobotOrientationAsQuaternion();
        // q and -q are the same rotation; compare the sign-normalised form.
        double sign = Math.signum(q.w);
        assertEquals(Math.cos(0.4), sign * q.w, 1e-5);
        assertEquals(0.0, sign * q.x, 1e-5);
        assertEquals(0.0, sign * q.y, 1e-5);
        assertEquals(Math.sin(0.4), sign * q.z, 1e-5);
    }

    // ---- resetYaw and power-on ----

    @Test
    void resetYawMakesTheCurrentHeadingReadZeroAndLaterTurnsReadRelativeToIt() {
        FakeImu imu = new FakeImu();
        imu.setChassisHeading(Math.toRadians(70.0));
        imu.resetYaw();
        assertEquals(0.0, yawDegrees(imu), DEGREES_TOLERANCE, "straight after the reset");

        imu.setChassisHeading(Math.toRadians(100.0));
        assertEquals(30.0, yawDegrees(imu), DEGREES_TOLERANCE, "30 deg CCW after the reset");
        imu.setChassisHeading(Math.toRadians(-130.0));
        assertEquals(160.0, yawDegrees(imu), DEGREES_TOLERANCE, "-200 deg relative wraps to +160");
    }

    @Test
    void resetYawDoesNotChangeTheRate() {
        FakeImu imu = new FakeImu();
        turnThrough(imu, 0.0, 0.02, 0.04);
        double before = imu.getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate;
        imu.resetYaw();
        assertEquals(before, imu.getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate, 1e-6);
    }

    @Test
    void yawIsZeroedAtPowerOnAndInitializeDoesNotZeroItAgain() {
        FakeImu imu = new FakeImu();
        imu.setChassisHeading(Math.toRadians(40.0));

        assertTrue(imu.initialize(declared(LogoFacingDirection.UP, UsbFacingDirection.FORWARD)),
                "initialize reports success");
        assertEquals(40.0, yawDegrees(imu), DEGREES_TOLERANCE,
                "BHI260IMU.initialize keeps the power-on zero; only resetYaw moves it");
    }

    // ---- angular velocity ----

    @Test
    void aCounterClockwiseTurnIsAPositiveZRateInEitherUnit() {
        FakeImu imu = new FakeImu();
        turnThrough(imu, 0.0, 0.02, 0.04, 0.06); // 1 rad/s CCW

        AngularVelocity radians = imu.getRobotAngularVelocity(AngleUnit.RADIANS);
        assertEquals(1.0, radians.zRotationRate, 1e-4, "rad/s");
        assertEquals(0.0, radians.xRotationRate, 1e-6);
        assertEquals(0.0, radians.yRotationRate, 1e-6);
        assertEquals(AngleUnit.RADIANS.getUnnormalized(), radians.angleUnit);

        AngularVelocity degrees = imu.getRobotAngularVelocity(AngleUnit.DEGREES);
        assertEquals(Math.toDegrees(1.0), degrees.zRotationRate, 1e-2, "deg/s");
        assertEquals(AngleUnit.DEGREES.getUnnormalized(), degrees.angleUnit);
    }

    @Test
    void aClockwiseTurnIsANegativeZRate() {
        FakeImu imu = new FakeImu();
        turnThrough(imu, 0.0, -0.05, -0.10);
        assertEquals(-2.5, imu.getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate, 1e-4);
    }

    @Test
    void aRateAboveOneTurnPerSecondIsNotWrapped() {
        FakeImu imu = new FakeImu();
        turnThrough(imu, 0.0, 0.2, 0.4); // 10 rad/s = 573 deg/s
        assertEquals(Math.toDegrees(10.0), imu.getRobotAngularVelocity(AngleUnit.DEGREES).zRotationRate, 0.05,
                "a rate is not an angle, so 573 deg/s must not come back as 573 - 720");
    }

    @Test
    void aHeadingSeededBeforeTheFirstTickIsNotATurn() {
        FakeImu imu = new FakeImu();
        imu.setChassisHeading(2.0);
        imu.update(DT);
        assertEquals(0.0, imu.getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate, 0.0,
                "the first tick has nothing to difference against");
    }

    @Test
    void crossingTheWrapPointIsNotASpike() {
        FakeImu imu = new FakeImu();
        // A harness that feeds already-wrapped headings: 179 deg to -179 deg is a 2 deg CCW turn.
        turnThrough(imu, Math.toRadians(179.0), Math.toRadians(-179.0));
        assertEquals(Math.toRadians(2.0) / DT, imu.getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate, 1e-4);
    }

    @Test
    void aTickThatIsNotRealTimeChangesNothing() {
        FakeImu imu = new FakeImu();
        turnThrough(imu, 0.0, 0.02);
        long acquired = imu.getRobotYawPitchRollAngles().getAcquisitionTime();
        imu.setChassisHeading(1.0);
        imu.update(0.0);
        imu.update(-DT);
        imu.update(Double.NaN);
        assertEquals(1.0, imu.getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate, 1e-4,
                "rate still the last real tick's");
        assertEquals(acquired, imu.getRobotYawPitchRollAngles().getAcquisitionTime(), "the clock did not move");
    }

    // ---- mounting ----

    @Test
    void everyMountingDeclaredCorrectlyReadsTheChassisHeadingAndRate() {
        int physicallyPossible = 0;
        for (LogoFacingDirection logo : LogoFacingDirection.values()) {
            for (UsbFacingDirection usb : UsbFacingDirection.values()) {
                RevHubOrientationOnRobot mounting;
                try {
                    mounting = new RevHubOrientationOnRobot(logo, usb);
                } catch (IllegalArgumentException parallelDirections) {
                    continue; // logo and USB on one axis cannot both be true of a hub; the SDK refuses it
                }
                physicallyPossible++;
                FakeImu imu = new FakeImu(mounting);
                imu.initialize(new IMU.Parameters(mounting));
                imu.resetYaw(); // what team init does after initialize; it also undoes the power-on zero
                turnThrough(imu, 0.0, 0.03, 0.06);

                String label = "logo " + logo + ", USB " + usb;
                YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
                assertEquals(Math.toDegrees(0.06), angles.getYaw(AngleUnit.DEGREES), DEGREES_TOLERANCE, label);
                assertEquals(0.0, angles.getPitch(AngleUnit.DEGREES), DEGREES_TOLERANCE, label);
                assertEquals(0.0, angles.getRoll(AngleUnit.DEGREES), DEGREES_TOLERANCE, label);
                AngularVelocity rate = imu.getRobotAngularVelocity(AngleUnit.RADIANS);
                assertEquals(1.5, rate.zRotationRate, 1e-4, label);
                assertEquals(0.0, rate.xRotationRate, 1e-4, label);
                assertEquals(0.0, rate.yRotationRate, 1e-4, label);
            }
        }
        assertEquals(24, physicallyPossible, "six logo directions times four perpendicular USB directions");
    }

    @Test
    void aHubRotatedOnlyAboutTheVerticalStillReadsTurnsExactlyWithoutADeclaration() {
        // Mounted USB-left but never declared: the power-on zero absorbs the 90 deg, turns still read true.
        FakeImu imu = new FakeImu(new RevHubOrientationOnRobot(LogoFacingDirection.UP, UsbFacingDirection.LEFT));
        imu.setChassisHeading(Math.toRadians(35.0));
        assertEquals(35.0, yawDegrees(imu), DEGREES_TOLERANCE);
        assertEquals(0.0, imu.getRobotYawPitchRollAngles().getPitch(AngleUnit.DEGREES), DEGREES_TOLERANCE);
        assertEquals(0.0, imu.getRobotYawPitchRollAngles().getRoll(AngleUnit.DEGREES), DEGREES_TOLERANCE);
    }

    @Test
    void aHubStoodOnItsEdgeButDeclaredFlatReadsAsARobotLyingOnItsSide() {
        FakeImu imu = new FakeImu(new RevHubOrientationOnRobot(LogoFacingDirection.FORWARD, UsbFacingDirection.UP));
        // The robot code keeps the SDK default (logo up), which is wrong for this robot.
        turnThrough(imu, 0.0, Math.toRadians(30.0));

        // Read through the wrong mounting, the robot looks tipped 90 degrees onto its side. (The SDK's
        // yaw/pitch/roll, intrinsic ZXY, is exactly at gimbal lock there and returns NaN; a real hub's noise
        // keeps it just off the lock, where its yaw is just as meaningless.)
        Orientation zyx = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        assertEquals(90.0, Math.abs(zyx.thirdAngle), DEGREES_TOLERANCE, "a phantom 90 degree tilt");
        AngularVelocity rate = imu.getRobotAngularVelocity(AngleUnit.DEGREES);
        assertEquals(0.0, rate.zRotationRate, 1e-2, "the turn never reaches the robot's z gyro rate");
        assertEquals(1500.0, Math.abs(rate.xRotationRate) + Math.abs(rate.yRotationRate), 0.05,
                "it lands on a horizontal axis instead: 30 deg in one 0.02 s tick");

        // Declaring the true mounting fixes it, after a reset, exactly as on a robot.
        imu.initialize(declared(LogoFacingDirection.FORWARD, UsbFacingDirection.UP));
        imu.resetYaw();
        imu.setChassisHeading(Math.toRadians(50.0));
        assertEquals(20.0, yawDegrees(imu), DEGREES_TOLERANCE);
    }

    @Test
    void editingTheParametersAfterInitializeHasNoEffect() {
        FakeImu imu = new FakeImu();
        IMU.Parameters parameters = declared(LogoFacingDirection.UP, UsbFacingDirection.FORWARD);
        imu.initialize(parameters);
        parameters.imuOrientationOnRobot =
                new RevHubOrientationOnRobot(LogoFacingDirection.FORWARD, UsbFacingDirection.UP);

        imu.setChassisHeading(Math.toRadians(25.0));
        assertEquals(25.0, yawDegrees(imu), DEGREES_TOLERANCE);
    }

    // ---- hardware map ----

    @Test
    void robotCodeFindsItAsAnImuUnderItsConfigName() {
        FakeHardwareMap map = new FakeHardwareMap();
        FakeImu imu = new FakeImu();
        map.register("imu", imu);

        assertSame(imu, map.get(IMU.class, "imu"), "hardwareMap.get(IMU.class, \"imu\")");
        assertSame(imu, map.get(IMU.class, " imu "), "trimmed, as the SDK trims");
        assertSame(imu, map.tryGet(IMU.class, "imu"));
        assertEquals(List.of(imu), map.getAll(IMU.class));
    }

    @Test
    void theFtcSampleInitCodeRunsUnchangedAndReadsTheHeading() {
        FakeHardwareMap hardwareMap = new FakeHardwareMap();
        FakeImu fake = new FakeImu(new RevHubOrientationOnRobot(LogoFacingDirection.LEFT, UsbFacingDirection.UP));
        hardwareMap.register("imu", fake);

        // Robot code, as the SDK's own SensorIMUOrthogonal sample writes it.
        IMU imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot orientationOnRobot =
                new RevHubOrientationOnRobot(LogoFacingDirection.LEFT, UsbFacingDirection.UP);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();

        fake.setChassisHeading(Math.toRadians(-45.0));
        assertEquals(-45.0, imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES), DEGREES_TOLERANCE);
    }

    @Test
    void updateAllTicksItLikeEveryOtherFake() {
        FakeHardwareMap map = new FakeHardwareMap();
        FakeImu imu = new FakeImu();
        map.register("imu", imu);
        imu.setChassisHeading(0.0);
        map.updateAll(DT);
        imu.setChassisHeading(0.1);
        map.updateAll(DT);

        assertEquals(5.0, map.get(IMU.class, "imu").getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate, 1e-4);
    }

    // ---- determinism and guards ----

    @Test
    void acquisitionTimeIsSimulatedTimeNotTheWallClock() {
        FakeImu imu = new FakeImu();
        assertEquals(0L, imu.getRobotYawPitchRollAngles().getAcquisitionTime(), "no time has been simulated");
        for (int i = 0; i < 50; i++) {
            imu.update(DT);
        }
        long oneSecond = 1_000_000_000L;
        assertEquals(oneSecond, imu.getRobotYawPitchRollAngles().getAcquisitionTime());
        assertEquals(oneSecond, imu.getRobotAngularVelocity(AngleUnit.DEGREES).acquisitionTime);
        assertEquals(oneSecond, imu.getRobotOrientationAsQuaternion().acquisitionTime);
    }

    @Test
    void twoRunsOfTheSameInputsReadTheSameBits() {
        assertEquals(recordRun(), recordRun());
    }

    private static List<String> recordRun() {
        FakeImu imu = new FakeImu(new RevHubOrientationOnRobot(LogoFacingDirection.LEFT, UsbFacingDirection.UP));
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(LogoFacingDirection.LEFT, UsbFacingDirection.UP)));
        List<String> readings = new ArrayList<>();
        for (int tick = 0; tick < 200; tick++) {
            imu.setChassisHeading(0.05 * tick + 0.3 * Math.sin(tick * 0.1));
            imu.update(DT);
            if (tick == 100) {
                imu.resetYaw();
            }
            YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
            AngularVelocity rate = imu.getRobotAngularVelocity(AngleUnit.RADIANS);
            readings.add(Double.doubleToLongBits(angles.getYaw()) + " " + Double.doubleToLongBits(angles.getPitch())
                    + " " + Double.doubleToLongBits(angles.getRoll()) + " " + angles.getAcquisitionTime()
                    + " " + Float.floatToIntBits(rate.zRotationRate) + " " + rate.acquisitionTime);
        }
        return readings;
    }

    @Test
    void aNonFiniteHeadingIsRefused() {
        FakeImu imu = new FakeImu();
        assertThrows(IllegalArgumentException.class, () -> imu.setChassisHeading(Double.NaN));
        assertThrows(IllegalArgumentException.class, () -> imu.setChassisHeading(Double.POSITIVE_INFINITY));
    }

    @Test
    void aNullPhysicalMountingIsRefused() {
        assertThrows(IllegalArgumentException.class, () -> new FakeImu(null));
    }

    private static DrivetrainSimConfig drivetrain() {
        return new DrivetrainSimConfig() {
            @Override public double trackWidth() { return 11.27362; }
            @Override public double wheelBase() { return 11.50976; }
            @Override public double wheelRadius() { return 1.88976; }
            @Override public double ticksPerInch() { return 26.2; }
            @Override public double maxVelocityTicksPerSecond() { return 2435.0; }
            @Override public double maxAccel() { return 2.0; }
            @Override public double fieldHalfWidth() { return 72.0; }
            @Override public double fieldHalfHeight() { return 72.0; }
        };
    }
}
