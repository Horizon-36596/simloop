package org.horizon36596.simloop.example;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.horizon36596.simloop.fakehardware.FakeHardwareMap;
import org.horizon36596.simloop.fakehardware.FakeImu;
import org.horizon36596.simloop.opmode.OpModePhase;
import org.horizon36596.simloop.opmode.OpModeRunResult;
import org.horizon36596.simloop.opmode.OpModeRunner;
import org.junit.jupiter.api.Test;

import java.util.Collections;
import java.util.List;

/**
 * The FTC SDK's own IMU sample, {@code SensorIMUOrthogonal} (FtcRobotController 12.0 external samples), run
 * unchanged through {@link OpModeRunner} against a {@link FakeImu}: {@code hardwareMap.get(IMU.class, "imu")},
 * {@code new IMU.Parameters(new RevHubOrientationOnRobot(...))}, {@code imu.initialize}, {@code imu.resetYaw}
 * on gamepad Y, and {@code getRobotYawPitchRollAngles} / {@code getRobotAngularVelocity} read back through
 * telemetry, while the simulated chassis turns at a constant rate.
 *
 * <p>Invariant tier for the numbers: they follow from the turn rate and the tick alone. The chassis turns
 * 90 degrees per second at 50 Hz, so 1.8 degrees per tick; the heading is injected after each tick's device
 * update, so pass {@code k} of the sample's loop reads the heading after {@code k} ticks of turning, and the
 * gyro rate, finite-differenced across updates, is one tick behind that.
 */
class SdkImuSampleExampleTest {

    // ---- the SDK sample, as shipped (comments trimmed) -----------------------------------------------

    @TeleOp(name = "Sensor: IMU Orthogonal", group = "Sensor")
    @Disabled
    public static class SensorIMUOrthogonal extends LinearOpMode {
        IMU imu;

        @Override public void runOpMode() throws InterruptedException {
            imu = hardwareMap.get(IMU.class, "imu");

            RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
            RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

            RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

            imu.initialize(new IMU.Parameters(orientationOnRobot));

            while (!isStopRequested()) {

                telemetry.addData("Hub orientation", "Logo=%s   USB=%s\n ", logoDirection, usbDirection);

                if (gamepad1.y) {
                    telemetry.addData("Yaw", "Resetting\n");
                    imu.resetYaw();
                } else {
                    telemetry.addData("Yaw", "Press Y (triangle) on Gamepad to reset\n");
                }

                YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
                AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);

                telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));
                telemetry.addData("Pitch (X)", "%.2f Deg.", orientation.getPitch(AngleUnit.DEGREES));
                telemetry.addData("Roll (Y)", "%.2f Deg.\n", orientation.getRoll(AngleUnit.DEGREES));
                telemetry.addData("Yaw (Z) velocity", "%.2f Deg/Sec", angularVelocity.zRotationRate);
                telemetry.addData("Pitch (X) velocity", "%.2f Deg/Sec", angularVelocity.xRotationRate);
                telemetry.addData("Roll (Y) velocity", "%.2f Deg/Sec", angularVelocity.yRotationRate);
                telemetry.update();
            }
        }
    }

    // ---- the harness --------------------------------------------------------------------------------

    private static final double TICK_SECONDS = 0.02;
    private static final double TURN_RATE_DEGREES_PER_SECOND = 90.0;
    private static final double DEGREES_PER_TICK = TURN_RATE_DEGREES_PER_SECOND * TICK_SECONDS; // 1.8
    private static final int ACTIVE_TICKS = 50;

    /** Y is pressed on this tick of the run (tick 0 is INIT, so this is the 10th ACTIVE tick). */
    private static final int Y_TICK = 10;

    @Test
    void theSdkImuSampleRunsUnchangedAndReadsTheSimulatedTurn() {
        FakeHardwareMap hardwareMap = new FakeHardwareMap();
        FakeImu imu = new FakeImu(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)); // mounted as the sample declares
        hardwareMap.register("imu", imu);
        double[] headingRadians = {0.0};

        OpModeRunResult result = new OpModeRunner(new SensorIMUOrthogonal(), hardwareMap)
                .tickSeconds(TICK_SECONDS)
                .activeTicks(ACTIVE_TICKS)
                .gamepads((phase, tickInPhase, gamepad1, gamepad2) ->
                        gamepad1.y = phase == OpModePhase.ACTIVE && tickInPhase == Y_TICK - 1)
                .eachTick(dt -> {
                    // the drive plant would integrate here; this chassis just turns at a constant rate
                    headingRadians[0] += Math.toRadians(TURN_RATE_DEGREES_PER_SECOND) * dt;
                    imu.setChassisHeading(headingRadians[0]);
                })
                .run();

        List<List<String>> passes = result.telemetry().transmissions();
        assertEquals(1 + ACTIVE_TICKS, passes.size(), "one pass of the sample's loop per tick before STOP");
        assertEquals(Collections.emptyList(), result.warnings());

        List<String> first = passes.get(0);
        assertEquals("Hub orientation : Logo=UP   USB=FORWARD\n ", first.get(0));
        assertEquals("Yaw : Press Y (triangle) on Gamepad to reset\n", first.get(1));

        for (int tick = 0; tick < passes.size(); tick++) {
            List<String> pass = passes.get(tick);
            double expectedYaw = tick < Y_TICK ? tick * DEGREES_PER_TICK : (tick - Y_TICK) * DEGREES_PER_TICK;
            double expectedRate = tick < 2 ? 0.0 : TURN_RATE_DEGREES_PER_SECOND;
            assertEquals(expectedYaw, number(pass, "Yaw (Z)"), 0.006, "yaw on tick " + tick);
            assertEquals(0.0, number(pass, "Pitch (X)"), 0.006, "pitch on tick " + tick);
            assertEquals(0.0, number(pass, "Roll (Y)"), 0.006, "roll on tick " + tick);
            assertEquals(expectedRate, number(pass, "Yaw (Z) velocity"), 0.006, "yaw rate on tick " + tick);
            assertEquals(0.0, number(pass, "Pitch (X) velocity"), 0.006, "pitch rate on tick " + tick);
            assertEquals(0.0, number(pass, "Roll (Y) velocity"), 0.006, "roll rate on tick " + tick);
        }
        assertEquals("Yaw : Resetting\n", passes.get(Y_TICK).get(1), "Y was seen on the tick it was pressed");
        assertEquals("Yaw : Press Y (triangle) on Gamepad to reset\n", passes.get(Y_TICK + 1).get(1));
    }

    /** {@return the number at the start of the value of the item captioned {@code caption}} */
    private static double number(List<String> pass, String caption) {
        String prefix = caption + " : ";
        for (String line : pass) {
            if (line.startsWith(prefix)) {
                String value = line.substring(prefix.length());
                return Double.parseDouble(value.substring(0, value.indexOf(' ')));
            }
        }
        assertTrue(false, "no '" + caption + "' in " + pass);
        return Double.NaN;
    }
}
