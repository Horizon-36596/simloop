package org.horizon36596.simloop.example;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.horizon36596.simloop.config.DrivetrainSimConfig;
import org.horizon36596.simloop.config.SimRobotConfig;
import org.horizon36596.simloop.fakehardware.FakeHardwareMap;
import org.horizon36596.simloop.fakehardware.FakeMotor;
import org.horizon36596.simloop.opmode.OpModePhase;
import org.horizon36596.simloop.opmode.OpModeRunResult;
import org.horizon36596.simloop.opmode.OpModeRunner;
import org.horizon36596.simloop.plant.MecanumDrivePlant;
import org.horizon36596.simloop.sim.RlogDecodedCompare;

import org.junit.jupiter.api.Test;
import org.psilynx.psikit.core.Logger;

import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.List;
import java.util.Map;

/**
 * The worked example for {@code OpModeRunner} in {@code SimLoop/README.md}: two OpModes written exactly as
 * a rookie team writes them - an iterative tank-drive TeleOp and a {@code LinearOpMode} mecanum auto that
 * drives on {@code sleep()} - run unchanged against fake motors. If you edit the README snippet, edit this
 * file, and the other way round.
 *
 * <p>Nothing in the two OpModes knows about SimLoop. They import only the FTC SDK.
 */
class OpModeRunnerExampleTest {

    // ---- the team's code, unchanged -----------------------------------------------------------------

    /** Left stick drives the left side, right stick the right side. */
    @TeleOp(name = "Tank Drive")
    public static class TankTeleOp extends OpMode {
        private DcMotor left;
        private DcMotor right;

        @Override
        public void init() {
            left = hardwareMap.get(DcMotor.class, "left");
            right = hardwareMap.get(DcMotor.class, "right");
            right.setDirection(DcMotorSimple.Direction.REVERSE);
            telemetry.addData("Status", "Initialized");
        }

        @Override
        public void loop() {
            left.setPower(-gamepad1.left_stick_y);
            right.setPower(-gamepad1.right_stick_y);
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", left.getPower(), right.getPower());
        }
    }

    /** Drive forward for a second, strafe right for half a second, stop. */
    @Autonomous(name = "Timed Mecanum Auto")
    public static class TimedMecanumAuto extends LinearOpMode {
        @Override
        public void runOpMode() {
            DcMotor frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
            DcMotor frontRight = hardwareMap.get(DcMotor.class, "frontRight");
            DcMotor backLeft = hardwareMap.get(DcMotor.class, "backLeft");
            DcMotor backRight = hardwareMap.get(DcMotor.class, "backRight");
            frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
            backRight.setDirection(DcMotorSimple.Direction.REVERSE);

            telemetry.addData("Status", "Ready");
            telemetry.update();
            waitForStart();

            setPowers(frontLeft, frontRight, backLeft, backRight, 0.5, 0.5, 0.5, 0.5);
            sleep(1000);
            setPowers(frontLeft, frontRight, backLeft, backRight, 0.5, -0.5, -0.5, 0.5);
            sleep(500);
            setPowers(frontLeft, frontRight, backLeft, backRight, 0, 0, 0, 0);

            while (opModeIsActive()) {
                telemetry.addData("Status", "Done");
                telemetry.update();
            }
        }

        private static void setPowers(DcMotor fl, DcMotor fr, DcMotor bl, DcMotor br,
                double flPower, double frPower, double blPower, double brPower) {
            fl.setPower(flPower);
            fr.setPower(frPower);
            bl.setPower(blPower);
            br.setPower(brPower);
        }
    }

    // ---- the sim --------------------------------------------------------------------------------------

    /** Free speed of the fake drive motors, in encoder ticks per second (a 312 rpm goBILDA motor). */
    private static final double MOTOR_TICKS_PER_SECOND = 2787.0;

    /** Motor response rate in 1/s: a 0.1 s time constant. */
    private static final double MOTOR_RESPONSE = 10.0;

    /** A plausible 14-inch mecanum chassis, with the right-hand motors bolted in mirrored. */
    private static final DrivetrainSimConfig DRIVETRAIN = new DrivetrainSimConfig() {
        @Override public double trackWidth() { return 14.0; }                  // inches
        @Override public double wheelBase() { return 12.0; }                   // inches
        @Override public double wheelRadius() { return 1.89; }                 // inches (96 mm wheel)
        @Override public double ticksPerInch() { return 45.3; }                // ticks per inch
        @Override public double maxVelocityTicksPerSecond() { return MOTOR_TICKS_PER_SECOND; }
        @Override public double maxAccel() { return MOTOR_RESPONSE; }          // 1/s
        @Override public double[] wheelMountingSigns() { return new double[] {1, -1, 1, -1}; }
        @Override public double fieldHalfWidth() { return 72.0; }              // inches
        @Override public double fieldHalfHeight() { return 72.0; }             // inches
    };

    private static final SimRobotConfig ROBOT = new SimRobotConfig() {
        @Override public String[] driveMotorNames() {
            return new String[] {"frontLeft", "frontRight", "backLeft", "backRight"};
        }
        @Override public String odometryName() { return "none"; }
        @Override public DrivetrainSimConfig drivetrain() { return DRIVETRAIN; }
    };

    @Test
    void theTankTeleOpDrivesItsMotorsFromTheSticks() {
        FakeHardwareMap hardwareMap = new FakeHardwareMap();
        FakeMotor left = new FakeMotor(MOTOR_TICKS_PER_SECOND, MOTOR_RESPONSE);
        FakeMotor right = new FakeMotor(MOTOR_TICKS_PER_SECOND, MOTOR_RESPONSE);
        hardwareMap.register("left", left);
        hardwareMap.register("right", right);

        OpModeRunResult result = new OpModeRunner(new TankTeleOp(), hardwareMap)
                .initLoopTicks(10)
                .activeTicks(100)                                   // 2 s at 50 Hz
                .gamepads((phase, tick, gamepad1, gamepad2) -> {
                    if (phase == OpModePhase.ACTIVE) {
                        gamepad1.left_stick_y = -1f;                // full forward on the left
                        gamepad1.right_stick_y = -0.5f;             // half forward on the right
                    }
                })
                .run();

        // Both sides drove forward (the right is reversed in software, so its shaft turned the other way)...
        assertTrue(left.getCurrentPosition() > 0, "left moved forward");
        assertTrue(right.getCurrentPosition() > 0, "right moved forward");
        // ...and the left, on full stick, went about twice as far as the right on half.
        double ratio = (double) left.getCurrentPosition() / right.getCurrentPosition();
        assertEquals(2.0, ratio, 0.05);
        // stop() is the SDK's empty default, so the last telemetry is the last loop's.
        List<List<String>> sent = result.telemetry().transmissions();
        assertEquals("Status : Initialized", sent.get(0).get(0));
        assertEquals("Motors : left (1.00), right (0.50)", sent.get(sent.size() - 2).get(0));
    }

    @Test
    void theTankTeleOpRunTwiceGivesTheSameMotorsAndTelemetry() {
        long[] positions = new long[4];
        List<List<String>> firstTelemetry = null;
        for (int run = 0; run < 2; run++) {
            FakeHardwareMap hardwareMap = new FakeHardwareMap();
            FakeMotor left = new FakeMotor(MOTOR_TICKS_PER_SECOND, MOTOR_RESPONSE);
            FakeMotor right = new FakeMotor(MOTOR_TICKS_PER_SECOND, MOTOR_RESPONSE);
            hardwareMap.register("left", left);
            hardwareMap.register("right", right);
            OpModeRunResult result = new OpModeRunner(new TankTeleOp(), hardwareMap)
                    .activeTicks(200)
                    .gamepads((phase, tick, gamepad1, gamepad2) -> {
                        gamepad1.left_stick_y = (float) -Math.sin(tick / 20.0);
                        gamepad1.right_stick_y = (float) Math.cos(tick / 30.0);
                    })
                    .run();
            positions[2 * run] = left.getCurrentPosition();
            positions[2 * run + 1] = right.getCurrentPosition();
            if (run == 0) {
                firstTelemetry = result.telemetry().transmissions();
            } else {
                assertEquals(firstTelemetry, result.telemetry().transmissions());
            }
        }
        assertEquals(positions[0], positions[2]);
        assertEquals(positions[1], positions[3]);
    }

    /** One simulated robot: the fake motors, the plant, and the map the OpMode reads them through. */
    private static final class MecanumRobot {
        final FakeHardwareMap hardwareMap = new FakeHardwareMap();
        final FakeMotor[] motors = new FakeMotor[4];
        final MecanumDrivePlant plant;

        MecanumRobot() {
            String[] names = ROBOT.driveMotorNames();
            for (int i = 0; i < 4; i++) {
                motors[i] = new FakeMotor(MOTOR_TICKS_PER_SECOND, MOTOR_RESPONSE);
                hardwareMap.register(names[i], motors[i]);
            }
            plant = new MecanumDrivePlant(motors, DRIVETRAIN);
        }

        OpModeRunner runner() {
            return new OpModeRunner(new TimedMecanumAuto(), hardwareMap)
                    .initLoopTicks(5)
                    .activeTicks(150)                               // 3 s at 50 Hz
                    .eachTick(deltaTime -> {
                        plant.update(deltaTime);
                        if (Logger.isRunning()) {
                            Logger.recordOutput("Drive/xInches", plant.getX());
                            Logger.recordOutput("Drive/yInches", plant.getY());
                        }
                    });
        }
    }

    @Test
    void theTimedMecanumAutoDrivesForwardThenStrafesThenStops() {
        MecanumRobot robot = new MecanumRobot();
        OpModeRunResult result = robot.runner().run();

        // One second forward at half power: about half of free speed for a second, less the spin-up.
        double freeSpeedInchesPerSecond = MOTOR_TICKS_PER_SECOND / DRIVETRAIN.ticksPerInch();
        assertEquals(0.5 * freeSpeedInchesPerSecond * 0.9, robot.plant.getX(), 0.1 * freeSpeedInchesPerSecond,
                "forward, in inches");
        assertTrue(robot.plant.getY() < -3.0, "strafed right (+y is left in the localizer frame): "
                + robot.plant.getY());
        for (FakeMotor motor : robot.motors) {
            assertEquals(0.0, motor.getPower(), 0.0);
        }
        assertEquals("Status : Ready", result.telemetry().transmissions().get(0).get(0));
        assertEquals("Status : Done", result.telemetry().lastTransmission().get(0));
        assertTrue(!result.endedBeforeStop(), "it waited in its loop for STOP");
    }

    @Test
    void theSameAutoRunTwiceGivesTheSameRobotAndTheSameLog() throws IOException {
        Path first = Paths.get("build", "sim", "opmode-runner-example-1.rlog");
        Path second = Paths.get("build", "sim", "opmode-runner-example-2.rlog");

        MecanumRobot robotA = new MecanumRobot();
        OpModeRunResult a = robotA.runner().runLogged(ROBOT, first);
        MecanumRobot robotB = new MecanumRobot();
        OpModeRunResult b = robotB.runner().runLogged(ROBOT, second);

        assertEquals(robotA.plant.getX(), robotB.plant.getX(), 0.0);
        assertEquals(robotA.plant.getY(), robotB.plant.getY(), 0.0);
        assertEquals(robotA.plant.getHeading(), robotB.plant.getHeading(), 0.0);
        assertEquals(a.telemetry().transmissions(), b.telemetry().transmissions());
        assertTrue(a.warnings().isEmpty(), "the RLOG writer's thread is not one the OpMode started: " + a.warnings());

        List<Map<String, String>> frames = RlogDecodedCompare.decode(first);
        assertEquals(a.ticks(), frames.size(), "one RLOG frame per tick, INIT to STOP");
        assertTrue(frames.get(frames.size() - 1).containsKey("RealOutputs/Drive/xInches"));
        assertEquals(0, RlogDecodedCompare.compare(first, second).size(),
                "the two logs decode identically, field for field");
    }
}
