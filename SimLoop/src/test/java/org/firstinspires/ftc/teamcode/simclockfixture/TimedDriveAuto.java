package org.firstinspires.ftc.teamcode.simclockfixture;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/** Drives a motor on timers every way a rookie auto does, for the run-it-twice determinism test. */
@Autonomous(name = "Timed Drive")
public class TimedDriveAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor motor = hardwareMap.get(DcMotor.class, "motor");
        ElapsedTime runtime = new ElapsedTime();
        waitForStart();

        long forwardMs = 400;
        motor.setPower(0.5);
        sleep(forwardMs + 100);
        motor.setPower(-0.25);
        Thread.sleep(300);
        motor.setPower(1.0);
        ElapsedTime burst = new ElapsedTime();
        while (opModeIsActive() && burst.milliseconds() < 200) {
            idle();
        }
        motor.setPower(0);
        while (opModeIsActive()) {
            telemetry.addData("runtime", "%.3f", runtime.seconds());
            telemetry.addData("nanos", System.nanoTime());
            telemetry.addData("millis", System.currentTimeMillis());
            telemetry.addData("position", motor.getCurrentPosition());
            telemetry.update();
        }
    }
}
