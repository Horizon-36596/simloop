package org.firstinspires.ftc.teamcode.simclockfixture;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/** Every pass, reports the sim clock three ways through telemetry (per-run, so safe to run in parallel). */
public class TickProbeAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        ElapsedTime timer = new ElapsedTime();
        waitForStart();
        timer.reset();
        long passes = 0;
        while (opModeIsActive()) {
            Thread.sleep(40);
            passes++;
            telemetry.addData("passes", passes);
            telemetry.addData("nanos", System.nanoTime());
            telemetry.addData("timerNanos", timer.nanoseconds());
            telemetry.update();
        }
    }
}
