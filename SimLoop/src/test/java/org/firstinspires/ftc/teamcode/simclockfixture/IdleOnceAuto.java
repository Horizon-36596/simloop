package org.firstinspires.ftc.teamcode.simclockfixture;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/** One idle() outside any loop, then a counted opModeIsActive() loop. */
@Autonomous(name = "Idle Once")
public class IdleOnceAuto extends LinearOpMode {

    @Override
    public void runOpMode() {
        waitForStart();
        idle();
        int passes = 0;
        while (opModeIsActive()) {
            passes++;
            telemetry.addData("passes", passes);
            telemetry.addData("time", "%.2f", time);
            telemetry.update();
        }
    }
}
