package org.firstinspires.ftc.teamcode.simclockfixture;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/** Sleeps for a time that has gone negative, which crashes a real robot with IllegalArgumentException. */
@Autonomous(name = "Negative Sleep")
public class NegativeSleepAuto extends LinearOpMode {

    @Override
    public void runOpMode() {
        waitForStart();
        long remaining = 100 - 250;
        sleep(remaining);
    }
}
