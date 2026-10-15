package org.firstinspires.ftc.teamcode.simclockfixture;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/** Waits on a timer with nothing inside the loop: on a robot it works, in a sim the clock never moves. */
@Autonomous(name = "Busy Wait")
public class BusyWaitAuto extends LinearOpMode {

    @Override
    public void runOpMode() {
        waitForStart();
        ElapsedTime timer = new ElapsedTime();
        while (timer.seconds() < 1.0) {
            // spin
        }
    }
}
