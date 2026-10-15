package org.firstinspires.ftc.teamcode.simclockfixture;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/** Waits on a timer with idle() in the loop and no lifecycle check, as the SDK samples show. */
@Autonomous(name = "Idle Wait")
public class IdleWaitAuto extends LinearOpMode {

    @Override
    public void runOpMode() {
        waitForStart();
        ElapsedTime timer = new ElapsedTime();
        int passes = 0;
        while (timer.seconds() < 1.0) {
            passes++;
            idle();
        }
        blackboard.put("passes", passes);
        blackboard.put("waited", timer.seconds());
        while (opModeIsActive()) {
            idle();
        }
    }
}
