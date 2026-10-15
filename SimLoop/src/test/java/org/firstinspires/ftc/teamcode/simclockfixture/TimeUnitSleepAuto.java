package org.firstinspires.ftc.teamcode.simclockfixture;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.TimeUnit;

/** Waits with TimeUnit.sleep, which SimLoop moves onto sim time like Thread.sleep. */
@Autonomous(name = "TimeUnit Sleep")
public class TimeUnitSleepAuto extends LinearOpMode {

    @Override
    public void runOpMode() {
        waitForStart();
        ElapsedTime timer = new ElapsedTime();
        try {
            TimeUnit.MILLISECONDS.sleep(300);
            blackboard.put("timeUnitSleep", "returned");
        } catch (InterruptedException e) {
            blackboard.put("timeUnitSleep", "interrupted");
        }
        blackboard.put("timeUnitWaited", timer.seconds());
        while (opModeIsActive()) {
            idle();
        }
    }
}
