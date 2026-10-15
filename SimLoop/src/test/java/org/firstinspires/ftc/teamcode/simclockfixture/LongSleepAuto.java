package org.firstinspires.ftc.teamcode.simclockfixture;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/** Parks for ten seconds, computed, as a "wait for the alliance partner" auto does. */
@Autonomous(name = "Long Sleep")
public class LongSleepAuto extends LinearOpMode {

    private final long waitSeconds = 10;

    @Override
    public void runOpMode() {
        waitForStart();
        ElapsedTime timer = new ElapsedTime();
        sleep(waitSeconds * 1000);
        blackboard.put("waited", timer.seconds());
        while (opModeIsActive()) {
            idle();
        }
    }
}
