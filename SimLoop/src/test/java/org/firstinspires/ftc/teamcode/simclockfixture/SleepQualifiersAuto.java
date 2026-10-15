package org.firstinspires.ftc.teamcode.simclockfixture;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/** Calls sleep(long) through the three qualifiers in {@link ThreadSubclasses} and times each. */
@Autonomous(name = "Sleep Qualifiers")
public class SleepQualifiersAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        ElapsedTime timer = new ElapsedTime();
        blackboard.put("hidingCalls", ThreadSubclasses.callHiding());
        blackboard.put("afterHiding", timer.milliseconds());
        timer.reset();
        ThreadSubclasses.Worker.nap(100);
        blackboard.put("afterWorkerNap", timer.milliseconds());
        timer.reset();
        ThreadSubclasses.viaJdkSubclass(200);
        blackboard.put("afterJdkSubclassSleep", timer.milliseconds());
        while (opModeIsActive()) {
            idle();
        }
    }
}
