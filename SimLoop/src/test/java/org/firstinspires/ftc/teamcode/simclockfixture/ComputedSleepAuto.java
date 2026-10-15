package org.firstinspires.ftc.teamcode.simclockfixture;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/** Sleeps for durations only known while it runs: a sum, a helper's parameter, and a ternary. */
@Autonomous(name = "Computed Sleep")
public class ComputedSleepAuto extends LinearOpMode {

    private final boolean fast = true;

    @Override
    public void runOpMode() {
        ElapsedTime runtime = new ElapsedTime();
        waitForStart();
        runtime.reset();

        long ms = 250;
        sleep(ms * 2);                                   // 500 ms
        blackboard.put("afterSum", runtime.seconds());

        pause(300);                                      // 300 ms, inside a helper
        blackboard.put("afterHelper", runtime.seconds());

        sleep(fast ? 100 : 1000);                        // 100 ms, from a branch
        blackboard.put("afterTernary", runtime.seconds());

        while (opModeIsActive()) {
            idle();
        }
    }

    private void pause(long millis) {
        sleep(millis);
    }
}
