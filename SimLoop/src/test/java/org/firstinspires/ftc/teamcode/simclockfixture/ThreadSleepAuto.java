package org.firstinspires.ftc.teamcode.simclockfixture;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/** Waits with Thread.sleep, both overloads, as code copied from a Java tutorial does. */
@Autonomous(name = "Thread Sleep")
public class ThreadSleepAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        ElapsedTime runtime = new ElapsedTime();
        waitForStart();
        runtime.reset();

        Thread.sleep(300);
        blackboard.put("afterMillis", runtime.seconds());

        Thread.sleep(100, 500_000);                      // 100.5 ms
        blackboard.put("afterMillisAndNanos", runtime.seconds());

        try {
            while (opModeIsActive()) {
                Thread.sleep(40);                        // two ticks per pass
                blackboard.put("passes", ((Integer) blackboard.getOrDefault("passes", 0)) + 1);
            }
        } catch (InterruptedException stopped) {
            blackboard.put("interruptedAtStop", true);   // what the SDK's interrupt at STOP does too
            throw stopped;
        }
    }
}
