package org.firstinspires.ftc.teamcode.simclockfixture;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/** Starts a helper thread that sleeps, as a team that "runs the arm in the background" does. */
@Autonomous(name = "Helper Thread")
public class HelperThreadAuto extends LinearOpMode {

    @Override
    public void runOpMode() {
        Thread helper = new Thread(() -> {
            try {
                Thread.sleep(50);
                blackboard.put("helperSlept", true);
            } catch (Throwable refused) {
                blackboard.put("helperFailure", refused);
            }
        }, "arm-helper");
        blackboard.put("helper", helper);
        helper.start();
        while (helper.isAlive()) {
            // Hold INIT (in real time, briefly) until the helper has tried its sleep, so it tries while the
            // run is going; the sim waits for this thread, and the helper needs microseconds.
        }
        waitForStart();
        while (opModeIsActive()) {
            idle();
        }
    }
}
