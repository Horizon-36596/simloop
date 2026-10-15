package org.firstinspires.ftc.teamcode.simclockfixture;

import com.example.simclockfixture.WallClockLibrary;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.concurrent.TimeUnit;

/** Waits three ways SimLoop treats differently: Object.wait, TimeUnit.sleep, and a library's Thread.sleep. */
@Autonomous(name = "Other Waits")
public class OtherWaitsAuto extends LinearOpMode {

    private final Object monitor = new Object();

    /** Something that waits and may be interrupted. */
    private interface Wait {
        String run() throws InterruptedException;
    }

    private void probe(String key, Wait wait) {
        isStopRequested(); // a lifecycle check, as a real loop makes just before it waits
        long real0 = WallClockLibrary.nanoTime();
        double sim0 = time;
        String outcome;
        try {
            outcome = wait.run();
        } catch (InterruptedException e) {
            outcome = "InterruptedException";
        }
        blackboard.put(key, outcome);
        blackboard.put(key + ".realMs", (WallClockLibrary.nanoTime() - real0) / 1e6);
        blackboard.put(key + ".simSeconds", time - sim0);
    }

    @Override
    public void runOpMode() {
        waitForStart();
        probe("objectWait", () -> {
            synchronized (monitor) {
                monitor.wait(300);
            }
            return "returned";
        });
        probe("timeUnitSleep", () -> {
            TimeUnit.MILLISECONDS.sleep(300);
            return "returned";
        });
        probe("librarySleep", () -> WallClockLibrary.pause(300));
        while (opModeIsActive()) {
            idle();
        }
    }
}
