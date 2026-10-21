package org.firstinspires.ftc.teamcode.simclockfixture;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * The usual "update the arm in the background" helper: a thread that sleeps between updates and catches only
 * InterruptedException, so the refused Thread.sleep kills it on its first pass, usually within one tick. The
 * OpMode holds its first tick until that first pass has begun, so the refusal happens on every run.
 */
@Autonomous(name = "Background Helper")
public class BackgroundHelperAuto extends LinearOpMode {

    private volatile int updates;

    @Override
    public void runOpMode() {
        Thread helper = new Thread(() -> {
            try {
                while (!isStopRequested()) {
                    updates++;
                    Thread.sleep(10);
                }
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }, "arm-updater");
        waitForStart();
        helper.start();
        while (updates == 0 && helper.isAlive()) {
            // Hold the first tick (in real time, briefly) until the helper has made its first pass. Its next
            // step is the sleep, with no stop check in between, so it always reaches the refusal. Without this
            // hold, a run whose ticks all finish before the helper is first scheduled stops it at its loop
            // check, it exits cleanly having never slept, and nothing is refused (the flake this fixed).
        }
        while (opModeIsActive()) {
            telemetry.addData("updates", updates);
            telemetry.update();
        }
    }
}
