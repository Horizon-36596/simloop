package org.firstinspires.ftc.teamcode.simclockfixture;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * The usual "update the arm in the background" helper: a thread that sleeps between updates and catches only
 * InterruptedException, so the refused Thread.sleep kills it on its first pass, usually within one tick.
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
        while (opModeIsActive()) {
            telemetry.addData("updates", updates);
            telemetry.update();
        }
    }
}
