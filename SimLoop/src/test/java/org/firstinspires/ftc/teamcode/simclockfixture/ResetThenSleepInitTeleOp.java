package org.firstinspires.ftc.teamcode.simclockfixture;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * An iterative OpMode whose {@code init()} calls {@code resetRuntime()} and then sleeps 500 ms. It reports
 * {@code getRuntime()} after the sleep, in seconds, through the blackboard.
 */
@TeleOp(name = "Reset Then Sleep Init TeleOp")
public class ResetThenSleepInitTeleOp extends OpMode {

    @Override
    public void init() {
        resetRuntime();
        try {
            Thread.sleep(500);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
        blackboard.put("runtimeAfterSleep", getRuntime());
    }

    @Override
    public void loop() {
    }
}
