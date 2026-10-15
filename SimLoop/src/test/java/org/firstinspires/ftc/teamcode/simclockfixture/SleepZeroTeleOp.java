package org.firstinspires.ftc.teamcode.simclockfixture;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/** Yields with Thread.sleep(0) and Thread.sleep(0, 0) in loop(), which is harmless on a robot. */
@TeleOp(name = "Sleep Zero")
public class SleepZeroTeleOp extends OpMode {

    private int loops;

    @Override
    public void init() {
    }

    @Override
    public void loop() {
        try {
            Thread.sleep(0);
            Thread.sleep(0, 0);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
        loops++;
        blackboard.put("loops", loops);
    }
}
