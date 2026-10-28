package org.firstinspires.ftc.teamcode.simclockfixture;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/** An iterative OpMode that sleeps in {@code init_loop()}, which on a robot stalls every pass before PLAY. */
@TeleOp(name = "Sleepy Init Loop TeleOp")
public class SleepyInitLoopTeleOp extends OpMode {

    @Override
    public void init() {
    }

    @Override
    public void init_loop() {
        try {
            Thread.sleep(10);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    @Override
    public void loop() {
    }
}
