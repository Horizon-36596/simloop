package org.firstinspires.ftc.teamcode.simclockfixture;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/** An iterative OpMode that sleeps in loop(), which freezes a real robot's loop too. */
@TeleOp(name = "Sleepy TeleOp")
public class SleepyTeleOp extends OpMode {

    @Override
    public void init() {
    }

    @Override
    public void loop() {
        try {
            Thread.sleep(10);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
}
