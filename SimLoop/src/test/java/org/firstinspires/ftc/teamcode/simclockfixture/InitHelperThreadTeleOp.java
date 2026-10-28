package org.firstinspires.ftc.teamcode.simclockfixture;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/** An iterative OpMode whose {@code init()} hands a sleep to a helper thread and waits for it to finish. */
@TeleOp(name = "Init Helper Thread TeleOp")
public class InitHelperThreadTeleOp extends OpMode {

    @Override
    public void init() {
        Thread helper = new Thread(() -> {
            try {
                Thread.sleep(50);
                blackboard.put("helperSlept", true);
            } catch (Throwable refused) {
                blackboard.put("helperFailure", refused);
            }
        }, "init-helper");
        helper.start();
        try {
            helper.join(); // real time, briefly: the refusal ends the helper at once
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    @Override
    public void loop() {
    }
}
