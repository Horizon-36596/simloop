package org.firstinspires.ftc.teamcode.simclockfixture;

import com.pedropathing.simclockfixture.PedroStyleLocalizer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/** An iterative OpMode that builds its path follower's localizer in {@code init()}, as Pedro teams do. */
@TeleOp(name = "Pedro Localizer TeleOp")
public class PedroLocalizerTeleOp extends OpMode {

    @Override
    public void init() {
        long before = System.nanoTime();
        new PedroStyleLocalizer();
        blackboard.put("buildNanos", System.nanoTime() - before);
    }

    @Override
    public void loop() {
    }
}
