package org.firstinspires.ftc.teamcode.simclockfixture;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

/** An iterative OpMode that shows how long it has been since PLAY, from its own ElapsedTime. */
@TeleOp(name = "Timer TeleOp")
public class TimerTeleOp extends OpMode {

    private final ElapsedTime sincePlay = new ElapsedTime();

    @Override
    public void init() {
    }

    @Override
    public void start() {
        sincePlay.reset();
    }

    @Override
    public void loop() {
        telemetry.addData("ms", "%.1f", sincePlay.milliseconds());
    }
}
