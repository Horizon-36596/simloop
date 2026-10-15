package org.firstinspires.ftc.teamcode.simclockfixture;

import com.pedropathing.simclockfixture.PedroStyleTimer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/** Times a sleep with a class from the Pedro Pathing package, as a team using Pedro's Timer does. */
@Autonomous(name = "Pedro User")
public class PedroUserAuto extends LinearOpMode {

    @Override
    public void runOpMode() {
        waitForStart();
        PedroStyleTimer timer = new PedroStyleTimer();
        sleep(500);
        blackboard.put("pedroSeconds", timer.seconds());
        while (opModeIsActive()) {
            idle();
        }
    }
}
