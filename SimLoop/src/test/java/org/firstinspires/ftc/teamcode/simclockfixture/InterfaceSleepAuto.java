package org.firstinspires.ftc.teamcode.simclockfixture;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/** Calls its own sleep through a team interface: an invokeinterface, which SimLoop does not rewrite. */
@Autonomous(name = "Interface Sleep")
public class InterfaceSleepAuto extends LinearOpMode implements Waits {

    @Override
    public void runOpMode() {
        Waits waits = this;
        waitForStart();
        waits.sleep(100);
    }
}
