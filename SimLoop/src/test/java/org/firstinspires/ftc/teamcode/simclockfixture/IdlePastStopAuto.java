package org.firstinspires.ftc.teamcode.simclockfixture;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Loops on idle() with nothing that sees STOP. On a robot STOP would end it at the SDK's watchdog; in the sim,
 * after STOP, idle() no longer ends a tick, so the run fails as stuck and must then end the thread.
 */
@Autonomous(name = "Idle Past Stop")
public class IdlePastStopAuto extends LinearOpMode {

    /** Never cleared: the loop below has no exit of its own. */
    private boolean keepGoing = true;

    @Override
    public void runOpMode() {
        waitForStart();
        while (keepGoing) {
            idle();
        }
    }
}
