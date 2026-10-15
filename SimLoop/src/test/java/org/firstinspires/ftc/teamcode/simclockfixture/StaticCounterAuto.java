package org.firstinspires.ftc.teamcode.simclockfixture;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/** Counts its runs in a static, as a team caching an end pose in a static field does. */
@Autonomous(name = "Static Counter")
public class StaticCounterAuto extends LinearOpMode {

    /** How many times this class's runOpMode has started, in this copy of the class. */
    public static int runs;

    @Override
    public void runOpMode() {
        runs++;
        blackboard.put("runs", runs);
        waitForStart();
    }
}
