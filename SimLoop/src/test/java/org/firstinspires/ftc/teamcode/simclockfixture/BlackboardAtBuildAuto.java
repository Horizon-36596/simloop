package org.firstinspires.ftc.teamcode.simclockfixture;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/** Reads and writes the blackboard in a field initializer, as a TeleOp picking up an auto's end pose does. */
@Autonomous(name = "Blackboard At Build")
public class BlackboardAtBuildAuto extends LinearOpMode {

    /** What the blackboard held for "pose" when the SDK built this OpMode. */
    private final Object poseAtBuild = blackboard.get("pose");

    {
        blackboard.put("builtBy", "BlackboardAtBuildAuto");
    }

    @Override
    public void runOpMode() {
        blackboard.put("poseAtBuild", String.valueOf(poseAtBuild));
    }
}
