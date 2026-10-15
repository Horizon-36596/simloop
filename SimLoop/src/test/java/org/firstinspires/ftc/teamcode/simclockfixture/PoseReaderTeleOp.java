package org.firstinspires.ftc.teamcode.simclockfixture;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/** Reads the pose {@link PoseHandOffAuto} left: the number, then the team type. */
@TeleOp(name = "Pose Reader")
public class PoseReaderTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() {
        blackboard.put("startX", blackboard.get("endX"));
        TeamPose start = (TeamPose) blackboard.get("endPose");
        blackboard.put("startPoseX", start.x);
        waitForStart();
    }
}
