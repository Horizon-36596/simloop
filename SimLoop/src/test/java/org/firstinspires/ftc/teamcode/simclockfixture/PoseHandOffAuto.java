package org.firstinspires.ftc.teamcode.simclockfixture;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Hands its end pose to the next run through the blackboard, twice: as a team type (which works on a robot,
 * where both OpModes share one class loader) and as a plain number (which works everywhere).
 */
@Autonomous(name = "Pose Hand-off")
public class PoseHandOffAuto extends LinearOpMode {

    @Override
    public void runOpMode() {
        waitForStart();
        blackboard.put("endPose", new TeamPose(42));
        blackboard.put("endX", 42.0);
    }
}
