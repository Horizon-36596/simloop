package org.firstinspires.ftc.teamcode.simclockfixture;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/** When is System.nanoTime() zero, and what do INIT and PLAY first see? */
@Autonomous(name = "Start Reads")
public class StartReadsAuto extends LinearOpMode {

    private final long builtNanos = System.nanoTime();

    @Override
    public void runOpMode() {
        blackboard.put("builtNanos", builtNanos);
        blackboard.put("initNanos", System.nanoTime());
        blackboard.put("initMillis", System.currentTimeMillis());
        blackboard.put("initTime", time);
        waitForStart();
        blackboard.put("startNanos", System.nanoTime());
        while (opModeIsActive()) {
            idle();
        }
    }
}
