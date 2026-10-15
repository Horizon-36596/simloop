package org.firstinspires.ftc.teamcode.simclockfixture;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/** Reads System.nanoTime() and System.currentTimeMillis() directly, as timing code often does. */
@Autonomous(name = "Clock Reads")
public class ClockReadsAuto extends LinearOpMode {

    @Override
    public void runOpMode() {
        long startNanos = System.nanoTime();
        long startMillis = System.currentTimeMillis();
        blackboard.put("startNanos", startNanos);
        blackboard.put("startMillis", startMillis);
        waitForStart();
        sleep(1000);
        blackboard.put("nanosAfterOneSecond", System.nanoTime() - startNanos);
        blackboard.put("millisAfterOneSecond", System.currentTimeMillis() - startMillis);
        while (opModeIsActive()) {
            telemetry.addData("nanos", System.nanoTime());
            telemetry.addData("millis", System.currentTimeMillis());
            telemetry.update();
        }
    }
}
