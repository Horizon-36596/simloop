package org.firstinspires.ftc.teamcode.simclockfixture;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/** Waits with Object.wait, which SimLoop does not move onto sim time. */
@Autonomous(name = "Object Wait")
public class ObjectWaitAuto extends LinearOpMode {

    private final Object monitor = new Object();

    @Override
    public void runOpMode() {
        waitForStart();
        ElapsedTime timer = new ElapsedTime();
        try {
            synchronized (monitor) {
                monitor.wait(300);
            }
            blackboard.put("objectWait", "returned");
        } catch (InterruptedException e) {
            blackboard.put("objectWait", "interrupted");
        }
        blackboard.put("objectWaited", timer.seconds());
        while (opModeIsActive()) {
            idle();
        }
    }
}
