package org.firstinspires.ftc.teamcode.simclockfixture;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/** Waits with Object.wait and lets its InterruptedException out of runOpMode(), as the SDK allows. */
@Autonomous(name = "JDK Wait Uncaught")
public class JdkWaitUncaughtAuto extends LinearOpMode {

    private final Object monitor = new Object();

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        synchronized (monitor) {
            monitor.wait(300);
        }
    }
}
