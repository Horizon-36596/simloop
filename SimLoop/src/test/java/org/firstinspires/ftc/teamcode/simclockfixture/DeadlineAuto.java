package org.firstinspires.ftc.teamcode.simclockfixture;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;

/**
 * Uses the FTC SDK's own Deadline timer, as the SDK's SensorHuskyLens and SampleRevBlinkinLedDriver samples
 * do, and a team subclass of it. Deadline is SDK code, so it keeps the real clock.
 */
@Autonomous(name = "Deadline")
public class DeadlineAuto extends LinearOpMode {

    /** A team timer built on the SDK's Deadline. */
    static final class RateLimit extends Deadline {
        RateLimit() {
            super(100, TimeUnit.MILLISECONDS);
        }
    }

    @Override
    public void runOpMode() {
        waitForStart();
        Deadline deadline = new Deadline(200, TimeUnit.MILLISECONDS);
        RateLimit limit = new RateLimit();
        sleep(1000);
        blackboard.put("deadlineExpired", deadline.hasExpired());
        blackboard.put("limitExpired", limit.hasExpired());
        while (opModeIsActive()) {
            idle();
        }
    }
}
