package org.firstinspires.ftc.teamcode.simclockfixture;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/** Every way a team makes an ElapsedTime: all three constructors, its own subclass, and an anonymous one. */
@Autonomous(name = "ElapsedTime Every Way")
public class ElapsedTimeAuto extends LinearOpMode {

    /** A field initializer runs when the SDK builds the OpMode, before INIT. */
    private final ElapsedTime fromField = new ElapsedTime();

    @Override
    public void runOpMode() {
        ElapsedTime seconds = new ElapsedTime();
        ElapsedTime fromStart = new ElapsedTime(System.nanoTime());
        ElapsedTime millis = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        LapTimer laps = new LapTimer();
        ElapsedTime anonymous = new ElapsedTime() {
            @Override
            public String toString() {
                return "anonymous " + seconds();
            }
        };
        waitForStart();
        sleep(500);
        laps.lap();
        sleep(250);
        laps.lap();

        blackboard.put("field", fromField.seconds());
        blackboard.put("seconds", seconds.seconds());
        blackboard.put("fromStart", fromStart.seconds());
        blackboard.put("millisTime", millis.time());
        blackboard.put("laps", laps.laps());
        blackboard.put("lastLap", laps.lastLapSeconds());
        blackboard.put("anonymous", anonymous.seconds());
        blackboard.put("secondsClass", seconds.getClass().getName());
        blackboard.put("lapTimerSuper", LapTimer.class.getSuperclass().getName());
        blackboard.put("anonymousSuper", anonymous.getClass().getSuperclass().getName());
        while (opModeIsActive()) {
            idle();
        }
    }
}
