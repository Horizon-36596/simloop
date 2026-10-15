package org.firstinspires.ftc.teamcode.simclockfixture;

import android.os.SystemClock;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Times and waits with Android's SystemClock, as some FTC code does. In a JVM test the android.jar's
 * SystemClock is a stub (it returns 0 and waits for nothing), so this only works because the rewrite
 * points every call at the sim clock and never loads SystemClock at all. ({@code elapsedRealtimeNanos()},
 * API 17, is pinned in {@code SimClockLoaderTest} with a hand-assembled class, so this file compiles against
 * any android.jar.)
 */
@Autonomous(name = "SystemClock")
public class SystemClockAuto extends LinearOpMode {

    @Override
    public void runOpMode() {
        waitForStart();
        long start = SystemClock.uptimeMillis();
        long startRealtime = SystemClock.elapsedRealtime();
        int passes = 0;
        while (opModeIsActive() && SystemClock.uptimeMillis() - start < 500) {
            passes++;
        }
        blackboard.put("passes", passes);
        blackboard.put("uptimeWaited", SystemClock.uptimeMillis() - start);
        blackboard.put("realtimeWaited", SystemClock.elapsedRealtime() - startRealtime);
        ElapsedTime timer = new ElapsedTime();
        SystemClock.sleep(300);
        blackboard.put("sleepWaited", timer.seconds());
        while (opModeIsActive()) {
            idle();
        }
    }
}
