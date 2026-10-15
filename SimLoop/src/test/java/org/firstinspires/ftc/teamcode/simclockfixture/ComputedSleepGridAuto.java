package org.firstinspires.ftc.teamcode.simclockfixture;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/** The same grid as SleepGridAuto, but computed durations, plus Thread.sleep(ms, nanos). */
public class ComputedSleepGridAuto extends LinearOpMode {
    private static final long[] GRID = {0, 1, 19, 20, 21, 999};
    private double last;
    private void mark(String key) {
        blackboard.put(key, (long) Math.round((time - last) / 0.02));
        last = time;
    }
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        last = time;
        for (long ms : GRID) {
            sleep(ms);
            mark("s" + ms);
        }
        for (long ms : GRID) {
            Thread.sleep(ms);
            mark("t" + ms);
        }
        Thread.sleep(0, 0);
        mark("t0_0");
        Thread.sleep(0, 1);
        mark("t0_1");
        Thread.sleep(19, 999_999);
        mark("t19_999999");
        Thread.sleep(20, 0);
        mark("t20_0");
        Thread.sleep(20, 1);
        mark("t20_1");
        long neg = -GRID[1];
        try {
            sleep(neg);
            blackboard.put("sNeg", "returned");
        } catch (RuntimeException e) {
            blackboard.put("sNeg", e.getClass().getSimpleName() + ": " + e.getMessage());
        }
        try {
            Thread.sleep(0, 1_000_000);
            blackboard.put("tBadNanos", "returned");
        } catch (RuntimeException e) {
            blackboard.put("tBadNanos", e.getClass().getSimpleName() + ": " + e.getMessage());
        }
        long max = Long.MAX_VALUE - GRID[0];
        try {
            Thread.sleep(max);
            blackboard.put("tMax", "returned");
        } catch (InterruptedException e) {
            blackboard.put("tMax", "InterruptedException: " + e.getMessage());
        }
        blackboard.put("afterMax.active", opModeIsActive());
        mark("tMaxTicks");
        sleep(max);                       // after stop: must return at once
        blackboard.put("sleepAfterStop.ticks", (long) Math.round((time - last) / 0.02));
        try {
            Thread.sleep(5);
            blackboard.put("threadSleepAfterStop", "returned");
        } catch (InterruptedException e) {
            blackboard.put("threadSleepAfterStop", "InterruptedException");
        }
    }
}
