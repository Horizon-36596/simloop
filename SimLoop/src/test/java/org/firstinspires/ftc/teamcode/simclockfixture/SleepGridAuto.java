package org.firstinspires.ftc.teamcode.simclockfixture;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/** Constant sleeps: runnable both as an instance (SleepDurationReader) and via forTeamClass (hooks). */
public class SleepGridAuto extends LinearOpMode {
    private double last;
    private void mark(String key) {
        blackboard.put(key, (long) Math.round((time - last) / 0.02));
        last = time;
    }
    @Override
    public void runOpMode() {
        waitForStart();
        last = time;
        sleep(0);
        mark("s0");
        sleep(1);
        mark("s1");
        sleep(19);
        mark("s19");
        sleep(20);
        mark("s20");
        sleep(21);
        mark("s21");
        sleep(999);
        mark("s999");
        try {
            sleep(-1);
            blackboard.put("sNeg", "returned");
        } catch (RuntimeException e) {
            blackboard.put("sNeg", e.getClass().getSimpleName() + ": " + e.getMessage());
        }
        mark("sNegTicks");
        sleep(Long.MAX_VALUE);
        blackboard.put("afterMax.active", opModeIsActive());
        blackboard.put("afterMax.stopRequested", isStopRequested());
        mark("sMaxTicks");
    }
}
