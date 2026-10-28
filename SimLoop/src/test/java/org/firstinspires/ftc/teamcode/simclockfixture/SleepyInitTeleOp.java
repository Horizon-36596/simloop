package org.firstinspires.ftc.teamcode.simclockfixture;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * An iterative OpMode whose {@code init()} starts a motor and then sleeps 500 ms, as a team waits for a sensor
 * to calibrate. It reports what it saw through the blackboard: the sim clock ({@code System.nanoTime()}) at
 * each lifecycle call, how the sleep ended, and the motor's encoder after it.
 */
@TeleOp(name = "Sleepy Init TeleOp")
public class SleepyInitTeleOp extends OpMode {

    private int initLoops;
    private boolean looped;

    @Override
    public void init() {
        blackboard.put("initNanos", System.nanoTime());
        DcMotor motor = hardwareMap.get(DcMotor.class, "motor");
        motor.setPower(1.0);
        try {
            Thread.sleep(500);
            blackboard.put("slept", "normally");
        } catch (InterruptedException e) {
            blackboard.put("slept", "interrupted");
        }
        blackboard.put("initReturnNanos", System.nanoTime());
        blackboard.put("runtimeAfterSleep", getRuntime());
        blackboard.put("encoderAfterSleep", motor.getCurrentPosition());
    }

    @Override
    public void init_loop() {
        if (initLoops == 0) {
            blackboard.put("firstInitLoopNanos", System.nanoTime());
        }
        initLoops++;
        blackboard.put("initLoops", initLoops);
        telemetry.addData("initLoops", initLoops);
    }

    @Override
    public void start() {
        blackboard.put("startNanos", System.nanoTime());
    }

    @Override
    public void loop() {
        if (!looped) {
            blackboard.put("firstLoopNanos", System.nanoTime());
            looped = true;
        }
    }

    @Override
    public void stop() {
        blackboard.put("stopNanos", System.nanoTime());
    }
}
