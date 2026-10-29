package org.horizon36596.simloop.opmode;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.horizon36596.simloop.fakehardware.FakeHardwareMap;
import org.horizon36596.simloop.sim.FakeTimer;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

/**
 * Pins how {@link OpModeRunner} steps an iterative {@link OpMode}: the SDK's call order, one call per tick,
 * gamepads delivered the way the SDK delivers them, telemetry updated after every call, the sim clock behind
 * {@code time} and {@code getRuntime()}, and failures surfaced with where they happened.
 *
 * <p>Invariant tier: the lifecycle order is the SDK's (RobotCore 12.0.0 {@code OpMode.internalRunOpMode}),
 * and a team's iterative OpMode depends on exactly it.
 */
class IterativeOpModeRunnerTest {

    private static final double TICK = 0.02;

    /** Records every lifecycle call, with the sim time and what the gamepad said. */
    static class Recorder extends OpMode {
        final FakeTimer timer;
        final List<String> calls = new ArrayList<>();
        final List<Double> runtimes = new ArrayList<>();
        final List<Double> times = new ArrayList<>();

        Recorder(FakeTimer timer) {
            this.timer = timer;
        }

        @Override public void init() { record("init"); }
        @Override public void init_loop() { record("init_loop"); }
        @Override public void start() { record("start"); }
        @Override public void loop() { record("loop"); }
        @Override public void stop() { record("stop"); }

        void record(String call) {
            calls.add(call);
            runtimes.add(getRuntime());
            times.add(time);
        }
    }

    private static OpModeRunner runner(OpMode opMode, FakeTimer timer) {
        return new OpModeRunner(opMode, new FakeHardwareMap()).timer(timer).tickSeconds(TICK);
    }

    @Test
    void theLifecycleRunsInTheSdksOrderOneCallPerTickWithStartAndTheFirstLoopTogether() {
        FakeTimer timer = new FakeTimer();
        Recorder opMode = new Recorder(timer);

        OpModeRunResult result = runner(opMode, timer).initLoopTicks(3).activeTicks(4).run();

        List<String> expected = new ArrayList<>();
        expected.add("init");
        expected.addAll(Collections.nCopies(3, "init_loop"));
        expected.add("start");
        expected.addAll(Collections.nCopies(4, "loop"));
        expected.add("stop");
        assertEquals(expected, opMode.calls);
        assertEquals(1 + 3 + 4 + 1, result.ticks());
        assertFalse(result.endedBeforeStop());
        assertEquals(result.ticks() * TICK, timer.time(), 1e-12, "the clock advanced once per tick");
    }

    @Test
    void zeroActiveTicksMeansStopDuringInitAndStartNeverRuns() {
        FakeTimer timer = new FakeTimer();
        Recorder opMode = new Recorder(timer);
        runner(opMode, timer).initLoopTicks(2).activeTicks(0).run();
        assertEquals(Arrays.asList("init", "init_loop", "init_loop", "stop"), opMode.calls);
    }

    @Test
    void timeAndGetRuntimeFollowTheSimClockFromInit() {
        FakeTimer timer = new FakeTimer();
        Recorder opMode = new Recorder(timer);
        runner(opMode, timer).initLoopTicks(1).activeTicks(50).run();

        // init at runtime 0; each later tick 0.02 s on. start() and the first loop() share tick 2.
        assertEquals(0.0, opMode.times.get(0), 1e-12);
        assertEquals(TICK, opMode.times.get(1), 1e-12);
        double lastLoop = opMode.times.get(opMode.times.size() - 2);
        assertEquals(51 * TICK, lastLoop, 1e-9, "the 50th loop is 51 ticks after init");
        for (int i = 0; i < opMode.times.size(); i++) {
            // getRuntime() is the same sim time plus the real time this tick's code has taken so far:
            // microseconds normally, more under a GC pause, but never the whole run's real time. A wall-clock
            // getRuntime() would read a few milliseconds at the end of this one-second (simulated) run.
            double lead = opMode.runtimes.get(i) - opMode.times.get(i);
            assertTrue(lead >= 0 && lead < 0.25, "getRuntime() tracks sim time, call " + i + " led by " + lead);
        }
    }

    @Test
    void resetRuntimeRestartsTheSimRuntime() {
        FakeTimer timer = new FakeTimer();
        List<Double> times = new ArrayList<>();
        OpMode opMode = new OpMode() {
            @Override public void init() { }
            @Override public void start() { resetRuntime(); }
            @Override public void loop() { times.add(time); }
        };
        runner(opMode, timer).initLoopTicks(10).activeTicks(3).run();
        // start() reset the runtime on the first ACTIVE tick, and loop() ran on that same tick.
        assertEquals(0.0, times.get(0), 1e-12);
        assertEquals(TICK, times.get(1), 1e-12);
        assertEquals(2 * TICK, times.get(2), 1e-12);
    }

    @Test
    void theSdkUpdatesTelemetryAfterEveryCallSoAnOpModeNeverHasTo() {
        OpMode opMode = new OpMode() {
            int loops;
            @Override public void init() { telemetry.addData("status", "ready"); }
            @Override public void loop() { telemetry.addData("loop", ++loops); }
        };
        OpModeRunResult result = runner(opMode, new FakeTimer()).activeTicks(3).run();
        List<List<String>> sent = result.telemetry().transmissions();
        // init, start, 3 loops, stop - each followed by one update.
        assertEquals(6, sent.size());
        assertEquals(Collections.singletonList("status : ready"), sent.get(0));
        assertEquals(Collections.singletonList("loop : 3"), sent.get(4));
        assertEquals(Collections.singletonList("loop : 3"), sent.get(5),
                "stop() added nothing, so the Driver Station still shows the last loop, as on the robot");
    }

    @Test
    void anOpModeThatUpdatesTelemetryItselfStillEndsOnWhatItShowed() {
        OpMode opMode = new OpMode() {
            int loops;
            @Override public void init() { }
            @Override public void loop() {
                telemetry.addData("loop", ++loops);
                telemetry.update(); // unneeded, since the SDK updates after loop(), but very common
            }
        };
        OpModeRunResult result = runner(opMode, new FakeTimer()).activeTicks(3).run();
        assertEquals(Collections.singletonList("loop : 3"), result.telemetry().lastTransmission(),
                "the SDK's own update after the OpMode's does not blank the Driver Station");
    }

    /** A TeleOp that records what it read from the gamepads. */
    static class GamepadReader extends OpMode {
        final List<Float> sticks = new ArrayList<>();
        int presses;
        int held;

        @Override public void init() { }

        @Override
        public void loop() {
            sticks.add(gamepad1.left_stick_y);
            if (gamepad2.aWasPressed()) {
                presses++;
            }
            if (gamepad2.a) {
                held++;
            }
        }
    }

    @Test
    void theScriptDrivesTheGamepadsTickByTickAndEdgeDetectionSeesOnePressPerPress() {
        GamepadReader opMode = new GamepadReader();
        GamepadScript script = (phase, tick, gamepad1, gamepad2) -> {
            if (phase != OpModePhase.ACTIVE) {
                return;
            }
            gamepad1.left_stick_y = -tick / 10f;
            gamepad2.a = (tick >= 2 && tick < 5) || tick == 7; // held for 3 ticks, then tapped once
        };
        runner(opMode, new FakeTimer()).activeTicks(10).gamepads(script).run();

        assertEquals(10, opMode.sticks.size());
        for (int tick = 0; tick < 10; tick++) {
            assertEquals(-tick / 10f, opMode.sticks.get(tick), 0f, "stick at ACTIVE tick " + tick);
        }
        assertEquals(4, opMode.held, "a was down on four ticks");
        assertEquals(2, opMode.presses, "aWasPressed() fires once per press, not once per tick held");
    }

    /** Records each tick what the OpMode sees of one button, under both its names. */
    static class TriangleReader extends OpMode {
        final List<String> seen = new ArrayList<>();
        int trianglePresses, yPresses, sharePresses;

        @Override public void init() { }

        @Override
        public void loop() {
            seen.add((gamepad1.triangle ? "T" : "-") + (gamepad1.y ? "Y" : "-") + (gamepad1.back ? "B" : "-")
                    + (gamepad1.share ? "S" : "-"));
            if (gamepad1.triangleWasPressed()) {
                trianglePresses++;
            }
            if (gamepad1.yWasPressed()) {
                yPresses++;
            }
            if (gamepad1.shareWasPressed()) {
                sharePresses++;
            }
        }
    }

    @Test
    void aButtonPressedByItsPlayStationNameReachesTheOpModeUnderBothNamesAndReleases() {
        TriangleReader opMode = new TriangleReader();
        GamepadScript script = (phase, tick, gamepad1, gamepad2) -> {
            if (phase != OpModePhase.ACTIVE) {
                return;
            }
            gamepad1.triangle = tick >= 1 && tick < 3; // pressed by its PlayStation name for two ticks
            gamepad1.share = tick == 5;                 // and share tapped once
        };
        runner(opMode, new FakeTimer()).activeTicks(7).gamepads(script).run();

        assertEquals(Arrays.asList("----", "TY--", "TY--", "----", "----", "--BS", "----"), opMode.seen,
                "triangle and y are one button, and so are share and back; released when the script lets go");
        assertEquals(1, opMode.trianglePresses, "triangleWasPressed() once");
        assertEquals(1, opMode.yPresses, "yWasPressed() once: it is the same button");
        assertEquals(1, opMode.sharePresses, "shareWasPressed() once");
    }

    @Test
    void requestOpModeStopRunsStopAtOnceAndNothingAfterIt() {
        FakeTimer timer = new FakeTimer();
        Recorder opMode = new Recorder(timer) {
            int loops;
            @Override public void loop() {
                super.loop();
                if (++loops == 2) {
                    requestOpModeStop();
                }
            }
        };
        OpModeRunResult result = runner(opMode, timer).activeTicks(10).run();
        assertEquals(Arrays.asList("init", "start", "loop", "loop", "stop"), opMode.calls);
        assertTrue(result.endedBeforeStop());
        assertEquals(2, result.endedAtTick(), "the second loop is on tick 2");
        assertEquals(result.ticks() * TICK, timer.time(), 1e-12, "the world kept ticking to the end");
    }

    @Test
    void terminateOpModeNowEndsTheOpModeWithoutStop() {
        FakeTimer timer = new FakeTimer();
        Recorder opMode = new Recorder(timer) {
            @Override public void loop() {
                super.loop();
                terminateOpModeNow();
            }
        };
        OpModeRunResult result = runner(opMode, timer).activeTicks(10).run();
        assertEquals(Arrays.asList("init", "start", "loop"), opMode.calls);
        assertTrue(result.endedBeforeStop());
    }

    @Test
    void anExceptionInLoopFailsTheRunNamingTheMethodPhaseAndTick() {
        IllegalStateException bug = new IllegalStateException("null motor");
        OpMode opMode = new OpMode() {
            int loops;
            @Override public void init() { }
            @Override public void loop() {
                if (++loops == 3) {
                    throw bug;
                }
            }
        };
        OpModeException failure = assertThrows(OpModeException.class,
                () -> runner(opMode, new FakeTimer()).initLoopTicks(1).activeTicks(10).run());
        assertSame(bug, failure.getCause());
        assertEquals(OpModePhase.ACTIVE, failure.phase());
        assertEquals(4, failure.tick(), "INIT 0, INIT_LOOP 1, loops on 2, 3, 4");
        assertTrue(failure.getMessage().contains("loop()"), failure.getMessage());
    }

    @Test
    void theOpModesFieldsAreTheRunnersFromInitOn() {
        FakeHardwareMap hardwareMap = new FakeHardwareMap();
        List<Object> seen = new ArrayList<>();
        OpMode opMode = new OpMode() {
            @Override public void init() {
                seen.add(hardwareMap);
                seen.add(telemetry);
            }
            @Override public void loop() { }
        };
        assertEquals(null, opMode.hardwareMap, "the SDK assigns hardwareMap after construction");
        OpModeRunResult result = new OpModeRunner(opMode, hardwareMap).run();
        assertSame(hardwareMap, seen.get(0));
        assertSame(result.telemetry(), seen.get(1));
    }

    @Test
    void badSettingsAreRefusedBeforeAnythingRuns() {
        OpModeRunner runner = new OpModeRunner(new Recorder(new FakeTimer()), new FakeHardwareMap());
        assertThrows(IllegalArgumentException.class, () -> runner.activeTicks(-1));
        assertThrows(IllegalArgumentException.class, () -> runner.initLoopTicks(-1));
        assertThrows(IllegalArgumentException.class, () -> runner.tickSeconds(0));
        assertThrows(IllegalArgumentException.class, () -> runner.tickSeconds(Double.NaN));
        assertThrows(IllegalArgumentException.class, () -> runner.gamepads(null));
        assertThrows(IllegalArgumentException.class, () -> runner.stuckTimeoutMillis(0));
        assertThrows(IllegalArgumentException.class, () -> new OpModeRunner(null, new FakeHardwareMap()));
    }
}
