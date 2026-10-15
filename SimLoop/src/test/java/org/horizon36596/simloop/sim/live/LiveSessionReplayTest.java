package org.horizon36596.simloop.sim.live;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.horizon36596.simloop.config.DrivetrainSimConfig;
import org.horizon36596.simloop.config.SimRobotConfig;
import org.horizon36596.simloop.sim.FakeTimer;
import org.horizon36596.simloop.sim.RlogDecodedCompare;
import org.horizon36596.simloop.sim.ScenarioRunner;

import org.junit.jupiter.api.Test;
import org.psilynx.psikit.core.LogDataReceiver;
import org.psilynx.psikit.core.LogTable;
import org.psilynx.psikit.core.Logger;

import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.Arrays;
import java.util.Collections;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.function.LongSupplier;

/**
 * The live-drive contract at the SimLoop level, with a toy robot instead of a season's: a session driven
 * from a (scripted) keyboard, paced to "real time", recorded as it runs, replays headless to an identical
 * decoded RLOG - twice. INVARIANT tier: this is the property that makes a live session worth recording.
 *
 * <p><b>Why the live run is paced and the replays are not.</b> That is exactly the difference between a
 * live session and its replay, so it is the difference this test must survive. The pacer runs on a clock
 * this test owns, so the test does not actually sleep; the point is that the pacer is in the loop.
 *
 * <p>A team's own version of this test runs its real TeleOp; this one proves the core pieces
 * compose without any season code, which is what a public SimLoop user gets.
 */
class LiveSessionReplayTest {

    private static final double DT = 0.02;

    private static final SimRobotConfig STUB_CONFIG = new SimRobotConfig() {
        @Override public String[] driveMotorNames() { return new String[] {"fl", "fr", "bl", "br"}; }
        @Override public String odometryName() { return "odo"; }
        @Override public DrivetrainSimConfig drivetrain() {
            return new DrivetrainSimConfig() {
                @Override public double trackWidth() { return 12.0; }
                @Override public double wheelBase() { return 12.0; }
                @Override public double wheelRadius() { return 2.0; }
                @Override public double ticksPerInch() { return 100.0; }
                @Override public double maxVelocityTicksPerSecond() { return 1000.0; }
                @Override public double maxAccel() { return 2.0; }
                @Override public double fieldHalfWidth() { return 72.0; }
                @Override public double fieldHalfHeight() { return 72.0; }
            };
        }
    };

    @Test
    void invariant_aPacedRecordedSessionReplaysTwiceToIdenticalDecodedRlogs() throws IOException {
        Path track = Paths.get("build", "sim", "live-session-test.track.txt");
        Path liveRlog = Paths.get("build", "sim", "live-session-test.rlog");
        Path replay1 = Paths.get("build", "sim", "live-session-test-replay1.rlog");
        Path replay2 = Paths.get("build", "sim", "live-session-test-replay2.rlog");

        int[] framesTheExtraReceiverSaw = {0};
        LogDataReceiver countingReceiver = new LogDataReceiver() {
            @Override
            public void putTable(LogTable table) {
                framesTheExtraReceiverSaw[0]++;
            }
        };

        int liveTicks;
        try (GamepadInputTrack.Writer writer = new GamepadInputTrack.Writer(track,
                Collections.singletonMap("opMode", "ToyRobot"), true)) {
            GamepadInputSource keyboard = new RecordingGamepadInputSource(
                    new KeyboardGamepadSource(new ScriptedKeyboard(), 10_000), writer);
            liveTicks = runToyRobot(keyboard, liveRlog, pacerOnATestClock(),
                    Collections.singletonList(countingReceiver));
        }
        GamepadInputTrack recorded = GamepadInputTrack.read(track);
        int replayTicks1 = runToyRobot(recorded.replay(), replay1, ScenarioRunner.NO_PACING,
                Collections.<LogDataReceiver>emptyList());
        int replayTicks2 = runToyRobot(recorded.replay(), replay2, ScenarioRunner.NO_PACING,
                Collections.<LogDataReceiver>emptyList());

        assertEquals(ScriptedKeyboard.ESCAPE_AT_TICK, liveTicks, "the session ends on the loop escape is pressed");
        assertEquals(liveTicks, recorded.frames().size(), "one track line per loop that ran");
        assertEquals(liveTicks, replayTicks1);
        assertEquals(liveTicks, replayTicks2);
        assertEquals(liveTicks, framesTheExtraReceiverSaw[0], "an extra receiver sees every frame the file does");

        List<RlogDecodedCompare.Mismatch> liveVsReplay = RlogDecodedCompare.compare(liveRlog, replay1);
        List<RlogDecodedCompare.Mismatch> replayVsReplay = RlogDecodedCompare.compare(replay1, replay2);
        assertTrue(liveVsReplay.isEmpty(), "live vs replay: " + RlogDecodedCompare.summarise(liveVsReplay));
        assertTrue(replayVsReplay.isEmpty(), "replay vs replay: " + RlogDecodedCompare.summarise(replayVsReplay));

        // Not vacuous: the toy robot really moved, so an identical log is not an identical log of nothing.
        List<java.util.Map<String, String>> frames = RlogDecodedCompare.decode(liveRlog);
        double finalX = Double.parseDouble(frames.get(frames.size() - 1).get("RealOutputs/Toy/xInches"));
        assertTrue(finalX > 10.0, "the toy robot must have driven forward, got x = " + finalX);
    }

    /**
     * A robot small enough to read at a glance: it reads the two SDK gamepads the way an OpMode would and
     * integrates a pose. Everything it logs is a function of the gamepads and the fixed tick only.
     */
    private static int runToyRobot(GamepadInputSource input, Path rlog, ScenarioRunner.TickPacer pacer,
            List<LogDataReceiver> extraReceivers) {
        FakeTimer timer = new FakeTimer();
        Gamepad gamepad1 = new Gamepad();
        Gamepad gamepad2 = new Gamepad();
        double[] pose = {0.0, 0.0};
        int[] presses = {0};
        GamepadInputGate gamepads = new GamepadInputGate(input);

        return ScenarioRunner.runUntil("ToyRobot", STUB_CONFIG, timer, DT, rlog, gamepads, pacer, extraReceivers,
                deltaTime -> {
                    gamepads.current().gamepad1.applyTo(gamepad1);
                    gamepads.current().gamepad2.applyTo(gamepad2);
                    double speed = 40.0 * (1.0 + gamepad1.right_trigger);
                    pose[0] += -gamepad1.left_stick_y * speed * deltaTime;
                    pose[1] += -gamepad1.left_stick_x * speed * deltaTime;
                    if (gamepad2.crossWasPressed()) {
                        presses[0]++;
                    }
                    Logger.recordOutput("Toy/xInches", pose[0]);
                    Logger.recordOutput("Toy/yInches", pose[1]);
                    Logger.recordOutput("Toy/operatorCrossPresses", presses[0]);
                    Logger.recordOutput("Toy/leftTriggerPressed", gamepad1.left_trigger_pressed);
                });
    }

    /** A person at the keyboard, as a script of which keys are down on which loop. */
    private static final class ScriptedKeyboard implements KeyboardGamepadSource.Keyboard {
        static final int ESCAPE_AT_TICK = 90;
        private int calls = 0;

        @Override
        public Set<String> keysDown() {
            int tick = calls++;
            if (tick < 30) {
                return keys("W");                          // forward
            } else if (tick < 45) {
                return keys("W", "D", "SHIFT", "C");       // forward and right, half travel, trigger
            } else if (tick < 50) {
                return keys("TAB");                         // over to gamepad 2
            } else if (tick < 60) {
                return tick % 4 == 0 ? keys("1") : keys(); // the operator taps cross
            } else if (tick < 65) {
                return keys("TAB");                         // back to gamepad 1
            } else if (tick < ESCAPE_AT_TICK) {
                return keys("S", "Z");                      // back up a little, left trigger in
            }
            return keys("ESCAPE");
        }

        @Override
        public boolean isOpen() {
            return true;
        }

        private static Set<String> keys(String... names) {
            return new HashSet<>(Arrays.asList(names));
        }
    }

    /** A real-time pacer whose wall clock moves only when it sleeps, so the test never really waits. */
    private static RealTimePacer pacerOnATestClock() {
        TestClock clock = new TestClock();
        return new RealTimePacer(clock, clock);
    }

    /** A wall clock that moves only when the pacer sleeps on it. */
    private static final class TestClock implements LongSupplier, RealTimePacer.Sleeper {
        private long nanos = 0L;

        @Override public long getAsLong() { return nanos; }

        @Override public void sleepNanos(long requested) { nanos += requested; }
    }
}
