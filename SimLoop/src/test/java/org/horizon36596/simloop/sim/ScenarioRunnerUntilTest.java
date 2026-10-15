package org.horizon36596.simloop.sim;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.horizon36596.simloop.config.DrivetrainSimConfig;
import org.horizon36596.simloop.config.SimRobotConfig;

import org.junit.jupiter.api.Test;
import org.psilynx.psikit.core.LogDataReceiver;
import org.psilynx.psikit.core.LogTable;
import org.psilynx.psikit.core.Logger;

import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Map;

/**
 * Pins {@link ScenarioRunner#runUntil} on its own, apart from the live-drive test that uses it: the gate
 * decides how many ticks run, it is asked once per tick in order, extra receivers see every frame, the
 * pacer is called once per tick after it, a run of nothing is refused, and the argument checks, the
 * one-run guard and the delete of an old RLOG apply to it as they do to {@code run}. INVARIANT tier: these are what
 * a caller of an open-ended run depends on. Writes into {@code build/sim}, as {@code ScenarioRunnerTest}
 * does and for the same reason.
 */
class ScenarioRunnerUntilTest {

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
    void invariant_theGateDecidesTheLengthAndIsAskedOncePerTickInOrder() throws IOException {
        Path rlog = Paths.get("build", "sim", "scenario-runner-until-gate.rlog");
        List<Integer> asked = new ArrayList<>();
        List<Double> pacedAt = new ArrayList<>();
        int[] framesSeenByExtra = {0};
        LogDataReceiver extra = new LogDataReceiver() {
            @Override
            public void putTable(LogTable table) {
                framesSeenByExtra[0]++;
            }
        };

        int ran = ScenarioRunner.runUntil("UntilTest", STUB_CONFIG, new FakeTimer(), DT, rlog,
                tickIndex -> {
                    asked.add(tickIndex);
                    return tickIndex < 7;
                },
                (ticksCompleted, simSeconds) -> pacedAt.add(simSeconds),
                Collections.singletonList(extra),
                deltaTime -> Logger.recordOutput("Test/ran", true));

        assertEquals(7, ran);
        assertEquals(java.util.Arrays.asList(0, 1, 2, 3, 4, 5, 6, 7), asked,
                "asked for 0..6, which ran, then once for 7, which did not; never twice for one tick");
        assertEquals(7, pacedAt.size(), "the pacer runs once after each tick that ran");
        assertEquals(7 * DT, pacedAt.get(6), 1e-9, "and is told the sim time after that tick");
        assertEquals(7, framesSeenByExtra[0], "an extra receiver sees every frame the file does");
        assertEquals(7, RlogDecodedCompare.decode(rlog).size());
    }

    @Test
    void invariant_fixedTickCountRunsExactlyThatManyTicksLikeRun() throws IOException {
        Path rlog = Paths.get("build", "sim", "scenario-runner-until-fixed.rlog");
        int ran = ScenarioRunner.runUntil("UntilTest", STUB_CONFIG, new FakeTimer(), DT, rlog,
                ScenarioRunner.fixedTickCount(12), ScenarioRunner.NO_PACING,
                Collections.<LogDataReceiver>emptyList(),
                deltaTime -> Logger.recordOutput("Test/ran", true));

        assertEquals(12, ran);
        List<Map<String, String>> frames = RlogDecodedCompare.decode(rlog);
        assertEquals(12, frames.size());
    }

    @Test
    void invariant_aRunTheGateEndsBeforeItsFirstTickIsRefused() {
        Path rlog = Paths.get("build", "sim", "scenario-runner-until-empty.rlog");
        IllegalStateException refused = assertThrows(IllegalStateException.class,
                () -> ScenarioRunner.runUntil("UntilTest", STUB_CONFIG, new FakeTimer(), DT, rlog,
                        tickIndex -> false, ScenarioRunner.NO_PACING, Collections.<LogDataReceiver>emptyList(),
                        deltaTime -> Logger.recordOutput("Test/ran", true)));
        assertTrue(refused.getMessage().contains("ran no ticks"), refused.getMessage());
        assertTrue(refused.getMessage().contains("gate said stop before tick 0"), refused.getMessage());
    }

    @Test
    void runRefusesZeroTicksUpFrontAsABadArgumentNotAsAnEmptyRun() {
        // The other "no ticks" case: a ticks count below 1 is a bad argument, known before anything starts.
        Path rlog = Paths.get("build", "sim", "scenario-runner-until-zero-ticks.rlog");
        for (int ticks : new int[] {0, -3}) {
            IllegalArgumentException refused = assertThrows(IllegalArgumentException.class,
                    () -> ScenarioRunner.run("UntilTest", STUB_CONFIG, new FakeTimer(), ticks, DT, rlog,
                            deltaTime -> Logger.recordOutput("Test/ran", true)));
            assertTrue(refused.getMessage().contains("ticks must be at least 1, got " + ticks), refused.getMessage());
        }
    }

    @Test
    void runUntilChecksItsArgumentsBeforeAnythingRuns() {
        Path rlog = Paths.get("build", "sim", "scenario-runner-until-args.rlog");
        List<LogDataReceiver> none = Collections.<LogDataReceiver>emptyList();
        ScenarioRunner.ScenarioBody body = deltaTime -> Logger.recordOutput("Test/ran", true);
        ScenarioRunner.TickGate gate = ScenarioRunner.fixedTickCount(3);
        FakeTimer timer = new FakeTimer();

        IllegalArgumentException notRlog = assertThrows(IllegalArgumentException.class,
                () -> ScenarioRunner.runUntil("Bad", STUB_CONFIG, timer, DT,
                        Paths.get("build", "sim", "scenario-runner-until-args.log"), gate, ScenarioRunner.NO_PACING,
                        none, body));
        assertTrue(notRlog.getMessage().contains(".rlog"), notRlog.getMessage());
        assertThrows(IllegalArgumentException.class, () -> ScenarioRunner.runUntil("Bad", STUB_CONFIG, timer, 0.0,
                rlog, gate, ScenarioRunner.NO_PACING, none, body));
        assertThrows(IllegalArgumentException.class, () -> ScenarioRunner.runUntil("Bad", STUB_CONFIG, timer, DT,
                rlog, null, ScenarioRunner.NO_PACING, none, body));
        assertThrows(IllegalArgumentException.class, () -> ScenarioRunner.runUntil("Bad", STUB_CONFIG, timer, DT,
                rlog, gate, null, none, body));
        assertThrows(IllegalArgumentException.class, () -> ScenarioRunner.runUntil("Bad", STUB_CONFIG, timer, DT,
                rlog, gate, ScenarioRunner.NO_PACING, null, body));
        assertEquals(0.0, timer.time(), "a refused run must not have advanced the clock");
    }

    @Test
    void aRunUntilStartedFromInsideAnotherRunIsRefused() {
        // The one-run-at-a-time guard lives in runUntil, so a live session cannot start inside a scenario.
        Path outer = Paths.get("build", "sim", "scenario-runner-until-outer.rlog");
        Path inner = Paths.get("build", "sim", "scenario-runner-until-inner.rlog");
        IllegalStateException refused = assertThrows(IllegalStateException.class,
                () -> ScenarioRunner.run("Outer", STUB_CONFIG, new FakeTimer(), 2, DT, outer,
                        deltaTime -> ScenarioRunner.runUntil("Inner", STUB_CONFIG, new FakeTimer(), DT, inner,
                                ScenarioRunner.fixedTickCount(1), ScenarioRunner.NO_PACING,
                                Collections.<LogDataReceiver>emptyList(),
                                innerDelta -> Logger.recordOutput("Test/ran", true))));
        assertTrue(refused.getMessage().contains("started from inside another logged run's tick"), refused.getMessage());
    }

    @Test
    void aSecondRunUntilToTheSamePathReplacesTheFirst() throws IOException {
        // runUntil deletes the old file itself, so a shorter second session is not appended to a longer first.
        Path rlog = Paths.get("build", "sim", "scenario-runner-until-same-path.rlog");
        ScenarioRunner.runUntil("First", STUB_CONFIG, new FakeTimer(), DT, rlog, ScenarioRunner.fixedTickCount(9),
                ScenarioRunner.NO_PACING, Collections.<LogDataReceiver>emptyList(),
                deltaTime -> Logger.recordOutput("Test/ran", true));
        ScenarioRunner.runUntil("Second", STUB_CONFIG, new FakeTimer(), DT, rlog, ScenarioRunner.fixedTickCount(4),
                ScenarioRunner.NO_PACING, Collections.<LogDataReceiver>emptyList(),
                deltaTime -> Logger.recordOutput("Test/ran", true));

        assertEquals(4, RlogDecodedCompare.decode(rlog).size());
    }
}
