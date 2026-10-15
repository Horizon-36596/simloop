package org.horizon36596.simloop.sim;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.horizon36596.simloop.config.DrivetrainSimConfig;
import org.horizon36596.simloop.config.SimRobotConfig;

import org.junit.jupiter.api.Test;
import org.psilynx.psikit.core.Logger;

import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;

/**
 * Pins Contract clause 1 (area B, {@code docs/process/parallel-chat-prompts.md}): a scenario runs headless
 * from one {@link ScenarioRunner#run} call -- naming the OpMode, the robot's sim config, and a duration --
 * writes one RLOG at a caller-chosen path with no bespoke per-test Logger/tick-loop code (this test file has
 * none), and rerunning the same scenario produces decoded-field-identical RLOGs (R5, via
 * {@link RlogDecodedCompare}). Season-agnostic: uses a minimal stub {@link SimRobotConfig}, not a real robot.
 *
 * <p>Writes into {@code build/sim} rather than a JUnit {@code @TempDir}. The original reason: on Windows a
 * still-open file handle made JUnit's post-test {@code @TempDir} cleanup fail with
 * {@code FileSystemException} (observed running this exact test). That was blamed on PsiKit's writer
 * thread, but the handle actually left open was {@link RlogDecodedCompare#decode}'s: it opened the file to
 * read it back and never closed it, while {@code Logger.end()} joins the writer thread, which closes its
 * own file first. The decode has closed its file since 2026-09-26 (pinned by
 * {@code RlogDecodedCompareTest.decodingAFileReleasesIt}); {@code build/sim} is kept because it is where a
 * human opens the RLOGs, and every other sim test writes there.
 */
class ScenarioRunnerTest {

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

    /** One logged run that records {@code id} every tick, and the ids its RLOG decodes to. */
    private static List<String> loggedIds(int id, int round) throws IOException {
        Path rlog = Paths.get("build", "sim", "scenario-runner-overlap-" + round + "-" + id + ".rlog");
        ScenarioRunner.run("Overlap" + id, STUB_CONFIG, new FakeTimer(), 150, DT, rlog,
                deltaTimeSeconds -> Logger.recordOutput("Test/id", id));
        List<String> ids = new ArrayList<>();
        for (Map<String, String> frame : RlogDecodedCompare.decode(rlog)) {
            ids.add(frame.get("RealOutputs/Test/id"));
        }
        return ids;
    }

    @Test
    void twoLoggedRunsStartedAtOnceTakeTurnsAndEachLogHoldsOnlyItsOwnRun() throws Exception {
        // PsiKit's Logger is one static: overlapping runs used to write into each other's RLOG, and one could
        // return normally with a log that was mostly the other's.
        ExecutorService pool = Executors.newFixedThreadPool(2);
        try {
            for (int round = 0; round < 3; round++) {
                final int r = round;
                Future<List<String>> one = pool.submit(() -> loggedIds(1, r));
                Future<List<String>> two = pool.submit(() -> loggedIds(2, r));
                for (int id = 1; id <= 2; id++) {
                    List<String> ids = (id == 1 ? one : two).get();
                    assertEquals(150, ids.size(), "round " + round + ", run " + id + ": one frame per tick");
                    for (String seen : ids) {
                        assertEquals(String.valueOf(id), String.valueOf(seen).replace(".0", ""),
                                "round " + round + ": run " + id + "'s log holds only its own frames");
                    }
                }
            }
        } finally {
            pool.shutdownNow();
        }
    }

    @Test
    void aLoggedRunStartedInsideAnotherOnesTickIsRefused() {
        Path outer = Paths.get("build", "sim", "scenario-runner-outer.rlog");
        IllegalStateException refused = assertThrows(IllegalStateException.class,
                () -> ScenarioRunner.run("Outer", STUB_CONFIG, new FakeTimer(), 3, DT, outer,
                        deltaTimeSeconds -> ScenarioRunner.run("Inner", STUB_CONFIG, new FakeTimer(), 3, DT,
                                Paths.get("build", "sim", "scenario-runner-inner.rlog"), dt -> { })));
        assertTrue(refused.getMessage().contains("'Inner' was started from inside another logged run's tick"),
                refused.getMessage());
    }

    @Test
    void runsHeadlessAndWritesOneRlogAtTheCallerChosenPath() throws IOException {
        Path rlogPath = Paths.get("build", "sim", "scenario-runner-test-1.rlog");
        FakeTimer timer = new FakeTimer();
        int ticks = 10;
        double[] loggedTimes = new double[ticks];
        int[] tickIndex = {0};

        ScenarioRunner.run("ScenarioRunnerTest", STUB_CONFIG, timer, ticks, DT, rlogPath, deltaTimeSeconds -> {
            loggedTimes[tickIndex[0]++] = timer.time();
            Logger.recordOutput("Test/time", timer.time());
        });

        // Decodes the RLOG independently of ScenarioRunner.run()'s own exists/non-empty guard (BACKLOG
        // B11): that guard only proves *some* bytes reached disk, not that every tick's fields decode
        // back out, so this checks decoded content instead of re-asserting the same postcondition.
        List<Map<String, String>> frames = RlogDecodedCompare.decode(rlogPath);
        assertEquals(ticks, frames.size(), "one decoded frame must exist per tick");
        assertEquals(loggedTimes[0], Double.parseDouble(frames.get(0).get("RealOutputs/Test/time")), 1e-9,
                "first frame must decode back the first tick's logged field");
        assertEquals(loggedTimes[ticks - 1],
                Double.parseDouble(frames.get(frames.size() - 1).get("RealOutputs/Test/time")), 1e-9,
                "last frame must decode back the last tick's logged field");
    }

    @Test
    void sameScenarioTwiceProducesDecodedFieldIdenticalRlogs() throws IOException {
        Path rlog1 = Paths.get("build", "sim", "scenario-runner-test-determinism-1.rlog");
        Path rlog2 = Paths.get("build", "sim", "scenario-runner-test-determinism-2.rlog");

        // No queue-fault assertions here: ScenarioRunner.run already latches PsiKit's fault flag every tick
        // and throws, so an assertion on it afterwards could never fail on its own.
        runCountingScenario(rlog1);
        runCountingScenario(rlog2);

        List<RlogDecodedCompare.Mismatch> mismatches = RlogDecodedCompare.compare(rlog1, rlog2);
        assertTrue(mismatches.isEmpty(),
                "identical scenarios must produce decoded-field-identical RLOGs, got: " + mismatches);
    }

    /**
     * Two runs to the SAME path in one JVM each leave exactly their own frames in the file.
     *
     * <p>The failure this pins: {@link RlogDecodedCompare#decode} used to open the file and never close it,
     * and {@code run} decodes every file it writes. PsiKit's {@code RLOGWriter.start()} then deletes the old
     * file, ignores the result, and opens it in APPEND mode - so on Windows, where a file another stream
     * still holds open cannot be deleted, the second run was written after the first run's frames. The two
     * runs use different tick counts so an appended file cannot pass by coincidence.
     *
     * <p>This only discriminates on Windows: Linux and macOS let a file that is still open be deleted, so
     * there the old leak never caused an append and this test passes with or without the fix.
     */
    @Test
    void twoRunsToTheSamePathEachLeaveOnlyTheirOwnFrames() throws IOException {
        Path rlogPath = Paths.get("build", "sim", "scenario-runner-test-same-path.rlog");

        runTicks(rlogPath, 30);
        assertEquals(30, RlogDecodedCompare.decode(rlogPath).size(), "first run: one frame per tick");

        runTicks(rlogPath, 20);
        assertEquals(20, RlogDecodedCompare.decode(rlogPath).size(),
                "second run to the same path must replace the first run's file, not append to it");
    }

    @Test
    void anRlogPathThatDoesNotEndInDotRlogIsRejected() {
        IllegalArgumentException thrown = assertThrows(IllegalArgumentException.class,
                () -> runTicks(Paths.get("build", "sim", "scenario-runner-test-wrong-suffix.log"), 5));
        assertTrue(thrown.getMessage().contains(".rlog"), thrown.getMessage());
    }

    @Test
    void invalidArgumentsAreRejectedBeforeAnythingRuns() {
        Path rlogPath = Paths.get("build", "sim", "scenario-runner-test-invalid.rlog");
        FakeTimer timer = new FakeTimer();
        ScenarioRunner.ScenarioBody body = deltaTimeSeconds -> { };

        assertThrows(IllegalArgumentException.class,
                () -> ScenarioRunner.run("Bad", STUB_CONFIG, timer, 0, DT, rlogPath, body), "zero ticks");
        assertThrows(IllegalArgumentException.class,
                () -> ScenarioRunner.run("Bad", STUB_CONFIG, timer, -1, DT, rlogPath, body), "negative ticks");
        assertThrows(IllegalArgumentException.class,
                () -> ScenarioRunner.run("Bad", STUB_CONFIG, timer, 5, 0.0, rlogPath, body), "zero step");
        assertThrows(IllegalArgumentException.class,
                () -> ScenarioRunner.run("Bad", STUB_CONFIG, timer, 5, -DT, rlogPath, body), "negative step");
        assertThrows(IllegalArgumentException.class,
                () -> ScenarioRunner.run("Bad", STUB_CONFIG, timer, 5, Double.NaN, rlogPath, body), "NaN step");
        assertThrows(IllegalArgumentException.class, () -> ScenarioRunner.run("Bad", STUB_CONFIG, timer, 5,
                Double.POSITIVE_INFINITY, rlogPath, body), "infinite step");
        assertThrows(IllegalArgumentException.class,
                () -> ScenarioRunner.run("Bad", null, timer, 5, DT, rlogPath, body), "null config");
        assertThrows(IllegalArgumentException.class,
                () -> ScenarioRunner.run("Bad", STUB_CONFIG, null, 5, DT, rlogPath, body), "null timer");
        assertThrows(IllegalArgumentException.class,
                () -> ScenarioRunner.run("Bad", STUB_CONFIG, timer, 5, DT, null, body), "null path");
        assertThrows(IllegalArgumentException.class,
                () -> ScenarioRunner.run("Bad", STUB_CONFIG, timer, 5, DT, rlogPath, null), "null body");
        assertEquals(0.0, timer.time(), 0.0, "a rejected run must not have advanced the clock");
    }

    @Test
    void anAnonymousConfigIsRecordedUnderItsFullClassName() throws IOException {
        Path rlogPath = Paths.get("build", "sim", "scenario-runner-test-metadata.rlog");
        runTicks(rlogPath, 3);

        String recorded = RlogDecodedCompare.decode(rlogPath).get(0).get("RealMetadata/SimRobotConfig");
        assertEquals(STUB_CONFIG.getClass().getName(), recorded,
                "an anonymous class has an empty simple name, so the full name is recorded instead");
    }

    @Test
    void aRunStartedFromInsideAnotherRunIsRefused() {
        Path outer = Paths.get("build", "sim", "scenario-runner-test-reentry-outer.rlog");
        Path inner = Paths.get("build", "sim", "scenario-runner-test-reentry-inner.rlog");
        FakeTimer timer = new FakeTimer();

        IllegalStateException thrown = assertThrows(IllegalStateException.class,
                () -> ScenarioRunner.run("Outer", STUB_CONFIG, timer, 3, DT, outer,
                        deltaTimeSeconds -> runTicks(inner, 3)));
        assertTrue(thrown.getMessage().contains("started from inside another"), thrown.getMessage());

        // The refusal must not leave the guard stuck: a normal run straight afterwards still works.
        runTicks(outer, 3);
    }

    private static void runTicks(Path rlogPath, int ticks) {
        FakeTimer timer = new FakeTimer();
        ScenarioRunner.run("TickScenario", STUB_CONFIG, timer, ticks, DT, rlogPath,
                deltaTimeSeconds -> Logger.recordOutput("Test/time", timer.time()));
    }

    private static void runCountingScenario(Path rlogPath) {
        FakeTimer timer = new FakeTimer();
        int[] ticksSeen = {0};
        ScenarioRunner.run("CountingScenario", STUB_CONFIG, timer, 25, DT, rlogPath, deltaTimeSeconds -> {
            ticksSeen[0]++;
            Logger.recordOutput("Test/tickCount", ticksSeen[0]);
            Logger.recordOutput("Test/time", timer.time());
        });
    }
}
