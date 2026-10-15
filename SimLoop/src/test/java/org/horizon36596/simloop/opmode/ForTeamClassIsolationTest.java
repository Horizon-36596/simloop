package org.horizon36596.simloop.opmode;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.firstinspires.ftc.teamcode.simclockfixture.TickProbeAuto;
import org.horizon36596.simloop.fakehardware.FakeHardwareMap;
import org.junit.jupiter.api.Test;

import java.lang.ref.WeakReference;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.Callable;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;

/**
 * Pins that {@link OpModeRunner#forTeamClass} runs are independent of each other: runners going at once on
 * different threads each keep their own sim clock, and a finished run leaves nothing behind that keeps its
 * {@code SimClockLoader} (and so every team class it loaded) reachable. Plain {@code run()} only: logged runs
 * take turns (see {@code ScenarioRunner}).
 *
 * <p>Invariant tier: a test suite runs hundreds of these in one JVM.
 */
class ForTeamClassIsolationTest {

    /** What the run's last telemetry showed: the passes and the sim clock read three ways. */
    private static Callable<List<String>> tickProbe(double tickSeconds) {
        return () -> OpModeRunner.forTeamClass(TickProbeAuto.class, new FakeHardwareMap()).tickSeconds(tickSeconds)
                .activeTicks(400).run().telemetry().lastTransmission();
    }

    @Test
    void runnersGoingAtOnceKeepTheirOwnClocks() throws Exception {
        List<String> alone20 = tickProbe(0.02).call();
        List<String> alone5 = tickProbe(0.005).call();
        ExecutorService pool = Executors.newFixedThreadPool(4);
        try {
            for (int round = 0; round < 5; round++) {
                List<Future<List<String>>> runs = new ArrayList<>();
                for (int i = 0; i < 4; i++) {
                    runs.add(pool.submit(tickProbe(i % 2 == 0 ? 0.02 : 0.005)));
                }
                for (int i = 0; i < 4; i++) {
                    assertEquals(i % 2 == 0 ? alone20 : alone5, runs.get(i).get(), "round " + round + ", run " + i);
                }
            }
        } finally {
            pool.shutdownNow();
        }
    }

    @Test
    void finishedRunsLeaveNoLoaderReachable() throws InterruptedException {
        List<WeakReference<ClassLoader>> loaders = new ArrayList<>();
        for (int i = 0; i < 50; i++) {
            OpModeRunner runner = OpModeRunner.forTeamClass(TickProbeAuto.class, new FakeHardwareMap()).activeTicks(5);
            runner.run();
            loaders.add(new WeakReference<>(runner.opMode().getClass().getClassLoader()));
        }
        int reachable = loaders.size();
        for (int attempt = 0; attempt < 10 && reachable > 2; attempt++) {
            System.gc();
            Thread.sleep(20);
            reachable = 0;
            for (WeakReference<ClassLoader> loader : loaders) {
                reachable += loader.get() != null ? 1 : 0;
            }
        }
        // A collector may keep the last one or two for a while; a leak keeps them all.
        assertTrue(reachable <= 2, reachable + " of 50 loaders still reachable after their runs");
    }
}
