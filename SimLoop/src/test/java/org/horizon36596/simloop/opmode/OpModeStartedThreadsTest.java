package org.horizon36596.simloop.opmode;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.horizon36596.simloop.fakehardware.FakeHardwareMap;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;

/**
 * Pins that threads an OpMode starts itself - which SimLoop cannot step - are noticed and reported, or fail
 * the run when asked, and that the runner's own threads (the OpMode thread, the RLOG writer) and threads
 * other code starts are never mistaken for them.
 *
 * <p>Implementation tier: which threads can be seen is bounded by how Java attributes a new thread to a
 * {@link ThreadGroup}; the class javadoc of {@code OpModeThreads} says what that misses.
 */
class OpModeStartedThreadsTest {

    /** Threads a test started through its OpMode, released and joined after it. */
    private final List<Thread> started = Collections.synchronizedList(new ArrayList<Thread>());
    private final CountDownLatch release = new CountDownLatch(1);

    @AfterEach
    void releaseEveryThreadTheOpModeStarted() throws InterruptedException {
        release.countDown();
        for (Thread thread : new ArrayList<>(started)) {
            thread.join(5_000);
        }
    }

    private Thread worker(String name) {
        Thread thread = new Thread(() -> {
            try {
                release.await();
            } catch (InterruptedException ignored) {
                // test over
            }
        }, name);
        thread.setDaemon(true);
        started.add(thread);
        return thread;
    }

    private static OpModeRunner runner(OpMode opMode) {
        return new OpModeRunner(opMode, new FakeHardwareMap()).initLoopTicks(2).activeTicks(5);
    }

    /** An iterative OpMode that starts a worker in {@code start()} and stops it in {@code stop()}. */
    final class WorkerStoppedInStop extends OpMode {
        private Thread thread;

        @Override public void init() { }
        @Override public void start() {
            thread = worker("vision-worker");
            thread.start();
        }
        @Override public void loop() { }
        @Override public void stop() {
            release.countDown();
            try {
                thread.join(5_000);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }
    }

    @Test
    void aThreadTheOpModeStartsIsReportedWithItsNameAndWhereItWasFirstSeen() {
        OpModeRunResult result = runner(new WorkerStoppedInStop()).run();
        assertEquals(1, result.warnings().size(), result.warnings().toString());
        String warning = result.warnings().get(0);
        assertEquals("WorkerStoppedInStop started a thread of its own, \"vision-worker\" (first seen in ACTIVE,"
                + " tick 3), which SimLoop does not step: it ran in real time beside the sim, so what it did is"
                + " not deterministic.", warning, "stopped in stop(), so not still running");
    }

    /** A LinearOpMode that starts a worker and never stops it. */
    final class LeaksAWorker extends LinearOpMode {
        @Override
        public void runOpMode() {
            worker("leaked-worker").start();
            waitForStart();
        }
    }

    @Test
    void aLinearOpModesThreadIsReportedAndSaidToBeStillRunning() {
        OpModeRunResult result = runner(new LeaksAWorker()).run();
        assertEquals(1, result.warnings().size(), result.warnings().toString());
        String warning = result.warnings().get(0);
        assertTrue(warning.contains("\"leaked-worker\" (first seen in INIT, tick 0)"), warning);
        assertTrue(warning.endsWith("It was still running when the run ended."), warning);
    }

    /** An executor built in init, used from loop, shut down in stop. */
    static final class UsesAnExecutor extends OpMode {
        private ExecutorService pool;

        @Override public void init() { pool = Executors.newSingleThreadExecutor(); }
        @Override public void loop() { pool.submit(() -> { }); }
        @Override public void stop() {
            pool.shutdown();
            try {
                pool.awaitTermination(5, TimeUnit.SECONDS);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }
    }

    @Test
    void aPoolThreadCreatedFromOpModeCodeIsSeenToo() {
        OpModeRunResult result = runner(new UsesAnExecutor()).run();
        assertEquals(1, result.warnings().size(), result.warnings().toString());
        assertTrue(result.warnings().get(0).contains("(first seen in ACTIVE, tick 3)"), result.warnings().get(0));
    }

    @Test
    void failOnOpModeThreadsFailsTheRunOnTheTickTheThreadIsFirstSeen() {
        OpModeException failure = assertThrows(OpModeException.class,
                () -> runner(new WorkerStoppedInStop()).failOnOpModeThreads(true).run());
        assertEquals(OpModePhase.ACTIVE, failure.phase());
        assertEquals(3, failure.tick());
        assertNull(failure.getCause());
        assertTrue(failure.getMessage().startsWith("WorkerStoppedInStop started a thread of its own,"
                + " \"vision-worker\", during ACTIVE (tick 3)"), failure.getMessage());
    }

    /** Uses no threads at all. */
    static final class Plain extends OpMode {
        @Override public void init() { }
        @Override public void loop() { }
    }

    @Test
    void threadsTheTestOrTheWorldStartAreNotTheOpModes() {
        Thread[] fromTheWorld = new Thread[1];
        OpModeRunResult result = runner(new Plain()).eachTick(dt -> {
            if (fromTheWorld[0] == null) {
                fromTheWorld[0] = worker("plant-helper");
                fromTheWorld[0].start();
            }
        }).run();
        assertEquals(Collections.emptyList(), result.warnings());
    }

    @Test
    void aThreadStartedAndEndedWithinOneTicksCodeIsNotSeen() {
        OpMode joinsItsThread = new OpMode() {
            @Override public void init() { }
            @Override public void loop() {
                Thread quick = new Thread(() -> { });
                quick.start();
                try {
                    quick.join();
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                }
            }
        };
        assertFalse(runner(joinsItsThread).failOnOpModeThreads(true).run().endedBeforeStop(),
                "the documented blind spot: the thread was gone before the tick ended");
    }
}
