package org.horizon36596.simloop.opmode;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTimeoutPreemptively;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.horizon36596.simloop.fakehardware.FakeHardwareMap;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.List;

/**
 * Pins the iterative OpMode's watchdog: a call to {@code init()}, {@code init_loop()}, {@code start()},
 * {@code loop()} or {@code stop()} that never returns fails the run with the method, phase and tick instead
 * of hanging the build, and the stuck thread is left harmless. Also pins that moving the calls onto a thread
 * of their own kept them in lockstep with the sim.
 *
 * <p>Invariant tier: "a broken OpMode fails the test, never hangs it" is the runner's contract for both kinds
 * of OpMode.
 */
class IterativeOpModeWatchdogTest {

    private static final int STUCK_MILLIS = 150;

    private final OpModeThreadLeakCheck leaks = new OpModeThreadLeakCheck(OpModeThreadLeakCheck.ITERATIVE);

    /** Every test leaves no iterative OpMode thread behind once any stuck one has been let go. */
    @AfterEach
    void noOpModeThreadOutlivesItsRun() throws InterruptedException {
        leaks.verify();
    }

    /** Waits for the threads this test started to end, then fails if any has not. */
    private void awaitTheAbandonedThreadsEnd(String message) throws InterruptedException {
        leaks.awaitStartedThreadsEnd();
        leaks.assertStartedThreadsEnded(message);
    }

    /** Spins in whichever lifecycle method it is told to, until the test lets it go. */
    static final class StuckIn extends OpMode {
        final String method;
        volatile boolean letGo;
        final List<String> callsAfterLetGo = new ArrayList<>();

        StuckIn(String method) {
            this.method = method;
        }

        @Override public void init() { maybeSpin("init"); }
        @Override public void init_loop() { maybeSpin("init_loop"); }
        @Override public void start() { maybeSpin("start"); }
        @Override public void loop() { maybeSpin("loop"); }
        @Override public void stop() { maybeSpin("stop"); }

        private void maybeSpin(String call) {
            if (letGo) {
                callsAfterLetGo.add(call);
                return;
            }
            if (call.equals(method)) {
                while (!letGo) {
                    Thread.onSpinWait();
                }
            }
        }
    }

    private static OpModeException stuckRun(OpMode opMode) {
        return assertTimeoutPreemptively(LinearOpModeRunnerTest.HANG_GUARD, () -> assertThrows(OpModeException.class,
                () -> new OpModeRunner(opMode, new FakeHardwareMap())
                        .initLoopTicks(2).activeTicks(3).stuckTimeoutMillis(STUCK_MILLIS).run()));
    }

    private void assertStuckAt(String method, OpModePhase phase, int tick) {
        StuckIn opMode = new StuckIn(method);
        leaks.releaseAfter(() -> opMode.letGo = true); // even if an assertion below fails first
        OpModeException failure = stuckRun(opMode);
        assertEquals(phase, failure.phase(), method);
        assertEquals(tick, failure.tick(), method);
        String message = failure.getMessage();
        assertTrue(message.startsWith("StuckIn." + method + "() ran for at least " + STUCK_MILLIS
                + " ms of real time (stuckTimeoutMillis, counted in one-millisecond waits) without returning during "
                + phase), message);
        assertTrue(message.contains("(tick " + tick + ")"), message);
        boolean stackShowsTheSpin = false;
        for (StackTraceElement frame : failure.getCause().getStackTrace()) {
            stackShowsTheSpin |= frame.getClassName().endsWith("StuckIn") && frame.getMethodName().equals(method);
        }
        assertTrue(stackShowsTheSpin, "the cause carries the OpMode thread's stack, in " + method + "()");
        opMode.letGo = true;
    }

    @Test
    void aStuckCallInEachLifecycleMethodFailsWithTheMethodPhaseAndTick() {
        // INIT is tick 0, INIT_LOOP ticks 1-2, ACTIVE ticks 3-5 (start and the first loop on 3), STOP tick 6.
        assertStuckAt("init", OpModePhase.INIT, 0);
        assertStuckAt("init_loop", OpModePhase.INIT_LOOP, 1);
        assertStuckAt("start", OpModePhase.ACTIVE, 3);
        assertStuckAt("loop", OpModePhase.ACTIVE, 3);
        assertStuckAt("stop", OpModePhase.STOP, 6);
    }

    @Test
    void anAbandonedThreadRunsNoMoreOpModeCodeWhenItFinallyReturns() throws InterruptedException {
        StuckIn opMode = new StuckIn("loop");
        leaks.releaseAfter(() -> opMode.letGo = true);
        stuckRun(opMode);
        opMode.letGo = true;
        awaitTheAbandonedThreadsEnd("the abandoned thread ended once let go");
        assertEquals(new ArrayList<String>(), opMode.callsAfterLetGo,
                "no stop() or later loop() ran on the abandoned thread");
    }

    /** Stuck the way a blocked read looks: asleep, forever. */
    static final class SleepsForever extends OpMode {
        volatile boolean woken;

        @Override public void init() { }

        @Override
        public void loop() {
            try {
                Thread.sleep(Long.MAX_VALUE);
            } catch (InterruptedException e) {
                woken = true;
            }
        }
    }

    @Test
    void aThreadStuckInABlockingWaitIsInterruptedAndEndsOnItsOwn() throws InterruptedException {
        SleepsForever opMode = new SleepsForever();
        OpModeException failure = stuckRun(opMode);
        assertEquals(OpModePhase.ACTIVE, failure.phase());
        awaitTheAbandonedThreadsEnd("nothing let it go but the runner's interrupt");
        assertTrue(opMode.woken, "the interrupt ended the sleep");
    }

    /** Takes a while, but less than the limit, on every call. */
    static final class Slow extends OpMode {
        int loops;

        @Override public void init() { }

        @Override
        public void loop() {
            long spins = 0;
            for (int i = 0; i < 2_000_000; i++) {
                spins += i % 7;
            }
            loops += spins >= 0 ? 1 : 0;
        }
    }

    @Test
    void aSlowButReturningCallIsNotStuck() {
        Slow opMode = new Slow();
        OpModeRunResult result = new OpModeRunner(opMode, new FakeHardwareMap())
                .activeTicks(5).stuckTimeoutMillis(5_000).run();
        assertEquals(5, opMode.loops);
        assertFalse(result.endedBeforeStop());
    }

    /**
     * Interrupts the thread running the sim from inside {@code start()}, as a test framework's timeout would,
     * then stays busy in {@code start()} for 300 ms of real time, well inside the default stuck timeout.
     */
    static final class InterruptsTheSimInStart extends OpMode {
        static final long BUSY_NANOS = 300_000_000L;
        final Thread simThread;

        InterruptsTheSimInStart(Thread simThread) {
            this.simThread = simThread;
        }

        @Override public void init() { }

        @Override
        public void start() {
            simThread.interrupt();
            long until = System.nanoTime() + BUSY_NANOS; // real time on purpose: the call must still be running
            while (System.nanoTime() < until) {
                Thread.onSpinWait();
            }
        }

        @Override public void loop() { }
    }

    /**
     * The sim thread is interrupted while the OpMode is busy in {@code start()}. The run fails on the
     * interrupt, and still stops and joins the OpMode thread before {@code run()} returns, and leaves the
     * caller's interrupt set.
     */
    @Test
    void anInterruptedSimThreadStillJoinsItsOpModeThreadAndKeepsItsInterrupt() {
        OpModeRunner runner = new OpModeRunner(new InterruptsTheSimInStart(Thread.currentThread()),
                new FakeHardwareMap()).activeTicks(3);
        boolean stillInterrupted;
        OpModeException failure;
        try {
            failure = assertThrows(OpModeException.class, runner::run);
        } finally {
            stillInterrupted = Thread.interrupted(); // read and clear, so no later test inherits it
        }
        assertTrue(failure.getMessage().contains("was interrupted while InterruptsTheSimInStart.start() was running"),
                failure.getMessage());
        assertTrue(stillInterrupted, "the caller's interrupt was kept");
        leaks.assertStartedThreadsEnded("the OpMode thread was joined before run() returned, mid-start()");
    }

    /** Records which thread each call ran on, and whether the sim was mid-tick at the time. */
    static final class ThreadRecorder extends OpMode {
        final List<String> threads = new ArrayList<>();
        volatile boolean simBusy;
        volatile boolean overlapped;

        @Override public void init() { record(); }
        @Override public void loop() { record(); }
        @Override public void stop() { record(); }

        private void record() {
            threads.add(Thread.currentThread().getName());
            overlapped |= simBusy;
        }
    }

    @Test
    void callsRunOnTheOpModesOwnThreadAndNeverWhileTheSimIsWorking() {
        ThreadRecorder opMode = new ThreadRecorder();
        String simThread = Thread.currentThread().getName();
        new OpModeRunner(opMode, new FakeHardwareMap()).activeTicks(20).eachTick(dt -> {
            opMode.simBusy = true;
            Thread.yield();
            opMode.simBusy = false;
        }).run();
        assertEquals(1 + 20 + 1, opMode.threads.size());
        for (String name : opMode.threads) {
            assertEquals("SimLoop-OpMode-ThreadRecorder", name);
            assertFalse(name.equals(simThread));
        }
        assertFalse(opMode.overlapped, "OpMode code and the sim's own step never ran at the same time");
    }
}
