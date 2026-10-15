package org.horizon36596.simloop.opmode;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTimeoutPreemptively;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.horizon36596.simloop.fakehardware.FakeHardwareMap;
import org.horizon36596.simloop.sim.FakeTimer;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;

import java.time.Duration;
import java.util.ArrayList;
import java.util.List;

/**
 * Pins how {@link OpModeRunner} steps a {@link LinearOpMode}: where a tick ends inside {@code runOpMode()},
 * that {@code sleep()} and {@code waitForStart()} move in sim time rather than real time, that STOP is
 * seen, and that a broken OpMode fails the run instead of hanging it or leaking its thread.
 *
 * <p>Invariant tier: these are the lockstep contract. Every OpMode here is written the way a team writes
 * one, and each records what it saw so the test can compare it with what the sim did.
 */
class LinearOpModeRunnerTest {

    private static final double TICK = 0.02;

    /**
     * The stuck timeout the stuck-OpMode tests use. The runner counts it in one-millisecond waits of the sim
     * thread, not by a clock, so a healthy tick exhausts it only if the OpMode thread misses about 200 turns in
     * a row while the sim thread keeps getting its own. A larger value buys little, and costs real time: on a
     * machine at 100% CPU each of those waits can take tens of milliseconds.
     */
    static final int STUCK_MILLIS = 200;

    /**
     * The real-time limit on a run that must fail rather than hang. It only has to catch a real hang, so it is
     * generous: a stuck-OpMode test spends its stuck timeout in one-millisecond waits, and on a machine
     * already at 100% CPU each of those can take tens of milliseconds.
     */
    static final Duration HANG_GUARD = Duration.ofSeconds(120);

    private final OpModeThreadLeakCheck leaks = new OpModeThreadLeakCheck(OpModeThreadLeakCheck.LINEAR);

    /** Every test must leave no OpMode thread behind, whether it passed or failed on purpose. */
    @AfterEach
    void noOpModeThreadOutlivesItsRun() throws InterruptedException {
        leaks.verify();
    }

    private static OpModeRunner runner(LinearOpMode opMode, FakeTimer timer) {
        return new OpModeRunner(opMode, new FakeHardwareMap()).timer(timer).tickSeconds(TICK);
    }

    // ---- the lifecycle ------------------------------------------------------------------------------

    /** The classic shape: set up, wait for PLAY, loop while active, clean up. */
    static final class ClassicLinear extends LinearOpMode {
        final FakeTimer timer;
        final List<String> events = new ArrayList<>();
        int loops;
        boolean stopRequestedAtTheEnd;
        boolean activeAtTheEnd = true;

        ClassicLinear(FakeTimer timer) {
            this.timer = timer;
        }

        @Override
        public void runOpMode() {
            events.add("init@" + timer.time());
            waitForStart();
            events.add("started@" + timer.time());
            while (opModeIsActive()) {
                loops++;
            }
            stopRequestedAtTheEnd = isStopRequested();
            activeAtTheEnd = opModeIsActive();
            events.add("ended@" + timer.time());
        }
    }

    @Test
    void initRunsOnTheFirstTickPlayOnTheFirstActiveTickAndTheLoopOncePerActiveTick() {
        FakeTimer timer = new FakeTimer();
        ClassicLinear opMode = new ClassicLinear(timer);

        OpModeRunResult result = runner(opMode, timer).initLoopTicks(5).activeTicks(10).run();

        assertEquals(1 + 5 + 10 + 1, result.ticks());
        assertEquals(10, opMode.loops, "one pass of while (opModeIsActive()) per ACTIVE tick");
        assertEquals(3, opMode.events.size(), opMode.events.toString());
        assertEquals("init@" + tickTime(0), opMode.events.get(0));
        assertEquals("started@" + tickTime(6), opMode.events.get(1), "PLAY is the first ACTIVE tick");
        assertEquals("ended@" + tickTime(16), opMode.events.get(2), "the loop exits on the STOP tick");
        assertTrue(opMode.stopRequestedAtTheEnd, "isStopRequested() is true after STOP");
        assertFalse(opMode.activeAtTheEnd);
        assertFalse(result.endedBeforeStop());
        assertEquals(tickTime(16), result.finalTimeSeconds(), 0.0);
    }

    /** The sim clock after {@code n + 1} ticks, computed the same way FakeTimer adds them up. */
    private static double tickTime(int tick) {
        FakeTimer t = new FakeTimer();
        for (int i = 0; i <= tick; i++) {
            t.advance(TICK);
        }
        return t.time();
    }

    /** Two lifecycle checks in one loop condition are still one pass per tick. */
    static final class DoubleCheckLoop extends LinearOpMode {
        int loops;

        @Override
        public void runOpMode() {
            waitForStart();
            while (opModeIsActive() && !isStopRequested()) {
                loops++;
            }
        }
    }

    @Test
    void aLoopConditionWithTwoChecksIsStillOnePassPerTick() {
        DoubleCheckLoop opMode = new DoubleCheckLoop();
        runner(opMode, new FakeTimer()).activeTicks(25).run();
        assertEquals(25, opMode.loops);
    }

    /** An init loop written with {@code opModeInInit()}, then an active loop. */
    static final class InitLoopLinear extends LinearOpMode {
        int initPasses;
        int activePasses;

        @Override
        public void runOpMode() {
            while (opModeInInit()) {
                initPasses++;
            }
            while (opModeIsActive()) {
                activePasses++;
            }
        }
    }

    @Test
    void anOpModeInInitLoopRunsOncePerInitTickAndHandsOverToTheActiveLoopAtPlay() {
        InitLoopLinear opMode = new InitLoopLinear();
        runner(opMode, new FakeTimer()).initLoopTicks(7).activeTicks(4).run();
        assertEquals(1 + 7, opMode.initPasses, "INIT tick plus one pass per INIT_LOOP tick");
        assertEquals(4, opMode.activePasses);
    }

    // ---- sleep() ------------------------------------------------------------------------------------

    /** The rookie autonomous: drive, sleep, stop. */
    static final class SleepyAuto extends LinearOpMode {
        static final long HALF_SECOND = 500;
        final FakeTimer timer;
        final List<Double> sleeps = new ArrayList<>();

        SleepyAuto(FakeTimer timer) {
            this.timer = timer;
        }

        @Override
        public void runOpMode() {
            waitForStart();
            double before = timer.time();
            sleep(1000);
            sleeps.add(timer.time() - before);
            before = timer.time();
            sleep(HALF_SECOND);
            sleeps.add(timer.time() - before);
            before = timer.time();
            sleep(30); // not a whole number of ticks: rounds up, like Thread.sleep's "at least"
            sleeps.add(timer.time() - before);
            before = timer.time();
            sleep(0);
            sleeps.add(timer.time() - before);
            before = timer.time();
            sleep(1);
            sleeps.add(timer.time() - before);
        }
    }

    @Test
    void sleepAdvancesSimTimeByItsDurationRoundedUpToWholeTicks() {
        FakeTimer timer = new FakeTimer();
        SleepyAuto opMode = new SleepyAuto(timer);

        OpModeRunResult result = runner(opMode, timer).activeTicks(200).run();

        assertEquals(5, opMode.sleeps.size(), opMode.sleeps.toString());
        assertEquals(1.000, opMode.sleeps.get(0), 1e-9, "sleep(1000) = 50 ticks");
        assertEquals(0.500, opMode.sleeps.get(1), 1e-9, "sleep(CONSTANT) = 25 ticks");
        assertEquals(0.040, opMode.sleeps.get(2), 1e-9, "sleep(30) = 2 ticks");
        assertEquals(0.000, opMode.sleeps.get(3), 1e-9, "sleep(0) returns at once");
        assertEquals(0.020, opMode.sleeps.get(4), 1e-9, "sleep(1) = 1 tick");
        assertTrue(result.endedBeforeStop(), "runOpMode() returned before STOP");
    }

    @Test
    void sleepDoesNotWaitInRealTime() {
        // 60 simulated seconds of sleep(1000), which would take a minute if the SDK's Thread.sleep ran.
        assertTimeoutPreemptively(HANG_GUARD, () -> {
            FakeTimer timer = new FakeTimer();
            LinearOpMode opMode = new LinearOpMode() {
                @Override
                public void runOpMode() {
                    waitForStart();
                    while (opModeIsActive()) {
                        sleep(1000);
                    }
                }
            };
            runner(opMode, timer).activeTicks(3000).run();
            assertEquals(tickTime(3001), timer.time(), 0.0);
        });
    }

    /** A helper that sleeps for a duration it was passed: the one shape SimLoop cannot read. */
    static final class ComputedSleep extends LinearOpMode {
        @Override
        public void runOpMode() {
            waitForStart();
            driveFor(750);
        }

        private void driveFor(long millis) {
            sleep(millis);
        }
    }

    @Test
    void aComputedSleepDurationFailsTheRunAndSaysWhere() {
        OpModeException failure = assertThrows(OpModeException.class,
                () -> runner(new ComputedSleep(), new FakeTimer()).activeTicks(10).run());
        assertEquals(OpModePhase.ACTIVE, failure.phase());
        assertEquals(1, failure.tick());
        String message = failure.getCause().getMessage();
        assertTrue(message.contains("driveFor"), message);
        assertTrue(message.contains("LinearOpModeRunnerTest.java:"), message);
        assertTrue(message.contains("computed"), message);
    }

    /** An argument that looks constant but is not a compile-time constant expression. */
    static final class SwitchThenSleep extends LinearOpMode {
        final FakeTimer timer;
        final int choice;
        double slept = -1;

        SwitchThenSleep(FakeTimer timer, int choice) {
            this.timer = timer;
            this.choice = choice;
        }

        @Override
        public void runOpMode() {
            waitForStart();
            int power;
            switch (choice) {
                case 0: power = 1; break;
                case 1: power = 2; break;
                case 2: power = 3; break;
                default: power = 4; break;
            }
            switch (choice * 1000) {
                case 0: power += 1; break;
                case 5000: power += 2; break;
                default: power += 3; break;
            }
            double before = timer.time();
            sleep(200 + 0 * power);
            slept = timer.time() - before;
        }
    }

    @Test
    void anArgumentThatOnlyLooksConstantIsRefused() {
        FakeTimer timer = new FakeTimer();
        SwitchThenSleep opMode = new SwitchThenSleep(timer, 2);
        OpModeException failure = assertThrows(OpModeException.class,
                () -> runner(opMode, timer).activeTicks(20).run(),
                "200 + 0 * power is not a constant expression: javac cannot fold it");
        assertTrue(failure.getCause().getMessage().contains("computed"));
    }

    /** Two constants, one call: whichever the bytecode loads last is not the one every run passes. */
    static final class ConditionalSleep extends LinearOpMode {
        final boolean fast;

        ConditionalSleep(boolean fast) {
            this.fast = fast;
        }

        @Override
        public void runOpMode() {
            waitForStart();
            sleep(fast ? 100 : 1000);
        }
    }

    @Test
    void aSleepWhoseDurationDependsOnABranchIsRefusedNotGuessed() {
        for (boolean fast : new boolean[] {true, false}) {
            OpModeException failure = assertThrows(OpModeException.class,
                    () -> runner(new ConditionalSleep(fast), new FakeTimer()).activeTicks(60).run(),
                    "sleep(fast ? 100 : 1000) must not be read as either constant");
            assertTrue(failure.getCause().getMessage().contains("computed"), failure.getCause().getMessage());
        }
    }

    /**
     * A constant sleep after a tableswitch and a lookupswitch, whose padded, variable-length encodings the
     * bytecode walk must step over to land on the right instruction.
     */
    static final class SwitchThenConstantSleep extends LinearOpMode {
        final FakeTimer timer;
        final int choice;
        double slept = -1;

        SwitchThenConstantSleep(FakeTimer timer, int choice) {
            this.timer = timer;
            this.choice = choice;
        }

        @Override
        public void runOpMode() {
            waitForStart();
            int power;
            switch (choice) {
                case 0: power = 1; break;
                case 1: power = 2; break;
                case 2: power = 3; break;
                default: power = 4; break;
            }
            switch (choice * 1000) {
                case 0: power += 1; break;
                case 5000: power += 2; break;
                default: power += 3; break;
            }
            telemetry.addData("power", power);
            double before = timer.time();
            sleep(200);
            slept = timer.time() - before;
        }
    }

    @Test
    void aConstantSleepIsReadCorrectlyAfterSwitchStatements() {
        FakeTimer timer = new FakeTimer();
        SwitchThenConstantSleep opMode = new SwitchThenConstantSleep(timer, 2);
        runner(opMode, timer).activeTicks(20).run();
        assertEquals(0.2, opMode.slept, 1e-9);
    }

    @Test
    void stopDuringASleepEndsTheSleepAtOnce() {
        FakeTimer timer = new FakeTimer();
        List<Double> after = new ArrayList<>();
        LinearOpMode opMode = new LinearOpMode() {
            @Override
            public void runOpMode() {
                waitForStart();
                sleep(10_000);
                after.add(timer.time());
            }
        };
        runner(opMode, timer).activeTicks(10).run();
        assertEquals(1, after.size());
        assertEquals(tickTime(11), after.get(0), 0.0, "sleep returned on the STOP tick");
    }

    // ---- waitForStart and STOP ----------------------------------------------------------------------

    @Test
    void stopDuringInitReleasesWaitForStartAndTheOpModeSeesTheStop() {
        List<Boolean> seen = new ArrayList<>();
        LinearOpMode opMode = new LinearOpMode() {
            @Override
            public void runOpMode() {
                waitForStart();
                seen.add(isStarted());
                seen.add(isStopRequested());
                seen.add(opModeIsActive());
            }
        };
        runner(opMode, new FakeTimer()).initLoopTicks(3).activeTicks(0).run();
        // isStarted() is true after a stop, exactly as on the robot, where the stop interrupts the thread.
        assertEquals(java.util.Arrays.asList(true, true, false), seen);
    }

    /** Lets a sensor settle with a Thread.sleep of its own and swallows the interrupt, a common init step. */
    static final class SwallowsItsOwnInterrupt extends LinearOpMode {
        final FakeTimer timer;
        double startedAt = -1;
        double slept = -1;

        SwallowsItsOwnInterrupt(FakeTimer timer) {
            this.timer = timer;
        }

        @Override
        public void runOpMode() {
            try {
                Thread.sleep(100);
            } catch (InterruptedException ignored) {
                // what a rookie writes; on the robot this just waits 100 ms
            }
            waitForStart();
            startedAt = timer.time();
            sleep(200);
            slept = timer.time() - startedAt;
        }
    }

    @Test
    void anOpModeThatSwallowsAnInterruptOfItsOwnStillWaitsForStartInSimTime() {
        FakeTimer timer = new FakeTimer();
        SwallowsItsOwnInterrupt opMode = new SwallowsItsOwnInterrupt(timer);
        assertTimeoutPreemptively(HANG_GUARD,
                () -> runner(opMode, timer).stuckTimeoutMillis(2000).initLoopTicks(3).activeTicks(20).run());
        assertEquals(tickTime(4), opMode.startedAt, 0.0, "waitForStart() returned at PLAY");
        assertEquals(0.2, opMode.slept, 1e-9);
    }

    @Test
    void anOpModeThatReturnsEarlyIsRecordedAndTheWorldKeepsTicking() {
        FakeTimer timer = new FakeTimer();
        LinearOpMode opMode = new LinearOpMode() {
            @Override
            public void runOpMode() {
                waitForStart();
                sleep(100); // 5 ticks
            }
        };
        OpModeRunResult result = runner(opMode, timer).initLoopTicks(2).activeTicks(50).run();
        assertTrue(result.endedBeforeStop());
        assertEquals(1 + 2 + 5, result.endedAtTick(), "PLAY at tick 3, then 5 ticks of sleep");
        assertEquals(tickTime(1 + 2 + 50), timer.time(), 0.0, "every tick still ran");
    }

    @Test
    void requestOpModeStopEndsTheOpModeLikeTheRobotDoes() {
        List<Boolean> seen = new ArrayList<>();
        LinearOpMode opMode = new LinearOpMode() {
            @Override
            public void runOpMode() {
                waitForStart();
                int loops = 0;
                while (opModeIsActive()) {
                    if (++loops == 3) {
                        requestOpModeStop();
                    }
                }
                seen.add(isStopRequested());
            }
        };
        OpModeRunResult result = runner(opMode, new FakeTimer()).activeTicks(20).run();
        assertEquals(java.util.Collections.singletonList(true), seen);
        assertTrue(result.endedBeforeStop());
        assertEquals(1 + 2, result.endedAtTick(), "the third pass is on the third ACTIVE tick");
    }

    @Test
    void terminateOpModeNowEndsTheRunCleanly() {
        LinearOpMode opMode = new LinearOpMode() {
            @Override
            public void runOpMode() {
                waitForStart();
                terminateOpModeNow();
            }
        };
        OpModeRunResult result = runner(opMode, new FakeTimer()).activeTicks(5).run();
        assertTrue(result.endedBeforeStop());
    }

    // ---- failures -----------------------------------------------------------------------------------

    @Test
    void anExceptionInRunOpModeFailsTheRunWithTheOriginalAsTheCause() {
        IllegalStateException bug = new IllegalStateException("motor not found");
        LinearOpMode opMode = new LinearOpMode() {
            @Override
            public void runOpMode() {
                waitForStart();
                while (opModeIsActive()) {
                    if (time > 0.1) {
                        throw bug;
                    }
                }
            }
        };
        OpModeException failure = assertTimeoutPreemptively(HANG_GUARD,
                () -> assertThrows(OpModeException.class,
                        () -> runner(opMode, new FakeTimer()).activeTicks(50).run()));
        assertSame(bug, failure.getCause());
        assertEquals(OpModePhase.ACTIVE, failure.phase());
    }

    @Test
    void anExceptionDuringInitFailsTheRunInInit() {
        LinearOpMode opMode = new LinearOpMode() {
            @Override
            public void runOpMode() {
                hardwareMap.get(com.qualcomm.robotcore.hardware.DcMotor.class, "missing");
            }
        };
        OpModeException failure = assertThrows(OpModeException.class,
                () -> runner(opMode, new FakeTimer()).activeTicks(5).run());
        assertEquals(OpModePhase.INIT, failure.phase());
        assertEquals(0, failure.tick());
    }

    /** Spins without ever asking the SDK anything, until the test lets it go. */
    static final class NeverYields extends LinearOpMode {
        volatile boolean letGo;

        @Override
        public void runOpMode() {
            waitForStart();
            while (!letGo) {
                Thread.onSpinWait();
            }
        }
    }

    @Test
    void anOpModeThatNeverReachesALifecycleCheckFailsInsteadOfHanging() {
        NeverYields opMode = new NeverYields();
        // The runner cannot stop it; @AfterEach lets it go, even if an assertion here fails first.
        leaks.releaseAfter(() -> opMode.letGo = true);
        OpModeException failure = assertTimeoutPreemptively(HANG_GUARD,
                () -> assertThrows(OpModeException.class,
                        () -> runner(opMode, new FakeTimer()).stuckTimeoutMillis(STUCK_MILLIS).activeTicks(5).run()));
        assertTrue(failure.getMessage().contains("without reaching a lifecycle check"), failure.getMessage());
        assertEquals(OpModePhase.ACTIVE, failure.phase());
        boolean stackShowsTheSpin = false;
        for (StackTraceElement frame : failure.getCause().getStackTrace()) {
            stackShowsTheSpin |= frame.getClassName().endsWith("NeverYields");
        }
        assertTrue(stackShowsTheSpin, "the cause carries the OpMode thread's stack");
    }

    /** Keeps looping after STOP, ignoring it. */
    static final class IgnoresStop extends LinearOpMode {
        volatile boolean letGo;

        @Override
        public void runOpMode() {
            waitForStart();
            while (!letGo) {
                idle();
                isStopRequested();
            }
        }
    }

    @Test
    void anOpModeThatIgnoresStopFailsTheRun() {
        IgnoresStop opMode = new IgnoresStop();
        leaks.releaseAfter(() -> opMode.letGo = true);
        OpModeException failure = assertTimeoutPreemptively(HANG_GUARD,
                () -> assertThrows(OpModeException.class,
                        () -> runner(opMode, new FakeTimer()).stuckTimeoutMillis(STUCK_MILLIS).activeTicks(5).run()));
        assertEquals(OpModePhase.STOP, failure.phase());
        assertTrue(failure.getMessage().contains("after STOP"), failure.getMessage());
    }

    /** Clears its own interrupt flag, then blocks in a plain wait() that nothing will ever notify. */
    static final class BlocksInAWaitOfItsOwn extends LinearOpMode {
        @Override
        public void runOpMode() throws InterruptedException {
            waitForStart();
            Thread.interrupted();
            Object latch = new Object();
            synchronized (latch) {
                latch.wait();
            }
        }
    }

    @Test
    void aStuckOpModeBlockedInAWaitOfItsOwnIsWokenSoItsThreadDoesNotOutliveTheRun() throws InterruptedException {
        OpModeException failure = assertTimeoutPreemptively(HANG_GUARD,
                () -> assertThrows(OpModeException.class,
                        () -> runner(new BlocksInAWaitOfItsOwn(), new FakeTimer()).stuckTimeoutMillis(STUCK_MILLIS)
                                .activeTicks(5).run()));
        assertEquals(OpModePhase.ACTIVE, failure.phase());
        assertEquals(0, failure.getSuppressed().length, "the interrupt at shutdown let it return");
        // Nothing but the runner's interrupt can end that wait. The runner gives a thread it has already given
        // up on only a short grace to end, which a loaded machine can outlast, so wait for it here.
        leaks.awaitStartedThreadsEnd();
        leaks.assertStartedThreadsEnded("the interrupt at shutdown ended the OpMode's own wait");
    }

    @Test
    void aFailureOutsideTheOpModeStillStopsItsThread() {
        LinearOpMode opMode = new LinearOpMode() {
            @Override
            public void runOpMode() {
                waitForStart();
                while (opModeIsActive()) {
                    idle();
                }
            }
        };
        RuntimeException worldBroke = new RuntimeException("plant exploded");
        int[] ticks = {0};
        RuntimeException failure = assertThrows(RuntimeException.class,
                () -> runner(opMode, new FakeTimer()).activeTicks(10).eachTick(dt -> {
                    if (++ticks[0] == 4) {
                        throw worldBroke;
                    }
                }).run());
        assertSame(worldBroke, failure);
        // @AfterEach checks the OpMode thread is gone.
    }

    /**
     * Interrupts the thread running the sim from inside its third loop pass, as a test framework's timeout
     * would, then stays busy for 300 ms of real time before its next lifecycle check.
     */
    static final class InterruptsTheSim extends LinearOpMode {
        static final long BUSY_NANOS = 300_000_000L;
        final Thread simThread;

        InterruptsTheSim(Thread simThread) {
            this.simThread = simThread;
        }

        @Override
        public void runOpMode() {
            waitForStart();
            int passes = 0;
            while (opModeIsActive()) {
                if (++passes == 3) {
                    simThread.interrupt();
                    long until = System.nanoTime() + BUSY_NANOS; // real time on purpose: still its turn
                    while (System.nanoTime() < until) {
                        Thread.onSpinWait();
                    }
                }
            }
        }
    }

    /**
     * The sim thread is interrupted while the OpMode has its turn. The run fails on the interrupt, and still
     * stops and joins the OpMode thread before {@code run()} returns, without a second failure from the stop,
     * and leaves the caller's interrupt set.
     */
    @Test
    void anInterruptedSimThreadStillStopsAndJoinsItsOpModeThreadAndKeepsItsInterrupt() {
        OpModeRunner runner = runner(new InterruptsTheSim(Thread.currentThread()), new FakeTimer()).activeTicks(10);
        boolean stillInterrupted;
        OpModeException failure;
        try {
            failure = assertThrows(OpModeException.class, runner::run);
        } finally {
            stillInterrupted = Thread.interrupted(); // read and clear, so no later test inherits it
        }
        assertTrue(failure.getMessage().contains("was interrupted while InterruptsTheSim had its turn"),
                failure.getMessage());
        assertEquals(OpModePhase.ACTIVE, failure.phase());
        assertEquals(0, failure.getSuppressed().length,
                "shutDown waited for the OpMode thread instead of failing on the caller's interrupt");
        assertTrue(stillInterrupted, "the caller's interrupt was kept");
        leaks.assertStartedThreadsEnded("the OpMode thread was joined before run() returned");
    }

    @Test
    void aRunnerRunsOnce() {
        OpModeRunner runner = runner(new DoubleCheckLoop(), new FakeTimer()).activeTicks(1);
        runner.run();
        assertThrows(IllegalStateException.class, runner::run);
    }
}
