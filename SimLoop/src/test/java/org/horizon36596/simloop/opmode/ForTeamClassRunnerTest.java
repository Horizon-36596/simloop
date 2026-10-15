package org.horizon36596.simloop.opmode;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertInstanceOf;
import static org.junit.jupiter.api.Assertions.assertNotSame;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTimeoutPreemptively;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.simclockfixture.BackgroundHelperAuto;
import org.firstinspires.ftc.teamcode.simclockfixture.BlackboardAtBuildAuto;
import org.firstinspires.ftc.teamcode.simclockfixture.BusyWaitAuto;
import org.firstinspires.ftc.teamcode.simclockfixture.ClockReadsAuto;
import org.firstinspires.ftc.teamcode.simclockfixture.ComputedSleepAuto;
import org.firstinspires.ftc.teamcode.simclockfixture.ElapsedTimeAuto;
import org.firstinspires.ftc.teamcode.simclockfixture.HelperThreadAuto;
import org.firstinspires.ftc.teamcode.simclockfixture.IdlePastStopAuto;
import org.firstinspires.ftc.teamcode.simclockfixture.IdleWaitAuto;
import org.firstinspires.ftc.teamcode.simclockfixture.LazySleepReferenceAuto;
import org.firstinspires.ftc.teamcode.simclockfixture.LongSleepAuto;
import org.firstinspires.ftc.teamcode.simclockfixture.MethodReferenceAuto;
import org.firstinspires.ftc.teamcode.simclockfixture.NegativeSleepAuto;
import org.firstinspires.ftc.teamcode.simclockfixture.ObjectWaitAuto;
import org.firstinspires.ftc.teamcode.simclockfixture.PlainMath;
import org.firstinspires.ftc.teamcode.simclockfixture.SleepMethodReferenceAuto;
import org.firstinspires.ftc.teamcode.simclockfixture.SleepyTeleOp;
import org.firstinspires.ftc.teamcode.simclockfixture.StaticCounterAuto;
import org.firstinspires.ftc.teamcode.simclockfixture.ThreadSleepAuto;
import org.firstinspires.ftc.teamcode.simclockfixture.TimedDriveAuto;
import org.firstinspires.ftc.teamcode.simclockfixture.TimeUnitSleepAuto;
import org.firstinspires.ftc.teamcode.simclockfixture.TimerTeleOp;
import org.horizon36596.simloop.config.DrivetrainSimConfig;
import org.horizon36596.simloop.config.SimRobotConfig;
import org.horizon36596.simloop.fakehardware.FakeHardwareMap;
import org.horizon36596.simloop.fakehardware.FakeMotor;
import org.horizon36596.simloop.sim.FakeTimer;
import org.horizon36596.simloop.sim.RlogDecodedCompare;
import org.horizon36596.simloop.simclock.SimClockHooks;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.psilynx.psikit.core.Logger;

import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.List;

/**
 * Pins {@link OpModeRunner#forTeamClass}: a team's OpMode, unchanged, with {@code ElapsedTime},
 * {@code System.nanoTime()}, {@code Thread.sleep}, computed {@code sleep(ms)} and {@code idle()} loops, runs
 * on sim time, exactly and the same every run; and what it cannot run fails with a message a student can act
 * on. The OpModes are the fixtures in {@code org.firstinspires.ftc.teamcode.simclockfixture}, which report
 * through {@code OpMode.blackboard}.
 *
 * <p>Invariant tier: this is the promise that outside teams' timer code runs on sim time without edits.
 */
class ForTeamClassRunnerTest {

    private static final double TICK = OpModeRunner.DEFAULT_TICK_SECONDS;

    @BeforeEach
    void emptyBlackboard() {
        OpMode.blackboard.clear();
    }

    private final OpModeThreadLeakCheck leaks = new OpModeThreadLeakCheck(OpModeThreadLeakCheck.LINEAR);

    @AfterEach
    void noOpModeThreadOutlivesItsRun() throws InterruptedException {
        OpMode.blackboard.clear();
        leaks.verify();
    }

    private static OpModeRunner runner(Class<? extends OpMode> teamClass) {
        return OpModeRunner.forTeamClass(teamClass, new FakeHardwareMap());
    }

    private static double seconds(String key) {
        return (Double) OpMode.blackboard.get(key);
    }

    /** A wait of {@code ticks} ticks, as the sim clock adds it up: what an ElapsedTime reads after it. */
    private static double ticks(int ticks) {
        return ticks * TICK;
    }

    // ---- each rewrite, in a run -------------------------------------------------------------------

    @Test
    void aComputedSleepAHelpersSleepAndATernarySleepAllWaitInSimTime() {
        runner(ComputedSleepAuto.class).activeTicks(100).run();

        assertEquals(ticks(25), seconds("afterSum"), 1e-9, "sleep(ms * 2) with ms = 250: 25 ticks");
        assertEquals(ticks(25 + 15), seconds("afterHelper"), 1e-9, "pause(300) -> sleep(millis): 15 more");
        assertEquals(ticks(25 + 15 + 5), seconds("afterTernary"), 1e-9, "sleep(fast ? 100 : 1000): 5 more");
    }

    @Test
    void theSameComputedSleepRunAsAnInstanceStillFailsWithItsFileAndLine() {
        // The existing path is unchanged: without the loader, only a constant sleep() can be stepped.
        OpModeException failure = assertThrows(OpModeException.class,
                () -> new OpModeRunner(new ComputedSleepAuto(), new FakeHardwareMap()).activeTicks(100).run());
        assertTrue(failure.getCause().getMessage().contains("ComputedSleepAuto.java:"), failure.getCause().getMessage());
    }

    @Test
    void threadSleepWaitsInSimTimeAndIsInterruptedAtStopAsOnTheRobot() {
        runner(ThreadSleepAuto.class).activeTicks(100).run();

        assertEquals(ticks(15), seconds("afterMillis"), 1e-9, "Thread.sleep(300): 15 ticks");
        assertEquals(ticks(15 + 6), seconds("afterMillisAndNanos"), 1e-9,
                "Thread.sleep(100, 500000) is 100.5 ms: 6 ticks, never fewer than it asked for");
        int passes = (Integer) OpMode.blackboard.get("passes");
        assertEquals((100 - 1 - 15 - 6) / 2, passes, "Thread.sleep(40) is two ticks a pass");
        assertEquals(Boolean.TRUE, OpMode.blackboard.get("interruptedAtStop"),
                "a Thread.sleep in progress at STOP throws InterruptedException, as the SDK's interrupt makes it");
    }

    @Test
    void nanoTimeAndCurrentTimeMillisReadTheSimClock() {
        runner(ClockReadsAuto.class).activeTicks(60).run();

        assertEquals(Math.round(ticks(1) * 1e9), (long) (Long) OpMode.blackboard.get("startNanos"),
                "INIT runs on the first tick, one tick into the run");
        assertEquals(SimClockHooks.SIM_EPOCH_MILLIS + 20, (long) (Long) OpMode.blackboard.get("startMillis"),
                "the fixed epoch plus sim time, so it repeats run to run");
        assertEquals(1_020_000_000L, (long) (Long) OpMode.blackboard.get("nanosAfterOneSecond"), 1_000,
                "PLAY one tick after INIT, then sleep(1000)");
        assertEquals(1020L, (long) (Long) OpMode.blackboard.get("millisAfterOneSecond"));
    }

    @Test
    void everyKindOfElapsedTimeReadsTheSimClock() {
        runner(ElapsedTimeAuto.class).activeTicks(60).run();

        // Built on tick 0 (INIT), PLAY on tick 1, then 25 + 13 ticks of sleep: 750 ms is 37.5 ticks.
        assertEquals(ticks(1 + 25 + 13), seconds("seconds"), 1e-9, "new ElapsedTime()");
        assertEquals(ticks(1 + 25 + 13), seconds("fromStart"), 1e-9, "new ElapsedTime(System.nanoTime())");
        assertEquals(ticks(1 + 25 + 13) * 1000, seconds("millisTime"), 1e-6,
                "new ElapsedTime(Resolution.MILLISECONDS) reports milliseconds");
        assertEquals(ticks(1 + 1 + 25 + 13), seconds("field"), 1e-9,
                "a field initializer's timer started when the OpMode was built, a tick before INIT ran");
        assertEquals(ticks(1 + 25 + 13), seconds("anonymous"), 1e-9, "an anonymous subclass");
        assertEquals(2, OpMode.blackboard.get("laps"), "the team's own ElapsedTime subclass");
        assertEquals(ticks(13), seconds("lastLap"), 1e-9);
        String simTimer = "org.horizon36596.simloop.simclock.SimClockElapsedTime";
        assertEquals(simTimer, OpMode.blackboard.get("secondsClass"));
        assertEquals(simTimer, OpMode.blackboard.get("lapTimerSuper"));
        assertEquals(simTimer, OpMode.blackboard.get("anonymousSuper"));
    }

    @Test
    void anIdleLoopOnATimerLetsSimTimeMoveOneTickAPass() {
        runner(IdleWaitAuto.class).activeTicks(100).run();

        assertEquals(51, OpMode.blackboard.get("passes"),
                "one pass per tick for 1 s at 50 Hz, plus one: the first idle() of a tick, like the SDK's yield,"
                        + " does not end it; the second at the same place does");
        assertEquals(1.0, seconds("waited"), 1e-9);
    }

    @Test
    void methodReferencesAndLambdasToTheClockFollowTheSimClock() {
        runner(MethodReferenceAuto.class).activeTicks(60).run();

        assertEquals(400_000_000L, (long) (Long) OpMode.blackboard.get("nanoClock"), 1_000, "System::nanoTime");
        assertEquals(400_000_000L, (long) (Long) OpMode.blackboard.get("lambdaClock"), 1_000,
                "() -> System.nanoTime()");
        assertEquals(0.4, seconds("timer"), 1e-9, "a timer from ElapsedTime::new");
        assertEquals("org.horizon36596.simloop.simclock.SimClockElapsedTime", OpMode.blackboard.get("timerClass"));
        assertTrue((Long) OpMode.blackboard.get("milliClock") > SimClockHooks.SIM_EPOCH_MILLIS, "System::currentTimeMillis");
    }

    @Test
    void anIterativeOpModesElapsedTimeIsExact() {
        OpModeRunResult result = runner(TimerTeleOp.class).initLoopTicks(3).activeTicks(10).run();

        List<List<String>> sent = result.telemetry().transmissions();
        // One transmission each for init, 3 x init_loop and start; then the first loop(), on the same tick as
        // start() reset the timer, shows 0 ms, and each loop() after it one tick more.
        assertEquals("ms : 0.0", sent.get(5).get(0));
        assertEquals("ms : 180.0", sent.get(5 + 9).get(0), "nine ticks later");
    }

    // ---- determinism and speed --------------------------------------------------------------------

    private static final SimRobotConfig ONE_MOTOR = new SimRobotConfig() {
        @Override
        public String[] driveMotorNames() {
            return new String[] {"motor"};
        }

        @Override
        public String odometryName() {
            return "none";
        }

        @Override
        public DrivetrainSimConfig drivetrain() {
            return null;
        }
    };

    private static OpModeRunResult timedDrive(Path rlog, long[] finalPosition) {
        FakeHardwareMap hardwareMap = new FakeHardwareMap();
        FakeMotor motor = new FakeMotor(2787, 10);
        hardwareMap.register("motor", motor);
        OpModeRunResult result = OpModeRunner.forTeamClass(TimedDriveAuto.class.getName(), hardwareMap)
                .initLoopTicks(5)
                .activeTicks(100)
                .eachTick(deltaTime -> {
                    if (Logger.isRunning()) {
                        Logger.recordOutput("Motor/positionTicks", motor.getCurrentPosition());
                    }
                })
                .runLogged(ONE_MOTOR, rlog);
        finalPosition[0] = motor.getCurrentPosition();
        return result;
    }

    @Test
    void theSameTeamOpModeRunTwiceGivesTheSameRobotTelemetryAndLog() throws IOException {
        Path first = Paths.get("build", "sim", "for-team-class-1.rlog");
        Path second = Paths.get("build", "sim", "for-team-class-2.rlog");
        long[] positionA = new long[1];
        long[] positionB = new long[1];

        OpModeRunResult a = timedDrive(first, positionA);
        OpModeRunResult b = timedDrive(second, positionB);

        assertTrue(positionA[0] != 0, "the motor moved");
        assertEquals(positionA[0], positionB[0]);
        assertEquals(a.telemetry().transmissions(), b.telemetry().transmissions(),
                "nanoTime, currentTimeMillis and ElapsedTime readings are the same every run");
        assertEquals(0, RlogDecodedCompare.compare(first, second).size(), "the two logs decode identically");
    }

    @Test
    void aTenSecondComputedSleepTakesFarLessThanTenSecondsOfRealTime() {
        long wallStart = System.nanoTime(); // the test's own clock, not the sim's: this is the point
        runner(LongSleepAuto.class).activeTicks(600).run();
        double wallSeconds = (System.nanoTime() - wallStart) / 1e9;

        assertEquals(10.0, seconds("waited"), 1e-9, "ten seconds of sim time passed");
        // Typically a few milliseconds; the bound is generous so a slow CI machine cannot make it flaky.
        assertTrue(wallSeconds < 5.0, "took " + wallSeconds + " s of real time");
    }

    // ---- what it refuses, and how it says so ------------------------------------------------------

    @Test
    void aMethodReferenceToSleepIsRefusedBeforeTheRunStarts() {
        IllegalStateException refused = assertThrows(IllegalStateException.class,
                () -> runner(SleepMethodReferenceAuto.class).activeTicks(10).run());
        assertTrue(refused.getMessage().contains("SleepMethodReferenceAuto.runOpMode (SleepMethodReferenceAuto.java:"),
                refused.getMessage());
    }

    @Test
    void aThreadSleepOnAThreadTheOpModeStartedFailsTheRunWithTheThreadAndCallSite() throws InterruptedException {
        OpModeException run = assertThrows(OpModeException.class,
                () -> runner(HelperThreadAuto.class).activeTicks(5).run(),
                "the helper caught the refusal, and the run still fails loudly, as documented");
        Thread helper = (Thread) OpMode.blackboard.get("helper");
        helper.join(10_000);

        Throwable failure = (Throwable) OpMode.blackboard.get("helperFailure");
        assertInstanceOf(IllegalStateException.class, failure);
        String message = failure.getMessage();
        assertTrue(message.startsWith("SimLoop cannot run Thread.sleep(50) at "
                + HelperThreadAuto.class.getName() + ".lambda$runOpMode$"), message);
        assertTrue(message.contains("thread \"arm-helper\""), message);
        assertTrue(message.contains("runs in real time beside the sim"), message);
        assertTrue(run.getMessage().contains(message), "the run's failure carries the refusal: " + run.getMessage());
        assertEquals(failure, run.getCause());
    }

    @Test
    void aHelperThatDiesOfARefusedSleepFailsEveryRunTheSameWay() {
        // The helper catches only InterruptedException, so the refusal kills it, usually within one tick. The
        // run must fail every time, whether the runner's thread scan saw the helper alive or not.
        for (int run = 0; run < 8; run++) {
            OpModeException failure = assertThrows(OpModeException.class,
                    () -> runner(BackgroundHelperAuto.class).activeTicks(10).run(), "run " + run);
            assertTrue(failure.getMessage().contains("SimLoop cannot run Thread.sleep(10) at "
                    + BackgroundHelperAuto.class.getName()), "run " + run + ": " + failure.getMessage());
            assertTrue(failure.getMessage().contains("thread \"arm-updater\""), failure.getMessage());

            OpModeException strict = assertThrows(OpModeException.class,
                    () -> runner(BackgroundHelperAuto.class).failOnOpModeThreads(true).activeTicks(10).run(),
                    "run " + run + " with failOnOpModeThreads");
            assertTrue(strict.getMessage().contains("SimLoop cannot run Thread.sleep(10)")
                    || strict.getMessage().contains("started a thread of its own, \"arm-updater\""),
                    strict.getMessage());
        }
    }

    @Test
    void aSleepReferenceInAHelperFirstLoadedMidRunFailsTheRunEvenWhenTheTeamCatchesIt() {
        OpModeException failure = assertThrows(OpModeException.class,
                () -> runner(LazySleepReferenceAuto.class).activeTicks(30).run());
        assertEquals("caught IllegalStateException", OpMode.blackboard.get("helper"),
                "the team's catch-all took the refusal where it was thrown");
        assertInstanceOf(IllegalStateException.class, failure.getCause());
        assertTrue(failure.getMessage().contains("::sleep"), failure.getMessage());
        assertTrue(failure.getMessage().contains("LazySleepReferenceAuto.java:"), failure.getMessage());
    }

    @Test
    void aThreadSleepInAnIterativeOpModeIsRefusedWithTheReason() {
        OpModeException failure = assertThrows(OpModeException.class,
                () -> runner(SleepyTeleOp.class).activeTicks(5).run());
        assertEquals(OpModePhase.ACTIVE, failure.phase());
        String message = failure.getCause().getMessage();
        assertTrue(message.startsWith("SimLoop cannot run Thread.sleep(10) at " + SleepyTeleOp.class.getName()
                + ".loop(SleepyTeleOp.java:"), message);
        assertTrue(message.contains("SleepyTeleOp is an iterative OpMode"), message);
    }

    @Test
    void aNegativeSleepFailsAsItDoesOnTheRobot() {
        OpModeException failure = assertThrows(OpModeException.class,
                () -> runner(NegativeSleepAuto.class).activeTicks(5).run());
        assertInstanceOf(IllegalArgumentException.class, failure.getCause());
        assertEquals("timeout value is negative", failure.getCause().getMessage());
    }

    @Test
    void aBusyWaitOnATimerFailsAfterTheStuckTimeoutAndSaysWhy() throws InterruptedException {
        FakeTimer clock = new FakeTimer();
        OpModeException failure = assertTimeoutPreemptively(LinearOpModeRunnerTest.HANG_GUARD, () -> assertThrows(
                OpModeException.class,
                () -> runner(BusyWaitAuto.class).timer(clock).stuckTimeoutMillis(LinearOpModeRunnerTest.STUCK_MILLIS)
                        .activeTicks(10).run()));
        assertTrue(failure.getMessage().contains("busy-wait"), failure.getMessage());
        assertTrue(failure.getMessage().contains("call idle() in its body"), failure.getMessage());
        // The spin waits on a sim clock that will never move again. The run ends the thread at its next clock
        // read instead of leaving it spinning for the rest of the JVM; the clock is not touched here.
        leaks.awaitStartedThreadsEnd();
        leaks.assertStartedThreadsEnded("the abandoned busy-wait thread ended");
    }

    @Test
    void timeUnitSleepWaitsInSimTimeLikeThreadSleep() {
        runner(TimeUnitSleepAuto.class).activeTicks(30).run();

        assertEquals("returned", OpMode.blackboard.get("timeUnitSleep"));
        assertEquals(ticks(15), seconds("timeUnitWaited"), 1e-9, "TimeUnit.MILLISECONDS.sleep(300): 15 ticks");
    }

    @Test
    void anIdleLoopStillRunningAfterStopFailsOnceAndItsThreadEnds() throws InterruptedException {
        OpModeException failure = assertTimeoutPreemptively(LinearOpModeRunnerTest.HANG_GUARD, () -> assertThrows(
                OpModeException.class, () -> runner(IdlePastStopAuto.class).stuckTimeoutMillis(300).activeTicks(5)
                        .run()));
        assertTrue(failure.getMessage().contains("did not return from runOpMode() after STOP"), failure.getMessage());
        assertTrue(failure.getMessage().contains("put opModeIsActive() in its condition"), failure.getMessage());
        for (Throwable suppressed : failure.getSuppressed()) {
            assertFalse(suppressed.getMessage().contains("after it was told to stop"),
                    "shutDown did not wait a second stuck timeout for a thread already given up on: " + suppressed);
        }
        // With no run left, idle() throws as sleep() does, so the thread does not spin for the rest of the JVM.
        leaks.awaitStartedThreadsEnd();
        leaks.assertStartedThreadsEnded("the abandoned idle() loop ended");
    }

    @Test
    void aWaitThatIsNotMovedThrowsInterruptedAtOnceOnTheOpModesThread() {
        // As the runner's javadoc and the guide say: the OpMode thread's interrupt flag is kept set, so an
        // Object.wait there neither waits in sim time nor in real time.
        runner(ObjectWaitAuto.class).activeTicks(10).run();

        assertEquals("interrupted", OpMode.blackboard.get("objectWait"));
        assertEquals(0.0, seconds("objectWaited"), 1e-9, "no sim time passed");
    }

    // ---- the class-identity trade-off -------------------------------------------------------------

    @Test
    void theOpModeIsASecondCopyOfTheClassWithFreshStaticsEveryRun() {
        for (int run = 0; run < 2; run++) {
            OpModeRunner runner = runner(StaticCounterAuto.class).activeTicks(2);
            runner.run();
            assertEquals(1, OpMode.blackboard.get("runs"), "a fresh copy of the class, so a fresh static");

            OpMode ran = runner.opMode();
            assertEquals(StaticCounterAuto.class.getName(), ran.getClass().getName());
            assertNotSame(StaticCounterAuto.class, ran.getClass());
            assertThrows(ClassCastException.class, () -> StaticCounterAuto.class.cast(ran),
                    "the test's own class is not the one that ran");
        }
        assertEquals(0, StaticCounterAuto.runs, "the test's own copy of the static never moved");
    }

    @Test
    void theBlackboardIsClearedBeforeTheTeamOpModeIsBuiltNotAfter() {
        OpMode.blackboard.put("pose", "left over from an earlier run");
        runner(BlackboardAtBuildAuto.class).activeTicks(2).run();

        assertEquals("null", OpMode.blackboard.get("poseAtBuild"),
                "a field initializer sees the run's empty blackboard, not an earlier run's");
        assertEquals("BlackboardAtBuildAuto", OpMode.blackboard.get("builtBy"),
                "what the OpMode wrote while it was built is still there");

        OpMode.blackboard.clear();
        OpMode.blackboard.put("pose", "from the auto");
        runner(BlackboardAtBuildAuto.class).keepBlackboard(true).activeTicks(2).run();
        assertEquals("from the auto", OpMode.blackboard.get("poseAtBuild"), "keepBlackboard(true) keeps it");
    }

    @Test
    void aClassThatCannotBeRunIsRefusedWithWhy() {
        IllegalArgumentException missing = assertThrows(IllegalArgumentException.class,
                () -> OpModeRunner.forTeamClass("org.firstinspires.ftc.teamcode.NoSuchAuto", new FakeHardwareMap())
                        .run());
        assertTrue(missing.getMessage().contains("cannot find that class"), missing.getMessage());

        IllegalArgumentException notOpMode = assertThrows(IllegalArgumentException.class,
                () -> OpModeRunner.forTeamClass(PlainMath.class.getName(), new FakeHardwareMap()).run());
        assertTrue(notOpMode.getMessage().contains("not an OpMode"), notOpMode.getMessage());

        IllegalArgumentException outside = assertThrows(IllegalArgumentException.class,
                () -> runner(StaticCounterAuto.class).teamPackages("com.example.otherteam.").run());
        assertTrue(outside.getMessage().contains("not in a package SimLoop rewrites (com.example.otherteam.)"),
                outside.getMessage());
    }

    @Test
    void teamPackagesCanBeNamedAndAreOnlyForATeamClassRunner() {
        runner(IdleWaitAuto.class).teamPackages("org.firstinspires.ftc.teamcode.simclockfixture.")
                .activeTicks(60).run();
        assertEquals(1.0, seconds("waited"), 1e-9);

        assertThrows(IllegalStateException.class,
                () -> new OpModeRunner(new IdleWaitAuto(), new FakeHardwareMap()).teamPackages("com.example."));
        assertThrows(IllegalArgumentException.class, () -> runner(IdleWaitAuto.class).teamPackages("java."));
    }
}
