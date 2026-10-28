package org.horizon36596.simloop.opmode;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.simclockfixture.InitHelperThreadTeleOp;
import org.firstinspires.ftc.teamcode.simclockfixture.PedroLocalizerTeleOp;
import org.firstinspires.ftc.teamcode.simclockfixture.ResetThenSleepInitTeleOp;
import org.firstinspires.ftc.teamcode.simclockfixture.SleepyInitLoopTeleOp;
import org.firstinspires.ftc.teamcode.simclockfixture.SleepyInitTeleOp;
import org.horizon36596.simloop.fakehardware.FakeHardwareMap;
import org.horizon36596.simloop.fakehardware.FakeMotor;
import org.horizon36596.simloop.simclock.SimClockLoader;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import java.util.List;

/**
 * Pins a wait in an iterative OpMode's {@code init()} under {@link OpModeRunner#forTeamClass}. The SDK
 * (RobotCore 12.0.0, {@code OpMode.internalRunOpMode()}) calls {@code init()} on the OpMode's own thread and
 * starts {@code init_loop()} only once it returns, so on a robot {@code init()} may block, as a library does
 * when it sleeps while a sensor calibrates. In the sim the wait holds {@code init()} for whole ticks of sim
 * time; every other iterative method still may not wait. The fixtures are in
 * {@code org.firstinspires.ftc.teamcode.simclockfixture} and report through {@code OpMode.blackboard}.
 *
 * <p>Invariant tier: each test states what the robot does with the same code.
 */
class IterativeInitWaitTest {

    /** One tick, in nanoseconds of sim time. */
    private static final long TICK_NANOS = Math.round(OpModeRunner.DEFAULT_TICK_SECONDS * 1e9);

    /** The 500 ms sleep in the fixtures' {@code init()}, in ticks: 500 ms / 20 ms. */
    private static final int SLEEP_TICKS = 25;

    @BeforeEach
    void emptyBlackboard() {
        OpMode.blackboard.clear();
    }

    private final OpModeThreadLeakCheck leaks = new OpModeThreadLeakCheck(OpModeThreadLeakCheck.ITERATIVE);

    @AfterEach
    void noOpModeThreadOutlivesItsRun() throws InterruptedException {
        OpMode.blackboard.clear();
        leaks.verify();
    }

    private static OpModeRunner runner(Class<? extends OpMode> teamClass) {
        FakeHardwareMap hardwareMap = new FakeHardwareMap();
        hardwareMap.register("motor", new FakeMotor(2787, 10));
        return OpModeRunner.forTeamClass(teamClass, hardwareMap);
    }

    /** {@return the sim clock's nanoseconds from {@code init()} starting to the reading under {@code key}} */
    private static long sinceInit(String key) {
        return (Long) OpMode.blackboard.get(key) - (Long) OpMode.blackboard.get("initNanos");
    }

    @Test
    void aWaitInInitHoldsItForSimTimeAndInitLoopStartsTheTickAfterItReturns() {
        runner(SleepyInitTeleOp.class).initLoopTicks(30).activeTicks(2).run();

        assertEquals("normally", OpMode.blackboard.get("slept"));
        assertEquals(SLEEP_TICKS * TICK_NANOS, sinceInit("initReturnNanos"), "500 ms of sim time, 25 ticks");
        assertEquals((SLEEP_TICKS + 1) * TICK_NANOS, sinceInit("firstInitLoopNanos"),
                "the tick init() returns on is its own; init_loop() starts on the next");
        assertEquals(30 - SLEEP_TICKS, OpMode.blackboard.get("initLoops"), "ticks 26 to 30");
        assertEquals(0.5, (Double) OpMode.blackboard.get("runtimeAfterSleep"), 0.01,
                "getRuntime() followed the sim clock through the wait, as it follows the real one on a robot");
        assertTrue((Integer) OpMode.blackboard.get("encoderAfterSleep") > 0,
                "the motor init() started kept turning while init() waited");
    }

    @Test
    void aResetRuntimeJustBeforeTheWaitCountsTheWholeWait() {
        runner(ResetThenSleepInitTeleOp.class).initLoopTicks(30).activeTicks(1).run();

        assertEquals(0.5, (Double) OpMode.blackboard.get("runtimeAfterSleep"), 0.005,
                "getRuntime() counts from the resetRuntime() call, as on a robot, not from the tick after it");
    }

    @Test
    void playDuringTheWaitSkipsInitLoopAndStartsOnTheTickAfterInitReturns() {
        runner(SleepyInitTeleOp.class).initLoopTicks(5).activeTicks(30).run();

        assertFalse(OpMode.blackboard.containsKey("initLoops"), "internalRunOpMode() goes straight to start()");
        assertEquals((SLEEP_TICKS + 1) * TICK_NANOS, sinceInit("startNanos"));
        assertEquals(OpMode.blackboard.get("startNanos"), OpMode.blackboard.get("firstLoopNanos"),
                "the first loop() comes straight after start(), as always");
    }

    @Test
    void stopDuringTheWaitEndsItWithoutAnInterruptAndStopRuns() {
        runner(SleepyInitTeleOp.class).initLoopTicks(5).activeTicks(0).run();

        assertEquals("normally", OpMode.blackboard.get("slept"),
                "the SDK does not interrupt an iterative OpMode at STOP; it waits for init() to return");
        assertEquals(6 * TICK_NANOS, sinceInit("initReturnNanos"), "sim time ends at STOP, on tick 6");
        assertFalse(OpMode.blackboard.containsKey("startNanos"), "PLAY never came");
        assertEquals(6 * TICK_NANOS, sinceInit("stopNanos"));
    }

    @Test
    void stopAfterPlayDuringTheWaitRunsStartThenStopButNoLoop() {
        runner(SleepyInitTeleOp.class).initLoopTicks(2).activeTicks(3).run();

        assertEquals(6 * TICK_NANOS, sinceInit("startNanos"), "start() on the STOP tick, once init() returned");
        assertFalse(OpMode.blackboard.containsKey("firstLoopNanos"), "stopRequested is checked before loop()");
        assertEquals(6 * TICK_NANOS, sinceInit("stopNanos"));
    }

    @Test
    void aLibraryThatSleepsWhileItCalibratesCanBeBuiltInInit() {
        runner(PedroLocalizerTeleOp.class)
                .teamPackages("org.firstinspires.ftc.teamcode.", "com.seattlesolvers.solverslib.",
                        SimClockLoader.PEDRO_PATHING)
                .initLoopTicks(30).activeTicks(1).run();

        assertEquals(SLEEP_TICKS * TICK_NANOS, (long) (Long) OpMode.blackboard.get("buildNanos"),
                "the library's 500 ms sleep took 500 ms of sim time");
    }

    @Test
    void aWaitInInitIsTheSameEveryRun() {
        List<List<String>> first = runner(SleepyInitTeleOp.class).initLoopTicks(30).activeTicks(2).run()
                .telemetry().transmissions();
        Object firstEncoder = OpMode.blackboard.get("encoderAfterSleep");
        OpMode.blackboard.clear();
        List<List<String>> second = runner(SleepyInitTeleOp.class).initLoopTicks(30).activeTicks(2).run()
                .telemetry().transmissions();

        assertEquals(first, second);
        assertEquals(firstEncoder, OpMode.blackboard.get("encoderAfterSleep"));
    }

    @Test
    void aWaitInInitLoopIsStillRefused() {
        OpModeException failure = assertThrows(OpModeException.class,
                () -> runner(SleepyInitLoopTeleOp.class).initLoopTicks(3).run());

        assertEquals(OpModePhase.INIT_LOOP, failure.phase());
        String message = failure.getCause().getMessage();
        assertTrue(message.startsWith("SimLoop cannot run Thread.sleep(10) at "
                + SleepyInitLoopTeleOp.class.getName() + ".init_loop(SleepyInitLoopTeleOp.java:"), message);
        assertTrue(message.contains("SleepyInitLoopTeleOp is an iterative OpMode"), message);
        assertTrue(message.contains("only init() may wait"), message);
    }

    @Test
    void aWaitOnAThreadStartedInInitIsStillRefused() {
        OpModeException failure = assertThrows(OpModeException.class,
                () -> runner(InitHelperThreadTeleOp.class).run());

        assertEquals(OpModePhase.INIT, failure.phase());
        assertTrue(failure.getMessage().contains("it was called on thread \"init-helper\""), failure.getMessage());
        assertFalse(OpMode.blackboard.containsKey("helperSlept"));
    }
}
