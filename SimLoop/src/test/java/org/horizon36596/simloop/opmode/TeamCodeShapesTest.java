package org.horizon36596.simloop.opmode;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertInstanceOf;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.simclockfixture.DeadlineAuto;
import org.firstinspires.ftc.teamcode.simclockfixture.InterfaceSleepAuto;
import org.firstinspires.ftc.teamcode.simclockfixture.JdkWaitUncaughtAuto;
import org.firstinspires.ftc.teamcode.simclockfixture.PedroUserAuto;
import org.firstinspires.ftc.teamcode.simclockfixture.PoseHandOffAuto;
import org.firstinspires.ftc.teamcode.simclockfixture.PoseReaderTeleOp;
import org.firstinspires.ftc.teamcode.simclockfixture.SleepQualifiersAuto;
import org.firstinspires.ftc.teamcode.simclockfixture.SleepZeroTeleOp;
import org.firstinspires.ftc.teamcode.simclockfixture.SystemClockAuto;
import org.firstinspires.ftc.teamcode.simclockfixture.WallDatesAuto;
import org.horizon36596.simloop.fakehardware.FakeHardwareMap;
import org.horizon36596.simloop.simclock.SimClockLoader;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import java.util.List;

/**
 * Pins that less common but legal shapes of team code do under {@link OpModeRunner#forTeamClass} what they do
 * on a robot, or fail saying why: calls qualified by a subclass, waits the rewriter moves beyond
 * {@code Thread.sleep}, and clock reads it leaves on the real clock but names. The fixtures are in
 * {@code org.firstinspires.ftc.teamcode.simclockfixture} and report through {@code OpMode.blackboard}.
 *
 * <p>Invariant tier: each test states what the robot does with the same code.
 */
class TeamCodeShapesTest {

    private static final double TICK_MS = OpModeRunner.DEFAULT_TICK_SECONDS * 1000;

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

    private static double number(String key) {
        return ((Number) OpMode.blackboard.get(key)).doubleValue();
    }

    @Test
    void aTeamStaticSleepThatHidesThreadSleepRunsTheTeamsMethodAndWaitsNothing() {
        runner(SleepQualifiersAuto.class).activeTicks(40).run();

        assertEquals(1, OpMode.blackboard.get("hidingCalls"), "HidingThread.sleep(5) is the team's own method");
        assertEquals(0.0, number("afterHiding"), 1e-9, "and it does not wait");
        assertEquals(5 * TICK_MS, number("afterWorkerNap"), 1e-6,
                "Worker.sleep(100) is the inherited Thread.sleep: 100 ms of sim time");
    }

    @Test
    void threadSleepQualifiedByAJdkSubclassOfThreadWaitsInSimTime() {
        runner(SleepQualifiersAuto.class).activeTicks(40).run();

        assertEquals(10 * TICK_MS, number("afterJdkSubclassSleep"), 1e-6,
                "ForkJoinWorkerThread.sleep(200) is Thread.sleep(200)");
    }

    @Test
    void androidSystemClockReadsAndWaitsInSimTime() {
        // Under a JVM test's stub android.jar these read 0 and wait for nothing; rewritten, they never load it.
        runner(SystemClockAuto.class).activeTicks(60).run();

        assertEquals(25, OpMode.blackboard.get("passes"), "500 ms of uptimeMillis() is 25 passes of 20 ms");
        assertEquals(500L, OpMode.blackboard.get("uptimeWaited"));
        assertEquals(500L, OpMode.blackboard.get("realtimeWaited"), "elapsedRealtime() reads the same clock");
        assertEquals(0.3, number("sleepWaited"), 1e-9, "SystemClock.sleep(300): 15 ticks");
    }

    @Test
    void threadSleepZeroIsHarmlessEvenInAnIterativeOpMode() {
        // On a robot Thread.sleep(0) freezes nothing, and as an instance it already passed; a yield must not
        // be refused as a wait.
        runner(SleepZeroTeleOp.class).activeTicks(5).run();

        assertEquals(5, OpMode.blackboard.get("loops"));
    }

    /** {@return the one warning that contains every one of {@code parts}; fails if there is not exactly one} */
    private static String theWarning(List<String> warnings, String... parts) {
        String found = null;
        for (String warning : warnings) {
            boolean all = true;
            for (String part : parts) {
                all &= warning.contains(part);
            }
            if (all) {
                assertEquals(null, found, "two warnings match " + String.join(" + ", parts) + ": " + warnings);
                found = warning;
            }
        }
        assertTrue(found != null, "no warning contains " + String.join(" + ", parts) + ": " + warnings);
        return found;
    }

    @Test
    void theSdksDeadlineKeepsTheRealClockAndTheRunSaysWhereItIsUsed() {
        List<String> warnings = runner(DeadlineAuto.class).activeTicks(60).run().warnings();

        assertEquals(false, OpMode.blackboard.get("deadlineExpired"),
                "a 200 ms Deadline has not expired after 1 s of sim time: it reads the laptop's clock");
        String constructed = theWarning(warnings, DeadlineAuto.class.getName() + ".runOpMode (DeadlineAuto.java:",
                "constructs the FTC SDK's Deadline timer");
        assertTrue(constructed.contains("keeps the real clock"), constructed);
        assertTrue(constructed.contains("Use an ElapsedTime instead"), constructed);
        theWarning(warnings, DeadlineAuto.class.getName() + "$RateLimit (DeadlineAuto.java)",
                "extends the FTC SDK's Deadline timer");
    }

    @Test
    void readsOfTheRealDateAndTimeAreNamedWithTheirLine() {
        List<String> warnings = runner(WallDatesAuto.class).activeTicks(2).run().warnings();

        String where = WallDatesAuto.class.getName() + ".runOpMode (WallDatesAuto.java:";
        for (String call : new String[] {"new java.util.Date()", "java.time.Instant.now()",
                "java.time.LocalDateTime.now()", "java.time.Clock.systemUTC()", "java.util.Calendar.getInstance()"}) {
            // One warning each: Instant.now(Clock.fixed(...)) reads the clock it is given, and is not named.
            String warning = theWarning(warnings, where, "(" + call + ")");
            assertTrue(warning.contains("is different on every run"), warning);
        }
    }

    @Test
    void aClockReadingLibraryOutsideTheTeamPackagesIsNamedOnceWithTheLineThatAddsIt() {
        List<String> warnings = runner(PedroUserAuto.class).activeTicks(40).run().warnings();

        String warning = theWarning(warnings, "Pedro Pathing");
        assertTrue(warning.contains("com.pedropathing.simclockfixture.PedroStyleTimer"), warning);
        assertTrue(warning.contains("teamPackages(\"org.firstinspires.ftc.teamcode.\","
                + " \"com.seattlesolvers.solverslib.\", \"com.pedropathing.\")"), warning);
        // Unrewritten, it read the laptop's clock across a sleep of 500 ms of sim time, so not exactly 0.5 s.
        assertTrue(Math.abs(number("pedroSeconds") - 0.5) > 1e-6, "pedroSeconds=" + number("pedroSeconds"));

        OpMode.blackboard.clear();
        List<String> added = runner(PedroUserAuto.class)
                .teamPackages("org.firstinspires.ftc.teamcode.", "com.seattlesolvers.solverslib.",
                        SimClockLoader.PEDRO_PATHING)
                .activeTicks(40).run().warnings();
        assertTrue(added.isEmpty(), "listed, it is rewritten and not warned about: " + added);
        assertEquals(0.5, number("pedroSeconds"), 1e-9, "and it reads the sim clock");
    }

    @Test
    void aTeamObjectHandedFromOneRunToTheNextFailsWithAHintAndAPlainNumberDoesNot() {
        runner(PoseHandOffAuto.class).activeTicks(2).run();
        OpModeException failure = assertThrows(OpModeException.class,
                () -> runner(PoseReaderTeleOp.class).keepBlackboard(true).activeTicks(2).run());

        assertInstanceOf(ClassCastException.class, failure.getCause());
        assertTrue(failure.getMessage().contains("SimLoop: each OpModeRunner.forTeamClass run loads the team's"
                + " classes afresh"), failure.getMessage());
        assertTrue(failure.getMessage().contains("Hand SDK or JDK types"), failure.getMessage());
        assertEquals(42.0, number("startX"), 1e-9, "the number crossed from the auto's run to the TeleOp's");
    }

    @Test
    void aJdkWaitThatThrowsInterruptedOnTheOpModeThreadSaysWhy() {
        OpModeException failure = assertThrows(OpModeException.class,
                () -> runner(JdkWaitUncaughtAuto.class).activeTicks(5).run());

        assertInstanceOf(InterruptedException.class, failure.getCause());
        assertTrue(failure.getMessage().contains("No STOP explains it"), failure.getMessage());
        assertTrue(failure.getMessage().contains("Object.wait"), failure.getMessage());
    }

    @Test
    void aSleepReachedThroughAnInterfaceSaysSoAndWhatToWrite() {
        // The call is an invokeinterface, which the rewrite leaves to the SDK's sleep; that cannot see a duration.
        OpModeException failure = assertThrows(OpModeException.class,
                () -> runner(InterfaceSleepAuto.class).activeTicks(10).run());

        String message = failure.getCause().getMessage();
        assertTrue(message.contains("InterfaceSleepAuto.runOpMode (InterfaceSleepAuto.java:"), message);
        assertTrue(message.contains("through an interface, a method reference or reflection"), message);
        assertTrue(message.contains("Call sleep(ms) directly on the OpMode"), message);
    }
}
