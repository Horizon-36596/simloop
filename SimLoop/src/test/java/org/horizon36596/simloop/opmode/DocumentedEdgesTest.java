package org.horizon36596.simloop.opmode;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.simclockfixture.OtherWaitsAuto;
import org.firstinspires.ftc.teamcode.simclockfixture.StartReadsAuto;
import org.horizon36596.simloop.fakehardware.FakeHardwareMap;
import org.horizon36596.simloop.simclock.SimClockHooks;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * Pins the edges {@code SimLoop/docs/unchanged-team-code.md} and the README describe, so the text and the
 * behaviour cannot drift apart again: which waits on a {@code LinearOpMode}'s thread end at once and which
 * wait in sim time, on both runners; and what the sim clock reads while the OpMode is built, at INIT and at
 * PLAY.
 *
 * <p>Implementation tier: these are documented limits, not promises; change them with the docs.
 */
class DocumentedEdgesTest {

    @BeforeEach
    @AfterEach
    void emptyBlackboard() {
        OpMode.blackboard.clear();
    }

    private static double number(String key) {
        return ((Number) OpMode.blackboard.get(key)).doubleValue();
    }

    private static void assertEndedAtOnce(String probe) {
        assertEquals("InterruptedException", OpMode.blackboard.get(probe), probe);
        assertEquals(0.0, number(probe + ".simSeconds"), 1e-9, probe + ": no sim time");
        assertTrue(number(probe + ".realMs") < 300, probe + ": and not the 300 ms of real time it asked for");
    }

    @Test
    void underForTeamClassTimeUnitSleepWaitsAndOtherJdkWaitsEndAtOnce() {
        OpModeRunner.forTeamClass(OtherWaitsAuto.class, new FakeHardwareMap()).activeTicks(40).run();

        assertEndedAtOnce("objectWait");
        assertEquals("returned", OpMode.blackboard.get("timeUnitSleep"));
        assertEquals(0.3, number("timeUnitSleep.simSeconds"), 1e-9, "TimeUnit.sleep is moved: 15 ticks");
        assertEndedAtOnce("librarySleep"); // a Thread.sleep outside the team packages is not moved
    }

    @Test
    void onAnInstanceRunEveryJdkWaitEndsAtOnce() {
        new OpModeRunner(new OtherWaitsAuto(), new FakeHardwareMap()).activeTicks(40).run();

        assertEndedAtOnce("objectWait");
        assertEndedAtOnce("timeUnitSleep");
        assertEndedAtOnce("librarySleep");
    }

    @Test
    void nanoTimeIsZeroWhileTheOpModeIsBuiltAndOneTickAtInit() {
        OpModeRunner.forTeamClass(StartReadsAuto.class, new FakeHardwareMap()).initLoopTicks(3).activeTicks(5).run();

        assertEquals(0L, OpMode.blackboard.get("builtNanos"), "a field initializer runs before the first tick");
        assertEquals(20_000_000L, OpMode.blackboard.get("initNanos"), "runOpMode() starts one tick in");
        assertEquals(SimClockHooks.SIM_EPOCH_MILLIS + 20, OpMode.blackboard.get("initMillis"));
        assertEquals(0.0, number("initTime"), 1e-9, "the OpMode's time field counts from INIT, a tick later");
        assertEquals(100_000_000L, OpMode.blackboard.get("startNanos"), "INIT, three INIT_LOOP ticks, then PLAY");
    }
}
