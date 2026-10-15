package org.horizon36596.simloop.opmode;

import static org.junit.jupiter.api.Assertions.assertEquals;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.simclockfixture.ComputedSleepGridAuto;
import org.firstinspires.ftc.teamcode.simclockfixture.IdleOnceAuto;
import org.firstinspires.ftc.teamcode.simclockfixture.SleepGridAuto;
import org.firstinspires.ftc.teamcode.simclockfixture.SleepThenIdleAuto;
import org.horizon36596.simloop.fakehardware.FakeHardwareMap;
import org.horizon36596.simloop.fakehardware.FakeMotor;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * Pins that code both runners can run takes the same ticks on both: {@code new OpModeRunner(opMode, ...)},
 * which reads a constant {@code sleep(ms)} from the bytecode and treats {@code idle()} as the SDK's yield, and
 * {@link OpModeRunner#forTeamClass}, which rewrites both. Also the rounding of computed sleeps and
 * {@code Thread.sleep(ms, nanos)}, which only the second can run.
 *
 * <p>Invariant tier: moving a test from one runner to the other must not change its timing.
 */
class InstancePathParityTest {

    @BeforeEach
    @AfterEach
    void emptyBlackboard() {
        OpMode.blackboard.clear();
    }

    @Test
    void constantSleepsWaitTheSameTicksOnBothPaths() {
        new OpModeRunner(new SleepGridAuto(), new FakeHardwareMap()).activeTicks(200).run();
        Map<String, Object> instance = new HashMap<>(OpMode.blackboard);
        OpMode.blackboard.clear();
        OpModeRunner.forTeamClass(SleepGridAuto.class, new FakeHardwareMap()).activeTicks(200).run();
        Map<String, Object> team = new HashMap<>(OpMode.blackboard);

        assertEquals(instance, team);
        assertEquals(0L, team.get("s0"));
        assertEquals(1L, team.get("s1"));
        assertEquals(1L, team.get("s19"));
        assertEquals(1L, team.get("s20"));
        assertEquals(2L, team.get("s21"));
        assertEquals(50L, team.get("s999"));
        assertEquals("IllegalArgumentException: timeout value is negative", team.get("sNeg"));
        assertEquals(false, team.get("afterMax.active"), "sleep(Long.MAX_VALUE) ends at STOP");
    }

    @Test
    void computedSleepsAndThreadSleepOverloadsRoundUpToWholeTicks() {
        OpModeRunner.forTeamClass(ComputedSleepGridAuto.class, new FakeHardwareMap()).activeTicks(400).run();
        Map<String, Object> b = OpMode.blackboard;
        long[][] expected = {{0, 0}, {1, 1}, {19, 1}, {20, 1}, {21, 2}, {999, 50}};
        for (long[] e : expected) {
            assertEquals(e[1], b.get("s" + e[0]), "sleep(" + e[0] + ")");
            assertEquals(e[1], b.get("t" + e[0]), "Thread.sleep(" + e[0] + ")");
        }
        assertEquals(0L, b.get("t0_0"));
        assertEquals(1L, b.get("t0_1"), "Thread.sleep(0, 1) waits a whole tick, never less than asked");
        assertEquals(1L, b.get("t19_999999"));
        assertEquals(1L, b.get("t20_0"));
        assertEquals(2L, b.get("t20_1"));
        assertEquals("IllegalArgumentException: nanosecond timeout value out of range", b.get("tBadNanos"));
        assertEquals("InterruptedException: sleep interrupted: the OpMode was stopped", b.get("tMax"));
        assertEquals(0L, b.get("sleepAfterStop.ticks"));
        assertEquals("InterruptedException", b.get("threadSleepAfterStop"));
    }

    /** The motor power at the end of every tick, for one run of {@link SleepThenIdleAuto}. */
    private static List<Double> powers(OpModeRunner runner, FakeMotor motor) {
        List<Double> powers = new ArrayList<>();
        runner.activeTicks(30).eachTick(deltaTime -> powers.add(Math.round(motor.getPower() * 100) / 100.0)).run();
        return powers;
    }

    @Test
    void aSleepThenIdleLoopTakesTheSameTicksOnBothPaths() {
        // The SDK's ConceptRampMotorSpeed loop: sleep(50) is 3 ticks, and the idle() after it adds none, as the
        // SDK's yield adds no time on the robot.
        FakeHardwareMap instanceMap = new FakeHardwareMap();
        FakeMotor instanceMotor = new FakeMotor(2787, 10);
        instanceMap.register("motor", instanceMotor);
        List<Double> instance = powers(new OpModeRunner(new SleepThenIdleAuto(), instanceMap), instanceMotor);

        FakeHardwareMap teamMap = new FakeHardwareMap();
        FakeMotor teamMotor = new FakeMotor(2787, 10);
        teamMap.register("motor", teamMotor);
        List<Double> team = powers(OpModeRunner.forTeamClass(SleepThenIdleAuto.class, teamMap), teamMotor);

        assertEquals(instance, team);
        // INIT, PLAY: 0.01 for ticks 1-3, 0.02 for 4-6, ...: three ticks per step.
        assertEquals(0.01, team.get(1), 1e-9);
        assertEquals(0.01, team.get(3), 1e-9);
        assertEquals(0.02, team.get(4), 1e-9);
        assertEquals(0.10, team.get(30), 1e-9);
    }

    @Test
    void aLoneIdleTakesNoTickOnEitherPath() {
        OpModeRunResult instance = new OpModeRunner(new IdleOnceAuto(), new FakeHardwareMap()).activeTicks(10).run();
        OpModeRunResult team = OpModeRunner.forTeamClass(IdleOnceAuto.class, new FakeHardwareMap()).activeTicks(10)
                .run();

        assertEquals(instance.telemetry().transmissions(), team.telemetry().transmissions());
        assertEquals(10, team.telemetry().transmissions().size(), "ten passes in ten active ticks");
    }
}
