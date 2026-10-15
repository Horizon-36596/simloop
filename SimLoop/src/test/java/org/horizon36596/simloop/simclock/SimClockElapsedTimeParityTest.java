package org.horizon36596.simloop.simclock;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.simclockfixture.SuperNsNowTimer;
import org.horizon36596.simloop.sim.FakeTimer;
import org.junit.jupiter.api.Test;

import java.lang.reflect.Constructor;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;

/**
 * Pins that {@link SimClockElapsedTime} is the SDK's {@code ElapsedTime} in every way a team can see except
 * the clock it reads: every public method, from every constructor, gives what the RobotCore 12.0.0 timer gives
 * when its {@code nsNow()} reads the same nanoseconds; and a team subclass that calls {@code super.nsNow()}
 * after re-parenting reads the sim clock.
 *
 * <p>Invariant tier: a timer that moved onto sim time but changed its arithmetic would be quietly wrong.
 */
class SimClockElapsedTimeParityTest {

    /** The SDK's own ElapsedTime, driven by a controlled nsNow: the reference. */
    static final class ControlledElapsedTime extends ElapsedTime {
        static long now;

        ControlledElapsedTime() {
            super();
        }

        ControlledElapsedTime(long start) {
            super(start);
        }

        ControlledElapsedTime(Resolution resolution) {
            super(resolution);
        }

        @Override
        protected long nsNow() {
            return now;
        }
    }

    private static void setClock(FakeTimer clock, long nanos) {
        clock.reset();
        clock.advance(nanos / 1e9);
    }

    private static List<String> readAll(ElapsedTime timer) {
        List<String> out = new ArrayList<>();
        out.add("seconds=" + timer.seconds());
        out.add("milliseconds=" + timer.milliseconds());
        out.add("nanoseconds=" + timer.nanoseconds());
        out.add("time=" + timer.time());
        for (TimeUnit unit : TimeUnit.values()) {
            out.add("time(" + unit + ")=" + timer.time(unit));
            out.add("now(" + unit + ")=" + timer.now(unit));
        }
        out.add("startTime=" + timer.startTime());
        out.add("startTimeNanoseconds=" + timer.startTimeNanoseconds());
        out.add("getResolution=" + timer.getResolution());
        out.add("toString=" + timer);
        return out;
    }

    @Test
    void everyPublicMethodReadsAsTheSdksTimerDoesAtTheSameNanoseconds() throws Exception {
        FakeTimer clock = new FakeTimer();
        SimClockLoader loader = new SimClockLoader(getClass().getClassLoader(), clock,
                SimClockLoader.DEFAULT_TEAM_PACKAGES);
        Class<?> simClass = Class.forName(SimClockElapsedTime.class.getName(), true, loader);
        Constructor<?> noArg = simClass.getConstructor();
        Constructor<?> fromLong = simClass.getConstructor(long.class);
        Constructor<?> fromResolution = simClass.getConstructor(ElapsedTime.Resolution.class);
        long[] starts = {0L, 20_000_000L, 1_234_567_891L, 3_600_000_000_000L};
        long[] deltas = {0L, 1L, 999_999L, 20_000_000L, 1_000_000_000L, 12_345_678_901L};
        List<String> diffs = new ArrayList<>();
        int compared = 0;
        for (long start : starts) {
            for (long delta : deltas) {
                for (int kind = 0; kind < 4; kind++) {
                    setClock(clock, start);
                    ControlledElapsedTime.now = start;
                    ElapsedTime sim;
                    ElapsedTime reference;
                    switch (kind) {
                        case 0:
                            sim = (ElapsedTime) noArg.newInstance();
                            reference = new ControlledElapsedTime();
                            break;
                        case 1:
                            sim = (ElapsedTime) fromLong.newInstance(start / 2);
                            reference = new ControlledElapsedTime(start / 2);
                            break;
                        case 2:
                            sim = (ElapsedTime) fromResolution.newInstance(ElapsedTime.Resolution.MILLISECONDS);
                            reference = new ControlledElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
                            break;
                        default:
                            sim = (ElapsedTime) fromResolution.newInstance(ElapsedTime.Resolution.SECONDS);
                            reference = new ControlledElapsedTime(ElapsedTime.Resolution.SECONDS);
                            break;
                    }
                    setClock(clock, start + delta);
                    ControlledElapsedTime.now = start + delta;
                    List<String> simReadings = readAll(sim);
                    List<String> referenceReadings = readAll(reference);
                    compared += simReadings.size();
                    if (!simReadings.equals(referenceReadings)) {
                        diffs.add("start=" + start + " delta=" + delta + " kind=" + kind + "\n sim=" + simReadings
                                + "\n ref=" + referenceReadings);
                    }
                    sim.reset();
                    reference.reset();
                    if (sim.startTimeNanoseconds() != reference.startTimeNanoseconds()) {
                        diffs.add("reset start=" + start + " delta=" + delta + " kind=" + kind);
                    }
                }
            }
        }
        assertTrue(compared > 2000, "compared " + compared + " readings");
        assertTrue(diffs.isEmpty(), diffs.size() + " differences, first: " + (diffs.isEmpty() ? "" : diffs.get(0)));
    }

    @Test
    void aReparentedSubclassCallingSuperNsNowReadsTheSimClock() throws Exception {
        FakeTimer clock = new FakeTimer();
        clock.advance(2.5);
        SimClockLoader loader = new SimClockLoader(getClass().getClassLoader(), clock,
                SimClockLoader.DEFAULT_TEAM_PACKAGES);
        Class<?> timerClass = Class.forName(SuperNsNowTimer.class.getName(), true, loader);
        Object timer = timerClass.getConstructor().newInstance();

        assertEquals(SimClockElapsedTime.class.getName(), timerClass.getSuperclass().getName());
        assertEquals(2_500_000_007L, timerClass.getMethod("rawNow").invoke(timer), "super.nsNow() + 7");
    }
}
