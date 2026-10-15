package org.horizon36596.simloop.sim;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ElapsedTime.Resolution;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import java.lang.reflect.Method;
import java.lang.reflect.Modifier;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.TreeSet;
import java.util.concurrent.TimeUnit;
import java.util.function.LongSupplier;
import java.util.function.Supplier;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotSame;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * Pins {@link SimElapsedTime}'s contract: it is the SDK's {@code ElapsedTime} in every observable respect,
 * except that "now" is the sim clock ({@link FakeTimer}) rather than the wall clock.
 *
 * <p>"The same as the real one" is checked two ways. Against hand-worked numbers (a 20 ms tick, fifty
 * ticks, one second), and against the real SDK class itself: {@link ScriptedElapsedTime} is an unmodified
 * {@code ElapsedTime} whose clock the test sets by hand, so any reading where the two differ is a place
 * where this class is not a drop-in.
 *
 * <p>Tiers (R9): the wall-clock independence and determinism tests are INVARIANT tier - they are the reason
 * the class exists (R5). The rest are implementation tier.
 */
public class SimElapsedTimeTest {

    /** One sim tick, seconds: the 50 Hz step SimLoop scenarios use. */
    private static final double TICK_SECONDS = 0.02;

    private FakeTimer timer;

    @BeforeEach
    public void setUp() {
        timer = new FakeTimer();
        ScriptedElapsedTime.nowNanos = 0L;
    }

    // ---- hand-worked values ------------------------------------------------------------------------

    @Test
    public void freshTimerReadsZeroInEveryUnit() {
        timer.advance(12.5);
        SimElapsedTime elapsed = new SimElapsedTime(timer);

        assertEquals(0L, elapsed.nanoseconds());
        assertEquals(0.0, elapsed.seconds());
        assertEquals(0.0, elapsed.milliseconds());
        assertEquals(0.0, elapsed.time());
        for (TimeUnit unit : TimeUnit.values()) {
            assertEquals(0L, elapsed.time(unit), unit.name());
        }
    }

    @Test
    public void startsAtTheSimClocksCurrentTime() {
        timer.advance(3.0);
        SimElapsedTime elapsed = new SimElapsedTime(timer);

        assertEquals(3_000_000_000L, elapsed.startTimeNanoseconds());
        assertEquals(3.0, elapsed.startTime());
        assertEquals(3L, elapsed.now(TimeUnit.SECONDS));
    }

    @Test
    public void fiftyTwentyMillisecondTicksReadExactlyOneSecond() {
        SimElapsedTime elapsed = new SimElapsedTime(timer);
        for (int tick = 0; tick < 50; tick++) {
            timer.advance(TICK_SECONDS);
        }

        // The double sum of fifty 0.02 s steps is not exactly 1.0; the nanosecond reading still is.
        assertEquals(1_000_000_000L, elapsed.nanoseconds());
        assertEquals(1.0, elapsed.seconds());
        assertEquals(1000.0, elapsed.milliseconds());
        assertEquals(1.0, elapsed.time());
        assertEquals(1000L, elapsed.time(TimeUnit.MILLISECONDS));
        assertEquals(1L, elapsed.time(TimeUnit.SECONDS));
    }

    @Test
    public void everyTickOfAThirtySecondRunReadsItsExactNanoseconds() {
        SimElapsedTime elapsed = new SimElapsedTime(timer);
        for (int tick = 1; tick <= 1500; tick++) {
            timer.advance(TICK_SECONDS);
            assertEquals(tick * 20_000_000L, elapsed.nanoseconds(), "tick " + tick);
            assertEquals(tick * 20L, elapsed.time(TimeUnit.MILLISECONDS), "tick " + tick);
        }
        assertEquals(30.0, elapsed.seconds());
    }

    @Test
    public void nanoTimeOfRoundsToTheNearestNanosecondRatherThanTruncating() {
        timer.advance(0.1);
        timer.advance(0.2); // 0.30000000000000004 s
        assertEquals(300_000_000L, SimElapsedTime.nanoTimeOf(timer));

        FakeTimer justUnderOne = new FakeTimer();
        justUnderOne.advance(0.9999999999999999); // one ulp under 1.0; truncation would give 999_999_999
        assertEquals(1_000_000_000L, SimElapsedTime.nanoTimeOf(justUnderOne));
    }

    @Test
    public void resetRestartsTheCountFromTheCurrentSimTime() {
        SimElapsedTime elapsed = new SimElapsedTime(timer);
        timer.advance(2.0);
        elapsed.reset();

        assertEquals(0L, elapsed.nanoseconds());
        assertEquals(2.0, elapsed.startTime());

        timer.advance(0.25);
        assertEquals(250_000_000L, elapsed.nanoseconds());
        assertEquals(250.0, elapsed.milliseconds());
    }

    @Test
    public void millisecondResolutionReportsTimeAndStartTimeInMilliseconds() {
        timer.advance(1.0);
        SimElapsedTime elapsed = new SimElapsedTime(timer, Resolution.MILLISECONDS);
        timer.advance(0.75);

        assertEquals(Resolution.MILLISECONDS, elapsed.getResolution());
        assertEquals(750.0, elapsed.time());
        assertEquals(1000.0, elapsed.startTime());
        // seconds()/milliseconds()/nanoseconds() do not depend on the resolution, as in the SDK.
        assertEquals(0.75, elapsed.seconds());
        assertEquals(750.0, elapsed.milliseconds());
        assertEquals(750_000_000L, elapsed.nanoseconds());
    }

    @Test
    public void explicitStartTimeCountsFromThatMomentOnTheSimClock() {
        timer.advance(5.0);
        SimElapsedTime elapsed = new SimElapsedTime(timer, 4_000_000_000L);

        assertEquals(Resolution.SECONDS, elapsed.getResolution());
        assertEquals(4_000_000_000L, elapsed.startTimeNanoseconds());
        assertEquals(1.0, elapsed.seconds());
    }

    @Test
    public void afterTheSimClockIsResetATimerReadsNegativeUntilItIsResetToo() {
        SimElapsedTime elapsed = new SimElapsedTime(timer);
        timer.advance(1.0);
        elapsed.reset();
        timer.reset();

        assertEquals(-1_000_000_000L, elapsed.nanoseconds());
        elapsed.reset();
        assertEquals(0L, elapsed.nanoseconds());
    }

    @Test
    public void toStringMatchesTheSdkFormat() {
        SimElapsedTime seconds = new SimElapsedTime(timer);
        SimElapsedTime millis = new SimElapsedTime(timer, Resolution.MILLISECONDS);
        timer.advance(1.5);

        assertEquals(String.format("%1.4f seconds", 1.5), seconds.toString());
        assertEquals(String.format("%1.4f milliseconds", 1500.0), millis.toString());
    }

    // ---- against the real SDK class --------------------------------------------------------------

    @Test
    public void everyReadingMatchesTheRealElapsedTimeAtTheSameInstant() {
        for (Resolution resolution : Resolution.values()) {
            timer.reset();
            timer.advance(0.4);
            ScriptedElapsedTime.nowNanos = SimElapsedTime.nanoTimeOf(timer);
            SimElapsedTime sim = new SimElapsedTime(timer, resolution);
            ScriptedElapsedTime real = new ScriptedElapsedTime(resolution);

            double[] steps = {0.0, 0.02, 0.001, 0.123456789, 1.0, 7.5, 0.0000005, 29.99};
            for (double step : steps) {
                timer.advance(step);
                ScriptedElapsedTime.nowNanos = SimElapsedTime.nanoTimeOf(timer);
                assertSameReadings(real, sim, resolution + " after +" + step + " s");
                if (step == 1.0) {
                    sim.reset();
                    real.reset();
                    assertSameReadings(real, sim, resolution + " after reset");
                }
            }
        }
    }

    @Test
    public void explicitStartTimeMatchesTheRealLongConstructor() {
        timer.advance(10.0);
        ScriptedElapsedTime.nowNanos = SimElapsedTime.nanoTimeOf(timer);
        SimElapsedTime sim = new SimElapsedTime(timer, 7_250_000_000L);
        ScriptedElapsedTime real = new ScriptedElapsedTime(7_250_000_000L);

        assertSameReadings(real, sim, "explicit start");
    }

    @Test
    public void nullResolutionFailsTheWayTheSdkDoes() {
        assertThrows(NullPointerException.class, () -> new ScriptedElapsedTime((Resolution) null));
        assertThrows(NullPointerException.class, () -> new SimElapsedTime(timer, (Resolution) null));
    }

    /**
     * Pins the SDK surface this class relies on. If an SDK bump adds a public method to {@code ElapsedTime},
     * or stops routing "now" through the protected, overridable {@code nsNow()}, this fails and the new
     * surface has to be checked against the sim clock before the bump lands.
     */
    @Test
    public void theSdkSurfaceIsTheOneThisClassWasCheckedAgainst() throws NoSuchMethodException {
        Method nsNow = ElapsedTime.class.getDeclaredMethod("nsNow");
        assertTrue(Modifier.isProtected(nsNow.getModifiers()), "nsNow() must stay protected");
        assertFalse(Modifier.isFinal(nsNow.getModifiers()), "nsNow() must stay overridable");

        TreeSet<String> publicMethods = new TreeSet<>();
        for (Method method : ElapsedTime.class.getDeclaredMethods()) {
            if (Modifier.isPublic(method.getModifiers()) && !method.isSynthetic()) {
                StringBuilder signature = new StringBuilder(method.getName()).append('(');
                for (Class<?> parameter : method.getParameterTypes()) {
                    signature.append(parameter.getSimpleName());
                }
                publicMethods.add(signature.append(')').toString());
            }
        }
        TreeSet<String> checked = new TreeSet<>(Arrays.asList(
                "getResolution()", "log(String)", "milliseconds()", "nanoseconds()", "now(TimeUnit)",
                "reset()", "seconds()", "startTime()", "startTimeNanoseconds()", "time()", "time(TimeUnit)",
                "toString()"));
        assertEquals(checked, publicMethods);
    }

    // ---- wall clock and determinism (INVARIANT tier) ---------------------------------------------

    @Test
    public void invariant_realTimePassingDoesNotMoveTheReading() throws InterruptedException {
        timer.advance(1.0);
        SimElapsedTime elapsed = new SimElapsedTime(timer);
        timer.advance(0.5);
        String before = elapsed.toString();

        Thread.sleep(50);
        long busyUntil = System.nanoTime() + 20_000_000L;
        while (System.nanoTime() < busyUntil) {
            // spin: real time passes, the sim clock does not
        }

        assertEquals(500_000_000L, elapsed.nanoseconds());
        assertEquals(before, elapsed.toString());
        assertEquals(1_000_000_000L, elapsed.startTimeNanoseconds());
    }

    @Test
    public void invariant_twoIdenticalRunsReadIdenticalTimes() {
        assertEquals(scriptedRun(), scriptedRun());
    }

    @Test
    public void invariant_aTimedWaitFinishesOnTheSameTickEveryRun() {
        // The WaitCommand shape: start a timer, finish when 250 ms have passed. On 20 ms ticks that is
        // tick 13 (260 ms), the first tick at or past 250 ms - every run, however long each tick takes.
        for (int run = 0; run < 3; run++) {
            FakeTimer clock = new FakeTimer();
            ElapsedTime wait = new SimElapsedTime(clock);
            int tick = 0;
            while (wait.milliseconds() < 250.0) {
                clock.advance(TICK_SECONDS);
                tick++;
            }
            assertEquals(13, tick, "run " + run);
        }
    }

    // ---- the injection seam -----------------------------------------------------------------------

    @Test
    public void nanoClockReadsTheSimClock() {
        LongSupplier nanoClock = SimElapsedTime.nanoClock(timer);
        assertEquals(0L, nanoClock.getAsLong());
        timer.advance(0.02);
        assertEquals(20_000_000L, nanoClock.getAsLong());
    }

    @Test
    public void factoryMakesAFreshTimerStartedAtTheCurrentSimTime() {
        Supplier<ElapsedTime> factory = SimElapsedTime.factory(timer);
        ElapsedTime first = factory.get();
        timer.advance(1.0);
        ElapsedTime second = factory.get();
        timer.advance(0.5);

        assertNotSame(first, second);
        assertTrue(first instanceof SimElapsedTime);
        assertEquals(1.5, first.seconds());
        assertEquals(0.5, second.seconds());
    }

    @Test
    public void aNullClockIsRefusedWithAMessageThatNamesTheFix() {
        NullPointerException refused =
                assertThrows(NullPointerException.class, () -> new SimElapsedTime(null));
        assertTrue(refused.getMessage().contains("FakeTimer"), refused.getMessage());
        assertThrows(NullPointerException.class, () -> new SimElapsedTime(null, Resolution.SECONDS));
        assertThrows(NullPointerException.class, () -> new SimElapsedTime(null, 0L));
        assertThrows(NullPointerException.class, () -> SimElapsedTime.nanoTimeOf(null));
        assertThrows(NullPointerException.class, () -> SimElapsedTime.nanoClock(null));
        assertThrows(NullPointerException.class, () -> SimElapsedTime.factory(null));
    }

    // ---- helpers ----------------------------------------------------------------------------------

    /** Every public reading of the two timers, compared one by one so a failure names the method. */
    private static void assertSameReadings(ElapsedTime real, ElapsedTime sim, String when) {
        assertEquals(real.nanoseconds(), sim.nanoseconds(), when + ": nanoseconds()");
        assertEquals(real.seconds(), sim.seconds(), when + ": seconds()");
        assertEquals(real.milliseconds(), sim.milliseconds(), when + ": milliseconds()");
        assertEquals(real.time(), sim.time(), when + ": time()");
        assertEquals(real.startTime(), sim.startTime(), when + ": startTime()");
        assertEquals(real.startTimeNanoseconds(), sim.startTimeNanoseconds(), when + ": startTimeNanoseconds()");
        assertEquals(real.getResolution(), sim.getResolution(), when + ": getResolution()");
        assertEquals(real.toString(), sim.toString(), when + ": toString()");
        for (TimeUnit unit : TimeUnit.values()) {
            assertEquals(real.time(unit), sim.time(unit), when + ": time(" + unit + ")");
            assertEquals(real.now(unit), sim.now(unit), when + ": now(" + unit + ")");
        }
    }

    /** A short run with uneven steps and resets, recorded as text so two runs compare as a whole. */
    private static List<String> scriptedRun() {
        FakeTimer clock = new FakeTimer();
        SimElapsedTime seconds = new SimElapsedTime(clock);
        SimElapsedTime millis = new SimElapsedTime(clock, Resolution.MILLISECONDS);
        List<String> readings = new ArrayList<>();
        for (int tick = 0; tick < 400; tick++) {
            clock.advance(tick % 7 == 0 ? 0.013 : TICK_SECONDS);
            if (tick % 97 == 0) {
                seconds.reset();
            }
            readings.add(seconds.nanoseconds() + " " + seconds.time() + " " + millis.time() + " " + millis);
        }
        return readings;
    }

    /**
     * The real SDK {@code ElapsedTime}, unmodified, except that its clock is {@link #nowNanos} instead of
     * {@code System.nanoTime()}. A static field rather than an instance one because the SDK's constructor
     * reads the clock before a subclass's own fields exist.
     */
    private static final class ScriptedElapsedTime extends ElapsedTime {
        static long nowNanos;

        ScriptedElapsedTime(Resolution resolution) {
            super(resolution);
        }

        ScriptedElapsedTime(long startTimeNanoseconds) {
            super(startTimeNanoseconds);
        }

        @Override
        protected long nsNow() {
            return nowNanos;
        }
    }
}
