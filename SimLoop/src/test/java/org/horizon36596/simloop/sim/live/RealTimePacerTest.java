package org.horizon36596.simloop.sim.live;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.List;
import java.util.function.LongSupplier;

/**
 * Pins what the pacer does with the wall clock, on a clock the test owns so nothing here sleeps.
 * IMPLEMENTATION tier: a different pacing policy could be just as valid. The invariant that matters - the
 * pacer cannot change what the sim computes - is pinned by {@code LiveSessionReplayTest}.
 */
class RealTimePacerTest {

    /** A wall clock that only moves when the test, or a sleep, moves it. */
    private static final class TestClock implements LongSupplier, RealTimePacer.Sleeper {
        long nanos = 1_000_000_000L;
        final List<Long> sleeps = new ArrayList<>();

        @Override public long getAsLong() { return nanos; }

        @Override public void sleepNanos(long requested) {
            sleeps.add(requested);
            nanos += requested;
        }
    }

    @Test
    void aFastSimIsHeldBackToOneTickPerTickOfWallClock() {
        TestClock clock = new TestClock();
        RealTimePacer pacer = new RealTimePacer(clock, clock);

        pacer.afterTick(1, 0.02);      // anchors
        clock.nanos += 5_000_000L;     // the next loop took 5 ms of real time
        pacer.afterTick(2, 0.04);

        assertEquals(1, clock.sleeps.size());
        assertEquals(15_000_000L, (long) clock.sleeps.get(0), "a 20 ms tick that took 5 ms waits 15 ms");
    }

    @Test
    void aSlowSimIsNeverMadeToWaitAndDoesNotRaceToCatchUpAfterALongStall() {
        TestClock clock = new TestClock();
        RealTimePacer pacer = new RealTimePacer(clock, clock);

        pacer.afterTick(1, 0.02);
        clock.nanos += 30_000_000L;    // 30 ms for a 20 ms tick: 10 ms behind, within the allowance
        pacer.afterTick(2, 0.04);
        assertTrue(clock.sleeps.isEmpty(), "behind real time means no sleep");
        assertEquals(0, pacer.lagResets());

        clock.nanos += 500_000_000L;   // a half-second stall
        pacer.afterTick(3, 0.06);
        assertEquals(1, pacer.lagResets(), "past the allowance the debt is forgotten");

        clock.nanos += 5_000_000L;
        pacer.afterTick(4, 0.08);
        assertEquals(15_000_000L, (long) clock.sleeps.get(0), "and pacing resumes from where the run is");
    }
}
