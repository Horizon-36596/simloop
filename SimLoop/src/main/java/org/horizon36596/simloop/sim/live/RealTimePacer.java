package org.horizon36596.simloop.sim.live;

import org.horizon36596.simloop.sim.ScenarioRunner;

import java.util.function.LongSupplier;

/**
 * Slows a sim run down to real time, so a person can drive it. <b>This is the only class in SimLoop's
 * main sources that reads a wall clock</b>, and it is kept at the outer edge on purpose.
 *
 * <p><b>What it may and may not touch.</b> {@code ScenarioRunner} calls {@link #afterTick} once a loop has
 * finished and been logged. This class then sleeps until the wall clock has caught up with the sim clock,
 * and returns. That is all. It never writes to the {@code FakeTimer}, never hands a wall-clock reading to
 * anything, and cannot change what the next loop computes: the sim advances by the same fixed step
 * whether this class slept 20 ms or not at all. So a live session and its headless replay - which runs
 * with {@link ScenarioRunner#NO_PACING} as fast as the machine allows - compute the same numbers. A test
 * pins that ({@code WallClockConfinementTest} pins that nothing else in SimLoop reads the clock).
 *
 * <p><b>When the sim cannot keep up.</b> If a loop takes longer than its tick, the run falls behind the
 * wall clock. This class does not try to catch up by running the next loops back to back, because a burst
 * of loops is not what a driver would see on a robot either; once it is more than
 * {@link #MAX_LAG_BEFORE_RESET_SECONDS} behind, it forgets the debt and paces from where it is.
 * {@link #lagResets()} counts how often that happened, so a session can say it ran slow.
 */
public final class RealTimePacer implements ScenarioRunner.TickPacer {

    /**
     * How far behind the wall clock the run may fall before the pacer stops trying to catch up, seconds.
     * SET BY HAND: a quarter of a second is long enough to absorb a garbage collection and short enough
     * that a driver never sees the sim race to make up for one.
     */
    public static final double MAX_LAG_BEFORE_RESET_SECONDS = 0.25;

    /** How the pacer waits. The real one is {@link Thread#sleep}; a test passes one that records instead. */
    public interface Sleeper {
        /**
         * Wait at least this long.
         *
         * @param nanos how long, nanoseconds; always positive
         * @throws InterruptedException if the waiting thread is interrupted
         */
        void sleepNanos(long nanos) throws InterruptedException;
    }

    private final LongSupplier wallClockNanos;
    private final Sleeper sleeper;

    private boolean anchored = false;
    private long anchorWallNanos;
    private double anchorSimSeconds;
    private int lagResets = 0;

    /** Paces against the JVM's monotonic clock, {@link System#nanoTime()}, sleeping with {@link Thread#sleep}. */
    public RealTimePacer() {
        this(new LongSupplier() {
            @Override
            public long getAsLong() {
                return System.nanoTime();
            }
        }, new Sleeper() {
            @Override
            public void sleepNanos(long nanos) throws InterruptedException {
                Thread.sleep(nanos / 1_000_000L, (int) (nanos % 1_000_000L));
            }
        });
    }

    /**
     * A pacer on an injected clock, for tests.
     *
     * @param wallClockNanos a monotonic clock, nanoseconds
     * @param sleeper        how to wait
     */
    public RealTimePacer(LongSupplier wallClockNanos, Sleeper sleeper) {
        this.wallClockNanos = wallClockNanos;
        this.sleeper = sleeper;
    }

    @Override
    public void afterTick(int ticksCompleted, double simSeconds) {
        long now = wallClockNanos.getAsLong();
        if (!anchored) {
            // The first loop is the anchor: real time is measured from here on, not from before init.
            anchorAt(now, simSeconds);
            return;
        }
        long due = anchorWallNanos + Math.round((simSeconds - anchorSimSeconds) * 1e9);
        long ahead = due - now;
        if (ahead > 0) {
            try {
                sleeper.sleepNanos(ahead);
            } catch (InterruptedException interrupted) {
                Thread.currentThread().interrupt();
                throw new IllegalStateException("interrupted while pacing the sim to real time", interrupted);
            }
        } else if (-ahead > Math.round(MAX_LAG_BEFORE_RESET_SECONDS * 1e9)) {
            lagResets++;
            anchorAt(now, simSeconds);
        }
    }

    /** {@return how many times the run fell too far behind real time and the pacer stopped catching up} */
    public int lagResets() {
        return lagResets;
    }

    private void anchorAt(long wallNanos, double simSeconds) {
        anchored = true;
        anchorWallNanos = wallNanos;
        anchorSimSeconds = simSeconds;
    }
}
