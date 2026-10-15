package org.horizon36596.simloop.simclock;

import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * The FTC SDK's {@link ElapsedTime}, reading the sim clock of the {@link SimClockLoader} that defined it.
 * This is what a team's unchanged {@code new ElapsedTime()} makes once {@link ClassFileRewriter} has
 * rewritten the team's class; a team never names it.
 *
 * <p>It has exactly the SDK's three public constructors, so the rewrite only swaps a class name and the call
 * looks the same to the JVM, and it overrides only {@link #nsNow()}, the one method every SDK reading of
 * "now" goes through ({@code seconds()}, {@code milliseconds()}, {@code reset()}, {@code time()}, ...). It is
 * not final: a team class that {@code extends ElapsedTime} is re-parented onto it.
 *
 * <p>Like {@link SimClockHooks}, a copy is defined inside each loader, so each sim's timers read that sim's
 * clock. Made anywhere else - {@code new SimClockElapsedTime()} in a test - it has no clock, and its
 * constructor fails with the hooks' "no sim clock" message.
 *
 * <p><b>Which sim timer to use.</b> For a timer the test builds itself, or hands to team code that accepts
 * one, use {@link org.horizon36596.simloop.sim.SimElapsedTime}: it takes the {@code FakeTimer} it reads.
 * This class is only what unchanged team code gets from the rewrite, and reads the clock of the loader that
 * defined it; a test never makes one.
 */
public class SimClockElapsedTime extends ElapsedTime {

    /** As {@link ElapsedTime#ElapsedTime()}: starts now, resolution seconds. */
    public SimClockElapsedTime() {
        super();
    }

    /**
     * As {@link ElapsedTime#ElapsedTime(long)}.
     *
     * @param startTime the start, in nanoseconds on this clock's scale (a {@code System.nanoTime()} reading,
     *                  which in rewritten code is sim nanoseconds)
     */
    public SimClockElapsedTime(long startTime) {
        super(startTime);
    }

    /**
     * As {@link ElapsedTime#ElapsedTime(ElapsedTime.Resolution)}: starts now.
     *
     * @param resolution the unit {@code time()} reports in
     */
    public SimClockElapsedTime(Resolution resolution) {
        super(resolution);
    }

    /** {@return the sim clock's reading, in nanoseconds}: what every SDK method of this timer reads. */
    @Override
    protected long nsNow() {
        return SimClockHooks.nanoTime();
    }
}
