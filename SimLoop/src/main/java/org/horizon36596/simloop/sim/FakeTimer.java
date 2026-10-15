package org.horizon36596.simloop.sim;

/**
 * The single, advanceable simulation clock (fakehardware-plant §6, domain R5). The harness advances
 * it by a fixed deltaTime each tick; wire it to PsiKit with {@code Logger.setTimeSource(timer::time)}
 * so log timestamps are deterministic. No wall-clock is ever read on a sim path.
 *
 * <p><b>How the time is kept.</b> Each {@link #advance(double)} adds its step to a running {@code double}
 * total. So after many ticks the reading carries ordinary floating-point rounding (one hundred steps of
 * 0.02 s read very slightly off 2.0 s), and it is exactly the same rounding on every run, which is what
 * determinism needs. It is kept this way deliberately: computing the time any other way would move every
 * timestamp already written into an RLOG.
 */
public final class FakeTimer {

    private double seconds = 0.0;

    /** Creates a clock reading 0.0 seconds. */
    public FakeTimer() {
    }

    /**
     * Current simulation time. Suitable for {@code Logger.setTimeSource}.
     *
     * @return seconds since this timer was created or last {@link #reset()}; never decreases between
     *         resets, because {@link #advance(double)} refuses a negative step
     */
    public double time() {
        return seconds;
    }

    /**
     * Advance the clock.
     *
     * @param deltaTime how far to advance, in seconds; zero or a finite positive number
     * @throws IllegalArgumentException if {@code deltaTime} is negative, NaN or infinite, any of which would
     *                                  break the promise that the clock only moves forward by real amounts
     */
    public void advance(double deltaTime) {
        if (!(deltaTime >= 0.0) || Double.isInfinite(deltaTime)) {
            throw new IllegalArgumentException(
                    "FakeTimer.advance: deltaTime must be zero or a finite positive number of seconds, got "
                            + deltaTime);
        }
        seconds += deltaTime;
    }

    /** Reset to zero (e.g. between determinism runs). */
    public void reset() {
        seconds = 0.0;
    }
}
