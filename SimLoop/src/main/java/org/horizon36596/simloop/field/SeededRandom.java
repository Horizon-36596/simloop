package org.horizon36596.simloop.field;

/**
 * A seeded random-number generator for anything on the field that is random on a real field and must
 * still replay identically in the sim: where the contents of a tipped goal land, for example.
 *
 * <p><b>Why not {@code java.util.Random}.</b> Two reasons, both about being able to check the sim against
 * something else. This is the <b>mulberry32</b> generator, a public-domain algorithm, bit for bit the
 * common JavaScript implementation, so a sim written in JavaScript that uses it draws the same numbers for
 * the same seed and the same order of draws, and a spill can be compared ball for ball. And its whole
 * state is one {@code int} that a test can read ({@link #internalState()}), which
 * {@code java.util.Random} does not offer. There is no setter: to start from a known state, build a new
 * generator with that state as its seed.
 *
 * <p><b>The algorithm</b>, one step, all arithmetic in wrapping 32-bit integers exactly as JavaScript's
 * {@code | 0} and {@code Math.imul} do it:
 * <pre>
 *   state = state + 0x6D2B79F5
 *   t = (state ^ (state >>> 15)) * (1 | state)
 *   t = (t + (t ^ (t >>> 7)) * (61 | t)) ^ t
 *   value = unsigned(t ^ (t >>> 14)) / 2^32          // in [0, 1)
 * </pre>
 *
 * <p>Determinism (domain R5): no clock, no global state. Two generators built with the same seed return
 * the same sequence forever.
 *
 * <p><b>One {@code SeededRandom} per random source, each with its own seed.</b> Every draw advances the
 * state, so two things sharing one generator change each other's numbers: add a draw to one and every
 * later draw of the other moves. Give each independent source (each goal that spills, say) its own
 * generator, so a change to one cannot shift the other's replay.
 */
public final class SeededRandom {

    /** The golden-ratio-derived step mulberry32 adds to its state each draw. */
    private static final int STATE_INCREMENT = 0x6D2B79F5;
    /** 2^32, to turn an unsigned 32-bit value into a fraction in [0, 1). */
    private static final double TWO_TO_THE_32 = 4294967296.0;

    private int state;

    /**
     * Creates a generator that starts from {@code seed}.
     *
     * @param seed the starting state; any int is allowed, including 0 and negatives
     */
    public SeededRandom(int seed) {
        this.state = seed;
    }

    /**
     * Draw the next number and advance the state by one step.
     *
     * @return a number in [0, 1), uniformly distributed
     */
    public double nextFraction() {
        state = state + STATE_INCREMENT;                 // Java int addition wraps, like JS "| 0"
        int t = (state ^ (state >>> 15)) * (1 | state);  // Java int multiply wraps, like Math.imul
        t = (t + (t ^ (t >>> 7)) * (61 | t)) ^ t;
        int bits = t ^ (t >>> 14);
        return Integer.toUnsignedLong(bits) / TWO_TO_THE_32;
    }

    /**
     * Draw a number uniformly between two bounds.
     *
     * @param low  the smallest value that can come out
     * @param high the value it never quite reaches
     * @return {@code low + nextFraction() * (high - low)}
     */
    public double nextBetween(double low, double high) {
        return low + nextFraction() * (high - low);
    }

    /** {@return the whole internal state, the value the next draw starts from} */
    public int internalState() {
        return state;
    }
}
