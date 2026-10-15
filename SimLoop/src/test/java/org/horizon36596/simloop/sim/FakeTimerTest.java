package org.horizon36596.simloop.sim;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;

import org.junit.jupiter.api.Test;

/**
 * Pins {@link FakeTimer}'s promise that its reading never goes backwards: a step that is negative, NaN or
 * infinite is refused and leaves the clock where it was.
 */
class FakeTimerTest {

    @Test
    void advancesByEachStepAndResetsToZero() {
        FakeTimer timer = new FakeTimer();
        timer.advance(0.02);
        timer.advance(0.0);
        timer.advance(0.03);
        assertEquals(0.05, timer.time(), 1e-12);

        timer.reset();
        assertEquals(0.0, timer.time(), 0.0);
    }

    @Test
    void aStepThatWouldBreakMonotonicTimeIsRefused() {
        FakeTimer timer = new FakeTimer();
        timer.advance(1.0);

        assertThrows(IllegalArgumentException.class, () -> timer.advance(-0.02));
        assertThrows(IllegalArgumentException.class, () -> timer.advance(Double.NaN));
        assertThrows(IllegalArgumentException.class, () -> timer.advance(Double.POSITIVE_INFINITY));
        assertThrows(IllegalArgumentException.class, () -> timer.advance(Double.NEGATIVE_INFINITY));

        assertEquals(1.0, timer.time(), 0.0, "a refused step leaves the clock where it was");
    }
}
