package org.horizon36596.simloop.field;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

/**
 * {@link SeededRandom} draws exactly what the reference JavaScript mulberry32 draws. The expected values
 * were produced on 2026-09-22 by running that implementation in Node.js, so this test is the proof that a
 * spill here and a spill in a JavaScript sim using it start from the same numbers.
 */
class SeededRandomTest {

    @Test
    void seedOneDrawsTheReferenceSequence() {
        SeededRandom random = new SeededRandom(1);
        double[] reference = {0.6270739405881613, 0.002735721180215478, 0.5274470399599522,
                0.9810509674716741, 0.9683778982143849};
        for (double expected : reference) {
            assertEquals(expected, random.nextFraction(), 0.0);
        }
    }

    @Test
    void seedZeroDrawsTheReferenceSequence() {
        SeededRandom random = new SeededRandom(0);
        assertEquals(0.26642920868471265, random.nextFraction(), 0.0);
        assertEquals(0.0003297457005828619, random.nextFraction(), 0.0);
        assertEquals(0.2232720274478197, random.nextFraction(), 0.0);
    }

    @Test
    void aNegativeSeedWrapsTheWayJavaScriptDoes() {
        SeededRandom random = new SeededRandom(-123456789);
        assertEquals(0.9335711891762912, random.nextFraction(), 0.0);
        assertEquals(0.2907314267940819, random.nextFraction(), 0.0);
        assertEquals(0.865351774264127, random.nextFraction(), 0.0);
        assertEquals(1076273354, random.internalState());
    }

    @Test
    void everyDrawIsInTheHalfOpenUnitInterval() {
        SeededRandom random = new SeededRandom(42);
        for (int i = 0; i < 10000; i++) {
            double value = random.nextFraction();
            assertTrue(value >= 0.0 && value < 1.0);
        }
    }

    @Test
    void nextBetweenScalesTheDraw() {
        // Seed 1's first draw is 0.6270739405881613, so 30..62 gives 30 + 32 x that.
        assertEquals(50.06636609882116, new SeededRandom(1).nextBetween(30.0, 62.0), 1e-12);
        // And a range that straddles zero, as the spill fan (-40..+40 degrees) does.
        assertEquals(-0.5 + 0.6270739405881613, new SeededRandom(1).nextBetween(-0.5, 0.5), 1e-12);
    }
}
