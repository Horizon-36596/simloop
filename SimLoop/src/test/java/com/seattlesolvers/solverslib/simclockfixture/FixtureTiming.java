package com.seattlesolvers.solverslib.simclockfixture;

/**
 * Stands in for SolversLib's {@code Timing}, which reads System.nanoTime(): SimClockLoader rewrites the
 * {@code com.seattlesolvers.solverslib.} package by default because many teams vendor it into TeamCode.
 * Test source only.
 */
public final class FixtureTiming {

    private FixtureTiming() {
    }

    /** @return System.nanoTime(), as SolversLib's timers read it */
    public static long now() {
        return System.nanoTime();
    }
}
