package com.example.simclockfixture;

/**
 * A library outside every team package, so SimClockLoader loads it from the parent untouched: it keeps
 * reading the real clock. Test source only.
 */
public final class WallClockLibrary {

    private WallClockLibrary() {
    }

    /** @return System.nanoTime(), the real one */
    public static long nanoTime() {
        return System.nanoTime();
    }

    /**
     * Waits with the real Thread.sleep, as a vendored driver might.
     *
     * @return "slept", or "InterruptedException" if the sleep was interrupted
     */
    public static String pause(long ms) {
        try {
            Thread.sleep(ms);
            return "slept";
        } catch (InterruptedException e) {
            return "InterruptedException";
        }
    }
}
