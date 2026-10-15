package com.pedropathing.simclockfixture;

/**
 * Stands in for a Pedro Pathing class that reads the clock inside the library (as Pedro's own
 * {@code com.pedropathing.utils.Timer} does), so SimLoop's tests can check how a run treats a known
 * clock-reading library without depending on Pedro. Test sources only.
 */
public final class PedroStyleTimer {

    private final long start = System.nanoTime();

    /** {@return seconds since this timer was made, by whichever clock this copy of the class reads} */
    public double seconds() {
        return (System.nanoTime() - start) / 1e9;
    }
}
