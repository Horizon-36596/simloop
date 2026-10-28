package com.pedropathing.simclockfixture;

/**
 * Stands in for a Pedro Pathing localizer that sleeps in its constructor while the odometry computer
 * calibrates (as Pedro's {@code PinpointLocalizer} does, 500 ms), wrapping the {@code InterruptedException}
 * the way that library does. Teams build it in an iterative OpMode's {@code init()}. Test sources only.
 */
public final class PedroStyleLocalizer {

    /** How long the constructor sleeps, in milliseconds. */
    public static final long CALIBRATION_MILLIS = 500;

    /** Sleeps {@link #CALIBRATION_MILLIS}. */
    public PedroStyleLocalizer() {
        try {
            Thread.sleep(CALIBRATION_MILLIS);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }
}
