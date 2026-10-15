package org.firstinspires.ftc.teamcode.simclockfixture;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.function.LongSupplier;
import java.util.function.Supplier;

/** Plain static helpers that read or wait on the clock, for loader tests that call them by reflection. */
public final class ClockProbe {

    /** How many times {@link #count()} has run in this copy of the class. */
    private static int counted;

    private ClockProbe() {
    }

    /** @return System.nanoTime() */
    public static long nanoTime() {
        return System.nanoTime();
    }

    /** @return System.currentTimeMillis() */
    public static long currentTimeMillis() {
        return System.currentTimeMillis();
    }

    /** @return a new ElapsedTime, started now */
    public static ElapsedTime newTimer() {
        return new ElapsedTime();
    }

    /** @return a new team LapTimer, started now */
    public static LapTimer newLapTimer() {
        return new LapTimer();
    }

    /** @return System::nanoTime as a supplier */
    public static LongSupplier nanoClock() {
        return System::nanoTime;
    }

    /** @return ElapsedTime::new as a supplier */
    public static Supplier<ElapsedTime> timerFactory() {
        return ElapsedTime::new;
    }

    /** Sleeps 10 ms with Thread.sleep. */
    public static void pause() throws InterruptedException {
        Thread.sleep(10);
    }

    /** @return how many times this has been called in this copy of the class, this call included */
    public static int count() {
        return ++counted;
    }

    /** @return what a library outside the team packages reads for the time */
    public static long libraryNanoTime() {
        return com.example.simclockfixture.WallClockLibrary.nanoTime();
    }

    /** @return what the (vendored) SolversLib timing fixture reads for the time */
    public static long solversLibNanoTime() {
        return com.seattlesolvers.solverslib.simclockfixture.FixtureTiming.now();
    }
}
