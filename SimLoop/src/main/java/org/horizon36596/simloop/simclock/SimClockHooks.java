package org.horizon36596.simloop.simclock;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.concurrent.TimeUnit;

/**
 * What a team's rewritten clock calls call instead. {@link ClassFileRewriter} points
 * {@code System.nanoTime()} at {@link #nanoTime()}, {@code Thread.sleep(ms)} at {@link #threadSleep(long)},
 * a {@code LinearOpMode}'s {@code sleep(ms)} at {@link #opModeSleep(LinearOpMode, long)}, and so on; a team
 * never calls these by name.
 *
 * <h2>One copy per sim</h2>
 * A {@link SimClockLoader} defines its own copy of this class instead of borrowing the test's, so the copy
 * knows which loader - and so which sim clock - it belongs to. Two sims in one JVM never share a clock.
 * Nothing here reads a wall clock, and there is no fallback to one: a copy that no loader defined (this
 * class used directly from a test) has no clock, and every method throws an {@link IllegalStateException}
 * saying so.
 */
public final class SimClockHooks {

    /**
     * What {@code System.currentTimeMillis()} reads when the sim clock reads zero: 2026-01-01T00:00:00Z, in
     * milliseconds since 1970. A fixed epoch rather than today's date, so a file name or log line made from
     * the time is the same on every run.
     */
    public static final long SIM_EPOCH_MILLIS = 1_767_225_600_000L;

    private static final long NANOS_PER_MILLI = 1_000_000L;

    /** The loader that defined this copy, or null for a copy no SimClockLoader defined. */
    private static final SimClockLoader LOADER = definingLoader();

    private SimClockHooks() {
    }

    private static SimClockLoader definingLoader() {
        ClassLoader loader = SimClockHooks.class.getClassLoader();
        return loader instanceof SimClockLoader ? (SimClockLoader) loader : null;
    }

    /** This copy's loader; throws if no SimClockLoader defined this copy. */
    private static SimClockLoader loader() {
        if (LOADER == null) {
            throw new IllegalStateException("SimLoop has no sim clock here: this SimClockHooks was loaded by "
                    + SimClockHooks.class.getClassLoader() + ", not by a SimClockLoader, so there is no sim"
                    + " clock for it to read and it will not fall back to the real one. The sim-clock hooks are"
                    + " only for team code a SimClockLoader rewrote: run the OpMode with"
                    + " OpModeRunner.forTeamClass(...).");
        }
        return LOADER;
    }

    /**
     * Replaces {@code System.nanoTime()}.
     *
     * @return the sim clock's reading, in nanoseconds (rounded to the nearest one)
     * @throws IllegalStateException if no SimClockLoader defined this copy
     * @throws Error                 on the thread {@link SimClockLoader#abandon} marked, to end it (as every
     *                               hook that waits does there)
     */
    public static long nanoTime() {
        SimClockLoader loader = loader();
        endIfAbandoned(loader);
        return Math.round(loader.clock().time() * 1e9);
    }

    /**
     * Ends the thread {@link SimClockLoader#abandon} marked, at whichever hook it reaches next. An Error, not
     * an exception, so a team's {@code catch (Exception e)} in the loop it is stuck in cannot swallow it.
     */
    private static void endIfAbandoned(SimClockLoader loader) {
        if (Thread.currentThread() == loader.abandonedThread()) {
            throw new Error("SimLoop ends this thread here: its OpMode run is over and gave up on it, and the sim"
                    + " clock will never move again, so a timer it waits on, or an idle() loop, would spin"
                    + " forever.");
        }
    }

    /**
     * Replaces {@code System.currentTimeMillis()}.
     *
     * @return {@link #SIM_EPOCH_MILLIS} plus the sim clock's reading, in whole milliseconds
     * @throws IllegalStateException if no SimClockLoader defined this copy
     */
    public static long currentTimeMillis() {
        return SIM_EPOCH_MILLIS + Math.floorDiv(nanoTime(), NANOS_PER_MILLI);
    }

    /**
     * Replaces Android's {@code SystemClock.uptimeMillis()}.
     *
     * @return the sim clock's reading, in whole milliseconds (zero while the OpMode is being built)
     * @throws IllegalStateException if no SimClockLoader defined this copy
     */
    public static long uptimeMillis() {
        return Math.floorDiv(nanoTime(), NANOS_PER_MILLI);
    }

    /**
     * Replaces Android's {@code SystemClock.elapsedRealtime()}: the same reading as {@link #uptimeMillis()},
     * since a sim never sleeps.
     *
     * @return the sim clock's reading, in whole milliseconds
     * @throws IllegalStateException if no SimClockLoader defined this copy
     */
    public static long elapsedRealtime() {
        return uptimeMillis();
    }

    /**
     * Replaces Android's {@code SystemClock.elapsedRealtimeNanos()}.
     *
     * @return the sim clock's reading, in nanoseconds, as {@link #nanoTime()}
     * @throws IllegalStateException if no SimClockLoader defined this copy
     */
    public static long elapsedRealtimeNanos() {
        return nanoTime();
    }

    /**
     * Replaces {@code Thread.sleep(long)}: holds the OpMode's thread for enough ticks to cover the time.
     *
     * @param millis how long, in milliseconds of sim time
     * @throws IllegalArgumentException if {@code millis} is negative, as the JDK's does
     * @throws InterruptedException     if the OpMode is stopped before or during the wait, as the SDK's
     *                                  interrupt at STOP makes the real one throw
     * @throws IllegalStateException    if called on a thread the sim does not step, or with no run going
     */
    public static void threadSleep(long millis) throws InterruptedException {
        if (millis < 0) {
            throw new IllegalArgumentException("timeout value is negative");
        }
        if (millis == 0) {
            return; // a yield in all but name, harmless on any thread and in an iterative OpMode
        }
        String call = "Thread.sleep(" + millis + ")";
        if (!schedulerFor(call).holdFor(millis / 1000.0)) {
            throw new InterruptedException("sleep interrupted: the OpMode was stopped");
        }
    }

    /**
     * Replaces {@code Thread.sleep(long, int)}.
     *
     * @param millis how long, in milliseconds of sim time
     * @param nanos  additional nanoseconds, 0 to 999999
     * @throws IllegalArgumentException if either is out of range, as the JDK's does
     * @throws InterruptedException     if the OpMode is stopped before or during the wait
     * @throws IllegalStateException    if called on a thread the sim does not step, or with no run going
     */
    public static void threadSleep(long millis, int nanos) throws InterruptedException {
        if (millis < 0) {
            throw new IllegalArgumentException("timeout value is negative");
        }
        if (nanos < 0 || nanos > 999_999) {
            throw new IllegalArgumentException("nanosecond timeout value out of range");
        }
        if (millis == 0 && nanos == 0) {
            return;
        }
        String call = "Thread.sleep(" + millis + ", " + nanos + ")";
        if (!schedulerFor(call).holdFor(millis / 1000.0 + nanos / 1e9)) {
            throw new InterruptedException("sleep interrupted: the OpMode was stopped");
        }
    }

    /**
     * Replaces {@code sleep(ms)} called on a {@code LinearOpMode}, whatever the duration's source: a
     * constant, a variable or a sum. Like the SDK's, a stop ends it early and quietly.
     *
     * @param opMode the OpMode {@code sleep} was called on (the receiver of the original call)
     * @param millis how long, in milliseconds of sim time
     * @throws IllegalArgumentException if {@code millis} is negative, as the SDK's (which hands it to
     *                                  {@code Thread.sleep}) does
     * @throws IllegalStateException    if called on a thread the sim does not step, or with no run going
     */
    public static void opModeSleep(LinearOpMode opMode, long millis) {
        if (millis < 0) {
            throw new IllegalArgumentException("timeout value is negative");
        }
        if (millis == 0) {
            return;
        }
        schedulerFor("sleep(" + millis + ")").holdFor(millis / 1000.0);
    }

    /**
     * Replaces {@code unit.sleep(duration)} on a {@link TimeUnit}: holds the OpMode's thread for enough ticks
     * to cover the time, as {@link #threadSleep(long)} does.
     *
     * @param unit     the unit the call was made on (the receiver of the original call)
     * @param duration how long, in {@code unit}s of sim time; zero or less returns at once, as the JDK's does
     * @throws InterruptedException  if the OpMode is stopped before or during the wait
     * @throws IllegalStateException if called on a thread the sim does not step, or with no run going
     * @throws NullPointerException  if {@code unit} is null, as calling a method on it would
     */
    public static void timeUnitSleep(TimeUnit unit, long duration) throws InterruptedException {
        long nanos = unit.toNanos(duration);
        if (duration <= 0) {
            return;
        }
        String call = "TimeUnit." + unit + ".sleep(" + duration + ")";
        if (!schedulerFor(call).holdFor(nanos / 1e9)) {
            throw new InterruptedException("sleep interrupted: the OpMode was stopped");
        }
    }

    /**
     * Replaces Android's {@code SystemClock.sleep(ms)}: holds the OpMode's thread for enough ticks to cover the
     * time. Like the real one it never throws {@code InterruptedException}; unlike the real one, which sleeps
     * on through an interrupt, it returns at STOP, because sim time no longer moves then.
     *
     * @param millis how long, in milliseconds of sim time; zero or less returns at once, as Android's does
     * @throws IllegalStateException if called on a thread the sim does not step, or with no run going
     */
    public static void systemClockSleep(long millis) {
        if (millis <= 0) {
            return;
        }
        schedulerFor("SystemClock.sleep(" + millis + ")").holdFor(millis / 1000.0);
    }

    /**
     * Replaces {@code idle()} called on a {@code LinearOpMode}. The SDK's yields the thread and takes no time;
     * here the tick ends when this same {@code idle()} has already run during it (see
     * {@link SimClockScheduler#idleAt}), so a wait loop with {@code idle()} in it, such as
     * {@code while (timer.seconds() < 2) idle();}, lets sim time move, and {@code sleep(50); idle();} waits
     * exactly as long as the sleep. On a thread the sim does not step it does nothing, as a yield would.
     *
     * @param opMode the OpMode {@code idle} was called on
     * @throws IllegalStateException if no run is going (while the OpMode is built, or after its run ended), as
     *                               {@code sleep} does, so a thread left looping on {@code idle()} ends
     */
    public static void opModeIdle(LinearOpMode opMode) {
        SimClockScheduler scheduler = schedulerOrRefuse(loader(), "idle()");
        if (scheduler.whyThreadCannotWait(Thread.currentThread()) == null) {
            scheduler.idleAt(callSite());
        }
    }

    /**
     * {@return the scheduler of the run in progress, once it has said the calling thread may wait}
     *
     * @param call the call being made, as the team wrote it, for the message
     * @throws IllegalStateException if no run is going, or this thread may not wait
     */
    private static SimClockScheduler schedulerFor(String call) {
        SimClockLoader loader = loader();
        SimClockScheduler scheduler = schedulerOrRefuse(loader, call);
        String why = scheduler.whyThreadCannotWait(Thread.currentThread());
        if (why != null) {
            throw refusal(loader, call, why);
        }
        return scheduler;
    }

    /** {@return the scheduler of the run in progress; with none, the refusal saying so is thrown} */
    private static SimClockScheduler schedulerOrRefuse(SimClockLoader loader, String call) {
        endIfAbandoned(loader);
        SimClockScheduler scheduler = loader.scheduler();
        if (scheduler == null) {
            throw refusal(loader, call, "no OpMode run is using this sim clock right now (the call came while the"
                    + " OpMode was being built, or after its run ended), so there is nothing to wait for.");
        }
        return scheduler;
    }

    /** {@return the refusal of {@code call} for {@code why}, already recorded on the loader for the run} */
    private static IllegalStateException refusal(SimClockLoader loader, String call, String why) {
        IllegalStateException refusal = new IllegalStateException("SimLoop cannot run " + call + " at "
                + callSite() + " on sim time: " + why);
        loader.refused(refusal);
        return refusal;
    }

    /** {@return the frame of the team's code that made the call: the first one that is not this class} */
    private static String callSite() {
        for (StackTraceElement frame : new Throwable().getStackTrace()) {
            if (!frame.getClassName().equals(SimClockHooks.class.getName())) {
                return frame.toString();
            }
        }
        return "an unknown place";
    }
}
