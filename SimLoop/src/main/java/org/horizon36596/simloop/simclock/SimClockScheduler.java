package org.horizon36596.simloop.simclock;

/**
 * What a rewritten wait ({@code Thread.sleep}, {@code LinearOpMode.sleep}, {@code idle()}) asks of the
 * OpMode run that owns a {@link SimClockLoader}'s clock. The OpMode runner provides one for each run; a
 * {@link SimClockHooks} copy finds it through the loader that defined that copy.
 *
 * <p>A team's code never calls this. It exists so the hooks, which are defined inside each loader, can reach
 * the runner, which is not.
 */
public interface SimClockScheduler {

    /**
     * {@return null if {@code thread} may wait in sim time, or else a sentence saying why it may not}, to go
     * after "SimLoop cannot run Thread.sleep(500) at MyAuto.java:12 on sim time: ". Only the thread the sim
     * steps can wait: a {@code LinearOpMode}'s own thread, or an iterative OpMode's own thread while it is in
     * {@code init()}. Any other thread runs in real time beside the sim.
     *
     * @param thread the thread that wants to wait
     */
    String whyThreadCannotWait(Thread thread);

    /**
     * Holds the calling thread, which {@link #whyThreadCannotWait} allowed, for as many whole ticks as it
     * takes to cover {@code seconds} of sim time; no real time passes.
     *
     * @param seconds how long to wait, in seconds of sim time; zero or less waits no ticks
     * @return true if the wait returns normally, false if it ends as interrupted: a {@code LinearOpMode} was
     *         stopped before or during it. An iterative OpMode is not interrupted at STOP, as on the robot, so
     *         its wait returns true, cut short because sim time ends at STOP
     */
    boolean holdFor(double seconds);

    /**
     * A rewritten {@code idle()} on the calling thread, which {@link #whyThreadCannotWait} allowed. The SDK's
     * {@code idle()} is a yield that takes no time, so this ends the tick only when the same {@code idle()} -
     * the same place in the code - has already run during this tick, the rule a lifecycle check follows. So
     * {@code while (timer.seconds() < 2) idle();} lets sim time move one tick a pass, and
     * {@code sleep(50); idle();} adds no tick to the sleep.
     *
     * @param callSite the place in the team's code that called {@code idle()}, as a stack frame prints
     */
    void idleAt(String callSite);
}
