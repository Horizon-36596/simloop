package org.horizon36596.simloop.opmode;

import org.horizon36596.simloop.simclock.SimClockScheduler;

/**
 * Connects a run's {@link org.horizon36596.simloop.simclock.SimClockLoader} to the stepper driving the
 * OpMode, so that a rewritten {@code Thread.sleep}, {@code sleep(ms)} or {@code idle()} waits in sim time.
 *
 * <p>Only the OpMode's own thread can wait, because it is the one thread the sim steps: a {@code LinearOpMode}'s
 * anywhere in {@code runOpMode()}, and an iterative OpMode's only in {@code init()}, which the SDK lets block
 * (see {@link IterativeOpModeStepper}). A wait anywhere else is refused, loudly, with a reason the hook puts
 * after the call and its place in the code: an iterative OpMode's other methods must return every tick, and a
 * thread the OpMode starts runs in real time beside the sim, so there is no sim-time answer to give it. The
 * loader keeps each refusal, and the run fails with it, because a thread the OpMode started dies of it with no
 * one watching and team code can catch it.
 */
final class SimClockBridge implements SimClockScheduler {

    private final String opModeName;

    /** The stepper of a {@code LinearOpMode} run, or null. */
    private final LinearOpModeStepper linear;

    /** The stepper of an iterative OpMode run, or null. */
    private final IterativeOpModeStepper iterative;

    SimClockBridge(String opModeName, OpModeStepper stepper) {
        this.opModeName = opModeName;
        this.linear = stepper instanceof LinearOpModeStepper ? (LinearOpModeStepper) stepper : null;
        this.iterative = stepper instanceof IterativeOpModeStepper ? (IterativeOpModeStepper) stepper : null;
    }

    @Override
    public String whyThreadCannotWait(Thread thread) {
        Thread opModeThread = linear != null ? linear.opModeThread() : iterative.thread();
        if (thread != opModeThread) {
            return "it was called on thread \"" + thread.getName() + "\", and only " + opModeName + "'s own thread"
                    + " (\"" + opModeThread.getName() + "\") waits in sim time. A thread the OpMode starts"
                    + " runs in real time beside the sim, so SimLoop cannot step its waits; do that work in "
                    + (linear != null ? "runOpMode()'s loop" : "loop()") + " instead.";
        }
        if (iterative != null && !iterative.inInit()) {
            return opModeName + " is an iterative OpMode, whose init_loop(), start(), loop() and stop() must each"
                    + " return promptly (on a robot a sleep there freezes the whole loop, gamepads and all; only"
                    + " init() may wait). Wait with an ElapsedTime check across loop() calls instead, or make it a"
                    + " LinearOpMode.";
        }
        return null;
    }

    @Override
    public boolean holdFor(double seconds) {
        return linear != null ? linear.holdForSimSeconds(seconds) : iterative.holdForSimSeconds(seconds);
    }

    @Override
    public void idleAt(String callSite) {
        // idle() is LinearOpMode's; reached from an iterative OpMode's init() it is the SDK's yield, no time.
        if (linear != null) {
            linear.idleAt(callSite);
        }
    }
}
