package org.horizon36596.simloop.opmode;

import org.horizon36596.simloop.simclock.SimClockScheduler;

/**
 * Connects a run's {@link org.horizon36596.simloop.simclock.SimClockLoader} to the stepper driving the
 * OpMode, so that a rewritten {@code Thread.sleep}, {@code sleep(ms)} or {@code idle()} waits in sim time.
 *
 * <p>Only a {@code LinearOpMode}'s own thread can wait: it is the one thread the sim hands turns to. A wait on
 * any other thread is refused, loudly, with a reason the hook puts after the call and its place in the code:
 * an iterative OpMode's methods must return every tick, and a thread the OpMode starts runs in real time
 * beside the sim, so there is no sim-time answer to give it. The loader keeps each refusal, and the run fails
 * with it, because a thread the OpMode started dies of it with no one watching and team code can catch it.
 */
final class SimClockBridge implements SimClockScheduler {

    private final String opModeName;

    /** The stepper of a {@code LinearOpMode} run, or null for an iterative OpMode, which cannot wait. */
    private final LinearOpModeStepper linear;

    SimClockBridge(String opModeName, OpModeStepper stepper) {
        this.opModeName = opModeName;
        this.linear = stepper instanceof LinearOpModeStepper ? (LinearOpModeStepper) stepper : null;
    }

    @Override
    public String whyThreadCannotWait(Thread thread) {
        if (linear == null) {
            return opModeName + " is an iterative OpMode, whose init(), init_loop(), start(), loop() and stop() must"
                    + " each return promptly (on a robot a sleep there freezes the whole loop, gamepads and all)."
                    + " Wait with an ElapsedTime check across loop() calls instead, or make it a LinearOpMode.";
        }
        if (thread != linear.opModeThread()) {
            return "it was called on thread \"" + thread.getName() + "\", and only " + opModeName + "'s own thread"
                    + " (\"" + linear.opModeThread().getName() + "\") waits in sim time. A thread the OpMode starts"
                    + " runs in real time beside the sim, so SimLoop cannot step its waits; do that work in"
                    + " runOpMode()'s loop instead.";
        }
        return null;
    }

    @Override
    public boolean holdFor(double seconds) {
        return linear.holdForSimSeconds(seconds);
    }

    @Override
    public void idleAt(String callSite) {
        linear.idleAt(callSite);
    }
}
