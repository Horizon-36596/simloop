package org.horizon36596.simloop.opmode;

/**
 * Runs one kind of OpMode one tick at a time. {@link OpModeRunner} owns the clock, the gamepads and the
 * world; a stepper owns only the question "what does this OpMode's code do on this tick".
 */
interface OpModeStepper {

    /**
     * Runs the OpMode's code for one tick and returns once it has finished that tick's work.
     *
     * @throws OpModeException if the OpMode threw, or stopped cooperating with the sim
     */
    void runTick(OpModePhase phase, int tickInPhase, int tick);

    /** Called, on the OpMode's own thread, when its code calls {@code requestOpModeStop()}. */
    void opModeAskedToStop();

    /** {@return true once the OpMode has finished: it returned, asked to stop, or was stopped} */
    boolean hasEnded();

    /** {@return the thread the OpMode's code runs on, which is not one the OpMode started itself} */
    Thread thread();

    /**
     * Makes sure nothing of the OpMode is left running. Safe to call more than once, and after a failure.
     * An interrupt of the calling thread does not cut its bounded wait for the OpMode thread short, and is
     * still set when this returns.
     *
     * @throws OpModeException if the OpMode could not be made to stop
     */
    void shutDown();
}
