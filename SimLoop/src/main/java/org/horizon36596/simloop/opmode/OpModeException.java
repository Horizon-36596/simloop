package org.horizon36596.simloop.opmode;

/**
 * An OpMode run could not be completed: the OpMode's own code threw, or it stopped cooperating with the
 * sim (it ran without ever reaching a lifecycle check, or it did not return after STOP). The message says
 * which, and names the phase and tick; when the OpMode threw, its exception is the {@link #getCause()}.
 *
 * <p>This is always a failure of the run, never something to catch and carry on from: a test that gets one
 * should fail, and the runner has already asked the OpMode to stop.
 */
public final class OpModeException extends RuntimeException {

    private static final long serialVersionUID = 1L;

    /** The phase the run was in. */
    private final OpModePhase phase;

    /** The tick of the run, counting from 0 at the INIT tick. */
    private final int tick;

    /**
     * Creates the exception.
     *
     * @param message what went wrong, in a sentence a student can act on
     * @param phase   the phase the run was in
     * @param tick    the tick of the run, counting from 0 at the INIT tick
     * @param cause   what the OpMode threw, or null when it threw nothing
     */
    OpModeException(String message, OpModePhase phase, int tick, Throwable cause) {
        super(message, cause);
        this.phase = phase;
        this.tick = tick;
    }

    /** {@return the phase the run was in when it failed} */
    public OpModePhase phase() {
        return phase;
    }

    /** {@return the tick of the run it failed on, counting from 0 at the INIT tick} */
    public int tick() {
        return tick;
    }
}
