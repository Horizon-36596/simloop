package org.horizon36596.simloop.sim.live;

/**
 * Where a session's gamepad input comes from, one loop at a time: a person at a keyboard
 * ({@link KeyboardGamepadSource}), a recorded track ({@link GamepadInputTrack#replay()}), or a test's
 * script.
 *
 * <p>This is the only thing that differs between a live session and its headless replay. Everything
 * after it - the OpMode, the plants, the log - is the same deterministic stepper either way, which is
 * why a replay of a recorded session reproduces it exactly.
 */
public interface GamepadInputSource {

    /**
     * The gamepads for one loop, or {@code null} when the session is over.
     *
     * <p>Called once per loop, before that loop runs, with ticks 0, 1, 2, ... in order. After it has
     * returned {@code null} it is not called again.
     *
     * @param tickIndex which loop is about to run, counting from 0
     * @return that loop's gamepads, whose {@code tick} is {@code tickIndex}; or {@code null} to end
     */
    GamepadInputTrack.Frame next(int tickIndex);
}
