package org.horizon36596.simloop.opmode;

import com.qualcomm.robotcore.hardware.Gamepad;

/**
 * What the drivers do with their gamepads, tick by tick. The runner calls this at the start of every tick,
 * before any OpMode code runs, and then hands the OpMode a copy of both gamepads the way the SDK delivers
 * Driver Station data - so edge detection such as {@code gamepad1.aWasPressed()} behaves as on the robot.
 *
 * <p>The two gamepads passed in are the same objects on every tick, so a button stays held until the script
 * releases it, exactly like a real one. Write only fields ({@code gamepad1.left_stick_y = -1f},
 * {@code gamepad1.a = true}); do not keep references to them outside the call. A button can be pressed by either
 * of its names ({@code a} or {@code cross}, {@code y} or {@code triangle}, {@code back} or {@code share}, ...): the
 * OpMode sees both names pressed, as it would on the robot.
 *
 * <p>A script must be a pure function of its arguments (and of state it owns) for a run to be deterministic.
 */
@FunctionalInterface
public interface GamepadScript {

    /** A script that never touches either gamepad: both stay at rest for the whole run. */
    GamepadScript NONE = (phase, tickInPhase, gamepad1, gamepad2) -> { };

    /**
     * Sets the drivers' inputs for one tick.
     *
     * @param phase       the phase this tick belongs to
     * @param tickInPhase the tick's index within {@code phase}, counting from 0
     * @param gamepad1    the driver's gamepad, as the Driver Station holds it
     * @param gamepad2    the operator's gamepad, as the Driver Station holds it
     */
    void apply(OpModePhase phase, int tickInPhase, Gamepad gamepad1, Gamepad gamepad2);
}
