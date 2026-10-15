package org.horizon36596.simloop.sim.live;

import org.horizon36596.simloop.sim.ScenarioRunner;

/**
 * Runs a scenario for as long as a {@link GamepadInputSource} has input, and hands each tick's gamepads to
 * the scenario body. This is how a live session, and the replay of one, plug a source into
 * {@code ScenarioRunner.runUntil}:
 *
 * <pre>{@code
 * GamepadInputGate input = new GamepadInputGate(source);
 * ScenarioRunner.runUntil(name, config, timer, dt, rlog, input, pacer, receivers, deltaTime -> {
 *     input.current().gamepad1.applyTo(opMode.gamepad1);
 *     ...
 * });
 * }</pre>
 *
 * <p>Before each tick the runner asks this gate whether the tick runs; the gate asks the source for that
 * tick's frame, and the tick runs if there is one. The body then reads the same frame back with
 * {@link #current()}. The source is asked exactly once per tick, which matters when it is a keyboard (the
 * keys are sampled once) or a recorder (the line is written once).
 */
public final class GamepadInputGate implements ScenarioRunner.TickGate {

    private final GamepadInputSource source;
    private GamepadInputTrack.Frame current;

    /**
     * A gate that runs a tick whenever the source has a frame for it.
     *
     * @param source where each tick's gamepads come from
     */
    public GamepadInputGate(GamepadInputSource source) {
        this.source = source;
    }

    @Override
    public boolean shouldRunTick(int tickIndex) {
        current = source.next(tickIndex);
        return current != null;
    }

    /**
     * {@return the gamepads for the tick that is running now}
     *
     * @throws IllegalStateException if called outside a tick, before the first or after the last
     */
    public GamepadInputTrack.Frame current() {
        if (current == null) {
            throw new IllegalStateException("no tick is running: current() is for the scenario body, which runs"
                    + " only after this gate has let a tick start");
        }
        return current;
    }
}
