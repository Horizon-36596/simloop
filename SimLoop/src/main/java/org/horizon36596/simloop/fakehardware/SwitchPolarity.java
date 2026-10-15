package org.horizon36596.simloop.fakehardware;

/**
 * How a switch is wired to a digital pin: which line level means "pressed". This is what lets the sim
 * drive a {@link FakeDigitalChannel} from a physical fact ("the limit switch is pressed") without baking
 * one robot's wiring into the channel, which reports a raw line level and nothing else.
 *
 * <p>Many teams read a REV Touch Sensor or a limit switch as a plain {@code DigitalChannel} rather than a
 * {@code TouchSensor}. The channel then hands robot code the raw line, and the robot code does the
 * inversion itself ({@code boolean pressed = !channel.getState();}). The sim has to put the same raw line
 * on the wire, so it needs to know the wiring:
 *
 * <pre>{@code
 * FakeDigitalChannel limit = new FakeDigitalChannel();
 * hardwareMap.register("liftLimit", limit);
 * // each tick, from whatever decides the switch is pressed:
 * SwitchPolarity.PRESSED_IS_LOW.drive(limit, liftAtBottom);
 * }</pre>
 *
 * <p>Deterministic and stateless.
 */
public enum SwitchPolarity {

    /**
     * Pressed pulls the line LOW; released reads HIGH. SPEC: this is the REV Touch Sensor, whose SDK 12.0
     * driver reports {@code isPressed()} as {@code !getDigitalChannelState(port)}, and any switch wired
     * between the pin and ground with the hub's pull-up holding the line HIGH.
     */
    PRESSED_IS_LOW,

    /** Pressed drives the line HIGH; released reads LOW. A switch wired to the supply with a pull-down. */
    PRESSED_IS_HIGH;

    /**
     * The line level a switch wired this way puts on the pin.
     *
     * @param pressed true when the switch is pushed in
     * @return the raw level, {@code true} for HIGH, as {@link FakeDigitalChannel#setSimulatedInputState}
     *         takes it
     */
    public boolean lineLevelWhen(boolean pressed) {
        return this == PRESSED_IS_HIGH ? pressed : !pressed;
    }

    /**
     * Reads a raw line level back as pressed or released - the inversion robot code does for this wiring.
     *
     * @param lineLevel the raw level, {@code true} for HIGH, as {@code DigitalChannel.getState()} reports it
     * @return true when that level means the switch is pushed in
     */
    public boolean isPressedAt(boolean lineLevel) {
        return this == PRESSED_IS_HIGH ? lineLevel : !lineLevel;
    }

    /**
     * Puts the level for {@code pressed} on {@code channel}'s wire, through the sim-only
     * {@link FakeDigitalChannel#setSimulatedInputState(boolean)}. Robot code reads it with
     * {@code getState()} while the channel is an input, exactly as on the robot.
     *
     * @param channel the fake the switch is wired to
     * @param pressed true when the switch is pushed in
     * @throws IllegalArgumentException if {@code channel} is null
     */
    public void drive(FakeDigitalChannel channel, boolean pressed) {
        if (channel == null) {
            throw new IllegalArgumentException("a switch needs a digital channel to drive");
        }
        channel.setSimulatedInputState(lineLevelWhen(pressed));
    }
}
