package org.horizon36596.simloop.fakehardware;

import com.qualcomm.robotcore.hardware.TouchSensor;

import java.util.function.BooleanSupplier;

/**
 * Fake {@link TouchSensor}: a REV Touch Sensor (or any button the robot configuration calls a "REV Touch
 * Sensor") that the sim presses and releases. Unchanged robot code resolves it with
 * {@code hardwareMap.get(TouchSensor.class, name)} and reads {@link #isPressed()} and {@link #getValue()}
 * exactly as it does on the real robot.
 *
 * <h2>Two ways to drive it</h2>
 * <ul>
 *   <li><b>Scripted:</b> {@link #setPressed(boolean)} takes effect at once, the way
 *       {@link FakeDigitalChannel#setSimulatedInputState(boolean)} does.</li>
 *   <li><b>From a source:</b> {@link #setPressedSource(BooleanSupplier)} hands the fake something to ask,
 *       for example {@code () -> tracker.getHeldCount() > 0} for a switch a held piece rests on, or a
 *       {@code org.horizon36596.simloop.sensor.ContactSwitch} that works out whether a bumper switch is
 *       against a wall. The source is asked once per {@link #update(double)}, so what robot code reads
 *       changes on a harness tick and never in the middle of one, which keeps every getter a pure read.</li>
 * </ul>
 * Whichever was called last wins: {@code setPressed} clears any source, and a new source replaces a
 * scripted value from the next tick.
 *
 * <h2>Fidelity notes, read out of the SDK 12.0 {@code RevTouchSensor} bytecode</h2>
 * <ul>
 *   <li><b>Pressed is LOW on the wire.</b> {@code RevTouchSensor.isPressed()} returns
 *       {@code !getDigitalChannelState(port)}. That inversion lives inside the SDK driver, so a robot that
 *       reads the sensor as a {@code TouchSensor} never sees it; a robot that configures the same port as a
 *       plain "Digital Device" reads the raw line through a {@link FakeDigitalChannel} instead and must
 *       invert it itself. {@link SwitchPolarity} is how the sim drives that raw line.</li>
 *   <li><b>{@link #getValue()} is 1.0 when pressed and 0.0 when not</b>, nothing in between.</li>
 *   <li><b>Metadata</b> is the driver's: manufacturer {@code Lynx}, device name {@code "REV Touch Sensor"},
 *       version 1, and connection info in its {@code "<controller>; digital channel <port>"} shape.</li>
 * </ul>
 *
 * <p>Deterministic: no clock, no randomness. A fresh sensor is released.
 */
public class FakeTouchSensor extends AbstractFakeDevice implements TouchSensor {

    private boolean pressed;
    private BooleanSupplier pressedSource;

    /** Creates a released sensor with no source; press it with {@link #setPressed(boolean)}. */
    public FakeTouchSensor() {
    }

    /**
     * Press or release the sensor from now on, as the physical world would. Clears any source set with
     * {@link #setPressedSource(BooleanSupplier)}. <b>Sim-only</b>: a real {@code TouchSensor} has no such
     * method, so robot code cannot fake its own reading.
     *
     * @param pressed true when the button is pushed in
     */
    public void setPressed(boolean pressed) {
        this.pressedSource = null;
        this.pressed = pressed;
    }

    /**
     * Ask {@code source} whether the sensor is pressed on every {@link #update(double)} from now on.
     * Replaces any earlier source and, from the next tick, any scripted value. Pass {@code null} to stop
     * asking; the sensor then keeps the last value the source gave.
     *
     * @param source what decides whether the button is pushed in, asked once per harness tick; may be null
     */
    public void setPressedSource(BooleanSupplier source) {
        this.pressedSource = source;
    }

    /**
     * Samples the source set with {@link #setPressedSource(BooleanSupplier)}, if there is one. A touch
     * sensor has no dynamics of its own.
     *
     * @param deltaTime how far to advance, in seconds; unused
     */
    @Override
    public void update(double deltaTime) {
        if (pressedSource != null) {
            pressed = pressedSource.getAsBoolean();
        }
    }

    // ---- TouchSensor ----

    /** {@return 1.0 while pressed and 0.0 while released, as {@code RevTouchSensor} reports; unitless} */
    @Override
    public double getValue() {
        return pressed ? 1.0 : 0.0;
    }

    /** {@return true while the button is pushed in} */
    @Override
    public boolean isPressed() {
        return pressed;
    }

    // ---- HardwareDevice: matched to RevTouchSensor rather than to AbstractFakeDevice's defaults ----

    /** {@return {@code Lynx}, what {@code RevTouchSensor} reports: it is read through a REV hub's digital port} */
    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.Lynx;
    }

    /** {@return {@code "REV Touch Sensor"}, the string {@code RevTouchSensor.getDeviceName} returns} */
    @Override
    public String getDeviceName() {
        return "REV Touch Sensor";
    }

    /** {@return the controller's connection info plus {@code "; digital channel N"}, in {@code RevTouchSensor}'s format} */
    @Override
    public String getConnectionInfo() {
        return "sim; digital channel 0";
    }

    /** {@return {@code 1}, as {@code RevTouchSensor} reports} */
    @Override
    public int getVersion() {
        return 1;
    }
}
