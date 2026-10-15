package org.horizon36596.simloop.fakehardware;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareDevice;

/**
 * Fake {@link AnalogInput} returning a settable voltage, so the unchanged SolversLib
 * {@code AbsoluteAnalogEncoder} wrapper (which reads {@code getVoltage()} / {@code getMaxVoltage()})
 * resolves headless. {@code AnalogInput} is a concrete SDK class, so we subclass it and pass a
 * {@code null} controller — EVERY inherited method that would touch the controller (the reads plus the
 * {@code HardwareDevice} metadata methods PsiKit logs, e.g. {@code getConnectionInfo}) is overridden, so
 * the null controller is never dereferenced. Deterministic; voltage settable.
 *
 * <p>A real analog input can only ever read between 0 V and its range's full scale, so a voltage set here
 * is clipped into {@code [0, maxVoltage]} the way the pin itself would saturate, and a non-finite voltage is
 * rejected as the caller bug it is.
 */
public class FakeAnalogInput extends AnalogInput implements FakeDevice {

    private double voltage;
    private final double maxVoltage;

    /** Creates an input reading 0.0 V on a 3.3 V range, the common FTC analog sensor wiring. */
    public FakeAnalogInput() {
        this(0.0, 3.3);
    }

    /**
     * Creates an input reading a chosen voltage on a chosen range.
     *
     * @param voltage    the voltage this input reports, in volts; clipped into {@code [0, maxVoltage]}
     * @param maxVoltage the full-scale voltage of the range, in volts; must be finite and positive
     * @throws IllegalArgumentException if {@code maxVoltage} is not finite and positive, or {@code voltage}
     *                                  is not finite
     */
    public FakeAnalogInput(double voltage, double maxVoltage) {
        super(null, 0);
        if (!(maxVoltage > 0) || Double.isInfinite(maxVoltage)) {
            throw new IllegalArgumentException("maxVoltage must be finite and positive, got " + maxVoltage);
        }
        this.maxVoltage = maxVoltage;
        setVoltage(voltage);
    }

    /**
     * Sets the voltage this input reports from now on, clipped into {@code [0, maxVoltage]} as a real pin
     * saturates at the ends of its range.
     *
     * @param voltage the reading, in volts
     * @throws IllegalArgumentException if {@code voltage} is NaN or infinite, which no wire can carry
     */
    public void setVoltage(double voltage) {
        if (!Double.isFinite(voltage)) {
            throw new IllegalArgumentException("analog voltage must be finite, got " + voltage);
        }
        this.voltage = Math.max(0.0, Math.min(maxVoltage, voltage));
    }

    @Override
    public void update(double deltaTime) {
        // Constant source; no dynamics.
    }

    /** {@return the reading, in volts} */
    @Override
    public double getVoltage() {
        return voltage;
    }

    /** {@return the full-scale voltage of the range, in volts} */
    @Override
    public double getMaxVoltage() {
        return maxVoltage;
    }

    // Controller-backed / metadata methods overridden so the null controller is never dereferenced
    // (PsiKit's HardwareMapWrapper reads getConnectionInfo/getDeviceName when wrapping devices).
    // The name and connection format are AnalogInput's own: a name read out of the SDK's resources, and the
    // controller's connection info plus "; analog port N".
    @Override public String getDeviceName() { return "Analog Input"; }
    @Override public String getConnectionInfo() { return "sim; analog port 0"; }
    @Override public HardwareDevice.Manufacturer getManufacturer() { return HardwareDevice.Manufacturer.Other; }
    /** {@code 1}, as {@code AnalogInput.getVersion} reports. */
    @Override public int getVersion() { return 1; }
    @Override public void resetDeviceConfigurationForOpMode() { }
    @Override public void close() { }
}
