package org.horizon36596.simloop.fakehardware;

import com.qualcomm.robotcore.hardware.HardwareDevice;

/**
 * Base for fake devices: supplies the boilerplate {@link HardwareDevice} metadata so concrete fakes
 * (e.g. {@link FakeMotor}) only implement their device interface + the plant {@link #update(double)}.
 * Seeded from PsiKit's test {@code FakeHardware}.
 *
 * <p>These are defaults, not claims about any one device. A fake whose real SDK device reports something
 * different — a device name read out of the SDK's resources, a reset that restores the direction — overrides
 * the method and says which SDK class it is matching.
 */
public abstract class AbstractFakeDevice implements HardwareDevice, FakeDevice {

    /**
     * Creates the device. Holds no state of its own; every subclass owns whatever it simulates.
     *
     * <p>Public rather than protected because that is what the implicit constructor this replaces was:
     * the default constructor of a {@code public} class is {@code public} (JLS 8.8.9), so narrowing it
     * would quietly remove an anonymous subclass in another package - an API break.
     */
    public AbstractFakeDevice() {
    }

    @Override public Manufacturer getManufacturer() { return Manufacturer.Other; }
    @Override public String getDeviceName() { return getClass().getSimpleName(); }
    @Override public String getConnectionInfo() { return "sim"; }

    /**
     * {@code 1}, the version every SDK device implementation this package fakes reports ({@code DcMotorImpl},
     * {@code ServoImpl}, {@code CRServoImpl}, {@code DigitalChannelImpl}, {@code AnalogInput}).
     */
    @Override public int getVersion() { return 1; }

    @Override public void resetDeviceConfigurationForOpMode() { }
    @Override public void close() { }
}
