package org.horizon36596.simloop.fakehardware;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.qualcomm.robotcore.hardware.HardwareDeviceHealth;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cWaitControl;

import org.junit.jupiter.api.Test;

/**
 * INVARIANT-tier — {@link FakeI2cDeviceSynchSimple} models the bus-configuration surface an I2C device
 * driver's constructor touches, and fails loudly the moment anything tries to move data, because there is
 * no peripheral behind it to answer. A device fake that silently got bytes back would be a fake returning
 * plausible-but-wrong readings.
 */
class FakeI2cDeviceSynchSimpleTest {

    @Test
    void busConfigurationIsStoredAndReadBack() {
        FakeI2cDeviceSynchSimple bus = new FakeI2cDeviceSynchSimple();
        // I2cAddr has no equals(), so addresses are compared by their 7-bit value or by identity.
        assertEquals(0, bus.getI2cAddress().get7Bit(), "a fresh bus client is at address zero");
        assertFalse(bus.isWriteCoalescingEnabled());
        assertFalse(bus.isArmed(), "never armed against real hardware in sim");

        I2cAddr address = I2cAddr.create7bit(0x30);
        bus.setI2cAddress(address);
        assertSame(address, bus.getI2cAddress());
        assertSame(address, bus.getI2cAddr(), "the two address accessor spellings are one address");
        bus.setI2cAddr(I2cAddr.create7bit(0x31));
        assertEquals(0x31, bus.getI2cAddress().get7Bit());

        bus.enableWriteCoalescing(true);
        assertTrue(bus.isWriteCoalescingEnabled());

        bus.setLogging(true);
        bus.setLoggingTag("octo");
        assertTrue(bus.getLogging());
        assertEquals("octo", bus.getLoggingTag());

        bus.setUserConfiguredName("odometry");
        assertEquals("odometry", bus.getUserConfiguredName());

        bus.setHealthStatus(HardwareDeviceHealth.HealthStatus.UNHEALTHY);
        assertEquals(HardwareDeviceHealth.HealthStatus.UNHEALTHY, bus.getHealthStatus());
    }

    @Test
    void everyDataTransferFailsLoudly() {
        FakeI2cDeviceSynchSimple bus = new FakeI2cDeviceSynchSimple();
        assertThrows(UnsupportedOperationException.class, bus::read8);
        assertThrows(UnsupportedOperationException.class, () -> bus.read8(0x10));
        assertThrows(UnsupportedOperationException.class, () -> bus.read(4));
        assertThrows(UnsupportedOperationException.class, () -> bus.read(0x10, 4));
        assertThrows(UnsupportedOperationException.class, () -> bus.readTimeStamped(4));
        assertThrows(UnsupportedOperationException.class, () -> bus.readTimeStamped(0x10, 4));
        assertThrows(UnsupportedOperationException.class, () -> bus.write8(1));
        assertThrows(UnsupportedOperationException.class, () -> bus.write8(0x10, 1));
        assertThrows(UnsupportedOperationException.class, () -> bus.write8(1, I2cWaitControl.NONE));
        assertThrows(UnsupportedOperationException.class, () -> bus.write8(0x10, 1, I2cWaitControl.NONE));
        assertThrows(UnsupportedOperationException.class, () -> bus.write(new byte[] {1}));
        assertThrows(UnsupportedOperationException.class, () -> bus.write(0x10, new byte[] {1}));
        assertThrows(UnsupportedOperationException.class, () -> bus.write(new byte[] {1}, I2cWaitControl.NONE));
        assertThrows(UnsupportedOperationException.class,
                () -> bus.write(0x10, new byte[] {1}, I2cWaitControl.NONE));
        assertThrows(UnsupportedOperationException.class, () -> bus.waitForWriteCompletions(I2cWaitControl.NONE));
    }
}
