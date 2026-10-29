package org.horizon36596.simloop.opmode;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertSame;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.horizon36596.simloop.fakehardware.FakeHardwareMap;
import org.horizon36596.simloop.fakehardware.FakeVoltageSensor;
import org.junit.jupiter.api.Test;

/**
 * Pins the battery the runner adds: robot code that compensates for the battery reads
 * {@code hardwareMap.voltageSensor.iterator().next()}, which a real robot's map always answers (its hubs' voltage
 * sensors) and an empty fake map used to answer with {@code NoSuchElementException} at INIT.
 *
 * <p>Implementation tier: the 12 V and the name are the runner's documented defaults.
 */
class DefaultBatteryTest {

    /** Reads the battery in init() the way voltage-compensated robot code does. */
    static class ReadsTheBattery extends OpMode {
        VoltageSensor battery;
        double volts = Double.NaN;

        @Override
        public void init() {
            battery = hardwareMap.voltageSensor.iterator().next();
            volts = battery.getVoltage();
        }

        @Override
        public void loop() {
        }
    }

    @Test
    void aMapWithNoVoltageSensorGetsATwelveVoltBatteryNamedForTheControlHub() {
        FakeHardwareMap hardwareMap = new FakeHardwareMap();
        ReadsTheBattery opMode = new ReadsTheBattery();

        new OpModeRunner(opMode, hardwareMap).initLoopTicks(1).activeTicks(1).run();

        assertEquals(12.0, opMode.volts, 0.0, "volts");
        assertEquals(1, hardwareMap.voltageSensor.size(), "one battery, the runner's");
        assertSame(opMode.battery, hardwareMap.voltageSensor.get(OpModeRunner.DEFAULT_VOLTAGE_SENSOR_NAME),
                "under the Control Hub's name");
    }

    @Test
    void aMapWithItsOwnVoltageSensorIsLeftAlone() {
        FakeHardwareMap hardwareMap = new FakeHardwareMap();
        FakeVoltageSensor low = new FakeVoltageSensor(11.2); // volts: a test of a tired battery
        hardwareMap.register("Expansion Hub 2", low);
        ReadsTheBattery opMode = new ReadsTheBattery();

        new OpModeRunner(opMode, hardwareMap).initLoopTicks(1).activeTicks(1).run();

        assertSame(low, opMode.battery, "the test's sensor, not one the runner made");
        assertEquals(11.2, opMode.volts, 0.0, "volts");
        assertEquals(1, hardwareMap.voltageSensor.size(), "the runner added none");
    }
}
