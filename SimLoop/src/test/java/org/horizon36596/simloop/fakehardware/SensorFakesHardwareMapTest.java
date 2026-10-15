package org.horizon36596.simloop.fakehardware;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.junit.jupiter.api.Test;

import java.util.Arrays;
import java.util.HashSet;
import java.util.Set;

/**
 * INVARIANT-tier (domain R9): robot code finds the touch, distance and colour fakes through
 * {@link FakeHardwareMap} by every type the real device answers to, and they are filed in the typed mappings
 * the SDK 12.0 configuration parser files the real sensors in - so unchanged robot code resolves them in sim
 * exactly as it does on a Control Hub, and a name that is not there fails the way the SDK fails.
 */
class SensorFakesHardwareMapTest {

    private final FakeHardwareMap map = new FakeHardwareMap();
    private final FakeTouchSensor touch = new FakeTouchSensor();
    private final FakeDistanceSensor range = new FakeDistanceSensor();
    private final FakeRevColorSensorV3 color = new FakeRevColorSensorV3();
    private final FakeDigitalChannel limit = new FakeDigitalChannel();

    SensorFakesHardwareMapTest() {
        map.register("touch", touch);
        map.register("range", range);
        map.register("color", color);
        map.register("limit", limit);
    }

    // ---- lookup by every interface ----

    @Test
    void theTouchSensorResolvesAsATouchSensor() {
        assertSame(touch, map.get(TouchSensor.class, "touch"));
        assertSame(touch, map.get(HardwareDevice.class, "touch"));
        assertSame(touch, map.get("touch"), "the untyped get");
    }

    @Test
    void theDistanceSensorResolvesAsADistanceSensor() {
        assertSame(range, map.get(DistanceSensor.class, "range"));
        assertSame(range, map.get(HardwareDevice.class, "range"));
    }

    @Test
    void theColourSensorResolvesByEveryTypeARealV3AnswersTo() {
        assertSame(color, map.get(ColorSensor.class, "color"));
        assertSame(color, map.get(NormalizedColorSensor.class, "color"));
        assertSame(color, map.get(RevColorSensorV3.class, "color"), "the concrete SDK driver class");
        assertSame(color, map.get(ColorRangeSensor.class, "color"), "what the SDK samples ask for");
        assertSame(color, map.get(DistanceSensor.class, "color"), "a V3 is a distance sensor too");
        assertSame(color, map.get(OpticalDistanceSensor.class, "color"));
        assertSame(color, map.get(LightSensor.class, "color"));
        assertEquals(6.0, map.get(DistanceSensor.class, "color").getDistance(DistanceUnit.INCH), 0.0,
                "and reads as one through that interface");
    }

    @Test
    void theDigitalChannelResolvesAsADigitalChannel() {
        assertSame(limit, map.get(DigitalChannel.class, "limit"));
    }

    // ---- the typed mappings ----

    @Test
    void eachSensorIsInTheTypedMappingsTheSdkFilesItIn() {
        assertSame(touch, map.touchSensor.get("touch"), "RevTouchSensor lands in hardwareMap.touchSensor");
        assertSame(color, map.colorSensor.get("color"), "RevColorSensorV3 lands in colorSensor...");
        assertSame(color, map.lightSensor.get("color"), "...and lightSensor...");
        assertSame(color, map.opticalDistanceSensor.get("color"), "...and opticalDistanceSensor");
        assertSame(limit, map.digitalChannel.get("limit"));

        assertEquals(1, map.touchSensor.size());
        assertEquals(1, map.colorSensor.size());
        assertEquals(1, map.lightSensor.size());
        assertEquals(1, map.opticalDistanceSensor.size());
        assertEquals(1, map.digitalChannel.size());
        assertFalse(map.colorSensor.contains("range"), "a REV 2m fits no typed mapping");
        assertFalse(map.lightSensor.contains("range"));
    }

    @Test
    void getAllFindsEveryDistanceSensorIncludingTheColourSensor() {
        // getAll initializes each I2C device it returns, as on a robot; the colour fake's initialization must
        // therefore run in a JVM.
        Set<DistanceSensor> expected = new HashSet<>(Arrays.<DistanceSensor>asList(range, color));
        assertEquals(expected, new HashSet<>(map.getAll(DistanceSensor.class)));
        assertEquals(4, map.size());
    }

    @Test
    void theColourSensorStaysReachableThroughItsTypedMappingsAfterAnOpModeReset() {
        // DeviceMapping.get initializes an I2C device on first use, logging through Android as it does; the
        // fake re-initializes itself on reset so that never happens in a JVM.
        color.setGain(4.0f);
        color.resetDeviceConfigurationForOpMode();

        assertSame(color, map.colorSensor.get("color"));
        assertEquals(1.0f, map.get(NormalizedColorSensor.class, "color").getGain(), 0.0f);
    }

    // ---- names ----

    @Test
    void invariant_anUnknownNameFailsTheWayTheSdkFails() {
        IllegalArgumentException missing = assertThrows(IllegalArgumentException.class,
                () -> map.get(TouchSensor.class, "nothingIsCalledThis"));
        assertTrue(missing.getMessage().contains("nothingIsCalledThis"), missing.getMessage());
        assertTrue(missing.getMessage().contains("TouchSensor"), missing.getMessage());
        assertNull(map.tryGet(DistanceSensor.class, "nothingIsCalledThis"), "tryGet's contract is null");
    }

    @Test
    void invariant_aSensorIsNotReturnedAsATypeItIsNot() {
        assertThrows(IllegalArgumentException.class, () -> map.get(ColorSensor.class, "touch"));
        assertThrows(IllegalArgumentException.class, () -> map.get(TouchSensor.class, "range"));
        assertThrows(IllegalArgumentException.class, () -> map.get(ColorSensor.class, "range"),
                "a distance sensor is not a colour sensor");
        assertThrows(IllegalArgumentException.class, () -> map.get(TouchSensor.class, "limit"),
                "a digital channel is not a touch sensor, even with a switch on it");
        assertNull(map.tryGet(DigitalChannel.class, "touch"), "nor a touch sensor a digital channel");
        assertThrows(IllegalArgumentException.class, () -> map.get(RevColorSensorV3.class, "range"));
    }

    @Test
    void namesAreTrimmedTheWayTheSdkTrimsThem() {
        FakeHardwareMap spaced = new FakeHardwareMap();
        FakeRevColorSensorV3 sensor = new FakeRevColorSensorV3();
        spaced.register("  intakeColor ", sensor);

        assertSame(sensor, spaced.get(NormalizedColorSensor.class, "intakeColor"));
        assertSame(sensor, spaced.colorSensor.get(" intakeColor"));
    }

    @Test
    void aTouchSensorAndADistanceSensorCanShareOneConfigName() {
        FakeHardwareMap shared = new FakeHardwareMap();
        FakeTouchSensor bumper = new FakeTouchSensor();
        FakeDistanceSensor bumperRange = new FakeDistanceSensor();
        shared.register("front", bumper);
        shared.register("front", bumperRange);

        assertSame(bumper, shared.get(TouchSensor.class, "front"));
        assertSame(bumperRange, shared.get(DistanceSensor.class, "front"));
    }

    // ---- ticking ----

    @Test
    void updateAllSamplesEverySensorsSource() {
        touch.setPressedSource(() -> true);
        range.setDistanceSource(DistanceUnit.CM, () -> 40.0);
        color.setRawCountsSource(() -> new ColorCounts(0, 0, 1000));

        map.updateAll(0.02);

        assertTrue(map.get(TouchSensor.class, "touch").isPressed());
        assertEquals(40.0, map.get(DistanceSensor.class, "range").getDistance(DistanceUnit.CM), 0.0);
        assertEquals(1550, map.get(ColorSensor.class, "color").blue());
    }
}
