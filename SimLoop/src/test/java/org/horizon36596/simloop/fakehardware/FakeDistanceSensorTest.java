package org.horizon36596.simloop.fakehardware;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareDevice;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.List;

/**
 * Pins {@link FakeDistanceSensor}: robot code reading a REV 2m Distance Sensor through the
 * {@link DistanceSensor} interface gets what the SDK 12.0 {@code VL53L0X} driver would hand it - whole
 * millimetres divided by the driver's own constants, and the chip's "no target" number when nothing is in
 * range.
 *
 * <p>Tiers (domain R9): the unit, rounding and out-of-range tests are INVARIANT tier, since robot code
 * branches on exactly these numbers. The source tests are implementation tier.
 */
class FakeDistanceSensorTest {

    // ---- units ----

    @Test
    void invariant_everyUnitIsTheDriversDivisionOfWholeMillimetres() {
        FakeDistanceSensor sensor = new FakeDistanceSensor();
        sensor.setDistance(DistanceUnit.MM, 254.0);

        assertEquals(254.0, sensor.getDistance(DistanceUnit.MM), 0.0);
        assertEquals(25.4, sensor.getDistance(DistanceUnit.CM), 0.0, "mm / 10");
        assertEquals(0.254, sensor.getDistance(DistanceUnit.METER), 0.0, "mm / 1000");
        assertEquals(254.0 / 25.4, sensor.getDistance(DistanceUnit.INCH), 0.0, "mm / 25.4, as the driver divides");
        assertEquals(10.0, sensor.getDistance(DistanceUnit.INCH), 1e-12);
    }

    @Test
    void aDistanceGivenInAnyUnitReadsBackInEveryUnit() {
        FakeDistanceSensor sensor = new FakeDistanceSensor();

        sensor.setDistance(DistanceUnit.INCH, 5.0);
        assertEquals(127.0, sensor.getDistance(DistanceUnit.MM), 0.0, "5 in is exactly 127 mm");
        assertEquals(5.0, sensor.getDistance(DistanceUnit.INCH), 1e-12);

        sensor.setDistance(DistanceUnit.CM, 30.0);
        assertEquals(300.0, sensor.getDistance(DistanceUnit.MM), 0.0);

        sensor.setDistance(DistanceUnit.METER, 1.5);
        assertEquals(1500.0, sensor.getDistance(DistanceUnit.MM), 0.0);
        assertEquals(1.5, sensor.getDistance(DistanceUnit.METER), 0.0);
    }

    @Test
    void invariant_theReadingIsRoundedToTheNearestMillimetre() {
        FakeDistanceSensor sensor = new FakeDistanceSensor();

        sensor.setDistance(DistanceUnit.MM, 123.4);
        assertEquals(123.0, sensor.getDistance(DistanceUnit.MM), 0.0);
        sensor.setDistance(DistanceUnit.MM, 123.6);
        assertEquals(124.0, sensor.getDistance(DistanceUnit.MM), 0.0);
        sensor.setDistance(DistanceUnit.INCH, 3.0);                     // 76.2 mm
        assertEquals(76.0 / 25.4, sensor.getDistance(DistanceUnit.INCH), 0.0,
                "3 in reads as 76 mm, 2.992 in - the chip's resolution, not a bug");
    }

    @Test
    void aTargetAtTheFaceReadsZero() {
        FakeDistanceSensor sensor = new FakeDistanceSensor();
        sensor.setDistance(DistanceUnit.INCH, 0.0);

        assertEquals(0.0, sensor.getDistance(DistanceUnit.INCH), 0.0);
        assertTrue(sensor.isTargetInRange());
    }

    // ---- out of range ----

    @Test
    void invariant_aFreshRev2mSeesNothingAndReportsTheChipsNoTargetNumber() {
        FakeDistanceSensor sensor = new FakeDistanceSensor();

        assertFalse(sensor.isTargetInRange());
        assertEquals(8190.0, sensor.getDistance(DistanceUnit.MM), 0.0);
        assertEquals(819.0, sensor.getDistance(DistanceUnit.CM), 0.0);
        assertEquals(8.19, sensor.getDistance(DistanceUnit.METER), 1e-12);
        assertEquals(8190.0 / 25.4, sensor.getDistance(DistanceUnit.INCH), 0.0);
        assertEquals(322.44, sensor.getDistance(DistanceUnit.INCH), 0.01, "about 322 in, as teams report");
    }

    @Test
    void invariant_aRev2mNeverReturnsTheSdkSentinel() {
        FakeDistanceSensor sensor = new FakeDistanceSensor();
        sensor.setNothingInRange();

        for (DistanceUnit unit : DistanceUnit.values()) {
            assertTrue(sensor.getDistance(unit) != DistanceSensor.distanceOutOfRange,
                    "the REV 2m driver passes the chip's number on; it never returns distanceOutOfRange (" + unit + ")");
        }
    }

    @Test
    void theRangeEndsAtTwoMetresExactly() {
        FakeDistanceSensor sensor = new FakeDistanceSensor();

        sensor.setDistance(DistanceUnit.MM, 2000.0);
        assertEquals(2000.0, sensor.getDistance(DistanceUnit.MM), 0.0, "the last millimetre in range");
        sensor.setDistance(DistanceUnit.MM, 2000.4);
        assertEquals(8190.0, sensor.getDistance(DistanceUnit.MM), 0.0,
                "just past the range reads as no target, even though it would round to 2000");
        sensor.setDistance(DistanceUnit.MM, Double.POSITIVE_INFINITY);
        assertEquals(8190.0, sensor.getDistance(DistanceUnit.MM), 0.0);
    }

    @Test
    void invariant_aSensorBuiltToReportTheSentinelReportsItUnconvertedInEveryUnit() {
        FakeDistanceSensor sensor = new FakeDistanceSensor(1000.0, DistanceSensor.distanceOutOfRange);

        for (DistanceUnit unit : DistanceUnit.values()) {
            assertEquals(DistanceSensor.distanceOutOfRange, sensor.getDistance(unit), 0.0,
                    "robot code's == distanceOutOfRange check must match in " + unit);
        }
        sensor.setDistance(DistanceUnit.MM, 999.6);
        assertEquals(1000.0, sensor.getDistance(DistanceUnit.MM), 0.0, "in range, it is an ordinary reading");
        sensor.setDistance(DistanceUnit.MM, 1001.0);
        assertEquals(DistanceSensor.distanceOutOfRange, sensor.getDistance(DistanceUnit.INCH), 0.0);
    }

    @Test
    void aCustomRangeAndNoTargetReadingAreHonoured() {
        FakeDistanceSensor sensor = new FakeDistanceSensor(500.0, 65535.0);
        sensor.setDistance(DistanceUnit.MM, 600.0);

        assertEquals(65535.0, sensor.getDistance(DistanceUnit.MM), 0.0);
        assertEquals(500.0, sensor.getMaxRangeMm(), 0.0);
        assertEquals(65535.0, sensor.getOutOfRangeReadingMm(), 0.0);
    }

    // ---- sources ----

    @Test
    void aSourceIsAskedOnUpdateInItsOwnUnitAndNotBefore() {
        FakeDistanceSensor sensor = new FakeDistanceSensor();
        double[] inches = {12.0};
        sensor.setDistanceSource(DistanceUnit.INCH, () -> inches[0]);

        assertFalse(sensor.isTargetInRange(), "a getter never asks the source");
        sensor.update(0.02);
        assertEquals(12.0 * 25.4, sensor.getDistance(DistanceUnit.MM), 0.5);

        inches[0] = Double.POSITIVE_INFINITY;
        sensor.update(0.02);
        assertFalse(sensor.isTargetInRange(), "the source said nothing is in view");
    }

    @Test
    void aScriptedDistanceClearsTheSourceAndANullSourceKeepsTheLastAnswer() {
        FakeDistanceSensor sensor = new FakeDistanceSensor();
        sensor.setDistanceSource(DistanceUnit.MM, () -> 100.0);
        sensor.update(0.02);
        sensor.setDistance(DistanceUnit.MM, 300.0);
        sensor.update(0.02);
        assertEquals(300.0, sensor.getDistance(DistanceUnit.MM), 0.0, "setDistance cleared the source");

        sensor.setDistanceSource(DistanceUnit.MM, () -> 400.0);
        sensor.update(0.02);
        sensor.setDistanceSource(DistanceUnit.MM, null);
        sensor.update(0.02);
        assertEquals(400.0, sensor.getDistance(DistanceUnit.MM), 0.0);
    }

    @Test
    void theHardwareMapTicksTheSource() {
        FakeHardwareMap map = new FakeHardwareMap();
        FakeDistanceSensor sensor = new FakeDistanceSensor();
        map.register("range", sensor);
        sensor.setDistanceSource(DistanceUnit.CM, () -> 50.0);

        map.updateAll(0.02);

        assertEquals(50.0, map.get(DistanceSensor.class, "range").getDistance(DistanceUnit.CM), 0.0);
    }

    // ---- guards and metadata ----

    @Test
    void badInputsAreRefused() {
        FakeDistanceSensor sensor = new FakeDistanceSensor();

        assertThrows(IllegalArgumentException.class, () -> sensor.setDistance(DistanceUnit.MM, -1.0));
        assertThrows(IllegalArgumentException.class, () -> sensor.setDistance(DistanceUnit.MM, Double.NaN));
        assertThrows(IllegalArgumentException.class, () -> sensor.setDistance(null, 1.0));
        assertThrows(IllegalArgumentException.class, () -> sensor.setDistanceSource(null, () -> 1.0));
        sensor.setDistanceSource(DistanceUnit.MM, () -> -5.0);
        assertThrows(IllegalArgumentException.class, () -> sensor.update(0.02), "a source answering a negative");

        assertThrows(IllegalArgumentException.class, () -> new FakeDistanceSensor(0.0, 8190.0));
        assertThrows(IllegalArgumentException.class, () -> new FakeDistanceSensor(Double.POSITIVE_INFINITY, 8190.0));
        assertThrows(IllegalArgumentException.class, () -> new FakeDistanceSensor(2000.0, Double.NaN));
        assertThrows(IllegalArgumentException.class, () -> new FakeDistanceSensor(2000.0, -1.0));
    }

    @Test
    void itIdentifiesItselfAsTheRev2mDriverDoes() {
        FakeDistanceSensor sensor = new FakeDistanceSensor();

        assertEquals("REV 2M ToF Distance Sensor", sensor.getDeviceName());
        assertEquals(HardwareDevice.Manufacturer.Other, sensor.getManufacturer());
        assertEquals(1, sensor.getVersion());
    }

    @Test
    void invariant_theSameScriptGivesTheSameBitsEveryRun() {
        assertEquals(recordRun(), recordRun());
    }

    private static List<Long> recordRun() {
        FakeDistanceSensor sensor = new FakeDistanceSensor();
        List<Long> readings = new ArrayList<>();
        for (int tick = 0; tick < 300; tick++) {
            sensor.setDistance(DistanceUnit.INCH, tick * 0.37);
            for (DistanceUnit unit : DistanceUnit.values()) {
                readings.add(Double.doubleToLongBits(sensor.getDistance(unit)));
            }
        }
        return readings;
    }
}
