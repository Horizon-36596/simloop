package org.horizon36596.simloop.fakehardware;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.qualcomm.hardware.broadcom.BroadcomColorSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

/**
 * Pins {@link FakeRevColorSensorV3}: robot code reading a REV Color Sensor V3 gets the SDK 12.0 driver's own
 * arithmetic on the counts the sim says the chip measures.
 *
 * <p>Expected values are worked by hand from the driver's bytecode ({@code BroadcomColorSensorImpl
 * .updateColors} and {@code RevColorSensorV3.inFromOptical}), in the same float and double steps the driver
 * takes, so a test failing here means the fake and the SDK disagree - or an SDK bump changed the arithmetic,
 * which must be looked at before it lands.
 *
 * <p>Tiers (domain R9): the SDK-arithmetic and determinism tests are INVARIANT tier - they are what makes the
 * fake honest. The source and metadata tests are implementation tier.
 */
class FakeRevColorSensorV3Test {

    /** The saturation count the driver divides by. SPEC: {@code Parameters.colorSaturation}. */
    private static final float COLOR_SATURATION = 65535.0f;

    private static FakeRevColorSensorV3 seeing(int red, int green, int blue) {
        FakeRevColorSensorV3 sensor = new FakeRevColorSensorV3();
        sensor.setRawCounts(new ColorCounts(red, green, blue));
        return sensor;
    }

    // ---- the raw channels, with the driver's per-channel correction ----

    @Test
    void invariant_rawChannelsCarryTheDriversCorrectionRedTimes107BlueTimes155() {
        FakeRevColorSensorV3 sensor = seeing(1000, 2000, 3000);

        assertEquals(1070, sensor.red(), "red is (int) (1.07 * 1000)");
        assertEquals(2000, sensor.green(), "green is passed through");
        assertEquals(4650, sensor.blue(), "blue is (int) (1.55 * 3000)");
        assertEquals((1070 + 2000 + 4650) / 3, sensor.alpha(), "alpha is the integer mean of the corrected three");
    }

    @Test
    void invariant_aCorrectedChannelClipsAt65535() {
        FakeRevColorSensorV3 sensor = seeing(65535, 65535, 50000);

        assertEquals(65535, sensor.red(), "1.07 * 65535 would be 70122; the driver clips it");
        assertEquals(65535, sensor.green());
        assertEquals(65535, sensor.blue(), "1.55 * 50000 would be 77500; the driver clips it");
    }

    // ---- normalization, gain and clamping ----

    @Test
    void invariant_normalizedColorsAreCorrectedCountsTimesGainOverTheSaturationCount() {
        FakeRevColorSensorV3 sensor = seeing(1000, 2000, 3000);

        NormalizedRGBA colors = sensor.getNormalizedColors();
        assertEquals(1070f * 1.0f / COLOR_SATURATION, colors.red, 0.0f);
        assertEquals(2000f * 1.0f / COLOR_SATURATION, colors.green, 0.0f);
        assertEquals(4650f * 1.0f / COLOR_SATURATION, colors.blue, 0.0f);
        assertEquals(0.0163272, colors.red, 1e-6, "about 1.6% of full scale, worked by hand");
    }

    @Test
    void invariant_normalizedAlphaIsTheDriversBrightnessCurve() {
        FakeRevColorSensorV3 sensor = seeing(1000, 2000, 3000);

        float mean = (1070 + 2000 + 4650) / 3.0f;
        float expected = (float) (-(65535.0 / (Math.pow(mean, 2.0) + 65535.0)) + 1.0);
        assertEquals(expected, sensor.getNormalizedColors().alpha, 0.0f);
        assertEquals(0.99011, sensor.getNormalizedColors().alpha, 1e-4, "worked by hand: 1 - 65535 / (2573.3^2 + 65535)");
    }

    @Test
    void invariant_gainScalesEveryColourChannelAndTheResultClampsToOne() {
        FakeRevColorSensorV3 sensor = seeing(1000, 2000, 3000);
        assertEquals(1.0f, sensor.getGain(), 0.0f, "a fresh driver's software gain is 1");

        sensor.setGain(10.0f);
        assertEquals(10.0f, sensor.getGain(), 0.0f);
        NormalizedRGBA tenTimes = sensor.getNormalizedColors();
        assertEquals(1070f * 10.0f / COLOR_SATURATION, tenTimes.red, 0.0f);
        assertEquals(2000f * 10.0f / COLOR_SATURATION, tenTimes.green, 0.0f);
        assertEquals(4650f * 10.0f / COLOR_SATURATION, tenTimes.blue, 0.0f);

        sensor.setGain(20.0f);
        NormalizedRGBA saturated = sensor.getNormalizedColors();
        assertEquals(1070f * 20.0f / COLOR_SATURATION, saturated.red, 0.0f, "0.33, still below 1");
        assertTrue(4650f * 20.0f / COLOR_SATURATION > 1.0f, "blue would be 1.42 before the clip...");
        assertEquals(1.0f, saturated.blue, 0.0f, "...but the driver never reports more than 1");
    }

    @Test
    void gainDoesNotChangeTheRawChannelsOrAlpha() {
        FakeRevColorSensorV3 sensor = seeing(1000, 2000, 3000);
        float alphaAtOne = sensor.getNormalizedColors().alpha;

        sensor.setGain(50.0f);

        assertEquals(1070, sensor.red(), "gain is applied only to the normalized colours, as in the SDK");
        assertEquals(alphaAtOne, sensor.getNormalizedColors().alpha, 0.0f, "alpha does not use the gain");
    }

    @Test
    void aNegativeGainClampsEveryColourToZero() {
        FakeRevColorSensorV3 sensor = seeing(1000, 2000, 3000);
        sensor.setGain(-2.0f);

        NormalizedRGBA colors = sensor.getNormalizedColors();
        assertEquals(0.0f, colors.red, 0.0f);
        assertEquals(0.0f, colors.green, 0.0f);
        assertEquals(0.0f, colors.blue, 0.0f);
    }

    @Test
    void resetDeviceConfigurationForOpModePutsTheGainBackToOne() {
        FakeRevColorSensorV3 sensor = seeing(1000, 2000, 3000);
        sensor.setGain(8.0f);

        sensor.resetDeviceConfigurationForOpMode();

        assertEquals(1.0f, sensor.getGain(), 0.0f, "BroadcomColorSensorImpl resets softwareGain between OpModes");
    }

    @Test
    void aDarkSensorReadsZeroEverywhere() {
        FakeRevColorSensorV3 sensor = new FakeRevColorSensorV3();

        assertEquals(ColorCounts.DARK, sensor.getRawCounts(), "a fresh sensor sees nothing");
        NormalizedRGBA colors = sensor.getNormalizedColors();
        assertEquals(0, sensor.red());
        assertEquals(0.0f, colors.red, 0.0f);
        assertEquals(0.0f, colors.alpha, 0.0f, "1 - 65535 / (0 + 65535) is exactly 0");
        assertEquals(0, sensor.argb());
    }

    // ---- argb packing ----

    @Test
    void invariant_argbPacksAlphaRedGreenBlueBytesTheWayToColorComputesThem() {
        // Green at half scale: 32768 / 65535 * 256 = 128.002, so 128. Alpha: mean 10922.7, so
        // 1 - 65535 / (10922.7^2 + 65535) = 0.99945, times 256 = 255.86, so 255.
        FakeRevColorSensorV3 sensor = seeing(0, 32768, 0);

        assertEquals(0xFF008000, sensor.argb());
    }

    @Test
    void argbIsTheFourNormalizedBytesInAlphaRedGreenBlueOrder() {
        FakeRevColorSensorV3 sensor = seeing(20000, 9000, 14000);
        sensor.setGain(1.5f);

        NormalizedRGBA colors = sensor.getNormalizedColors();
        int argb = sensor.argb();
        assertEquals(toByte(colors.alpha), (argb >>> 24) & 0xFF, "alpha in the top byte");
        assertEquals(toByte(colors.red), (argb >>> 16) & 0xFF, "then red");
        assertEquals(toByte(colors.green), (argb >>> 8) & 0xFF, "then green");
        assertEquals(toByte(colors.blue), argb & 0xFF, "blue in the bottom byte");
    }

    @Test
    void aFullySaturatedSensorPacksToAllOnes() {
        FakeRevColorSensorV3 sensor = seeing(65535, 65535, 65535);

        // Each normalized channel is exactly 1.0, and 1.0 * 256 = 256 clips to 255.
        assertEquals(0xFFFFFFFF, sensor.argb());
    }

    /** {@code NormalizedRGBA.toColor}'s step, worked independently: times 256, truncated, clipped to a byte. */
    private static int toByte(float normalized) {
        return Math.max(0, Math.min(255, (int) (normalized * 256.0f)));
    }

    // ---- distance, through the SDK's own curve ----

    @Test
    void invariant_aScriptedDistanceReadsBackThroughTheSdkCurveToTheCountsResolution() {
        FakeRevColorSensorV3 sensor = new FakeRevColorSensorV3();
        double[][] inchesAndTolerance = {{0.5, 0.002}, {1.0, 0.001}, {2.0, 0.01}, {3.0, 0.02}, {4.0, 0.04}, {5.0, 0.07}};
        for (double[] row : inchesAndTolerance) {
            sensor.setDistance(DistanceUnit.INCH, row[0]);
            assertEquals(row[0], sensor.getDistance(DistanceUnit.INCH), row[1], row[0] + " in");
        }
    }

    @Test
    void theCountForOneInchIsTheCurvesCOffsetPlusA() {
        // counts = C + A * 1 ^ anything = 26.98 + 325.961 = 352.94, rounded to 353.
        assertEquals(353, FakeRevColorSensorV3.proximityCountsAt(DistanceUnit.INCH, 1.0));
        FakeRevColorSensorV3 sensor = new FakeRevColorSensorV3();
        sensor.setProximityCounts(353);
        double expected = Math.pow((353 - 26.98) / 325.961, -0.75934);
        assertEquals(expected, sensor.getDistance(DistanceUnit.INCH), 0.0, "the SDK's curve, to the bit");
    }

    @Test
    void theDistanceIsTheSdksUnitConversionOfItsInches() {
        FakeRevColorSensorV3 sensor = new FakeRevColorSensorV3();
        sensor.setDistance(DistanceUnit.CM, 5.0);

        double inches = sensor.getDistance(DistanceUnit.INCH);
        assertEquals(DistanceUnit.CM.fromUnit(DistanceUnit.INCH, inches), sensor.getDistance(DistanceUnit.CM), 0.0);
        assertEquals(DistanceUnit.MM.fromUnit(DistanceUnit.INCH, inches), sensor.getDistance(DistanceUnit.MM), 0.0);
        assertEquals(DistanceUnit.METER.fromUnit(DistanceUnit.INCH, inches),
                sensor.getDistance(DistanceUnit.METER), 0.0);
        assertEquals(5.0, sensor.getDistance(DistanceUnit.CM), 0.05, "and 5 cm round-trips to the count resolution");
    }

    @Test
    void invariant_withNothingInFrontItReadsSixInchesNotTheSdkSentinel() {
        FakeRevColorSensorV3 sensor = new FakeRevColorSensorV3();
        assertEquals(6.0, sensor.getDistance(DistanceUnit.INCH), 0.0, "a fresh sensor");

        sensor.setDistance(DistanceUnit.INCH, 2.0);
        sensor.setNothingInRange();
        assertEquals(0, sensor.getProximityCounts());
        assertEquals(6.0, sensor.getDistance(DistanceUnit.INCH), 0.0, "after setNothingInRange");

        sensor.setDistance(DistanceUnit.INCH, Double.POSITIVE_INFINITY);
        assertEquals(6.0, sensor.getDistance(DistanceUnit.INCH), 0.0, "infinitely far");

        sensor.setDistance(DistanceUnit.INCH, 20.0);
        assertEquals(6.0, sensor.getDistance(DistanceUnit.INCH), 0.0, "past the curve's reach, clipped to maxDist");
        assertEquals(FakeRevColorSensorV3.MAX_REPORTED_DISTANCE_INCHES, sensor.getDistance(DistanceUnit.INCH), 0.0);
    }

    @Test
    void touchingTheSensorSaturatesTheProximityCountAtAQuarterInch() {
        FakeRevColorSensorV3 sensor = new FakeRevColorSensorV3();
        sensor.setDistance(DistanceUnit.INCH, 0.0);

        assertEquals(FakeRevColorSensorV3.MAX_PROXIMITY_COUNTS, sensor.getProximityCounts());
        double closest = Math.pow((2047 - 26.98) / 325.961, -0.75934);
        assertEquals(closest, sensor.getDistance(DistanceUnit.INCH), 0.0);
        assertEquals(0.25, closest, 0.01, "about a quarter inch, worked by hand");
    }

    @Test
    void aNearerSurfaceNeverReadsFarther() {
        FakeRevColorSensorV3 sensor = new FakeRevColorSensorV3();
        double previous = -1.0;
        for (double inches = 0.0; inches <= 8.0; inches += 0.05) {
            sensor.setDistance(DistanceUnit.INCH, inches);
            double reading = sensor.getDistance(DistanceUnit.INCH);
            assertTrue(reading >= previous, "at " + inches + " in the reading went from " + previous + " to " + reading);
            previous = reading;
        }
    }

    @Test
    void theLightDetectedIsTheProximityCountOverItsSaturation() {
        FakeRevColorSensorV3 sensor = new FakeRevColorSensorV3();
        sensor.setProximityCounts(1000);

        assertEquals(1000.0, sensor.getRawLightDetected(), 0.0);
        assertEquals(2047.0, sensor.getRawLightDetectedMax(), 0.0);
        assertEquals(1000.0 / 2047.0, sensor.getLightDetected(), 1e-12);
        assertEquals(1000, sensor.rawOptical());
    }

    // ---- the parts of the real sensor robot code sometimes trips over ----

    @Test
    void theLedIsAlwaysOnAndTheSensorIsNotASwitchableLight() {
        FakeRevColorSensorV3 sensor = new FakeRevColorSensorV3();
        sensor.enableLed(false);

        assertTrue(sensor.isLightOn(), "the SDK's enableLed is a no-op on this sensor");
        assertFalse(SwitchableLight.class.isInstance(sensor), "the real V3 is not a SwitchableLight either");
    }

    @Test
    void itIdentifiesItselfAsTheSdkDriverDoes() {
        FakeRevColorSensorV3 sensor = new FakeRevColorSensorV3();

        assertEquals("Rev Color Sensor v3", sensor.getDeviceName());
        assertEquals(HardwareDevice.Manufacturer.Broadcom, sensor.getManufacturer());
        assertEquals((byte) 0xC2, sensor.getDeviceID(), "the APDS-9151 part ID the driver expects");
        assertEquals(BroadcomColorSensor.BROADCOM_APDS9151_ADDRESS, sensor.getI2cAddress());
        assertEquals(1, sensor.getVersion());
        assertTrue(sensor instanceof RevColorSensorV3, "it is the real driver class");
    }

    @Test
    void initializeRunsAndKeepsTheParametersButRefusesAnotherDeviceType() {
        FakeRevColorSensorV3 sensor = seeing(1000, 2000, 3000);
        BroadcomColorSensor.Parameters parameters = BroadcomColorSensor.Parameters.createForAPDS9151();

        assertTrue(sensor.initialize(parameters));
        assertEquals(parameters.deviceId, sensor.getParameters().deviceId);
        assertEquals(1070, sensor.red(), "readings are unchanged by initialization");

        BroadcomColorSensor.Parameters otherDevice = BroadcomColorSensor.Parameters.createForAPDS9151();
        otherDevice.deviceId = 0x12;
        IllegalArgumentException refused = assertThrows(IllegalArgumentException.class,
                () -> sensor.initialize(otherDevice));
        assertTrue(refused.getMessage().contains("can't change device types"), refused.getMessage());
    }

    @Test
    void aRegisterTheEmulatedChipDoesNotModelFailsLoud() {
        FakeRevColorSensorV3 sensor = new FakeRevColorSensorV3();

        assertThrows(UnsupportedOperationException.class, () -> sensor.read8(BroadcomColorSensor.Register.LS_GAIN));
        assertThrows(UnsupportedOperationException.class,
                () -> sensor.write8(BroadcomColorSensor.Register.MAIN_CTRL, 0x06));
    }

    // ---- scripted values and sources ----

    @Test
    void aColourSourceIsAskedOnUpdateAndNotBefore() {
        FakeRevColorSensorV3 sensor = seeing(100, 100, 100);
        AtomicReference<ColorCounts> scene = new AtomicReference<>(new ColorCounts(4000, 5000, 6000));
        sensor.setRawCountsSource(scene::get);

        assertEquals(new ColorCounts(100, 100, 100), sensor.getRawCounts(), "a getter never asks the source");
        sensor.update(0.02);
        assertEquals(new ColorCounts(4000, 5000, 6000), sensor.getRawCounts());
        assertEquals(5000, sensor.green());

        scene.set(new ColorCounts(1, 2, 3));
        sensor.update(0.02);
        assertEquals(2, sensor.green(), "asked again on the next tick");
    }

    @Test
    void aScriptedValueReplacesTheSourceAndANullSourceKeepsTheLastAnswer() {
        FakeRevColorSensorV3 sensor = new FakeRevColorSensorV3();
        sensor.setRawCountsSource(() -> new ColorCounts(7, 8, 9));
        sensor.update(0.02);

        sensor.setRawCounts(new ColorCounts(10, 20, 30));
        sensor.update(0.02);
        assertEquals(new ColorCounts(10, 20, 30), sensor.getRawCounts(), "setRawCounts clears the source");

        sensor.setRawCountsSource(() -> new ColorCounts(40, 50, 60));
        sensor.update(0.02);
        sensor.setRawCountsSource(null);
        sensor.update(0.02);
        assertEquals(new ColorCounts(40, 50, 60), sensor.getRawCounts(), "no source: the last answer stays");
    }

    @Test
    void aDistanceSourceIsAskedOnUpdateInItsOwnUnit() {
        FakeRevColorSensorV3 sensor = new FakeRevColorSensorV3();
        double[] millimetres = {25.4};
        sensor.setDistanceSource(DistanceUnit.MM, () -> millimetres[0]);

        assertEquals(6.0, sensor.getDistance(DistanceUnit.INCH), 0.0, "not asked yet");
        sensor.update(0.02);
        assertEquals(1.0, sensor.getDistance(DistanceUnit.INCH), 0.001, "25.4 mm is one inch");

        millimetres[0] = Double.POSITIVE_INFINITY;
        sensor.update(0.02);
        assertEquals(6.0, sensor.getDistance(DistanceUnit.INCH), 0.0, "infinity means nothing in front");

        sensor.setProximityCounts(353);
        sensor.update(0.02);
        assertEquals(353, sensor.getProximityCounts(), "setProximityCounts clears the distance source");
    }

    @Test
    void theHardwareMapTicksTheSources() {
        FakeHardwareMap map = new FakeHardwareMap();
        FakeRevColorSensorV3 sensor = new FakeRevColorSensorV3();
        map.register("color", sensor);
        sensor.setRawCountsSource(() -> new ColorCounts(0, 30000, 0));

        map.updateAll(0.02);

        assertEquals(30000, map.get(RevColorSensorV3.class, "color").green());
    }

    @Test
    void badInputsAreRefused() {
        FakeRevColorSensorV3 sensor = new FakeRevColorSensorV3();

        assertThrows(IllegalArgumentException.class, () -> sensor.setRawCounts(null));
        assertThrows(IllegalArgumentException.class, () -> sensor.setProximityCounts(-1));
        assertThrows(IllegalArgumentException.class, () -> sensor.setProximityCounts(2048));
        assertThrows(IllegalArgumentException.class, () -> sensor.setDistance(DistanceUnit.INCH, -0.1));
        assertThrows(IllegalArgumentException.class, () -> sensor.setDistance(DistanceUnit.INCH, Double.NaN));
        assertThrows(IllegalArgumentException.class, () -> sensor.setDistance(null, 1.0));
        assertThrows(IllegalArgumentException.class, () -> sensor.setDistanceSource(null, () -> 1.0));

        sensor.setRawCountsSource(() -> null);
        assertThrows(IllegalArgumentException.class, () -> sensor.update(0.02), "a source answering null");
        sensor.setRawCountsSource(null);
        sensor.setDistanceSource(DistanceUnit.INCH, () -> Double.NaN);
        assertThrows(IllegalArgumentException.class, () -> sensor.update(0.02), "a source answering NaN");
    }

    @Test
    void colorCountsRefuseCountsOutsideTheChipsRange() {
        assertThrows(IllegalArgumentException.class, () -> new ColorCounts(-1, 0, 0));
        assertThrows(IllegalArgumentException.class, () -> new ColorCounts(0, 65536, 0));
        assertThrows(IllegalArgumentException.class, () -> new ColorCounts(0, 0, Integer.MAX_VALUE));
        assertEquals(new ColorCounts(1, 2, 3), new ColorCounts(1, 2, 3));
        assertEquals(new ColorCounts(1, 2, 3).hashCode(), new ColorCounts(1, 2, 3).hashCode());
        assertFalse(new ColorCounts(1, 2, 3).equals(new ColorCounts(3, 2, 1)));
    }

    // ---- determinism ----

    @Test
    void invariant_theSameScriptGivesTheSameBitsEveryRun() {
        assertEquals(recordRun(), recordRun());
    }

    @Test
    void getNormalizedColorsReturnsTheDriversOneSharedObject() {
        FakeRevColorSensorV3 sensor = seeing(1000, 2000, 3000);
        // The SDK refreshes and returns the same NormalizedRGBA every call; code that keeps the reference sees
        // it change, on the robot and here.
        assertSame(sensor.getNormalizedColors(), sensor.getNormalizedColors());
    }

    private static List<String> recordRun() {
        FakeRevColorSensorV3 sensor = new FakeRevColorSensorV3();
        List<String> readings = new ArrayList<>();
        for (int tick = 0; tick < 200; tick++) {
            int shade = (tick * 331) % 65536;
            sensor.setRawCounts(new ColorCounts(shade, 65535 - shade, (shade * 7) % 65536));
            sensor.setGain(1.0f + tick % 5);
            sensor.setDistance(DistanceUnit.INCH, (tick % 70) / 10.0);
            NormalizedRGBA colors = sensor.getNormalizedColors();
            readings.add(sensor.red() + " " + sensor.green() + " " + sensor.blue() + " " + sensor.alpha() + " "
                    + Float.floatToIntBits(colors.red) + " " + Float.floatToIntBits(colors.alpha) + " "
                    + sensor.argb() + " " + Double.doubleToLongBits(sensor.getDistance(DistanceUnit.MM)));
        }
        return readings;
    }
}
