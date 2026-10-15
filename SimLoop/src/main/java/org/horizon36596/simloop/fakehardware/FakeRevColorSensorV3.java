package org.horizon36596.simloop.fakehardware;

import com.qualcomm.hardware.broadcom.BroadcomColorSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/**
 * Headless fake of the REV Color Sensor V3. It <b>is</b> the SDK 12.0 {@link RevColorSensorV3} driver, running
 * on an emulated Broadcom APDS-9151 chip instead of a real I2C bus, so robot code resolves it by any of the
 * types a real one answers to - {@code RevColorSensorV3}, {@code ColorRangeSensor}, {@code ColorSensor},
 * {@code NormalizedColorSensor}, {@code DistanceSensor}, {@code OpticalDistanceSensor},
 * {@code LightSensor} - and reads it through the SDK's own code.
 *
 * <h2>Why the real driver, not a copy of it</h2>
 * Everything a robot reads from this sensor is the driver's arithmetic on a few chip registers: a
 * per-channel correction (red times 1.07, blue times 1.55, each clipped to 65535), the software
 * {@link #setGain(float) gain}, the division by the saturation count, the clip to {@code [0, 1]}, the
 * {@code alpha} formula, and a fitted curve from the proximity count to inches. Re-deriving that would be
 * hoping to match it. Instead the fake supplies only the other side of the bus - the raw counts the
 * photodiodes and the proximity LED would measure - and the SDK turns them into readings exactly as it does
 * on a robot. The same pattern as {@code FakeGoBildaPinpoint}.
 *
 * <p><b>Two public methods are overridden, and one protected one.</b> {@link #argb()} is the one whose
 * answer is computed here; the other two are about initialization and are described below. The SDK packs
 * {@code argb()} with
 * {@code android.graphics.Color.argb}, which does not exist off an Android device: in a JVM test it throws
 * "not mocked", or returns 0 under {@code unitTests.returnDefaultValues}. The override packs the same four
 * bytes the same way ({@code NormalizedRGBA.toColor}'s arithmetic, then {@code a<<24 | r<<16 | g<<8 | b}).
 * Robot code that calls {@code getNormalizedColors().toColor()} itself still reaches Android and cannot run
 * off the robot; that is the SDK's limit, not this fake's.
 *
 * <h2>Two ways to drive it</h2>
 * <ul>
 *   <li><b>Scripted:</b> {@link #setRawCounts(ColorCounts)} for what the photodiodes see, and
 *       {@link #setDistance(DistanceUnit, double)} (or {@link #setProximityCounts(int)}) for how far the
 *       surface is. Both take effect at once.</li>
 *   <li><b>From a source:</b> {@link #setRawCountsSource(Supplier)} and
 *       {@link #setDistanceSource(DistanceUnit, DoubleSupplier)}, asked once per {@link #update(double)} -
 *       for example by a {@code org.horizon36596.simloop.sensor.ColorProbe} that works out which game piece
 *       is in front of the sensor's mount.</li>
 * </ul>
 * A scripted value clears the matching source; a new source replaces a scripted value from the next tick.
 *
 * <h2>What the emulated chip does, and does not</h2>
 * <ul>
 *   <li>It answers the reads the driver makes to produce a reading: the part ID ({@code 0xC2}, what the
 *       driver expects), the main status (new light-sensor data ready on every read), the 11-bit proximity
 *       count, and the three 20-bit light-sensor channels at the SDK's 16-bit resolution. Any other read, and
 *       every write, fails loud, because there is nothing behind it to model.</li>
 *   <li>PLACEHOLDER: fresh data on every read. A real chip measures every 100 ms by default (the driver's
 *       default rate) and the driver keeps its last values in between, so a robot polling faster reads the
 *       same numbers twice; the sim's reading changes the moment the scene does.</li>
 *   <li>The distance curve saturates both ways, as a real sensor's does: nothing closer than about 0.25 in
 *       (proximity count 2047) and nothing farther than {@link #MAX_REPORTED_DISTANCE_INCHES} - with nothing
 *       in front of it, the sensor reads 6 in, <b>not</b> {@code DistanceSensor.distanceOutOfRange}. The SDK's
 *       curve is a fit, so the round trip from a distance to a count and back is exact only to the count's
 *       resolution: about 0.02% at 1 in and about 1% at 4 in.</li>
 *   <li><b>Initialization keeps the SDK's bookkeeping and skips the chip programming.</b> The SDK's
 *       {@code internalInitialize} logs through Android's {@code Log} (which a JVM does not have) and then
 *       writes the chip's gain, LED and measurement-rate registers (which the emulated chip does not model).
 *       This fake replaces that one protected method with the parts that are observable: it refuses a
 *       change of device type with the SDK's own message, stores the parameters and sets the I2C address.
 *       So {@code initialize(...)} works in a JVM test.</li>
 *   <li><b>A fake is initialized as soon as it is built, and again as soon as an OpMode reset clears that.</b>
 *       On a robot the SDK initializes an I2C device the first time the hardware map hands it out - through
 *       {@code getAll(...)} or a typed mapping such as {@code hardwareMap.colorSensor.get(name)} - and logs
 *       a line through Android's {@code Log} as it does, which would stop a JVM test dead.
 *       {@link #resetDeviceConfigurationForOpMode()} is the SDK's own reset followed by that initialization,
 *       so the gain still goes back to 1 between OpModes and the device is never left needing a first-use
 *       initialization.</li>
 *   <li>The LED is the real sensor's: always on. {@code enableLed(false)} does nothing and
 *       {@code isLightOn()} is always true, because that is what the SDK driver does; the real V3 is not a
 *       {@code SwitchableLight}, so neither is this fake.</li>
 *   <li>Not modelled: noise, ambient light, how a surface's counts change with distance or angle, the
 *       measurement rate and the chip's hardware gain.</li>
 * </ul>
 *
 * <p>Deterministic: no clock, no randomness. A fresh sensor sees {@link ColorCounts#DARK} with nothing in
 * front of it.
 */
public class FakeRevColorSensorV3 extends RevColorSensorV3 implements FakeDevice {

    /**
     * The farthest distance the sensor ever reports, inches. SPEC: {@code maxDist} in the SDK 12.0
     * {@code RevColorSensorV3} constructor; the driver clips every reading to it.
     */
    public static final double MAX_REPORTED_DISTANCE_INCHES = 6.0;

    /** The largest proximity count, an 11-bit number. SPEC: the SDK's {@code proximitySaturation}, 2047. */
    public static final int MAX_PROXIMITY_COUNTS = 2047;

    // The SDK 12.0 RevColorSensorV3 distance curve, inches = ((counts - C) / A) ^ B_INVERSE, clipped to
    // MAX_REPORTED_DISTANCE_INCHES and taken as that maximum when counts <= C. SPEC: the aParam, binvParam and
    // cParam its constructor sets, read out of the bytecode. Used only to turn a scripted distance into the
    // count that reads back as it; the reading itself is the SDK's own arithmetic.
    private static final double CURVE_A = 325.961;
    private static final double CURVE_B_INVERSE = -0.75934;
    private static final double CURVE_C = 26.98;

    private final Apds9151 chip;

    private Supplier<ColorCounts> rawCountsSource;
    private DistanceUnit distanceSourceUnit;
    private DoubleSupplier distanceSource;

    /** A sensor seeing {@link ColorCounts#DARK} with nothing in front of it. */
    public FakeRevColorSensorV3() {
        this(new Apds9151());
    }

    private FakeRevColorSensorV3(Apds9151 chip) {
        super(chip, true);
        this.chip = chip;
        // Built already initialized, as if the SDK's first-use initialization had run (class javadoc). On the
        // emulated chip that initialization changes nothing else: the parameters are already the defaults and
        // the bus is already at the chip's address.
        this.isInitialized = true;
    }

    // ---- The sim's side: what the chip measures ----

    /**
     * Make the photodiodes measure {@code counts} from now on. Clears any colour source. <b>Sim-only</b>.
     *
     * @param counts the raw per-channel counts the chip measures, before any driver arithmetic
     * @throws IllegalArgumentException if {@code counts} is null
     */
    public void setRawCounts(ColorCounts counts) {
        this.rawCountsSource = null;
        chip.setLightCounts(requireCounts(counts));
    }

    /** {@return the raw per-channel counts the emulated chip is measuring right now} */
    public ColorCounts getRawCounts() {
        return chip.getLightCounts();
    }

    /**
     * Ask {@code source} what the photodiodes measure on every {@link #update(double)} from now on. Replaces
     * any earlier colour source and, from the next tick, any scripted counts. Pass null to stop asking; the
     * sensor keeps the last counts the source gave.
     *
     * @param source what decides the raw counts, asked once per harness tick; must not answer null. May be
     *               null
     */
    public void setRawCountsSource(Supplier<ColorCounts> source) {
        this.rawCountsSource = source;
    }

    /**
     * Set the raw proximity count the chip measures from now on. Clears any distance source.
     * <b>Sim-only</b>.
     *
     * @param counts the 11-bit proximity count, {@code 0..}{@link #MAX_PROXIMITY_COUNTS}; bigger is closer
     * @throws IllegalArgumentException if {@code counts} is outside that range
     */
    public void setProximityCounts(int counts) {
        if (counts < 0 || counts > MAX_PROXIMITY_COUNTS) {
            throw new IllegalArgumentException("proximity counts must be 0.." + MAX_PROXIMITY_COUNTS
                    + ", got " + counts);
        }
        this.distanceSource = null;
        this.distanceSourceUnit = null;
        chip.setProximityCounts(counts);
    }

    /** {@return the raw 11-bit proximity count the emulated chip is measuring right now} */
    public int getProximityCounts() {
        return chip.getProximityCounts();
    }

    /**
     * Put a surface {@code distance} in front of the sensor from now on, by setting the proximity count the
     * SDK's curve reads back as that distance (see the class javadoc for how close the round trip is).
     * Clears any distance source. <b>Sim-only</b>.
     *
     * @param unit     the unit {@code distance} is in
     * @param distance how far the surface is; {@code +infinity} means nothing is there
     * @throws IllegalArgumentException if {@code unit} is null, or {@code distance} is NaN or negative
     */
    public void setDistance(DistanceUnit unit, double distance) {
        int counts = proximityCountsAt(unit, distance);
        this.distanceSource = null;
        this.distanceSourceUnit = null;
        chip.setProximityCounts(counts);
    }

    /** Take the surface away from now on: proximity count 0, which the SDK reads as {@link #MAX_REPORTED_DISTANCE_INCHES}. */
    public void setNothingInRange() {
        setProximityCounts(0);
    }

    /**
     * Ask {@code source} how far the surface is on every {@link #update(double)} from now on. Replaces any
     * earlier distance source and, from the next tick, any scripted distance. Pass a null source to stop
     * asking; the sensor keeps the last distance the source gave.
     *
     * @param unit   the unit the source answers in
     * @param source what decides the distance, asked once per harness tick; may answer {@code +infinity} for
     *               nothing in front of the sensor, never NaN or a negative number. May be null
     * @throws IllegalArgumentException if {@code source} is not null and {@code unit} is
     */
    public void setDistanceSource(DistanceUnit unit, DoubleSupplier source) {
        if (source != null && unit == null) {
            throw new IllegalArgumentException("a distance source needs the unit it answers in");
        }
        this.distanceSourceUnit = unit;
        this.distanceSource = source;
    }

    /**
     * The proximity count the SDK's curve reads back as {@code distance}: the curve inverted, rounded to a
     * whole count and clipped to {@code 0..}{@link #MAX_PROXIMITY_COUNTS}.
     *
     * @param unit     the unit {@code distance} is in
     * @param distance how far the surface is; {@code +infinity} gives 0 (nothing there)
     * @return the count, {@code 0..}{@link #MAX_PROXIMITY_COUNTS}
     * @throws IllegalArgumentException if {@code unit} is null, or {@code distance} is NaN or negative
     */
    public static int proximityCountsAt(DistanceUnit unit, double distance) {
        if (unit == null) {
            throw new IllegalArgumentException("a distance needs a unit");
        }
        if (Double.isNaN(distance) || distance < 0.0) {
            throw new IllegalArgumentException("a distance must be zero or more (or +infinity for nothing in"
                    + " front of the sensor), got " + distance);
        }
        if (distance == Double.POSITIVE_INFINITY) {
            return 0;
        }
        double inches = unit.toInches(distance);
        // inches = ((counts - C) / A) ^ B_INVERSE, so counts = C + A * inches ^ (1 / B_INVERSE). At zero
        // inches the power is +infinity, which the clip turns into the saturated count.
        double counts = CURVE_C + CURVE_A * StrictMath.pow(inches, 1.0 / CURVE_B_INVERSE);
        return (int) Math.max(0L, Math.min(MAX_PROXIMITY_COUNTS, Math.round(counts)));
    }

    /**
     * Samples the colour and distance sources, if there are any. The sensor has no dynamics of its own.
     *
     * @param deltaTime how far to advance, in seconds; unused
     * @throws IllegalArgumentException if a source answers null counts, or a NaN or negative distance
     */
    @Override
    public void update(double deltaTime) {
        if (rawCountsSource != null) {
            chip.setLightCounts(requireCounts(rawCountsSource.get()));
        }
        if (distanceSource != null) {
            chip.setProximityCounts(proximityCountsAt(distanceSourceUnit, distanceSource.getAsDouble()));
        }
    }

    // ---- The SDK methods replaced ----

    /**
     * The SDK's reset between OpModes - default parameters, software gain back to 1 - followed at once by
     * the initialization the SDK would otherwise do on the next lookup (see the class javadoc).
     */
    @Override
    public void resetDeviceConfigurationForOpMode() {
        super.resetDeviceConfigurationForOpMode();
        initialize();
    }

    /**
     * The SDK's chip initialization, minus the logging a JVM cannot do and the register writes the emulated
     * chip does not model (see the class javadoc). As the SDK does: a {@code deviceId} other than the
     * sensor's is refused, and the parameters and I2C address are taken.
     *
     * @param parameters the parameters to initialize with; the SDK clones them
     * @return true: the emulated chip is always there to initialize
     * @throws IllegalArgumentException if {@code parameters} names a different device type, with the SDK's
     *                                  message
     */
    @Override
    protected synchronized boolean internalInitialize(BroadcomColorSensor.Parameters parameters) {
        if (this.parameters.deviceId != parameters.deviceId) {
            throw new IllegalArgumentException(String.format(
                    "can't change device types (modify existing params instead): old=%d new=%d",
                    this.parameters.deviceId, parameters.deviceId));
        }
        this.parameters = parameters.clone();
        setI2cAddress(parameters.i2cAddr);
        return true;
    }

    /**
     * The normalized colour packed as an Android colour int, alpha in the top byte: the SDK's own
     * {@code NormalizedRGBA.toColor} arithmetic (each channel times 256, truncated, clipped to
     * {@code 0..255}), packed without {@code android.graphics.Color} - see the class javadoc.
     *
     * @return {@code alpha << 24 | red << 16 | green << 8 | blue}, each byte from {@link #getNormalizedColors()}
     */
    @Override
    public synchronized int argb() {
        NormalizedRGBA colors = getNormalizedColors();
        return (colorByte(colors.alpha) << 24) | (colorByte(colors.red) << 16)
                | (colorByte(colors.green) << 8) | colorByte(colors.blue);
    }

    /** {@code NormalizedRGBA.toColor}'s per-channel step: times 256, truncated toward zero, clipped to a byte. */
    private static int colorByte(float normalized) {
        int value = (int) (normalized * 256.0f);
        return Math.max(0, Math.min(255, value));
    }

    private static ColorCounts requireCounts(ColorCounts counts) {
        if (counts == null) {
            throw new IllegalArgumentException("raw colour counts must not be null; use ColorCounts.DARK for"
                    + " no light");
        }
        return counts;
    }

    /**
     * The emulated Broadcom APDS-9151: the registers the SDK driver reads to produce a reading, and nothing
     * else. Register addresses are the SDK's {@code BroadcomColorSensor.Register} values; each light channel
     * is three bytes, little-endian, of which the driver reads the low two.
     */
    private static final class Apds9151 extends FakeI2cDeviceSynchSimple {

        private static final int PART_ID = 0x06;
        private static final int MAIN_STATUS = 0x07;
        private static final int PS_DATA = 0x08;
        private static final int LS_DATA_GREEN = 0x0D;
        private static final int LS_DATA_BLUE = 0x10;
        private static final int LS_DATA_RED = 0x13;
        /** The last register the driver reads: the high byte of the red channel. */
        private static final int LAST_MODELLED_REGISTER = 0x15;

        /** SPEC: the device ID {@code BroadcomColorSensor.Parameters.createForAPDS9151} expects, -62 as a byte. */
        private static final byte APDS9151_PART_ID = (byte) 0xC2;

        /** "Light-sensor data ready" ({@code MainStatus.LS_DATA_STATUS}) and "proximity data ready". */
        private static final byte FRESH_DATA_STATUS = 0x09;

        private final byte[] registers = new byte[LAST_MODELLED_REGISTER + 1];
        private ColorCounts lightCounts = ColorCounts.DARK;
        private int proximityCounts;

        Apds9151() {
            // A real client is addressed at the chip's own address before the driver reads anything.
            setI2cAddress(BroadcomColorSensor.BROADCOM_APDS9151_ADDRESS);
            registers[PART_ID] = APDS9151_PART_ID;
            registers[MAIN_STATUS] = FRESH_DATA_STATUS;
            setProximityCounts(0);
            setLightCounts(ColorCounts.DARK);
        }

        ColorCounts getLightCounts() {
            return lightCounts;
        }

        void setLightCounts(ColorCounts counts) {
            lightCounts = counts;
            writeChannel(LS_DATA_GREEN, counts.getGreen());
            writeChannel(LS_DATA_BLUE, counts.getBlue());
            writeChannel(LS_DATA_RED, counts.getRed());
        }

        int getProximityCounts() {
            return proximityCounts;
        }

        void setProximityCounts(int counts) {
            proximityCounts = counts;
            registers[PS_DATA] = (byte) (counts & 0xFF);
            registers[PS_DATA + 1] = (byte) ((counts >> 8) & 0x07);
        }

        private void writeChannel(int firstRegister, int count) {
            registers[firstRegister] = (byte) (count & 0xFF);
            registers[firstRegister + 1] = (byte) ((count >> 8) & 0xFF);
            registers[firstRegister + 2] = 0;
        }

        @Override
        public byte read8(int ireg) {
            return read(ireg, 1)[0];
        }

        @Override
        public byte[] read(int ireg, int creg) {
            if (ireg < PART_ID || creg < 1 || ireg + creg - 1 > LAST_MODELLED_REGISTER) {
                throw new UnsupportedOperationException("the emulated APDS-9151 models registers 0x06..0x15"
                        + " (part ID, status, proximity and colour data) - the reads the SDK driver makes for a"
                        + " reading - not " + creg + " byte(s) from 0x" + Integer.toHexString(ireg));
            }
            byte[] bytes = new byte[creg];
            System.arraycopy(registers, ireg, bytes, 0, creg);
            return bytes;
        }
    }
}
