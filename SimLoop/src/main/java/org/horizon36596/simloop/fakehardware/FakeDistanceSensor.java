package org.horizon36596.simloop.fakehardware;

import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.function.DoubleSupplier;

/**
 * Fake {@link DistanceSensor} modelled on the REV 2m Distance Sensor (a VL53L0X time-of-flight chip).
 * Unchanged robot code resolves it with {@code hardwareMap.get(DistanceSensor.class, name)} and reads
 * {@link #getDistance(DistanceUnit)} exactly as it does on the real robot.
 *
 * <h2>Two ways to drive it</h2>
 * <ul>
 *   <li><b>Scripted:</b> {@link #setDistance(DistanceUnit, double)} and {@link #setNothingInRange()} take
 *       effect at once.</li>
 *   <li><b>From a source:</b> {@link #setDistanceSource(DistanceUnit, DoubleSupplier)} hands the fake
 *       something to ask once per {@link #update(double)}, such as a
 *       {@code org.horizon36596.simloop.sensor.RangeFinder} casting a ray from the sensor's mount to the
 *       field walls and game pieces. Asking on the tick, not on every read, keeps the getter a pure read.</li>
 * </ul>
 * Whichever was called last wins: a scripted value clears any source.
 *
 * <h2>What the robot reads, read out of the SDK 12.0 {@code VL53L0X} bytecode</h2>
 * <ul>
 *   <li><b>Whole millimetres.</b> The chip reports an integer number of millimetres; the driver divides it
 *       by 10 for {@code CM}, 1000 for {@code METER} and 25.4 for {@code INCH}, and returns it unchanged
 *       for {@code MM}. This fake rounds the true distance to the nearest millimetre and divides the same
 *       way, so a reading is the same double a robot would print.</li>
 *   <li><b>Nothing in range is a large number, not the SDK's sentinel.</b> The REV 2m driver never returns
 *       {@link DistanceSensor#distanceOutOfRange}: it passes on whatever the chip reports, and with no
 *       target in range that is the chip's own "no target" value ({@link #REV_2M_NO_TARGET_MM}), which
 *       reads as roughly 819 cm or 322 in. Robot code that tests {@code == distanceOutOfRange} never sees
 *       a match on a real REV 2m, and in this sim it does not either. A sensor whose driver <em>does</em>
 *       return the sentinel is built with
 *       {@code new FakeDistanceSensor(maxRangeMm, DistanceSensor.distanceOutOfRange)}; the sentinel is then
 *       returned as it is in every unit, never divided, so the comparison works.</li>
 *   <li><b>Metadata</b>: device name {@code "REV 2M ToF Distance Sensor"} and manufacturer {@code Other},
 *       as the driver reports.</li>
 * </ul>
 *
 * <p><b>Not modelled</b>: noise, the sensor's field of view (a ray caster decides what it sees), the
 * target's reflectivity, the chip's timing budget and the driver's timeout (which reads 65535 mm). The
 * SDK's concrete {@code Rev2mDistanceSensor} class is not faked, so {@code hardwareMap.get(
 * Rev2mDistanceSensor.class, name)} does not resolve; robot code that asks for the {@code DistanceSensor}
 * interface, as the SDK's own sample does, works.
 *
 * <p>Deterministic: no clock, no randomness. A fresh sensor sees nothing in range.
 */
public class FakeDistanceSensor extends AbstractFakeDevice implements DistanceSensor {

    /**
     * The farthest target a REV 2m sees, millimetres. SPEC: REV's product name and datasheet range, 2 m, in
     * the driver's default long-range mode. A real sensor's range falls short of this on a dark or angled
     * target; that is not modelled.
     */
    public static final double REV_2M_MAX_RANGE_MM = 2000.0;

    /**
     * What a REV 2m reports with nothing in range, millimetres. NEEDS MEASUREMENT (2026-09-28): 8190 mm is
     * the VL53L0X "no target" reading FTC teams widely report from this sensor (819 cm, 322.4 in); it has
     * not been read off this team's sensor. The SDK driver passes the chip's number straight through, so
     * whatever the chip sends is what robot code sees.
     */
    public static final double REV_2M_NO_TARGET_MM = 8190.0;

    /** Millimetres per inch, the divisor the SDK driver uses for {@code INCH}. SPEC (a unit definition). */
    private static final double MM_PER_INCH = 25.4;

    private final double maxRangeMm;
    private final double outOfRangeReadingMm;

    /** The true distance to the target, millimetres; {@code +infinity} when there is nothing to see. */
    private double trueDistanceMm = Double.POSITIVE_INFINITY;
    private DistanceUnit sourceUnit;
    private DoubleSupplier distanceSource;

    /** A REV 2m Distance Sensor: {@link #REV_2M_MAX_RANGE_MM} range, {@link #REV_2M_NO_TARGET_MM} when nothing is in range. */
    public FakeDistanceSensor() {
        this(REV_2M_MAX_RANGE_MM, REV_2M_NO_TARGET_MM);
    }

    /**
     * A range sensor with its own range and its own "nothing in range" reading.
     *
     * @param maxRangeMm          the farthest target the sensor reports, millimetres (&gt; 0); anything
     *                            farther reads as {@code outOfRangeReadingMm}
     * @param outOfRangeReadingMm what the sensor reports with nothing in range, millimetres, or
     *                            {@link DistanceSensor#distanceOutOfRange} for a driver that returns the
     *                            SDK's sentinel (returned unconverted in every unit)
     * @throws IllegalArgumentException if {@code maxRangeMm} is not positive and finite, or
     *                                  {@code outOfRangeReadingMm} is NaN or negative
     */
    public FakeDistanceSensor(double maxRangeMm, double outOfRangeReadingMm) {
        if (!Double.isFinite(maxRangeMm) || maxRangeMm <= 0.0) {
            throw new IllegalArgumentException("maxRangeMm must be positive and finite, got " + maxRangeMm);
        }
        if (Double.isNaN(outOfRangeReadingMm) || outOfRangeReadingMm < 0.0) {
            throw new IllegalArgumentException("outOfRangeReadingMm must be a non-negative number, got "
                    + outOfRangeReadingMm);
        }
        this.maxRangeMm = maxRangeMm;
        this.outOfRangeReadingMm = outOfRangeReadingMm;
    }

    /**
     * Put a target {@code distance} away from the sensor's face, from now on. Clears any source.
     * <b>Sim-only</b>: a real {@code DistanceSensor} has no such method.
     *
     * @param unit     the unit {@code distance} is in
     * @param distance how far the target is; {@code +infinity} means nothing is there, and anything past
     *                 the sensor's range reads as out of range
     * @throws IllegalArgumentException if {@code unit} is null, or {@code distance} is NaN or negative
     */
    public void setDistance(DistanceUnit unit, double distance) {
        this.distanceSource = null;
        this.sourceUnit = null;
        this.trueDistanceMm = toMillimetres(unit, distance);
    }

    /** Take every target away from now on, so the sensor reports out of range. Clears any source. */
    public void setNothingInRange() {
        setDistance(DistanceUnit.MM, Double.POSITIVE_INFINITY);
    }

    /**
     * Ask {@code source} how far the target is on every {@link #update(double)} from now on. Replaces any
     * earlier source and, from the next tick, any scripted value. Pass a null source to stop asking; the
     * sensor then keeps the last distance the source gave.
     *
     * @param unit   the unit the source answers in
     * @param source what decides how far the target is, asked once per harness tick; it may answer
     *               {@code +infinity} for nothing in view, never NaN or a negative number. May be null
     * @throws IllegalArgumentException if {@code source} is not null and {@code unit} is
     */
    public void setDistanceSource(DistanceUnit unit, DoubleSupplier source) {
        if (source != null && unit == null) {
            throw new IllegalArgumentException("a distance source needs the unit it answers in");
        }
        this.sourceUnit = unit;
        this.distanceSource = source;
    }

    /**
     * Samples the source set with {@link #setDistanceSource(DistanceUnit, DoubleSupplier)}, if there is
     * one. A range sensor has no dynamics of its own.
     *
     * @param deltaTime how far to advance, in seconds; unused
     * @throws IllegalArgumentException if the source answers NaN or a negative distance
     */
    @Override
    public void update(double deltaTime) {
        if (distanceSource != null) {
            trueDistanceMm = toMillimetres(sourceUnit, distanceSource.getAsDouble());
        }
    }

    /** {@return true when the current target is within the sensor's range, so a reading is a real distance} */
    public boolean isTargetInRange() {
        return trueDistanceMm <= maxRangeMm;
    }

    /** {@return the farthest target the sensor reports, millimetres} */
    public double getMaxRangeMm() {
        return maxRangeMm;
    }

    /** {@return what the sensor reports with nothing in range, millimetres, or the SDK's sentinel} */
    public double getOutOfRangeReadingMm() {
        return outOfRangeReadingMm;
    }

    // ---- DistanceSensor ----

    /**
     * The distance the sensor reports: the true distance rounded to whole millimetres, or the out-of-range
     * reading when nothing is within range, converted the way the SDK's VL53L0X driver converts it (see the
     * class javadoc).
     *
     * @param unit the unit to report in
     * @return the reading in {@code unit}; the SDK's {@link DistanceSensor#distanceOutOfRange} unconverted
     *         if this sensor was built to report it
     */
    @Override
    public double getDistance(DistanceUnit unit) {
        if (!isTargetInRange() && outOfRangeReadingMm == distanceOutOfRange) {
            return distanceOutOfRange;
        }
        double millimetres = isTargetInRange() ? Math.rint(trueDistanceMm) : outOfRangeReadingMm;
        if (unit == DistanceUnit.CM) {
            return millimetres / 10.0;
        }
        if (unit == DistanceUnit.METER) {
            return millimetres / 1000.0;
        }
        if (unit == DistanceUnit.INCH) {
            return millimetres / MM_PER_INCH;
        }
        return millimetres;
    }

    // ---- HardwareDevice: matched to Rev2mDistanceSensor rather than to AbstractFakeDevice's defaults ----

    /** {@return {@code "REV 2M ToF Distance Sensor"}, the string {@code Rev2mDistanceSensor.getDeviceName} returns} */
    @Override
    public String getDeviceName() {
        return "REV 2M ToF Distance Sensor";
    }

    /** {@return {@code 1}, as the SDK's I2C device drivers report} */
    @Override
    public int getVersion() {
        return 1;
    }

    private static double toMillimetres(DistanceUnit unit, double distance) {
        if (unit == null) {
            throw new IllegalArgumentException("a distance needs a unit");
        }
        if (Double.isNaN(distance) || distance < 0.0) {
            throw new IllegalArgumentException("a distance must be zero or more (or +infinity for nothing in"
                    + " view), got " + distance);
        }
        if (distance == Double.POSITIVE_INFINITY) {
            return Double.POSITIVE_INFINITY;
        }
        return unit.toMm(distance);
    }
}
