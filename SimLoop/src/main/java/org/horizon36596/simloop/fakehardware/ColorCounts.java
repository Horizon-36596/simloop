package org.horizon36596.simloop.fakehardware;

/**
 * What a colour sensor's photodiodes measure: one raw count per channel, the numbers the chip puts in its
 * light-sensor data registers before any driver arithmetic. For a REV Color Sensor V3 (a Broadcom
 * APDS-9151 at the SDK's 16-bit resolution) each count is {@code 0..65535}.
 *
 * <p><b>Why raw counts and not a colour.</b> What robot code reads from a colour sensor is the SDK
 * driver's arithmetic on these counts: a per-channel correction, the software gain, a division by the
 * saturation count and a clip. {@link FakeRevColorSensorV3} feeds these counts to the SDK's own driver so
 * that arithmetic is the SDK's, not a copy. It also means the right numbers for "a red ball in the intake"
 * are the counts a real sensor measured on one - how bright a piece looks depends on its paint, the
 * distance and the LED, none of which the sim knows. Measure them and write them down with their
 * provenance; do not reason them out.
 *
 * <p>Immutable value; {@link #equals(Object)} compares the three counts.
 */
public final class ColorCounts {

    /** The largest count a channel holds at the SDK's 16-bit light-sensor resolution. SPEC. */
    public static final int MAX_COUNT = 65535;

    /** No light on any channel. */
    public static final ColorCounts DARK = new ColorCounts(0, 0, 0);

    private final int red;
    private final int green;
    private final int blue;

    /**
     * One reading's three raw counts.
     *
     * @param red   the red channel's raw count, {@code 0..}{@link #MAX_COUNT}
     * @param green the green channel's raw count, {@code 0..}{@link #MAX_COUNT}
     * @param blue  the blue channel's raw count, {@code 0..}{@link #MAX_COUNT}
     * @throws IllegalArgumentException if any count is outside {@code 0..}{@link #MAX_COUNT}
     */
    public ColorCounts(int red, int green, int blue) {
        this.red = requireCount(red, "red");
        this.green = requireCount(green, "green");
        this.blue = requireCount(blue, "blue");
    }

    /** {@return the red channel's raw count, unitless, {@code 0..65535}} */
    public int getRed() {
        return red;
    }

    /** {@return the green channel's raw count, unitless, {@code 0..65535}} */
    public int getGreen() {
        return green;
    }

    /** {@return the blue channel's raw count, unitless, {@code 0..65535}} */
    public int getBlue() {
        return blue;
    }

    /**
     * {@inheritDoc}
     *
     * @return true when {@code other} is a {@code ColorCounts} with the same three counts
     */
    @Override
    public boolean equals(Object other) {
        if (!(other instanceof ColorCounts)) {
            return false;
        }
        ColorCounts that = (ColorCounts) other;
        return red == that.red && green == that.green && blue == that.blue;
    }

    /** {@return a hash of the three counts, consistent with {@link #equals(Object)}} */
    @Override
    public int hashCode() {
        return (red * 31 + green) * 31 + blue;
    }

    /** {@return the counts, for test messages} */
    @Override
    public String toString() {
        return "ColorCounts(r=" + red + " g=" + green + " b=" + blue + ")";
    }

    private static int requireCount(int count, String channel) {
        if (count < 0 || count > MAX_COUNT) {
            throw new IllegalArgumentException(channel + " count must be 0.." + MAX_COUNT + ", got " + count);
        }
        return count;
    }
}
