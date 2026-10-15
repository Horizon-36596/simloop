package org.horizon36596.simloop.sensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.horizon36596.simloop.fakehardware.ColorCounts;
import org.horizon36596.simloop.fakehardware.FakeRevColorSensorV3;
import org.horizon36596.simloop.field.GamePiece;

import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/**
 * Binds a colour sensor to the simulated field: finds the game piece in front of a {@link SensorMount} and
 * makes a {@link FakeRevColorSensorV3} see that piece's colour at that piece's distance. The device fake
 * stays a register file; this class decides what is in front of it.
 *
 * <pre>{@code
 * Map<String, ColorCounts> colours = new LinkedHashMap<>();
 * colours.put("purple", new ColorCounts(900, 700, 1800));   // MEASURED on the robot, with a date
 * colours.put("green", new ColorCounts(300, 2100, 900));
 * ColorProbe slot = new ColorProbe(new SensorMount(6.0, 0.0, 4.0, 0.0, Math.PI / 2), 2.0,
 *         Math.toRadians(30), colours, ColorCounts.DARK);
 * // each tick:
 * slot.apply(slotColorSensor, tracker.getPieces(), x, y, heading);
 * }</pre>
 *
 * <p><b>What "in view" means.</b> A piece is in view when it is not held, the angle between the mount's
 * direction and the line to the piece's centre is at most {@code halfFieldOfViewRad}, and the gap from the
 * sensor face to the piece's surface is at most {@code maxRangeInches}. The piece in view with the smallest
 * gap is the one seen; an exact tie goes to the lowest id. Nothing else occludes.
 *
 * <p><b>What the sensor then reads.</b> The piece's counts are looked up by its
 * {@linkplain GamePiece#getKind() kind}, and the gap becomes the sensor's distance. With nothing in view the
 * sensor sees the {@code background} counts and nothing in range. PLACEHOLDER: a piece's counts do not
 * change with distance or angle, which they do on a real sensor; measure them at the distance the piece
 * really sits from the sensor, which is the one that matters for an intake.
 *
 * <p><b>A held piece is not in view</b>, because the tracker parks a held piece at the robot's centre. A
 * sensor that looks at a piece inside the robot is the season's business: it knows which slot holds which
 * piece, and shows it with {@link #show(FakeRevColorSensorV3, GamePiece, double)}.
 *
 * <p>Immutable once built. Deterministic and clock-free; all trigonometry in {@link StrictMath}.
 */
public final class ColorProbe {

    private final SensorMount mount;
    private final double maxRangeInches;
    private final double halfFieldOfViewRad;
    private final Map<String, ColorCounts> countsByKind;
    private final ColorCounts background;

    /**
     * A probe looking out of one mount.
     *
     * @param mount              where the sensor is on the robot and which way it looks
     * @param maxRangeInches     the largest gap from the sensor face to a piece's surface that still counts
     *                           as in view, inches (&gt; 0)
     * @param halfFieldOfViewRad half the sensor's viewing cone, radians, in {@code (0, π]}
     * @param countsByKind       the raw counts each kind of piece reads as, by kind; copied
     * @param background         what the sensor reads with no piece in view
     * @throws IllegalArgumentException if anything is null, a count in the map is null, the range is not
     *                                  positive and finite, or the half-angle is outside {@code (0, π]}
     */
    public ColorProbe(SensorMount mount, double maxRangeInches, double halfFieldOfViewRad,
            Map<String, ColorCounts> countsByKind, ColorCounts background) {
        if (mount == null || countsByKind == null || background == null) {
            throw new IllegalArgumentException("a colour probe needs a mount, a colour for each kind and a"
                    + " background");
        }
        if (!Double.isFinite(maxRangeInches) || maxRangeInches <= 0.0) {
            throw new IllegalArgumentException("maxRangeInches must be positive and finite, got " + maxRangeInches);
        }
        if (!(halfFieldOfViewRad > 0.0 && halfFieldOfViewRad <= Math.PI)) {
            throw new IllegalArgumentException("halfFieldOfViewRad must be in (0, pi], got " + halfFieldOfViewRad);
        }
        Map<String, ColorCounts> copy = new LinkedHashMap<>();
        for (Map.Entry<String, ColorCounts> entry : countsByKind.entrySet()) {
            if (entry.getKey() == null || entry.getValue() == null) {
                throw new IllegalArgumentException("every kind needs a name and counts, got " + entry);
            }
            copy.put(entry.getKey(), entry.getValue());
        }
        this.mount = mount;
        this.maxRangeInches = maxRangeInches;
        this.halfFieldOfViewRad = halfFieldOfViewRad;
        this.countsByKind = Collections.unmodifiableMap(copy);
        this.background = background;
    }

    /** What the probe found: a piece and how far its surface is, or nothing. Immutable. */
    public static final class Sighting {

        /** The one empty sighting, shared. */
        static final Sighting NOTHING = new Sighting(null, Double.POSITIVE_INFINITY);

        private final GamePiece piece;
        private final double gapInches;

        Sighting(GamePiece piece, double gapInches) {
            this.piece = piece;
            this.gapInches = gapInches;
        }

        /** {@return true when a piece is in view} */
        public boolean isPieceInView() {
            return piece != null;
        }

        /** {@return the piece in view, or null when there is none} */
        public GamePiece getPiece() {
            return piece;
        }

        /** {@return the gap from the sensor face to the piece's surface, inches; {@code +infinity} with nothing in view} */
        public double getGapInches() {
            return gapInches;
        }

        /** {@return the sighting, for test messages} */
        @Override
        public String toString() {
            return piece == null ? "Sighting(nothing)" : "Sighting(" + piece + " at " + gapInches + " in)";
        }
    }

    /**
     * Finds the piece in view for a robot at the given pose.
     *
     * @param pieces          every piece, from {@code GamePieceTracker.getPieces()}; held pieces are skipped
     * @param robotXInches    robot centre, field frame, inches
     * @param robotYInches    robot centre, field frame, inches
     * @param robotHeadingRad robot heading, radians CCW from field +X
     * @return the nearest piece in view, or an empty sighting
     * @throws IllegalArgumentException if {@code pieces} is null or the pose is not finite
     */
    public Sighting look(List<GamePiece> pieces, double robotXInches, double robotYInches,
            double robotHeadingRad) {
        if (pieces == null) {
            throw new IllegalArgumentException("a colour probe needs the piece list");
        }
        SensorMount.requireFinite(robotXInches, "robotXInches");
        SensorMount.requireFinite(robotYInches, "robotYInches");
        SensorMount.requireFinite(robotHeadingRad, "robotHeadingRad");

        double[] face = mount.fieldPosition(robotXInches, robotYInches, robotHeadingRad);
        double[] direction = mount.fieldDirection(robotHeadingRad);
        double cosHalfAngle = StrictMath.cos(halfFieldOfViewRad);

        Sighting best = Sighting.NOTHING;
        for (GamePiece piece : pieces) {
            if (!piece.isVisibleOnField()) {
                continue;
            }
            double dx = piece.getFieldXInches() - face[0];
            double dy = piece.getFieldYInches() - face[1];
            double dz = piece.getFieldZInches() - face[2];
            double centreDistance = StrictMath.sqrt(dx * dx + dy * dy + dz * dz);
            double gap = Math.max(0.0, centreDistance - piece.getRadiusInches());
            if (gap > maxRangeInches) {
                continue;
            }
            // Inside the piece, every direction points at it; otherwise compare the cosine of the angle
            // off the mount's axis with the cone's.
            boolean inCone = centreDistance <= piece.getRadiusInches()
                    || (dx * direction[0] + dy * direction[1] + dz * direction[2]) / centreDistance >= cosHalfAngle;
            if (inCone && gap < best.getGapInches()) {
                best = new Sighting(piece, gap);
            }
        }
        return best;
    }

    /**
     * Looks, then makes the sensor see what the probe found: the piece's counts at its gap, or the
     * background with nothing in range.
     *
     * @param sensor          the fake to drive
     * @param pieces          every piece; held pieces are skipped
     * @param robotXInches    robot centre, field frame, inches
     * @param robotYInches    robot centre, field frame, inches
     * @param robotHeadingRad robot heading, radians CCW from field +X
     * @return the sighting, so a test can check what the sensor saw
     * @throws IllegalArgumentException if {@code sensor} or {@code pieces} is null, or the pose is not finite
     * @throws IllegalStateException    if the piece in view is of a kind this probe has no counts for
     */
    public Sighting apply(FakeRevColorSensorV3 sensor, List<GamePiece> pieces, double robotXInches,
            double robotYInches, double robotHeadingRad) {
        if (sensor == null) {
            throw new IllegalArgumentException("a colour probe needs a sensor to drive");
        }
        Sighting sighting = look(pieces, robotXInches, robotYInches, robotHeadingRad);
        if (sighting.isPieceInView()) {
            show(sensor, sighting.getPiece(), sighting.getGapInches());
        } else {
            sensor.setRawCounts(background);
            sensor.setNothingInRange();
        }
        return sighting;
    }

    /**
     * Makes the sensor see one piece at a given gap, whatever the geometry says - for a sensor that looks at
     * a piece the robot is holding, which the tracker does not place in front of any mount.
     *
     * @param sensor    the fake to drive
     * @param piece     the piece the sensor sees
     * @param gapInches the gap from the sensor face to the piece's surface, inches (&ge; 0)
     * @throws IllegalArgumentException if {@code sensor} or {@code piece} is null, or the gap is NaN or
     *                                  negative
     * @throws IllegalStateException    if this probe has no counts for the piece's kind
     */
    public void show(FakeRevColorSensorV3 sensor, GamePiece piece, double gapInches) {
        if (sensor == null || piece == null) {
            throw new IllegalArgumentException("show needs a sensor and a piece");
        }
        ColorCounts counts = countsFor(piece.getKind());
        sensor.setRawCounts(counts);
        sensor.setDistance(DistanceUnit.INCH, gapInches);
    }

    /**
     * The counts a kind of piece reads as.
     *
     * @param kind the piece's kind, as {@link GamePiece#getKind()} reports it
     * @return the counts given for that kind
     * @throws IllegalStateException if this probe was given no counts for {@code kind} - a season that forgot
     *                               a kind should hear about it rather than see a dark sensor
     */
    public ColorCounts countsFor(String kind) {
        ColorCounts counts = countsByKind.get(kind);
        if (counts == null) {
            throw new IllegalStateException("no colour counts for piece kind \"" + kind + "\"; this probe knows "
                    + countsByKind.keySet());
        }
        return counts;
    }

    /** {@return the mount the probe looks out of} */
    public SensorMount getMount() {
        return mount;
    }

    /** {@return the largest gap to a piece's surface that still counts as in view, inches} */
    public double getMaxRangeInches() {
        return maxRangeInches;
    }

    /** {@return half the viewing cone, radians} */
    public double getHalfFieldOfViewRad() {
        return halfFieldOfViewRad;
    }

    /** {@return what the sensor reads with no piece in view} */
    public ColorCounts getBackground() {
        return background;
    }
}
