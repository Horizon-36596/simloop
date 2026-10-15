package org.horizon36596.simloop.viz;

import org.psilynx.psikit.core.wpi.math.Translation3d;

import java.util.Arrays;

/**
 * An immutable 3-D polyline for one shot: the path a game piece took, or was meant to take, from the
 * launcher to wherever it stopped. It exists so a shot can be <i>drawn</i> — AdvantageScope's 3D Field
 * draws a {@code Translation3d[]} logged as a <i>Trajectory</i> as one connected line — and so two arcs
 * for the same shot (intended and actual) can be compared by a number, not just by eye.
 *
 * <p><b>Frame and units.</b> Same as {@code field.GamePieceTracker}: the FTC-Cartesian field frame, +X
 * right, +Y forward, origin at the field centre, +Z up from the tile top, in <b>inches</b>.
 * {@link #toTranslations()} is the only place inches become metres, because that is what AdvantageScope
 * reads.
 *
 * <p><b>What this models, and what it refuses to.</b> {@link #ballistic} is a point mass under gravity
 * with optional quadratic drag — no spin, no lift, no collisions. It is the arc a shot <i>should</i>
 * follow given a launch state, which is exactly what an intended-vs-actual comparison needs on both
 * sides: the intended arc is {@code ballistic} of the commanded launch, and on a real robot the actual
 * arc is {@code ballistic} of the <i>measured</i> launch (flywheel speed, hood angle). In a sim that has
 * real game-piece physics, the actual arc is the sampled positions instead, via {@link #ofPoints}.
 *
 * <p><b>Determinism.</b> No clock, no randomness, and the arithmetic uses {@link StrictMath}, so the same
 * launch produces the same points on every JVM.
 */
public final class ShotArc {

    /** Standard gravity, inches per second squared (9.80665 m/s² ÷ 0.0254). */
    public static final double GRAVITY_IN_PER_S2 = 9.80665 / 0.0254;

    private static final double METRES_PER_INCH = 0.0254;

    /** Integration step for {@link #ballistic}, seconds. One millisecond keeps Euler error well under 0.1 in. */
    private static final double INTEGRATION_STEP_S = 0.001;

    /** The longest flight {@link #ballistic} will integrate, seconds. A shot still airborne after this is a bad launch. */
    private static final double MAX_FLIGHT_S = 5.0;

    /** x, y, z interleaved, inches, field frame. Never shared outside this object. */
    private final double[] xyz;

    private ShotArc(double[] xyz) {
        this.xyz = xyz;
    }

    /**
     * Builds an arc from points already known — sampled game-piece positions out of a physics sim, or a
     * tracked ball out of a camera.
     *
     * @param xyzInches x, y, z interleaved, inches, field frame; at least one point
     * @return the arc, holding its own copy of the points
     * @throws IllegalArgumentException if the array is null, empty, not a multiple of three, or not finite
     */
    public static ShotArc ofPoints(double... xyzInches) {
        if (xyzInches == null || xyzInches.length == 0 || xyzInches.length % 3 != 0) {
            throw new IllegalArgumentException("a shot arc needs x,y,z triples, got "
                    + (xyzInches == null ? "null" : xyzInches.length + " numbers"));
        }
        for (double v : xyzInches) {
            if (!Double.isFinite(v)) {
                throw new IllegalArgumentException("a shot arc point is not finite: " + v);
            }
        }
        return new ShotArc(Arrays.copyOf(xyzInches, xyzInches.length));
    }

    /**
     * Integrates a point-mass flight from a launch state until the piece comes down through
     * {@code stopZInches} (or {@link #MAX_FLIGHT_S} passes), emitting one point every
     * {@code sampleSeconds} plus the exact crossing point.
     *
     * <p>Drag is {@code a = -k·|v|·v}, with {@code k = ½·ρ·Cd·A / m} in 1/inch. A 2.8-in foam ball is
     * roughly 0.001–0.002; zero gives the textbook parabola.
     *
     * <p><b>Stopping.</b> The flight ends the first time the piece comes <i>down through</i>
     * {@code stopZInches} from above, so a launch from below the stop height climbs through it first. A
     * flight that never gets above it ends at the floor (z = 0) instead. Pass the target's height to end
     * an intended arc at the target plane, or 0 to end at the floor.
     *
     * @param launch where the piece leaves the launcher and how fast, field frame
     * @param dragPerInch quadratic drag coefficient {@code k}, 1/inch; zero or positive
     * @param stopZInches the height, inches, the flight ends at on the way down
     * @param sampleSeconds how often a point is emitted, seconds; positive. One sim tick is the natural choice,
     *                      because then point {@code i} is where the piece is {@code i} ticks after launch
     * @return the arc, first point at the launch position
     * @throws IllegalArgumentException if {@code dragPerInch} is negative or {@code sampleSeconds} is not positive
     */
    public static ShotArc ballistic(Launch launch, double dragPerInch, double stopZInches, double sampleSeconds) {
        if (!(dragPerInch >= 0.0)) {
            throw new IllegalArgumentException("drag coefficient must be zero or positive, got " + dragPerInch);
        }
        if (!(sampleSeconds > 0.0)) {
            throw new IllegalArgumentException("sample interval must be positive seconds, got " + sampleSeconds);
        }
        int stepsPerSample = Math.max(1, (int) StrictMath.round(sampleSeconds / INTEGRATION_STEP_S));
        double dt = sampleSeconds / stepsPerSample;
        int maxSteps = (int) StrictMath.ceil(MAX_FLIGHT_S / dt);

        double x = launch.xInches;
        double y = launch.yInches;
        double z = launch.zInches;
        double vx = launch.vxInchesPerSecond();
        double vy = launch.vyInchesPerSecond();
        double vz = launch.vzInchesPerSecond();
        double floor = Math.min(0.0, stopZInches);

        double[] out = new double[3 * 64];
        int n = 0;
        out[n++] = x;
        out[n++] = y;
        out[n++] = z;

        for (int step = 1; step <= maxSteps; step++) {
            double speed = StrictMath.sqrt(vx * vx + vy * vy + vz * vz);
            double px = x;
            double py = y;
            double pz = z;
            // Semi-implicit Euler: velocity first, then position from the new velocity.
            vx += -dragPerInch * speed * vx * dt;
            vy += -dragPerInch * speed * vy * dt;
            vz += (-GRAVITY_IN_PER_S2 - dragPerInch * speed * vz) * dt;
            x += vx * dt;
            y += vy * dt;
            z += vz * dt;

            // Stop on a DOWNWARD crossing of the stop plane from above it. A flight that never gets above the
            // plane (a weak shot at a high target) falls through to the floor instead of stopping at its apex.
            double plane = Double.NaN;
            if (vz < 0.0 && pz > stopZInches && z <= stopZInches) {
                plane = stopZInches;
            } else if (vz < 0.0 && pz > floor && z <= floor) {
                plane = floor;
            }
            if (!Double.isNaN(plane)) {
                // Land the last point exactly on the plane, so an intended arc ends AT the target height.
                double f = (pz - plane) / (pz - z);
                out = ensure(out, n + 3);
                out[n++] = px + f * (x - px);
                out[n++] = py + f * (y - py);
                out[n++] = plane;
                return new ShotArc(Arrays.copyOf(out, n));
            }
            if (step % stepsPerSample == 0) {
                out = ensure(out, n + 3);
                out[n++] = x;
                out[n++] = y;
                out[n++] = z;
            }
        }
        return new ShotArc(Arrays.copyOf(out, n));
    }

    /**
     * The launch speed that puts a drag-free shot through a point, for a fixed launch pitch. This is the
     * textbook inverse, {@code v² = g·d² / (2·cos²θ·(d·tanθ − h))}; it is what an intended arc is solved
     * from when a shooter has a fixed or commanded hood angle.
     *
     * @param horizontalInches horizontal distance from launch point to target, inches; positive
     * @param riseInches target height minus launch height, inches
     * @param pitchRad launch elevation above horizontal, radians
     * @return the speed in inches per second, or {@link Double#NaN} if no speed at that pitch reaches the
     *         target (the target is above the line the pitch points along)
     */
    public static double speedToReach(double horizontalInches, double riseInches, double pitchRad) {
        double cos = StrictMath.cos(pitchRad);
        double denominator = 2.0 * cos * cos * (horizontalInches * StrictMath.tan(pitchRad) - riseInches);
        if (!(horizontalInches > 0.0) || !(denominator > 0.0)) {
            return Double.NaN;
        }
        return StrictMath.sqrt(GRAVITY_IN_PER_S2 * horizontalInches * horizontalInches / denominator);
    }

    /** {@return how many points the arc has} */
    public int size() {
        return xyz.length / 3;
    }

    /**
     * {@return point {@code i}'s x, inches, field frame}
     *
     * @param i the point's index, from 0 at the release
     */
    public double x(int i) {
        return xyz[3 * i];
    }

    /**
     * {@return point {@code i}'s y, inches, field frame}
     *
     * @param i the point's index, from 0 at the release
     */
    public double y(int i) {
        return xyz[3 * i + 1];
    }

    /**
     * {@return point {@code i}'s z, inches above the tile top}
     *
     * @param i the point's index, from 0 at the release
     */
    public double z(int i) {
        return xyz[3 * i + 2];
    }

    /** {@return the highest z on the arc, inches} */
    public double apexZInches() {
        double best = Double.NEGATIVE_INFINITY;
        for (int i = 0; i < size(); i++) {
            best = Math.max(best, z(i));
        }
        return best;
    }

    /**
     * The first {@code count} points of this arc — where the piece has got to {@code count − 1} samples
     * after launch. This is how an in-flight shot is drawn growing, tick by tick.
     *
     * @param count how many points to keep; clamped to {@code 1..size()}
     * @return the leading part of the arc
     */
    public ShotArc prefix(int count) {
        int keep = Math.max(1, Math.min(size(), count));
        return new ShotArc(Arrays.copyOf(xyz, 3 * keep));
    }

    /**
     * How close this arc comes to a point, measured to the nearest point on any segment. This is the
     * number an intended-vs-actual comparison reports: an actual arc that misses a cell by four inches
     * says {@code 4.0} here, whichever side it missed on.
     *
     * @param x the point's x, inches, field frame
     * @param y the point's y, inches, field frame
     * @param z the point's z, inches
     * @return the closest distance, inches
     */
    public double closestApproachInches(double x, double y, double z) {
        if (size() == 1) {
            return distance(x(0) - x, y(0) - y, z(0) - z);
        }
        double best = Double.POSITIVE_INFINITY;
        for (int i = 0; i + 1 < size(); i++) {
            double ax = x(i), ay = y(i), az = z(i);
            double dx = x(i + 1) - ax, dy = y(i + 1) - ay, dz = z(i + 1) - az;
            double lengthSquared = dx * dx + dy * dy + dz * dz;
            double t = lengthSquared == 0.0 ? 0.0
                    : ((x - ax) * dx + (y - ay) * dy + (z - az) * dz) / lengthSquared;
            t = Math.max(0.0, Math.min(1.0, t));
            best = Math.min(best, distance(ax + t * dx - x, ay + t * dy - y, az + t * dz - z));
        }
        return best;
    }

    /**
     * The arc as AdvantageScope reads it: metres, same axes. Log it with {@code Logger.recordOutput} and
     * add the key to a 3D Field tab as a <i>Trajectory</i>.
     *
     * @return one translation per point, in metres
     */
    public Translation3d[] toTranslations() {
        Translation3d[] out = new Translation3d[size()];
        for (int i = 0; i < out.length; i++) {
            out[i] = new Translation3d(x(i) * METRES_PER_INCH, y(i) * METRES_PER_INCH, z(i) * METRES_PER_INCH);
        }
        return out;
    }

    private static double distance(double dx, double dy, double dz) {
        return StrictMath.sqrt(dx * dx + dy * dy + dz * dz);
    }

    private static double[] ensure(double[] buffer, int length) {
        return length <= buffer.length ? buffer : Arrays.copyOf(buffer, Math.max(length, buffer.length * 2));
    }

    /**
     * Where a game piece leaves the launcher, and how fast: a position plus a speed along a direction.
     * Field frame, inches; angles in radians.
     */
    public static final class Launch {

        /** Release point x, inches, field frame. */
        public final double xInches;

        /** Release point y, inches, field frame. */
        public final double yInches;

        /** Release point height, inches above the tile top. */
        public final double zInches;

        /** Exit speed, inches per second. */
        public final double speedInchesPerSecond;

        /** Direction in the floor plane, radians CCW from field +X — the turret/heading bearing. */
        public final double yawRad;

        /** Elevation above horizontal, radians — the hood angle as an exit angle. */
        public final double pitchRad;

        /**
         * A launch from the given release point, speed and direction.
         *
         * @param xInches release x, inches, field frame
         * @param yInches release y, inches, field frame
         * @param zInches release height, inches
         * @param speedInchesPerSecond exit speed, inches per second; zero or positive
         * @param yawRad bearing, radians CCW from field +X
         * @param pitchRad elevation, radians above horizontal
         * @throws IllegalArgumentException if any value is not finite or the speed is negative
         */
        public Launch(double xInches, double yInches, double zInches,
                double speedInchesPerSecond, double yawRad, double pitchRad) {
            double[] all = {xInches, yInches, zInches, speedInchesPerSecond, yawRad, pitchRad};
            for (double v : all) {
                if (!Double.isFinite(v)) {
                    throw new IllegalArgumentException("launch values must be finite, got " + Arrays.toString(all));
                }
            }
            if (speedInchesPerSecond < 0.0) {
                throw new IllegalArgumentException("launch speed must be zero or positive, got " + speedInchesPerSecond);
            }
            this.xInches = xInches;
            this.yInches = yInches;
            this.zInches = zInches;
            this.speedInchesPerSecond = speedInchesPerSecond;
            this.yawRad = yawRad;
            this.pitchRad = pitchRad;
        }

        /**
         * {@return the same launch at a different exit speed — a flywheel that came up short, say}
         *
         * @param speedInchesPerSecond the new exit speed, inches per second; zero or positive
         */
        public Launch withSpeed(double speedInchesPerSecond) {
            return new Launch(xInches, yInches, zInches, speedInchesPerSecond, yawRad, pitchRad);
        }

        /**
         * {@return the same launch at a different elevation — a hood that settled off its setpoint}
         *
         * @param pitchRad the new elevation above horizontal, radians
         */
        public Launch withPitch(double pitchRad) {
            return new Launch(xInches, yInches, zInches, speedInchesPerSecond, yawRad, pitchRad);
        }

        double vxInchesPerSecond() {
            return speedInchesPerSecond * StrictMath.cos(pitchRad) * StrictMath.cos(yawRad);
        }

        double vyInchesPerSecond() {
            return speedInchesPerSecond * StrictMath.cos(pitchRad) * StrictMath.sin(yawRad);
        }

        double vzInchesPerSecond() {
            return speedInchesPerSecond * StrictMath.sin(pitchRad);
        }
    }
}
