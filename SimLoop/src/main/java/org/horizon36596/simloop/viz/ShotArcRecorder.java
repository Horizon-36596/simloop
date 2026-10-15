package org.horizon36596.simloop.viz;

import org.psilynx.psikit.core.Logger;
import org.psilynx.psikit.core.wpi.math.Pose3d;
import org.psilynx.psikit.core.wpi.math.Rotation3d;
import org.psilynx.psikit.core.wpi.math.Translation3d;

/**
 * Logs the last few shots as intended-vs-actual arc pairs, in a shape AdvantageScope's 3D Field can draw
 * without any setup beyond adding the keys.
 *
 * <p><b>Why fixed slots.</b> AdvantageScope draws a {@code Translation3d[]} trajectory as <i>one</i>
 * connected line, so ten shots packed into one array come out as one zig-zag. Each shot therefore gets
 * its own key, and the keys are a fixed ring of {@code slots}: shot 0 goes in slot 0, shot {@code slots}
 * overwrites it. A layout written once keeps working for the whole match.
 *
 * <p><b>Keys</b>, under the prefix given to {@link #record(String)} (PsiKit files them beneath
 * {@code RealOutputs/}):
 * <pre>
 *   &lt;prefix&gt;/intended/&lt;slot&gt;    Translation3d[]  the arc the shot was meant to fly
 *   &lt;prefix&gt;/actual/&lt;slot&gt;      Translation3d[]  the arc it flew (or is flying - it grows while in flight)
 *   &lt;prefix&gt;/missInches/&lt;slot&gt;  double           closest approach of actual to the aim point; NaN until it has a segment
 *   &lt;prefix&gt;/aimPoints        Pose3d[]         one per filled slot, for an Axes or Cone marker
 *   &lt;prefix&gt;/shotCount        int              shots fired since construction
 *   &lt;prefix&gt;/lastSlot         int              the slot the newest shot is in; -1 before the first
 * </pre>
 * Every slot is logged every call, empty ones as empty arrays, so a key never appears mid-log and a
 * layout can name all of them up front. AdvantageScope hides a trajectory of one point or fewer.
 *
 * <p><b>Frame and units.</b> Inches in, field frame (see {@link ShotArc}); metres out.
 *
 * <p><b>Determinism.</b> No clock: a shot is fired when the caller says, and its actual arc grows when the
 * caller adds to it, so a replay logs the same arrays on the same ticks.
 */
public final class ShotArcRecorder {

    private static final double METRES_PER_INCH = 0.0254;

    private final ShotArc[] intended;
    private final ShotArc[] actual;
    private final double[] aimXyz;
    private int shotCount;
    private int lastSlot = -1;

    /**
     * A recorder with no shots yet.
     *
     * @param slots how many recent shots stay drawn; at least 1. Three is enough to see a trend without
     *              the field filling up with lines
     * @throws IllegalArgumentException if {@code slots} is less than 1
     */
    public ShotArcRecorder(int slots) {
        if (slots < 1) {
            throw new IllegalArgumentException("a shot recorder needs at least one slot, got " + slots);
        }
        intended = new ShotArc[slots];
        actual = new ShotArc[slots];
        aimXyz = new double[3 * slots];
    }

    /** {@return how many slots this recorder cycles through} */
    public int slots() {
        return intended.length;
    }

    /**
     * Starts a new shot in the next slot, replacing whatever was there.
     *
     * @param intendedArc the arc the shot is meant to fly — usually {@link ShotArc#ballistic} of the
     *                    commanded launch, stopped at the aim height
     * @param aimX the point the shot is aimed at, x, inches, field frame
     * @param aimY aim point y, inches, field frame
     * @param aimZ aim point height, inches
     * @return the slot the shot went into
     * @throws IllegalArgumentException if {@code intendedArc} is null
     */
    public int fire(ShotArc intendedArc, double aimX, double aimY, double aimZ) {
        if (intendedArc == null) {
            throw new IllegalArgumentException("a fired shot needs an intended arc");
        }
        int slot = shotCount % intended.length;
        intended[slot] = intendedArc;
        actual[slot] = null;
        aimXyz[3 * slot] = aimX;
        aimXyz[3 * slot + 1] = aimY;
        aimXyz[3 * slot + 2] = aimZ;
        shotCount++;
        lastSlot = slot;
        return slot;
    }

    /**
     * Sets the newest shot's actual arc, whole or so far. Call it every tick with a growing
     * {@link ShotArc#prefix} to draw the shot in flight, or once with the full arc.
     *
     * @param actualArc the arc the piece flew
     * @throws IllegalStateException if no shot has been fired yet
     * @throws IllegalArgumentException if {@code actualArc} is null
     */
    public void setActual(ShotArc actualArc) {
        if (lastSlot < 0) {
            throw new IllegalStateException("setActual called before any shot was fired");
        }
        if (actualArc == null) {
            throw new IllegalArgumentException("an actual arc cannot be null; leave it unset instead");
        }
        actual[lastSlot] = actualArc;
    }

    /**
     * The newest shot's miss distance, for an assertion or a scorer: the actual arc's closest approach to
     * the aim point.
     *
     * @return inches, or {@link Double#NaN} if there is no shot or its actual arc has fewer than two points
     */
    public double lastMissInches() {
        return lastSlot < 0 ? Double.NaN : missInches(lastSlot);
    }

    /**
     * Writes every slot to the log. Call it once per tick, after the shot state for that tick is set.
     *
     * @param keyPrefix the log key prefix, e.g. {@code "Shots"}; non-blank
     * @throws IllegalArgumentException if {@code keyPrefix} is null or blank
     */
    public void record(String keyPrefix) {
        if (keyPrefix == null || keyPrefix.trim().isEmpty()) {
            throw new IllegalArgumentException("shot log key prefix must be a non-blank string");
        }
        String prefix = keyPrefix.trim();
        int filled = Math.min(shotCount, intended.length);
        Pose3d[] aims = new Pose3d[filled];
        for (int slot = 0; slot < intended.length; slot++) {
            Logger.recordOutput(prefix + "/intended/" + slot,
                    intended[slot] == null ? new Translation3d[0] : intended[slot].toTranslations());
            Logger.recordOutput(prefix + "/actual/" + slot,
                    actual[slot] == null ? new Translation3d[0] : actual[slot].toTranslations());
            Logger.recordOutput(prefix + "/missInches/" + slot, missInches(slot));
            if (slot < filled) {
                aims[slot] = new Pose3d(aimXyz[3 * slot] * METRES_PER_INCH, aimXyz[3 * slot + 1] * METRES_PER_INCH,
                        aimXyz[3 * slot + 2] * METRES_PER_INCH, new Rotation3d());
            }
        }
        Logger.recordOutput(prefix + "/aimPoints", aims);
        Logger.recordOutput(prefix + "/shotCount", shotCount);
        Logger.recordOutput(prefix + "/lastSlot", lastSlot);
    }

    private double missInches(int slot) {
        ShotArc arc = actual[slot];
        if (arc == null || arc.size() < 2) {
            return Double.NaN;
        }
        return arc.closestApproachInches(aimXyz[3 * slot], aimXyz[3 * slot + 1], aimXyz[3 * slot + 2]);
    }
}
