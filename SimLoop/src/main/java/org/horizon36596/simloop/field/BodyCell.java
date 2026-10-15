package org.horizon36596.simloop.field;

/**
 * A compartment of a {@link KinematicBody} that game pieces can come to rest in and be counted in: a box in the
 * body's own frame. Unlike a {@link ScoringVolume}, nothing is parked here. A piece is <i>in</i> the cell while
 * its centre is inside the box, wherever the physics left it, and it leaves the cell the moment the physics
 * carries it out - which is how a goal that tips its contents out is modelled without a scripted spill.
 *
 * <p><b>Frame and units.</b> The owning body's local frame (see {@link KinematicBody}), inches. The box's local
 * +Z face is its top: the open side a piece falls in through.
 *
 * <p><b>Getting in, staying in.</b> A piece that is not yet in the cell counts as having entered only once its
 * centre is inside the box <i>and</i> at least {@link #getSeatDepthInches()} below the top face - so a ball that
 * skims the rim and bounces off is never counted. Once in, it stays in until its centre leaves the box itself
 * (seat depth zero), so a piece settling a little does not flicker in and out.
 *
 * <p>Season-agnostic (domain R7). Immutable.
 */
public final class BodyCell {

    private final String name;
    private final double minX;
    private final double maxX;
    private final double minY;
    private final double maxY;
    private final double minZ;
    private final double maxZ;
    private final double seatDepthInches;

    /**
     * Creates a cell from its name, its box in the body's frame and its seat depth.
     *
     * @param name            the cell's name: the cause logged when a piece comes to rest in it, and the name
     *                        {@link GamePieceTracker#getScoredCountIn} counts it by. Unique across the world's
     *                        cells and scoring volumes.
     * @param minX            the box, inches, in the owning body's local frame; each min below its max
     * @param maxX            ditto
     * @param minY            ditto
     * @param maxY            ditto
     * @param minZ            ditto
     * @param maxZ            ditto; the top face
     * @param seatDepthInches how far below the top face a piece's centre must be to count as entered, inches;
     *                        0 or more and less than the box's height
     * @throws IllegalArgumentException on a blank name, a non-finite number or an empty box
     */
    public BodyCell(String name, double minX, double maxX, double minY, double maxY, double minZ, double maxZ,
            double seatDepthInches) {
        if (name == null || name.trim().isEmpty() || name.indexOf(' ') >= 0) {
            throw new IllegalArgumentException("a body cell needs a non-blank name with no spaces, got \"" + name + "\"");
        }
        double[] all = {minX, maxX, minY, maxY, minZ, maxZ, seatDepthInches};
        for (double value : all) {
            if (!Double.isFinite(value)) {
                throw new IllegalArgumentException(name + ": every number must be finite");
            }
        }
        if (!(minX < maxX) || !(minY < maxY) || !(minZ < maxZ)) {
            throw new IllegalArgumentException(name + ": a cell needs min < max on every axis");
        }
        if (seatDepthInches < 0.0 || seatDepthInches >= maxZ - minZ) {
            throw new IllegalArgumentException(name + ": seat depth must be in [0, box height), got " + seatDepthInches);
        }
        this.name = name;
        this.minX = minX;
        this.maxX = maxX;
        this.minY = minY;
        this.maxY = maxY;
        this.minZ = minZ;
        this.maxZ = maxZ;
        this.seatDepthInches = seatDepthInches;
    }

    /** {@return the cell's name} */
    public String getName() {
        return name;
    }

    /** {@return how far below the top face a piece's centre must be to count as entered, inches} */
    public double getSeatDepthInches() {
        return seatDepthInches;
    }

    /** {@return the box's lower X bound, inches, body-local} */
    public double getMinX() {
        return minX;
    }

    /** {@return the box's upper X bound, inches, body-local} */
    public double getMaxX() {
        return maxX;
    }

    /** {@return the box's lower Y bound, inches, body-local} */
    public double getMinY() {
        return minY;
    }

    /** {@return the box's upper Y bound, inches, body-local} */
    public double getMaxY() {
        return maxY;
    }

    /** {@return the box's lower Z bound, inches, body-local} */
    public double getMinZ() {
        return minZ;
    }

    /** {@return the box's upper Z bound (the top face), inches, body-local} */
    public double getMaxZ() {
        return maxZ;
    }

    /**
     * Whether a point, body-local, is inside the cell.
     *
     * @param x         the point, inches, body-local
     * @param y         ditto
     * @param z         ditto
     * @param alreadyIn true for a piece already counted in this cell: then the whole box counts; false for a
     *                  piece arriving: then it must also be {@link #getSeatDepthInches()} below the top
     * @return true when the point is in
     */
    public boolean contains(double x, double y, double z, boolean alreadyIn) {
        double top = alreadyIn ? maxZ : maxZ - seatDepthInches;
        return x >= minX && x <= maxX && y >= minY && y <= maxY && z >= minZ && z <= top;
    }
}
