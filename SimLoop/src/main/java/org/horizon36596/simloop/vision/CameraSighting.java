package org.horizon36596.simloop.vision;

/**
 * One game piece as a {@link SimCamera} sees it this tick: where its centre lands in the image, how big
 * it looks, how far away it is, and what it is. This is the ground truth a synthetic frame would be
 * rendered from, and it is shaped to line up field-for-field with a typical per-frame detection record
 * ({@code centreXPx, centreYPx, radiusPx, clipped, type}) so that a test can feed sightings into a
 * tracker as if a detector had produced them, or compare a detector's output against them.
 *
 * <p>Pixel frame: {@code (0, 0)} top-left, {@code +x} right, {@code +y} down. Range is measured from the
 * lens to the piece's centre, in inches.
 *
 * <p>Immutable.
 */
public final class CameraSighting {

    private final int pieceId;
    private final String kind;
    private final double centreXPx;
    private final double centreYPx;
    private final double radiusPx;
    private final double rangeInches;
    private final boolean clipped;

    CameraSighting(int pieceId, String kind, double centreXPx, double centreYPx, double radiusPx,
            double rangeInches, boolean clipped) {
        this.pieceId = pieceId;
        this.kind = kind;
        this.centreXPx = centreXPx;
        this.centreYPx = centreYPx;
        this.radiusPx = radiusPx;
        this.rangeInches = rangeInches;
        this.clipped = clipped;
    }

    /** {@return the id of the piece seen — ground truth a real detector never has} */
    public int getPieceId() {
        return pieceId;
    }

    /** {@return the piece's kind label, e.g. {@code "ball"} — the class a detector would have to infer} */
    public String getKind() {
        return kind;
    }

    /** {@return the piece centre's image x, pixels from the left edge; may lie outside the image if clipped} */
    public double getCentreXPx() {
        return centreXPx;
    }

    /** {@return the piece centre's image y, pixels from the top edge; may lie outside the image if clipped} */
    public double getCentreYPx() {
        return centreYPx;
    }

    /**
     * The piece's apparent radius, pixels, computed as horizontal focal length times real radius over range.
     *
     * <p>This is an approximation, and it is the one a simple synthetic renderer draws. A real sphere seen
     * through a pinhole projects exactly to a circle only on the optical axis, and even there its image
     * radius is {@code f * radius / sqrt(range^2 - radius^2)}, about 1% larger than this at a range of 7
     * radii and less beyond. Off the axis the image becomes an ellipse stretched away from the image centre,
     * by roughly {@code 1 / cos} of the angle off the axis (about 30% in the corner of a 70-degree-wide
     * 640x400 image). A test that compares a detector's blob size with this number should allow for that.
     *
     * @return the apparent radius, pixels
     */
    public double getRadiusPx() {
        return radiusPx;
    }

    /** {@return straight-line distance from the lens to the piece's centre, inches} */
    public double getRangeInches() {
        return rangeInches;
    }

    /** {@return true when part of the piece's disc falls outside the image edges} */
    public boolean isClipped() {
        return clipped;
    }

    @Override
    public String toString() {
        return "CameraSighting(piece " + pieceId + " " + kind + " at (" + centreXPx + ", " + centreYPx
                + ") r=" + radiusPx + "px range=" + rangeInches + "in" + (clipped ? " clipped" : "") + ")";
    }
}
