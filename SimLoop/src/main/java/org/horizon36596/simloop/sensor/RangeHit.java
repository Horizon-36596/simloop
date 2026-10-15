package org.horizon36596.simloop.sensor;

/**
 * What a {@link RangeFinder} ray hit first, and how far along the ray it was. Immutable.
 */
public final class RangeHit {

    /** What sort of thing stopped the ray. */
    public enum Target {
        /** Nothing: the ray left an unbounded field, or went up and over everything. */
        NOTHING,
        /** The field perimeter wall. */
        WALL,
        /** One of the world's {@code ObstacleBox}es. */
        OBSTACLE,
        /** A game piece that is not held. */
        PIECE,
        /** The floor tiles, for a sensor tilted down. */
        FLOOR
    }

    /** The one miss, shared: nothing was hit. */
    static final RangeHit MISS = new RangeHit(Target.NOTHING, Double.POSITIVE_INFINITY, "", -1);

    private final Target target;
    private final double distanceInches;
    private final String targetName;
    private final int pieceId;

    RangeHit(Target target, double distanceInches, String targetName, int pieceId) {
        this.target = target;
        this.distanceInches = distanceInches;
        this.targetName = targetName;
        this.pieceId = pieceId;
    }

    /** {@return what the ray hit first; {@link Target#NOTHING} for a miss} */
    public Target getTarget() {
        return target;
    }

    /** {@return how far from the sensor face the hit was, inches; {@code +infinity} for a miss, 0 if the face is inside the target} */
    public double getDistanceInches() {
        return distanceInches;
    }

    /**
     * {@return the obstacle's name for an {@link Target#OBSTACLE} hit, the piece's kind for a {@link Target#PIECE}
     * hit, {@code "perimeter"} for the wall, {@code "floor"} for the floor, and empty for a miss}
     */
    public String getTargetName() {
        return targetName;
    }

    /** {@return the piece's id for a {@link Target#PIECE} hit, otherwise -1} */
    public int getPieceId() {
        return pieceId;
    }

    /** {@return true unless the ray hit nothing} */
    public boolean isHit() {
        return target != Target.NOTHING;
    }

    /** {@return the hit, for test messages} */
    @Override
    public String toString() {
        return "RangeHit(" + target + (targetName.isEmpty() ? "" : " " + targetName)
                + (pieceId >= 0 ? " #" + pieceId : "") + " at " + distanceInches + " in)";
    }
}
