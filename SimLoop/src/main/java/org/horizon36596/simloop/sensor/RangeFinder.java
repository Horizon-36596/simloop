package org.horizon36596.simloop.sensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.horizon36596.simloop.fakehardware.FakeDistanceSensor;
import org.horizon36596.simloop.fakehardware.FakeRevColorSensorV3;
import org.horizon36596.simloop.field.FieldWorld;
import org.horizon36596.simloop.field.GamePiece;
import org.horizon36596.simloop.field.ObstacleBox;

import java.util.List;

/**
 * Binds a distance sensor to the simulated field: casts one straight ray from a {@link SensorMount} on the
 * robot and reports the first thing it hits - the perimeter wall, an {@link ObstacleBox}, a game piece that
 * is not held, or the floor. The device fake stays a number; this class decides what the number is.
 *
 * <pre>{@code
 * RangeFinder frontRange = new RangeFinder(SensorMount.level(8.0, 0.0, 3.0, 0.0));
 * // each tick, after the plants have moved and the tracker knows the robot's pose:
 * frontRange.apply(frontDistanceSensor, tracker.getWorld(), tracker.getPieces(), x, y, heading);
 * }</pre>
 *
 * <p><b>The geometry, stated plainly.</b> The ray starts at the sensor face and runs along the mount's
 * direction. The perimeter is the four vertical planes {@code |x| = halfWidth}, {@code |y| = halfLength}
 * of a {@linkplain FieldWorld#isBounded() bounded} world, as tall as the ray needs (PLACEHOLDER: the world
 * does not say how high the wall is, so a ray tilted up is still stopped by it). An obstacle is its full
 * 3-D box, so a level ray passes under a box that starts above the sensor's height and over one that ends
 * below it. A piece is its sphere; a held piece is inside the robot and is never hit. The floor is the
 * plane {@code z = 0}. A sensor face already inside something reads 0.
 *
 * <p><b>What it does not model.</b> A real time-of-flight sensor sees a cone about 25 degrees wide, not a
 * ray, and misses dark or steeply angled surfaces; this sees a single ray and every surface. The robot's own
 * body does not block the ray.
 *
 * <p>Frames: the robot pose is field frame (FTC-Cartesian, centre origin, inches, heading radians CCW from
 * +X), as {@link SensorMount} explains. Deterministic and clock-free: the same world, pieces and pose give
 * the same hit, with ties between equally near pieces going to the lowest id. All trigonometry is
 * {@link StrictMath}.
 */
public final class RangeFinder {

    private final SensorMount mount;

    /**
     * A ray from one mount.
     *
     * @param mount where the sensor is on the robot and which way it looks
     * @throws IllegalArgumentException if {@code mount} is null
     */
    public RangeFinder(SensorMount mount) {
        if (mount == null) {
            throw new IllegalArgumentException("a range finder needs a mount");
        }
        this.mount = mount;
    }

    /** {@return the mount the ray starts from} */
    public SensorMount getMount() {
        return mount;
    }

    /**
     * Casts the ray for a robot at the given pose.
     *
     * @param world           the fixed field: perimeter and obstacles
     * @param pieces          every piece, from {@code GamePieceTracker.getPieces()}; held pieces are skipped
     * @param robotXInches    robot centre, field frame, inches
     * @param robotYInches    robot centre, field frame, inches
     * @param robotHeadingRad robot heading, radians CCW from field +X
     * @return the nearest hit, or a miss ({@link RangeHit.Target#NOTHING}, infinite distance)
     * @throws IllegalArgumentException if {@code world} or {@code pieces} is null, or the pose is not finite
     */
    public RangeHit cast(FieldWorld world, List<GamePiece> pieces, double robotXInches, double robotYInches,
            double robotHeadingRad) {
        if (world == null || pieces == null) {
            throw new IllegalArgumentException("a range finder needs the world and the piece list");
        }
        SensorMount.requireFinite(robotXInches, "robotXInches");
        SensorMount.requireFinite(robotYInches, "robotYInches");
        SensorMount.requireFinite(robotHeadingRad, "robotHeadingRad");

        double[] origin = mount.fieldPosition(robotXInches, robotYInches, robotHeadingRad);
        double[] direction = mount.fieldDirection(robotHeadingRad);
        RangeHit nearest = RangeHit.MISS;

        if (world.isBounded()) {
            double t = distanceToPerimeter(origin, direction, world.getHalfWidthInches(),
                    world.getHalfLengthInches());
            nearest = nearer(nearest, t, RangeHit.Target.WALL, "perimeter", -1);
        }
        for (ObstacleBox box : world.getObstacles()) {
            double t = distanceToBox(origin, direction, box);
            nearest = nearer(nearest, t, RangeHit.Target.OBSTACLE, box.getName(), -1);
        }
        for (GamePiece piece : pieces) {
            if (!piece.isVisibleOnField()) {
                continue;
            }
            double t = distanceToSphere(origin, direction, piece.getFieldXInches(), piece.getFieldYInches(),
                    piece.getFieldZInches(), piece.getRadiusInches());
            nearest = nearer(nearest, t, RangeHit.Target.PIECE, piece.getKind(), piece.getId());
        }
        if (direction[2] < 0.0) {
            double t = origin[2] / -direction[2];
            nearest = nearer(nearest, t, RangeHit.Target.FLOOR, "floor", -1);
        }
        return nearest;
    }

    /**
     * Casts the ray and hands the distance to a fake distance sensor, which reports it - or its
     * out-of-range reading, when the hit is beyond its range or there is none.
     *
     * @param sensor          the fake to drive
     * @param world           the fixed field: perimeter and obstacles
     * @param pieces          every piece; held pieces are skipped
     * @param robotXInches    robot centre, field frame, inches
     * @param robotYInches    robot centre, field frame, inches
     * @param robotHeadingRad robot heading, radians CCW from field +X
     * @return the hit, so a test can check what the sensor saw
     * @throws IllegalArgumentException if {@code sensor}, {@code world} or {@code pieces} is null, or the
     *                                  pose is not finite
     */
    public RangeHit apply(FakeDistanceSensor sensor, FieldWorld world, List<GamePiece> pieces,
            double robotXInches, double robotYInches, double robotHeadingRad) {
        if (sensor == null) {
            throw new IllegalArgumentException("a range finder needs a sensor to drive");
        }
        RangeHit hit = cast(world, pieces, robotXInches, robotYInches, robotHeadingRad);
        sensor.setDistance(DistanceUnit.INCH, hit.getDistanceInches());
        return hit;
    }

    /**
     * Casts the ray and hands the distance to a fake colour sensor's proximity channel. A miss, or anything
     * past the sensor's reach, reads as {@link FakeRevColorSensorV3#MAX_REPORTED_DISTANCE_INCHES}.
     *
     * @param sensor          the fake to drive
     * @param world           the fixed field: perimeter and obstacles
     * @param pieces          every piece; held pieces are skipped
     * @param robotXInches    robot centre, field frame, inches
     * @param robotYInches    robot centre, field frame, inches
     * @param robotHeadingRad robot heading, radians CCW from field +X
     * @return the hit, so a test can check what the sensor saw
     * @throws IllegalArgumentException if {@code sensor}, {@code world} or {@code pieces} is null, or the
     *                                  pose is not finite
     */
    public RangeHit apply(FakeRevColorSensorV3 sensor, FieldWorld world, List<GamePiece> pieces,
            double robotXInches, double robotYInches, double robotHeadingRad) {
        if (sensor == null) {
            throw new IllegalArgumentException("a range finder needs a sensor to drive");
        }
        RangeHit hit = cast(world, pieces, robotXInches, robotYInches, robotHeadingRad);
        sensor.setDistance(DistanceUnit.INCH, hit.getDistanceInches());
        return hit;
    }

    /** The nearer of a hit so far and a candidate at {@code t}; a tie keeps the one found first. */
    private static RangeHit nearer(RangeHit soFar, double t, RangeHit.Target target, String name, int pieceId) {
        if (t < soFar.getDistanceInches()) {
            return new RangeHit(target, t, name, pieceId);
        }
        return soFar;
    }

    /**
     * Distance along the ray to the inside of the perimeter. Zero when the face is on or outside the wall;
     * infinite for a ray pointing straight up or down, which never meets a vertical wall.
     */
    static double distanceToPerimeter(double[] origin, double[] direction, double halfWidth, double halfLength) {
        if (Math.abs(origin[0]) >= halfWidth || Math.abs(origin[1]) >= halfLength) {
            return 0.0;
        }
        double t = Double.POSITIVE_INFINITY;
        if (direction[0] > 0.0) {
            t = Math.min(t, (halfWidth - origin[0]) / direction[0]);
        } else if (direction[0] < 0.0) {
            t = Math.min(t, (-halfWidth - origin[0]) / direction[0]);
        }
        if (direction[1] > 0.0) {
            t = Math.min(t, (halfLength - origin[1]) / direction[1]);
        } else if (direction[1] < 0.0) {
            t = Math.min(t, (-halfLength - origin[1]) / direction[1]);
        }
        return t;
    }

    /**
     * Distance along the ray to an axis-aligned box, by the slab method: the ray is inside the box between
     * the latest of its three entry distances and the earliest of its three exit distances. Zero when the
     * face is inside; infinite on a miss or a box wholly behind the face.
     */
    static double distanceToBox(double[] origin, double[] direction, ObstacleBox box) {
        double[] min = {box.getMinXInches(), box.getMinYInches(), box.getMinZInches()};
        double[] max = {box.getMaxXInches(), box.getMaxYInches(), box.getMaxZInches()};
        double enter = Double.NEGATIVE_INFINITY;
        double exit = Double.POSITIVE_INFINITY;
        for (int axis = 0; axis < 3; axis++) {
            if (direction[axis] == 0.0) {
                if (origin[axis] < min[axis] || origin[axis] > max[axis]) {
                    return Double.POSITIVE_INFINITY;   // parallel to this slab and outside it
                }
                continue;
            }
            double t1 = (min[axis] - origin[axis]) / direction[axis];
            double t2 = (max[axis] - origin[axis]) / direction[axis];
            enter = Math.max(enter, Math.min(t1, t2));
            exit = Math.min(exit, Math.max(t1, t2));
        }
        if (exit < enter || exit < 0.0) {
            return Double.POSITIVE_INFINITY;
        }
        return Math.max(0.0, enter);
    }

    /**
     * Distance along the ray to a sphere: the smaller root of {@code |origin + t*direction - centre| = r}.
     * Zero when the face is inside; infinite on a miss or a sphere wholly behind the face.
     */
    static double distanceToSphere(double[] origin, double[] direction, double centreX, double centreY,
            double centreZ, double radius) {
        double ox = origin[0] - centreX;
        double oy = origin[1] - centreY;
        double oz = origin[2] - centreZ;
        double c = ox * ox + oy * oy + oz * oz - radius * radius;
        if (c <= 0.0) {
            return 0.0;
        }
        double b = ox * direction[0] + oy * direction[1] + oz * direction[2];
        double discriminant = b * b - c;
        if (b >= 0.0 || discriminant < 0.0) {
            return Double.POSITIVE_INFINITY;           // pointing away, or passing wide
        }
        return -b - StrictMath.sqrt(discriminant);
    }
}
