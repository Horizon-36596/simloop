package org.horizon36596.simloop.vision;

import org.horizon36596.simloop.field.ObstacleBox;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * The solid things a camera cannot see through, in two frames: boxes fixed to the field (structures,
 * walls, goals) and boxes fixed to the robot (its own chassis, a raised arm, a bumper). Handed to
 * {@link SimCamera}, {@link SimTagCamera} and {@link SimFrameRenderer} each frame, because both kinds can
 * move: a field structure may be rebuilt every tick, and a robot's arm changes the robot's shape.
 *
 * <p><b>Frames.</b>
 * <ul>
 *   <li><b>Field boxes</b>: {@link ObstacleBox} as everywhere else in SimLoop, field frame, inches,
 *       axis-aligned with the field.</li>
 *   <li><b>Robot boxes</b>: also {@link ObstacleBox}, but read in the <b>robot frame</b> of
 *       {@link CameraMount}: {@code X} is forward, {@code Y} is left, {@code Z} is up off the floor, origin
 *       on the floor under the robot's centre of rotation. So {@code new ObstacleBox("chassis", -9, 9, -8,
 *       8, 0, 4)} is an 18 x 16 in chassis 4 in tall, and it turns with the robot. Robot boxes are
 *       axis-aligned with the robot.</li>
 * </ul>
 *
 * <p>A camera mounted inside one of the robot's own boxes would see nothing, so a box that contains the
 * lens is skipped for that camera; give the camera's housing no box.
 *
 * <p>Immutable: both lists are copied.
 */
public final class CameraOccluders {

    private static final CameraOccluders NONE = new CameraOccluders(Collections.<ObstacleBox>emptyList(),
            Collections.<ObstacleBox>emptyList());

    private final List<ObstacleBox> fieldBoxes;
    private final List<ObstacleBox> robotBoxes;

    private CameraOccluders(List<ObstacleBox> fieldBoxes, List<ObstacleBox> robotBoxes) {
        this.fieldBoxes = Collections.unmodifiableList(new ArrayList<ObstacleBox>(fieldBoxes));
        this.robotBoxes = Collections.unmodifiableList(new ArrayList<ObstacleBox>(robotBoxes));
    }

    /** {@return nothing blocks the view} */
    public static CameraOccluders none() {
        return NONE;
    }

    /**
     * Boxes in both frames.
     *
     * @param fieldBoxes boxes fixed to the field, field frame, inches; may be empty, not null
     * @param robotBoxes boxes fixed to the robot, robot frame ({@code X} forward, {@code Y} left, {@code Z}
     *                   up), inches; may be empty, not null
     * @return the occluders
     */
    public static CameraOccluders of(List<ObstacleBox> fieldBoxes, List<ObstacleBox> robotBoxes) {
        if (fieldBoxes == null || robotBoxes == null) {
            throw new IllegalArgumentException("occluder lists must not be null; pass an empty list");
        }
        return new CameraOccluders(fieldBoxes, robotBoxes);
    }

    /**
     * Field boxes only.
     *
     * @param fieldBoxes boxes fixed to the field, field frame, inches
     * @return the occluders
     */
    public static CameraOccluders field(List<ObstacleBox> fieldBoxes) {
        return of(fieldBoxes, Collections.<ObstacleBox>emptyList());
    }

    /** {@return the boxes fixed to the field, field frame, unmodifiable} */
    public List<ObstacleBox> getFieldBoxes() {
        return fieldBoxes;
    }

    /** {@return the boxes fixed to the robot, robot frame, unmodifiable} */
    public List<ObstacleBox> getRobotBoxes() {
        return robotBoxes;
    }

    /** {@return true when there are no boxes at all} */
    public boolean isEmpty() {
        return fieldBoxes.isEmpty() && robotBoxes.isEmpty();
    }

    @Override
    public String toString() {
        return "CameraOccluders(" + fieldBoxes.size() + " field, " + robotBoxes.size() + " robot)";
    }

    // ------------------------------------------------------------------------------ shared ray geometry

    /**
     * The ray parameter at which {@code origin + t * dir} first enters the box, or {@code +infinity} when it
     * does not within {@code (minT, maxT)}. A ray starting inside the box enters at {@code minT}.
     */
    static double enterParameter(double[] origin, double[] dir, ObstacleBox box, double minT, double maxT) {
        double enter = minT;
        double exit = maxT;
        double[] min = {box.getMinXInches(), box.getMinYInches(), box.getMinZInches()};
        double[] max = {box.getMaxXInches(), box.getMaxYInches(), box.getMaxZInches()};
        for (int axis = 0; axis < 3; axis++) {
            double d = dir[axis];
            if (Math.abs(d) < 1e-12) {
                if (origin[axis] <= min[axis] || origin[axis] >= max[axis]) {
                    return Double.POSITIVE_INFINITY;
                }
                continue;
            }
            double t0 = (min[axis] - origin[axis]) / d;
            double t1 = (max[axis] - origin[axis]) / d;
            enter = Math.max(enter, Math.min(t0, t1));
            exit = Math.min(exit, Math.max(t0, t1));
            if (enter >= exit) {
                return Double.POSITIVE_INFINITY;
            }
        }
        return enter;
    }

    /** Whether a point is inside the box or on its surface. */
    static boolean contains(ObstacleBox box, double x, double y, double z) {
        return x >= box.getMinXInches() && x <= box.getMaxXInches()
                && y >= box.getMinYInches() && y <= box.getMaxYInches()
                && z >= box.getMinZInches() && z <= box.getMaxZInches();
    }
}
