package org.horizon36596.simloop.vision;

/**
 * One AprilTag as a {@link SimTagCamera} sees it this tick: the ground truth a tag detector would have to
 * recover, with no noise. The angles and pixels describe where the tag's centre lands in the image; the
 * two transforms describe where the whole tag is relative to the lens and to the robot.
 *
 * <p>Frames:
 * <ul>
 *   <li><b>Optical</b> (camera space): origin at the lens, {@code +x} image right, {@code +y} image down,
 *       {@code +z} along the boresight, as {@link CameraMount} defines it.</li>
 *   <li><b>Robot</b>: origin on the floor under the robot centre, {@code +x} forward, {@code +y} left,
 *       {@code +z} up, as {@link CameraMount} defines it.</li>
 *   <li><b>Tag</b> (target space): see {@link AprilTagLayout.Tag#getFieldToTag()}.</li>
 * </ul>
 * All lengths are inches; angles are degrees where the name says so.
 *
 * <p>Immutable.
 */
public final class TagSighting {

    private final AprilTagLayout.Tag tag;
    private final RigidTransform3 cameraToTag;
    private final RigidTransform3 robotToTag;
    private final double centreXPx;
    private final double centreYPx;
    private final double[][] cornersPx;
    private final double txDegrees;
    private final double tyDegrees;
    private final double areaPercent;

    TagSighting(AprilTagLayout.Tag tag, RigidTransform3 cameraToTag, RigidTransform3 robotToTag,
            double centreXPx, double centreYPx, double[][] cornersPx, double txDegrees, double tyDegrees,
            double areaPercent) {
        this.tag = tag;
        this.cameraToTag = cameraToTag;
        this.robotToTag = robotToTag;
        this.centreXPx = centreXPx;
        this.centreYPx = centreYPx;
        this.cornersPx = cornersPx;
        this.txDegrees = txDegrees;
        this.tyDegrees = tyDegrees;
        this.areaPercent = areaPercent;
    }

    /** {@return the tag seen, from the layout} */
    public AprilTagLayout.Tag getTag() {
        return tag;
    }

    /** {@return the tag's id} */
    public int getId() {
        return tag.getId();
    }

    /** {@return the tag's frame in the optical frame, {@code T_camera_tag}, inches} */
    public RigidTransform3 getCameraToTag() {
        return cameraToTag;
    }

    /** {@return the tag's frame in the robot frame, {@code T_robot_tag}, inches} */
    public RigidTransform3 getRobotToTag() {
        return robotToTag;
    }

    /** {@return straight-line distance from the lens to the tag centre, inches} */
    public double getRangeInches() {
        return cameraToTag.getDistanceInches();
    }

    /**
     * Horizontal angle from the boresight to the tag centre, degrees, positive when the tag is to the right
     * of the image centre: {@code atan(x / z)} in the optical frame, which is also
     * {@code atan((centreXPx - cx) / fx)} for the pinhole lens.
     *
     * @return the angle, degrees
     */
    public double getTxDegrees() {
        return txDegrees;
    }

    /**
     * Vertical angle from the boresight to the tag centre, degrees, positive when the tag is <em>above</em>
     * the image centre: {@code atan(-y / z)} in the optical frame.
     *
     * @return the angle, degrees
     */
    public double getTyDegrees() {
        return tyDegrees;
    }

    /** {@return the tag centre's image x, pixels from the left edge} */
    public double getCentreXPx() {
        return centreXPx;
    }

    /** {@return the tag centre's image y, pixels from the top edge} */
    public double getCentreYPx() {
        return centreYPx;
    }

    /**
     * The four corners of the tag square in the image, pixels, in the order top-left, top-right,
     * bottom-right, bottom-left as the camera sees them. Each entry is {@code {x, y}}.
     *
     * @return a fresh copy of the corners
     */
    public double[][] getCornersPx() {
        double[][] copy = new double[4][];
        for (int i = 0; i < 4; i++) {
            copy[i] = cornersPx[i].clone();
        }
        return copy;
    }

    /** {@return the area of the tag's image quadrilateral as a percentage of the whole image, 0 to 100} */
    public double getAreaPercent() {
        return areaPercent;
    }

    @Override
    public String toString() {
        return "TagSighting(tag " + tag.getId() + " tx=" + txDegrees + " ty=" + tyDegrees + " range="
                + getRangeInches() + "in area=" + areaPercent + "%)";
    }
}
