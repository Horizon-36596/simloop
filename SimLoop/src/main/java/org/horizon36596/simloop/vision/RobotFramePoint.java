package org.horizon36596.simloop.vision;

/**
 * A point on the floor plane in the robot frame: how far ahead of the robot's centre and how far to its
 * left, in inches. Returned by {@link SimCamera#groundPointInRobotFrame} so a caller reads
 * {@code forwardInches} and {@code leftInches} by name rather than guessing which index of an array is
 * which.
 *
 * <p>Robot frame: {@code +forward} out the nose, {@code +left} out the left side, both from the robot's
 * centre. Immutable.
 */
public final class RobotFramePoint {

    /** Distance ahead of the robot centre, inches; negative is behind. */
    public final double forwardInches;

    /** Distance to the left of the robot centre, inches; negative is to the right. */
    public final double leftInches;

    /**
     * A point on the floor, robot frame.
     *
     * @param forwardInches distance ahead of the robot centre, inches; negative is behind
     * @param leftInches    distance to the left of the robot centre, inches; negative is to the right
     */
    public RobotFramePoint(double forwardInches, double leftInches) {
        this.forwardInches = forwardInches;
        this.leftInches = leftInches;
    }

    @Override
    public String toString() {
        return "RobotFramePoint(forward " + forwardInches + " in, left " + leftInches + " in)";
    }
}
