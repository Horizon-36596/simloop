package org.horizon36596.simloop.field;

/**
 * One game piece in the sim: a ball of some kind, at a field position in inches, moving at some velocity,
 * in exactly one {@link PossessionState}.
 *
 * <p><b>Frame and units.</b> {@link #getFieldXInches()} / {@link #getFieldYInches()} are in the repo's
 * FTC-Cartesian field frame — <b>+X right, +Y forward, origin at the centre of the field</b>, inches, so
 * the reachable field is {@code x, y in [-72, +72]} (conventions §7). {@link #getFieldZInches()} is the
 * height of the piece's <em>centre</em> above the floor tiles, so a piece resting on the floor has
 * {@code z == radius}. Velocities are inches per second along the same three axes.
 * {@link #getRadiusInches()} is the piece's own radius in inches; it exists so a trigger volume can decide
 * whether it <em>overlaps</em> a piece rather than only whether it contains the piece's centre point, and
 * so the physics can tell when two pieces touch.
 *
 * <p><b>The tracker owns every change.</b> All the setters are package-private on purpose: a piece is
 * only ever moved or re-stated by {@link GamePieceTracker} (and its physics helper in this package), which
 * is what guarantees every transition gets logged (domain R5). Season code holds a {@code GamePiece} as a
 * read-only view and calls the tracker to change anything.
 *
 * <p>Season-agnostic (domain R7): a piece is a sphere of some size in some state, labelled with a
 * {@link #getKind() kind} string the season chose ({@code "ball"}, {@code "cube"}). Nothing here knows
 * what the kind means, what colour it is, or what it scores — the season layer
 * supplies the radius, the kind, and how many of them fit in the robot.
 */
public final class GamePiece {

    private final int id;
    private final String kind;
    private final double radiusInches;

    private double fieldXInches;
    private double fieldYInches;
    private double fieldZInches;
    private double velocityXInchesPerSecond;
    private double velocityYInchesPerSecond;
    private double velocityZInchesPerSecond;
    private PossessionState state;

    // True while this piece is SCORED and parked at a slot inside one of the world's goals. A piece scored
    // with the tracker's label-only score(...) is SCORED but in no goal, so this stays false for it and it
    // is not drawn or seen anywhere. Cleared by the tracker on every state change.
    private boolean parkedInGoal;

    // The name of the goal this piece was just released from by GamePieceTracker.releaseScoredPiece, or
    // null. While set, the physics does not let the piece score back into that same goal: a released piece
    // can start inside the goal's box, and without this it would be re-scored on the very next sub-step.
    // Cleared once the piece's centre has left that goal's box, and on every state change.
    private String releasedFromGoalName;

    // True from the moment GamePieceTracker.launch throws this piece until its centre is first clear of the
    // robot's box. A launcher's exit point is usually inside the robot, so without this every shot would hit
    // the robot it came out of. Cleared by the physics once the piece is out, and on every state change.
    private boolean leavingRobot;

    // Spin, radians per second about field X, Y, Z. Only the 3-D contact model near a KinematicBody uses it
    // (it is what makes a ball roll down a slope rather than slide); zero everywhere else.
    private double spinX;
    private double spinY;
    private double spinZ;

    // The name of the KinematicBody cell this SCORED piece is resting in, or null. A seated piece is not
    // parked: the physics keeps moving it, and it leaves the cell when the physics carries it out.
    private String seatedCellName;

    // A piece at rest on a body or on other pieces is put to sleep so a pile sits exactly still; the physics
    // skips it until something disturbs it. stillSeconds is how long it has been still so far.
    private boolean sleeping;
    private double stillSeconds;

    // The index, in the world's list, of the KinematicBody this piece last rested on: touched from below, or
    // through a piece it rests on. -1 for none. While the piece sleeps, its weight goes to that body.
    private int supportBodyIndex = -1;

    /**
     * Created only by {@link GamePieceTracker}, which validates every argument first and assigns the id.
     * The piece starts at rest on the floor ({@code z == radius}) in the given state.
     */
    GamePiece(int id, String kind, double fieldXInches, double fieldYInches, double radiusInches,
            PossessionState state) {
        this.id = id;
        this.kind = kind;
        this.fieldXInches = fieldXInches;
        this.fieldYInches = fieldYInches;
        this.fieldZInches = radiusInches;
        this.radiusInches = radiusInches;
        this.state = state;
    }

    /**
     * Stable identifier, assigned in the order pieces were added (first piece is {@code 0}). Acquisition
     * scans pieces in this order, so a partly-full robot fills up reproducibly on replay (domain R5).
     *
     * @return the piece's id, unitless, counting from zero
     */
    public int getId() {
        return id;
    }

    /**
     * The season's label for what sort of piece this is, e.g. {@code "ball"}. Used to group pieces in the
     * log so a viewer can draw each kind with its own model, and reported by a sim camera so a vision test
     * can check its classifier. The core never interprets it.
     *
     * @return the kind label, never blank
     */
    public String getKind() {
        return kind;
    }

    /** {@return the field position, inches, FTC-Cartesian +X right (see class javadoc for the frame)} */
    public double getFieldXInches() {
        return fieldXInches;
    }

    /** {@return the field position, inches, FTC-Cartesian +Y forward (see class javadoc for the frame)} */
    public double getFieldYInches() {
        return fieldYInches;
    }

    /** {@return the height of the piece's centre above the floor, inches; equals the radius when resting} */
    public double getFieldZInches() {
        return fieldZInches;
    }

    /** {@return velocity along field +X, inches per second; zero unless loose-and-rolling or in flight} */
    public double getVelocityXInchesPerSecond() {
        return velocityXInchesPerSecond;
    }

    /** {@return velocity along field +Y, inches per second; zero unless loose-and-rolling or in flight} */
    public double getVelocityYInchesPerSecond() {
        return velocityYInchesPerSecond;
    }

    /** {@return vertical velocity, inches per second, positive upward; zero unless in flight} */
    public double getVelocityZInchesPerSecond() {
        return velocityZInchesPerSecond;
    }

    /** {@return the piece's radius, in inches} - half its diameter, e.g. 1.4 for a 2.8 in ball */
    public double getRadiusInches() {
        return radiusInches;
    }

    /** {@return which of the possession states this piece is in right now} */
    public PossessionState getState() {
        return state;
    }

    /** {@return true while this piece is being carried by the robot} */
    public boolean isHeld() {
        return state == PossessionState.HELD;
    }

    /** {@return true while this piece is airborne after a launch} */
    public boolean isInFlight() {
        return state == PossessionState.IN_FLIGHT;
    }

    /**
     * True when the piece is somewhere a camera or a viewer could see it: on the floor, in the air, or
     * parked in one of the world's goals.
     *
     * <p>Two kinds of piece are not visible. A held piece is hidden inside the robot. And a piece scored
     * with {@link GamePieceTracker#score(int, String)} - the label-only score, which puts the piece in no
     * goal - has left the field as far as the sim knows: it is not drawn on the tiles at the robot's last
     * position, and a sim camera does not see it there.
     *
     * @return true for a loose or in-flight piece, and for a scored piece parked in a goal or resting in a
     *         {@link BodyCell}
     */
    public boolean isVisibleOnField() {
        if (state == PossessionState.LOOSE || state == PossessionState.IN_FLIGHT) {
            return true;
        }
        if (state == PossessionState.SCORED) {
            return parkedInGoal || seatedCellName != null;
        }
        return false;                                  // HELD: inside the robot
    }

    /** {@return true while this piece is SCORED and parked at a slot inside one of the world's goals} */
    public boolean isParkedInGoal() {
        return parkedInGoal;
    }

    /**
     * {@return the name of the {@link BodyCell} this SCORED piece is resting in, or null} A seated piece is
     * still moved by the physics, and is counted in the cell for exactly as long as it stays inside it.
     */
    public String getSeatedCellName() {
        return seatedCellName;
    }

    /** {@return spin about field +X, radians per second; zero except near a {@link KinematicBody}} */
    public double getSpinXRadPerSecond() {
        return spinX;
    }

    /** {@return spin about field +Y, radians per second; zero except near a {@link KinematicBody}} */
    public double getSpinYRadPerSecond() {
        return spinY;
    }

    /** {@return spin about field +Z, radians per second; zero except near a {@link KinematicBody}} */
    public double getSpinZRadPerSecond() {
        return spinZ;
    }

    /** {@return true while the physics has put this piece to sleep, at rest until something disturbs it} */
    public boolean isSleeping() {
        return sleeping;
    }

    /** Package-private: the tracker seats a SCORED piece in a cell (a name) or takes it out (null). */
    void setSeatedCellName(String seatedCellName) {
        this.seatedCellName = seatedCellName;
    }

    /** Package-private: only the physics spins a piece. */
    void setSpinRadPerSecond(double spinX, double spinY, double spinZ) {
        this.spinX = spinX;
        this.spinY = spinY;
        this.spinZ = spinZ;
    }

    /** Package-private: the physics puts a piece to sleep (true) or wakes it (false); either resets the still time. */
    void setSleeping(boolean sleeping) {
        this.sleeping = sleeping;
        this.stillSeconds = 0.0;
    }

    /** {@return how long this piece has been still, seconds, toward being put to sleep} */
    double getStillSeconds() {
        return stillSeconds;
    }

    /** Package-private: the physics counts up a piece's still time, or resets it to zero. */
    void setStillSeconds(double stillSeconds) {
        this.stillSeconds = stillSeconds;
    }

    /** {@return the index of the body this piece last rested on, in the world's list, or -1 for none} */
    int getSupportBodyIndex() {
        return supportBodyIndex;
    }

    /** Package-private: the physics records what a piece rests on (a body's index), or -1 for nothing. */
    void setSupportBodyIndex(int supportBodyIndex) {
        this.supportBodyIndex = supportBodyIndex;
    }

    /** Package-private: only the tracker and its physics move a piece, so that the move gets logged. */
    void setFieldPositionInches(double fieldXInches, double fieldYInches, double fieldZInches) {
        this.fieldXInches = fieldXInches;
        this.fieldYInches = fieldYInches;
        this.fieldZInches = fieldZInches;
    }

    /** Package-private: sets the piece down at rest on the floor at this spot, velocity zeroed. */
    void restOnFloorAt(double fieldXInches, double fieldYInches) {
        setFieldPositionInches(fieldXInches, fieldYInches, radiusInches);
        setVelocityInchesPerSecond(0.0, 0.0, 0.0);
    }

    /** Package-private: only the tracker and its physics change a velocity. */
    void setVelocityInchesPerSecond(double velocityX, double velocityY, double velocityZ) {
        this.velocityXInchesPerSecond = velocityX;
        this.velocityYInchesPerSecond = velocityY;
        this.velocityZInchesPerSecond = velocityZ;
    }

    /** Package-private: only {@link GamePieceTracker} restates a piece, so that the change gets logged. */
    void setState(PossessionState state) {
        this.state = state;
    }

    /** Package-private: the tracker marks a piece parked in a goal (true) or not (false). */
    void setParkedInGoal(boolean parkedInGoal) {
        this.parkedInGoal = parkedInGoal;
    }

    /** {@return the goal this piece was just released from and may not re-score into yet, or null} */
    String getReleasedFromGoalName() {
        return releasedFromGoalName;
    }

    /** Package-private: the tracker sets it on release; the physics clears it once the piece is out. */
    void setReleasedFromGoalName(String releasedFromGoalName) {
        this.releasedFromGoalName = releasedFromGoalName;
    }

    /** {@return true while a just-launched piece has not yet cleared the robot it was launched from} */
    boolean isLeavingRobot() {
        return leavingRobot;
    }

    /** Package-private: the tracker sets it on launch; the physics clears it once the piece is clear. */
    void setLeavingRobot(boolean leavingRobot) {
        this.leavingRobot = leavingRobot;
    }

    @Override
    public String toString() {
        return "GamePiece#" + id + " " + kind + " " + state;
    }
}
