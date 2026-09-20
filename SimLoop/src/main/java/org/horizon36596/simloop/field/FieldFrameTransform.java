package org.horizon36596.simloop.field;

/**
 * The one and only place the localizer &harr; field frame rotation is written.
 *
 * <p>Two frames describe where the robot is, both of them correct and both of them deliberate, and every
 * bug this class exists to prevent is a bug where one was mistaken for the other:
 *
 * <pre>
 *   Localizer frame                               Field frame (FTC-Cartesian)
 *   (what a drivetrain and a dead-wheel           (what every game piece, trigger volume,
 *    odometry pod report)                          log and human-typed pose means)
 *   ----------------------------------------      ---------------------------------------------
 *     +x = robot forward at heading 0               +X = field right
 *     +y = robot left at heading 0                  +Y = field forward
 *     heading CCW-positive, radians                 heading CCW-positive, radians
 * </pre>
 *
 * <p>Both are in <b>inches</b> and <b>radians</b>. Nothing here converts units; a device that reports
 * millimetres converts before calling in.
 *
 * <p><b>The rotation.</b> Localizer &rarr; field is a +90&deg; rotation about the origin:
 * {@code fieldX = -localizerY}, {@code fieldY = +localizerX}, {@code fieldHeading = localizerHeading +
 * PI/2}. The inverse goes back. There is no translation in either direction: both frames share the same
 * origin, which for this project's field is its centre.
 *
 * <p><b>Why this class is here rather than in a season's own code.</b> The two frames meet in exactly one
 * situation — a pose that came out of {@code MecanumDrivePlant} or {@code MecanumPoseIntegrator} being
 * asked a field question, such as whether a {@link TriggerVolume} overlaps a {@link GamePiece}. Both
 * halves of that sentence live in this module, so the rotation has to live here too: a season's copy of
 * it would not be reachable from here, because nothing in this module may import season code. Before
 * this class existed the rotation was only written in a season's localization package, so every consumer
 * of this module had to rewrite it and nothing told them they had to. That was the defect (BACKLOG B37).
 *
 * <p><b>Which frame is which, per type.</b> Worth checking rather than guessing:
 * <ul>
 *   <li>{@code MecanumPoseIntegrator} and {@code MecanumDrivePlant} — <b>localizer frame</b>. They
 *       integrate in the frame a localizer reports, because in a real robot that is what feeds them.</li>
 *   <li>{@link GamePiece}, {@link GamePieceTracker} and {@link TriggerVolume#overlapsPiece} —
 *       <b>field frame</b>. These are the coordinates a human reads off a field drawing.</li>
 * </ul>
 * So a drivetrain pose handed straight to {@code overlapsPiece} is wrong by ninety degrees, and wrong in
 * the way that hurts: the robot lands somewhere plausible, facing the wrong way, and the test fails as
 * though the geometry were mis-measured. Rotate it here first.
 *
 * <p><b>Angular velocity is NOT rotated.</b> The two frames differ by a <i>constant</i> heading offset,
 * and the derivative of a constant is zero, so a heading rate is the same number in both frames. Only
 * the position axes swap.
 *
 * <p><b>A linear velocity already in the localizer frame</b> does rotate, because it is a vector in the
 * rotated axes — pass it through {@link #fieldXFromLocalizer} and {@link #fieldYFromLocalizer} like a
 * position. A <b>robot-frame</b> velocity is a different thing and these methods are the wrong tool for
 * it: a chassis velocity such as the {@code vx}/{@code vy} inside {@code MecanumPoseIntegrator} is
 * expressed along the robot's own forward/left axes at its current heading, so it has to be rotated by
 * that heading into the localizer frame first. Only then does this class apply.
 *
 * <p>All methods are pure functions of their arguments: no state, no clock, no hardware (domain R5), so
 * a replay of the same run rotates the same way.
 */
public final class FieldFrameTransform {

    /**
     * Constant heading offset between the two frames, in radians: the field frame is +90&deg; from the
     * localizer frame.
     */
    public static final double LOCALIZER_TO_FIELD_HEADING_OFFSET_RADIANS = Math.PI / 2.0;

    private FieldFrameTransform() {
        // Static helpers only — there is nothing to construct.
    }

    // ---------------------------------------------------------------------------------------------
    // Localizer frame -> field frame. This is the direction almost every caller wants: a pose came out
    // of a drivetrain plant and a field question is about to be asked of it.
    // ---------------------------------------------------------------------------------------------

    /**
     * Field X (field right) from a localizer-frame position.
     *
     * <p>Reads only {@code localizerYInches}; it takes both so that a call site reads as a whole pose
     * and so the six methods here stay one shape. Pass them in the order the getters are declared —
     * {@code fieldXFromLocalizer(drive.getX(), drive.getY())} — because transposing them compiles and
     * returns a wrong number rather than failing.
     *
     * @param localizerXInches localizer X (robot forward at heading 0), in inches; not read
     * @param localizerYInches localizer Y (robot left at heading 0), in inches
     * @return field-frame X, in inches, positive toward field right
     */
    public static double fieldXFromLocalizer(double localizerXInches, double localizerYInches) {
        return -localizerYInches;
    }

    /**
     * Field Y (field forward) from a localizer-frame position.
     *
     * <p>Reads only {@code localizerXInches}, for the reason given on {@link #fieldXFromLocalizer}.
     *
     * @param localizerXInches localizer X (robot forward at heading 0), in inches
     * @param localizerYInches localizer Y (robot left at heading 0), in inches; not read
     * @return field-frame Y, in inches, positive toward field forward
     */
    public static double fieldYFromLocalizer(double localizerXInches, double localizerYInches) {
        return localizerXInches;
    }

    /**
     * Field heading from a localizer-frame heading. Not wrapped to any range: a caller that needs a
     * normalized angle normalizes at the point of use, because wrapping here would silently change a
     * pose that a caller was accumulating.
     *
     * @param localizerHeadingRadians localizer heading, in radians, CCW-positive
     * @return field-frame heading, in radians, CCW-positive, measured from field {@code +X}
     */
    public static double fieldHeadingFromLocalizer(double localizerHeadingRadians) {
        return localizerHeadingRadians + LOCALIZER_TO_FIELD_HEADING_OFFSET_RADIANS;
    }

    // ---------------------------------------------------------------------------------------------
    // Field frame -> localizer frame. Needed when a human-written field pose has to be pushed into a
    // drivetrain or a localizer, which is how an autonomous run gets its starting pose.
    // ---------------------------------------------------------------------------------------------

    /**
     * Localizer X (robot forward at heading 0) from a field-frame position.
     *
     * <p>Reads only {@code fieldYInches}, for the reason given on {@link #fieldXFromLocalizer}.
     *
     * @param fieldXInches field-frame X (field right), in inches; not read
     * @param fieldYInches field-frame Y (field forward), in inches
     * @return localizer-frame X, in inches
     */
    public static double localizerXFromField(double fieldXInches, double fieldYInches) {
        return fieldYInches;
    }

    /**
     * Localizer Y (robot left at heading 0) from a field-frame position.
     *
     * <p>Reads only {@code fieldXInches}, for the reason given on {@link #fieldXFromLocalizer}.
     *
     * @param fieldXInches field-frame X (field right), in inches
     * @param fieldYInches field-frame Y (field forward), in inches; not read
     * @return localizer-frame Y, in inches
     */
    public static double localizerYFromField(double fieldXInches, double fieldYInches) {
        return -fieldXInches;
    }

    /**
     * Localizer heading from a field-frame heading. Not wrapped, for the same reason
     * {@link #fieldHeadingFromLocalizer} is not.
     *
     * @param fieldHeadingRadians field-frame heading, in radians, CCW-positive from field {@code +X}
     * @return localizer-frame heading, in radians, CCW-positive
     */
    public static double localizerHeadingFromField(double fieldHeadingRadians) {
        return fieldHeadingRadians - LOCALIZER_TO_FIELD_HEADING_OFFSET_RADIANS;
    }
}
