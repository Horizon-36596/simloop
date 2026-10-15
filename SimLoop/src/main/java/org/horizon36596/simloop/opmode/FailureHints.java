package org.horizon36596.simloop.opmode;

import java.util.regex.Matcher;
import java.util.regex.Pattern;

/**
 * What to add to the message of an OpMode's failure when the JVM's own text does not say what to change: a
 * class cast between two copies of one class, and an {@code InterruptedException} on a {@code LinearOpMode}'s
 * thread that no STOP explains. Each hint starts with a space and names the guide; most failures get none.
 */
final class FailureHints {

    /** "class A cannot be cast to class B (...)" on JDK 9+, "A cannot be cast to B" on Java 8 and Android. */
    private static final Pattern CAST =
            Pattern.compile("(?:class )?([\\w.$]+) cannot be cast to (?:class )?([\\w.$]+)");

    /** How far down a cause chain the hints look. */
    static final int MAX_CAUSES = 32;

    private FailureHints() {
    }

    /**
     * {@return the hint for {@code failure}, or an empty string}
     *
     * @param failure            what the OpMode's code threw
     * @param onLinearOpModeThread true if it was thrown by {@code runOpMode()}, whose thread SimLoop runs with its
     *                           interrupt flag set
     */
    static String forFailure(Throwable failure, boolean onLinearOpModeThread) {
        // Bounded, as a cause chain can be a cycle (a.initCause(b) after b.initCause(a)).
        Throwable cause = failure;
        for (int depth = 0; cause != null && depth < MAX_CAUSES; depth++, cause = cause.getCause()) {
            if (cause instanceof ClassCastException && castsAClassToItself(cause.getMessage())) {
                return " SimLoop: each OpModeRunner.forTeamClass run loads the team's classes afresh, so an object"
                        + " of a team class (or of a library in the team packages) that the test made, or that an"
                        + " earlier run left on OpMode.blackboard, is of a different class in this run and cannot be"
                        + " cast. Hand SDK or JDK types (numbers, strings, a Pose2D, maps and lists of those) through"
                        + " the blackboard instead, and build team objects inside the run. See"
                        + " SimLoop/docs/unchanged-team-code.md.";
            }
        }
        if (onLinearOpModeThread && failure instanceof InterruptedException) {
            return " No STOP explains it: SimLoop keeps a LinearOpMode thread's interrupt flag set (that is how the"
                    + " SDK's sleep() comes back to the sim), so a wait SimLoop does not move - Object.wait,"
                    + " Thread.join, CountDownLatch.await, Future.get, BlockingQueue.take, a library's Thread.sleep,"
                    + " and on a run of an OpMode instance the team's own Thread.sleep - throws"
                    + " InterruptedException at once there. Wait with sleep(ms) instead (or, under forTeamClass,"
                    + " Thread.sleep or TimeUnit.sleep in the team's own code). See"
                    + " SimLoop/docs/unchanged-team-code.md.";
        }
        return "";
    }

    private static boolean castsAClassToItself(String message) {
        if (message == null) {
            return false;
        }
        Matcher cast = CAST.matcher(message);
        return cast.find() && cast.group(1).equals(cast.group(2));
    }
}
