package org.horizon36596.simloop.loop;

import org.horizon36596.simloop.sim.RlogDecodedCompare;

import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/**
 * What the loop says when it is stopped: the best answer it found, and proof that the answer reproduces.
 *
 * <h2>The report reproduces before it claims</h2>
 * The loop may be stopped at any moment, and whatever it reports at that point is what a human will act
 * on ({@code docs/process/scorer-interface.md} §5). So before this report presents a score as the best,
 * the candidate's scenario is run <b>a second time</b> and the replay is compared field by field against
 * the frames the accepted run was scored from. If they differ, the report says a determinism failure
 * happened and <b>does not present the score</b>. The comparison is of rendered values only, not of each
 * field's logged type; {@link LoopDriver#stopReport} says why that is enough for the score.
 *
 * <p>That ordering is the point of the class. A score that cannot be reproduced is not a small caveat on
 * a real result — it is not a result. The run that produced it may have been the lucky one, and every
 * decision made downstream ("the new gains are better") would rest on a number nobody can get back. The
 * loop engine's goal is exactly this: it never reports an improvement it cannot reproduce.
 *
 * <h2>What this check cannot prove</h2>
 * It proves the scenario produced the same decoded fields twice <i>given a replayer that really
 * re-runs it</i>. A replayer that copied the original RLOG to a new path would satisfy every check
 * here, and this class cannot tell the difference -- it sees two logs, not two executions. The only
 * mechanical guard is the refusal to accept a replayer that hands back the original file.
 * Supplying a replayer that genuinely re-runs the scenario is the caller's responsibility, and in the
 * season glue it is a {@code ScenarioRunner} call that does it.
 *
 * <p>The refusal is enforced in the type, not by asking the reader to check a flag first:
 * {@link #reportedScore()} throws unless {@link #reproduction()} is {@link Reproduction#REPRODUCED}. A
 * caller that wants the number has to have got past the reproduction check to reach it.
 *
 * <p>Season-agnostic core (R7). Reads no clock (R5).
 */
public final class StopReport {

    /** How many differing keys {@link #text()} lists before it counts the rest, so the report stays short. */
    private static final int MISMATCH_KEYS_SHOWN = 10;

    /** How the second run of the best candidate's scenario came out. */
    public enum Reproduction {
        /** Replayed, and every decoded field matched. The score is safe to report. */
        REPRODUCED,
        /**
         * Replayed, and the decoded fields differed. This is a determinism failure (R5) and the score
         * is not reported — the mismatches are.
         */
        DID_NOT_REPRODUCE,
        /**
         * No replay was attempted, because there is no last-best to replay. The loop was stopped before
         * the gate accepted anything, the baseline included.
         */
        NOT_ATTEMPTED
    }

    /**
     * Runs one accepted candidate's scenario again, from the top, and returns the fresh RLOG.
     *
     * <p>This is a seam rather than a direct {@code ScenarioRunner} call because the driver genuinely
     * cannot do it itself: a {@link RunResult} is a finished log, and it does not carry the robot config,
     * the tick count or the gamepad script that produced it. Whoever ran the scenario the first time is
     * the only one who can run it again the same way, so they hand in the means to do it.
     */
    @FunctionalInterface
    public interface ScenarioReplayer {

        /**
         * Re-runs the scenario that produced {@code accepted} and returns the path to the new RLOG.
         *
         * <p>Must write to a different path from {@code accepted.rlogPath()}. Handing back the original
         * file is refused, because it is not a replay; and two writers on one RLOG path corrupt each
         * other's logs, which then fails as a determinism bug that is not one.
         *
         * @param accepted the run being reproduced, carrying its OpMode name and its original RLOG path
         * @return the new RLOG, to be compared field by field against the accepted run's frames
         * @throws IOException if the replay could not be run or its RLOG could not be written
         */
        Path replay(RunResult accepted) throws IOException;
    }

    private final Reproduction reproduction;
    private final Scorer.Score bestScore;
    private final String candidateDescription;
    private final boolean bestIsBaseline;
    private final Path acceptedRunRlog;
    private final Path replayRlog;
    private final List<RlogDecodedCompare.Mismatch> mismatches;
    private final List<Milestone> milestones;
    private final int offeredCount;
    private final int acceptedCount;
    private final Gate.Verdict baselineVerdict;

    StopReport(Reproduction reproduction, Scorer.Score bestScore, String candidateDescription,
            boolean bestIsBaseline, Path acceptedRunRlog, Path replayRlog,
            List<RlogDecodedCompare.Mismatch> mismatches, List<Milestone> milestones,
            int offeredCount, int acceptedCount, Gate.Verdict baselineVerdict) {
        this.reproduction = reproduction;
        this.bestScore = bestScore;
        this.candidateDescription = candidateDescription;
        this.bestIsBaseline = bestIsBaseline;
        this.acceptedRunRlog = acceptedRunRlog;
        this.replayRlog = replayRlog;
        this.mismatches = Collections.unmodifiableList(new ArrayList<>(mismatches));
        this.milestones = Collections.unmodifiableList(new ArrayList<>(milestones));
        this.offeredCount = offeredCount;
        this.acceptedCount = acceptedCount;
        this.baselineVerdict = baselineVerdict;
    }

    /** {@return how the second run came out} */
    public Reproduction reproduction() {
        return reproduction;
    }

    /** {@return true only when a score is safe to report — replayed, and identical field by field} */
    public boolean hasReportableScore() {
        return reproduction == Reproduction.REPRODUCED;
    }

    /**
     * The best score, once it has been reproduced.
     *
     * @throws IllegalStateException if the candidate did not reproduce, or if there was never one to
     *                               reproduce. Deliberately an exception rather than a null or a
     *                               best-effort number: the one mistake this class exists to prevent is a
     *                               caller printing a score the second run did not agree with.
     * @return the reproduced best score
     */
    public Scorer.Score reportedScore() {
        if (reproduction != Reproduction.REPRODUCED) {
            throw new IllegalStateException(
                    "there is no reportable score: reproduction came out " + reproduction
                            + ". A score the second run did not agree with is not a result, and this "
                            + "report will not hand one out. Read reproduction() and mismatches().");
        }
        return bestScore;
    }

    /**
     * The best score whether or not it reproduced, for diagnosis only, or null when there was never one.
     * Everything a human acts on goes through {@link #reportedScore()}; this is here so a determinism
     * failure can say which number failed to come back.
     *
     * @return the best score as first measured, or null when nothing was ever accepted
     */
    public Scorer.Score unreproducedScoreForDiagnosis() {
        return bestScore;
    }

    /** {@return true when no candidate beat the baseline, so the answer being reported is the baseline itself} */
    public boolean bestIsBaseline() {
        return bestIsBaseline;
    }

    /**
     * Where the accepted run's RLOG was when it was offered. The file may have been overwritten since by a
     * later candidate; the replay was compared against the frames held in memory, not this file.
     *
     * @return the accepted run's RLOG path, or null when nothing was replayed
     */
    public Path acceptedRunRlog() {
        return acceptedRunRlog;
    }

    /** {@return where the replay wrote its RLOG, or null when nothing was replayed} */
    public Path replayRlog() {
        return replayRlog;
    }

    /**
     * Every decoded-field mismatch between the accepted run and the replay. Empty unless reproduction
     * failed. {@code valueA} is the accepted run's value and {@code valueB} the replay's.
     *
     * @return every decoded-field mismatch, in frame order
     */
    public List<RlogDecodedCompare.Mismatch> mismatches() {
        return mismatches;
    }

    /** {@return every milestone the loop emitted, in order} */
    public List<Milestone> milestones() {
        return milestones;
    }

    /** {@return how many candidates were offered to the gate, not counting the baseline} */
    public int offeredCount() {
        return offeredCount;
    }

    /** {@return how many the gate accepted, not counting the baseline} */
    public int acceptedCount() {
        return acceptedCount;
    }

    /** {@return the gate's verdict on the loop's baseline} */
    public Gate.Verdict baselineVerdict() {
        return baselineVerdict;
    }

    /**
     * The report as a block of text. Literal {@code '\n'} throughout so the same inputs produce the same
     * bytes on every platform (R5); the RLOG paths it prints are part of those inputs.
     *
     * <p>A determinism failure is summarised, not dumped: one early divergence makes every later frame
     * differ on every key it touches, so the raw mismatch list of a real failure runs to thousands of lines
     * and hides the one that matters. The report names the first divergence, then up to
     * {@value #MISMATCH_KEYS_SHOWN} keys with how often each differed, earliest first.
     *
     * @return the whole report, newline-separated, ending in a newline
     */
    public String text() {
        StringBuilder out = new StringBuilder();
        out.append("LOOP STOP REPORT\n");
        out.append("  baseline: ").append(baselineVerdict.accepted()
                ? "passed the gate, so it was the starting last-best"
                : "did NOT pass the gate (blocked by " + baselineVerdict.blockedBy()
                        + "), so the loop started with no working answer").append('\n');
        out.append("  candidates offered: ").append(offeredCount).append('\n');
        out.append("  candidates accepted: ").append(acceptedCount).append('\n');
        out.append("  milestones emitted: ").append(milestones.size()).append('\n');

        switch (reproduction) {
            case NOT_ATTEMPTED:
                out.append("  result: NO LAST-BEST. The gate accepted nothing, not even the baseline, so\n");
                out.append("          there is no answer to report and nothing was replayed.\n");
                break;
            case DID_NOT_REPRODUCE:
                out.append("  result: DETERMINISM FAILURE (R5). The best answer's scenario was run a second\n");
                out.append("          time and the decoded fields differed, so its score is NOT reported.\n");
                out.append("          Fix the non-determinism before trusting any number here.\n");
                appendWhichAnswer(out);
                appendMismatchSummary(out);
                out.append("  unreproduced total -- NOT a result, for diagnosis only: ")
                        .append(bestScore.total()).append('\n');
                break;
            case REPRODUCED:
            default:
                out.append("  result: LAST-BEST, REPRODUCED. The scenario was run a second time and every\n");
                out.append("          decoded field matched.\n");
                appendWhichAnswer(out);
                for (String line : bestScore.breakdown().split("\n", -1)) {
                    if (!line.isEmpty()) {
                        out.append("    ").append(line).append('\n');
                    }
                }
                break;
        }

        if (!milestones.isEmpty()) {
            out.append("  milestone trail:\n");
            for (Milestone milestone : milestones) {
                out.append("    ").append(milestone.summaryLine()).append('\n');
            }
        }
        return out.toString();
    }

    /** The answer's own description and both RLOG paths, so a failure can be chased without the loop. */
    private void appendWhichAnswer(StringBuilder out) {
        out.append(bestIsBaseline
                ? "  the answer is the baseline (no candidate beat it):\n"
                : "  what the accepted candidate says it changed:\n");
        out.append("    ").append(candidateDescription).append('\n');
        out.append("  accepted run's RLOG: ").append(acceptedRunRlog)
                .append(" (compared from the frames held in memory; a later run may have overwritten this file)\n");
        out.append("  replay's RLOG:       ").append(replayRlog).append('\n');
    }

    /** First divergence, then per-key counts earliest first, capped at {@link #MISMATCH_KEYS_SHOWN} keys. */
    private void appendMismatchSummary(StringBuilder out) {
        RlogDecodedCompare.Mismatch firstFieldMismatch = null;
        Map<String, Integer> countByKey = new LinkedHashMap<>();
        Map<String, Integer> firstFrameByKey = new LinkedHashMap<>();
        for (RlogDecodedCompare.Mismatch mismatch : mismatches) {
            if (mismatch.isFrameCountMismatch()) {
                out.append("  frame count differs: accepted run ").append(mismatch.valueA)
                        .append(" frames, replay ").append(mismatch.valueB).append(" frames\n");
                continue;
            }
            if (firstFieldMismatch == null) {
                firstFieldMismatch = mismatch;
            }
            Integer seen = countByKey.get(mismatch.key);
            countByKey.put(mismatch.key, seen == null ? 1 : seen + 1);
            if (!firstFrameByKey.containsKey(mismatch.key)) {
                firstFrameByKey.put(mismatch.key, mismatch.frameIndex);
            }
        }
        if (firstFieldMismatch == null) {
            return;
        }
        out.append("  first divergence: frame ").append(firstFieldMismatch.frameIndex).append(", key ")
                .append(firstFieldMismatch.key).append(": accepted ").append(firstFieldMismatch.valueA)
                .append(", replay ").append(firstFieldMismatch.valueB).append('\n');
        int fieldMismatchCount = 0;
        for (int count : countByKey.values()) {
            fieldMismatchCount += count;
        }
        out.append("  differing keys (").append(countByKey.size()).append(" keys, ")
                .append(fieldMismatchCount).append(" mismatches; earliest divergence first):\n");
        // Mismatches arrive in frame order, so first-seen order is already earliest-divergence order.
        int shown = 0;
        for (Map.Entry<String, Integer> entry : countByKey.entrySet()) {
            if (shown == MISMATCH_KEYS_SHOWN) {
                out.append("    ... and ").append(countByKey.size() - MISMATCH_KEYS_SHOWN)
                        .append(" more keys\n");
                break;
            }
            out.append("    frame ").append(firstFrameByKey.get(entry.getKey())).append("  ")
                    .append(entry.getKey()).append("  x").append(entry.getValue()).append('\n');
            shown++;
        }
    }

    @Override
    public String toString() {
        return text();
    }
}
