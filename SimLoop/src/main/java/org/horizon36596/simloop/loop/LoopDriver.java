package org.horizon36596.simloop.loop;

import org.horizon36596.simloop.sim.RlogDecodedCompare;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Set;

/**
 * The bookkeeping half of the loop: scores each candidate run with its one fixed {@link Scorer}, judges
 * it with {@link Gate}, holds the last-best, emits a {@link Milestone} when the answer gets qualitatively
 * better, and writes the {@link StopReport} ({@code docs/process/scorer-interface.md} §5, §6, §8).
 *
 * <h2>What this does not do, and why that is the design</h2>
 * <b>This driver proposes nothing.</b> It never edits code, never picks what to try next, never runs
 * Gradle and never decides what the objective should be. Candidate runs are handed to it; it scores and
 * judges them and remembers the outcome. Proposing the edit is the {@code /run-loop} skill's job — a
 * Claude activity that happens outside this JVM.
 *
 * <p>That split is the safety story, not a layering preference. The gate exists to referee an optimizer.
 * An optimizer that lived inside the referee would be judging its own work, and the one thing a referee
 * must not be able to do is decide what it is refereeing. It also makes the driver a plain function of
 * its inputs, so all of this is testable without generating a single edit.
 *
 * <h2>The driver owns the scorer, so a candidate cannot bring its own</h2>
 * The {@link Scorer} is fixed when the driver is built, and {@link #offer} scores every run with it. A
 * candidate hands in a run, never a score. If candidates carried their own scores, a proposer could keep
 * every guardrail's name and swap its body, change the objective's key or direction, or score a different
 * run than the one it hands in — and the gate, which only compares scores, would see nothing wrong.
 *
 * <h2>A baseline first, so the first candidate has something to beat</h2>
 * The constructor takes the run of the code as it stands before the loop starts, with both supplied facts
 * confirmed, and judges it on every clause except improvement (there is nothing before it). The facts are
 * required rather than judged, because a baseline rejected on an unchecked fact would leave nothing to beat
 * and reopen the hole below. If the baseline passes, it is the starting
 * last-best and every candidate must beat it. Without a baseline the first candidate was accepted on the
 * other three clauses alone, so an edit that made the robot worse than where the loop started could
 * become the answer. If the baseline itself does not pass — the current code breaks the envelope, say —
 * there is no working answer yet, and the first candidate the gate accepts becomes the first one.
 *
 * <p>The baseline's score also pins the guardrail names every later score must carry. The scorer is fixed,
 * so those names can only change if a guardrail's {@link Guardrail#name()} is not stable, which is a bug in
 * the guardrail and is refused loudly.
 *
 * <h2>Last-best is always a fully working answer</h2>
 * The loop can be stopped at any moment, and §5 promises that whatever is held at that instant is a
 * complete working solution rather than a half-applied edit. So the driver stores a candidate only when
 * {@link Gate.Verdict#accepted()} is true, which by construction means every clause is HOLDS or
 * NOT_YET_APPLICABLE. A rejected candidate is discarded whole and the previous best stands untouched.
 *
 * <h2>What this guarantees only inside one JVM</h2>
 * All of this state lives in this object. The {@code /run-loop} skill today runs each candidate as a fresh
 * Gradle run, so one driver never sees more than one candidate there, and none of the last-best, milestone
 * or stop-report bookkeeping carries from one candidate to the next unless the caller keeps a driver alive
 * across them. The skill's own documentation says what it does by hand instead.
 *
 * <h2>Determinism (R5)</h2>
 * No clock of any kind, fake or real. Milestones are numbered in emission order, which is the only "when"
 * the loop needs; a timestamp would make two identical runs produce different reports for no gain.
 *
 * <p>Season-agnostic core (R7): touches {@link Gate}, {@link Scorer}, {@link RunResult} and
 * {@link RlogDecodedCompare}, and nothing from {@code teamcode}.
 */
public final class LoopDriver {

    /**
     * One accepted run, with the score this driver gave it, the two facts the gate cannot measure, and the
     * candidate's own account of what it changed.
     *
     * <p>Built only by the driver, after it has scored the run itself — which is why there is no public
     * factory.
     *
     * <p>The description is not decoration. It is half of what makes a milestone readable, and §7 is
     * blunt about why: every reward hack improves the number, so the number alone cannot tell a hack from
     * a real gain. Reading "raised the flywheel target by 200 rpm" beside a score that moved is what lets
     * a human notice when the actual answer is "stopped logging the term that was failing".
     */
    public static final class Candidate {

        private final String description;
        private final RunResult run;
        private final Scorer.Score score;
        private final Gate.SuppliedFacts facts;

        private Candidate(String description, RunResult run, Scorer.Score score, Gate.SuppliedFacts facts) {
            this.description = description;
            this.run = run;
            this.score = score;
            this.facts = facts;
        }

        /** {@return what this candidate changed, in its own words} */
        public String description() {
            return description;
        }

        /** {@return the finished run this candidate was scored from} */
        public RunResult run() {
            return run;
        }

        /** {@return what the driver's scorer made of that run} */
        public Scorer.Score score() {
            return score;
        }

        /** {@return the two facts no log carries: tests-green and review-passed} */
        public Gate.SuppliedFacts facts() {
            return facts;
        }

        @Override
        public String toString() {
            return "Candidate{" + description + ", total=" + score.total() + "}";
        }
    }

    /** What the loop did with one offered run. */
    public static final class Decision {

        private final boolean accepted;
        private final Scorer.Score score;
        private final Gate.Verdict verdict;
        private final Milestone milestone;

        Decision(boolean accepted, Scorer.Score score, Gate.Verdict verdict, Milestone milestone) {
            this.accepted = accepted;
            this.score = score;
            this.verdict = verdict;
            this.milestone = milestone;
        }

        /** {@return true when this candidate became the new last-best} */
        public boolean accepted() {
            return accepted;
        }

        /**
         * What the driver's scorer made of the run, accepted or not.
         *
         * @return the score, or null when the run could not be scored at all (the verdict says why)
         */
        public Scorer.Score score() {
            return score;
        }

        /**
         * The gate's full verdict, accepted or not. Never null.
         *
         * @return the gate's full verdict, accepted or not
         */
        public Gate.Verdict verdict() {
            return verdict;
        }

        /**
         * The milestone this candidate produced, or null. An accepted candidate that has not yet moved the
         * best by the threshold since the last milestone, and brought no guardrail back inside its budget,
         * is still the new best but does not get one.
         *
         * @return the milestone, or null when this candidate did not earn one
         */
        public Milestone milestone() {
            return milestone;
        }

        /** {@return true when this decision emitted a milestone} */
        public boolean emittedMilestone() {
            return milestone != null;
        }

        /**
         * A one-line status, deliberately shorter than {@link Gate.Verdict#toString()}.
         *
         * <p>The sibling {@code Verdict} prints its whole report from {@code toString()}, because it is
         * printed beside a {@code Score} in a milestone and the two should read as one document. A
         * {@code Decision} is printed once per offered candidate in a long run, where a multi-line block
         * per iteration buries everything worth seeing. The full reasoning is one call away at
         * {@link #verdict()}, and every milestone and the stop report carry it in full.
         */
        @Override
        public String toString() {
            return (accepted ? "ACCEPTED" : "DISCARDED")
                    + (milestone == null ? "" : " (milestone " + milestone.number() + ")")
                    + " " + verdict.blockedBy();
        }
    }

    private final Scorer scorer;
    private final Gate gate;
    private final double milestoneThreshold;
    private final Candidate baseline;
    private final Gate.Verdict baselineVerdict;
    private final Set<String> declaredGuardrailNames;
    private final List<Milestone> milestones = new ArrayList<>();

    private Candidate lastBest;
    /** The best as of the last milestone (or the accepted baseline); what the next milestone is measured from. */
    private Scorer.Score lastMilestoneScore;
    private int offeredCount;
    private int acceptedCount;

    /**
     * Builds a driver around one fixed scorer, starting from the baseline run of the code as it stands.
     *
     * @param scorer              scores every run this driver sees, the baseline included. Required, and
     *                            fixed for the driver's life: candidates cannot bring their own.
     * @param gate                the acceptance gate every run is judged by. Required.
     * @param milestoneThreshold  how much the total must rise, in score units, since the last milestone
     *                            before the rise alone is worth a new one. Must be finite and
     *                            non-negative; zero means every accepted improvement gets one.
     *                            <p>It is declared here rather than read off the {@link Objective} because
     *                            {@link Objective.Budget} is a sentence a human wrote, not a number — §2
     *                            made it prose on purpose and kept it out of the score. Cadence is a
     *                            property of how closely someone wants to watch this particular run, so
     *                            the loop is where it belongs.
     * @param baselineDescription what the baseline is, e.g. {@code "master at 3f2a1c0, before any edit"}
     * @param baselineRun         the run of the unedited code, on the same scenario the candidates will run
     * @param baselineFacts       tests-green for the unedited code, and review-passed meaning a human agreed
     *                            this is the code to start from. Both must be {@link Gate.Fact#CONFIRMED_TRUE}.
     * @throws IllegalArgumentException if either baseline fact is not {@code CONFIRMED_TRUE} — the baseline
     *                                  can then fail the gate only on its envelope, which is the one reason
     *                                  that legitimately means "no working answer yet" — or if the baseline
     *                                  run cannot be scored at all: a loop whose objective cannot measure the
     *                                  starting code is mis-wired, and finding that out here is cheaper than
     *                                  finding it on every candidate
     */
    public LoopDriver(Scorer scorer, Gate gate, double milestoneThreshold, String baselineDescription,
            RunResult baselineRun, Gate.SuppliedFacts baselineFacts) {
        if (scorer == null) {
            throw new IllegalArgumentException("a loop needs its one scorer; candidates never bring their own");
        }
        if (gate == null) {
            throw new IllegalArgumentException("a loop without a gate would accept anything");
        }
        if (!Double.isFinite(milestoneThreshold) || milestoneThreshold < 0.0) {
            throw new IllegalArgumentException(
                    "the milestone threshold must be finite and non-negative, got " + milestoneThreshold);
        }
        requireCandidateParts(baselineDescription, baselineRun, baselineFacts);
        if (baselineFacts.testsGreen() != Gate.Fact.CONFIRMED_TRUE
                || baselineFacts.reviewPassed() != Gate.Fact.CONFIRMED_TRUE) {
            throw new IllegalArgumentException("the baseline needs both facts CONFIRMED_TRUE, got " + baselineFacts
                    + ". TESTS_GREEN: run the verify command on the unedited code. REVIEW_PASSED: for a baseline "
                    + "this means a human agreed this is the code to start from. A baseline rejected on an "
                    + "unchecked fact would leave the first candidate with nothing to beat.");
        }
        this.scorer = scorer;
        this.gate = gate;
        this.milestoneThreshold = milestoneThreshold;

        Scorer.Score baselineScore;
        try {
            baselineScore = scorer.score(baselineRun);
        } catch (RunResult.MetricNotFoundException | RunResult.MetricNotNumericException
                | Objective.NonFiniteMetricException cannotScore) {
            throw new IllegalArgumentException("the baseline run '" + baselineRun.opModeName() + "' could not "
                    + "be scored, so the loop has nothing to start from: " + cannotScore.getMessage(), cannotScore);
        }
        this.baseline = new Candidate(baselineDescription, baselineRun, baselineScore, baselineFacts);
        this.declaredGuardrailNames = Collections.unmodifiableSet(
                new LinkedHashSet<>(baselineScore.verdicts().keySet()));
        this.baselineVerdict = gate.judge(baselineScore, Gate.LastBest.firstCandidateOfRun(), baselineFacts);
        if (baselineVerdict.accepted()) {
            lastBest = baseline;
            lastMilestoneScore = baselineScore;
        }
    }

    /**
     * Score one candidate run with this driver's scorer, judge it, and, if the gate accepts it, make it the
     * new last-best.
     *
     * <p>A rejected candidate is discarded whole: the previous last-best is not touched, not partially
     * updated, and not annotated. There is no state in this class that a rejected candidate can reach
     * beyond the offered count.
     *
     * <p><b>A run that cannot be scored is a rejection, not a crash.</b> If the objective reads a key the
     * run never logged, reads a value that is not a number, or measures {@code NaN}, the candidate is
     * discarded with a verdict saying so ({@link Gate#judgeUnscorable}), and the loop carries on. The edit
     * that broke the measurement is the one to throw away.
     *
     * @param description what this candidate changed, in its own words, for the milestone diff. Required.
     * @param run         the finished run of the edited code. Required.
     * @param facts       tests-green and review-passed, neither of which is in any log. Required.
     * @return what happened, including the gate's full verdict either way
     * @throws IllegalStateException if the run's score carries a different set of guardrail names from the
     *                               baseline's — possible only when a guardrail's name is not stable
     */
    public Decision offer(String description, RunResult run, Gate.SuppliedFacts facts) {
        requireCandidateParts(description, run, facts);
        offeredCount++;

        Scorer.Score score;
        try {
            score = scorer.score(run);
        } catch (RunResult.MetricNotFoundException | RunResult.MetricNotNumericException
                | Objective.NonFiniteMetricException cannotScore) {
            return new Decision(false, null, gate.judgeUnscorable(cannotScore.getMessage(), facts), null);
        }
        requireDeclaredGuardrails(score);

        Gate.LastBest toBeat = lastBest == null
                ? Gate.LastBest.firstCandidateOfRun()
                : Gate.LastBest.of(lastBest.score());
        Gate.Verdict verdict = gate.judge(score, toBeat, facts);
        if (!verdict.accepted()) {
            return new Decision(false, score, verdict, null);
        }

        Candidate accepted = new Candidate(description, run, score, facts);
        lastBest = accepted;
        acceptedCount++;
        return new Decision(true, score, verdict, maybeEmitMilestone(accepted));
    }

    /** Rejects a missing description, run or facts, for the baseline and every candidate alike. */
    private static void requireCandidateParts(String description, RunResult run, Gate.SuppliedFacts facts) {
        if (description == null || description.trim().isEmpty()) {
            throw new IllegalArgumentException(
                    "a candidate must say what it changed; an unexplained edit cannot be reviewed at a "
                            + "milestone, which is where gaming is supposed to become visible");
        }
        if (run == null) {
            throw new IllegalArgumentException(
                    "a candidate must carry its run; the driver scores it and the stop report replays it");
        }
        if (facts == null) {
            throw new IllegalArgumentException(
                    "a candidate must carry the two facts the gate cannot measure; see Gate.SuppliedFacts");
        }
    }

    /**
     * Refuses a score whose guardrail names differ from the baseline's.
     *
     * <p>With one fixed scorer the names cannot legitimately change, so a difference means a guardrail
     * whose {@link Guardrail#name()} is not stable from run to run. That is a bug in the guardrail, not a
     * property of the candidate, and continuing would compare envelopes that do not line up.
     */
    private void requireDeclaredGuardrails(Scorer.Score score) {
        if (!score.verdicts().keySet().equals(declaredGuardrailNames)) {
            throw new IllegalStateException("this run's score carries guardrails " + score.verdicts().keySet()
                    + " but the baseline's carried " + declaredGuardrailNames + ". The scorer is fixed, so "
                    + "a guardrail's name() is changing between runs; names must be stable.");
        }
    }

    /**
     * Works out whether this acceptance deserves a milestone, and records one if so.
     *
     * <p>Everything is measured from the best as of the last milestone, not from the immediately previous
     * best. Measured from the previous best, a run of small gains that together cleared the threshold many
     * times over would never emit a milestone, because no single step did.
     *
     * @return the milestone, or null when nothing qualitatively new happened
     */
    private Milestone maybeEmitMilestone(Candidate accepted) {
        List<Milestone.Reason> reasons = new ArrayList<>();
        List<String> newlyHolding = new ArrayList<>();

        if (lastMilestoneScore == null) {
            reasons.add(Milestone.Reason.FIRST_ACCEPTED_CANDIDATE);
        } else {
            if (accepted.score().total() - lastMilestoneScore.total() >= milestoneThreshold) {
                reasons.add(Milestone.Reason.IMPROVED_BY_AT_LEAST_THE_THRESHOLD);
            }
            // Only a guardrail present on both sides counts, and only OVER_BUDGET -> HOLDS. An accepted score
            // can hold no VIOLATED or UNKNOWN term (the gate rejects both), and with one fixed scorer no new
            // name can appear, so bringing a soft budget back inside its limit is the one real transition.
            for (Map.Entry<String, Guardrail.Verdict> entry : accepted.score().verdicts().entrySet()) {
                Guardrail.Verdict before = lastMilestoneScore.verdicts().get(entry.getKey());
                if (entry.getValue().holdsCleanly() && before != null && before.isOverBudget()) {
                    newlyHolding.add("guardrail " + entry.getKey());
                }
            }
            if (!newlyHolding.isEmpty()) {
                reasons.add(Milestone.Reason.A_GUARDRAIL_IS_NOW_CHECKED_AND_HOLDING);
            }
        }

        if (reasons.isEmpty()) {
            return null;
        }
        Milestone milestone = new Milestone(milestones.size() + 1, reasons, lastMilestoneScore,
                accepted.score(), accepted.description(), newlyHolding);
        milestones.add(milestone);
        lastMilestoneScore = accepted.score();
        return milestone;
    }

    /** {@return true once there is a working answer: an accepted baseline or an accepted candidate} */
    public boolean hasLastBest() {
        return lastBest != null;
    }

    /**
     * The best answer so far — the baseline itself until a candidate beats it.
     *
     * @throws IllegalStateException if neither the baseline nor any candidate has passed the gate.
     *                               Deliberately not null and deliberately not an empty placeholder: "the
     *                               loop has no answer yet" is a state the caller has to handle, and a
     *                               stand-in score would be a number nobody measured.
     *
     * @return the best accepted so far
     */
    public Candidate lastBest() {
        if (lastBest == null) {
            throw new IllegalStateException(
                    "there is no last-best: the gate has accepted nothing yet, not even the baseline. Check "
                            + "hasLastBest() first.");
        }
        return lastBest;
    }

    /** {@return the baseline this driver was built with, scored by its scorer, whether or not it passed} */
    public Candidate baseline() {
        return baseline;
    }

    /** {@return the gate's verdict on the baseline, judged without the improvement clause} */
    public Gate.Verdict baselineVerdict() {
        return baselineVerdict;
    }

    /** {@return every milestone emitted so far, in order} */
    public List<Milestone> milestones() {
        return Collections.unmodifiableList(new ArrayList<>(milestones));
    }

    /** {@return how many candidates have been offered, not counting the baseline} */
    public int offeredCount() {
        return offeredCount;
    }

    /** {@return how many candidates the gate accepted, not counting the baseline} */
    public int acceptedCount() {
        return acceptedCount;
    }

    /** {@return the threshold this loop was built with, in score units} */
    public double milestoneThreshold() {
        return milestoneThreshold;
    }

    /**
     * Stop the loop and report — replaying the best candidate's scenario first, and reporting a
     * determinism failure instead of a score if the replay differs from the accepted run.
     *
     * <p>The replay happens here, at the moment of reporting, rather than at acceptance. The reason is
     * cost: a loop may accept a hundred candidates and only the last one's number is ever acted on, so
     * re-running every acceptance would double the cost of the whole loop to prove ninety-nine results
     * nobody will read. What a human acts on is what gets proved.
     *
     * <p><b>The replay is compared against the frames held in memory, not against the file.</b> The
     * accepted run's frames were decoded when it was offered, and they are what its score came from. Its
     * RLOG file may have been overwritten since — the skill reuses one file name per candidate, so a later
     * candidate, even a rejected one, writes over the accepted one's log — and re-reading that file would
     * report a determinism failure that never happened.
     *
     * <p><b>This comparison is of values only, not of logged types.</b> {@link RunResult#frames()} holds
     * each value as {@link RlogDecodedCompare#decode(Path)} renders it, without its type, so the replay is
     * decoded the same way. {@link RlogDecodedCompare#compare(Path, Path)} also checks each field's logged
     * type; this does not, so a field whose type changed while its rendered value did not would still read
     * as reproduced. That cannot move the score — every number the scorer reads is parsed from the rendered
     * value — and it is what keeps the comparison on the exact frames the score came from.
     *
     * @param replayer re-runs the accepted candidate's scenario. Required, even when there is no
     *                 last-best — a caller that cannot reproduce its runs should find that out when it
     *                 builds the report, not when the loop happens to have found an answer.
     * @return the report; {@link StopReport#reportedScore()} is available only if it reproduced
     * @throws IllegalArgumentException if {@code replayer} is null
     * @throws IllegalStateException    if the replayer returns no path, returns the accepted run's own file
     *                                  (comparing a run with itself proves nothing), or returns a path
     *                                  where no RLOG was written
     * @throws IOException              if the replayer fails, or the replay's RLOG cannot be read
     */
    public StopReport stopReport(StopReport.ScenarioReplayer replayer) throws IOException {
        if (replayer == null) {
            throw new IllegalArgumentException(
                    "a replayer is required: this report reproduces the best candidate before it reports "
                            + "a score, and a score the second run did not agree with is not a result");
        }
        if (lastBest == null) {
            return new StopReport(StopReport.Reproduction.NOT_ATTEMPTED, null, null, false, null, null,
                    Collections.<RlogDecodedCompare.Mismatch>emptyList(), milestones, offeredCount,
                    acceptedCount, baselineVerdict);
        }

        Path original = lastBest.run().rlogPath();
        Path second = replayer.replay(lastBest.run());
        if (second == null) {
            throw new IllegalStateException("the replayer returned no RLOG for " + lastBest.run().opModeName());
        }
        if (isTheSameFile(original, second)) {
            throw new IllegalStateException(String.format(Locale.ROOT,
                    "the replayer wrote the second run to the same file as the first (%s vs %s). Handing "
                            + "back the original file is not a replay, and two writers on one RLOG path "
                            + "corrupt each other's logs.",
                    original, second));
        }
        if (!Files.exists(second) || Files.size(second) == 0) {
            throw new IllegalStateException("the replayer returned " + second + " for "
                    + lastBest.run().opModeName() + ", but no RLOG was written there, so there is nothing "
                    + "to compare the accepted run against");
        }

        List<RlogDecodedCompare.Mismatch> mismatches = RlogDecodedCompare.compareFrames(
                lastBest.run().frames(), RlogDecodedCompare.decode(second), RlogDecodedCompare.EXCLUDED_KEYS);
        StopReport.Reproduction reproduction = mismatches.isEmpty()
                ? StopReport.Reproduction.REPRODUCED
                : StopReport.Reproduction.DID_NOT_REPRODUCE;
        return new StopReport(reproduction, lastBest.score(), lastBest.description(), lastBest == baseline,
                original, second, mismatches, milestones, offeredCount, acceptedCount, baselineVerdict);
    }

    /**
     * Whether two paths name one file on disk.
     *
     * <p>Not {@code Path.equals}, which compares how a path was <i>spelled</i>. {@code build/sim/a.rlog}
     * and an absolute path to that same file are different {@code Path} objects and the same file, and a
     * replayer that returned the second spelling would walk straight past an equality check.
     *
     * <p>{@link java.nio.file.Files#isSameFile} needs both files to exist; if either does not, there is no
     * way they are the same file.
     */
    private static boolean isTheSameFile(Path original, Path second) {
        if (original.toAbsolutePath().normalize().equals(second.toAbsolutePath().normalize())) {
            return true;
        }
        try {
            return Files.exists(original) && Files.exists(second) && Files.isSameFile(original, second);
        } catch (IOException cannotTell) {
            // This does swallow the error, on purpose: the two paths are already known to be spelled
            // differently, and a filesystem that cannot answer isSameFile is not a reason to abandon the
            // report. The replay is compared against the in-memory frames either way.
            return false;
        }
    }
}
