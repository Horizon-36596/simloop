package org.horizon36596.simloop.loop;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * One milestone: a point where the loop's best answer got qualitatively better, and the side-by-side
 * evidence for saying so ({@code docs/process/scorer-interface.md} §5).
 *
 * <h2>Event-driven, never on a clock</h2>
 * A milestone is emitted because something happened — the best improved by at least the loop's declared
 * threshold since the last milestone, or a guardrail that was over its soft budget then is holding now.
 * It is never emitted
 * because an interval elapsed. That is not a style preference: a time-sliced milestone stream is mostly
 * made of entries that say nothing changed, and a reader who has learned that milestones are usually
 * empty stops reading them. §5 puts the point plainly — the milestone is where gaming becomes visible, so
 * it only exists when there is something to look at.
 *
 * <p>This class reads no clock, which is also why it has no timestamp field. Milestones are numbered in
 * the order they were emitted, and that ordering is the only "when" the loop needs. A wall-clock stamp
 * here would break R5 for nothing: it would make two identical runs produce different milestone text.
 *
 * <h2>Why the diff is the whole point</h2>
 * {@link #text()} prints both scores' full breakdowns next to each other plus the candidate's own
 * description of what it changed. A milestone that said only "score improved from 11.2 to 12.8" is where
 * a reward hack survives, because every hack improves the number — that is what makes it a hack. Seeing
 * <i>which term</i> moved, and reading the candidate's account of what it edited beside it, is what lets a
 * human notice that the objective went up because a guardrail quietly stopped being checked.
 *
 * <p>Season-agnostic core (R7).
 */
public final class Milestone {

    /**
     * Why a milestone was emitted. More than one can apply to the same milestone.
     *
     * <p>There is no "a gate clause now holds" reason, though §5's "qualitatively new capability" sounds
     * like one: both sides of a milestone are accepted answers, and an accepted verdict already has every
     * clause HOLDS or NOT_YET_APPLICABLE, so no clause can newly start holding between them.
     */
    public enum Reason {
        /**
         * The first answer the gate accepted, when the loop's baseline had not passed the gate. The loop went
         * from having no working answer to having one, which is the largest qualitative step it can take.
         * An accepted baseline is the starting point rather than a milestone, so this does not fire then.
         */
        FIRST_ACCEPTED_CANDIDATE,
        /**
         * The total rose by at least the loop's milestone threshold since the last milestone (or since the
         * accepted baseline, before the first one). An accepted improvement smaller than that still becomes
         * the new best — it passed the gate — but does not get a milestone yet, which is what stops a long
         * run from producing hundreds of them. The gains accumulate: ten small steps that together clear the
         * threshold emit one.
         */
        IMPROVED_BY_AT_LEAST_THE_THRESHOLD,
        /**
         * A guardrail that was over its soft budget at the last milestone (or in the accepted baseline) is
         * checked and holding now: the answer stopped doing something it was being penalised for, which is a
         * real gain even when the score barely moves.
         *
         * <p>That is the only route. "Was UNKNOWN, is now holding" cannot happen because an UNKNOWN term
         * blocks the gate, so no accepted answer carries one. "Newly added to the envelope" cannot happen
         * because the loop scores every run with one fixed scorer, so its guardrail names never change.
         */
        A_GUARDRAIL_IS_NOW_CHECKED_AND_HOLDING
    }

    private final int number;
    private final List<Reason> reasons;
    private final Scorer.Score previousBest;
    private final Scorer.Score newBest;
    private final String candidateDescription;
    private final List<String> newlyHolding;

    Milestone(int number, List<Reason> reasons, Scorer.Score previousBest, Scorer.Score newBest,
            String candidateDescription, List<String> newlyHolding) {
        this.number = number;
        this.reasons = Collections.unmodifiableList(new ArrayList<>(reasons));
        this.previousBest = previousBest;
        this.newBest = newBest;
        this.candidateDescription = candidateDescription;
        this.newlyHolding = Collections.unmodifiableList(new ArrayList<>(newlyHolding));
    }

    /**
     * Which milestone this is, counting from 1 in emission order. The loop's only notion of "when".
     *
     * @return which milestone this is, counting from 1 in emission order
     */
    public int number() {
        return number;
    }

    /**
     * Every reason this milestone was emitted. Never empty.
     *
     * @return every reason this milestone was emitted
     */
    public List<Reason> reasons() {
        return reasons;
    }

    /**
     * {@return what this milestone is measured from: the best at the previous milestone, or the accepted
     * baseline before the first one; null when this is {@link Reason#FIRST_ACCEPTED_CANDIDATE}}
     */
    public Scorer.Score previousBest() {
        return previousBest;
    }

    /** {@return the new best} */
    public Scorer.Score newBest() {
        return newBest;
    }

    /** {@return what the candidate said it changed, in its own words} */
    public String candidateDescription() {
        return candidateDescription;
    }

    /**
     * The guardrails that were over their soft budget in {@link #previousBest()} and are holding now. Empty
     * when the milestone is purely a score improvement.
     *
     * <p>Each entry reads {@code "guardrail loopPeriodMs"}, in the order the scorer declares its guardrails.
     *
     * @return an unmodifiable list of the newly-holding guardrail names
     */
    public List<String> newlyHolding() {
        return newlyHolding;
    }

    /**
     * How much the total moved since {@link #previousBest()}. Zero when there was none.
     *
     * @return the rise in {@link Scorer.Score#total()} since the previous milestone (or the accepted
     *         baseline), in the objective's own units, bigger-is-better
     */
    public double improvement() {
        if (previousBest == null) {
            return 0.0;
        }
        return newBest.total() - previousBest.total();
    }

    /**
     * The milestone as a block of text: why it fired, what changed in the candidate's own words, and both
     * breakdowns side by side. This is the thing a human actually reads, so it spells everything out
     * rather than assuming the reader will go and look the scores up.
     *
     * <p>Every line ends in a literal {@code '\n'} rather than the platform separator, so the same run
     * produces the same bytes on a student's Windows machine and on a CI runner (R5).
     *
     * @return the whole milestone, newline-separated, ending in a newline
     */
    public String text() {
        StringBuilder out = new StringBuilder();
        out.append("MILESTONE ").append(number).append('\n');
        out.append("  why: ").append(reasons).append('\n');
        if (!newlyHolding.isEmpty()) {
            out.append("  newly holding: ").append(newlyHolding).append('\n');
        }
        out.append("  what the candidate says it changed:\n");
        out.append("    ").append(candidateDescription).append('\n');

        out.append("  previous best (at the last milestone, or the baseline):\n");
        if (previousBest == null) {
            out.append("    (none — the baseline did not pass the gate, and this is the first answer that did)\n");
        } else {
            out.append(indent(previousBest.breakdown()));
        }
        out.append("  new best:\n");
        out.append(indent(newBest.breakdown()));
        return out.toString();
    }

    /** Indents a breakdown by four spaces so it reads as a block nested under its heading. */
    private static String indent(String breakdown) {
        StringBuilder out = new StringBuilder();
        for (String line : breakdown.split("\n", -1)) {
            if (line.isEmpty()) {
                continue;
            }
            out.append("    ").append(line).append('\n');
        }
        return out.toString();
    }

    /**
     * The milestone in one line, for the trail in the stop report:
     * {@code "#2 [IMPROVED_BY_AT_LEAST_THE_THRESHOLD] +2.5 -> -9.44: cut the idle step before the tip"}.
     *
     * @return one line, without a trailing newline
     */
    public String summaryLine() {
        return "#" + number + " " + reasons + " +" + improvement() + " -> " + newBest.total() + ": "
                + candidateDescription;
    }

    @Override
    public String toString() {
        return text();
    }
}
