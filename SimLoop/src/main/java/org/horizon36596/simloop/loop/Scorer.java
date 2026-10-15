package org.horizon36596.simloop.loop;

import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/**
 * Turns a finished run into one number, plus the breakdown that number came from
 * ({@code docs/process/scorer-interface.md} §1, §8).
 *
 * <pre>
 *     score = objective − Σ guardrail penalties
 * </pre>
 *
 * <h2>The breakdown is not a nicety</h2>
 * {@link #score(RunResult)} returns a {@link Score}, never a bare {@code double}. A total with no
 * breakdown is a number nobody can argue with — a human looking at a milestone cannot tell a genuinely
 * faster auto from one that got faster by skipping a guardrail nobody could evaluate. Every term is kept:
 * what it was called, what it concluded, what it charged, and why.
 *
 * <h2>Unknowns are carried, never absorbed</h2>
 * A guardrail that could not evaluate itself contributes no penalty, which makes it arithmetically
 * identical to one that passed. {@link Score#unknownCount()} and {@link Score#unknowns()} are what keep
 * the two distinguishable, so the acceptance {@link Gate} can refuse to promote a run whose envelope was mostly
 * unmeasured. This class deliberately does not decide that policy — scoring measures, the gate judges.
 *
 * <h2>Scenario discipline</h2>
 * Two scores are only comparable if they measure the same objective, read the same way, over the same
 * scenario. The objective's name, its direction, its metric description and the scenario name all ride
 * along on every {@link Score} and {@link Score#isComparableTo(Score)} checks all four, because "the number
 * went up" after quietly switching scenarios, flipping MINIMIZE to MAXIMIZE, or reading a different key
 * under the same objective name is an improvement that is not one.
 */
public final class Scorer {

    private final Objective objective;
    private final List<Guardrail> guardrails;

    /**
     * Builds a scorer from one objective and the envelope terms that can disqualify a run.
     *
     * @param objective  what "better" means for this scenario. Required.
     * @param guardrails the envelope terms, evaluated in order. Required, but may be empty.
     */
    public Scorer(Objective objective, List<Guardrail> guardrails) {
        if (objective == null) {
            throw new IllegalArgumentException("a scorer needs an objective");
        }
        if (guardrails == null) {
            throw new IllegalArgumentException("a scorer needs a guardrail list; pass an empty one "
                    + "explicitly if a scenario genuinely has no envelope, so that choice is visible");
        }
        this.objective = objective;
        this.guardrails = Collections.unmodifiableList(new ArrayList<>(guardrails));
    }

    /** {@return the objective being measured} */
    public Objective objective() {
        return objective;
    }

    /** {@return the envelope terms applied, in order} */
    public List<Guardrail> guardrails() {
        return guardrails;
    }

    /**
     * Scores one finished run.
     *
     * <p>Every guardrail is evaluated even after one has already returned infinity. A short-circuit would
     * be cheaper and would hide the rest of the breakdown, which is the half a human actually reads when
     * asking why a candidate was rejected.
     *
     * @param run the finished run to judge
     * @return the score, with the full per-guardrail breakdown attached
     * @throws RunResult.MetricNotFoundException    if the objective (or a guardrail that ignored its "do not
     *                                              throw" contract) reads a key the run never logged
     * @throws RunResult.MetricNotNumericException  if it reads a key whose values are not numbers
     * @throws Objective.NonFiniteMetricException   if the objective measured {@code NaN} or an infinity.
     *                                              {@link LoopDriver#offer} turns all three into a rejected
     *                                              candidate rather than a crashed loop.
     */
    public Score score(RunResult run) {
        // Measure the run ONCE. The metric is a caller-supplied function this class does not control, so
        // calling it twice - which is what asking the objective for both value() and rawMetric() used to do -
        // let a stateful metric put two different numbers in one breakdown, with only the first validated.
        double rawMetric = objective.rawMetric(run);
        double objectiveValue = objective.orient(rawMetric, run);

        Map<String, Guardrail.Verdict> verdicts = new LinkedHashMap<>();
        double totalPenalty = 0.0;
        for (Guardrail guardrail : guardrails) {
            Guardrail.Verdict verdict = guardrail.evaluate(run);
            if (verdict == null) {
                throw new IllegalStateException("guardrail '" + guardrail.name() + "' returned null rather "
                        + "than a Verdict. If it cannot evaluate itself from the run it must say so with "
                        + "Verdict.unknown(reason) -- a null read as 'fine' is the exact failure the "
                        + "UNKNOWN status exists to prevent.");
            }
            if (verdicts.putIfAbsent(guardrail.name(), verdict) != null) {
                throw new IllegalStateException("two guardrails are both called '" + guardrail.name()
                        + "'. Names are how a human reads the breakdown, so duplicates are rejected "
                        + "rather than silently overwriting one another.");
            }
            totalPenalty += verdict.penalty();
        }

        return new Score(objective.name(), objective.scenarioName(), objective.direction(),
                objective.metricDescription(), rawMetric, objectiveValue, totalPenalty, verdicts);
    }

    /** One run's score: the total, and every term that produced it. */
    public static final class Score {

        private final String objectiveName;
        private final String scenarioName;
        private final Objective.Direction direction;
        private final String metricDescription;
        private final double rawMetric;
        private final double objectiveValue;
        private final double totalPenalty;
        private final Map<String, Guardrail.Verdict> verdicts;

        Score(String objectiveName, String scenarioName, Objective.Direction direction,
                String metricDescription, double rawMetric, double objectiveValue, double totalPenalty,
                Map<String, Guardrail.Verdict> verdicts) {
            this.objectiveName = objectiveName;
            this.scenarioName = scenarioName;
            this.direction = direction;
            this.metricDescription = metricDescription;
            this.rawMetric = rawMetric;
            this.objectiveValue = objectiveValue;
            this.totalPenalty = totalPenalty;
            this.verdicts = Collections.unmodifiableMap(new LinkedHashMap<>(verdicts));
        }

        /**
         * The headline number: {@code objective − Σ penalties}, oriented so bigger is better.
         *
         * <p>{@link Double#NEGATIVE_INFINITY} when any envelope term was violated, which is what makes a
         * violating run ineligible no matter how good its objective was.
         *
         * @return the total, in the objective's own units, bigger-is-better
         */
        public double total() {
            return objectiveValue - totalPenalty;
        }

        /** {@return the metric in its own units and direction, for a human reading a milestone} */
        public double rawMetric() {
            return rawMetric;
        }

        /** {@return the objective's contribution, already oriented so bigger is better} */
        public double objectiveValue() {
            return objectiveValue;
        }

        /** {@return everything the guardrails charged, summed, in the objective's own units} */
        public double totalPenalty() {
            return totalPenalty;
        }

        /** {@return what the objective is called} */
        public String objectiveName() {
            return objectiveName;
        }

        /** {@return which scenario this was measured over} */
        public String scenarioName() {
            return scenarioName;
        }

        /** {@return which way the objective's raw metric points} */
        public Objective.Direction direction() {
            return direction;
        }

        /** {@return what number the objective read out of the run, in words -- see Objective#metricDescription()} */
        public String metricDescription() {
            return metricDescription;
        }

        /** {@return every guardrail's verdict, in the order they were applied} */
        public Map<String, Guardrail.Verdict> verdicts() {
            return verdicts;
        }

        /** {@return true if any envelope term was checked and found broken} */
        public boolean hasViolation() {
            return verdicts.values().stream().anyMatch(Guardrail.Verdict::isViolation);
        }

        /** {@return the names of every envelope term that was checked and found broken} */
        public List<String> violations() {
            return namesWhere(Guardrail.Verdict::isViolation);
        }

        /**
         * Names of every envelope term that could not be checked from this run.
         *
         * <p>The list a human should read before trusting a score: an envelope with four unknowns in it
         * is guarding less than it appears to.
         *
         * @return the guardrail names that returned unknown, in evaluation order
         */
        public List<String> unknowns() {
            return namesWhere(Guardrail.Verdict::isUnknown);
        }

        /** {@return how many envelope terms could not be checked} */
        public int unknownCount() {
            return unknowns().size();
        }

        /**
         * True when this score may be compared against {@code other}: same objective name, same direction,
         * same metric description, same scenario.
         *
         * <p>Comparing across scenarios is how a loop reports an improvement it did not make, so the
         * check is offered here rather than left to each caller to remember. The name alone is not enough:
         * an objective that kept its name and flipped to MAXIMIZE, or started reading a different key,
         * produces totals that mean something else, and a "better" total against the old one is noise.
         *
         * @param other the score to compare against; null is never comparable
         * @return true when all four match
         */
        public boolean isComparableTo(Score other) {
            return other != null
                    && objectiveName.equals(other.objectiveName)
                    && direction == other.direction
                    && metricDescription.equals(other.metricDescription)
                    && scenarioName.equals(other.scenarioName);
        }

        /**
         * How this score's objective reads, in one line, for a "not comparable" message:
         * {@code "auto finish time (MINIMIZE final value of Auto/finishTimeSec) on 'auto'"}.
         *
         * @return the objective's name, direction, metric and scenario
         */
        public String objectiveIdentity() {
            return objectiveName + " (" + direction + " " + metricDescription + ") on '" + scenarioName + "'";
        }

        private List<String> namesWhere(java.util.function.Predicate<Guardrail.Verdict> test) {
            List<String> names = new ArrayList<>();
            for (Map.Entry<String, Guardrail.Verdict> entry : verdicts.entrySet()) {
                if (test.test(entry.getValue())) {
                    names.add(entry.getKey());
                }
            }
            return Collections.unmodifiableList(names);
        }

        /** {@return the breakdown as a human-readable block — what a milestone report prints} */
        public String breakdown() {
            StringBuilder out = new StringBuilder();
            out.append(objectiveName).append(" on '").append(scenarioName).append("'\n");
            out.append("  raw metric      ").append(rawMetric).append('\n');
            out.append("  objective       ").append(objectiveValue).append(" (bigger is better)\n");
            for (Map.Entry<String, Guardrail.Verdict> entry : verdicts.entrySet()) {
                out.append("  ").append(entry.getKey()).append("  ").append(entry.getValue()).append('\n');
            }
            out.append("  TOTAL           ").append(total());
            if (unknownCount() > 0) {
                out.append("   [").append(unknownCount()).append(" guardrail(s) UNKNOWN -- this envelope ")
                        .append("is guarding less than it looks like: ").append(unknowns()).append(']');
            }
            return out.toString();
        }

        @Override
        public String toString() {
            return breakdown();
        }
    }
}
