package org.horizon36596.simloop.loop;

import java.util.function.ToDoubleFunction;

/**
 * What the loop is trying to make better, declared up front
 * ({@code docs/process/scorer-interface.md} §2).
 *
 * <h2>Bigger is always better, and the negation happens in exactly one place</h2>
 * Half of what anyone wants to optimize is a number that should go <i>down</i> — auto finish time, path
 * error, loop period. Rather than let every caller remember which way its own metric points, an
 * objective carries its {@link Direction} and {@link #value(RunResult)} returns an already-oriented
 * number: bigger is better, always, for every objective.
 *
 * <p>That negation lives in {@link #value(RunResult)} and nowhere else. It is one line, and the reason it
 * is worth naming is that a second copy of it — in a scorer, in a gate, in a comparison — is a sign flip
 * that makes the loop confidently optimize a metric backwards while every test still passes.
 * {@link #rawMetric(RunResult)} is there for logging and for a human reading a milestone, and it is the
 * only way to see the un-oriented number.
 *
 * <h2>Scenario and budget</h2>
 * {@link #scenarioName()} says which run this is measured over, so a score can never be compared across
 * two different scenarios by accident. {@link #metricDescription()} says what number is read out of that
 * run ({@code "final value of Auto/finishTimeSec"}), and it rides along on every score for the same reason:
 * two objectives with the same name that read different numbers are not the same objective. {@link #budget()} is the human's rough estimate; per §2 it tunes
 * milestone cadence and is deliberately <b>not</b> part of the score.
 */
public final class Objective {

    /** Which way a raw metric points. */
    public enum Direction {
        /** The raw metric is already "bigger is better" — points scored, distance covered. */
        MAXIMIZE,
        /** The raw metric is "smaller is better" — finish time, path error, loop period. */
        MINIMIZE
    }

    private final String name;
    private final String scenarioName;
    private final String metricDescription;
    private final Direction direction;
    private final Budget budget;
    private final ToDoubleFunction<RunResult> metric;

    private Objective(String name, String scenarioName, String metricDescription, Direction direction,
            Budget budget, ToDoubleFunction<RunResult> metric) {
        this.name = name;
        this.scenarioName = scenarioName;
        this.metricDescription = metricDescription;
        this.direction = direction;
        this.budget = budget;
        this.metric = metric;
    }

    /**
     * Declares an objective computed by an arbitrary function of the run.
     *
     * @param name         what a human calls this, e.g. {@code "auto finish time"}
     * @param scenarioName      the scenario it is measured over; scores from different scenarios are not
     *                          comparable, and {@link Scorer} refuses to mix them
     * @param metricDescription what {@code metric} reads, in words, e.g. {@code "tips logged minus fouls"}.
     *                          Part of what makes two scores comparable: {@code metric} is a function and
     *                          cannot be compared, so this sentence is how a changed metric is noticed.
     * @param direction         which way {@code metric} points
     * @param budget            the human's rough estimate; tunes milestone cadence only
     * @param metric            reads the headline number out of a finished run
     * @return the declared objective
     */
    public static Objective of(String name, String scenarioName, String metricDescription,
            Direction direction, Budget budget, ToDoubleFunction<RunResult> metric) {
        if (name == null || name.trim().isEmpty()) {
            throw new IllegalArgumentException("an objective needs a name a human can read in a milestone");
        }
        if (scenarioName == null || scenarioName.trim().isEmpty()) {
            throw new IllegalArgumentException("objective '" + name + "' needs a scenario name -- a score "
                    + "with no scenario can be compared against a score from a different scenario, which "
                    + "is how a loop 'improves' by measuring something else");
        }
        if (metricDescription == null || metricDescription.trim().isEmpty()) {
            throw new IllegalArgumentException("objective '" + name + "' needs a metric description -- it is "
                    + "how two scores tell whether they read the same number");
        }
        if (direction == null) {
            throw new IllegalArgumentException("objective '" + name + "' needs a direction");
        }
        if (budget == null) {
            throw new IllegalArgumentException("objective '" + name + "' needs a budget (Budget.unstated() "
                    + "is allowed and says so out loud)");
        }
        if (metric == null) {
            throw new IllegalArgumentException("objective '" + name + "' needs a metric");
        }
        return new Objective(name, scenarioName, metricDescription, direction, budget, metric);
    }

    /**
     * Declares an objective whose headline number is the <b>last</b> value logged under one RLOG key —
     * the common case (a finish time, a final pose error, a count at the end of a run).
     *
     * <p>Resolving the key through {@link RunResult#finalValue(String)} means a typo'd key throws with the
     * key named rather than scoring zero.
     *
     * <p><b>A key that is logged once and never updated scores whatever it was set to.</b> An RLOG is
     * delta-encoded, so the last value of a key written at frame 0 and never again is that frame-0 value.
     * For a {@link Direction#MINIMIZE} finish time initialised to {@code 0} and only set when the routine
     * finishes, a candidate that never finishes scores a perfect zero. Guard it with
     * {@link EnvelopeGuardrails#metricWasUpdated(String)} in the envelope, or initialise the key to a value
     * that scores badly.
     *
     * @param name         what a human calls this, e.g. {@code "auto finish time"}
     * @param scenarioName the scenario it is measured over
     * @param metricKey    the RLOG key to read, e.g. {@code "Auto/finishTimeSeconds"}
     * @param direction    which way the value under {@code metricKey} points
     * @param budget       the human's rough estimate; tunes milestone cadence only
     * @return the declared objective
     */
    public static Objective ofFinalValue(String name, String scenarioName, String metricKey,
            Direction direction, Budget budget) {
        return of(name, scenarioName, "final value of " + metricKey, direction, budget,
                run -> run.finalValue(metricKey));
    }

    /**
     * Declares an objective whose headline number is the largest value logged under one RLOG key.
     *
     * @param name         what a human calls this
     * @param scenarioName the scenario it is measured over
     * @param metricKey    the RLOG key to read
     * @param direction    which way the value under {@code metricKey} points
     * @param budget       the human's rough estimate; tunes milestone cadence only
     * @return the declared objective
     */
    public static Objective ofMaxValue(String name, String scenarioName, String metricKey,
            Direction direction, Budget budget) {
        return of(name, scenarioName, "largest value of " + metricKey, direction, budget,
                run -> run.maxValue(metricKey));
    }

    /**
     * Declares an objective whose headline number is the mean of every value under one RLOG key.
     *
     * @param name         what a human calls this
     * @param scenarioName the scenario it is measured over
     * @param metricKey    the RLOG key to read
     * @param direction    which way the value under {@code metricKey} points
     * @param budget       the human's rough estimate; tunes milestone cadence only
     * @return the declared objective
     */
    public static Objective ofMeanValue(String name, String scenarioName, String metricKey,
            Direction direction, Budget budget) {
        return of(name, scenarioName, "mean value of " + metricKey, direction, budget,
                run -> run.meanValue(metricKey));
    }

    /**
     * What a human calls this objective.
     *
     * @return the objective's name; never null or blank
     */
    public String name() {
        return name;
    }

    /**
     * The scenario this objective is measured over.
     *
     * @return the scenario name; never null or blank
     */
    public String scenarioName() {
        return scenarioName;
    }

    /**
     * What number this objective reads out of a run, in words: {@code "final value of Auto/finishTimeSec"}
     * for the key-based factories, or the caller's own sentence for {@link #of}.
     *
     * @return the metric description; never null or blank
     */
    public String metricDescription() {
        return metricDescription;
    }

    /**
     * Which way the raw metric points.
     *
     * @return the direction; never null
     */
    public Direction direction() {
        return direction;
    }

    /**
     * The human's rough time/token estimate. Tunes milestone cadence, never the score (§2).
     *
     * @return the budget; never null, though it may be {@link Budget#unstated()}
     */
    public Budget budget() {
        return budget;
    }

    /**
     * The metric exactly as measured, in its own units and its own direction — 12.4 seconds is 12.4 here
     * even for a minimizing objective. For logging and for humans; never for comparing two runs.
     *
     * @param run the finished run to measure
     * @return the metric in its own units and its own direction
     */
    public double rawMetric(RunResult run) {
        return metric.applyAsDouble(run);
    }

    /**
     * The objective's contribution to the score, oriented so that <b>bigger is better</b>.
     *
     * <p>This is the one place a minimizing objective is negated. Callers compare these numbers directly
     * and never re-orient them.
     *
     * @param run the finished run to measure
     * @return the metric in its own units, negated when {@link Direction#MINIMIZE}, so that a larger
     *         number is always the better candidate
     * @throws NonFiniteMetricException if the metric came out non-finite. A {@code NaN} objective compares
     *                               false against everything, so a loop using it would silently never
     *                               accept an improvement, and would look like it had simply plateaued.
     */
    public double value(RunResult run) {
        return orient(rawMetric(run), run);
    }

    /**
     * Validates an already-measured raw metric and orients it so that <b>bigger is better</b>.
     *
     * <p>This exists so a caller that needs both the raw number and the oriented one can measure the run
     * <b>once</b>. {@link #value(RunResult)} and {@link #rawMetric(RunResult)} each call the caller-supplied
     * metric function, and the metric is an arbitrary {@code ToDoubleFunction} this class does not control:
     * calling it twice for one score let a stateful metric report one number as the objective and a
     * different, unvalidated one as the raw metric in the same breakdown. It also meant the same run could
     * score differently on a second read, which is the determinism rule (R5) failing in the scorer.
     * (2026-09-17 adversarial review, finding objective-callback-evaluated-twice.)
     *
     * @param rawMetric the value the metric measured, exactly as {@link #rawMetric(RunResult)} returned it
     * @param run       the run it was measured on; used only to name the run if this throws
     * @return the oriented objective value, negated when this objective minimizes
     * @throws NonFiniteMetricException if {@code rawMetric} is non-finite, for the reason
     *                                  {@link #value(RunResult)} gives
     */
    public double orient(double rawMetric, RunResult run) {
        if (!Double.isFinite(rawMetric)) {
            throw new NonFiniteMetricException(name, rawMetric, run);
        }
        return direction == Direction.MINIMIZE ? -rawMetric : rawMetric;
    }

    /**
     * Thrown when an objective's metric measured {@code NaN} or an infinity.
     *
     * <p>A named type rather than a bare {@code IllegalStateException} so {@link LoopDriver#offer} can
     * catch exactly this -- "the objective could not be measured on this candidate" is a run to discard,
     * and it should not have to be told apart from an unrelated illegal-state bug by reading a message.
     */
    public static final class NonFiniteMetricException extends IllegalStateException {
        NonFiniteMetricException(String objectiveName, double measured, RunResult run) {
            super("objective '" + objectiveName + "' measured a non-finite value (" + measured
                    + ") on run '" + run.opModeName() + "'. This is not silently passed through: every "
                    + "comparison against NaN is false, so the loop would stop accepting improvements and "
                    + "look like it had plateaued.");
        }
    }

    /**
     * The human's rough estimate of what this objective is worth spending. Per
     * {@code scorer-interface.md} §2 it sets milestone cadence and is explicitly not a metric, so it is a
     * plain record rather than anything the scorer reads.
     */
    public static final class Budget {

        private final String description;

        private Budget(String description) {
            this.description = description;
        }

        /**
         * A budget the human stated, in their own words: {@code "about an hour"}, {@code "50k tokens"}.
         *
         * @param description the budget in the human's own words; must not be null or blank
         * @return the stated budget
         */
        public static Budget of(String description) {
            if (description == null || description.trim().isEmpty()) {
                throw new IllegalArgumentException("use Budget.unstated() rather than an empty budget, so "
                        + "a milestone report can say the budget is unstated instead of showing a blank");
            }
            return new Budget(description);
        }

        /**
         * No budget given. Said out loud rather than represented as zero or null.
         *
         * @return a budget whose description is the literal word {@code "unstated"}
         */
        public static Budget unstated() {
            return new Budget("unstated");
        }

        /**
         * The budget in the human's words.
         *
         * @return the description, or {@code "unstated"}; never null or blank
         */
        public String description() {
            return description;
        }

        @Override
        public String toString() {
            return description;
        }
    }
}
