package org.horizon36596.simloop.loop;

import java.util.Arrays;
import java.util.Collections;
import java.util.List;

/**
 * The seven starter envelope terms from {@code docs/process/scorer-interface.md} §3, sorted honestly into
 * the ones a run's log can actually answer and the ones it cannot.
 *
 * <h2>Three of the seven are log facts. Four are not.</h2>
 * This is the sorting §3 asks for, and getting it wrong in the comfortable direction — letting
 * an unanswerable term quietly return "holds" — is what turns the safety envelope into "whatever happened
 * to be loggable".
 *
 * <table>
 *   <caption>§3's starter set</caption>
 *   <tr><th>§3 term</th><th>Answerable from an RLOG?</th></tr>
 *   <tr><td>No {@code NaN}/{@code Inf} in pose, velocity or any control output</td>
 *       <td><b>Yes</b> — {@link #noNonFiniteValues()}</td></tr>
 *   <tr><td>No actuator commanded past its end-limit / configured bound</td>
 *       <td><b>Yes, given the bound</b> — {@link #withinBound(String, double, double)}</td></tr>
 *   <tr><td>Logging overhead stays under its loop-time budget</td>
 *       <td><b>Yes, given a budget and a logged loop time</b> — {@link #loopTimeUnderBudget(String, double)},
 *           which says {@code UNKNOWN} when the key is absent</td></tr>
 *   <tr><td>All invariant-tier tests green</td>
 *       <td><b>No</b> — a Gradle exit code. {@link #invariantTestsGreen()}</td></tr>
 *   <tr><td>{@code R8}: hardware runs only after the same code passed in sim</td>
 *       <td><b>No</b> — a fact about history. {@link #passedInSimFirst()}</td></tr>
 *   <tr><td>Robot comes to a safe stop on disable / opmode end</td>
 *       <td><b>Partly</b> — {@link #safeStopAtEnd(String, double)} answers it only for a named output key,
 *           and says so; "the physical robot stopped" is not a thing a log can prove</td></tr>
 *   <tr><td>Drivetrain FK/IK round-trip holds</td>
 *       <td><b>No, not from a run</b> — it is a property test over the kinematics.
 *           {@link #forwardInverseKinematicsRoundTrip()}</td></tr>
 * </table>
 *
 * <p>The three that a log cannot answer at all are still <b>declared</b> here, each one returning
 * {@code UNKNOWN} with a reason, so the sort above is written in code and not only in prose.
 *
 * <p><b>Do not put those three in a loop scorer's envelope.</b> {@link Gate} rejects any candidate whose
 * envelope holds an {@code UNKNOWN} term, so an always-{@code UNKNOWN} guardrail makes every candidate
 * unacceptable, forever. Each of the three is enforced somewhere else, and that is where to look:
 * <ul>
 *   <li>Invariant-tier tests green: {@link Gate.Clause#TESTS_GREEN}, from
 *       {@link Gate.SuppliedFacts#testsGreen()} — the caller runs the verify command and says what
 *       happened.</li>
 *   <li>FK/IK round trip: an invariant-tier property test, so it runs inside that same verify command and
 *       is {@link Gate.Clause#TESTS_GREEN} again.</li>
 *   <li>R8, sim before hardware: the loop only ever runs in sim, so it cannot break R8 by construction.
 *       Putting a loop's answer on a robot is a human's step, outside the loop.</li>
 * </ul>
 *
 * <p>Separately from the seven, {@link #runIsComplete()} and {@link #metricWasUpdated(String)} close two
 * cheap reward hacks: scoring better by logging less, and scoring perfectly on a key nobody updated.
 *
 * <p><b>Every built-in term returns a verdict and never throws</b>, as {@link Guardrail#evaluate} asks. A
 * key whose values are not numbers is {@code UNKNOWN} with the key named, not an exception out of the
 * scorer.
 *
 * <p><b>Every threshold here is {@code GUESSED}</b> until a real loop run tunes it, per the brief's
 * provenance rule. Nothing in this file has been calibrated against a robot.
 */
public final class EnvelopeGuardrails {

    private EnvelopeGuardrails() {
    }

    // ---------------------------------------------------------------------------------------------
    // Answerable from the log
    // ---------------------------------------------------------------------------------------------

    /**
     * §3: no {@code NaN} or infinity in any logged number.
     *
     * <p>The cleanest log fact in the set. A non-finite pose or control output is a broken run whatever
     * else it achieved, and it is visible in the decoded fields without any extra configuration.
     *
     * @return a guardrail that violates when any key logged a {@code NaN} or an infinity
     */
    public static Guardrail noNonFiniteValues() {
        return named("noNonFiniteValues", run -> {
            if (!run.hasNonFiniteValue()) {
                return Guardrail.Verdict.holds();
            }
            return Guardrail.Verdict.violated("non-finite values logged under " + run.nonFiniteKeys());
        });
    }

    /**
     * §3: an actuator is never commanded past its configured bound.
     *
     * <p>Answerable only because the caller supplies the bound — the core cannot know a season
     * mechanism's limits without importing season code, which R7 forbids. A key that was never logged is
     * {@code UNKNOWN}, not a pass: "the slide never exceeded its limit" and "nobody logged the slide" are
     * different facts and must not score the same.
     *
     * @param key        the logged command/measurement to bound
     * @param minAllowed inclusive lower bound, in the same unit and frame as the values logged under
     *                   {@code key} (this class cannot know that unit -- the season scenario that names
     *                   the key owns it)
     * @param maxAllowed inclusive upper bound, same unit and frame as {@code minAllowed}
     * @return a guardrail that violates when any value under {@code key} leaves the bounds
     */
    public static Guardrail withinBound(String key, double minAllowed, double maxAllowed) {
        requireFiniteThreshold(key, "minAllowed", minAllowed);
        requireFiniteThreshold(key, "maxAllowed", maxAllowed);
        if (minAllowed > maxAllowed) {
            throw new IllegalArgumentException("bound for '" + key + "' is inverted: min " + minAllowed
                    + " > max " + maxAllowed);
        }
        // The bounds are IN THE NAME, and that is load-bearing rather than cosmetic. Gate proves the envelope
        // was retained by comparing guardrail names between the last-best run and a candidate, so a name that
        // says only "withinBound(slide)" lets a candidate swap a 10-inch limit for a 10,000-inch one, drop
        // nothing by name, and be reported as holding the same envelope. With the numbers in the name, a
        // loosened bound reads as a dropped term, which is what it is. (2026-09-17 adversarial review,
        // finding guardrail-identity-by-name.)
        //
        // It cuts both ways and that is accepted, not overlooked: TIGHTENING a bound mid-run also changes the
        // name, so Gate's envelope clause fails with "the envelope no longer carries guardrail(s) the
        // last-best did" for a change that made the robot safer. Gate's verdict names both readings, and the
        // fix for the safe one is to re-baseline the last-best. Erring the other way -- comparing on the key
        // alone -- means a loosened guardrail is indistinguishable from an unchanged one for the rest of the
        // run, and that is the failure this whole clause exists to catch.
        return named("withinBound(" + key + ", " + minAllowed + ", " + maxAllowed + ")", run -> {
            if (!run.hasKey(key)) {
                return Guardrail.Verdict.unknown("'" + key + "' was never logged, so this bound could not "
                        + "be checked. An unlogged actuator is not a safe one.");
            }
            double min = run.minValue(key);
            double max = run.maxValue(key);
            // NaN first, and before any comparison. A single NaN frame poisons both min and max (the
            // stream reduction propagates it), and every comparison against NaN is false -- so without
            // this the bound check would return HOLDS on a run that commanded a non-finite value to the
            // actuator. "Nobody can say where it went" is not "it stayed inside its limits".
            if (!Double.isFinite(min) || !Double.isFinite(max)) {
                return Guardrail.Verdict.violated("'" + key + "' logged a non-finite value, so its bound "
                        + "cannot be judged -- treated as a violation, never as staying in bounds");
            }
            if (min < minAllowed || max > maxAllowed) {
                return Guardrail.Verdict.violated("'" + key + "' ranged [" + min + ", " + max
                        + "], outside the allowed [" + minAllowed + ", " + maxAllowed + "]");
            }
            return Guardrail.Verdict.holds();
        });
    }

    /**
     * §3 + {@code vision.md} §5: the loop stays inside its time budget, so the robot is never degraded in
     * order to log.
     *
     * <p>{@code UNKNOWN} when the loop-time key is absent — which is the common case today, because no
     * scenario logs one yet. That absence is exactly the kind of gap this class refuses to paper over.
     *
     * @param loopTimeKey     the logged per-loop duration, milliseconds
     * @param budgetMillis    the ceiling. {@code GUESSED} — no measured budget exists yet
     * @return a guardrail that VIOLATES when the worst loop exceeds {@code budgetMillis} - an over-budget
     *         loop is an envelope violation, not a graded charge, so it carries the infinite penalty
     */
    public static Guardrail loopTimeUnderBudget(String loopTimeKey, double budgetMillis) {
        requireFiniteThreshold(loopTimeKey, "budgetMillis", budgetMillis);
        requireNonNegativeThreshold(loopTimeKey, "budgetMillis", budgetMillis);
        return named("loopTimeUnderBudget(" + loopTimeKey + ", " + budgetMillis + ")", run -> {
            if (!run.hasKey(loopTimeKey)) {
                return Guardrail.Verdict.unknown("'" + loopTimeKey + "' was never logged, so loop-time "
                        + "budget could not be checked. No scenario logs a loop time yet.");
            }
            double worst = run.maxValue(loopTimeKey);
            if (!Double.isFinite(worst)) {
                return Guardrail.Verdict.violated("'" + loopTimeKey + "' logged a non-finite loop time, "
                        + "which cannot be under any budget; NaN > budget is false, so this is checked "
                        + "before the comparison rather than after it");
            }
            if (worst > budgetMillis) {
                return Guardrail.Verdict.violated("worst loop took " + worst + " ms, over the "
                        + budgetMillis + " ms budget (budget is GUESSED, not measured)");
            }
            return Guardrail.Verdict.holds();
        });
    }

    /**
     * §3: the robot comes to a safe stop when the run ends — as far as a log can tell.
     *
     * <p><b>Deliberately narrow, and it says so in its own name.</b> This checks one named output key is
     * near zero in the final frame. It cannot prove the physical robot stopped, and it cannot prove
     * anything about outputs nobody logged. It is offered because the partial check is genuinely useful
     * and because leaving §3's term entirely unrepresented would hide it.
     *
     * @param outputKey        the logged actuator output to check at the end of the run
     * @param stoppedTolerance how close to zero counts as stopped, in the same unit as the values logged
     *                         under {@code outputKey}. {@code GUESSED} -- no measured value exists yet.
     *
     * @return a guardrail that violates when the output under {@code outputKey} has not stopped by the last frame
     */
    public static Guardrail safeStopAtEnd(String outputKey, double stoppedTolerance) {
        requireFiniteThreshold(outputKey, "stoppedTolerance", stoppedTolerance);
        requireNonNegativeThreshold(outputKey, "stoppedTolerance", stoppedTolerance);
        return named("safeStopAtEnd(" + outputKey + ", " + stoppedTolerance + ")", run -> {
            if (!run.hasKey(outputKey)) {
                return Guardrail.Verdict.unknown("'" + outputKey + "' was never logged, so the safe-stop "
                        + "check could not run. Note this check only ever covers the key it is given -- it "
                        + "is not a whole-robot safe-stop proof.");
            }
            // No "did it keep logging to the end?" check here, and that is a decision, not an oversight.
            // An RLOG is delta-encoded: a field not written on a cycle decodes as unchanged, so ceasing to
            // log an output is indistinguishable from re-logging the same value, and finalValue genuinely
            // is the value the run ended on (RunResult#metric explains the decoder's semantics).
            double last = run.finalValue(outputKey);
            if (!Double.isFinite(last)) {
                return Guardrail.Verdict.violated("'" + outputKey + "' ended at the non-finite value "
                        + last + "; Math.abs(NaN) > tolerance is false, so this is checked first rather "
                        + "than letting a broken output read as stopped");
            }
            if (Math.abs(last) > stoppedTolerance) {
                return Guardrail.Verdict.violated("'" + outputKey + "' was still at " + last
                        + " in the final frame, outside the +/-" + stoppedTolerance
                        + " stopped tolerance (tolerance is GUESSED)");
            }
            return Guardrail.Verdict.holds();
        });
    }

    /**
     * The key really was updated during the run, rather than set once at the start and left alone.
     *
     * <p>Not one of §3's seven. It exists for {@link Objective#ofFinalValue}: an RLOG is delta-encoded, so a
     * key written once at frame 0 and never again still has a "final value" — the frame-0 one. A finish
     * time initialised to {@code 0} and minimized scores perfectly on a run that never finished. Put this
     * in the envelope beside such an objective and that run is a violation instead of a win.
     *
     * <p>"Updated" means: the key first appears after frame 0 (it was written mid-run), or its value is not
     * the same in every frame. A key present from frame 0 with one value throughout is VIOLATED. A run of a
     * single frame cannot show a change either way, so it is {@code UNKNOWN}, as is a run that never logged
     * the key.
     *
     * @param key the RLOG key the objective reads
     * @return a guardrail that violates when {@code key} held one value from the first frame to the last
     */
    public static Guardrail metricWasUpdated(String key) {
        return named("metricWasUpdated(" + key + ")", run -> {
            if (!run.hasKey(key)) {
                return Guardrail.Verdict.unknown("'" + key + "' was never logged, so there is no update to see");
            }
            if (run.frameCount() < 2) {
                return Guardrail.Verdict.unknown("a one-frame run cannot show whether '" + key + "' changed");
            }
            List<Double> values = run.metric(key);
            boolean presentFromFirstFrame = values.size() == run.frameCount();
            if (!presentFromFirstFrame) {
                return Guardrail.Verdict.holds();
            }
            for (Double value : values) {
                if (Double.compare(value, values.get(0)) != 0) {
                    return Guardrail.Verdict.holds();
                }
            }
            return Guardrail.Verdict.violated("'" + key + "' held " + values.get(0) + " from the first frame "
                    + "to the last, so the objective read a value nobody updated during the run");
        });
    }

    /**
     * The run produced every frame it was asked for.
     *
     * <p>Not one of §3's seven, but §7's "reject runs whose logs are missing or short", and it belongs in
     * the envelope for the same reason they do: a candidate that scores better by logging less is the
     * cheapest reward hack available. A violation rather than a penalty, because a short log makes every
     * other term in the breakdown untrustworthy too.
     *
     * @return a guardrail that violates when the run logged fewer frames than it was told to produce
     */
    public static Guardrail runIsComplete() {
        return named("runIsComplete", run -> {
            if (run.isComplete()) {
                return Guardrail.Verdict.holds();
            }
            return Guardrail.Verdict.violated("run logged " + run.frameCount() + " of "
                    + run.expectedFrameCount() + " expected frames -- a short log is a violation, not a "
                    + "low score, or 'win by not logging' becomes an optimization");
        });
    }

    // ---------------------------------------------------------------------------------------------
    // NOT answerable from the log — declared so the sort is written in code. Never put these in a
    // loop scorer's envelope: the gate rejects on UNKNOWN, so they would block every candidate.
    // ---------------------------------------------------------------------------------------------

    /**
     * §3: all invariant-tier tests green.
     *
     * <p><b>A Gradle exit code, not a log fact.</b> Always {@code UNKNOWN} here. The loop enforces it as
     * {@link Gate.Clause#TESTS_GREEN}, from {@link Gate.SuppliedFacts#testsGreen()}; a guardrail handed only
     * an RLOG cannot answer it, and must not pretend to. Not for a scorer's envelope (see the class javadoc).
     *
     * @return a guardrail that always reports UNKNOWN, because a log cannot show whether the tests ran
     */
    public static Guardrail invariantTestsGreen() {
        return named("invariantTestsGreen", run -> Guardrail.Verdict.unknown(
                "invariant-tier test results are a Gradle exit code, not a field in the run's log. The "
                        + "gate's TESTS_GREEN clause owns this term; scoring an RLOG cannot answer it."));
    }

    /**
     * §3 / domain {@code R8}: code may actuate real hardware only after the same code passed in sim.
     *
     * <p><b>A fact about history, not about this run.</b> Always {@code UNKNOWN} here: it is a question
     * about which commit ran where, which a single RLOG cannot show. The loop runs only in sim, so it cannot
     * break R8 itself; moving its answer to a robot is a human's step. Not for a scorer's envelope.
     *
     * @return a guardrail that always reports UNKNOWN, because a log cannot show what ran where
     */
    public static Guardrail passedInSimFirst() {
        return named("passedInSimFirst", run -> Guardrail.Verdict.unknown(
                "R8 is a fact about this code's history -- did the same commit pass in sim before touching "
                        + "hardware -- and no single run's log records it."));
    }

    /**
     * §3: the drivetrain's FK/IK round-trip holds.
     *
     * <p><b>A property of the kinematics, not of a run.</b> Always {@code UNKNOWN} here. It is pinned by
     * an invariant-tier property test over many inputs; a single run exercises whatever path it happened
     * to drive, which would be a far weaker claim wearing the same name. That test runs inside the verify
     * command, so the loop enforces it as {@link Gate.Clause#TESTS_GREEN}. Not for a scorer's envelope.
     *
     * @return a guardrail that always reports UNKNOWN, because a log cannot show a kinematics round trip
     */
    public static Guardrail forwardInverseKinematicsRoundTrip() {
        return named("forwardInverseKinematicsRoundTrip", run -> Guardrail.Verdict.unknown(
                "FK/IK round-trip is a property over many inputs, pinned by an invariant-tier test. One "
                        + "run only exercises the path it drove, so answering from this log would claim "
                        + "more than the evidence supports."));
    }

    /**
     * Every §3 term that cannot be answered from a run, as one list — for a report that wants to name them.
     *
     * <p><b>Not for a scorer's envelope.</b> Every term in it is always {@code UNKNOWN}, and the gate rejects
     * any candidate whose envelope holds an {@code UNKNOWN} term, so a scorer built with this list can never
     * accept anything. The class javadoc says where each of the three is actually enforced.
     *
     * @return an unmodifiable list of the three guardrails a single RLOG cannot answer
     */
    public static List<Guardrail> notAnswerableFromALog() {
        return Collections.unmodifiableList(Arrays.asList(
                invariantTestsGreen(),
                passedInSimFirst(),
                forwardInverseKinematicsRoundTrip()));
    }

    /**
     * Rejects a non-finite threshold at construction, because a NaN threshold turns a guardrail into a
     * guarantee of nothing while still reporting HOLDS.
     *
     * <p>Every check in this class already refuses a non-finite value read out of the run, for the reason
     * each of them states: comparisons against NaN are false, so a broken value would otherwise read as
     * staying inside its limits. The thresholds were never given the same treatment, and they need it more —
     * {@code withinBound(key, NaN, NaN)} holds for every finite value the actuator could possibly log, so the
     * envelope contains a term that can never fail and a reader sees a guardrail name and assumes it means
     * something. (2026-09-17 adversarial review, finding guardrail-nan-thresholds-pass.)
     *
     * @param key       the logged key the guardrail is about, named in the message so the caller can find it
     * @param parameter the threshold's parameter name, as written in this class's signature
     * @param value     the threshold as supplied
     */
    private static void requireFiniteThreshold(String key, String parameter, double value) {
        if (!Double.isFinite(value)) {
            throw new IllegalArgumentException(parameter + " for '" + key + "' is " + value
                    + ", which is not a finite number. Every comparison against a non-finite threshold is "
                    + "false, so this guardrail would report HOLDS for every possible run -- an envelope term "
                    + "that cannot fail is worse than no term at all, because it reads like a check.");
        }
    }

    /**
     * Rejects a negative budget or tolerance at construction. A negative one is not a false guarantee — it
     * violates for every possible value — but it is certainly a mistake, and catching it where it is written
     * beats reading a VIOLATED verdict and hunting for why.
     *
     * @param key       the logged key the guardrail is about
     * @param parameter the threshold's parameter name, as written in this class's signature
     * @param value     the threshold as supplied; already known to be finite
     */
    private static void requireNonNegativeThreshold(String key, String parameter, double value) {
        if (value < 0.0) {
            throw new IllegalArgumentException(parameter + " for '" + key + "' is " + value
                    + ", which is negative. A negative budget or tolerance can never be met, so this "
                    + "guardrail would report VIOLATED for every possible run.");
        }
    }

    private static Guardrail named(String name, Check check) {
        return new NamedGuardrail(name, check);
    }

    /** The check half of a guardrail, separated from its name so {@link NamedGuardrail} can carry both. */
    @FunctionalInterface
    private interface Check {
        Guardrail.Verdict evaluate(RunResult run);
    }

    /**
     * A guardrail that knows its own name.
     *
     * <p>A named class rather than an anonymous one, and a stored name rather than a derived one, because
     * {@link Guardrail#name()} is what keys the score breakdown a human audits: a name that came from the
     * JVM's idea of a lambda's class would vary between runs and break replay comparison (R5).
     */
    private static final class NamedGuardrail implements Guardrail {

        private final String name;
        private final Check check;

        NamedGuardrail(String name, Check check) {
            this.name = name;
            this.check = check;
        }

        @Override
        public String name() {
            return name;
        }

        /**
         * Runs the check, turning a non-numeric value into {@code UNKNOWN} rather than an exception.
         *
         * <p>{@link Guardrail#evaluate} says "return a verdict; do not throw". Every check above tests
         * {@code hasKey} first, so a missing key is already {@code UNKNOWN}; what is left is a key that
         * exists but holds a string or a boolean, which {@link RunResult#metric} refuses with
         * {@link RunResult.MetricNotNumericException}. A bound cannot be judged on a word, so that is a term
         * that could not be checked: {@code UNKNOWN}, which still blocks the gate, and not a crashed loop.
         */
        @Override
        public Verdict evaluate(RunResult run) {
            try {
                return check.evaluate(run);
            } catch (RunResult.MetricNotNumericException notANumber) {
                return Verdict.unknown(name + " could not be checked: " + notANumber.getMessage());
            }
        }
    }
}
