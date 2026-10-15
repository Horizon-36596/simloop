package org.horizon36596.simloop.loop;

import org.junit.jupiter.api.Test;

import java.io.IOException;
import java.lang.reflect.Constructor;
import java.lang.reflect.Method;
import java.lang.reflect.Modifier;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.TreeSet;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * Pins the loop driver's Contract: the driver scores every run with its own fixed scorer, starts from a
 * baseline, keeps as last-best only an answer that passed the gate in full, emits milestones that are
 * event-driven and carry a readable diff, reproduces before it reports, and proposes nothing.
 *
 * <p>Every RLOG here is named {@code loop-driver-*}, a prefix no other test in the build uses. Two test
 * classes writing one RLOG path corrupt each other's logs, and that fails as a determinism bug that is
 * not one — which, in this class especially, would be a confusing way to spend an afternoon.
 *
 * <p>The scorer is fixed for a driver's life, so a test that needs different candidates to fare
 * differently against the envelope does it through the run: {@link #envelopeTerm()} reads
 * {@link #ENVELOPE_STATE} out of each run (absent = UNKNOWN, 0 = HOLDS, positive = OVER_BUDGET by that
 * much, negative = VIOLATED).
 */
public class LoopDriverTest {

    private static final String FINISH_TIME = "RealOutputs/Auto/finishTimeSec";
    private static final String ENVELOPE_STATE = "RealOutputs/Test/envelopeState";

    /** Marks a run that logs no envelope state, so {@link #envelopeTerm()} says UNKNOWN. */
    private static final Double NOT_LOGGED = null;

    // ---------------------------------------------------------------------------------------------
    // Clause 1 — last-best is always an answer that passed the gate in full, and must beat the baseline
    // ---------------------------------------------------------------------------------------------

    /**
     * Contract: a candidate the gate rejected never becomes last-best, and the previous best is left
     * exactly as it was. Invariant tier.
     *
     * <p>This is §5's promise that whenever a human stops the loop, what they are handed is a fully
     * working solution. Three rejections are offered in a row — a red build, a violated envelope, and an
     * unchecked envelope term — and after all three the answer is still the one that earned it.
     */
    @Test
    public void aRejectedCandidateNeverBecomesLastBestAndLeavesThePreviousOneAlone() throws IOException {
        LoopDriver driver = driver("rejected", 0.0, 40.0);

        LoopDriver.Decision good = driver.offer("first good answer", run("rejected-good", 30.0, 0.0),
                Gate.SuppliedFacts.bothConfirmed());
        assertTrue(good.accepted(), "precondition: a candidate that beats the baseline is accepted");
        LoopDriver.Candidate goodAnswer = driver.lastBest();

        LoopDriver.Decision redBuild = driver.offer("edit with failing tests", run("rejected-tests", 1.0, 0.0),
                Gate.SuppliedFacts.of(Gate.Fact.CONFIRMED_FALSE, Gate.Fact.CONFIRMED_TRUE));
        LoopDriver.Decision brokenEnvelope = driver.offer("edit that broke an invariant",
                run("rejected-envelope", 1.0, -1.0), Gate.SuppliedFacts.bothConfirmed());
        LoopDriver.Decision uncheckedEnvelope = driver.offer("edit nobody could check",
                run("rejected-unknown", 1.0, NOT_LOGGED), Gate.SuppliedFacts.bothConfirmed());

        for (LoopDriver.Decision rejected : Arrays.asList(redBuild, brokenEnvelope, uncheckedEnvelope)) {
            assertFalse(rejected.accepted(), "must be rejected: " + rejected.verdict().explain());
            assertNull(rejected.milestone(), "a rejected candidate must not produce a milestone");
        }
        assertSame(goodAnswer, driver.lastBest(), "last-best must still be the candidate that earned it");
        assertEquals(4, driver.offeredCount());
        assertEquals(1, driver.acceptedCount());
    }

    /**
     * Contract: the first candidate must beat the baseline; being the first is not enough. Invariant tier.
     *
     * <p>Without a baseline the first candidate was judged with nothing to improve on and accepted on the
     * other three clauses alone, so an edit that made the robot slower than where the loop started could
     * become the answer the human is handed.
     */
    @Test
    public void theFirstCandidateMustBeatTheBaseline() throws IOException {
        LoopDriver driver = driver("beat-baseline", 0.0, 30.0);
        assertTrue(driver.baselineVerdict().accepted(), "precondition: the baseline passes the gate");
        assertSame(driver.baseline(), driver.lastBest(), "an accepted baseline is the starting last-best");

        LoopDriver.Decision slower = driver.offer("made the auto slower", run("beat-baseline-slower", 35.0, 0.0),
                Gate.SuppliedFacts.bothConfirmed());

        assertFalse(slower.accepted(), "35 s does not beat a 30 s baseline: " + slower.verdict().explain());
        assertEquals(Gate.Outcome.FAILED, slower.verdict().clause(Gate.Clause.IMPROVES_ON_LAST_BEST).outcome());
        assertSame(driver.baseline(), driver.lastBest(), "the baseline must still be the answer");
    }

    /**
     * Contract: a baseline that fails the gate leaves the loop with no answer, and the first candidate that
     * passes becomes the first one. Invariant tier — going from broken to working is progress, and the
     * baseline's rejection must not become a last-best out of nothing.
     */
    @Test
    public void whenTheBaselineFailsTheGateTheFirstPassingCandidateIsTheFirstAnswer() throws IOException {
        LoopDriver driver = driver("broken-baseline", 0.0, 40.0, -1.0);
        assertFalse(driver.baselineVerdict().accepted(), "precondition: the baseline breaks the envelope");
        assertFalse(driver.hasLastBest());
        IllegalStateException thrown = assertThrows(IllegalStateException.class, driver::lastBest);
        assertTrue(thrown.getMessage().contains("accepted nothing"),
                "the refusal must say why: " + thrown.getMessage());

        LoopDriver.Decision stillBroken = driver.offer("still broken", run("broken-baseline-2", 30.0, -1.0),
                Gate.SuppliedFacts.bothConfirmed());
        assertFalse(stillBroken.accepted());
        assertFalse(driver.hasLastBest(), "a rejection must not create a last-best out of nothing");

        LoopDriver.Decision fixed = driver.offer("stopped breaking the envelope",
                run("broken-baseline-3", 50.0, 0.0), Gate.SuppliedFacts.bothConfirmed());
        assertTrue(fixed.accepted(), fixed.verdict().explain());
        assertEquals(Collections.singletonList(Milestone.Reason.FIRST_ACCEPTED_CANDIDATE),
                fixed.milestone().reasons());
        assertNull(fixed.milestone().previousBest(), "there was no working answer before it");
    }

    /**
     * Contract: a candidate hands in a run, never a score; the driver scores it with the scorer it was
     * built with. Invariant tier.
     *
     * <p>Pinned structurally, because the attack is a type: when a candidate carried its own score, a
     * proposer could keep every guardrail's name and swap its body, flip the objective's direction, or
     * score a different run from the one it handed in, and the gate compared the forged score faithfully.
     * If any public way to hand the driver a score comes back, this fails.
     */
    @Test
    public void aCandidateCannotBringItsOwnScore() {
        for (Method method : LoopDriver.class.getDeclaredMethods()) {
            if (Modifier.isPublic(method.getModifiers())) {
                assertFalse(Arrays.asList(method.getParameterTypes()).contains(Scorer.Score.class),
                        "LoopDriver." + method.getName() + " takes a Score; the driver must score runs itself");
            }
        }
        for (Constructor<?> constructor : LoopDriver.Candidate.class.getDeclaredConstructors()) {
            assertFalse(Modifier.isPublic(constructor.getModifiers()), "Candidate must not be buildable outside");
        }
        for (Method method : LoopDriver.Candidate.class.getDeclaredMethods()) {
            assertFalse(Modifier.isStatic(method.getModifiers()) && Modifier.isPublic(method.getModifiers()),
                    "Candidate must have no public factory: " + method.getName());
        }
    }

    /**
     * Contract: a run the objective cannot measure is a rejected candidate, not a crashed loop, and it is
     * counted. Invariant tier.
     *
     * <p>A long autonomous run must be able to throw away the edit that broke the measurement and carry on.
     */
    @Test
    public void aRunThatCannotBeScoredIsRejectedAndTheLoopCarriesOn() throws IOException {
        LoopDriver driver = driver("unscorable", 0.0, 40.0);
        Path noObjective = LoopTestRlogs.write("loop-driver-unscorable-run",
                table -> table.put(ENVELOPE_STATE, 0.0));

        LoopDriver.Decision decision = driver.offer("stopped logging the finish time",
                RunResult.fromRlog("unscorable-run", noObjective, 1), Gate.SuppliedFacts.bothConfirmed());

        assertFalse(decision.accepted());
        assertNull(decision.score(), "there is no score to show");
        Gate.ClauseResult improvement = decision.verdict().clause(Gate.Clause.IMPROVES_ON_LAST_BEST);
        assertEquals(Gate.Outcome.UNKNOWN, improvement.outcome());
        assertTrue(improvement.detail().contains(FINISH_TIME),
                "the verdict must name the key the objective could not read: " + improvement.detail());
        assertEquals(1, driver.offeredCount(), "an unscorable candidate was still offered");

        assertTrue(driver.offer("a real gain", run("unscorable-next", 30.0, 0.0),
                Gate.SuppliedFacts.bothConfirmed()).accepted(), "the loop must carry on after it");
    }

    /**
     * Contract: a candidate whose score does not carry exactly the guardrails the baseline carried never
     * replaces the last-best. Invariant tier.
     *
     * <p>This replaces the old "a candidate that dropped a guardrail does not replace the last-best" test.
     * A candidate can no longer bring its own score, so it can no longer drop a term by declaring fewer;
     * the one way left for the envelope to change shape between two scores is a guardrail whose name is not
     * stable, and that is refused before the gate is consulted. What the loop depends on is the same as
     * before: last-best still points at the honest answer afterwards.
     */
    @Test
    public void aGuardrailWhoseNameChangesBetweenRunsIsRefused() throws IOException {
        Guardrail unstable = new Guardrail() {
            private int calls;

            @Override
            public String name() {
                return "unstable" + calls;
            }

            @Override
            public Verdict evaluate(RunResult ignored) {
                calls++;
                return Verdict.holds();
            }
        };
        Scorer scorer = new Scorer(objective(), Collections.singletonList(unstable));
        LoopDriver driver = new LoopDriver(scorer, new Gate(), 0.0, "baseline",
                run("unstable-baseline", 40.0, 0.0), Gate.SuppliedFacts.bothConfirmed());

        LoopDriver.Candidate honest = driver.lastBest();

        IllegalStateException thrown = assertThrows(IllegalStateException.class,
                () -> driver.offer("anything", run("unstable-candidate", 30.0, 0.0),
                        Gate.SuppliedFacts.bothConfirmed()));
        assertTrue(thrown.getMessage().contains("stable"), thrown.getMessage());
        assertSame(honest, driver.lastBest(), "the envelope changed shape, so the honest answer must stand");
        assertEquals(0, driver.acceptedCount());
        assertTrue(driver.milestones().isEmpty());
    }

    /**
     * Contract: the baseline must come with both supplied facts confirmed. Invariant tier.
     *
     * <p>A baseline rejected on a supplied fact alone left the loop with no last-best, so the first
     * candidate was judged with nothing to beat — exactly the hole the baseline exists to close. For
     * unedited code "review passed" means a human agreed this is the code to start from; not having asked
     * is a reason not to start the loop, not a reason to start it with no target.
     */
    @Test
    public void aBaselineWithAnUncheckedFactIsRefused() throws IOException {
        IllegalArgumentException thrown = assertThrows(IllegalArgumentException.class,
                () -> new LoopDriver(scorer(), new Gate(), 0.0, "baseline", run("unconfirmed-baseline", 40.0, 0.0),
                        Gate.SuppliedFacts.of(Gate.Fact.CONFIRMED_TRUE, Gate.Fact.NOT_CHECKED)));
        assertTrue(thrown.getMessage().contains("REVIEW_PASSED"), thrown.getMessage());
        assertThrows(IllegalArgumentException.class,
                () -> new LoopDriver(scorer(), new Gate(), 0.0, "baseline", run("red-baseline", 40.0, 0.0),
                        Gate.SuppliedFacts.of(Gate.Fact.CONFIRMED_FALSE, Gate.Fact.CONFIRMED_TRUE)));
    }

    // ---------------------------------------------------------------------------------------------
    // Clause 2 — milestones are event-driven, and carry a human-readable diff
    // ---------------------------------------------------------------------------------------------

    /**
     * Contract: an accepted candidate that improved by less than the threshold becomes the new best but
     * does not get a milestone. Invariant tier.
     *
     * <p>Both halves matter and they pull in opposite directions, which is why they are pinned together.
     * If every acceptance produced a milestone, a long run would produce hundreds and nobody would read
     * them. If a below-threshold improvement were also discarded, the loop would refuse progress it had
     * already judged good enough to accept.
     */
    @Test
    public void anImprovementBelowTheThresholdIsKeptButDoesNotGetAMilestone() throws IOException {
        LoopDriver driver = driver("below-threshold", 10.0, 30.0);

        // MINIMIZE, so a smaller finish time is a bigger score: 30.0 -> 29.0 is an improvement of 1.0.
        LoopDriver.Decision decision = driver.offer("shaved a hair off", run("below-threshold-2", 29.0, 0.0),
                Gate.SuppliedFacts.bothConfirmed());

        assertTrue(decision.accepted(), "a real improvement must still be accepted: "
                + decision.verdict().explain());
        assertEquals("shaved a hair off", driver.lastBest().description(), "it must become the new best");
        assertFalse(decision.emittedMilestone(),
                "an improvement of 1.0 against a threshold of 10.0 must not emit a milestone");
        assertTrue(driver.milestones().isEmpty(), "an accepted baseline is the starting point, not a milestone");
    }

    /**
     * Contract: an improvement of at least the threshold over the baseline emits a milestone measured from
     * the baseline. Invariant tier.
     */
    @Test
    public void aBigEnoughImprovementEmitsAMilestoneMeasuredFromTheBaseline() throws IOException {
        LoopDriver driver = driver("big", 10.0, 40.0);

        LoopDriver.Decision decision = driver.offer("cut a whole lap out", run("big-2", 20.0, 0.0),
                Gate.SuppliedFacts.bothConfirmed());

        assertTrue(decision.emittedMilestone(), "an improvement of 20.0 must clear a threshold of 10.0");
        assertEquals(Collections.singletonList(Milestone.Reason.IMPROVED_BY_AT_LEAST_THE_THRESHOLD),
                decision.milestone().reasons());
        assertEquals(1, decision.milestone().number(), "milestones are numbered in emission order");
        assertEquals(20.0, decision.milestone().improvement(), 1e-9);
        assertSame(driver.baseline().score(), decision.milestone().previousBest());
    }

    /**
     * Contract: small gains accumulate — the threshold is measured from the last milestone, not from the
     * immediately previous best. Invariant tier.
     *
     * <p>Measured step by step, a loop that improves by 6 every time against a threshold of 10 would never
     * emit a milestone at all, however far it went, and the human would never be shown the diff.
     */
    @Test
    public void smallGainsAccumulateIntoAMilestone() throws IOException {
        LoopDriver driver = driver("accumulate", 10.0, 40.0);
        Gate.SuppliedFacts ok = Gate.SuppliedFacts.bothConfirmed();

        assertFalse(driver.offer("step 1", run("accumulate-1", 34.0, 0.0), ok).emittedMilestone(), "6 < 10");
        LoopDriver.Decision second = driver.offer("step 2", run("accumulate-2", 28.0, 0.0), ok);
        assertTrue(second.emittedMilestone(), "6 + 6 = 12 since the baseline clears 10");
        assertEquals(12.0, second.milestone().improvement(), 1e-9);
        assertSame(driver.baseline().score(), second.milestone().previousBest());

        assertFalse(driver.offer("step 3", run("accumulate-3", 25.0, 0.0), ok).emittedMilestone(),
                "3 since the last milestone");
        LoopDriver.Decision fourth = driver.offer("step 4", run("accumulate-4", 17.0, 0.0), ok);
        assertTrue(fourth.emittedMilestone(), "3 + 8 = 11 since the last milestone clears 10");
        assertEquals(11.0, fourth.milestone().improvement(), 1e-9);
        assertSame(second.score(), fourth.milestone().previousBest(), "measured from the last milestone");
    }

    /**
     * Contract: a guardrail that was over its soft budget and is now holding emits a milestone even when the
     * score barely moved. Invariant tier — §5's "qualitatively new working capability".
     */
    @Test
    public void aGuardrailThatIsNowHoldingEmitsAMilestoneEvenWhenTheScoreBarelyMoved() throws IOException {
        LoopDriver driver = driver("newly-holding", 10.0, 30.0, 0.5);
        assertTrue(driver.baselineVerdict().accepted(), "precondition: over budget is not a violation");

        LoopDriver.Decision decision = driver.offer("brought the loop period back in budget",
                run("newly-holding-2", 30.0, 0.0), Gate.SuppliedFacts.bothConfirmed());

        assertTrue(decision.accepted(), decision.verdict().explain());
        assertTrue(decision.emittedMilestone(),
                "dropping a soft-budget penalty is a new capability, not just a better number");
        assertEquals(0.5, decision.milestone().improvement(), 1e-9,
                "precondition: the score moved by far less than the threshold");
        assertEquals(Collections.singletonList(Milestone.Reason.A_GUARDRAIL_IS_NOW_CHECKED_AND_HOLDING),
                decision.milestone().reasons());
        assertEquals(Collections.singletonList("guardrail testEnvelope"), decision.milestone().newlyHolding());
    }

    /**
     * Contract: a milestone carries a human-readable diff — both breakdowns and the candidate's own
     * account of what it changed. Invariant tier.
     *
     * <p>§7's point, pinned: every reward hack improves the number, so a milestone showing only the number
     * shows nothing that could distinguish one.
     */
    @Test
    public void aMilestoneShowsBothBreakdownsAndWhatTheCandidateSaysItChanged() throws IOException {
        LoopDriver driver = driver("diff", 1.0, 40.0);
        LoopDriver.Decision decision = driver.offer("raised the flywheel target by 200 rpm",
                run("diff-2", 20.0, 0.0), Gate.SuppliedFacts.bothConfirmed());

        String text = decision.milestone().text();

        assertTrue(text.contains("raised the flywheel target by 200 rpm"),
                "the candidate's own words must be in the milestone: " + text);
        assertTrue(text.contains("previous best") && text.contains("new best:"),
                "both sides of the diff must be present: " + text);
        assertTrue(text.contains("testEnvelope"), "the guardrail terms must be visible, not just the total: " + text);
        assertTrue(text.contains("TOTAL"), "each side must show its full breakdown: " + text);
        assertFalse(text.contains("\r"), "the milestone must read the same on every platform (R5): " + text);
    }

    // ---------------------------------------------------------------------------------------------
    // Clause 3 — the stop report reproduces before it claims
    // ---------------------------------------------------------------------------------------------

    /**
     * Contract: the stop report replays the best candidate's scenario and only then reports its score,
     * with the milestone trail. Invariant tier.
     */
    @Test
    public void theStopReportReplaysTheBestCandidateBeforeReportingItsScore() throws IOException {
        LoopDriver driver = driver("reproduce-ok", 0.0, 40.0);
        driver.offer("the answer", run("reproduce-ok", 20.0, 0.0), Gate.SuppliedFacts.bothConfirmed());

        List<String> replayed = new ArrayList<>();
        StopReport report = driver.stopReport(accepted -> {
            replayed.add(accepted.opModeName());
            return rlog("loop-driver-reproduce-ok-replay", 20.0, 0.0);
        });

        assertEquals(Collections.singletonList("reproduce-ok"), replayed,
                "the report must actually run the scenario again, not assume it would match");
        assertEquals(StopReport.Reproduction.REPRODUCED, report.reproduction());
        assertEquals(driver.lastBest().score().total(), report.reportedScore().total(), 1e-9);
        assertFalse(report.bestIsBaseline());
        assertTrue(report.mismatches().isEmpty());
        assertTrue(report.text().contains("REPRODUCED"), report.text());
        assertTrue(report.text().contains("milestone trail") && report.text().contains("#1 ")
                && report.text().contains("the answer"), "the stop report must print the trail: " + report.text());
    }

    /**
     * Contract: a later candidate that writes over the accepted run's RLOG does not turn the stop report
     * into a false determinism failure. Invariant tier.
     *
     * <p>The skill reuses one RLOG name per candidate, so the accepted run's file is routinely overwritten
     * by the next candidate, accepted or not. The replay must be compared against the frames the accepted
     * score came from, which the driver holds in memory.
     */
    @Test
    public void aLaterCandidateOverwritingTheAcceptedRlogDoesNotFakeADeterminismFailure() throws IOException {
        LoopDriver driver = driver("overwrite", 0.0, 40.0);
        // The files are copied by reading and writing bytes rather than with Files.copy, because
        // RlogDecodedCompare.decode leaked its read handle on every file it decoded until PR #26 closed it,
        // and on Windows Files.copy refuses a file that is still open; a byte copy works either way. Nothing here re-writes a decoded file
        // under its own name through PsiKit's writer.
        Path sharedName = LoopTestRlogs.simDir().resolve("loop-driver-overwrite-shared-name.rlog");
        Files.write(sharedName, Files.readAllBytes(rlog("loop-driver-overwrite-accepted-source", 20.0, 0.0)));
        RunResult accepted = RunResult.fromRlog("overwrite-accepted", sharedName, 1);
        assertTrue(driver.offer("the answer", accepted, Gate.SuppliedFacts.bothConfirmed()).accepted());
        RunResult worse = run("overwrite-worse", 35.0, 0.0);
        assertFalse(driver.offer("a worse edit", worse, Gate.SuppliedFacts.bothConfirmed()).accepted());
        // What reusing one RLOG name across fresh Gradle runs does: the later run's file replaces the
        // accepted one's on disk.
        Files.write(sharedName, Files.readAllBytes(worse.rlogPath()));

        StopReport report = driver.stopReport(best -> rlog("loop-driver-overwrite-replay", 20.0, 0.0));

        assertEquals(StopReport.Reproduction.REPRODUCED, report.reproduction(), report.text());
    }

    /**
     * Contract: with no candidate beating the baseline, the baseline is the answer, and it is replayed and
     * labelled as such. Implementation tier.
     */
    @Test
    public void whenNothingBeatsTheBaselineTheReportReplaysAndNamesTheBaseline() throws IOException {
        LoopDriver driver = driver("baseline-only", 0.0, 30.0);
        driver.offer("slower", run("baseline-only-2", 35.0, 0.0), Gate.SuppliedFacts.bothConfirmed());

        StopReport report = driver.stopReport(accepted -> rlog("loop-driver-baseline-only-replay", 30.0, 0.0));

        assertEquals(StopReport.Reproduction.REPRODUCED, report.reproduction(), report.text());
        assertTrue(report.bestIsBaseline());
        assertTrue(report.text().contains("the answer is the baseline"), report.text());
    }

    /**
     * Contract: when the replay differs, the report is a determinism failure and the score is not handed
     * out, and the report says where to look: which answer, both RLOG paths, the first divergence, and the
     * unreproduced total labelled as not a result. Invariant tier.
     */
    @Test
    public void aBestCandidateThatDoesNotReproduceIsReportedAsADeterminismFailureAndItsScoreIsWithheld()
            throws IOException {
        LoopDriver driver = driver("reproduce-bad", 0.0, 40.0);
        driver.offer("looked good once", run("reproduce-bad", 20.0, 0.0), Gate.SuppliedFacts.bothConfirmed());

        StopReport report = driver.stopReport(accepted -> rlog("loop-driver-reproduce-bad-replay", 21.0, 0.0));

        assertEquals(StopReport.Reproduction.DID_NOT_REPRODUCE, report.reproduction());
        assertFalse(report.hasReportableScore());
        assertFalse(report.mismatches().isEmpty(), "the report must name what differed");
        IllegalStateException thrown = assertThrows(IllegalStateException.class, report::reportedScore);
        assertTrue(thrown.getMessage().contains("not a result"),
                "the refusal must say why the score is withheld: " + thrown.getMessage());
        assertNotNull(report.unreproducedScoreForDiagnosis());

        String text = report.text();
        assertTrue(text.contains("DETERMINISM FAILURE"), text);
        assertTrue(text.contains("looked good once"), "which answer failed: " + text);
        assertTrue(text.contains("loop-driver-reproduce-bad.rlog")
                && text.contains("loop-driver-reproduce-bad-replay.rlog"), "both RLOG paths: " + text);
        assertTrue(text.contains("first divergence: frame 0, key " + FINISH_TIME), text);
        assertTrue(text.contains("NOT a result"), "the unreproduced total must say it is not a result: " + text);
    }

    /**
     * Contract: a determinism failure report stays short however many keys diverge. Implementation tier —
     * one early divergence makes every later frame differ on every key it touches, and a report that dumps
     * them all hides the one that matters.
     */
    @Test
    public void aDeterminismFailureReportStaysShortWhenManyKeysDiverge() throws IOException {
        LoopDriver driver = driver("many-keys", 0.0, 40.0);
        Path accepted = manyKeyRlog("loop-driver-many-keys", 0.0);
        driver.offer("the answer", RunResult.fromRlog("many-keys", accepted, 1), Gate.SuppliedFacts.bothConfirmed());

        StopReport report = driver.stopReport(run -> manyKeyRlog("loop-driver-many-keys-replay", 1.0));

        String text = report.text();
        assertEquals(StopReport.Reproduction.DID_NOT_REPRODUCE, report.reproduction());
        assertTrue(text.split("\n").length <= 30, "the report must stay short: " + text);
        assertTrue(text.contains("more keys"), "the keys not shown must be counted: " + text);
    }

    /** Contract: with nothing accepted, the report says so and nothing is replayed. */
    @Test
    public void aStopReportWithNoLastBestSaysSoAndReplaysNothing() throws IOException {
        LoopDriver driver = driver("never-called", 0.0, 40.0, -1.0);

        List<String> replayed = new ArrayList<>();
        StopReport report = driver.stopReport(accepted -> {
            replayed.add(accepted.opModeName());
            return rlog("loop-driver-never-called-replay", 1.0, 0.0);
        });

        assertTrue(replayed.isEmpty(), "there was nothing to replay");
        assertEquals(StopReport.Reproduction.NOT_ATTEMPTED, report.reproduction());
        assertThrows(IllegalStateException.class, report::reportedScore);
        assertTrue(report.text().contains("NO LAST-BEST"), report.text());
        assertTrue(report.text().contains("did NOT pass the gate"), report.text());
    }

    /**
     * Contract: a replayer that hands back the original run's file is refused. Implementation tier.
     *
     * <p>Handing back the original file is not a replay, and comparing it with the run it came from would
     * report a reproduction that never happened.
     */
    @Test
    public void aReplayerThatWritesOverTheOriginalRunIsRefused() throws IOException {
        LoopDriver driver = driver("same-path", 0.0, 40.0);
        driver.offer("the answer", run("same-path", 20.0, 0.0), Gate.SuppliedFacts.bothConfirmed());

        IllegalStateException thrown = assertThrows(IllegalStateException.class,
                () -> driver.stopReport(RunResult::rlogPath));
        assertTrue(thrown.getMessage().contains("same file"),
                "the refusal must say what went wrong: " + thrown.getMessage());

        assertThrows(IllegalArgumentException.class, () -> driver.stopReport(null));
    }

    /**
     * Contract: a replayer that hands back a different spelling of the same file is refused.
     * Implementation tier — {@code Path.equals} compares how a path was written down, not what it points at.
     */
    @Test
    public void aReplayerThatReturnsAnotherSpellingOfTheSameFileIsRefused() throws IOException {
        LoopDriver driver = driver("other-spelling", 0.0, 40.0);
        driver.offer("the answer", run("other-spelling", 20.0, 0.0), Gate.SuppliedFacts.bothConfirmed());

        IllegalStateException thrown = assertThrows(IllegalStateException.class,
                () -> driver.stopReport(accepted -> accepted.rlogPath().toAbsolutePath()));
        assertTrue(thrown.getMessage().contains("same file"),
                "the refusal must say what went wrong: " + thrown.getMessage());
    }

    /** Contract: a replayer that returns no RLOG, or a path with nothing written at it, is refused. */
    @Test
    public void aReplayerThatReturnsNoRlogIsRefused() throws IOException {
        LoopDriver driver = driver("null-replay", 0.0, 40.0);
        driver.offer("the answer", run("null-replay", 20.0, 0.0), Gate.SuppliedFacts.bothConfirmed());

        IllegalStateException noPath = assertThrows(IllegalStateException.class,
                () -> driver.stopReport(accepted -> null));
        assertTrue(noPath.getMessage().contains("null-replay"),
                "the refusal must name the run it could not reproduce: " + noPath.getMessage());

        IllegalStateException nothingThere = assertThrows(IllegalStateException.class,
                () -> driver.stopReport(accepted -> LoopTestRlogs.simDir().resolve("loop-driver-never-written.rlog")));
        assertTrue(nothingThere.getMessage().contains("no RLOG was written"), nothingThere.getMessage());
    }

    // ---------------------------------------------------------------------------------------------
    // Clause 4 — the driver proposes nothing
    // ---------------------------------------------------------------------------------------------

    /**
     * Contract: the driver has no way to propose an edit. Invariant tier.
     *
     * <p>Pinned structurally, because "does not propose" is a negative and there is no call that
     * demonstrates its absence. What can be pinned is the driver's whole public surface: judging, asking
     * what the best (and the baseline) is, and reporting. If someone later adds a {@code proposeEdit} —
     * or anything else that would put the optimizer inside the referee — this test fails and says so.
     */
    @Test
    public void theDriverHasNoWayToProposeAnEdit() {
        TreeSet<String> actual = new TreeSet<>();
        for (Method method : LoopDriver.class.getDeclaredMethods()) {
            if (Modifier.isPublic(method.getModifiers()) && !method.isSynthetic()) {
                actual.add(method.getName());
            }
        }

        TreeSet<String> expected = new TreeSet<>(Arrays.asList(
                "offer", "hasLastBest", "lastBest", "baseline", "baselineVerdict", "milestones",
                "offeredCount", "acceptedCount", "milestoneThreshold", "stopReport"));

        assertEquals(expected, actual,
                "the driver's public surface is judge, remember and report. Anything new here needs a "
                        + "reason that survives the question 'is the optimizer now inside the referee?'");
    }

    // ---------------------------------------------------------------------------------------------
    // Clause 5 — deterministic (R5)
    // ---------------------------------------------------------------------------------------------

    /**
     * Contract: deterministic. Implementation tier — the driver reads no clock at all, so two drivers
     * handed the same runs in the same order produce byte-identical milestones and stop reports.
     */
    @Test
    public void twoDriversGivenTheSameCandidatesProduceTheSameReports() throws IOException {
        RunResult baseline = run("determinism-baseline", 40.0, 0.0);
        RunResult first = run("determinism-1", 30.0, 0.0);
        RunResult second = run("determinism-2", 20.0, 0.0);

        String reportA = runAndReport(baseline, first, second, "loop-driver-determinism-replay-a");
        String reportB = runAndReport(baseline, first, second, "loop-driver-determinism-replay-b");

        assertEquals(reportA, reportB);
        assertFalse(reportA.contains("\r"), "the report must be the same bytes on every platform: " + reportA);
    }

    /** Offers both runs to a fresh driver and returns every milestone plus the stop report. */
    private static String runAndReport(RunResult baseline, RunResult first, RunResult second, String replayName)
            throws IOException {
        LoopDriver driver = new LoopDriver(scorer(), new Gate(), 1.0, "baseline", baseline,
                Gate.SuppliedFacts.bothConfirmed());
        driver.offer("a first gain", first, Gate.SuppliedFacts.bothConfirmed());
        driver.offer("a second gain", second, Gate.SuppliedFacts.bothConfirmed());

        StringBuilder out = new StringBuilder();
        for (Milestone milestone : driver.milestones()) {
            out.append(milestone.text());
        }
        // The replay path is the only thing that legitimately differs between the two; blank it out.
        out.append(driver.stopReport(accepted -> rlog(replayName, 20.0, 0.0)).text().replace(replayName, "REPLAY"));
        return out.toString();
    }

    // ---------------------------------------------------------------------------------------------
    // Helpers
    // ---------------------------------------------------------------------------------------------

    /** A driver whose baseline finishes in {@code baselineFinishTimeSec} with a holding envelope. */
    private static LoopDriver driver(String name, double threshold, double baselineFinishTimeSec)
            throws IOException {
        return driver(name, threshold, baselineFinishTimeSec, 0.0);
    }

    private static LoopDriver driver(String name, double threshold, double baselineFinishTimeSec,
            Double baselineEnvelopeState) throws IOException {
        return new LoopDriver(scorer(), new Gate(), threshold, "the code before the loop started",
                run(name + "-baseline", baselineFinishTimeSec, baselineEnvelopeState),
                Gate.SuppliedFacts.bothConfirmed());
    }

    /** A one-frame run named {@code opModeName}, written to {@code loop-driver-<opModeName>.rlog}. */
    private static RunResult run(String opModeName, double finishTimeSec, Double envelopeState)
            throws IOException {
        return RunResult.fromRlog(opModeName, rlog("loop-driver-" + opModeName, finishTimeSec, envelopeState), 1);
    }

    private static Path rlog(String baseName, double finishTimeSec, Double envelopeState) throws IOException {
        return LoopTestRlogs.write(baseName, table -> {
            table.put(FINISH_TIME, finishTimeSec);
            if (envelopeState != null) {
                table.put(ENVELOPE_STATE, envelopeState);
            }
        });
    }

    /** A holding one-frame run that also logs 40 extra keys, all set to {@code extraValue}. */
    private static Path manyKeyRlog(String baseName, double extraValue) throws IOException {
        return LoopTestRlogs.write(baseName, table -> {
            table.put(FINISH_TIME, 20.0);
            table.put(ENVELOPE_STATE, 0.0);
            for (int i = 0; i < 40; i++) {
                table.put(String.format("RealOutputs/Test/extra%02d", i), extraValue);
            }
        });
    }

    private static Scorer scorer() {
        return new Scorer(objective(), Collections.singletonList(envelopeTerm()));
    }

    private static Objective objective() {
        return Objective.ofFinalValue("auto finish time", "auto", FINISH_TIME,
                Objective.Direction.MINIMIZE, Objective.Budget.unstated());
    }

    /**
     * The one test guardrail, driven by the run: {@link #ENVELOPE_STATE} absent is UNKNOWN, 0 is HOLDS,
     * positive is OVER_BUDGET by that much, negative is VIOLATED.
     */
    private static Guardrail envelopeTerm() {
        return new Guardrail() {
            @Override
            public String name() {
                return "testEnvelope";
            }

            @Override
            public Verdict evaluate(RunResult run) {
                if (!run.hasKey(ENVELOPE_STATE)) {
                    return Verdict.unknown("the run logged no envelope state");
                }
                double state = run.finalValue(ENVELOPE_STATE);
                if (state < 0.0) {
                    return Verdict.violated("the test envelope is deliberately broken");
                }
                if (state > 0.0) {
                    return Verdict.overBudget(state, "the test envelope is deliberately over its soft budget");
                }
                return Verdict.holds();
            }
        };
    }
}
