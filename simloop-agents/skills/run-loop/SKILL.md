---
name: run-loop
description: >-
  Run SimLoop's goal-optimization loop on a team's robot code: propose one edit, replay the scenario
  deterministically, score it, pass it through the acceptance gate, and keep the last-best WORKING
  version until the user stops it. Use only when the user starts it with /run-loop to optimize a
  robot-code goal (a faster or more reliable auto, a lower loop time, a better-tuned mechanism). It
  edits code, so it never starts on its own.
disable-model-invocation: true
allowed-tools: [Read, Grep, Glob, Edit, Write, Bash, Agent]
---

# run-loop

An edit, replay, score, gate loop for FTC robot code, built on SimLoop's `loop` package
(`org.horizon36596.simloop.loop`, SimLoop `v0.2.0-alpha.2` or later). **It changes code: start it
deliberately.** Package reference: `SimLoop/docs/packages/loop.md` in the SimLoop repository.

## The engine: use it, don't re-derive it

| What | Class | What it does |
|------|-------|--------------|
| A finished run | `RunResult` | reads an `.rlog` back through decoded fields |
| What we're optimizing | `Objective` | metric + direction + scenario + budget; MINIMIZE is negated in one place, so bigger is always better |
| One envelope term | `Guardrail`, `EnvelopeGuardrails` | `HOLDS` / `VIOLATED` / `OVER_BUDGET` / **`UNKNOWN`** |
| The number | `Scorer` | `objective − Σ penalties`, returned as a breakdown, not a bare double |
| The decision | `Gate` | the four acceptance clauses below, every one reported every time |
| The bookkeeping | `LoopDriver` | owns one fixed `Scorer`, starts from a baseline run, scores every offered run itself, holds last-best, emits `Milestone`s, writes the `StopReport` |

**What stays yours, every iteration.** The engine deliberately does none of this:

1. **Propose the edit.** `LoopDriver` has no method for it, by design: a referee that picks what it
   referees would be judging its own work.
2. **Run Gradle.** SimLoop never shells out. Run the team's test command and read the result.
3. **Hand the gate the two facts it cannot measure.** `Gate.SuppliedFacts` takes tests-green and
   review-passed as a tri-state, with no default. Pass `Gate.Fact.NOT_CHECKED` when you did not run one;
   it blocks, which is correct. **Never pass `CONFIRMED_TRUE` for a check you did not actually run.**
   That is the one lie the whole structure exists to prevent.
4. **Supply a `StopReport.ScenarioReplayer`** to `LoopDriver.stopReport(...)`. A `RunResult` is a
   finished log and does not carry the config that produced it, so only you can run it again.
5. **Write the objective and the envelope**, and get the human to confirm them (Intake, below).

**Engine behaviours to know before they surprise you:**
- **A candidate hands in a run, never a score.** `LoopDriver` is built with the `Scorer` and a baseline
  run of the unedited code; `offer(description, run, facts)` scores the run itself. The baseline needs
  both facts `CONFIRMED_TRUE` (tests ran green on the unedited code, and the human agreed it is the
  starting point), or the constructor throws. The first candidate must beat the baseline. A run the
  objective cannot measure (missing key, non-number, NaN) is a rejected candidate, not a crash.
- **Milestones are measured from the last milestone**, so small accepted gains add up to one.
- **Don't put `EnvelopeGuardrails.notAnswerableFromALog()` in the envelope.** Those terms are always
  `UNKNOWN`, so they block every candidate. Tests-green is the gate's own clause instead.
- **`UNKNOWN` blocks.** A guardrail nobody could evaluate rejects the candidate. Don't drop the
  guardrail to get past it.
- **An empty envelope also blocks**, as `UNKNOWN`. Deleting guardrails to make the envelope stop
  objecting is the cheapest reward hack there is, and the gate refuses it by name.
- **The stop report reproduces before it reports.** It replays the best candidate a second time and
  compares decoded fields; on a mismatch `reportedScore()` throws and the report says determinism failure.
  Never quote `unreproducedScoreForDiagnosis()` as a result.

## Intake (before the first edit)

1. **Objective.** Set by the human, or proposed by you and confirmed by the human. Capture:
   - **metric:** a decoded RLOG field, e.g. the time the auto's last scoring action finished, the path's
     maximum cross-track error, or the mean loop period. Score decoded numeric fields, never raw log bytes.
   - **direction:** maximize or minimize.
   - **scenario:** which OpMode or routine, and which seeds or starting positions. Prefer several (both
     alliances, a few seeds) so the loop cannot overfit one lucky replay.
   - **budget:** the human's rough time or token estimate. It sets how often milestones come, nothing else.
2. **Envelope.** Invariants that must hold whatever the implementation. The loop optimizes **inside**
   them and may **never** disable one to score higher. A starter set; extend it, never shrink it without
   the human's review:
   - the team's invariant tests are green;
   - no actuator is commanded past its end-limit or configured bound;
   - the robot stops safely on disable and OpMode end;
   - no NaN or Inf in pose, velocity or any control output;
   - the run is complete (a frame per tick) and actually did the task (an auto that skips scoring is not
     "faster");
   - nothing actuates real hardware unless the same code already passed in the sim.

   Envelope terms are **invariants, not implementation facts.** "`kP > 0`" is not an envelope term: it
   would block a legitimate swap from a PIDF controller to a motion profile.

**Two test tiers.** Invariant tests belong to the envelope and are never retired. Implementation tests
pin the current design; the loop may propose replacing one, but only as part of a redesign that a
fresh-context reviewer approves, confirming the new tests pin the new design's contract.

## The loop: repeat until the user stops it, and always leave last-best working

1. Propose **one** edit toward the objective.
2. Run the tests, then replay the scenario deterministically; read the `RunResult`s.
3. `offer` it. The **gate** accepts only if ALL hold:
   1. tests green (invariant tier always; implementation tier unless legitimately retired);
   2. the envelope holds, with no `VIOLATED` or `UNKNOWN` term;
   3. `score(candidate) > score(last-best)` on the declared scenario, same scenario only;
   4. a fresh-context review of the diff and any test changes passed (a subagent that did not write
      the edit).

   Fail any: discard the edit and restore the last-best code. Last-best stands.
4. Accepted: it becomes last-best. When the best improves past a threshold, or a new working capability
   lands, write a **milestone** (format below). Milestones are where gaming becomes visible and where
   rollback points live.

## Stop

On stop, call `stopReport(replayer)` and report **last-best** with its reproduced score and the milestone
trail. Never leave a half-applied edit.

## Milestone format

```
MILESTONE <n>: <one line on what changed>
objective:   <metric>  <before> -> <after>  (<scenario, seeds>)
why:         <the reason it is better>
envelope:    <every term HOLDS; list them>
review:      <who reviewed, findings>
rollback:    last-best before this = MILESTONE <n-1> (commit <ref>)
```

Commit at each milestone, so every one is a working version to roll back to.

## Worked example: "fastest reliable auto"

- Objective: the auto's finish time (decoded RLOG field), minimized. Scenario: the auto on both
  alliances, seeds 0-4. Envelope: the starter set, plus "scored the target count on every seed".
- Edit 1: tighten a path waypoint. 0.3 s faster, all gates pass: new last-best, below the milestone
  threshold.
- Edit 2: raise the drive velocity cap. Faster on paper, but 2 of 5 seeds miss a shot, so the
  "scored the target count" term is `VIOLATED`: **discarded**. This is the reward hack the envelope
  exists to block.
- Edit 3: reorder two scoring actions so the shooter spins up during travel. 0.6 s faster, every seed
  still scores, review passes: new last-best and **milestone**.

## Guards

- **Reward hacking:** never optimize a bare metric. Anything you would hate to lose goes in the
  envelope, not a soft penalty. Reject runs with missing or short logs ("win by not logging").
- **Sim before robot:** no live-hardware actuation until the same code passed in the sim.
- **The sim is not the robot:** a sim gain is a hypothesis until it is checked on the real robot.
  SimLoop is an alpha, and its physics has not yet been compared with a real robot.
- **Overfitting:** prefer several seeds and both alliances over one replay.
