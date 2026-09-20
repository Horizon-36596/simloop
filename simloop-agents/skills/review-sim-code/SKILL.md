---
name: review-sim-code
description: Review SimLoop simulation code against the invariants that make it trustworthy - assertions that can actually fail, tick order, log keys, frame conversions, determinism, and honest physical constants. Use before merging a sim test or a new mechanism model.
---

# Review simulation code

You are checking one thing above all: **would this test have caught the bug it claims to catch?**

A green simulation that cannot fail is worse than no simulation, because the team trusts it. Review with
that as the question, not style.

Report findings as one line each: `path:line: severity: problem. fix.` No praise, no summary of what the
code does. If there is nothing wrong, say so in one line.

## The checklist

Go through all of it. Most of these produce code that compiles and passes.

### Can the assertions fail?

For **every** assertion, name a change to the robot code that would make it go red. If you cannot, that
is a finding.

- An assertion on a bound the plant clamps to every tick (`height <= maxPosition`) tests the model, not
  the robot. This is the most common real defect.
- An assertion with a tolerance so wide that any behaviour satisfies it.
- An assertion on a value the test itself set, rather than on a value the run produced.
- A test with no assertion on the *interesting* quantity — it checks arrival but not overshoot, or
  presence but not timing.

### Tick order and wiring

- `robot.periodic()` before `advancePhysics(deltaTime)`. Reversed lags every response one tick and still
  passes.
- The plant's position is actually pushed into the sensor the subsystem reads. Without it the subsystem
  reads its own command back and every test is vacuous.
- The fake devices are registered under **the same config names** the real subsystem asks `hardwareMap`
  for. A different name means sim and robot are not running the same code path.

### Log keys and reading results

- Keys are read back with the `RealOutputs/` prefix.
- Assertions read the log, not the plant's internal state. Reaching into the plant tests the physics
  model; reading the log tests the robot.
- `Scorer.Score.unknowns()` is checked wherever a score is trusted. `UNKNOWN` is not a pass.

### Frames and units

- Any pose crossing between `plant` and `field` goes through `FieldFrameTransform`. Unrotated is a
  confident wrong answer, not an exception — so its absence is invisible at runtime and this is the only
  place it gets caught.
- Every number has its unit in its name or in a comment. `height` is a finding; `heightInches` is not.
- The config's units and the subsystem's units are the same units.

### Determinism

- No `System.nanoTime()`, `System.currentTimeMillis()`, `Instant.now()`, `Math.random()` or unseeded
  `Random` anywhere on the simulated path.
- No iteration over a `HashMap`/`HashSet` whose order reaches a logged value.
- Time comes from `FakeTimer` and `deltaTime`.

### Honesty of the physical constants

- Each constant in a sim config says where it came from. "Estimated" is a fine answer; **unlabelled is
  not**, because the next reader assumes it was measured.
- `gravityHoldPowerFraction` is in `[0, 1)` and positive, and is `0.0` for a horizontal mechanism.
- `minPosition` / `maxPosition` match the real travel.
- A constant that was tuned until the test went green, rather than measured, is a finding — the
  simulation has been fitted to the test instead of to the robot.

### Scope

- Robot code was not edited to make the simulation work. If it was, that is the headline finding: the
  shipped code has been changed to satisfy a test harness.
- The test does not depend on something SimLoop deliberately does not model
  (<https://libraries.horizon36596.org/simloop/limits/>): contact physics, vision, command scheduling,
  controller tuning.

## Confirm before you report

Check each finding against the actual code before writing it down. A review that reports plausible
problems that are not there trains people to ignore reviews. If you are not sure, say the finding is
unconfirmed and say what would settle it.

## One thing a review cannot do

Say so at the end: a human should open the `.rlog` in AdvantageScope and look at the curve. Numbers
passing and motion that looks nothing like the real mechanism is a real and common outcome, and only an
eye catches it. Name the file and the keys worth dragging in.
