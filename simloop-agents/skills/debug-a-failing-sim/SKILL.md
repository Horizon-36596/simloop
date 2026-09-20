---
name: debug-a-failing-sim
description: Diagnose a failing or throwing SimLoop simulation test - wrong log key, tick-order lag, frame mismatch, a run that wrote no RLOG, or a test that passes for the wrong reason. Use when a sim test is red, throws, or produces a result that looks impossible.
---

# Debug a failing SimLoop test

Most SimLoop failures are **not** robot-code bugs. They are the harness being wired slightly wrong, and
they have recognisable signatures. Work through this before changing any robot code — changing a gain to
make a mis-wired test go green is the worst outcome available here, because it breaks the real robot to
satisfy a broken simulation.

## Rule zero: read the actual error

Quote the real exception and the real message. SimLoop's exceptions are written to say what went wrong
and are worth reading literally — several of them name the exact cause. Do not paraphrase an error into
a guess.

## The signature table

| What you see | Almost always | Fix |
|---|---|---|
| `MetricNotFoundException` on a key you definitely logged | The `RealOutputs/` prefix | Read back `"RealOutputs/Slide/x"`, not `"Slide/x"`. It throws rather than returning zero on purpose. |
| The mechanism responds one tick late, and no gain fixes it | Tick order | `robot.periodic()` **then** `advancePhysics(dt)`. Physics first lags everything by exactly one tick. |
| The mechanism never moves at all | The plant's position is never pushed into the encoder | The subsystem is reading its own command back. Find the line in `advancePhysics` that feeds the encoder; it is missing. |
| Position is right, but the robot is in the wrong place on the field | The two frames | Localizer is +x forward / +y left; field is +X right / +Y forward. Convert with `FieldFrameTransform`. Nothing does it for you, and unrotated gives a confident wrong answer. |
| Two runs of the same scenario differ | Something read the wall clock | Hunt `System.nanoTime`, `currentTimeMillis`, `Math.random`, `HashSet`/`HashMap` iteration order. Time is `FakeTimer`. |
| "wrote no bytes", "queue overflowed", "stopped making progress" | The RLOG writer, not your robot | These come from `ScenarioRunner` and are about logging. See the troubleshooting page. |
| "ran N ticks" mismatch | The scenario ended early or the tick count disagrees | Read the message; it states both numbers. |
| `cannot find symbol: class DcMotorEx` pointing at SimLoop's jar | Build wiring | The SDK is an `api` dependency; the project's own SDK version is what resolves. |
| `Failed to load JUnit Platform` before any test runs | Missing vintage engine | JUnit 5 without the vintage engine silently stops collecting JUnit 4 tests. |

The full list, with the reasoning behind each:
<https://libraries.horizon36596.org/simloop/troubleshooting/>.

## If it is not in the table

Do this in order. Stop as soon as one of them explains it.

1. **Is the test asserting on something the model guarantees?** A test that passes because the plant
   clamps to an end stop is not measuring the robot. If it is failing, check the expectation is about
   the controller and not about the model.
2. **Did the run get enough time?** Ticks × `deltaTime` is the robot-seconds available. If the mechanism
   needs four seconds and the scenario runs three, it fails for no reason worth fixing.
3. **Log more and look at it.** Add `Logger.recordOutput` for the commanded power, the measured
   position, and the error, then open the `.rlog` in AdvantageScope. A curve answers in five seconds what
   an hour of reading the code does not. Say to the human which keys to drag in.
4. **Check the units.** SimLoop converts nothing. A config in inches and a subsystem in ticks produces
   numbers that look almost right, which is the hardest kind of wrong to see.
5. **Ask whether SimLoop models this at all.** <https://libraries.horizon36596.org/simloop/limits/>. If
   the test needs contact physics or vision, the test is asking for something that does not exist and
   the honest fix is to delete it or reduce its scope.

## Before you call it fixed

- Re-run the test and **quote the real output**. Not "it should pass now".
- Confirm the fix was in the harness or in the robot code, and say which. If you changed robot code, say
  exactly what behaviour changed on the real robot — that is a hardware consequence, not a test edit.
- If the test now passes, **make it fail once on purpose** to prove it is still testing something. A
  common bad outcome of debugging is a test that now passes unconditionally.
- Point the human at the `.rlog` under `build/sim/` and say what they should see in AdvantageScope.
