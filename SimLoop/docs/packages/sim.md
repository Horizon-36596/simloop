# `sim`

`org.horizon36596.simloop.sim`

The clock, the loop, and getting a log back out again.

## What you use

| Type | What it is |
|---|---|
| `FakeTimer` | The deterministic clock. Advances only when a tick advances it. |
| `ScenarioRunner` | Runs a whole scenario: owns the fixed-timestep loop and the entire PsiKit logging lifecycle. |
| `RlogDecodedCompare` | Decodes an RLOG back into per-frame field maps, and compares two runs field by field. |
| `SimElapsedTime` | The SDK's `ElapsedTime`, reading a `FakeTimer`. Hand it to code that takes a timer. |

`ScenarioRunner.runUntil(...)` runs a scenario whose length is not known in advance, such as a live
session that ends when the driver stops. It takes a `TickGate`, a `TickPacer` and extra PsiKit receivers;
`run(...)` is `runUntil` with a fixed tick count, no pacing and no extra receivers. See
[`sim.live`](live.md).

## Running a scenario

```java
ScenarioRunner.run(
        "SlideExample",        // OpMode name, recorded in the log's metadata
        robotConfig,           // your SimRobotConfig
        timer,                 // the FakeTimer, constructed and wired BEFORE this call
        150,                   // ticks
        0.02,                  // seconds per tick - 50 Hz
        Paths.get("build", "sim", "slide.rlog"),
        deltaTime -> {
            // one tick of robot behaviour
        });
```

The timer must be constructed, and anything sim-side that reads time must be wired to it, **before** this
call — that wiring has to happen before the robot under test is built.

### Logging ownership

`ScenarioRunner` starts and stops PsiKit for the run. Log values inside the tick body; do not call
`Logger.start()` or `Logger.end()` there. Use `runUntil`'s extra-receivers argument when a live session
needs another output.

PsiKit's logger is static. Run scenarios sequentially within one JVM, and give separate test processes
different log paths. If another test creates its own PsiKit logging session, initialise that session
explicitly rather than relying on state left by an earlier test.

## Log completeness checks

`ScenarioRunner.run` throws rather than returning quietly when any of four things is true: PsiKit's async
writer queue overflowed, the file is missing or empty, it decodes to a different number of frames than
ticks you ran, or the writer thread stopped making progress while the run waited for it.

That last one is why the runner **paces itself against the writer**. A simulator has no 50 Hz clock
holding it back; left alone it outruns the writer and loses the tail of the log with no error at all.

## Units and frames

- `FakeTimer.time()` and `advance(seconds)`: **seconds**, and it starts at zero.
- Per-tick `deltaTime`: **seconds**, fixed for the whole run.
- Decoded RLOG values: whatever you logged, as strings keyed by field name.

## Reading the log back

Field keys gain a prefix. `Logger.recordOutput("Slide/positionInches", x)` is read back as
`RealOutputs/Slide/positionInches`, both by `RlogDecodedCompare` and by
[`RunResult`](loop.md).

Use `RlogDecodedCompare` to compare repeated runs field by field. It ignores PsiKit's console and
writer queue-depth metadata, which reflect writer scheduling. Differences in robot-authored fields
remain failures.

## What this package will not do

- **It will not run your OpMode's lifecycle.** The scenario body is a plain lambda and you call what you
  want in it. To run an `OpMode` or `LinearOpMode` through INIT, the loop and STOP, use
  [`opmode`](opmode.md).
- **It will not schedule commands.** If you use SolversLib, SolversLib still owns control flow.
- **It will not vary the time step.** The tick is fixed, on purpose — a variable step is the single
  easiest way to make two runs disagree.
- **It will not make wall-clock-dependent robot behaviour deterministic.** Model time comes from
  `FakeTimer`. The runner may wait for log I/O, and `sim.live.RealTimePacer` can pace live input, but
  neither advances model time. In your scenario code, use the fake clock or the
  [OpMode runner's supported timer rewriting](opmode.md).
