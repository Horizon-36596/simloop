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

## It refuses to hand you a log you cannot trust

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

Two runs of the same scenario must decode identically, field for field. That is what
`RlogDecodedCompare` is for, and it is how determinism is tested rather than asserted.

`RlogDecodedCompare.decode(path)` holds every frame in memory at once: one map of the whole logged state
per frame. A long run that logs many keys can need more memory than a default 512 MB test JVM has. To look through a long log, stream it with `forEachFrame(path, frame -> ...)`, which hands you each
frame and keeps none. To check its length, use `countFrames(path)`.

## What this package will not do

- **It will not run your OpMode's lifecycle.** The scenario body is a plain lambda and you call what you
  want in it. To run an `OpMode` or `LinearOpMode` through INIT, the loop and STOP, use
  [`opmode`](opmode.md).
- **It will not schedule commands.** If you use SolversLib, SolversLib still owns control flow.
- **It will not vary the time step.** The tick is fixed, on purpose — a variable step is the single
  easiest way to make two runs disagree.
- **It will not read wall-clock time anywhere.** If you introduce a `System.nanoTime()` on a sim path, you
  have broken determinism, and nothing here will catch it for you. The one class that may wait on the
  wall clock is `sim.live.RealTimePacer`, and it never changes what a tick computes.
