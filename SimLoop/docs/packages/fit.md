# `fit`

`org.horizon36596.simloop.fit`

Fits the sim's motor model to a log from your real robot, and writes a report of how far the sim's own
numbers are from what the robot did. It never changes a constant. What to change is your decision.

## What you use

| Type | What it is |
|---|---|
| `RlogTimeSeries` | Numeric columns of an RLOG, one row per frame, with each frame's time. |
| `FirstOrderFit` | Fits free speed, time constant and (optionally) friction to logged power and velocity. |
| `FirstOrderFit.Samples` | One axis of a log: time, applied power, measured velocity, and battery voltage if you logged it. |
| `FirstOrderFit.Model` | A first-order plant: free speed, time constant, friction. Both the fit's answer and the sim's numbers. |
| `FirstOrderFit.Result` | The fitted model, a standard error on every parameter, and `warnings()`. |
| `GapReport` | A markdown table of fit against sim, written beside the log as `<name>.fit.md`. |
| `LeastSquares` | The linear least-squares solver underneath (Householder QR, no library). |

## The model

The same one `FakeMotor` steps, plus friction:

```
dv/dt = (freeSpeed * (u_eff - friction * direction) - v) / timeConstant
u_eff = u * volts / nominalVolts
```

`maxAccel` in the sim configs is `1 / timeConstant` (`Model.maxAccelPerSecond()`). The sim does not model
friction, so a large fitted friction is itself a sim-against-robot gap.

## Fitting one axis of a robot log

The keys here are examples; use whatever your robot logs.

```java
Path rlog = Paths.get("logs", "practice-run.rlog");
String powerKey = "RealOutputs/Flywheel/appliedPower";
String velocityKey = "RealOutputs/Flywheel/velocityTicksPerSecond";
String voltsKey = "RealOutputs/Robot/batteryVolts";

RlogTimeSeries log = RlogTimeSeries.read(rlog, Arrays.asList(powerKey, velocityKey, voltsKey));
FirstOrderFit.Samples samples = new FirstOrderFit.Samples(log.timeSeconds(), log.column(powerKey),
        log.column(velocityKey), log.column(voltsKey), 12.0);

FirstOrderFit.Result fit = FirstOrderFit.fit(samples, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY, true);
FirstOrderFit.Model sim = new FirstOrderFit.Model(2800.0, 0.25, 0.0);   // the sim's own numbers
double simError = FirstOrderFit.outputErrorRms(sim, samples, fit.windowStartSeconds, fit.windowEndSeconds);

new GapReport("Flywheel sim-vs-real gap report", rlog, "logged under RealOutputs/Robot/batteryVolts")
        .add(new GapReport.Quantity("Flywheel", "ticks/s", powerKey, velocityKey, fit, sim, simError))
        .write();                                                        // logs/practice-run.fit.md
```

`src/test/java/org/horizon36596/simloop/fit/GapReportTest.java` runs the second half of this against a
synthetic log and checks every row of the report.

## Units

- Time: **seconds**, on the log's own clock.
- Power: the fraction written to the motor, unitless, `[-1, 1]`.
- Velocity: **any unit you logged**. Free speed comes back in the same unit (ticks/s, in/s).
- Time constant: **seconds**, for an Euler step of one log frame. That is what a sim stepping at the log's
  frame time should use. The continuous plant's is about half a frame shorter; the report gives both.
- Voltage: **volts**. Pass `null` if the log has none, and every frame is taken to be at `nominalVolts`.
- Friction: a fraction of full power, unitless.

## Read the warnings before the numbers

Logs from closed-loop control fool a fit. A flywheel held at a steady setpoint by a PIDF gives power that
is almost a copy of velocity, and the fit then finds parameters that trade off against each other, with a
clean R squared and meaningless values. `Result.warnings()` flags that: a condition number above 100
(`MAX_CONDITION_NUMBER`), or a standard error above 10% of the value (`MAX_RELATIVE_STANDARD_ERROR`). `isTrustworthy()` is true only when
there are none.

That flag covers the fit's diagnostics. It does not establish that a first-order model represents
your mechanism or that it predicts a separate real-robot run. Compare against another recording
before using the fitted constants as a model of the robot.

Data where the driver drove open-loop, the controller saturated, or the setpoint changed separates the
parameters. Give the fit that.

## Fixed rules it applies

- Windows of `WINDOW_FRAMES` (10) frames, not single frames, so velocity noise does not drag the time
  constant short.
- A window with a gap over `MAX_GAP_SECONDS` (0.1 s) is skipped: the log paused.
- A window that stays at rest throughout is skipped. Rest is under `REST_FRACTION` (2%) of the 99th
  percentile speed.
- `fit(...)` throws `IllegalArgumentException` rather than return a number it cannot stand behind: fewer
  than `MIN_INDEPENDENT_WINDOWS` (20) usable windows, a power term that is zero, negative or not finite
  (usually a sign mix-up between the power and velocity keys), or a velocity that would grow on its own.

## What this package will not do

- **It will not change any constant.** The report says so in its notes.
- **It will not fit anything but a first-order velocity plant.** No position loops, no gravity, no
  backlash, no second-order dynamics.
- **It will not tune a controller.** It measures the plant; gains are yours.
- **It will not remove all noise bias.** Windowing dilutes it. On the test's drivetrain log with 1% noise
  the time constant comes out about 1.5% long.
- **It will not pick your log keys.** A season names its own keys and sim parameters.
