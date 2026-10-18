# What SimLoop does not do

Read this page before you plan anything around SimLoop. Everything here is a deliberate boundary or a
known limit, not a gap waiting to be filled quietly. Each package page ends with its own list; this page
collects the ones that shape a plan.

!!! warning "If you are an AI agent"
    This page is the authoritative list of what is out of scope. If a task needs something on it, the
    answer is not to look harder for the API: there isn't one. Say so and propose the alternative named
    here.

## The robot is not a rigid body

Mechanisms and the drivetrain are **parameterized first-order models**: one time constant, one speed
ceiling, end stops, a gravity term. That answers "does my controller settle where I told it, in about the
time I expect, without running past its own limit", which is the question most control bugs are.

What that leaves out:

- **The robot does not collide by itself.** The drive plant knows nothing about the field. You ask
  `FootprintContact` or `RobotContact` whether the robot overlaps something and move it back out. See
  [`field`](packages/field.md).
- **Robots do not push each other.** A `StaticRobot` is a solid box that stands still; there is no second
  driven robot.
- **A game piece never pushes the robot back.** To a piece the robot is immovable.
- **No tipping, no backlash, no motor heating.**

## Game pieces: physics, with stated limits

Game pieces do have physics: they roll, bounce, collide, fly and score
([`field`](packages/field.md)). The limits:

- **Intaking is a box test.** A running mechanism with a piece in its box and room to carry it takes the
  piece. Rollers and compliance are not simulated, so "will my intake grab it" is out of scope.
- **Obstacle contact is not swept.** A small, fast piece can pass through a thin obstacle. Goals are swept.
- **A flying piece passes through pieces lying on the floor.**
- **`ObstacleBox` has no top**, so pieces cannot rest on one. Use a `KinematicBody`.
- **The physics numbers are defaults** for a foam ball a few inches across. Measure yours.

## Vision is geometry only

`vision` says which pieces and AprilTags a camera would see and where, and `FakeLimelight3A` turns that
into Limelight results. There is **no image**. So:

- Colour thresholds, blob detection and lighting cannot be tested, and a synthetic scene must never be
  used to tune them.
- No lens distortion, no motion blur, no dropouts. The robot's own body does not block the view.
- Limelight: AprilTag pipelines only. Its network-only calls throw.
- **No webcams.** A `WebcamName` lookup fails in `init()`.

## Sensors are simple

- A distance sensor is a single ray, not a 25 degree cone, and sees every surface.
- A colour sensor reads one set of counts per kind of piece, whatever the distance or angle.
- No noise or drift on the IMU, Pinpoint, OTOS, distance or colour fakes. The Limelight has optional
  seeded noise.
- No legacy `BNO055IMU`, and no generic I2C devices.

## It ships no field

No season's field, goals, tag layout or game pieces. You build a `FieldWorld` and an `AprilTagLayout` from
numbers you are entitled to use. **Do not build them from the FIRST field CAD**: its licence forbids
redistribution.

## It does not schedule anything

No command scheduler, no subsystem registry, no requirements. If you use SolversLib, SolversLib keeps
owning your control flow exactly as it does on the robot. SimLoop replaces hardware underneath it.

## Some timers stay on the real clock

`OpModeRunner.forTeamClass` moves your own `ElapsedTime`, `sleep`, `Thread.sleep` and `System.nanoTime()`
onto sim time. It does **not** move clock reads inside the SDK or other libraries, the real date and
time, `Object.wait`, `Thread.join`, latches, futures or executors. Threads your OpMode starts run in real
time. See [Running your timer code unchanged](unchanged-team-code.md).

## It does not tune your controller

There is no auto-tuner and no optimizer over gains. `fit` measures the plant from a real log (free speed,
time constant, friction); choosing gains is still yours. The `loop` package scores runs and decides
whether a candidate is better, but *what to change* comes from you or from an AI agent driving it.

## Its numbers are as good as your numbers

A plant is a model with constants in it. If `timeConstant` and `maxSpeed` were guessed, the simulation is
a plausible story, not a prediction. SimLoop cannot tell a measured constant from an invented one.

`fit` compares the sim's numbers with a real robot log and writes the gap as a report. It changes no
constant, and it fits only a first-order velocity plant. Logs from closed-loop control can fool it;
read its warnings first.

## It does not know what your robot is

No season code, no subsystem classes, no drivetrain of yours. SimLoop never imports your package. The
dependency runs one way, through the [`config`](packages/config.md) interfaces your code implements. A test
enforces it.

## It is not a robot-side library

Nothing here is meant to run on a Control Hub. It is a `testImplementation` dependency.

## Live driving is keyboard only

No real game controller: a controller needs a native input library. The keyboard window ships with the
starter; see [`sim.live`](packages/live.md).

## Known rough edges

These are defects or unfinished corners rather than boundaries.

- **Nothing calibrated has shipped.** Every constant in every example on this site is plausible, not
  measured. The hub's motor-loop firmware constants are labelled APPROXIMATION
  ([RUN_TO_POSITION in the sim](run-to-position.md)).
- **There are two frames, ninety degrees apart, and converting is your job.** The drivetrain integrates
  into the localizer frame (+x forward / +y left); game pieces, sensors, cameras and trigger volumes live
  in the field frame (+X right / +Y forward). `FieldFrameTransform` does the rotation, but nothing calls
  it for you and nothing warns you when you forget. See [`field`](packages/field.md).
- **Loop-time guardrails have nothing to check.** `EnvelopeGuardrails.loopTimeUnderBudget` returns
  `UNKNOWN` until a scenario logs a loop time, which none does yet. `Scorer.Score.unknowns()` is how you
  find out.
- **OpMode tests cannot run in parallel inside one JVM.** Runs share `OpMode.blackboard` and PsiKit's
  static `Logger`.
