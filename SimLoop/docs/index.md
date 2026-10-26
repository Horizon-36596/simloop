# SimLoop

**Run your FTC robot code on a laptop, deterministically, and get a log you can score.**

Your `Robot.init()`, your subsystems and your OpMode run unchanged against fake hardware in a JVM unit
test. Time comes from a fake clock rather than the wall, so the same run produces the same log every
time. The log is a PsiKit RLOG, which AdvantageScope opens and which SimLoop's own `loop` package can
score automatically.

No robot, no Driver Station, no emulator. A test run is a few seconds.

## What you open

Run SimLoop from your robot project's tests or simulation harness. **AdvantageScope opens in its
own window** to display the simulation's logs and visualizations. The standalone app being built
around SimLoop is not ready and is not included in this release.

!!! warning "Status: alpha"
    `v0.2.0-alpha.1` is the current release and the first public alpha. The API may change between
    alpha versions, so **pin the exact version** in your build. It needs **FTC SDK 12.0**. Questions go
    to [Discussions](https://github.com/Horizon-36596/simloop/discussions); bugs go to
    [Issues](https://github.com/Horizon-36596/simloop/issues). `v0.1.0-beta1` is superseded and gets no
    fixes.

    **It has been used by one team, on one robot's code, and its physics has not yet been compared with
    a real robot.** You are testing it. [What has been tested](tested.md) says what has been checked and
    what has not.

[Every feature](features.md) lists what it does, one line each.

!!! note "Match the docs to your dependency"
    These source docs can include changes newer than the published tag. The vision guide marks its
    development APIs. For a build pinned to `v0.2.0-alpha.1`, use the
    [docs at that tag](https://github.com/Horizon-36596/simloop/tree/v0.2.0-alpha.1/SimLoop/docs).

## The fastest way in

Copy one folder into your FTC project and add one line to `TeamCode/build.gradle`:

```groovy
apply from: "$rootDir/simloop-starter/simloop.gradle"
```

That folder carries every build setting SimLoop needs and a small example robot that drives itself, so
`./gradlew :TeamCode:testDebugUnitTest` passes two simulated tests immediately. You never clone this
repository. [Installing it](getting-started.md) has the three steps, and then says exactly what the
folder is doing if you would rather wire it by hand.

## Who this is for

Two readers, and the site is shaped for both.

**A team that has written robot code but never a simulator.** You know what a `DcMotorEx` is and what a
subsystem does. You do not need convincing that testing is good; you need the twenty lines of Gradle that
make it work and a first test that passes. Start at [Installing it](getting-started.md), then
[Your first simulated test](first-test.md).

**An AI agent writing code against this library.** Every page states units and frame for every number,
because those cannot be inferred from a signature. Every package page ends with what that package will
not do, because the expensive failure is not a wrong call — it is confidently writing against a feature
that does not exist. [What it does not do](limits.md) is the page that matters most for that, and
[Index for AI agents](llms.md) is a flat list of every page and what is on it.

## The idea in one paragraph

A robot loop reads hardware, decides, and writes hardware. SimLoop supplies SDK-compatible fakes:
`FakeMotor` implements `DcMotorEx`, and `FakeHardwareMap` extends `HardwareMap`. Robot code using the
supported methods can run against those devices. **Plants** model mechanism and drivetrain motion;
your test wires their state into sensors and supplies your robot's parameters through `config`
interfaces. SimLoop never imports your team's package. `ScenarioRunner` owns the fixed clock and
logging session, and your test asserts on the resulting log.

## What is in it

| Package | What it gives you |
|---|---|
| [`fakehardware`](packages/fakehardware.md) | Fake devices that implement the real SDK interfaces, including the IMU, Pinpoint, OTOS, Limelight 3A and REV sensors, and the `FakeHardwareMap` you hand to `init()`. |
| [`plant`](packages/plant.md) | The physics behind those fakes: drivetrain, 1-DOF and multi-DOF mechanisms, positional servos. |
| [`config`](packages/config.md) | The interfaces your season code implements to state its own numbers. The one-way seam. |
| [`sim`](packages/sim.md) | `FakeTimer`, `ScenarioRunner`, and decoding an RLOG back to fields. |
| [`opmode`](packages/opmode.md) | Running your `OpMode` or `LinearOpMode` unchanged, with scripted gamepads. |
| [`simclock`](packages/simclock.md) | Moving your own timer code onto sim time without changing it. |
| [`sim.live`](packages/live.md) | Driving from the keyboard, recording the session, and replaying it to the same log. |
| [`field`](packages/field.md) | Game pieces with possession and physics, goals, field structures, and robot contact. |
| [`sensor`](packages/sensor.md) | What a touch, distance or colour sensor on the robot sees on the field. |
| [`vision`](packages/vision.md) | Which game pieces and AprilTags a camera sees, and where, with distortion and occlusion. Synthetic frames for pixel pipelines, not photos. |
| [`fit`](packages/fit.md) | Fitting the motor model to a real robot log, and a sim-against-robot gap report. |
| [`loop`](packages/loop.md) | Scoring a run, and the automated edit-and-replay iteration built on top of it. |
| [`viz`](packages/viz.md) | Mechanisms, robot parts and shot arcs drawn into the log for AdvantageScope. |

The generated API reference — every class, method and field, with units and frame on every number — is
built by `./gradlew :SimLoop:apiDocs` and ships in the release as `SimLoop-<version>-javadoc.jar`.

## The rules it holds itself to

These are enforced by tests in the module, not by good intentions.

- **No season imports.** No file in `org.horizon36596.simloop` imports `org.firstinspires.ftc.teamcode`.
  Season facts arrive through the `config` interfaces.
- **Depends only on** the FTC SDK 12.0 and PsiKit. No SolversLib, no follower library.
- **Deterministic model time.** Plants advance from the supplied `deltaTime`; getters are pure reads.
  I/O waits and live pacing do not advance that clock. Robot code must use sim time too.
- **SDK-facing fakes.** Robot code uses FTC SDK types. The fake devices implement their state and
  control interfaces; readings come from the simulation model. See the
  [hardware guide](packages/fakehardware.md) for behavior and limits.

## Help build SimLoop

SimLoop is built by Horizon (FTC 36596) for any FTC team, and other teams and simulator authors are
welcome to add to it. It is season-agnostic, so what you add helps every season. Wanted most:

- **New fake devices** that replicate a real FTC SDK class, with tests against the SDK's behaviour.
- **Field and game-piece models** built from the general pieces in [`field`](packages/field.md), never
  one game's field.
- **Documentation**: a page that confused you, a missing unit, a missing example.
- **Examples**: worked tests for common robots.
- **Interop with other tools**: log formats, AdvantageScope layouts, replay and analysis tools.

Read [`CONTRIBUTING.md`](https://github.com/Horizon-36596/simloop/blob/main/CONTRIBUTING.md) first. Small
starter tasks are in
[`docs/good-first-issues.md`](https://github.com/Horizon-36596/simloop/blob/main/docs/good-first-issues.md).
For anything bigger, start a
[Discussion](https://github.com/Horizon-36596/simloop/discussions).
