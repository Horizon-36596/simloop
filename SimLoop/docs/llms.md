# Index for AI agents

A flat index of this site. Every page, what is on it, and when to read it — so an agent can go straight to
the right page instead of crawling navigation.

**SimLoop** is a season-agnostic JVM simulation library for FIRST Tech Challenge robot code. Package root
`org.horizon36596.simloop`. Version `0.2.0-alpha.1`. Licence BSD-3-Clause: use, modify and ship it freely, and license your own robot code however you like; keep the copyright notice with any copy. Status: alpha, so pin an exact version; the API may change between alpha versions. `v0.2.0-alpha.1` is the current alpha and `v0.1.0-beta1` is superseded. The coordinate is `com.github.Horizon-36596:simloop:v0.2.0-alpha.1` from JitPack, and the version keeps its leading `v` because a JitPack version IS the git tag. The consuming project must be on FTC SDK 12.0.

## Read this first

- **[Every feature](features.md)**: one line per feature, with a link to its page.
- **[What it does not do](limits.md)**: the authoritative list of what is out of scope. If a task needs
  something on that list, there is no API for it; say so rather than searching. A rendered camera image,
  intake rollers, a shipped field, a command scheduler and controller tuning are out of scope. Game-piece
  physics, a geometry-only camera and fitting to a real log exist, each with stated limits.

## Pages

| Page | Contains | Read it when |
|---|---|---|
| [What SimLoop is](index.md) | The idea, the package table, the four rules the module holds itself to. | Orienting. |
| [Every feature](features.md) | Every feature, one line each, grouped, each linked to its page. | Checking whether a feature exists. |
| [Installing it](getting-started.md) | The complete Gradle change: both repository lines, the dependency, and the JUnit 5 switch — or the one-folder starter that carries all of it. | Writing or fixing a build file. |
| [Your first simulated test](first-test.md) | A complete runnable test — fake motor, plant, controller, scenario, scored result. The same file runs in CI. | Writing a first test, or a template for any test. |
| [What it does not do](limits.md) | Every deliberate boundary, with the reason, plus known rough edges. | Before planning anything; before claiming a feature exists. |
| [Packages overview](packages/index.md) | Which package does what and how they connect. | Choosing where a thing lives. |
| [`fakehardware`](packages/fakehardware.md) | Fake SDK 12.0 devices and `FakeHardwareMap`: motors with the hub's closed loops, servos, IMU, Pinpoint, OTOS, Limelight 3A, touch, distance and colour sensors. Ticks, volts, `[-1, 1]` power; millimetres and radians on the odometry seam. | Wiring hardware into a test. |
| [`plant`](packages/plant.md) | First-order mechanism and drivetrain models: time constant, max speed, end stops, gravity fraction. Drivetrain pose is in the **localizer frame**: +x forward, +y left, CCW-positive. | Modelling motion, or explaining a response. |
| [`config`](packages/config.md) | The interfaces the season implements. Full unit table per method. The two defaults that quietly lie. | Writing a robot's sim config. |
| [`sim`](packages/sim.md) | `FakeTimer`, `ScenarioRunner`, `RlogDecodedCompare`. The four reasons a run refuses to return a log. | Running a scenario, or diagnosing a thrown run. |
| [`opmode`](packages/opmode.md) | `OpModeRunner`: an unchanged `OpMode` or `LinearOpMode` through INIT, the loop and STOP, with scripted gamepads and recorded telemetry. | Testing an OpMode as written. |
| [`simclock`](packages/simclock.md) | How `OpModeRunner.forTeamClass` moves a team's own `ElapsedTime`, `sleep` and `System.nanoTime()` onto sim time, and what it cannot move. | Timer code that reads the real clock. |
| [`sim.live`](packages/live.md) | Keyboard driving, recorded gamepad tracks, headless replay to the identical log, and how to use the unpublished keyboard-window module. | Driving the sim by hand, or replaying a session. |
| [`field`](packages/field.md) | Game pieces with possession and physics (roll, bounce, fly, score), goals, obstacles, kinematic and hinged bodies, the robot as boxes, robot contact, and `FieldFrameTransform`. Intaking is a box test. | Modelling game pieces or the field, or crossing between the two frames. |
| [`sensor`](packages/sensor.md) | `RangeFinder`, `ColorProbe`, `ContactSwitch`: what a sensor on a robot mount sees on the field, driving the sensor fakes. | Simulating touch, distance or colour sensors. |
| [`vision`](packages/vision.md) | `SimCamera`, `SimTagCamera`, lens and mount: which pieces and AprilTags a camera sees, and where. Geometry only. | Testing a vision pipeline's geometry. |
| [`fit`](packages/fit.md) | `FirstOrderFit`, `GapReport`: the plant fitted to a real robot log, with standard errors and warnings. | Comparing the sim with a real robot. |
| [`loop`](packages/loop.md) | `RunResult`, `Objective`, `Guardrail`, `Scorer`, `Gate`, `LoopDriver`, `StopReport`. | Scoring a run or driving iteration. |
| [`viz`](packages/viz.md) | Mechanism and turret sketches, `ArticulatedRobotModel`, `ShotArc` for AdvantageScope. Inches and radians in, metres in the log. | Drawing a mechanism, robot parts or a shot. |
| [Making your timers sim-safe](sim-safe-timers.md) | Why `ElapsedTime`, `sleep` and `System.nanoTime()` read the wrong clock in a sim, and the change that fixes each. | Timer code you can change. |
| [Running your timer code unchanged](unchanged-team-code.md) | `OpModeRunner.forTeamClass(...)`: build setup, what it rewrites, every warning and error message. | Timer code you cannot change. |
| [RUN_TO_POSITION in the sim](run-to-position.md) | What is exact (the SDK side) and approximate (hub firmware) in `FakeMotor`'s closed loops. | Using `RUN_TO_POSITION` or `setVelocity`. |
| [Using SimLoop with an AI agent](agents.md) | The `simloop-agents/` toolkit: a portable `AGENTS.md`, four skills, prompts, token notes — and the rules an agent most often gets wrong. | A team is setting up agent-assisted development, or you are one. |
| [Troubleshooting](troubleshooting.md) | Failures that read like a robot-code bug and are not. | A build or run failed. |
| [Reporting a bug](support.md) | Where issues go, and what a complete report contains. | Filing something. |

## Facts worth having without opening a page

- **Log keys gain a prefix.** `Logger.recordOutput("A/b", x)` is read back as `"RealOutputs/A/b"`.
  Querying the unprefixed key throws `MetricNotFoundException`, never returns zero.
- **Tick order is read, decide, command, then `plant.update(deltaTime)`.** Updating first delays every
  response one tick.
- **Time is always seconds.** `deltaTime` is the scenario's fixed step; 0.02 s is the usual 50 Hz.
- **Mechanism position units are the caller's choice** and nothing converts. Drivetrain pose is always
  inches and radians.
- **There are two frames, ninety degrees apart, and both are deliberate.** `plant` integrates a
  drivetrain pose in the **localizer frame** (+x forward, +y left); `field` places game pieces in the
  **field frame** (+X right, +Y forward). Convert with
  `org.horizon36596.simloop.field.FieldFrameTransform` — `fieldXFromLocalizer`, `fieldYFromLocalizer`,
  `fieldHeadingFromLocalizer`, and the three inverses. Rotation only, no translation; heading *rates*
  are not rotated. Nothing calls it for you, and passing a pose across unrotated returns a confident
  wrong answer rather than throwing.
- **Gravity fraction is always positive**, unitless, in `[0, 1)`. The plant subtracts it, so gravity always
  pulls toward `minPosition`.
- **`UNKNOWN` is a real guardrail verdict** and is not a pass. Read `Scorer.Score.unknowns()` before
  trusting a score.
- **SimLoop never imports season code.** Facts reach it only through the `config` interfaces.
- **SimLoop is a `testImplementation` dependency.** Nothing in it is meant to run on a Control Hub.
- **There is a portable context file for you.** `simloop-agents/AGENTS.md` in the repository is this
  library condensed for an agent: the nine rules, the complete public surface, and the shape of every
  test. A team copies it to their repository root. If you are working in a project that uses SimLoop and
  it has no `AGENTS.md`, suggesting one is a genuinely useful thing to offer.
- **The API reference** is generated by `./gradlew :SimLoop:apiDocs` and ships as
  `SimLoop-<version>-javadoc.jar`. Every public member carries units and frame; that is enforced by the
  build, not by convention.
