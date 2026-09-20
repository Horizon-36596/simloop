# AGENTS.md — this project uses SimLoop

Instructions for an AI coding agent working in this repository. Written in the open `AGENTS.md` format,
which Cursor, Codex, Gemini CLI, Copilot's coding agent, Zed, Windsurf, Jules, Junie and Aider read from
the repository root without being told to. Claude Code reads it too when the project has no
`CLAUDE.md`; if yours does, one import line in it points here — see `simloop-agents/pointers/`.

Everything above the last heading is about **SimLoop**, the simulation library this project tests with,
and is the same in every project. The last section is about your robot and is yours to fill in.

---

## What SimLoop is, in four sentences

SimLoop runs this project's real robot code — the actual `Robot`, the actual subsystems, the actual
OpMode logic — against **fake hardware** inside a plain JVM unit test. No Control Hub, no Driver
Station, no Android emulator, no Robolectric. Time comes from a fake clock instead of the wall, so the
same test produces the same log every time. That log is a PsiKit RLOG: AdvantageScope opens it, and
SimLoop's `loop` package can score it without a human watching.

It is a **`testImplementation` dependency**. Nothing in it runs on a robot, and no robot code should
ever import it.

- Package root: `org.horizon36596.simloop`
- Coordinate: `com.github.Horizon-36596:simloop:v0.1.0-beta1` (JitPack; the version keeps its leading
  `v` because a JitPack version *is* the git tag)
- Documentation: <https://libraries.horizon36596.org/simloop/>
- Licence: AGPL-3.0-or-later

## Run the tests with

```
./gradlew :TeamCode:testDebugUnitTest
```

On Windows, in PowerShell or cmd, that is `.\gradlew.bat :TeamCode:testDebugUnitTest` — the
extensionless `gradlew` is the Unix script and will not execute there. Most FTC teams are on Windows, so
check which shell you are in before reporting that a command failed.

Seconds, not minutes, and no hardware. **Run it after every change.** If you changed something and did
not run it, say so rather than implying you did.

---

## Rules for you, the agent

These are the things that make SimLoop code wrong in ways that still compile and still pass. Read them
before writing a test, not after one fails.

### 1. Do not invent an API

The public surface is small and completely listed below. If the thing you want is not in it, it does not
exist, and the honest move is to say so. **Before claiming SimLoop can do something, check
[what it does not do](https://libraries.horizon36596.org/simloop/limits/).** Contact physics, vision,
AprilTags, field geometry, command scheduling, controller tuning and calibration are all deliberately
out of scope. A plausible method name half-remembered from another library is the single most common way
this goes wrong.

Every class, method and field, with units and frame on every number:
<https://libraries.horizon36596.org/simloop/javadoc/>.

### 2. Log keys gain a prefix when you read them back

`Logger.recordOutput("Slide/heightInches", x)` is read back as **`"RealOutputs/Slide/heightInches"`**.

Querying the unprefixed key throws `MetricNotFoundException`. It does not return zero, and that is
deliberate — a silent zero would make a broken assertion pass.

### 3. The tick order is read, decide, command, then update the world

```java
sim.robot.periodic();          // 1. robot code, unchanged - the same call the OpMode loop makes
sim.advancePhysics(deltaTime); // 2. the world moves, and the encoders learn what happened
Logger.recordOutput(...);      // 3. record what you will assert on
```

Updating physics first delays every response by exactly one tick. That produces a test that is subtly,
consistently wrong and looks fine.

### 4. Time is always seconds, and never the wall clock

`deltaTime` is the scenario's fixed step; `0.02` is 50 Hz, the rate an FTC OpMode loop is written for.
Never call `System.currentTimeMillis()`, `System.nanoTime()`, or anything else that reads real time, on
a simulated path. The clock is `FakeTimer`. Determinism is the whole point of the library — a test that
reads the wall clock is not reproducible, and its log cannot be compared against another run.

### 5. There are two frames, ninety degrees apart, and both are deliberate

- `plant` integrates a drivetrain pose in the **localizer frame**: +x forward, +y left, CCW positive.
- `field` places game pieces and trigger volumes in the **field frame** (FTC-Cartesian): +X right,
  +Y forward.

Convert with `org.horizon36596.simloop.field.FieldFrameTransform` — `fieldXFromLocalizer`,
`fieldYFromLocalizer`, `fieldHeadingFromLocalizer`, and the three inverses. Rotation only, no
translation; heading *rates* are not rotated.

**Nothing calls it for you.** Passing a pose across unrotated returns a confident wrong answer instead
of throwing: the robot lands somewhere plausible, rotated ninety degrees, and the scenario looks like it
ran.

### 6. Units are the caller's choice for mechanisms, and fixed for the drivetrain

Mechanism position units are whatever you decide they are, and **nothing converts them** — if the config
is in inches, the assertions are in inches. Drivetrain pose is always inches and radians. Put the unit in
every name you write (`heightInches`, not `height`), because the name is the only place it is recorded.

### 7. Write assertions that can fail

This is the rule an agent breaks most often, because a passing test looks like success.

Do not assert on a bound the model already guarantees. If the plant clamps a slide to `[0, 24]` inches
every tick, then `assertTrue(height <= 24)` passes no matter what the controller does — it tests the
model, not the robot. Assert on what the code under test can actually get wrong: overshoot, settling
time, final error, whether it held position, whether it arrived at all.

Before you finish, ask of every assertion: **what change to the robot code would make this fail?** If
you cannot name one, the assertion is decoration. The way to prove it is to break the thing on purpose,
watch the test go red, and put it back.

### 8. `UNKNOWN` is a real verdict and is not a pass

A guardrail can return `UNKNOWN`, meaning it could not tell. Read `Scorer.Score.unknowns()` before
trusting a score. Treating `UNKNOWN` as success is how a run that never really happened gets reported as
a good one.

### 9. SimLoop never imports season code

Facts reach it only through the `config` interfaces. If you find yourself wanting SimLoop to know about
a specific subsystem, the answer is a config implementation on this side, not a change to the library.

---

## The whole public surface

Seven packages. Complete as of `v0.1.0-beta1`.

| Package | Classes | What it is for |
|---|---|---|
| `fakehardware` | `FakeMotor`, `FakeServo`, `FakeCRServo`, `FakeExternalEncoder`, `FakeDigitalChannel`, `FakeAnalogInput`, `FakeVoltageSensor`, `FakeI2cDeviceSynchSimple`, `FakeHardwareMap`, plus `FakeDevice` and `AbstractFakeDevice`, the interface and base class the devices share | Fake SDK devices. Ticks, volts, `[-1, 1]` power. They behave like the real SDK classes, including the awkward parts. |
| `plant` | `Mechanism1DofPlant`, `MultiDofMechanismPlant`, `PositionalServoPlant`, `OneDofPlant`, `MecanumDrivePlant`, `MecanumPoseIntegrator` | The physics. First-order models: time constant, max speed, end stops, gravity fraction. Not rigid-body, not contact. |
| `config` | `SimRobotConfig`, `MechanismSimConfig`, `DrivetrainSimConfig`, `PositionalServoSimConfig`, `ContinuousRotationSimConfig` | The interfaces this project implements to describe its own robot. The only way facts get in. |
| `sim` | `FakeTimer`, `ScenarioRunner`, `RlogDecodedCompare` | Running a scenario and writing the log. |
| `field` | `GamePiece`, `GamePieceTracker`, `PossessionState`, `TriggerVolume`, `FieldFrameTransform` | Possession *logic* — where a piece is and who holds it. Not intaking, not contact. |
| `loop` | `RunResult`, `Objective`, `Guardrail`, `EnvelopeGuardrails`, `Scorer`, `Gate`, `Milestone`, `LoopDriver`, `StopReport` | Reading a log back and scoring it. |
| `viz` | `MechanismSketch`, `MechanismSketchDefinition`, `JointDefinition`, `RevoluteJointDefinition`, `LinearJointDefinition`, `RobotVector3` | Drawing a mechanism's pose for AdvantageScope. Inches, radians, robot frame. |

## The shape of every test

```java
FakeTimer timer = new FakeTimer();
ExampleRobotSim sim = new ExampleRobotSim();
Path rlogPath = Paths.get("build", "sim", "my-scenario.rlog");

sim.robot.slide.setLevel(ExampleSlide.Level.SCORE);

ScenarioRunner.run("MyScenario", ExampleRobotSim.ROBOT_CONFIG, timer, 150, 0.02, rlogPath,
        deltaTime -> {
            sim.robot.periodic();
            sim.advancePhysics(deltaTime);
            Logger.recordOutput("Slide/heightInches", sim.robot.slide.getPositionInches());
        });

RunResult run = RunResult.fromRlog("MyScenario", rlogPath, 150);
assertEquals(24.0, run.finalValue("RealOutputs/Slide/heightInches"), 0.5);
```

Logs go under `build/`, which is gitignored — that is where a run belongs. Open one in AdvantageScope by
dragging the `.rlog` file in.

Assert by reading the log back, not by reaching into the subsystem. A log-based assertion also works
against a run recorded on the real field, which is the point.

---

## When you are stuck

In this order, stopping as soon as one answers it:

1. <https://libraries.horizon36596.org/simloop/limits/> — is this out of scope on purpose?
2. <https://libraries.horizon36596.org/simloop/troubleshooting/> — failures that read like a robot-code
   bug and are not.
3. <https://libraries.horizon36596.org/simloop/llms/> — a flat index of every page, written for you.
4. <https://libraries.horizon36596.org/simloop/javadoc/> — the exact signature.

If none of them answer it, **say you do not know**. Guessing an API and writing a test that passes for
the wrong reason costs the team more than an honest stop does.

---

## About this project

<!-- Everything above is about SimLoop and is the same in every project.
     Everything below is yours. Fill it in - an agent that knows your subsystems and your units writes
     far better code than one that has to guess them. -->

- **Team:** <!-- number and name -->
- **Robot code lives in:** `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/`
- **Subsystems:** <!-- one line each, with the units each one uses -->
- **Sim config lives in:** <!-- where your SimRobotConfig implementation is -->
- **Sim tests live in:** <!-- where the tests are -->
- **Conventions we hold to:** <!-- naming, units, anything an agent should match -->
