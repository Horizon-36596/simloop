# `opmode`

`org.horizon36596.simloop.opmode`

Runs your `OpMode` or `LinearOpMode` exactly as you wrote it, against a `FakeHardwareMap`, one fixed tick
at a time. The OpMode imports only the FTC SDK and never learns it is in a test.

## What you use

| Type | What it is |
|---|---|
| `OpModeRunner` | Builds the run: tick counts, gamepads, extra per-tick steps. Then `run()` or `runLogged(...)`. |
| `OpModeRunResult` | What the run left: ticks run, telemetry, warnings, whether the OpMode ended before STOP. |
| `GamepadScript` | A lambda that sets `gamepad1` and `gamepad2` at the start of every tick. |
| `OpModePhase` | `INIT`, `INIT_LOOP`, `ACTIVE`, `STOP`. Handed to the gamepad script. |
| `RecordingTelemetry` | The OpMode's `telemetry`. Keeps every transmission so a test can read what the Driver Station would show. |
| `OpModeException` | An exception thrown by the OpMode, wrapped with the phase and tick it happened on. |

## Running a TeleOp

From `src/test/java/org/horizon36596/simloop/example/OpModeRunnerExampleTest.java`, which runs with the
module's tests. `TankTeleOp` is a plain SDK `OpMode` defined in that file.

```java
FakeHardwareMap hardwareMap = new FakeHardwareMap();
FakeMotor left = new FakeMotor(MOTOR_TICKS_PER_SECOND, MOTOR_RESPONSE);
FakeMotor right = new FakeMotor(MOTOR_TICKS_PER_SECOND, MOTOR_RESPONSE);
hardwareMap.register("left", left);
hardwareMap.register("right", right);

OpModeRunResult result = new OpModeRunner(new TankTeleOp(), hardwareMap)
        .initLoopTicks(10)
        .activeTicks(100)                                   // 2 s at 50 Hz
        .gamepads((phase, tick, gamepad1, gamepad2) -> {
            if (phase == OpModePhase.ACTIVE) {
                gamepad1.left_stick_y = -1f;                // full forward on the left
                gamepad1.right_stick_y = -0.5f;             // half forward on the right
            }
        })
        .run();

List<List<String>> sent = result.telemetry().transmissions();
```

Add `.eachTick(deltaTime -> plant.update(deltaTime))` for physics the fakes do not do on their own, such
as a drive plant. Use `.runLogged(robotConfig, rlogPath)` instead of `.run()` to get an RLOG with one frame
per tick; it goes through [`ScenarioRunner`](sim.md), so it has the same completeness checks.

## The shape of a run

One INIT tick, `initLoopTicks` INIT_LOOP ticks, `activeTicks` ACTIVE ticks, then one STOP tick. Each tick:

1. the `FakeTimer` advances by the tick;
2. the gamepad script runs;
3. the OpMode's code runs;
4. every fake device updates, then the `eachTick` step runs.

An iterative `OpMode` gets the SDK's own call order: `init()`, `init_loop()` each INIT_LOOP tick, `start()`
and the first `loop()` on the first ACTIVE tick, `loop()` each tick after, `stop()` on STOP.

A `LinearOpMode`'s `runOpMode()` runs on its own thread in lockstep with the sim. One pass of
`while (opModeIsActive()) { ... }` is one tick. `waitForStart()` holds until PLAY. `sleep(ms)` holds for
`ceil(ms / tick)` ticks of sim time and no real time.

## Units

- Tick: **seconds**, default `0.02` (`OpModeRunner.DEFAULT_TICK_SECONDS`, 50 Hz). Change it with
  `tickSeconds(...)`.
- `activeTicks(1500)` at 50 Hz is a **30 s** autonomous.
- `stuckTimeoutMillis`: **milliseconds of real time**, default `10000`. An OpMode that runs longer than
  this inside one tick fails the run with its phase, tick and stack.
- `OpModeRunResult.finalTimeSeconds()`: sim **seconds**.

## Things worth knowing

- **Failures fail the test, never hang it.** An exception in the OpMode becomes an `OpModeException`. A
  stuck OpMode fails after `stuckTimeoutMillis`.
- **`OpMode.blackboard` is cleared before INIT.** `.keepBlackboard(true)` keeps it, to pass data from an
  auto run to the TeleOp run after it.
- **Telemetry copied into a field before INIT is still recorded.** The runner swaps those copies for the
  recording telemetry. What it cannot swap is listed in `result.warnings()`.
- **Threads the OpMode starts run in real time beside the sim.** They are listed in `result.warnings()`.
  `.failOnOpModeThreads(true)` fails the run instead.
- **A runner runs once.** Build a new runner and a new OpMode to run again, as the SDK builds a fresh
  OpMode for every INIT.

## Timer code

Run as an instance (`new OpModeRunner(opMode, ...)`), your `ElapsedTime`, `System.nanoTime()` and
`Thread.sleep` read the laptop's clock, and `sleep(ms)` needs a constant duration. Two fixes:

- Change the code to take its clock from one place: [Making your timers sim-safe](../sim-safe-timers.md).
- Leave the code alone and run it with `OpModeRunner.forTeamClass("org.firstinspires.ftc.teamcode.MyAuto",
  hardwareMap)`, which moves those calls onto the sim clock. See [`simclock`](simclock.md) and
  [Running your timer code unchanged](../unchanged-team-code.md).

## What this package will not do

- **It will not step threads the OpMode starts itself.** They run in real time and are reported.
- **It will not move clock reads inside the SDK or other libraries**, the real date and time,
  `Object.wait`, `Thread.join`, latches, futures or executors. `forTeamClass` names the ones it can see.
- **It will not fake hardware that has no fake.** A webcam lookup fails in `init()` exactly as it would
  on a robot missing the device.
- **It will not run OpMode tests in parallel inside one JVM.** Runs share `OpMode.blackboard` and PsiKit's
  static `Logger`. Gradle's separate test JVMs are fine.
- **It will not find your OpModes by annotation.** You name the class or build the instance.

The full list of edges, with every warning message, is in the module
[README](https://github.com/Horizon-36596/simloop/tree/main/SimLoop#running-a-teams-opmode-unchanged-opmode).
