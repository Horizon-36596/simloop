# Every feature

Everything SimLoop `0.2.0-alpha.1` does, one line each. Each line links to the page that explains it. What
it does **not** do is on [What it does not do](limits.md).

## Run your code

- Your robot code runs unchanged against fake hardware in a JVM unit test. No robot, no emulator. [Installing it](getting-started.md)
- Drop-in starter folder: one folder and one Gradle line, with an example robot whose two tests pass. [Installing it](getting-started.md)
- Fixed-tick scenario runner with a deterministic clock: the same run gives the same log. [`sim`](packages/sim.md)
- Run an `OpMode` or `LinearOpMode` as written through INIT, the loop and STOP, with scripted gamepads. [`opmode`](packages/opmode.md)
- Telemetry recorded per transmission, including telemetry copied into fields before INIT. [`opmode`](packages/opmode.md)
- Your unchanged `ElapsedTime`, `sleep`, `Thread.sleep` and `System.nanoTime()` moved onto sim time. [`simclock`](packages/simclock.md)
- `SimElapsedTime`: an SDK `ElapsedTime` that reads the sim clock, for code that takes a timer. [Making your timers sim-safe](sim-safe-timers.md)
- Drive the sim live from the keyboard, paced to real time, on Windows, macOS or Linux. The starter includes the keyboard window. [`sim.live`](packages/live.md)
- Stuck or crashing OpModes fail the test with phase, tick and stack, instead of hanging it. [`opmode`](packages/opmode.md)

## Fake hardware

- `FakeHardwareMap`: resolves fakes by config name and files them into the SDK's typed maps. [`fakehardware`](packages/fakehardware.md)
- `FakeMotor` as a `DcMotorEx`: power, encoder, velocity, current, direction, PIDF. [`fakehardware`](packages/fakehardware.md)
- `RUN_TO_POSITION`, `RUN_USING_ENCODER` and `setVelocity`, with the hub's closed loops. [RUN_TO_POSITION in the sim](run-to-position.md)
- Servos, continuous-rotation servos, external encoders, analog and digital channels, battery voltage. [`fakehardware`](packages/fakehardware.md)
- Universal `IMU`, with physical against declared hub mounting. [`fakehardware`](packages/fakehardware.md#imu)
- goBILDA Pinpoint: the SDK 12.0 driver itself, on an emulated register file. [`fakehardware`](packages/fakehardware.md#odometry-pinpoint-and-otos)
- SparkFun OTOS: the SDK 12.0 driver itself, on an emulated register file. [`fakehardware`](packages/fakehardware.md#odometry-pinpoint-and-otos)
- Limelight 3A: AprilTag results from the tags the robot can see, parsed by the SDK. [`fakehardware`](packages/fakehardware.md#limelight-3a)
- REV touch, 2m distance and Color Sensor V3 fakes, scripted or driven from the field. [`fakehardware`](packages/fakehardware.md#touch-distance-and-colour-sensors)
- Sensors driven from the field: range ray, colour probe, contact switch on a robot mount. [`sensor`](packages/sensor.md)
- Simulated camera: which pieces and AprilTags are in the image, and where, through a distorted lens, with boxes, the robot body and other pieces in the way. [`vision`](packages/vision.md)
- Synthetic camera frames: the scene drawn as RGB pixels for a real `VisionProcessor`, with ground-truth labels and a PNG writer. Flat colours, not for tuning thresholds. [`vision`](packages/vision.md#synthetic-frames-for-a-pixel-pipeline)

## Physics and field

- First-order mechanism plants: one or more joints, end stops, gravity. [`plant`](packages/plant.md)
- Positional servo plant, and a mecanum drive plant with its pose integrator. [`plant`](packages/plant.md)
- Your robot's numbers come in through interfaces you implement; SimLoop never imports your code. [`config`](packages/config.md)
- Game-piece possession: intake trigger volumes, carrying capacity, every transition logged. [`field`](packages/field.md)
- Game pieces that roll, get pushed, bounce, collide and fly a ballistic arc. [`field`](packages/field.md#the-physics-in-brief)
- Goals as scoring volumes, optionally descending-only or open on one face. [`field`](packages/field.md)
- Field perimeter and solid obstacles with a height range. [`field`](packages/field.md)
- 3-D game-piece contact with kinematic bodies and with each other, with rolling friction. [`field`](packages/field.md#the-physics-in-brief)
- Hinged bodies that tip under the pieces' weight, and cells that hold what the physics puts there. [`field`](packages/field.md)
- The robot as a few oriented boxes posed for its mechanisms, and parked robots as solid boxes. [`field`](packages/field.md)
- Robot-against-structure contact: the smallest move that takes the robot back out. [`field`](packages/field.md#stopping-the-robot-at-a-structure)
- Seeded random source for field randomness that replays identically. [`field`](packages/field.md)
- One transform between the drivetrain's frame and the field frame. [`field`](packages/field.md#converting-between-the-two-frames)

## Logs and replay

- Every run writes a PsiKit RLOG that AdvantageScope opens. [`sim`](packages/sim.md)
- The runner refuses to return a truncated or incomplete log. [`sim`](packages/sim.md)
- Decode an RLOG back to fields, and compare two runs field by field. [`sim`](packages/sim.md#reading-the-log-back)
- Live keyboard sessions record a plain-text gamepad track that replays to the identical log. [`sim.live`](packages/live.md)
- Mechanism drawings for AdvantageScope, including a turret with a hood. [`viz`](packages/viz.md)
- Articulated robot model: joint values to the component poses a custom robot model draws. [`viz`](packages/viz.md#robot-parts-articulatedrobotmodel)
- Shot arcs: intended against actual, with each shot's miss distance. [`viz`](packages/viz.md#shots-shotarc-and-shotarcrecorder)
- Game-piece positions logged per kind in 3-D, for the 3D Field tab. [`field`](packages/field.md#one-tick)
- Camera pose logged so AdvantageScope can look through the sim camera. [`vision`](packages/vision.md)

## Scoring and AI iteration

- `RunResult`: read a finished run back and query any logged value. [`loop`](packages/loop.md)
- Objectives, guardrails and a scorer, with `UNKNOWN` kept apart from a pass. [`loop`](packages/loop.md)
- A gate and a loop driver that keep the best candidate and refuse a score that does not reproduce. [`loop`](packages/loop.md)
- An agent task loop: an AI agent changes your code, runs the sim headlessly, reads the score from the log, and keeps the change only if it scores better and the score reproduces. [`loop`](packages/loop.md)

## Programming with an AI agent

- A portable `AGENTS.md` that teaches an agent the library, the complete public API and the rules it gets wrong. [Using SimLoop with an AI agent](agents.md)
- Works with any coding agent: Cursor, Codex, Gemini CLI, Copilot's coding agent and others read `AGENTS.md` with no setup. Claude Code is the best-tested route. [Which tool](agents.md#which-tool)
- Four skills: write a sim test, add a mechanism, debug a failing sim, review sim code. [What is in the folder](agents.md#what-is-in-the-folder)
- Prompts to start from, including one for learning and one for auditing your tests before a competition. [What is in the folder](agents.md#what-is-in-the-folder)
- A flat index of these pages for agents. [Index for AI agents](llms.md)
- **Planned, not in this alpha:** give an agent your own robot's logs and it tunes the sim to match your robot.

## Fitting to real robots

- Read numeric columns out of a real robot's RLOG. [`fit`](packages/fit.md)
- Fit free speed, time constant and friction to logged power and velocity, with standard errors. [`fit`](packages/fit.md)
- Warnings when the data cannot pin the fit down, such as closed-loop logs. [`fit`](packages/fit.md#read-the-warnings-before-the-numbers)
- A sim-against-robot gap report written beside the log. It changes no constant. [`fit`](packages/fit.md)
