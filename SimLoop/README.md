# SimLoop

Run your FTC robot code on a laptop, deterministically, and get a log you can score.

SimLoop is the season-agnostic core of the Horizon (FTC 36596) sim + loop tooling. Your `Robot.init()`,
your subsystems and your OpMode run **unchanged** against fake hardware in a JVM unit test; time comes
from a fake clock rather than the wall, so the same run produces the same log every time; and the log
is a PsiKit RLOG, which AdvantageScope opens and which the `loop` package can score automatically.

Package root: `org.horizon36596.simloop`. Status: **alpha**. The API may change between alpha versions,
so pin an exact version. Support is best effort. `v0.2.0-alpha.2` is the current alpha; it supersedes
`v0.1.0-beta1`, which gets no fixes. It needs **FTC SDK 12.0**. Questions go to
[Discussions](https://github.com/Horizon-36596/simloop/discussions), bugs to
[Issues](https://github.com/Horizon-36596/simloop/issues). Every feature, one line each:
[`docs/features.md`](docs/features.md).

## Installing

**Your project must be on FTC SDK 12.0.** SimLoop passes the SDK 12.0.0 through at compile scope (see the
end of this section), and its Pinpoint, OTOS and Limelight 3A fakes are the SDK 12.0 driver classes.

The coordinate spelling below was confirmed on 2026-09-19 against `v0.1.0-beta1`, the earlier build:
a throwaway stock-layout FTC project resolved it for real from JitPack, with nothing published locally.
Each later tag uses the same spelling with its own tag.

Note the two spellings, because mixing them up fails with `Could not find`, which reads like a missing
library rather than a wrong name. **JitPack derives its own coordinate from the repository address** and
ignores what this project calls itself: `com.github.Horizon-36596:simloop:v0.2.0-alpha.2`, with the tag as
the version, `v` included. The POM inside says `org.horizon36596:SimLoop:0.2.0-alpha.2`, which is what
`publishToMavenLocal` writes and what a future Maven Central release would use.

JitPack, which builds a tag of this repository on demand:

In a `build.gradle` — your root one's `allprojects { }` block, or the module's own:

```groovy
repositories {
    mavenCentral()
    google()
    maven { url "https://jitpack.io" }
    maven { url "https://repo.dairy.foundation/releases" }   // PsiKit lives here, not on Maven Central
}
```

Or, if your project declares its repositories in `settings.gradle` instead — which newer Android project
templates do — the same four lines go inside `dependencyResolutionManagement`:

```groovy
// settings.gradle
dependencyResolutionManagement {
    repositories {
        mavenCentral()
        google()
        maven { url "https://jitpack.io" }
        maven { url "https://repo.dairy.foundation/releases" }
    }
}
```

**A bare `repositories { }` block does not work in `settings.gradle`.** `repositories` is part of the
project DSL; a settings file has no such method, in any Gradle version, so pasting the first block there
fails while Gradle is still evaluating settings — before it ever tries to resolve SimLoop — with
`Could not find method repositories()`. The error names the wrong thing, which is why both forms are
written out here.

**Check your `settings.gradle` for `repositoriesMode` before you pick a form.** If it contains
`repositoriesMode.set(RepositoriesMode.FAIL_ON_PROJECT_REPOS)` — which Android Studio's newer project
templates put there by default — the settings block is the only one allowed, and adding the
`build.gradle` form as well fails with a different message again: "Build was configured to prefer settings
repositories over project repositories". Without that line, both forms work and either will do.

```groovy
// TeamCode/build.gradle
dependencies {
    testImplementation 'com.github.Horizon-36596:simloop:v0.2.0-alpha.2'

    // JUnit 5, if you are not already on it. SimLoop's own tests use it; yours need not.
    testImplementation 'org.junit.jupiter:junit-jupiter-api:5.11.3'
    testRuntimeOnly 'org.junit.jupiter:junit-jupiter-engine:5.11.3'
    testRuntimeOnly 'org.junit.platform:junit-platform-launcher:1.11.3'
}

android {
    testOptions { unitTests.all { useJUnitPlatform() } }
}
```

**That Dairy repository line is not optional.** PsiKit is not on Maven Central, and a POM can tell your
build what a dependency is but never where to find it. Leaving it out fails at resolution with
`Could not find org.psilynx.psikit:core`, which reads like SimLoop is broken and is not.

`testImplementation` rather than `implementation` is the normal case: SimLoop is for the tests that run
on your laptop, and nothing here belongs on the robot. Put it on `implementation` only if you are
writing an OpMode that uses the fakes on the Control Hub, which is a strange thing to want.

SimLoop pulls in the FTC SDK (`RobotCore` and `Hardware`, 12.0.0) and PsiKit at **compile** scope,
because its own public types are SDK types — `FakeMotor` is a `DcMotorEx`, `FakeHardwareMap` is a
`HardwareMap`. If your project pins an older SDK, Gradle quietly moves your test classpath up to 12.0.0
while your robot code still compiles against the older one, so the two disagree. Keep your project on
12.0; the note in `build.gradle` says why the versions have to agree.

## The documentation

Full documentation lives in `docs/` in this directory and builds into a site: installing it, a first test
that actually runs, a page per package with units and frame on every number, what SimLoop deliberately
does **not** do, and a flat index written for AI agents. Start at `docs/index.md`.

## What is in it

| Package | What is in it |
|---|---|
| `fakehardware` | Fake FTC devices that implement the real SDK 12.0.0 interfaces (`FakeMotor`, `FakeServo`, `FakeCRServo`, `FakeExternalEncoder`, `FakeDigitalChannel`, ...) and `FakeHardwareMap`, which resolves them by name. |
| `plant` | The physics behind the fakes: first-order motor response, the 1-DOF and multi-DOF mechanism plants with end-limits, the positional-servo plant, and the mecanum drive plant and pose integrator. |
| `config` | The interfaces a season implements to hand its robot's numbers in (`SimRobotConfig`, `DrivetrainSimConfig`, `MechanismSimConfig`, servo configs). The core never reaches out for a season fact. |
| `sim` | `FakeTimer` (the one clock), `ScenarioRunner` (the fixed-timestep tick loop plus the whole PsiKit lifecycle, and `runUntil` for a run of unknown length), and `RlogDecodedCompare` (field-by-field RLOG comparison for determinism tests). |
| `sim/live` | Live drive: a recorded, replayable gamepad input track (`GamepadInputTrack`), a keyboard-to-gamepad keymap (`KeyboardGamepadSource`), and `RealTimePacer`. |
| `field` | Field bookkeeping and game-piece simulation: the localizer-to-field frame transform, trigger and scoring volumes, obstacles, game-piece possession and rolling/flight physics, and a seeded random source. |
| `vision` | A simulated camera: lens (optionally distorted) and mount geometry, which game pieces and AprilTags fall inside the image and where, with boxes, the robot body and other pieces in the way, and `SimFrameRenderer`'s synthetic RGB frames for running a pixel pipeline. Synthetic frames are flat colours, never for tuning thresholds. |
| `viz` | `MechanismSketch` and its definitions: a robot's joints drawn as a PsiKit mechanism for AdvantageScope. `ArticulatedRobotModel`: a joint tree that turns joint values into the component poses a custom AdvantageScope robot asset draws. |
| `loop` | Scoring runs from their logs and keeping the best: `RunResult` (a run read back from its RLOG), `Objective` and `Scorer` (measure), guardrails and `Gate` (accept or reject a candidate), and `LoopDriver` (last-best bookkeeping, `Milestone`s, the `StopReport`). It proposes nothing. |
| `fit` | Fitting the sim to a real robot log: `LeastSquares` (Householder QR, written out), `FirstOrderFit` (the plant model `FakeMotor` steps, plus friction), `RlogTimeSeries` (timestamped columns of an RLOG) and `GapReport` (the markdown sim-vs-real table). A season supplies its own key names and sim parameters. |
| `sensor` | What a sensor on a robot mount sees or touches from the field (`RangeFinder`, `ColorProbe`, `ContactSwitch`), driving the touch, distance and colour fakes. |
| `opmode` | `OpModeRunner`: drives a team's unchanged `OpMode` or `LinearOpMode` through INIT, the loop and STOP against a `FakeHardwareMap`, with scripted gamepads. |
| `simclock` | Puts a team's `ElapsedTime`, `System.nanoTime()` and `Thread.sleep` on the sim clock by rewriting the team's own classes as they load (`OpModeRunner.forTeamClass`). |

## The smallest run

Excerpted from `src/test/java/org/horizon36596/simloop/example/SlideReachesTargetExampleTest.java`, which
compiles and runs with this module's tests. Read that file for the `SLIDE` and `ROBOT` configs and the
full comments.

```java
FakeTimer timer = new FakeTimer();

FakeMotor slideMotor = new FakeMotor(2000.0, 8.0);
FakeHardwareMap hardwareMap = new FakeHardwareMap();
hardwareMap.register("slide", slideMotor);
Mechanism1DofPlant slidePlant = new Mechanism1DofPlant(slideMotor, SLIDE);

Path rlogPath = Paths.get("build", "sim", "slide-reaches-target.rlog");

ScenarioRunner.run("SlideExample", ROBOT, timer, TICKS, DELTA_TIME_SECONDS, rlogPath, deltaTime -> {
    double error = TARGET_INCHES - slidePlant.getPosition();
    double power = Math.max(-1.0, Math.min(1.0, error * 0.25));
    slideMotor.setPower(power);

    slidePlant.update(deltaTime);
    slideMotor.setMeasuredShaftSpeedFraction(slidePlant.getVelocityFractionOfMaxSpeed());
    hardwareMap.updateAll(deltaTime);

    Logger.recordOutput("Slide/positionInches", slidePlant.getPosition());
});

RunResult run = RunResult.fromRlog("SlideExample", rlogPath, TICKS);
double finalInches = run.finalValue("RealOutputs/Slide/positionInches");
```

`ScenarioRunner.run` throws rather than returning a log you cannot trust: if PsiKit's writer queue
overflowed, if the file is empty, if it decodes to a different number of frames than ticks you ran, or
if the writer thread stopped making progress while the run waited for it.
That last one is why the class paces itself against the writer — a simulator has no 50 Hz clock holding
it back, and will otherwise outrun the writer and lose the tail of the log with no error at all.

However the run ends, PsiKit is shut down. A scenario body that throws propagates its own exception, and
the writer is still flushed and closed on the way out, so the partial log is readable and the next run in
the same JVM starts clean.

## Live drive (`sim/live/`, `desktop/`)

A recorded, replayable gamepad input track (`GamepadInputTrack`, plain text, one line per loop), a
keyboard-to-gamepad keymap (`KeyboardGamepadSource`), and `RealTimePacer`, which slows a run to real time from
the outside and is the one class here allowed to read a wall clock. `ScenarioRunner.runUntil` runs a scenario
until its input runs out. The keyboard window itself is the plain Java module `desktop/` (`:SimLoop:desktop`,
JDK AWT/Swing only), because this Android module compiles against `android.jar`, which has no AWT. It is
published as a second jar of this module's publication, classifier `desktop`
(`com.github.Horizon-36596:simloop:<tag>:desktop`), and the starter folder already depends on it. A team
wires them to its own OpMode in its own test sources; the keyboard jar is a `testImplementation`
dependency there, never something the robot build sees.

## Fitting the sim to a real log (`fit/`)

`FirstOrderFit` fits `dv/dt = (freeSpeed * (u - friction * sign v) - v) / timeConstant` to a logged applied
power and measured velocity over 10-frame integral windows, with a standard error on every parameter. A fit
the data cannot pin down (typically closed-loop data at a steady setpoint) is flagged in the report, not
presented as a measurement, and a fit whose power gain is zero, non-finite or negative is refused. It writes
a report beside the log and changes no constant. A team supplies its own log key names and sim parameters,
typically from a small test class in its own `TeamCode` that points the fit at one of its robot's logs.

## Fake goBILDA Pinpoint (`fakehardware/FakeGoBildaPinpoint`)
`FakeGoBildaPinpoint` *is* the SDK 12.0 `GoBildaPinpointDriver`, overriding none of its public methods,
running on an emulated device register file. Robot code that does
`hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint")` gets it and runs unchanged.

Wiring, in the sim harness (never in robot code):

```java
FakeGoBildaPinpoint pinpoint = new FakeGoBildaPinpoint();
pinpoint.setPhysicalPodMount(xOffsetMm, yOffsetMm, xDirection, yDirection, ticksPerMm); // how the pods REALLY sit
hardwareMap.register("pinpoint", pinpoint);
pinpoint.attachChassisAt(plant.getX() * 25.4, plant.getY() * 25.4, plant.getHeading());
// every tick, after the drive plant has integrated, then updateAll:
pinpoint.moveChassisTo(plant.getX() * 25.4, plant.getY() * 25.4, plant.getHeading());
```

- **Physical vs configured.** `setPhysicalPodMount` is the truth; what the robot writes (pod offsets,
  directions, resolution) is what it believes, so a mis-set value reads wrong, as on a robot. **Required
  wiring step:** a robot that writes its real offsets, directions or resolution needs its sim wiring to
  call `setPhysicalPodMount` with the same values; against the default mount (pods on the centre, 4-bar
  resolution) it drifts or reads at the wrong scale.
- **Millimetres** on the seam (convert a drive plant's inches), the same seam a season's fake OctoQuad uses.
- **Bulk-read scope.** Because the real driver runs, a getter whose register is outside the driver's
  bulk-read scope reads the bus on every call - the default scope leaves out the ID, version, yaw scalar,
  offsets, quaternion, pitch and roll - as it does on a robot.
- What it models, and which of its values are PLACEHOLDER or NEEDS MEASUREMENT, is in its class javadoc.

## Touch, distance and colour sensors (`fakehardware/`, `sensor/`)
Register them like any fake; unchanged robot code finds them by the SDK types it already uses:

```java
FakeTouchSensor touch = new FakeTouchSensor();          // hardwareMap.get(TouchSensor.class, "liftDown")
FakeDistanceSensor range = new FakeDistanceSensor();    // get(DistanceSensor.class, "frontRange"), a REV 2m
FakeRevColorSensorV3 color = new FakeRevColorSensorV3(); // RevColorSensorV3, ColorRangeSensor, ColorSensor,
hardwareMap.register("liftDown", touch);                // NormalizedColorSensor or DistanceSensor
hardwareMap.register("frontRange", range);
hardwareMap.register("intakeColor", color);
```

- **Scripted:** `touch.setPressed(true)`, `range.setDistance(DistanceUnit.INCH, 12)`,
  `color.setRawCounts(new ColorCounts(r, g, b))` and `color.setDistance(...)` take effect at once.
- **From a source:** `touch.setPressedSource(() -> tracker.getHeldCount() > 0)` and the other `set...Source`
  methods are asked once per `hardwareMap.updateAll(dt)`.
- **From the field:** `sensor/RangeFinder`, `sensor/ColorProbe` and `sensor/ContactSwitch` work out what a
  sensor on a `SensorMount` sees or touches, from the `FieldWorld`, the pieces and the robot's field-frame
  pose, and drive the fake: `new RangeFinder(mount).apply(range, world, tracker.getPieces(), x, y, heading)`.
- **A switch read as a `DigitalChannel`:** `SwitchPolarity.PRESSED_IS_LOW.drive(channel, pressed)` puts the
  REV Touch Sensor's inverted level on a `FakeDigitalChannel`.
- **Real-device numbers:** a REV 2m reports 8190 mm with nothing in range (never
  `DistanceSensor.distanceOutOfRange`), a Color Sensor V3 never reads farther than 6 in, and colour counts
  for a game piece must be measured on the real sensor. The class javadocs give every number's provenance.

## Fake IMU (`fakehardware/FakeImu`)
`FakeImu` implements the SDK's universal `IMU`, so the init code nearly every FTC robot runs works
unchanged in sim:

```java
IMU imu = hardwareMap.get(IMU.class, "imu");
imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(LogoFacingDirection.UP, UsbFacingDirection.FORWARD)));
imu.resetYaw();
double headingDeg = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
```

Wiring, in the sim harness (never in robot code):

```java
FakeImu imu = new FakeImu(new RevHubOrientationOnRobot(logo, usb)); // how the hub is REALLY mounted
hardwareMap.register("imu", imu);
// every tick, after the drive plant has integrated:
imu.setChassisHeading(drivePlant.getHeading()); // radians, CCW, not wrapped
```

- **The SDK does the maths.** Every `IMU` method runs through the SDK's own `QuaternionBasedImuHelper`, the
  class the SDK 12.0 `BHI260IMU` driver uses, so units, the `[-180, 180)` wrap, `resetYaw()` and the
  mounting remap are the SDK's. The fake supplies only the raw device quaternion and gyro rates.
- **Physical vs declared mounting.** The constructor says how the hub is really mounted; `initialize`
  says what the robot code believes. When they match, yaw is the plant heading; when they do not, the
  robot reads what a mis-configured robot reads (a hub on its edge declared flat looks tipped 90 degrees).
- **Power-on zero.** Yaw is zeroed once, at construction, as a real hub zeroes it when the hardware map is
  built; `initialize` does not zero it again. Seed a start heading, then call `resetYaw()` if the robot
  should have powered up facing that way.
- **Gyro rate** is the injected heading finite-differenced across `updateAll` ticks, CCW positive about
  robot `+z`; a harness that injects after `updateAll` reads it one tick behind the heading.
- **Deterministic:** acquisition times are simulated nanoseconds (the sum of `update` ticks).
- **Your test JVM needs `unitTests.returnDefaultValues = true`** (in `android { testOptions { ... } }`).
  The SDK's orientation types, including the `new RevHubOrientationOnRobot(logo, usb)` above, call
  `android.opengl.Matrix.setIdentityM`, which otherwise throws "not mocked". With the flag the results are
  exact: the SDK only uses the matrix's 3x3 rotation block.
- **Not modelled:** noise, drift, quantization, tilt of the chassis, accelerometer and magnetometer, and
  the legacy `BNO055IMU` interface.

## Fake SparkFun OTOS (`fakehardware/FakeSparkFunOTOS`)
`FakeSparkFunOTOS` *is* the SDK 12.0 `SparkFunOTOS` driver, overriding none of its methods, running on an
emulated sensor register file. Robot code that does `hardwareMap.get(SparkFunOTOS.class, "otos")` gets it
and runs unchanged, starting in inches and degrees as the SDK's hardware map leaves a real one.

Wiring, in the sim harness (never in robot code):

```java
FakeSparkFunOTOS otos = new FakeSparkFunOTOS();
otos.setPhysicalMount(forwardMm, leftMm, headingRad);  // where the sensor REALLY sits (default: centre, facing forward)
hardwareMap.register("otos", otos);
otos.attachChassisAt(plant.getX() * 25.4, plant.getY() * 25.4, plant.getHeading());
// every tick, after the drive plant has integrated, then updateAll:
otos.moveChassisTo(plant.getX() * 25.4, plant.getY() * 25.4, plant.getHeading());
```

- **Physical vs declared mounting.** `setPhysicalMount` is the truth; the robot's `setOffset` is what it
  believes. When they match the pose is the robot centre's; when they do not it reads what a mis-declared
  real OTOS reads. `setPhysicalTrackingGains` is the same idea for the scalars: how much the real sensor
  over- or under-reads, which the robot's `setLinearScalar` / `setAngularScalar` should cancel.
- **Millimetres and radians** on the seam, the same as `FakeGoBildaPinpoint`, so one line feeds either.
- **The SDK's quirks are kept**, because its code runs: the int16 register quantisation (about 0.3 mm),
  the scalar setter's truncation (0.95 is stored as 0.951), and `AngleUnit` wrapping an angular velocity
  above 180 deg/s.
- **Calibration takes no sim time when blocking.** `calibrateImu()` returns true inside the call, as a real
  one does inside one loop; a non-blocking calibration counts down in sim time. The SDK's own
  `Thread.sleep` in `calibrateImu`/`selfTest` still takes real time (about 0.6 s for 255 samples).
- **Not modelled:** noise (the standard deviations read zero), the Kalman filter and its
  `SignalProcessConfig`, tilt, and optical tracking loss. Status flags can be forced with `forceStatus`.
  Which values are PLACEHOLDER is in the class javadoc.

## Fake Limelight 3A (`fakehardware/FakeLimelight3A`)
A drop-in SDK 12.0 `Limelight3A`: robot code calls `hardwareMap.get(Limelight3A.class, "limelight")`,
`start()`, `pipelineSwitch(i)`, `updateRobotOrientation(yawDeg)` and `getLatestResult()` unchanged, and gets
`botpose`, `getBotpose_MT2()` and `FiducialResult`s from the tags the sim robot can see.

```java
AprilTagLayout layout = AprilTagLayout.builder()
        .addTag(1, 70.0, 30.0, 6.0, Math.PI, 4.0)   // id, x, y, z (in), facing (rad), size (in): YOUR numbers
        .build();
FakeLimelight3A limelight = FakeLimelight3A.builder(layout,
        new CameraMount(7.0, 0.0, 9.0, 0.0, Math.toRadians(-15.0)), 54.5, 42.0)   // mount, H/V FOV (deg)
        .rangeInches(6.0, 120.0)
        .build();
hardwareMap.register("limelight", limelight);
// every tick, before hardwareMap.updateAll(dt):
limelight.setRobotPose(fieldX, fieldY, fieldHeading);   // or setRobotPoseFromLocalizer(...)
```

- **No field geometry ships with SimLoop.** The tag layout is yours to supply, from numbers you are
  entitled to use. Do not derive one from the FIRST field CAD: its licence forbids redistribution.
- **The SDK parses the results.** The fake builds the JSON the device serves and hands it to the SDK's own
  `LLResult` constructor, so every getter is the SDK's code. Staleness, timestamps and `isConnected()` run
  on sim time (the sum of `update` ticks), never the wall clock. Before the first poll `getLatestResult()`
  is an empty, invalid result, as on the robot; `stop()` keeps the last result, which goes stale.
- **Geometry** is `vision/SimTagCamera`: all four corners must be in the image and in front of the lens,
  the tag must face the camera (optionally within `maxIncidenceDegrees`), be within range, and not be
  behind an `ObstacleBox` given to `occluders(...)` or `robotBody(...)` (robot frame). Tags may be tilted.
  MegaTag2 uses the yaw you sent.
- **Optional:** seeded Gaussian noise on tx/ty and botpose (`noise(seed, ...)`, off by default,
  byte-identical for a seed) with `stdev_mt1`/`stdev_mt2` reporting its spread, seeded dropouts
  (`dropout(seed, p)`), lens distortion on the pixel outputs (`distortion(k1, k2, p1, p2, k3)`), capture
  and targeting latency, and a camera frame rate.
- **Your test JVM needs the real `org.json`** (`testRuntimeOnly "org.json:json:20250517"`); Android's
  `android.jar` copy is a stub. `SimLoop/build.gradle` shows the line.
- **Not modelled:** non-AprilTag pipelines, blur, tag-on-tag occlusion, and the network-only calls (`uploadFieldmap`, snapshots, python,
  calibration), which throw. Robot-space poses use `+y` left; this has not been checked on a real 3A.

## Running a team's OpMode unchanged (`opmode/`)
`OpModeRunner` runs an FTC `OpMode` or `LinearOpMode` exactly as the team wrote it (it imports only
the SDK) against a `FakeHardwareMap`, one fixed tick at a time, with no wall clock. The worked example is
`src/test/java/org/horizon36596/simloop/example/OpModeRunnerExampleTest.java`.

```java
FakeHardwareMap hardwareMap = new FakeHardwareMap();
hardwareMap.register("left", new FakeMotor(2787, 10));
hardwareMap.register("right", new FakeMotor(2787, 10));

OpModeRunResult result = new OpModeRunner(new TankTeleOp(), hardwareMap)
        .initLoopTicks(10)                         // ticks between INIT and PLAY
        .activeTicks(150)                          // 3 s at the default 50 Hz
        .gamepads((phase, tick, gamepad1, gamepad2) -> gamepad1.left_stick_y = -1f)
        .eachTick(dt -> drivePlant.update(dt))     // optional: plants the fakes do not model
        .run();                                    // or .runLogged(robotConfig, Paths.get("build/sim/x.rlog"))
result.telemetry().lastTransmission();             // what the Driver Station would show
result.warnings();                                 // anything the run could not simulate faithfully
```

**A run** is one INIT tick, `initLoopTicks` INIT_LOOP ticks, `activeTicks` ACTIVE ticks and one STOP tick.
Each tick advances the `FakeTimer`, applies the gamepad script, runs the OpMode's code, then advances every
fake device and the `eachTick` step. The OpMode gets `hardwareMap`, `gamepad1`/`gamepad2` (delivered the
SDK's way, so `aWasPressed()` works), a `RecordingTelemetry`, `time` and `getRuntime()` on sim time, and
working `requestOpModeStop()` / `terminateOpModeNow()`. `runLogged` goes through `ScenarioRunner`, so the
RLOG has one frame per tick and the usual completeness checks.

- **Iterative `OpMode`**: the SDK's own order - `init()`, `init_loop()` per INIT_LOOP tick, `start()` and
  the first `loop()` on the first ACTIVE tick, `loop()` per tick, `stop()` - with `telemetry.update()`
  after each, as the SDK does. Each call runs on the OpMode's own thread while the sim waits (never both
  at once); breakpoints still work, and a debugger that suspends all threads pauses the watchdog too.
  Under `forTeamClass` a wait in `init()` holds it for ticks of sim time, as the SDK lets `init()` block;
  waits in the other calls are refused.
- **`LinearOpMode`**: `runOpMode()` runs on its own thread in strict lockstep with the sim (never both at
  once). A tick ends when the OpMode repeats a lifecycle check (`opModeIsActive()`, `isStopRequested()`,
  `opModeInInit()`, `isStarted()`) it already made this tick at the same place, so one pass of
  `while (opModeIsActive()) { ... }` is one tick. `waitForStart()` holds until PLAY. `sleep(ms)` holds for
  `ceil(ms / tick)` ticks of **sim** time and no real time. After STOP every check reports the stop.
- **Failures fail the test, never hang it**: an exception in the OpMode becomes an `OpModeException`
  (phase, tick, original cause). An iterative call (`init()` ... `stop()`) that does not return, or a
  `LinearOpMode` that runs without a lifecycle check or ignores STOP, for `stuckTimeoutMillis` (default
  10 s, counted, not clocked) fails with the phase, tick and its thread's stack. The stuck thread is left
  as a daemon that runs no more OpMode code (an iterative one is interrupted first, which ends a stuck
  `Thread.sleep`); every other OpMode thread is gone when the run returns.
- **`OpMode.blackboard`** is one static map per JVM. A run clears it before INIT, so tests cannot leak
  into each other; `.keepBlackboard(true)` keeps it, to hand an auto's data to the TeleOp run after it.
  It is not cleared after a run, so a test can read what the OpMode left.
- **Telemetry copied before INIT** (`Telemetry t = telemetry;`, or a dashboard
  `new MultipleTelemetry(telemetry, dashboard.getTelemetry())` as a field) is recorded: the SDK creates
  the OpMode's telemetry before any field initializer runs and offers no earlier hook, so the runner
  swaps every copy it finds afterwards - in the OpMode's fields, in the objects they hold up to four deep,
  and in arrays, lists and maps there - for the recording one, its `log()` included.

**Timer code, unchanged: `OpModeRunner.forTeamClass(...)`.** Name the OpMode's class instead of building
it, and the runner builds it inside a `SimClockLoader` that moves the team's own clock calls onto the sim
clock: `ElapsedTime` (all three constructors, and team subclasses), `System.nanoTime()`,
`System.currentTimeMillis()`, `Thread.sleep`, `TimeUnit.sleep`, Android's `SystemClock` reads and
`SystemClock.sleep`, and `sleep(ms)` with any duration, computed or not, and `idle()`. Only the team's
packages are rewritten (`org.firstinspires.ftc.teamcode.` and `com.seattlesolvers.solverslib.` unless
`teamPackages(...)` says otherwise, and never a prefix that would take in the JDK, the SDK, SimLoop, PsiKit or
JUnit). What it cannot move but can see - the SDK's `Deadline` timer, the real date and time, a clock-reading
library such as Pedro Pathing outside the team packages - is listed in `result.warnings()` with its place;
what it must refuse fails the run, whatever the team's code catches. The price is a second copy of the team's
classes each run: the OpMode cannot be cast to its own class, a team object cannot cross from the test or from
an earlier run, and statics start fresh. The guide, with every warning and error message it can give:
[`docs/unchanged-team-code.md`](docs/unchanged-team-code.md).

```java
OpModeRunResult result = OpModeRunner.forTeamClass("org.firstinspires.ftc.teamcode.MyAuto", hardwareMap)
        .activeTicks(1500)                         // a 30 s auto
        .run();
```

**What it cannot simulate, and what an outside team's OpMode may still hit:**
- Run as an instance (`new OpModeRunner(opMode, ...)`), `sleep(ms)` needs a **constant** duration
  (`sleep(500)`, `sleep(STATIC_FINAL)`). The SDK makes `sleep` `final` around `Thread.sleep`, so SimLoop
  reads the constant from the caller's bytecode; a computed duration (`sleep(ms)` inside a helper) fails
  loudly with the file and line. `forTeamClass` takes any duration.
- Run as an instance, `ElapsedTime`, `System.nanoTime()`/`currentTimeMillis()` and SolversLib's `Timing`
  and `WaitCommand` read the wall clock; in a sim that runs far faster than real time they are wrong and
  non-deterministic. A `Thread.sleep` on the OpMode's thread is not moved either, and usually ends at once
  (see the interrupt flag below). `forTeamClass` moves all of these onto the sim clock in the team's
  packages.
- Either way, clock reads inside the SDK (its `Deadline` timer included) and other libraries, the real date
  and time (`java.time`, `Date`, `Calendar`), `Object.wait`, `Thread.join`, latches, futures, executors and a
  method reference `this::sleep` are not moved. `forTeamClass` names the first three in `result.warnings()`,
  refuses `this::sleep` (write `ms -> sleep(ms)`) and a `Thread.sleep` on any thread but a `LinearOpMode`'s
  own, naming the call site, and fails the run on any refusal.
- `getRuntime()` is sim time plus the real time the current tick's code has taken (microseconds): exact
  enough for `getRuntime() > 2.0`, but telemetry that shows it differs slightly on every run; `time` is the
  exact field.
- Threads the OpMode starts itself are not stepped: they run in real time beside the sim. The runner
  notices any its code creates (they join the OpMode thread's `ThreadGroup`) and lists them in
  `result.warnings()`, or fails the run on the tick one appears with `.failOnOpModeThreads(true)`. It sees
  one that dies of an uncaught exception however briefly it lived, and misses a thread given its own
  `ThreadGroup`, one that starts and ends normally within one tick's code, and a pool thread other code
  created first. A `LinearOpMode`'s thread runs with its interrupt flag set (that is how the SDK's
  `Thread.sleep` returns at once), so a blocking call SimLoop does not move (`Object.wait`, `Thread.join`,
  `CountDownLatch.await`, `Future.get`, `BlockingQueue.take`, NIO channels, a library's `Thread.sleep`)
  usually throws `InterruptedException` at once, taking no sim or real time (`LockSupport.parkNanos` just
  returns). If the OpMode swallows that exception, the flag stays clear until its next lifecycle check, and
  a wait in that gap - an SDK `sleep()` on an instance run included - waits in real time and advances no sim
  time; `waitForStart()` and later `sleep()` calls are unaffected. An uncaught one fails the run with a
  message saying why.
- After STOP the sim clock no longer moves, so a loop still waiting on a timer, `sleep()` or `idle()` then
  never finishes: the run fails after `stuckTimeoutMillis`, once, and under `forTeamClass` the left-over
  thread is ended at its next clock read, `sleep()` or `idle()`. Put `opModeIsActive()` in such loops.
- Runs in one JVM share `OpMode.blackboard` (cleared at each run's start unless kept) and PsiKit's static
  `Logger`. Two `runLogged` runs started at once take turns (`ScenarioRunner` holds a JVM-wide lock), but
  tests that run OpModes must not run in parallel inside one JVM; Gradle's separate test JVMs are fine.
- Telemetry copied before INIT that the runner cannot swap - in a `Set` or an immutable list, a `static`
  field, a lambda, more than four objects deep - and items added before INIT are not recorded; each is
  in `result.warnings()`, found at INIT, the first time the OpMode sends through the SDK's object, or at
  the end of the run if it wrote to that object and never sent it.
- Hardware the fakes lack (e.g. webcams) fails in `init()` exactly where it would on a robot missing it. The
  IMU, the touch, distance and colour sensors, and `RUN_TO_POSITION` on `FakeMotor`
  ([docs/run-to-position.md](docs/run-to-position.md)) are faked; the SDK's own `SensorIMUOrthogonal`
  sample runs unchanged in `example/SdkImuSampleExampleTest.java`.

## Guides
- [Making your timers sim-safe](docs/sim-safe-timers.md) - why `ElapsedTime`, `LinearOpMode.sleep`,
  SolversLib's `WaitCommand` and `System.nanoTime()` read the wrong clock in a sim, and the small change
  that fixes each one, using `sim/SimElapsedTime`.
- [Running your timer code unchanged](docs/unchanged-team-code.md) - `OpModeRunner.forTeamClass(...)`,
  which moves a team's `ElapsedTime`, `System.nanoTime()`, `Thread.sleep` and computed `sleep(ms)` onto the
  sim clock without a code change: the build setup it needs, what it rewrites, what it does not (Pedro
  Pathing included), and every warning and error message it gives.

## The rules it holds itself to

- **No season imports.** No file here imports `org.firstinspires.ftc.teamcode.*` or names a season.
  Season facts arrive through the `config` interfaces, and `SeparabilityTest` enforces it. (The tests'
  own sim-clock fixtures live in `org.firstinspires.ftc.teamcode.simclockfixture` on purpose, because the
  sim-clock loader rewrites that package by default; they are SimLoop's test code, not anyone's TeamCode.)
- **Depends only on** the FTC SDK (its hardware interfaces, plus the concrete SDK drivers a fake extends,
  such as `GoBildaPinpointDriver`) and PsiKit core. No SolversLib, no follower library, no season code.
- **Deterministic.** No wall-clock reads on any sim path. `update(deltaTime)` mutates; getters are pure
  reads; iteration order is fixed. The one exception is `sim/live/RealTimePacer`, which can only make a
  live run wait (`WallClockConfinementTest`).
- **The fakes replicate real device semantics exactly** — every method of the interface, including the
  ones nobody calls, behaving the way the hardware does.

## Building and testing it

```bash
./gradlew :SimLoop:testDebugUnitTest
./gradlew :SimLoop:desktop:test      # the keyboard-window module
```

1,212 JVM tests, no device and no emulator. They live in `src/test/` and travel with the module.

Every command below is run from the repository root, not from this directory. The `SimLoop/` path prefix
and the `:SimLoop` task prefix are correct in every checkout of it: SimLoop is always a Gradle subproject
and never the root project, which is what the Android Gradle Plugin's library plugin expects and what
keeps the published coordinate stable.

Building the documentation site (one dependency, `pip install mkdocs-material`):

```bash
python -m mkdocs build -f SimLoop/mkdocs.yml --strict
python -m mkdocs serve -f SimLoop/mkdocs.yml
```

Generating the API reference, which the site links to and every release ships:

```bash
./gradlew :SimLoop:apiDocs
```

Releasing: `PUBLISHING.md`. Conventions for changing this subtree: `CLAUDE.md`.

## Licence

**BSD 3-Clause** — `LICENSE` in this directory is the licence text, `NOTICE.md` beside it is the
copyright and what it covers.

In practice, for an FTC team: use it, change it, ship it on your robot, and publish your robot code
under any licence you like. Keep the copyright notice with any copy of SimLoop's source or build you
hand on, and do not use Horizon's name to endorse your fork.

The dependencies keep their own licences, and there are three of them: the FIRST Tech Challenge SDK's
`RobotCore` and `Hardware`, which are BSD, and PsiKit, which is BSD 3-Clause. This licence covers
SimLoop's own code and nothing else.
