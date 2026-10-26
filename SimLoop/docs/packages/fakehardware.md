# `fakehardware`

`org.horizon36596.simloop.fakehardware`

Fake devices that implement the **real** FTC SDK 12.0 interfaces. Your robot code resolves them from a
`FakeHardwareMap` by the same config name it uses on the robot, and cannot tell the difference. That is
why your code does not have to be modified to be tested.

## What you use

| Type | Stands in for | Notes |
|---|---|---|
| `FakeHardwareMap` | `HardwareMap` | Hand this to your `init()`. Register each fake under its config name. Files sensors into the SDK's typed maps the way the SDK's own configuration parser does. |
| `FakeMotor` | `DcMotorEx` | Modes, encoders, power, velocity, current, direction, and the hub's closed loops: `RUN_USING_ENCODER`, `setVelocity` and `RUN_TO_POSITION`. |
| `FakeExternalEncoder` | a separate encoder on a motor port | For a mechanism whose encoder is not on its own motor. |
| `FakeServo` | `Servo` | Positional. Honours `scaleRange` the way the SDK does. |
| `FakeCRServo` | `CRServo` | Continuous rotation. Starts stopped. |
| `FakeImu` | `IMU` | The universal IMU. Every `IMU` method runs through the SDK's own helper class. |
| `FakeGoBildaPinpoint` | `GoBildaPinpointDriver` | **Is** the SDK 12.0 driver, running on an emulated register file. |
| `FakeSparkFunOTOS` | `SparkFunOTOS` | **Is** the SDK 12.0 driver, running on an emulated register file. |
| `FakeLimelight3A` | `Limelight3A` | Results built from the AprilTags the sim robot can see, parsed by the SDK's own `LLResult`. |
| `FakeTouchSensor` | `TouchSensor` | A REV Touch Sensor. |
| `FakeDistanceSensor` | `DistanceSensor` | A REV 2m distance sensor. |
| `FakeRevColorSensorV3` | `RevColorSensorV3` | Also a `ColorRangeSensor`, `ColorSensor`, `NormalizedColorSensor` and `DistanceSensor`. Fed raw `ColorCounts`; the SDK's own driver does the arithmetic. |
| `ColorCounts` | — | Raw red, green, blue photodiode counts, `0..65535` each. |
| `FakeAnalogInput` | `AnalogInput` | A voltage you set. |
| `FakeDigitalChannel` | `DigitalChannel` | Input or output, with the SDK's mode rules. |
| `SwitchPolarity` | — | Puts a switch's raw line level on a `FakeDigitalChannel` (`PRESSED_IS_LOW` for a REV Touch Sensor). |
| `FakeVoltageSensor` | `VoltageSensor` | 12.0 V by default; `setVoltage` for brown-out tests. |
| `FakeI2cDeviceSynchSimple` | the I2C bus client inside a device driver | Bus config only. See below. |
| `FakeDevice` | — | The interface every fake implements: `update(deltaTime)`. |
| `AbstractFakeDevice` | — | Base class supplying the `HardwareDevice` boilerplate. |

## How it is used

```java
FakeMotor slideMotor = new FakeMotor(2000.0, 8.0);   // max ticks/s, accel constant (1/s)
FakeHardwareMap hardwareMap = new FakeHardwareMap();
hardwareMap.register("slide", slideMotor);           // the name in YOUR robot configuration

// ... your robot code does hardwareMap.get(DcMotorEx.class, "slide") and never knows ...

hardwareMap.updateAll(deltaTime);                    // once per tick, from the scenario body
```

## Units and frames

- **Motor position:** encoder **ticks**. Velocity: **ticks per second**.
- **Motor power:** unitless, `[-1, 1]`.
- **Servo position:** unitless, `[0, 1]`, before `scaleRange` is applied.
- **Voltage:** **volts**.
- **Odometry seam** (`FakeGoBildaPinpoint`, `FakeSparkFunOTOS`): **millimetres** and **radians**. A drive
  plant reports inches, so multiply by 25.4.
- **`FakeImu.setChassisHeading`:** **radians**, counter-clockwise, not wrapped.
- **`FakeLimelight3A.setRobotPose`:** field frame, **inches**, heading **radians** CCW from `+X`.
- **`update(deltaTime)`:** `deltaTime` in **seconds**, always the scenario's fixed tick.

## Motors in closed loop

Robot code that uses `RUN_TO_POSITION`, `RUN_USING_ENCODER` or `setVelocity` works unchanged. The SDK side
is exact; the hub firmware's loops are a labelled approximation. The one thing the fake needs from you is
the load, as a share of stall torque, every tick:

```java
arm.setExternalTorqueFraction(-0.3 * Math.cos(angle));
```

[RUN_TO_POSITION in the sim](../run-to-position.md) has the full table of what is exact and what is
approximate, and what to measure on a real hub.

## Odometry: Pinpoint and OTOS

Both fakes **are** the SDK 12.0 driver classes, so robot code gets them from `hardwareMap.get(...)` and
runs unchanged. Wire them in the sim harness, never in robot code:

```java
FakeGoBildaPinpoint pinpoint = new FakeGoBildaPinpoint();
pinpoint.setPhysicalPodMount(xOffsetMm, yOffsetMm, xDirection, yDirection, ticksPerMm); // how the pods REALLY sit
hardwareMap.register("pinpoint", pinpoint);
pinpoint.attachChassisAt(plant.getX() * 25.4, plant.getY() * 25.4, plant.getHeading());
// every tick, after the drive plant has integrated, then updateAll:
pinpoint.moveChassisTo(plant.getX() * 25.4, plant.getY() * 25.4, plant.getHeading());
```

```java
FakeSparkFunOTOS otos = new FakeSparkFunOTOS();
otos.setPhysicalMount(forwardMm, leftMm, headingRad);  // where the sensor REALLY sits
hardwareMap.register("otos", otos);
otos.attachChassisAt(plant.getX() * 25.4, plant.getY() * 25.4, plant.getHeading());
// every tick, after the drive plant has integrated, then updateAll:
otos.moveChassisTo(plant.getX() * 25.4, plant.getY() * 25.4, plant.getHeading());
```

**Physical against configured.** The `setPhysical...` call is the truth; what your robot code writes
(offsets, directions, resolution, scalars) is what it believes. If they disagree, the reading is wrong the
same way a mis-configured real sensor is wrong. So a robot that writes its real offsets needs its sim
wiring to call `setPhysical...` with the same values.

## IMU

```java
FakeImu imu = new FakeImu(new RevHubOrientationOnRobot(logo, usb)); // how the hub is REALLY mounted
hardwareMap.register("imu", imu);
// every tick, after the drive plant has integrated:
imu.setChassisHeading(drivePlant.getHeading()); // radians, CCW, not wrapped
```

The constructor says how the hub is really mounted; your `imu.initialize(...)` says what the robot code
believes. Yaw is zeroed once, at construction, as a real hub zeroes at power-on.

**Your test JVM needs `unitTests.returnDefaultValues = true`.** The SDK's orientation types call an
Android matrix method that otherwise throws "not mocked". See [Installing it](../getting-started.md).

## Limelight 3A

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

Robot code calls `start()`, `pipelineSwitch(i)`, `updateRobotOrientation(yawDeg)` and `getLatestResult()`
as usual and gets `botpose`, `getBotpose_MT2()` and fiducial results. Staleness and timestamps run on sim
time. Optional on the builder: seeded noise (and the matching `getStddevMt1/Mt2`), seeded dropouts,
latency, a frame rate, lens distortion (`distortion(k1, k2, p1, p2, k3)`, which moves corner and centre
pixels) and the robot's own body as boxes (`robotBody(...)`). The geometry is [`vision`](vision.md)'s
`SimTagCamera`, so tilted tags in the layout work too.

**Your test JVM needs the real `org.json`:** `testRuntimeOnly "org.json:json:20250517"`. Android's copy
is a stub.

## Touch, distance and colour sensors

```java
FakeTouchSensor touch = new FakeTouchSensor();          // hardwareMap.get(TouchSensor.class, "liftDown")
FakeDistanceSensor range = new FakeDistanceSensor();    // get(DistanceSensor.class, "frontRange"), a REV 2m
FakeRevColorSensorV3 color = new FakeRevColorSensorV3(); // RevColorSensorV3, ColorRangeSensor, ColorSensor,
hardwareMap.register("liftDown", touch);                // NormalizedColorSensor or DistanceSensor
hardwareMap.register("frontRange", range);
hardwareMap.register("intakeColor", color);
```

Three ways to drive them:

- **Scripted:** `touch.setPressed(true)`, `range.setDistance(DistanceUnit.INCH, 12)`,
  `color.setRawCounts(new ColorCounts(r, g, b))`. They take effect at once.
- **From a source:** `touch.setPressedSource(() -> tracker.getHeldCount() > 0)` and the other
  `set...Source` methods are asked once per `hardwareMap.updateAll(dt)`.
- **From the field:** [`sensor`](sensor.md) works out what a sensor on the robot sees or touches.

A REV 2m reports 8190 mm with nothing in range. A Color Sensor V3 never reads farther than 6 in. Colour
counts for a game piece must be measured on the real sensor.

## Two things worth knowing

**State changes only in `update`.** Every getter is a pure read. That is what makes a tick reproducible:
nothing moves because you looked at it.

**Sim-only accessors are marked as such.** Methods like `FakeMotor.getPhysicalPositionTicks()` and
`FakeCRServo.getPhysicalPower()` report the simulation's truth rather than what the device would report.
They are useful in a test and do not exist on real hardware, so never wire season code to one.

## What this package will not do

- **It will not move a mechanism.** A fake on its own only remembers what it was told. The motion is in
  [`plant`](plant.md). A `FakeMotor` in open loop with no plant behind it reports a position that never
  changes.
- **No generic I2C peripherals.** `FakeI2cDeviceSynchSimple` models the bus *configuration* a driver's
  constructor performs. Every read and write **throws**, on purpose: there is no peripheral behind the
  bus. The Pinpoint and OTOS fakes emulate their own register files instead.
- **No sensor noise or drift** on the IMU, Pinpoint, OTOS, distance or colour fakes. The Limelight has
  optional seeded noise and dropouts.
- **No webcams.** A `WebcamName` lookup fails in `init()` as on a robot without one.
- **No Limelight pipelines but AprilTags.** Network-only calls (field map upload, snapshots, Python,
  calibration) throw.
- **No legacy `BNO055IMU`.** Only the universal `IMU`.
- **No USB, no Lynx module, no firmware behaviour beyond the motor loops.** `isArmed()` is always false.

Every model's limits, and which of its numbers are PLACEHOLDER or NEEDS MEASUREMENT, are in each class's
javadoc.
