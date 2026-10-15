# `sensor`

`org.horizon36596.simloop.sensor`

Works out what a sensor bolted to the robot sees or touches on the simulated field, and drives the
matching fake device with it. The fakes in [`fakehardware`](fakehardware.md) stay simple values; this
package decides what the value is.

## What you use

| Type | Drives | What it decides |
|---|---|---|
| `SensorMount` | — | Where a sensor sits on the robot and which way it looks. |
| `RangeFinder` | `FakeDistanceSensor`, or a `FakeRevColorSensorV3`'s distance | The first thing a straight ray hits: wall, obstacle, loose piece or floor. |
| `RangeHit` | — | What a `RangeFinder` hit (`WALL`, `OBSTACLE`, `PIECE`, `FLOOR`, `NOTHING`) and how far away. |
| `ColorProbe` | `FakeRevColorSensorV3` | Which loose piece is in front of the sensor, and so which colour counts it reads. |
| `ContactSwitch` | `FakeTouchSensor`, or a `FakeDigitalChannel` with a `SwitchPolarity` | Whether a small actuator on the robot touches a wall, an obstacle or a loose piece. |

## How it is used

Each class is built once and applied every tick, after the plants have moved and you know the robot's
field pose. These lines are from the class javadocs and the tests in `src/test/.../sensor/`.

```java
// A level distance sensor 8 in ahead of centre, 3 in up, looking forward.
RangeFinder frontRange = new RangeFinder(SensorMount.level(8.0, 0.0, 3.0, 0.0));

// A bumper switch whose lever sticks 0.25 in out of an 18 in robot's front face.
ContactSwitch bumper = new ContactSwitch("bumper", 9.125, 0.0, 0.25, 4.0, 2.0);

// each tick:
frontRange.apply(frontDistanceSensor, tracker.getWorld(), tracker.getPieces(), x, y, heading);
bumper.apply(bumperTouchSensor, tracker.getWorld(), tracker.getPieces(), x, y, heading);
```

A colour sensor looking at an intake slot:

```java
Map<String, ColorCounts> colours = new LinkedHashMap<>();
colours.put("red", new ColorCounts(1800, 700, 600));     // MEASURE these on your robot
colours.put("blue", new ColorCounts(400, 900, 2100));
ColorProbe slot = new ColorProbe(new SensorMount(6.0, 0.0, 4.0, 0.0, Math.PI / 2), 2.0,
        Math.toRadians(30), colours, ColorCounts.DARK);
// each tick:
slot.apply(slotColorSensor, tracker.getPieces(), x, y, heading);
```

`x`, `y` and `heading` are the robot's **field-frame** pose. A drive plant reports the localizer frame;
rotate it first with `FieldFrameTransform` (see [`field`](field.md)).

## Units and frames

- `SensorMount`: **inches** in the robot frame, `+forward`, `+left`, `+up`, origin on the floor under the
  robot centre. Yaw in **radians**, counter-clockwise from forward. Pitch in **radians**, **positive tilts
  the sensor down**; `π/2` looks straight down.
- Robot pose: field frame, FTC-Cartesian, centre origin, **inches**, heading **radians** CCW from `+X`.
- `RangeHit.getDistanceInches()`: **inches**, from the sensor face, not the robot centre.
- `ColorProbe` range: **inches** from the sensor face to the piece's surface. Field of view is a
  **half-angle in radians**.
- `ContactSwitch` offsets and sizes: **inches**, robot frame. Depth is along forward, width along left,
  height above the floor.
- `ColorCounts`: raw photodiode counts, `0..65535` per channel.

## Things worth knowing

- **A held piece is invisible here.** The tracker parks held pieces at the robot's centre. Drive a sensor
  that looks inside the robot from the tracker instead: `ColorProbe.show(...)` for colour, or
  `touch.setPressedSource(() -> tracker.getHeldCount() > 0)` for a switch.
- **Make a switch's actuator stick out by its travel.** A drive plant stops a robot flush with a wall. A
  lever flush with the bumper would then be exactly touching, which counts as pressed but is a knife-edge.
- **Real-device readings are the fakes'.** A REV 2m distance sensor reads 8190 mm with nothing in range
  (`FakeDistanceSensor.REV_2M_NO_TARGET_MM`); a Color Sensor V3 never reports farther than 6 in.
- **Deterministic.** Ties between equally near pieces go to the lowest id; all trigonometry is
  `StrictMath`.

## What this package will not do

- **A range finder is a ray, not a cone.** A real time-of-flight sensor sees about 25 degrees and misses
  dark or steeply angled surfaces. This sees one line and every surface.
- **The robot's own body does not block anything.**
- **The wall has no height.** A ray tilted up is still stopped by the perimeter.
- **Colour does not change with distance or angle.** One set of counts per kind of piece. Measure them at
  the distance the piece really sits from your sensor.
- **No sensor noise.**
