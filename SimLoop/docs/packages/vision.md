# `vision`

`org.horizon36596.simloop.vision`

A simulated camera on the robot. It works out which game pieces and AprilTags fall inside the image and
where, as numbers. **No image is rendered.** It is for testing the geometry half of a vision pipeline:
which pixel is which point on the floor, tracking, and pose from tags.

## What you use

| Type | What it is |
|---|---|
| `CameraLens` | The intrinsics: focal lengths, principal point and image size, in pixels. An OpenCV camera matrix. |
| `CameraMount` | The extrinsics: where the lens sits on the robot and which way it points. |
| `SimCamera` | Sees game pieces. Returns `CameraSighting`s. |
| `CameraSighting` | One piece in the image: centre pixel, apparent radius, range, kind, clipped or not. |
| `AprilTagLayout` | Where every tag on your field is. **You supply it.** |
| `SimTagCamera` | Sees AprilTags. Returns `TagSighting`s. |
| `TagSighting` | One tag: `tx`/`ty` in degrees, centre and corner pixels, range, area, camera-to-tag and robot-to-tag transforms. |
| `RigidTransform3`, `RobotFramePoint` | A 3-D rotation plus translation, and a point on the floor in the robot frame. |

`FakeLimelight3A` in [`fakehardware`](fakehardware.md) is built on `SimTagCamera`, so a Limelight and a
plain camera share one description.

## How it is used

From `src/test/java/org/horizon36596/simloop/vision/SimCameraTest.java`: a 70 degree, 640 x 400 camera,
6 in forward, 10 in up, tilted 22 degrees down.

```java
CameraLens lens = CameraLens.fromHorizontalFov(70.0, 640, 400);
CameraMount front = CameraMount.front(6.0, 10.0, 22.0);
SimCamera camera = new SimCamera("front", lens, front);

GamePieceTracker tracker = new GamePieceTracker(4, FieldWorld.unbounded());
tracker.updateRobotPose(0.0, 0.0, 0.0);
tracker.addLoosePiece("piece", 30.0, 0.0, 1.4);

// each tick, after the pieces and the robot have moved:
List<CameraSighting> seen = camera.observe(tracker.getPieces(), 0.0, 0.0, 0.0);
camera.record("Vision", tracker.getPieces());   // logs the camera pose and the sightings
```

To check a pipeline's floor projection, put a sighting's centre pixel back on the floor and compare with
the piece's true position:

```java
CameraSighting s = seen.get(0);
RobotFramePoint p = camera.groundPointInRobotFrame(s.getCentreXPx(), s.getCentreYPx(), 1.4);  // ball radius, in
```

For tags, build a layout from numbers you are entitled to use, then a `SimTagCamera` with a range window,
a maximum viewing angle and any boxes that block the view:

```java
AprilTagLayout layout = AprilTagLayout.builder()
        .addTag(1, 70.0, 30.0, 6.0, Math.PI, 4.0)   // id, x, y, z (in), facing (rad), size (in): YOUR numbers
        .build();
SimTagCamera tags = new SimTagCamera(lens, front, 0.0, Double.POSITIVE_INFINITY, 90.0,
        Collections.<ObstacleBox>emptyList());
List<TagSighting> visible = tags.observe(layout, robotX, robotY, robotHeading);
```

## Units and frames

- Robot pose, piece positions, tag positions: field frame, FTC-Cartesian, centre origin, **inches**,
  heading **radians** CCW from `+X`.
- `CameraMount`: **inches** in the robot frame (`+forward`, `+left`, `+up`, origin on the floor under the
  robot centre). Yaw **radians** CCW from forward. Pitch **radians**, **positive tilts down**.
  `CameraMount.front(...)` takes its pitch in **degrees**.
- Pixels: `(0, 0)` top-left, `+x` right, `+y` down (the OpenCV convention).
- Optical frame: `+x` image right, `+y` image down, `+z` along the boresight.
- `SimCamera.fieldPose3d(...)`: **metres**, WPILib camera axes, so AdvantageScope's 3D Field can use it
  as a camera override and look through the camera.
- Tag size: **inches**, the edge of the square a detector reports corners for. `TagSighting` angles are **degrees** where the name says.

## Visibility rules

A piece is sighted when it is on the field (not held), more than its own radius in front of the lens, and
its apparent disc overlaps the image. It is **clipped** when part of the disc is outside the image. A
piece whose centre pixel is covered by a nearer piece's disc is not reported.

A tag is seen only if, in this order: it is within range; it faces the camera within
`maxIncidenceDegrees`; all four corners are in front of the lens; all four corners are inside the image;
and no `ObstacleBox` you passed blocks the line from the lens to any corner. `visibility(...)` tells you
which rule failed.

## What this package will not do

- **It will not render an image.** And synthetic frames must never be used to tune colour thresholds.
- **No lens distortion.** An ideal pinhole.
- **No roll.** The image's horizontal stays level with the floor.
- **Most things do not block the view.** The robot's own body, field structures (unless given to
  `SimTagCamera` as boxes), the far wall and other tags are transparent.
- **Tags hang vertical.** A tilted or flat tag is not modelled.
- **It ships no field.** Not this season's, not any season's. Do not build a layout from the FIRST field
  CAD: its licence forbids redistribution.
- **No noise in `SimCamera` or `SimTagCamera`.** `FakeLimelight3A` can add seeded noise on top.
