# `field`

`org.horizon36596.simloop.field`

Game pieces and possession, modelled the only way a simulator without contact physics honestly can:
positions, states, and boxes relative to the robot.

## What you use

| Type | What it is |
|---|---|
| `GamePiece` | One piece: an id, a field position, a radius, a possession state. |
| `PossessionState` | `LOOSE`, `HELD`, `SCORED`. |
| `GamePieceTracker` | Owns every piece, enforces the held capacity, logs every transition. |
| `TriggerVolume` | A rectangle fixed to the robot — "is a piece inside my intake's box right now". |
| `FieldFrameTransform` | The rotation between the drivetrain's frame and this one. Static, pure, both directions. |

## The model

A piece is `LOOSE` until the robot acquires it, `HELD` while carried, `SCORED` once it is out of play.
Acquisition happens when a piece overlaps a trigger volume and the robot is not already at capacity.
Pieces are scanned in id order, so a partly-full robot fills up reproducibly on a replay.

**Every state change is logged, by construction** — every transition in the tracker goes through one
private method that queues its log line, so "every transition is logged" is true because of the code
shape, not because somebody remembered.

## Units and frames

- Piece positions: **inches**, field frame, FTC-Cartesian — **+X** right, **+Y** forward, origin at the
  centre of the field. **This is not the frame the [`plant`](plant.md) package integrates a drivetrain
  pose into**, which is the localizer frame, +x forward / +y left. Rotate with `FieldFrameTransform`
  below.
- Piece radius: **inches**, half its diameter.
- Robot heading: **radians**, counter-clockwise-positive; the robot's forward direction is
  `(cos, sin)` of it.
- `TriggerVolume` offsets: **inches**, **robot** frame — forward-positive and left-positive, so a box in
  front of the robot has a positive `forwardOffsetInches`.

## Converting between the two frames

A drivetrain pose and a game piece are in **different frames, ninety degrees apart**. Both are
deliberate: [`plant`](plant.md) integrates in the localizer frame because that is what a real odometry
device reports, and this package uses the field frame because that is what a person reads off a field
drawing. `FieldFrameTransform` is the only place that rotation is written.

```java
// A pose out of the drivetrain, asked a field question.
double fieldX = FieldFrameTransform.fieldXFromLocalizer(drive.getX(), drive.getY());
double fieldY = FieldFrameTransform.fieldYFromLocalizer(drive.getX(), drive.getY());
double fieldHeading = FieldFrameTransform.fieldHeadingFromLocalizer(drive.getHeading());

tracker.updateRobotPose(fieldX, fieldY, fieldHeading);
tracker.tryAcquire(intakeMouth, intake.isRunning());
```

And the other way, for a starting pose a human wrote down:

```java
integrator.setPose(
        FieldFrameTransform.localizerXFromField(startX, startY),
        FieldFrameTransform.localizerYFromField(startX, startY),
        FieldFrameTransform.localizerHeadingFromField(startHeading));
```

| Going | Position | Heading |
|---|---|---|
| localizer → field | `fieldX = -localizerY`, `fieldY = +localizerX` | `+ π/2` |
| field → localizer | `localizerX = +fieldY`, `localizerY = -fieldX` | `− π/2` |

Three things worth knowing:

- **Nothing calls it for you.** Handing a localizer pose straight to `overlapsPiece` or
  `updateRobotPose` does not throw — it returns a confident, wrong answer, and the failure looks like a
  mis-measured intake mouth.
- **There is no translation**, only rotation. Both frames share the field's centre origin.
- **Heading *rates* are not rotated.** The frames differ by a constant offset, so an angular velocity is
  the same number in both. Linear velocities *are* rotated — pass them through the position methods.

## Live views, not snapshots

`getPieces()` hands back an unmodifiable list, but the pieces in it keep changing as the tracker updates
them. A scenario asking "where is piece 3 now" wants *now*. Read what you need in the same tick you got
it; do not cache a position across ticks and assume it is still true.

## What this package will not do

- **It will not decide whether an intake works.** Overlap with a box is a model of intake *logic*, not of
  intaking. A real intake that misses is not modelled.
- **No collisions between pieces, no stacking, no physics of any kind.** A piece has a position and a
  radius; it does not move unless something sets it.
- **No scoring rules.** `SCORED` means "out of play"; what it is worth is your season's business.
- **No field geometry.** Nothing here knows where a wall or a goal is.
