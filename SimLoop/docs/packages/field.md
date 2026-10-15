# `field`

`org.horizon36596.simloop.field`

Game pieces, the field they move on, and the robot they meet. A piece is a sphere with a position, a
velocity and a possession state. Loose pieces roll, get pushed and bounce; launched pieces fly a gravity
arc into a goal or onto the floor. Picking a piece up is still a box test, not a roller simulation.

## What you use

| Type | What it is |
|---|---|
| `GamePieceTracker` | Owns every piece. Acquires, launches, ejects and scores them, steps their physics, logs every transition. |
| `GamePiece` | One piece: an id, a kind, a position, a velocity, a radius, a possession state. |
| `PossessionState` | `LOOSE`, `HELD`, `IN_FLIGHT`, `SCORED`. |
| `TriggerVolume` | A rectangle fixed to the robot: "is a piece inside my intake's box right now". |
| `FieldWorld` | Everything fixed a piece can touch: perimeter, obstacles, goals, rigid bodies, the robot's footprint and height, and the physics numbers. |
| `GamePiecePhysicsConfig` | Gravity, restitution per kind of contact, rolling deceleration, drag (off by default), sub-step length, piece mass per kind. |
| `ObstacleBox` | A fixed, solid, axis-aligned box with a height range. |
| `ScoringVolume` | A goal mouth an in-flight piece scores by entering. Optionally descending-only, or open on one face only. |
| `KinematicBody` | A rigid structure built from `ConvexShape`s, which may turn about one axis at an angle you set each tick. |
| `HingeDynamics` | Makes a `KinematicBody` dynamic: it turns by itself under gravity and the pieces' weight and impacts, between hard limits. |
| `BodyCell` | A compartment of a body. A piece is in it while the physics leaves it there. |
| `ConvexShape` | A convex polyhedron, given by its corners and triangles. |
| `RobotShape`, `OrientedBox` | The robot as a few solid boxes, posed for its mechanisms, instead of one footprint rectangle. |
| `StaticRobot` | A robot that stands still, such as a parked partner, made solid. |
| `FootprintContact`, `RobotContact` | Whether the robot overlaps a structure, and the smallest move that takes it back out. |
| `SeededRandom` | A seeded random source for things that are random on a real field and must replay identically. |
| `FieldFrameTransform` | The rotation between the drivetrain's frame and this one. Static, pure, both directions. |

## The model

A piece is `LOOSE` on the floor, `HELD` while carried, `IN_FLIGHT` after a launch, and `SCORED` once it
is in a goal or out of play. Acquisition happens when a loose piece overlaps a trigger volume while the
mechanism runs and the robot has room. Pieces are scanned in id order, so a partly-full robot fills up
the same way on every replay.

**Every state change is logged, by construction.** Every transition in the tracker goes through one
private method that queues its log line, so "every transition is logged" is true because of the code
shape, not because somebody remembered.

## One tick

```java
tracker.updateRobotPose(fieldX, fieldY, fieldHeading);  // top: carry held pieces along
tracker.tryAcquire(intakeVolume, intake.isRunning());   // whenever a mechanism could pick up
// ... launch(...) / eject(...) as the robot releases pieces ...
tracker.advance(deltaTime);                             // physics for everything not held
tracker.record("GamePieces");                           // bottom: flush the log
```

`record` writes counts, the transition list, and 3-D positions per state and per kind, in **metres**, so
AdvantageScope's 3D Field can draw each kind as its own game-piece model.

## A shot into a goal

From `src/test/java/org/horizon36596/simloop/field/GamePiecePhysicsTest.java`: a goal 50 to 70 in up,
entered only on the way down, and a piece launched at 260 in/s and 60 degrees from 16 in up.

```java
ScoringVolume goal = ScoringVolume.builder("goal")
        .box(90.0, 130.0, -10.0, 10.0, 50.0, 70.0)
        .descendingOnly(true)
        .restPoint(93.0, 0.0, 60.0).restSpacing(3.0, 0.0, 0.0)
        .build();
FieldWorld world = FieldWorld.builder().scoringVolume(goal).build();

GamePieceTracker tracker = new GamePieceTracker(4, world);
tracker.updateRobotPose(0.0, 0.0, 0.0);
tracker.addHeldPiece("ball", 1.4);

double angle = Math.toRadians(60.0);
tracker.launch(0, "launcher", 0.0, 0.0, 16.0, 260.0 * Math.cos(angle), 0.0, 260.0 * Math.sin(angle));
for (int i = 0; i < 100; i++) {
    tracker.advance(0.02);                              // 2 s
}
// tracker.getPiece(0).getState() is now SCORED, parked at the goal's first rest slot
```

You say where a launched piece leaves the robot and how fast, because that depends on your launcher.
`ShotArc` in [`viz`](viz.md) draws the intended arc beside the flown one.

## The physics, in brief

- **Sub-steps** of at most 5 ms by default, whatever your tick. Semi-implicit Euler.
- **Rolling** is a constant deceleration, snapped to rest below a small speed.
- **Flight** is gravity, plus quadratic drag if you turn it on. No spin, no lift.
- **Contact** pushes a piece out along the contact normal and reverses and scales its velocity by that
  contact's restitution. Walls, obstacles and the robot are immovable to a piece.
- **Goals are swept.** A fast piece cannot skip a thin goal between two sub-steps.
- **The 3-D model**, in a world with bodies. Pieces in flight, or resting in a body's cell, collide with
  the bodies' shapes and with each other as spheres, with friction that makes them roll. A turning body
  carries pieces along and can tip them out. A cell holds whatever the physics put in it.
- **A robot made of parts.** With a `RobotShape`, a loose piece can roll under a part whose underside is
  above it (an intake mouth), and a flying piece meets every face of every part.

Every number is in `GamePiecePhysicsConfig`. The defaults are plausible for a foam ball a few inches
across. Measure your own and pass them in with `FieldWorld.Builder.physics(...)`.

## Stopping the robot at a structure

The drive plant knows nothing about the field. After it moves, ask `FootprintContact` (one rectangle), or
find `RobotContact`s for a `RobotShape` against the perimeter, the obstacles and any `StaticRobot`, and
move the robot back out. You decide whether to push the robot or only record the contact; a replay of a
real robot's poses should be drawn as it happened.

## Units and frames

- Piece positions: **inches**, field frame, FTC-Cartesian: **+X** right, **+Y** forward, origin at the
  centre of the field, **+Z** up from the floor. **This is not the frame the [`plant`](plant.md) package
  integrates a drivetrain pose into**, which is the localizer frame, +x forward / +y left. Rotate with
  `FieldFrameTransform` below.
- Piece radius: **inches**, half its diameter. Piece height: **inches**, of the piece's centre above the
  floor.
- Velocities: **inches per second**, field frame, `+Z` up. Gravity: **inches per second squared**.
- Robot heading: **radians**, counter-clockwise-positive; the robot's forward direction is
  `(cos, sin)` of it.
- `TriggerVolume` offsets: **inches**, **robot** frame, forward-positive and left-positive, so a box in
  front of the robot has a positive `forwardOffsetInches`.
- Body angles: **radians**, right-handed about the body's axis. Mass: any unit, as long as a hinged body
  and the pieces use the same one.
- Logged positions: **metres**, converted at the log and nowhere else.

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
  intaking. Rollers, compliance and an intake that misses are not modelled.
- **The robot is not rigid-body.** It is immovable to the pieces, a ball never pushes it back, and its
  mechanisms are first-order plants. A robot spinning in place does not fling a ball with its corner.
- **Obstacle contact is not swept.** A small, fast piece can pass through a thin obstacle: at the
  default 5 ms sub-step and 260 in/s, a 1 in ball can pass through an obstacle thinner than 1.6 in. A
  2.8 in ball cannot pass through anything. Goals are swept and do not have this limit.
- **A piece squeezed between the robot and a wall** can be left inside the robot's footprint for a
  sub-step. The next sub-step pushes it again.
- **`ObstacleBox` has no top.** A piece dropping onto one is pushed out sideways. Use a `KinematicBody` for
  a surface pieces should rest on.
- **A flying piece passes through pieces lying on the floor**, a low shot included. Loose pieces on the floor
  do not meet kinematic bodies either.
- **No scoring rules.** `SCORED` means in a goal or out of play; what it is worth is your season's
  business.
- **No field ships with it.** You build the `FieldWorld` from your own measurements.
