# `viz`

`org.horizon36596.simloop.viz`

Drawing what the robot is doing into the log, so AdvantageScope can show you the arm, the turret and the
shot instead of line graphs you have to hold in your head.

## What you use

| Type | What it is |
|---|---|
| `MechanismSketchDefinition` | A planar mechanism's fixed geometry, built once with a builder. |
| `MechanismSketch` | The live drawing: give it joint values each tick, it logs a `Mechanism2d`. |
| `JointDefinition` | The base for one drawn segment. |
| `RevoluteJointDefinition` | A segment that pivots, such as an arm. Constant length, varying angle. |
| `LinearJointDefinition` | A segment that extends, such as a slide stage. Varying length, fixed direction. |
| `TurretSketchDefinition`, `TurretSketch` | A turret that yaws, with a planar chain riding on it, drawn in the 3D Field tab. |
| `ArticulatedRobotModel` | The robot's moving parts as a joint tree, turned into the `Pose3d[]` a custom AdvantageScope robot model draws. |
| `ShotArc` | One shot's path as a 3-D polyline: a drag-free or dragged ballistic arc, or sampled points. |
| `ShotArcRecorder` | Logs the last few shots as intended-against-actual arc pairs, with each shot's miss distance. |
| `RobotVector3` | A direction or point in the robot frame. |

## Mechanisms: `MechanismSketch`

A planar chain of revolute and linear joints, logged as a `Mechanism2d`. A yaw joint cannot be drawn in
one fixed plane, so a turret is a `TurretSketch`: a ghost robot pose turned by the yaw, the chain drawn in
the turret's own plane on that ghost, and a cone at the chain's tip pointing along the shot.

## Robot parts: `ArticulatedRobotModel`

From `src/test/java/org/horizon36596/simloop/viz/ArticulatedRobotModelTest.java`: a turret on the robot,
a hood on the turret, and a camera riding on the turret.

```java
ArticulatedRobotModel model = ArticulatedRobotModel.builder()
        .revolute("turret", ArticulatedRobotModel.ROBOT, new RobotVector3(0, 0, 10), new RobotVector3(0, 0, 1))
        .revolute("hood", 0, new RobotVector3(4, 0, 12), new RobotVector3(0, 1, 0))
        .fixed("camera", 0, new RobotVector3(6, 0, 11))
        .build();

Pose3d[] poses = model.poses(turretRad, hoodRad, 0);   // one value per component, in builder order
Logger.recordOutput("Robot/components", poses);
```

The parent is `ArticulatedRobotModel.ROBOT` or the index of an earlier component. Each component's mesh
in the AdvantageScope robot asset is authored relative to its joint origin, with the robot frame's axes
and every joint at zero. Then the logged poses draw each part where the CAD put it at zero, and move it
the way its joint says.

## Shots: `ShotArc` and `ShotArcRecorder`

From `src/test/java/org/horizon36596/simloop/viz/ShotArcDemoTest.java`, inside a scenario body:

```java
double speed = ShotArc.speedToReach(horizontal, AIM_Z - RELEASE_HEIGHT, PITCH);
ShotArc.Launch commanded = new ShotArc.Launch(rx, ROBOT_Y, RELEASE_HEIGHT, speed, bearing, PITCH);
ShotArc intended = ShotArc.ballistic(commanded, 0.0, AIM_Z, DT);
shots.fire(intended, AIM_X, AIM_Y, AIM_Z);            // shots is a ShotArcRecorder(3)
// ... each tick, as the real shot flies:
shots.setActual(soFar);                               // the flown arc so far, from the sim or a log
shots.record("Shots");
```

Each shot gets its own key in a fixed ring of slots, because AdvantageScope joins one array into one line.
`<prefix>/missInches/<slot>` is the closest approach of the actual arc to the aim point.

## Units and frames

- Lengths: **inches**. Logged poses and translations: **metres**, converted at the log and nowhere else.
- Angles: **radians**. A revolute joint turns right-handed about its axis, measured from the zero pose.
- `ShotArc.Launch`: release point in the field frame, **inches**; speed **inches per second**; yaw
  **radians** CCW from field `+X`; pitch **radians** above horizontal.
- `ShotArc` points: field frame, FTC-Cartesian, **inches**, `+Z` up from the tile top.
- Robot-frame directions and points (`RobotVector3`): `+X` forward, `+Y` left, `+Z` up, origin on the
  floor under the drivetrain centre.
- Line weight: **pixels**, and colour is `"#RRGGBB"` upper-case.

The **zero pose** is the mechanism as drawn in CAD: every joint at its own zero. A revolute joint's
angles are relative to that pose, and its min angle is always `<= 0` while its max is always `>= 0`.

## What this package will not do

- **It will not render anything.** It writes numbers into the log; AdvantageScope draws them. There is no
  image output here.
- **It will not do inverse kinematics.** You give it joint values; it does not solve for them.
- **It is not a collision model.** Two segments drawn through each other are drawn through each other.
  Solid robot parts for the physics are [`field`](field.md)'s `RobotShape`.
- **`ShotArc.ballistic` has no spin, lift or collisions.** For a shot in a sim with game-piece physics,
  draw the flown piece's sampled positions with `ShotArc.ofPoints(...)`.
- **A turret is clamped to its yaw limits.** Wrap a continuous turret's angle into its declared range
  before drawing it.
- **A `MechanismSketch` does not know where the robot is.** It is drawn in the robot frame; putting the
  robot on the field is the drivetrain's business.
