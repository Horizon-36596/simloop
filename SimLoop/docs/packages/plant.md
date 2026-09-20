# `plant`

`org.horizon36596.simloop.plant`

The physics behind the fakes. A plant reads what a fake device was commanded, advances its own state by
one tick, and that state is what the fake then reports back to your robot code.

## What you use

| Type | Models |
|---|---|
| `Mechanism1DofPlant` | One powered axis with end stops: an arm, a slide, a turret. Driven by motor power. |
| `MultiDofMechanismPlant` | Several such axes that move together, built with a builder. |
| `PositionalServoPlant` | A servo-driven joint, including the two ways a real servo refuses to go where it is told. |
| `MecanumDrivePlant` | A mecanum drivetrain's wheel speeds from the four motors. |
| `MecanumPoseIntegrator` | Those wheel speeds integrated into a field pose. |
| `OneDofPlant` | The interface the 1-DOF plants share. |

## The model, stated plainly

Each mechanism axis is **first-order**, with four parameters:

- a **time constant** (seconds) — a commanded step reaches ~63% of the way in one, and settles in four or
  five;
- a **max speed** (position units per second) — the effort ceiling, which saturates large steps so the
  response ramps instead of jumping;
- **end stops** (min and max position) — the state can never integrate past them, under any command;
- a **gravity hold fraction** (unitless, `[0, 1)`) — how much of full effort the mechanism's own weight
  costs it just to stay put. Always a **positive** number: the plant subtracts it, so gravity always pulls
  toward `minPosition`. A turret or a horizontal extension reports `0.0`.

That is the whole model. It is calibration-ready — every parameter is something you can measure on a real
mechanism — and it is deliberately not more than that.

## Units and frames

- **Mechanism position:** *your* units, whatever the config's `minPosition`/`maxPosition` are in. Inches
  for a slide, degrees or radians for an arm — SimLoop never converts, so pick one and stay in it.
- **Mechanism velocity:** those same units per **second**.
- **Drivetrain pose:** **inches** and **radians**, **localizer frame** — **+x forward** at heading 0,
  **+y left**, heading **counter-clockwise-positive**, measured from +x.

!!! danger "This is not the frame the `field` package uses — rotate before you cross"
    `MecanumDrivePlant` and `MecanumPoseIntegrator` integrate into the **localizer frame**, +x forward /
    +y left. That is the frame a dead-wheel odometry device reports, which is why the plant works in it.
    The [`field`](field.md) package's game pieces and trigger volumes use the **field frame**, +X right /
    +Y forward. Same handedness, rotated 90 degrees, so a pose handed straight across is silently wrong
    rather than obviously wrong.

    Both frames are deliberate. The rotation between them is
    [`FieldFrameTransform`](field.md#converting-between-the-two-frames) in the `field` package:

    ```java
    double fieldX = FieldFrameTransform.fieldXFromLocalizer(drive.getX(), drive.getY());
    double fieldY = FieldFrameTransform.fieldYFromLocalizer(drive.getX(), drive.getY());
    double fieldHeading = FieldFrameTransform.fieldHeadingFromLocalizer(drive.getHeading());
    ```

    Nothing calls it for you. Forgetting it is the single easiest mistake to make against this library,
    so it has its own test in the module: `DrivetrainPoseMeetsFieldFrameTest`.
- **Wheel order** everywhere: `[frontLeft, frontRight, backLeft, backRight]`.

## Order inside a tick

```java
double error = target - plant.getPosition();   // 1. read
motor.setPower(clamp(error * kP));             // 2. decide and command
plant.update(deltaTime);                       // 3. advance
```

Advancing before commanding shifts every response one tick late. It looks like a tuning problem and is
not.

## What this package will not do

- **No rigid-body dynamics, no contact, no collisions.** Nothing here knows the mechanism has mass
  distribution, or that two things could touch.
- **No motor curve, no back-EMF, no current limiting as physics.** `FakeMotor` reports a plausible current
  when a plant tells it how hard it is working; that is a reported number, not a model of the electrical
  system.
- **No belt stretch, no backlash, no friction beyond the gravity term.** A real slide that sticks at the
  bottom will not stick here.
- **No inter-mechanism coupling.** `MultiDofMechanismPlant` runs several axes; it does not model one
  loading another.
- **It will not tell you your constants are wrong.** A guessed time constant simulates just as
  confidently as a measured one.
