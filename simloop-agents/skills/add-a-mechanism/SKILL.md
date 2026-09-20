---
name: add-a-mechanism
description: Add a new mechanism (slide, arm, turret, intake, servo claw) to a SimLoop simulation - write its sim config, wire its plant and fake devices into the robot sim harness, and prove it moves. Use when a subsystem exists in robot code but cannot yet be simulated.
---

# Add a mechanism to the simulation

The subsystem already exists in robot code, or is being written alongside this. The job here is to give
it a **physical model** and the **fake devices** it reads and commands, so the unchanged subsystem can
run in a JVM test.

Nothing in the robot code changes. If you find yourself editing the subsystem to make the simulation
work, stop — that is the simulation being wrong, and it defeats the point, which is that the tested code
is the shipped code.

## 1. Read the subsystem first

Answer these four before writing anything. Get them from the source, not from the name:

- **What devices does it ask `hardwareMap` for, and under exactly what names?** The fake devices must be
  registered under the *same* strings, or the subsystem will not resolve them in simulation.
- **What does it command?** Motor power, a servo position, a velocity?
- **What does it read?** An encoder, a limit switch, an analog sensor?
- **What units is its position in?** Inches, degrees, ticks, revolutions? SimLoop converts nothing —
  whatever unit the config is written in is the unit every assertion is in.

## 2. Pick the right config interface

| The mechanism is | Use | Notes |
|---|---|---|
| One axis driven by a motor — slide, arm, turret, elevator | `MechanismSimConfig` | The common case. |
| One subsystem with several axes that move independently | `MultiDofMechanismPlant` | One plant per degree of freedom, ticked together and logged apart. It does no physics of its own and never lets one plant see another. |
| A positional servo — claw, wrist, hood | `PositionalServoSimConfig` | Position units are the servo's own command units unless you map them. |
| A continuous-rotation servo or a roller intake | `ContinuousRotationSimConfig` | Speed, not position. |

**Degrees of freedom do not couple.** A heavy slide loading the pivot that carries it is rigid-body
physics, and SimLoop rules it out on purpose — `MultiDofMechanismPlant` holds independent plants side by
side and nothing hands one a reference to another. If a mechanism genuinely seems to need coupling, stop
and say so rather than approximating it.

If it is none of these — anything needing contact, collision, or a piece being physically grabbed —
SimLoop does not model it. Read <https://libraries.horizon36596.org/simloop/limits/> and say so rather
than faking it. `field` models possession as *logic* (where a piece is and who holds it), which is often
what is actually wanted; it does not model intaking.

## 3. Write the config

`MechanismSimConfig` has exactly five methods. Every one needs a real number and a comment saying where
that number came from:

```java
private static final MechanismSimConfig SLIDE = new MechanismSimConfig() {
    /** Seconds for the carriage speed to follow a change in commanded power. */
    @Override public double timeConstant() { return 0.12; }
    /** Carriage speed at full power with nothing fighting it, in inches per second. */
    @Override public double maxSpeed() { return 30.0; }
    /** Bottom of travel, in inches. */
    @Override public double minPosition() { return 0.0; }
    /** Top of travel, in inches. */
    @Override public double maxPosition() { return 24.0; }
    /** Power the carriage's own weight costs, unitless, in [0, 1). */
    @Override public double gravityHoldPowerFraction() { return 0.08; }
};
```

**`gravityHoldPowerFraction` is always positive** and the plant subtracts it, so gravity always pulls
toward `minPosition`. For a horizontal mechanism — a turret, a flat slide — it is `0.0`.

### Be honest about where the numbers came from

A first pass is estimated, and that is fine and normal. Say so in the comment: an invented number
labelled as invented is useful, and an invented number presented as measured is how a team ends up
trusting a simulation that does not match their robot.

`maxPosition` and `minPosition` are usually known exactly from the CAD or a tape measure. `maxSpeed` and
`timeConstant` are the ones worth measuring later, by driving the real mechanism at full power and
timing it.

## 4. Wire it into the sim harness

The project has one class that owns the fake hardware and the plants — in the starter it is
`ExampleRobotSim`. Add the mechanism **there, once**, not in each test.

Four things to add:

1. The **fake device(s)**, registered under the same config names the subsystem asks for.
2. The **plant**, built from the config.
3. A line in **`advancePhysics(deltaTime)`** that steps the plant and then **pushes the plant's position
   into the encoder the subsystem reads**. This is the step people forget, and without it the subsystem
   sees only its own command and never the truth — every test passes and none of them mean anything.
4. Whatever the tests need to reach it.

The order inside one tick, which matters:

```
robot.periodic()        // robot code reads sensors, writes powers
  plant.update(dt)      // the world moves, reading those powers
  encoder <- plant      // the sensor learns what happened
  hardwareMap.updateAll // fakes derive what they derive from elapsed time, e.g. velocity
```

A test calls `robot.periodic()` then `advancePhysics(dt)`; steps two to four are inside `advancePhysics`.

## 5. Prove it moves before you prove anything else

Write one throwaway check that commands full power and asserts the position changed. If it does not
move, the wiring is wrong and no clever test will tell you that more clearly.

Then write the real test — see the `write-a-sim-test` skill — and make sure its assertions can fail.

## Finish

```
./gradlew :TeamCode:testDebugUnitTest
```

On Windows use `.\gradlew.bat :TeamCode:testDebugUnitTest` instead; the extensionless `gradlew` is the
Unix script.

Quote the actual result. Then tell the human:

- which numbers you **estimated** versus took from the robot, listed plainly, because those are the ones
  that decide whether the simulation is worth believing;
- where the `.rlog` is, so they can open it in AdvantageScope and see whether the motion looks like the
  real mechanism.
