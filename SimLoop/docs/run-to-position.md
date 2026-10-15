# RUN_TO_POSITION and RUN_USING_ENCODER in `FakeMotor`

`FakeMotor` runs the two closed-loop modes the way a REV Control Hub or Expansion Hub does: the SDK side
exactly as the FTC SDK 12.0.0 source does it, and the firmware side as a labelled approximation. This page
says which is which, and what to measure on a real hub to check the approximations.

## Using it

Nothing special. Robot code that works on the robot works here:

```java
arm.setTargetPosition(178);                       // target first...
arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);     // ...then the mode...
arm.setPower(0.8);                                // ...then the speed cap
while (arm.isBusy()) { /* ... */ }
```

The one thing the sim needs from you is the **load**. A `FakeMotor` knows nothing about the arm or slide on
it, so gravity has to be told, every tick, as a share of the motor's stall torque in the physical encoder
direction:

```java
// Zero ticks = arm horizontal, positive lifts it. Holding it level takes 30% of stall torque.
double angle = arm.getPhysicalPositionTicks() / TICKS_PER_RADIAN;
arm.setExternalTorqueFraction(-0.3 * Math.cos(angle));
arm.update(deltaTime);
```

A vertical slide is a constant, for example `setExternalTorqueFraction(-0.15)`. `getAppliedPhysicalPower()`
tells you what the hub is really putting on the windings; `getPower()` only reports what the robot code sent,
which in RUN_TO_POSITION is the speed cap.

`Mechanism1DofPlant` reads `getPower()`, so it can only be driven open loop (RUN_WITHOUT_ENCODER). For a
mechanism in RUN_TO_POSITION or RUN_USING_ENCODER, put its load on the motor as above.

## Exact: the SDK's own behaviour

Read off the SDK 12.0.0 sources (`DcMotorImpl`, `DcMotorImplEx`, `LynxDcMotorController`, the Lynx command
classes). Tests pin every one; none should ever be loosened.

| Behaviour | What happens |
|---|---|
| Missing target | Entering RUN_TO_POSITION before any `setTargetPosition` throws `TargetPositionNotSetException`: "Failed to enable motor. You must set a target position before switching to RUN_TO_POSITION mode". The hub refuses the channel enable that follows the mode change. The mode is recorded first, so `getMode()` then reports RUN_TO_POSITION, with the channel off. Every later `setPower`, `setVelocity` or `setMotorEnable` throws the same until a target is sent. |
| Order | `setTargetPosition` is accepted in any mode. `setMode(RUN_TO_POSITION)` re-sends the last power, so the motor starts moving at once, capped by the power already set. |
| Power | In RUN_TO_POSITION `setPower(p)` sends `abs(p)`, and `getPower()` reports `abs(p)`. The hub turns it into a speed cap of `abs(p) * maxTicksPerSecond`, where `maxTicksPerSecond` is the configured motor type's `getAchieveableMaxTicksPerSecondRounded()`. In RUN_USING_ENCODER `setPower(p)` sends `(int)(direction * p * maxTicksPerSecond)` as the velocity target. |
| `isBusy()` | False in every mode but RUN_TO_POSITION. In RUN_TO_POSITION it is `!atTarget`, the hub's answer for the target and tolerance it was sent. Being "not busy" does not stop the motor: the hub keeps holding the target. |
| Tolerance | Default 5 ticks. `setTargetPositionTolerance` only stores it on the handle. It reaches the hub with the next `setTargetPosition`, so changing it alone changes nothing. A value outside `[0, 65535]` makes that `setTargetPosition` throw `IllegalArgumentException("illegal tolerance: N")`. |
| Direction | `REVERSE` negates the target, the power and a velocity command when they are sent, and the position and velocity when they are read. `setDirection` sends nothing, so flipping it on a running motor changes the readings, not the motion. |
| Encoder reset | `STOP_AND_RESET_ENCODER` stops the shaft and zeroes the count. The last power is kept and re-sent when the mode changes back. |
| Leaving RUN_TO_POSITION | The mode change re-sends the cached power, which is `abs(p)` with no direction applied. An arm switched straight to RUN_USING_ENCODER runs at that power in the positive raw direction. |
| `setVelocity` | Switches to RUN_USING_ENCODER unless already in it or RUN_TO_POSITION, then sends `round(direction * v)` clipped to +/-32767 ticks/s. The `AngleUnit` overload converts through the configured motor type's counts per revolution. |
| `getVelocity` | Whole ticks per second, because the hub reports an `int`. The `AngleUnit` overload converts that whole number: `ticks / CPR * 360` degrees. |
| PIDF | `setPIDFCoefficients` on a mode that is not a PID mode throws `IllegalArgumentException`. RUN_TO_POSITION with a non-zero I, D or F under the PIDF algorithm throws `UnsupportedOperationException` ("setting of pidf coefficents not supported: ...", the SDK's spelling) and changes nothing; LegacyPID is exempt. The hub stores each gain as `round(abs(c) * 65536) * sign(c)`, so what comes back is quantized to 1/65536. `getPIDFCoefficients` and `getPIDCoefficients` accept exactly RUN_USING_ENCODER and RUN_TO_POSITION. `resetDeviceConfigurationForOpMode` puts the default gains back. |
| Structure | The position loop feeds the velocity loop ("our fw runs a position loop, that then runs a velocity loop", `LynxDcMotorController`). |

## Approximation: the firmware

The hub firmware is closed source. These numbers are the fake's best reading and are labelled
APPROXIMATION in `HubMotorControlLoop`. Tests of them carry headroom.

| Approximation | Value in the fake | Why this value |
|---|---|---|
| Default velocity gains | P 10, I 3, D 0, F 0 (PIDF) | What teams read back from `getPIDFCoefficients(RUN_USING_ENCODER)`. goBILDA motor types in the SDK declare none, so these come from the firmware. |
| Default position gain | P 10 (PIDF) | The SDK's own Core Hex motor type declares position P = 10. Unconfirmed for other motors on current firmware. |
| Gain units | Output 32767 = full power; velocity error in ticks/s; position P gives ticks/s of velocity target per tick of error. | The FTC tuning rule `F = 32767 / maxTicksPerSecond` only works in these units. |
| Loop rate | One pass every 10 ms (100 Hz). The I term adds `I * error` once per pass. | Not published. Chosen so the default gains settle a velocity step in a few hundred ms with light overshoot. |
| Integral wind-up | The I term is clamped to full scale, and cleared on every mode change, a disabled channel and an encoder reset. | Not published. |
| Velocity measurement | The loop sees the modelled shaft speed exactly; `getVelocity()` rounds it to the nearest whole tick/s. | The real hub measures counts over a window, with some lag and noise. |
| At-target test | `abs(target - count) <= tolerance`, position only, no speed condition, so a motor passing through the target at speed reads not busy for that moment. | The hub's test is not published, including whether the boundary is inclusive. |
| Target memory | A target, once sent, is kept across mode changes and encoder resets; nothing clears it. | Whether the firmware forgets its target on a reset is not published. |
| Not modelled | Battery voltage. The SDK's 500 ms power and enable caches: the fake sends every command and every enable, so where a robot repeating the same power within 500 ms sends nothing, and would throw `TargetPositionNotSetException` one call later, the fake throws at once. Bulk-caching `isBusy()` outside RUN_TO_POSITION (the fake says false). `FLOAT` vs `BRAKE`. | |

What follows from these, and should be read as approximate: how long a move takes, how far it overshoots,
how much an arm sags before the loop catches it, and whether a sluggish mechanism rings. With the default
gains a sluggish motor rings in RUN_TO_POSITION: at `maxAccel` 6 per second it takes over a second to
settle, and at 4 it is still ringing after 8 s.

## The real-hub check

One motor with an encoder, on a hub with current firmware, logging at the bulk-read rate. For each item,
compare the robot's numbers with the same commands run through `FakeMotor` with that motor's free speed and
`maxAccel`.

1. **Default gains.** Print `getPIDFCoefficients(RUN_USING_ENCODER)` and `getPIDFCoefficients(RUN_TO_POSITION)`
   on a freshly powered hub, for a goBILDA motor type. Expect 10/3/0/0 and 10/0/0/0. Anything else replaces
   `defaultVelocityCoefficients()` or `defaultPositionCoefficients()`.
2. **Velocity step.** Unloaded, `setVelocity(half of free speed)` from rest. Measure the time to reach 98%,
   the peak overshoot and the steady error. This checks the gain units and the loop rate together. The fake
   reaches 98% in about 0.3 s with about 10% overshoot for `maxAccel` 12.
3. **Feedforward units.** Set P = I = D = 0 and F = 32767 / free speed, then `setVelocity(half)`. If the
   motor settles at half speed, the output units are right.
4. **Position step.** Unloaded, `setTargetPosition(2000)`, RUN_TO_POSITION, power 1.0. Log position and
   `isBusy()` every loop. Measure the time to first not-busy, the peak overshoot in ticks, and whether
   `isBusy()` flickers as the motor passes through the target. Repeat at power 0.3 and check the peak speed
   is close to 0.3 of free speed.
5. **Holding under load.** An arm held horizontal in RUN_TO_POSITION. Measure the steady error and how the
   current compares with holding it open loop. Push it off by hand and time the return.
6. **Tolerance boundary.** In RUN_TO_POSITION at power 0 the hub holds the motor still, so set the target
   to exactly the current position + 5 (tolerance 5) and read `isBusy()`. False means the boundary is
   inclusive, as the fake assumes.
7. **Target after reset.** Reach a target, `STOP_AND_RESET_ENCODER`, then RUN_TO_POSITION with no new
   `setTargetPosition`. If it throws `TargetPositionNotSetException`, the firmware forgets its target on a
   reset, and `hubTargetPositionSet` must be cleared there.

Record the date, firmware version and motor for each result next to the constant it confirms or replaces.
