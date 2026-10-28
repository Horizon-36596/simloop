# `simclock`

`org.horizon36596.simloop.simclock`

Puts your own timer code on the sim clock without changing a line of it. You rarely touch this package
directly: `OpModeRunner.forTeamClass(...)` in [`opmode`](opmode.md) uses it for you.

## Why it exists

A default tick advances 20 ms of simulated time, regardless of how long it takes on the laptop.
Code that reads the laptop's clock (`new ElapsedTime()`, `System.nanoTime()`) or waits with
`Thread.sleep` can therefore behave differently between runs. This package loads your team's classes
with supported clock reads and sleeps rewritten to use sim time.

## How you use it

Name the OpMode's class instead of building it:

```java
OpModeRunResult result = OpModeRunner.forTeamClass("org.firstinspires.ftc.teamcode.MyAuto", hardwareMap)
        .activeTicks(1500)                         // a 30 s auto at 50 Hz
        .run();
```

That is the whole change. The full guide, with the build settings it needs and every warning and error
message, is [Running your timer code unchanged](../unchanged-team-code.md).

## What you might see

| Type | What it is |
|---|---|
| `SimClockLoader` | The class loader. Loads your team's classes a second time, with their clock calls rewritten. `OpModeRunner.forTeamClass` makes one per run. |
| `SimClockElapsedTime` | What your unchanged `new ElapsedTime()` becomes. It is the SDK's `ElapsedTime`, reading the run's sim clock. You never build one. |
| `SimClockHooks` | The static methods the rewritten calls go to (`nanoTime()`, `threadSleep(...)`, `opModeSleep(...)`, ...). You never call them. |
| `SimClockScheduler`, `JvmInstructions` | Plumbing between the loader and the runner, and the bytecode reader. Public only so the runner can reach them. |

For a timer your **test** builds and hands to your code, use `org.horizon36596.simloop.sim.SimElapsedTime`
instead. It takes the `FakeTimer` it reads. See [Making your timers sim-safe](../sim-safe-timers.md).

## What it rewrites

Only classes in your team packages. By default those are `org.firstinspires.ftc.teamcode.` and
`com.seattlesolvers.solverslib.`; `.teamPackages(...)` on the runner replaces the list. The JDK, the FTC
SDK, SimLoop, PsiKit and JUnit are never rewritten, whatever the list says.

Inside those packages it moves:

- `ElapsedTime`: all three constructors, and your own subclasses of it;
- `System.nanoTime()` and `System.currentTimeMillis()`;
- `Thread.sleep`, `TimeUnit.sleep`, Android's `SystemClock` reads and `SystemClock.sleep`;
- `LinearOpMode.sleep(ms)` with any duration, computed or not, and `idle()`.

## Units

- Sim time: **seconds** on the `FakeTimer`, read back as **nanoseconds** or **milliseconds** wherever the
  call it replaces returned those units.
- A sleep of `ms` milliseconds holds the OpMode for `ceil(ms / tick)` ticks of sim time.

## The price

Your classes are loaded a second time for each run. So:

- the OpMode cannot be cast to its own class in the test (read its fields by reflection, or through the
  SDK's types, telemetry, the blackboard or the fake hardware);
- a team object built by the test, or left by an earlier run, cannot cross into the run;
- your statics start fresh every run.

## What this package will not do

- **It will not move clock reads outside your team packages.** The SDK's `Deadline` timer, the real date
  and time (`java.time`, `Date`, `Calendar`) and libraries that read the clock are left alone. The ones it
  can see are listed in `result.warnings()`.
- **It will not move `Object.wait`, `Thread.join`, latches, futures or executors.**
- **It refuses some things outright**, and fails the run even if your code catches the refusal: a
  `Thread.sleep` on any thread but a `LinearOpMode`'s own or an iterative OpMode's own during `init()`,
  and a method reference `this::sleep` (write
  `ms -> sleep(ms)` instead).
- **It will not rewrite an OpMode instance you built yourself.** Only `forTeamClass` loads classes through
  it.
