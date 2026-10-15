# Making your timers sim-safe

If your robot code waits for anything - "run the intake for half a second", "give the servo 300 ms to
move", "stop after 2 seconds" - this page is for you. It explains why those waits go wrong in a
simulator, and exactly what to change so they come out right, on the robot and in the sim.

You do not need to read the rest of the SimLoop docs first. You need to know what `ElapsedTime` is.

## The problem in one paragraph

On the robot there is one clock: real time. In a SimLoop test there are two. The **sim clock** is a
`FakeTimer`, and SimLoop moves it forward by a fixed step each tick - usually 20 ms, the same as one loop
on a Control Hub. The **wall clock** is your laptop's real time, and it keeps running at its own speed. A
laptop does one 20 ms tick in well under a millisecond, so the two clocks drift apart fast. Everything the
sim simulates - motors turning, mechanisms moving, the robot driving - follows the sim clock. So any code
that looks at the wall clock is looking at the wrong clock. A 250 ms wait that reads the wall clock can
last 30 *simulated* seconds, and a different number every run. (That is a real measurement: it happened
to our own autonomous before we fixed it.)

Two runs of the same test giving different logs is the tell-tale sign. The fix is always the same idea:

> **Your code gets its time from one place, and a test can swap that place for the sim clock.**

**Cannot change the code, or not yet?** `OpModeRunner.forTeamClass(...)` runs an OpMode as it is and moves
its `ElapsedTime`, `sleep`, `Thread.sleep` and `System.nanoTime()` calls onto the sim clock for you, at the
cost of loading your classes a second time. See [Running your timer code unchanged](unchanged-team-code.md).
The changes on this page are still the better long-term fix: they work with `new OpModeRunner(opMode, ...)`,
with any test harness, and let a test hand your code any clock it likes.

## Quick reference

| Your code uses | What it does in the sim | The fix |
|---|---|---|
| `new ElapsedTime()` | Measures laptop time, not sim time. Waits come out far too long and different every run. | Make timers through a timer factory the test can swap. [Details](#elapsedtime) |
| `LinearOpMode.sleep(ms)` | Stops your code for real milliseconds while the sim clock does not move at all. The robot you are waiting for never gets to move. | Replace each `sleep` with a wait loop that checks a swappable timer and runs one robot loop per pass. [Details](#linearopmodesleep) |
| SolversLib `WaitCommand` | Measures laptop time (it has its own `ElapsedTime` inside, which you cannot reach). | Use a small wait command of your own that reads your clock. [Details](#waitcommand-solverslib) |
| `System.nanoTime()` / `System.currentTimeMillis()` | Laptop time. | Read time from a swappable clock instead. [Details](#systemnanotime-and-systemcurrenttimemillis) |
| `getRuntime()` / `time` in an `OpMode` | Laptop time. | Override `getRuntime()` and `resetRuntime()` in your OpMode. [Details](#getruntime-and-time-in-an-opmode) |

## What SimLoop gives you: `SimElapsedTime`

`org.horizon36596.simloop.sim.SimElapsedTime` **is** an `ElapsedTime` - it extends the SDK class - but it
reads the sim clock. Everything else about it is the SDK's own code: `seconds()`, `milliseconds()`,
`nanoseconds()`, `time()`, `time(unit)`, `reset()`, `startTime()`, `getResolution()` and `toString()` all
give the same answers a real `ElapsedTime` would, just in sim time. So anything that accepts an
`ElapsedTime` accepts one of these without knowing the difference.

```java
FakeTimer timer = new FakeTimer();                 // the sim clock; SimLoop advances it each tick

ElapsedTime elapsed = new SimElapsedTime(timer);   // starts at 0, like new ElapsedTime()
timer.advance(0.25);                               // 250 ms of sim time pass
elapsed.milliseconds();                            // 250.0 - exactly, every run
```

It has the same constructors as the SDK class, each with the `FakeTimer` first:

- `new SimElapsedTime(timer)` - like `new ElapsedTime()`
- `new SimElapsedTime(timer, ElapsedTime.Resolution.MILLISECONDS)` - like `new ElapsedTime(resolution)`
- `new SimElapsedTime(timer, startNanoseconds)` - like `new ElapsedTime(startTime)`

And three small helpers, for the "swappable place" your code will get its time from:

| Helper | Gives you | Your robot code uses, on the robot |
|---|---|---|
| `SimElapsedTime.factory(timer)` | a `Supplier<ElapsedTime>` that makes a fresh sim timer each call | `ElapsedTime::new` |
| `SimElapsedTime.nanoClock(timer)` | a `LongSupplier` that returns sim time in nanoseconds | `System::nanoTime` |
| `SimElapsedTime.nanoTimeOf(timer)` | sim time in nanoseconds, right now | `System.nanoTime()` |

Use the **same** `FakeTimer` your scenario advances - the one you pass to `ScenarioRunner.run`. A timer
reading a different `FakeTimer` would be reading a clock nobody moves.

Two things to know:

- **If you call `timer.reset()`** to start a second run, reset or remake your timers too. A timer started
  before the reset would otherwise read a negative time, which is what a real `ElapsedTime` would do if
  its clock went backwards.
- **Do not call `log(...)`** on any `ElapsedTime` in a test. It writes to the Android log, which does not
  exist on a laptop. That is true of the real class too.

SimLoop is a test-only library. **Never import it from your robot code.** Your robot code offers a way in;
only your tests hand it a `SimElapsedTime`.

## `ElapsedTime`

**What goes wrong.** `new ElapsedTime()` always makes a real, wall-clock timer. Java gives a test no way
to change what `new` makes, so as long as your subsystem writes `new ElapsedTime()` itself, a test cannot
reach it.

**The fix.** Let whoever builds the subsystem say how to make timers. Keep a constructor that works
exactly as before, so nothing else in your code changes:

```java
public class Intake {
    private final Supplier<ElapsedTime> newTimer;
    private final ElapsedTime spinTimer;

    /** The robot uses this one - real timers, exactly as before. */
    public Intake(HardwareMap hardwareMap) {
        this(hardwareMap, ElapsedTime::new);
    }

    /** Tests use this one, to hand in timers that read the sim clock. */
    public Intake(HardwareMap hardwareMap, Supplier<ElapsedTime> newTimer) {
        this.newTimer = newTimer;
        this.spinTimer = newTimer.get();   // was: new ElapsedTime()
        // ...
    }
}
```

In the test:

```java
FakeTimer timer = new FakeTimer();
Intake intake = new Intake(fakeHardwareMap, SimElapsedTime.factory(timer));
```

Replace every `new ElapsedTime()` inside the class with `newTimer.get()`. If your timer used
`new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS)`, either switch it to `milliseconds()` (which works at
any resolution) or pass a `Function<ElapsedTime.Resolution, ElapsedTime>` the same way.

**If you have lots of subsystems,** do it once instead: give your `Robot` class a single clock and have
every subsystem ask it. Our own robot does exactly this with `robot.nanoTime()` - on the robot it returns
`System.nanoTime()`, and in a test it returns sim time. A `LongSupplier` set up once in `Robot`, with
`System::nanoTime` on the robot and `SimElapsedTime.nanoClock(timer)` in a test, is all you need.

## `LinearOpMode.sleep`

**What goes wrong.** `sleep(500)` stops your code for half a real second. It is a `final` method, so
nothing can change what it does. In a sim this is backwards: while your code is asleep, nothing moves the
sim clock or the simulated robot, so after the "wait" the mechanism is exactly where it was. The sim does
not slow down to wait for you - it simply cannot run while you sleep.

**The fix.** Stop waiting by sleeping, and wait by *looping*: check a timer, and run one pass of your
robot's loop each time round. That is better on the real robot too - a sleeping OpMode stops updating its
sensors, telemetry and any control loops.

```java
/** Waits the given time, running the robot the whole time. Use instead of sleep(ms). */
void waitMillis(double millis) {
    ElapsedTime waited = newTimer.get();           // a swappable timer, as above
    while (opModeIsActive() && waited.milliseconds() < millis) {
        robot.update();                            // one loop: read sensors, run subsystems
        afterEachLoop.run();                       // nothing on the robot; one sim tick in a test
    }
}
```

`afterEachLoop` is a `Runnable` your OpMode keeps. On the robot it is `this::idle` (or does nothing). In a
test it is what moves the sim forward one tick: advance the `FakeTimer`, update the plants, log. That hook
is the part SimLoop cannot supply for you: in a `LinearOpMode` **your** code owns the loop, so your code
has to let the sim take its turn.

The simplest route of all is not to use `LinearOpMode` for code you want to simulate. An iterative
`OpMode` (or a command-based one) already runs one short `loop()` per tick, and a "wait" there is just
"check the timer this loop": `if (spinTimer.milliseconds() > 500) { ... }`.

## `WaitCommand` (SolversLib)

**What goes wrong.** `WaitCommand` builds its own timer inside its constructor, and that timer is an
`ElapsedTime` - the wall clock. You cannot hand it a different one. In our own autonomous, a
`WaitCommand(250)` lasted about 30 simulated seconds and changed from run to run.

**The fix.** Write a tiny wait command of your own that reads your clock, and use it everywhere you used
`WaitCommand`. This is the whole thing:

```java
public class WaitOnRobotClock extends CommandBase {
    private final LongSupplier nanoClock;   // System::nanoTime on the robot
    private final long durationNanos;
    private long startNanos;

    public WaitOnRobotClock(long millis, LongSupplier nanoClock) {
        this.nanoClock = nanoClock;
        this.durationNanos = TimeUnit.MILLISECONDS.toNanos(Math.max(0L, millis));
    }

    @Override public void initialize() { startNanos = nanoClock.getAsLong(); }

    @Override public boolean isFinished() {
        return nanoClock.getAsLong() - startNanos >= durationNanos;
    }

    @Override public boolean runsWhenDisabled() { return true; }   // WaitCommand does the same
}
```

On the robot, give it `System::nanoTime` (or your `Robot`'s clock). In a test, give it
`SimElapsedTime.nanoClock(timer)`. On 20 ms ticks a 250 ms wait then finishes on the 13th tick, the first
one at or past 250 ms, every single run.

Other SolversLib pieces read the wall clock in the same hidden way: `Timing.Timer`, `Timing.Stopwatch`
and `Timing.Rate`, `SlewRateLimiter`, `Debouncer`, `MecanumControllerCommand`, and pure-pursuit
timeouts. If a test that uses one of them gives different results from run to run, that is why; the fix is
the same (your own small version that takes a clock).

## `System.nanoTime()` and `System.currentTimeMillis()`

**What goes wrong.** These are the wall clock, directly.

**The fix.** Read time through a `LongSupplier` your code is given (or your `Robot`'s single clock), as in
the sections above. For anything that measures how long something took, use nanoseconds:
`currentTimeMillis()` is the calendar clock and can jump if the device's time is corrected, even on the
robot.

## `getRuntime()` and `time` in an `OpMode`

**What goes wrong.** `OpMode.getRuntime()`, `resetRuntime()` and the `time` field all read the wall clock.

**The fix.** These two methods are not `final`, so your OpMode can override them to use your clock, and
the SDK fills in `time` from `getRuntime()` before each loop, so `time` follows along. (On the robot the
SDK calls `resetRuntime()` for you when INIT is pressed, just *before* your `init()` runs, so it uses your
override there too. That means `nanoClock` must already be set when the OpMode is created - give it a
value where you declare it, not in `init()`, or INIT will crash with a `NullPointerException`.)

```java
private long runtimeStartNanos;

@Override public double getRuntime() {
    return (nanoClock.getAsLong() - runtimeStartNanos) / 1e9;   // seconds
}

@Override public void resetRuntime() {
    runtimeStartNanos = nanoClock.getAsLong();
}
```

## How to check you found them all

1. **Search your code** for `ElapsedTime`, `sleep(`, `WaitCommand`, `nanoTime`, `currentTimeMillis`,
   `getRuntime` and `time`. Every hit is either fixed as above, or only runs on the robot (never in a
   test).
2. **Run the same scenario twice** and compare the two logs with `RlogDecodedCompare`. If the logs differ,
   something is still reading the wall clock; the compare tells you which logged value differed first,
   which points you at the subsystem.
3. **Check a wait lasts what it says.** Log the sim time when a timed step starts and ends. A 500 ms step
   should end 500 ms (rounded up to the next tick) after it started.

## What SimLoop cannot do for you

Everything on this page needs a small change in your code, and that is not an oversight. SimLoop can
only give your code a sim clock if your code has a way to be given one. Without rewriting your classes as
they load (which `OpModeRunner.forTeamClass`, above, does), it cannot reach:

- a `new ElapsedTime()` written inside your class - Java has no way for a test to change what `new` makes;
- `LinearOpMode.sleep` - it is `final`, and while your code sleeps, the sim cannot run;
- `WaitCommand` and the other library classes that build their own timers inside;
- a direct `System.nanoTime()` or `System.currentTimeMillis()` call.

The change is the same every time, and small: **get time from one place a test can swap.** Once your
code does that, `SimElapsedTime` makes every wait in the sim last exactly as long as it says, on every
run.
