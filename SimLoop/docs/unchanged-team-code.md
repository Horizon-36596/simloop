# Running your timer code unchanged

Your robot code waits for things. "Run the intake for half a second." "Wait until the arm has had 300 ms."
"Park for ten seconds, then go." It does that with `ElapsedTime`, `sleep(ms)`, `Thread.sleep`, or
`System.nanoTime()`. This page shows how to run that code in a SimLoop test **without changing a line of
it**, and explains the warnings and error messages you might see.

You need to know how to write a JUnit test and what `ElapsedTime` is, and your build needs the few lines in
[Before you start](#before-you-start).

## Why a sim needs this

A SimLoop test has its own clock, the **sim clock**. It moves forward one tick at a time, usually 20 ms,
the length of one loop on a Control Hub. Motors, mechanisms and the robot's position all follow the sim
clock. A tick's wall-clock duration depends on the work it does and the laptop running it.

Code that reads the **real** clock therefore measures a different duration from the simulation.
A plain `new ElapsedTime()` measures laptop time: "wait 500 ms" can span a different number of sim
ticks on each run.

`OpModeRunner.forTeamClass(...)` fixes this. It loads your classes in a special way so that every clock
call in them reads the sim clock instead.

## Before you start

1. Add SimLoop to your project as [Getting started](getting-started.md) describes.
2. In `TeamCode/build.gradle`, make sure SimLoop and JUnit 5 are on the test classpath, and switch on the
   two test settings (the repository lines can go wherever your project lists its repositories, such as
   `build.dependencies.gradle`):

```groovy
repositories {
    maven { url "https://jitpack.io" }                       // SimLoop itself
    maven { url "https://repo.dairy.foundation/releases" }   // PsiKit, which SimLoop logs with
}

android {
    testOptions {
        unitTests.all { useJUnitPlatform() }                 // JUnit 5
        unitTests.returnDefaultValues = true                 // Android calls return 0/null in tests
    }
}

dependencies {
    testImplementation 'com.github.Horizon-36596:simloop:v0.2.0-alpha.2'
    testImplementation "org.junit.jupiter:junit-jupiter-api:5.11.3"
    testRuntimeOnly "org.junit.jupiter:junit-jupiter-engine:5.11.3"
    testRuntimeOnly "org.junit.platform:junit-platform-launcher:1.11.3"  // Gradle 9 no longer adds it
}
```

`returnDefaultValues` is needed by the SDK's own classes: `new RevHubOrientationOnRobot(...)`, which nearly
every IMU init calls, touches an Android class that otherwise throws "not mocked" in a JVM test.
(`testImplementation` keeps SimLoop out of the robot app.)

The snippets below need these imports:

```java
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.horizon36596.simloop.fakehardware.FakeHardwareMap;
import org.horizon36596.simloop.fakehardware.FakeMotor;
import org.horizon36596.simloop.opmode.OpModeRunResult;
import org.horizon36596.simloop.opmode.OpModeRunner;
import org.junit.jupiter.api.Test;
```

## How to use it

Name your OpMode's class instead of building it with `new`:

```java
@Test
void myAutoParksInTime() {
    FakeHardwareMap hardwareMap = new FakeHardwareMap();
    hardwareMap.register("left", new FakeMotor(2787, 10));
    hardwareMap.register("right", new FakeMotor(2787, 10));

    OpModeRunResult result = OpModeRunner.forTeamClass("org.firstinspires.ftc.teamcode.MyAuto", hardwareMap)
            .activeTicks(1500)          // 30 s at 50 Hz
            .run();

    assertTrue(result.endedBeforeStop(), "the auto finished inside the 30 s");
    assertEquals("Status : Parked", result.telemetry().lastTransmission().get(0));
}
```

Why the first check: when STOP comes, every `opModeIsActive()` loop ends and the code after it still runs,
on the robot and in the sim. So an auto that prints "Parked" after its drive loop prints it even when STOP
cut the drive short, and the second check alone would pass with a 1 second run. `endedBeforeStop()` is true
only if `runOpMode()` returned before STOP, and `endedAtTick()` says on which tick.

You can also pass the class itself: `OpModeRunner.forTeamClass(MyAuto.class, hardwareMap)`. Either way,
the runner builds a **fresh** copy of your OpMode when the run starts, the way the robot builds one when
you press INIT. The other runner options (`initLoopTicks`, `activeTicks`, `gamepads`, `eachTick`,
`runLogged`) work exactly as they do with `new OpModeRunner(opMode, hardwareMap)`, and code both runners
can run (a constant `sleep(ms)`, `idle()`) takes the same ticks on both.

### Which classes are changed

Only your own code. By default that is every class whose name starts with:

- `org.firstinspires.ftc.teamcode.` (the FTC `TeamCode` package), and
- `com.seattlesolvers.solverslib.` (SolversLib, which many teams copy into TeamCode).

If your code lives somewhere else, list your packages. Each one ends with a dot:

```java
OpModeRunner.forTeamClass("com.example.ourteam.MyAuto", hardwareMap)
        .teamPackages("com.example.ourteam.", "com.seattlesolvers.solverslib.")
        .activeTicks(1500)
        .run();
```

The Java library, the FTC SDK (including its Inspection, Blocks and FtcRobotController packages), SimLoop,
PsiKit and JUnit are **never** changed. A prefix inside one of them, or one so broad it would take one in
(`"org."`, `"com."`, `"org.firstinspires."`), is refused with a message saying so. A library is listed by
its top-level package (see [Pedro Pathing](#pedro-pathing-and-other-libraries)).

## What changes, and what it does in the sim

| Your code | In the sim |
|---|---|
| `new ElapsedTime()`, `new ElapsedTime(startNanos)`, `new ElapsedTime(Resolution.MILLISECONDS)` | A timer on the sim clock. `seconds()`, `milliseconds()`, `reset()`, `toString()` and the rest give exactly what the SDK's timer gives. |
| A class of yours that `extends ElapsedTime` | It now extends the sim-clock timer instead, so it reads the sim clock too, `super.nsNow()` included. |
| `System.nanoTime()` | Sim time, in nanoseconds: 0 while the OpMode is being built (field initializers, constructor), one tick (20 ms) when `runOpMode()` or `init()` starts, because INIT is the first tick. The OpMode's own `time` field counts from INIT, so it reads 0 there. |
| `System.currentTimeMillis()` | 1 January 2026, 00:00 UTC, plus sim time. The same date every run, so a file name made from it repeats. |
| `sleep(ms)` in a `LinearOpMode` | Waits that many ms of **sim** time: `sleep(1000)` is exactly 50 ticks. The duration can be anything: `sleep(500)`, `sleep(ms * 2)`, `sleep(fast ? 100 : 1000)`, or `sleep(millis)` inside a helper method. |
| `Thread.sleep(ms)`, `Thread.sleep(ms, nanos)`, `TimeUnit.MILLISECONDS.sleep(ms)` | The same as `sleep(ms)`, but like the real ones they throw `InterruptedException` if STOP is pressed while they wait. `Thread.sleep(0)` returns at once, as on the robot, even in an iterative OpMode. |
| Android's `SystemClock.uptimeMillis()`, `elapsedRealtime()`, `elapsedRealtimeNanos()`, `SystemClock.sleep(ms)` | Sim time in ms (or ns), and a sim-time wait. (In a JVM test Android's own `SystemClock` reads 0 and waits for nothing; rewritten, it is never used.) |
| `idle()` in a `LinearOpMode` | Like the SDK's, it takes no time by itself; the tick ends when the same `idle()` runs a second time in it. So `while (timer.seconds() < 2) { idle(); }` waits 2 s of sim time, one pass per tick after the first, and `sleep(50); idle();` takes exactly the 3 ticks of the sleep. |
| `System::nanoTime`, `ElapsedTime::new`, `() -> System.nanoTime()` | Method references and lambdas follow along. |

A wait rounds **up** to whole ticks, never down. At 50 Hz, `sleep(30)` is 2 ticks (40 ms).

## What does not change

Some things keep the real clock. The ones SimLoop can spot are listed in the run's `result.warnings()`,
each with its class, method and line; the rest are listed here.

- **`getRuntime()`** is the SDK's method, so it stays approximate: sim time plus the few microseconds of
  real time your code has taken this tick. An OpMode that shows `getRuntime()` in telemetry therefore sends
  slightly different telemetry every run. Use the `time` field, which is exact.
- **The SDK's `Deadline` timer** (`org.firstinspires.ftc.robotcore.internal.system.Deadline`, used by the
  SDK's HuskyLens and Blinkin samples) is SDK code, so it keeps the real clock: in the sim it expires by
  laptop time. A class of yours that builds one or extends it gets a warning. Use an `ElapsedTime`.
- **The real date and time**: `Instant.now()`, `LocalDateTime.now()` and the other `java.time` `now()`
  methods, `Clock.systemUTC()`, `new Date()` and `Calendar.getInstance()` read today's date and differ every
  run (each gets a warning). `System.currentTimeMillis()` reads the sim clock's fixed date instead. Kotlin's
  `kotlin.time` (`TimeSource.Monotonic`, `measureTime`) also keeps the real clock and gets no warning;
  `System.nanoTime()` and `measureTimeMillis` in Kotlin code are moved.
- **Other libraries** outside your team packages keep the real clock. Add their package with
  `teamPackages(...)` if they should follow the sim clock. Pedro Pathing and Road Runner get a warning; see
  [Pedro Pathing](#pedro-pathing-and-other-libraries).
- **Other kinds of waiting**: `Object.wait`, `Thread.join`, `CountDownLatch.await`, `Future.get` with a
  timeout, `BlockingQueue.poll`/`take`, `LockSupport.parkNanos`, a library's own `Thread.sleep`, executors
  and scheduled tasks are not moved. On a thread you start they wait in real time. On a `LinearOpMode`'s own
  thread SimLoop keeps the interrupt flag set (that is how the SDK's `sleep()` comes back to the sim), so:
  - **usually they end at once**: `InterruptedException` straight away (`parkNanos` just returns), no sim
    time and no real time passes. If your code catches the exception and carries on, the wait silently
    never happened; if it does not, the run fails with a message that says why;
  - **but after your code has caught one**, the flag stays clear until your next lifecycle call
    (`opModeIsActive()`, `sleep()`, ...), and a wait in that gap waits in **real** time and moves no sim
    time, so the run depends on the laptop.

  Use `sleep(ms)`, `Thread.sleep(ms)` or `TimeUnit.sleep` in your own code instead.
- **`sleep()` reached through an interface, a method reference or reflection** (`Waits w = this;
  w.sleep(100);`) is the SDK's `sleep`, which SimLoop cannot read a duration from, so the run fails saying
  so. Call `sleep(ms)` directly on the OpMode.
- **Threads you start yourself** are not stepped by the sim. A `Thread.sleep` on one is refused, and the run
  fails (see below).
- **A serializable method reference to a clock call** (`(LongSupplier & Serializable) System::nanoTime`)
  runs on the sim clock but cannot be deserialized after the rewrite. FTC code does not do this.

## Pedro Pathing and other libraries

Pedro Pathing reads the real clock inside its own code: `Follower.update()` with no argument, its `Timer`,
`PIDController` and the PID-style controllers, the two- and three-wheel and fusion localizers, and the
follower's hold timeout behind `isBusy()`. Pedro is **not** in the default team packages, so under
`forTeamClass` all of that keeps laptop time. The run warns once, and the result can be wrong and differ
from run to run: dead-wheel velocities come out far too large, an acceleration limit can stop the robot
moving at all, and `isBusy()` clears depending on how fast the computer is.

To put Pedro on the sim clock, add its package, which `SimClockLoader.PEDRO_PATHING` names:

```java
OpModeRunner.forTeamClass("org.firstinspires.ftc.teamcode.MyAuto", hardwareMap)
        .teamPackages("org.firstinspires.ftc.teamcode.", "com.seattlesolvers.solverslib.",
                SimClockLoader.PEDRO_PATHING)
        .activeTicks(1500)
        .run();
```

What that costs: Pedro's classes are then loaded afresh for each run, like yours, so a Pedro `Pose` or
`Path` cannot be handed from the test into the run, or from one run to the next through the blackboard (see
the trade-off below). Hand over numbers instead.

List a library by its **top-level** package (`"com.pedropathing."`, not `"com.pedropathing.utils."`):
rewriting part of a library gives it two copies of those classes on two clocks, or fails with a
`LinkageError`. SimLoop refuses a partial prefix of Pedro Pathing or Road Runner
(`com.acmerobotics.roadrunner.`); for other libraries, check the package list yourself.

## The trade-off: a second copy of your classes

To change your classes without touching your files, SimLoop loads them a second time for every run. That has
these effects you might notice:

1. **You cannot cast the OpMode to its own class.** `(MyAuto) runner.opMode()` throws
   `ClassCastException`, because the test's `MyAuto` and the one that ran are different copies. Check what the
   OpMode did through the fake hardware, its telemetry, or `OpMode.blackboard`.
2. **An object you build in the test cannot be handed in.** A `new MyArmController()` made in your test is
   the test's copy, not the loaded one.
3. **An object one run leaves on the blackboard cannot be cast in the next.** On the robot an auto can leave
   its end pose as a `MyPose` for the TeleOp after it. In the sim, each run has its own copy of `MyPose`, so
   the TeleOp run fails with a `ClassCastException` naming the same class twice, and SimLoop adds a hint.
   Hand SDK or JDK types instead: numbers, strings, a `Pose2D`, maps and lists of those.
4. **Static fields start fresh on every run**, as they do on a robot after a restart. A static your test
   sets is not the one the OpMode reads.
5. **A timer's class is SimLoop's.** `timer.getClass()` is `SimClockElapsedTime`, not `ElapsedTime`, and a
   subclass of yours has `SimClockElapsedTime` as its superclass. `instanceof ElapsedTime` is still true.

If you need to build the OpMode yourself (to pass something into its constructor), use
`new OpModeRunner(opMode, hardwareMap)` instead. Then nothing is changed: `sleep()` must be given a
constant and timers read the real clock. For a timer your test builds itself, SimLoop's `SimElapsedTime`
reads a `FakeTimer` you hand it; [Making your timers sim-safe](sim-safe-timers.md) shows the small code
changes that let a test hand one in, which is the better fix when you can make it.

## Error messages, and what to do

**"SimLoop cannot run Thread.sleep(10) at ...loop(MyTeleOp.java:42) on sim time: MyTeleOp is an iterative
OpMode, whose init(), init_loop(), start(), loop() and stop() must each return promptly"**
You called `Thread.sleep` inside an iterative `OpMode`. On a robot this freezes the whole loop, gamepads
included. Instead, start an `ElapsedTime` and check it on each `loop()` call, or make it a `LinearOpMode`.

**"SimLoop refused a call in MyAuto's code, so the run fails even though that code carried on ...: SimLoop
cannot run Thread.sleep(50) at ... on sim time: it was called on thread "arm-helper", and only MyAuto's own
thread ... waits in sim time"**
A thread your OpMode started is sleeping. The sim only steps the OpMode's own thread, so it cannot tell a
helper thread how long 50 ms is. The helper usually dies of this at once, and your code may even catch it,
so SimLoop fails the run rather than let it pass with the helper's work never done. Do that work in
`runOpMode()`'s main loop instead.

**"SimLoop cannot move the method reference ::sleep in ...runOpMode (MyAuto.java:30) onto sim time ...
Write it as a lambda (ms -> sleep(ms))"**
You passed `this::sleep` somewhere. Change it to `ms -> sleep(ms)`: it does the same thing, and SimLoop can
move a lambda. Usually the run fails before it starts. If the reference is in a class your OpMode first uses
after PLAY, the error comes then, and the run fails even if your code catches it.

**"MyAuto ran for at least 10000 ms of real time ... without reaching a lifecycle check, sleep(), idle() or
waitForStart() ... a loop that waits for a timer ... is a busy-wait that never ends"**
Your code has a loop like `while (timer.seconds() < 2) { }` with nothing inside. On a robot real time moves
while it spins. In the sim, time only moves between ticks, and a tick only ends when your code calls
`opModeIsActive()`, `sleep()`, `idle()` or similar. Add `opModeIsActive()` to the loop's condition
(`while (opModeIsActive() && timer.seconds() < 2)`), which is also safer on the robot because it lets STOP
work, or call `idle()` inside the loop. SimLoop ends the stuck thread at its next clock read.

**"MyAuto did not return from runOpMode() after STOP ... Sim time does not move after STOP"**
A loop that waits for a timer, `sleep()` or `idle()` was still going when STOP came. After STOP the sim
clock stops, so it can never finish. Put `opModeIsActive()` in its condition.

**"MyTeleOp.runOpMode() threw during INIT ...: java.lang.ClassCastException: class ...MyPose cannot be cast
to class ...MyPose ... SimLoop: each OpModeRunner.forTeamClass run loads the team's classes afresh"**
An object of one of your classes came from the test or from an earlier run. See the trade-off, point 3.

**"MyAuto.runOpMode() threw during ACTIVE ...: java.lang.InterruptedException ... No STOP explains it"**
A wait SimLoop does not move ended at once. See "Other kinds of waiting" above.

**"SimLoop cannot tell how long the sleep() in ... is: no LinearOpMode.sleep call was found on that line, so
sleep() was reached some other way"**
See "sleep() reached through an interface" above.

**"OpModeRunner.forTeamClass("...MyAuto"): that class is not in a package SimLoop rewrites (...)"**
Your OpMode is outside the team packages, so its timers would read the real clock. Add its package with
`teamPackages(...)`.

**"SimClockLoader: "org." would take in ..."** or **"... is only part of Pedro Pathing"**
The prefix is too broad, or too narrow. Name your own packages, and a library by its top-level package.

**"OpModeRunner.forTeamClass("..."): SimLoop cannot find that class on the test classpath"**
Check the spelling, and give the full name including the package, such as
`org.firstinspires.ftc.teamcode.auto.MyAuto`.

**"SimLoop has no sim clock here: this SimClockHooks was loaded by ..., not by a SimClockLoader"**
Something made a sim-clock timer or called a sim-clock hook outside a `forTeamClass` run, for example
`new SimClockElapsedTime()` in a test. Those classes only work inside a run. Use `forTeamClass`, or
`SimElapsedTime` for a timer of your own.

**"SimLoop cannot run sleep(500) at ... on sim time: no OpMode run is using this sim clock right now"**
Your OpMode slept (or called `idle()`) while it was being built (in a field initializer or its constructor),
or a thread kept running after the run ended. Move the wait into `runOpMode()`.

**"SimLoop cannot read the class file of ..."**
A class file is damaged, or was compiled for a newer Java than SimLoop knows. Team code for the FTC SDK
should be compiled for Java 8. Rebuild, and if it still fails, report it with the class name.

## Is it the same every run?

Yes, for your own code: the same OpMode, gamepad script and fake hardware give the same telemetry, the same
motor positions and an RLOG that decodes to the same values every time, however fast or slow the computer
is. A rewritten `sleep` advances by sim ticks rather than waiting for its duration on the laptop.
The exceptions are the things listed under
[What does not change](#what-does-not-change): `getRuntime()` in telemetry, the real date and time, and any
library outside your team packages that reads the clock, such as Pedro Pathing under the default packages.
The RLOG's raw bytes differ from run to run, because PsiKit logs its own writer queue
(`Logger/QueuedCycles`); compare logs with `RlogDecodedCompare`, which skips it.

Runs share two things in one JVM: `OpMode.blackboard`, which each run clears when it starts (unless it
keeps it), and PsiKit's `Logger`. Two `runLogged` runs started at once take turns; tests that run OpModes
must not run in parallel inside one JVM (Gradle's separate test JVMs are fine).

## For the curious: how it works

When the test asks for your OpMode, a `SimClockLoader` reads your compiled `.class` files and makes a few
small edits before Java loads them. Each edit swaps one reference for another of exactly the same size:
`System.nanoTime` becomes `SimClockHooks.nanoTime`, `new ElapsedTime` becomes `new SimClockElapsedTime`,
and a call to a `LinearOpMode`'s `sleep` or a `TimeUnit`'s `sleep` becomes a call to a static hook that
takes the object as its first argument. Because nothing changes size, no other part of the class file has
to be recomputed, and Java's own verifier checks the result when the class loads. The code is
`org.horizon36596.simloop.simclock.ClassFileRewriter`, written by hand against the Java class-file
specification (JVMS chapter 4) so SimLoop needs no extra library; a damaged class file is refused quickly
rather than hanging it. Each loader gets its own copy of the hooks, so two sims in one test never share a
clock. Every refusal is recorded on the loader as well as thrown, which is how the run fails even when the
exception is caught or kills a thread no one watches.
