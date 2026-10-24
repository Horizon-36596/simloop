# Troubleshooting

Start with the error message or symptom below. Check the build setup, clock and log path before
changing your robot code.

## `Could not find org.psilynx.psikit:core`

You are missing the Dairy repository line. PsiKit is not on Maven Central, and a POM can say what a
dependency is but never where to find it.

```groovy
maven { url "https://repo.dairy.foundation/releases" }
```

See [Installing it](getting-started.md#the-repository-lines).

## `Failed to load JUnit Platform`, before any test runs

You are on Gradle 9 without the JUnit Platform launcher. Gradle 8 put it on the test runtime classpath for
you; Gradle 9 does not.

```groovy
testRuntimeOnly 'org.junit.platform:junit-platform-launcher:1.11.3'
```

The version tracks your Jupiter version — 1.11.3 pairs with 5.11.3, and they move together.

## `RunResult.MetricNotFoundException` on a key you definitely logged

You logged `"Slide/positionInches"`; read it back as `"RealOutputs/Slide/positionInches"`. PsiKit files
everything `Logger.recordOutput` writes beneath `RealOutputs/`.

It throws rather than returning zero on purpose: a silent zero is a passing test that measured nothing.

## `cannot find symbol: class DcMotorEx` pointing at SimLoop's jar

Your project is not pulling the FTC SDK through SimLoop. SimLoop declares it at `api` scope precisely so
this cannot happen, so if you see it, something in your build is stripping transitive dependencies —
check for an `exclude` or a platform constraint on `org.firstinspires.ftc`.

## `SDK location not found`

Gradle cannot find the Android SDK. The tests run on the JVM, but the Android project still needs its
SDK installed and configured. On the same machine, a fresh checkout or worktree can use
`local.properties` from a working checkout. On another machine, set `sdk.dir` to that machine's SDK
location; do not copy someone else's absolute path. Keep this file out of version control.

## The mechanism moves a tick late, and no gain fixes it

Check the order inside your scenario body. It must be: read state, decide, command the fake, **then**
`plant.update(deltaTime)`. Updating before commanding delays every response by exactly one tick.

## Two runs of the same scenario produce different logs

First check for wall-clock reads: `System.nanoTime()`, `System.currentTimeMillis()`, and timers built
on them, including the SDK's `ElapsedTime`. Also check unseeded randomness, iteration order, shared
static state, and concurrent tests writing to the same log path.

Everything sim-side must take its time from the `FakeTimer` you constructed. `RlogDecodedCompare` is what
tells you the two runs differ and in which field. Use the
[OpMode runner](packages/opmode.md) for supported timer rewriting in unchanged team classes, or
[inject sim time](sim-safe-timers.md) into code you control.

## The run threw, and the message is about the RLOG rather than my robot

`ScenarioRunner` refuses to hand back a log it cannot vouch for. Four things cause it, and they are all
the same statement — *the log is not a record of the run*:

- PsiKit's async writer queue **overflowed**;
- the file is **missing or empty**;
- it decodes to a **different number of frames** than ticks you ran;
- the writer thread **stopped making progress** while the run waited.

None of them is affected by what your own assertions found. The commonest cause in a new project is a
second test JVM writing the same RLOG path — give each scenario its own file name.

## The intake never picks anything up, and the mouth measurements are right

Check whether a drivetrain pose reached a field question unrotated. `MecanumDrivePlant` reports the
localizer frame (+x forward, +y left); `GamePieceTracker.updateRobotPose` and `TriggerVolume.overlapsPiece`
want the field frame (+X right, +Y forward). They are ninety degrees apart, nothing throws, and the
symptom is an empty mouth that looks like bad geometry.

```java
tracker.updateRobotPose(
        FieldFrameTransform.fieldXFromLocalizer(drive.getX(), drive.getY()),
        FieldFrameTransform.fieldYFromLocalizer(drive.getX(), drive.getY()),
        FieldFrameTransform.fieldHeadingFromLocalizer(drive.getHeading()));
```

A quick way to confirm it is the frame and not the mouth: drive straight forward and print both the
drivetrain's X and the rotated field Y. If forward motion is showing up as field X rather than field Y,
the rotation is missing.

## The RLOG is not there any more

`build/` is a build output directory and `./gradlew clean` deletes it. Re-run the test; that is the
command that regenerates the log.

## AdvantageScope opens the file but the graph is empty

Drag the key in from the tree on the left — `RealOutputs/...`, not the bare name. An empty graph with the
file open usually means the key you dragged had no numeric values, not that the run logged nothing.

## `./gradlew :SimLoop:check` fails in lint

Known, and not yours: lint reports `NewApi` errors on `java.nio.file` calls in classes that only ever run
in a JVM test. Run `:SimLoop:testDebugUnitTest` instead, which is what CI runs and what the library is
verified with.
