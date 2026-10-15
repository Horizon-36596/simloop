# Contributing to SimLoop

Thanks for helping. SimLoop is built by Horizon (FTC 36596), and it gets better when other FTC teams
use it on their robots and tell us where it is wrong. You do not need to be on our team, in our GitHub
organisation, or an expert in anything. A good bug report is a real contribution.

SimLoop is an **alpha**. The API can still change, and reviews are best effort: we are students with a
season to run. We will get to your pull request, but it may take a few days.

## The short version

1. **Fork** this repository on GitHub.
2. **Make a branch** in your fork, named for what it does (`fix/servo-range-check`, `docs/first-test-typo`).
3. **Make the change, with a test** that fails before it and passes after it.
4. **Run the tests** (below).
5. **Open a pull request** against `main` here. Fill in the template.

A maintainer from Horizon reviews it. When it is approved and the checks are green, we merge it.

You keep the copyright on what you write. By opening a pull request you agree it is shared under this
repository's licence, [BSD 3-Clause](LICENSE), as GitHub's terms of service already provide. There is
no contributor licence agreement and no sign-off line to add.

## Building and testing

You need:

- **JDK 17.** The Android Gradle Plugin needs it. Temurin is a good free choice. Newer JDKs may work;
  17 is what CI uses.
- **The Android SDK.** The library is an Android library, so Gradle needs to find the SDK even though
  the tests run on a plain JVM. If you have Android Studio (every FTC team does), you have it. Gradle
  finds it through the `ANDROID_HOME` environment variable, or a `local.properties` file in the
  repository root with one line, `sdk.dir=<path to your SDK>`. `local.properties` is ignored by git;
  never commit it.

You do not need a robot, a Control Hub or an emulator.

Run the library's tests from the repository root:

```bash
# macOS and Linux
./gradlew :SimLoop:testDebugUnitTest
```

```powershell
# Windows (PowerShell or Command Prompt)
.\gradlew.bat :SimLoop:testDebugUnitTest
```

Run one test class, or one test method:

```bash
./gradlew :SimLoop:testDebugUnitTest --tests "org.horizon36596.simloop.plant.Mechanism1DofPlantTest"
./gradlew :SimLoop:testDebugUnitTest --tests "org.horizon36596.simloop.plant.Mechanism1DofPlantTest.someMethodName"
```

Before you open a pull request, run everything CI runs. `test` also compiles and tests the example
robot in `simloop-starter/`, which is the folder other teams copy:

```bash
./gradlew test
```

If you changed anything under `SimLoop/docs/` or any public javadoc, build the documentation too. It
fails on a broken link or on a public method with no javadoc:

```bash
pip install -r SimLoop/docs-requirements.txt
./gradlew :SimLoop:docsSite
```

You can also run the leak check that every pull request runs. It needs bash (Git Bash on Windows):

```bash
bash tools/leak-gate/leak-gate.sh
```

## How code here is written

Read a file or two near your change before you write, and match it. The habits that matter most:

- **Units and frame on every number.** A javadoc or a comment that mentions a distance, angle, speed or
  time says its unit (inches, radians, seconds) and, for a pose, its frame and sign convention. A bare
  `double distance` is a bug waiting for a team to use the wrong unit.
- **Say where every physical number came from.** Measured on a robot, read off a datasheet, or an
  estimate. An estimate is marked as one, so nobody trusts it like a measurement.
- **Deterministic.** Nothing reads the wall clock (`System.nanoTime()`, `System.currentTimeMillis()`,
  `ElapsedTime`). Time comes from the injected `FakeTimer`. The same run must produce the same log.
  `update(deltaTime)` changes state; getters only read.
- **Fail loudly.** Throw with a message that says what to fix, rather than returning a silent zero.
- **Java 8 language level** in `SimLoop/src/main` (no `var`, no records), because FTC robot projects
  compile at Java 8.
- **Every public class and method has javadoc.** The docs build fails without it.
- **Every change has a test.** A bug fix comes with the test that would have caught it. A new fake
  device comes with tests that compare it to what the real SDK device does.
- **Do not break the public API without saying so.** Other teams compile against it. If you must rename
  or remove something public, say so at the top of your pull request.

## What belongs here, and what does not

SimLoop is **season-agnostic**. It must work for any FTC game, past or future.

- **Belongs here:** fake devices that implement real FTC SDK interfaces, plant models, the scenario
  runner, logging and scoring, documentation, the starter folder, fixes to any of these.
- **Belongs in your own repository:** your robot's code, your season's field, game pieces and scoring
  rules, your tuned constants, your CAD. SimLoop gives you interfaces to plug those in; it does not
  hold them. A pull request that imports `org.firstinspires.ftc.teamcode`, names a specific game, or
  adds CAD, logs or other large files will fail the leak check.

Not sure? Open an issue and ask first.

## Bigger changes

For anything larger than a bug fix or a doc edit, such as a new package, a new kind of plant, or a
change to how runs are logged, **open an issue or a Discussion first** and describe what you want to
do. That way nobody spends a weekend on something that will not fit. A short description and a sketch
of the API is enough.

## Where to start

Look at the issues labelled
[`good first issue`](https://github.com/Horizon-36596/simloop/labels/good%20first%20issue), or at
[`docs/good-first-issues.md`](docs/good-first-issues.md), which lists small starter tasks with the
files to touch and how to test each one. Doc fixes are always welcome too: if a page confused you, it will confuse the next team.

Found a bug but have no time to fix it? An issue is still very helpful. Use the bug form, or the
"Sim doesn't match my robot" form if the sim runs but gives different numbers than your robot.

## Pull requests written with an AI agent

Welcome, on one condition: **a human has read the change and run the tests** on their own machine
before opening the pull request, and says so in the template. You are responsible for what you submit,
whoever typed it. The [`simloop-agents/`](simloop-agents/README.md) folder has notes on working with
an agent in this codebase.

## Review

- A maintainer from Horizon (FTC 36596) reviews every pull request.
- The checks (tests, docs build, leak check, secret scan) must pass.
- We may ask for changes. That is normal and not a rejection.
- Alpha means best effort. If a week passes with no reply, a polite comment on the pull request is fine.

## Security problems

Do not open a public issue. See [SECURITY.md](SECURITY.md).

## Conduct

Everyone here follows the [Code of Conduct](CODE_OF_CONDUCT.md). Be kind; we are all here to build
robots.
