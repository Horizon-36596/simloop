# SimLoop

Run your FTC robot code on a laptop, deterministically, and get a log you can score.

SimLoop is a season-agnostic library from Horizon (FTC 36596). Your `Robot.init()`, your subsystems and
your OpMode run **unchanged** against fake hardware in a JVM unit test — no Control Hub, no emulator, no
Robolectric. Time comes from a fake clock rather than the wall, so the same run produces the same log
every time, and that log is a PsiKit RLOG, which AdvantageScope opens and which the `loop` package can
score automatically.

Package root: `org.horizon36596.simloop`. Licence: BSD 3-Clause.

> **SimLoop is an alpha.** `v0.2.0-alpha.1` is the current alpha and the first public one. The API may
> change between alpha versions, so pin an exact version in your build. It needs **FTC SDK 12.0**.
> Support is best effort: a small FTC team maintains it in its own time. `v0.1.0-beta1`, the earlier
> build on JitPack, is superseded and will not get fixes.
>
> Questions go to [Discussions](https://github.com/Horizon-36596/simloop/discussions). Bugs go to
> [Issues](https://github.com/Horizon-36596/simloop/issues). Pull requests are welcome; see
> [`CONTRIBUTING.md`](CONTRIBUTING.md).

**What it does, one line per feature:** [`SimLoop/docs/features.md`](SimLoop/docs/features.md).

## Adopting it is one folder and one line

**First, put your FTC project on SDK 12.0.** SimLoop is built against FTC SDK 12.0.0; the earlier beta
used 11.2.1.

Copy [`simloop-starter/`](simloop-starter) into the root of your FTC project, add one line to the bottom
of `TeamCode/build.gradle`, and run the tests:

```groovy
apply from: "$rootDir/simloop-starter/simloop.gradle"
```

```bash
./gradlew :TeamCode:testDebugUnitTest
```

Two tests pass in a few seconds, with no robot attached — that is the example robot in that folder
driving itself. The folder carries every build setting SimLoop needs, so nothing else in your build
changes, and removing it is deleting the folder and the line.

You do not clone this repository to use SimLoop. It resolves from JitPack like any other dependency, and
the starter folder is what declares it.

## Start here

| If you want to | Read |
|---|---|
| See every feature in one list | [`SimLoop/docs/features.md`](SimLoop/docs/features.md) |
| Adopt it in an existing FTC project | [`simloop-starter/README.md`](simloop-starter/README.md) |
| Install it by hand, or know what the starter is doing | [`SimLoop/docs/getting-started.md`](SimLoop/docs/getting-started.md) |
| Write a first test, line by line | [`SimLoop/docs/first-test.md`](SimLoop/docs/first-test.md) |
| Understand a package, with units and frame on every number | [`SimLoop/docs/`](SimLoop/docs/index.md) |
| Know what it deliberately does **not** do | [`SimLoop/docs/limits.md`](SimLoop/docs/limits.md) |
| Program it with an AI agent | [`simloop-agents/README.md`](simloop-agents/README.md) |
| Cut a release, or check the coordinate | [`SimLoop/PUBLISHING.md`](SimLoop/PUBLISHING.md) |
| Contribute a fix or a feature | [`CONTRIBUTING.md`](CONTRIBUTING.md) |
| Change code in this repository | [`SimLoop/CLAUDE.md`](SimLoop/CLAUDE.md) |

`SimLoop/docs/` is written to be read as Markdown in the repository and also builds into a site. That
site is live at **<https://libraries.horizon36596.org/simloop/>**, with the generated API reference
under [`/javadoc/`](https://libraries.horizon36596.org/simloop/javadoc/). The
`horizon-36596.github.io/simloop/` address still answers and redirects there. How the hosting is put
together is in [`SimLoop/PUBLISHING.md`](SimLoop/PUBLISHING.md).

That path is this repository's name, in lower case, because GitHub serves a project site at its
repository name under the organisation's domain and the path is case-sensitive. It is also why the
published coordinate is `com.github.Horizon-36596:simloop`.

## Help build SimLoop

SimLoop is built by Horizon (FTC 36596) for any FTC team, and it gets better when other teams and
simulator authors add to it. The library is season-agnostic, so everything below helps every season.
Contributions we want most:

- **New fake devices** that replicate a real FTC SDK class method for method, with tests that compare
  them to the SDK: a sensor, an odometry computer, a motor controller feature the fakes lack.
- **Field and game-piece models**: new shapes, contact cases, goals and structures, built from the
  `field` package's general pieces, never one game's field.
- **Documentation**: a page that confused you, a missing unit, an example that should exist.
- **Examples**: a worked test for a common robot, such as a drivetrain with odometry or an arm in
  `RUN_TO_POSITION`.
- **Interop with other tools**: log formats, AdvantageScope layouts, other replay or analysis tools that
  could read or write what SimLoop produces.

Start with [`CONTRIBUTING.md`](CONTRIBUTING.md). [`docs/good-first-issues.md`](docs/good-first-issues.md)
lists small starter tasks. For anything bigger, open a
[Discussion](https://github.com/Horizon-36596/simloop/discussions) first so nobody spends a weekend on
something that will not fit.

## What is in this repository

| Path | What it is |
|---|---|
| [`SimLoop/`](SimLoop) | The library. The only thing that is published. |
| [`simloop-starter/`](simloop-starter) | The folder teams copy: the build settings, and the example robot. |
| [`simloop-agents/`](simloop-agents/README.md) | An optional toolkit for teams programming with an AI agent: a portable `AGENTS.md`, four skills, prompts, and notes on cost. Markdown only — no build file reads it, and deleting it changes nothing. |
| [`examples/`](examples/build.gradle) | A Gradle module with no sources of its own — it compiles the starter folder and runs its tests, so the thing teams copy cannot rot. Never published. |
| [`docs/landing-site/`](docs/landing-site/README.md) | The page that fronts `libraries.horizon36596.org`. Nothing builds it; it belongs in the `Horizon-36596.github.io` repository and is kept here so it is version-controlled somewhere. |

## Why the library sits in a subdirectory

The library is a Gradle **subproject** rather than the root project. That reads as redundant and is not,
for a reason that is cheaper to keep than to change:

- The Android Gradle Plugin's library plugin is applied to something it expects. A root project that is
  itself an Android library is a layout nothing else in the FTC ecosystem uses.
What does **not** depend on it is the published coordinate, which is worth saying because this file used
to claim the opposite. JitPack names the artifact after the repository, not after the Gradle module, as
long as the build publishes exactly one module — measured, not assumed: the first build of
`v0.1.0-beta1` served `com.github.Horizon-36596:simloop`. What would rename it is publishing a **second**
module, which switches JitPack to its multi-module spelling. So the rule to keep is "one published
module", not "one particular directory layout".

So `SimLoop/` in a path and `:SimLoop` in a Gradle task are correct everywhere they appear, including in
the CI workflows.

## Building and testing it

From the repository root:

```bash
./gradlew test
```

1,218 JVM tests (1,212 in the library, 4 in the keyboard-window module, 2 in the example robot), with no
device and no emulator. The
documentation site and the API reference:

```bash
pip install -r SimLoop/docs-requirements.txt
./gradlew :SimLoop:docsSite
```

All of it is built on every push and every pull request by
[`.github/workflows/test.yml`](.github/workflows/test.yml), so none of it can quietly become something
that only builds on one laptop. Pull requests also run a leak check
([`tools/leak-gate/`](tools/leak-gate/leak-gate.sh)) and a secret scan
([`.github/workflows/leak-gate.yml`](.github/workflows/leak-gate.yml)).

## Licence

**BSD 3-Clause** — [`LICENSE`](LICENSE) is the licence text, [`NOTICE.md`](NOTICE.md) is the copyright
and what it covers. Copyright (c) 2026, Horizon (FTC 36596).

For an FTC team, in practice: use it, change it, ship it on your robot, and publish your robot code
under any licence you like. Keep the copyright notice with any copy of SimLoop's source or build you
hand on, and do not use Horizon's name to endorse your fork.

SimLoop's own dependencies are three and they keep their own licences: the FIRST Tech Challenge SDK's
`RobotCore` and `Hardware` (BSD), and PsiKit (BSD 3-Clause). This licence covers SimLoop's own code and
nothing else, which is what [`NOTICE.md`](NOTICE.md) states exactly.
