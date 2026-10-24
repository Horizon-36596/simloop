# SimLoop

Run your FTC robot code on a laptop, deterministically, and get a log you can score.

SimLoop is a season-agnostic library from Horizon (FTC 36596). Your `Robot.init()`, your subsystems and
your OpMode run **unchanged** against fake hardware in a JVM unit test — no Control Hub, no emulator, no
Robolectric. Time comes from a fake clock rather than the wall, so the same run produces the same log
every time, and that log is a PsiKit RLOG, which AdvantageScope opens and which the `loop` package can
score automatically.

Package root: `org.horizon36596.simloop`. Licence: BSD 3-Clause.

Run SimLoop from your robot project's tests or simulation harness and view the result in a separate
AdvantageScope window. This release covers that library-and-viewer workflow. The standalone app
being built around SimLoop is not ready and is not part of this release.

The source tree can be newer than the latest tag. Distortion, occlusion, tilted tags and synthetic
camera frames are development additions after `v0.2.0-alpha.1`; they are not in that pinned artifact.
Use the starter and documentation from the tag you install, or pin the exact source commit you test.

> **SimLoop is an alpha.** `v0.2.0-alpha.1` is the current alpha and the first public one. The API may
> change between alpha versions, so pin an exact version in your build. It needs **FTC SDK 12.0**.
> Support is best effort: a small FTC team maintains it in its own time. `v0.1.0-beta1`, the earlier
> build on JitPack, is superseded and will not get fixes.
>
> **It has been used by one team, on one robot's code, and its physics has not yet been compared with a
> real robot.** Expect gaps, and tell us what you find: you are testing it.
> [What has been tested](SimLoop/docs/tested.md) lists what has been checked and what has not.
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

`SimLoop/docs/` can be read as Markdown and built into a site. Its publishing address is
**<https://libraries.horizon36596.org/simloop/>**, with generated Javadoc under
[`/javadoc/`](https://libraries.horizon36596.org/simloop/javadoc/). If the site is unavailable, read the
docs at your pinned tag or build them locally with `./gradlew :SimLoop:docsSite`.
[`SimLoop/PUBLISHING.md`](SimLoop/PUBLISHING.md) explains the hosting and deployment checks.

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

`SimLoop/` is the Android library subproject; the root build coordinates it with the starter examples
and desktop keyboard window. Gradle tasks therefore use the `:SimLoop` prefix.

The published coordinate follows the repository name, `com.github.Horizon-36596:simloop`. Keep one
published module: the desktop jar is a classifier of that same publication, not a second module.
Publishing a second module would change JitPack's artifact naming.

## Building and testing it

From the repository root:

```bash
./gradlew test
```

The library, keyboard-window and starter-example tests run on the JVM, with no device or emulator.
To build the documentation site and API reference:

```bash
pip install -r SimLoop/docs-requirements.txt
./gradlew :SimLoop:docsSite
```

[`.github/workflows/test.yml`](.github/workflows/test.yml) defines the automated test and documentation
checks. Pull-request checks also include the [leak gate](tools/leak-gate/leak-gate.sh) and
[secret scan](.github/workflows/leak-gate.yml). Check the workflow results for the commit you use; the
commands above run the tests and documentation build locally.

## Licence

**BSD 3-Clause** — [`LICENSE`](LICENSE) is the licence text, [`NOTICE.md`](NOTICE.md) is the copyright
and what it covers. Copyright (c) 2026, Horizon (FTC 36596).

For an FTC team, in practice: use it, change it, ship it on your robot, and publish your robot code
under any licence you like. Keep the copyright notice with any copy of SimLoop's source or build you
hand on, and do not use Horizon's name to endorse your fork.

SimLoop's own dependencies are three and they keep their own licences: the FIRST Tech Challenge SDK's
`RobotCore` and `Hardware` (BSD), and PsiKit (BSD 3-Clause). This licence covers SimLoop's own code and
nothing else, which is what [`NOTICE.md`](NOTICE.md) states exactly.
