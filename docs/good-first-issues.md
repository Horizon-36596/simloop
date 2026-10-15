# Good first issues

Small, well-scoped starter tasks found while refreshing the documentation for `v0.2.0-alpha.1`. Each one
is meant to become a GitHub issue labelled `good first issue`. None needs a robot. Read
[`CONTRIBUTING.md`](../CONTRIBUTING.md) first for how to build, test and open a pull request.

Run the tests with `./gradlew :SimLoop:testDebugUnitTest` (or `.\gradlew.bat` on Windows), and the docs
with `python -m mkdocs build -f SimLoop/mkdocs.yml --strict`.

---

## 1. Give the starter folder the two test settings the IMU and Limelight fakes need

**Why.** `FakeImu` needs `unitTests.returnDefaultValues = true`, because the SDK's orientation types call
an Android method that otherwise throws "not mocked". `FakeLimelight3A` needs the real `org.json` on the
test runtime classpath. `SimLoop/build.gradle` sets both for the library's own tests, but
`simloop-starter/simloop.gradle` sets neither, so a team that copies the starter and adds an IMU gets a
"not mocked" failure. `SimLoop/docs/getting-started.md` currently tells teams to add the lines by hand.

**Files.** `simloop-starter/simloop.gradle`, `SimLoop/docs/getting-started.md` (remove the "starter does not
set these yet" note), and a test under `simloop-starter/tests/`.

**How to test.** Add a starter test that builds `new FakeImu(new RevHubOrientationOnRobot(...))` and reads
a yaw. Run `./gradlew :examples:test`. It should fail before the change and pass after.

## 2. Fix the stale `@return` on `FirstOrderFit.fit`

**Why.** The javadoc says a negative free speed is "returned rather than refused so the report can say
so". The code now throws `IllegalArgumentException` when the fitted power term is zero, negative or not
finite, so a negative free speed cannot come back. The comment sends a reader looking for a case that no
longer exists.

**Files.** `SimLoop/src/main/java/org/horizon36596/simloop/fit/FirstOrderFit.java` (the javadoc above
`public static Result fit(...)`).

**How to test.** `./gradlew :SimLoop:apiDocs` must still report zero warnings. `FirstOrderFitTest` already
pins the refusal (the test that flips the velocity column), so the javadoc only has to match it; name that
test in the comment.

## 3. Pin the example on the `fit` documentation page in a test

**Why.** `SimLoop/docs/packages/fit.md` shows reading an RLOG with `RlogTimeSeries`, fitting it, and writing
a `GapReport`. The second half is covered by `GapReportTest`; the RLOG-reading half is not run anywhere, so
it could drift from the API.

**Files.** A new test in `SimLoop/src/test/java/org/horizon36596/simloop/fit/`, and the snippet in
`SimLoop/docs/packages/fit.md` if anything has to change.

**How to test.** Write a short synthetic run with `ScenarioRunner.run` that logs power, velocity and
volts from a `FakeMotor`, then run the page's snippet on that RLOG and assert the fitted free speed is
close to the motor's. Mention the test's path on the page.

## 4. Add troubleshooting entries for "not mocked" and an empty Limelight result

**Why.** Both failures read like a SimLoop bug and are a missing test setting.
`SimLoop/docs/troubleshooting.md` covers several such traps, but not these two.

**Files.** `SimLoop/docs/troubleshooting.md`.

**How to test.** Reproduce each message in a scratch test (remove the setting, run `FakeImuTest` or
`FakeLimelight3ATest`), quote the exact message in the entry, and build the docs with `--strict`.

## 5. Replace a season-specific example in `ObstacleBox`'s javadoc

**Why.** `SimLoop/CLAUDE.md` rule 1 says no javadoc may name a season's game or its pieces. The class
comment of `ObstacleBox` lists "a flower's foot" as an example obstacle, which is one game's field
element.

**Files.** `SimLoop/src/main/java/org/horizon36596/simloop/field/ObstacleBox.java`.

**How to test.** Swap it for a generic example (a goal's frame, a wall segment, a raised platform).
`./gradlew :SimLoop:apiDocs` and `bash tools/leak-gate/leak-gate.sh` must pass.

## 6. Show how to run a continuous-rotation servo on the `plant` page

**Why.** `ContinuousRotationSimConfig` is listed on the `config` page, and `ContinuousRotationPlantTest`
shows that a CR servo runs through `Mechanism1DofPlant` with no end stops. The `plant` page never says
so, so a team with a CR-servo intake has to find it in the tests.

**Files.** `SimLoop/docs/packages/plant.md`.

**How to test.** Copy the wiring from `ContinuousRotationPlantTest` into the page, with units. Build the
docs with `--strict`.

## 7. Replay a recorded gamepad track through `OpModeRunner`

**Why.** `sim.live` records keyboard sessions as `GamepadInputTrack`s, and the only replay example drives a
toy robot through `ScenarioRunner.runUntil`. Teams will want to replay a session through their real
OpMode. `OpModeRunner.gamepads(...)` takes a `GamepadScript`, and a track's frames can feed it.

**Files.** A new example test in `SimLoop/src/test/java/org/horizon36596/simloop/example/`, and a short
section in `SimLoop/docs/packages/live.md`.

**How to test.** Write a three-line track by hand, replay it into the `TankTeleOp` from
`OpModeRunnerExampleTest` using `GamepadState.applyTo`, and assert the motor positions. Run it twice and
assert identical results.

## 8. Read the tick length from a track's header

**Why.** A replay must run at the session's tick, but `GamepadInputTrack` does not check it; the tick is
only a free-text `# tickSeconds:` metadata line by convention. A helper that reads it as a number, or
fails clearly when it is missing, saves every team writing the same parsing code.

**Files.** `SimLoop/src/main/java/org/horizon36596/simloop/sim/live/GamepadInputTrack.java` and
`GamepadInputTrackTest`.

**How to test.** A track with `# tickSeconds: 0.02` returns 0.02; one without the line, or with a
non-number, fails with a message naming the line. Open an issue first to agree the method name, since it
is public API.

## 9. Take the version in the docs footer from `build.gradle`

**Why.** The version is written in `SimLoop/build.gradle` and again in the `copyright:` line of
`SimLoop/mkdocs.yml`. The comment in `mkdocs.yml` says they are not linked and `PUBLISHING.md` lists the
second copy as something to bump by hand.

**Files.** `SimLoop/build.gradle` (the `docsSite` task), `SimLoop/mkdocs.yml`, `SimLoop/PUBLISHING.md`.

**How to test.** Change the version in `build.gradle` only, run `./gradlew :SimLoop:docsSite`, and check
the footer of a built page shows the new version. Put the version back before committing.

## 10. Lead the bug form's SDK list with 12.0

**Why.** `.github/ISSUE_TEMPLATE/bug_report.yml` lists FTC SDK `11.2.1` first. `v0.2.0-alpha.1` needs SDK
12.0, so the form should lead with 12.0.0 and say that 11.x is only for reports against the superseded
`v0.1.0-beta1`.

**Files.** `.github/ISSUE_TEMPLATE/bug_report.yml`.

**How to test.** Open the form from a fork's "New issue" page and check the dropdown.

## 11. A worked example: a mecanum robot with a Pinpoint, run as an OpMode

**Why.** Each piece is documented on its own (`MecanumDrivePlant`, `FakeGoBildaPinpoint`, `OpModeRunner`),
but nothing shows them together, and the inches-to-millimetres seam is easy to get wrong.

**Files.** A new example test in `SimLoop/src/test/java/org/horizon36596/simloop/example/`, linked from
`SimLoop/docs/packages/fakehardware.md`.

**How to test.** A `LinearOpMode` reads the Pinpoint's pose and drives forward 24 in. Assert the
Pinpoint's reported X is within 0.5 in of the drive plant's. Run it twice and assert identical logs with
`RlogDecodedCompare`.
