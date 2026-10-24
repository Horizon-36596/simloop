# What has been tested

SimLoop is an alpha. So far one team has used it, Horizon (FTC 36596), on one robot's code, on Windows.
Most of it has never run in another team's project, and its physics has not yet been compared with a
real robot. This page says what has been checked and what has not, so you know where to trust it and
where to look twice.

If you try it, you are testing it. Reports help most where this page says "not yet".

## Checked

- **Your code runs unchanged.** One team's season code runs its OpModes, autos and TeleOp under the
  runner, along with the example robot in the starter folder. The library's own suite is about
  1,200 tests.
- **Replay determinism.** Replay tests repeat scenarios and compare decoded robot-authored fields.
  This checks the scenarios covered by those tests, not every possible consuming robot program.
- **The fakes match the FTC SDK.** Every fake device is checked against the SDK 12.0 class it stands in
  for. The Pinpoint and OTOS fakes run the SDK's own drivers on emulated registers.
- **Installing from JitPack.** The starter folder was copied into one fresh FTC project, which resolved
  `v0.2.0-alpha.1` from JitPack and passed its tests.
- **CI.** The tests, the docs build and the leak gate run on Linux on every pull request.

## Not checked yet

- **Against a real robot.** No sim number has been compared with a measured one. The physics defaults
  are placeholders or carried over from elsewhere, and are labelled that way where they are defined.
  A drivetrain, a slide or a shot that looks right in the sim may still be wrong on your robot.
- **Fitting from real logs.** The `fit` package has only been run on logs the sim wrote itself, never
  on a log from a real robot.
- **Other teams' code.** Only one team's code has been run, so only one drivetrain layout and one set
  of libraries. Other drivetrains, other path followers, Kotlin, Blocks and OnBot Java have not been
  tried.
- **macOS and Linux desktops.** Development is on Windows. Linux runs only in CI, with no display.
  The keyboard window has been opened on Windows only.
- **Game-piece physics against a real field.** Bounces, rolling and contacts are plausible, not
  measured.
- **The simulated camera and Limelight against real captures.** They are checked against the SDK's
  parsing and against their own geometry, not against what a real camera sees.
- **Synthetic frames against real frames.** `SimFrameRenderer` is checked against `SimCamera`'s geometry:
  pieces land where `SimCamera` puts them, occluded ones are hidden, distortion bends the image the way
  the lens model says, and the same seed gives the same pixels. Nobody has compared a synthetic frame with
  a photo, and no colour threshold has been tuned on one. None should be.
- **Lens distortion against a real calibration.** The model is OpenCV's five-coefficient one, checked
  against its own inverse, not against a calibrated camera.

## How to help

- **Run the starter in your own project** and say what happened, good or bad, in
  [Discussions](https://github.com/Horizon-36596/simloop/discussions).
- **When the sim disagrees with your robot**, open a "Sim doesn't match my robot" issue with the
  robot's log. That is the report that improves the defaults for every team.
- **On a Mac or Linux desktop**, tell us whether the starter and the keyboard window work.

This page changes as things are checked. If it says something has been tested and your experience
says otherwise, that is a bug: [report it](support.md).
