---
name: write-a-sim-test
description: Write a SimLoop simulation test for an FTC subsystem - drive real robot code against fake hardware, log it, and assert on the log. Use when asked to add a test for a subsystem, prove a mechanism reaches a position, or check an autonomous routine without a robot.
---

# Write a SimLoop simulation test

A SimLoop test drives the team's **real** subsystem against fake hardware, writes a log, and asserts by
reading that log back. It is not a mock test and it is not a physics demo: the thing under test is the
robot code, and the log is the evidence.

## Before you write anything

1. **Read the subsystem you are testing.** What does `periodic()` do, what does it command, what does it
   read, and what units does it use? Do not guess the units from the name alone — open the file.
2. **Find an existing sim test in this project and read it.** Match its shape. If there is none, the
   shape below is the one to use.
3. **Decide what could actually be wrong.** Write this down in one sentence before writing code. "The
   slide overshoots the scoring height" is a test. "The slide works" is not.

If the behaviour you have been asked to test needs something SimLoop does not model — contact between
two robots, a camera, a game-piece being physically intaked — stop and say so. Check
<https://libraries.horizon36596.org/simloop/limits/> rather than approximating it.

## The shape

```java
@Test
void theSlideReachesScoringHeightAndHoldsIt() throws IOException {
    FakeTimer timer = new FakeTimer();
    ExampleRobotSim sim = new ExampleRobotSim();
    Path rlogPath = Paths.get("build", "sim", "slide-reaches-score.rlog");

    // Set up the situation: what has the driver, or the autonomous routine, asked for?
    sim.robot.slide.setLevel(ExampleSlide.Level.SCORE);

    ScenarioRunner.run("SlideReachesScore", ExampleRobotSim.ROBOT_CONFIG, timer, 150, 0.02, rlogPath,
            deltaTime -> {
                sim.robot.periodic();            // 1. robot code, unchanged
                sim.advancePhysics(deltaTime);   // 2. the world moves
                Logger.recordOutput("Slide/heightInches", sim.robot.slide.getPositionInches());
            });

    RunResult run = RunResult.fromRlog("SlideReachesScore", rlogPath, 150);

    assertEquals(24.0, run.finalValue("RealOutputs/Slide/heightInches"), 0.5,
            "the slide should finish within tolerance of the scoring height");
    assertTrue(run.maxValue("RealOutputs/Slide/heightInches") <= 26.0,
            "the slide overshot scoring height by more than two inches");
}
```

## The five things that go wrong

1. **The log key.** You write `"Slide/heightInches"` and read back `"RealOutputs/Slide/heightInches"`.
   PsiKit adds the prefix. Querying the unprefixed key throws — it does not return zero.
2. **The tick order.** `periodic()` first, then `advancePhysics()`, then log. Physics first delays every
   response one tick and the test still passes, wrongly.
3. **Reaching into the subsystem instead of the log.** Read through the subsystem's own getter, the
   number the robot itself believes — then assert on the logged value. An assertion on the plant's
   internal state tests the model, not the code.
4. **Too few ticks.** 150 ticks at `0.02` s is three seconds of robot time. If the mechanism needs four
   seconds to settle, the test fails for a reason that is not a bug. Work out the time it needs.
5. **An assertion that cannot fail.** See below. This is the important one.

## Make sure the assertion can fail

The plant clamps a mechanism to its end stops every tick. So `assertTrue(height >= 0 && height <= 24)`
passes for every possible controller, including one that does nothing. It is testing the model's
guarantee, not the robot's achievement.

Assert on what the robot code can get wrong:

| Instead of | Assert |
|---|---|
| it stayed in range | it finished within tolerance of the target |
| it moved | it overshot by no more than X |
| it did not crash | it settled within N ticks and stayed there |
| a boolean flag is set | the commanded power went to ~0 once it arrived |

**Then prove it.** Change the gain, or the target, so the behaviour is wrong, run the test, and confirm
it goes red. Put it back. A test never seen to fail has not been shown to test anything.

## Finish

Run the project's test command, quote the real result, and say plainly whether it passed:

```
./gradlew :TeamCode:testDebugUnitTest
```

On Windows use `.\gradlew.bat :TeamCode:testDebugUnitTest` instead; the extensionless `gradlew` is the
Unix script.

Then tell the human where the `.rlog` landed — `build/sim/<name>.rlog` — so they can drag it into
AdvantageScope and look at the curve. A number that passed and a curve that looks wrong is a real
outcome, and only a human eye catches it.
