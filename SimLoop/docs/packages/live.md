# `sim.live` and the keyboard window

`org.horizon36596.simloop.sim.live`, and the separate module `SimLoop/desktop/`
(`org.horizon36596.simloop.desktop`)

Drive the simulated robot from your laptop keyboard in real time, record exactly what you pressed, and
replay that recording headless to the same log, bit for bit.

## What you use

| Type | What it is |
|---|---|
| `GamepadInputSource` | Where each loop's gamepads come from: a keyboard, a recorded track, or a test's script. |
| `KeyboardGamepadSource` | Turns held keys into a gamepad through a fixed keymap. One keyboard drives gamepad 1 or gamepad 2. |
| `GamepadInputTrack` | A recorded session: one plain-text line per loop. `read(path)`, `replay()`, and a `Writer`. |
| `RecordingGamepadInputSource` | Wraps a source and writes every frame to a track before handing it on. |
| `GamepadInputGate` | Plugs a source into `ScenarioRunner.runUntil`: the run lasts as long as the source has input. |
| `GamepadState` | One gamepad on one loop, as a value. `applyTo(gamepad)` delivers it the way a Driver Station packet does. |
| `RealTimePacer` | Slows a run to real time so a person can drive it. The only class in SimLoop that reads a wall clock. |
| `KeyboardWindow` (in `desktop/`) | A small AWT window that reports which keys are held while it has focus. |

## A session and its replay

From `src/test/java/org/horizon36596/simloop/sim/live/LiveSessionReplayTest.java`. A live session wraps a
keyboard source in a recorder and paces the run; a replay reads the track back and runs as fast as it can.

```java
try (GamepadInputTrack.Writer writer = new GamepadInputTrack.Writer(track,
        Collections.singletonMap("opMode", "ToyRobot"), true)) {
    GamepadInputSource keyboard = new RecordingGamepadInputSource(
            new KeyboardGamepadSource(new ScriptedKeyboard(), 10_000), writer);
    liveTicks = runToyRobot(keyboard, liveRlog, pacerOnATestClock(),
            Collections.singletonList(countingReceiver));
}
GamepadInputTrack recorded = GamepadInputTrack.read(track);
int replayTicks1 = runToyRobot(recorded.replay(), replay1, ScenarioRunner.NO_PACING,
        Collections.<LogDataReceiver>emptyList());
```

and the run itself:

```java
GamepadInputGate gamepads = new GamepadInputGate(input);

return ScenarioRunner.runUntil("ToyRobot", STUB_CONFIG, timer, DT, rlog, gamepads, pacer, extraReceivers,
        deltaTime -> {
            gamepads.current().gamepad1.applyTo(gamepad1);
            gamepads.current().gamepad2.applyTo(gamepad2);
            // ... your robot code reads gamepad1 and gamepad2 ...
        });
```

The test then checks that the live log and both replays decode identically, field for field. In a real
session, use `new RealTimePacer()` in place of the test's pacer, and a real keyboard in place of
`ScriptedKeyboard`.

## The keymap

| Keys | Gamepad |
|---|---|
| `W` `S` / `A` `D` | left stick y / x (`W` is stick up, which is **negative** y, as on a real gamepad) |
| `I` `K` / `J` `L` | right stick y / x |
| `Z` / `C` | left / right trigger |
| arrow keys | d-pad |
| `1` `2` `3` `4` | `a` `b` `x` `y` (cross, circle, square, triangle) |
| `Q` / `E` | left / right bumper |
| `F` / `H` | left / right stick button |
| `ENTER` / `BACKSPACE` | `start` / `back` |
| `SHIFT` held | every axis at half travel |
| `TAB` | switch the keys between gamepad 1 and gamepad 2 |
| `ESCAPE` | end the session |

## Using the keyboard window

`KeyboardWindow` comes with the starter folder: `simloop-starter/simloop.gradle` already has the line
that adds it to your tests. It works on Windows, macOS and Linux. If you installed SimLoop by hand, add
the same line yourself, in `TeamCode/build.gradle`:

```groovy
dependencies {
    testImplementation "com.github.Horizon-36596:simloop:v0.2.0-alpha.1:desktop"
}
```

It is the same coordinate as the library with a `desktop` classifier: a second jar, kept apart because
the window uses the desktop JDK's window toolkit (AWT), which Android code compiles without. Then hand it
to `KeyboardGamepadSource`. Its public API is strings and booleans, so no AWT type reaches your Android
test sources:

```java
KeyboardWindow window = KeyboardWindow.open("SimLoop live drive", helpLines);
KeyboardGamepadSource.Keyboard keys = new KeyboardGamepadSource.Keyboard() {
    @Override public Set<String> keysDown() { return window.keysDown(); }
    @Override public boolean isOpen() { return window.isOpen(); }
};
GamepadInputSource source = new KeyboardGamepadSource(keys, 15_000);   // 5 min at 50 Hz, at most
```

Run the live session from a test on a machine with a display. `KeyboardWindow.open` throws
`IllegalStateException` on a headless machine such as CI; replay a recorded track there instead.

## Units

- A track line is one **loop**, not a span of time. A replay must run at the same tick as the session.
  The track does not check this, so put the tick in its header metadata (`# tickSeconds: 0.02`) and read
  it back from `metadata()`.
- Axis values: unitless, sticks in `[-1, 1]`, triggers in `[0, 1]`, written with `Float.toString` so they
  read back to the same bits.
- `RealTimePacer.MAX_LAG_BEFORE_RESET_SECONDS`: **0.25 s**. A run that falls further behind real time than
  this stops trying to catch up, and `lagResets()` counts how often that happened.

## What this package will not do

- **It will not read a real game controller.** Keyboard only. A controller needs a native input library.
- **It will not make a paced run differ from its replay.** The pacer only waits; it never changes what a
  tick computes. That is the point, and a test pins it.
- **It will not filter key auto-repeat on Linux (X11).** A held key can read as up for the odd loop. The
  recording is still what the sim saw. `xset r off` while driving avoids it.
- **It will not open a window in CI.**
