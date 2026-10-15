package org.horizon36596.simloop.sim.live;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.TreeMap;
import java.util.TreeSet;

/**
 * A person at a keyboard, standing in for an FTC gamepad. It reads which keys are down once per loop and
 * turns them into a {@link GamepadState} through {@link #KEYMAP}.
 *
 * <p><b>One keyboard, two gamepads.</b> The keys drive one gamepad at a time, gamepad 1 to start with;
 * {@link #SWITCH_GAMEPAD_KEY} moves them to the other one. The gamepad the keys are not on reads
 * {@link GamepadState#AT_REST}, so switching is also how a person lets go of everything on one gamepad at
 * once.
 *
 * <p><b>Where the keys come from is not this class's business.</b> A {@link Keyboard} is anything that can
 * say which keys are down: a desktop window in a live session, a script in a test. The key names are plain
 * strings - letters and digits as themselves, and {@code UP DOWN LEFT RIGHT SHIFT TAB ESCAPE ENTER
 * BACKSPACE} - so this class needs no windowing library and runs in any JVM. See {@link #KEY_NAMES}.
 *
 * <p>Deterministic given the key sequence (domain R5): nothing here reads a clock. The only
 * non-determinism in a live session is the person, and the recorded track captures exactly what this
 * class produced from them.
 */
public final class KeyboardGamepadSource implements GamepadInputSource {

    /** Something that can report which keys are held down right now. */
    public interface Keyboard {
        /** {@return the names of the keys held down at this instant; see {@link #KEY_NAMES}} */
        Set<String> keysDown();

        /** {@return false once the person has closed the keyboard window; the session then ends} */
        boolean isOpen();
    }

    /** One key's effect on the gamepad it is driving. */
    public static final class KeyBinding {
        /** The key, by name (see {@link #KEY_NAMES}). */
        public final String key;
        /** The {@link GamepadState} control it drives: an axis name or a button name. */
        public final String control;
        /** For an axis, the value it pushes the axis to while held; unused for a button. */
        public final float axisValue;

        private KeyBinding(String key, String control, float axisValue) {
            this.key = key;
            this.control = control;
            this.axisValue = axisValue;
        }

        /** {@return true if this key drives an axis rather than a button} */
        public boolean isAxis() {
            return !GamepadState.BUTTONS.contains(control);
        }

        static KeyBinding axis(String key, String axis, float value) {
            return new KeyBinding(key, axis, value);
        }

        static KeyBinding button(String key, String button) {
            return new KeyBinding(key, button, 0.0f);
        }

        /** e.g. {@code W -> left_stick_y -1.0} or {@code 1 -> a}. */
        @Override
        public String toString() {
            return key + " -> " + control + (isAxis() ? " " + axisValue : "");
        }
    }

    /** Held: sticks and triggers go only halfway, for fine positioning. */
    public static final String HALF_TRAVEL_KEY = "SHIFT";

    /**
     * How far a stick or trigger goes while {@link #HALF_TRAVEL_KEY} is held, as a fraction of full travel.
     * SET BY HAND: half is easy to predict and fine enough to line up on a ball.
     */
    public static final float HALF_TRAVEL_FRACTION = 0.5f;

    /** Pressed: the keys stop driving this gamepad and start driving the other one. */
    public static final String SWITCH_GAMEPAD_KEY = "TAB";

    /** Pressed: the session ends after the loop that is running. Closing the window does the same. */
    public static final String END_SESSION_KEY = "ESCAPE";

    /**
     * The keymap. Sticks on the two hand clusters, the d-pad on the arrow keys, the face buttons on the
     * number row. The stick directions follow a real gamepad: pushing a stick UP is NEGATIVE y.
     *
     * <p>Opposite keys held together cancel ({@code W} and {@code S} leaves {@code left_stick_y} at 0).
     * The PlayStation names in the comments are the aliases robot code may read instead; see
     * {@link GamepadState}.
     */
    public static final List<KeyBinding> KEYMAP = Collections.unmodifiableList(Arrays.asList(
            KeyBinding.axis("W", "left_stick_y", -1.0f),
            KeyBinding.axis("S", "left_stick_y", 1.0f),
            KeyBinding.axis("A", "left_stick_x", -1.0f),
            KeyBinding.axis("D", "left_stick_x", 1.0f),
            KeyBinding.axis("I", "right_stick_y", -1.0f),
            KeyBinding.axis("K", "right_stick_y", 1.0f),
            KeyBinding.axis("J", "right_stick_x", -1.0f),
            KeyBinding.axis("L", "right_stick_x", 1.0f),
            KeyBinding.axis("Z", "left_trigger", 1.0f),
            KeyBinding.axis("C", "right_trigger", 1.0f),
            KeyBinding.button("UP", "dpad_up"),
            KeyBinding.button("DOWN", "dpad_down"),
            KeyBinding.button("LEFT", "dpad_left"),
            KeyBinding.button("RIGHT", "dpad_right"),
            KeyBinding.button("1", "a"),                    // cross
            KeyBinding.button("2", "b"),                    // circle
            KeyBinding.button("3", "x"),                    // square
            KeyBinding.button("4", "y"),                    // triangle
            KeyBinding.button("Q", "left_bumper"),
            KeyBinding.button("E", "right_bumper"),
            KeyBinding.button("F", "left_stick_button"),
            KeyBinding.button("H", "right_stick_button"),
            KeyBinding.button("ENTER", "start"),            // options
            KeyBinding.button("BACKSPACE", "back")));       // share

    /**
     * Every key name a {@link Keyboard} may report that this class does something with. A keyboard
     * implementation must use exactly these names.
     */
    public static final Set<String> KEY_NAMES;

    static {
        TreeSet<String> names = new TreeSet<>();
        for (KeyBinding binding : KEYMAP) {
            names.add(binding.key);
        }
        names.add(HALF_TRAVEL_KEY);
        names.add(SWITCH_GAMEPAD_KEY);
        names.add(END_SESSION_KEY);
        KEY_NAMES = Collections.unmodifiableSet(names);
    }

    private final Keyboard keyboard;
    private final int maxTicks;
    private int drivenGamepad = 1;
    private boolean switchKeyWasDown = false;

    /**
     * A source that drives one gamepad at a time from the keyboard, gamepad 1 to start with.
     *
     * @param keyboard where the keys come from
     * @param maxTicks the session ends after this many loops even if nobody stops it, so a session left
     *                 running cannot fill the disk; must be positive
     */
    public KeyboardGamepadSource(Keyboard keyboard, int maxTicks) {
        if (maxTicks <= 0) {
            throw new IllegalArgumentException("maxTicks must be positive, got " + maxTicks);
        }
        this.keyboard = keyboard;
        this.maxTicks = maxTicks;
    }

    /** {@return 1 or 2: the gamepad the keys are driving now} */
    public int drivenGamepad() {
        return drivenGamepad;
    }

    @Override
    public GamepadInputTrack.Frame next(int tickIndex) {
        if (tickIndex >= maxTicks || !keyboard.isOpen()) {
            return null;
        }
        // One snapshot per loop, so every decision below is made from the same instant.
        Set<String> keys = new TreeSet<>(keyboard.keysDown());
        if (keys.contains(END_SESSION_KEY)) {
            return null;
        }

        boolean switchKeyDown = keys.contains(SWITCH_GAMEPAD_KEY);
        if (switchKeyDown && !switchKeyWasDown) {
            drivenGamepad = drivenGamepad == 1 ? 2 : 1;
        }
        switchKeyWasDown = switchKeyDown;

        GamepadState driven = stateFor(keys);
        return drivenGamepad == 1
                ? new GamepadInputTrack.Frame(tickIndex, driven, GamepadState.AT_REST)
                : new GamepadInputTrack.Frame(tickIndex, GamepadState.AT_REST, driven);
    }

    /**
     * The keymap itself: which gamepad state a set of held keys means.
     *
     * @param keysDown key names, see {@link #KEY_NAMES}; names this class does not know are ignored
     * @return the state of the gamepad the keys are driving
     */
    public static GamepadState stateFor(Set<String> keysDown) {
        float travel = keysDown.contains(HALF_TRAVEL_KEY) ? HALF_TRAVEL_FRACTION : 1.0f;
        Map<String, Float> axes = new TreeMap<>();
        List<String> buttons = new ArrayList<>();
        for (KeyBinding binding : KEYMAP) {
            if (!keysDown.contains(binding.key)) {
                continue;
            }
            if (binding.isAxis()) {
                Float sum = axes.get(binding.control);
                axes.put(binding.control, (sum == null ? 0.0f : sum) + binding.axisValue * travel);
            } else {
                buttons.add(binding.control);
            }
        }
        return GamepadState.of(axes, new TreeSet<>(buttons));
    }
}
