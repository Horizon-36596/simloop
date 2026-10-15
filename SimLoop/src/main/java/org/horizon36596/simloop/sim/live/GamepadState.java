package org.horizon36596.simloop.sim.live;

import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.Arrays;
import java.util.Collections;
import java.util.LinkedHashSet;
import java.util.Map;
import java.util.Set;
import java.util.SortedMap;
import java.util.SortedSet;
import java.util.TreeMap;
import java.util.TreeSet;

/**
 * Everything one FTC gamepad reports on one loop, as a plain value: which buttons are down and where
 * every stick, trigger and touchpad finger is.
 *
 * <p><b>The names are the SDK's own field names</b> on {@link Gamepad}, so a line in a recorded track
 * reads the same as the robot code that consumes it: {@code left_stick_y=-1.0 a dpad_right}. The
 * PlayStation names ({@code cross}, {@code circle}, {@code square}, {@code triangle}, {@code share},
 * {@code options}, {@code ps}) are not separate entries. On a real Driver Station they are aliases the SDK
 * derives from {@code a}, {@code b}, {@code x}, {@code y}, {@code back}, {@code start} and {@code guide}
 * when a gamepad packet arrives, and {@link #applyTo} goes through that same SDK code, so both names read
 * true in robot code whichever one it uses.
 *
 * <p><b>How it reaches the OpMode.</b> {@link #applyTo} fills a scratch {@link Gamepad} and hands it to
 * {@link Gamepad#copy}, which in SDK 12.0 is {@code fromByteArray(toByteArray())}: the path a real
 * Driver Station packet takes. That one call sets the PlayStation aliases, sets
 * {@code left_trigger_pressed} and {@code right_trigger_pressed} against the gamepad's own trigger
 * threshold, and updates the edge detection behind {@code aWasPressed()} and friends, exactly as on the
 * robot. Writing the fields one by one would skip all three.
 *
 * <p>Immutable, and equal to another state when every control reads the same. An axis at zero is stored
 * as absent, so there is exactly one way to write any state and two equal states always print the same
 * line.
 */
public final class GamepadState {

    /** The stick axes, in the SDK's range {@code [-1, 1]}. Up on a real stick is NEGATIVE y. */
    public static final Set<String> STICK_AXES = names(
            "left_stick_x", "left_stick_y", "right_stick_x", "right_stick_y");

    /** The two analog triggers, in the SDK's range {@code [0, 1]}. */
    public static final Set<String> TRIGGER_AXES = names("left_trigger", "right_trigger");

    /** The touchpad finger positions, in the SDK's range {@code [-1, 1]}. A keyboard never sets these. */
    public static final Set<String> TOUCHPAD_AXES = names(
            "touchpad_finger_1_x", "touchpad_finger_1_y", "touchpad_finger_2_x", "touchpad_finger_2_y");

    /**
     * Every button this records, by SDK field name. The PlayStation aliases are deliberately absent: see
     * the class comment.
     */
    public static final Set<String> BUTTONS = names(
            "dpad_up", "dpad_down", "dpad_left", "dpad_right",
            "a", "b", "x", "y", "guide", "start", "back",
            "left_bumper", "right_bumper", "left_stick_button", "right_stick_button",
            "touchpad", "touchpad_finger_1", "touchpad_finger_2");

    /** A gamepad nobody is touching: every button up, every axis at zero. */
    public static final GamepadState AT_REST = new GamepadState(
            new TreeMap<String, Float>(), new TreeSet<String>());

    private final SortedMap<String, Float> axes;
    private final SortedSet<String> buttonsDown;

    private GamepadState(SortedMap<String, Float> axes, SortedSet<String> buttonsDown) {
        this.axes = Collections.unmodifiableSortedMap(axes);
        this.buttonsDown = Collections.unmodifiableSortedSet(buttonsDown);
    }

    /**
     * A state from explicit values.
     *
     * @param axes        axis name to value; a name not in {@link #STICK_AXES}, {@link #TRIGGER_AXES} or
     *                    {@link #TOUCHPAD_AXES} is refused, and so is a value outside that axis's range or
     *                    a NaN. Zero values may be included and are dropped.
     * @param buttonsDown the buttons held down; a name not in {@link #BUTTONS} is refused
     * @return the state
     * @throws IllegalArgumentException naming the first bad name or value
     */
    public static GamepadState of(Map<String, Float> axes, Set<String> buttonsDown) {
        SortedMap<String, Float> checkedAxes = new TreeMap<>();
        for (Map.Entry<String, Float> axis : axes.entrySet()) {
            String name = axis.getKey();
            float value = axis.getValue();
            float min = requireAxisMinimum(name);
            if (Float.isNaN(value) || value < min || value > 1.0f) {
                throw new IllegalArgumentException("axis " + name + " must be in [" + min + ", 1.0], got "
                        + value);
            }
            if (value != 0.0f) {       // true for -0.0f as well, so negative zero is never stored
                checkedAxes.put(name, value);
            }
        }
        SortedSet<String> checkedButtons = new TreeSet<>();
        for (String button : buttonsDown) {
            if (!BUTTONS.contains(button)) {
                throw new IllegalArgumentException("unknown button '" + button + "'; known buttons are "
                        + BUTTONS);
            }
            checkedButtons.add(button);
        }
        return new GamepadState(checkedAxes, checkedButtons);
    }

    /** {@return the lower end of this axis's range: -1 for sticks and touchpad, 0 for triggers} */
    private static float requireAxisMinimum(String axis) {
        if (STICK_AXES.contains(axis) || TOUCHPAD_AXES.contains(axis)) {
            return -1.0f;
        }
        if (TRIGGER_AXES.contains(axis)) {
            return 0.0f;
        }
        throw new IllegalArgumentException("unknown axis '" + axis + "'; known axes are " + STICK_AXES + " "
                + TRIGGER_AXES + " " + TOUCHPAD_AXES);
    }

    /**
     * {@return the value of one axis, 0 when it is at rest}
     *
     * @param name the axis, e.g. {@code left_stick_y}
     */
    public float axis(String name) {
        requireAxisMinimum(name);
        Float value = axes.get(name);
        return value == null ? 0.0f : value;
    }

    /**
     * {@return whether one button is held down}
     *
     * @param button the button, e.g. {@code a}
     */
    public boolean isDown(String button) {
        if (!BUTTONS.contains(button)) {
            throw new IllegalArgumentException("unknown button '" + button + "'");
        }
        return buttonsDown.contains(button);
    }

    /**
     * {@return every axis that is not at zero, by name, in name order} The order is a promise: it is the
     * order a track line lists them in, which is what makes two equal states print the same line.
     */
    public SortedMap<String, Float> nonZeroAxes() {
        return axes;
    }

    /** {@return every button that is down, in name order - the order a track line lists them in} */
    public SortedSet<String> buttonsDown() {
        return buttonsDown;
    }

    /**
     * Make {@code target} read this state, by the same path a Driver Station packet takes (class comment).
     *
     * <p><b>Which gamepad it is stays as it was.</b> {@link Gamepad#copy} copies EVERYTHING in the packet,
     * including the four fields that say which physical gamepad this is: the user (1 or 2, read by
     * {@link Gamepad#getUser()}), the gamepad id, the packet timestamp, and the gamepad type. This class
     * records none of those, so the scratch packet is given the target's own values first; otherwise every
     * loop would reset the OpMode's {@code gamepad1} to a freshly constructed gamepad's user -1 and type
     * UNKNOWN. On a robot those four come from the Driver Station and do not change from loop to loop either.
     *
     * @param target the OpMode's gamepad, typically {@code gamepad1} or {@code gamepad2}
     */
    public void applyTo(Gamepad target) {
        Gamepad packet = new Gamepad();
        // Carry over which gamepad the target is (see above). The user field itself is protected; getUser()
        // is null for any value but 1 or 2, and then the packet keeps a new Gamepad's -1, which is what an
        // unassigned target holds too.
        if (target.getUser() != null) {
            packet.setUser(target.getUser());
        }
        packet.setGamepadId(target.getGamepadId());
        packet.setTimestamp(target.timestamp);
        packet.type = target.type();

        packet.left_stick_x = axis("left_stick_x");
        packet.left_stick_y = axis("left_stick_y");
        packet.right_stick_x = axis("right_stick_x");
        packet.right_stick_y = axis("right_stick_y");
        packet.left_trigger = axis("left_trigger");
        packet.right_trigger = axis("right_trigger");
        packet.touchpad_finger_1_x = axis("touchpad_finger_1_x");
        packet.touchpad_finger_1_y = axis("touchpad_finger_1_y");
        packet.touchpad_finger_2_x = axis("touchpad_finger_2_x");
        packet.touchpad_finger_2_y = axis("touchpad_finger_2_y");

        packet.dpad_up = buttonsDown.contains("dpad_up");
        packet.dpad_down = buttonsDown.contains("dpad_down");
        packet.dpad_left = buttonsDown.contains("dpad_left");
        packet.dpad_right = buttonsDown.contains("dpad_right");
        packet.a = buttonsDown.contains("a");
        packet.b = buttonsDown.contains("b");
        packet.x = buttonsDown.contains("x");
        packet.y = buttonsDown.contains("y");
        packet.guide = buttonsDown.contains("guide");
        packet.start = buttonsDown.contains("start");
        packet.back = buttonsDown.contains("back");
        packet.left_bumper = buttonsDown.contains("left_bumper");
        packet.right_bumper = buttonsDown.contains("right_bumper");
        packet.left_stick_button = buttonsDown.contains("left_stick_button");
        packet.right_stick_button = buttonsDown.contains("right_stick_button");
        packet.touchpad = buttonsDown.contains("touchpad");
        packet.touchpad_finger_1 = buttonsDown.contains("touchpad_finger_1");
        packet.touchpad_finger_2 = buttonsDown.contains("touchpad_finger_2");

        target.copy(packet);
    }

    @Override
    public boolean equals(Object other) {
        if (!(other instanceof GamepadState)) {
            return false;
        }
        GamepadState that = (GamepadState) other;
        return axes.equals(that.axes) && buttonsDown.equals(that.buttonsDown);
    }

    @Override
    public int hashCode() {
        return 31 * axes.hashCode() + buttonsDown.hashCode();
    }

    /** The same text a track line uses for this gamepad, e.g. {@code left_stick_y=-1.0 a}. */
    @Override
    public String toString() {
        return GamepadInputTrack.formatState(this);
    }

    private static Set<String> names(String... names) {
        return Collections.unmodifiableSet(new LinkedHashSet<>(Arrays.asList(names)));
    }
}
