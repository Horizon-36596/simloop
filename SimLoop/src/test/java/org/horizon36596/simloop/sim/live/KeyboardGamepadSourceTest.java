package org.horizon36596.simloop.sim.live;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import java.util.Arrays;
import java.util.HashSet;
import java.util.Set;

/**
 * Pins the keymap's behaviour a driver relies on. IMPLEMENTATION tier for the particular key choices
 * (a redesigned keymap may move them, and should update the docs table with them); INVARIANT tier for the
 * rules that make a keyboard behave like a gamepad at all - opposite keys cancel, only one gamepad driven
 * at a time, the session ends when asked.
 */
class KeyboardGamepadSourceTest {

    /** A keyboard a test controls: the keys down right now, and whether its window is open. */
    private static final class ScriptedKeyboard implements KeyboardGamepadSource.Keyboard {
        Set<String> down = new HashSet<>();
        boolean open = true;

        void hold(String... keys) {
            down = new HashSet<>(Arrays.asList(keys));
        }

        @Override public Set<String> keysDown() { return down; }
        @Override public boolean isOpen() { return open; }
    }

    @Test
    void implementation_wasdIsTheLeftStickWithUpNegativeLikeARealGamepad() {
        assertEquals(-1.0f, KeyboardGamepadSource.stateFor(keys("W")).axis("left_stick_y"));
        assertEquals(1.0f, KeyboardGamepadSource.stateFor(keys("S")).axis("left_stick_y"));
        assertEquals(-1.0f, KeyboardGamepadSource.stateFor(keys("A")).axis("left_stick_x"));
        assertEquals(1.0f, KeyboardGamepadSource.stateFor(keys("D")).axis("left_stick_x"));
    }

    @Test
    void invariant_oppositeKeysCancel() {
        assertEquals(GamepadState.AT_REST, KeyboardGamepadSource.stateFor(keys("W", "S", "A", "D")));
    }

    @Test
    void implementation_shiftGivesHalfTravelOnSticksAndTriggers() {
        GamepadState state = KeyboardGamepadSource.stateFor(keys("SHIFT", "W", "C"));
        assertEquals(-0.5f, state.axis("left_stick_y"));
        assertEquals(0.5f, state.axis("right_trigger"));
    }

    @Test
    void invariant_everyBindingDrivesARealGamepadControlAndEveryKeyIsListed() {
        for (KeyboardGamepadSource.KeyBinding binding : KeyboardGamepadSource.KEYMAP) {
            boolean isKnownControl = GamepadState.BUTTONS.contains(binding.control)
                    || GamepadState.STICK_AXES.contains(binding.control)
                    || GamepadState.TRIGGER_AXES.contains(binding.control);
            assertTrue(isKnownControl, binding + " drives something that is not a gamepad control");
            assertTrue(KeyboardGamepadSource.KEY_NAMES.contains(binding.key), binding.key + " is not in KEY_NAMES");
        }
    }

    @Test
    void invariant_tabMovesTheKeysToTheOtherGamepadOncePerPressAndTheOtherGoesToRest() {
        ScriptedKeyboard keyboard = new ScriptedKeyboard();
        KeyboardGamepadSource source = new KeyboardGamepadSource(keyboard, 100);

        keyboard.hold("1");
        GamepadInputTrack.Frame onPad1 = source.next(0);
        keyboard.hold("TAB", "1");
        GamepadInputTrack.Frame afterPress = source.next(1);
        keyboard.hold("TAB", "1");
        GamepadInputTrack.Frame tabStillHeld = source.next(2);

        assertTrue(onPad1.gamepad1.isDown("a") && onPad1.gamepad2.equals(GamepadState.AT_REST));
        assertTrue(afterPress.gamepad2.isDown("a") && afterPress.gamepad1.equals(GamepadState.AT_REST));
        assertEquals(2, source.drivenGamepad(), "holding TAB must not keep switching back and forth");
        assertTrue(tabStillHeld.gamepad2.isDown("a"));
    }

    @Test
    void invariant_theSessionEndsOnEscapeOnClosingTheWindowAndAtTheTickLimit() {
        ScriptedKeyboard keyboard = new ScriptedKeyboard();
        KeyboardGamepadSource source = new KeyboardGamepadSource(keyboard, 3);

        keyboard.hold("ESCAPE");
        assertNull(source.next(0), "escape ends the session");

        keyboard.hold();
        keyboard.open = false;
        assertNull(new KeyboardGamepadSource(keyboard, 3).next(0), "a closed window ends the session");

        keyboard.open = true;
        KeyboardGamepadSource limited = new KeyboardGamepadSource(keyboard, 3);
        assertEquals(0, limited.next(0).tick);
        assertEquals(2, limited.next(2).tick);
        assertNull(limited.next(3), "three ticks allowed, so tick 3 does not run");
    }

    @Test
    void keysTheKeymapDoesNotKnowAreIgnored() {
        assertEquals(GamepadState.AT_REST, KeyboardGamepadSource.stateFor(keys("F12", "semicolon")));
    }

    private static Set<String> keys(String... names) {
        return new HashSet<>(Arrays.asList(names));
    }
}
