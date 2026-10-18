package org.horizon36596.simloop.desktop;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import java.awt.GraphicsEnvironment;
import java.awt.event.KeyEvent;
import java.util.Collections;
import java.util.HashMap;
import java.util.Map;

/**
 * Pins the key names the window reports, which the keymap in SimLoop's {@code KeyboardGamepadSource}
 * depends on. Runs headless: no test here opens a window. INVARIANT tier for "every name is one the
 * keymap can use"; IMPLEMENTATION tier for which physical keys are covered.
 */
class KeyboardWindowTest {

    @Test
    void lettersDigitsAndTheNamedKeysReportTheirNames() {
        assertEquals("W", KeyboardWindow.keyName(KeyEvent.VK_W));
        assertEquals("Z", KeyboardWindow.keyName(KeyEvent.VK_Z));
        assertEquals("1", KeyboardWindow.keyName(KeyEvent.VK_1));
        assertEquals("UP", KeyboardWindow.keyName(KeyEvent.VK_UP));
        assertEquals("SHIFT", KeyboardWindow.keyName(KeyEvent.VK_SHIFT));
        assertEquals("TAB", KeyboardWindow.keyName(KeyEvent.VK_TAB));
        assertEquals("ESCAPE", KeyboardWindow.keyName(KeyEvent.VK_ESCAPE));
        assertEquals("BACKSPACE", KeyboardWindow.keyName(KeyEvent.VK_BACK_SPACE));
    }

    @Test
    void keysTheWindowDoesNotReportHaveNoName() {
        assertNull(KeyboardWindow.keyName(KeyEvent.VK_F12));
        assertNull(KeyboardWindow.keyName(KeyEvent.VK_WINDOWS));
    }

    @Test
    void invariant_keyNamesListsExactlyWhatKeyNameCanReturn() {
        assertEquals(26 + 10 + 10, KeyboardWindow.REPORTED_KEY_NAMES.size());
        assertTrue(KeyboardWindow.REPORTED_KEY_NAMES.contains("A") && KeyboardWindow.REPORTED_KEY_NAMES.contains("ENTER"));
    }

    @Test
    void withoutADisplayOpeningSaysSoInsteadOfCrashingSomewhereElse() {
        if (!GraphicsEnvironment.isHeadless()) {
            return;     // this build forces headless; on a desktop run by hand there is nothing to check
        }
        IllegalStateException refused = assertThrows(IllegalStateException.class,
                () -> KeyboardWindow.open("test", Collections.<String>emptyList()));
        assertTrue(refused.getMessage().contains("no display"), refused.getMessage());
    }

    @Test
    void windowsAndMacOsAreDesktopsUnlessTheyAreCi() {
        Map<String, String> none = Collections.emptyMap();
        assertTrue(KeyboardWindow.looksLikeADesktop("Windows 11", none));
        assertTrue(KeyboardWindow.looksLikeADesktop("Mac OS X", none));
        assertFalse(KeyboardWindow.looksLikeADesktop("Windows 11", env("CI", "true")));
        assertFalse(KeyboardWindow.looksLikeADesktop("Mac OS X", env("CI", "1")));
    }

    @Test
    void linuxIsADesktopOnlyWithANamedDisplay() {
        assertFalse(KeyboardWindow.looksLikeADesktop("Linux", Collections.<String, String>emptyMap()));
        assertFalse(KeyboardWindow.looksLikeADesktop("Linux", env("DISPLAY", " ")));
        assertTrue(KeyboardWindow.looksLikeADesktop("Linux", env("DISPLAY", ":0")));
        assertTrue(KeyboardWindow.looksLikeADesktop("Linux", env("WAYLAND_DISPLAY", "wayland-0")));
        assertFalse(KeyboardWindow.looksLikeADesktop(null, Collections.<String, String>emptyMap()));
    }

    private static Map<String, String> env(String name, String value) {
        Map<String, String> environment = new HashMap<>();
        environment.put(name, value);
        return environment;
    }
}
