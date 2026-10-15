package org.horizon36596.simloop.sim.live;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.internal.ui.GamepadUser;
import org.junit.jupiter.api.Test;

import java.util.Arrays;
import java.util.Collections;
import java.util.Map;
import java.util.TreeMap;
import java.util.TreeSet;

/**
 * Pins that a recorded state reaches robot code the way a Driver Station packet does: through the real
 * SDK {@link Gamepad}, aliases, trigger-pressed flags and edge detection included. INVARIANT tier.
 */
class GamepadStateTest {

    @Test
    void invariant_robotCodeSeesThePlayStationAliasesTooBecauseTheSdkDerivesThem() {
        GamepadState state = GamepadState.of(Collections.<String, Float>emptyMap(),
                new TreeSet<>(Arrays.asList("a", "b", "x", "y", "back", "start", "guide")));
        Gamepad gamepad = new Gamepad();

        state.applyTo(gamepad);

        assertTrue(gamepad.cross && gamepad.circle && gamepad.square && gamepad.triangle,
                "a/b/x/y must also read as cross/circle/square/triangle");
        assertTrue(gamepad.share && gamepad.options && gamepad.ps,
                "back/start/guide must also read as share/options/ps");
    }

    @Test
    void invariant_axesAndButtonsArriveExactlyAndTriggerPressedFollowsTheSdkThreshold() {
        Map<String, Float> axes = new TreeMap<>();
        axes.put("left_stick_x", -0.35f);
        axes.put("right_stick_y", 1.0f / 3.0f);
        axes.put("right_trigger", 0.9f);
        axes.put("left_trigger", 0.01f);
        GamepadState state = GamepadState.of(axes, new TreeSet<>(Arrays.asList("dpad_left", "right_bumper")));
        Gamepad gamepad = new Gamepad();

        state.applyTo(gamepad);

        assertEquals(-0.35f, gamepad.left_stick_x);
        assertEquals(1.0f / 3.0f, gamepad.right_stick_y);
        assertTrue(gamepad.dpad_left && gamepad.right_bumper);
        assertFalse(gamepad.dpad_right || gamepad.left_bumper || gamepad.a);
        assertTrue(gamepad.right_trigger_pressed, "0.9 is past the SDK's default trigger threshold");
        assertFalse(gamepad.left_trigger_pressed, "0.01 is not");
    }

    @Test
    void invariant_edgeDetectionSeesOnePressPerPressNotOnePerLoop() {
        GamepadState pressed = GamepadState.of(Collections.<String, Float>emptyMap(),
                Collections.singleton("a"));
        Gamepad gamepad = new Gamepad();

        pressed.applyTo(gamepad);
        assertTrue(gamepad.aWasPressed(), "the loop the button goes down is a press");
        pressed.applyTo(gamepad);
        assertFalse(gamepad.aWasPressed(), "holding it on the next loop is not a second press");
        GamepadState.AT_REST.applyTo(gamepad);
        assertTrue(gamepad.aWasReleased());
    }

    @Test
    void invariant_applyingAStateKeepsWhichGamepadItIs() {
        // Gamepad.copy() also copies user, id, timestamp and type, so a scratch packet would reset the
        // OpMode's gamepad to user -1 and type UNKNOWN. A real Driver Station packet for gamepad 1 says user 1.
        Gamepad gamepad = new Gamepad();
        gamepad.setUser(GamepadUser.ONE);
        gamepad.setGamepadId(7);
        gamepad.setTimestamp(1234L);
        gamepad.type = Gamepad.Type.SONY_PS4;

        GamepadState.of(Collections.<String, Float>emptyMap(), Collections.singleton("a")).applyTo(gamepad);

        assertEquals(GamepadUser.ONE, gamepad.getUser());
        assertEquals(7, gamepad.getGamepadId());
        assertEquals(1234L, gamepad.timestamp);
        assertEquals(Gamepad.Type.SONY_PS4, gamepad.type());
        assertTrue(gamepad.a, "and the state itself still arrives");
    }

    @Test
    void aStateAtRestLeavesTheGamepadAtRest() {
        Gamepad gamepad = new Gamepad();
        gamepad.a = true;
        gamepad.left_stick_y = 0.5f;

        GamepadState.AT_REST.applyTo(gamepad);

        assertTrue(gamepad.atRest(), "every stick and trigger at zero");
        assertFalse(gamepad.a);
    }

    @Test
    void zeroAndNegativeZeroAreTheSameStateAsNotListingTheAxis() {
        Map<String, Float> zero = new TreeMap<>();
        zero.put("left_stick_x", 0.0f);
        zero.put("left_stick_y", -0.0f);

        assertEquals(GamepadState.AT_REST, GamepadState.of(zero, Collections.<String>emptySet()));
    }

    @Test
    void unknownNamesAreRefused() {
        assertThrows(IllegalArgumentException.class,
                () -> GamepadState.of(Collections.<String, Float>emptyMap(), Collections.singleton("cross")));
        assertThrows(IllegalArgumentException.class,
                () -> GamepadState.of(Collections.singletonMap("left_stick_z", 0.5f),
                        Collections.<String>emptySet()));
    }
}
