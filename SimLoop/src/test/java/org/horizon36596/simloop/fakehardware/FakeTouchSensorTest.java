package org.horizon36596.simloop.fakehardware;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.List;

/**
 * Pins {@link FakeTouchSensor} and {@link SwitchPolarity}: a switch the sim presses reads as pressed to robot
 * code, whether that code asks for a {@link TouchSensor} (the SDK driver inverts the wire for it) or for a
 * raw {@link DigitalChannel} (robot code inverts it itself).
 *
 * <p>Tiers (domain R9): the polarity tests are INVARIANT tier - a wrong polarity is the classic bug these
 * fakes exist to catch before the robot does. The source and metadata tests are implementation tier.
 */
class FakeTouchSensorTest {

    // ---- the TouchSensor ----

    @Test
    void aFreshSensorIsReleased() {
        FakeTouchSensor touch = new FakeTouchSensor();

        assertFalse(touch.isPressed());
        assertEquals(0.0, touch.getValue(), 0.0);
    }

    @Test
    void invariant_pressedReadsPressedAndValueIsOneOrZero() {
        FakeTouchSensor touch = new FakeTouchSensor();

        touch.setPressed(true);
        assertTrue(touch.isPressed());
        assertEquals(1.0, touch.getValue(), 0.0, "RevTouchSensor.getValue is 1.0 when pressed");

        touch.setPressed(false);
        assertFalse(touch.isPressed());
        assertEquals(0.0, touch.getValue(), 0.0);
    }

    @Test
    void aSourceIsAskedOnUpdateAndNotBefore() {
        FakeTouchSensor touch = new FakeTouchSensor();
        boolean[] holding = {true};
        touch.setPressedSource(() -> holding[0]);

        assertFalse(touch.isPressed(), "a getter never asks the source");
        touch.update(0.02);
        assertTrue(touch.isPressed());

        holding[0] = false;
        assertTrue(touch.isPressed(), "still the last tick's answer");
        touch.update(0.02);
        assertFalse(touch.isPressed());
    }

    @Test
    void setPressedClearsTheSourceAndANullSourceKeepsTheLastAnswer() {
        FakeTouchSensor touch = new FakeTouchSensor();
        touch.setPressedSource(() -> true);
        touch.update(0.02);

        touch.setPressed(false);
        touch.update(0.02);
        assertFalse(touch.isPressed(), "setPressed cleared the source");

        touch.setPressedSource(() -> true);
        touch.update(0.02);
        touch.setPressedSource(null);
        touch.update(0.02);
        assertTrue(touch.isPressed(), "no source: the last answer stays");
    }

    @Test
    void theHardwareMapTicksTheSource() {
        FakeHardwareMap map = new FakeHardwareMap();
        FakeTouchSensor touch = new FakeTouchSensor();
        map.register("liftDown", touch);
        touch.setPressedSource(() -> true);

        map.updateAll(0.02);

        assertTrue(map.get(TouchSensor.class, "liftDown").isPressed());
    }

    @Test
    void itIdentifiesItselfAsRevTouchSensorDoes() {
        FakeTouchSensor touch = new FakeTouchSensor();

        assertEquals("REV Touch Sensor", touch.getDeviceName());
        assertEquals(HardwareDevice.Manufacturer.Lynx, touch.getManufacturer());
        assertEquals(1, touch.getVersion());
        assertTrue(touch.getConnectionInfo().endsWith("; digital channel 0"), touch.getConnectionInfo());
    }

    // ---- the same switch read as a raw DigitalChannel ----

    @Test
    void invariant_aRevTouchSensorPullsTheLineLowWhenPressed() {
        assertFalse(SwitchPolarity.PRESSED_IS_LOW.lineLevelWhen(true), "pressed is LOW");
        assertTrue(SwitchPolarity.PRESSED_IS_LOW.lineLevelWhen(false), "released is HIGH");
        assertTrue(SwitchPolarity.PRESSED_IS_LOW.isPressedAt(false));
        assertFalse(SwitchPolarity.PRESSED_IS_LOW.isPressedAt(true));

        assertTrue(SwitchPolarity.PRESSED_IS_HIGH.lineLevelWhen(true));
        assertFalse(SwitchPolarity.PRESSED_IS_HIGH.lineLevelWhen(false));
        assertTrue(SwitchPolarity.PRESSED_IS_HIGH.isPressedAt(true));
    }

    @Test
    void invariant_readingTheLineBackThroughTheSameWiringGivesThePressedStateBack() {
        for (SwitchPolarity polarity : SwitchPolarity.values()) {
            for (boolean pressed : new boolean[] {true, false}) {
                assertEquals(pressed, polarity.isPressedAt(polarity.lineLevelWhen(pressed)), polarity + " " + pressed);
            }
        }
    }

    @Test
    void invariant_robotCodeReadingTheChannelSeesWhatTheTouchSensorDriverWouldHaveSeen() {
        // The same physical REV Touch Sensor, configured two ways. As a TouchSensor, the SDK driver returns
        // !line; as a Digital Device, robot code gets the line and must write the ! itself.
        for (boolean pressed : new boolean[] {true, false}) {
            FakeTouchSensor asTouchSensor = new FakeTouchSensor();
            asTouchSensor.setPressed(pressed);
            FakeDigitalChannel asDigitalDevice = new FakeDigitalChannel();
            SwitchPolarity.PRESSED_IS_LOW.drive(asDigitalDevice, pressed);

            DigitalChannel robotView = asDigitalDevice;
            assertEquals(asTouchSensor.isPressed(), !robotView.getState(), "pressed=" + pressed);
        }
    }

    @Test
    void theChannelReportsTheWireOnlyWhileItIsAnInput() {
        FakeDigitalChannel limit = new FakeDigitalChannel();
        SwitchPolarity.PRESSED_IS_LOW.drive(limit, false);          // released: HIGH on the wire
        assertEquals(DigitalChannel.Mode.INPUT, limit.getMode(), "a fresh pin is an input");
        assertTrue(limit.getState());

        limit.setMode(DigitalChannel.Mode.OUTPUT);                  // robot code misconfigures the pin
        assertFalse(limit.getState(), "an output reports its own bit, cleared on the switch to OUTPUT");
        SwitchPolarity.PRESSED_IS_LOW.drive(limit, true);
        assertFalse(limit.getState(), "and the wire does not reach an output");

        limit.setMode(DigitalChannel.Mode.INPUT);
        SwitchPolarity.PRESSED_IS_LOW.drive(limit, false);
        assertTrue(limit.getState(), "back as an input it reads the wire again");
    }

    @Test
    void robotCodeCannotPressItsOwnSwitch() {
        FakeDigitalChannel limit = new FakeDigitalChannel();
        SwitchPolarity.PRESSED_IS_LOW.drive(limit, false);

        limit.setState(false);                                      // robot code writing an input pin

        assertFalse(SwitchPolarity.PRESSED_IS_LOW.isPressedAt(limit.getState()), "still released");
    }

    @Test
    void drivingNoChannelIsRefused() {
        assertThrows(IllegalArgumentException.class, () -> SwitchPolarity.PRESSED_IS_LOW.drive(null, true));
    }

    @Test
    void invariant_theSameScriptGivesTheSameReadingsEveryRun() {
        assertEquals(recordRun(), recordRun());
    }

    private static List<Boolean> recordRun() {
        FakeTouchSensor touch = new FakeTouchSensor();
        FakeDigitalChannel channel = new FakeDigitalChannel();
        int[] tick = {0};
        touch.setPressedSource(() -> tick[0] % 3 == 0);
        List<Boolean> readings = new ArrayList<>();
        for (tick[0] = 0; tick[0] < 100; tick[0]++) {
            touch.update(0.02);
            SwitchPolarity.PRESSED_IS_LOW.drive(channel, touch.isPressed());
            readings.add(touch.isPressed());
            readings.add(channel.getState());
        }
        return readings;
    }
}
