package org.horizon36596.simloop.fakehardware;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.ServoImpl;

import org.junit.jupiter.api.Test;

/**
 * INVARIANT-tier — {@link FakeServo} behaves as the SDK's {@link ServoImpl} does, checked by running
 * {@code ServoImpl} itself rather than by restating its arithmetic.
 *
 * <p>{@code ServoImpl} touches nothing but its {@link ServoController}, so a small recording controller
 * stands it up on a desktop JVM, and every position, direction and scale-range case is compared against
 * what the real class actually sent. The comparison on the controller's position is <b>exact</b> (a delta
 * of zero): the fake calls the same {@code Range.clip}/{@code Range.scale} in the same order, so it must
 * round identically, bit for bit.
 *
 * <p>The PWM half ({@link PwmControl}) is pinned against {@code LynxServoController}'s behaviour read off
 * the SDK 12.0.0 bytecode, since {@code ServoImplEx} only forwards those calls to its controller and
 * running it would test the recording controller, not the SDK.
 */
class FakeServoTest {

    /** Positions worth checking: the ends, the middle, off-centre values, and two that must be clipped. */
    private static final double[] POSITIONS = {-0.5, 0.0, 0.1, 0.2, 1.0 / 3.0, 0.5, 0.73, 0.9, 1.0, 1.5};

    /** Scale ranges worth checking, including two whose endpoints the SDK clips rather than rejects. */
    private static final double[][] SCALE_RANGES = {{0.0, 1.0}, {0.2, 0.6}, {0.1, 0.9}, {-0.1, 0.5}, {0.35, 1.2}};

    /**
     * The smallest {@link ServoController} that lets a real {@link ServoImpl} run: it records the position
     * written to it and hands it back, which is what the hub controller does from {@code ServoImpl}'s point
     * of view while the last commanded position is known.
     */
    private static final class RecordingServoController implements ServoController {

        private double servoPosition = 0.0;

        @Override public void pwmEnable() { }
        @Override public void pwmDisable() { }
        @Override public PwmStatus getPwmStatus() { return PwmStatus.ENABLED; }
        @Override public void setServoPosition(int servo, double position) { this.servoPosition = position; }
        @Override public double getServoPosition(int servo) { return servoPosition; }
        @Override public void forgetLastKnownPosition(int servo) { }

        @Override public Manufacturer getManufacturer() { return Manufacturer.Other; }
        @Override public String getDeviceName() { return "RecordingServoController"; }
        @Override public String getConnectionInfo() { return "test"; }
        @Override public int getVersion() { return 1; }
        @Override public void resetDeviceConfigurationForOpMode() { }
        @Override public void close() { }
    }

    // ---- Differential: identical to the real ServoImpl, case for case ----

    @Test
    void sendsExactlyThePositionTheRealServoSendsAndReadsItBackTheSameWay() {
        for (Servo.Direction direction : Servo.Direction.values()) {
            for (double[] range : SCALE_RANGES) {
                RecordingServoController controller = new RecordingServoController();
                ServoImpl real = new ServoImpl(controller, 0);
                FakeServo fake = new FakeServo();
                real.setDirection(direction);
                fake.setDirection(direction);
                real.scaleRange(range[0], range[1]);
                fake.scaleRange(range[0], range[1]);

                for (double position : POSITIONS) {
                    real.setPosition(position);
                    fake.setPosition(position);
                    String scenario = "setPosition(" + position + ") with " + direction + " and scaleRange("
                            + range[0] + ", " + range[1] + ")";
                    assertEquals(controller.getServoPosition(0), fake.getScaledPosition(), 0.0,
                            scenario + ": the physical position must be bit-identical to what ServoImpl sent");
                    assertEquals(real.getPosition(), fake.getPosition(), 0.0,
                            scenario + ": getPosition() must be bit-identical to ServoImpl's");
                }
            }
        }
    }

    @Test
    void scaleRangeRejectsExactlyWhatTheRealServoRejects() {
        double[][] invalid = {{0.8, 0.2}, {0.5, 0.5}, {1.2, 1.5}, {-0.5, -0.1}};
        for (double[] range : invalid) {
            ServoImpl real = new ServoImpl(new RecordingServoController(), 0);
            assertThrows(IllegalArgumentException.class, () -> real.scaleRange(range[0], range[1]),
                    "precondition: the real ServoImpl rejects scaleRange(" + range[0] + ", " + range[1] + ")");
            assertThrows(IllegalArgumentException.class, () -> new FakeServo().scaleRange(range[0], range[1]),
                    "so the fake must reject scaleRange(" + range[0] + ", " + range[1] + ") too");
        }
    }

    @Test
    void resetDeviceConfigurationForOpModeRestoresTheSameDefaultsTheRealServoDoes() {
        RecordingServoController controller = new RecordingServoController();
        ServoImpl real = new ServoImpl(controller, 0);
        FakeServo fake = new FakeServo();
        real.setDirection(Servo.Direction.REVERSE);
        fake.setDirection(Servo.Direction.REVERSE);
        real.scaleRange(0.2, 0.6);
        fake.scaleRange(0.2, 0.6);

        real.resetDeviceConfigurationForOpMode();
        fake.resetDeviceConfigurationForOpMode();

        assertEquals(real.getDirection(), fake.getDirection(), "the direction goes back to FORWARD");
        real.setPosition(0.3);
        fake.setPosition(0.3);
        assertEquals(controller.getServoPosition(0), fake.getScaledPosition(), 0.0,
                "and the scale range goes back to [0, 1], so 0.3 is sent as 0.3");
    }

    @Test
    void reportsTheSameVersionTheRealServoReports() {
        assertEquals(new ServoImpl(new RecordingServoController(), 0).getVersion(), new FakeServo().getVersion());
    }

    // ---- PwmControl, as LynxServoController behaves ----

    @Test
    void aFreshServoHasPwmOffAndTheSdkDefaultRange() {
        FakeServo servo = new FakeServo();
        assertFalse(servo.isPwmEnabled(), "the hub floats every servo port at start-up");
        assertSame(PwmControl.PwmRange.defaultRange, servo.getPwmRange(), "the SDK's 600-2400 us default");
        assertEquals(0.0, servo.getPulseWidth(), 0.0, "nothing has been sent yet");
    }

    @Test
    void aPositionCommandEnablesPwmAndSendsTheWholeMicrosecondPulseWidthItMapsTo() {
        FakeServo servo = new FakeServo();
        servo.setPosition(0.5);
        assertTrue(servo.isPwmEnabled(), "LynxServoController enables PWM whenever it sends a pulse width");
        assertEquals(1500.0, servo.getPulseWidth(), 0.0, "0.5 of 600-2400 us is 1500 us");

        servo.setPwmRange(new PwmControl.PwmRange(500, 2500));
        servo.setPosition(1.0 / 3.0);
        assertEquals(1166.0, servo.getPulseWidth(), 0.0,
                "a third of 500-2500 us is 1166.67 us, truncated to whole microseconds as the command carries it");
    }

    @Test
    void disablingPwmIsReportedAndTheNextPositionTurnsItBackOn() {
        FakeServo servo = new FakeServo();
        servo.setPosition(0.4);
        servo.setPwmDisable();
        assertFalse(servo.isPwmEnabled());
        servo.setPosition(0.6);
        assertTrue(servo.isPwmEnabled());
        servo.setPwmDisable();
        servo.setPwmEnable();
        assertTrue(servo.isPwmEnabled(), "and setPwmEnable turns it on directly");
    }

    @Test
    void aRawPulseWidthBecomesThePositionTheHubReconstructsFromIt() {
        FakeServo servo = new FakeServo();
        servo.setPulseWidth(2100.9);
        assertEquals(2100.0, servo.getPulseWidth(), 0.0, "truncated to whole microseconds");
        assertTrue(servo.isPwmEnabled());
        assertEquals((2100.0 - 600.0) / (2400.0 - 600.0), servo.getPosition(), 1e-12,
                "the position reads back as that pulse width mapped through the PWM range");
    }

    @Test
    void isReachableAsPwmControlThroughTheHardwareMap() {
        // On a real robot the servo is a ServoImplEx; robot code that asks for the PwmControl surface gets
        // it. Asking for the concrete ServoImplEx class does not work in sim (FakeServo class javadoc).
        FakeHardwareMap map = new FakeHardwareMap();
        FakeServo servo = new FakeServo();
        map.register("claw", servo);
        assertSame(servo, map.get(PwmControl.class, "claw"));
    }
}
