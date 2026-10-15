package org.horizon36596.simloop.fakehardware;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.junit.jupiter.api.Test;

import java.util.Locale;

/**
 * Pins {@link FakeMotor}'s {@code RUN_USING_ENCODER}: the hub's velocity loop, what {@code setPower} and
 * {@code setVelocity} send it, the whole-tick velocity readings and their {@link AngleUnit} overloads, and the
 * PIDF coefficient calls.
 *
 * <p>As in {@link FakeMotorRunToPositionTest}, <b>SDK exact</b> assertions (what is sent, rounding and
 * clipping, exceptions and messages, coefficient storage) came off the SDK 12.0.0 sources, and <b>model</b>
 * assertions (how fast and how closely the loop tracks) depend on {@link HubMotorControlLoop}'s
 * APPROXIMATIONS and carry headroom. Sim time only.
 */
class FakeMotorVelocityControlTest {

    private static final double TICK = 0.02;

    // A goBILDA 5203 312 rpm motor: 537.7 counts per rev, free speed 2796 ticks/s. Illustrative.
    private static final double FREE_SPEED = 2796.0;
    private static final double MAX_ACCEL = 12.0;
    private static final double CPR = 537.7;

    private static FakeMotor motor() {
        return new FakeMotor(FREE_SPEED, MAX_ACCEL, CPR);
    }

    private static void run(FakeMotor motor, double seconds) {
        for (int i = 0; i < Math.round(seconds / TICK); i++) {
            motor.update(TICK);
        }
    }

    // ---- Velocity tracking (model) ----

    @Test
    void aCommandedVelocityIsReachedQuicklyAndHeld() {
        FakeMotor motor = motor();
        motor.setVelocity(1400.0);

        double fastest = 0.0;
        for (int i = 0; i < 25; i++) {
            motor.update(TICK);
            fastest = Math.max(fastest, motor.getVelocity());
        }
        assertEquals(1400.0, motor.getVelocity(), 1400.0 * 0.02, "within 2% half a second after the command");
        assertTrue(fastest <= 1400.0 * 1.15, "with at most 15% overshoot on the way: " + fastest);

        run(motor, 2.0);
        assertEquals(1400.0, motor.getVelocity(), 1.0, "and then held to the tick per second");
    }

    @Test
    void theVelocityLoopHoldsSpeedUnderALoadThatSlowsAnOpenLoopMotor() {
        // The point of RUN_USING_ENCODER: the hub adds power until the speed is right. Open loop, the same
        // load simply costs speed.
        FakeMotor closedLoop = motor();
        closedLoop.setExternalTorqueFraction(-0.3);
        closedLoop.setVelocity(1400.0);
        run(closedLoop, 3.0);
        assertEquals(1400.0, closedLoop.getVelocity(), 1.0, "the loop makes up for the load");
        assertTrue(closedLoop.getAppliedPhysicalPower() > 0.75,
                "by applying about 0.5 + 0.3 of full power: " + closedLoop.getAppliedPhysicalPower());

        FakeMotor openLoop = motor();
        openLoop.setExternalTorqueFraction(-0.3);
        openLoop.setPower(0.5);
        run(openLoop, 3.0);
        assertEquals(0.2 * FREE_SPEED, openLoop.getVelocity(), 2.0, "open loop, 0.5 power minus 0.3 of load");
    }

    @Test
    void aTargetBeyondFreeSpeedSaturatesAtFreeSpeed() {
        FakeMotor motor = motor();
        motor.setVelocity(FREE_SPEED * 3.0);
        run(motor, 3.0);
        assertEquals(FREE_SPEED, motor.getVelocity(), 1.0, "a motor cannot outrun its own free speed");
    }

    @Test
    void aZeroVelocityGainSetLeavesTheMotorStanding() {
        // The coefficients are live: with every velocity gain zero the hub's loop outputs nothing.
        FakeMotor motor = motor();
        motor.setVelocityPIDFCoefficients(0, 0, 0, 0);
        motor.setVelocity(1400.0);
        run(motor, 2.0);
        assertEquals(0.0, motor.getVelocity(), 0.0);
    }

    @Test
    void aFeedforwardOfFullScaleOverFreeSpeedAloneTracksTheTarget() {
        // Model: the FTC tuning rule F = 32767 / maxTicksPerSecond gives exactly the right power with no
        // feedback, which is the unit convention HubMotorControlLoop assumes.
        FakeMotor motor = motor();
        motor.setVelocityPIDFCoefficients(0, 0, 0, 32767.0 / FREE_SPEED);
        motor.setVelocity(1400.0);
        run(motor, 3.0);
        assertEquals(1400.0, motor.getVelocity(), 1400.0 * 0.01);
    }

    // ---- What setPower and setVelocity send (SDK exact) ----

    @Test
    void setPowerInRunUsingEncoderIsAVelocityTargetOfPowerTimesMax() {
        // LynxDcMotorController.internalSetMotorPower: signum(p) * scale(|p|, 0, 1, 0, max), then (int).
        FakeMotor motor = motor();
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setPower(0.5);
        assertEquals(0.5, motor.getPower(), 0.0, "getPower() reports what was sent");

        run(motor, 3.0);
        assertEquals((int) (0.5 * FREE_SPEED), motor.getVelocity(), 1.0, "0.5 * 2796 truncated is 1398 ticks/s");
    }

    @Test
    void setVelocitySwitchesAnOpenLoopMotorToRunUsingEncoderAndReportsItAsPower() {
        FakeMotor motor = motor();
        motor.setVelocity(699.4);
        assertEquals(DcMotor.RunMode.RUN_USING_ENCODER, motor.getMode());
        assertEquals(699.0 / 2796.0, motor.getPower(), 1e-12,
                "the hub holds round(699.4) = 699 ticks/s, which reads back as power over the rounded max");
    }

    @Test
    void aVelocityBeyondTheHubsFieldIsClipped() {
        // setMotorVelocity clips to LynxSetMotorTargetVelocityCommand's +/-32767 before sending.
        FakeMotor motor = motor();
        motor.setVelocity(-1.0e6);
        assertEquals(-1.0, motor.getPower(), 0.0, "-32767 ticks/s is beyond -max, so it reads back as -1");
        run(motor, 3.0);
        assertEquals(-FREE_SPEED, motor.getVelocity(), 1.0);
    }

    // ---- Readings (SDK exact) ----

    @Test
    void getVelocityReportsWholeTicksPerSecond() {
        FakeMotor motor = motor();
        motor.setPower(0.37);
        for (int i = 0; i < 30; i++) {
            motor.update(TICK);
            double reported = motor.getVelocity();
            assertEquals(Math.rint(reported), reported, 0.0, "the hub reports an int: " + reported);
            assertEquals(motor.getVelocityTicksPerSecond(), reported, 0.5, "the nearest one to the modelled rate");
        }
    }

    @Test
    void angularVelocitiesConvertThroughTheConfiguredCountsPerRevolution() {
        FakeMotor motor = motor();
        motor.setVelocity(Math.PI, AngleUnit.RADIANS); // half a revolution per second
        run(motor, 3.0);

        int ticksPerSecond = (int) Math.round(CPR / 2.0); // 268.85 rounds to 269 when sent
        assertEquals(ticksPerSecond, motor.getVelocity(), 0.0, "the hub holds the rounded command exactly");
        assertEquals(ticksPerSecond / CPR * 360.0, motor.getVelocity(AngleUnit.DEGREES), 1e-9,
                "and the reading converts back from whole ticks: ticks / CPR * 360 degrees");
        assertEquals(Math.toRadians(ticksPerSecond / CPR * 360.0), motor.getVelocity(AngleUnit.RADIANS), 1e-12);

        FakeMotor reversed = motor();
        reversed.setDirection(DcMotor.Direction.REVERSE);
        reversed.setVelocity(180.0, AngleUnit.DEGREES);
        run(reversed, 3.0);
        assertEquals(ticksPerSecond / CPR * 360.0, reversed.getVelocity(AngleUnit.DEGREES), 1e-9,
                "a REVERSE handle reads its own command back with the same sign");
        assertTrue(reversed.getPhysicalVelocityTicksPerSecond() < 0.0, "while the shaft turns the other way");
    }

    // ---- PIDF coefficients (defaults: model; everything else: SDK exact) ----

    @Test
    void aFreshMotorReportsTheHubsDefaultGains() {
        FakeMotor motor = motor();
        assertCoefficients(new PIDFCoefficients(10, 3, 0, 0, MotorControlAlgorithm.PIDF),
                motor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER));
        assertCoefficients(new PIDFCoefficients(10, 0, 0, 0, MotorControlAlgorithm.PIDF),
                motor.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION));
    }

    @Test
    void gainsReadBackQuantizedToTheHubsFixedPoint() {
        // The hub stores (int)(|c| * 65536 + 0.5) * signum(c); getPIDFCoefficients reads it back over 65536.
        FakeMotor motor = motor();
        motor.setVelocityPIDFCoefficients(1.23456789, 0.1, -0.3, 12.3);

        PIDFCoefficients read = motor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        assertEquals(quantized(1.23456789), read.p, 0.0);
        assertEquals(quantized(0.1), read.i, 0.0);
        assertEquals(quantized(-0.3), read.d, 0.0);
        assertEquals(quantized(12.3), read.f, 0.0);
        assertEquals(MotorControlAlgorithm.PIDF, read.algorithm);
        assertTrue(read.i != 0.1, "0.1 is not a multiple of 1/65536, so what comes back is not what went in");
    }

    private static double quantized(double coefficient) {
        return ((int) (Math.abs(coefficient) * 65536.0 + 0.5) * (int) Math.signum(coefficient)) / 65536.0;
    }

    @Test
    void thePositionGainIsSetAloneAndLeavesTheVelocityGainsAlone() {
        FakeMotor motor = motor();
        motor.setPositionPIDFCoefficients(5.5);
        assertCoefficients(new PIDFCoefficients(5.5, 0, 0, 0, MotorControlAlgorithm.PIDF),
                motor.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION));
        assertCoefficients(new PIDFCoefficients(10, 3, 0, 0, MotorControlAlgorithm.PIDF),
                motor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER));
    }

    @Test
    void positionLoopIOrDOrFIsRefusedUnderThePidfAlgorithm() {
        // LynxDcMotorController.internalSetPIDFCoefficients: "avoid having the user give coefficients that
        // cause double-integration (our fw runs a position loop, that then runs a velocity loop)".
        FakeMotor motor = motor();
        PIDFCoefficients withI = new PIDFCoefficients(5, 0.1, 0, 0, MotorControlAlgorithm.PIDF);

        UnsupportedOperationException refused = assertThrows(UnsupportedOperationException.class,
                () -> motor.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, withI));
        assertEquals(String.format(Locale.getDefault(),
                        "setting of pidf coefficents not supported: motor=0 mode=RUN_TO_POSITION pidf=%s", withI),
                refused.getMessage(), "the SDK's message, typo included");
        assertCoefficients(new PIDFCoefficients(10, 0, 0, 0, MotorControlAlgorithm.PIDF),
                motor.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION), "and nothing changed");

        // The legacy algorithm is exempt, and setPIDCoefficients is how it is reached.
        motor.setPIDCoefficients(DcMotor.RunMode.RUN_TO_POSITION, new PIDCoefficients(10, 0.05, 0));
        assertCoefficients(new PIDFCoefficients(10, quantized(0.05), 0, 0, MotorControlAlgorithm.LegacyPID),
                motor.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION));
    }

    @Test
    @SuppressWarnings("deprecation")
    void onlyTheTwoClosedLoopModesHaveGains() {
        FakeMotor motor = motor();
        PIDFCoefficients gains = new PIDFCoefficients(1, 0, 0, 0);

        // Setting: validatePIDMode checks RunMode.isPIDMode() on the mode as given, before migrating it.
        IllegalArgumentException setRefused = assertThrows(IllegalArgumentException.class,
                () -> motor.setPIDFCoefficients(DcMotor.RunMode.RUN_WITHOUT_ENCODER, gains));
        assertEquals("motor 0: mode RUN_WITHOUT_ENCODER is invalid as PID Mode", setRefused.getMessage());
        motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODERS, gains);
        assertEquals(1.0, motor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).p, 0.0,
                "the deprecated alias is a PID mode, and is migrated to RUN_USING_ENCODER");

        // Reading: the hub command's constructor accepts exactly the two modes, unmigrated.
        IllegalArgumentException getRefused = assertThrows(IllegalArgumentException.class,
                () -> motor.getPIDFCoefficients(DcMotor.RunMode.STOP_AND_RESET_ENCODER));
        assertEquals("illegal mode: STOP_AND_RESET_ENCODER", getRefused.getMessage());
        assertThrows(IllegalArgumentException.class,
                () -> motor.getPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODERS));
    }

    @Test
    void setPidCoefficientsIsLegacyPidWithNoFeedforwardAndGetPidDropsTheFeedforward() {
        FakeMotor motor = motor();
        motor.setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDCoefficients(2, 0.5, 0.25));
        assertCoefficients(new PIDFCoefficients(2, 0.5, 0.25, 0, MotorControlAlgorithm.LegacyPID),
                motor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER));

        motor.setVelocityPIDFCoefficients(2, 0.5, 0.25, 11);
        PIDCoefficients pid = motor.getPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        assertEquals(2.0, pid.p, 0.0);
        assertEquals(0.5, pid.i, 0.0);
        assertEquals(0.25, pid.d, 0.0);
    }

    @Test
    void resetDeviceConfigurationForOpModePutsTheHubsDefaultGainsBack() {
        // LynxDcMotorController.resetDeviceConfigurationForOpMode restores the coefficients it read from the
        // hub before the first change, which on a fresh hub are the defaults.
        FakeMotor motor = motor();
        motor.setVelocityPIDFCoefficients(1, 2, 3, 4);
        motor.setPositionPIDFCoefficients(7);

        motor.resetDeviceConfigurationForOpMode();

        assertCoefficients(new PIDFCoefficients(10, 3, 0, 0, MotorControlAlgorithm.PIDF),
                motor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER));
        assertCoefficients(new PIDFCoefficients(10, 0, 0, 0, MotorControlAlgorithm.PIDF),
                motor.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION));
    }

    @Test
    void theReturnedGainsAreACopy() {
        FakeMotor motor = motor();
        motor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).p = 99;
        assertEquals(10.0, motor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).p, 0.0);
    }

    private static void assertCoefficients(PIDFCoefficients expected, PIDFCoefficients actual) {
        assertCoefficients(expected, actual, "");
    }

    private static void assertCoefficients(PIDFCoefficients expected, PIDFCoefficients actual, String message) {
        assertEquals(expected.p, actual.p, 0.0, message + " p");
        assertEquals(expected.i, actual.i, 0.0, message + " i");
        assertEquals(expected.d, actual.d, 0.0, message + " d");
        assertEquals(expected.f, actual.f, 0.0, message + " f");
        assertEquals(expected.algorithm, actual.algorithm, message + " algorithm");
    }
}
