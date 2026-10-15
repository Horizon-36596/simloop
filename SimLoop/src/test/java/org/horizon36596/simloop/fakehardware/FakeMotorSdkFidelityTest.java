package org.horizon36596.simloop.fakehardware;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.qualcomm.robotcore.exception.TargetPositionNotSetException;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.junit.jupiter.api.Test;

/**
 * Pins the {@link FakeMotor} behaviours that an adversarial review on 2026-09-17 found were accepted and
 * then ignored — {@code setMode}, {@code setVelocity} and {@code setMotorDisable} each took a command,
 * reported success, and changed nothing about what the shaft did.
 *
 * <p>Every expectation here was read off the FTC SDK 11.2.1 sources ({@code DcMotorImpl},
 * {@code DcMotorImplEx}, {@code LynxDcMotorController}) rather than off what seemed reasonable. That is the
 * whole point of the fidelity rule: the fake is only worth having if a divergence from the real device is a
 * test failure.
 */
class FakeMotorSdkFidelityTest {

    private static final double MAX_VELOCITY_TICKS_PER_SECOND = 1000.0;
    private static final double MAX_ACCEL = 10.0;
    private static final double TICK = 0.02;

    private static FakeMotor spunUpMotor() {
        FakeMotor motor = new FakeMotor(MAX_VELOCITY_TICKS_PER_SECOND, MAX_ACCEL);
        motor.setPower(1.0);
        for (int i = 0; i < 50; i++) {
            motor.update(TICK);
        }
        return motor;
    }

    @Test
    void aDisabledMotorStopsDrivingEvenThoughItsCommandedPowerIsUnchanged() {
        FakeMotor motor = spunUpMotor();
        double positionWhenDisabled = motor.getPhysicalPositionTicks();

        motor.setMotorDisable();
        for (int i = 0; i < 200; i++) {
            motor.update(TICK);
        }

        assertFalse(motor.isMotorEnabled(), "the channel reports itself disabled");
        assertEquals(1.0, motor.getPower(), 1e-9,
                "the commanded power survives, as it does on a real controller — only the channel is off");
        assertEquals(0.0, motor.getSpeed(), 1e-6, "the shaft coasts to a stop with no drive behind it");
        // It coasts rather than stopping dead, so it travels a little further before it settles. What must
        // not happen is what used to: the encoder climbing forever while the motor reported itself disabled.
        double travelledWhileDisabled = motor.getPhysicalPositionTicks() - positionWhenDisabled;
        assertTrue(travelledWhileDisabled < MAX_VELOCITY_TICKS_PER_SECOND * TICK * 20,
                "a disabled motor coasts to a stop, it does not keep driving; travelled "
                        + travelledWhileDisabled + " ticks");
    }

    @Test
    void aFreshPowerCommandReEnablesTheChannelTheWayTheRealControllerDoes() {
        FakeMotor motor = spunUpMotor();
        motor.setMotorDisable();
        assertFalse(motor.isMotorEnabled());

        motor.setPower(0.5);

        assertTrue(motor.isMotorEnabled(),
                "LynxDcMotorController.internalSetMotorPower enables the channel on every power command");
    }

    @Test
    void stopAndResetEncoderStopsTheShaftAndZeroesTheCount() {
        FakeMotor motor = spunUpMotor();
        assertTrue(motor.getPhysicalPositionTicks() > 0.0, "precondition: the motor is actually turning");

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        assertEquals(DcMotor.RunMode.STOP_AND_RESET_ENCODER, motor.getMode(),
                "the mode is reported back, not swallowed");
        assertEquals(0, motor.getCurrentPosition(), "the encoder is reset");
        assertEquals(0.0, motor.getSpeed(), 1e-9, "the shaft is stopped");

        // And it STAYS stopped: the real controller refuses to send a power command while in this mode, so
        // nothing can start the motor again until the mode changes.
        for (int i = 0; i < 100; i++) {
            motor.update(TICK);
        }
        assertEquals(0, motor.getCurrentPosition(), "the count stays at zero while the mode holds");
        assertEquals(1.0, motor.getPower(), 1e-9,
                "the API-visible power is preserved across the reset, matching LynxDcMotorController");
    }

    @Test
    void aMotorThatIsNotDrivingDoesNotDrawStallCurrent() {
        // Current draw is how robot code detects a jam, so the shaft and the ammeter have to tell the same
        // story. Both ways of stopping a motor leave getPower() reporting the commanded value, and a naive
        // current model reads that value and reports full stall current for a motor that is being held
        // still — which a stall detector cannot tell apart from a real jam.
        FakeMotor stoppedByMode = spunUpMotor();
        stoppedByMode.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        stoppedByMode.update(TICK);
        assertEquals(0.0, stoppedByMode.getCurrent(CurrentUnit.AMPS), 1e-9,
                "a motor the mode is holding stopped draws nothing");

        FakeMotor stoppedByDisable = spunUpMotor();
        stoppedByDisable.setMotorDisable();
        for (int i = 0; i < 200; i++) {
            stoppedByDisable.update(TICK);
        }
        assertEquals(0.0, stoppedByDisable.getCurrent(CurrentUnit.AMPS), 1e-6,
                "and neither does one whose channel is disabled");

        FakeMotor driving = spunUpMotor();
        assertTrue(driving.getCurrent(CurrentUnit.AMPS) > 0.0,
                "while a motor that IS driving still draws current — the check above is not vacuous");
    }

    @Test
    void resettingTheEncoderAlsoDropsAStalePlantShaftSpeed() {
        // A mechanism plant can take authority over the shaft speed. That measurement is stale the instant
        // the encoder is reset, because this mode holds the shaft still; left in place it made getSpeed()
        // report a turning shaft on a stopped motor.
        FakeMotor motor = spunUpMotor();
        motor.setMeasuredShaftSpeedFraction(0.5);

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.update(TICK);

        assertEquals(0.0, motor.getSpeed(), 1e-9, "the shaft reads as stopped, not as still turning at 0.5");
    }

    @Test
    void theModeIsWhateverWasSetRatherThanAlwaysRunWithoutEncoder() {
        FakeMotor motor = new FakeMotor(MAX_VELOCITY_TICKS_PER_SECOND, MAX_ACCEL);
        assertEquals(DcMotor.RunMode.RUN_WITHOUT_ENCODER, motor.getMode(), "the mode a fresh motor is in");

        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        assertEquals(DcMotor.RunMode.RUN_USING_ENCODER, motor.getMode());

        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        assertEquals(DcMotor.RunMode.RUN_WITHOUT_ENCODER, motor.getMode());
    }

    @Test
    void runToPositionWithNoTargetEverSetThrowsWhatTheSdkThrows() {
        // LynxDcMotorController.setMotorMode records the mode, re-issues the power, and enables the channel;
        // the hub NACKs that enable (MOTOR_NOT_CONFIG_BEFORE_ENABLED) and the SDK rethrows it. The full
        // RUN_TO_POSITION behaviour is pinned in FakeMotorRunToPositionTest.
        FakeMotor motor = new FakeMotor(MAX_VELOCITY_TICKS_PER_SECOND, MAX_ACCEL);

        TargetPositionNotSetException refused = assertThrows(TargetPositionNotSetException.class,
                () -> motor.setMode(DcMotor.RunMode.RUN_TO_POSITION));

        assertEquals("Failed to enable motor. You must set a target position before switching to "
                + "RUN_TO_POSITION mode", refused.getMessage(), "the SDK's own exception and message");
        assertEquals(DcMotor.RunMode.RUN_TO_POSITION, motor.getMode(), "the mode was recorded before the throw");
        assertFalse(motor.isMotorEnabled(), "and the channel the hub refused to enable stays off");
    }

    @Test
    void setVelocityActuallyTurnsTheShaft() {
        FakeMotor motor = new FakeMotor(MAX_VELOCITY_TICKS_PER_SECOND, MAX_ACCEL);

        motor.setVelocity(500.0);
        for (int i = 0; i < 100; i++) {
            motor.update(TICK);
        }

        assertEquals(DcMotor.RunMode.RUN_USING_ENCODER, motor.getMode(),
                "LynxDcMotorController.setMotorVelocity switches an unsuitable mode to RUN_USING_ENCODER");
        assertTrue(motor.isMotorEnabled(), "and enables the channel");
        assertEquals(500.0, motor.getVelocity(), 1.0,
                "the shaft settles at the commanded ticks/second rather than standing still");
    }

    @Test
    void setVelocityRespectsTheConfiguredDirection() {
        FakeMotor reversed = new FakeMotor(MAX_VELOCITY_TICKS_PER_SECOND, MAX_ACCEL);
        reversed.setDirection(DcMotor.Direction.REVERSE);

        reversed.setVelocity(500.0);
        for (int i = 0; i < 100; i++) {
            reversed.update(TICK);
        }

        assertEquals(500.0, reversed.getVelocity(), 1.0,
                "the SDK-reported velocity is in the motor's logical frame, so it reads back positive");
        assertTrue(reversed.getPhysicalVelocityTicksPerSecond() < 0.0,
                "while the physical shaft really does turn the other way");
    }

    @Test
    void setVelocitySaturatesAtTheMotorsFreeSpeedRatherThanExceedingIt() {
        FakeMotor motor = new FakeMotor(MAX_VELOCITY_TICKS_PER_SECOND, MAX_ACCEL);

        motor.setVelocity(MAX_VELOCITY_TICKS_PER_SECOND * 10.0);
        for (int i = 0; i < 200; i++) {
            motor.update(TICK);
        }

        assertEquals(MAX_VELOCITY_TICKS_PER_SECOND, motor.getVelocity(), 1.0,
                "a motor cannot be commanded past its own free speed, in sim or on the field");
    }

    @Test
    void aPowerCommandWhileTheEncoderIsResettingDoesNotBringADisabledChannelBack() {
        // LynxDcMotorController.internalSetMotorPower builds command = null in STOP_AND_RESET_ENCODER
        // ("setting motor power in this mode doesn't do anything") and only calls internalSetMotorEnable
        // when it built a command. So this one mode is the exception to "a power command re-enables".
        FakeMotor motor = spunUpMotor();
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMotorDisable();

        motor.setPower(1.0);

        assertFalse(motor.isMotorEnabled(),
                "a power command is discarded outright while the encoder is resetting, so it cannot "
                        + "re-enable the channel either");
    }

    @Test
    void anyRealModeChangeBringsADisabledChannelBack() {
        // Every route through LynxDcMotorController.setMotorMode ends in a power command, and a power
        // command that is actually sent calls internalSetMotorEnable(true). Entering STOP_AND_RESET_ENCODER
        // counts, because its internalSetMotorPower(motor, 0) runs BEFORE the new mode is recorded and so
        // is built in the outgoing mode.
        FakeMotor intoReset = spunUpMotor();
        intoReset.setMotorDisable();
        intoReset.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        assertTrue(intoReset.isMotorEnabled(), "entering the reset mode re-enables the channel");

        FakeMotor outOfReset = spunUpMotor();
        outOfReset.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outOfReset.setMotorDisable();
        outOfReset.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        assertTrue(outOfReset.isMotorEnabled(), "and so does leaving it");

        // Setting the mode it is ALREADY in is a no-op on the real controller, which guards its whole body
        // on "if the mode is not already this one" -- so it must not re-enable either.
        FakeMotor sameMode = spunUpMotor();
        sameMode.setMotorDisable();
        sameMode.setMode(sameMode.getMode());
        assertFalse(sameMode.isMotorEnabled(),
                "re-setting the mode it is already in changes nothing, including the enable");
    }

    @Test
    void aRunToPositionTargetIsRecordedInEveryModeEvenThoughNothingDrivesToIt() {
        // DcMotorImpl accepts and reports a target in any mode, so subsystem code that sets one during
        // configuration reads back the same value here as on the robot. Only RUN_TO_POSITION acts on it, so
        // in any other mode it is recorded and nothing drives toward it.
        FakeMotor motor = new FakeMotor(MAX_VELOCITY_TICKS_PER_SECOND, MAX_ACCEL);

        motor.setTargetPosition(750);
        motor.setTargetPositionTolerance(8);

        assertEquals(750, motor.getTargetPosition(), "the target round-trips outside RUN_TO_POSITION");
        assertEquals(8, motor.getTargetPositionTolerance(), "and so does the tolerance");

        motor.setPower(1.0);
        for (int i = 0; i < 100; i++) {
            motor.update(TICK);
        }
        assertTrue(motor.getCurrentPosition() > 750,
                "and nothing stops at it outside RUN_TO_POSITION");
        assertFalse(motor.isBusy(), "isBusy is false outside RUN_TO_POSITION, as LynxDcMotorController reports");
    }

    // ---- 2026-09-26 review: SDK facts below were read off the 12.0.0 bytecode (DcMotorImpl, DcMotorImplEx,
    //      LynxDcMotorController) with javap. They are asserted as plain values rather than differentially,
    //      because a real DcMotorImpl cannot be built on a desktop JVM: its constructor logs through
    //      RobotLog.v, which calls android.util.Log.println, and that is "not mocked" off a device. ----

    @Test
    void resettingTheEncoderZeroesABorrowedEncoderToo() {
        // LynxDcMotorController.setMotorMode(STOP_AND_RESET_ENCODER) sends LynxResetMotorEncoderCommand for
        // the port, which zeroes whichever encoder is physically plugged in -- including one counting another
        // mechanism. That mechanism is driven elsewhere, so it keeps moving and the count carries on from 0.
        FakeMotor port = new FakeMotor(MAX_VELOCITY_TICKS_PER_SECOND, MAX_ACCEL);
        port.setEncoderMeasuringAnotherMechanism(800.0, 0.0);
        assertEquals(800, port.getCurrentPosition(), "precondition: the port reports the borrowed count");

        port.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        assertEquals(0, port.getCurrentPosition(), "the reset zeroes the borrowed encoder");

        port.setEncoderMeasuringAnotherMechanism(850.0, 0.0);
        assertEquals(50, port.getCurrentPosition(), "and it counts on from zero as the other mechanism moves");

        port.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        assertEquals(50, port.getCurrentPosition(),
                "re-setting the mode it is already in sends no second reset, as on the real controller");

        port.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        port.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        assertEquals(0, port.getCurrentPosition(), "while a real second entry into the mode resets again");
    }

    @Test
    void aResetMadeBeforeTheFirstBorrowedCountZeroesAtThatFirstCount() {
        // The order a real init() runs in: reset the encoders, and only then does the sim's first tick inject
        // the other mechanism's count. On the robot the reset zeroed that encoder, so it must read zero here.
        FakeMotor port = new FakeMotor(MAX_VELOCITY_TICKS_PER_SECOND, MAX_ACCEL);
        port.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        port.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        port.setEncoderMeasuringAnotherMechanism(800.0, 0.0);
        assertEquals(0, port.getCurrentPosition(), "the pending reset zeroes at the first injected count");

        port.setEncoderMeasuringAnotherMechanism(850.0, 0.0);
        assertEquals(50, port.getCurrentPosition(), "and the count carries on from there");
    }

    @Test
    void setPowerFloatAndGetPowerFloatAreWhatDcMotorImplMakesThem() {
        // DcMotorImpl.setPowerFloat = setZeroPowerBehavior(FLOAT); setPower(0).
        // DcMotorImpl.getPowerFloat = zeroPowerBehavior == FLOAT && getPower() == 0.0.
        FakeMotor motor = spunUpMotor();
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        assertFalse(motor.getPowerFloat(), "BRAKE at full power is not floating");

        motor.setPowerFloat();
        assertEquals(DcMotor.ZeroPowerBehavior.FLOAT, motor.getZeroPowerBehavior());
        assertEquals(0.0, motor.getPower(), 0.0);
        assertTrue(motor.getPowerFloat(), "FLOAT at zero power is floating");

        motor.setPower(0.3);
        assertFalse(motor.getPowerFloat(), "FLOAT while driven is not floating");

        motor.setPower(0.0);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        assertFalse(motor.getPowerFloat(), "BRAKE at zero power is not floating");
    }

    @Test
    void closeFloatsTheMotorAsDcMotorImplDoes() {
        FakeMotor motor = spunUpMotor();
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.close();
        assertTrue(motor.getPowerFloat(), "DcMotorImpl.close() calls setPowerFloat()");
    }

    @Test
    void resetDeviceConfigurationForOpModePutsTheConfiguredMotorTypeBack() {
        // LynxDcMotorController.resetDeviceConfigurationForOpMode(port) calls
        // setMotorType(port, internalMotorType), so a type set by the last OpMode does not survive.
        FakeMotor motor = new FakeMotor(MAX_VELOCITY_TICKS_PER_SECOND, MAX_ACCEL, 500.0);
        MotorConfigurationType configured = motor.getMotorType().clone();
        configured.setTicksPerRev(1000.0);
        motor.setMotorType(configured);
        assertEquals(1000.0, motor.getMotorType().getTicksPerRev(), 0.0, "precondition: the set type is reported");

        motor.resetDeviceConfigurationForOpMode();

        assertEquals(500.0, motor.getMotorType().getTicksPerRev(), 0.0, "the constructor-derived type is back");
    }

    @Test
    void aNanPowerIsReportedBackButDrivesTheShaftAtZero() {
        // LynxDcMotorController.internalSetMotorPower: Range.clip passes NaN through and it is cached as the
        // last known power, but the hub command is built with a double-to-int cast, which makes NaN zero.
        FakeMotor motor = spunUpMotor();
        motor.setPower(Double.NaN);
        assertTrue(Double.isNaN(motor.getPower()), "getPower() reports the cached NaN, as the real one does");
        assertFalse(motor.getPowerFloat(), "NaN is not exactly zero power");

        for (int i = 0; i < 200; i++) {
            motor.update(TICK);
        }
        assertEquals(0.0, motor.getSpeed(), 1e-6, "the shaft is driven at zero, and its speed never becomes NaN");
        assertTrue(Double.isFinite(motor.getPhysicalPositionTicks()), "the count stays a number");
        assertTrue(Double.isFinite(motor.getCurrent(CurrentUnit.AMPS)), "and so does the current");

        motor.setPower(0.5);
        motor.update(TICK);
        assertTrue(motor.getSpeed() > 0.0, "a later finite command drives it again");
    }

    @Test
    void resetDeviceConfigurationForOpModeRestoresForwardAndNothingElse() {
        FakeMotor motor = spunUpMotor();
        motor.setDirection(DcMotor.Direction.REVERSE);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motor.resetDeviceConfigurationForOpMode();

        assertEquals(DcMotor.Direction.FORWARD, motor.getDirection(), "DcMotorImpl restores FORWARD");
        assertEquals(DcMotor.RunMode.RUN_USING_ENCODER, motor.getMode(), "and leaves the mode alone");
        assertEquals(1.0, motor.getPower(), 0.0, "and the power");
    }

    @Test
    void aFreshMotorReportsTheSdksDefaultsForToleranceVersionAndName() {
        FakeMotor motor = new FakeMotor(MAX_VELOCITY_TICKS_PER_SECOND, MAX_ACCEL);
        assertEquals(5, motor.getTargetPositionTolerance(), "DcMotorImplEx's constructor sets 5");
        assertEquals(1, motor.getVersion(), "DcMotorImpl.getVersion() returns 1");
        assertEquals("Motor", motor.getDeviceName(), "DcMotorImpl reads R.string.configTypeMotor, \"Motor\"");
        assertEquals(DcMotor.Direction.FORWARD, motor.getDirection());
        // LynxDcMotorController.initializeHardware() calls floatHardware(), which floats every motor: FLOAT
        // at zero power. So a fresh real motor reports getPowerFloat() == true, and so must this one.
        assertEquals(DcMotor.ZeroPowerBehavior.FLOAT, motor.getZeroPowerBehavior());
        assertTrue(motor.getPowerFloat(), "a fresh motor is floating, as the hub leaves it at start-up");
    }

    @Test
    void aMotorTypeThatIsSetIsTheOneReportedAndTheOneAngularRatesUse() {
        // SolversLib's Motor(hardwareMap, id, cpr, rpm) clones getMotorType(), edits it and hands it back
        // through setMotorType. On a real port the controller keeps it; it used to be dropped here.
        FakeMotor motor = new FakeMotor(MAX_VELOCITY_TICKS_PER_SECOND, MAX_ACCEL, 500.0);
        MotorConfigurationType configured = motor.getMotorType().clone();
        configured.setTicksPerRev(1000.0);
        configured.setMaxRPM(60.0);
        motor.setMotorType(configured);
        configured.setTicksPerRev(7.0); // editing the caller's object afterwards must not reach the port

        assertEquals(1000.0, motor.getMotorType().getTicksPerRev(), 0.0, "the set type is reported back");
        assertEquals(60.0, motor.getMotorType().getMaxRPM(), 0.0);

        motor.getMotorType().setTicksPerRev(3.0); // nor may editing a returned copy
        assertEquals(1000.0, motor.getMotorType().getTicksPerRev(), 0.0);

        motor.setVelocity(360.0, AngleUnit.DEGREES); // one revolution per second at the configured 1000 CPR
        for (int i = 0; i < 200; i++) {
            motor.update(TICK);
        }
        assertEquals(1000.0, motor.getVelocity(), 1.0, "one rev/s is 1000 ticks/s at the configured CPR");
        assertEquals(360.0, motor.getVelocity(AngleUnit.DEGREES), 0.5, "and reads back as 360 deg/s");
    }

    @Test
    void aNonPositiveOrNonFiniteFreeSpeedOrAccelerationIsRejected() {
        double[] bad = {0.0, -1.0, Double.NaN, Double.POSITIVE_INFINITY};
        for (double value : bad) {
            assertThrows(IllegalArgumentException.class, () -> new FakeMotor(value, MAX_ACCEL),
                    "free speed " + value + " must be rejected: it would divide into NaN power");
            assertThrows(IllegalArgumentException.class, () -> new FakeMotor(MAX_VELOCITY_TICKS_PER_SECOND, value),
                    "maxAccel " + value + " must be rejected");
        }
    }

    @Test
    void aLongTickNeverOvershootsTheCommandedPower() {
        // The old explicit-Euler step multiplied the gap by maxAccel * dt, which overshoots once that exceeds
        // 1 and diverges above 2. The exact first-order step closes (1 - e^(-maxAccel * dt)) of the gap, which
        // is always below 1.
        FakeMotor motor = new FakeMotor(MAX_VELOCITY_TICKS_PER_SECOND, 75.0); // maxAccel * TICK = 1.5
        motor.setPower(1.0);
        for (int i = 0; i < 50; i++) {
            motor.update(TICK);
            assertTrue(motor.getSpeed() <= 1.0, "speed must never pass the commanded power; got "
                    + motor.getSpeed() + " on tick " + i);
        }
        assertEquals(1.0, motor.getSpeed(), 1e-9, "and it still settles on the commanded power");

        FakeMotor oneTick = new FakeMotor(MAX_VELOCITY_TICKS_PER_SECOND, MAX_ACCEL);
        oneTick.setPower(1.0);
        oneTick.update(TICK);
        assertEquals(1.0 - Math.exp(-MAX_ACCEL * TICK), oneTick.getSpeed(), 1e-15,
                "one tick from rest closes exactly 1 - e^(-maxAccel * dt) of the gap");
    }
}
