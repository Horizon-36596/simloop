package org.horizon36596.simloop.fakehardware;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.qualcomm.robotcore.exception.TargetPositionNotSetException;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import org.junit.jupiter.api.Test;

/**
 * Batch 2.2: {@link FakeExternalEncoder} — the plant &rarr; sensor seam for a 1-DOF mechanism. Pins the
 * behaviours the slide subsystem depends on: position is <b>injected</b> (not integrated), velocity is
 * finite-differenced from injected counts (never a wall-clock, domain R5), the SDK direction/CPR contracts
 * hold, and non-finite input fails loud (fakehardware-plant §0).
 */
class FakeExternalEncoderTest {

    private static final double DT = 0.02;
    private static final double CPR = 103.6;

    private static FakeExternalEncoder encoder() {
        // maxVel ticks/s = CPR * 1620rpm / 60; only feeds getMotorType() coherence.
        return new FakeExternalEncoder(CPR * 1620.0 / 60.0, CPR);
    }

    @Test
    void injectedPositionReadsBackImmediately() {
        FakeExternalEncoder enc = encoder();
        enc.setEncoderPosition(500.7);
        // Truncated to int like a real encoder count (SDK getCurrentPosition returns int).
        assertEquals(500, enc.getCurrentPosition());
    }

    @Test
    void commandedPowerDoesNotMovePositionItIsASensor() {
        FakeExternalEncoder enc = encoder();
        enc.setPower(1.0);
        for (int i = 0; i < 100; i++) enc.update(DT);
        // No injection -> no motion. A sensor's count comes only from the shaft (injection), never power.
        assertEquals(0, enc.getCurrentPosition());
        assertEquals(1.0, enc.getPower());   // stored for API fidelity
    }

    @Test
    void velocityIsFiniteDifferenceOfInjectedPosition() {
        FakeExternalEncoder enc = encoder();
        enc.setEncoderPosition(0.0);
        enc.update(DT);                 // establish baseline
        enc.setEncoderPosition(200.0);  // moved 200 ticks over the next tick
        enc.update(DT);
        assertEquals(200.0 / DT, enc.getVelocity(), 1e-9);
    }

    @Test
    void reverseDirectionNegatesCountAndVelocity() {
        FakeExternalEncoder enc = encoder();
        enc.setDirection(DcMotorSimple.Direction.REVERSE);
        enc.setEncoderPosition(0.0);
        enc.update(DT);
        enc.setEncoderPosition(300.0);
        enc.update(DT);
        assertEquals(-300, enc.getCurrentPosition());
        assertEquals(-300.0 / DT, enc.getVelocity(), 1e-9);
    }

    @Test
    void nonFiniteInjectedPositionFailsLoud() {
        FakeExternalEncoder enc = encoder();
        assertThrows(IllegalArgumentException.class, () -> enc.setEncoderPosition(Double.NaN));
        assertThrows(IllegalArgumentException.class, () -> enc.setEncoderPosition(Double.POSITIVE_INFINITY));
    }

    @Test
    void nonPositiveOrNonFiniteDeltaTimeLeavesVelocityUntouched() {
        FakeExternalEncoder enc = encoder();
        // First establish a known NON-ZERO velocity with two valid ticks, so the assertion below can't pass
        // trivially off the rest value.
        enc.setEncoderPosition(0.0);
        enc.update(DT);
        enc.setEncoderPosition(150.0);
        enc.update(DT);
        double established = enc.getVelocity();
        assertEquals(150.0 / DT, established, 1e-9);   // sanity: baseline really is non-zero

        // Now a bad dt is not a real tick (FakeTimer is monotonic): even with a fresh injected position it
        // must NOT recompute velocity — the previously established value is left untouched (never NaN/Inf).
        enc.setEncoderPosition(999.0);
        enc.update(0.0);
        enc.update(-1.0);
        enc.update(Double.NaN);
        assertEquals(established, enc.getVelocity(), 0.0);
    }

    @Test
    void ctorRejectsNonPositiveTicksPerRev() {
        assertThrows(IllegalArgumentException.class, () -> new FakeExternalEncoder(1000.0, 0.0));
        assertThrows(IllegalArgumentException.class, () -> new FakeExternalEncoder(1000.0, -5.0));
    }

    @Test
    void motorTypeReportsRealCprAndAchievableMax() {
        FakeExternalEncoder enc = encoder();
        MotorConfigurationType type = enc.getMotorType();
        assertEquals(CPR, type.getTicksPerRev());
        // getAchieveableMaxTicksPerSecond == the maxVel passed in (ticksPerRev * maxRPM * fraction / 60).
        assertEquals(CPR * 1620.0 / 60.0, type.getAchieveableMaxTicksPerSecond(), 1e-6);
    }

    // ---- 2026-09-26 review: the port stores what a real motor port stores (SDK 12.0.0 bytecode) ----

    @Test
    void theFirstTickReportsNoVelocityRatherThanASpikeFromZero() {
        // An encoder injected at 800 ticks before its first update used to difference against 0 and report
        // 800 / dt = 40000 ticks/s for one tick -- a spike no real encoder produces.
        FakeExternalEncoder enc = encoder();
        enc.setEncoderPosition(800.0);
        enc.update(DT);
        assertEquals(0.0, enc.getVelocity(), 0.0, "nothing to difference against yet, so standing still");

        enc.setEncoderPosition(810.0);
        enc.update(DT);
        assertEquals(10.0 / DT, enc.getVelocity(), 1e-9, "and from the second tick on it is the real rate");
    }

    @Test
    void theRunModeIsStoredAndTheDeprecatedModesAreMigrated() {
        FakeExternalEncoder enc = encoder();
        assertEquals(DcMotor.RunMode.RUN_WITHOUT_ENCODER, enc.getMode(), "the mode a fresh port is in");

        enc.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        assertEquals(DcMotor.RunMode.RUN_USING_ENCODER, enc.getMode(), "stored, not always RUN_WITHOUT_ENCODER");

        enc.setMode(DcMotor.RunMode.RESET_ENCODERS);
        assertEquals(DcMotor.RunMode.STOP_AND_RESET_ENCODER, enc.getMode(),
                "RESET_ENCODERS migrates to STOP_AND_RESET_ENCODER, as DcMotorImpl.setMode does");

    }

    @Test
    void runToPositionWithoutATargetThrowsAsTheHubDoes() {
        FakeExternalEncoder enc = encoder();
        enc.setMotorDisable();
        assertThrows(TargetPositionNotSetException.class, () -> enc.setMode(DcMotor.RunMode.RUN_TO_POSITION),
                "the hub NACKs the enable with MOTOR_NOT_CONFIG_BEFORE_ENABLED and the SDK rethrows it");
        assertEquals(DcMotor.RunMode.RUN_TO_POSITION, enc.getMode(),
                "LynxDcMotorController.setMotorMode records the mode before the enable that throws");
        assertFalse(enc.isMotorEnabled(), "the refused enable leaves the channel as it was");
    }

    @Test
    void runToPositionWithATargetIsAcceptedAndIsBusyComparesTheCountWithTheTarget() {
        FakeExternalEncoder enc = encoder();
        enc.setTargetPosition(1000);
        assertFalse(enc.isBusy(), "outside RUN_TO_POSITION a real port is never busy");

        enc.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        assertEquals(DcMotor.RunMode.RUN_TO_POSITION, enc.getMode());
        assertTrue(enc.isBusy(), "the count is 1000 ticks from the target");

        enc.setEncoderPosition(995.0);
        assertFalse(enc.isBusy(), "5 ticks off is inside the default tolerance of 5");
        enc.setEncoderPosition(994.0);
        assertTrue(enc.isBusy(), "6 ticks off is outside it");

        // DcMotorImplEx sends the tolerance only inside setTargetPosition (SDK 12.0.0 bytecode), so the hub
        // keeps the old one until the next target.
        enc.setTargetPositionTolerance(10);
        assertEquals(10, enc.getTargetPositionTolerance(), "the handle stores it at once");
        assertTrue(enc.isBusy(), "but the hub still holds the tolerance sent with the target, 5");
        enc.setTargetPosition(1000);
        assertFalse(enc.isBusy(), "re-sending the target sends the new tolerance of 10");

        // The hub compares raw count with raw target; the direction was applied when the target was sent.
        enc.setDirection(DcMotorSimple.Direction.REVERSE);
        assertEquals(-1000, enc.getTargetPosition(), "the target reads back through the new direction");
        assertFalse(enc.isBusy(), "and a later direction change does not move the hub's target or count");
    }

    @Test
    void aVelocityCommandInRunToPositionKeepsTheModeAsTheHubDoes() {
        // LynxDcMotorController.setMotorVelocity leaves RUN_TO_POSITION and RUN_USING_ENCODER alone and
        // switches every other mode to RUN_USING_ENCODER; RUN_TO_POSITION plus setVelocity is how robot code
        // caps the speed of a move, so leaving the mode would end the move's isBusy() at once.
        FakeExternalEncoder enc = encoder();
        enc.setTargetPosition(1000);
        enc.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        enc.setVelocity(500.0);

        assertEquals(DcMotor.RunMode.RUN_TO_POSITION, enc.getMode());
        assertTrue(enc.isBusy(), "still 1000 ticks from the target");

        enc.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        enc.setVelocity(500.0);
        assertEquals(DcMotor.RunMode.RUN_USING_ENCODER, enc.getMode(), "any other mode is switched");
    }

    @Test
    void stopAndResetEncoderZeroesTheCountWhichThenCarriesOnFromZero() {
        FakeExternalEncoder enc = encoder();
        enc.setEncoderPosition(800.0);
        enc.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        assertEquals(0, enc.getCurrentPosition(), "the reset zeroes the reported count");

        enc.setEncoderPosition(900.0);
        assertEquals(100, enc.getCurrentPosition(),
                "the mechanism is driven elsewhere and keeps moving, so the count carries on from zero");

        enc.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        assertEquals(100, enc.getCurrentPosition(), "re-setting the same mode is not a second reset");

        enc.setDirection(DcMotorSimple.Direction.REVERSE);
        assertEquals(-100, enc.getCurrentPosition(), "and direction still applies to the reset count");
    }

    @Test
    void aResetDoesNotShowUpAsAVelocity() {
        FakeExternalEncoder enc = encoder();
        enc.setEncoderPosition(500.0);
        enc.update(DT);
        enc.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        enc.setEncoderPosition(510.0);
        enc.update(DT);
        assertEquals(10.0 / DT, enc.getVelocity(), 1e-9, "the zeroing is an offset, not motion");
    }

    @Test
    void theTargetPositionAndToleranceAreStoredWithTheSdksDefaultTolerance() {
        FakeExternalEncoder enc = encoder();
        assertEquals(5, enc.getTargetPositionTolerance(), "DcMotorImplEx's constructor sets 5");
        enc.setTargetPosition(1234);
        enc.setTargetPositionTolerance(9);
        assertEquals(1234, enc.getTargetPosition());
        assertEquals(9, enc.getTargetPositionTolerance());
    }

    @Test
    void theCurrentAlertHonoursItsUnit() {
        FakeExternalEncoder enc = encoder();
        enc.setCurrentAlert(2500.0, CurrentUnit.MILLIAMPS);
        assertEquals(2.5, enc.getCurrentAlert(CurrentUnit.AMPS), 1e-12, "2500 mA is 2.5 A");
        assertEquals(2500.0, enc.getCurrentAlert(CurrentUnit.MILLIAMPS), 1e-9);
    }

    @Test
    void setPowerFloatResetDeviceConfigurationAndMotorTypeBehaveAsOnARealPort() {
        FakeExternalEncoder enc = encoder();
        enc.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        enc.setPower(0.4);
        assertFalse(enc.getPowerFloat());
        enc.setPowerFloat();
        assertTrue(enc.getPowerFloat(), "DcMotorImpl.setPowerFloat = FLOAT + setPower(0)");

        enc.setDirection(DcMotorSimple.Direction.REVERSE);
        enc.resetDeviceConfigurationForOpMode();
        assertEquals(DcMotorSimple.Direction.FORWARD, enc.getDirection(), "DcMotorImpl restores FORWARD");

        MotorConfigurationType configured = enc.getMotorType().clone();
        configured.setTicksPerRev(8192.0);
        enc.setMotorType(configured);
        assertEquals(8192.0, enc.getMotorType().getTicksPerRev(), 0.0, "a set motor type is kept and reported");

        enc.resetDeviceConfigurationForOpMode();
        assertEquals(CPR, enc.getMotorType().getTicksPerRev(), 0.0,
                "LynxDcMotorController.resetDeviceConfigurationForOpMode puts the configured motor type back");
        assertEquals(1, enc.getVersion(), "DcMotorImpl.getVersion() returns 1");
    }
}
