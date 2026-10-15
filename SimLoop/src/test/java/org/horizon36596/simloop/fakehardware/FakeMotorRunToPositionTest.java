package org.horizon36596.simloop.fakehardware;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.qualcomm.robotcore.exception.TargetPositionNotSetException;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.junit.jupiter.api.Test;

import java.util.function.DoubleUnaryOperator;

/**
 * Pins {@link FakeMotor}'s {@code RUN_TO_POSITION}: the hub's position loop feeding its velocity loop, the
 * power cap, holding at the target, {@code isBusy()}, the tolerance, direction, encoder resets and the
 * exception for a missing target.
 *
 * <p>Two kinds of assertion live here, and each test says which it is. <b>SDK exact</b> ones (the exception,
 * its message, when {@code isBusy()} is true, what direction and the tolerance do, what a mode change
 * re-sends) were read off the SDK 12.0.0 sources and must never be loosened. <b>Model</b> ones (how fast it
 * gets there, how far it overshoots) depend on the firmware APPROXIMATIONS in {@link HubMotorControlLoop};
 * their bounds have headroom so that ordinary retuning passes and a loop that rings or stalls fails.
 *
 * <p>Everything runs on sim time: a fixed tick, stepped by hand. Nothing here reads a clock.
 */
class FakeMotorRunToPositionTest {

    /** Sim tick, seconds (a 50 Hz OpMode loop). */
    private static final double TICK = 0.02;

    /** The SDK's default RUN_TO_POSITION tolerance, ticks. */
    private static final int DEFAULT_TOLERANCE = 5;

    // A vertical slide on a goBILDA 5203 312 rpm motor: 537.7 counts per output rev, free speed
    // 537.7 * 312 / 60 = 2796 ticks/s. Illustrative plant numbers, not a measured robot.
    private static final double SLIDE_FREE_SPEED = 2796.0;
    private static final double SLIDE_MAX_ACCEL = 12.0;
    private static final double SLIDE_CPR = 537.7;
    /** The slide's weight, as the share of stall torque it takes to hold it (pulls toward negative ticks). */
    private static final double SLIDE_GRAVITY = 0.15;
    private static final int SLIDE_TARGET = 2000;

    // An arm pivoting straight on a goBILDA 5203 117 rpm motor: 1425.1 counts per rev, free speed
    // 1425.1 * 117 / 60 = 2779 ticks/s. Zero ticks is horizontal, positive lifts it.
    private static final double ARM_FREE_SPEED = 2779.0;
    private static final double ARM_MAX_ACCEL = 10.0;
    private static final double ARM_CPR = 1425.1;
    private static final double ARM_TICKS_PER_RADIAN = ARM_CPR / (2.0 * Math.PI);
    /** Share of stall torque it takes to hold the arm horizontal; the load falls off with cos(angle). */
    private static final double ARM_GRAVITY_AT_HORIZONTAL = 0.3;
    /** 45 degrees above horizontal, in ticks. */
    private static final int ARM_TARGET = (int) Math.round(ARM_CPR / 8.0);

    /** A motor and the load on its shaft, stepped one sim tick at a time and recorded. */
    private static final class Rig {
        final FakeMotor motor;
        private final DoubleUnaryOperator loadFromPhysicalTicks;

        Rig(FakeMotor motor, DoubleUnaryOperator loadFromPhysicalTicks) {
            this.motor = motor;
            this.loadFromPhysicalTicks = loadFromPhysicalTicks;
        }

        /** One tick: the load for the current pose, then the motor. */
        void step() {
            motor.setExternalTorqueFraction(loadFromPhysicalTicks.applyAsDouble(motor.getPhysicalPositionTicks()));
            motor.update(TICK);
        }

        /** Steps {@code ticks} times and returns the reported position after each step. */
        int[] run(int ticks) {
            int[] positions = new int[ticks];
            for (int i = 0; i < ticks; i++) {
                step();
                positions[i] = motor.getCurrentPosition();
            }
            return positions;
        }

        /** Sends a RUN_TO_POSITION move, then steps {@code ticks} times; returns the reported positions. */
        int[] moveTo(int target, double power, int ticks) {
            runToPosition(motor, target, power);
            return run(ticks);
        }
    }

    private static Rig verticalSlide() {
        return new Rig(new FakeMotor(SLIDE_FREE_SPEED, SLIDE_MAX_ACCEL, SLIDE_CPR), ticks -> -SLIDE_GRAVITY);
    }

    private static Rig horizontalSlide() {
        return new Rig(new FakeMotor(SLIDE_FREE_SPEED, SLIDE_MAX_ACCEL, SLIDE_CPR), ticks -> 0.0);
    }

    private static Rig gravityArm() {
        return new Rig(new FakeMotor(ARM_FREE_SPEED, ARM_MAX_ACCEL, ARM_CPR),
                ticks -> -ARM_GRAVITY_AT_HORIZONTAL * Math.cos(ticks / ARM_TICKS_PER_RADIAN));
    }

    /** The order the SDK requires: target, then mode, then power. */
    private static void runToPosition(FakeMotor motor, int target, double power) {
        motor.setTargetPosition(target);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(power);
    }

    private static int seconds(double seconds) {
        return (int) Math.round(seconds / TICK);
    }

    private static int max(int[] values) {
        int max = Integer.MIN_VALUE;
        for (int value : values) {
            max = Math.max(max, value);
        }
        return max;
    }

    // ---- Reaching the target (model) ----

    @Test
    void aVerticalSlideReachesItsTargetAndHoldsItAgainstGravity() {
        Rig slide = verticalSlide();
        runToPosition(slide.motor, SLIDE_TARGET, 1.0);

        slide.run(seconds(2.0));
        assertEquals(SLIDE_TARGET, slide.motor.getCurrentPosition(), DEFAULT_TOLERANCE,
                "a 2000-tick lift at full power is within tolerance inside two seconds");
        assertFalse(slide.motor.isBusy());

        // Then it stays there with gravity pulling on it for another five seconds.
        for (int position : slide.run(seconds(5.0))) {
            assertEquals(SLIDE_TARGET, position, DEFAULT_TOLERANCE, "the slide holds, it does not sag");
        }
    }

    @Test
    void aGravityLoadedArmReachesItsTargetAndHoldsItWithoutCuttingPower() {
        Rig arm = gravityArm();
        runToPosition(arm.motor, ARM_TARGET, 1.0);

        arm.run(seconds(2.0));
        assertEquals(ARM_TARGET, arm.motor.getCurrentPosition(), DEFAULT_TOLERANCE, "the arm reaches 45 degrees");

        // Holding at 45 degrees takes 0.3 * cos(45 deg) = 0.21 of stall torque. The hub keeps servoing: if it
        // cut power once the target was reached, the arm would fall.
        for (int i = 0; i < seconds(5.0); i++) {
            arm.step();
            assertEquals(ARM_TARGET, arm.motor.getCurrentPosition(), DEFAULT_TOLERANCE, "held at tick " + i);
            assertTrue(arm.motor.getAppliedPhysicalPower() > 0.1,
                    "the hub is still pushing up against gravity at tick " + i + ": "
                            + arm.motor.getAppliedPhysicalPower());
            assertFalse(arm.motor.isBusy(), "and reports itself done while it does so");
        }
    }

    @Test
    void overshootIsBoundedAndTheMotionSettles() {
        // Model: pins "arrives without ringing" with headroom. The fake overshoots a few ticks here; a loop
        // that rings (a sluggish plant, or a gain change) goes past 50 and fails.
        int[] slide = horizontalSlide().moveTo(SLIDE_TARGET, 1.0, seconds(4.0));
        assertTrue(max(slide) <= SLIDE_TARGET + 50, "slide overshoot at full power: " + (max(slide) - SLIDE_TARGET));
        assertEquals(SLIDE_TARGET, slide[slide.length - 1], DEFAULT_TOLERANCE);

        int[] arm = gravityArm().moveTo(ARM_TARGET, 1.0, seconds(4.0));
        assertTrue(max(arm) <= ARM_TARGET + 40, "arm overshoot at full power: " + (max(arm) - ARM_TARGET));
        assertEquals(ARM_TARGET, arm[arm.length - 1], DEFAULT_TOLERANCE);
    }

    @Test
    void theArmReachesItsTargetWhateverTheSimTick() {
        // Model: the closed loop runs in sub-steps of the hub's control period, so halving the sim tick
        // changes the trace a little and the outcome not at all.
        for (double tick : new double[] {0.005, 0.01, 0.02, 0.05}) {
            FakeMotor motor = new FakeMotor(ARM_FREE_SPEED, ARM_MAX_ACCEL, ARM_CPR);
            runToPosition(motor, ARM_TARGET, 1.0);
            for (double t = 0.0; t < 3.0; t += tick) {
                motor.setExternalTorqueFraction(-ARM_GRAVITY_AT_HORIZONTAL
                        * Math.cos(motor.getPhysicalPositionTicks() / ARM_TICKS_PER_RADIAN));
                motor.update(tick);
            }
            assertEquals(ARM_TARGET, motor.getCurrentPosition(), DEFAULT_TOLERANCE, "sim tick " + tick + " s");
        }
    }

    // ---- The power cap (SDK exact: |power| * max becomes the speed limit; model: how closely it is held) ----

    @Test
    void thePowerCapsTheSpeedAndTheMotorStillArrives() {
        Rig slide = horizontalSlide();
        runToPosition(slide.motor, SLIDE_TARGET, 0.3);
        assertEquals(0.3, slide.motor.getPower(), 0.0, "getPower() reports the cap, not the loop's output");

        double fastest = 0.0;
        for (int i = 0; i < seconds(5.0); i++) {
            slide.step();
            fastest = Math.max(fastest, Math.abs(slide.motor.getVelocity()));
        }
        double cap = 0.3 * SLIDE_FREE_SPEED;
        // Model: the velocity loop overshoots its target a little on the way up, never far.
        assertTrue(fastest <= cap * 1.15, "0.3 power caps the speed near " + cap + " ticks/s; saw " + fastest);
        assertTrue(fastest >= cap * 0.9, "and the move actually runs at the cap; saw " + fastest);
        assertEquals(SLIDE_TARGET, slide.motor.getCurrentPosition(), DEFAULT_TOLERANCE, "it still gets there");
    }

    @Test
    void setVelocityInRunToPositionStaysInItAndBecomesTheSpeedCap() {
        // SDK exact: LynxDcMotorController.setMotorVelocity switches the mode only when it is neither
        // RUN_USING_ENCODER nor RUN_TO_POSITION, so the usual DcMotorEx speed cap (target, RUN_TO_POSITION,
        // setVelocity) keeps the position loop and isBusy(). Model: how closely the cap is held.
        Rig slide = horizontalSlide();
        slide.motor.setTargetPosition(SLIDE_TARGET);
        slide.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide.motor.setVelocity(800.0);
        assertEquals(DcMotor.RunMode.RUN_TO_POSITION, slide.motor.getMode(), "setVelocity leaves the mode alone");
        assertTrue(slide.motor.isBusy(), "so isBusy() still means 'not at the target yet'");

        double fastest = 0.0;
        for (int i = 0; i < seconds(5.0); i++) {
            slide.step();
            fastest = Math.max(fastest, Math.abs(slide.motor.getVelocity()));
        }
        assertTrue(fastest <= 800.0 * 1.15, "800 ticks/s caps the speed; saw " + fastest);
        assertTrue(fastest >= 800.0 * 0.9, "and the move actually runs at the cap; saw " + fastest);
        assertEquals(SLIDE_TARGET, slide.motor.getCurrentPosition(), DEFAULT_TOLERANCE, "it still gets there");
        assertFalse(slide.motor.isBusy());
    }

    @Test
    void aNegativePowerIsTheSameCapAsAPositiveOne() {
        // SDK exact: DcMotorImpl.setPower sends Math.abs(power) in RUN_TO_POSITION, and getPower() reports the
        // absolute value; the position loop decides the direction.
        Rig positive = horizontalSlide();
        Rig negative = horizontalSlide();
        runToPosition(positive.motor, SLIDE_TARGET, 0.5);
        runToPosition(negative.motor, SLIDE_TARGET, -0.5);

        assertEquals(0.5, negative.motor.getPower(), 0.0);
        assertArrayEquals(positive.run(seconds(3.0)), negative.run(seconds(3.0)),
                "a negative power drives exactly as its absolute value does");
    }

    @Test
    void zeroPowerHoldsTheMotorWhereItIs() {
        // SDK exact: a zero cap is a zero velocity target, so the hub holds rather than moving.
        Rig slide = horizontalSlide();
        runToPosition(slide.motor, SLIDE_TARGET, 0.0);
        slide.run(seconds(2.0));
        assertEquals(0, slide.motor.getCurrentPosition(), "no speed allowed, so no motion");
        assertTrue(slide.motor.isBusy(), "and it is still busy, since it is nowhere near the target");
    }

    // ---- isBusy (SDK exact) ----

    @Test
    void isBusyIsTrueExactlyWhileTheCountIsOutsideTheTolerance() {
        // LynxDcMotorController.isBusy returns !isAtTarget() in RUN_TO_POSITION, and the hub's at-target test
        // compares the count with the target and the tolerance sent with it.
        Rig slide = verticalSlide();
        runToPosition(slide.motor, SLIDE_TARGET, 1.0);
        assertTrue(slide.motor.isBusy(), "busy the moment the move is commanded");

        int busyTicks = 0;
        for (int i = 0; i < seconds(4.0); i++) {
            slide.step();
            int error = SLIDE_TARGET - slide.motor.getCurrentPosition();
            assertEquals(Math.abs(error) > DEFAULT_TOLERANCE, slide.motor.isBusy(),
                    "tick " + i + ", error " + error + " ticks");
            if (slide.motor.isBusy()) {
                busyTicks++;
            }
        }
        assertTrue(busyTicks >= seconds(0.5), "busy for the whole climb, " + busyTicks + " ticks");
        assertFalse(slide.motor.isBusy(), "and not busy once it is there");
    }

    @Test
    void isBusyIsFalseInEveryOtherModeHoweverFarTheTarget() {
        FakeMotor motor = new FakeMotor(SLIDE_FREE_SPEED, SLIDE_MAX_ACCEL, SLIDE_CPR);
        motor.setTargetPosition(10_000);
        for (DcMotor.RunMode mode : new DcMotor.RunMode[] {DcMotor.RunMode.RUN_WITHOUT_ENCODER,
                DcMotor.RunMode.RUN_USING_ENCODER, DcMotor.RunMode.STOP_AND_RESET_ENCODER}) {
            motor.setMode(mode);
            assertFalse(motor.isBusy(), mode + " never reports busy");
        }
    }

    @Test
    void aWiderToleranceReportsDoneSooner() {
        int narrow = firstTickNotBusy(DEFAULT_TOLERANCE);
        int wide = firstTickNotBusy(150);
        assertTrue(wide < narrow, "150 ticks of tolerance ends the wait before 5 does: " + wide + " vs " + narrow);
    }

    private static int firstTickNotBusy(int tolerance) {
        Rig slide = horizontalSlide();
        slide.motor.setTargetPositionTolerance(tolerance);
        runToPosition(slide.motor, SLIDE_TARGET, 1.0);
        for (int i = 0; i < seconds(4.0); i++) {
            slide.step();
            if (!slide.motor.isBusy()) {
                return i;
            }
        }
        throw new AssertionError("never stopped being busy with tolerance " + tolerance);
    }

    @Test
    void theToleranceReachesTheHubOnlyWithTheNextTarget() {
        // SDK exact: DcMotorImplEx.setTargetPositionTolerance only stores the value on the handle;
        // internalSetTargetPosition is the one place it is sent. A zero power keeps the motor still.
        FakeMotor motor = new FakeMotor(SLIDE_FREE_SPEED, SLIDE_MAX_ACCEL, SLIDE_CPR);
        runToPosition(motor, 30, 0.0);
        assertTrue(motor.isBusy(), "30 ticks off with the default tolerance of 5");

        motor.setTargetPositionTolerance(50);
        assertEquals(50, motor.getTargetPositionTolerance(), "the handle reports the new tolerance at once");
        assertTrue(motor.isBusy(), "but the hub is still using 5");

        motor.setTargetPosition(30);
        assertFalse(motor.isBusy(), "re-sending the target sends 50 with it");
    }

    @Test
    void aToleranceTheHubCannotHoldIsRejectedWhenTheTargetIsSent() {
        // SDK exact: LynxSetMotorTargetPositionCommand rejects a tolerance outside its unsigned 16-bit field.
        FakeMotor motor = new FakeMotor(SLIDE_FREE_SPEED, SLIDE_MAX_ACCEL, SLIDE_CPR);
        motor.setTargetPosition(100);

        for (int bad : new int[] {-1, 65536}) {
            motor.setTargetPositionTolerance(bad);
            assertEquals(bad, motor.getTargetPositionTolerance(), "the handle stores it without checking");
            IllegalArgumentException refused =
                    assertThrows(IllegalArgumentException.class, () -> motor.setTargetPosition(200));
            assertEquals("illegal tolerance: " + bad, refused.getMessage());
            assertEquals(100, motor.getTargetPosition(), "and nothing was sent");
        }
    }

    // ---- Direction (SDK exact) ----

    @Test
    void aReversedMotorReachesTheSameReportedTargetWithItsShaftTurningTheOtherWay() {
        Rig forward = horizontalSlide();
        Rig reversed = horizontalSlide();
        reversed.motor.setDirection(DcMotor.Direction.REVERSE);
        runToPosition(forward.motor, SLIDE_TARGET, 1.0);
        runToPosition(reversed.motor, SLIDE_TARGET, 1.0);

        assertEquals(SLIDE_TARGET, reversed.motor.getTargetPosition(), "the target reads back as it was set");
        assertArrayEquals(forward.run(seconds(3.0)), reversed.run(seconds(3.0)),
                "DcMotorImpl negates target, position and power alike, so the reported move is identical");
        assertTrue(reversed.motor.getPhysicalPositionTicks() < -SLIDE_TARGET + DEFAULT_TOLERANCE + 1,
                "while the physical shaft went to the negative side: " + reversed.motor.getPhysicalPositionTicks());
        assertFalse(reversed.motor.isBusy());
    }

    @Test
    void flippingTheDirectionAfterTheTargetWasSentMovesNothing() {
        // SDK exact: setTargetPosition negates the target when it is SENT, and getTargetPosition negates the
        // hub's value when it is READ. setDirection sends nothing, so the hub keeps driving to the same place.
        Rig slide = horizontalSlide();
        runToPosition(slide.motor, SLIDE_TARGET, 1.0);
        slide.motor.setDirection(DcMotor.Direction.REVERSE);

        assertEquals(-SLIDE_TARGET, slide.motor.getTargetPosition(), "the same hub target, read the other way");
        slide.run(seconds(3.0));
        assertEquals(SLIDE_TARGET, slide.motor.getPhysicalPositionTicks(), DEFAULT_TOLERANCE,
                "the shaft still went where it was first sent");
        assertEquals(-SLIDE_TARGET, slide.motor.getCurrentPosition(), DEFAULT_TOLERANCE, "reported the other way");
        assertFalse(slide.motor.isBusy(), "and it is at the target, in either reading");
    }

    // ---- Encoder resets and mode changes (SDK exact) ----

    @Test
    void anEncoderResetZeroesTheCountAndTheKeptTargetIsMeasuredFromTheNewZero() {
        Rig slide = horizontalSlide();
        runToPosition(slide.motor, 1000, 1.0);
        slide.run(seconds(2.0));
        assertEquals(1000, slide.motor.getCurrentPosition(), DEFAULT_TOLERANCE, "precondition: it got there");

        slide.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.step();
        assertEquals(0, slide.motor.getCurrentPosition(), "the reset zeroes the count");
        assertFalse(slide.motor.isBusy(), "and isBusy is false outside RUN_TO_POSITION");
        assertEquals(1000, slide.motor.getTargetPosition(), "the target survives the reset");

        // No new setTargetPosition: the hub still holds 1000 and the power 1.0, so it goes another 1000.
        slide.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        assertTrue(slide.motor.isBusy(), "1000 ticks from the new zero");
        slide.run(seconds(2.0));
        assertEquals(1000, slide.motor.getCurrentPosition(), DEFAULT_TOLERANCE, "there again, from the new zero");
    }

    @Test
    void leavingRunToPositionReIssuesTheCapAsARawPower() {
        // SDK exact, and a real trap: setMotorMode re-issues the cached power in the new mode, and in
        // RUN_TO_POSITION that cache holds |power| with no direction applied. So an arm switched out of
        // RUN_TO_POSITION without a fresh setPower runs at that power in the positive RAW direction, which for
        // a REVERSE motor is the negative reported direction.
        Rig slide = horizontalSlide();
        slide.motor.setDirection(DcMotor.Direction.REVERSE);
        runToPosition(slide.motor, 500, 0.4);
        slide.run(seconds(2.0));

        slide.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        assertEquals(-0.4, slide.motor.getPower(), 1e-12, "the raw +0.4, read through a REVERSE handle");
        slide.run(seconds(2.0));
        assertEquals(-(int) (0.4 * SLIDE_FREE_SPEED), slide.motor.getVelocity(), 2.0,
                "and it runs at 0.4 of max speed, away from the target it was holding");
    }

    // ---- A missing target (SDK exact) ----

    @Test
    void runToPositionWithNoTargetThrowsTheSdksExceptionAndDrivesNothing() {
        FakeMotor motor = new FakeMotor(SLIDE_FREE_SPEED, SLIDE_MAX_ACCEL, SLIDE_CPR);
        motor.setPower(0.5);

        TargetPositionNotSetException refused = assertThrows(TargetPositionNotSetException.class,
                () -> motor.setMode(DcMotor.RunMode.RUN_TO_POSITION));
        assertEquals("Failed to enable motor. You must set a target position before switching to "
                + "RUN_TO_POSITION mode", refused.getMessage());
        assertEquals(DcMotor.RunMode.RUN_TO_POSITION, motor.getMode(), "the mode was recorded before the NACK");
        assertFalse(motor.isMotorEnabled(), "the channel the hub refused to enable is off");

        // Every later command ends in the same refused enable.
        assertThrows(TargetPositionNotSetException.class, () -> motor.setPower(0.5));
        assertThrows(TargetPositionNotSetException.class, () -> motor.setVelocity(500.0));
        assertThrows(TargetPositionNotSetException.class, motor::setMotorEnable);
        for (int i = 0; i < seconds(1.0); i++) {
            motor.update(TICK);
        }
        assertEquals(0, motor.getCurrentPosition(), "and nothing moved");

        // Sending a target is the fix, as the message says; the next command then enables and drives.
        motor.setTargetPosition(500);
        motor.setPower(0.5);
        assertTrue(motor.isMotorEnabled());
        for (int i = 0; i < seconds(2.0); i++) {
            motor.update(TICK);
        }
        assertEquals(500, motor.getCurrentPosition(), DEFAULT_TOLERANCE);
    }

    @Test
    void aTargetSetInAnotherModeIsEnoughToEnterRunToPosition() {
        // SDK exact: DcMotorImpl accepts a target in any mode, and the usual order (target, mode, power) is
        // what makes the hub accept the enable.
        FakeMotor motor = new FakeMotor(SLIDE_FREE_SPEED, SLIDE_MAX_ACCEL, SLIDE_CPR);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setTargetPosition(300);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        assertTrue(motor.isMotorEnabled());
        assertEquals(300, motor.getTargetPosition());
    }

    // ---- Determinism ----

    @Test
    void twoIdenticalRunsGiveBitIdenticalTraces() {
        assertArrayEquals(scriptedTrace(), scriptedTrace(),
                "the same commands on the same sim ticks produce the same doubles, bit for bit");
    }

    /**
     * An arm under gravity driven through a script that touches every closed-loop path: RUN_TO_POSITION up,
     * a new target down, a velocity move, a reset and back. Records raw bits of position, velocity and
     * winding power every tick.
     */
    private static long[] scriptedTrace() {
        Rig arm = gravityArm();
        long[] trace = new long[3 * seconds(8.0)];
        int sample = 0;
        runToPosition(arm.motor, ARM_TARGET, 1.0);
        for (int i = 0; i < seconds(8.0); i++) {
            if (i == seconds(2.0)) {
                arm.motor.setTargetPosition(-ARM_TARGET / 2);
            } else if (i == seconds(4.0)) {
                arm.motor.setVelocity(400.0);
            } else if (i == seconds(5.0)) {
                arm.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            } else if (i == seconds(5.5)) {
                arm.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            arm.step();
            trace[sample++] = Double.doubleToRawLongBits(arm.motor.getPhysicalPositionTicks());
            trace[sample++] = Double.doubleToRawLongBits(arm.motor.getPhysicalVelocityTicksPerSecond());
            trace[sample++] = Double.doubleToRawLongBits(arm.motor.getAppliedPhysicalPower());
        }
        return trace;
    }
}
