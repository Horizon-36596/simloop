package org.horizon36596.simloop.plant;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.horizon36596.simloop.config.DrivetrainSimConfig;
import org.horizon36596.simloop.fakehardware.FakeMotor;

import org.junit.jupiter.api.Test;

/**
 * INVARIANT-tier (domain R9) — the Contract: <b>a simulated chassis coasts at the configured natural
 * deceleration on each axis while its motors float, never changes speed on either axis faster than the
 * configured traction limit, and with neither configured goes exactly where its wheels say.</b>
 *
 * <p>Why it is pinned. Before these existed the chassis velocity <i>was</i> the motor model's wheel speed, so
 * at zero power it shed about 387 in/s² where the real robot floats down at 89 forwards and 37 sideways, and
 * a full-power reversal changed speed by thousands of in/s² in a tick. A path follower that plans its stops
 * from the measured coast then stops short in sim for a reason the real robot does not have.
 *
 * <p>The numbers are a real mecanum robot's, measured on 2026-08-30: 96.7 in/s forward, 84.6 sideways, coasting at 89.3638
 * and 37.0042 in/s², and a traction limit of 384.6 in/s² from its braking fit. Inches, seconds, the
 * integrator's robot frame. No clock: every step is an explicit {@code deltaTime} (domain R5).
 */
class MecanumChassisDynamicsTest {

    private static final double DT = 0.02;
    private static final double FORWARD_TOP_SPEED = 96.7176;
    private static final double STRAFE_TOP_SPEED = 84.5661;
    private static final double NATURAL_FORWARD = 89.3638;
    private static final double NATURAL_LATERAL = 37.0042;
    private static final double TRACTION = 384.6;
    private static final double TICKS_PER_INCH = 26.2;

    // ------------------------------------------------------------------------------------------- coast

    @Test
    void aFloatingChassisCoastsForwardsAtTheNaturalForwardDeceleration() {
        MecanumPoseIntegrator integrator = new MecanumPoseIntegrator(config(true, true));
        for (int i = 0; i < 50; i++) {
            integrator.integrate(80.0, 80.0, 80.0, 80.0, false, DT);       // up to 80 in/s forward
        }
        assertEquals(80.0, integrator.getForwardVelocity(), 1e-9);

        for (int i = 1; i <= 20; i++) {
            // The wheels have stopped asking for anything; the chassis must ignore that and coast.
            integrator.integrate(0.0, 0.0, 0.0, 0.0, true, DT);
            assertEquals(80.0 - NATURAL_FORWARD * DT * i, integrator.getForwardVelocity(), 1e-9,
                    "tick " + i + ": forward coast must shed exactly the natural deceleration");
        }
    }

    @Test
    void aFloatingChassisCoastsSidewaysAtTheNaturalLateralDecelerationAndStopsAtZero() {
        MecanumPoseIntegrator integrator = new MecanumPoseIntegrator(config(true, true));
        double wheel = 60.0;
        for (int i = 0; i < 50; i++) {
            integrator.integrate(-wheel, wheel, wheel, -wheel, false, DT);  // strafe left
        }
        double start = integrator.getLateralVelocity();
        assertEquals(wheel * STRAFE_TOP_SPEED / FORWARD_TOP_SPEED, start, 1e-9);

        int ticksToStop = (int) Math.ceil(start / (NATURAL_LATERAL * DT));
        for (int i = 1; i <= ticksToStop + 5; i++) {
            integrator.integrate(0.0, 0.0, 0.0, 0.0, true, DT);
            double expected = Math.max(0.0, start - NATURAL_LATERAL * DT * i);
            assertEquals(expected, integrator.getLateralVelocity(), 1e-9,
                    "tick " + i + ": lateral coast must shed exactly the natural deceleration, then rest");
        }
        assertEquals(0.0, integrator.getForwardVelocity(), 0.0, "a pure strafe has no forward part");
    }

    @Test
    void theCoastMovesThePoseByTheCoastingVelocity() {
        MecanumPoseIntegrator integrator = new MecanumPoseIntegrator(config(true, true));
        for (int i = 0; i < 50; i++) {
            integrator.integrate(50.0, 50.0, 50.0, 50.0, false, DT);
        }
        double before = integrator.getX();
        integrator.integrate(0.0, 0.0, 0.0, 0.0, true, DT);
        assertEquals((50.0 - NATURAL_FORWARD * DT) * DT, integrator.getX() - before, 1e-9);
    }

    // ---------------------------------------------------------------------------------------- traction

    @Test
    void neitherAxisEverChangesSpeedFasterThanTheTractionLimitEvenOnAFullReversal() {
        MecanumPoseIntegrator integrator = new MecanumPoseIntegrator(config(true, true));
        double previousForward = 0.0;
        double previousLateral = 0.0;
        double largestForward = 0.0;
        double largestLateral = 0.0;
        // Step commands at full speed, then flipped, on both axes: the wheels jump instantly, which is far
        // harsher than any motor model and so the worst case the limit has to hold against.
        double[][] phases = {
                {FORWARD_TOP_SPEED, FORWARD_TOP_SPEED, FORWARD_TOP_SPEED, FORWARD_TOP_SPEED},
                {-FORWARD_TOP_SPEED, -FORWARD_TOP_SPEED, -FORWARD_TOP_SPEED, -FORWARD_TOP_SPEED},
                {-FORWARD_TOP_SPEED, FORWARD_TOP_SPEED, FORWARD_TOP_SPEED, -FORWARD_TOP_SPEED},
                {FORWARD_TOP_SPEED, -FORWARD_TOP_SPEED, -FORWARD_TOP_SPEED, FORWARD_TOP_SPEED},
        };
        for (double[] wheels : phases) {
            for (int i = 0; i < 40; i++) {
                integrator.integrate(wheels[0], wheels[1], wheels[2], wheels[3], false, DT);
                largestForward = Math.max(largestForward,
                        Math.abs(integrator.getForwardVelocity() - previousForward) / DT);
                largestLateral = Math.max(largestLateral,
                        Math.abs(integrator.getLateralVelocity() - previousLateral) / DT);
                previousForward = integrator.getForwardVelocity();
                previousLateral = integrator.getLateralVelocity();
            }
        }
        assertTrue(largestForward <= TRACTION + 1e-6, "forward peaked at " + largestForward + " in/s^2");
        assertTrue(largestLateral <= TRACTION + 1e-6, "lateral peaked at " + largestLateral + " in/s^2");
        assertEquals(TRACTION, largestForward, 1e-6, "and the limit is reached, so it is the thing binding");
        assertEquals(TRACTION, largestLateral, 1e-6);
    }

    @Test
    void activeBrakingMayExceedTheCoastButNotTheTractionLimit() {
        MecanumPoseIntegrator integrator = new MecanumPoseIntegrator(config(true, true));
        for (int i = 0; i < 50; i++) {
            integrator.integrate(80.0, 80.0, 80.0, 80.0, false, DT);
        }
        // Reversed power, motors not floating: the wheels ask for -80, the floor allows the traction limit.
        integrator.integrate(-80.0, -80.0, -80.0, -80.0, false, DT);
        assertEquals(80.0 - TRACTION * DT, integrator.getForwardVelocity(), 1e-9);
    }

    // ---------------------------------------------------------------------------------------- defaults

    @Test
    void withNothingConfiguredTheChassisGoesExactlyWhereItsWheelsSayFloatingOrNot() {
        MecanumPoseIntegrator integrator = new MecanumPoseIntegrator(config(false, false));
        integrator.integrate(90.0, 90.0, 90.0, 90.0, false, DT);
        assertEquals(90.0, integrator.getForwardVelocity(), 0.0, "no traction limit: one tick is enough");
        integrator.integrate(-90.0, -90.0, -90.0, -90.0, false, DT);
        assertEquals(-90.0, integrator.getForwardVelocity(), 0.0);
        integrator.integrate(10.0, 10.0, 10.0, 10.0, true, DT);
        assertEquals(10.0, integrator.getForwardVelocity(), 0.0, "no coast configured: floating follows the wheels");
    }

    @Test
    void badAccelerationNumbersAreRefused() {
        assertThrows(IllegalArgumentException.class, () -> new MecanumPoseIntegrator(
                config(-1.0, NATURAL_LATERAL, TRACTION, TRACTION)));
        assertThrows(IllegalArgumentException.class, () -> new MecanumPoseIntegrator(
                config(NATURAL_FORWARD, Double.NaN, TRACTION, TRACTION)));
        assertThrows(IllegalArgumentException.class, () -> new MecanumPoseIntegrator(
                config(NATURAL_FORWARD, NATURAL_LATERAL, 0.0, TRACTION)));
        assertThrows(IllegalArgumentException.class, () -> new MecanumPoseIntegrator(
                config(NATURAL_FORWARD, NATURAL_LATERAL, TRACTION, Double.NaN)));
    }

    // ---------------------------------------------------------------- through the plant and fake motors

    @Test
    void thePlantCoastsOnlyWhenEveryMotorIsAtZeroPowerAndFloating() {
        double floatDrop = speedLostInTenTicksAfterCuttingPower(DcMotor.ZeroPowerBehavior.FLOAT);
        double brakeDrop = speedLostInTenTicksAfterCuttingPower(DcMotor.ZeroPowerBehavior.BRAKE);
        assertEquals(NATURAL_FORWARD * 10 * DT, floatDrop, 1e-9, "FLOAT at zero power coasts");
        assertTrue(brakeDrop > floatDrop + 1.0,
                "BRAKE follows the motor model's decay instead, which is harder: " + brakeDrop);
        assertTrue(brakeDrop <= TRACTION * 10 * DT + 1e-9, "but still within traction: " + brakeDrop);
    }

    private static double speedLostInTenTicksAfterCuttingPower(DcMotor.ZeroPowerBehavior behaviour) {
        DrivetrainSimConfig config = config(true, true);
        FakeMotor[] motors = new FakeMotor[4];
        for (int i = 0; i < 4; i++) {
            motors[i] = new FakeMotor(config.maxVelocityTicksPerSecond(), config.maxAccel());
            motors[i].setZeroPowerBehavior(behaviour);
            motors[i].setPower(1.0);
        }
        MecanumDrivePlant plant = new MecanumDrivePlant(motors, config);
        for (int i = 0; i < 100; i++) {
            step(motors, plant);
        }
        double speedAtCut = plant.getForwardVelocity();
        for (FakeMotor motor : motors) {
            motor.setPower(0.0);
        }
        for (int i = 0; i < 10; i++) {
            step(motors, plant);
        }
        return speedAtCut - plant.getForwardVelocity();
    }

    private static void step(FakeMotor[] motors, MecanumDrivePlant plant) {
        for (FakeMotor motor : motors) {
            motor.update(DT);
        }
        plant.update(DT);
    }

    // ------------------------------------------------------------------------------------------ configs

    private static DrivetrainSimConfig config(boolean coast, boolean traction) {
        return config(coast ? NATURAL_FORWARD : 0.0, coast ? NATURAL_LATERAL : 0.0,
                traction ? TRACTION : Double.POSITIVE_INFINITY, traction ? TRACTION : Double.POSITIVE_INFINITY);
    }

    private static DrivetrainSimConfig config(double naturalForward, double naturalLateral,
            double maxForward, double maxLateral) {
        return new DrivetrainSimConfig() {
            @Override public double trackWidth() { return 13.5748; }
            @Override public double wheelBase() { return 10.6555; }
            @Override public double wheelRadius() { return 2.04724; }
            @Override public double ticksPerInch() { return TICKS_PER_INCH; }
            @Override public double maxVelocityTicksPerSecond() { return FORWARD_TOP_SPEED * TICKS_PER_INCH; }
            @Override public double maxLateralVelocityTicksPerSecond() { return STRAFE_TOP_SPEED * TICKS_PER_INCH; }
            @Override public double maxAccel() { return 4.0; }
            @Override public double fieldHalfWidth() { return 1000.0; }
            @Override public double fieldHalfHeight() { return 1000.0; }
            @Override public double naturalForwardDecelerationInchesPerSecondSquared() { return naturalForward; }
            @Override public double naturalLateralDecelerationInchesPerSecondSquared() { return naturalLateral; }
            @Override public double maxForwardAccelerationInchesPerSecondSquared() { return maxForward; }
            @Override public double maxLateralAccelerationInchesPerSecondSquared() { return maxLateral; }
        };
    }
}
