package org.horizon36596.simloop.plant;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;

import org.horizon36596.simloop.config.DrivetrainSimConfig;
import org.horizon36596.simloop.fakehardware.FakeHardwareMap;
import org.horizon36596.simloop.fakehardware.FakeMotor;
import org.horizon36596.simloop.sim.FakeTimer;

import org.junit.jupiter.api.Test;

/**
 * Determinism of the drivetrain plant chain as a sim run uses it: four fake motors ticked through a
 * {@link FakeHardwareMap}, a real {@link MecanumDrivePlant} reading them, and a {@link FakeTimer}
 * advanced alongside. Run twice with an identical command schedule, it must produce a bit-for-bit
 * identical pose trajectory — the property replay depends on.
 */
class PlantDeterminismTest {

    private static final double DT = 0.02;
    private static final int LOOPS = 250;

    private static final DrivetrainSimConfig CONFIG = new DrivetrainSimConfig() {
        @Override public double trackWidth() { return 11.27362; }
        @Override public double wheelBase() { return 11.50976; }
        @Override public double wheelRadius() { return 1.88976; }
        @Override public double ticksPerInch() { return 100.0; }
        @Override public double maxVelocityTicksPerSecond() { return 93.0 * 100.0; }
        @Override public double maxAccel() { return 1.8; }
        @Override public double fieldHalfWidth() { return 72.0; }
        @Override public double fieldHalfHeight() { return 72.0; }
    };

    @Test
    void identicalRunsProduceIdenticalTrajectory() {
        assertArrayEquals(runOnce(), runOnce(),
                "two identical plant runs must yield bit-identical pose trajectories");
    }

    /** One headless run; returns the flattened [x, y, heading] trajectory, one triple per loop. */
    private static double[] runOnce() {
        FakeTimer timer = new FakeTimer();
        FakeHardwareMap map = new FakeHardwareMap();
        String[] names = {"FL", "FR", "BL", "BR"};
        FakeMotor[] motors = new FakeMotor[4];
        for (int i = 0; i < 4; i++) {
            motors[i] = new FakeMotor(CONFIG.maxVelocityTicksPerSecond(), CONFIG.maxAccel());
            map.register(names[i], motors[i]);
        }
        MecanumDrivePlant drivePlant = new MecanumDrivePlant(motors, CONFIG);

        double[] trajectory = new double[LOOPS * 3];
        for (int loop = 0; loop < LOOPS; loop++) {
            // Scripted routine: forward, then an arc, then a strafe — deterministic, no wall-clock.
            double forward = loop < 100 ? 1.0 : 0.5;
            double turn = loop >= 100 && loop < 175 ? 0.3 : 0.0;
            double strafe = loop >= 175 ? 0.4 : 0.0;
            motors[0].setPower(forward - turn - strafe); // FL
            motors[1].setPower(forward + turn + strafe); // FR
            motors[2].setPower(forward - turn + strafe); // BL
            motors[3].setPower(forward + turn - strafe); // BR

            // Same order as the harness: motors first, then the plant reads their new speeds.
            map.updateAll(DT);
            drivePlant.update(DT);
            timer.advance(DT);

            trajectory[loop * 3] = drivePlant.getX();
            trajectory[loop * 3 + 1] = drivePlant.getY();
            trajectory[loop * 3 + 2] = drivePlant.getHeading();
        }
        return trajectory;
    }
}
