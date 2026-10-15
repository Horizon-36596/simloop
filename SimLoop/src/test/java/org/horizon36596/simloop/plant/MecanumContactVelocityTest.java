package org.horizon36596.simloop.plant;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.horizon36596.simloop.config.DrivetrainSimConfig;
import org.junit.jupiter.api.Test;

/**
 * INVARIANT-tier (domain R9) — the Contract: <b>a contact removes exactly the part of the chassis velocity that
 * points into it, and nothing else.</b> A robot driven into a wall at an angle keeps its speed along the wall
 * and loses its speed into it; a robot already moving away keeps everything. Without this a pushed-back robot
 * kept its full momentum into the wall, and the traction limit only let it lose that slowly, so it drove into
 * the wall at full speed every tick.
 *
 * <p>Inches, seconds, localizer frame, no clock (domain R5).
 */
class MecanumContactVelocityTest {

    private static final double DT = 0.02;
    private static final double TICKS_PER_INCH = 26.2;

    /** Four wheels all at speed v drive forward at v; a strafe sets the left velocity (no slip here). */
    private static MecanumPoseIntegrator movingAt(double forward, double left, double heading) {
        MecanumPoseIntegrator integrator = new MecanumPoseIntegrator(config());
        integrator.setPose(0.0, 0.0, heading);
        // Mecanum inverse kinematics: FL = f - l, FR = f + l, BL = f + l, BR = f - l.
        integrator.integrate(forward - left, forward + left, forward + left, forward - left, DT);
        return integrator;
    }

    @Test
    void headOnIntoAWallStopsTheRobot() {
        MecanumPoseIntegrator integrator = movingAt(40.0, 0.0, 0.0);
        integrator.removeVelocityInto(-1.0, 0.0);        // the wall pushes back along -x
        assertEquals(0.0, integrator.getForwardVelocity(), 1e-12);
        assertEquals(0.0, integrator.getLateralVelocity(), 1e-12);
    }

    @Test
    void atAnAngleTheRobotKeepsItsSpeedAlongTheWall() {
        MecanumPoseIntegrator integrator = movingAt(40.0, 30.0, 0.0);
        integrator.removeVelocityInto(-3.0, 0.0);        // need not be unit length
        assertEquals(0.0, integrator.getForwardVelocity(), 1e-12);
        assertEquals(30.0, integrator.getLateralVelocity(), 1e-12);
    }

    @Test
    void movingAwayFromTheContactKeepsEverything() {
        MecanumPoseIntegrator integrator = movingAt(40.0, 30.0, 0.0);
        integrator.removeVelocityInto(1.0, 0.0);
        assertEquals(40.0, integrator.getForwardVelocity(), 1e-12);
        assertEquals(30.0, integrator.getLateralVelocity(), 1e-12);
    }

    @Test
    void theNormalIsInTheLocalizerFrameWhateverTheHeading() {
        // Facing localizer +y and driving forward: the robot moves along +y. A wall pushing along -y stops it.
        MecanumPoseIntegrator integrator = movingAt(40.0, 0.0, StrictMath.PI / 2.0);
        integrator.removeVelocityInto(0.0, -1.0);
        assertEquals(0.0, integrator.getForwardVelocity(), 1e-9);
        MecanumPoseIntegrator sideways = movingAt(40.0, 0.0, StrictMath.PI / 2.0);
        sideways.removeVelocityInto(-1.0, 0.0);          // a wall across its side: it is not moving into that
        assertEquals(40.0, sideways.getForwardVelocity(), 1e-9);
    }

    @Test
    void aZeroOrNonFiniteNormalChangesNothing() {
        MecanumPoseIntegrator integrator = movingAt(40.0, 30.0, 0.0);
        integrator.removeVelocityInto(0.0, 0.0);
        integrator.removeVelocityInto(Double.NaN, 1.0);
        assertEquals(40.0, integrator.getForwardVelocity(), 1e-12);
        assertEquals(30.0, integrator.getLateralVelocity(), 1e-12);
    }

    private static DrivetrainSimConfig config() {
        return new DrivetrainSimConfig() {
            @Override public double trackWidth() { return 13.5748; }
            @Override public double wheelBase() { return 10.6555; }
            @Override public double wheelRadius() { return 2.04724; }
            @Override public double ticksPerInch() { return TICKS_PER_INCH; }
            @Override public double maxVelocityTicksPerSecond() { return 100.0 * TICKS_PER_INCH; }
            @Override public double maxLateralVelocityTicksPerSecond() { return 100.0 * TICKS_PER_INCH; }
            @Override public double maxAccel() { return 4.0; }
            @Override public double fieldHalfWidth() { return 1000.0; }
            @Override public double fieldHalfHeight() { return 1000.0; }
        };
    }
}
