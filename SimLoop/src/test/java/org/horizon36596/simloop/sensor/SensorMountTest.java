package org.horizon36596.simloop.sensor;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;

import org.junit.jupiter.api.Test;

/**
 * Pins {@link SensorMount}'s frames: a mount described in the robot frame (forward, left, up; yaw CCW,
 * pitch positive down) lands on the field where the FTC-Cartesian field frame says it should, for any robot
 * pose. INVARIANT tier (domain R9): every world-bound sensor reads through this conversion.
 */
class SensorMountTest {

    private static final double EPS = 1e-12;

    @Test
    void invariant_aMountOnTheNoseOfARobotFacingPlusXIsAheadOnPlusX() {
        SensorMount mount = new SensorMount(9.0, 2.0, 3.0, 0.0, 0.0);
        double[] face = mount.fieldPosition(10.0, 20.0, 0.0);

        assertEquals(19.0, face[0], EPS, "forward is +X for heading 0");
        assertEquals(22.0, face[1], EPS, "left is +Y for heading 0");
        assertEquals(3.0, face[2], EPS, "up is height above the floor");
    }

    @Test
    void invariant_turningTheRobotCounterClockwiseCarriesTheMountWithIt() {
        SensorMount mount = new SensorMount(9.0, 2.0, 3.0, 0.0, 0.0);
        double[] face = mount.fieldPosition(0.0, 0.0, Math.PI / 2.0);

        assertEquals(-2.0, face[0], EPS, "facing +Y, the robot's left is -X");
        assertEquals(9.0, face[1], EPS, "and its nose is +Y");
    }

    @Test
    void theDirectionIsTheRobotHeadingPlusTheYawTiltedByThePitch() {
        double[] level = SensorMount.level(0.0, 0.0, 3.0, Math.PI / 2.0).fieldDirection(0.0);
        assertEquals(0.0, level[0], EPS);
        assertEquals(1.0, level[1], EPS, "yaw pi/2 looks out the robot's left, +Y at heading 0");
        assertEquals(0.0, level[2], EPS);

        double[] down = new SensorMount(0.0, 0.0, 3.0, 0.0, Math.PI / 6.0).fieldDirection(Math.PI);
        assertEquals(-Math.cos(Math.PI / 6.0), down[0], EPS, "a robot facing -X, sensor looking forward");
        assertEquals(-0.5, down[2], EPS, "30 degrees down");
        assertEquals(1.0, down[0] * down[0] + down[1] * down[1] + down[2] * down[2], EPS, "unit length");

        double[] straightDown = new SensorMount(0.0, 0.0, 3.0, 0.0, Math.PI / 2.0).fieldDirection(1.0);
        assertEquals(-1.0, straightDown[2], EPS);
    }

    @Test
    void badMountsAreRefused() {
        assertThrows(IllegalArgumentException.class, () -> new SensorMount(0.0, 0.0, -0.1, 0.0, 0.0));
        assertThrows(IllegalArgumentException.class, () -> new SensorMount(Double.NaN, 0.0, 1.0, 0.0, 0.0));
        assertThrows(IllegalArgumentException.class, () -> new SensorMount(0.0, 0.0, 1.0, 0.0, 1.6));
        assertThrows(IllegalArgumentException.class,
                () -> new SensorMount(0.0, 0.0, 1.0, Double.POSITIVE_INFINITY, 0.0));
    }

    @Test
    void theGettersReportWhatWasGiven() {
        SensorMount mount = new SensorMount(1.0, 2.0, 3.0, 0.4, 0.5);
        assertEquals(1.0, mount.getForwardInches(), 0.0);
        assertEquals(2.0, mount.getLeftInches(), 0.0);
        assertEquals(3.0, mount.getUpInches(), 0.0);
        assertEquals(0.4, mount.getYawRad(), 0.0);
        assertEquals(0.5, mount.getPitchRad(), 0.0);
    }
}
