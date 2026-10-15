package org.horizon36596.simloop.vision;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;

import org.junit.jupiter.api.Test;

/**
 * INVARIANT-tier (domain R9): the rigid-transform arithmetic every tag pose the fake Limelight reports is
 * built from. Composition and inversion must be exact inverses, and roll/pitch/yaw must decompose in the
 * WPILib {@code Rz * Ry * Rx} order that Limelight pose arrays are read in.
 */
class RigidTransform3Test {

    private static final double EPS = 1e-12;

    private static void assertSameTransform(RigidTransform3 expected, RigidTransform3 actual, double tol) {
        assertEquals(expected.getXInches(), actual.getXInches(), tol, "x");
        assertEquals(expected.getYInches(), actual.getYInches(), tol, "y");
        assertEquals(expected.getZInches(), actual.getZInches(), tol, "z");
        for (int row = 0; row < 3; row++) {
            for (int col = 0; col < 3; col++) {
                assertEquals(expected.rotation(row, col), actual.rotation(row, col), tol,
                        "rotation[" + row + "][" + col + "]");
            }
        }
    }

    @Test
    void aTransformComposedWithItsInverseIsTheIdentityBothWays() {
        RigidTransform3 t = RigidTransform3.fromRollPitchYaw(3.0, -7.5, 12.0, 0.3, -0.4, 2.1);
        assertSameTransform(RigidTransform3.identity(), t.compose(t.inverse()), EPS);
        assertSameTransform(RigidTransform3.identity(), t.inverse().compose(t), EPS);
    }

    @Test
    void applyingAComposedTransformIsApplyingEachInTurn() {
        RigidTransform3 ab = RigidTransform3.fromRollPitchYaw(1.0, 2.0, 3.0, 0.1, 0.2, 0.3);
        RigidTransform3 bc = RigidTransform3.fromRollPitchYaw(-4.0, 5.0, -6.0, -0.5, 0.25, 1.5);
        double[] inC = {0.7, -1.3, 2.9};
        double[] viaB = bc.apply(inC[0], inC[1], inC[2]);
        double[] expected = ab.apply(viaB[0], viaB[1], viaB[2]);
        double[] actual = ab.compose(bc).apply(inC[0], inC[1], inC[2]);
        for (int i = 0; i < 3; i++) {
            assertEquals(expected[i], actual[i], EPS);
        }
    }

    @Test
    void rollPitchYawRoundTripAwayFromGimbalLock() {
        double[][] cases = {
                {0.0, 0.0, 0.0}, {0.4, -0.3, 2.5}, {-2.9, 1.2, -1.0}, {1.5, -1.4, 3.1}, {0.0, 0.0, -3.0},
        };
        for (double[] rpy : cases) {
            RigidTransform3 t = RigidTransform3.fromRollPitchYaw(0.0, 0.0, 0.0, rpy[0], rpy[1], rpy[2]);
            assertEquals(rpy[0], t.getRollRad(), 1e-9, "roll");
            assertEquals(rpy[1], t.getPitchRad(), 1e-9, "pitch");
            assertEquals(rpy[2], t.getYawRad(), 1e-9, "yaw");
        }
    }

    @Test
    void yawIsCounterClockwiseAboutZAndTurnsXTowardY() {
        RigidTransform3 quarterTurn = RigidTransform3.planar(0.0, 0.0, Math.PI / 2.0);
        double[] p = quarterTurn.apply(1.0, 0.0, 0.0);
        assertEquals(0.0, p[0], EPS);
        assertEquals(1.0, p[1], EPS);
        assertEquals(0.0, p[2], EPS);
    }

    @Test
    void positivePitchTiltsXDownTowardMinusZ() {
        RigidTransform3 noseDown = RigidTransform3.fromRollPitchYaw(0.0, 0.0, 0.0, 0.0, 0.3, 0.0);
        double[] p = noseDown.apply(1.0, 0.0, 0.0);
        assertEquals(-StrictMath.sin(0.3), p[2], EPS);
    }

    @Test
    void atGimbalLockRollIsZeroAndYawCarriesTheRest() {
        RigidTransform3 t = RigidTransform3.fromRollPitchYaw(0.0, 0.0, 0.0, 0.0, Math.PI / 2.0, 0.7);
        assertEquals(Math.PI / 2.0, t.getPitchRad(), 1e-6);
        assertEquals(0.0, t.getRollRad(), 0.0);
        assertEquals(0.7, t.getYawRad(), 1e-9);
        RigidTransform3 rebuilt = RigidTransform3.fromRollPitchYaw(0.0, 0.0, 0.0, t.getRollRad(), t.getPitchRad(),
                t.getYawRad());
        assertSameTransform(t, rebuilt, 1e-6);
    }

    @Test
    void fromAxesRejectsAReflectionAndNonOrthonormalAxes() {
        assertThrows(IllegalArgumentException.class, () -> RigidTransform3.fromAxes(
                new double[] {1, 0, 0}, new double[] {0, 1, 0}, new double[] {0, 0, -1}, 0, 0, 0));
        assertThrows(IllegalArgumentException.class, () -> RigidTransform3.fromAxes(
                new double[] {1, 0, 0}, new double[] {0.1, 1, 0}, new double[] {0, 0, 1}, 0, 0, 0));
        assertThrows(IllegalArgumentException.class, () -> RigidTransform3.fromAxes(
                new double[] {1, 0, 0}, new double[] {0, 1, 0}, new double[] {0, 0, 1}, Double.NaN, 0, 0));
    }

    @Test
    void distanceIsTheTranslationLength() {
        assertEquals(13.0, RigidTransform3.fromRollPitchYaw(3.0, 4.0, 12.0, 1, 2, 3).getDistanceInches(), EPS);
    }
}
