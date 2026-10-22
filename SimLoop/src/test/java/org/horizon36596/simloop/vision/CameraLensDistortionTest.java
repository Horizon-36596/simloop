package org.horizon36596.simloop.vision;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

/**
 * INVARIANT-tier (domain R9) — the lens model's distortion, which every camera in this package projects
 * through:
 *
 * <ol>
 *   <li>with no coefficients it is the old pinhole, bit for bit;</li>
 *   <li>the forward model is OpenCV's Brown-Conrady formula, checked against values worked by hand;</li>
 *   <li>{@code undistortPixel} inverts {@code project} everywhere in the image;</li>
 *   <li>past the fold-over radius a point is not imaged, so a far-off point cannot land inside the
 *       picture;</li>
 *   <li>the local magnification is 1 for a pinhole and shrinks toward the corners under barrel
 *       distortion.</li>
 * </ol>
 *
 * <p>The coefficients are made up, in the range a cheap wide webcam's calibration gives.
 */
class CameraLensDistortionTest {

    private static final CameraLens PINHOLE = CameraLens.fromHorizontalFov(70.0, 640, 480);
    private static final CameraLens BARREL = PINHOLE.withDistortion(-0.30, 0.08, 0.001, -0.0008, -0.01);

    @Test
    void noCoefficientsIsThePinholeBitForBit() {
        double[][] points = {{1.0, 2.0, 30.0}, {-7.5, 3.25, 11.0}, {0.0, 0.0, 5.0}, {40.0, -30.0, 12.0}};
        for (double[] p : points) {
            double[] px = PINHOLE.project(p[0], p[1], p[2]);
            assertEquals(PINHOLE.getCentreXPx() + PINHOLE.getFocalXPx() * p[0] / p[2], px[0], 0.0);
            assertEquals(PINHOLE.getCentreYPx() + PINHOLE.getFocalYPx() * p[1] / p[2], px[1], 0.0);
        }
        assertFalse(PINHOLE.hasDistortion());
        assertEquals(Double.POSITIVE_INFINITY, PINHOLE.getMaxValidRadius());
        assertArrayEquals(new double[] {0.1, -0.2}, PINHOLE.undistortPixel(
                PINHOLE.getCentreXPx() + 0.1 * PINHOLE.getFocalXPx(),
                PINHOLE.getCentreYPx() - 0.2 * PINHOLE.getFocalYPx()), 1e-15);
    }

    @Test
    void radialTermsMatchTheFormulaWorkedByHand() {
        CameraLens lens = PINHOLE.withDistortion(-0.2, 0.0, 0.0, 0.0, 0.0);
        // r2 = 0.09 + 0.01 = 0.1; radial = 1 - 0.02 = 0.98.
        assertArrayEquals(new double[] {0.294, 0.098}, lens.distortNormalized(0.3, 0.1), 1e-15);
        double[] px = lens.project(0.3, 0.1, 1.0);
        assertEquals(lens.getCentreXPx() + lens.getFocalXPx() * 0.294, px[0], 1e-9);
        assertEquals(lens.getCentreYPx() + lens.getFocalYPx() * 0.098, px[1], 1e-9);
    }

    @Test
    void tangentialTermsMatchTheFormulaWorkedByHand() {
        CameraLens lens = PINHOLE.withDistortion(0.0, 0.0, 0.01, -0.005, 0.0);
        // x = 0.2, y = -0.1, r2 = 0.05.
        // xd = 0.2 + 2(0.01)(0.2)(-0.1) + (-0.005)(0.05 + 0.08) = 0.2 - 0.0004 - 0.00065 = 0.19895
        // yd = -0.1 + 0.01(0.05 + 0.02) + 2(-0.005)(0.2)(-0.1) = -0.1 + 0.0007 + 0.0002 = -0.0991
        assertArrayEquals(new double[] {0.19895, -0.0991}, lens.distortNormalized(0.2, -0.1), 1e-15);
    }

    @Test
    void undistortInvertsProjectEverywhereInTheImage() {
        double worst = 0.0;
        for (int y = 0; y <= 480; y += 16) {
            for (int x = 0; x <= 640; x += 16) {
                double[] ray = BARREL.undistortPixel(x, y);
                double[] back = BARREL.project(ray[0], ray[1], 1.0);
                assertNotNull(back, "pixel (" + x + ", " + y + ") came back outside the valid radius");
                worst = Math.max(worst, Math.hypot(back[0] - x, back[1] - y));
            }
        }
        assertTrue(worst < 1e-9, "worst round trip " + worst + " px");
    }

    @Test
    void barrelDistortionPullsTheCornersIn() {
        double[] corner = PINHOLE.undistortPixel(0.0, 0.0);
        double[] distorted = BARREL.project(corner[0], corner[1], 1.0);
        assertTrue(distorted[0] > 0.0 && distorted[1] > 0.0, "barrel pulls a corner point toward the centre");
        double[] centre = BARREL.project(0.0, 0.0, 1.0);
        assertEquals(BARREL.getCentreXPx(), centre[0], 1e-12);
        assertEquals(BARREL.getCentreYPx(), centre[1], 1e-12);
    }

    @Test
    void pastTheFoldOverRadiusNothingIsImaged() {
        CameraLens lens = PINHOLE.withDistortion(-0.3, 0.0, 0.0, 0.0, 0.0);
        // d/dr of r(1 - 0.3 r^2) is 1 - 0.9 r^2: zero at r = 1 / sqrt(0.9).
        assertEquals(1.0 / Math.sqrt(0.9), lens.getMaxValidRadius(), 1e-9);
        assertNotNull(lens.project(1.0, 0.0, 1.0));
        assertNull(lens.project(1.5, 0.0, 1.0), "r = 1.5 would fold back to 1.5(1 - 0.675) = 0.49: inside the image");
        assertNull(lens.project(0.0, 0.0, -1.0), "behind the lens");
        assertNull(lens.project(0.0, 0.0, 0.0), "on the lens plane");
    }

    @Test
    void localMagnificationIsOneForAPinholeAndShrinksTowardTheCorners() {
        assertEquals(1.0, PINHOLE.localMagnification(0.4, 0.3), 0.0);
        assertEquals(1.0, BARREL.localMagnification(0.0, 0.0), 1e-12);
        assertTrue(BARREL.localMagnification(0.6, 0.4) < BARREL.localMagnification(0.2, 0.1));
        assertTrue(BARREL.localMagnification(0.6, 0.4) < 1.0);
    }

    @Test
    void withDistortionKeepsTheIntrinsics() {
        assertEquals(PINHOLE.getFocalXPx(), BARREL.getFocalXPx(), 0.0);
        assertEquals(PINHOLE.getCentreYPx(), BARREL.getCentreYPx(), 0.0);
        assertEquals(PINHOLE.getWidthPx(), BARREL.getWidthPx());
        assertArrayEquals(new double[] {-0.30, 0.08, 0.001, -0.0008, -0.01}, BARREL.getDistortionCoefficients(), 0.0);
        assertTrue(BARREL.hasDistortion());
        assertTrue(BARREL.toString().contains("k1=-0.3"));
    }

    @Test
    void nonFiniteCoefficientsAreRejected() {
        assertThrows(IllegalArgumentException.class, () -> PINHOLE.withDistortion(Double.NaN, 0, 0, 0, 0));
        assertThrows(IllegalArgumentException.class,
                () -> PINHOLE.withDistortion(0, 0, 0, 0, Double.POSITIVE_INFINITY));
    }
}
