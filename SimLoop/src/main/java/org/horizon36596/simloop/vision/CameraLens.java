package org.horizon36596.simloop.vision;

/**
 * A camera's intrinsics: focal lengths and principal point in pixels, the image size, and optionally a lens
 * distortion model. This is the same model, with the same quantities, as an OpenCV camera matrix
 * ({@code fx, fy, cx, cy} plus {@code widthPx, heightPx}) and OpenCV's five distortion coefficients in
 * OpenCV's order ({@code k1, k2, p1, p2, k3}), so a lens described once, or copied out of a real camera's
 * calibration, can be handed to both the sim camera and a real OpenCV-based pipeline unchanged.
 *
 * <p><b>The distortion model</b> is the Brown-Conrady model OpenCV's {@code projectPoints} uses. A point
 * on the image plane at normalised coordinates {@code (x, y) = (X / Z, Y / Z)} in the optical frame lands
 * at
 * <pre>
 *   r2 = x*x + y*y
 *   radial = 1 + k1*r2 + k2*r2^2 + k3*r2^3
 *   xd = x*radial + 2*p1*x*y + p2*(r2 + 2*x*x)
 *   yd = y*radial + p1*(r2 + 2*y*y) + 2*p2*x*y
 *   pixel = (cx + fx*xd, cy + fy*yd)
 * </pre>
 * With every coefficient zero (the default, and what the six-argument constructor gives) it is the ideal
 * pinhole, computed with exactly the arithmetic SimLoop has always used, so a lens without distortion gives
 * bit-identical results to earlier versions.
 *
 * <p><b>Where the model stops.</b> A strong barrel distortion ({@code k1 < 0}) is not one-to-one far
 * outside the image: past some radius the distorted radius turns round and shrinks again, so a point well
 * outside the field of view would land back inside the picture. A real lens does not do that; the model
 * simply stops describing it. This class finds that radius once, at construction
 * ({@link #getMaxValidRadius()}), and {@link #project} reports a point beyond it as not imaged. Only the
 * radial terms are used to find it; the tangential terms are small enough in any real calibration not to
 * move it.
 *
 * <p><b>Synthetic images are never ground truth for a real lens.</b> Distortion here makes the sim's
 * geometry match a calibrated lens; it does not make a rendered frame look like that lens's pictures, and
 * colour thresholds must never be tuned on rendered frames (see {@link SimFrameRenderer}).
 *
 * <p>Pixel frame: {@code (0, 0)} is the top-left corner, {@code +x} right, {@code +y} down, the OpenCV
 * convention.
 *
 * <p>Immutable and pure.
 */
public final class CameraLens {

    /** Newton steps {@link #undistortPixel} takes at most; it converges in a handful inside the image. */
    private static final int UNDISTORT_MAX_STEPS = 50;

    /** Normalised radius past which the fold-over search stops looking: about 84 degrees off the axis. */
    private static final double FOLD_SEARCH_LIMIT = 10.0;

    private final double focalXPx;
    private final double focalYPx;
    private final double centreXPx;
    private final double centreYPx;
    private final int widthPx;
    private final int heightPx;
    private final double k1;
    private final double k2;
    private final double p1;
    private final double p2;
    private final double k3;
    private final boolean distorted;
    private final double maxValidRadius;

    /**
     * A pinhole lens with the given intrinsics and no distortion.
     *
     * @param focalXPx  horizontal focal length, pixels
     * @param focalYPx  vertical focal length, pixels (equal to {@code focalXPx} for square pixels)
     * @param centreXPx principal point x, pixels from the left edge
     * @param centreYPx principal point y, pixels from the top edge
     * @param widthPx   image width, pixels
     * @param heightPx  image height, pixels
     * @throws IllegalArgumentException on a non-positive focal length or size, or a non-finite number
     */
    public CameraLens(double focalXPx, double focalYPx, double centreXPx, double centreYPx, int widthPx,
            int heightPx) {
        this(focalXPx, focalYPx, centreXPx, centreYPx, widthPx, heightPx, 0.0, 0.0, 0.0, 0.0, 0.0);
    }

    /**
     * A lens with the given intrinsics and Brown-Conrady distortion, coefficients in OpenCV's order: the
     * numbers a camera calibration ({@code calibrateCamera}, or a {@code teamwebcamcalibrations.xml} entry)
     * produces.
     *
     * @param focalXPx  horizontal focal length, pixels
     * @param focalYPx  vertical focal length, pixels
     * @param centreXPx principal point x, pixels from the left edge
     * @param centreYPx principal point y, pixels from the top edge
     * @param widthPx   image width, pixels
     * @param heightPx  image height, pixels
     * @param k1        first radial coefficient, dimensionless; negative is barrel distortion
     * @param k2        second radial coefficient, dimensionless
     * @param p1        first tangential coefficient, dimensionless
     * @param p2        second tangential coefficient, dimensionless
     * @param k3        third radial coefficient, dimensionless
     * @throws IllegalArgumentException on a non-positive focal length or size, or a non-finite number
     */
    public CameraLens(double focalXPx, double focalYPx, double centreXPx, double centreYPx, int widthPx,
            int heightPx, double k1, double k2, double p1, double p2, double k3) {
        if (!Double.isFinite(focalXPx) || focalXPx <= 0.0 || !Double.isFinite(focalYPx) || focalYPx <= 0.0) {
            throw new IllegalArgumentException("focal lengths must be positive and finite");
        }
        if (!Double.isFinite(centreXPx) || !Double.isFinite(centreYPx)) {
            throw new IllegalArgumentException("principal point must be finite");
        }
        if (widthPx <= 0 || heightPx <= 0) {
            throw new IllegalArgumentException("image size must be positive, got " + widthPx + "x" + heightPx);
        }
        if (!Double.isFinite(k1) || !Double.isFinite(k2) || !Double.isFinite(p1) || !Double.isFinite(p2)
                || !Double.isFinite(k3)) {
            throw new IllegalArgumentException("distortion coefficients must be finite, got k1=" + k1 + " k2="
                    + k2 + " p1=" + p1 + " p2=" + p2 + " k3=" + k3);
        }
        this.focalXPx = focalXPx;
        this.focalYPx = focalYPx;
        this.centreXPx = centreXPx;
        this.centreYPx = centreYPx;
        this.widthPx = widthPx;
        this.heightPx = heightPx;
        this.k1 = k1;
        this.k2 = k2;
        this.p1 = p1;
        this.p2 = p2;
        this.k3 = k3;
        this.distorted = k1 != 0.0 || k2 != 0.0 || p1 != 0.0 || p2 != 0.0 || k3 != 0.0;
        this.maxValidRadius = foldOverRadius(k1, k2, k3);
    }

    /**
     * A square-pixel lens from its horizontal field of view, principal point at the image centre, no
     * distortion — the way a datasheet describes a webcam: {@code fx = fy = (width / 2) / tan(hfov / 2)}.
     *
     * @param horizontalFovDegrees the full horizontal field of view, degrees, in {@code (0, 180)}
     * @param widthPx              image width, pixels
     * @param heightPx             image height, pixels
     * @return the lens
     */
    public static CameraLens fromHorizontalFov(double horizontalFovDegrees, int widthPx, int heightPx) {
        if (!(horizontalFovDegrees > 0.0 && horizontalFovDegrees < 180.0)) {
            throw new IllegalArgumentException("horizontal FOV must be in (0, 180) degrees, got "
                    + horizontalFovDegrees);
        }
        double halfFovRad = StrictMath.toRadians(horizontalFovDegrees) / 2.0;
        double focal = (widthPx / 2.0) / StrictMath.tan(halfFovRad);
        return new CameraLens(focal, focal, widthPx / 2.0, heightPx / 2.0, widthPx, heightPx);
    }

    /**
     * This lens with a different distortion model: same focal lengths, principal point and image size.
     *
     * @param k1 first radial coefficient
     * @param k2 second radial coefficient
     * @param p1 first tangential coefficient
     * @param p2 second tangential coefficient
     * @param k3 third radial coefficient
     * @return the new lens
     */
    public CameraLens withDistortion(double k1, double k2, double p1, double p2, double k3) {
        return new CameraLens(focalXPx, focalYPx, centreXPx, centreYPx, widthPx, heightPx, k1, k2, p1, p2, k3);
    }

    /**
     * Where a point in the optical frame lands in the image, distortion included.
     *
     * @param xOptical the point's optical {@code x} (image right), any length unit
     * @param yOptical the point's optical {@code y} (image down), same unit
     * @param zOptical the point's depth along the boresight, same unit
     * @return {@code {x, y}} in pixels, or {@code null} when the point is not in front of the lens or lies
     *         beyond {@link #getMaxValidRadius()}, where the distortion model no longer describes a lens.
     *         The pixel may lie outside the image rectangle; checking that is the caller's job.
     */
    public double[] project(double xOptical, double yOptical, double zOptical) {
        if (!(zOptical > 0.0)) {
            return null;
        }
        if (!distorted) {
            // Exactly the arithmetic every earlier version used, so an undistorted lens is bit-identical.
            return new double[] {centreXPx + focalXPx * xOptical / zOptical,
                    centreYPx + focalYPx * yOptical / zOptical};
        }
        double x = xOptical / zOptical;
        double y = yOptical / zOptical;
        if (x * x + y * y > maxValidRadius * maxValidRadius) {
            return null;
        }
        double[] d = distortNormalized(x, y);
        return new double[] {centreXPx + focalXPx * d[0], centreYPx + focalYPx * d[1]};
    }

    /**
     * The distortion model on its own: undistorted normalised image coordinates in, distorted ones out.
     *
     * @param x undistorted {@code X / Z}
     * @param y undistorted {@code Y / Z}
     * @return {@code {xd, yd}}, the distorted normalised coordinates
     */
    public double[] distortNormalized(double x, double y) {
        if (!distorted) {
            return new double[] {x, y};
        }
        double r2 = x * x + y * y;
        double radial = 1.0 + r2 * (k1 + r2 * (k2 + r2 * k3));
        double xd = x * radial + 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x);
        double yd = y * radial + p1 * (r2 + 2.0 * y * y) + 2.0 * p2 * x * y;
        return new double[] {xd, yd};
    }

    /**
     * The inverse of {@link #project}'s image step: a pixel in, the undistorted normalised direction it
     * looks along out. This is what OpenCV's {@code undistortPoints} computes, and what a ground-plane
     * pipeline does before casting a ray through a pixel. Newton's method on the distortion model, started
     * from the distorted point; deterministic.
     *
     * @param xPx pixel x, from the left edge
     * @param yPx pixel y, from the top edge
     * @return {@code {X / Z, Y / Z}} of the ray through the pixel, optical frame
     */
    public double[] undistortPixel(double xPx, double yPx) {
        double xd = (xPx - centreXPx) / focalXPx;
        double yd = (yPx - centreYPx) / focalYPx;
        if (!distorted) {
            return new double[] {xd, yd};
        }
        double x = xd;
        double y = yd;
        for (int step = 0; step < UNDISTORT_MAX_STEPS; step++) {
            double r2 = x * x + y * y;
            double radial = 1.0 + r2 * (k1 + r2 * (k2 + r2 * k3));
            double fx = x * radial + 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x) - xd;
            double fy = y * radial + p1 * (r2 + 2.0 * y * y) + 2.0 * p2 * x * y - yd;
            // d(radial)/d(r2), times 2 so that d(radial)/dx = slope * x.
            double slope = 2.0 * (k1 + r2 * (2.0 * k2 + 3.0 * k3 * r2));
            double jxx = radial + x * x * slope + 2.0 * p1 * y + 6.0 * p2 * x;
            double jxy = x * y * slope + 2.0 * p1 * x + 2.0 * p2 * y;
            double jyy = radial + y * y * slope + 6.0 * p1 * y + 2.0 * p2 * x;
            double det = jxx * jyy - jxy * jxy;
            if (det == 0.0 || !Double.isFinite(det)) {
                break;
            }
            double dx = (jyy * fx - jxy * fy) / det;
            double dy = (jxx * fy - jxy * fx) / det;
            x -= dx;
            y -= dy;
            if (Math.abs(dx) < 1e-15 && Math.abs(dy) < 1e-15) {
                break;
            }
        }
        return new double[] {x, y};
    }

    /**
     * How much the lens magnifies a small patch at this undistorted normalised position, relative to the
     * pinhole: the square root of the distortion map's Jacobian determinant. 1 everywhere for an
     * undistorted lens; below 1 toward the corners under barrel distortion. {@link SimCamera} scales a
     * piece's apparent radius by it.
     *
     * @param x undistorted {@code X / Z}
     * @param y undistorted {@code Y / Z}
     * @return the linear magnification, dimensionless
     */
    public double localMagnification(double x, double y) {
        if (!distorted) {
            return 1.0;
        }
        double r2 = x * x + y * y;
        double radial = 1.0 + r2 * (k1 + r2 * (k2 + r2 * k3));
        double slope = 2.0 * (k1 + r2 * (2.0 * k2 + 3.0 * k3 * r2));
        double jxx = radial + x * x * slope + 2.0 * p1 * y + 6.0 * p2 * x;
        double jxy = x * y * slope + 2.0 * p1 * x + 2.0 * p2 * y;
        double jyy = radial + y * y * slope + 6.0 * p1 * y + 2.0 * p2 * x;
        return StrictMath.sqrt(Math.abs(jxx * jyy - jxy * jxy));
    }

    /**
     * The normalised radius at which {@code r * radial(r)} stops growing, found by a fixed-step scan and a
     * bisection; infinite when it keeps growing out to {@link #FOLD_SEARCH_LIMIT}.
     */
    private static double foldOverRadius(double k1, double k2, double k3) {
        if (k1 >= 0.0 && k2 >= 0.0 && k3 >= 0.0) {
            return Double.POSITIVE_INFINITY;                // every term grows: never folds
        }
        double step = 0.001;
        for (double r = step; r <= FOLD_SEARCH_LIMIT; r += step) {
            if (radialSlope(r, k1, k2, k3) <= 0.0) {
                double low = r - step;
                double high = r;
                for (int i = 0; i < 60; i++) {
                    double mid = 0.5 * (low + high);
                    if (radialSlope(mid, k1, k2, k3) <= 0.0) {
                        high = mid;
                    } else {
                        low = mid;
                    }
                }
                return low;
            }
        }
        return Double.POSITIVE_INFINITY;
    }

    /** d/dr of {@code r * (1 + k1 r^2 + k2 r^4 + k3 r^6)}. */
    private static double radialSlope(double r, double k1, double k2, double k3) {
        double r2 = r * r;
        return 1.0 + 3.0 * k1 * r2 + 5.0 * k2 * r2 * r2 + 7.0 * k3 * r2 * r2 * r2;
    }

    /** {@return horizontal focal length, pixels} */
    public double getFocalXPx() {
        return focalXPx;
    }

    /** {@return vertical focal length, pixels} */
    public double getFocalYPx() {
        return focalYPx;
    }

    /** {@return principal point x, pixels from the left edge} */
    public double getCentreXPx() {
        return centreXPx;
    }

    /** {@return principal point y, pixels from the top edge} */
    public double getCentreYPx() {
        return centreYPx;
    }

    /** {@return image width, pixels} */
    public int getWidthPx() {
        return widthPx;
    }

    /** {@return image height, pixels} */
    public int getHeightPx() {
        return heightPx;
    }

    /** {@return the first radial distortion coefficient, OpenCV's {@code k1}} */
    public double getK1() {
        return k1;
    }

    /** {@return the second radial distortion coefficient, OpenCV's {@code k2}} */
    public double getK2() {
        return k2;
    }

    /** {@return the first tangential distortion coefficient, OpenCV's {@code p1}} */
    public double getP1() {
        return p1;
    }

    /** {@return the second tangential distortion coefficient, OpenCV's {@code p2}} */
    public double getP2() {
        return p2;
    }

    /** {@return the third radial distortion coefficient, OpenCV's {@code k3}} */
    public double getK3() {
        return k3;
    }

    /** {@return the five coefficients in OpenCV's order, {@code {k1, k2, p1, p2, k3}}, as a fresh array} */
    public double[] getDistortionCoefficients() {
        return new double[] {k1, k2, p1, p2, k3};
    }

    /** {@return true when any distortion coefficient is non-zero} */
    public boolean hasDistortion() {
        return distorted;
    }

    /**
     * The normalised image radius ({@code sqrt((X/Z)^2 + (Y/Z)^2)}) beyond which the distortion model is
     * not one-to-one, so {@link #project} reports nothing there.
     *
     * @return the radius, dimensionless; infinite for a lens that never folds over (including no distortion)
     */
    public double getMaxValidRadius() {
        return maxValidRadius;
    }

    /** {@return the full horizontal field of view of the pinhole part of this lens, degrees} */
    public double getHorizontalFovDegrees() {
        return StrictMath.toDegrees(2.0 * StrictMath.atan((widthPx / 2.0) / focalXPx));
    }

    @Override
    public String toString() {
        return "CameraLens(" + widthPx + "x" + heightPx + ", fx=" + focalXPx
                + (distorted ? ", k1=" + k1 + " k2=" + k2 + " p1=" + p1 + " p2=" + p2 + " k3=" + k3 : "") + ")";
    }
}
