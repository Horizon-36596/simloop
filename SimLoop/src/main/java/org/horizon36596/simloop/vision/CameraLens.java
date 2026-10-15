package org.horizon36596.simloop.vision;

/**
 * A pinhole camera's intrinsics: focal lengths and principal point in pixels, and the image size. This is
 * the same model, with the same quantities, as an OpenCV camera matrix ({@code fx, fy, cx, cy} plus
 * {@code widthPx, heightPx}), so a lens described once can be handed to both the sim camera and a real
 * OpenCV-based pipeline unchanged.
 *
 * <p><b>No distortion.</b> A sim camera projects with the ideal pinhole model only. That is deliberate: a
 * vision pipeline's geometry (where a pixel is on the floor) can be tested against an ideal lens, while
 * its distortion handling and its colour thresholds need real frames from a real camera. Synthetic frames
 * must never tune a colour range: a range fitted to a renderer's flat colours says nothing about real
 * lighting.
 *
 * <p>Pixel frame: {@code (0, 0)} is the top-left corner, {@code +x} right, {@code +y} down, the OpenCV
 * convention.
 *
 * <p>Immutable and pure.
 */
public final class CameraLens {

    private final double focalXPx;
    private final double focalYPx;
    private final double centreXPx;
    private final double centreYPx;
    private final int widthPx;
    private final int heightPx;

    /**
     * A pinhole lens with the given intrinsics.
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
        if (!Double.isFinite(focalXPx) || focalXPx <= 0.0 || !Double.isFinite(focalYPx) || focalYPx <= 0.0) {
            throw new IllegalArgumentException("focal lengths must be positive and finite");
        }
        if (!Double.isFinite(centreXPx) || !Double.isFinite(centreYPx)) {
            throw new IllegalArgumentException("principal point must be finite");
        }
        if (widthPx <= 0 || heightPx <= 0) {
            throw new IllegalArgumentException("image size must be positive, got " + widthPx + "x" + heightPx);
        }
        this.focalXPx = focalXPx;
        this.focalYPx = focalYPx;
        this.centreXPx = centreXPx;
        this.centreYPx = centreYPx;
        this.widthPx = widthPx;
        this.heightPx = heightPx;
    }

    /**
     * A square-pixel lens from its horizontal field of view, principal point at the image centre — the
     * way a datasheet describes a webcam: {@code fx = fy = (width / 2) / tan(hfov / 2)}.
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

    /** {@return the full horizontal field of view this lens gives, degrees} */
    public double getHorizontalFovDegrees() {
        return StrictMath.toDegrees(2.0 * StrictMath.atan((widthPx / 2.0) / focalXPx));
    }

    @Override
    public String toString() {
        return "CameraLens(" + widthPx + "x" + heightPx + ", fx=" + focalXPx + ")";
    }
}
