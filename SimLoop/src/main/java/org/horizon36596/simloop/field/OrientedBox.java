package org.horizon36596.simloop.field;

/**
 * A solid box that may be turned any way: a centre, three perpendicular unit axes and a half extent along each.
 * It is the one shape the robot is built from ({@link RobotShape}) and the shape a parked robot
 * ({@link StaticRobot}) and a field {@link ObstacleBox} become when the robot meets them.
 *
 * <p><b>Frame and units.</b> Whatever frame the numbers are given in, inches: a robot's parts are built in the
 * robot frame (+X forward, +Y left, +Z up, origin on the floor under the robot centre) and {@link #placedOnRobot
 * placed} into the field frame (FTC-Cartesian, centre origin, +Z up, conventions §7) once per tick. Axes are rows:
 * {@code axis(0)} is the box's own x direction expressed in the frame the box lives in.
 *
 * <p><b>What it answers.</b> Whether two boxes overlap (the separating-axis test over the fifteen axes two boxes
 * in space have), the shortest <em>horizontal</em> move that takes one out of the other
 * ({@link #horizontalPushOutOf}), and the point of the box nearest a sphere's centre ({@link #closestPoint}).
 * Every trigonometric call is {@link StrictMath} and nothing allocates in an order that matters, so two runs give
 * bit-identical answers (domain R5). Immutable and season-agnostic (domain R7).
 */
public final class OrientedBox {

    private final String name;
    private final double centreX;
    private final double centreY;
    private final double centreZ;
    private final double[][] axes;
    private final double[] half;

    /**
     * A box from its centre, its three axes and its half extents, all in the frame it lives in.
     *
     * @param name    a short label for contact logs, non-blank
     * @param centreX centre, inches
     * @param centreY centre, inches
     * @param centreZ centre, inches
     * @param axes    three rows, the box's own x, y and z directions; each unit length and perpendicular to the
     *                others within 1e-6 (copied)
     * @param halfX   half extent along {@code axes[0]}, inches, &gt; 0
     * @param halfY   half extent along {@code axes[1]}, inches, &gt; 0
     * @param halfZ   half extent along {@code axes[2]}, inches, &gt; 0
     * @throws IllegalArgumentException on a blank name, a non-finite number, a non-positive half extent, or axes
     *                                  that are not orthonormal
     */
    public OrientedBox(String name, double centreX, double centreY, double centreZ, double[][] axes,
            double halfX, double halfY, double halfZ) {
        if (name == null || name.trim().isEmpty()) {
            throw new IllegalArgumentException("an oriented box needs a non-blank name");
        }
        requireFinite(centreX, "centreX");
        requireFinite(centreY, "centreY");
        requireFinite(centreZ, "centreZ");
        requirePositive(halfX, "halfX");
        requirePositive(halfY, "halfY");
        requirePositive(halfZ, "halfZ");
        if (axes == null || axes.length != 3) {
            throw new IllegalArgumentException("an oriented box needs exactly three axes");
        }
        double[][] copy = new double[3][];
        for (int i = 0; i < 3; i++) {
            if (axes[i] == null || axes[i].length != 3) {
                throw new IllegalArgumentException("axis " + i + " must have three components");
            }
            copy[i] = axes[i].clone();
            for (double component : copy[i]) {
                requireFinite(component, "axis component");
            }
        }
        for (int i = 0; i < 3; i++) {
            for (int j = i; j < 3; j++) {
                double dot = dot(copy[i], copy[j]);
                double expected = i == j ? 1.0 : 0.0;
                if (Math.abs(dot - expected) > 1e-6) {
                    throw new IllegalArgumentException("axes must be orthonormal; axis " + i + " . axis " + j
                            + " = " + dot);
                }
            }
        }
        this.name = name.trim();
        this.centreX = centreX;
        this.centreY = centreY;
        this.centreZ = centreZ;
        this.axes = copy;
        this.half = new double[] {halfX, halfY, halfZ};
    }

    /**
     * {@return a box lined up with the frame's axes, from its centre and half extents, inches}
     *
     * @param name    a short label for contact logs, non-blank
     * @param centreX centre, inches
     * @param centreY centre, inches
     * @param centreZ centre, inches
     * @param halfX   half extent along the frame's x, inches, &gt; 0
     * @param halfY   half extent along the frame's y, inches, &gt; 0
     * @param halfZ   half extent along the frame's z, inches, &gt; 0
     */
    public static OrientedBox axisAligned(String name, double centreX, double centreY, double centreZ,
            double halfX, double halfY, double halfZ) {
        return new OrientedBox(name, centreX, centreY, centreZ, identity(), halfX, halfY, halfZ);
    }

    /**
     * {@return a box turned about the vertical by {@code yawRad} (radians CCW from +X), from its centre and half extents}
     *
     * @param name    a short label for contact logs, non-blank
     * @param centreX centre, inches
     * @param centreY centre, inches
     * @param centreZ centre, inches
     * @param yawRad  turn about the vertical, radians CCW from the frame's +X
     * @param halfX   half extent along the box's own x, inches, &gt; 0
     * @param halfY   half extent along the box's own y, inches, &gt; 0
     * @param halfZ   half extent along the box's own z, inches, &gt; 0
     */
    public static OrientedBox yawed(String name, double centreX, double centreY, double centreZ, double yawRad,
            double halfX, double halfY, double halfZ) {
        requireFinite(yawRad, "yawRad");
        double cos = StrictMath.cos(yawRad);
        double sin = StrictMath.sin(yawRad);
        return new OrientedBox(name, centreX, centreY, centreZ,
                new double[][] {{cos, sin, 0.0}, {-sin, cos, 0.0}, {0.0, 0.0, 1.0}}, halfX, halfY, halfZ);
    }

    /**
     * {@return the field obstacle as an oriented box with the same name, field frame inches}
     *
     * @param box the axis-aligned field obstacle, field frame, inches
     */
    public static OrientedBox of(ObstacleBox box) {
        return axisAligned(box.getName(),
                (box.getMinXInches() + box.getMaxXInches()) / 2.0,
                (box.getMinYInches() + box.getMaxYInches()) / 2.0,
                (box.getMinZInches() + box.getMaxZInches()) / 2.0,
                (box.getMaxXInches() - box.getMinXInches()) / 2.0,
                (box.getMaxYInches() - box.getMinYInches()) / 2.0,
                (box.getMaxZInches() - box.getMinZInches()) / 2.0);
    }

    /**
     * This box carried by a rigid motion: {@code p' = rotation * p + translation}.
     *
     * @param rotation    a 3 x 3 rotation matrix, rows, unitless
     * @param translation where the old origin goes, inches, three components
     * @return the moved box, same name and size
     */
    public OrientedBox transformed(double[][] rotation, double[] translation) {
        double[] centre = multiply(rotation, new double[] {centreX, centreY, centreZ});
        double[][] moved = new double[3][];
        for (int i = 0; i < 3; i++) {
            moved[i] = multiply(rotation, axes[i]);
        }
        return new OrientedBox(name, centre[0] + translation[0], centre[1] + translation[1],
                centre[2] + translation[2], moved, half[0], half[1], half[2]);
    }

    /**
     * This box, built in a robot's frame, placed in the field: turned by the robot's heading about the vertical
     * and moved to the robot's centre (the robot frame's origin is on the floor, so heights are unchanged).
     *
     * @param robotXInches    robot centre, field X, inches
     * @param robotYInches    robot centre, field Y, inches
     * @param robotHeadingRad robot heading, radians CCW from field +X
     * @return the box in the field frame
     */
    public OrientedBox placedOnRobot(double robotXInches, double robotYInches, double robotHeadingRad) {
        double cos = StrictMath.cos(robotHeadingRad);
        double sin = StrictMath.sin(robotHeadingRad);
        return transformed(new double[][] {{cos, -sin, 0.0}, {sin, cos, 0.0}, {0.0, 0.0, 1.0}},
                new double[] {robotXInches, robotYInches, 0.0});
    }

    /** {@return the label given at construction} */
    public String getName() {
        return name;
    }

    /** {@return the centre's x, inches} */
    public double getCentreX() {
        return centreX;
    }

    /** {@return the centre's y, inches} */
    public double getCentreY() {
        return centreY;
    }

    /** {@return the centre's z, inches} */
    public double getCentreZ() {
        return centreZ;
    }

    /**
     * {@return a copy of the box's own axis {@code i} (0, 1 or 2) in its frame, a unit vector}
     *
     * @param i which axis: 0, 1 or 2
     */
    public double[] axis(int i) {
        return axes[i].clone();
    }

    /**
     * {@return the half extent along axis {@code i}, inches}
     *
     * @param i which axis: 0, 1 or 2
     */
    public double halfExtent(int i) {
        return half[i];
    }

    /** {@return the lowest point of the box, inches up} */
    public double getMinZ() {
        return centreZ - shadowRadius(0.0, 0.0, 1.0);
    }

    /** {@return the highest point of the box, inches up} */
    public double getMaxZ() {
        return centreZ + shadowRadius(0.0, 0.0, 1.0);
    }

    /**
     * Half the length of this box's shadow on a line through its centre along {@code (nx, ny, nz)}.
     *
     * @param nx the line's direction, x component, in the box's frame; need not be unit length
     * @param ny the line's direction, y component
     * @param nz the line's direction, z component
     * @return {@code sum_i half_i * |axis_i . n|}, inches times the length of n
     */
    public double shadowRadius(double nx, double ny, double nz) {
        return half[0] * Math.abs(axes[0][0] * nx + axes[0][1] * ny + axes[0][2] * nz)
                + half[1] * Math.abs(axes[1][0] * nx + axes[1][1] * ny + axes[1][2] * nz)
                + half[2] * Math.abs(axes[2][0] * nx + axes[2][1] * ny + axes[2][2] * nz);
    }

    /** {@return the eight corners, inches, as {x, y, z} rows} */
    public double[][] corners() {
        double[][] corners = new double[8][];
        int k = 0;
        for (int sx = -1; sx <= 1; sx += 2) {
            for (int sy = -1; sy <= 1; sy += 2) {
                for (int sz = -1; sz <= 1; sz += 2) {
                    corners[k++] = new double[] {
                            centreX + sx * half[0] * axes[0][0] + sy * half[1] * axes[1][0] + sz * half[2] * axes[2][0],
                            centreY + sx * half[0] * axes[0][1] + sy * half[1] * axes[1][1] + sz * half[2] * axes[2][1],
                            centreZ + sx * half[0] * axes[0][2] + sy * half[1] * axes[1][2] + sz * half[2] * axes[2][2]};
                }
            }
        }
        return corners;
    }

    /**
     * The point of this box nearest {@code (x, y, z)}: the point itself when it is inside.
     *
     * @param x the point, inches, in this box's frame
     * @param y the point, inches, in this box's frame
     * @param z the point, inches, in this box's frame
     * @return {x, y, z}, inches, in this box's frame
     */
    public double[] closestPoint(double x, double y, double z) {
        double[] local = toLocal(x, y, z);
        double rx = centreX;
        double ry = centreY;
        double rz = centreZ;
        for (int i = 0; i < 3; i++) {
            double clamped = Math.max(-half[i], Math.min(half[i], local[i]));
            rx += clamped * axes[i][0];
            ry += clamped * axes[i][1];
            rz += clamped * axes[i][2];
        }
        return new double[] {rx, ry, rz};
    }

    /**
     * {@return the point {@code (x, y, z)} in this box's own coordinates: along each of its axes from its centre, inches}
     *
     * @param x the point, inches, in this box's frame
     * @param y the point, inches, in this box's frame
     * @param z the point, inches, in this box's frame
     */
    public double[] toLocal(double x, double y, double z) {
        double dx = x - centreX;
        double dy = y - centreY;
        double dz = z - centreZ;
        return new double[] {
                dx * axes[0][0] + dy * axes[0][1] + dz * axes[0][2],
                dx * axes[1][0] + dy * axes[1][1] + dz * axes[1][2],
                dx * axes[2][0] + dy * axes[2][1] + dz * axes[2][2]};
    }

    /**
     * {@return a direction given in this box's own axes, in the box's frame}
     *
     * @param a component along the box's own axis 0, any unit (the result keeps it)
     * @param b component along the box's own axis 1, same unit
     * @param c component along the box's own axis 2, same unit
     */
    public double[] fromLocalDirection(double a, double b, double c) {
        return new double[] {
                a * axes[0][0] + b * axes[1][0] + c * axes[2][0],
                a * axes[0][1] + b * axes[1][1] + c * axes[2][1],
                a * axes[0][2] + b * axes[1][2] + c * axes[2][2]};
    }

    /**
     * Whether two boxes overlap: the separating-axis test. Two boxes in space are apart exactly when their
     * shadows are apart on one of fifteen lines: each box's three face normals, and the nine cross products of
     * one box's edge with the other's. Touching exactly counts as apart.
     *
     * @param a one box, in the same frame as {@code b}
     * @param b the other box
     * @return true when the boxes share volume
     */
    public static boolean overlap(OrientedBox a, OrientedBox b) {
        double gapX = b.centreX - a.centreX;
        double gapY = b.centreY - a.centreY;
        double gapZ = b.centreZ - a.centreZ;
        for (int i = 0; i < 3; i++) {
            if (separates(a, b, a.axes[i], gapX, gapY, gapZ) || separates(a, b, b.axes[i], gapX, gapY, gapZ)) {
                return false;
            }
        }
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                double[] cross = cross(a.axes[i], b.axes[j]);
                // Parallel edges give no new line; the face normals above already cover that case.
                if (dot(cross, cross) < 1e-12) {
                    continue;
                }
                if (separates(a, b, cross, gapX, gapY, gapZ)) {
                    return false;
                }
            }
        }
        return true;
    }

    /**
     * The shortest <b>horizontal</b> move of {@code this} box that takes it out of {@code fixed}, or {@code null}
     * when the two do not overlap.
     *
     * <p>A robot on the tiles can only be pushed sideways, so the move is chosen among horizontal directions: field
     * X and Y, and the horizontal part of each box's own axes (for boxes that are only turned about the vertical,
     * which is every robot part and every field box, those are exactly the axes the plane separating-axis test
     * needs). Along each, the overlap of the two shadows is how far this box must move; the shortest wins, pointing
     * away from {@code fixed}'s centre. Moving that far separates the shadows on that line, so the boxes are apart.
     *
     * <p>Limit, on purpose: a box that only grazes the top of a low one (a tilted arm resting on a bar) is still
     * pushed sideways, never lifted, because nothing a robot carries can climb.
     *
     * @param fixed the box that does not move
     * @return the push, or null when they are apart
     */
    public FootprintContact.Push horizontalPushOutOf(OrientedBox fixed) {
        if (!overlap(this, fixed)) {
            return null;
        }
        double gapX = centreX - fixed.centreX;
        double gapY = centreY - fixed.centreY;
        double bestDepth = Double.POSITIVE_INFINITY;
        double bestX = 0.0;
        double bestY = 0.0;
        double[][] candidates = new double[8][];
        candidates[0] = new double[] {1.0, 0.0};
        candidates[1] = new double[] {0.0, 1.0};
        for (int i = 0; i < 3; i++) {
            candidates[2 + i] = horizontal(axes[i]);
            candidates[5 + i] = horizontal(fixed.axes[i]);
        }
        for (double[] axis : candidates) {
            if (axis == null) {
                continue;
            }
            double reach = shadowRadius(axis[0], axis[1], 0.0) + fixed.shadowRadius(axis[0], axis[1], 0.0);
            double along = gapX * axis[0] + gapY * axis[1];
            double depth = reach - Math.abs(along);
            if (depth <= 0.0) {
                return null;                       // apart along this line after all
            }
            if (depth < bestDepth) {
                bestDepth = depth;
                double away = along >= 0.0 ? 1.0 : -1.0;
                bestX = axis[0] * away;
                bestY = axis[1] * away;
            }
        }
        return new FootprintContact.Push(bestX * bestDepth, bestY * bestDepth, bestDepth);
    }

    @Override
    public String toString() {
        return "OrientedBox(" + name + ")";
    }

    // ---- helpers ----------------------------------------------------------------------------------------

    private static boolean separates(OrientedBox a, OrientedBox b, double[] n, double gapX, double gapY,
            double gapZ) {
        double along = Math.abs(gapX * n[0] + gapY * n[1] + gapZ * n[2]);
        return along >= a.shadowRadius(n[0], n[1], n[2]) + b.shadowRadius(n[0], n[1], n[2]);
    }

    // The horizontal part of a direction as a unit vector, or null when the direction is (nearly) vertical.
    private static double[] horizontal(double[] axis) {
        double length = StrictMath.sqrt(axis[0] * axis[0] + axis[1] * axis[1]);
        if (length < 1e-9) {
            return null;
        }
        return new double[] {axis[0] / length, axis[1] / length};
    }

    private static double[][] identity() {
        return new double[][] {{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}};
    }

    private static double dot(double[] a, double[] b) {
        return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
    }

    private static double[] cross(double[] a, double[] b) {
        return new double[] {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
    }

    private static double[] multiply(double[][] m, double[] v) {
        return new double[] {
                m[0][0] * v[0] + m[0][1] * v[1] + m[0][2] * v[2],
                m[1][0] * v[0] + m[1][1] * v[1] + m[1][2] * v[2],
                m[2][0] * v[0] + m[2][1] * v[1] + m[2][2] * v[2]};
    }

    private static void requireFinite(double value, String what) {
        if (!Double.isFinite(value)) {
            throw new IllegalArgumentException(what + " must be finite, got " + value);
        }
    }

    private static void requirePositive(double value, String what) {
        if (!Double.isFinite(value) || value <= 0.0) {
            throw new IllegalArgumentException(what + " must be a positive finite number, got " + value);
        }
    }
}
