package org.horizon36596.simloop.field;

/**
 * A solid convex polyhedron a ball can rest on, roll along and bounce off: a convex hull given as its corner
 * points and its triangles. A {@link KinematicBody} is made of these, in the body's own frame.
 *
 * <p><b>Frame and units.</b> Whatever frame the owner uses - a {@link KinematicBody}'s local frame, or the field
 * frame for a body that never moves - in inches.
 *
 * <p><b>Contact is exact.</b> {@link #contact} finds the point of the solid nearest a sphere's centre: on a face,
 * an edge or a corner. A ball near an edge therefore touches the rounded edge of the hull grown by its radius,
 * not a square corner. The one approximation is for a centre already <i>inside</i> the solid, which the physics
 * avoids by never stepping a ball further than its own radius in one sub-step; then the ball is pushed out
 * through the face it is least deep behind.
 *
 * <p>Season-agnostic (domain R7): a shape knows nothing about what it is part of. Immutable.
 */
public final class ConvexShape {

    /**
     * How far a corner may stand in front of a face and the solid still count as convex, inches: a hundred
     * thousandth of an inch. Corners exported from CAD to six decimals can tip a near-flat face of a thin part
     * by about a millionth; this is ten times that, and far below anything a ball could feel.
     */
    private static final double CONVEXITY_TOLERANCE_IN = 1e-5;

    private final String name;
    private final double[][] vertices;
    private final int[][] triangles;
    /** Per triangle: outward unit normal x, y, z and the plane offset d (normal . point = d on the face). */
    private final double[][] planes;
    private final double centreX;
    private final double centreY;
    private final double centreZ;
    private final double boundingRadius;

    /**
     * Creates a hull and checks that every face points out and the solid is convex.
     *
     * @param name      a label for logs and test messages, e.g. {@code "cell_north_floor"}
     * @param vertices  the hull's corner points, {x, y, z} inches; at least four
     * @param triangles the hull's faces as indices into {@code vertices}, wound counter-clockwise seen from
     *                  outside so every normal points out; at least four
     * @throws IllegalArgumentException if a number is not finite, an index is out of range, a triangle is
     *                                  degenerate, a face points inward or the solid is not convex
     */
    public ConvexShape(String name, double[][] vertices, int[][] triangles) {
        if (name == null || name.trim().isEmpty()) {
            throw new IllegalArgumentException("a convex shape needs a name");
        }
        if (vertices == null || vertices.length < 4 || triangles == null || triangles.length < 4) {
            throw new IllegalArgumentException(name + ": a solid needs at least four corners and four faces");
        }
        this.name = name;
        this.vertices = new double[vertices.length][];
        double sumX = 0.0;
        double sumY = 0.0;
        double sumZ = 0.0;
        for (int i = 0; i < vertices.length; i++) {
            double[] v = vertices[i];
            if (v == null || v.length != 3 || !Double.isFinite(v[0]) || !Double.isFinite(v[1]) || !Double.isFinite(v[2])) {
                throw new IllegalArgumentException(name + ": corner " + i + " must be three finite numbers");
            }
            this.vertices[i] = v.clone();
            sumX += v[0];
            sumY += v[1];
            sumZ += v[2];
        }
        this.centreX = sumX / vertices.length;
        this.centreY = sumY / vertices.length;
        this.centreZ = sumZ / vertices.length;
        double radius = 0.0;
        for (double[] v : this.vertices) {
            double dx = v[0] - centreX;
            double dy = v[1] - centreY;
            double dz = v[2] - centreZ;
            radius = Math.max(radius, StrictMath.sqrt(dx * dx + dy * dy + dz * dz));
        }
        this.boundingRadius = radius;

        this.triangles = new int[triangles.length][];
        this.planes = new double[triangles.length][];
        for (int t = 0; t < triangles.length; t++) {
            int[] tri = triangles[t];
            if (tri == null || tri.length != 3) {
                throw new IllegalArgumentException(name + ": face " + t + " must be three corner indices");
            }
            for (int index : tri) {
                if (index < 0 || index >= vertices.length) {
                    throw new IllegalArgumentException(name + ": face " + t + " names corner " + index
                            + ", but there are " + vertices.length);
                }
            }
            this.triangles[t] = tri.clone();
            double[] a = this.vertices[tri[0]];
            double[] b = this.vertices[tri[1]];
            double[] c = this.vertices[tri[2]];
            double ux = b[0] - a[0];
            double uy = b[1] - a[1];
            double uz = b[2] - a[2];
            double vx = c[0] - a[0];
            double vy = c[1] - a[1];
            double vz = c[2] - a[2];
            double nx = uy * vz - uz * vy;
            double ny = uz * vx - ux * vz;
            double nz = ux * vy - uy * vx;
            double length = StrictMath.sqrt(nx * nx + ny * ny + nz * nz);
            if (!(length > 0.0)) {
                throw new IllegalArgumentException(name + ": face " + t + " has no area");
            }
            nx /= length;
            ny /= length;
            nz /= length;
            double d = nx * a[0] + ny * a[1] + nz * a[2];
            if (nx * centreX + ny * centreY + nz * centreZ - d >= 0.0) {
                throw new IllegalArgumentException(name + ": face " + t + " points inward");
            }
            for (double[] v : this.vertices) {
                if (nx * v[0] + ny * v[1] + nz * v[2] - d > CONVEXITY_TOLERANCE_IN) {
                    throw new IllegalArgumentException(name + ": not convex - a corner stands in front of face " + t);
                }
            }
            this.planes[t] = new double[] {nx, ny, nz, d};
        }
    }

    /** {@return the label given at construction} */
    public String getName() {
        return name;
    }

    /** {@return how many corners the hull has} */
    public int getVertexCount() {
        return vertices.length;
    }

    /**
     * {@return corner {@code index}, {x, y, z} inches, a copy}
     *
     * @param index which corner, 0 to {@link #getVertexCount()} minus 1
     */
    public double[] getVertex(int index) {
        return vertices[index].clone();
    }

    /** {@return the radius, inches, of a sphere about the corners' mean that holds the whole solid} */
    public double getBoundingRadiusInches() {
        return boundingRadius;
    }

    /** {@return the centre of that bounding sphere, {x, y, z} inches} */
    public double[] getBoundingCentre() {
        return new double[] {centreX, centreY, centreZ};
    }

    /**
     * Whether a sphere touches this solid, and if so how to push it out.
     *
     * @param px     the sphere's centre, inches, in this shape's frame
     * @param py     ditto
     * @param pz     ditto
     * @param radius the sphere's radius, inches
     * @param out    on contact, filled with {normal x, normal y, normal z, depth}: the unit normal points from
     *               the solid toward the sphere's centre, and depth is how far the sphere must move along it to
     *               just touch, inches
     * @return true when the sphere overlaps the solid
     */
    public boolean contact(double px, double py, double pz, double radius, double[] out) {
        double dx = px - centreX;
        double dy = py - centreY;
        double dz = pz - centreZ;
        double reach = boundingRadius + radius;
        if (dx * dx + dy * dy + dz * dz >= reach * reach) {
            return false;
        }
        // The largest signed distance to any face plane. Beyond the radius from one plane, no contact.
        double maxSeparation = Double.NEGATIVE_INFINITY;
        int deepestFace = -1;
        for (int t = 0; t < planes.length; t++) {
            double[] p = planes[t];
            double separation = p[0] * px + p[1] * py + p[2] * pz - p[3];
            if (separation > maxSeparation) {
                maxSeparation = separation;
                deepestFace = t;
            }
        }
        if (maxSeparation >= radius) {
            return false;
        }
        if (maxSeparation <= 0.0) {
            // The centre is inside the solid: out through the face it is least deep behind.
            double[] p = planes[deepestFace];
            out[0] = p[0];
            out[1] = p[1];
            out[2] = p[2];
            out[3] = radius - maxSeparation;
            return true;
        }
        // Outside: the nearest point of the surface is on a face the centre is in front of.
        double bestDistanceSquared = Double.POSITIVE_INFINITY;
        double bestX = 0.0;
        double bestY = 0.0;
        double bestZ = 0.0;
        double[] closest = new double[3];
        for (int t = 0; t < triangles.length; t++) {
            double[] p = planes[t];
            if (p[0] * px + p[1] * py + p[2] * pz - p[3] < 0.0) {
                continue;
            }
            int[] tri = triangles[t];
            closestPointOnTriangle(px, py, pz, vertices[tri[0]], vertices[tri[1]], vertices[tri[2]], closest);
            double ex = px - closest[0];
            double ey = py - closest[1];
            double ez = pz - closest[2];
            double distanceSquared = ex * ex + ey * ey + ez * ez;
            if (distanceSquared < bestDistanceSquared) {
                bestDistanceSquared = distanceSquared;
                bestX = closest[0];
                bestY = closest[1];
                bestZ = closest[2];
            }
        }
        double distance = StrictMath.sqrt(bestDistanceSquared);
        if (distance >= radius) {
            return false;
        }
        if (distance < 1e-9) {
            double[] p = planes[deepestFace];
            out[0] = p[0];
            out[1] = p[1];
            out[2] = p[2];
        } else {
            out[0] = (px - bestX) / distance;
            out[1] = (py - bestY) / distance;
            out[2] = (pz - bestZ) / distance;
        }
        out[3] = radius - distance;
        return true;
    }

    /**
     * The point of triangle abc nearest p (Ericson, Real-Time Collision Detection, 5.1.5): which of the seven
     * Voronoi regions - three corners, three edges, the face - p projects into, and the nearest point there.
     */
    static void closestPointOnTriangle(double px, double py, double pz, double[] a, double[] b, double[] c,
            double[] out) {
        double abx = b[0] - a[0];
        double aby = b[1] - a[1];
        double abz = b[2] - a[2];
        double acx = c[0] - a[0];
        double acy = c[1] - a[1];
        double acz = c[2] - a[2];
        double apx = px - a[0];
        double apy = py - a[1];
        double apz = pz - a[2];
        double d1 = abx * apx + aby * apy + abz * apz;
        double d2 = acx * apx + acy * apy + acz * apz;
        if (d1 <= 0.0 && d2 <= 0.0) {
            set(out, a[0], a[1], a[2]);
            return;
        }
        double bpx = px - b[0];
        double bpy = py - b[1];
        double bpz = pz - b[2];
        double d3 = abx * bpx + aby * bpy + abz * bpz;
        double d4 = acx * bpx + acy * bpy + acz * bpz;
        if (d3 >= 0.0 && d4 <= d3) {
            set(out, b[0], b[1], b[2]);
            return;
        }
        double vc = d1 * d4 - d3 * d2;
        if (vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0) {
            double v = d1 / (d1 - d3);
            set(out, a[0] + v * abx, a[1] + v * aby, a[2] + v * abz);
            return;
        }
        double cpx = px - c[0];
        double cpy = py - c[1];
        double cpz = pz - c[2];
        double d5 = abx * cpx + aby * cpy + abz * cpz;
        double d6 = acx * cpx + acy * cpy + acz * cpz;
        if (d6 >= 0.0 && d5 <= d6) {
            set(out, c[0], c[1], c[2]);
            return;
        }
        double vb = d5 * d2 - d1 * d6;
        if (vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0) {
            double w = d2 / (d2 - d6);
            set(out, a[0] + w * acx, a[1] + w * acy, a[2] + w * acz);
            return;
        }
        double va = d3 * d6 - d5 * d4;
        if (va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0) {
            double w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
            set(out, b[0] + w * (c[0] - b[0]), b[1] + w * (c[1] - b[1]), b[2] + w * (c[2] - b[2]));
            return;
        }
        double denominator = 1.0 / (va + vb + vc);
        double v = vb * denominator;
        double w = vc * denominator;
        set(out, a[0] + abx * v + acx * w, a[1] + aby * v + acy * w, a[2] + abz * v + acz * w);
    }

    private static void set(double[] out, double x, double y, double z) {
        out[0] = x;
        out[1] = y;
        out[2] = z;
    }

    /**
     * An axis-aligned box as a convex shape - for tests and for any season whose structure is boxes.
     *
     * @param name a label
     * @param minX the box's extent, inches, in the owner's frame; each min below its max
     * @param maxX ditto
     * @param minY ditto
     * @param maxY ditto
     * @param minZ ditto
     * @param maxZ ditto
     * @return the box
     */
    public static ConvexShape box(String name, double minX, double maxX, double minY, double maxY, double minZ,
            double maxZ) {
        if (!(minX < maxX) || !(minY < maxY) || !(minZ < maxZ)) {
            throw new IllegalArgumentException(name + ": a box needs min < max on every axis");
        }
        double[][] corners = {
                {minX, minY, minZ}, {maxX, minY, minZ}, {maxX, maxY, minZ}, {minX, maxY, minZ},
                {minX, minY, maxZ}, {maxX, minY, maxZ}, {maxX, maxY, maxZ}, {minX, maxY, maxZ},
        };
        int[][] faces = {
                {0, 2, 1}, {0, 3, 2},       // bottom, -Z
                {4, 5, 6}, {4, 6, 7},       // top, +Z
                {0, 1, 5}, {0, 5, 4},       // -Y
                {3, 6, 2}, {3, 7, 6},       // +Y
                {0, 4, 7}, {0, 7, 3},       // -X
                {1, 2, 6}, {1, 6, 5},       // +X
        };
        return new ConvexShape(name, corners, faces);
    }
}
