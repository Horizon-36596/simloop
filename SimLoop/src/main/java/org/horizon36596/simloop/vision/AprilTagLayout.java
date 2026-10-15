package org.horizon36596.simloop.vision;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Map;
import java.util.TreeMap;

/**
 * Where every AprilTag on a field is: tag id to its centre, the way it faces, and its size. A
 * {@link SimTagCamera} (and so a {@code FakeLimelight3A}) sees exactly the tags in the layout it is given
 * and nothing else.
 *
 * <p><b>The team supplies the layout.</b> SimLoop ships no field: not this season's, not any season's. A
 * layout is built by the season code or test that uses it, from numbers that team is entitled to use (a
 * game manual's published tag table, their own field measurements). Do not generate a layout from the
 * FIRST field CAD: its licence forbids redistribution, and a layout committed to a public repository would
 * redistribute it. That is why there is no {@code AprilTagLayout.forSeason(...)} here and never will be.
 *
 * <p><b>Frame.</b> The field frame of the rest of SimLoop: FTC-Cartesian, origin at the field centre on the
 * floor, {@code +Z} up, inches, headings counter-clockwise from {@code +X}. Whatever frame the layout is
 * written in is the frame a fake Limelight's {@code botpose} comes back in, exactly as a real Limelight's
 * {@code botpose} is in the frame of the field map uploaded to it.
 *
 * <p><b>Tags are vertical.</b> Each tag hangs plumb, its face pointing horizontally in the direction
 * {@code facingRad}. A tag tilted back or lying flat is not modelled.
 *
 * <p>Immutable. Build one with {@link #builder()}; {@link #empty()} is a field with no tags.
 */
public final class AprilTagLayout {

    private static final AprilTagLayout EMPTY = new AprilTagLayout(new TreeMap<Integer, Tag>());

    private final List<Tag> tags;
    private final Map<Integer, Tag> tagsById;

    private AprilTagLayout(TreeMap<Integer, Tag> byId) {
        this.tagsById = Collections.unmodifiableMap(new TreeMap<Integer, Tag>(byId));
        this.tags = Collections.unmodifiableList(new ArrayList<Tag>(byId.values()));
    }

    /** {@return a layout with no tags: a camera given it never sees anything} */
    public static AprilTagLayout empty() {
        return EMPTY;
    }

    /** {@return a builder for a new layout, holding no tags yet} */
    public static Builder builder() {
        return new Builder();
    }

    /** {@return every tag, in ascending id order, unmodifiable} */
    public List<Tag> getTags() {
        return tags;
    }

    /**
     * The tag with an id.
     *
     * @param id the tag id
     * @return the tag, or null when the layout has no tag with that id
     */
    public Tag getTag(int id) {
        return tagsById.get(id);
    }

    /** {@return how many tags the layout holds} */
    public int size() {
        return tags.size();
    }

    @Override
    public String toString() {
        return "AprilTagLayout(" + tags.size() + " tags)";
    }

    /**
     * One AprilTag: its id, the centre of its square, the direction its printed face looks, and the edge
     * length of the square whose corners a detector reports. Immutable.
     */
    public static final class Tag {

        private final int id;
        private final double xInches;
        private final double yInches;
        private final double zInches;
        private final double facingRad;
        private final double sizeInches;
        private final RigidTransform3 fieldToTag;

        private Tag(int id, double xInches, double yInches, double zInches, double facingRad,
                double sizeInches) {
            this.id = id;
            this.xInches = xInches;
            this.yInches = yInches;
            this.zInches = zInches;
            this.facingRad = facingRad;
            this.sizeInches = sizeInches;
            double cos = StrictMath.cos(facingRad);
            double sin = StrictMath.sin(facingRad);
            // Target space, as Limelight describes it: +X to the right and +Y down for someone looking at
            // the tag, +Z along the line of sight, into the tag. Right-handed. A viewer in front looks along
            // -normal, and their right is that direction turned 90 degrees clockwise seen from above.
            this.fieldToTag = RigidTransform3.fromAxes(
                    new double[] {-sin, cos, 0.0},
                    new double[] {0.0, 0.0, -1.0},
                    new double[] {-cos, -sin, 0.0},
                    xInches, yInches, zInches);
        }

        /** {@return the tag id, as printed on the tag and reported by a detector} */
        public int getId() {
            return id;
        }

        /** {@return the tag centre's field X, inches} */
        public double getXInches() {
            return xInches;
        }

        /** {@return the tag centre's field Y, inches} */
        public double getYInches() {
            return yInches;
        }

        /** {@return the tag centre's height above the floor, inches} */
        public double getZInches() {
            return zInches;
        }

        /** {@return the direction the printed face points, radians CCW from field {@code +X}} */
        public double getFacingRad() {
            return facingRad;
        }

        /** {@return the edge length of the tag square whose corners are reported, inches} */
        public double getSizeInches() {
            return sizeInches;
        }

        /**
         * The tag's own frame in the field frame: {@code T_field_tag}. The tag frame is Limelight's target
         * space: origin at the tag centre, {@code +X} to the right and {@code +Y} down as seen by someone
         * standing in front of the tag, {@code +Z} pointing away from that viewer, into the tag.
         *
         * @return the transform, inches
         */
        public RigidTransform3 getFieldToTag() {
            return fieldToTag;
        }

        @Override
        public String toString() {
            return "Tag(" + id + " at (" + xInches + ", " + yInches + ", " + zInches + ") in, facing "
                    + StrictMath.toDegrees(facingRad) + " deg, " + sizeInches + " in)";
        }
    }

    /** Collects tags for an {@link AprilTagLayout}. Not thread-safe; build once and share the layout. */
    public static final class Builder {

        private final TreeMap<Integer, Tag> tags = new TreeMap<Integer, Tag>();

        private Builder() {
        }

        /**
         * Adds a vertical tag.
         *
         * @param id         the tag id; each id may appear once
         * @param xInches    tag centre, field X, inches
         * @param yInches    tag centre, field Y, inches
         * @param zInches    tag centre height above the floor, inches
         * @param facingRad  the direction the printed face points, radians CCW from field {@code +X}; a tag
         *                   on the wall at {@code +X} that faces the field centre has {@code facingRad = PI}
         * @param sizeInches the edge length of the square a detector reports corners for, inches (&gt; 0)
         * @return this builder
         * @throws IllegalArgumentException on a repeated id, a non-finite number, or a non-positive size
         */
        public Builder addTag(int id, double xInches, double yInches, double zInches, double facingRad,
                double sizeInches) {
            requireFinite(xInches, "xInches");
            requireFinite(yInches, "yInches");
            requireFinite(zInches, "zInches");
            requireFinite(facingRad, "facingRad");
            requireFinite(sizeInches, "sizeInches");
            if (sizeInches <= 0.0) {
                throw new IllegalArgumentException("tag " + id + " needs a positive size, got " + sizeInches);
            }
            if (tags.containsKey(id)) {
                throw new IllegalArgumentException("tag id " + id + " is already in the layout");
            }
            tags.put(id, new Tag(id, xInches, yInches, zInches, facingRad, sizeInches));
            return this;
        }

        /** {@return the layout holding every tag added so far} */
        public AprilTagLayout build() {
            return tags.isEmpty() ? EMPTY : new AprilTagLayout(tags);
        }
    }

    private static void requireFinite(double value, String name) {
        if (!Double.isFinite(value)) {
            throw new IllegalArgumentException(name + " must be finite, got " + value);
        }
    }
}
