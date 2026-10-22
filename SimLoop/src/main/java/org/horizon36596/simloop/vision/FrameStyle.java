package org.horizon36596.simloop.vision;

import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/**
 * The flat colours and the one light a {@link SimFrameRenderer} paints a scene with. Every colour is a
 * packed {@code 0xRRGGBB} integer, 8 bits per channel, sRGB as a webcam delivers it.
 *
 * <p><b>These colours are made up.</b> The defaults are greys chosen to tell surfaces apart, and the
 * colours you give pieces are whatever you choose. None of them is a measurement of a real material under
 * real light. A colour threshold tuned on frames painted with them says nothing about a real camera, so
 * never tune one on a rendered frame: tune on real photos, and use rendered frames only to check that the
 * pipeline's code paths and geometry work (see {@link SimFrameRenderer}).
 *
 * <p>Shading is Lambert with one directional light plus ambient: a surface facing the light gets
 * {@code ambient + diffuse} times its colour, one facing away gets {@code ambient} times it. No shadows,
 * no specular highlights, no reflections.
 *
 * <p>Immutable. Build one with {@link #builder()}; {@link #defaults()} is the builder's starting point.
 */
public final class FrameStyle {

    private static final FrameStyle DEFAULTS = builder().build();

    private final int floorRgb;
    private final double tileSizeInches;
    private final int seamRgb;
    private final double seamWidthInches;
    private final int backgroundRgb;
    private final int boxRgb;
    private final int robotRgb;
    private final int pieceRgb;
    private final int tagBlackRgb;
    private final int tagWhiteRgb;
    private final int tagBackRgb;
    private final Map<String, Integer> boxColoursByPrefix;
    private final Map<String, Integer> pieceColoursByKind;
    private final double ambient;
    private final double diffuse;
    private final double[] lightDirection;

    private FrameStyle(Builder b) {
        this.floorRgb = b.floorRgb;
        this.tileSizeInches = b.tileSizeInches;
        this.seamRgb = b.seamRgb;
        this.seamWidthInches = b.seamWidthInches;
        this.backgroundRgb = b.backgroundRgb;
        this.boxRgb = b.boxRgb;
        this.robotRgb = b.robotRgb;
        this.pieceRgb = b.pieceRgb;
        this.tagBlackRgb = b.tagBlackRgb;
        this.tagWhiteRgb = b.tagWhiteRgb;
        this.tagBackRgb = b.tagBackRgb;
        this.boxColoursByPrefix = Collections.unmodifiableMap(new LinkedHashMap<String, Integer>(b.boxColours));
        this.pieceColoursByKind = Collections.unmodifiableMap(new LinkedHashMap<String, Integer>(b.pieceColours));
        this.ambient = b.ambient;
        this.diffuse = b.diffuse;
        double length = StrictMath.sqrt(b.lightX * b.lightX + b.lightY * b.lightY + b.lightZ * b.lightZ);
        this.lightDirection = new double[] {b.lightX / length, b.lightY / length, b.lightZ / length};
    }

    /** {@return the default style: grey tiles, grey structures, a dark robot, light grey pieces} */
    public static FrameStyle defaults() {
        return DEFAULTS;
    }

    /** {@return a builder starting from the defaults} */
    public static Builder builder() {
        return new Builder();
    }

    /** {@return the floor colour, {@code 0xRRGGBB}} */
    public int getFloorRgb() {
        return floorRgb;
    }

    /** {@return the tile edge, inches; 0 means a plain floor with no seams} */
    public double getTileSizeInches() {
        return tileSizeInches;
    }

    /** {@return the colour of the seams between tiles, {@code 0xRRGGBB}} */
    public int getSeamRgb() {
        return seamRgb;
    }

    /** {@return the seam width, inches} */
    public double getSeamWidthInches() {
        return seamWidthInches;
    }

    /** {@return the colour of a ray that hits nothing, {@code 0xRRGGBB}} */
    public int getBackgroundRgb() {
        return backgroundRgb;
    }

    /** {@return the colour of a field box no prefix matches, {@code 0xRRGGBB}} */
    public int getBoxRgb() {
        return boxRgb;
    }

    /** {@return the colour of a robot box no prefix matches, {@code 0xRRGGBB}} */
    public int getRobotRgb() {
        return robotRgb;
    }

    /** {@return the colour of a piece whose kind has no colour of its own, {@code 0xRRGGBB}} */
    public int getPieceRgb() {
        return pieceRgb;
    }

    /** {@return the dark cells of a tag, {@code 0xRRGGBB}} */
    public int getTagBlackRgb() {
        return tagBlackRgb;
    }

    /** {@return the light cells and border of a tag, {@code 0xRRGGBB}} */
    public int getTagWhiteRgb() {
        return tagWhiteRgb;
    }

    /** {@return the back of a tag, {@code 0xRRGGBB}} */
    public int getTagBackRgb() {
        return tagBackRgb;
    }

    /** {@return the ambient share of a surface's colour, 0 to 1} */
    public double getAmbient() {
        return ambient;
    }

    /** {@return the share a surface facing the light gets on top of ambient, 0 to 1} */
    public double getDiffuse() {
        return diffuse;
    }

    /** {@return the unit vector toward the light, field frame, {@code {x, y, z}}, a fresh copy} */
    public double[] getLightDirection() {
        return lightDirection.clone();
    }

    /**
     * The colour a box is painted: the first prefix (in the order they were added) its name starts with,
     * else the field or robot default.
     *
     * @param boxName  the {@code ObstacleBox}'s name
     * @param robotBox true for a box fixed to the robot
     * @return the colour, {@code 0xRRGGBB}
     */
    public int colourOfBox(String boxName, boolean robotBox) {
        for (Map.Entry<String, Integer> entry : boxColoursByPrefix.entrySet()) {
            if (boxName.startsWith(entry.getKey())) {
                return entry.getValue();
            }
        }
        return robotBox ? robotRgb : boxRgb;
    }

    /**
     * The colour a piece is painted.
     *
     * @param kind the piece's kind label
     * @return its colour, or {@link #getPieceRgb()} when the kind has none, {@code 0xRRGGBB}
     */
    public int colourOfPiece(String kind) {
        Integer rgb = pieceColoursByKind.get(kind);
        return rgb != null ? rgb : pieceRgb;
    }

    /** {@return the piece kinds that have their own colour, in the order added} */
    public List<String> getColouredKinds() {
        return Collections.unmodifiableList(new ArrayList<String>(pieceColoursByKind.keySet()));
    }

    /** Collects a {@link FrameStyle}. Every setting starts at the default stated on it. */
    public static final class Builder {

        private int floorRgb = 0x808080;
        private double tileSizeInches = 24.0;
        private int seamRgb = 0x5C5C5C;
        private double seamWidthInches = 0.75;
        private int backgroundRgb = 0x262626;
        private int boxRgb = 0x4A4E58;
        private int robotRgb = 0x2E2E2E;
        private int pieceRgb = 0xC8C8C8;
        private int tagBlackRgb = 0x141414;
        private int tagWhiteRgb = 0xF0F0F0;
        private int tagBackRgb = 0x6E6E6E;
        private final Map<String, Integer> boxColours = new LinkedHashMap<String, Integer>();
        private final Map<String, Integer> pieceColours = new LinkedHashMap<String, Integer>();
        private double ambient = 0.45;
        private double diffuse = 0.55;
        private double lightX = 0.25;
        private double lightY = 0.15;
        private double lightZ = 1.0;

        private Builder() {
        }

        /**
         * The floor. Default mid grey {@code 0x808080}, 24 in tiles with 0.75 in seams of {@code 0x5C5C5C}.
         *
         * @param rgb             floor colour, {@code 0xRRGGBB}
         * @param tileSizeInches  tile edge, inches; 0 for a plain floor
         * @param seamRgb         seam colour, {@code 0xRRGGBB}
         * @param seamWidthInches seam width, inches, &ge; 0 and below the tile size
         * @return this builder
         */
        public Builder floor(int rgb, double tileSizeInches, int seamRgb, double seamWidthInches) {
            if (!(tileSizeInches >= 0.0) || Double.isInfinite(tileSizeInches)) {
                throw new IllegalArgumentException("tile size must be finite and >= 0, got " + tileSizeInches);
            }
            if (!(seamWidthInches >= 0.0) || (tileSizeInches > 0.0 && seamWidthInches >= tileSizeInches)) {
                throw new IllegalArgumentException("seam width must be >= 0 and below the tile size, got "
                        + seamWidthInches);
            }
            this.floorRgb = rgb(rgb);
            this.tileSizeInches = tileSizeInches;
            this.seamRgb = rgb(seamRgb);
            this.seamWidthInches = seamWidthInches;
            return this;
        }

        /**
         * What a ray that hits nothing sees. Default {@code 0x262626}.
         *
         * @param rgb the colour, {@code 0xRRGGBB}
         * @return this builder
         */
        public Builder background(int rgb) {
            this.backgroundRgb = rgb(rgb);
            return this;
        }

        /**
         * Field boxes no prefix matches. Default {@code 0x4A4E58}.
         *
         * @param rgb the colour, {@code 0xRRGGBB}
         * @return this builder
         */
        public Builder boxes(int rgb) {
            this.boxRgb = rgb(rgb);
            return this;
        }

        /**
         * Robot boxes no prefix matches. Default {@code 0x2E2E2E}.
         *
         * @param rgb the colour, {@code 0xRRGGBB}
         * @return this builder
         */
        public Builder robot(int rgb) {
            this.robotRgb = rgb(rgb);
            return this;
        }

        /**
         * Paints every box, field or robot, whose name starts with {@code prefix}. Prefixes are tried in the
         * order added; the first match wins.
         *
         * @param prefix the start of the box names, non-empty
         * @param rgb    the colour, {@code 0xRRGGBB}
         * @return this builder
         */
        public Builder boxColour(String prefix, int rgb) {
            if (prefix == null || prefix.isEmpty()) {
                throw new IllegalArgumentException("a box colour prefix must be non-empty");
            }
            boxColours.put(prefix, rgb(rgb));
            return this;
        }

        /**
         * Pieces whose kind has no colour of its own. Default {@code 0xC8C8C8}.
         *
         * @param rgb the colour, {@code 0xRRGGBB}
         * @return this builder
         */
        public Builder pieces(int rgb) {
            this.pieceRgb = rgb(rgb);
            return this;
        }

        /**
         * Paints every piece of one kind.
         *
         * @param kind the {@code GamePiece} kind label
         * @param rgb  the colour, {@code 0xRRGGBB}
         * @return this builder
         */
        public Builder pieceColour(String kind, int rgb) {
            if (kind == null) {
                throw new IllegalArgumentException("piece kind must not be null");
            }
            pieceColours.put(kind, rgb(rgb));
            return this;
        }

        /**
         * The tag colours. Defaults {@code 0x141414}, {@code 0xF0F0F0} and, for the back, {@code 0x6E6E6E}.
         *
         * @param blackRgb dark cells, {@code 0xRRGGBB}
         * @param whiteRgb light cells and border, {@code 0xRRGGBB}
         * @param backRgb  the back of the tag, {@code 0xRRGGBB}
         * @return this builder
         */
        public Builder tags(int blackRgb, int whiteRgb, int backRgb) {
            this.tagBlackRgb = rgb(blackRgb);
            this.tagWhiteRgb = rgb(whiteRgb);
            this.tagBackRgb = rgb(backRgb);
            return this;
        }

        /**
         * The light. Default ambient 0.45, diffuse 0.55, light from above, a little toward {@code +X} and
         * {@code +Y}: direction {@code (0.25, 0.15, 1)} before normalising.
         *
         * @param ambient share of colour every surface gets, 0 to 1
         * @param diffuse share a surface facing the light gets on top, 0 to 1
         * @param towardX direction toward the light, field X, any length
         * @param towardY direction toward the light, field Y
         * @param towardZ direction toward the light, up
         * @return this builder
         */
        public Builder light(double ambient, double diffuse, double towardX, double towardY, double towardZ) {
            if (!(ambient >= 0.0 && ambient <= 1.0) || !(diffuse >= 0.0 && diffuse <= 1.0)) {
                throw new IllegalArgumentException("ambient and diffuse must be in [0, 1], got " + ambient
                        + " and " + diffuse);
            }
            double length = StrictMath.sqrt(towardX * towardX + towardY * towardY + towardZ * towardZ);
            if (!(length > 0.0) || Double.isInfinite(length)) {
                throw new IllegalArgumentException("the light direction must be a finite, non-zero vector");
            }
            this.ambient = ambient;
            this.diffuse = diffuse;
            this.lightX = towardX;
            this.lightY = towardY;
            this.lightZ = towardZ;
            return this;
        }

        /** {@return the style} */
        public FrameStyle build() {
            return new FrameStyle(this);
        }

        private static int rgb(int rgb) {
            if ((rgb & 0xFF000000) != 0) {
                throw new IllegalArgumentException("a colour is 0xRRGGBB, with nothing above bit 23; got 0x"
                        + Integer.toHexString(rgb));
            }
            return rgb;
        }
    }
}
