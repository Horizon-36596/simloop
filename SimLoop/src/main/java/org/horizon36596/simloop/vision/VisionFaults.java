package org.horizon36596.simloop.vision;

/**
 * Faults to inject into a simulated camera's output, for stress-testing the vision code that consumes it:
 * what a pipeline does when the camera is noisier than usual, late, jittery, blocked, drops or repeats
 * frames, delivers them out of order, or reports things that are not there.
 *
 * <p>This is a description only. Two classes act on it:
 * <ul>
 *   <li>{@link VisionFaultInjector} changes what is in a frame: the sightings ({@link CameraSighting},
 *       {@link TagSighting}) or the pixels of a {@link SimFrame}.</li>
 *   <li>{@link FaultedFrameStream} changes when, and in what order, frames are delivered.</li>
 * </ul>
 * {@code FakeLimelight3A.Builder#faults} uses both.
 *
 * <p><b>Determinism.</b> Every fault draws from its own {@link org.horizon36596.simloop.field.SeededRandom},
 * seeded from {@link #getSeed()} and the fault's name, so the same seed and the same sequence of frames give
 * the same faults, and turning one fault on or off does not move another fault's draws.
 *
 * <p><b>Every rate here is a test input, not a measurement.</b> None of the defaults describe a real camera;
 * the defaults are "off". Pick the levels your test needs and say so in the test.
 *
 * <p>Immutable. Build with {@link #builder(int)}; {@link #none()} turns everything off.
 */
public final class VisionFaults {

    private static final VisionFaults NONE = new Builder(0).build();

    private final int seed;
    private final double centreNoiseStdDevPx;
    private final double radiusNoiseStdDevPx;
    private final double tagAngleNoiseStdDevDegrees;
    private final double pixelNoiseStdDevCounts;
    private final double extraLatencySeconds;
    private final double latencyJitterSeconds;
    private final double dropProbability;
    private final double repeatProbability;
    private final double reorderProbability;
    private final double missProbability;
    private final double blackoutProbability;
    private final int blackoutFrames;
    private final double occlusionPatchProbability;
    private final double occlusionPatchFraction;
    private final double falseDetectionProbability;
    private final String falseDetectionKind;
    private final double falseDetectionMinRadiusPx;
    private final double falseDetectionMaxRadiusPx;
    private final int falseDetectionColourRgb;
    private final double wrongIdProbability;
    private final String[] wrongKinds;

    private VisionFaults(Builder b) {
        this.seed = b.seed;
        this.centreNoiseStdDevPx = b.centreNoiseStdDevPx;
        this.radiusNoiseStdDevPx = b.radiusNoiseStdDevPx;
        this.tagAngleNoiseStdDevDegrees = b.tagAngleNoiseStdDevDegrees;
        this.pixelNoiseStdDevCounts = b.pixelNoiseStdDevCounts;
        this.extraLatencySeconds = b.extraLatencySeconds;
        this.latencyJitterSeconds = b.latencyJitterSeconds;
        this.dropProbability = b.dropProbability;
        this.repeatProbability = b.repeatProbability;
        this.reorderProbability = b.reorderProbability;
        this.missProbability = b.missProbability;
        this.blackoutProbability = b.blackoutProbability;
        this.blackoutFrames = b.blackoutFrames;
        this.occlusionPatchProbability = b.occlusionPatchProbability;
        this.occlusionPatchFraction = b.occlusionPatchFraction;
        this.falseDetectionProbability = b.falseDetectionProbability;
        this.falseDetectionKind = b.falseDetectionKind;
        this.falseDetectionMinRadiusPx = b.falseDetectionMinRadiusPx;
        this.falseDetectionMaxRadiusPx = b.falseDetectionMaxRadiusPx;
        this.falseDetectionColourRgb = b.falseDetectionColourRgb;
        this.wrongIdProbability = b.wrongIdProbability;
        this.wrongKinds = b.wrongKinds.clone();
    }

    /** {@return no faults at all: an injector and a stream built from this pass frames through unchanged} */
    public static VisionFaults none() {
        return NONE;
    }

    /**
     * Starts a description with every fault off.
     *
     * @param seed the seed every fault's random draws derive from; same seed, same faults
     * @return the builder
     */
    public static Builder builder(int seed) {
        return new Builder(seed);
    }

    /** {@return the seed every fault's random draws derive from} */
    public int getSeed() {
        return seed;
    }

    /** {@return extra Gaussian noise on each piece sighting's centre, per axis, standard deviation, pixels} */
    public double getCentreNoiseStdDevPx() {
        return centreNoiseStdDevPx;
    }

    /** {@return extra Gaussian noise on each piece sighting's apparent radius, standard deviation, pixels} */
    public double getRadiusNoiseStdDevPx() {
        return radiusNoiseStdDevPx;
    }

    /** {@return extra Gaussian noise on each tag sighting's tx and ty, standard deviation, degrees} */
    public double getTagAngleNoiseStdDevDegrees() {
        return tagAngleNoiseStdDevDegrees;
    }

    /** {@return extra Gaussian noise on every channel of every pixel of a frame, standard deviation, 8-bit counts} */
    public double getPixelNoiseStdDevCounts() {
        return pixelNoiseStdDevCounts;
    }

    /** {@return a fixed delay added to every frame's delivery, seconds} */
    public double getExtraLatencySeconds() {
        return extraLatencySeconds;
    }

    /** {@return the most a frame's delivery is moved earlier or later at random, uniform, seconds} */
    public double getLatencyJitterSeconds() {
        return latencyJitterSeconds;
    }

    /** {@return the chance a whole frame is never delivered} */
    public double getDropProbability() {
        return dropProbability;
    }

    /** {@return the chance a delivered frame is delivered a second time, with the same capture stamp} */
    public double getRepeatProbability() {
        return repeatProbability;
    }

    /** {@return the chance a frame is held back and delivered after the next frame that is not dropped} */
    public double getReorderProbability() {
        return reorderProbability;
    }

    /** {@return the chance each sighting in a frame is missed: partial occlusion} */
    public double getMissProbability() {
        return missProbability;
    }

    /** {@return the chance a frame starts a blackout: the camera sees nothing for {@link #getBlackoutFrames()}} */
    public double getBlackoutProbability() {
        return blackoutProbability;
    }

    /** {@return how many frames in a row a blackout lasts, the starting frame included} */
    public int getBlackoutFrames() {
        return blackoutFrames;
    }

    /** {@return the chance a frame's pixels get one occluding rectangle} */
    public double getOcclusionPatchProbability() {
        return occlusionPatchProbability;
    }

    /** {@return the occluding rectangle's width and height as a share of the image's, 0 to 1} */
    public double getOcclusionPatchFraction() {
        return occlusionPatchFraction;
    }

    /** {@return the chance a frame gains one false detection: a piece, a blob or a tag that is not there} */
    public double getFalseDetectionProbability() {
        return falseDetectionProbability;
    }

    /** {@return the kind label a false piece sighting carries} */
    public String getFalseDetectionKind() {
        return falseDetectionKind;
    }

    /** {@return the smallest apparent radius of a false piece sighting or blob, pixels} */
    public double getFalseDetectionMinRadiusPx() {
        return falseDetectionMinRadiusPx;
    }

    /** {@return the largest apparent radius of a false piece sighting or blob, pixels} */
    public double getFalseDetectionMaxRadiusPx() {
        return falseDetectionMaxRadiusPx;
    }

    /** {@return the colour a false blob is painted in a frame, {@code 0xRRGGBB}} */
    public int getFalseDetectionColourRgb() {
        return falseDetectionColourRgb;
    }

    /**
     * {@return the chance each sighting is mislabelled: a tag reports another layout tag's id, a piece
     * another of {@link #getWrongKinds()}}
     */
    public double getWrongIdProbability() {
        return wrongIdProbability;
    }

    /** {@return the kind labels a mislabelled piece sighting can take; a copy} */
    public String[] getWrongKinds() {
        return wrongKinds.clone();
    }

    /** {@return true when no fault is on} */
    public boolean isNone() {
        return centreNoiseStdDevPx == 0.0 && radiusNoiseStdDevPx == 0.0 && tagAngleNoiseStdDevDegrees == 0.0
                && pixelNoiseStdDevCounts == 0.0 && extraLatencySeconds == 0.0 && latencyJitterSeconds == 0.0
                && dropProbability == 0.0 && repeatProbability == 0.0 && reorderProbability == 0.0
                && missProbability == 0.0 && blackoutProbability == 0.0 && occlusionPatchProbability == 0.0
                && falseDetectionProbability == 0.0 && wrongIdProbability == 0.0;
    }

    @Override
    public String toString() {
        if (isNone()) {
            return "VisionFaults(none)";
        }
        StringBuilder s = new StringBuilder("VisionFaults(seed ").append(seed);
        append(s, "centreNoisePx", centreNoiseStdDevPx);
        append(s, "radiusNoisePx", radiusNoiseStdDevPx);
        append(s, "tagAngleNoiseDeg", tagAngleNoiseStdDevDegrees);
        append(s, "pixelNoiseCounts", pixelNoiseStdDevCounts);
        append(s, "extraLatencyS", extraLatencySeconds);
        append(s, "jitterS", latencyJitterSeconds);
        append(s, "drop", dropProbability);
        append(s, "repeat", repeatProbability);
        append(s, "reorder", reorderProbability);
        append(s, "miss", missProbability);
        append(s, "blackout", blackoutProbability);
        append(s, "patch", occlusionPatchProbability);
        append(s, "falseDetection", falseDetectionProbability);
        append(s, "wrongId", wrongIdProbability);
        return s.append(')').toString();
    }

    private static void append(StringBuilder s, String name, double value) {
        if (value != 0.0) {
            s.append(", ").append(name).append(' ').append(value);
        }
    }

    /** Builds a {@link VisionFaults}. Every fault starts off; each setter turns one on. */
    public static final class Builder {

        private final int seed;
        private double centreNoiseStdDevPx;
        private double radiusNoiseStdDevPx;
        private double tagAngleNoiseStdDevDegrees;
        private double pixelNoiseStdDevCounts;
        private double extraLatencySeconds;
        private double latencyJitterSeconds;
        private double dropProbability;
        private double repeatProbability;
        private double reorderProbability;
        private double missProbability;
        private double blackoutProbability;
        private int blackoutFrames = 1;
        private double occlusionPatchProbability;
        private double occlusionPatchFraction;
        private double falseDetectionProbability;
        private String falseDetectionKind = "false";
        private double falseDetectionMinRadiusPx = 4.0;
        private double falseDetectionMaxRadiusPx = 30.0;
        private int falseDetectionColourRgb = 0xFFFFFF;
        private double wrongIdProbability;
        private String[] wrongKinds = new String[0];

        private Builder(int seed) {
            this.seed = seed;
        }

        /**
         * Extra Gaussian noise on each piece sighting's centre and apparent radius, on top of whatever noise
         * the caller already adds. A radius pushed to zero or below is not a blob a detector can report, so
         * that sighting is dropped instead.
         *
         * @param centreStdDevPx standard deviation per axis, pixels, finite and &ge; 0
         * @param radiusStdDevPx standard deviation, pixels, finite and &ge; 0
         * @return this builder
         */
        public Builder pixelNoise(double centreStdDevPx, double radiusStdDevPx) {
            this.centreNoiseStdDevPx = nonNegative(centreStdDevPx, "centre noise");
            this.radiusNoiseStdDevPx = nonNegative(radiusStdDevPx, "radius noise");
            return this;
        }

        /**
         * Extra Gaussian noise on each tag sighting's {@code tx} and {@code ty}. The centre pixel moves with
         * them; a tag pushed out of the image is dropped, because a detector cannot report it there.
         *
         * @param stdDevDegrees standard deviation, degrees, finite and &ge; 0
         * @return this builder
         */
        public Builder tagAngleNoise(double stdDevDegrees) {
            this.tagAngleNoiseStdDevDegrees = nonNegative(stdDevDegrees, "tag angle noise");
            return this;
        }

        /**
         * Extra Gaussian noise on every channel of every pixel of a frame, clamped to 0..255.
         *
         * @param stdDevCounts standard deviation, 8-bit counts, finite and &ge; 0
         * @return this builder
         */
        public Builder frameNoise(double stdDevCounts) {
            this.pixelNoiseStdDevCounts = nonNegative(stdDevCounts, "frame noise");
            return this;
        }

        /**
         * Delay every frame's delivery by a fixed amount, and move each one by up to {@code jitterSeconds}
         * earlier or later, uniform. Jitter never makes a frame arrive before it was captured, and on its own
         * it never reorders frames: a pipeline delivers in order, so a frame that would overtake the one
         * before it waits for it instead. Use {@link #reorderFrames} for out-of-order delivery.
         *
         * @param extraSeconds  fixed extra delay, seconds, finite and &ge; 0
         * @param jitterSeconds largest random shift, seconds, finite and &ge; 0
         * @return this builder
         */
        public Builder latency(double extraSeconds, double jitterSeconds) {
            this.extraLatencySeconds = nonNegative(extraSeconds, "extra latency");
            this.latencyJitterSeconds = nonNegative(jitterSeconds, "latency jitter");
            return this;
        }

        /**
         * Lose whole frames.
         *
         * @param probability chance each frame is never delivered, in {@code [0, 1]}
         * @return this builder
         */
        public Builder dropFrames(double probability) {
            this.dropProbability = probability(probability, "drop");
            return this;
        }

        /**
         * Deliver a frame twice: a stale frame handed over again with the same capture stamp and the same
         * contents, on the next delivery after the first.
         *
         * @param probability chance each delivered frame is repeated once, in {@code [0, 1]}
         * @return this builder
         */
        public Builder repeatFrames(double probability) {
            this.repeatProbability = probability(probability, "repeat");
            return this;
        }

        /**
         * Deliver a frame after the next frame that is not dropped, so capture stamps go backwards. A frame
         * held back when no later frame is ever delivered is not delivered either. At 1.0 every frame is held
         * behind the next, so no frame is ever delivered: use it only to model a stream that never arrives.
         *
         * @param probability chance each frame is held back, in {@code [0, 1]}
         * @return this builder
         */
        public Builder reorderFrames(double probability) {
            this.reorderProbability = probability(probability, "reorder");
            return this;
        }

        /**
         * Partial occlusion of sightings: each piece or tag in a frame is missed independently.
         *
         * @param probability chance each sighting is missed, in {@code [0, 1]}
         * @return this builder
         */
        public Builder missDetections(double probability) {
            this.missProbability = probability(probability, "miss");
            return this;
        }

        /**
         * Full occlusion: a frame may start a run of frames in which the camera sees nothing (a robot parked
         * in front of it, a hand over the lens). Sightings are all removed; a frame's pixels are painted
         * black.
         *
         * @param probability chance a frame not already blacked out starts a blackout, in {@code [0, 1]}
         * @param frames      how many frames the blackout lasts, the starting one included, &ge; 1
         * @return this builder
         */
        public Builder blackout(double probability, int frames) {
            if (frames < 1) {
                throw new IllegalArgumentException("a blackout lasts at least one frame, got " + frames);
            }
            this.blackoutProbability = probability(probability, "blackout");
            this.blackoutFrames = frames;
            return this;
        }

        /**
         * Partial occlusion of pixels: a frame may get one flat dark-grey rectangle at a random place.
         *
         * @param probability chance a frame gets a rectangle, in {@code [0, 1]}
         * @param fraction    the rectangle's width and height as a share of the image's, in {@code (0, 1]}
         * @return this builder
         */
        public Builder occlusionPatch(double probability, double fraction) {
            if (!(fraction > 0.0 && fraction <= 1.0)) {
                throw new IllegalArgumentException("patch fraction must be in (0, 1], got " + fraction);
            }
            this.occlusionPatchProbability = probability(probability, "occlusion patch");
            this.occlusionPatchFraction = fraction;
            return this;
        }

        /**
         * False detections: a frame may gain one thing that is not there. For piece sightings, a sighting of
         * kind {@code kind} with piece id {@link VisionFaultInjector#FALSE_PIECE_ID} at a random place in the
         * image; for a frame's pixels, a disc of {@code colourRgb}; for tags, a sighting of a random layout
         * tag at a random place in front of the camera, so a pose solved from it is wrong.
         *
         * @param probability chance a frame gains one, in {@code [0, 1]}
         * @param kind        the kind label a false piece sighting carries, not null
         * @param minRadiusPx smallest apparent radius of a false piece or blob, pixels, &gt; 0
         * @param maxRadiusPx largest, pixels, &ge; {@code minRadiusPx}
         * @param colourRgb   the colour of a false blob in a frame, {@code 0xRRGGBB}
         * @return this builder
         */
        public Builder falseDetections(double probability, String kind, double minRadiusPx, double maxRadiusPx,
                int colourRgb) {
            if (kind == null) {
                throw new IllegalArgumentException("false detection kind must not be null");
            }
            if (!(minRadiusPx > 0.0) || !(maxRadiusPx >= minRadiusPx) || Double.isInfinite(maxRadiusPx)) {
                throw new IllegalArgumentException("false detection radii must satisfy 0 < min <= max < infinity,"
                        + " got " + minRadiusPx + " and " + maxRadiusPx);
            }
            this.falseDetectionProbability = probability(probability, "false detection");
            this.falseDetectionKind = kind;
            this.falseDetectionMinRadiusPx = minRadiusPx;
            this.falseDetectionMaxRadiusPx = maxRadiusPx;
            this.falseDetectionColourRgb = colourRgb & 0xFFFFFF;
            return this;
        }

        /**
         * Mislabelled sightings: a tag reports the id of another tag in the layout (its geometry stays the
         * tag actually seen, as a misdecode does); a piece reports one of {@code kinds} other than its own.
         *
         * @param probability chance each sighting is mislabelled, in {@code [0, 1]}
         * @param kinds       the kind labels a piece can be mislabelled as; empty leaves pieces alone
         * @return this builder
         */
        public Builder wrongIds(double probability, String... kinds) {
            if (kinds == null) {
                throw new IllegalArgumentException("kinds must not be null; pass none to leave pieces alone");
            }
            for (String kind : kinds) {
                if (kind == null) {
                    throw new IllegalArgumentException("a kind label must not be null");
                }
            }
            this.wrongIdProbability = probability(probability, "wrong id");
            this.wrongKinds = kinds.clone();
            return this;
        }

        /** {@return the faults} */
        public VisionFaults build() {
            return new VisionFaults(this);
        }

        private static double nonNegative(double value, String what) {
            if (!(value >= 0.0) || Double.isInfinite(value)) {
                throw new IllegalArgumentException(what + " must be finite and >= 0, got " + value);
            }
            return value;
        }

        private static double probability(double value, String what) {
            if (!(value >= 0.0 && value <= 1.0)) {
                throw new IllegalArgumentException(what + " probability must be in [0, 1], got " + value);
            }
            return value;
        }
    }
}
