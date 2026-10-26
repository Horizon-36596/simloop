package org.horizon36596.simloop.sim;

import org.psilynx.psikit.core.LogTable;
import org.psilynx.psikit.core.rlog.RLOGReplay;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.Set;
import java.util.TreeMap;
import java.util.TreeSet;
import java.util.function.Consumer;

/**
 * Compares two RLOG files by <b>decoded</b> field value, not raw bytes (BACKLOG B3,
 * {@code docs/process/scorer-interface.md} §2/§7). Raw {@code .rlog} bytes are not run-to-run
 * stable even for a fully deterministic scenario, but this is narrower than PsiKit's hash-map key
 * ordering being unstable in general: empirically verified 2026-07-18 against reruns of
 * {@code DrivetrainSimTest} that every robot-authored field decodes identically and the only keys
 * that ever differ are PsiKit's own async RLOG-writer-thread metadata (real-time queue depth / a
 * startup console line), which are excluded here by name because they reflect writer timing, not
 * robot state. Any other differing key — including one present in only one run's frame — is a real
 * mismatch, not filtered.
 */
public final class RlogDecodedCompare {

    /**
     * PsiKit's own writer-thread metadata: real-time queue depth and a startup console line. These reflect
     * when the writer thread happened to be scheduled, not what the robot did, so two identical runs
     * legitimately differ on them and {@link #compare} ignores them.
     *
     * <p>Public because anything that scores a run has to refuse the same keys. A field the replay check
     * ignores must not be scoreable: an objective reading {@code RealOutputs/Logger/QueuedCycles} would
     * otherwise be scored on a number that {@code compare} cannot see, so a candidate could be reported as
     * REPRODUCED while the field its score came from did not reproduce at all. (2026-09-17 adversarial
     * review, finding replay-skips-scoreable-fields.)
     */
    public static final Set<String> EXCLUDED_KEYS = Collections.unmodifiableSet(new HashSet<>(Arrays.asList(
            "RealOutputs/Console",
            "RealOutputs/Logger/QueuedCycles")));

    private RlogDecodedCompare() {
    }

    /** How many differing keys {@link #summarise} names before it stops and counts the rest. */
    private static final int SUMMARY_KEY_LIMIT = 40;

    /**
     * A mismatch list rendered for a human: how many differed, which KEYS differed and how often, and one
     * worked example of each, capped at {@link #SUMMARY_KEY_LIMIT} keys.
     *
     * <p><b>Why this exists rather than {@code mismatches.toString()}.</b> One divergence early in a run
     * makes every later frame differ on every key it touches, so the raw list of a real failure runs to
     * tens of thousands of entries and megabytes. That lands in the JUnit XML, is truncated by whatever
     * reads it, and answers the one question a person actually has -- WHICH FIELD went wrong and WHEN --
     * nowhere in the part that survives. Measured 2026-09-22: a single subsystem diverging produced a
     * 2.5 MiB assertion message from which the offending key could not be read at all.
     *
     * @param mismatches the list {@link #compare(Path, Path)} returned
     * @return a "frame count differs" line when the runs had different lengths, then one line of totals
     *         followed by one line per differing key, EARLIEST divergence first; {@code "none"} when the
     *         list is empty
     */
    public static String summarise(List<Mismatch> mismatches) {
        if (mismatches.isEmpty()) {
            return "none";
        }
        // The frame-count mismatch is not a field diff and has no frame, so it is reported on its own line
        // rather than sorted in with the fields: its frameIndex of -1 would otherwise make it the "earliest
        // divergence" of every run whose lengths differ, and hide the frame where the fields first differed.
        Mismatch frameCountMismatch = null;
        Map<String, Integer> countByKey = new TreeMap<>();
        Map<String, Mismatch> firstByKey = new TreeMap<>();
        for (Mismatch mismatch : mismatches) {
            if (mismatch.isFrameCountMismatch()) {
                frameCountMismatch = mismatch;
                continue;
            }
            Integer seen = countByKey.get(mismatch.key);
            countByKey.put(mismatch.key, seen == null ? 1 : seen + 1);
            if (!firstByKey.containsKey(mismatch.key)) {
                firstByKey.put(mismatch.key, mismatch);
            }
        }

        // EARLIEST FIRST, because the earliest divergence is the CAUSE and every later one is
        // consequence. Sorting by key name instead reads tidily and buries the answer: a list capped at
        // SUMMARY_KEY_LIMIT names whichever keys sort first, which on a 77-key failure meant the two
        // subsystems whose names begin with A and D and not the one that went wrong at frame 30.
        List<Mismatch> firsts = new ArrayList<>(firstByKey.values());
        Collections.sort(firsts, new Comparator<Mismatch>() {
            @Override
            public int compare(Mismatch a, Mismatch b) {
                if (a.frameIndex != b.frameIndex) {
                    return a.frameIndex < b.frameIndex ? -1 : 1;
                }
                return a.key.compareTo(b.key);
            }
        });

        StringBuilder out = new StringBuilder();
        if (frameCountMismatch != null) {
            out.append("frame count differs: ").append(frameCountMismatch.valueA)
                    .append(" vs ").append(frameCountMismatch.valueB).append('\n');
        }
        if (firsts.isEmpty()) {
            out.append("no field mismatches in the frames both runs have");
            return out.toString();
        }
        out.append(mismatches.size() - (frameCountMismatch == null ? 0 : 1)).append(" field mismatches across ")
                .append(countByKey.size()).append(" keys, earliest at frame ")
                .append(firsts.get(0).frameIndex)
                .append(" (keys listed earliest-divergence first)");
        int shown = 0;
        for (Mismatch first : firsts) {
            if (shown++ == SUMMARY_KEY_LIMIT) {
                out.append("\n  ... and ").append(firsts.size() - SUMMARY_KEY_LIMIT)
                        .append(" more keys, all diverging at frame ")
                        .append(first.frameIndex).append(" or later");
                break;
            }
            out.append("\n  frame ").append(first.frameIndex)
                    .append("  ").append(first.key)
                    .append(" x").append(countByKey.get(first.key))
                    .append(": ").append(first.valueA).append(" vs ").append(first.valueB);
        }
        return out.toString();
    }

    /** One decoded-field mismatch between two runs; {@code frameIndex} is -1 for a frame-count mismatch. */
    public static final class Mismatch {
        /** Which frame differed, counting from 0; {@code -1} when the two runs had different frame counts. */
        public final int frameIndex;

        /** The RLOG key whose decoded value differed. */
        public final String key;

        /**
         * The value the first run logged under {@link #key}: PsiKit's rendering of the value followed by its
         * logged type in brackets, e.g. {@code "12.5 [Double]"}; {@code null} if the first run did not log
         * the key in this frame. For a frame-count mismatch, the first run's frame count.
         */
        public final String valueA;

        /** As {@link #valueA}, for the second run. */
        public final String valueB;

        Mismatch(int frameIndex, String key, String valueA, String valueB) {
            this.frameIndex = frameIndex;
            this.key = key;
            this.valueA = valueA;
            this.valueB = valueB;
        }

        /**
         * {@return true if this entry reports the two runs having a different total frame count, not a field diff}
         */
        public boolean isFrameCountMismatch() {
            return frameIndex == -1;
        }

        @Override
        public String toString() {
            return "frame " + frameIndex + " key " + key + ": " + valueA + " vs " + valueB;
        }
    }

    /**
     * Decodes both RLOG files via PsiKit's {@link RLOGReplay} and returns every decoded-field
     * mismatch outside PsiKit's own writer-metadata exclude list, frame by frame. An empty list means
     * the two runs satisfy the decoded-field-equality contract.
     *
     * <p>A field matches only if its value AND its logged type match. PsiKit's own
     * {@code LogValue.toString()} renders the value alone (checked by disassembling the pinned
     * {@code core-0.2.0-beta2} jar), so the string {@code "1.0"} and the double {@code 1.0}, or two values
     * logged under different custom type names such as a struct schema, would otherwise compare as equal.
     *
     * @param pathA the first RLOG to read
     * @param pathB the second RLOG to read
     * @return every mismatch found, in frame order; empty when the two runs agree
     * @throws IOException              if either RLOG exists but cannot be read
     * @throws IllegalArgumentException if either RLOG is missing or empty (see {@link #decode(Path)})
     */
    public static List<Mismatch> compare(Path pathA, Path pathB) throws IOException {
        return compare(pathA, pathB, EXCLUDED_KEYS);
    }

    /**
     * As {@link #compare(Path, Path)}, but with a caller-supplied exclude set instead of the PsiKit default.
     *
     * @param pathA        the first RLOG to read
     * @param pathB        the second RLOG to read
     * @param excludedKeys keys to ignore entirely, in place of PsiKit's writer-metadata list
     * @return every mismatch found outside {@code excludedKeys}, in frame order
     * @throws IOException              if either RLOG exists but cannot be read
     * @throws IllegalArgumentException if either RLOG is missing or empty (see {@link #decode(Path)})
     */
    public static List<Mismatch> compare(Path pathA, Path pathB, Set<String> excludedKeys) throws IOException {
        return compareFrames(decodeFrames(pathA, true), decodeFrames(pathB, true), excludedKeys);
    }

    /**
     * As {@link #compare(Path, Path, Set)}, but on frames that were already decoded.
     *
     * <p>For a caller that decoded a run earlier and must not re-read it from disk: the loop's stop report
     * compares a replay against the frames it held in memory when it accepted a candidate, because the RLOG
     * file that candidate was read from may have been overwritten by a later candidate since.
     *
     * <p><b>Both lists must be rendered the same way (both typed or both untyped).</b> {@link #compare}
     * decodes both sides with each
     * value's logged type appended ({@code "12.5 [Double]"}); {@link #decode(Path)} renders the value alone
     * ({@code "12.5"}). Comparing one of each would report every field as different. Frames from
     * {@link #decode(Path)} compare values only: a field whose type changed but whose rendering did not is
     * not caught.
     *
     * @param framesA      the first run's frames; rendered the same way as {@code framesB}
     * @param framesB      the second run's frames; rendered the same way as {@code framesA}
     * @param excludedKeys keys to ignore entirely
     * @return every mismatch found outside {@code excludedKeys}, in frame order; a frame-count difference
     *         comes first, as one entry with {@code frameIndex == -1}
     */
    public static List<Mismatch> compareFrames(List<Map<String, String>> framesA,
            List<Map<String, String>> framesB, Set<String> excludedKeys) {
        List<Mismatch> mismatches = new ArrayList<>();
        if (framesA.size() != framesB.size()) {
            mismatches.add(new Mismatch(-1, "<frame count>",
                    String.valueOf(framesA.size()), String.valueOf(framesB.size())));
        }

        int frameCount = Math.min(framesA.size(), framesB.size());
        for (int i = 0; i < frameCount; i++) {
            Map<String, String> a = framesA.get(i);
            Map<String, String> b = framesB.get(i);
            Set<String> keys = new TreeSet<>();
            keys.addAll(a.keySet());
            keys.addAll(b.keySet());
            for (String key : keys) {
                if (excludedKeys.contains(key)) {
                    continue;
                }
                String valueA = a.get(key);
                String valueB = b.get(key);
                if (!Objects.equals(valueA, valueB)) {
                    mismatches.add(new Mismatch(i, key, valueA, valueB));
                }
            }
        }
        return mismatches;
    }

    /**
     * Every field key that appears anywhere in one RLOG, sorted, minus PsiKit's own writer metadata.
     *
     * <p>This is how a test asserts what a subsystem <i>actually published</i>, rather than asserting
     * against a list of key names copied out of that subsystem — which would pass no matter what the
     * subsystem did. The logging contract's rule 2 (units live in the key name, because AdvantageScope
     * shows a key with no javadoc beside it) is only checkable against the real artifact.
     *
     * @param path the RLOG to read
     * @return the sorted key names, minus PsiKit's own writer metadata
     * @throws IOException              if the RLOG exists but cannot be read
     * @throws IllegalArgumentException if the RLOG is missing or empty (see {@link #decode(Path)})
     */
    public static Set<String> decodedKeys(Path path) throws IOException {
        Set<String> keys = new TreeSet<>();
        forEachFrame(path, frame -> keys.addAll(frame.keySet()));
        keys.removeAll(EXCLUDED_KEYS);
        return keys;
    }

    /**
     * Every frame of one RLOG, in order, each as its decoded {@code key -> value} map (values rendered the
     * way PsiKit renders them: {@code "12.5"}, {@code "true"}, {@code "HOLDING"}).
     *
     * <p>Public so a test can assert what a subsystem published <b>by value</b>, not just by key name —
     * {@link #decodedKeys(Path)} proves a key called {@code measuredIn} exists, and only this proves the
     * number under it is actually inches.
     *
     * <p>It holds every frame in memory at once, one map of the whole robot state per frame, so a long run with
     * many keys can need hundreds of MB. To look at a long log, stream it with {@link #forEachFrame}; to check
     * its length, use {@link #countFrames}.
     *
     * @param path the RLOG to read
     * @return one map per frame, in order, each holding the whole decoded robot state at that frame
     * @throws IOException              if the RLOG exists but cannot be read
     * @throws IllegalArgumentException if the RLOG is missing or empty: PsiKit's own reader silently
     *                                  decodes a missing file as zero frames, which would let two failed
     *                                  runs compare as a false "no mismatches" pass
     */
    public static List<Map<String, String>> decode(Path path) throws IOException {
        return decodeFrames(path, false);
    }

    /**
     * Every frame of one RLOG, in order, handed to {@code visitor} one at a time and then dropped: the same
     * maps {@link #decode(Path)} returns, without holding them all. A test that reads a long run's log keeps
     * only what it needs from each frame.
     *
     * @param path    the RLOG to read
     * @param visitor called once per frame with that frame's whole decoded state; the map is the visitor's
     *                to keep or drop
     * @return how many frames there were
     * @throws IOException              if the RLOG exists but cannot be read
     * @throws IllegalArgumentException if the RLOG is missing or empty (see {@link #decode(Path)})
     */
    public static int forEachFrame(Path path, Consumer<Map<String, String>> visitor) throws IOException {
        return visitFrames(path, false, Objects.requireNonNull(visitor, "visitor"));
    }

    /**
     * How many frames one RLOG holds, read without rendering or keeping any of them.
     *
     * <p><b>On the production path.</b> {@link ScenarioRunner#run} calls this on every real scenario run, to
     * check that the file it just wrote holds one frame per tick before handing it back (BACKLOG B32), so its
     * cost and its failure modes are a scenario's. The rest of this class is verification-only. Until
     * 2026-10-20 that check decoded the whole log into memory, which ran a 300 s run with the hardware-input
     * keys out of a 512 MB test JVM.
     *
     * @param path the RLOG to read
     * @return the number of frames
     * @throws IOException              if the RLOG exists but cannot be read
     * @throws IllegalArgumentException if the RLOG is missing or empty (see {@link #decode(Path)})
     */
    public static int countFrames(Path path) throws IOException {
        return visitFrames(path, false, null);
    }

    /**
     * Reads every frame of one RLOG into a list. With {@code withType} false, each value is PsiKit's own
     * rendering ({@code "12.5"}); with it true, the logged type follows in brackets ({@code "12.5 [Double]"},
     * or {@code "... [Raw:struct:Pose2d]"} when a custom type name was logged), which is what
     * {@link #compare} needs to tell a string {@code "1.0"} from a double {@code 1.0}.
     */
    private static List<Map<String, String>> decodeFrames(Path path, boolean withType) throws IOException {
        List<Map<String, String>> frames = new ArrayList<>();
        visitFrames(path, withType, frames::add);
        return frames;
    }

    /**
     * The one frame loop. Each frame is rendered as {@link #decodeFrames} describes and handed to
     * {@code visitor}; with a null visitor nothing is rendered and the frames are only counted.
     *
     * @return the number of frames
     */
    private static int visitFrames(Path path, boolean withType, Consumer<Map<String, String>> visitor)
            throws IOException {
        if (!Files.exists(path) || Files.size(path) == 0) {
            throw new IllegalArgumentException(
                    "RLOG file missing or empty, cannot decode: " + path
                    + " (PsiKit's RLOGReplay silently decodes a missing file as zero frames, which would "
                    + "otherwise let two missing/failed logs compare as a false 'no mismatches' pass)");
        }
        RLOGReplay replay = new RLOGReplay(path.toString());
        replay.start();
        // end() closes the file. Without it the stream stayed open until garbage collection, and on Windows
        // a file held open cannot be deleted - so the next ScenarioRunner.run to the same path had PsiKit's
        // writer fail its delete and APPEND the new run to the old one (2026-09-26 review).
        try {
            LogTable table = new LogTable(0);
            int frames = 0;
            while (replay.updateTable(table)) {
                frames++;
                if (visitor == null) {
                    continue;
                }
                Map<String, String> snapshot = new TreeMap<>();
                for (Map.Entry<String, LogTable.LogValue> entry : table.getAll(true).entrySet()) {
                    LogTable.LogValue value = entry.getValue();
                    String rendered = String.valueOf(value);
                    if (withType && value != null) {
                        rendered = rendered + " [" + value.type
                                + (value.customTypeStr == null ? "" : ":" + value.customTypeStr) + "]";
                    }
                    snapshot.put(entry.getKey(), rendered);
                }
                visitor.accept(snapshot);
            }
            return frames;
        } finally {
            replay.end();
        }
    }
}
