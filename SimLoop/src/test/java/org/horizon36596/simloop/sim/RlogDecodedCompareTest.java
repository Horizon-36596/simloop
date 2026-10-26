package org.horizon36596.simloop.sim;

import org.junit.jupiter.api.Test;

import org.psilynx.psikit.core.LogTable;
import org.psilynx.psikit.core.rlog.RLOGWriter;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * Pins {@code RlogDecodedCompare}'s contract (BACKLOG B3): decoded robot fields must match between
 * two runs; PsiKit's own writer-thread metadata (Console / QueuedCycles) must NOT be compared; any
 * other differing key -- including a key present in only one run's frame, or the two runs having a
 * different total frame count -- must be reported.
 */
public class RlogDecodedCompareTest {

    @Test
    public void identicalRobotFieldsWithDifferingPsikitMetaProduceNoMismatch() throws IOException {
        Path dir = Files.createTempDirectory("rlog-compare-test");
        Path pathA = writeLog(dir, "run-a", 1.0, 111);
        Path pathB = writeLog(dir, "run-b", 1.0, 222);

        List<RlogDecodedCompare.Mismatch> mismatches = RlogDecodedCompare.compare(pathA, pathB);

        assertTrue(mismatches.isEmpty(), "expected no mismatches, got: " + mismatches);
    }

    @Test
    public void differingRobotFieldIsReportedAsAMismatch() throws IOException {
        Path dir = Files.createTempDirectory("rlog-compare-test");
        Path pathA = writeLog(dir, "run-a", 1.0, 111);
        Path pathB = writeLog(dir, "run-b", 2.0, 111);

        List<RlogDecodedCompare.Mismatch> mismatches = RlogDecodedCompare.compare(pathA, pathB);

        assertEquals(1, mismatches.size());
        assertEquals("RealOutputs/Drive/pose/x", mismatches.get(0).key);
    }

    @Test
    public void differingFrameCountIsReportedAsAMismatch() throws IOException {
        Path dir = Files.createTempDirectory("rlog-compare-test");
        Path pathA = writeLogWithFrames(dir, "run-a", 2);
        Path pathB = writeLogWithFrames(dir, "run-b", 1);

        List<RlogDecodedCompare.Mismatch> mismatches = RlogDecodedCompare.compare(pathA, pathB);

        assertEquals(1, mismatches.size());
        assertTrue(mismatches.get(0).isFrameCountMismatch(), "expected a frame-count mismatch, got: " + mismatches);
        assertEquals("2", mismatches.get(0).valueA);
        assertEquals("1", mismatches.get(0).valueB);
    }

    @Test
    public void keyPresentInOnlyOneRunIsReportedAsAMismatch() throws IOException {
        Path dir = Files.createTempDirectory("rlog-compare-test");
        Path pathA = writeLogWithExtraKey(dir, "run-a", true);
        Path pathB = writeLogWithExtraKey(dir, "run-b", false);

        List<RlogDecodedCompare.Mismatch> mismatches = RlogDecodedCompare.compare(pathA, pathB);

        assertEquals(1, mismatches.size());
        assertTrue(!mismatches.get(0).isFrameCountMismatch());
        assertEquals("RealOutputs/Drive/pose/y", mismatches.get(0).key);
    }

    @Test
    public void callerSuppliedExcludeSetOverridesTheDefault() throws IOException {
        Path dir = Files.createTempDirectory("rlog-compare-test");
        Path pathA = writeLog(dir, "run-a", 1.0, 111);
        Path pathB = writeLog(dir, "run-b", 2.0, 111);

        // With an exclude set that also covers the robot field, the one real difference is filtered too.
        List<RlogDecodedCompare.Mismatch> mismatches = RlogDecodedCompare.compare(pathA, pathB,
                java.util.Collections.singleton("RealOutputs/Drive/pose/x"));

        assertTrue(mismatches.isEmpty(), "expected no mismatches with a caller override, got: " + mismatches);
    }

    @Test
    public void aStringAndADoubleThatRenderTheSameAreStillAMismatch() throws IOException {
        Path dir = Files.createTempDirectory("rlog-compare-test");
        Path pathA = writeFrames(dir, "run-a", table -> table.put("RealOutputs/Drive/mode", "1.0"));
        Path pathB = writeFrames(dir, "run-b", table -> table.put("RealOutputs/Drive/mode", 1.0));

        List<RlogDecodedCompare.Mismatch> mismatches = RlogDecodedCompare.compare(pathA, pathB);

        // PsiKit renders both as "1.0"; only the logged type tells them apart.
        assertEquals(1, mismatches.size(), "expected the type difference to be reported, got: " + mismatches);
        assertEquals("1.0 [String]", mismatches.get(0).valueA);
        assertEquals("1.0 [Double]", mismatches.get(0).valueB);
    }

    @Test
    public void aDivergenceAfterTheFirstFrameIsReportedAtItsOwnFrame() throws IOException {
        Path dir = Files.createTempDirectory("rlog-compare-test");
        Path pathA = writeFrames(dir, "run-a",
                table -> table.put("RealOutputs/Drive/pose/x", 1.0),
                table -> table.put("RealOutputs/Drive/pose/x", 2.0),
                table -> table.put("RealOutputs/Drive/pose/x", 3.0));
        Path pathB = writeFrames(dir, "run-b",
                table -> table.put("RealOutputs/Drive/pose/x", 1.0),
                table -> table.put("RealOutputs/Drive/pose/x", 2.0),
                table -> table.put("RealOutputs/Drive/pose/x", 3.5));

        List<RlogDecodedCompare.Mismatch> mismatches = RlogDecodedCompare.compare(pathA, pathB);

        assertEquals(1, mismatches.size(), "only the last frame differs, got: " + mismatches);
        assertEquals(2, mismatches.get(0).frameIndex);
        assertEquals("RealOutputs/Drive/pose/x", mismatches.get(0).key);
    }

    @Test
    public void aValueLoggedOnceCarriesForwardIntoLaterFrames() throws IOException {
        // A key not logged again in a later frame still holds its last value in that frame's decoded state,
        // the way AdvantageScope shows it. So a run that re-logs an unchanged value every frame and a run that
        // logs it once agree, and a run that logs a NEW value at frame 2 differs from frame 2 on.
        Path dir = Files.createTempDirectory("rlog-compare-test");
        Path everyFrame = writeFrames(dir, "run-every-frame",
                table -> table.put("RealOutputs/Drive/pose/x", 1.0),
                table -> table.put("RealOutputs/Drive/pose/x", 1.0),
                table -> table.put("RealOutputs/Drive/pose/x", 1.0));
        Path onceOnly = writeFrames(dir, "run-once-only",
                table -> table.put("RealOutputs/Drive/pose/x", 1.0),
                table -> table.put("RealOutputs/Drive/other", 0.0),
                table -> table.put("RealOutputs/Drive/other", 0.0));
        Path changedLate = writeFrames(dir, "run-changed-late",
                table -> table.put("RealOutputs/Drive/pose/x", 1.0),
                table -> table.put("RealOutputs/Drive/pose/x", 1.0),
                table -> table.put("RealOutputs/Drive/pose/x", 9.0));

        List<Map<String, String>> decodedOnce = RlogDecodedCompare.decode(onceOnly);
        assertEquals("1.0", decodedOnce.get(2).get("RealOutputs/Drive/pose/x"),
                "the value logged at frame 0 is still the decoded value at frame 2");

        List<RlogDecodedCompare.Mismatch> againstEveryFrame = RlogDecodedCompare.compare(everyFrame, onceOnly);
        for (RlogDecodedCompare.Mismatch mismatch : againstEveryFrame) {
            assertEquals("RealOutputs/Drive/other", mismatch.key,
                    "pose/x carried forward, so only the extra key differs, got: " + againstEveryFrame);
        }

        List<RlogDecodedCompare.Mismatch> againstChanged = RlogDecodedCompare.compare(everyFrame, changedLate);
        assertEquals(1, againstChanged.size(), "got: " + againstChanged);
        assertEquals(2, againstChanged.get(0).frameIndex);
    }

    @Test
    public void streamingVisitsTheFramesDecodeReturnsAndCountingAgrees() throws IOException {
        // forEachFrame and countFrames read the log without holding it; they must see what decode sees.
        Path dir = Files.createTempDirectory("rlog-compare-test");
        Path path = writeFrames(dir, "run-a",
                table -> table.put("RealOutputs/Drive/pose/x", 1.0),
                table -> table.put("RealOutputs/Drive/other", 0.0),
                table -> table.put("RealOutputs/Drive/pose/x", 9.0));

        List<Map<String, String>> decoded = RlogDecodedCompare.decode(path);
        List<Map<String, String>> visited = new ArrayList<>();
        int returned = RlogDecodedCompare.forEachFrame(path, visited::add);

        assertEquals(decoded, visited, "the same frames, in the same order, carried-forward values included");
        assertEquals(decoded.size(), returned, "forEachFrame returns the number of frames it visited");
        assertEquals(decoded.size(), RlogDecodedCompare.countFrames(path));

        // Like decode, both release the file (only discriminates on Windows; see decodingAFileReleasesIt).
        Files.delete(path);
        assertTrue(!Files.exists(path), "the streamed file could be deleted straight afterwards");
    }

    @Test
    public void summariseListsTheEarliestFortyKeysAndCountsTheRest() {
        List<RlogDecodedCompare.Mismatch> mismatches = new ArrayList<>();
        // 45 keys; key k first diverges at frame 100 - k, so the key that went wrong earliest sorts LAST by
        // name. The summary must still lead with it.
        for (int k = 0; k < 45; k++) {
            String key = String.format("RealOutputs/Key%02d", k);
            mismatches.add(new RlogDecodedCompare.Mismatch(100 - k, key, "1.0 [Double]", "2.0 [Double]"));
            mismatches.add(new RlogDecodedCompare.Mismatch(101 - k, key, "1.0 [Double]", "3.0 [Double]"));
        }

        String summary = RlogDecodedCompare.summarise(mismatches);
        String[] lines = summary.split("\n");

        assertTrue(lines[0].startsWith("90 field mismatches across 45 keys, earliest at frame 56"), summary);
        assertTrue(lines[1].contains("frame 56  RealOutputs/Key44 x2"), summary);
        assertEquals(1 + 40 + 1, lines.length, "totals line, 40 key lines, one 'and N more' line: " + summary);
        assertTrue(lines[41].contains("and 5 more keys, all diverging at frame 96 or later"), summary);
    }

    @Test
    public void summariseReportsAFrameCountMismatchOnItsOwnLine() {
        List<RlogDecodedCompare.Mismatch> mismatches = new ArrayList<>();
        mismatches.add(new RlogDecodedCompare.Mismatch(-1, "<frame count>", "20", "30"));
        mismatches.add(new RlogDecodedCompare.Mismatch(7, "RealOutputs/Drive/pose/x", "1.0", "2.0"));

        String summary = RlogDecodedCompare.summarise(mismatches);
        String[] lines = summary.split("\n");

        assertEquals("frame count differs: 20 vs 30", lines[0], summary);
        assertTrue(lines[1].startsWith("1 field mismatches across 1 keys, earliest at frame 7"),
                "the earliest divergence is the first real frame, not the frame-count entry's -1: " + summary);
    }

    /**
     * The decode must close the file. Only discriminates on Windows, where a file still held open cannot be
     * deleted; Linux and macOS allow deleting an open file, so there this passes with or without the fix.
     */
    @Test
    public void decodingAFileReleasesIt() throws IOException {
        Path dir = Files.createTempDirectory("rlog-compare-test");
        Path path = writeLog(dir, "run-a", 1.0, 111);

        RlogDecodedCompare.decode(path);

        Files.delete(path);
        assertTrue(!Files.exists(path), "the decoded file could be deleted straight afterwards");
    }

    /** What one frame of a test RLOG logs. */
    private interface FrameWriter {
        void write(LogTable table);
    }

    /** Writes an RLOG with one frame per {@code frames} entry, timestamps stepping by 20 ms. */
    private static Path writeFrames(Path dir, String name, FrameWriter... frames) {
        RLOGWriter writer = new RLOGWriter(dir.toString(), name);
        writer.start();
        for (int i = 0; i < frames.length; i++) {
            LogTable table = new LogTable(i * 0.02);
            frames[i].write(table);
            writer.putTable(table);
        }
        writer.end();
        return dir.resolve(name + ".rlog");
    }

    /** Writes a single-frame RLOG with a robot field plus PsiKit-meta-shaped keys at the given values. */
    private static Path writeLog(Path dir, String name, double poseX, int queuedCycles) {
        RLOGWriter writer = new RLOGWriter(dir.toString(), name);
        writer.start();
        LogTable table = new LogTable(0.0);
        table.put("RealOutputs/Drive/pose/x", poseX);
        table.put("RealOutputs/Logger/QueuedCycles", queuedCycles);
        table.put("RealOutputs/Console", "line-" + queuedCycles);
        writer.putTable(table);
        writer.end();
        return dir.resolve(name + ".rlog");
    }

    /** Writes an RLOG with {@code frameCount} frames, each carrying the same robot field value. */
    private static Path writeLogWithFrames(Path dir, String name, int frameCount) {
        RLOGWriter writer = new RLOGWriter(dir.toString(), name);
        writer.start();
        for (int i = 0; i < frameCount; i++) {
            LogTable table = new LogTable(i * 0.02);
            table.put("RealOutputs/Drive/pose/x", 1.0);
            writer.putTable(table);
        }
        writer.end();
        return dir.resolve(name + ".rlog");
    }

    /** Writes a single-frame RLOG that includes an extra robot key only when {@code includeExtraKey} is true. */
    private static Path writeLogWithExtraKey(Path dir, String name, boolean includeExtraKey) {
        RLOGWriter writer = new RLOGWriter(dir.toString(), name);
        writer.start();
        LogTable table = new LogTable(0.0);
        table.put("RealOutputs/Drive/pose/x", 1.0);
        if (includeExtraKey) {
            table.put("RealOutputs/Drive/pose/y", 2.0);
        }
        writer.putTable(table);
        writer.end();
        return dir.resolve(name + ".rlog");
    }
}
