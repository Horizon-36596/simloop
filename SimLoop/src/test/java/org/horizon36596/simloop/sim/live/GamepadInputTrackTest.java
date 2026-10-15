package org.horizon36596.simloop.sim.live;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.Arrays;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.TreeMap;
import java.util.TreeSet;

/**
 * Pins the track file's two promises: a person can read it, and it replays the session bit for bit.
 * INVARIANT tier: any redesign of the format must keep both, or a replay stops being the session.
 *
 * <p>Writes under {@code build/sim/live-track-test} rather than a JUnit temp dir, for the same Windows
 * file-handle reason the other sim tests give.
 */
class GamepadInputTrackTest {

    private static final Path DIR = Paths.get("build", "sim", "live-track-test");

    /** Values chosen because their decimal form is NOT exact: 0.1f, 1/3f and friends. */
    private static final float[] AWKWARD_VALUES = {
            0.1f, -0.1f, 1.0f / 3.0f, -2.0f / 3.0f, 0.35f, -0.35f, 0.7f, 1.0f, -1.0f,
            Float.MIN_VALUE, -Float.MIN_VALUE, 0.99999994f, 1e-7f, Math.nextDown(1.0f), Math.nextUp(-1.0f)};

    @Test
    void invariant_everyAxisValueSurvivesTheTextBitForBit() {
        for (float value : AWKWARD_VALUES) {
            Map<String, Float> axes = new TreeMap<>();
            axes.put("left_stick_y", value);
            GamepadInputTrack.Frame frame = new GamepadInputTrack.Frame(0,
                    GamepadState.of(axes, Collections.<String>emptySet()), GamepadState.AT_REST);

            GamepadInputTrack.Frame parsed = GamepadInputTrack.parseLine(GamepadInputTrack.formatLine(frame), 1);

            assertEquals(Float.floatToIntBits(value),
                    Float.floatToIntBits(parsed.gamepad1.axis("left_stick_y")),
                    "left_stick_y=" + value + " must read back as exactly the same float");
        }
    }

    @Test
    void invariant_aWrittenTrackReadsBackAsTheSameFrames() throws IOException {
        Files.createDirectories(DIR);
        Path file = DIR.resolve("round-trip.track.txt");
        List<GamepadInputTrack.Frame> frames = Arrays.asList(
                new GamepadInputTrack.Frame(0, state("left_stick_y", -1.0f, "a", "dpad_right"), GamepadState.AT_REST),
                new GamepadInputTrack.Frame(1, state("right_trigger", 0.5f), state("left_stick_x", 0.35f, "y")),
                new GamepadInputTrack.Frame(2, GamepadState.AT_REST, GamepadState.AT_REST));
        Map<String, String> metadata = new LinkedHashMap<>();
        metadata.put("opMode", "SomeTeleOp");
        metadata.put("tickSeconds", "0.02");

        try (GamepadInputTrack.Writer writer = new GamepadInputTrack.Writer(file, metadata, true)) {
            for (GamepadInputTrack.Frame frame : frames) {
                writer.append(frame);
            }
        }
        GamepadInputTrack read = GamepadInputTrack.read(file);

        assertEquals(frames, read.frames());
        assertEquals(metadata, read.metadata());
    }

    @Test
    void theFileIsOneReadableLinePerLoop() throws IOException {
        Files.createDirectories(DIR);
        Path file = DIR.resolve("readable.track.txt");
        try (GamepadInputTrack.Writer writer = new GamepadInputTrack.Writer(file,
                Collections.singletonMap("opMode", "SomeTeleOp"), true)) {
            writer.append(new GamepadInputTrack.Frame(0, state("left_stick_y", -1.0f, "a"), GamepadState.AT_REST));
        }
        List<String> lines = Files.readAllLines(file, StandardCharsets.UTF_8);

        assertEquals(GamepadInputTrack.FORMAT_LINE, lines.get(0));
        assertEquals("# opMode: SomeTeleOp", lines.get(2));
        assertEquals("0 | g1 left_stick_y=-1.0 a | g2", lines.get(3));
        assertEquals(4, lines.size(), "header of three lines, then one line for the one loop");
    }

    @Test
    void aTrackWithAGapInItsTicksIsRefused() throws IOException {
        Files.createDirectories(DIR);
        Path file = DIR.resolve("gap.track.txt");
        Files.write(file, Arrays.asList(GamepadInputTrack.FORMAT_LINE, "0 | g1 | g2", "2 | g1 | g2"),
                StandardCharsets.UTF_8);

        IllegalArgumentException refused = assertThrows(IllegalArgumentException.class,
                () -> GamepadInputTrack.read(file));
        assertTrue(refused.getMessage().contains("line 3"), refused.getMessage());
    }

    @Test
    void aHandWrittenHeaderWithoutASpaceAfterTheColonIsStillMetadata() throws IOException {
        Files.createDirectories(DIR);
        Path file = DIR.resolve("no-space.track.txt");
        Files.write(file, Arrays.asList(GamepadInputTrack.FORMAT_LINE, "# tickSeconds:0.02",
                "# a comment: with a colon", "0 | g1 | g2"), StandardCharsets.UTF_8);

        Map<String, String> metadata = GamepadInputTrack.read(file).metadata();

        assertEquals("0.02", metadata.get("tickSeconds"));
        assertEquals(1, metadata.size(), "a line whose text before the colon has a space is a comment");
    }

    @Test
    void aFileWithoutTheFormatLineIsRefused() throws IOException {
        Files.createDirectories(DIR);
        Path file = DIR.resolve("not-a-track.txt");
        Files.write(file, Collections.singletonList("0 | g1 | g2"), StandardCharsets.UTF_8);

        assertThrows(IllegalArgumentException.class, () -> GamepadInputTrack.read(file));
    }

    @Test
    void unknownControlsAndOutOfRangeValuesAreRefusedWithTheirLineNumber() {
        String[] badLines = {
                "0 | g1 cross | g2",                  // an alias, not an SDK packet field
                "0 | g1 left_stick_y=1.5 | g2",       // outside [-1, 1]
                "0 | g1 left_trigger=-0.5 | g2",      // triggers are [0, 1]
                "0 | g1 left_stick_y=NaN | g2",
                "0 | g1 a a | g2",                    // listed twice
                "0 | g2 | g1",                        // parts swapped
                "zero | g1 | g2"};
        for (String line : badLines) {
            IllegalArgumentException refused = assertThrows(IllegalArgumentException.class,
                    () -> GamepadInputTrack.parseLine(line, 7), line);
            assertTrue(refused.getMessage().contains("line 7"), refused.getMessage());
        }
    }

    @Test
    void theWriterRefusesToSkipATick() throws IOException {
        Files.createDirectories(DIR);
        try (GamepadInputTrack.Writer writer = new GamepadInputTrack.Writer(DIR.resolve("skip.track.txt"),
                Collections.<String, String>emptyMap(), true)) {
            assertThrows(IllegalArgumentException.class, () -> writer.append(
                    new GamepadInputTrack.Frame(1, GamepadState.AT_REST, GamepadState.AT_REST)));
        }
    }

    @Test
    void aTrackSavedWithAByteOrderMarkStillReads() throws IOException {
        // Windows PowerShell 5.1's Out-File and Notepad's "UTF-8 with BOM" put U+FEFF in front of line 1.
        Files.createDirectories(DIR);
        Path file = DIR.resolve("bom.track.txt");
        Files.write(file, Arrays.asList("﻿" + GamepadInputTrack.FORMAT_LINE, "0 | g1 a | g2"),
                StandardCharsets.UTF_8);

        GamepadInputTrack read = GamepadInputTrack.read(file);

        assertEquals(1, read.frames().size());
        assertTrue(read.frames().get(0).gamepad1.isDown("a"));
    }

    @Test
    void aTrackInAFormatThisCodeDoesNotKnowIsRefusedByItsFormatNumber() throws IOException {
        Files.createDirectories(DIR);
        Path file = DIR.resolve("format-2.track.txt");
        Files.write(file, Arrays.asList(GamepadInputTrack.FORMAT_PREFIX + "2", "0 | g1 | g2"),
                StandardCharsets.UTF_8);

        IllegalArgumentException refused = assertThrows(IllegalArgumentException.class,
                () -> GamepadInputTrack.read(file));
        assertTrue(refused.getMessage().contains("format 2"), refused.getMessage());
    }

    @Test
    void aMetadataKeyGivenTwiceIsRefusedWithItsLineNumber() throws IOException {
        Files.createDirectories(DIR);
        Path file = DIR.resolve("duplicate-key.track.txt");
        Files.write(file, Arrays.asList(GamepadInputTrack.FORMAT_LINE, "# alliance: RED", "# alliance: BLUE",
                "0 | g1 | g2"), StandardCharsets.UTF_8);

        IllegalArgumentException refused = assertThrows(IllegalArgumentException.class,
                () -> GamepadInputTrack.read(file));
        assertTrue(refused.getMessage().contains("line 3"), refused.getMessage());
        assertTrue(refused.getMessage().contains("alliance"), refused.getMessage());
    }

    @Test
    void badMetadataIsRefusedBeforeTheFileIsCreated() throws IOException {
        // Checked before the file is opened, so a refusal leaves no half-written file and no open handle.
        Files.createDirectories(DIR);
        Path file = DIR.resolve("bad-metadata.track.txt");
        Files.deleteIfExists(file);

        assertThrows(IllegalArgumentException.class, () -> new GamepadInputTrack.Writer(file,
                Collections.singletonMap("two words", "value"), true));

        assertFalse(Files.exists(file), "a refused writer must not have created the file");
    }

    @Test
    void theWriterRefusesToReplaceAnExistingTrackUnlessToldTo() throws IOException {
        Files.createDirectories(DIR);
        Path file = DIR.resolve("existing.track.txt");
        Files.write(file, Collections.singletonList("an earlier session"), StandardCharsets.UTF_8);

        assertThrows(IllegalStateException.class, () -> new GamepadInputTrack.Writer(file,
                Collections.<String, String>emptyMap(), false));
        assertEquals(Collections.singletonList("an earlier session"),
                Files.readAllLines(file, StandardCharsets.UTF_8), "the earlier file must be untouched");

        new GamepadInputTrack.Writer(file, Collections.<String, String>emptyMap(), true).close();
        assertEquals(GamepadInputTrack.FORMAT_LINE, Files.readAllLines(file, StandardCharsets.UTF_8).get(0));
    }

    /** A state with one axis and some buttons down. */
    private static GamepadState state(String axis, float value, String... buttons) {
        Map<String, Float> axes = new TreeMap<>();
        axes.put(axis, value);
        return GamepadState.of(axes, new TreeSet<>(Arrays.asList(buttons)));
    }

    /** A state with only buttons down. */
    private static GamepadState state(String... buttons) {
        return GamepadState.of(Collections.<String, Float>emptyMap(), new TreeSet<>(Arrays.asList(buttons)));
    }
}
