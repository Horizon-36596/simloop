package org.horizon36596.simloop.sim.live;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertThrows;

import org.junit.jupiter.api.Test;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.Collections;

/**
 * Pins the recorder and the gate that sit between a source and the runner. INVARIANT tier: a recorder that
 * wrote a frame the runner did not run, or skipped one it did, would make the track a different session.
 */
class RecordingGamepadInputSourceTest {

    private static final Path DIR = Paths.get("build", "sim", "live-track-test");

    @Test
    void invariant_everyFramePassedOnIsWrittenAndTheEndIsNot() throws IOException {
        Files.createDirectories(DIR);
        Path file = DIR.resolve("recorded.track.txt");
        GamepadInputTrack original = trackOf(3);

        try (GamepadInputTrack.Writer writer = new GamepadInputTrack.Writer(file,
                Collections.<String, String>emptyMap(), true)) {
            GamepadInputSource recording = new RecordingGamepadInputSource(original.replay(), writer);
            for (int tick = 0; tick < 3; tick++) {
                assertEquals(original.frames().get(tick), recording.next(tick));
            }
            assertNull(recording.next(3));
            assertEquals(3, writer.framesWritten());
        }
        assertEquals(original.frames(), GamepadInputTrack.read(file).frames());
    }

    @Test
    void aSourceThatAnswersForTheWrongTickIsRefused() throws IOException {
        Files.createDirectories(DIR);
        try (GamepadInputTrack.Writer writer = new GamepadInputTrack.Writer(DIR.resolve("wrong-tick.track.txt"),
                Collections.<String, String>emptyMap(), true)) {
            GamepadInputSource answersTickFive = new GamepadInputSource() {
                @Override
                public GamepadInputTrack.Frame next(int tickIndex) {
                    return new GamepadInputTrack.Frame(5, GamepadState.AT_REST, GamepadState.AT_REST);
                }
            };
            RecordingGamepadInputSource recording = new RecordingGamepadInputSource(answersTickFive, writer);

            assertThrows(IllegalStateException.class, () -> recording.next(0));
            assertEquals(0, writer.framesWritten(), "nothing is written for a frame that is refused");
        }
    }

    @Test
    void invariant_theGateHandsTheBodyTheFrameItFetchedAndFetchesOncePerTick() {
        int[] fetches = {0};
        GamepadInputTrack track = trackOf(2);
        GamepadInputSource counted = new GamepadInputSource() {
            @Override
            public GamepadInputTrack.Frame next(int tickIndex) {
                fetches[0]++;
                return track.replay().next(tickIndex);
            }
        };
        GamepadInputGate gate = new GamepadInputGate(counted);

        assertEquals(true, gate.shouldRunTick(0));
        assertEquals(track.frames().get(0), gate.current());
        assertEquals(track.frames().get(0), gate.current(), "reading it twice does not fetch again");
        assertEquals(true, gate.shouldRunTick(1));
        assertEquals(track.frames().get(1), gate.current());
        assertEquals(false, gate.shouldRunTick(2));
        assertEquals(3, fetches[0]);
        assertThrows(IllegalStateException.class, gate::current, "no tick is running after the last one");
    }

    /** A track of {@code ticks} frames with gamepad 1's {@code a} down on even ticks. */
    private static GamepadInputTrack trackOf(int ticks) {
        StringBuilder text = new StringBuilder(GamepadInputTrack.FORMAT_LINE).append('\n');
        for (int tick = 0; tick < ticks; tick++) {
            text.append(tick).append(tick % 2 == 0 ? " | g1 a | g2\n" : " | g1 | g2\n");
        }
        try {
            Files.createDirectories(DIR);
            Path file = DIR.resolve("source-" + ticks + ".track.txt");
            Files.write(file, text.toString().getBytes(java.nio.charset.StandardCharsets.UTF_8));
            return GamepadInputTrack.read(file);
        } catch (IOException e) {
            throw new java.io.UncheckedIOException(e);
        }
    }
}
