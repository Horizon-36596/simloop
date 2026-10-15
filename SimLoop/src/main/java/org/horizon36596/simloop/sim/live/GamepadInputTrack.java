package org.horizon36596.simloop.sim.live;

import java.io.BufferedWriter;
import java.io.Closeable;
import java.io.IOException;
import java.io.UncheckedIOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.FileAlreadyExistsException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.StandardOpenOption;
import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.TreeMap;
import java.util.TreeSet;

/**
 * A recorded driving session: what both gamepads read on every sim loop, one plain-text line per loop.
 *
 * <p>The file is meant to be read by a person as well as replayed by the sim. A session that drove
 * forward for two loops with the right trigger half in, pressed {@code a} once, and never touched
 * gamepad 2 looks like this:
 * <pre>
 * # Horizon (FTC 36596) SimLoop gamepad input track, format 1
 * # One line per sim loop: tick | g1 &lt;gamepad 1&gt; | g2 &lt;gamepad 2&gt;
 * # opMode: FullTeleOp
 * # tickSeconds: 0.02
 * 0 | g1 left_stick_y=-1.0 right_trigger=0.5 a | g2
 * 1 | g1 left_stick_y=-1.0 right_trigger=0.5 | g2
 * </pre>
 *
 * <ul>
 *   <li><b>Only what is not at rest is written.</b> An axis appears as {@code name=value} when it is not
 *       zero; a button appears as its bare name when it is down. Names are {@link GamepadState}'s, which
 *       are the SDK's own field names.</li>
 *   <li><b>Values replay bit for bit.</b> Each is written with {@link Float#toString(float)}, which
 *       prints a decimal that {@link Float#parseFloat} reads back to exactly the same {@code float} bits.
 *       That is the whole reason a replay can reproduce a live session exactly: the robot code sees the
 *       same bits both times.</li>
 *   <li><b>Header lines start with {@code #}.</b> The first must be {@link #FORMAT_LINE}; a byte order mark
 *       in front of it (Windows PowerShell 5.1's {@code Out-File} and Notepad's "UTF-8 with BOM" add one)
 *       is ignored, and a track in a format number this code does not know is refused by that number. A
 *       header line of the form {@code # key: value} (one-word key) is metadata (which OpMode, which
 *       alliance, the tick length), and a key may appear only once; any other {@code #} line is a
 *       comment.</li>
 *   <li><b>Ticks are counted from 0 with no gaps.</b> A file that skips or repeats a tick is refused,
 *       because a replay of it could not be the session that was recorded.</li>
 * </ul>
 *
 * <p>Pure data and text: no clock, no randomness (domain R5). Season-agnostic: nothing here names a
 * robot's controls.
 */
public final class GamepadInputTrack {

    /** The first line of every track, up to its format number. */
    public static final String FORMAT_PREFIX = "# Horizon (FTC 36596) SimLoop gamepad input track, format ";

    /** The only format number this code reads and writes. Bumped only if the line format ever changes. */
    public static final int FORMAT_NUMBER = 1;

    /** The first line of every track. */
    public static final String FORMAT_LINE = FORMAT_PREFIX + FORMAT_NUMBER;

    /** U+FEFF, the byte order mark some Windows editors put at the very start of a UTF-8 file. */
    private static final char BYTE_ORDER_MARK = '\uFEFF';

    /** The second line, which says how to read the rest. Written, never required on read. */
    static final String LAYOUT_LINE = "# One line per sim loop: tick | g1 <gamepad 1> | g2 <gamepad 2>";

    /** Both gamepads on one loop. */
    public static final class Frame {
        /** Which loop this is, counting from 0. */
        public final int tick;
        /** What gamepad 1 (the driver) read on this loop. */
        public final GamepadState gamepad1;
        /** What gamepad 2 (the operator) read on this loop. */
        public final GamepadState gamepad2;

        /**
         * Both gamepads' states on one loop.
         *
         * @param tick     which loop, counting from 0; must not be negative
         * @param gamepad1 gamepad 1's state on that loop
         * @param gamepad2 gamepad 2's state on that loop
         */
        public Frame(int tick, GamepadState gamepad1, GamepadState gamepad2) {
            if (tick < 0) {
                throw new IllegalArgumentException("tick must not be negative, got " + tick);
            }
            if (gamepad1 == null || gamepad2 == null) {
                throw new IllegalArgumentException("both gamepad states are required; use GamepadState.AT_REST");
            }
            this.tick = tick;
            this.gamepad1 = gamepad1;
            this.gamepad2 = gamepad2;
        }

        @Override
        public boolean equals(Object other) {
            if (!(other instanceof Frame)) {
                return false;
            }
            Frame that = (Frame) other;
            return tick == that.tick && gamepad1.equals(that.gamepad1) && gamepad2.equals(that.gamepad2);
        }

        @Override
        public int hashCode() {
            return 31 * (31 * tick + gamepad1.hashCode()) + gamepad2.hashCode();
        }

        /** The line this frame is written as. */
        @Override
        public String toString() {
            return formatLine(this);
        }
    }

    private final Map<String, String> metadata;
    private final List<Frame> frames;

    private GamepadInputTrack(Map<String, String> metadata, List<Frame> frames) {
        this.metadata = Collections.unmodifiableMap(metadata);
        this.frames = Collections.unmodifiableList(frames);
    }

    /** {@return the {@code # key: value} header entries, in file order} */
    public Map<String, String> metadata() {
        return metadata;
    }

    /** {@return every loop, in order; frame {@code i} is tick {@code i}} */
    public List<Frame> frames() {
        return frames;
    }

    /**
     * A source that plays this track back, one frame per loop, and ends after the last one. This is what a
     * headless replay runs on.
     *
     * @return a source that hands out this track's frames in order
     */
    public GamepadInputSource replay() {
        return new GamepadInputSource() {
            @Override
            public Frame next(int tickIndex) {
                return tickIndex < frames.size() ? frames.get(tickIndex) : null;
            }
        };
    }

    // ---------------------------------------------------------------------------------------------
    // Text <-> frames
    // ---------------------------------------------------------------------------------------------

    /**
     * {@return one frame as its track line, e.g. {@code 12 | g1 left_stick_y=-1.0 a | g2}}
     *
     * @param frame the frame to write
     */
    public static String formatLine(Frame frame) {
        StringBuilder line = new StringBuilder();
        line.append(frame.tick).append(" | g1");
        appendState(line, frame.gamepad1);
        line.append(" | g2");
        appendState(line, frame.gamepad2);
        return line.toString();
    }

    /** {@return one gamepad's part of a line, without the {@code g1}/{@code g2} label} */
    static String formatState(GamepadState state) {
        StringBuilder text = new StringBuilder();
        appendState(text, state);
        return text.toString().trim();
    }

    private static void appendState(StringBuilder line, GamepadState state) {
        for (Map.Entry<String, Float> axis : state.nonZeroAxes().entrySet()) {
            line.append(' ').append(axis.getKey()).append('=').append(Float.toString(axis.getValue()));
        }
        for (String button : state.buttonsDown()) {
            line.append(' ').append(button);
        }
    }

    /**
     * Parse one frame line.
     *
     * @param line       the text, without its line ending
     * @param lineNumber where it came from, counting from 1, used only in error messages
     * @return the frame
     * @throws IllegalArgumentException saying what is wrong and on which line
     */
    public static Frame parseLine(String line, int lineNumber) {
        String[] parts = line.split("\\|", -1);
        if (parts.length != 3) {
            throw bad(lineNumber, line, "expected 'tick | g1 ... | g2 ...' (three parts separated by '|')");
        }
        int tick;
        try {
            tick = Integer.parseInt(parts[0].trim());
        } catch (NumberFormatException notANumber) {
            throw bad(lineNumber, line, "the tick '" + parts[0].trim() + "' is not a whole number");
        }
        GamepadState gamepad1 = parseState(parts[1], "g1", lineNumber, line);
        GamepadState gamepad2 = parseState(parts[2], "g2", lineNumber, line);
        try {
            return new Frame(tick, gamepad1, gamepad2);
        } catch (IllegalArgumentException invalid) {
            throw bad(lineNumber, line, invalid.getMessage());
        }
    }

    private static GamepadState parseState(String part, String label, int lineNumber, String line) {
        String[] tokens = part.trim().split("\\s+");
        if (tokens.length == 0 || !tokens[0].equals(label)) {
            throw bad(lineNumber, line, "expected the gamepad part to start with '" + label + "'");
        }
        Map<String, Float> axes = new TreeMap<>();
        TreeSet<String> buttons = new TreeSet<>();
        for (int i = 1; i < tokens.length; i++) {
            String token = tokens[i];
            int equals = token.indexOf('=');
            if (equals < 0) {
                if (!buttons.add(token)) {
                    throw bad(lineNumber, line, "button '" + token + "' is listed twice");
                }
                continue;
            }
            String name = token.substring(0, equals);
            float value;
            try {
                value = Float.parseFloat(token.substring(equals + 1));
            } catch (NumberFormatException notANumber) {
                throw bad(lineNumber, line, "the value of '" + name + "' is not a number");
            }
            if (axes.put(name, value) != null) {
                throw bad(lineNumber, line, "axis '" + name + "' is listed twice");
            }
        }
        try {
            return GamepadState.of(axes, buttons);
        } catch (IllegalArgumentException invalid) {
            throw bad(lineNumber, line, invalid.getMessage());
        }
    }

    private static IllegalArgumentException bad(int lineNumber, String line, String why) {
        return new IllegalArgumentException("gamepad input track line " + lineNumber + ": " + why + ": " + line);
    }

    // ---------------------------------------------------------------------------------------------
    // Files
    // ---------------------------------------------------------------------------------------------

    /**
     * Read a whole track file.
     *
     * @param path the file
     * @return the track
     * @throws UncheckedIOException     if the file cannot be read
     * @throws IllegalArgumentException if its first line is not {@link #FORMAT_LINE} (a byte order mark in
     *                                  front of it is allowed), if a metadata key is given twice, if any
     *                                  line does not parse, or if the ticks are not 0, 1, 2, ... with no gaps
     */
    public static GamepadInputTrack read(Path path) {
        List<String> lines;
        try {
            lines = Files.readAllLines(path, StandardCharsets.UTF_8);
        } catch (IOException e) {
            throw new UncheckedIOException("could not read gamepad input track " + path, e);
        }
        checkFormatLine(path, lines.isEmpty() ? "" : lines.get(0));
        Map<String, String> metadata = new LinkedHashMap<>();
        List<Frame> frames = new ArrayList<>();
        for (int i = 1; i < lines.size(); i++) {
            String line = lines.get(i);
            int lineNumber = i + 1;
            if (line.trim().isEmpty()) {
                continue;
            }
            if (line.startsWith("#")) {
                readMetadata(line, lineNumber, metadata);
                continue;
            }
            Frame frame = parseLine(line, lineNumber);
            if (frame.tick != frames.size()) {
                throw bad(lineNumber, line, "expected tick " + frames.size() + " next, found tick " + frame.tick
                        + " - a track must count every loop from 0 with no gaps");
            }
            frames.add(frame);
        }
        return new GamepadInputTrack(metadata, frames);
    }

    /**
     * Check a track's first line: {@link #FORMAT_LINE} exactly, after dropping a byte order mark and any
     * trailing spaces. A line that starts with {@link #FORMAT_PREFIX} but carries another number is a track
     * from a newer (or older) version of this code, and is refused by saying so rather than as "not a track".
     */
    private static void checkFormatLine(Path path, String firstLine) {
        String line = !firstLine.isEmpty() && firstLine.charAt(0) == BYTE_ORDER_MARK
                ? firstLine.substring(1) : firstLine;
        line = line.trim();
        if (line.equals(FORMAT_LINE)) {
            return;
        }
        if (line.startsWith(FORMAT_PREFIX)) {
            throw new IllegalArgumentException(path + " is a gamepad input track in format "
                    + line.substring(FORMAT_PREFIX.length()) + ", and this code reads only format " + FORMAT_NUMBER);
        }
        throw new IllegalArgumentException(path + " is not a gamepad input track: its first line must be '"
                + FORMAT_LINE + "'");
    }

    /**
     * A {@code # key: value} header line goes into {@code metadata} - the key one word, the space after the
     * colon optional, so a hand-written {@code # tickSeconds:0.02} counts too. Any other {@code #} line, one
     * whose text before its first colon has a space in it or that has no colon, is a comment. A key given
     * twice is refused: which of two alliances a replay should use is not something to guess.
     */
    private static void readMetadata(String line, int lineNumber, Map<String, String> metadata) {
        String body = line.substring(1).trim();
        int colon = body.indexOf(':');
        if (colon <= 0 || body.substring(0, colon).contains(" ")) {
            return;
        }
        String key = body.substring(0, colon);
        if (metadata.containsKey(key)) {
            throw bad(lineNumber, line, "the metadata key '" + key + "' was already given as '"
                    + metadata.get(key) + "'; each key may appear once");
        }
        metadata.put(key, body.substring(colon + 1).trim());
    }

    /**
     * Writes a track one line at a time, flushing after every line, so a session that ends abruptly - the
     * window closed, the process killed - still leaves every loop it ran on disk.
     */
    public static final class Writer implements Closeable {
        private final Path path;
        private final BufferedWriter out;
        private int nextTick = 0;

        /**
         * Create the file and write its header.
         *
         * <p>The metadata is checked BEFORE the file is opened, so a refusal leaves no half-written file
         * behind and no file handle open.
         *
         * @param path            where to write; parent directories are created
         * @param metadata        header entries written as {@code # key: value}, in iteration order; keys
         *                        must be single words and values must be one line
         * @param replaceExisting {@code true} to replace a file already at {@code path}; {@code false} to
         *                        refuse, so a recorded session is never overwritten by accident
         * @throws IllegalArgumentException if a metadata entry is not a one-word key and a one-line value
         * @throws IllegalStateException    if {@code replaceExisting} is false and {@code path} already exists
         * @throws UncheckedIOException     if the file cannot be created or written
         */
        public Writer(Path path, Map<String, String> metadata, boolean replaceExisting) {
            this.path = path;
            for (Map.Entry<String, String> entry : metadata.entrySet()) {
                String key = entry.getKey();
                String value = entry.getValue();
                if (key.isEmpty() || key.contains(" ") || key.contains(":")
                        || value.contains("\n") || value.contains("\r")) {
                    throw new IllegalArgumentException("metadata must be a one-word key and a one-line value, got '"
                            + key + "' = '" + value + "'");
                }
            }
            try {
                Path parent = path.toAbsolutePath().getParent();
                if (parent != null) {
                    Files.createDirectories(parent);
                }
                if (replaceExisting) {
                    out = Files.newBufferedWriter(path, StandardCharsets.UTF_8);
                } else {
                    // CREATE_NEW makes "the file is absent" and "create it" one step, so nothing can slip in between.
                    out = Files.newBufferedWriter(path, StandardCharsets.UTF_8, StandardOpenOption.CREATE_NEW,
                            StandardOpenOption.WRITE);
                }
            } catch (FileAlreadyExistsException exists) {
                throw new IllegalStateException(path + " already exists; refusing to overwrite it", exists);
            } catch (IOException e) {
                throw new UncheckedIOException("could not create gamepad input track " + path, e);
            }
            try {
                writeLine(FORMAT_LINE);
                writeLine(LAYOUT_LINE);
                for (Map.Entry<String, String> entry : metadata.entrySet()) {
                    writeLine("# " + entry.getKey() + ": " + entry.getValue());
                }
            } catch (IOException e) {
                closeAfterFailedWrite(e);
                throw new UncheckedIOException("could not write gamepad input track " + path, e);
            }
        }

        /**
         * Append one loop.
         *
         * @param frame the next frame; its tick must be exactly one more than the last one written
         */
        public void append(Frame frame) {
            if (frame.tick != nextTick) {
                throw new IllegalArgumentException("expected tick " + nextTick + " next, got " + frame.tick);
            }
            try {
                writeLine(formatLine(frame));
            } catch (IOException e) {
                throw new UncheckedIOException("could not write gamepad input track " + path, e);
            }
            nextTick++;
        }

        /** {@return how many loops have been written} */
        public int framesWritten() {
            return nextTick;
        }

        /** {@return the file being written} */
        public Path path() {
            return path;
        }

        /** Close the file after a write failed, keeping the write failure as the one that is reported. */
        private void closeAfterFailedWrite(IOException writeFailure) {
            try {
                out.close();
            } catch (IOException closeFailure) {
                writeFailure.addSuppressed(closeFailure);
            }
        }

        private void writeLine(String line) throws IOException {
            out.write(line);
            out.write('\n');
            out.flush();
        }

        @Override
        public void close() {
            try {
                out.close();
            } catch (IOException e) {
                throw new UncheckedIOException("could not close gamepad input track " + path, e);
            }
        }
    }
}
