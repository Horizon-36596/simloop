package org.horizon36596.simloop;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assertions.fail;

import java.io.IOException;
import java.io.UncheckedIOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;
import java.util.regex.Pattern;
import java.util.stream.Stream;

import org.junit.jupiter.api.Test;

/**
 * Pins domain R5 for SimLoop's main sources: <b>no wall-clock read anywhere except the one real-time
 * pacer</b>, which is the outer edge of a live session and cannot reach the sim (its own javadoc says
 * why). A clock read anywhere else would make a replay depend on how fast the machine ran.
 *
 * <p>INVARIANT tier. Built the same way as {@link SeparabilityTest}: a line scan of {@code src/main/java}
 * that skips comment lines, so a javadoc that names {@code System.nanoTime()} is not a violation.
 */
class WallClockConfinementTest {

    /**
     * The only file allowed to read a wall clock, relative to {@code src/main/java}. This exemption is the
     * whole allowance: every other main-source file is scanned.
     */
    private static final Path THE_ONE_PACER =
            Paths.get("org", "horizon36596", "simloop", "sim", "live", "RealTimePacer.java");

    /** Text that means a wall-clock read. Each one is pinned by a sample line in the second test below. */
    private static final String[] WALL_CLOCK_READS = {
            "System.nanoTime(",
            "System.currentTimeMillis(",
            "System::nanoTime",          // a method reference reads the clock the same way, just later
            "System::currentTimeMillis",
            "Instant.now(",
            "LocalDateTime.now(",
            "LocalTime.now(",
            "ZonedDateTime.now(",
            "OffsetDateTime.now(",
            "Clock.system",
            "new Date(",
            "new java.util.Date(",
            "Calendar.getInstance(",
            "SystemClock.",              // android.os.SystemClock
            "NanoClock.system",          // the FTC SDK's clock, the same
    };

    /**
     * Building the FTC SDK's timer, which runs on {@code System.nanoTime()}: {@code new ElapsedTime(},
     * with any spacing, a package-qualified name, or the constructor reference {@code ElapsedTime::new}.
     * Only naming the type (an import, {@code extends}, a field type) is not a read.
     */
    private static final Pattern ELAPSED_TIME_CONSTRUCTION =
            Pattern.compile("new\\s+([\\w.]*\\.)?ElapsedTime\\s*\\(|ElapsedTime\\s*::\\s*new");

    /** One line a careless edit could add, per pattern above, each of which must be caught. */
    private static final String[] SAMPLE_VIOLATIONS = {
            "long now = System.nanoTime();",
            "long now = System.currentTimeMillis();",
            "LongSupplier clock = System::nanoTime;",
            "LongSupplier clock = System::currentTimeMillis;",
            "Instant stamp = Instant.now();",
            "LocalDateTime stamp = LocalDateTime.now();",
            "LocalTime stamp = LocalTime.now();",
            "ZonedDateTime stamp = ZonedDateTime.now();",
            "OffsetDateTime stamp = OffsetDateTime.now();",
            "Clock clock = Clock.systemUTC();",
            "Date stamp = new Date();",
            "java.util.Date stamp = new java.util.Date();",
            "Calendar stamp = Calendar.getInstance();",
            "long now = SystemClock.uptimeMillis();",
            "private final ElapsedTime timer = new ElapsedTime();",
            "private final ElapsedTime timer = new ElapsedTime ();",
            "private final ElapsedTime timer = new com.qualcomm.robotcore.util.ElapsedTime();",
            "Supplier<ElapsedTime> timers = ElapsedTime::new;",
            "NanoClock clock = NanoClock.system();",
    };

    @Test
    void invariant_onlyTheRealTimePacerReadsAWallClock() {
        Path sourceRoot = locateMainSourceRoot();
        List<String> violations = new ArrayList<>();
        int[] scanned = {0};

        try (Stream<Path> files = Files.walk(sourceRoot)) {
            files.filter(p -> p.toString().endsWith(".java")).forEach(p -> {
                scanned[0]++;
                if (sourceRoot.relativize(p).equals(THE_ONE_PACER)) {
                    return;
                }
                try {
                    for (String line : Files.readAllLines(p)) {
                        String trimmed = line.trim();
                        if (trimmed.startsWith("//") || trimmed.startsWith("*") || trimmed.startsWith("/*")) {
                            continue;
                        }
                        if (readsAWallClock(withoutStringLiterals(line))) {
                            violations.add(sourceRoot.relativize(p) + ": " + trimmed);
                        }
                    }
                } catch (IOException e) {
                    throw new UncheckedIOException(e);
                }
            });
        } catch (IOException e) {
            throw new UncheckedIOException(e);
        }

        assertTrue(scanned[0] > 0, "found no .java files under " + sourceRoot + " - the scan would pass vacuously");
        assertTrue(Files.isRegularFile(sourceRoot.resolve(THE_ONE_PACER)),
                "the allowed pacer moved; update THE_ONE_PACER rather than widening the allowance");
        if (!violations.isEmpty()) {
            fail("SimLoop main sources must not read a wall clock outside RealTimePacer (domain R5):\n  "
                    + String.join("\n  ", violations));
        }
    }

    @Test
    void everyKindOfWallClockReadIsCaught() {
        for (String sample : SAMPLE_VIOLATIONS) {
            assertTrue(readsAWallClock(sample), "the scan must catch: " + sample);
        }
        assertFalse(readsAWallClock("double now = timer.time();"), "reading the sim's FakeTimer is not a wall clock");
        assertFalse(readsAWallClock("public class SimElapsedTime extends ElapsedTime {"),
                "naming the ElapsedTime type is not a clock read");
    }

    /**
     * {@return the line with every string literal emptied}. A user-facing message that tells a student to
     * avoid {@code System.nanoTime()} is text, not a clock read.
     */
    private static String withoutStringLiterals(String line) {
        return line.replaceAll("\"(?:[^\"\\\\]|\\\\.)*\"", "\"\"");
    }

    /** {@return whether one source line contains any of {@link #WALL_CLOCK_READS} or builds an ElapsedTime} */
    private static boolean readsAWallClock(String line) {
        if (ELAPSED_TIME_CONSTRUCTION.matcher(line).find()) {
            return true;
        }
        for (String read : WALL_CLOCK_READS) {
            if (line.contains(read)) {
                return true;
            }
        }
        return false;
    }

    /** Same resolution as {@link SeparabilityTest}: module dir first, repo root as a fallback. */
    private static Path locateMainSourceRoot() {
        Path[] candidates = {
                Paths.get("src", "main", "java"),
                Paths.get("SimLoop", "src", "main", "java"),
        };
        for (Path candidate : candidates) {
            if (Files.isDirectory(candidate)) {
                return candidate.toAbsolutePath();
            }
        }
        throw new AssertionError("could not locate SimLoop src/main/java from working dir "
                + Paths.get("").toAbsolutePath());
    }
}
