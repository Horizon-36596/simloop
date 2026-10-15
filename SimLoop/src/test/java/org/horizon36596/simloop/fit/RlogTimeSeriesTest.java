package org.horizon36596.simloop.fit;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;
import org.psilynx.psikit.core.LogTable;
import org.psilynx.psikit.core.rlog.RLOGWriter;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.NoSuchFileException;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.Arrays;
import java.util.Collections;

/**
 * Pins that a fit reads what the log says, at the time the log says it. INVARIANT tier. Writes with
 * PsiKit's own {@code RLOGWriter} into {@code build/sim}, for the reason {@code LoopTestRlogs} gives.
 */
class RlogTimeSeriesTest {

    private static final Path DIR = Paths.get("build", "sim");

    @Test
    void invariant_valuesAndFrameTimesReadBackAsWritten() throws IOException {
        Path rlog = write("fit-series-roundtrip", 6, true);

        RlogTimeSeries series = RlogTimeSeries.read(rlog, Arrays.asList("RealOutputs/Test/power", "RealOutputs/Test/speed"));

        assertEquals(6, series.size());
        assertArrayEquals(new double[] {0.0, 0.02, 0.04, 0.06, 0.08, 0.10}, series.timeSeconds(), 1e-9,
                "frame times are the log's own timestamps, in seconds");
        assertArrayEquals(new double[] {0.0, 0.1, 0.2, 0.3, 0.4, 0.5}, series.column("RealOutputs/Test/power"), 1e-12);
        assertArrayEquals(new double[] {0.0, 10.0, 20.0, 30.0, 40.0, 50.0}, series.column("RealOutputs/Test/speed"), 1e-12);
    }

    @Test
    void aKeyTheLogNeverHadIsReportedNotSilentlyEmpty() throws IOException {
        Path rlog = write("fit-series-missing", 3, false);

        assertFalse(RlogTimeSeries.hasKey(rlog, "RealOutputs/Test/speed"));
        assertTrue(RlogTimeSeries.hasKey(rlog, "RealOutputs/Test/power"));
        IllegalArgumentException refused = assertThrows(IllegalArgumentException.class,
                () -> RlogTimeSeries.read(rlog, Collections.singletonList("RealOutputs/Test/speed")));
        assertTrue(refused.getMessage().contains("RealOutputs/Test/speed"), refused.getMessage());
    }

    @Test
    void aMissingFileIsAMissingFile() {
        assertThrows(NoSuchFileException.class,
                () -> RlogTimeSeries.read(DIR.resolve("fit-series-does-not-exist.rlog"),
                        Collections.singletonList("RealOutputs/Test/power")));
    }

    private static Path write(String name, int frames, boolean withSpeed) throws IOException {
        Files.createDirectories(DIR);
        RLOGWriter writer = new RLOGWriter(DIR.toString(), name);
        writer.start();
        for (int i = 0; i < frames; i++) {
            LogTable table = new LogTable(i * 0.02);
            table.put("RealOutputs/Test/power", i * 0.1);
            if (withSpeed) {
                table.put("RealOutputs/Test/speed", i * 10.0);
            }
            writer.putTable(table);
        }
        writer.end();
        return DIR.resolve(name + ".rlog");
    }
}
