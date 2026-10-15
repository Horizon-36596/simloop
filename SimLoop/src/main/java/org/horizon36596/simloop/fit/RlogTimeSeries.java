package org.horizon36596.simloop.fit;

import org.psilynx.psikit.core.LogTable;
import org.psilynx.psikit.core.rlog.RLOGReplay;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.NoSuchFileException;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/**
 * The numeric columns of an RLOG, with each frame's time: what a fit reads. Horizon (FTC 36596) SimLoop.
 *
 * <p>Decoded with PsiKit's own {@code RLOGReplay}, the way {@code sim.RlogDecodedCompare} decodes, so a
 * fit reads the same values a replay check compares. Each frame carries the latest value of every key logged
 * so far, because an RLOG stores a key only when it changes and the replay table accumulates them.
 *
 * <p>A frame is kept only once every requested key has appeared in it, so every column has the same
 * length and row {@code i} of every column is the same moment. A value that does not parse as a number (a
 * string, a boolean, an array) is refused, naming the key.
 */
public final class RlogTimeSeries {

    private final double[] timeSeconds;
    private final Map<String, double[]> columns;

    private RlogTimeSeries(double[] timeSeconds, Map<String, double[]> columns) {
        this.timeSeconds = timeSeconds;
        this.columns = Collections.unmodifiableMap(columns);
    }

    /**
     * A series from values already in memory - how a test builds one without writing a file.
     *
     * @param timeSeconds frame times, seconds
     * @param columns     key to one value per frame
     * @return the series
     */
    public static RlogTimeSeries of(double[] timeSeconds, Map<String, double[]> columns) {
        Map<String, double[]> copy = new LinkedHashMap<>();
        for (Map.Entry<String, double[]> column : columns.entrySet()) {
            if (column.getValue().length != timeSeconds.length) {
                throw new IllegalArgumentException("column " + column.getKey() + " has " + column.getValue().length
                        + " values for " + timeSeconds.length + " frames");
            }
            copy.put(column.getKey(), column.getValue().clone());
        }
        return new RlogTimeSeries(timeSeconds.clone(), copy);
    }

    /**
     * Read the given keys from an RLOG.
     *
     * @param rlog the file
     * @param keys the keys to read, spelled as they decode - e.g. {@code RealOutputs/Drive/FL/output}
     * @return one row per frame in which every key has a value, in file order
     * @throws IOException if the file is missing or unreadable
     * @throws IllegalArgumentException if a key's value is not a number, or no frame has every key
     */
    public static RlogTimeSeries read(Path rlog, Collection<String> keys) throws IOException {
        if (!Files.isRegularFile(rlog)) {
            // RLOGReplay decodes a missing file as zero frames, which would read as "no data" instead of
            // "wrong path" - the same trap RlogDecodedCompare guards.
            throw new NoSuchFileException(rlog.toString());
        }
        List<Double> times = new ArrayList<>();
        Map<String, List<Double>> values = new LinkedHashMap<>();
        for (String key : keys) {
            values.put(key, new ArrayList<Double>());
        }

        RLOGReplay replay = new RLOGReplay(rlog.toString());
        replay.start();
        LogTable table = new LogTable(0);
        while (replay.updateTable(table)) {
            Map<String, LogTable.LogValue> all = table.getAll(true);
            boolean complete = true;
            for (String key : keys) {
                if (!all.containsKey(key)) {
                    complete = false;
                    break;
                }
            }
            if (!complete) {
                continue;
            }
            // The one PsiKit read this class depends on beyond RlogDecodedCompare's: the frame's own time.
            // PsiKit's Logger stamps each frame with System.nanoTime() / 1e9, so this is already SECONDS (confirmed
            // by decompiling the PsiKit jar during the LogReview review). No unit conversion belongs here.
            times.add((double) table.getTimestamp());
            for (String key : keys) {
                values.get(key).add(parse(key, String.valueOf(all.get(key))));
            }
        }
        if (times.isEmpty()) {
            throw new IllegalArgumentException(rlog + " has no frame carrying all of " + keys
                    + " - check the key names against the log (they include the RealOutputs/ prefix)");
        }

        double[] time = new double[times.size()];
        for (int i = 0; i < time.length; i++) {
            time[i] = times.get(i);
        }
        Map<String, double[]> columns = new LinkedHashMap<>();
        for (Map.Entry<String, List<Double>> column : values.entrySet()) {
            double[] data = new double[time.length];
            for (int i = 0; i < data.length; i++) {
                data[i] = column.getValue().get(i);
            }
            columns.put(column.getKey(), data);
        }
        return new RlogTimeSeries(time, columns);
    }

    /**
     * Whether an RLOG has a key at all, so an optional column (a battery voltage the robot may not log)
     * can be looked for before it is asked for.
     *
     * @param rlog the file
     * @param key  the key, as it decodes
     * @return true if any frame carries it
     * @throws IOException if the file is missing or unreadable
     */
    public static boolean hasKey(Path rlog, String key) throws IOException {
        if (!Files.isRegularFile(rlog)) {
            throw new NoSuchFileException(rlog.toString());
        }
        RLOGReplay replay = new RLOGReplay(rlog.toString());
        replay.start();
        LogTable table = new LogTable(0);
        while (replay.updateTable(table)) {
            if (table.getAll(true).containsKey(key)) {
                return true;
            }
        }
        return false;
    }

    /** {@return how many frames} */
    public int size() {
        return timeSeconds.length;
    }

    /** {@return a copy of the frame times, seconds} */
    public double[] timeSeconds() {
        return timeSeconds.clone();
    }

    /**
     * {@return a copy of one column}
     *
     * @param key the log key the column was read from
     * @throws IllegalArgumentException if the key was not read
     */
    public double[] column(String key) {
        double[] column = columns.get(key);
        if (column == null) {
            throw new IllegalArgumentException("no column " + key + "; read " + columns.keySet());
        }
        return column.clone();
    }

    private static double parse(String key, String text) {
        try {
            return Double.parseDouble(text);
        } catch (NumberFormatException notANumber) {
            throw new IllegalArgumentException(key + " decodes as '" + text + "', which is not a number", notANumber);
        }
    }
}
