package org.horizon36596.simloop.opmode;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Locale;

/**
 * A {@link Telemetry} that keeps what the Driver Station would have shown, so a test can read it back.
 *
 * <p>Every {@link #update()} renders the current items and lines to text, one string per Driver Station
 * line, and appends that list to {@link #transmissions()}. The rendering follows the SDK's defaults: an item
 * reads {@code "caption : value"}, the items on one {@link Line} are joined with {@code " | "}, and with
 * auto-clear on (the default) every item that is not retained is removed when the next item or line is
 * added after an update - not at the update itself, which is when the SDK's own telemetry clears too. So an
 * update with nothing new added shows the same lines again, as the Driver Station would: an iterative
 * OpMode that calls {@code telemetry.update()} itself, followed by the SDK's own update after
 * {@code loop()}, still ends on what it added. Lines added through {@link #log()} go to
 * {@link #loggedLines()}.
 *
 * <p><b>What it deliberately does not do.</b> Nothing is sent anywhere, {@link #speak(String)} is silent, and
 * the transmission interval is stored but not honoured: every update is recorded, because a sim tick is not
 * a real 250 ms. Formatted values ({@code addData(caption, "%.2f", x)}) use {@link Locale#ROOT}, so a run
 * reads the same on a laptop set to any language; the robot uses its own default locale.
 *
 * <p>Not thread-safe. The runner only lets one thread touch it at a time, and a test reads it after the run.
 */
public final class RecordingTelemetry implements Telemetry {

    /** The SDK's default gap between items on one line. */
    private static final String DEFAULT_ITEM_SEPARATOR = " | ";

    /** The SDK's default gap between a caption and its value. */
    private static final String DEFAULT_CAPTION_VALUE_SEPARATOR = " : ";

    /** The SDK's default {@link Telemetry#getMsTransmissionInterval()}, in milliseconds. */
    private static final int DEFAULT_TRANSMISSION_INTERVAL_MS = 250;

    /** Top-level items and lines, in the order they were added; one entry is one Driver Station line. */
    private final List<Object> entries = new ArrayList<>();

    private final List<Runnable> actions = new ArrayList<>();
    private final List<List<String>> transmissions = new ArrayList<>();
    private final RecordingLog log = new RecordingLog();

    private boolean autoClear = true;

    /** Set by an auto-clearing {@link #update()}: the next add clears first, as the SDK's clearOnAdd does. */
    private boolean clearOnAdd;
    private int msTransmissionInterval = DEFAULT_TRANSMISSION_INTERVAL_MS;
    private String itemSeparator = DEFAULT_ITEM_SEPARATOR;
    private String captionValueSeparator = DEFAULT_CAPTION_VALUE_SEPARATOR;

    /** Creates an empty telemetry with the SDK's default separators and auto-clear on. */
    public RecordingTelemetry() {
    }

    /**
     * {@return every update so far, oldest first: each one is the list of Driver Station lines it showed}
     * The list is a snapshot; later updates do not change it.
     */
    public List<List<String>> transmissions() {
        return Collections.unmodifiableList(new ArrayList<>(transmissions));
    }

    /** {@return the lines shown by the most recent {@link #update()}, or an empty list if none has happened} */
    public List<String> lastTransmission() {
        return transmissions.isEmpty() ? Collections.<String>emptyList()
                : transmissions.get(transmissions.size() - 1);
    }

    /** {@return every line added through {@link #log()}, oldest first, however many the log's capacity shows} */
    public List<String> loggedLines() {
        return Collections.unmodifiableList(new ArrayList<>(log.lines));
    }

    // ---- Telemetry ---------------------------------------------------------------------------------

    @Override
    public Item addData(String caption, String format, Object... args) {
        return addItem(entries, caption, formatted(format, args));
    }

    @Override
    public Item addData(String caption, Object value) {
        return addItem(entries, caption, ValueSource.of(value));
    }

    @Override
    public <T> Item addData(String caption, Func<T> valueProducer) {
        return addItem(entries, caption, ValueSource.of(valueProducer));
    }

    @Override
    public <T> Item addData(String caption, String format, Func<T> valueProducer) {
        return addItem(entries, caption, ValueSource.of(format, valueProducer));
    }

    @Override
    public boolean removeItem(Item item) {
        if (entries.remove(item)) {
            return true;
        }
        for (Object entry : entries) {
            if (entry instanceof RecordingLine && ((RecordingLine) entry).items.remove(item)) {
                return true;
            }
        }
        return false;
    }

    /** Removes every item that is not retained, and every line left with no retained item. */
    @Override
    public void clear() {
        clearOnAdd = false;
        List<Object> kept = new ArrayList<>();
        for (Object entry : entries) {
            if (entry instanceof RecordingItem && ((RecordingItem) entry).isRetained()) {
                kept.add(entry);
            } else if (entry instanceof RecordingLine) {
                RecordingLine line = (RecordingLine) entry;
                line.items.removeIf(item -> !((RecordingItem) item).isRetained());
                if (!line.items.isEmpty()) {
                    kept.add(line);
                }
            }
        }
        entries.clear();
        entries.addAll(kept);
    }

    /** Removes every item and line, retained or not, every action, and the log. */
    @Override
    public void clearAll() {
        clearOnAdd = false;
        entries.clear();
        actions.clear();
        log.clear();
    }

    @Override
    public Object addAction(Runnable action) {
        actions.add(action);
        return action;
    }

    @Override
    public boolean removeAction(Object token) {
        return actions.remove(token);
    }

    /** Does nothing: there is no Driver Station to speak through. */
    @Override
    public void speak(String text) {
    }

    /** Does nothing: there is no Driver Station to speak through. */
    @Override
    public void speak(String text, String languageCode, String countryCode) {
    }

    /**
     * Runs the actions, renders every item and line to text and records that as one transmission. With
     * auto-clear on, the next item or line added clears the non-retained ones first.
     *
     * @return true, as the SDK does when it sends
     */
    @Override
    public boolean update() {
        for (Runnable action : new ArrayList<>(actions)) {
            action.run();
        }
        List<String> shown = new ArrayList<>();
        for (Object entry : entries) {
            shown.add(entry instanceof RecordingLine ? ((RecordingLine) entry).render()
                    : ((RecordingItem) entry).render());
        }
        transmissions.add(Collections.unmodifiableList(shown));
        clearOnAdd = autoClear;
        return true;
    }

    @Override
    public Line addLine() {
        return addLine("");
    }

    @Override
    public Line addLine(String lineCaption) {
        onAdd();
        RecordingLine line = new RecordingLine(lineCaption);
        entries.add(line);
        return line;
    }

    @Override
    public boolean removeLine(Line line) {
        return entries.remove(line);
    }

    @Override
    public boolean isAutoClear() {
        return autoClear;
    }

    @Override
    public void setAutoClear(boolean autoClear) {
        this.autoClear = autoClear;
    }

    /** {@return the stored interval, in milliseconds; it is not honoured, see the class javadoc} */
    @Override
    public int getMsTransmissionInterval() {
        return msTransmissionInterval;
    }

    /** Stores the interval, in milliseconds; it is not honoured, see the class javadoc. */
    @Override
    public void setMsTransmissionInterval(int msTransmissionInterval) {
        this.msTransmissionInterval = msTransmissionInterval;
    }

    @Override
    public String getItemSeparator() {
        return itemSeparator;
    }

    @Override
    public void setItemSeparator(String itemSeparator) {
        this.itemSeparator = itemSeparator;
    }

    @Override
    public String getCaptionValueSeparator() {
        return captionValueSeparator;
    }

    @Override
    public void setCaptionValueSeparator(String captionValueSeparator) {
        this.captionValueSeparator = captionValueSeparator;
    }

    /** Accepted and ignored: the recorded text is the same in every display format. */
    @Override
    public void setDisplayFormat(DisplayFormat displayFormat) {
    }

    @Override
    public Log log() {
        return log;
    }

    // ---- internals ---------------------------------------------------------------------------------

    private Item addItem(List<Object> container, String caption, ValueSource value) {
        onAdd();
        RecordingItem item = new RecordingItem(container, caption, value);
        container.add(item);
        return item;
    }

    /** The SDK's {@code onAddData()}: the first add after an auto-clearing update clears the old items. */
    private void onAdd() {
        if (clearOnAdd) {
            clear();
        }
    }

    private static ValueSource formatted(String format, Object... args) {
        String text = String.format(Locale.ROOT, format, args);
        return () -> text;
    }

    /** Where an item's value comes from: fixed when added, or read at each update like the SDK's Func. */
    private interface ValueSource {
        String read();

        static ValueSource of(Object value) {
            return () -> String.valueOf(value);
        }

        static <T> ValueSource of(Func<T> producer) {
            return () -> String.valueOf(producer.value());
        }

        static <T> ValueSource of(String format, Func<T> producer) {
            return () -> String.format(Locale.ROOT, format, producer.value());
        }
    }

    /** One caption-and-value item, living either at the top level or on a {@link RecordingLine}. */
    private final class RecordingItem implements Item {
        private final List<Object> container;
        private String caption;
        private ValueSource value;
        private boolean retained;

        RecordingItem(List<Object> container, String caption, ValueSource value) {
            this.container = container;
            this.caption = caption;
            this.value = value;
        }

        String render() {
            return caption + captionValueSeparator + value.read();
        }

        @Override public String getCaption() { return caption; }
        @Override public Item setCaption(String caption) { this.caption = caption; return this; }
        @Override public Item setValue(String format, Object... args) { value = formatted(format, args); return this; }
        @Override public Item setValue(Object value) { this.value = ValueSource.of(value); return this; }
        @Override public <T> Item setValue(Func<T> producer) { value = ValueSource.of(producer); return this; }
        @Override public <T> Item setValue(String format, Func<T> producer) {
            value = ValueSource.of(format, producer);
            return this;
        }
        @Override public Item setRetained(Boolean retained) { this.retained = Boolean.TRUE.equals(retained); return this; }
        @Override public boolean isRetained() { return retained; }
        @Override public Item addData(String caption, String format, Object... args) {
            return addItem(container, caption, formatted(format, args));
        }
        @Override public Item addData(String caption, Object value) {
            return addItem(container, caption, ValueSource.of(value));
        }
        @Override public <T> Item addData(String caption, Func<T> producer) {
            return addItem(container, caption, ValueSource.of(producer));
        }
        @Override public <T> Item addData(String caption, String format, Func<T> producer) {
            return addItem(container, caption, ValueSource.of(format, producer));
        }
    }

    /** A line: an optional caption followed by its items, all on one Driver Station row. */
    private final class RecordingLine implements Line {
        private final String caption;
        private final List<Object> items = new ArrayList<>();

        RecordingLine(String caption) {
            this.caption = caption;
        }

        String render() {
            StringBuilder text = new StringBuilder(caption);
            for (int i = 0; i < items.size(); i++) {
                text.append(i == 0 ? "" : itemSeparator).append(((RecordingItem) items.get(i)).render());
            }
            return text.toString();
        }

        @Override public Item addData(String caption, String format, Object... args) {
            return addItem(items, caption, formatted(format, args));
        }
        @Override public Item addData(String caption, Object value) {
            return addItem(items, caption, ValueSource.of(value));
        }
        @Override public <T> Item addData(String caption, Func<T> producer) {
            return addItem(items, caption, ValueSource.of(producer));
        }
        @Override public <T> Item addData(String caption, String format, Func<T> producer) {
            return addItem(items, caption, ValueSource.of(format, producer));
        }
    }

    /** The SDK's scrolling log. Every line is kept, whatever the capacity says. */
    private static final class RecordingLog implements Log {
        /** The SDK's default number of log lines shown. */
        private static final int DEFAULT_CAPACITY = 9;

        private final List<String> lines = new ArrayList<>();
        private int capacity = DEFAULT_CAPACITY;
        private DisplayOrder displayOrder = DisplayOrder.OLDEST_FIRST;

        @Override public int getCapacity() { return capacity; }
        @Override public void setCapacity(int capacity) { this.capacity = capacity; }
        @Override public DisplayOrder getDisplayOrder() { return displayOrder; }
        @Override public void setDisplayOrder(DisplayOrder displayOrder) { this.displayOrder = displayOrder; }
        @Override public void add(String entry) { lines.add(entry); }
        @Override public void add(String format, Object... args) { lines.add(String.format(Locale.ROOT, format, args)); }
        @Override public void clear() { lines.clear(); }
    }
}
