package org.horizon36596.simloop.opmode;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.horizon36596.simloop.fakehardware.FakeHardwareMap;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

/**
 * Pins what happens to telemetry an OpMode copies before INIT - in a field initializer or its constructor,
 * when {@code telemetry} is still the SDK's own object: every copy the runner can reach is pointed at the
 * recording telemetry, and every copy it cannot is reported rather than silently dropped.
 *
 * <p>Invariant tier: "what the Driver Station would have shown" must not depend on where in the OpMode the
 * team happened to copy {@code telemetry}.
 */
class TelemetryCopiedBeforeInitTest {

    /**
     * A stand-in for FTC Dashboard's {@code MultipleTelemetry}, written the same way: a list of telemetries
     * built from varargs, a combined log built from their logs, and every call forwarded to each one. The real
     * one cannot be used here (its repository is unreachable from CI), and SimLoop must not depend on it.
     */
    static final class MultipleTelemetry implements Telemetry {
        private final List<Telemetry> telemetryList;
        private final MultipleLog log;

        MultipleTelemetry(Telemetry... telemetries) {
            this(new ArrayList<>(Arrays.asList(telemetries)));
        }

        MultipleTelemetry(List<Telemetry> telemetries) {
            this.telemetryList = telemetries;
            List<Log> logs = new ArrayList<>();
            for (Telemetry telemetry : telemetries) {
                logs.add(telemetry.log());
            }
            this.log = new MultipleLog(logs);
        }

        @Override public Item addData(String caption, String format, Object... args) {
            Item last = null;
            for (Telemetry t : telemetryList) {
                last = t.addData(caption, format, args);
            }
            return last;
        }
        @Override public Item addData(String caption, Object value) {
            Item last = null;
            for (Telemetry t : telemetryList) {
                last = t.addData(caption, value);
            }
            return last;
        }
        @Override public <T> Item addData(String caption, Func<T> valueProducer) {
            return addData(caption, (Object) valueProducer.value());
        }
        @Override public <T> Item addData(String caption, String format, Func<T> valueProducer) {
            return addData(caption, format, valueProducer.value());
        }
        @Override public boolean removeItem(Item item) { return false; }
        @Override public void clear() { telemetryList.forEach(Telemetry::clear); }
        @Override public void clearAll() { telemetryList.forEach(Telemetry::clearAll); }
        @Override public Object addAction(Runnable action) { return null; }
        @Override public boolean removeAction(Object token) { return false; }
        @Override public void speak(String text) { }
        @Override public void speak(String text, String languageCode, String countryCode) { }
        @Override public boolean update() {
            boolean sent = true;
            for (Telemetry t : telemetryList) {
                sent &= t.update();
            }
            return sent;
        }
        @Override public Line addLine() { return null; }
        @Override public Line addLine(String lineCaption) { return null; }
        @Override public boolean removeLine(Line line) { return false; }
        @Override public boolean isAutoClear() { return true; }
        @Override public void setAutoClear(boolean autoClear) { }
        @Override public int getMsTransmissionInterval() { return 250; }
        @Override public void setMsTransmissionInterval(int msTransmissionInterval) { }
        @Override public String getItemSeparator() { return " | "; }
        @Override public void setItemSeparator(String itemSeparator) { }
        @Override public String getCaptionValueSeparator() { return " : "; }
        @Override public void setCaptionValueSeparator(String captionValueSeparator) { }
        @Override public void setDisplayFormat(DisplayFormat displayFormat) { }
        @Override public Log log() { return log; }
    }

    /** Dashboard's {@code MultipleLog}: forwards every entry to each log it was built from. */
    static final class MultipleLog implements Telemetry.Log {
        private final List<Telemetry.Log> logs;

        MultipleLog(List<Telemetry.Log> logs) {
            this.logs = logs;
        }

        @Override public int getCapacity() { return 9; }
        @Override public void setCapacity(int capacity) { }
        @Override public DisplayOrder getDisplayOrder() { return DisplayOrder.OLDEST_FIRST; }
        @Override public void setDisplayOrder(DisplayOrder displayOrder) { }
        @Override public void add(String entry) { logs.forEach(l -> l.add(entry)); }
        @Override public void add(String format, Object... args) { logs.forEach(l -> l.add(format, args)); }
        @Override public void clear() { logs.forEach(Telemetry.Log::clear); }
    }

    private static OpModeRunResult run(OpMode opMode, int activeTicks) {
        return new OpModeRunner(opMode, new FakeHardwareMap()).activeTicks(activeTicks).run();
    }

    /** {@return every non-empty line the OpMode's telemetry showed, in order} */
    private static List<String> shown(OpModeRunResult result) {
        List<String> lines = new ArrayList<>();
        for (List<String> transmission : result.telemetry().transmissions()) {
            lines.addAll(transmission);
        }
        return lines;
    }

    // ---- what is rewired ----------------------------------------------------------------------------

    /** The plainest copy: a field. */
    static final class FieldCopy extends OpMode {
        private final Telemetry copy = telemetry;
        final Telemetry sdkTelemetryAtConstruction = copy;

        @Override public void init() { copy.addData("status", "ready"); }
        @Override public void loop() { copy.addData("loop", "running"); }
    }

    @Test
    void aTelemetryCopiedInAFieldInitializerIsRecorded() {
        FieldCopy opMode = new FieldCopy();
        OpModeRunResult result = run(opMode, 2);
        assertEquals(Arrays.asList("status : ready", "status : ready", "loop : running", "loop : running", "loop : running"),
                shown(result));
        assertSame(result.telemetry(), opMode.copy, "the final field now holds the recording telemetry");
        assertEquals(Collections.emptyList(), result.warnings());
    }

    /** The FTC Dashboard idiom, as a field: the SDK's telemetry inside a wrapper, next to the dashboard's. */
    static final class DashboardTeleOp extends OpMode {
        final RecordingTelemetry dashboard = new RecordingTelemetry(); // stands in for dashboard.getTelemetry()
        final Telemetry both = new MultipleTelemetry(telemetry, dashboard);

        @Override public void init() {
            both.log().add("init");
        }
        @Override public void loop() {
            both.addData("x", 1);
            both.update();
        }
    }

    @Test
    void aDashboardStyleMultipleTelemetryFieldReachesTheRecordingTelemetryAndItsLog() {
        DashboardTeleOp opMode = new DashboardTeleOp();
        OpModeRunResult result = run(opMode, 1);
        assertTrue(shown(result).contains("x : 1"), shown(result).toString());
        assertEquals(Collections.singletonList("init"), result.telemetry().loggedLines(),
                "the wrapper's log went to the recording telemetry's log, not the SDK's");
        assertEquals(Collections.singletonList(Collections.singletonList("x : 1")),
                opMode.dashboard.transmissions(), "the dashboard side still gets its copy");
        assertEquals(Collections.emptyList(), result.warnings());
    }

    /** The same wrapper built from a fixed-size {@code Arrays.asList}, which still allows {@code set}. */
    static final class FixedSizeList extends OpMode {
        final Telemetry both = new MultipleTelemetry(Arrays.asList(telemetry, new RecordingTelemetry()));

        @Override public void init() { both.addData("list", "fixed"); }
        @Override public void loop() { }
    }

    @Test
    void aCopyInAFixedSizeListIsRewired() {
        OpModeRunResult result = run(new FixedSizeList(), 0);
        assertEquals("list : fixed", shown(result).get(0));
        assertEquals(Collections.emptyList(), result.warnings());
    }

    /** The wrapper installed as the OpMode's own {@code telemetry}, in its constructor. */
    static final class WrapperAsTelemetry extends OpMode {
        final RecordingTelemetry dashboard = new RecordingTelemetry();
        final Telemetry installed;

        WrapperAsTelemetry() {
            telemetry = new MultipleTelemetry(telemetry, dashboard);
            installed = telemetry;
        }

        @Override public void init() { telemetry.addData("via", "wrapper"); }
        @Override public void loop() { }
    }

    @Test
    void aWrapperTheConstructorInstalledAsTelemetryStaysAndItsSdkTelemetryIsRewired() {
        WrapperAsTelemetry opMode = new WrapperAsTelemetry();
        OpModeRunResult result = run(opMode, 0);
        assertSame(opMode.installed, opMode.telemetry, "the OpMode keeps the object it chose, as on the robot");
        assertEquals("via : wrapper", shown(result).get(0));
        assertEquals("via : wrapper", opMode.dashboard.transmissions().get(0).get(0),
                "the runner's own update after init() went through the wrapper to both sides");
        assertEquals(Collections.emptyList(), result.warnings());
    }

    /** A team's helper class that keeps the telemetry it was built with, in a list and a map as well. */
    static final class Reporter {
        private final Telemetry telemetry;
        private final List<Telemetry> sinks = new ArrayList<>();
        private final Map<String, Telemetry> byName = new HashMap<>();
        private final Telemetry[] asArray;

        Reporter(Telemetry telemetry) {
            this.telemetry = telemetry;
            sinks.add(telemetry);
            byName.put("ds", telemetry);
            asArray = new Telemetry[] {telemetry};
        }

        void report(String what) {
            telemetry.addData("field", what);
            sinks.get(0).addData("list", what);
            byName.get("ds").addData("map", what);
            asArray[0].addData("array", what);
        }
    }

    static final class HelperTeleOp extends OpMode {
        final Reporter reporter = new Reporter(telemetry);

        @Override public void init() { reporter.report("ok"); }
        @Override public void loop() { }
    }

    @Test
    void aHelperObjectsFieldsListsMapsAndArraysAreRewired() {
        OpModeRunResult result = run(new HelperTeleOp(), 0);
        assertEquals(Arrays.asList("field : ok", "list : ok", "map : ok", "array : ok"),
                result.telemetry().transmissions().get(0));
        assertEquals(Collections.emptyList(), result.warnings());
    }

    static final class LinearFieldCopy extends LinearOpMode {
        private final Telemetry copy = telemetry;

        @Override
        public void runOpMode() {
            copy.addData("linear", "ready");
            copy.update();
            waitForStart();
        }
    }

    @Test
    void aLinearOpModesFieldCopyIsRecordedToo() {
        OpModeRunResult result = run(new LinearFieldCopy(), 1);
        assertEquals("linear : ready", shown(result).get(0));
        assertEquals(Collections.emptyList(), result.warnings());
    }

    /** A team base class holds the copy; the OpMode extends it. */
    abstract static class TeamBase extends OpMode {
        protected final Telemetry shared = telemetry;
    }

    static final class FromTeamBase extends TeamBase {
        @Override public void init() { shared.addData("base", "class"); }
        @Override public void loop() { }
    }

    @Test
    void aCopyInATeamBaseClassIsRewired() {
        assertEquals("base : class", shown(run(new FromTeamBase(), 0)).get(0));
    }

    // ---- what cannot be, and is reported ------------------------------------------------------------

    static final class InAnUnmodifiableList extends OpMode {
        final Telemetry both = new MultipleTelemetry(
                Collections.unmodifiableList(Arrays.asList(telemetry, new RecordingTelemetry())));

        @Override public void init() { }
        @Override public void loop() {
            both.addData("lost", 1);
            both.update();
        }
    }

    @Test
    void aCopyInAnUnmodifiableListIsReportedWithWhereItIsAndThatItWasUsed() {
        OpModeRunResult result = run(new InAnUnmodifiableList(), 2);
        assertTrue(shown(result).isEmpty(), "nothing reached the recording telemetry: " + shown(result));
        assertEquals(2, result.warnings().size(), result.warnings().toString());
        assertTrue(result.warnings().get(0).contains("InAnUnmodifiableList.both.telemetryList[0]"),
                result.warnings().get(0));
        assertTrue(result.warnings().get(1).contains("sent telemetry through the SDK's own telemetry object"
                + " (first in ACTIVE, tick 1)"), result.warnings().get(1));
    }

    static final class InASet extends OpMode {
        final Set<Telemetry> sinks = new HashSet<>(Collections.singleton(telemetry));

        @Override public void init() { }
        @Override public void loop() { }
    }

    @Test
    void aCopyInASetIsReported() {
        List<String> warnings = run(new InASet(), 0).warnings();
        assertEquals(1, warnings.size(), warnings.toString());
        assertTrue(warnings.get(0).contains("HashSet at InASet.sinks"), warnings.get(0));
    }

    static final class InAStaticField extends OpMode {
        static Telemetry shared;

        InAStaticField() {
            shared = telemetry;
        }

        @Override public void init() { }
        @Override public void init_loop() {
            shared.addData("static", 1);
            shared.update();
        }
        @Override public void loop() { }
    }

    @Test
    void aCopyTheWalkCannotReachIsReportedTheFirstTimeItIsUsed() {
        OpModeRunResult result = run(new InAStaticField(), 0);
        assertEquals(Collections.emptyList(), result.warnings(), "never used (no INIT_LOOP ticks), so silent");

        OpModeRunResult used = new OpModeRunner(new InAStaticField(), new FakeHardwareMap()).initLoopTicks(3).run();
        assertEquals(1, used.warnings().size(), used.warnings().toString());
        assertTrue(used.warnings().get(0).startsWith("InAStaticField sent telemetry through the SDK's own"
                + " telemetry object (first in INIT_LOOP, tick 1)"), used.warnings().get(0));
    }

    /** The same unreachable copy, written every loop and never sent: it leaves the update to the SDK. */
    static final class InAStaticFieldNeverSent extends OpMode {
        static Telemetry shared;

        InAStaticFieldNeverSent() {
            shared = telemetry;
        }

        @Override public void init() { }
        @Override public void loop() { shared.addData("static", 1); }
    }

    @Test
    void aCopyTheWalkCannotReachIsReportedAtTheEndIfItWasWrittenButNeverSent() {
        OpModeRunResult result = run(new InAStaticFieldNeverSent(), 2);
        assertEquals(1, result.warnings().size(), result.warnings().toString());
        assertTrue(result.warnings().get(0).startsWith("InAStaticFieldNeverSent wrote telemetry to the SDK's own"
                + " telemetry object and never sent it, which SimLoop does not record"), result.warnings().get(0));
    }

    /** A chain of holders one object deeper than the walk looks. */
    static final class Holder {
        final Object next;

        Holder(Object next) {
            this.next = next;
        }
    }

    static final class TooDeep extends OpMode {
        final Holder chain;

        TooDeep() {
            Object held = telemetry;
            for (int i = 0; i <= TelemetryRewiring.MAX_DEPTH; i++) {
                held = new Holder(held);
            }
            chain = (Holder) held;
        }

        @Override public void init() { }
        @Override public void loop() {
            Object at = chain;
            while (at instanceof Holder) {
                at = ((Holder) at).next;
            }
            ((Telemetry) at).addData("deep", 1);
            ((Telemetry) at).update();
        }
    }

    static final class JustDeepEnough extends OpMode {
        final Holder chain;

        JustDeepEnough() {
            Object held = telemetry;
            for (int i = 0; i < TelemetryRewiring.MAX_DEPTH; i++) {
                held = new Holder(held);
            }
            chain = (Holder) held;
        }

        @Override public void init() {
            Object at = chain;
            while (at instanceof Holder) {
                at = ((Holder) at).next;
            }
            ((Telemetry) at).addData("deep", "enough");
        }
        @Override public void loop() { }
    }

    @Test
    void theWalkReachesMaxDepthObjectsDownAndReportsUseBeyondIt() {
        assertEquals("deep : enough", shown(run(new JustDeepEnough(), 0)).get(0));
        List<String> warnings = run(new TooDeep(), 1).warnings();
        assertEquals(1, warnings.size(), warnings.toString());
        assertTrue(warnings.get(0).contains("(first in ACTIVE, tick 1)"), warnings.get(0));
    }

    static final class ItemBeforeInit extends OpMode {
        final Telemetry.Item status = telemetry.addData("status", "constructing");

        @Override public void init() { }
        @Override public void loop() { }
    }

    @Test
    void itemsAddedBeforeInitAreReportedBecauseTheyStayOnTheSdksObject() {
        List<String> warnings = run(new ItemBeforeInit(), 0).warnings();
        assertEquals(1, warnings.size(), warnings.toString());
        assertTrue(warnings.get(0).contains("added telemetry items, lines or log entries before INIT"),
                warnings.get(0));
    }

    // ---- what is left alone -------------------------------------------------------------------------

    static final class Ordinary extends OpMode {
        final List<String> notTelemetry = new ArrayList<>(Arrays.asList("a", "b"));
        final Map<String, Integer> counts = new HashMap<>();
        final Thread.State anEnum = Thread.State.NEW;

        @Override public void init() { telemetry.addData("plain", "init"); }
        @Override public void loop() { }
    }

    @Test
    void anOpModeThatCopiesNothingHasNoWarningsAndItsOtherFieldsAreUntouched() {
        Ordinary opMode = new Ordinary();
        OpModeRunResult result = run(opMode, 1);
        assertEquals(Collections.emptyList(), result.warnings());
        assertEquals(Arrays.asList("a", "b"), opMode.notTelemetry);
        assertSame(Thread.State.NEW, opMode.anEnum);
        assertSame(result.telemetry(), opMode.telemetry);
    }
}
