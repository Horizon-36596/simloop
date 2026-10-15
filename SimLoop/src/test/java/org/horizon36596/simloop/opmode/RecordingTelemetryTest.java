package org.horizon36596.simloop.opmode;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.junit.jupiter.api.Test;

import java.util.Arrays;
import java.util.Collections;
import java.util.Locale;

/**
 * Pins {@link RecordingTelemetry}: what a Driver Station would have shown, rendered with the SDK's default
 * separators, in a locale-independent way, with the SDK's auto-clear and retention rules.
 *
 * <p>Implementation tier: the exact rendering is this class's own choice where the SDK leaves it open.
 */
class RecordingTelemetryTest {

    @Test
    void itemsAndLinesRenderWithTheSdkDefaultSeparators() {
        RecordingTelemetry telemetry = new RecordingTelemetry();
        telemetry.addData("heading", "%.1f", 12.345);
        telemetry.addData("target", 7);
        Telemetry.Line line = telemetry.addLine("Drive ");
        line.addData("x", 1).addData("y", 2);
        telemetry.addLine("plain words");
        telemetry.update();

        assertEquals(Arrays.asList("heading : 12.3", "target : 7", "Drive x : 1 | y : 2", "plain words"),
                telemetry.lastTransmission());
    }

    @Test
    void formattingDoesNotDependOnTheLaptopsLocale() {
        Locale before = Locale.getDefault();
        try {
            Locale.setDefault(Locale.GERMANY); // would print 12,3
            RecordingTelemetry telemetry = new RecordingTelemetry();
            telemetry.addData("v", "%.1f", 12.345);
            telemetry.log().add("%.2f", 0.5);
            telemetry.update();
            assertEquals(Collections.singletonList("v : 12.3"), telemetry.lastTransmission());
            assertEquals(Collections.singletonList("0.50"), telemetry.loggedLines());
        } finally {
            Locale.setDefault(before);
        }
    }

    @Test
    void autoClearDropsEverythingButRetainedItemsAndFuncValuesAreReadAtUpdate() {
        RecordingTelemetry telemetry = new RecordingTelemetry();
        int[] counter = {0};
        telemetry.addData("count", () -> counter[0]).setRetained(true);
        telemetry.addData("once", "x");
        counter[0] = 5;
        telemetry.update();
        counter[0] = 6;
        telemetry.update(); // nothing added since: the SDK clears on the next add, so the screen stays
        telemetry.addData("next", "y");
        telemetry.update();

        assertEquals(Arrays.asList(Arrays.asList("count : 5", "once : x"), Arrays.asList("count : 6", "once : x"),
                        Arrays.asList("count : 6", "next : y")),
                telemetry.transmissions());
    }

    @Test
    void withAutoClearOffItemsStayAndActionsRunBeforeEachUpdate() {
        RecordingTelemetry telemetry = new RecordingTelemetry();
        telemetry.setAutoClear(false);
        int[] actionRuns = {0};
        telemetry.addAction(() -> actionRuns[0]++);
        Telemetry.Item item = telemetry.addData("a", 1);
        telemetry.update();
        item.setValue("%d%%", 50);
        telemetry.update();
        assertEquals(2, actionRuns[0]);
        assertEquals(Collections.singletonList("a : 50%"), telemetry.lastTransmission());
        assertTrue(telemetry.removeItem(item));
        telemetry.update();
        assertEquals(Collections.emptyList(), telemetry.lastTransmission());
    }

    @Test
    void theLogKeepsEveryLineAndClearAllEmptiesEverything() {
        RecordingTelemetry telemetry = new RecordingTelemetry();
        telemetry.log().setCapacity(1);
        telemetry.log().add("first");
        telemetry.log().add("second");
        assertEquals(Arrays.asList("first", "second"), telemetry.loggedLines());
        telemetry.addData("k", "v").setRetained(true);
        telemetry.clearAll();
        telemetry.update();
        assertEquals(Collections.emptyList(), telemetry.lastTransmission());
        assertEquals(Collections.emptyList(), telemetry.loggedLines());
    }
}
