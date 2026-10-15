package org.horizon36596.simloop.fit;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import java.nio.file.Paths;

/**
 * Pins what the gap report must say: every quantity's real and sim numbers with units, the gap, the fit
 * window, and that nothing was changed. IMPLEMENTATION tier for the layout; INVARIANT tier for "the report
 * says it wrote nothing anywhere else", which is the promise that keeps a fit out of the MEASURED constants,
 * and for "a fit the data could not pin down is flagged", which keeps a closed-loop log from passing as a
 * measurement.
 */
class GapReportTest {

    @Test
    void theReportCarriesTheGapTableTheWindowAndTheNothingChangedLine() {
        FirstOrderFit.Samples log = FirstOrderFitTest.synthesize(
                new FirstOrderFit.Model(55.0, 0.15, 0.05), 12.0, 12.0, 0.0, 1500, 4);
        FirstOrderFit.Result fit = FirstOrderFit.fit(log, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY, true);
        FirstOrderFit.Model sim = new FirstOrderFit.Model(60.0, 0.12, 0.0);
        double simError = FirstOrderFit.outputErrorRms(sim, log, fit.windowStartSeconds, fit.windowEndSeconds);

        String md = new GapReport("Horizon (FTC 36596) sim-vs-real gap report", Paths.get("robot-run.rlog"),
                "not in the log; 12.0 V assumed throughout")
                .add(new GapReport.Quantity("Drivetrain forward", "in/s", "mix of Drive/*/output",
                        "RealOutputs/Drive/measuredForwardVelocityInchesPerSecond", fit, sim, simError))
                .note("Lateral: not fitted - the robot never strafed in this log.")
                .render();

        assertEquals(55.0, fit.model.freeSpeed, 55.0 * 0.01, "the synthetic plant's free speed, for the row below");
        assertTrue(md.contains("| Drivetrain forward | free speed at full power | "
                + GapReport.number(fit.model.freeSpeed) + " | " + GapReport.number(fit.freeSpeedStandardError)
                + " | 60.00 | " + GapReport.gap(fit.model.freeSpeed, 60.0) + " | in/s |"), md);
        assertTrue(md.contains("| Drivetrain forward | time constant (Euler step of "
                + GapReport.number(1000.0 * fit.meanStepSeconds) + " ms) | "
                + GapReport.number(fit.model.timeConstantSeconds) + " | "), md);
        assertTrue(md.contains("## Model error vs the log"), md);
        assertTrue(md.contains("| Drivetrain forward | " + GapReport.number(fit.outputErrorRms) + " | "
                + GapReport.number(simError) + " | in/s |"), "the open-loop errors have their own table:\n" + md);
        assertFalse(md.contains("open-loop velocity error"), "and are no longer rows of the parameter table:\n" + md);
        assertFalse(md.contains("## Warnings"), "a clean open-loop fit carries no warnings:\n" + md);
        assertTrue(md.contains("Closed-loop logs"), "the report says closed-loop data can mislead:\n" + md);
        assertTrue(md.contains("12.0 V assumed"), md);
        assertTrue(md.contains("Nothing was changed"), md);
        assertTrue(md.contains("never strafed"), md);
        assertTrue(md.contains(GapReport.number(fit.windowStartSeconds) + " to "
                + GapReport.number(fit.windowEndSeconds)), md);
    }

    /** A sign mix-up no longer reaches the report: FirstOrderFit refuses a non-positive gain outright. */
    @Test
    void aFitWithoutFrictionSaysFrictionWasNotFitted() {
        FirstOrderFit.Samples log = FirstOrderFitTest.synthesize(
                new FirstOrderFit.Model(55.0, 0.15, 0.0), 12.0, 12.0, 0.0, 1500, 4);
        FirstOrderFit.Result fit = FirstOrderFit.fit(log, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY, false);
        FirstOrderFit.Model sim = new FirstOrderFit.Model(60.0, 0.12, 0.0);

        String md = new GapReport("report", Paths.get("nofriction.rlog"), "assumed")
                .add(new GapReport.Quantity("Strafe", "in/s", "mix", "v", fit, sim, 1.0))
                .render();

        assertFalse(md.contains("free speed is negative"), md);
        assertTrue(md.contains("| Strafe | friction | not fitted |"), md);
    }

    @Test
    void aQuantityIsFoundByName() {
        FirstOrderFit.Samples log = FirstOrderFitTest.synthesize(
                new FirstOrderFit.Model(55.0, 0.15, 0.05), 12.0, 12.0, 0.0, 1500, 4);
        FirstOrderFit.Result fit = FirstOrderFit.fit(log, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY, true);
        GapReport report = new GapReport("report", Paths.get("x.rlog"), "assumed")
                .add(new GapReport.Quantity("Flywheel", "ticks/s", "p", "v", fit, fit.model, 0.0));

        assertEquals(fit.model.freeSpeed, report.quantity("Flywheel").fit.model.freeSpeed, 0.0);
    }

    /** A fit the data could not pin down is listed under a Warnings heading, by name, before the numbers. */
    @Test
    void aFitTheDataCouldNotPinDownIsWarnedAboutAboveTheGapTable() {
        // FirstOrderFitTest's near-one-setpoint closed-loop case: the robot's flywheel, its setpoint wandering.
        FirstOrderFit.Model flywheel = new FirstOrderFit.Model(2800.0, 2800.0 / 467.0, 0.05);
        FirstOrderFit.Samples log = FirstOrderFitTest.synthesizeClosedLoop(flywheel, 0.01, 0.00055,
                t -> 1500.0 + 50.0 * Math.sin(0.5 * t), 1523.0, 15.0, 3000, 5);
        FirstOrderFit.Result fit = FirstOrderFit.fit(log, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY, true);

        String md = new GapReport("report", Paths.get("flywheel.rlog"), "assumed")
                .add(new GapReport.Quantity("Flywheel", "ticks/s", "p", "v", fit, flywheel, 1.0))
                .render();

        assertTrue(md.contains("## Warnings"), md);
        assertTrue(md.indexOf("## Warnings") < md.indexOf("## Gap table"), md);
        assertTrue(md.contains("- **Flywheel:** free speed is not pinned down"), md);
    }

    @Test
    void theReportIsWrittenBesideItsLog() {
        assertEquals(Paths.get("logs", "match-3.fit.md").toAbsolutePath(),
                GapReport.reportPathFor(Paths.get("logs", "match-3.rlog")));
    }

    @Test
    void numbersAreTheSameOnEveryMachine() {
        assertEquals("0.1235", GapReport.number(0.123456));
        assertEquals("+1.000 (+10.0%)", GapReport.gap(11.0, 10.0));
        assertEquals("n/a", GapReport.number(Double.NaN));
    }
}
