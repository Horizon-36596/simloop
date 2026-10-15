package org.horizon36596.simloop.fit;

import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Locale;

/**
 * The sim-versus-real gap report: for each fitted quantity, what the real log says, what the sim's own
 * parameters say, and how far apart they are, written as one markdown page beside the RLOG it came from.
 * Horizon (FTC 36596) SimLoop.
 *
 * <p>Season-agnostic: a season names its quantities, their units and its sim's parameters, and this class
 * lays them out. It never writes a number anywhere but the report. Changing a sim or robot constant from
 * what a fit says is a person's decision, made in a tuning session.
 */
public final class GapReport {

    /** One fitted quantity: a drivetrain axis, a flywheel, anything that is a first-order velocity plant. */
    public static final class Quantity {
        /** What it is, e.g. "Drivetrain forward". */
        public final String name;
        /** The velocity's unit, e.g. "in/s". */
        public final String velocityUnit;
        /** Where the power came from: the key, or how several keys were mixed. */
        public final String powerSource;
        /** The key the velocity came from. */
        public final String velocitySource;
        /** The fit to the real log. */
        public final FirstOrderFit.Result fit;
        /** The sim's own parameters for the same quantity. */
        public final FirstOrderFit.Model sim;
        /**
         * The sim's parameters run open-loop against the same log: RMS error, velocity units
         * ({@link FirstOrderFit#outputErrorRms}).
         */
        public final double simOutputErrorRms;

        /**
         * One fitted quantity, with the sim's parameters beside it.
         *
         * @param name              what it is, e.g. "Drivetrain forward"
         * @param velocityUnit      the velocity's unit, e.g. "in/s"
         * @param powerSource       where the power came from, e.g. the key or the mix of keys
         * @param velocitySource    the key the velocity came from
         * @param fit               the fit to the real log
         * @param sim               the sim's own parameters for the same quantity
         * @param simOutputErrorRms the sim's parameters run open-loop against the same log: RMS error,
         *                          velocity units
         */
        public Quantity(String name, String velocityUnit, String powerSource, String velocitySource,
                FirstOrderFit.Result fit, FirstOrderFit.Model sim, double simOutputErrorRms) {
            this.name = name;
            this.velocityUnit = velocityUnit;
            this.powerSource = powerSource;
            this.velocitySource = velocitySource;
            this.fit = fit;
            this.sim = sim;
            this.simOutputErrorRms = simOutputErrorRms;
        }
    }

    private final String title;
    private final Path rlog;
    private final String voltageNote;
    private final List<Quantity> quantities = new ArrayList<>();
    private final List<String> notes = new ArrayList<>();

    /**
     * An empty report; add quantities and notes, then write it.
     *
     * @param title       the report's heading, e.g. "Horizon (FTC 36596) sim-vs-real gap report"
     * @param rlog        the log that was fitted
     * @param voltageNote how battery voltage was treated, e.g. "logged under Robot/voltage" or "not in the
     *                    log; 12.0 V assumed throughout"
     */
    public GapReport(String title, Path rlog, String voltageNote) {
        this.title = title;
        this.rlog = rlog;
        this.voltageNote = voltageNote;
    }

    /**
     * Add a fitted quantity.
     *
     * @param quantity the fitted quantity
     * @return this report, for chaining
     */
    public GapReport add(Quantity quantity) {
        quantities.add(quantity);
        return this;
    }

    /**
     * Add a line to the notes at the end: a quantity that could not be fitted, and why, for instance.
     *
     * @param line the note, one line of text
     * @return this report, for chaining
     */
    public GapReport note(String line) {
        notes.add(line);
        return this;
    }

    /**
     * {@return {@code robot.rlog} to {@code robot.fit.md}, in the same directory}
     *
     * @param rlog the log the report is for
     */
    public static Path reportPathFor(Path rlog) {
        String name = rlog.getFileName().toString().replaceFirst("\\.rlog$", "") + ".fit.md";
        return rlog.toAbsolutePath().resolveSibling(name);
    }

    /**
     * Write the report beside the RLOG.
     *
     * @return where it was written
     * @throws IOException if it cannot be
     */
    public Path write() throws IOException {
        Path out = reportPathFor(rlog);
        Files.write(out, render().getBytes(StandardCharsets.UTF_8));
        return out;
    }

    /** {@return the report as markdown} */
    public String render() {
        StringBuilder md = new StringBuilder();
        md.append("# ").append(title).append("\n\n");
        md.append("| | |\n|---|---|\n");
        md.append("| Log | `").append(rlog.getFileName()).append("` |\n");
        md.append("| Battery voltage | ").append(voltageNote).append(" |\n");
        md.append("| Model | first order: `dv/dt = (freeSpeed * (u - friction * direction) - v) / timeConstant`,"
                + " the sim's own motor model plus a sliding-friction term the sim does not have |\n\n");

        boolean anyWarning = false;
        for (Quantity q : quantities) {
            anyWarning |= !q.fit.isTrustworthy();
        }
        if (anyWarning) {
            md.append("## Warnings: do not trust these rows\n\n");
            for (Quantity q : quantities) {
                for (String warning : q.fit.warnings()) {
                    md.append("- **").append(q.name).append(":** ").append(warning).append('\n');
                }
            }
            md.append('\n');
        }

        md.append("## Gap table\n\n");
        md.append("Real is the least-squares fit to the log, and +- is one standard error on it. Sim is the sim's"
                + " own parameter. Gap is real minus sim, and as a share of sim. The +- counts the measurement"
                + " noise only, not how far the model itself is from the robot (sensor lag, friction that is not"
                + " exactly Coulomb, a controller's noise in the power): on a noise-free synthetic log the fit"
                + " still lands several standard errors from the truth. Read it as a lower bound on the"
                + " uncertainty; a gap within it is certainly not evidence of a difference, and a gap a little"
                + " outside it is not proof of one.\n\n");
        md.append("| Quantity | Parameter | Real (fitted) | +- | Sim | Gap | Unit |\n");
        md.append("|---|---|---|---|---|---|---|\n");
        for (Quantity q : quantities) {
            row(md, q.name, "free speed at full power", q.fit.model.freeSpeed, q.fit.freeSpeedStandardError,
                    q.sim.freeSpeed, q.velocityUnit);
            row(md, q.name, "time constant (Euler step of " + number(1000.0 * q.fit.meanStepSeconds) + " ms)",
                    q.fit.model.timeConstantSeconds, q.fit.timeConstantStandardError, q.sim.timeConstantSeconds, "s");
            if (q.fit.frictionFitted) {
                row(md, q.name, "friction", q.fit.model.coulombFrictionPower, q.fit.frictionStandardError,
                        q.sim.coulombFrictionPower, "fraction of full power");
            } else {
                md.append("| ").append(q.name).append(" | friction | not fitted | | ")
                        .append(number(q.sim.coulombFrictionPower)).append(" | n/a | fraction of full power |\n");
            }
        }

        md.append("\n## Model error vs the log\n\n");
        md.append("Each model run open-loop from the log's own power, compared with the log's measured velocity:"
                + " RMS of the difference. This is the number to trust most.\n\n");
        md.append("| Quantity | Fitted model | Sim's model | Unit |\n");
        md.append("|---|---|---|---|\n");
        for (Quantity q : quantities) {
            md.append("| ").append(q.name)
                    .append(" | ").append(number(q.fit.outputErrorRms))
                    .append(" | ").append(number(q.simOutputErrorRms))
                    .append(" | ").append(q.velocityUnit).append(" |\n");
        }

        md.append("\n## Fit quality and window\n\n");
        md.append("| Quantity | Window, s | Windows used | Skipped | Window residual RMS | R squared | Condition"
                + " number | Power from | Velocity from |\n");
        md.append("|---|---|---|---|---|---|---|---|---|\n");
        for (Quantity q : quantities) {
            md.append("| ").append(q.name)
                    .append(" | ").append(number(q.fit.windowStartSeconds)).append(" to ")
                    .append(number(q.fit.windowEndSeconds))
                    .append(" | ").append(q.fit.windowsUsed)
                    .append(" | ").append(q.fit.windowsSkipped)
                    .append(" | ").append(number(q.fit.windowResidualRms)).append(' ').append(q.velocityUnit)
                    .append(" | ").append(number(q.fit.rSquared))
                    .append(" | ").append(number(q.fit.conditionNumber))
                    .append(" | ").append(q.powerSource)
                    .append(" | `").append(q.velocitySource).append("` |\n");
        }

        md.append("\n## Reading it\n\n");
        md.append("- **Open-loop error** is the number to trust: each model is run from the log's own power and"
                + " compared with the log's velocity. If the sim's error is much larger than the fit's, the sim"
                + " is not describing this robot.\n");
        md.append("- **Window residual** and **R squared** say how well the velocity change over each "
                + FirstOrderFit.WINDOW_FRAMES + "-frame window was explained; noisy velocity makes them look"
                + " worse without making the fit wrong, and a high R squared does NOT make the parameters right."
                + " The **standard errors** and the **condition number** (above " + number(FirstOrderFit.MAX_CONDITION_NUMBER)
                + " the data barely tells the parameters apart) say whether the parameters are pinned down.\n");
        md.append("- **Closed-loop logs.** The power a robot logs usually comes from a controller: a flywheel's"
                + " velocity PIDF, or a path follower in autonomous. While a controller holds a steady setpoint"
                + " the power is a fixed mix of the velocity, and the fit cannot separate free speed, time"
                + " constant and friction - it can still show a clean R squared. Spin-ups, setpoint changes and a"
                + " driver's open-loop stick input are what pin the parameters down. Any fit the data could not"
                + " pin down is listed under Warnings.\n");
        md.append("- **Time constant** is the one to give a sim that steps once per log frame (the step it"
                + " names). The real, continuous plant's is about half a step shorter.\n");
        md.append("- **Friction** is modelled by the fit only. The sim has none, so any friction here is a gap"
                + " the sim cannot close by changing a number.\n");
        md.append("- **Time constants include sensor lag.** A velocity measured as a difference over the sensor's"
                + " own sample interval arrives late, and the fit reads that lateness as a slower plant.\n");
        md.append("- **Nothing was changed.** These numbers are printed here and nowhere else. A MEASURED"
                + " constant changes only in a tuning session on the robot; a sim parameter changes when a"
                + " person decides the fit is trustworthy.\n");
        for (String line : notes) {
            md.append("- ").append(line).append('\n');
        }
        return md.toString();
    }

    private static void row(StringBuilder md, String quantity, String parameter, double real,
            double standardError, double sim, String unit) {
        md.append("| ").append(quantity)
                .append(" | ").append(parameter)
                .append(" | ").append(number(real))
                .append(" | ").append(number(standardError))
                .append(" | ").append(number(sim))
                .append(" | ").append(gap(real, sim))
                .append(" | ").append(unit).append(" |\n");
    }

    /** {@code real - sim}, and the share of sim when sim is not zero. */
    static String gap(double real, double sim) {
        double difference = real - sim;
        String text = signed(difference);
        if (sim != 0.0 && Double.isFinite(sim)) {
            text += String.format(Locale.ROOT, " (%+.1f%%)", 100.0 * difference / Math.abs(sim));
        }
        return text;
    }

    /** Four significant figures, locale-independent, so a report reads the same on every machine. */
    static String number(double value) {
        if (Double.isNaN(value)) {
            return "n/a";
        }
        return String.format(Locale.ROOT, "%.4g", value);
    }

    private static String signed(double value) {
        if (Double.isNaN(value)) {
            return "n/a";
        }
        return String.format(Locale.ROOT, "%+.4g", value);
    }

    /**
     * {@return the quantity of that name}
     *
     * @param name the name it was added under, e.g. "Drivetrain forward"
     * @throws IllegalArgumentException if none was added under it
     */
    public Quantity quantity(String name) {
        for (Quantity q : quantities) {
            if (q.name.equals(name)) {
                return q;
            }
        }
        throw new IllegalArgumentException("no quantity '" + name + "' in this report");
    }

    /** {@return the quantities added so far, for a caller that wants the numbers as well as the page} */
    public List<Quantity> quantities() {
        return Collections.unmodifiableList(quantities);
    }
}
