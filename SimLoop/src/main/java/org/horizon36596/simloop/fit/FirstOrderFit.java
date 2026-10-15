package org.horizon36596.simloop.fit;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.Locale;

/**
 * Fits a first-order velocity plant - the model SimLoop's {@code FakeMotor} and drivetrain already step -
 * to a logged record of applied power and measured velocity, by least squares. Horizon (FTC 36596).
 *
 * <p><b>The model.</b> Velocity {@code v} chases a target set by the power {@code u}:
 * <pre>
 *   dv/dt = (freeSpeed * (u_eff - friction * direction) - v) / timeConstant
 *   u_eff = u * volts / nominalVolts
 * </pre>
 * {@code freeSpeed} is the velocity full power settles at, at the nominal battery voltage, in the
 * velocity's own units. {@code timeConstant} is how long it takes to get 63% of the way there, seconds.
 * The sim's {@code maxAccel} is {@code 1 / timeConstant}, so with no friction and a steady battery this is
 * exactly {@code FakeMotor}'s {@code speed += (power - speed) * maxAccel * dt}, written in velocity units.
 * The friction term is optional; it is reported as {@link Model#coulombFrictionPower}, the share of full
 * power that sliding (Coulomb) friction eats while the mechanism is moving. The sim does not model it, so a
 * large one is itself a sim-vs-real gap. {@code direction} is which way friction acts: for one wheel or a
 * flywheel it is the sign of {@code v}, with a deadband (0 inside the rest band, see below) because the
 * sign of a velocity that is noise around zero is itself noise. A caller fitting a mix of several wheels -
 * a mecanum axis - passes its own per-frame direction instead ({@link Samples}), because each wheel's
 * friction follows that wheel.
 *
 * <p><b>The fit.</b> Written the way the sim steps it, the model says how much the velocity changes over
 * any stretch of the log:
 * <pre>
 *   v[k+W] - v[k] = a * sum(dt u_eff) + b * sum(dt v) + c * sum(dt direction)      sums over frames k .. k+W-1
 * </pre>
 * which is linear in {@code (a, b, c)}, so one {@link LeastSquares} solve over every such window gives them;
 * then {@code timeConstant = -1/b}, {@code freeSpeed = -a/b} and {@code friction = -c/a}. The power logged on
 * frame {@code i} is the one that acted until frame {@code i+1}, because a loop reads its sensors, then
 * writes its outputs, and the world responds before the next loop - on the robot and in the sim alike.
 *
 * <p><b>Which time constant.</b> That equation is the sim's own Euler step, one step per log frame. A real
 * (continuous) plant stepped that way looks slightly slower than it is: the fitted time constant is about
 * the continuous one plus half a frame ({@code tau_fit = dt / (1 - exp(-dt / tau)) ~ tau + dt/2}). So the
 * fitted {@link Model#timeConstantSeconds} is the value to give a sim that steps at the log's own frame
 * time, {@link Result#meanStepSeconds}; the continuous plant's is about {@code mean step / 2} shorter. The
 * report says both.
 *
 * <p><b>Why a window of {@link #WINDOW_FRAMES} frames and not one.</b> Measured velocity is noisy, and it
 * appears on both sides of the equation. Over a single frame the change in velocity is tiny and mostly
 * noise, and that noise is correlated with the velocity on the right, which drags the fit towards a
 * shorter time constant and a lower free speed: on this robot's own flywheel model (a 6 s time
 * constant) with +-15 ticks/s of noise, a one-frame fit found 2409 ticks/s against a true 2800
 * ({@code FirstOrderFitTest}'s slow-flywheel case, which fails if the window goes back to one frame).
 * Summing over a window lets the change grow out of the noise while the noise in the sums averages down.
 *
 * <p><b>Which windows count.</b> Only windows that lie inside the requested time range, in which every
 * step between frames is positive and at most {@link #MAX_GAP_SECONDS} (a longer gap means the log paused,
 * and summing across it would be fiction), and in which the velocity does not stay inside the rest band
 * throughout: a sensor dithering at rest, or a mechanism held still by static friction under a small
 * power, is not a first-order plant moving. The rest band is {@link #REST_FRACTION} of the
 * {@link #REST_PERCENTILE} speed inside the requested time range - a percentile, so one glitch frame
 * cannot inflate it.
 *
 * <p><b>How sure the fit is.</b> Every parameter comes with a standard error, and the fit carries
 * {@link Result#warnings()} when the data could not pin it down: a large {@link LeastSquares.Solution#conditionNumber}
 * (the columns barely differ), or a standard error above {@link #MAX_RELATIVE_STANDARD_ERROR} of the value.
 * This matters because <b>the signals a robot logs usually come from closed-loop control</b>. A flywheel
 * under a velocity PIDF with a constant setpoint has {@code u = kP (setpoint - v) + kF setpoint}, so the
 * power column is almost exactly a combination of the velocity column and the friction-direction column
 * (a constant +1 while the flywheel spins forward): the fit then finds
 * a, b and c that trade off against each other, with a clean R squared and meaningless values. A
 * path-following drivetrain (Pedro in autonomous) is the same. Data where the controller saturated, or
 * the setpoint changed, or a driver drove open-loop, separates the columns; data where a controller held a
 * steady setpoint does not. The warnings are what tells the two apart.
 *
 * <p><b>Known limits.</b> The noise bias comes from the frame at each end of a window: {@code v[k]} is in
 * the target and also in the sums (errors in variables). Windowing dilutes that by the window's length; it
 * does not remove it - on {@code FirstOrderFitTest}'s drivetrain log with 1% noise it leaves the time
 * constant about 1.5% long. Reading {@code v[k+1]} for the first term of the sums would take the shared
 * sample out, but it adds a deterministic error of about {@code dt / (WINDOW_FRAMES * timeConstant)} (1.3%
 * on that same log, noise-free), so it buys nothing and is not done. A mechanism that never settles within
 * a window is fitted mostly from its transients. A velocity sensor that lags (the OctoQuad's is a
 * difference over its own sample interval) adds its lag to the fitted time constant. Under closed-loop
 * control the power itself carries the velocity noise ({@code kP} times it), which windowing does not
 * remove. {@link Result#outputErrorRms} -
 * the fitted model run open-loop against the log - is the check that catches a fit that is still off. Pure
 * and deterministic (domain R5).
 */
public final class FirstOrderFit {

    /**
     * The longest gap between two log frames that is still treated as one step, seconds. SET BY HAND: five
     * 20 ms loops; anything longer is a log that stalled or was paused.
     */
    public static final double MAX_GAP_SECONDS = 0.1;

    /**
     * How many frames each fitted window spans. SET BY HAND: 10 frames, about 0.2 s at a 20 ms loop - a
     * window must be long enough for the velocity change to stand clear of the sensor noise, and short
     * against the fastest time constant it has to see (0.25 s on this robot's drivetrain). Chosen by a sweep
     * of 1, 5, 10, 25 and 50 frames on synthetic logs like {@code FirstOrderFitTest}'s: 1 was biased by up
     * to 17%, 5 by about 1%, and 10, 25 and 50 all recovered every parameter to well under 1%. The shortest
     * of those keeps the most windows on a short log. Those tests therefore check this choice rather than
     * validate it independently; a real log's open-loop error is the independent check.
     */
    public static final int WINDOW_FRAMES = 10;

    /**
     * Below this share of the {@link #REST_PERCENTILE} speed in the fitted time range, a velocity counts as
     * "not moving". SET BY HAND: 2%, a few times the noise of an odometry pod or a REV hub velocity reading
     * at full speed, so a sensor that dithers around zero reads as at rest, and a mechanism held still by
     * static friction under a small power - a regime this model does not describe - is left out rather than
     * fitted as accelerating.
     */
    public static final double REST_FRACTION = 0.02;

    /**
     * Which speed the rest band is a share of: this percentile of {@code |v|} over the frames in the fitted
     * time range. SET BY HAND: the 99th, so the top 1% of frames - where a single glitch would sit - cannot
     * move it, while it stays close to the top speed the log actually reached.
     */
    public static final double REST_PERCENTILE = 0.99;

    /**
     * Fewer independent windows than this and the fit is refused rather than reported. Windows overlap - one
     * starts on every frame - so a log has only about {@code windows / WINDOW_FRAMES} windows that share no
     * frame; this counts those. SET BY HAND: 20, enough to fit three parameters with room to spare.
     */
    public static final int MIN_INDEPENDENT_WINDOWS = 20;

    /**
     * Above this {@link LeastSquares.Solution#conditionNumber}, the fit is flagged: the data barely tells the
     * parameters apart. SET BY HAND: 100. The open-loop synthetic logs in {@code FirstOrderFitTest} sit
     * between 3 and 7, and its closed-loop log at one setpoint near 670, so 100 is far from both.
     */
    public static final double MAX_CONDITION_NUMBER = 100.0;

    /**
     * Above this standard error, as a share of the value, a free speed or time constant is flagged as not
     * pinned down by the data. SET BY HAND: 10%, the tolerance the recovery tests hold the fit to.
     */
    public static final double MAX_RELATIVE_STANDARD_ERROR = 0.10;

    /**
     * Above this standard error, in fractions of full power, a fitted friction is flagged. An absolute limit,
     * not a relative one, because a true friction near zero has an unbounded relative error. SET BY HAND:
     * 0.02, a third of the 0.04-0.06 frictions the tests use.
     */
    public static final double MAX_FRICTION_STANDARD_ERROR = 0.02;

    private FirstOrderFit() {
    }

    /** A first-order plant: what the fit produces, and what the sim's own parameters describe. */
    public static final class Model {
        /** Velocity at full power and nominal voltage, in the velocity's own units (e.g. in/s, ticks/s). */
        public final double freeSpeed;
        /**
         * Seconds to cover 63% of a step change, for an Euler step of one log frame (see the class comment,
         * "Which time constant"). The sim's {@code maxAccel} is its reciprocal.
         */
        public final double timeConstantSeconds;
        /**
         * Power fraction lost to sliding (Coulomb) friction when moving; 0 means none, which is what the sim
         * assumes.
         */
        public final double coulombFrictionPower;

        /**
         * A plant with the given parameters.
         *
         * @param freeSpeed            velocity at full power, velocity units
         * @param timeConstantSeconds  seconds; must be positive
         * @param coulombFrictionPower power fraction, usually in {@code [0, 1)}
         */
        public Model(double freeSpeed, double timeConstantSeconds, double coulombFrictionPower) {
            if (!(timeConstantSeconds > 0.0)) {
                throw new IllegalArgumentException("timeConstantSeconds must be positive, got " + timeConstantSeconds);
            }
            this.freeSpeed = freeSpeed;
            this.timeConstantSeconds = timeConstantSeconds;
            this.coulombFrictionPower = coulombFrictionPower;
        }

        /** {@return the sim's name for how fast velocity chases its target: 1 / time constant, per second} */
        public double maxAccelPerSecond() {
            return 1.0 / timeConstantSeconds;
        }
    }

    /** A logged record for one axis: time, the power applied, the velocity measured, and the battery. */
    public static final class Samples {
        final double[] timeSeconds;
        final double[] power;
        final double[] velocity;
        final double[] volts;
        final double nominalVolts;
        final double[] frictionDirection;

        /**
         * A record whose friction acts against its own velocity: one wheel, one flywheel.
         *
         * @param timeSeconds  frame times, seconds, increasing
         * @param power        applied power per frame, the fraction written to the actuator, in {@code [-1, 1]}
         * @param velocity     measured velocity per frame, any unit; the fitted free speed comes out in it
         * @param volts        battery voltage per frame, volts; {@code null} when the log has none, in which
         *                     case every frame is taken to be at {@code nominalVolts}
         * @param nominalVolts the battery voltage free speed is quoted at, volts
         */
        public Samples(double[] timeSeconds, double[] power, double[] velocity, double[] volts, double nominalVolts) {
            this(timeSeconds, power, velocity, volts, nominalVolts, null);
        }

        /**
         * A record whose friction direction the caller works out: a mix of several wheels, where each wheel's
         * friction follows that wheel's own direction.
         *
         * @param timeSeconds       as above
         * @param power             as above
         * @param velocity          as above
         * @param volts             as above
         * @param nominalVolts      as above
         * @param frictionDirection per frame, the direction friction acts along this axis, unitless in
         *                          {@code [-1, 1]}: for a mecanum axis, the axis's wheel mix of each wheel's
         *                          sign of speed (0 for a wheel at rest). {@code null} to use the sign of
         *                          {@code velocity} with the rest-band deadband, as the other constructor does.
         */
        public Samples(double[] timeSeconds, double[] power, double[] velocity, double[] volts, double nominalVolts,
                double[] frictionDirection) {
            int n = timeSeconds.length;
            if (power.length != n || velocity.length != n || (volts != null && volts.length != n)
                    || (frictionDirection != null && frictionDirection.length != n)) {
                throw new IllegalArgumentException(
                        "time, power, velocity, volts and friction direction must all have one entry per frame");
            }
            if (!(nominalVolts > 0.0)) {
                throw new IllegalArgumentException("nominalVolts must be positive, got " + nominalVolts);
            }
            this.timeSeconds = timeSeconds.clone();
            this.power = power.clone();
            this.velocity = velocity.clone();
            this.volts = volts == null ? null : volts.clone();
            this.nominalVolts = nominalVolts;
            this.frictionDirection = frictionDirection == null ? null : frictionDirection.clone();
        }

        /** {@return whether a real battery voltage was logged, rather than assumed} */
        public boolean hasVoltage() {
            return volts != null;
        }

        int size() {
            return timeSeconds.length;
        }

        /** Power scaled by how full the battery was: what the motor actually felt. */
        double effectivePower(int k) {
            return volts == null ? power[k] : power[k] * volts[k] / nominalVolts;
        }

        /**
         * Which way friction acts on frame {@code k}, unitless: the caller's direction if it gave one, else
         * the sign of the measured velocity, 0 inside {@code restBand} (velocity units).
         */
        double frictionDirection(int k, double restBand) {
            if (frictionDirection != null) {
                return frictionDirection[k];
            }
            return signOutsideRestBand(velocity[k], restBand);
        }
    }

    /** What one fit found, and how well. */
    public static final class Result {
        /** The fitted plant. */
        public final Model model;
        /** One standard error on {@link Model#freeSpeed}, velocity units. */
        public final double freeSpeedStandardError;
        /** One standard error on {@link Model#timeConstantSeconds}, seconds. */
        public final double timeConstantStandardError;
        /** One standard error on {@link Model#coulombFrictionPower}, power fraction; {@code NaN} if not fitted. */
        public final double frictionStandardError;
        /** The least-squares {@link LeastSquares.Solution#conditionNumber}: large means the data barely separates the parameters. */
        public final double conditionNumber;
        /** Windows of {@link #WINDOW_FRAMES} frames the fit used. */
        public final int windowsUsed;
        /** Windows inside the time range that were left out: a gap too long, or at rest throughout. */
        public final int windowsSkipped;
        /** Whether friction was fitted; false means it was held at zero, as the sim assumes. */
        public final boolean frictionFitted;
        /** Time of the first frame the fit used, seconds, on the log's clock: the start of its time window. */
        public final double windowStartSeconds;
        /** Time of the last frame the fit used, seconds, on the log's clock: the end of its time window. */
        public final double windowEndSeconds;
        /**
         * The mean step between frames in the windows used, seconds: the Euler step the fitted time constant
         * applies to (see the class comment, "Which time constant").
         */
        public final double meanStepSeconds;
        /** Below this speed, velocity units, the fit treated the mechanism as at rest. */
        public final double restBand;
        /**
         * RMS of the fit's own residual ({@code measured - predicted} change in velocity over one window), in
         * velocity units. Small does not by itself mean the model is right; see {@link #outputErrorRms}.
         */
        public final double windowResidualRms;
        /**
         * Share of the windows' velocity changes the fit explains: 1 is perfect. The regression has no
         * intercept and this is measured against the changes' mean, so a very poor fit can read below 0.
         * A high value does NOT mean the parameters are right; see {@link #warnings()}.
         */
        public final double rSquared;
        /**
         * RMS of {@code measured - simulated} velocity when the fitted model is run open-loop from the log's
         * power, in velocity units: the honest "does this model reproduce the log" number.
         */
        public final double outputErrorRms;
        private final double[] windowResiduals;
        private final List<String> warnings;

        Result(Model model, double freeSpeedStandardError, double timeConstantStandardError,
                double frictionStandardError, double conditionNumber, boolean frictionFitted, int windowsUsed,
                int windowsSkipped, double windowStartSeconds, double windowEndSeconds, double meanStepSeconds,
                double restBand, double windowResidualRms, double rSquared, double outputErrorRms,
                double[] windowResiduals, List<String> warnings) {
            this.model = model;
            this.freeSpeedStandardError = freeSpeedStandardError;
            this.timeConstantStandardError = timeConstantStandardError;
            this.frictionStandardError = frictionStandardError;
            this.conditionNumber = conditionNumber;
            this.frictionFitted = frictionFitted;
            this.windowResiduals = windowResiduals;
            this.windowsUsed = windowsUsed;
            this.windowsSkipped = windowsSkipped;
            this.windowStartSeconds = windowStartSeconds;
            this.windowEndSeconds = windowEndSeconds;
            this.meanStepSeconds = meanStepSeconds;
            this.restBand = restBand;
            this.windowResidualRms = windowResidualRms;
            this.rSquared = rSquared;
            this.outputErrorRms = outputErrorRms;
            this.warnings = Collections.unmodifiableList(new ArrayList<>(warnings));
        }

        /**
         * {@return a copy of every window's residual - measured minus predicted change in velocity over
         * that window, velocity units - in log order}
         */
        public double[] windowResiduals() {
            return windowResiduals.clone();
        }

        /**
         * {@return why this fit should not be trusted, one sentence each; empty when the data pinned every
         * parameter down}
         */
        public List<String> warnings() {
            return warnings;
        }

        /** {@return whether the data pinned every parameter down: no {@link #warnings()}} */
        public boolean isTrustworthy() {
            return warnings.isEmpty();
        }
    }

    /**
     * Fit one axis.
     *
     * @param samples              the log
     * @param fromSeconds          ignore frames before this time; {@code Double.NEGATIVE_INFINITY} for all
     * @param toSeconds            ignore frames after this time; {@code Double.POSITIVE_INFINITY} for all
     * @param includeFriction      whether to fit the friction term, or hold it at zero as the sim does
     * @return the fit, with {@link Result#warnings()} if the data could not pin it down. A negative
     *         {@link Model#freeSpeed} means the power and velocity keys disagree in sign (one of them is
     *         reversed); it is returned rather than refused so the report can say so.
     * @throws IllegalArgumentException if fewer than {@link #MIN_INDEPENDENT_WINDOWS} independent windows are
     *                                  usable, if the data cannot tell the parameters apart at all, or if the
     *                                  fit is not a stable first-order plant (a velocity that grows on its own)
     */
    public static Result fit(Samples samples, double fromSeconds, double toSeconds, boolean includeFriction) {
        double restBand = restBand(samples, fromSeconds, toSeconds);
        List<double[]> rows = new ArrayList<>();
        List<Double> targets = new ArrayList<>();
        int skipped = 0;
        double first = Double.NaN;
        double last = Double.NaN;
        double stepSum = 0.0;
        int stepCount = 0;
        for (int k = 0; k + WINDOW_FRAMES < samples.size(); k++) {
            int end = k + WINDOW_FRAMES;
            if (samples.timeSeconds[k] < fromSeconds || samples.timeSeconds[end] > toSeconds) {
                continue;
            }
            double sumPower = 0.0;
            double sumVelocity = 0.0;
            double sumDirection = 0.0;
            boolean usable = true;
            boolean moving = false;
            for (int i = k; i < end; i++) {
                double dt = samples.timeSeconds[i + 1] - samples.timeSeconds[i];
                if (!(dt > 0.0) || dt > MAX_GAP_SECONDS) {
                    usable = false;
                    break;
                }
                sumPower += dt * samples.effectivePower(i);
                sumVelocity += dt * samples.velocity[i];
                sumDirection += dt * samples.frictionDirection(i, restBand);
                moving |= Math.abs(samples.velocity[i]) > restBand;
            }
            moving |= Math.abs(samples.velocity[end]) > restBand;
            if (!usable || !moving) {
                skipped++;
                continue;
            }
            rows.add(includeFriction
                    ? new double[] {sumPower, sumVelocity, sumDirection}
                    : new double[] {sumPower, sumVelocity});
            targets.add(samples.velocity[end] - samples.velocity[k]);
            if (Double.isNaN(first)) {
                first = samples.timeSeconds[k];
            }
            last = samples.timeSeconds[end];
            stepSum += samples.timeSeconds[k + 1] - samples.timeSeconds[k];
            stepCount++;
        }
        if (rows.size() < MIN_INDEPENDENT_WINDOWS * WINDOW_FRAMES) {
            throw new IllegalArgumentException("only " + rows.size() + " usable " + WINDOW_FRAMES + "-frame windows in ["
                    + fromSeconds + ", " + toSeconds + "] s, about " + rows.size() / WINDOW_FRAMES + " that share no"
                    + " frame; at least " + MIN_INDEPENDENT_WINDOWS + " of those are needed");
        }

        double[][] a = rows.toArray(new double[0][]);
        double[] b = new double[targets.size()];
        for (int i = 0; i < b.length; i++) {
            b[i] = targets.get(i);
        }
        LeastSquares.Solution solution = includeFriction
                ? LeastSquares.solve(a, b, "power", "velocity", "friction direction")
                : LeastSquares.solve(a, b, "power", "velocity");

        double gain = solution.coefficients[0];
        double decay = solution.coefficients[1];
        if (!Double.isFinite(gain) || gain <= 0.0) {
            throw new IllegalArgumentException("the fit found a power term of " + gain + " (must be finite and"
                    + " positive): the velocity does not rise with the commanded power in this window, so there"
                    + " is no free speed to report. Check the power and velocity keys, and that the window covers"
                    + " the mechanism being driven");
        }
        if (!(decay < 0.0)) {
            throw new IllegalArgumentException("the fit is not a stable first-order plant (velocity term "
                    + decay + " per s, power term " + gain + "): check that the power and velocity keys belong to"
                    + " the same mechanism, and that the window covers it being driven");
        }
        double friction = includeFriction ? -solution.coefficients[2] / gain : 0.0;
        Model model = new Model(-gain / decay, -1.0 / decay, friction);

        // Standard errors. LeastSquares assumes the rows' errors are independent. Overlapping windows are not
        // quite that, but a window's error is mostly the noise on its two END frames, and two windows share an
        // end frame only when they are exactly WINDOW_FRAMES apart - so the rows are close to independent.
        // Checked against the actual spread of 40 re-noised synthetic logs (FirstOrderFitTest's drivetrain
        // and both flywheels): these errors match the drivetrain's spread and are 1.5 to 6 times the
        // flywheels', so they are honest-to-cautious. They say nothing about closed-loop bias.
        double[][] cov = solution.covariance;
        double varGain = cov[0][0];
        double varDecay = cov[1][1];
        double covGainDecay = cov[0][1];
        // First-order error propagation ("delta method") through the three formulas above.
        // timeConstant = -1/b:  d/db = 1/b^2.
        double timeConstantError = Math.sqrt(varDecay) / (decay * decay);
        // freeSpeed = -a/b:  d/da = -1/b, d/db = a/b^2.
        double freeSpeedError = Math.sqrt(Math.max(0.0, varGain / (decay * decay)
                + gain * gain * varDecay / Math.pow(decay, 4)
                - 2.0 * gain * covGainDecay / Math.pow(decay, 3)));
        double frictionError = Double.NaN;
        if (includeFriction) {
            double c = solution.coefficients[2];
            double varC = cov[2][2];
            double covGainC = cov[0][2];
            // friction = -c/a:  d/dc = -1/a, d/da = c/a^2.
            frictionError = Math.sqrt(Math.max(0.0, varC / (gain * gain)
                    + c * c * varGain / Math.pow(gain, 4)
                    - 2.0 * c * covGainC / Math.pow(gain, 3)));
        }

        List<String> warnings = new ArrayList<>();
        if (solution.conditionNumber > MAX_CONDITION_NUMBER) {
            warnings.add(String.format(Locale.ROOT, "the data barely tells the parameters apart"
                    + " (condition number %.0f, limit %.0f): the power was mostly a fixed combination of the"
                    + " velocity, as it is under closed-loop control at a steady setpoint. Fit a stretch where"
                    + " the power changed on its own - a spin-up, a driver's stick input.",
                    solution.conditionNumber, MAX_CONDITION_NUMBER));
        }
        if (!(freeSpeedError <= MAX_RELATIVE_STANDARD_ERROR * Math.abs(model.freeSpeed))) {
            warnings.add(String.format(Locale.ROOT, "free speed is not pinned down: +-%.3g on %.4g,"
                    + " more than %.0f%%", freeSpeedError, model.freeSpeed, 100 * MAX_RELATIVE_STANDARD_ERROR));
        }
        if (!(timeConstantError <= MAX_RELATIVE_STANDARD_ERROR * model.timeConstantSeconds)) {
            warnings.add(String.format(Locale.ROOT, "time constant is not pinned down: +-%.3g s on"
                    + " %.4g s, more than %.0f%%", timeConstantError, model.timeConstantSeconds,
                    100 * MAX_RELATIVE_STANDARD_ERROR));
        }
        if (includeFriction && !(frictionError <= MAX_FRICTION_STANDARD_ERROR)) {
            warnings.add(String.format(Locale.ROOT, "friction is not pinned down: +-%.3g on %.3g,"
                    + " more than %.2f of full power", frictionError, friction, MAX_FRICTION_STANDARD_ERROR));
        }

        double outputError = outputErrorRms(model, samples, first, last, restBand);
        return new Result(model, freeSpeedError, timeConstantError, frictionError, solution.conditionNumber,
                includeFriction, rows.size(), skipped, first, last, stepSum / stepCount, restBand,
                solution.rmsResidual, solution.rSquared, outputError, solution.residuals, warnings);
    }

    /**
     * {@return the speed below which a mechanism counts as at rest, velocity units: {@link #REST_FRACTION} of
     * the {@link #REST_PERCENTILE} value of {@code |v|} over the frames in {@code [fromSeconds, toSeconds]};
     * 0 if no frame is in it}
     */
    static double restBand(Samples samples, double fromSeconds, double toSeconds) {
        double[] speeds = new double[samples.size()];
        int count = 0;
        for (int k = 0; k < samples.size(); k++) {
            double t = samples.timeSeconds[k];
            if (t >= fromSeconds && t <= toSeconds) {
                speeds[count++] = Math.abs(samples.velocity[k]);
            }
        }
        if (count == 0) {
            return 0.0;
        }
        Arrays.sort(speeds, 0, count);
        int index = (int) Math.floor(REST_PERCENTILE * (count - 1));
        return REST_FRACTION * speeds[index];
    }

    /**
     * {@return the sign of {@code velocity}, or 0 when it is inside {@code restBand}: which way friction acts,
     * unitless}
     *
     * @param velocity any velocity, in any unit
     * @param restBand at or above this magnitude, same unit, the velocity counts as moving
     */
    public static double signOutsideRestBand(double velocity, double restBand) {
        return Math.abs(velocity) < restBand ? 0.0 : Math.signum(velocity);
    }

    /**
     * Run a model open-loop from the log's power and compare its velocity with the logged one: the RMS of
     * {@code measured - simulated}, in velocity units, over the frames in {@code [fromSeconds, toSeconds]}.
     *
     * <p>The simulation starts from the first measured velocity in the window and steps exactly as the fit's
     * equation does. Across a gap longer than {@link #MAX_GAP_SECONDS} it restarts from the measured velocity,
     * because the log does not say what happened in between. This is how both the fitted model and the sim's
     * own parameters are scored against a real log, so the two numbers are comparable.
     *
     * @param model       the plant to run
     * @param samples     the log
     * @param fromSeconds window start, seconds
     * @param toSeconds   window end, seconds
     * @return the RMS error, velocity units; {@code NaN} if no frame is in the window
     */
    public static double outputErrorRms(Model model, Samples samples, double fromSeconds, double toSeconds) {
        return outputErrorRms(model, samples, fromSeconds, toSeconds, restBand(samples, fromSeconds, toSeconds));
    }

    /**
     * As {@link #outputErrorRms(Model, Samples, double, double)}, with the rest band (velocity units) given:
     * pass a fit's {@link Result#restBand} so the simulation's friction deadband is exactly the one the fit
     * used, for the fitted model and the sim's alike.
     *
     * @param model       the plant to run
     * @param samples     the log
     * @param fromSeconds window start, seconds
     * @param toSeconds   window end, seconds
     * @param restBand    the measured speed below which a frame counts as at rest, velocity units
     * @return the RMS error, velocity units; {@code NaN} if no frame is in the window
     */
    public static double outputErrorRms(Model model, Samples samples, double fromSeconds, double toSeconds,
            double restBand) {
        boolean[] stepped = new boolean[samples.size()];
        double[] simulated = simulate(model, samples, fromSeconds, toSeconds, restBand, stepped);
        double sumSquares = 0.0;
        int count = 0;
        for (int k = 0; k < samples.size(); k++) {
            // Only frames the model actually predicted: a start or restart frame is copied from the log, so
            // its error is zero by construction and would only dilute the average.
            if (stepped[k]) {
                double error = samples.velocity[k] - simulated[k];
                sumSquares += error * error;
                count++;
            }
        }
        return count == 0 ? Double.NaN : Math.sqrt(sumSquares / count);
    }

    /**
     * The model's velocity on every frame of {@code samples} inside the window, driven by the logged power;
     * {@code NaN} outside it. See {@link #outputErrorRms} for how it starts and restarts.
     *
     * @param model       the plant to run
     * @param samples     the log
     * @param fromSeconds window start, seconds
     * @param toSeconds   window end, seconds
     * @return the simulated velocity per frame, velocity units; {@code NaN} outside the window
     */
    public static double[] simulate(Model model, Samples samples, double fromSeconds, double toSeconds) {
        return simulate(model, samples, fromSeconds, toSeconds, restBand(samples, fromSeconds, toSeconds),
                new boolean[samples.size()]);
    }

    private static double[] simulate(Model model, Samples samples, double fromSeconds, double toSeconds,
            double restBand, boolean[] stepped) {
        double[] out = new double[samples.size()];
        double decayPerSecond = 1.0 / model.timeConstantSeconds;
        boolean running = false;
        double v = 0.0;
        for (int k = 0; k < samples.size(); k++) {
            double t = samples.timeSeconds[k];
            if (t < fromSeconds || t > toSeconds) {
                out[k] = Double.NaN;
                running = false;
                continue;
            }
            double dt = k == 0 ? Double.NaN : t - samples.timeSeconds[k - 1];
            if (!running || !(dt > 0.0) || dt > MAX_GAP_SECONDS) {
                v = samples.velocity[k];          // start, or restart after a gap, from what was measured
                running = true;
            } else {
                double u = samples.effectivePower(k - 1);
                double target = model.freeSpeed * u;
                // Friction's direction: the simulated velocity's own sign, with the same rest-band deadband the
                // fit uses - or, for a mix of wheels, the caller's logged direction, since the simulation has
                // no wheels to take it from.
                double direction = samples.frictionDirection != null
                        ? samples.frictionDirection[k - 1]
                        : signOutsideRestBand(v, restBand);
                double friction = model.freeSpeed * model.coulombFrictionPower * direction;
                // Plain Euler, one step per frame: the same equation the fit solves.
                v += dt * decayPerSecond * (target - v - friction);
                stepped[k] = true;
            }
            out[k] = v;
        }
        return out;
    }
}
