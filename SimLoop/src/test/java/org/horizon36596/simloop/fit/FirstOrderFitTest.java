package org.horizon36596.simloop.fit;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.horizon36596.simloop.field.SeededRandom;
import org.junit.jupiter.api.Test;

/**
 * The Contract's recovery test at the core: a log generated from KNOWN plant parameters, with noise, is
 * fitted, and the fit must give those parameters back within the tolerances stated on each assertion.
 * INVARIANT tier - it holds for any correct fitting method, and the tolerances are set from what the noise
 * allows, not from what this implementation happens to achieve.
 *
 * <p><b>How the synthetic log is made.</b> The plant is the CONTINUOUS first-order model, stepped with
 * {@link #SUBSTEPS} small Euler steps per log frame so it is exact to well under the noise - not the fit's
 * own one-step-per-frame equation, which would make recovery circular. It is driven from a power schedule
 * with steps, ramps and reversals so every parameter is exercised, at a loop time that jitters between 15
 * and 25 ms the way a real robot's does. Because the fit reports the time constant for an Euler step of one
 * frame ({@link FirstOrderFit}, "Which time constant"), the expected value is {@link #eulerTimeConstant}. Measurement
 * noise is added to the LOGGED velocity only, never to the plant's true state - the same as an encoder.
 * {@code SeededRandom} has no Gaussian, so the noise is uniform, which is harsher than Gaussian for the
 * same standard deviation at the extremes and gentler at the centre; the tolerances do not depend on the
 * shape.
 *
 * <p><b>Where the tolerances come from.</b> Free speed within 3%: the noise is about 1% of full speed and
 * the fit uses thousands of windows, so a correct method lands well inside, and a method with a systematic
 * error - like the one-frame fit - lands outside. Time constant within 10%: it is read from the transients
 * only, a few hundred windows' worth, so it is the noisiest parameter. Friction within 20% of its true value
 * (0.04 to 0.06 in these logs): tight enough that a fit that ignored friction's direction, or got its size
 * badly wrong, fails. None of these was tightened to what this implementation happens to achieve.
 *
 * <p><b>One caveat.</b> {@link FirstOrderFit#WINDOW_FRAMES} was chosen by a sweep over logs like
 * these, so they check that choice rather than validate it independently; a real robot log's open-loop
 * error is the independent check.
 */
class FirstOrderFitTest {

    /** A flywheel-like plant: 2800 ticks/s free speed, 0.25 s time constant, 4% friction. */
    private static final FirstOrderFit.Model FLYWHEEL = new FirstOrderFit.Model(2800.0, 0.25, 0.04);

    /** A drivetrain axis: 60 in/s, 0.12 s, 6% friction. */
    private static final FirstOrderFit.Model DRIVE_AXIS = new FirstOrderFit.Model(60.0, 0.12, 0.06);

    /** Euler steps per log frame in the synthetic plant: enough that it is the continuous plant, to well under the noise. */
    static final int SUBSTEPS = 50;

    /** The mean frame time of the synthetic logs, seconds: the loop time jitters uniformly between 15 and 25 ms. */
    static final double MEAN_STEP_SECONDS = 0.020;

    /**
     * {@return the time constant an Euler step of one mean frame must use to match a continuous plant with
     * time constant {@code tau}: {@code dt / (1 - exp(-dt / tau))}, about {@code tau + dt/2}}
     */
    static double eulerTimeConstant(double tau) {
        return MEAN_STEP_SECONDS / (1.0 - Math.exp(-MEAN_STEP_SECONDS / tau));
    }

    @Test
    void invariant_aNoiselessLogGivesTheParametersBack() {
        FirstOrderFit.Samples log = synthesize(DRIVE_AXIS, 12.0, 12.0, 0.0, 1500, 3);

        FirstOrderFit.Result fit = FirstOrderFit.fit(log, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY, true);

        // Not exact: the plant is continuous and the fit is one Euler step per frame with a jittering frame,
        // and the friction deadband at rest differs slightly from this plant, which chatters through zero.
        // What is left must be well inside the noisy cases' tolerances.
        assertEquals(60.0, fit.model.freeSpeed, 60.0 * 0.01, "free speed within 1%");
        assertEquals(eulerTimeConstant(0.12), fit.model.timeConstantSeconds, eulerTimeConstant(0.12) * 0.03,
                "time constant within 3% of the one-frame Euler value");
        assertEquals(0.06, fit.model.coulombFrictionPower, 0.06 * 0.10, "friction within 10%");
        assertEquals(MEAN_STEP_SECONDS, fit.meanStepSeconds, 0.001, "the step the time constant applies to");
        assertTrue(fit.isTrustworthy(), "an open-loop log pins every parameter down: " + fit.warnings());
        assertTrue(fit.outputErrorRms < 0.5, "the fitted model reproduces the log it came from, got " + fit.outputErrorRms);
    }

    @Test
    void invariant_aNoisyDrivetrainLogIsRecoveredWithinStatedTolerances() {
        // Noise: +-1 in/s uniform on the measured velocity (sd 0.58 in/s, about 1% of full speed) - worse
        // than an odometry pod, so the tolerance is realistic for a real log.
        FirstOrderFit.Samples log = synthesize(DRIVE_AXIS, 12.0, 12.0, 1.0, 3000, 5);

        FirstOrderFit.Result fit = FirstOrderFit.fit(log, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY, true);

        assertEquals(60.0, fit.model.freeSpeed, 60.0 * 0.03, "free speed within 3%");
        assertEquals(eulerTimeConstant(0.12), fit.model.timeConstantSeconds, eulerTimeConstant(0.12) * 0.10,
                "time constant within 10%");
        assertEquals(0.06, fit.model.coulombFrictionPower, 0.06 * 0.20, "friction within 20%");
        assertTrue(fit.isTrustworthy(), "an open-loop log pins every parameter down: " + fit.warnings());
        assertTrue(fit.outputErrorRms < 2.0, "open-loop error a small multiple of the noise, got " + fit.outputErrorRms);
    }

    @Test
    void invariant_aNoisyFlywheelLogIsRecoveredWithinStatedTolerances() {
        // +-20 ticks/s uniform: about 0.7% of free speed, a REV hub velocity reading on a 28 cpr motor.
        FirstOrderFit.Samples log = synthesize(FLYWHEEL, 12.0, 12.0, 20.0, 3000, 9);

        FirstOrderFit.Result fit = FirstOrderFit.fit(log, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY, true);

        assertEquals(2800.0, fit.model.freeSpeed, 2800.0 * 0.03, "free speed within 3%");
        assertEquals(eulerTimeConstant(0.25), fit.model.timeConstantSeconds, eulerTimeConstant(0.25) * 0.10,
                "time constant within 10%");
        assertEquals(0.04, fit.model.coulombFrictionPower, 0.04 * 0.20, "friction within 20%");
    }

    /**
     * The case that showed a one-frame fit is not good enough: this robot's own flywheel model (2800 ticks/s,
     * a 6 s time constant from {@code FLYWHEEL_ACCEL_TPS2 = 467}) with +-15 ticks/s of noise. Over one frame
     * the velocity changes by about 9 ticks/s - less than the noise - and a one-frame fit comes out at 2409
     * ticks/s. A regression test: it fails if the fit ever goes back to single-frame differences.
     */
    @Test
    void invariant_aSlowNoisyFlywheelIsRecoveredDespiteTinyPerFrameChanges() {
        FirstOrderFit.Model slowFlywheel = new FirstOrderFit.Model(2800.0, 2800.0 / 467.0, 0.05);
        FirstOrderFit.Samples log = synthesize(slowFlywheel, 12.0, 12.0, 15.0, 3000, 36596, true);

        FirstOrderFit.Result fit = FirstOrderFit.fit(log, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY, true);

        assertEquals(2800.0, fit.model.freeSpeed, 2800.0 * 0.03, "free speed within 3%");
        assertEquals(eulerTimeConstant(2800.0 / 467.0), fit.model.timeConstantSeconds,
                0.10 * eulerTimeConstant(2800.0 / 467.0), "time constant within 10%");
        assertEquals(0.05, fit.model.coulombFrictionPower, 0.05 * 0.20, "friction within 20%");
    }

    /**
     * A robot run on a tired battery: the plant saw 11 V, the log says so, and the fit must still report
     * the free speed at the nominal 12 V - not 11/12 of it.
     */
    @Test
    void invariant_theFreeSpeedIsReportedAtNominalVoltageWhenTheLogHasTheBattery() {
        FirstOrderFit.Samples log = synthesize(DRIVE_AXIS, 11.0, 12.0, 0.0, 1500, 13);

        FirstOrderFit.Result fit = FirstOrderFit.fit(log, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY, true);

        assertEquals(60.0, fit.model.freeSpeed, 60.0 * 0.01, "not 11/12 of it, which would be 8% low");
    }

    @Test
    void theWindowLimitsWhichFramesAreFitted() {
        FirstOrderFit.Samples log = synthesize(DRIVE_AXIS, 12.0, 12.0, 0.0, 1500, 3);

        FirstOrderFit.Result fit = FirstOrderFit.fit(log, 5.0, 17.0, true);

        assertTrue(fit.windowStartSeconds >= 5.0 && fit.windowEndSeconds <= 17.0,
                "window " + fit.windowStartSeconds + " to " + fit.windowEndSeconds);
        assertEquals(60.0, fit.model.freeSpeed, 60.0 * 0.01);
    }

    @Test
    void aGapInTheLogIsNotFittedAcross() {
        FirstOrderFit.Samples log = synthesize(DRIVE_AXIS, 12.0, 12.0, 0.0, 1500, 3);
        double[] time = log.timeSeconds.clone();
        for (int k = 700; k < time.length; k++) {
            time[k] += 2.0;                          // the log stalled for two seconds at frame 700
        }
        FirstOrderFit.Samples gapped = new FirstOrderFit.Samples(time, log.power, log.velocity, null, 12.0);

        FirstOrderFit.Result fit = FirstOrderFit.fit(gapped, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY, true);

        assertTrue(fit.windowsSkipped >= 1, "the windows spanning the stall are skipped");
        assertEquals(60.0, fit.model.freeSpeed, 60.0 * 0.01, "and the fit is unharmed by it");
    }

    @Test
    void theSimsOwnModelWithoutFrictionScoresWorseOnAFrictionLogThanTheFit() {
        FirstOrderFit.Samples log = synthesize(DRIVE_AXIS, 12.0, 12.0, 0.5, 2000, 21);
        FirstOrderFit.Model frictionless = new FirstOrderFit.Model(60.0, 0.12, 0.0);

        FirstOrderFit.Result fit = FirstOrderFit.fit(log, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY, true);
        double frictionlessError = FirstOrderFit.outputErrorRms(frictionless, log,
                fit.windowStartSeconds, fit.windowEndSeconds);

        assertTrue(fit.outputErrorRms < frictionlessError,
                "fit " + fit.outputErrorRms + " should beat the frictionless model " + frictionlessError);
    }

    @Test
    void aLogWhosePowerAndVelocityDisagreeInSignIsRefused() {
        FirstOrderFit.Samples log = synthesize(DRIVE_AXIS, 12.0, 12.0, 0.0, 1500, 3);
        double[] flipped = log.velocity.clone();
        for (int k = 0; k < flipped.length; k++) {
            flipped[k] = -flipped[k];
        }
        FirstOrderFit.Samples wrong = new FirstOrderFit.Samples(log.timeSeconds, log.power, flipped, null, 12.0);

        // Flipping the velocity turns the power gain negative while the decay stays a decay. A negative (or
        // zero, or non-finite) gain is no plant at all, so the fit refuses it instead of reporting a negative
        // top speed.
        IllegalArgumentException refused = assertThrows(IllegalArgumentException.class, () ->
                FirstOrderFit.fit(wrong, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY, false));
        assertTrue(refused.getMessage().contains("power term"), refused.getMessage());
    }

    @Test
    void tooShortALogIsRefused() {
        FirstOrderFit.Samples log = synthesize(DRIVE_AXIS, 12.0, 12.0, 0.0, 10, 3);
        assertThrows(IllegalArgumentException.class,
                () -> FirstOrderFit.fit(log, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY, true));
    }

    /**
     * Overlapping windows are not independent: 150 frames give 140 windows but only 14 that share no frame,
     * fewer than {@link FirstOrderFit#MIN_INDEPENDENT_WINDOWS}, so the fit is refused.
     */
    @Test
    void aLogWithManyOverlappingButFewIndependentWindowsIsRefused() {
        FirstOrderFit.Samples log = synthesize(DRIVE_AXIS, 12.0, 12.0, 0.0, 150, 3);
        assertThrows(IllegalArgumentException.class,
                () -> FirstOrderFit.fit(log, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY, true));
    }

    /** One glitch frame must not widen the rest band: it is a percentile, not the maximum. */
    @Test
    void oneGlitchFrameDoesNotMoveTheRestBand() {
        FirstOrderFit.Samples log = synthesize(DRIVE_AXIS, 12.0, 12.0, 0.0, 1500, 3);
        double clean = FirstOrderFit.restBand(log, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);
        double[] glitched = log.velocity.clone();
        glitched[400] = 10000.0;
        FirstOrderFit.Samples withGlitch = new FirstOrderFit.Samples(log.timeSeconds, log.power, glitched, null, 12.0);

        assertEquals(clean, FirstOrderFit.restBand(withGlitch, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY),
                clean * 0.01);
    }

    /** The rest band comes from the requested time range, not the whole log. */
    @Test
    void theRestBandComesFromTheRequestedTimeRange() {
        FirstOrderFit.Samples log = synthesize(DRIVE_AXIS, 12.0, 12.0, 0.0, 1500, 3);
        double[] slowAfterTen = log.velocity.clone();
        for (int k = 0; k < slowAfterTen.length; k++) {
            if (log.timeSeconds[k] > 10.0) {
                slowAfterTen[k] *= 0.1;
            }
        }
        FirstOrderFit.Samples slow = new FirstOrderFit.Samples(log.timeSeconds, log.power, slowAfterTen, null, 12.0);

        double whole = FirstOrderFit.restBand(slow, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);
        double late = FirstOrderFit.restBand(slow, 10.0, Double.POSITIVE_INFINITY);
        assertEquals(0.1 * whole, late, 0.02 * whole);
    }

    /**
     * The robot's own flywheel for the closed-loop cases: 2800 ticks/s, the 6 s time constant from
     * {@code FLYWHEEL_ACCEL_TPS2 = 467}, 5% friction. With this robot's gains (kP 0.01, kF 0.00055) the loop
     * is stable: per frame the error shrinks by a factor of about {@code 1 - dt (1 + freeSpeed kP) / tau}
     * = 0.90. (A 0.25 s flywheel with the same kP is not: that factor is about -1.3, and the loop falls into
     * a bang-bang limit cycle, which is not the case these tests are about.)
     */
    private static final FirstOrderFit.Model ROBOT_FLYWHEEL = new FirstOrderFit.Model(2800.0, 2800.0 / 467.0, 0.05);
    private static final double FLYWHEEL_KP = 0.01;
    private static final double FLYWHEEL_KF = 0.00055;

    /**
     * The closed-loop trap, exact form: a velocity PIDF holding ONE setpoint, never saturating. Then on every
     * frame {@code u = kP (setpoint - v) + kF setpoint}, so the window sums obey
     * {@code sum(dt u) = (kP + kF) setpoint * sum(dt direction) - kP * sum(dt v)} exactly (the flywheel
     * always spins forward, so direction is 1): the power column IS a combination of the other two. The data
     * cannot separate the parameters, and the fit must refuse, naming why.
     */
    @Test
    void invariant_aClosedLoopLogHeldAtOneSetpointIsRefused() {
        // Setpoint 1500 ticks/s (kF * setpoint = 0.83, well inside full power), starting at its steady speed.
        FirstOrderFit.Samples log = synthesizeClosedLoop(ROBOT_FLYWHEEL, FLYWHEEL_KP, FLYWHEEL_KF, t -> 1500.0,
                1523.0, 15.0, 3000, 5);
        assertEquals(0, saturatedFrames(log), "the premise: the controller never saturates");

        IllegalArgumentException refused = assertThrows(IllegalArgumentException.class,
                () -> FirstOrderFit.fit(log, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY, true));
        assertTrue(refused.getMessage().contains("combination of the columns before it"), refused.getMessage());
    }

    /**
     * The closed-loop trap, near form: the same controller, its setpoint wandering +-50 ticks/s, never
     * saturating. Now the power column is only NEARLY a combination of the others, so least squares
     * returns numbers - with a fine-looking fit and meaningless parameters. The fit must SAY so: warnings,
     * not silent numbers.
     */
    @Test
    void invariant_aClosedLoopLogNearOneSetpointIsFlaggedAsNotPinnedDown() {
        FirstOrderFit.Samples log = synthesizeClosedLoop(ROBOT_FLYWHEEL, FLYWHEEL_KP, FLYWHEEL_KF,
                t -> 1500.0 + 50.0 * Math.sin(0.5 * t), 1523.0, 15.0, 3000, 5);
        assertEquals(0, saturatedFrames(log), "the premise: the controller never saturates");

        FirstOrderFit.Result fit = FirstOrderFit.fit(log, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY, true);

        assertFalse(fit.isTrustworthy(), "closed-loop data near one setpoint must be flagged; condition "
                + fit.conditionNumber + ", free speed " + fit.model.freeSpeed + " +- " + fit.freeSpeedStandardError);
    }

    /**
     * The same controller, spinning up from rest to a few different setpoints: it saturates at full power
     * on the way up and settles at different powers, which separates the columns. The fit recovers the
     * plant and is not flagged - the warning tracks the data, not the word "closed loop".
     */
    @Test
    void invariant_aClosedLoopLogThatSaturatesAndChangesSetpointIsRecovered() {
        FirstOrderFit.Samples log = synthesizeClosedLoop(ROBOT_FLYWHEEL, FLYWHEEL_KP, FLYWHEEL_KF,
                t -> {
                    double phase = t % 12.0;
                    return phase < 4.0 ? 2200.0 : (phase < 8.0 ? 1200.0 : (phase < 9.0 ? 0.0 : 1700.0));
                }, 0.0, 15.0, 3000, 5);
        assertTrue(saturatedFrames(log) > 0, "the premise: the controller saturates on spin-up");

        FirstOrderFit.Result fit = FirstOrderFit.fit(log, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY, true);

        double tau = ROBOT_FLYWHEEL.timeConstantSeconds;
        assertEquals(2800.0, fit.model.freeSpeed, 2800.0 * 0.03, "free speed within 3%");
        assertEquals(eulerTimeConstant(tau), fit.model.timeConstantSeconds, eulerTimeConstant(tau) * 0.10,
                "time constant within 10%");
        assertEquals(0.05, fit.model.coulombFrictionPower, 0.05 * 0.20, "friction within 20%");
        assertTrue(fit.isTrustworthy(), "saturation and setpoint changes pin the plant down: " + fit.warnings());
    }

    /** {@return how many frames of {@code log} have the power at full, +-1: where the controller saturated} */
    private static int saturatedFrames(FirstOrderFit.Samples log) {
        int count = 0;
        for (double power : log.power) {
            if (Math.abs(power) >= 1.0) {
                count++;
            }
        }
        return count;
    }

    /** Every parameter carries a standard error, and it is small on a clean open-loop log. */
    @Test
    void everyParameterCarriesAStandardError() {
        FirstOrderFit.Samples log = synthesize(DRIVE_AXIS, 12.0, 12.0, 1.0, 3000, 5);

        FirstOrderFit.Result fit = FirstOrderFit.fit(log, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY, true);

        assertTrue(fit.freeSpeedStandardError > 0.0 && fit.freeSpeedStandardError < 0.03 * 60.0,
                "free speed +- " + fit.freeSpeedStandardError);
        assertTrue(fit.timeConstantStandardError > 0.0 && fit.timeConstantStandardError < 0.1 * 0.12,
                "time constant +- " + fit.timeConstantStandardError);
        assertTrue(fit.frictionStandardError > 0.0 && fit.frictionStandardError < 0.02,
                "friction +- " + fit.frictionStandardError);
        assertTrue(fit.conditionNumber < FirstOrderFit.MAX_CONDITION_NUMBER, "condition " + fit.conditionNumber);
    }

    /**
     * A log from a known plant: power steps, a ramp and a reversal, a jittered 15-25 ms loop, and uniform
     * noise of +-{@code noise} on the logged velocity only.
     *
     * @param truth        the plant
     * @param plantVolts   the battery the plant actually ran on
     * @param nominalVolts the voltage the model's free speed is quoted at
     * @param noise        half-width of the uniform velocity noise, velocity units
     * @param frames       how many frames
     * @param seed         the noise and jitter seed
     */
    static FirstOrderFit.Samples synthesize(FirstOrderFit.Model truth, double plantVolts, double nominalVolts,
            double noise, int frames, int seed) {
        return synthesize(truth, plantVolts, nominalVolts, noise, frames, seed, false);
    }

    /**
     * As above; {@code flywheelSchedule} swaps the power schedule for the way a flywheel is really driven -
     * spin-up levels between 0.5 and 0.9, one stop, never reversed - which keeps the velocity in a narrow band
     * and is exactly where noise hurts a fit most.
     */
    static FirstOrderFit.Samples synthesize(FirstOrderFit.Model truth, double plantVolts, double nominalVolts,
            double noise, int frames, int seed, boolean flywheelSchedule) {
        SeededRandom random = new SeededRandom(seed);
        double[] time = new double[frames];
        double[] power = new double[frames];
        double[] measured = new double[frames];
        double[] volts = new double[frames];
        double t = 0.0;
        double v = 0.0;
        for (int k = 0; k < frames; k++) {
            time[k] = t;
            power[k] = flywheelSchedule ? flywheelSchedule(t) : schedule(t);
            volts[k] = plantVolts;
            measured[k] = v + (noise == 0.0 ? 0.0 : random.nextBetween(-noise, noise));
            double dt = random.nextBetween(0.015, 0.025);
            v = stepContinuousPlant(truth, v, power[k] * plantVolts / nominalVolts, dt);
            t += dt;
        }
        return new FirstOrderFit.Samples(time, power, measured, volts, nominalVolts);
    }

    /**
     * A log from a known plant driven by a velocity PIDF controller the way the robot's flywheel is: every
     * frame the controller reads the NOISY measured velocity, computes {@code kP (setpoint - v) + kF setpoint}
     * (no I or D), and that unclamped value is what is logged - while the motor, like the SDK's, clips it to
     * [-1, 1]. The fit is given the clipped power, which is what a caller must do with a real log.
     *
     * @param setpoint      the setpoint as a function of log time, velocity units
     * @param startVelocity the plant's velocity at frame 0, velocity units
     */
    static FirstOrderFit.Samples synthesizeClosedLoop(FirstOrderFit.Model truth, double kP, double kF,
            java.util.function.DoubleUnaryOperator setpoint, double startVelocity, double noise, int frames, int seed) {
        SeededRandom random = new SeededRandom(seed);
        double[] time = new double[frames];
        double[] appliedPower = new double[frames];
        double[] measured = new double[frames];
        double t = 0.0;
        double v = startVelocity;
        for (int k = 0; k < frames; k++) {
            time[k] = t;
            measured[k] = v + random.nextBetween(-noise, noise);
            double target = setpoint.applyAsDouble(t);
            double controllerOutput = kP * (target - measured[k]) + kF * target;
            appliedPower[k] = Math.max(-1.0, Math.min(1.0, controllerOutput));
            double dt = random.nextBetween(0.015, 0.025);
            v = stepContinuousPlant(truth, v, appliedPower[k], dt);
            t += dt;
        }
        return new FirstOrderFit.Samples(time, appliedPower, measured, null, 12.0);
    }

    /**
     * Advance the continuous plant {@code dv/dt = (F (u - f sign v) - v) / tau} by {@code dt} seconds, in
     * {@link #SUBSTEPS} Euler steps: fine enough to be the continuous plant, to far below the test noise.
     */
    static double stepContinuousPlant(FirstOrderFit.Model truth, double v, double effectivePower, double dt) {
        double h = dt / SUBSTEPS;
        for (int s = 0; s < SUBSTEPS; s++) {
            double friction = truth.freeSpeed * truth.coulombFrictionPower * Math.signum(v);
            v += h / truth.timeConstantSeconds * (truth.freeSpeed * effectivePower - v - friction);
        }
        return v;
    }

    /** 0.9, 0.5, off, 0.7, repeating every 12 s: a flywheel held at a few shot speeds. */
    private static double flywheelSchedule(double t) {
        double phase = t % 12.0;
        return phase < 4.0 ? 0.9 : (phase < 8.0 ? 0.5 : (phase < 9.0 ? 0.0 : 0.7));
    }

    /** Full, half, off, a reversal, and a slow ramp, repeating every 8 s. */
    private static double schedule(double t) {
        double phase = t % 8.0;
        if (phase < 1.5) {
            return 1.0;
        } else if (phase < 3.0) {
            return 0.5;
        } else if (phase < 3.5) {
            return 0.0;
        } else if (phase < 5.0) {
            return -0.8;
        } else if (phase < 5.5) {
            return 0.0;
        }
        return (phase - 5.5) / 2.5;          // 0 to 1 over the last 2.5 s
    }
}
