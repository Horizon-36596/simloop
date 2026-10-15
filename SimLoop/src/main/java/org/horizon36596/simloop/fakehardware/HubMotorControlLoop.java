package org.horizon36596.simloop.fakehardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

/**
 * The closed loops a REV Control Hub or Expansion Hub runs in its own firmware for one motor port, as
 * {@link FakeMotor} models them in {@link DcMotor.RunMode#RUN_USING_ENCODER} and
 * {@link DcMotor.RunMode#RUN_TO_POSITION}.
 *
 * <p><b>Structure: EXACT (from the SDK source).</b> {@code LynxDcMotorController.internalSetPIDFCoefficients}
 * says it outright: "our fw runs a position loop, that then runs a velocity loop". So RUN_TO_POSITION is a
 * proportional position loop whose output is a velocity target, limited by the speed the SDK sent, feeding
 * the same PIDF velocity loop RUN_USING_ENCODER uses. That is also why the SDK refuses I, D or F on the
 * position loop.
 *
 * <p><b>Arithmetic: APPROXIMATION.</b> The firmware is closed source. The equations here are the plain
 * reading of that structure, with the units and rates stated as named constants below so a real-hub
 * measurement can replace each one. Every constant in this class is labelled APPROXIMATION; see
 * {@code SimLoop/docs/run-to-position.md} for what a real-hub check would measure.
 * <pre>
 *   position loop:  velocityTarget = clamp(positionP * (target - position), -speedLimit, +speedLimit)
 *   velocity loop:  error  = velocityTarget - measuredVelocity                 (ticks/s)
 *                   integral += I * error * cycles        (clamped to +/- FULL_SCALE_OUTPUT)
 *                   out    = P * error + integral + D * (error - previousError) / cycles + F * velocityTarget
 *                   power  = clamp(out / FULL_SCALE_OUTPUT, -1, 1)
 * </pre>
 * where {@code cycles} is the elapsed time over {@link #CONTROL_PERIOD_SECONDS}.
 *
 * <p>Deterministic: state changes only in {@link #velocityLoopPower} and {@link #reset()}; no clock is read.
 */
final class HubMotorControlLoop {

    /**
     * APPROXIMATION: how often the hub firmware runs one pass of its motor loops, in seconds (10 ms, 100 Hz).
     * REV does not publish it. It sets how fast the I term accumulates, since the firmware adds I times the
     * error once per pass. Chosen so the default gains give velocity tracking that settles in a few hundred
     * milliseconds with light overshoot, which is how teams describe the stock hub; not measured.
     */
    static final double CONTROL_PERIOD_SECONDS = 0.010;

    /**
     * APPROXIMATION: the loop output that means full power, in firmware units. 32767 is the SDK's full-scale
     * constant-power command ({@code LynxSetMotorConstantPowerCommand}), and the FTC PIDF tuning rule
     * {@code F = 32767 / maxTicksPerSecond} only gives full power at free speed if the velocity loop's output
     * is in these units, with its error in ticks per second. Strong public evidence, but not in the SDK source.
     */
    static final double FULL_SCALE_OUTPUT = 32767.0;

    /**
     * APPROXIMATION: the velocity-loop gains a fresh hub reports for a motor type that declares none (every
     * goBILDA type in SDK 12.0.0 declares none): P = 10, I = 3, D = 0, F = 0, algorithm PIDF. These are the
     * values teams read back from {@code getPIDFCoefficients(RUN_USING_ENCODER)}; they live in the firmware,
     * not the SDK.
     *
     * @return a fresh copy of the default velocity coefficients
     */
    static PIDFCoefficients defaultVelocityCoefficients() {
        return new PIDFCoefficients(10.0, 3.0, 0.0, 0.0, MotorControlAlgorithm.PIDF);
    }

    /**
     * APPROXIMATION: the position-loop gain a fresh hub reports, P = 10 (velocity target in ticks per second
     * per tick of position error), algorithm PIDF. Like the velocity gains, it lives in the firmware. 10 is the
     * position P the SDK's own {@code RevRoboticsCoreHexMotor} type declares (with its legacy algorithm); that
     * the PIDF firmware's default for other motors is also 10 is unconfirmed.
     *
     * @return a fresh copy of the default position coefficients
     */
    static PIDFCoefficients defaultPositionCoefficients() {
        return new PIDFCoefficients(10.0, 0.0, 0.0, 0.0, MotorControlAlgorithm.PIDF);
    }

    /** The velocity loop's gains, as the hub stores them. */
    private PIDFCoefficients velocityCoefficients = defaultVelocityCoefficients();
    /** The position loop's gains, as the hub stores them; only P is used. */
    private PIDFCoefficients positionCoefficients = defaultPositionCoefficients();

    /** The velocity loop's accumulated I term, firmware output units; cleared by {@link #reset()}. */
    private double integralOutput = 0.0;
    /** The velocity error on the previous pass, ticks/second, for the D term. */
    private double previousErrorTicksPerSecond = 0.0;
    /** Whether {@link #previousErrorTicksPerSecond} holds a real pass yet (the first pass has no D term). */
    private boolean hasPreviousError = false;

    /** A loop with the hub's default gains and no memory, as on a freshly powered hub. */
    HubMotorControlLoop() {
    }

    /**
     * The coefficients stored for a closed-loop mode.
     *
     * @param mode {@link DcMotor.RunMode#RUN_USING_ENCODER} or {@link DcMotor.RunMode#RUN_TO_POSITION}
     * @return a copy, so a caller cannot edit what the loop runs with
     */
    PIDFCoefficients coefficients(DcMotor.RunMode mode) {
        return new PIDFCoefficients(mode == DcMotor.RunMode.RUN_TO_POSITION
                ? positionCoefficients : velocityCoefficients);
    }

    /**
     * Replaces the coefficients for a closed-loop mode. The caller has already applied the SDK's validation.
     *
     * @param mode         {@link DcMotor.RunMode#RUN_USING_ENCODER} or {@link DcMotor.RunMode#RUN_TO_POSITION}
     * @param coefficients the new coefficients, already quantized the way the hub stores them; copied
     */
    void setCoefficients(DcMotor.RunMode mode, PIDFCoefficients coefficients) {
        PIDFCoefficients copy = new PIDFCoefficients(coefficients);
        if (mode == DcMotor.RunMode.RUN_TO_POSITION) {
            positionCoefficients = copy;
        } else {
            velocityCoefficients = copy;
        }
    }

    /** Puts both modes' coefficients back to the hub defaults. */
    void restoreDefaultCoefficients() {
        velocityCoefficients = defaultVelocityCoefficients();
        positionCoefficients = defaultPositionCoefficients();
    }

    /**
     * Forgets the velocity loop's integral and previous error. APPROXIMATION: the firmware's own reset points
     * are not published; this fake resets whenever the loop stops driving (mode change, disabled channel,
     * encoder reset), so a new move never inherits an old move's wind-up.
     */
    void reset() {
        integralOutput = 0.0;
        previousErrorTicksPerSecond = 0.0;
        hasPreviousError = false;
    }

    /**
     * The position loop: a velocity target proportional to the position error, limited to the speed the SDK
     * sent. Only P is used, as for the SDK's PIDF algorithm (the SDK refuses I, D and F here); a LegacyPID
     * position I or D is stored and reported but not modelled.
     *
     * @param targetTicks            the hub's target, in raw encoder ticks (direction already applied)
     * @param measuredTicks          the port's raw encoder count, in ticks
     * @param speedLimitTicksPerSecond the largest velocity target allowed, in ticks/s; non-negative
     * @return the velocity target, in raw ticks/s
     */
    double positionLoopVelocityTarget(double targetTicks, double measuredTicks, double speedLimitTicksPerSecond) {
        double velocityTarget = positionCoefficients.p * (targetTicks - measuredTicks);
        return clamp(velocityTarget, speedLimitTicksPerSecond);
    }

    /**
     * One pass of the velocity loop.
     *
     * @param targetTicksPerSecond   the velocity target, in raw ticks/s
     * @param measuredTicksPerSecond the port's measured velocity, in raw ticks/s
     * @param elapsedSeconds         how long this pass stands for, in seconds; positive
     * @return the power the firmware applies to the windings, in [-1, 1], raw (physical) direction
     */
    double velocityLoopPower(double targetTicksPerSecond, double measuredTicksPerSecond, double elapsedSeconds) {
        double cycles = elapsedSeconds / CONTROL_PERIOD_SECONDS;
        double error = targetTicksPerSecond - measuredTicksPerSecond;
        integralOutput = clamp(integralOutput + velocityCoefficients.i * error * cycles, FULL_SCALE_OUTPUT);
        double derivative = hasPreviousError
                ? velocityCoefficients.d * (error - previousErrorTicksPerSecond) / cycles
                : 0.0;
        previousErrorTicksPerSecond = error;
        hasPreviousError = true;
        double output = velocityCoefficients.p * error + integralOutput + derivative
                + velocityCoefficients.f * targetTicksPerSecond;
        return clamp(output / FULL_SCALE_OUTPUT, 1.0);
    }

    /**
     * Symmetric clamp.
     *
     * @param value the value
     * @param limit the largest magnitude allowed; non-negative
     * @return {@code value} clamped into {@code [-limit, limit]}
     */
    private static double clamp(double value, double limit) {
        return Math.max(-limit, Math.min(limit, value));
    }
}
