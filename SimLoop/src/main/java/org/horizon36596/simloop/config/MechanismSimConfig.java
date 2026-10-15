package org.horizon36596.simloop.config;

/**
 * Season-glue interface: the parameterized first-order dynamics + end-limits of one 1-DOF mechanism,
 * supplied by the season layer in {@code TeamCode}.
 *
 * <p>The core never hard-codes a season's mechanism constants; it asks for them through this interface,
 * so a {@code Mechanism1DofPlant} is fully described by these five numbers. They are the calibration
 * surface: a sim-to-real calibration fits {@link #timeConstant()} and {@link #maxSpeed()} to real-robot
 * data without touching the plant.
 *
 * <p><b>Two plants read this interface, and two of the numbers mean different things in each.</b>
 * {@code Mechanism1DofPlant} is power-driven: the motor's power sets a target <i>speed</i>, and the time
 * constant is how fast the speed chases it. {@code PositionalServoPlant} (through
 * {@link PositionalServoSimConfig}) is command-driven: the servo's command sets a target <i>position</i>,
 * and the time constant is how fast the position chases it. Each method below says what it means for
 * both.
 *
 * <p>Units are the mechanism's own position unit (inches for a linear slide, radians for an arm) and
 * seconds; the plant is unit-agnostic as long as every value here shares one position unit.
 */
public interface MechanismSimConfig {

    /**
     * First-order time constant (s, &gt; 0): the single pole of the mechanism's response.
     *
     * <ul>
     *   <li>For {@code Mechanism1DofPlant} it is the <b>velocity</b> pole: after a power step the speed
     *       reaches ~63% of its new steady value in one time constant and settles in ~4–5. Position is
     *       the integral of that speed, so it keeps moving for as long as the power is held.</li>
     *   <li>For {@code PositionalServoPlant} it is the <b>position</b> pole: after a command step the
     *       joint covers ~63% of the remaining distance in one time constant, unless the travel-rate
     *       ceiling is limiting it.</li>
     * </ul>
     *
     * @return the time constant, in seconds; strictly positive
     */
    double timeConstant();

    /**
     * Top speed (position units/s, &gt; 0).
     *
     * <ul>
     *   <li>For {@code Mechanism1DofPlant} it is the speed the mechanism settles at under full <i>net</i>
     *       power (commanded power minus {@link #gravityHoldPowerFraction()}, clamped to {@code [-1, 1]});
     *       a smaller net power settles at that fraction of it. It stands in for the actuator's
     *       force/current ceiling.</li>
     *   <li>For {@code PositionalServoPlant} it is the travel-rate ceiling: a large command step ramps at
     *       this rate rather than jumping, the way a real servo slews.</li>
     * </ul>
     *
     * @return the top speed, in the mechanism's own position units per second; strictly positive
     */
    double maxSpeed();

    /** {@return the lower end-limit, in the mechanism's own position units}: position can never integrate below this under any command */
    double minPosition();

    /** {@return the upper end-limit, in the mechanism's own position units}: position can never integrate above this under any command */
    double maxPosition();

    /**
     * How much of the motor's full effort the mechanism's own weight takes away, as a fraction in
     * {@code [0, 1)} — a vertical slide carrying a load needs a continuous power just to stay where it is,
     * and gets nothing back when it lets go.
     *
     * <p><b>Sign convention: always report a POSITIVE number.</b> The plant subtracts it, so gravity always
     * pulls toward {@link #minPosition()}. A mechanism gravity does not load — a turret, a horizontal
     * extension, a flywheel — reports {@code 0.0}, and the plant then behaves exactly as it did before this
     * method existed.
     *
     * <p>This is still parameterized first-order dynamics, not rigid-body physics: one scalar
     * standing for "weight costs this much effort", not a force model. It is the number a gravity
     * feedforward in the robot's own controller has to cancel, which is exactly why the plant has to model
     * it — a plant with no gravity holds any height at zero power, so a controller with no feedforward would
     * look perfect in sim and sag on the robot.
     *
     * @return the held-power fraction, unitless, in {@code [0, 1)}
     */
    double gravityHoldPowerFraction();
}
