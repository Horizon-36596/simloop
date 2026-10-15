package org.horizon36596.simloop.fakehardware;

import com.qualcomm.robotcore.exception.TargetPositionNotSetException;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

/**
 * Fake external encoder: the plant &rarr; sensor &rarr; subsystem seam for a 1-DOF mechanism. A mechanism's
 * subsystem closes its own loop on an <b>external encoder</b> that measures the true carriage/joint position
 * (not RUN_TO_POSITION); this fake models that encoder. Its count is <b>injected</b> each tick with the
 * mechanism's simulated position from a {@link org.horizon36596.simloop.plant.Mechanism1DofPlant}
 * (via {@link #setEncoderPosition(double)}) — NOT integrated from a commanded power. That is the whole point
 * of a dedicated device: the actuating {@link FakeMotor}s integrate their own (end-limit-unaware) position,
 * so the subsystem must read the plant's clamped position through a separate sensor, exactly as on hardware
 * where the encoder reports the real shaft and the same robot code runs in sim and on the robot.
 *
 * <p>It presents the full {@link DcMotorEx} surface (so SolversLib's {@code Motor}/{@code Motor.Encoder}
 * wrapper resolves and reads it unchanged, like a through-bore encoder wired to a motor-encoder port), but
 * it is a <b>sensor</b>: {@code setPower} is stored for API fidelity yet drives nothing — the shaft position
 * comes only from injection. Velocity is derived by finite-differencing the injected position across
 * {@link #update(double)} ticks (mirroring how the SDK derives encoder velocity from counts over time), so
 * it never reads the plant's modelled velocity as a feedback sensor.
 *
 * <h2>What the port stores, and what it does with it</h2>
 * Everything a real motor port stores is stored here and read back as the SDK reports it: direction, run
 * mode, zero-power behaviour, target position and tolerance, motor type, enable state and current alert.
 * Three of those change what the port reports:
 * <ul>
 *   <li><b>Direction</b> negates the reported count and velocity, as {@code DcMotorImpl} does.</li>
 *   <li><b>{@code STOP_AND_RESET_ENCODER}</b> zeroes the reported count at the moment the mode is entered,
 *       as the real controller's reset command zeroes whichever encoder is plugged into the port. The
 *       mechanism is driven by other motors and keeps moving, so the count carries on from zero; the
 *       injected position stays absolute and the zero is kept as an offset subtracted on read.</li>
 *   <li><b>{@code RUN_TO_POSITION}</b> makes {@link #isBusy()} report whether the count is outside the
 *       tolerance of the target, as the hub does. Nothing moves toward the target, because there is no motor
 *       behind this port; the mechanism's own motors move it, and the count follows the injected position.</li>
 * </ul>
 *
 * <p>Season-agnostic core: depends only on the SDK {@link DcMotorEx} surface. Deterministic: position
 * changes only via injection and velocity only in {@link #update(double)} off the injected tick
 * {@code deltaTime}, never a wall-clock; getters are pure reads. Every {@link DcMotorEx} method is modelled
 * or fails loud — none returns a plausible-but-wrong value.
 */
public class FakeExternalEncoder extends AbstractFakeDevice implements DcMotorEx {

    /**
     * The RUN_TO_POSITION tolerance a fresh real motor port reports, in encoder ticks: {@code DcMotorImplEx}'s
     * constructor sets its {@code targetPositionTolerance} field to 5 (read off the SDK 12.0.0 bytecode).
     */
    private static final int DEFAULT_TARGET_POSITION_TOLERANCE_TICKS = 5;

    private final double motorTypeMaxVelocityTicksPerSecond;
    private final double ticksPerRev;

    // Injected truth (the plant's clamped position, in encoder ticks) + finite-differenced velocity.
    private double positionTicks = 0.0;
    private double prevPositionTicks = 0.0;
    // False until the first real tick has recorded a previous position. Without it the first tick would
    // difference against the 0.0 above, so an encoder injected at 800 ticks before its first update would
    // report a velocity spike of 800 / dt instead of standing still.
    private boolean havePreviousPosition = false;
    private double velocityTicksPerSecond = 0.0;
    // The injected position, in encoder ticks, at the last STOP_AND_RESET_ENCODER; subtracted on read.
    private double positionTicksAtLastReset = 0.0;

    private double power = 0.0;
    private Direction direction = Direction.FORWARD;
    private ZeroPowerBehavior zeroPowerBehavior = ZeroPowerBehavior.FLOAT;
    private RunMode mode = RunMode.RUN_WITHOUT_ENCODER;
    // The target as the hub holds it, in raw encoder ticks: DcMotorImpl negates it for a REVERSE handle when
    // it is sent, so a later direction change does not move it.
    private int hubTargetPositionTicks = 0;
    // The tolerance the hub holds, in ticks: the handle's tolerance as it was when the target was last sent.
    // DcMotorImplEx sends the tolerance only inside setTargetPosition, so setting it alone changes nothing.
    private int hubTargetPositionToleranceTicks = DEFAULT_TARGET_POSITION_TOLERANCE_TICKS;
    // False until setTargetPosition is first called. The hub refuses to enable a channel in RUN_TO_POSITION
    // that has never been given a target, and the SDK turns that refusal into TargetPositionNotSetException.
    private boolean targetPositionHasBeenSet = false;
    // The tolerance stored on the handle (DcMotorImplEx's own field), in ticks; sent by the next target.
    private int targetPositionToleranceTicks = DEFAULT_TARGET_POSITION_TOLERANCE_TICKS;
    // The motor type last handed to setMotorType, stored as a private copy; null until something sets one.
    private MotorConfigurationType motorTypeSetByCaller = null;
    private double currentAlertAmps = 0.0;
    private boolean motorEnabled = true;

    /**
     * Builds an encoder reading zero ticks at zero velocity.
     *
     * @param motorTypeMaxVelocityTicksPerSecond the mechanism's free-speed shaft rate (ticks/s). This is an
     *                                    actuator concept the SolversLib {@code Motor} ctor reads via
     *                                    {@link #getMotorType()} — it exists ONLY to keep that type coherent;
     *                                    a sensor never senses with it and the RawPower loop never touches it.
     * @param ticksPerRev                 encoder counts per output-shaft revolution — the real device CPR,
     *                                    so {@code Motor.getCPR()} and {@link #getVelocity(AngleUnit)} read
     *                                    true.
     */
    public FakeExternalEncoder(double motorTypeMaxVelocityTicksPerSecond, double ticksPerRev) {
        if (ticksPerRev <= 0) {
            throw new IllegalArgumentException("ticksPerRev must be positive, got " + ticksPerRev);
        }
        this.motorTypeMaxVelocityTicksPerSecond = motorTypeMaxVelocityTicksPerSecond;
        this.ticksPerRev = ticksPerRev;
    }

    /**
     * Inject the mechanism's current position (in encoder ticks) — the harness calls this each tick with
     * {@code plant.getPosition() * ticksPerUnit}. Reads immediately (position is not integrated); the next
     * {@link #update(double)} finite-differences it into a velocity.
     *
     * @param ticks the mechanism's absolute count from now on, in encoder ticks, before direction and before
     *              any {@code STOP_AND_RESET_ENCODER} offset
     * @throws IllegalArgumentException if {@code ticks} is non-finite (a caller bug that would poison the
     *                                  count the subsystem closes its loop on).
     */
    public void setEncoderPosition(double ticks) {
        if (!Double.isFinite(ticks)) {
            throw new IllegalArgumentException("injected encoder position must be finite, got " + ticks);
        }
        this.positionTicks = ticks;
    }

    @Override
    public void update(double deltaTime) {
        // A non-finite or non-positive deltaTime is not a real sim tick (the sim clock only moves forward);
        // treat it as no time passing so a bad dt cannot inject a NaN/±Inf velocity.
        if (!Double.isFinite(deltaTime) || deltaTime <= 0.0) {
            return;
        }
        // Derive velocity from the change in injected position over the tick (how the SDK gets encoder
        // velocity from counts) rather than reading the plant's modelled velocity as a feedback sensor. The
        // first tick has nothing to difference against, so it only records where the encoder is.
        if (havePreviousPosition) {
            velocityTicksPerSecond = (positionTicks - prevPositionTicks) / deltaTime;
        } else {
            velocityTicksPerSecond = 0.0;
            havePreviousPosition = true;
        }
        prevPositionTicks = positionTicks;
    }

    // ---- DcMotorSimple ----
    @Override public Direction getDirection() { return direction; }
    @Override public void setDirection(Direction direction) { this.direction = direction; }
    @Override public double getPower() { return power; }

    /**
     * Stored for API fidelity, but a sensor drives nothing: position comes only from injection. Like a real
     * power command it re-enables the channel, except in {@code STOP_AND_RESET_ENCODER}, where the real
     * controller discards power commands (see {@link FakeMotor#setPower(double)}).
     */
    @Override public void setPower(double power) {
        this.power = Math.max(-1.0, Math.min(1.0, power));
        if (mode != RunMode.STOP_AND_RESET_ENCODER) {
            this.motorEnabled = true;
        }
    }

    // ---- DcMotor ----
    @Override public ZeroPowerBehavior getZeroPowerBehavior() { return zeroPowerBehavior; }
    @Override public void setZeroPowerBehavior(ZeroPowerBehavior behavior) { this.zeroPowerBehavior = behavior; }

    /**
     * The count since the last {@code STOP_AND_RESET_ENCODER}, in encoder ticks, truncated to an int like a
     * real count; negated for a {@code REVERSE} handle (SDK contract).
     */
    @Override public int getCurrentPosition() {
        double countSinceReset = positionTicks - positionTicksAtLastReset;
        double signed = (direction == Direction.REVERSE) ? -countSinceReset : countSinceReset;
        return (int) signed;
    }

    /**
     * Sets {@code FLOAT} and commands zero power, exactly as {@code DcMotorImpl.setPowerFloat} does.
     */
    @Override @Deprecated public void setPowerFloat() {
        setZeroPowerBehavior(ZeroPowerBehavior.FLOAT);
        setPower(0.0);
    }

    /** {@code DcMotorImpl.getPowerFloat}: {@code FLOAT} and a reported power of exactly zero. */
    @Override public boolean getPowerFloat() {
        return getZeroPowerBehavior() == ZeroPowerBehavior.FLOAT && getPower() == 0.0;
    }

    /** {@code DcMotorImpl.close}: floats the port (zero power, {@code FLOAT}). */
    @SuppressWarnings("deprecation")
    @Override public void close() {
        setPowerFloat();
    }

    /**
     * Restores {@code FORWARD} and forgets any motor type set with {@link #setMotorType}, so
     * {@link #getMotorType()} reports the constructor-derived type again. That is what the real method does:
     * {@code DcMotorImpl.resetDeviceConfigurationForOpMode} restores the direction, and
     * {@code LynxDcMotorController.resetDeviceConfigurationForOpMode(port)} calls
     * {@code setMotorType(port, internalMotorType)}, putting back the type the port was configured with. The
     * controller also restores its original PID coefficients, which this fake does not store. Power, mode,
     * target and count are left alone, as on the robot.
     */
    @Override public void resetDeviceConfigurationForOpMode() {
        setDirection(Direction.FORWARD);
        this.motorTypeSetByCaller = null;
    }

    /** {@code "Motor"}: a real robot configures an encoder-only port as a motor, so the SDK names it one. */
    @Override public String getDeviceName() { return "Motor"; }

    /** The controller's connection info plus {@code "; port N"}, in {@code DcMotorImpl}'s format. */
    @Override public String getConnectionInfo() { return "sim; port " + getPortNumber(); }

    /**
     * Sends a RUN_TO_POSITION target, in encoder ticks, with the handle's current tolerance, as
     * {@code DcMotorImplEx} does: the target is negated for a {@code REVERSE} handle <b>now</b>, and the
     * tolerance from {@link #setTargetPositionTolerance(int)} travels with it (it is only ever sent here).
     * {@link #isBusy()} compares against what was sent. Setting a target also lets {@link #setMode} enter
     * RUN_TO_POSITION (see there).
     *
     * @param position the target, in encoder ticks, in the handle's (logical) direction
     */
    @Override public void setTargetPosition(int position) {
        this.hubTargetPositionTicks = direction == Direction.REVERSE ? -position : position;
        this.hubTargetPositionToleranceTicks = targetPositionToleranceTicks;
        this.targetPositionHasBeenSet = true;
    }

    /**
     * The hub's target with the handle's current direction applied on read, as
     * {@code DcMotorImpl.getTargetPosition} does; 0 before any target is sent.
     *
     * @return the target, in encoder ticks, in the handle's current direction
     */
    @Override public int getTargetPosition() {
        return direction == Direction.REVERSE ? -hubTargetPositionTicks : hubTargetPositionTicks;
    }

    /**
     * True in {@link RunMode#RUN_TO_POSITION} while the port's raw count is more than the tolerance away from
     * the hub's target; false in every other mode. That is {@code LynxDcMotorController.isBusy}: outside
     * RUN_TO_POSITION it returns false without asking the hub, and inside it asks the hub whether the motor is
     * at its target. The hub compares in its own raw frame, with the tolerance sent with the last
     * {@link #setTargetPosition(int)}, so neither a later {@link #setTargetPositionTolerance(int)} nor a later
     * {@link #setDirection} changes the answer.
     */
    @Override public boolean isBusy() {
        if (mode != RunMode.RUN_TO_POSITION) {
            return false;
        }
        int rawCountSinceReset = (int) (positionTicks - positionTicksAtLastReset);
        return Math.abs((long) hubTargetPositionTicks - rawCountSinceReset) > hubTargetPositionToleranceTicks;
    }

    /**
     * Stores the run mode, migrating the three deprecated modes first as {@code DcMotorImpl.setMode} does.
     *
     * <p>Entering {@link RunMode#STOP_AND_RESET_ENCODER} zeroes the reported count (class javadoc). Setting
     * the mode the port is already in does nothing, because the real controller guards its whole mode change
     * on "the mode is not already this one" — so a second reset call while still in the mode is not a second
     * reset.
     *
     * <p>{@link RunMode#RUN_TO_POSITION} is accepted once a target has been set, and throws
     * {@link TargetPositionNotSetException} if none ever has, which is what a real port does. The path on the
     * robot: {@code LynxDcMotorController.setMotorMode} records the new mode, then re-issues the saved power,
     * which enables the channel; the hub NACKs that enable with {@code MOTOR_NOT_CONFIG_BEFORE_ENABLED}, and
     * {@code internalSetMotorEnable} rethrows the NACK as {@code TargetPositionNotSetException}. Because the
     * mode is recorded before the throw, {@link #getMode()} reports RUN_TO_POSITION afterwards here too. Once
     * in the mode, {@link #isBusy()} compares the count with the target; nothing drives toward it, because
     * this port has no motor behind it.
     *
     * @param mode the requested mode
     * @throws TargetPositionNotSetException if {@code mode} is {@link RunMode#RUN_TO_POSITION} and
     *                                       {@link #setTargetPosition} has never been called on this port
     */
    @Override public void setMode(RunMode mode) {
        RunMode migrated = mode.migrate();
        if (migrated == this.mode) {
            return;
        }
        this.mode = migrated;
        if (migrated == RunMode.STOP_AND_RESET_ENCODER) {
            this.positionTicksAtLastReset = positionTicks;
        }
        if (migrated == RunMode.RUN_TO_POSITION && !targetPositionHasBeenSet) {
            // The hub refused to enable the channel, so the enable state stays as it was.
            throw new TargetPositionNotSetException();
        }
        // Every real mode change ends in a power command that re-enables the channel; see FakeMotor.setMode.
        this.motorEnabled = true;
    }

    @Override public RunMode getMode() { return mode; }

    /**
     * The motor type configured on this port: the last one passed to {@link #setMotorType}, or, until
     * something sets one, a type derived from the constructor numbers. The derived type carries the real CPR,
     * with maxRPM chosen so {@code getAchieveableMaxTicksPerSecond()} equals
     * {@code motorTypeMaxVelocityTicksPerSecond}, because SolversLib's {@code Motor} reads both and an empty
     * type returns 0. Every call returns a fresh copy, where the real controller returns the object it stores;
     * see {@link #setMotorType} for why that matters.
     */
    @Override public MotorConfigurationType getMotorType() {
        if (motorTypeSetByCaller != null) {
            return motorTypeSetByCaller.clone();
        }
        MotorConfigurationType type = new MotorConfigurationType();
        type.setTicksPerRev(ticksPerRev);
        type.setMaxRPM(motorTypeMaxVelocityTicksPerSecond / ticksPerRev * 60.0);
        type.setAchieveableMaxRPMFraction(1.0);
        return type;
    }

    /**
     * Stores a copy of {@code motorType}, which {@link #getMotorType()} then reports.
     *
     * <p><b>Deliberate difference from the SDK:</b> the real controller keeps the very object it was handed
     * and {@code getMotorType} returns that same object, so on a robot editing either one changes the port's
     * configuration. This fake copies on the way in and on the way out, so a caller has to hand a changed
     * type back through this method. Robot code that edits a returned type in place and expects the port to
     * change behaves differently here.
     */
    @Override public void setMotorType(MotorConfigurationType motorType) {
        this.motorTypeSetByCaller = motorType.clone();
    }

    @Override public int getPortNumber() { return 0; }

    @Override
    public DcMotorController getController() {
        throw new UnsupportedOperationException(
                "FakeExternalEncoder has no DcMotorController in sim — read state via getCurrentPosition()");
    }

    // ---- DcMotorEx ----
    // A position sensor: velocity READS work (finite-differenced counts); closed-loop velocity/PIDF control
    // and current draw are not modelled (no-op / zero), exactly as on an encoder-only port.
    @Override public double getVelocity() {
        // Finite-differenced from the raw (unquantized) injected position, whereas getCurrentPosition()
        // truncates to int — so getVelocity() and Δ(getCurrentPosition())/Δt can differ by up to a tick of
        // quantization. Deliberate simplification: keeping velocity off the raw value avoids a spurious
        // ±1-tick jitter in the rate. A reset offset is a constant, so it never shows up in the rate.
        return (direction == Direction.REVERSE) ? -velocityTicksPerSecond : velocityTicksPerSecond;
    }
    @Override public double getVelocity(AngleUnit unit) {
        // Mirror the SDK: ticks/s -> rad/s (via the configured CPR), then convert with the UNNORMALIZED unit
        // — this is a rate, not an angle, so it must NOT be wrapped to [-pi,pi].
        double radiansPerSecond = getVelocity() / getMotorType().getTicksPerRev() * 2.0 * Math.PI;
        return unit.getUnnormalized().fromRadians(radiansPerSecond);
    }

    /**
     * Does not drive anything — this is a sensor — but makes the two changes a real velocity command makes
     * to the port: the mode becomes {@code RUN_USING_ENCODER} unless it is already that or
     * {@code RUN_TO_POSITION} (where {@code LynxDcMotorController.setMotorVelocity} leaves it alone, the rate
     * then being the speed cap), and the channel is enabled (see {@link FakeMotor#setVelocity(double)}).
     */
    @Override public void setVelocity(double angularRate) {
        if (mode != RunMode.RUN_USING_ENCODER && mode != RunMode.RUN_TO_POSITION) {
            this.mode = RunMode.RUN_USING_ENCODER;
        }
        this.motorEnabled = true;
    }
    /** Same as {@link #setVelocity(double)}; the rate itself drives nothing. */
    @Override public void setVelocity(double angularRate, AngleUnit unit) { setVelocity(angularRate); }
    @Override public void setMotorEnable() { motorEnabled = true; }
    @Override public void setMotorDisable() { motorEnabled = false; }
    @Override public boolean isMotorEnabled() { return motorEnabled; }
    @Override public void setPIDCoefficients(RunMode mode, PIDCoefficients pidCoefficients) { /* no closed loop */ }
    @Override public void setPIDFCoefficients(RunMode mode, PIDFCoefficients pidfCoefficients) { /* no-op */ }
    @Override public void setVelocityPIDFCoefficients(double p, double i, double d, double f) { /* no-op */ }
    @Override public void setPositionPIDFCoefficients(double p) { /* no-op */ }
    @Override public PIDCoefficients getPIDCoefficients(RunMode mode) { return new PIDCoefficients(0, 0, 0); }
    @Override public PIDFCoefficients getPIDFCoefficients(RunMode mode) { return new PIDFCoefficients(0, 0, 0, 0); }
    /**
     * Stores the RUN_TO_POSITION tolerance on the handle, in encoder ticks. As on {@code DcMotorImplEx} it
     * reaches the hub, and {@link #isBusy()}, only with the next {@link #setTargetPosition(int)}.
     */
    @Override public void setTargetPositionTolerance(int tolerance) { this.targetPositionToleranceTicks = tolerance; }
    /** {@return the stored tolerance in encoder ticks; 5 until set, as on a fresh {@code DcMotorImplEx}} */
    @Override public int getTargetPositionTolerance() { return targetPositionToleranceTicks; }
    /** Always zero: there is no motor on this port drawing current. */
    @Override public double getCurrent(CurrentUnit unit) { return 0.0; }
    /** {@return the stored over-current threshold, converted into {@code unit}} */
    @Override public double getCurrentAlert(CurrentUnit unit) {
        return unit.convert(currentAlertAmps, CurrentUnit.AMPS);
    }
    /** Stores the over-current threshold, converting it from {@code unit} to amps first. */
    @Override public void setCurrentAlert(double current, CurrentUnit unit) {
        this.currentAlertAmps = CurrentUnit.AMPS.convert(current, unit);
    }
    /** Always false: the current is always zero, so it never reaches any threshold. */
    @Override public boolean isOverCurrent() { return false; }
}
