package org.horizon36596.simloop.fakehardware;

import com.qualcomm.robotcore.exception.TargetPositionNotSetException;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

import java.util.Locale;

/**
 * First-order drive-motor plant, seeded from PsiKit's test {@code FakeMotor}, behind the REV hub's motor
 * controller. Models the power reaching the windings -> speed with first-order lag, then integrates encoder
 * position:
 * <pre>
 *   drive = clamp(windingPower + externalTorqueFraction, -1, 1)
 *   speed += (drive - speed) * (1 - exp(-maxAccel * deltaTime))
 *   if (|speed| &lt; eps &amp;&amp; |drive| &lt; eps) speed = 0
 *   positionTicks += shaftSpeedFraction * maxVelocityInTicksPerSecond * deltaTime
 * </pre>
 * where {@code shaftSpeedFraction} is {@code speed} unless something outside the motor has taken
 * authority over the shaft -- see {@link #setMeasuredShaftSpeedFraction(double)} -- and
 * {@code externalTorqueFraction} is a load such as gravity on an arm, see
 * {@link #setExternalTorqueFraction(double)}. {@code windingPower} depends on the run mode (next section).
 *
 * <h2>Run modes: who decides the winding power</h2>
 * <ul>
 *   <li><b>{@code RUN_WITHOUT_ENCODER}</b>: the power last commanded, open loop. One step per
 *       {@link #update(double)}.</li>
 *   <li><b>{@code RUN_USING_ENCODER}</b>: the hub's velocity loop chases a velocity target, either
 *       {@link #setVelocity(double)} or {@code power * maxTicksPerSecond} from {@link #setPower(double)}.</li>
 *   <li><b>{@code RUN_TO_POSITION}</b>: the hub's position loop chases {@link #setTargetPosition(int)}, with
 *       {@code |power|} capping the speed, and keeps servoing once there: it holds, it does not cut power.
 *       {@link #isBusy()} is true while the count is more than the tolerance from the target.</li>
 *   <li><b>{@code STOP_AND_RESET_ENCODER}</b>: nothing drives, and the count is held at zero.</li>
 * </ul>
 * In the two closed-loop modes {@link #update(double)} runs the firmware loops of {@code HubMotorControlLoop}
 * in sub-steps no longer than its control period. The SDK side of all this (what each call sends, the
 * exceptions, the defaults, the isBusy rule, direction and encoder resets) is EXACT, read off the SDK
 * 12.0.0 sources. The firmware side (gains' units, loop rate, velocity measurement) is an APPROXIMATION;
 * {@code SimLoop/docs/run-to-position.md} lists which is which and how to check each on a real hub.
 *
 * <p>The lag step is the EXACT solution of {@code d(speed)/dt = maxAccel * (physicalPower - speed)} over one
 * tick with the power held constant, the same form the mechanism plants use. It replaced the explicit
 * Euler step {@code speed += (physicalPower - speed) * maxAccel * deltaTime}, which overshoots the commanded
 * power whenever {@code maxAccel * deltaTime > 1} and diverges above 2; the exact step never overshoots,
 * whatever the tick length.
 *
 * {@code speed} is a unitless fraction of max wheel velocity in [-1, 1]; multiply by
 * {@link #getMaxVelocityTicksPerSecond()} for ticks/s. Pure plant: the modelled shaft moves only in
 * {@link #update(double)}; the SDK getters are pure reads.
 *
 * <h2>Two accessor pairs</h2>
 * {@code speed}/{@code positionTicks} track the motor's true PHYSICAL shaft motion -- direction is applied
 * once, on the way in, because a {@code REVERSE}-configured motor commanded {@code +power} really does spin
 * its shaft the other way. A real {@code DcMotorEx} then applies direction a SECOND time on the way back
 * out, so software always sees "positive power moves the encoder positive" regardless of which way the
 * motor happens to be bolted in. As on the robot, direction is applied to each command <b>when it is
 * sent</b> ({@code setPower}, {@code setVelocity}, {@code setTargetPosition}) and to each reading when it is
 * read, so flipping the direction of a running motor changes what it reports but not what it is doing
 * until the next command. This class matches that with two differently-named accessor pairs:
 * <ul>
 *   <li><b>SDK-reported</b> — {@link #getCurrentPosition()}, {@link #getVelocityTicksPerSecond()} (and
 *       everything built on them: {@link #getVelocity()}, {@link #getVelocity(AngleUnit)}). Direction
 *       applied on read, matching real {@code DcMotorEx}. This is what season/robot code should read.</li>
 *   <li><b>Physical</b> — {@link #getPhysicalPositionTicks()}, {@link #getPhysicalVelocityTicksPerSecond()}.
 *       Direction already baked into how the shaft moved; NOT re-applied on read. This is what a plant that
 *       models the mechanism (e.g. {@code MecanumDrivePlant}, which combines it with a separate physical
 *       mounting-sign fact) needs to read, because it cares about which way the shaft actually turned, not
 *       about the software abstraction layered on top of it.</li>
 * </ul>
 *
 * <p><b>The two pairs normally describe the same shaft, and one case breaks that.</b> Ordinarily the only
 * difference between the pairs is the direction sign, so a reader may treat the physical pair as "the
 * unsigned version of" the reported pair. That stops being true the moment
 * {@link #setEncoderMeasuringAnotherMechanism(double, double)} is used: the reported pair then describes a
 * DIFFERENT mechanism while the physical pair still describes this motor's shaft. Anything generic that
 * walks a motor and dumps both — telemetry, a replay comparator — is reading two mechanisms, not one
 * number twice. See the next section.
 *
 * <h2>When the encoder on this port is not counting this motor's shaft</h2>
 * An FTC motor port carries a motor <i>and</i> an encoder input, and nothing makes them the same mechanism.
 * A team short of encoder ports plugs mechanism A's encoder into the port whose motor drives mechanism B,
 * and then reads A's position through B's motor handle. Robots really are wired this way, so the fake has
 * to be wirable that way too.
 *
 * <p>{@link #setEncoderMeasuringAnotherMechanism(double, double)} models that. It replaces what this port
 * REPORTS — {@link #getCurrentPosition()} and {@link #getVelocityTicksPerSecond()}, because on a real REV
 * port both are derived from the one encoder channel that is physically plugged in. Redirecting only the
 * position would hand a control loop a position from one shaft and a velocity from another.
 *
 * <p>It changes nothing else. This motor still integrates its own power, still models its own current, and
 * still reports its own shaft through {@link #getPhysicalPositionTicks()} and
 * {@link #getPhysicalVelocityTicksPerSecond()}. That is not a simplification, it is what the hardware does:
 * the motor whose port was borrowed really does keep spinning its own shaft while the number its port
 * reports belongs to somebody else.
 *
 * <h2>Zero-power behaviour: stored and reported, not modelled</h2>
 * {@link #setZeroPowerBehavior} is stored and read back exactly as on a real motor, and it feeds
 * {@link #getPowerFloat()}, but it does <b>not</b> change how the shaft slows down. At zero power the
 * first-order lag above decays the speed toward zero at the same {@code maxAccel} rate whether the
 * behaviour is {@code BRAKE} or {@code FLOAT}. That decay is a braking-like stop; a {@code FLOAT} motor on
 * a real robot coasts noticeably longer, and this fake does not model that coast. A robot whose
 * behaviour depends on the difference (a drivetrain coasting through a stop, an arm drifting down under
 * {@code FLOAT}) is not checked by this fake.
 *
 * <h2>Current draw</h2>
 * {@link #getCurrent(CurrentUnit)} models a brushed DC motor's back-EMF relationship rather than returning
 * zero, because current is how robot code detects a hardstop or a jam. A motor with a mechanism on it must
 * be told how fast its shaft is really turning — see {@link #setMeasuredShaftSpeedFraction(double)}.
 */
public class FakeMotor extends AbstractFakeDevice implements DcMotorEx {

    /** Below this share of free speed, with this little drive, the shaft is taken as stopped (unitless). */
    private static final double EPS = 0.02;

    /** goBILDA 5203 312-RPM output CPR; used by the 2-arg ctor for tests that don't assert angular velocity. */
    private static final double DEFAULT_TICKS_PER_REV = 537.7;

    /**
     * Stall current at 12 V, amps, for the <b>stand-in</b> motor the short constructors assume: a goBILDA
     * 5203-series Yellow Jacket, off its published sheet. Every 5203 ratio shares one RS-555 motor, so the
     * figure does not change with the gearbox — which is why it makes a serviceable default for a fake that
     * nobody is asking about amps.
     *
     * <p><b>Any caller whose robot code reads current must pass its own datasheet numbers</b> through the
     * five-argument constructor rather than inheriting these. This is a default so that {@code getCurrent()}
     * has something honest to return, not a claim about what motor is on the other end. Same standing as
     * {@link #DEFAULT_TICKS_PER_REV}.
     */
    private static final double DEFAULT_STALL_CURRENT_AMPS = 9.2;

    /** No-load current at 12 V, amps, for the same stand-in motor and off the same sheet as above. */
    private static final double DEFAULT_FREE_CURRENT_AMPS = 0.25;

    /**
     * The RUN_TO_POSITION tolerance a fresh real motor reports, in encoder ticks: {@code DcMotorImplEx}'s
     * constructor sets its {@code targetPositionTolerance} field to 5 (read off the SDK 12.0.0 bytecode).
     */
    private static final int DEFAULT_TARGET_POSITION_TOLERANCE_TICKS = 5;

    /** The tolerance range {@code LynxSetMotorTargetPositionCommand} accepts, in ticks (an unsigned 16-bit field). */
    private static final int MAX_TARGET_POSITION_TOLERANCE_TICKS = 65535;

    /** The velocity range {@code LynxSetMotorTargetVelocityCommand} accepts, in ticks/s (a signed 16-bit field). */
    private static final int MAX_VELOCITY_COMMAND_TICKS_PER_SECOND = 32767;

    /** The scale the hub stores PID coefficients at: a 32-bit integer of coefficient times 65536 (SDK exact). */
    private static final double COEFFICIENT_FIXED_POINT_SCALE = 65536.0;

    /** The only port a fake motor sits on; named because the SDK's messages quote it. */
    private static final int PORT_NUMBER = 0;

    /** Free speed of the physical motor, encoder ticks/second. */
    private final double maxVelocityInTicksPerSecond;
    /** How fast speed chases its drive, reciprocal seconds (the reciprocal of the lag's time constant). */
    private final double maxAccel;
    /** Encoder counts per output-shaft revolution of the physical motor. */
    private final double ticksPerRev;
    /** Datasheet stall current at 12 V, amps. */
    private final double stallCurrentAmps;
    /** Datasheet no-load current at 12 V, amps. */
    private final double freeCurrentAmps;

    /**
     * The power the SDK last sent, in the HUB's frame: direction already applied when it was sent, and
     * {@code |power|} in RUN_TO_POSITION, exactly as {@code LynxDcMotorController} caches it; {@code getPower()}
     * undoes the direction on read. Stored in this frame rather than the caller's because that is what the
     * robot does: flipping the direction of a running motor changes what {@code getPower()} reports, not
     * which way the shaft turns.
     */
    private double power = 0.0;
    /**
     * The velocity target the SDK last sent to the hub, raw ticks/second, used by the two closed-loop modes:
     * {@code setVelocity} sends it directly, {@code setPower} sends {@code power * maxTicksPerSecond}
     * (truncated, as the SDK does), and in RUN_TO_POSITION its magnitude is the speed cap.
     */
    private int velocityCommandTicksPerSecond = 0;
    /** The hub's RUN_TO_POSITION target, raw encoder ticks: direction applied when it was sent. */
    private int hubTargetPositionTicks = 0;
    /**
     * The tolerance the hub holds, ticks: the handle's tolerance as it was when the target was last sent.
     * {@code DcMotorImplEx} only sends the tolerance inside {@code setTargetPosition}, so changing the
     * tolerance alone changes nothing on the hub.
     */
    private int hubTargetPositionToleranceTicks = DEFAULT_TARGET_POSITION_TOLERANCE_TICKS;
    /**
     * False until the first {@code setTargetPosition}. The hub refuses to enable a channel in RUN_TO_POSITION
     * that has no target, and the SDK turns that refusal into {@link TargetPositionNotSetException}.
     * APPROXIMATION: nothing here clears it again, because whether the firmware forgets its target (on an
     * encoder reset, say) is not published.
     */
    private boolean hubTargetPositionSet = false;
    /** The hub firmware's closed loops for this port, with their coefficients and memory. */
    private final HubMotorControlLoop controlLoop = new HubMotorControlLoop();
    /** The power the hub's closed loop applied on the last sub-step of the last update, physical direction. */
    private double closedLoopWindingPower = 0.0;
    /** An outside torque on the shaft, signed fraction of stall torque, physical direction; see its setter. */
    private double externalTorqueFraction = 0.0;
    /** The shaft's own encoder count, physical ticks (direction not applied). */
    private double positionTicks = 0.0;
    /** The shaft's own modelled speed, fraction of free speed, physical direction. */
    private double speed = 0.0;
    /** The handle's direction, applied to each command when sent and to each reading when read. */
    private Direction direction = Direction.FORWARD;
    /** Stored and reported; not modelled (class javadoc). */
    private ZeroPowerBehavior zeroPowerBehavior = ZeroPowerBehavior.FLOAT;
    /** Over-current alert threshold, amps; 0 means none set. */
    private double currentAlert = 0.0;
    /** Whether the hub channel is enabled; a disabled channel drives nothing in any mode. */
    private boolean motorEnabled = true;
    /**
     * The run mode the SDK would report. Stored rather than pretended, because a real controller's mode is
     * readable and every mode changes what drives the shaft (class javadoc). See {@link #setMode}.
     */
    private RunMode mode = RunMode.RUN_WITHOUT_ENCODER;
    /** The tolerance stored on the HANDLE ({@code DcMotorImplEx}'s field), ticks; sent by the next target. */
    private int targetPositionToleranceTicks = DEFAULT_TARGET_POSITION_TOLERANCE_TICKS;
    /**
     * The motor type last handed to {@code setMotorType}, stored as a private copy. Null means nobody has set
     * one, and {@code getMotorType()} then reports the type derived from this plant's own constructor numbers.
     */
    private MotorConfigurationType motorTypeSetByCaller = null;
    /**
     * How fast the shaft is REALLY turning, fraction of free speed, when a plant has said so. Unset means
     * "nobody is holding this motor back, so its own modelled speed is the truth" — the drivetrain case. A
     * mechanism plant that end-limits or jams the shaft takes authority by calling
     * {@code setMeasuredShaftSpeedFraction} every tick.
     */
    private double measuredShaftSpeedFraction = 0.0;
    /** Whether {@link #measuredShaftSpeedFraction} has authority over the shaft. */
    private boolean measuredShaftSpeedFractionSet = false;
    /**
     * The count of an encoder wired into this port that counts a DIFFERENT mechanism than this motor drives
     * (class javadoc), physical ticks, absolute (before any reset offset).
     */
    private double anotherMechanismPositionTicks = 0.0;
    /** That other encoder's rate, physical ticks/second. */
    private double anotherMechanismVelocityTicksPerSecond = 0.0;
    /** Whether this port reports another mechanism's encoder; unset is the ordinary case, its own shaft. */
    private boolean anotherMechanismSet = false;
    /**
     * What the other mechanism's encoder read, physical ticks, at the last STOP_AND_RESET_ENCODER. The
     * injected count is that encoder's absolute count, but a real reset zeroes whichever encoder is plugged
     * into the port, so every later read reports the count minus this. Zero until a reset happens.
     */
    private double anotherMechanismTicksAtLastReset = 0.0;
    /**
     * True when a STOP_AND_RESET_ENCODER happened before any count had been injected, so there was no number
     * to take the offset from. The next injected count is then taken as the count at the reset: on the robot
     * the reset zeroed that encoder, so the first thing the glue reports is where it was zeroed.
     */
    private boolean anotherMechanismResetWaitingForFirstCount = false;

    /**
     * A motor with the default encoder resolution and the default current figures.
     *
     * @param maxVelocityInTicksPerSecond free speed, in encoder ticks/second; must be positive
     * @param maxAccel                    how fast speed chases commanded power, in reciprocal seconds; must be
     *                                    positive
     */
    public FakeMotor(double maxVelocityInTicksPerSecond, double maxAccel) {
        this(maxVelocityInTicksPerSecond, maxAccel, DEFAULT_TICKS_PER_REV);
    }

    /**
     * A motor with a stated encoder resolution and the default current figures.
     *
     * @param maxVelocityInTicksPerSecond free speed, in encoder ticks/second; must be positive
     * @param maxAccel                    how fast speed chases commanded power, in reciprocal seconds; must be
     *                                    positive
     * @param ticksPerRev encoder counts per output-shaft revolution — lets {@link #getVelocity(AngleUnit)}
     *                    return a real angular velocity (the harness passes the actual motor's CPR).
     */
    public FakeMotor(double maxVelocityInTicksPerSecond, double maxAccel, double ticksPerRev) {
        this(maxVelocityInTicksPerSecond, maxAccel, ticksPerRev,
                DEFAULT_STALL_CURRENT_AMPS, DEFAULT_FREE_CURRENT_AMPS);
    }

    /**
     * A motor with a stated encoder resolution and stated datasheet current figures.
     *
     * @param maxVelocityInTicksPerSecond free speed, in encoder ticks/second; must be finite and positive
     * @param maxAccel                    how fast speed chases commanded power, in reciprocal seconds (the
     *                                    reciprocal of the lag's time constant); must be finite and positive
     * @param ticksPerRev                 encoder counts per output-shaft revolution
     * @param stallCurrentAmps the motor's datasheet stall current at 12 V (&gt; 0) — the ceiling
     *                         {@link #getCurrent(CurrentUnit)} reports and the number a stall/jam threshold
     *                         is set below.
     * @param freeCurrentAmps  the motor's datasheet no-load current at 12 V (&ge; 0), what it draws spinning
     *                         against nothing but its own friction.
     * @throws IllegalArgumentException if any argument is outside the range stated for it
     */
    public FakeMotor(double maxVelocityInTicksPerSecond, double maxAccel, double ticksPerRev,
            double stallCurrentAmps, double freeCurrentAmps) {
        // Both of these are divided by or multiplied into every tick, so a zero, negative or non-finite value
        // is a caller bug that would otherwise surface much later as NaN power or a motor that never moves.
        if (!(maxVelocityInTicksPerSecond > 0) || Double.isInfinite(maxVelocityInTicksPerSecond)) {
            throw new IllegalArgumentException("maxVelocityInTicksPerSecond must be finite and positive, got "
                    + maxVelocityInTicksPerSecond);
        }
        if (!(maxAccel > 0) || Double.isInfinite(maxAccel)) {
            throw new IllegalArgumentException("maxAccel must be finite and positive, got " + maxAccel);
        }
        if (ticksPerRev <= 0) {
            throw new IllegalArgumentException("ticksPerRev must be positive, got " + ticksPerRev);
        }
        if (!(stallCurrentAmps > 0)) {
            throw new IllegalArgumentException("stallCurrentAmps must be positive, got " + stallCurrentAmps);
        }
        if (!(freeCurrentAmps >= 0) || freeCurrentAmps > stallCurrentAmps) {
            throw new IllegalArgumentException("freeCurrentAmps must be in [0, stallCurrentAmps], got "
                    + freeCurrentAmps);
        }
        this.maxVelocityInTicksPerSecond = maxVelocityInTicksPerSecond;
        this.maxAccel = maxAccel;
        this.ticksPerRev = ticksPerRev;
        this.stallCurrentAmps = stallCurrentAmps;
        this.freeCurrentAmps = freeCurrentAmps;
    }

    /**
     * Tell this motor how fast its shaft is REALLY turning, as a fraction of free speed in {@code [-1, 1]},
     * when something outside the motor decides that — a mechanism plant's end-stop, or a jam.
     *
     * <p><b>Why this exists.</b> A {@code FakeMotor} integrates its own unloaded speed and knows nothing
     * about the mechanism bolted to it, so a slide sitting on its bottom hardstop would still model a
     * happily spinning shaft and report near-zero current. Current is what a re-zero and a stall detector
     * read, so the motor has to be told. A drivetrain motor never calls this and keeps using its own
     * modelled speed.
     *
     * <p><b>Call it every tick once you start.</b> The value is used until it is replaced; it is not
     * consumed. The season glue calls this right beside the line that injects the plant's position into the
     * mechanism's external-encoder fake, so both halves of "what the plant knows" reach the hardware layer
     * in one place.
     *
     * @param fractionOfFreeSpeed the shaft's true speed over its free speed, clamped into {@code [-1, 1]};
     *                            e.g. {@code Mechanism1DofPlant.getVelocityFractionOfMaxSpeed()}. A
     *                            non-finite value is rejected rather than silently poisoning the current.
     */
    public void setMeasuredShaftSpeedFraction(double fractionOfFreeSpeed) {
        if (!Double.isFinite(fractionOfFreeSpeed)) {
            throw new IllegalArgumentException(
                    "measured shaft speed fraction must be finite, got " + fractionOfFreeSpeed);
        }
        this.measuredShaftSpeedFraction = Math.max(-1.0, Math.min(1.0, fractionOfFreeSpeed));
        this.measuredShaftSpeedFractionSet = true;
    }

    /**
     * Tell this port that the encoder plugged into it is counting a DIFFERENT mechanism than this motor
     * drives, and say what that encoder reads right now.
     *
     * <p><b>Why this exists.</b> See the class javadoc: an FTC motor port carries a motor and an encoder
     * input, and a team short of encoder ports wires mechanism A's encoder into the port whose motor drives
     * mechanism B, then reads A through B's handle. Without this the sim would report B's own shaft count to
     * code that is asking where A is, and A would read as frozen at zero while B ran.
     *
     * <p><b>Why position and velocity go in together.</b> A real REV port derives both from the one encoder
     * channel that is physically plugged into it. Redirecting only the position would give a control loop a
     * position from one shaft and a velocity from another, which is a robot that does not exist. Passing
     * both in one call also means there is no way to set one and forget the other.
     *
     * <p><b>What it does not change.</b> This motor goes on integrating its own power in
     * {@link #update(double)}, goes on modelling its own current, and goes on reporting its own shaft
     * through {@link #getPhysicalPositionTicks()} and {@link #getPhysicalVelocityTicksPerSecond()}. That is
     * the hardware being modelled, not a corner being cut: the motor whose port is borrowed really is still
     * turning.
     *
     * <p><b>Both numbers are PHYSICAL, and direction is still applied on read.</b> Pass what the encoder
     * actually counted; {@link #getCurrentPosition()} and {@link #getVelocityTicksPerSecond()} multiply by
     * this handle's direction sign the way a real {@code DcMotorEx} does, so a {@code REVERSE}
     * handle on this port reports both negated, exactly as it would on the robot.
     *
     * <p><b>Call it every tick once you start.</b> The values are used until they are replaced; they are not
     * consumed on read. That matches {@link #setMeasuredShaftSpeedFraction(double)} and is deliberate: a
     * value that expired on read would make the answer depend on the order in which callers happened to
     * read it. The cost is that a tick where the glue forgets to call this reports last tick's numbers and
     * looks like a slow-moving mechanism, so set it in the same place that steps the plant owning that
     * encoder, unconditionally, every tick.
     *
     * @param physicalTicks          the encoder's count in ticks, before direction is applied
     * @param physicalTicksPerSecond how fast that count is changing, in ticks/second, before direction is
     *                               applied. This is the OTHER mechanism's rate — do not scale it by this
     *                               motor's free speed. A non-finite value in either argument is rejected
     *                               rather than silently turning every read on this port into nonsense.
     */
    public void setEncoderMeasuringAnotherMechanism(double physicalTicks, double physicalTicksPerSecond) {
        if (!Double.isFinite(physicalTicks)) {
            throw new IllegalArgumentException(
                    "another mechanism's encoder position must be finite, got " + physicalTicks);
        }
        if (!Double.isFinite(physicalTicksPerSecond)) {
            throw new IllegalArgumentException(
                    "another mechanism's encoder velocity must be finite, got " + physicalTicksPerSecond);
        }
        if (anotherMechanismResetWaitingForFirstCount) {
            // A reset arrived before any count did (see the field); this first count is where it zeroed.
            this.anotherMechanismTicksAtLastReset = physicalTicks;
            this.anotherMechanismResetWaitingForFirstCount = false;
        }
        this.anotherMechanismPositionTicks = physicalTicks;
        this.anotherMechanismVelocityTicksPerSecond = physicalTicksPerSecond;
        this.anotherMechanismSet = true;
    }

    /**
     * Hand this port's reported position and velocity back to this motor's own encoder (the state a fresh
     * {@code FakeMotor} is in).
     *
     * <p>The count this motor kept while something else was being reported is still there and is what comes
     * back — it never stopped integrating, so clearing does not resume from where the other mechanism's
     * number left off.
     */
    public void clearEncoderMeasuringAnotherMechanism() {
        this.anotherMechanismPositionTicks = 0.0;
        this.anotherMechanismVelocityTicksPerSecond = 0.0;
        this.anotherMechanismSet = false;
        this.anotherMechanismTicksAtLastReset = 0.0;
        this.anotherMechanismResetWaitingForFirstCount = false;
    }

    /** {@return whether this port is reporting another mechanism's encoder rather than this motor's own shaft} */
    public boolean hasEncoderMeasuringAnotherMechanism() {
        return anotherMechanismSet;
    }

    /** Hand shaft-speed authority back to the motor's own model (the state a fresh {@code FakeMotor} is in). */
    public void clearMeasuredShaftSpeedFraction() {
        this.measuredShaftSpeedFraction = 0.0;
        this.measuredShaftSpeedFractionSet = false;
    }

    /**
     * Put an outside torque on the shaft — gravity on an arm or a lift, a spring, a rubber band — as a signed
     * fraction of this motor's stall torque, in the PHYSICAL direction: positive pushes the shaft toward
     * increasing {@link #getPhysicalPositionTicks()}. An arm that lifts toward positive ticks and needs 30% of
     * stall torque to hold level is {@code -0.3 * cos(angle)}.
     *
     * <p>In this plant's normalized units a torque fraction and a power fraction are the same number: a
     * brushed motor holding a load of 30% of stall torque still needs 30% power, which is exactly what the
     * shaft model does with it (class javadoc). So a motor at zero power under a load drifts, and a closed
     * loop has to push back. It is the same "effort tax" {@code Mechanism1DofPlant}'s gravity term is.
     *
     * <p><b>Call it every tick once you start</b> when the load depends on the pose (an arm), reading the
     * pose off {@link #getPhysicalPositionTicks()}; the value is used until replaced. It has no effect while
     * {@link #setMeasuredShaftSpeedFraction(double)} has authority over the shaft, because the plant that
     * injects the speed already accounts for its own load.
     *
     * @param fractionOfStallTorque the load, signed, physical direction, in {@code [-1, 1]}; finite
     * @throws IllegalArgumentException if the value is non-finite or outside {@code [-1, 1]} (a load past stall
     *                                  torque is one the motor could not hold at any power)
     */
    public void setExternalTorqueFraction(double fractionOfStallTorque) {
        if (!(fractionOfStallTorque >= -1.0 && fractionOfStallTorque <= 1.0)) {
            throw new IllegalArgumentException(
                    "external torque must be a finite fraction of stall torque in [-1, 1], got "
                            + fractionOfStallTorque);
        }
        this.externalTorqueFraction = fractionOfStallTorque;
    }

    /** {@return the outside torque on the shaft, a signed fraction of stall torque, physical direction; 0 by default} */
    public double getExternalTorqueFraction() {
        return externalTorqueFraction;
    }

    /**
     * Advance the motor by {@code deltaTime} seconds.
     *
     * <p>{@code RUN_WITHOUT_ENCODER} takes one step with the commanded power. {@code RUN_USING_ENCODER} and
     * {@code RUN_TO_POSITION} run the hub's loops ({@code HubMotorControlLoop}) in equal sub-steps no longer
     * than its 10 ms control period (an APPROXIMATION), each reading the port's encoder, choosing a
     * winding power and stepping the shaft. {@code STOP_AND_RESET_ENCODER} holds the shaft stopped at zero.
     *
     * @param deltaTime the tick length, in seconds; a closed-loop mode ignores a non-positive or non-finite one
     */
    @Override
    public void update(double deltaTime) {
        // Direction is applied ONCE, when a command is sent (setPower/setVelocity/setTargetPosition store the
        // hub-frame value), so every number below is PHYSICAL. It is re-applied only by the SDK-reported pair
        // (getCurrentPosition(), getVelocity()), on read, matching real DcMotorEx.
        // STOP_AND_RESET_ENCODER holds the shaft stopped and the count at zero for as long as it is the
        // mode: the real controller refuses to send a power command while in it, so nothing can start the
        // motor again until the mode changes. Returning early is how that reads most plainly.
        if (mode == RunMode.STOP_AND_RESET_ENCODER) {
            speed = 0.0;
            positionTicks = 0.0;
            // A plant's injected shaft speed is stale the moment the encoder is reset. This mode holds the
            // shaft still, so nothing outside can be measuring it turning; left set, it would make
            // getSpeed() and getCurrent() report a moving shaft on a motor that is stopped. A plant that
            // still has authority re-asserts it on its next tick, as it does every tick anyway.
            clearMeasuredShaftSpeedFraction();
            controlLoop.reset();
            closedLoopWindingPower = 0.0;
            return;
        }
        if (isClosedLoopMode()) {
            updateClosedLoop(deltaTime);
            return;
        }
        stepShaft(drivenPhysicalPower(), deltaTime);
    }

    /**
     * The closed-loop modes' tick: sub-steps of at most one firmware control period, so the loops run at
     * (roughly) the hub's own rate whatever the sim's tick. The step count is a pure function of
     * {@code deltaTime}, so two identical runs step identically.
     *
     * @param deltaTime the tick length, seconds
     */
    private void updateClosedLoop(double deltaTime) {
        if (!(deltaTime > 0.0) || Double.isInfinite(deltaTime)) {
            return;
        }
        // The small allowance keeps a tick that is a whole number of periods (0.02 s / 0.01 s, which is not
        // exact in binary) from gaining a spurious extra sub-step.
        int subSteps = Math.max(1,
                (int) Math.ceil(deltaTime / HubMotorControlLoop.CONTROL_PERIOD_SECONDS - 1e-9));
        double subStep = deltaTime / subSteps;
        for (int step = 0; step < subSteps; step++) {
            closedLoopWindingPower = closedLoopPowerForOneControlPass(subStep);
            stepShaft(closedLoopWindingPower, subStep);
        }
    }

    /**
     * One pass of the hub's firmware loops: the power it puts on the windings for the next
     * {@code elapsedSeconds}, physical direction. It reads the port's encoder, which is this motor's own
     * shaft or, when one is declared, another mechanism's (see
     * {@link #setEncoderMeasuringAnotherMechanism(double, double)}); a count injected once per tick is held
     * for every sub-step of that tick, as it would be for a loop reading it.
     *
     * @param elapsedSeconds how long this pass stands for, seconds
     * @return the winding power, in [-1, 1], physical direction
     */
    private double closedLoopPowerForOneControlPass(double elapsedSeconds) {
        if (!motorEnabled) {
            controlLoop.reset();
            return 0.0;
        }
        double velocityTarget = mode == RunMode.RUN_TO_POSITION
                ? controlLoop.positionLoopVelocityTarget(hubTargetPositionTicks, rawEncoderCount(),
                        Math.abs((double) velocityCommandTicksPerSecond))
                : velocityCommandTicksPerSecond;
        return controlLoop.velocityLoopPower(velocityTarget, countedVelocityTicksPerSecond(), elapsedSeconds);
    }

    /**
     * Steps the shaft model (class javadoc) by {@code deltaTime} seconds under a winding power, physical
     * direction. The outside torque is added to it and the sum clamped into {@code [-1, 1]}, so a load cannot
     * drive the shaft past free speed — the same clamp {@code Mechanism1DofPlant} applies.
     *
     * @param windingPhysicalPower the power on the windings, in [-1, 1], physical direction
     * @param deltaTime            how long to step, seconds
     */
    private void stepShaft(double windingPhysicalPower, double deltaTime) {
        double drive = Math.max(-1.0, Math.min(1.0, windingPhysicalPower + externalTorqueFraction));
        // Exact first-order step (class javadoc): the fraction of the remaining gap closed in one tick.
        // StrictMath rather than Math so the result is bit-identical on every JVM, which replay relies on.
        double fractionOfGapClosedThisTick = 1.0 - StrictMath.exp(-maxAccel * deltaTime);
        speed += (drive - speed) * fractionOfGapClosedThisTick;
        if (Math.abs(speed) < EPS && Math.abs(drive) < EPS) {
            speed = 0.0;
        }
        // The shaft's encoder counts what the shaft ACTUALLY did. When something outside the motor knows
        // that better than the motor's own model does (a mechanism plant on its hardstop, a jam), that
        // override is the truth for position, velocity and current alike -- a real motor encoder does not
        // keep counting while the shaft is held still.
        positionTicks += shaftSpeedFraction() * maxVelocityInTicksPerSecond * deltaTime;
    }

    /** {@return whether the hub, not the robot code, decides the winding power in the current mode} */
    private boolean isClosedLoopMode() {
        return mode == RunMode.RUN_USING_ENCODER || mode == RunMode.RUN_TO_POSITION;
    }

    /**
     * How fast the shaft is really turning, as a fraction of free speed in {@code [-1, 1]}: the value
     * {@link #setMeasuredShaftSpeedFraction(double)} injected if there is one, otherwise this motor's own
     * unloaded first-order model. Physical (direction already applied), see class javadoc.
     *
     * @return the shaft speed, fraction of free speed, physical direction
     */
    private double shaftSpeedFraction() {
        return measuredShaftSpeedFractionSet ? measuredShaftSpeedFraction : speed;
    }

    /** {@return the unitless shaft speed in [-1, 1] (fraction of max velocity); physical, see class javadoc} */
    public double getSpeed() {
        return shaftSpeedFraction();
    }

    /**
     * SDK-reported wheel velocity in encoder ticks/second: direction applied on read, so a REVERSE motor
     * commanded positive power reports positive, matching real {@code DcMotorEx}. Season code
     * should read this; a plant modelling the mechanism should read {@link #getPhysicalVelocityTicksPerSecond()}.
     * Unlike {@link #getVelocity()}, which reports whole ticks per second as the hub does, this is the
     * modelled rate before rounding.
     *
     * <p>Reports the other mechanism's rate when
     * {@link #setEncoderMeasuringAnotherMechanism(double, double)} has been used, because a real port
     * derives velocity from the same encoder channel it derives position from.
     *
     * @return the velocity in encoder ticks/second, with this motor's direction applied
     */
    public double getVelocityTicksPerSecond() {
        return reportedSign() * countedVelocityTicksPerSecond();
    }

    /**
     * What the encoder on this port is counting per second, in physical ticks/second: the other mechanism's
     * rate if one has been declared, otherwise this motor's own shaft rate. Physical — direction is
     * applied by the reader, see class javadoc.
     *
     * @return the counted rate, physical ticks/second
     */
    private double countedVelocityTicksPerSecond() {
        return anotherMechanismSet
                ? anotherMechanismVelocityTicksPerSecond
                : shaftSpeedFraction() * maxVelocityInTicksPerSecond;
    }

    /**
     * True modelled shaft velocity in encoder ticks/second, direction already baked in and NOT re-applied
     * (see class javadoc). This is what a plant that models the mechanism reads.
     *
     * @return the true shaft velocity in encoder ticks/second, direction already baked in
     */
    public double getPhysicalVelocityTicksPerSecond() {
        return shaftSpeedFraction() * maxVelocityInTicksPerSecond;
    }

    /** {@return the free speed this motor was built with, in encoder ticks/second} */
    public double getMaxVelocityTicksPerSecond() {
        return maxVelocityInTicksPerSecond;
    }

    /** {@return +1 for {@code FORWARD}, -1 for {@code REVERSE}: the SDK's direction-to-report-sign rule} */
    private double reportedSign() {
        return direction == Direction.REVERSE ? -1.0 : 1.0;
    }

    /**
     * The port's encoder count as the hub's position loop and "at target" test see it: whole raw ticks,
     * direction not applied. {@link #getCurrentPosition()} is this with the handle's direction applied.
     *
     * @return the count, whole raw ticks
     */
    private int rawEncoderCount() {
        return (int) countedPositionTicks();
    }

    /**
     * The power actually reaching the windings right now, direction already applied.
     *
     * <p>In {@code RUN_WITHOUT_ENCODER} that is whatever {@link #setPower(double)} commanded; in the two
     * closed-loop modes it is what the hub's loop applied on the last sub-step of the last
     * {@link #update(double)} (so a command sent since then shows up after the next update, as a loop output
     * would). Two things stop any of it reaching the shaft, and both leave {@link #getPower()} still
     * reporting the commanded value, which is exactly why this is a separate number. A disabled channel
     * cannot drive at all - on the real REV controller {@code setMotorDisable} sends
     * {@code LynxSetMotorChannelEnableCommand(false)}, and only a fresh power or velocity command turns it
     * back on, which is why {@link #setPower(double)} and {@link #setVelocity(double)} re-enable it here.
     * {@link RunMode#STOP_AND_RESET_ENCODER} does the same by refusing to send a power command while it is
     * the mode.
     *
     * <p>One method rather than the same condition written twice, because {@link #update(double)} and
     * {@link #getCurrent(CurrentUnit)} have to agree: a shaft that is not being driven cannot also be
     * drawing stall current. Getting that wrong would have a deliberate encoder reset look like a jam to
     * any current-based stall detector.
     *
     * @return the winding power, in [-1, 1], physical direction
     */
    private double drivenPhysicalPower() {
        if (!motorEnabled || mode == RunMode.STOP_AND_RESET_ENCODER) {
            return 0.0;
        }
        if (isClosedLoopMode()) {
            return closedLoopWindingPower;
        }
        // A NaN command reaches the hub as zero; see setPower. getPower() still reports the NaN.
        if (Double.isNaN(power)) {
            return 0.0;
        }
        return power;
    }

    /**
     * The power actually reaching the windings, in {@code [-1, 1]}, PHYSICAL direction (see the class
     * javadoc's two accessor pairs).
     *
     * <p><b>Why this exists.</b> In {@code RUN_TO_POSITION} and {@code RUN_USING_ENCODER} the hub, not the
     * robot code, decides the winding power, and {@link #getPower()} reports what the robot code sent — in
     * {@code RUN_TO_POSITION} only the speed cap. Anything outside this motor that needs to know how hard the
     * shaft is really being driven (a mechanism plant, a telemetry line) reads this instead. Note that
     * {@code Mechanism1DofPlant} reads {@code getPower()}, so it models open-loop power only; to simulate a
     * mechanism in a closed-loop mode, put its load on this motor with
     * {@link #setExternalTorqueFraction(double)}.
     *
     * @return the winding power: the commanded power in {@code RUN_WITHOUT_ENCODER}, the hub loop's output on
     *         the last sub-step of the last {@link #update(double)} in the closed-loop modes, and 0 while the
     *         channel is disabled or the encoder is resetting
     */
    public double getAppliedPhysicalPower() {
        return drivenPhysicalPower();
    }

    // ---- DcMotorSimple ----
    @Override public Direction getDirection() { return direction; }

    /**
     * Sets the direction applied to the NEXT command and to every reading from now on, as
     * {@code DcMotorImpl.setDirection} does: it sends nothing to the hub, so a motor already running keeps
     * doing what it was told until its next {@code setPower}, {@code setVelocity} or
     * {@code setTargetPosition}.
     */
    @Override public void setDirection(Direction direction) { this.direction = direction; }

    /**
     * The power last sent, as {@code DcMotorImpl.getPower} reports it: {@code |power|} in RUN_TO_POSITION,
     * otherwise with the handle's current direction applied.
     *
     * <p>After {@link #setVelocity(double)} it is that velocity over the port's configured maximum, which is
     * what the SDK reads back from the hub once its 500 ms power cache has lapsed (SDK exact, except that the
     * fake does not model the cache's first 500 ms).
     */
    @Override public double getPower() {
        return mode == RunMode.RUN_TO_POSITION ? Math.abs(power) : reportedSign() * power;
    }

    /**
     * Commands power, exactly as {@code DcMotorImpl.setPower} and {@code LynxDcMotorController} handle it.
     *
     * <ul>
     *   <li><b>{@code RUN_TO_POSITION}</b>: only {@code |power|} is sent, because the position loop decides
     *       the direction; it becomes the speed cap {@code |power| * maxTicksPerSecond}.</li>
     *   <li><b>{@code RUN_USING_ENCODER}</b>: {@code direction * power * maxTicksPerSecond}, truncated to whole
     *       ticks/s, becomes the velocity target.</li>
     *   <li><b>{@code RUN_WITHOUT_ENCODER}</b>: {@code direction * power} drives the windings open loop.</li>
     *   <li><b>{@code STOP_AND_RESET_ENCODER}</b>: the power is remembered and nothing is sent.</li>
     * </ul>
     * {@code maxTicksPerSecond} is the configured motor type's
     * {@code getAchieveableMaxTicksPerSecondRounded()}, as in the SDK.
     *
     * <p>A command that is sent also enables the channel. {@code LynxDcMotorController.internalSetMotorPower}
     * builds a command for the current mode and enables the channel <b>only if it built one</b>; the
     * {@code STOP_AND_RESET_ENCODER} branch is explicitly {@code command = null} ("setting motor power in this
     * mode doesn't do anything"). So on a real robot a power command is how a disabled motor comes back —
     * except while the encoder is being reset, where it is ignored outright and a disabled channel stays
     * disabled.
     *
     * <p><b>NaN.</b> {@code getPower()} reports it back, but the shaft is driven at zero. That is what the
     * real controller does: {@code internalSetMotorPower} runs the power through {@code Range.clip(p, -1, 1)},
     * which returns NaN unchanged because both of its comparisons are false; caches that NaN as the last
     * known power, which {@code getPower()} reads; and builds the hub command with a {@code double -> int}
     * cast, which turns NaN into 0. {@code Math.max}/{@code Math.min} below pass NaN through the same way,
     * and the command built from it is zero, so the modelled speed never becomes NaN.
     *
     * @param power the power, in {@code [-1, 1]}; clipped into that range
     * @throws TargetPositionNotSetException in {@code RUN_TO_POSITION} when no target has ever been set,
     *                                       because the hub refuses the enable that follows the command
     */
    @Override public void setPower(double power) {
        double sent = mode == RunMode.RUN_TO_POSITION ? Math.abs(power) : reportedSign() * power;
        this.power = Math.max(-1.0, Math.min(1.0, sent));
        if (mode == RunMode.STOP_AND_RESET_ENCODER) {
            return;
        }
        sendPowerCommand();
    }

    /**
     * What {@code LynxDcMotorController.internalSetMotorPower} sends for the cached power in the current
     * (non-reset) mode, then the channel enable that follows every sent command.
     */
    private void sendPowerCommand() {
        if (isClosedLoopMode()) {
            // Math.signum(p) * Range.scale(|p|, 0, 1, 0, max), then an (int) cast: truncation toward zero,
            // and NaN becomes 0.
            double scaled = power * configuredMaxTicksPerSecondForPower();
            this.velocityCommandTicksPerSecond = clampVelocityCommand((int) scaled);
        }
        enableChannel();
    }

    /**
     * Enables the channel, as {@code LynxDcMotorController.internalSetMotorEnable} does after every command it
     * sends. In {@code RUN_TO_POSITION} with no target ever set, the hub NACKs the enable
     * ({@code MOTOR_NOT_CONFIG_BEFORE_ENABLED}) and the SDK rethrows that as
     * {@link TargetPositionNotSetException}; the channel then stays off.
     */
    private void enableChannel() {
        if (mode == RunMode.RUN_TO_POSITION && !hubTargetPositionSet) {
            this.motorEnabled = false;
            throw new TargetPositionNotSetException();
        }
        this.motorEnabled = true;
    }

    /** {@return the configured motor type's rounded achievable max, ticks/s: what the SDK scales power by} */
    private int configuredMaxTicksPerSecondForPower() {
        return getMotorType().getAchieveableMaxTicksPerSecondRounded();
    }

    /**
     * Clips a velocity command to the hub's signed 16-bit field.
     *
     * @param ticksPerSecond the command, ticks/second
     * @return the command the hub can hold, ticks/second
     */
    private static int clampVelocityCommand(long ticksPerSecond) {
        return (int) Math.max(-MAX_VELOCITY_COMMAND_TICKS_PER_SECOND,
                Math.min(MAX_VELOCITY_COMMAND_TICKS_PER_SECOND, ticksPerSecond));
    }

    // ---- DcMotor ----
    @Override public ZeroPowerBehavior getZeroPowerBehavior() { return zeroPowerBehavior; }
    @Override public void setZeroPowerBehavior(ZeroPowerBehavior behavior) { this.zeroPowerBehavior = behavior; }

    /**
     * SDK-reported encoder position in ticks: direction applied on read, so a REVERSE motor commanded
     * positive power reports positive, matching real {@code DcMotorEx}. Season code should
     * read this; a plant modelling the mechanism should read {@link #getPhysicalPositionTicks()}.
     *
     * <p>Reports the value from {@link #setEncoderMeasuringAnotherMechanism(double, double)} when one has
     * been declared — the case where the encoder plugged into this port counts some other mechanism —
     * minus what that encoder read at the last {@link RunMode#STOP_AND_RESET_ENCODER}, because a real reset
     * zeroes whichever encoder is plugged into the port. A reset made before the first count was injected
     * zeroes at that first count. Direction is applied either way, because it is the handle that applies it,
     * not the encoder.
     */
    @Override public int getCurrentPosition() { return (int) (reportedSign() * countedPositionTicks()); }

    /**
     * What the encoder on this port has counted, in physical ticks: the other mechanism's count if one has
     * been declared, otherwise this motor's own shaft count. Physical — direction is applied by the
     * reader, see class javadoc.
     *
     * @return the counted position, physical ticks
     */
    private double countedPositionTicks() {
        return anotherMechanismSet
                ? anotherMechanismPositionTicks - anotherMechanismTicksAtLastReset
                : positionTicks;
    }

    /**
     * True modelled shaft position in encoder ticks, direction already baked in and NOT re-applied (see
     * class javadoc). This is what a plant that models the mechanism reads.
     *
     * @return the true shaft position in encoder ticks, direction already baked in
     */
    public double getPhysicalPositionTicks() { return positionTicks; }

    /**
     * Sets {@code FLOAT} and commands zero power, exactly as {@code DcMotorImpl.setPowerFloat} does — two
     * ordinary calls, so the channel re-enable and every other effect of {@link #setPower(double)} happen
     * here too.
     */
    @Override @Deprecated public void setPowerFloat() {
        setZeroPowerBehavior(ZeroPowerBehavior.FLOAT);
        setPower(0.0);
    }

    /**
     * {@code DcMotorImpl.getPowerFloat}: true when the zero-power behaviour is {@code FLOAT} <b>and</b> the
     * reported power is exactly zero. A {@code FLOAT} motor that is being driven is not floating.
     */
    @Override public boolean getPowerFloat() {
        return getZeroPowerBehavior() == ZeroPowerBehavior.FLOAT && getPower() == 0.0;
    }

    /** {@code DcMotorImpl.close}: floats the motor (zero power, {@code FLOAT}). */
    @SuppressWarnings("deprecation")
    @Override public void close() {
        setPowerFloat();
    }

    /**
     * Restores {@code FORWARD}, forgets any motor type set with {@link #setMotorType} (so
     * {@link #getMotorType()} reports the constructor-derived type again), and puts the hub's default PID
     * coefficients back.
     *
     * <p>That is what the real method does. {@code DcMotorImpl.resetDeviceConfigurationForOpMode} restores
     * the direction on the handle, then calls the controller's
     * {@code resetDeviceConfigurationForOpMode(port)}, which puts back the original PID coefficients (the ones
     * it read from the hub before anything was set) and calls {@code setMotorType(port, internalMotorType)},
     * the type the port was configured with. Power, mode, target and position are deliberately left alone,
     * as on the robot.
     */
    @Override public void resetDeviceConfigurationForOpMode() {
        setDirection(Direction.FORWARD);
        this.motorTypeSetByCaller = null;
        controlLoop.restoreDefaultCoefficients();
    }

    /** {@code "Motor"}, the string {@code DcMotorImpl.getDeviceName} reads out of the SDK's resources. */
    @Override public String getDeviceName() { return "Motor"; }

    /** The controller's connection info plus {@code "; port N"}, in {@code DcMotorImpl}'s format. */
    @Override public String getConnectionInfo() { return "sim; port " + getPortNumber(); }

    /**
     * Sends a RUN_TO_POSITION target, in encoder ticks, with the handle's current tolerance, exactly as
     * {@code DcMotorImplEx} does: the target is negated for a {@code REVERSE} handle <b>now</b>, and the
     * tolerance from {@link #setTargetPositionTolerance(int)} travels with it (it is only ever sent here).
     * Accepted in any mode; the hub acts on it only in RUN_TO_POSITION.
     *
     * @param position the target, in encoder ticks, in the handle's (logical) direction
     * @throws IllegalArgumentException {@code "illegal tolerance: N"} if the handle's tolerance is outside
     *                                  {@code [0, 65535]}, which {@code LynxSetMotorTargetPositionCommand}
     *                                  rejects; nothing is sent
     */
    @Override public void setTargetPosition(int position) {
        int tolerance = targetPositionToleranceTicks;
        if (tolerance < 0 || tolerance > MAX_TARGET_POSITION_TOLERANCE_TICKS) {
            throw new IllegalArgumentException(String.format("illegal tolerance: %d", tolerance));
        }
        this.hubTargetPositionTicks = direction == Direction.REVERSE ? -position : position;
        this.hubTargetPositionToleranceTicks = tolerance;
        this.hubTargetPositionSet = true;
    }

    /**
     * The hub's target with the handle's direction applied on read, as {@code DcMotorImpl.getTargetPosition}
     * does; 0 before any target is sent.
     *
     * @return the target, in encoder ticks, in the handle's current direction
     */
    @Override public int getTargetPosition() {
        return direction == Direction.REVERSE ? -hubTargetPositionTicks : hubTargetPositionTicks;
    }

    /**
     * In {@link RunMode#RUN_TO_POSITION}, true while the port's raw count is more than the tolerance away
     * from the hub's target; false in every other mode.
     *
     * <p>SDK exact: {@code LynxDcMotorController.isBusy} returns false outside RUN_TO_POSITION without asking
     * the hub, and inside it returns {@code !isAtTarget()} from the hub. The tolerance is the one sent with
     * the last {@link #setTargetPosition(int)}, not a later {@link #setTargetPositionTolerance(int)}. The
     * hub's own "at target" test is taken as {@code |target - count| <= tolerance} (APPROXIMATION: whether
     * the boundary is inclusive is firmware), and it says nothing about speed, so a motor passing through the
     * target at speed reads not busy for that instant. Being "not busy" does not stop the motor: the hub
     * keeps holding the target. (With bulk caching on, the SDK reads the hub's busy bit in every mode; this
     * fake assumes the hub reports not-busy outside RUN_TO_POSITION.)
     */
    @Override public boolean isBusy() {
        if (mode != RunMode.RUN_TO_POSITION) {
            return false;
        }
        long error = (long) hubTargetPositionTicks - rawEncoderCount();
        return Math.abs(error) > hubTargetPositionToleranceTicks;
    }

    /**
     * Sets the run mode as {@code LynxDcMotorController.setMotorMode} does. Setting the mode it is already in
     * does nothing at all, because the controller guards its whole body on "not already this mode".
     *
     * <p>{@link RunMode#STOP_AND_RESET_ENCODER} stops the shaft and zeroes the count: the controller commands
     * zero power and sends {@code LynxResetMotorEncoderCommand}. Note it deliberately does <b>not</b> clear
     * {@link #getPower()} — the real controller saves the previous power and re-issues it, so the API-visible
     * power survives a reset even though the shaft stops. This fake reproduces that, rather than the more
     * obvious and wrong "zero everything".
     *
     * <p>Every other mode <b>re-issues the last power in the new mode</b>, then enables the channel. That has
     * consequences a robot can see, all SDK exact:
     * <ul>
     *   <li>{@code setTargetPosition}, then {@code setMode(RUN_TO_POSITION)} starts moving at once, capped by
     *       the power already set.</li>
     *   <li>Entering RUN_TO_POSITION before any target has ever been set throws
     *       {@link TargetPositionNotSetException}: the hub NACKs the enable. The mode has already been
     *       recorded, so {@link #getMode()} reports RUN_TO_POSITION afterwards, with the channel off.</li>
     *   <li>Leaving RUN_TO_POSITION re-issues {@code |power|} in the hub's frame, so switching an arm from
     *       RUN_TO_POSITION to RUN_USING_ENCODER or RUN_WITHOUT_ENCODER without a new {@code setPower} drives
     *       it at that power, in the positive raw direction.</li>
     * </ul>
     * The target and tolerance are kept across mode changes and encoder resets; after a reset the target is
     * measured from the new zero. The hub's loop memory is cleared on every change (APPROXIMATION).
     *
     * @param mode the requested mode; the three deprecated modes are migrated first, as {@code DcMotorImpl}
     *             does via {@code RunMode.migrate()}
     * @throws TargetPositionNotSetException if {@code mode} is {@link RunMode#RUN_TO_POSITION}, the motor is
     *                                       not already in it, and no target has ever been set
     */
    @Override public void setMode(RunMode mode) {
        RunMode migrated = mode.migrate();
        if (migrated == this.mode) {
            return;
        }
        controlLoop.reset();
        this.closedLoopWindingPower = 0.0;
        this.mode = migrated;
        if (migrated != RunMode.STOP_AND_RESET_ENCODER) {
            // internalSetMotorPower(motor, prevPower, true) in the new mode: a command, then the enable.
            sendPowerCommand();
            return;
        }
        this.speed = 0.0;
        this.positionTicks = 0.0;
        // The reset zeroes the encoder that is physically plugged into this port. When that encoder counts
        // another mechanism, its injected number is an absolute count that keeps arriving every tick, so the
        // zero is kept as an offset subtracted on read (countedPositionTicks). Only a real change of mode
        // resets it (the early return above): re-setting STOP_AND_RESET_ENCODER while in it sends no second
        // reset. (This motor's own count is held at zero throughout the mode anyway, so for it the
        // distinction is invisible.) If no count has been injected yet there is nothing to take the offset
        // from, so the reset is remembered and the first injected count is used
        // (setEncoderMeasuringAnotherMechanism). That is the order an init() that resets encoders runs in,
        // before the sim's first tick injects anything.
        if (anotherMechanismSet) {
            this.anotherMechanismTicksAtLastReset = anotherMechanismPositionTicks;
        } else {
            this.anotherMechanismResetWaitingForFirstCount = true;
        }
        // Entering the reset mode re-enables the channel too, which is not obvious from the outside:
        // LynxDcMotorController.setMotorMode calls internalSetMotorPower(motor, 0) BEFORE recording the new
        // mode, so that zero-power command is built in the OUTGOING mode, is therefore sent, and enables.
        // Note this is the opposite of setPower: a power command while ALREADY in STOP_AND_RESET_ENCODER
        // builds no command and enables nothing.
        this.motorEnabled = true;
    }

    @Override public RunMode getMode() { return mode; }

    /**
     * The motor type configured on this port: the last one passed to {@link #setMotorType}, or, until
     * something sets one, a type derived from this plant's constructor numbers.
     *
     * <p>The derived type exists because SolversLib's {@code Motor.getCPR()} reads
     * {@code getMotorType().getTicksPerRev()} and its constructor reads
     * {@code getAchieveableMaxTicksPerSecond()}, and an empty {@code MotorConfigurationType} returns 0 for
     * both. So it carries the real CPR, with maxRPM and the achievable fraction chosen so that
     * {@code getAchieveableMaxTicksPerSecond()} equals this plant's free speed
     * ({@code ticksPerRev * maxRPM * fraction / 60}).
     *
     * <p>Every call returns a fresh copy, so a caller that edits the returned object cannot change what this
     * port reports without handing it back through {@link #setMotorType}. <b>This differs from the SDK</b>,
     * whose {@code LynxDcMotorController.getMotorType} returns the very object it stores; see
     * {@link #setMotorType}.
     */
    @Override public MotorConfigurationType getMotorType() {
        if (motorTypeSetByCaller != null) {
            return motorTypeSetByCaller.clone();
        }
        MotorConfigurationType type = new MotorConfigurationType();
        type.setTicksPerRev(ticksPerRev);
        type.setMaxRPM(maxVelocityInTicksPerSecond / ticksPerRev * 60.0);
        type.setAchieveableMaxRPMFraction(1.0);
        return type;
    }

    /**
     * Stores a copy of {@code motorType} as this port's configured type, which {@link #getMotorType()} then
     * reports — as {@code DcMotorImpl.setMotorType} hands it to the controller, which keeps it. SolversLib's
     * {@code Motor(hardwareMap, name, cpr, rpm)} constructor relies on this to record the CPR it was given.
     *
     * <p><b>Deliberate difference from the SDK:</b> the real controller keeps the very object it was handed
     * and returns that same object from {@code getMotorType}, so on a robot editing either one after the call
     * changes the port's configuration. This fake copies on the way in and on the way out, so a changed type
     * only takes effect when it is handed back through this method. Robot code that edits a returned type in
     * place and expects the port to change behaves differently here. (SolversLib clones, edits and hands
     * back, which works the same on both.)
     *
     * <p>It changes what this port <b>reports</b>, not how the modelled shaft moves: the plant's free speed
     * stays the one it was constructed with, because that is a fact about the physical motor, not about its
     * configuration. The angular-rate conversions in {@link #getVelocity(AngleUnit)} and
     * {@link #setVelocity(double, AngleUnit)} do follow the configured CPR, as a real controller's do.
     */
    @Override public void setMotorType(MotorConfigurationType motorType) {
        this.motorTypeSetByCaller = motorType.clone();
    }

    /**
     * Encoder counts per output-shaft revolution of the motor type configured on this port — what a real
     * controller converts angular rates with.
     *
     * @return counts per output-shaft revolution of the configured type
     */
    private double configuredTicksPerRev() {
        return getMotorType().getTicksPerRev();
    }

    /** {@return the port number, always 0 for a fake} */
    @Override public int getPortNumber() { return PORT_NUMBER; }

    @Override
    public DcMotorController getController() {
        throw new UnsupportedOperationException(
                "FakeMotor has no DcMotorController in sim — read state via getCurrentPosition()/getSpeed()");
    }

    // ---- DcMotorEx ----

    /**
     * The port's velocity in encoder ticks per second, as {@code DcMotorImplEx.getVelocity} reports it: the
     * hub's reading, which is a whole number of ticks per second, with the handle's direction applied. Reports
     * the other mechanism's rate when {@link #setEncoderMeasuringAnotherMechanism(double, double)} is in use.
     * The whole-number reading is SDK exact (the hub returns an {@code int}); rounding the modelled rate to
     * the nearest one is an APPROXIMATION of how the firmware measures it.
     *
     * @return the velocity, in whole encoder ticks per second, in the handle's direction
     */
    @Override public double getVelocity() {
        return reportedSign() * hubVelocityReadingTicksPerSecond();
    }

    /**
     * The port's velocity as an angular rate, converted exactly as {@code LynxDcMotorController} converts it:
     * the hub's whole ticks/s over this port's configured CPR, times 360 degrees, into {@code unit}'s
     * UNNORMALIZED form (a rate, so it is never wrapped to a half-turn), then the handle's direction.
     *
     * <p>The CPR is THIS port's configured motor type even when the encoder plugged into it belongs to another
     * mechanism with a different CPR. That is not a corner cut: a real Control Hub converts with the motor
     * type configured on the port, so an angle read off a borrowed port is wrong on the robot in exactly the
     * same way.
     *
     * @param unit the angle unit to report in, per second
     * @return the angular velocity of the output shaft, in {@code unit} per second, in the handle's direction
     */
    @Override public double getVelocity(AngleUnit unit) {
        double revolutionsPerSecond = hubVelocityReadingTicksPerSecond() / configuredTicksPerRev();
        return reportedSign() * unit.getUnnormalized().fromDegrees(revolutionsPerSecond * 360.0);
    }

    /** {@return the hub's velocity reading for this port: raw direction, whole ticks per second} */
    private int hubVelocityReadingTicksPerSecond() {
        return (int) Math.round(countedVelocityTicksPerSecond());
    }

    /**
     * Commands a shaft velocity in encoder ticks per second, as {@code DcMotorImplEx.setVelocity} and
     * {@code LynxDcMotorController.setMotorVelocity} do (SDK exact):
     * <ol>
     *   <li>switch to {@link RunMode#RUN_USING_ENCODER} unless already in it or in RUN_TO_POSITION, through
     *       {@link #setMode}, which re-issues the last power in the new mode first;</li>
     *   <li>send {@code round(direction * angularRate)}, clipped to +/-32767 ticks/s, as the hub's velocity
     *       target — in RUN_TO_POSITION its magnitude is the speed cap instead;</li>
     *   <li>enable the channel.</li>
     * </ol>
     * The hub's velocity loop then drives toward it on each {@link #update(double)}; the shaft approaches the
     * target rather than jumping to it, and a target beyond the motor's free speed saturates. Afterwards
     * {@link #getPower()} reports the target over the port's configured maximum (see there).
     *
     * @param angularRate target shaft velocity in encoder ticks per second, in the motor's logical
     *                    direction — {@link #setDirection} is applied on top, as it is for power
     * @throws TargetPositionNotSetException in RUN_TO_POSITION with no target ever set (the enable is NACKed)
     */
    @Override public void setVelocity(double angularRate) {
        if (!isClosedLoopMode()) {
            setMode(RunMode.RUN_USING_ENCODER);
        }
        this.velocityCommandTicksPerSecond = clampVelocityCommand(Math.round(reportedSign() * angularRate));
        this.power = Math.max(-1.0, Math.min(1.0,
                (double) velocityCommandTicksPerSecond / configuredMaxTicksPerSecondForPower()));
        enableChannel();
    }

    /**
     * Commands a shaft velocity given as an angular rate, converted through this port's configured CPR
     * exactly as {@code LynxDcMotorController.setMotorVelocity(int, double, AngleUnit)} converts it, then
     * handled as {@link #setVelocity(double)}.
     *
     * @param angularRate target angular velocity of the output shaft, in {@code unit} per second
     * @param unit        the angle unit {@code angularRate} is expressed in
     * @throws TargetPositionNotSetException as {@link #setVelocity(double)}
     */
    @Override public void setVelocity(double angularRate, AngleUnit unit) {
        double degreesPerSecond = UnnormalizedAngleUnit.DEGREES.fromUnit(unit.getUnnormalized(), angularRate);
        setVelocity(configuredTicksPerRev() * (degreesPerSecond / 360.0));
    }

    /**
     * Enables the channel, as {@code LynxDcMotorController.setMotorEnable} does.
     *
     * @throws TargetPositionNotSetException in RUN_TO_POSITION with no target ever set: the hub refuses the
     *                                       enable, and the channel stays off
     */
    @Override public void setMotorEnable() { enableChannel(); }

    /** Disables the channel: the windings get no power, in every mode, until a command re-enables it. */
    @Override public void setMotorDisable() { motorEnabled = false; }

    /** {@return whether the channel is enabled} */
    @Override public boolean isMotorEnabled() { return motorEnabled; }

    /**
     * {@code DcMotorImplEx.setPIDCoefficients}: the same as {@link #setPIDFCoefficients} with {@code f = 0}
     * and the {@link MotorControlAlgorithm#LegacyPID} algorithm, which {@code PIDFCoefficients(PIDCoefficients)}
     * builds.
     *
     * @param mode            {@code RUN_USING_ENCODER} or {@code RUN_TO_POSITION}
     * @param pidCoefficients the gains
     */
    @Override public void setPIDCoefficients(RunMode mode, PIDCoefficients pidCoefficients) {
        setPIDFCoefficients(mode, new PIDFCoefficients(pidCoefficients));
    }

    /**
     * Stores the hub's closed-loop gains for a mode, with the SDK's own checks (SDK exact, from
     * {@code LynxDcMotorController.setPIDFCoefficients}):
     * <ul>
     *   <li>a mode that is not a PID mode ({@code RunMode.isPIDMode()}, checked before migration) throws
     *       {@code IllegalArgumentException("motor 0: mode M is invalid as PID Mode")};</li>
     *   <li>RUN_TO_POSITION with any non-zero I, D or F under an algorithm other than
     *       {@link MotorControlAlgorithm#LegacyPID} throws {@code UnsupportedOperationException("setting of
     *       pidf coefficents not supported: ...")} and changes nothing, because the firmware's position loop
     *       feeds its velocity loop and those terms would double-integrate;</li>
     *   <li>the hub stores each gain as {@code round(|c| * 65536) * sign(c)}, so what
     *       {@link #getPIDFCoefficients} reads back is quantized to 1/65536.</li>
     * </ul>
     * The new gains are what the modelled loops then run with ({@code HubMotorControlLoop}; what each gain
     * means numerically is an APPROXIMATION of the firmware).
     *
     * @param mode              {@code RUN_USING_ENCODER} (or its deprecated alias) or {@code RUN_TO_POSITION}
     * @param pidfCoefficients  the gains
     * @throws IllegalArgumentException      if {@code mode} is not a PID mode
     * @throws UnsupportedOperationException if RUN_TO_POSITION is given I, D or F outside LegacyPID
     */
    @Override public void setPIDFCoefficients(RunMode mode, PIDFCoefficients pidfCoefficients) {
        if (!mode.isPIDMode()) {
            throw new IllegalArgumentException(
                    String.format("motor %d: mode %s is invalid as PID Mode", PORT_NUMBER, mode));
        }
        RunMode migrated = mode.migrate();
        if (migrated == RunMode.RUN_TO_POSITION && pidfCoefficients.algorithm != MotorControlAlgorithm.LegacyPID
                && (pidfCoefficients.i != 0 || pidfCoefficients.d != 0 || pidfCoefficients.f != 0)) {
            throw new UnsupportedOperationException(String.format(Locale.getDefault(),
                    "setting of pidf coefficents not supported: motor=%d mode=%s pidf=%s",
                    PORT_NUMBER, migrated, pidfCoefficients));
        }
        controlLoop.setCoefficients(migrated, new PIDFCoefficients(
                quantizedAsTheHubStoresIt(pidfCoefficients.p), quantizedAsTheHubStoresIt(pidfCoefficients.i),
                quantizedAsTheHubStoresIt(pidfCoefficients.d), quantizedAsTheHubStoresIt(pidfCoefficients.f),
                pidfCoefficients.algorithm));
    }

    /**
     * A gain after the round trip through the hub: {@code LynxSetMotorPIDControlLoopCoefficientsCommand}'s
     * {@code (int)(|c| * 65536 + 0.5) * (int) signum(c)} out, then that integer over 65536 back.
     *
     * @param coefficient the gain as given
     * @return the gain as the hub reads it back
     */
    private static double quantizedAsTheHubStoresIt(double coefficient) {
        int stored = (int) (Math.abs(coefficient) * COEFFICIENT_FIXED_POINT_SCALE + 0.5) * (int) Math.signum(coefficient);
        return stored / COEFFICIENT_FIXED_POINT_SCALE;
    }

    /**
     * {@code DcMotorImplEx.setVelocityPIDFCoefficients}: {@link #setPIDFCoefficients} for RUN_USING_ENCODER
     * with the PIDF algorithm.
     *
     * @param p proportional gain
     * @param i integral gain
     * @param d derivative gain
     * @param f feedforward gain
     */
    @Override public void setVelocityPIDFCoefficients(double p, double i, double d, double f) {
        setPIDFCoefficients(RunMode.RUN_USING_ENCODER, new PIDFCoefficients(p, i, d, f, MotorControlAlgorithm.PIDF));
    }

    /**
     * {@code DcMotorImplEx.setPositionPIDFCoefficients}: {@link #setPIDFCoefficients} for RUN_TO_POSITION with
     * only P set, PIDF algorithm.
     *
     * @param p the position loop's proportional gain
     */
    @Override public void setPositionPIDFCoefficients(double p) {
        setPIDFCoefficients(RunMode.RUN_TO_POSITION, new PIDFCoefficients(p, 0, 0, 0, MotorControlAlgorithm.PIDF));
    }

    /**
     * The P, I and D the hub holds for a mode (its F, if any, is dropped, as the SDK's fallback does).
     *
     * @param mode {@code RUN_USING_ENCODER} or {@code RUN_TO_POSITION} exactly (not migrated, as in the SDK)
     * @return a fresh copy of the gains
     * @throws IllegalArgumentException {@code "illegal mode: M"} for any other mode, as the hub command's
     *                                  constructor throws
     */
    @Override public PIDCoefficients getPIDCoefficients(RunMode mode) {
        PIDFCoefficients held = heldCoefficients(mode);
        return new PIDCoefficients(held.p, held.i, held.d);
    }

    /**
     * The gains and algorithm the hub holds for a mode, quantized as the hub stores them. A fresh hub reports
     * P 10, I 3, D 0, F 0 for RUN_USING_ENCODER and P 10 for RUN_TO_POSITION, both with the PIDF algorithm
     * (APPROXIMATION: these defaults live in the firmware, not the SDK).
     *
     * @param mode {@code RUN_USING_ENCODER} or {@code RUN_TO_POSITION} exactly (not migrated, as in the SDK)
     * @return a fresh copy of the gains
     * @throws IllegalArgumentException {@code "illegal mode: M"} for any other mode, as the hub command's
     *                                  constructor throws
     */
    @Override public PIDFCoefficients getPIDFCoefficients(RunMode mode) {
        return heldCoefficients(mode);
    }

    /**
     * The gains held for a mode, after the mode check the hub command's constructor makes.
     *
     * @param mode the mode asked about, unmigrated
     * @return a copy of the gains
     * @throws IllegalArgumentException {@code "illegal mode: M"} unless the mode is exactly one of the two
     */
    private PIDFCoefficients heldCoefficients(RunMode mode) {
        if (mode != RunMode.RUN_USING_ENCODER && mode != RunMode.RUN_TO_POSITION) {
            throw new IllegalArgumentException(String.format("illegal mode: %s", mode.toString()));
        }
        return controlLoop.coefficients(mode);
    }

    /**
     * Stores the RUN_TO_POSITION tolerance on the handle, in encoder ticks, as {@code DcMotorImplEx} does. It
     * reaches the hub only with the next {@link #setTargetPosition(int)}, so on its own it changes nothing
     * about {@link #isBusy()} (SDK exact). Not range-checked here; {@code setTargetPosition} rejects a value
     * outside {@code [0, 65535]}.
     *
     * @param tolerance the tolerance, in encoder ticks
     */
    @Override public void setTargetPositionTolerance(int tolerance) { this.targetPositionToleranceTicks = tolerance; }

    /** {@return the tolerance stored on the handle, in encoder ticks; 5 until set, as on a fresh {@code DcMotorImplEx}} */
    @Override public int getTargetPositionTolerance() { return targetPositionToleranceTicks; }

    /**
     * Modelled current draw. A brushed DC motor's winding current follows the voltage across it minus its
     * own back-EMF, and in this plant's normalized units both of those are already fractions of full scale:
     * <pre>
     *   torqueCurrent   = stallCurrent * |commandedPhysicalPower - shaftSpeedFraction|
     *   frictionCurrent = freeCurrent  * |shaftSpeedFraction|
     *   current         = min(stallCurrent, torqueCurrent + frictionCurrent)
     * </pre>
     * So a motor commanded full power from rest draws stall current; one running freely at the speed it was
     * asked for draws only its no-load current; and one commanded 0.3 while something holds its shaft still
     * draws 30% of stall. That last case is the one this exists for — a slide driven into its hardstop, or a
     * jammed mechanism.
     *
     * <p>{@code shaftSpeedFraction} is this motor's own modelled {@link #getSpeed()} unless a mechanism
     * plant has taken authority via {@link #setMeasuredShaftSpeedFraction(double)} — a motor with a
     * mechanism on it cannot know it is being held back, so it has to be told.
     *
     * <p><b>Not modelled:</b> battery sag, winding heat, and the inrush spike of a direction reversal. This
     * is a steady-state figure good enough to set a stall threshold against, not an electrical simulation
     * — the plant is first-order dynamics, not an electrical model. Pure read — no state changes here, so
     * replay stays bit-stable.
     */
    @Override public double getCurrent(CurrentUnit unit) {
        double shaftSpeedFraction = shaftSpeedFraction();
        double physicalPower = drivenPhysicalPower();
        double torqueCurrent = stallCurrentAmps * Math.abs(physicalPower - shaftSpeedFraction);
        double frictionCurrent = freeCurrentAmps * Math.abs(shaftSpeedFraction);
        double amps = Math.min(stallCurrentAmps, torqueCurrent + frictionCurrent);
        return unit.convert(amps, CurrentUnit.AMPS);
    }
    @Override public double getCurrentAlert(CurrentUnit unit) {
        return unit.convert(currentAlert, CurrentUnit.AMPS);
    }
    @Override public void setCurrentAlert(double current, CurrentUnit unit) {
        this.currentAlert = CurrentUnit.AMPS.convert(current, unit);
    }
    /** True once the modelled current reaches the alert threshold, matching the SDK's own over-current flag. */
    @Override public boolean isOverCurrent() {
        // A zero/unset alert means "no threshold configured", which the SDK reports as never over-current
        // rather than as always over-current.
        return currentAlert > 0.0 && getCurrent(CurrentUnit.AMPS) >= currentAlert;
    }
}
