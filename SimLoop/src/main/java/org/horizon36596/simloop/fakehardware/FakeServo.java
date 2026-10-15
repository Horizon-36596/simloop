package org.horizon36596.simloop.fakehardware;

import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.Range;

/**
 * Kinematic fake {@link Servo}: a commanded position is held instantly (a bare servo has no dynamics
 * worth modelling for the drivetrain sim — mechanism dynamics live in {@code Mechanism1DofPlant}).
 * Stores the commanded position in [0, 1] and the direction; reads are pure. Lets the unchanged
 * SolversLib {@code ServoEx} wrapper construct and drive it headless.
 *
 * <h2>Same arithmetic as {@code ServoImpl}, through the same SDK calls</h2>
 * {@link #setPosition(double)} and {@link #getPosition()} call the SDK's own {@link Range#clip} and
 * {@link Range#scale} in the order {@code ServoImpl} does, rather than writing the formulas out, so the
 * rounding is the real device's rounding bit for bit. An equivalent hand-written formula rounds differently
 * in the last place, which is a fake that quietly disagrees with the hardware.
 *
 * <h2>PWM control, and the one lookup that does not work</h2>
 * On a real robot a servo on a REV hub is a {@code ServoImplEx}, which also implements {@link PwmControl},
 * and this fake implements {@link PwmControl} too — so {@code ((PwmControl) servo).setPwmRange(...)} and
 * {@code hardwareMap.get(PwmControl.class, name)} work in sim exactly as on the robot. What does <b>not</b>
 * work is {@code hardwareMap.get(ServoImplEx.class, name)}: this class is not a {@code ServoImplEx} and
 * cannot be one without a fake {@code ServoControllerEx} behind it, because {@code ServoImplEx}'s
 * constructor talks to its controller. Robot code that asks for the concrete class throws in sim; ask for
 * {@link Servo} or {@link PwmControl} instead, which is the same device on the robot.
 *
 * <p>The PWM state follows {@code LynxServoController}: a fresh servo has PWM <b>disabled</b> (the hub
 * floats every servo port at start-up) and the SDK's default range of 600-2400 microseconds; every
 * {@link #setPosition(double)} enables PWM and sends the pulse width that position maps to, truncated to
 * whole microseconds as the hub's command carries it.
 */
public class FakeServo extends AbstractFakeDevice implements Servo, PwmControl {

    /** Creates a servo commanded to position 0.0, FORWARD, with the full {@code [0, 1]} scale range. */
    public FakeServo() {
    }

    private double position = 0.0;          // scaled (physical) position last commanded, in [limitMin, limitMax]
    private Direction direction = Direction.FORWARD;
    private double limitMin = MIN_POSITION; // Servo.MIN_POSITION (0.0)
    private double limitMax = MAX_POSITION; // Servo.MAX_POSITION (1.0)

    // ---- PWM state, as LynxServoController holds it for one port ----
    private PwmRange pwmRange = PwmRange.defaultRange;
    private boolean pwmEnabled = false;
    // The pulse width last sent to the port, in whole microseconds; 0 until something is sent.
    private int pulseWidthMicroseconds = 0;

    @Override
    public void update(double deltaTime) {
        // Instantaneous: commanded position is the position. No dynamics for a bare servo.
    }

    @Override
    public void setPosition(double position) {
        // Exactly ServoImpl.setPosition's three steps, in its order: clip the [0,1] request, apply the
        // direction, then map into the scaled [limitMin, limitMax] window.
        //
        // The ORDER is the part that matters. Reversing before scaling means REVERSE + scaleRange(0.2, 0.6) +
        // setPosition(0.2) puts the horn at 0.52; reversing after scaling - or not at all - puts it at 0.28. A
        // subsystem that reverses a servo in hardware and tunes its endpoints against the sim would otherwise
        // be tuned against a mirror image of the real robot.
        double clipped = Range.clip(position, MIN_POSITION, MAX_POSITION);
        double directed = direction == Direction.REVERSE ? reverse(clipped) : clipped;
        this.position = Range.scale(directed, MIN_POSITION, MAX_POSITION, limitMin, limitMax);
        sendPulseWidthForPosition(this.position);
    }

    /**
     * What {@code LynxServoController.setServoPosition} does with a scaled position: map it onto the PWM
     * range, clip to what the command can carry, truncate to whole microseconds, send it, and enable PWM.
     */
    private void sendPulseWidthForPosition(double scaledPosition) {
        double microseconds = Range.scale(scaledPosition, 0.0, 1.0, pwmRange.usPulseLower, pwmRange.usPulseUpper);
        this.pulseWidthMicroseconds = (int) Range.clip(microseconds, 1.0, 65535.0);
        this.pwmEnabled = true;
    }

    /** {@return the last commanded position on the user-facing {@code [0, 1]} scale, unitless} */
    @Override
    public double getPosition() {
        // Undo setPosition, in the mirrored order, exactly as ServoImpl.getPosition does: un-scale, un-apply
        // the direction, clip. Reversal applied twice cancels, so this round-trips what was commanded - which
        // is why a direction bug is invisible from getPosition() and only shows up in getScaledPosition(),
        // the physical value a sim mechanism actually reads. scaleRange refuses limitMin == limitMax, so the
        // un-scale never divides by zero.
        double unscaled = Range.scale(position, limitMin, limitMax, MIN_POSITION, MAX_POSITION);
        double undirected = direction == Direction.REVERSE ? reverse(unscaled) : unscaled;
        return Range.clip(undirected, MIN_POSITION, MAX_POSITION);
    }

    /** {@code ServoImpl.reverse}: mirrors a [0, 1] position about the middle of the range. */
    private static double reverse(double position) {
        return MAX_POSITION - position + MIN_POSITION;
    }

    /**
     * The scaled (physical) position actually applied, in {@code [limitMin, limitMax]} — the value a
     * downstream sim mechanism would see (analogous to {@code FakeMotor.getCurrentPosition()}). This is
     * where {@link #scaleRange(double, double)} is observable; {@link #getPosition()} reverses it. After a
     * raw {@link #setPulseWidth(double)} it is whatever that pulse width maps to, which may lie outside the
     * scale range, as it would on the robot.
     * <b>Sim-only — no such method exists on a real {@code Servo}.</b>
     *
     * @return the physical position in {@code [limitMin, limitMax]}, unitless servo travel
     */
    public double getScaledPosition() { return position; }

    /**
     * Sets the direction. Deliberately does <b>not</b> invalidate the stored position.
     *
     * <p><b>How the hub answers {@code getPosition()}.</b> {@code LynxServoController} keeps the last
     * commanded position, as a double, in a cache, and {@code getServoPosition} returns that cached double
     * when it has one. Only when the cache is empty does it read the channel's pulse width back and
     * reconstruct a position from it through the <b>current</b> PWM range (and caches the result). The cache
     * is emptied by exactly three things: {@code forgetLastKnownPosition}, which {@code ServoImpl.setDirection}
     * and {@code ServoImpl.scaleRange} call when the mapping changes; {@code setPulseWidth}; and disabling
     * PWM. {@code setPwmRange} does <b>not</b> empty it.
     *
     * <p>So {@code setDirection} and {@code scaleRange} do not make {@code getPosition()} unreadable, even
     * though its javadoc says it returns {@code Double.NaN} "if that is unavailable": the hub rebuilds the
     * physical position from the pulse width and {@code ServoImpl} un-maps it through the new mapping. This
     * class does the same by keeping {@link #position} and un-mapping it in {@link #getPosition()}. The one
     * difference: after the cache is emptied the hub's rebuilt value comes from a whole-microsecond pulse
     * width, so it can differ from the commanded double by up to one microsecond's worth of travel, while
     * this fake keeps the exact double. The NaN wording covers controllers with no pulse-width readback, and
     * a Control Hub channel that was never commanded (pulse width zero).
     */
    @Override public void setDirection(Direction direction) { this.direction = direction; }
    @Override public Direction getDirection() { return direction; }

    @Override
    public void scaleRange(double min, double max) {
        // SDK contract, read off ServoImpl.scaleRange rather than assumed: each endpoint is CLIPPED into
        // [0, 1] first, and the only thing that throws is an inverted or empty range after clipping. So
        // scaleRange(-0.1, 0.5) is a legal request for [0.0, 0.5] on a real servo, not an error.
        double clippedMin = Range.clip(min, MIN_POSITION, MAX_POSITION);
        double clippedMax = Range.clip(max, MIN_POSITION, MAX_POSITION);
        if (clippedMin >= clippedMax) {
            throw new IllegalArgumentException("min must be less than max");
        }
        this.limitMin = clippedMin;
        this.limitMax = clippedMax;
    }

    /**
     * Restores what {@code ServoImpl.resetDeviceConfigurationForOpMode} restores: the full {@code [0, 1]}
     * scale range and {@code FORWARD}. The commanded position is left alone, as on the real device; like
     * {@code ServoImpl}, this does not forget or resend anything.
     */
    @Override
    public void resetDeviceConfigurationForOpMode() {
        this.limitMin = MIN_POSITION;
        this.limitMax = MAX_POSITION;
        this.direction = Direction.FORWARD;
    }

    /** {@code "Servo"}, the string {@code ServoImpl.getDeviceName} reads out of the SDK's resources. */
    @Override public String getDeviceName() { return "Servo"; }

    /** The controller's connection info plus {@code "; port N"}, in {@code ServoImpl}'s format. */
    @Override public String getConnectionInfo() { return "sim; port " + getPortNumber(); }

    @Override public int getPortNumber() { return 0; }

    @Override
    public ServoController getController() {
        throw new UnsupportedOperationException(
                "FakeServo has no ServoController in sim — read state via getPosition()");
    }

    // ---- PwmControl (what ServoImplEx adds) ----

    /**
     * Stores the PWM range, in microseconds, used to turn later positions into pulse widths. As on the hub,
     * nothing is resent: the pulse width already on the port is unchanged until the next
     * {@link #setPosition(double)}. Nor does the hub empty its position cache here (see
     * {@link #setDirection}), so {@link #getPosition()} goes on reporting the commanded position, as it does
     * in this fake.
     */
    @Override public void setPwmRange(PwmRange range) { this.pwmRange = range; }

    /** {@return the PWM range in microseconds; the SDK's default 600-2400 us range until set} */
    @Override public PwmRange getPwmRange() { return pwmRange; }

    /** Enables the PWM signal on this port. */
    @Override public void setPwmEnable() { this.pwmEnabled = true; }

    /**
     * Disables the PWM signal on this port, so a real servo goes limp; the next position re-enables it. On
     * the hub this also empties the position cache (see {@link #setDirection}), so a real
     * {@code getPosition()} afterwards is rebuilt from the pulse width the hub reports; this fake keeps
     * reporting the last commanded position.
     */
    @Override public void setPwmDisable() { this.pwmEnabled = false; }

    /** {@return whether the PWM signal is on; false on a fresh servo, true after any position command} */
    @Override public boolean isPwmEnabled() { return pwmEnabled; }

    /**
     * Sends a raw pulse width, as {@code LynxServoController.setPulseWidth} does: truncated to whole
     * microseconds, PWM enabled, and the commanded position replaced by the one that pulse width maps back
     * to through the PWM range. That is what the hub reports from then on: {@code setPulseWidth} empties its
     * position cache, so the next read rebuilds the position from the pulse width (see
     * {@link #setDirection}).
     *
     * @param microseconds the pulse width to send, in microseconds
     */
    @Override
    public void setPulseWidth(double microseconds) {
        this.pulseWidthMicroseconds = (int) microseconds;
        this.pwmEnabled = true;
        this.position = Range.clip(
                Range.scale(pulseWidthMicroseconds, pwmRange.usPulseLower, pwmRange.usPulseUpper, 0.0, 1.0),
                0.0, 1.0);
    }

    /** {@return the pulse width last sent to this port, in whole microseconds; 0 before anything is sent} */
    @Override public double getPulseWidth() { return pulseWidthMicroseconds; }
}
