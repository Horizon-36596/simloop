package org.horizon36596.simloop.fakehardware;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;

/**
 * Headless fake of the SparkFun Qwiic Optical Tracking Odometry Sensor (OTOS), for SimLoop users whose robots
 * use one. Horizon (FTC 36596) SimLoop.
 *
 * <p><b>The public API is the real driver's, unchanged.</b> This class IS a
 * {@code com.qualcomm.hardware.sparkfun.SparkFunOTOS} - the class FTC SDK 12.0 ships in its {@code Hardware}
 * artifact - and it overrides <i>none</i> of that driver's methods, public or protected. Robot code that does
 * {@code hardwareMap.get(SparkFunOTOS.class, "otos")} gets this object and runs unchanged. What it replaces is
 * the one thing a JVM does not have: the sensor on the other end of the I2C bus. The driver is built on
 * {@code Firmware}, an emulated OTOS that answers the driver's register reads and writes the way the sensor's
 * registers do. So every public method - the unit conversions ({@code setLinearUnit}, {@code setAngularUnit}),
 * the int16 register scaling, the scalar range check and its rounding, the {@code Status}, {@code Version},
 * {@code SelfTestConfig} and {@code SignalProcessConfig} bit layouts, the calibration and self-test polling -
 * runs the SDK's own code. A reflection test pins that no method is overridden.
 *
 * <p>This is the same shape as {@code FakeGoBildaPinpoint}, for the same reason: the SDK type is a concrete
 * driver, not an interface, so re-implementing its ~30 public methods would be re-deriving the SDK's code and
 * hoping to match it. The sim-side seam is the same too ({@link #attachChassisAt} and {@link #moveChassisTo},
 * millimetres and radians), so one harness line can feed either device.
 *
 * <h2>What the emulated sensor does</h2>
 * <ul>
 *   <li><b>It is mounted the way the sim says, and read the way the robot declared.</b> Two separate things,
 *       as on a real robot. The <i>physical</i> mount - where the sensor really sits on the chassis and which
 *       way it really faces - is sim-only, set with {@link #setPhysicalMount} (default: on the tracking
 *       centre, facing forward). The <i>declared</i> mount is what robot code wrote with {@code setOffset}.
 *       {@link #moveChassisTo} turns the chassis's motion into what the sensor itself sees - its own
 *       translation, in its own frame, and its gyro's turn - through the physical mount; the emulated firmware
 *       turns that back into the robot centre's pose through the declared offset. When the two agree the pose
 *       is the chassis's motion; when a robot declares its offset wrong, its pose is wrong the way a real
 *       OTOS's would be (an undeclared forward offset makes a turn in place read as a circle; an undeclared
 *       90-degree mounting makes driving forward read as a strafe).</li>
 *   <li><b>Frame</b>: heading counter-clockwise positive; X and Y are whatever robot frame the declared offset
 *       is written in, as on the real chip. With a zero offset and the default physical mount that is the
 *       plant's frame, X forward and Y left. (The SDK's {@code SensorSparkFunOTOS} sample's comments use X
 *       right and Y forward instead; a robot that declares its offset in that frame reads its pose in that
 *       frame. {@link #setPhysicalMount} always stays in the plant's forward/left frame.) The registers hold
 *       metres and radians as signed 16-bit counts ({@code METER_TO_INT16}, {@code RAD_TO_INT16} and friends
 *       are the SDK's own constants), so a pose comes back to the robot quantised to about 0.3 mm and 0.0055
 *       degrees, exactly as the real register format forces. Heading is wrapped into [-pi, pi) by the 16-bit
 *       register.</li>
 *   <li><b>Pose</b> is an on-sensor integrator, as on the real board: each step it adds the motion the sensor
 *       measured onto whatever pose it holds - zero at power-on and after {@code resetTracking()}, or the pose
 *       last written with {@code setPosition}. Within one step the chassis is taken to turn at a steady rate,
 *       so the step is an arc, both when the sensor sees it and when the firmware integrates it.</li>
 *   <li><b>Scalars</b> multiply what the sensor measures, as SparkFun documents them: the linear scalar every
 *       translation, the angular scalar every turn. {@link #setPhysicalTrackingGains} is the sim-only other
 *       half - how far the real sensor over- or under-reads on this floor - so a test can show a scalar
 *       calibrating an error away (default 1: a perfect sensor).</li>
 *   <li><b>Velocity and acceleration</b> are the pose's rates of change over the last {@link #update(double)}
 *       tick, in the same (field) frame as the pose. Note the driver converts every heading-like value with
 *       {@code AngleUnit.fromRadians}, which wraps into [-180, 180) degrees, so an angular velocity above
 *       180 deg/s reads wrapped in {@code DEGREES} - that is the SDK's behaviour, reproduced, not a fake bug.
 *       The standard deviations are zero: the sim has no noise.</li>
 *   <li><b>IMU calibration</b> counts samples down, {@link #CALIBRATION_SECONDS_PER_SAMPLE} of device time
 *       each. Device time is sim time ({@link #update(double)}) plus the SDK's own {@code Thread.sleep(3)}
 *       between its polls: a blocking {@code calibrateImu()} completes and returns {@code true} within the
 *       one sim instant it is called in, as a real one does within one (long) loop iteration. See
 *       {@link #DRIVER_POLL_SECONDS}.</li>
 * </ul>
 *
 * <p><b>Units are the SDK's defaults: inches and degrees.</b> The real driver sets them in its protected
 * {@code doInitialize()}, which the SDK's hardware map runs the first time robot code looks the device up.
 * {@code FakeHardwareMap} does not run that step, so this fake runs the driver's own {@code doInitialize()}
 * once, at construction; a real OTOS is equally initialised before robot code first holds it.
 *
 * <p>Determinism (domain R5): no wall clock, no randomness. The sensor changes only when the driver writes a
 * register, when {@link #moveChassisTo} reports motion, when {@link #update(double)} ticks it, or when the
 * driver polls calibration inside its own blocking call (a fixed, counted step). The SDK's {@code Thread.sleep}
 * calls in {@code calibrateImu} and {@code selfTest} still take real time (about 0.6 s for a blocking
 * 255-sample calibration) but never change a reading.
 */
public class FakeSparkFunOTOS extends SparkFunOTOS implements FakeDevice {

    /**
     * How long the OTOS takes per IMU calibration sample, seconds of device time. PLACEHOLDER: 2.4 ms, as
     * SparkFun's OTOS library documentation states ("each sample takes about 2.4ms"); not measured on a unit.
     * A full 255-sample calibration is therefore about 0.61 s.
     */
    public static final double CALIBRATION_SECONDS_PER_SAMPLE = 0.0024;

    /**
     * How long the SDK driver sleeps between polls of the calibration register, seconds. SPEC: the SDK 12.0
     * {@code SparkFunOTOS.calibrateImu(int, boolean)} bytecode calls {@code Thread.sleep(3)} after writing the
     * sample count and again after each poll that is not yet zero. The fake cannot see the sleep, so it
     * credits this much device time to each poll of the calibration register made in the same sim instant as
     * the write that started it - exactly the polls the driver's blocking loop makes. A poll made after a sim
     * tick (a robot polling {@code getImuCalibrationProgress()} from its loop) gets no credit: its time comes
     * from the tick.
     */
    public static final double DRIVER_POLL_SECONDS = 0.003;

    /**
     * What the hardware-version register reports, as {@code Version}'s packed byte (major in the high
     * nibble, minor in the low). PLACEHOLDER: v1.0; not read off a unit, and nothing in the driver checks it.
     */
    public static final int HARDWARE_VERSION = 0x10;

    /**
     * What the firmware-version register reports, as {@code Version}'s packed byte. PLACEHOLDER: v1.0; not
     * read off a unit, and nothing in the driver checks it.
     */
    public static final int FIRMWARE_VERSION = 0x10;

    /**
     * What the signal-process register holds before robot code writes it, as {@code SignalProcessConfig}'s
     * packed byte. PLACEHOLDER: 0x0F, all four stages enabled, which SparkFun's library documentation calls
     * the default; not read off a unit. The fake's tracking ignores this register: it models no Kalman filter.
     */
    public static final int DEFAULT_SIGNAL_PROCESS_CONFIG = 0x0F;

    private final Firmware firmware;

    /**
     * An OTOS at the origin, connected, not calibrating, with no status flags, on the tracking centre facing
     * forward, reading in inches and degrees (the SDK's defaults).
     */
    public FakeSparkFunOTOS() {
        this(new Firmware());
    }

    private FakeSparkFunOTOS(Firmware firmware) {
        super(firmware);
        this.firmware = firmware;
        // What the SDK's HardwareMap.get would have done before handing the device out (class javadoc):
        // the driver's own initialisation - inches, degrees, and a product-ID check over the bus.
        isInitialized = doInitialize();
    }

    // =================================================================================================
    // Sim input: the chassis moving under the sensor (the FakeGoBildaPinpoint / FakeOctoQuad seam)
    // =================================================================================================

    /**
     * Tell the sensor where the sim's chassis is before any motion is reported, without moving the sensor's
     * pose. Call it once, when a drive plant is wired to this sensor, with that plant's current pose.
     *
     * <p><b>Millimetres, not inches.</b> SimLoop's drive plants report inches, so multiply by 25.4 first:
     * {@code attachChassisAt(plant.getX() * 25.4, plant.getY() * 25.4, plant.getHeading())}. The same seam,
     * in the same units, as {@code FakeGoBildaPinpoint}.
     *
     * <p><b>Call it again after any teleport of the plant.</b> {@link #moveChassisTo} treats every change as
     * driven motion, so a teleport not announced here reaches the sensor as one enormous step.
     *
     * @param plantXMm        the chassis X in the drive plant's frame (forward at heading 0), millimetres
     * @param plantYMm        the chassis Y in the drive plant's frame (left at heading 0), millimetres
     * @param plantHeadingRad the chassis heading in the drive plant's frame, radians CCW
     */
    public void attachChassisAt(double plantXMm, double plantYMm, double plantHeadingRad) {
        firmware.chassisXMetres = plantXMm / MILLIMETRES_PER_METRE;
        firmware.chassisYMetres = plantYMm / MILLIMETRES_PER_METRE;
        firmware.chassisHeading = plantHeadingRad;
    }

    /**
     * The chassis is now here, in the drive plant's frame. The sensor sees the motion since the last call
     * through its physical mount ({@link #setPhysicalMount}) and its physical gains
     * ({@link #setPhysicalTrackingGains}); the emulated firmware scales it by the robot's scalars and adds the
     * robot-centre motion it describes, through the declared offset, onto its own pose - so a pose the robot
     * wrote stays written, and the plant and the sensor do not need to share an origin. While an IMU
     * calibration is running the pose ignores the motion. Millimetres, as {@link #attachChassisAt}.
     *
     * <p>The heading may be wrapped or not. The turn since the last call is taken the short way round, into
     * (-pi, pi], so one call must never be asked to report a turn of half a circle or more.
     *
     * @param plantXMm        the chassis X in the drive plant's frame (forward at heading 0), millimetres
     * @param plantYMm        the chassis Y in the drive plant's frame (left at heading 0), millimetres
     * @param plantHeadingRad the chassis heading in the drive plant's frame, radians CCW, wrapped or not
     */
    public void moveChassisTo(double plantXMm, double plantYMm, double plantHeadingRad) {
        firmware.moveChassisTo(plantXMm / MILLIMETRES_PER_METRE, plantYMm / MILLIMETRES_PER_METRE,
                plantHeadingRad);
    }

    /**
     * Sim only: where the sensor REALLY sits on the simulated robot - not what robot code tells it with
     * {@code setOffset}, but the truth that offset is supposed to describe. Same convention as
     * {@code setOffset}: the sensor's position relative to the robot's tracking centre in the robot's frame
     * (X forward, Y left), and the angle its own X axis is turned from the robot's, CCW positive. The sensor
     * measures the chassis's motion through this mount; the firmware reads it back through the declared one.
     * Takes effect from the next {@link #moveChassisTo}.
     *
     * <p>Default: on the tracking centre, facing forward (0, 0, 0). PLACEHOLDER: a real OTOS is rarely on the
     * centre; the default only makes a robot that declares no offset read the chassis exactly, because the
     * sensor's own offset registers start at zero too.
     *
     * @param forwardMm  how far FORWARD of the tracking centre the sensor really is, millimetres
     * @param leftMm     how far LEFT of the tracking centre the sensor really is, millimetres
     * @param headingRad how far the sensor's X axis is really turned from the robot's forward, radians CCW
     */
    public void setPhysicalMount(double forwardMm, double leftMm, double headingRad) {
        requireFinite(forwardMm, "forwardMm");
        requireFinite(leftMm, "leftMm");
        requireFinite(headingRad, "headingRad");
        firmware.physicalForwardMetres = forwardMm / MILLIMETRES_PER_METRE;
        firmware.physicalLeftMetres = leftMm / MILLIMETRES_PER_METRE;
        firmware.physicalHeading = headingRad;
    }

    /**
     * Sim only: how much the physical sensor over- or under-reads before the robot's scalars are applied - a
     * real OTOS reads distance a little off depending on its height above the floor and the floor's texture,
     * which is what {@code setLinearScalar} exists to calibrate away. The sensor reports
     * {@code true motion * physical gain * scalar}, so a robot whose scalar is {@code 1 / gain} reads true.
     * Takes effect from the next {@link #moveChassisTo}.
     *
     * <p>Default: 1 and 1, a perfect sensor. PLACEHOLDER: no floor was measured.
     *
     * @param linearGain  factor on every translation the sensor sees, unitless; must be positive and finite
     * @param angularGain factor on every turn the sensor's gyro sees, unitless; must be positive and finite
     */
    public void setPhysicalTrackingGains(double linearGain, double angularGain) {
        if (!(linearGain > 0.0) || Double.isInfinite(linearGain)) {
            throw new IllegalArgumentException("linearGain must be positive and finite, got " + linearGain);
        }
        if (!(angularGain > 0.0) || Double.isInfinite(angularGain)) {
            throw new IllegalArgumentException("angularGain must be positive and finite, got " + angularGain);
        }
        firmware.physicalLinearGain = linearGain;
        firmware.physicalAngularGain = angularGain;
    }

    /**
     * Advance the sensor's own clock: count down a running IMU calibration, and work out the velocity and
     * acceleration it reports from the motion since the last tick. A scenario calls this (or
     * {@code FakeHardwareMap.updateAll}) once per tick, after {@link #moveChassisTo}.
     *
     * @param deltaTime the tick, seconds; zero is allowed and leaves the rates as they were
     */
    @Override
    public void update(double deltaTime) {
        firmware.tick(deltaTime);
    }

    // =================================================================================================
    // Test-only fault injection and instrumentation
    // =================================================================================================

    /**
     * Make the status register report these flags, as if the sensor had the fault, until
     * {@link #clearForcedStatus()}. Tracking itself is not changed: the flags are what robot code sees
     * through {@code getStatus()}, nothing more.
     *
     * @param status the flags {@code getStatus()} should decode to; not null
     */
    public void forceStatus(Status status) {
        if (status == null) {
            throw new IllegalArgumentException("status must not be null; use clearForcedStatus()");
        }
        firmware.forcedStatus = status.get() & BYTE_MASK;
    }

    /** Stop forcing status flags; the status register reports no flags again. */
    public void clearForcedStatus() {
        firmware.forcedStatus = -1;
    }

    /**
     * Decide how the next and later self tests end. Default: they pass. PLACEHOLDER: a self test finishes
     * before the driver's first poll (5 ms after it starts); how long a real one runs is not documented.
     *
     * @param passes true for a passing self test, false for a failing one
     */
    public void setSelfTestResult(boolean passes) {
        firmware.selfTestPasses = passes;
    }

    /**
     * Unplug (false) or re-plug (true) the sensor's I2C cable. While unplugged every read returns zero bytes
     * and every write is lost, so {@code isConnected()} and {@code begin()} return false; the sensor itself
     * stays powered and keeps tracking. PLACEHOLDER: zeros are what this fake chooses for a device that does
     * not answer; what a REV hub's bus returns then was not measured.
     *
     * @param connected whether the sensor answers the bus
     */
    public void setConnected(boolean connected) {
        firmware.connected = connected;
    }

    /**
     * How many I2C reads the driver has made. Real hardware has no such counter; it exists so a test can pin,
     * say, that {@code getPosVelAcc} is one bus read.
     *
     * @return the number of reads since construction
     */
    public int busReadCount() {
        return firmware.reads;
    }

    // =================================================================================================
    // Shared helpers
    // =================================================================================================

    private static final double MILLIMETRES_PER_METRE = 1000.0;
    private static final int BYTE_MASK = 0xFF;

    private static void requireFinite(double value, String name) {
        if (Double.isNaN(value) || Double.isInfinite(value)) {
            throw new IllegalArgumentException(name + " must be finite, got " + value);
        }
    }

    // =================================================================================================
    // The emulated sensor
    // =================================================================================================

    /**
     * The OTOS on the far side of the bus: its register file, and what it does when one is written. Register
     * addresses are the SDK driver's own {@code REG_*} constants. Poses are three little-endian signed 16-bit
     * values (x, y, h) back to back, and a read of several bytes runs on through consecutive registers, as the
     * driver's {@code getPosVelAcc} (18 bytes from {@code REG_POS_XL}) expects. A read or write of any register
     * the SDK 12.0 driver never touches fails loud: what a real unit does there is not modelled.
     */
    private static final class Firmware extends FakeI2cDeviceSynchSimple implements I2cDeviceSynch {

        /** One past the last register address the driver reads ({@code REG_ACC_STD_HH}). */
        static final int REGISTER_COUNT = REG_ACC_STD_HH + 1;

        /** The only value the driver writes to {@code REG_RESET}: bit 0, reset tracking. */
        static final int RESET_TRACKING = 1;

        /** {@code SelfTestConfig}'s bits, as the SDK decodes them. */
        static final int SELF_TEST_START = 1;
        static final int SELF_TEST_PASS = 4;
        static final int SELF_TEST_FAIL = 8;

        /** How close to a whole sample a calibration's elapsed time counts as reaching it, seconds. */
        static final double SAMPLE_BOUNDARY_TOLERANCE_SECONDS = 1e-9;

        // --- The sensor's pose of the ROBOT CENTRE, in its own field frame: metres and radians. The heading
        // is kept continuous here and wrapped only by the register. ---
        double x;
        double y;
        double heading;
        double velocityX;
        double velocityY;
        double velocityHeading;
        double accelerationX;
        double accelerationY;
        double accelerationHeading;
        private double xAtLastTick;
        private double yAtLastTick;
        private double headingAtLastTick;
        private double velocityXAtLastTick;
        private double velocityYAtLastTick;
        private double velocityHeadingAtLastTick;

        // --- Registers the robot writes. The offset is kept as the raw bytes the driver wrote, so it reads
        // back exactly, and decoded for the maths. ---
        final byte[] offsetRegisters = new byte[6];
        double declaredForwardMetres;
        double declaredLeftMetres;
        double declaredHeading;
        byte linearScalarRegister;
        byte angularScalarRegister;
        int signalProcessConfig = DEFAULT_SIGNAL_PROCESS_CONFIG;

        // --- The mount as it really is (sim only; not registers). ---
        double physicalForwardMetres;
        double physicalLeftMetres;
        double physicalHeading;
        double physicalLinearGain = 1.0;
        double physicalAngularGain = 1.0;

        // --- IMU calibration: samples asked for, device time since it started, and whether the driver's
        // blocking poll loop can still be running (no sim tick since the write). ---
        int calibrationSamples;
        double calibrationElapsedSeconds;
        boolean inTheInstantOfTheCalibrationWrite;

        int selfTestRegister;
        boolean selfTestPasses = true;
        int forcedStatus = -1;
        boolean connected = true;
        boolean engaged;
        int reads;

        // Where the sim's drive plant last said the chassis was, in the plant's frame, metres. Not a register.
        double chassisXMetres;
        double chassisYMetres;
        double chassisHeading;

        void moveChassisTo(double plantX, double plantY, double plantHeading) {
            // --- The world: the chassis moves, and the sensor sees it through the PHYSICAL mount. ---

            // The step in the chassis's own frame at its start (chord forward, chord left) and the turn, taken
            // the short way round so a plant that wraps its heading at +-pi is not a jump.
            double dx = plantX - chassisXMetres;
            double dy = plantY - chassisYMetres;
            double cos = Math.cos(chassisHeading);
            double sin = Math.sin(chassisHeading);
            double chordForward = cos * dx + sin * dy;
            double chordLeft = -sin * dx + cos * dy;
            double turn = shortWayRound(plantHeading - chassisHeading);
            chassisXMetres = plantX;
            chassisYMetres = plantY;
            chassisHeading = plantHeading;

            // The arc the tracking centre drove, in the chassis frame, turning at a steady rate.
            double[] arc = arcFromChord(chordForward, chordLeft, turn);

            // The sensor's own travel: the chassis's, plus the turn swinging it about the centre
            // (v + omega x r), then turned into the sensor's own axes.
            double sensorTravelForward = arc[0] - turn * physicalLeftMetres;
            double sensorTravelLeft = arc[1] + turn * physicalForwardMetres;
            double[] seenBySensor = rotate(sensorTravelForward, sensorTravelLeft, -physicalHeading);
            double gyroTurn = turn * physicalAngularGain;

            if (calibrationRemaining() > 0) {
                return;                            // calibrating: the pose is not tracking (PLACEHOLDER)
            }

            // --- The firmware: scale by the robot's scalars, then back to the robot centre through the
            // DECLARED offset, then integrate. ---
            double linear = physicalLinearGain * linearScalar();
            double measuredTurn = gyroTurn * angularScalar();
            double[] sensorTravelInRobotAxes = rotate(seenBySensor[0] * linear, seenBySensor[1] * linear,
                    declaredHeading);
            double robotArcForward = sensorTravelInRobotAxes[0] + measuredTurn * declaredLeftMetres;
            double robotArcLeft = sensorTravelInRobotAxes[1] - measuredTurn * declaredForwardMetres;
            double[] chord = chordFromArc(robotArcForward, robotArcLeft, measuredTurn);
            double[] fieldStep = rotate(chord[0], chord[1], heading);
            x += fieldStep[0];
            y += fieldStep[1];
            heading += measuredTurn;
        }

        void tick(double deltaTime) {
            if (calibrationSamples > 0) {
                calibrationElapsedSeconds += deltaTime;
            }
            inTheInstantOfTheCalibrationWrite = false;
            if (deltaTime > 0.0) {
                velocityX = (x - xAtLastTick) / deltaTime;
                velocityY = (y - yAtLastTick) / deltaTime;
                velocityHeading = (heading - headingAtLastTick) / deltaTime;
                accelerationX = (velocityX - velocityXAtLastTick) / deltaTime;
                accelerationY = (velocityY - velocityYAtLastTick) / deltaTime;
                accelerationHeading = (velocityHeading - velocityHeadingAtLastTick) / deltaTime;
            }
            xAtLastTick = x;
            yAtLastTick = y;
            headingAtLastTick = heading;
            velocityXAtLastTick = velocityX;
            velocityYAtLastTick = velocityY;
            velocityHeadingAtLastTick = velocityHeading;
        }

        double linearScalar() {
            return 1.0 + 0.001 * linearScalarRegister;
        }

        double angularScalar() {
            return 1.0 + 0.001 * angularScalarRegister;
        }

        /** Samples still to take, as the calibration register reports it; 0 when idle or done. */
        int calibrationRemaining() {
            if (calibrationSamples <= 0) {
                return 0;
            }
            int taken = (int) Math.floor(
                    (calibrationElapsedSeconds + SAMPLE_BOUNDARY_TOLERANCE_SECONDS) / CALIBRATION_SECONDS_PER_SAMPLE);
            int remaining = calibrationSamples - taken;
            if (remaining <= 0) {
                calibrationSamples = 0;
                return 0;
            }
            return remaining;
        }

        /**
         * Move the pose to {@code (newX, newY, newHeading)} without it being motion: the field frame is re-based,
         * so the last tick's pose and the rates are carried through the same rigid change, and the next tick
         * reports only real motion. PLACEHOLDER: what a real unit's filter does to its velocity on a write is not
         * documented; this is "a written pose is not motion", the same rule {@code FakeGoBildaPinpoint} uses.
         */
        void rebaseTo(double newX, double newY, double newHeading) {
            double turn = newHeading - heading;
            double[] lastTick = rotate(xAtLastTick - x, yAtLastTick - y, turn);
            xAtLastTick = newX + lastTick[0];
            yAtLastTick = newY + lastTick[1];
            headingAtLastTick += turn;
            double[] velocity = rotate(velocityX, velocityY, turn);
            velocityX = velocity[0];
            velocityY = velocity[1];
            double[] velocityAtLastTick = rotate(velocityXAtLastTick, velocityYAtLastTick, turn);
            velocityXAtLastTick = velocityAtLastTick[0];
            velocityYAtLastTick = velocityAtLastTick[1];
            double[] acceleration = rotate(accelerationX, accelerationY, turn);
            accelerationX = acceleration[0];
            accelerationY = acceleration[1];
            x = newX;
            y = newY;
            heading = newHeading;
        }

        /**
         * {@code resetTracking()}: the pose goes to the origin and the velocity and acceleration registers read
         * zero at once. PLACEHOLDER: zeroing the rates is this fake's reading of "resets the tracking
         * algorithm"; the history is re-based rather than zeroed, so a robot that resets while moving reads its
         * true velocity again from the next tick, with no acceleration spike.
         */
        void resetTracking() {
            rebaseTo(0.0, 0.0, 0.0);
            velocityX = 0.0;
            velocityY = 0.0;
            velocityHeading = 0.0;
            accelerationX = 0.0;
            accelerationY = 0.0;
            accelerationHeading = 0.0;
        }

        // ---------------------------------------------------------------------------------------------
        // The bus
        // ---------------------------------------------------------------------------------------------

        @Override
        public byte read8(int ireg) {
            return read(ireg, 1)[0];
        }

        @Override
        public byte[] read(int ireg, int creg) {
            reads++;
            if (creg <= 0 || ireg < 0 || ireg + creg > REGISTER_COUNT) {
                throw new UnsupportedOperationException("FakeSparkFunOTOS: a read of " + creg + " bytes from register "
                        + ireg + "; the SDK 12.0 driver never makes one");
            }
            if (!connected) {
                return new byte[creg];
            }
            if (ireg <= REG_IMU_CALIB && REG_IMU_CALIB < ireg + creg && inTheInstantOfTheCalibrationWrite) {
                // The driver slept DRIVER_POLL_SECONDS before this poll; the sensor kept calibrating.
                if (calibrationRemaining() > 0) {
                    calibrationElapsedSeconds += DRIVER_POLL_SECONDS;
                }
            }
            byte[] out = new byte[creg];
            for (int i = 0; i < creg; i++) {
                out[i] = registerByte(ireg + i);
            }
            return out;
        }

        /** One register's byte, as the sensor would send it. */
        private byte registerByte(int address) {
            if (address >= REG_OFF_XL && address <= REG_OFF_HH) {
                return offsetRegisters[address - REG_OFF_XL];
            }
            if (address >= REG_POS_XL && address <= REG_POS_HH) {
                return poseByte(address - REG_POS_XL, x, y, wrapToRegister(heading), METER_TO_INT16, RAD_TO_INT16);
            }
            if (address >= REG_VEL_XL && address <= REG_VEL_HH) {
                return poseByte(address - REG_VEL_XL, velocityX, velocityY, velocityHeading,
                        MPS_TO_INT16, RPS_TO_INT16);
            }
            if (address >= REG_ACC_XL && address <= REG_ACC_HH) {
                return poseByte(address - REG_ACC_XL, accelerationX, accelerationY, accelerationHeading,
                        MPSS_TO_INT16, RPSS_TO_INT16);
            }
            if (address >= REG_POS_STD_XL && address <= REG_ACC_STD_HH) {
                return 0;                                  // no noise, so no uncertainty
            }
            switch (address) {
                case REG_PRODUCT_ID: return PRODUCT_ID;
                case REG_HW_VERSION: return (byte) HARDWARE_VERSION;
                case REG_FW_VERSION: return (byte) FIRMWARE_VERSION;
                case REG_SCALAR_LINEAR: return linearScalarRegister;
                case REG_SCALAR_ANGULAR: return angularScalarRegister;
                case REG_IMU_CALIB: return (byte) calibrationRemaining();
                case REG_SIGNAL_PROCESS: return (byte) signalProcessConfig;
                case REG_SELF_TEST: return (byte) selfTestRegister;
                case REG_STATUS: return (byte) (forcedStatus >= 0 ? forcedStatus : 0);
                default:
                    throw new UnsupportedOperationException("FakeSparkFunOTOS has no readable register " + address
                            + "; the SDK 12.0 driver never reads one");
            }
        }

        /**
         * Byte {@code index} (0-5) of a pose-shaped register block: x, y, h as little-endian int16 counts,
         * each value times its scale, rounded, and held at the int16 limits. PLACEHOLDER: rounding and
         * saturation are this fake's choices; the real firmware's are not documented (the SDK's own writes
         * truncate).
         */
        private static byte poseByte(int index, double xValue, double yValue, double hValue,
                                     double linearScale, double angularScale) {
            double value = index < 2 ? xValue * linearScale : index < 4 ? yValue * linearScale : hValue * angularScale;
            int count = saturateToInt16(value);
            return (byte) ((index % 2 == 0 ? count : count >> 8) & BYTE_MASK);
        }

        private static int saturateToInt16(double value) {
            long rounded = Math.round(value);
            if (rounded > Short.MAX_VALUE) {
                return Short.MAX_VALUE;
            }
            if (rounded < Short.MIN_VALUE) {
                return Short.MIN_VALUE;
            }
            return (int) rounded;
        }

        /**
         * A heading, radians, into the register's range: [-pi, pi) as a 16-bit count, so pi itself and the
         * rounding just under it wrap to -32768, the same angle.
         */
        private static double wrapToRegister(double headingRad) {
            double wrapped = shortWayRound(headingRad);
            return wrapped >= Math.PI - 0.5 * INT16_TO_RAD ? wrapped - 2.0 * Math.PI : wrapped;
        }

        @Override
        public void write8(int ireg, int bVal) {
            if (!connected) {
                return;
            }
            switch (ireg) {
                case REG_SCALAR_LINEAR:
                    linearScalarRegister = (byte) bVal;
                    return;
                case REG_SCALAR_ANGULAR:
                    angularScalarRegister = (byte) bVal;
                    return;
                case REG_IMU_CALIB:
                    calibrationSamples = bVal & BYTE_MASK;
                    calibrationElapsedSeconds = 0.0;
                    inTheInstantOfTheCalibrationWrite = true;
                    return;
                case REG_RESET:
                    if (bVal != RESET_TRACKING) {
                        throw new UnsupportedOperationException("FakeSparkFunOTOS: reset register value " + bVal
                                + "; the SDK 12.0 driver only writes " + RESET_TRACKING + " (reset tracking)");
                    }
                    resetTracking();
                    return;
                case REG_SIGNAL_PROCESS:
                    signalProcessConfig = bVal & BYTE_MASK;
                    return;
                case REG_SELF_TEST:
                    if (bVal != SELF_TEST_START) {
                        throw new UnsupportedOperationException("FakeSparkFunOTOS: self-test register value " + bVal
                                + "; the SDK 12.0 driver only writes " + SELF_TEST_START + " (start)");
                    }
                    selfTestRegister = selfTestPasses ? SELF_TEST_PASS : SELF_TEST_FAIL;
                    return;
                default:
                    throw new UnsupportedOperationException("FakeSparkFunOTOS: a one-byte write to register " + ireg
                            + "; the SDK 12.0 driver never makes one");
            }
        }

        @Override
        public void write(int ireg, byte[] data) {
            if (data.length != 6 || (ireg != REG_OFF_XL && ireg != REG_POS_XL)) {
                throw new UnsupportedOperationException("FakeSparkFunOTOS: a " + data.length
                        + "-byte write to register " + ireg + "; the SDK 12.0 driver only writes 6-byte poses to "
                        + REG_OFF_XL + " (offset) and " + REG_POS_XL + " (position)");
            }
            if (!connected) {
                return;
            }
            double decodedX = int16(data, 0) * INT16_TO_METER;
            double decodedY = int16(data, 2) * INT16_TO_METER;
            double decodedHeading = int16(data, 4) * INT16_TO_RAD;
            if (ireg == REG_OFF_XL) {
                // PLACEHOLDER: a new offset changes how motion from now on is read, and leaves the pose where
                // it is; what a real unit does to its pose on an offset write is not documented (the SDK's
                // sample writes the offset and then resets tracking, which makes the question moot).
                System.arraycopy(data, 0, offsetRegisters, 0, 6);
                declaredForwardMetres = decodedX;
                declaredLeftMetres = decodedY;
                declaredHeading = decodedHeading;
            } else {
                rebaseTo(decodedX, decodedY, decodedHeading);
            }
        }

        private static int int16(byte[] data, int at) {
            return (short) ((data[at] & BYTE_MASK) | (data[at + 1] << 8));
        }

        // --- Engagable: the driver's constructor engages the bus; nothing else is ever called. ---
        @Override public void engage() { engaged = true; }
        @Override public void disengage() { engaged = false; }
        @Override public boolean isEngaged() { return engaged; }

        // --- The rest of I2cDeviceSynch: the SDK 12.0 OTOS driver never calls it, so it fails loud. ---
        @Override public void setReadWindow(ReadWindow window) { throw notUsed("setReadWindow"); }
        @Override public ReadWindow getReadWindow() { throw notUsed("getReadWindow"); }
        @Override public void ensureReadWindow(ReadWindow windowNeeded, ReadWindow windowToSet) {
            throw notUsed("ensureReadWindow");
        }
        @Override public com.qualcomm.robotcore.hardware.TimestampedData readTimeStamped(
                int ireg, int creg, ReadWindow readWindowNeeded, ReadWindow readWindowSet) {
            throw notUsed("readTimeStamped");
        }
        @Override public void setHeartbeatInterval(int ms) { throw notUsed("setHeartbeatInterval"); }
        @Override public int getHeartbeatInterval() { throw notUsed("getHeartbeatInterval"); }
        @Override public void setHeartbeatAction(HeartbeatAction action) { throw notUsed("setHeartbeatAction"); }
        @Override public HeartbeatAction getHeartbeatAction() { throw notUsed("getHeartbeatAction"); }

        private static UnsupportedOperationException notUsed(String method) {
            return new UnsupportedOperationException("FakeSparkFunOTOS: I2cDeviceSynch." + method
                    + " is not modelled; the SDK 12.0 OTOS driver never calls it");
        }

        // ---------------------------------------------------------------------------------------------
        // Planar geometry (the same constant-twist step as FakeGoBildaPinpoint's)
        // ---------------------------------------------------------------------------------------------

        /** The vector (a, b) turned CCW by {@code angle} radians. */
        static double[] rotate(double a, double b, double angle) {
            double c = Math.cos(angle);
            double s = Math.sin(angle);
            return new double[] {c * a - s * b, s * a + c * b};
        }

        /** An angle, radians, into (-pi, pi]: the short way round. */
        static double shortWayRound(double angleRad) {
            double wrapped = angleRad % (2.0 * Math.PI);
            if (wrapped > Math.PI) {
                wrapped -= 2.0 * Math.PI;
            } else if (wrapped <= -Math.PI) {
                wrapped += 2.0 * Math.PI;
            }
            return wrapped;
        }

        /**
         * Arc to chord: a body that moves {@code arcForward} forward and {@code arcLeft} left in its own frame
         * while turning steadily by {@code turn} radians ends up displaced by this chord, in its frame at the
         * START of the move. chord = [[s, -c], [c, s]] * arc, with s = sin(turn)/turn, c = (1 - cos(turn))/turn.
         */
        static double[] chordFromArc(double arcForward, double arcLeft, double turn) {
            double s = sinOverAngle(turn);
            double c = oneMinusCosOverAngle(turn);
            return new double[] {s * arcForward - c * arcLeft, c * arcForward + s * arcLeft};
        }

        /** Chord to arc: the exact inverse of {@link #chordFromArc}, for the same turn. */
        static double[] arcFromChord(double chordForward, double chordLeft, double turn) {
            double s = sinOverAngle(turn);
            double c = oneMinusCosOverAngle(turn);
            double determinant = s * s + c * c;                      // never zero for |turn| <= pi
            return new double[] {
                    (s * chordForward + c * chordLeft) / determinant,
                    (-c * chordForward + s * chordLeft) / determinant};
        }

        private static double sinOverAngle(double a) {
            return Math.abs(a) < 1e-6 ? 1.0 - a * a / 6.0 : Math.sin(a) / a;
        }

        private static double oneMinusCosOverAngle(double a) {
            return Math.abs(a) < 1e-6 ? a / 2.0 : (1.0 - Math.cos(a)) / a;
        }
    }
}
