package org.horizon36596.simloop.fakehardware;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.util.TypeConversion;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.Arrays;

/**
 * Headless fake of the goBILDA Pinpoint odometry computer, for SimLoop users whose robots use one.
 * Horizon (FTC 36596) SimLoop.
 *
 * <p><b>The public API is the real driver's, unchanged.</b> This class IS a
 * {@code com.qualcomm.hardware.gobilda.GoBildaPinpointDriver} - the class FTC SDK 12.0 ships in its
 * {@code Hardware} artifact - and it overrides none of that driver's public methods. What it replaces is the
 * one thing a JVM does not have: the device on the other end of the I2C bus. The driver is built on
 * {@code Firmware}, an emulated Pinpoint that answers the driver's register reads and writes the way the
 * device's registers do. So every public method - the unit conversions, the heading normalisation, the
 * status decoding, the bulk-read parsing, the CRC check, the "is this reading corrupt" filter - runs the
 * SDK's own code, and behaves exactly as it does on a robot for the register values the device reports.
 * A reflection test pins that no public method is overridden.
 *
 * <p>This differs from {@code FakeOctoQuad} in the one place it has to. The OctoQuad's SDK type is an
 * interface, so its fake implements the interface; the Pinpoint's is a concrete driver, and re-implementing
 * its ~35 public methods would be re-deriving the SDK's code and hoping to match it. The rest follows
 * {@code FakeOctoQuad}: the same sim-side seam ({@link #attachChassisAt} and {@link #moveChassisTo}), the
 * same on-device integrator, the same fault injection, the same fail-loud rule for anything not modelled.
 *
 * <h2>What the emulated device does</h2>
 * <ul>
 *   <li><b>The pods are mounted the way the sim says, and read the way the robot configured.</b> Two
 *       separate things, as on a real robot. The <i>physical</i> mount - where each pod really sits, which
 *       way it really counts, how many ticks per millimetre it really makes - is sim-only, set with
 *       {@link #setPhysicalPodMount} (default: goBILDA 4-bar pods on the tracking centre, counting up
 *       forward and left). The <i>configured</i> mount is what the robot code wrote with
 *       {@code setOffsets}, {@code setEncoderDirections} and {@code setEncoderResolution}. {@link
 *       #moveChassisTo} turns the chassis's motion into pod counts through the physical mount; the emulated
 *       firmware turns those counts back into a pose through the configured one. When the two agree the
 *       pose is the chassis's motion; when a robot sets a pod direction, offset or resolution wrong, its
 *       pose is wrong the way a real Pinpoint's would be.</li>
 *   <li><b>Pose</b> is an on-device integrator, as on the real board: each step it takes the change in the
 *       two pod counts and the IMU's turn, and adds the motion they describe onto whatever pose it holds -
 *       the pose last written with {@code setPosition}/{@code setPosX}/{@code setPosY}/{@code setHeading},
 *       or zero after {@code resetPosAndIMU()}. Frame: X forward, Y left, heading CCW-positive; millimetres
 *       and radians in the registers, as the driver expects. The heading register is continuous (not
 *       wrapped), which is what lets the driver's {@code getHeading(UnnormalizedAngleUnit)} exist; the
 *       driver wraps it for {@code getHeading(AngleUnit)} itself. Within one step the chassis is taken to
 *       turn at a steady rate, so the step is an arc, both when making pod counts and when integrating
 *       them.</li>
 *   <li><b>The IMU is ideal</b> except for the yaw scalar, which the device multiplies into every heading
 *       change, exactly as it corrects a real IMU - so a scalar other than 1 makes the fake's heading wrong
 *       by that factor, as it would a perfect IMU's. PLACEHOLDER: the pod counts are integrated exactly,
 *       before they are rounded to the whole numbers the encoder registers report, so the sim adds no
 *       count quantisation (a 4-bar pod's is 0.05 mm).</li>
 *   <li><b>Velocity</b> is the pose's change over the last {@link #update(double)} tick, in the same frame
 *       as the pose, mm/s and rad/s.</li>
 *   <li><b>Status</b> is {@code READY} unless calibrating or forced ({@link #forceFault}).
 *       {@code recalibrateIMU()} and {@code resetPosAndIMU()} put it in {@code CALIBRATING} for
 *       {@link #CALIBRATION_SECONDS} of sim time. PLACEHOLDER: while calibrating, the pose ignores chassis
 *       motion (the device must be still to calibrate; what a real unit does if it is moved then is not
 *       documented), but the raw pod counts keep counting, as quadrature counters do.
 *       {@code resetPosAndIMU()} also zeroes the pose and the velocity. PLACEHOLDER: a new fake is READY at
 *       once. A real unit powers up {@code NOT_READY} and calibrates first, but a SimLoop scenario starts
 *       where robot code starts, long after the robot was switched on.</li>
 *   <li><b>A written pose is not motion.</b> {@code setPosition}/{@code setPosX}/{@code setPosY}/
 *       {@code setHeading} move the pose without any velocity, as on the real board, whose velocity comes
 *       from the pods and the IMU.</li>
 *   <li><b>Firmware version</b> decides the bulk-read format, as in the driver: versions 1 and 2 answer the
 *       fixed 40-byte bulk read, 3 and later the configurable one ({@code setBulkReadScope}) and CRC-checked
 *       reads ({@code setErrorDetectionType(CRC)}). The driver itself refuses what a version cannot do.
 *       PLACEHOLDER: firmware 1 and 2 are taken to have no quaternion, pitch or roll registers, so the fake
 *       fails loud if one is read. The driver does read them on old firmware when it has not yet learnt the
 *       version; what a real v1 or v2 unit answers then is not known.</li>
 * </ul>
 *
 * <p><b>Getters are not always pure reads here</b>, unlike the other fakes (conventions of
 * {@link FakeDevice}). The real driver reads a register over the bus, on every call and on every firmware,
 * whenever its getter's register is not in the driver's bulk-read scope. The default scope holds the ten
 * registers {@code update()} refreshes (status, loop time, both encoders, pose, velocity), so
 * {@code getDeviceID}, {@code getDeviceVersion}, {@code getYawScalar}, {@code getXOffset},
 * {@code getYOffset}, {@code getQuaternion}, {@code getPitch} and {@code getRoll} always read the bus; on
 * firmware 3 and up, a narrower {@code setBulkReadScope} adds every register it leaves out. That is the
 * SDK's own behaviour, so the fake keeps it; {@link #busReadCount()} shows it.
 *
 * <p>Determinism (domain R5): no wall clock, no randomness. The device changes only when something writes
 * a register, when {@link #moveChassisTo} reports motion, or when {@link #update(double)} ticks it.
 */
public class FakeGoBildaPinpoint extends GoBildaPinpointDriver implements FakeDevice {

    /**
     * The firmware version a new fake reports. SET BY HAND: 3, the first version the SDK 12.0 driver reads
     * with the configurable bulk read and CRC (every later version is read the same way).
     */
    public static final int DEFAULT_FIRMWARE_VERSION = 3;

    /**
     * How long an IMU calibration keeps the device in {@code CALIBRATING}, seconds of sim time. PLACEHOLDER:
     * a quarter second, from the SDK driver's javadoc for {@code recalibrateIMU()} and
     * {@code resetPosAndIMU()} ("This takes approximately 0.25 seconds"); not measured on a unit.
     */
    public static final double CALIBRATION_SECONDS = 0.25;

    /**
     * The device's own loop time, microseconds, as its {@code LOOP_TIME} register reports it. PLACEHOLDER:
     * 667 us, about 1.5 kHz, inside the range the SDK driver's javadoc for {@code getLoopTime()} calls
     * healthy (it says values commonly below 500 or above 1100 mean something is wrong with the device);
     * not measured on a unit. It must be at least 2, because on firmware 1 and 2 the driver's fixed bulk
     * read treats a loop time of 1 or less as a corrupt read (the configurable read of firmware 3 and up
     * does not check it).
     */
    public static final int LOOP_TIME_MICROSECONDS = 667;

    /** What the {@code DEVICE_ID} register reports. PLACEHOLDER: 1; nothing in the driver checks it. */
    public static final int DEVICE_ID = 1;

    /**
     * The goBILDA 4-bar odometry pod's resolution, ticks per millimetre. SPEC: the SDK driver's own
     * {@code goBILDA_4_BAR_POD} constant (19.89436789 ticks/mm). Used twice: as the resolution the device's
     * register holds before anyone sets one (PLACEHOLDER: the driver does not say what a real unit starts
     * with), and as the resolution of the sim's default physical pods ({@link #setPhysicalPodMount}).
     */
    public static final float DEFAULT_TICKS_PER_MM = 19.89436789f;

    private final Firmware firmware;

    /** A Pinpoint at the origin, READY, reporting {@link #DEFAULT_FIRMWARE_VERSION}. */
    public FakeGoBildaPinpoint() {
        this(DEFAULT_FIRMWARE_VERSION);
    }

    /**
     * A Pinpoint at the origin, READY, reporting the given firmware version.
     *
     * @param firmwareVersion the version the {@code DEVICE_VERSION} register reports; 1 or 2 selects the
     *                        fixed bulk read, 3 and up the configurable one; must be positive
     */
    public FakeGoBildaPinpoint(int firmwareVersion) {
        this(new Firmware(firmwareVersion));
    }

    private FakeGoBildaPinpoint(Firmware firmware) {
        super(firmware, true);
        this.firmware = firmware;
    }

    /**
     * The one driver hook overridden. The real one casts the bus to the REV hub's {@code LynxI2cDeviceSynch}
     * to set its speed to 400 kHz; this bus is not a REV hub and has no speed. Called only by
     * {@code initialize()}, which the SDK runs when the hardware is armed.
     */
    @Override
    protected synchronized boolean doInitialize() {
        return true;
    }

    // =================================================================================================
    // Sim input: the chassis moving under the device (the FakeOctoQuad seam)
    // =================================================================================================

    /**
     * Tell the device where the sim's chassis is before any motion is reported, without moving the device's
     * pose. Call it once, when a drive plant is wired to this device, with that plant's current pose.
     *
     * <p><b>Millimetres, not inches.</b> SimLoop's drive plants report inches, so multiply by 25.4 first:
     * {@code attachChassisAt(plant.getX() * 25.4, plant.getY() * 25.4, plant.getHeading())}. Nothing can
     * catch inches passed here; the robot would simply read a pose 25.4 times too small.
     *
     * <p><b>Call it again after any teleport of the plant</b> ({@code setPose} on a drive plant, a scenario
     * placing the robot). {@link #moveChassisTo} treats every change as driven motion, so a teleport not
     * announced here reaches the pods as one enormous step - a jump in the pose and a velocity spike that
     * the real driver then refuses as a bad read.
     *
     * <p><b>Wiring a robot that configures its Pinpoint also means calling {@link #setPhysicalPodMount}</b>
     * with the same pod offsets, directions and resolution the robot writes. The default physical mount is
     * pods on the centre at 4-bar resolution, counting forward; a robot that writes its real offsets or
     * resolution against that default drifts while it turns, or reads every distance at the wrong scale,
     * exactly as a real robot whose settings do not match its pods would.
     *
     * @param plantXMm        the chassis X in the drive plant's frame (forward at heading 0), millimetres
     * @param plantYMm        the chassis Y in the drive plant's frame (left at heading 0), millimetres
     * @param plantHeadingRad the chassis heading in the drive plant's frame, radians CCW
     */
    public void attachChassisAt(double plantXMm, double plantYMm, double plantHeadingRad) {
        firmware.chassisX = plantXMm;
        firmware.chassisY = plantYMm;
        firmware.chassisHeading = plantHeadingRad;
    }

    /**
     * The chassis is now here, in the drive plant's frame. The pods roll through the motion since the last
     * call and count it, through the physical mount ({@link #setPhysicalPodMount}); the emulated firmware
     * adds the motion those counts and the IMU describe onto its own pose, as the real board does - so a pose
     * the robot wrote stays written, and the plant and the device do not need to share an origin. While
     * calibrating, the pose ignores the motion; the raw pod counts do not. Millimetres, as
     * {@link #attachChassisAt}: convert a plant's inches first.
     *
     * <p>The heading may be wrapped or not. The turn since the last call is taken the short way round, into
     * (-pi, pi], so a plant whose heading jumps from +pi to -pi has turned a little, not most of a circle;
     * that means one call must never be asked to report a turn of half a circle or more.
     *
     * @param plantXMm        the chassis X in the drive plant's frame (forward at heading 0), millimetres
     * @param plantYMm        the chassis Y in the drive plant's frame (left at heading 0), millimetres
     * @param plantHeadingRad the chassis heading in the drive plant's frame, radians CCW, wrapped or not
     */
    public void moveChassisTo(double plantXMm, double plantYMm, double plantHeadingRad) {
        firmware.moveChassisTo(plantXMm, plantYMm, plantHeadingRad);
    }

    /**
     * Sim only: how the two odometry pods are REALLY mounted on the simulated robot - not what the robot
     * code tells the device ({@code setOffsets}, {@code setEncoderDirections}, {@code setEncoderResolution}),
     * but the truth those settings are supposed to describe. The pods count the chassis's motion through
     * this mount; the device reads the counts through the configured one. Setting the two differently is
     * how a test shows what a robot with a mis-set Pinpoint would read. It takes effect from the next
     * {@link #moveChassisTo}; counts already made stay.
     *
     * <p>Default: goBILDA 4-bar pods ({@link #DEFAULT_TICKS_PER_MM}) on the tracking centre, both counting
     * up in the device's positive direction. PLACEHOLDER: a robot's real pods are rarely on the centre; the
     * default only makes a robot that configures nothing read the chassis exactly, because the device's own
     * offset registers start at zero too.
     *
     * @param xPodOffsetMm   how far LEFT of the tracking centre the X (forward-measuring) pod really is,
     *                       millimetres, right negative - the same sign convention as {@code setOffsets}
     * @param yPodOffsetMm   how far FORWARD of the tracking centre the Y (strafe-measuring) pod really is,
     *                       millimetres, backward negative - the same sign convention as {@code setOffsets}
     * @param xPodDirection  {@code FORWARD} if the X pod really counts up as the robot drives forward,
     *                       {@code REVERSED} if it counts down - that is, the setting the robot has to pass
     *                       to {@code setEncoderDirections} to read it correctly
     * @param yPodDirection  {@code FORWARD} if the Y pod really counts up as the robot drives left,
     *                       {@code REVERSED} if it counts down
     * @param podTicksPerMm  how many counts both pods really make per millimetre rolled; must be positive
     */
    public void setPhysicalPodMount(double xPodOffsetMm, double yPodOffsetMm,
                                    EncoderDirection xPodDirection, EncoderDirection yPodDirection,
                                    double podTicksPerMm) {
        if (!(podTicksPerMm > 0.0) || Double.isInfinite(podTicksPerMm)) {
            throw new IllegalArgumentException("pod ticks per mm must be positive and finite, got " + podTicksPerMm);
        }
        firmware.physicalXPodOffsetMm = xPodOffsetMm;
        firmware.physicalYPodOffsetMm = yPodOffsetMm;
        firmware.physicalXPodSign = xPodDirection == EncoderDirection.FORWARD ? 1 : -1;
        firmware.physicalYPodSign = yPodDirection == EncoderDirection.FORWARD ? 1 : -1;
        firmware.physicalTicksPerMm = podTicksPerMm;
    }

    /**
     * Advance the device's own clock: finish a calibration that has run its time, and work out the velocity
     * the device reports from the motion since the last tick.
     *
     * <p>Not the driver's {@code update()}, which reads the device; this is the device itself running. A
     * scenario calls this (or {@code FakeHardwareMap.updateAll}) once per tick, then the robot code calls
     * {@code update()} as it would on the robot. The two share a name because every SimLoop fake's tick is
     * {@link FakeDevice#update(double)}; a scenario that calls only one of them sees stale readings, exactly
     * as a robot that never calls {@code update()} does.
     *
     * @param deltaTime the tick, seconds
     */
    @Override
    public void update(double deltaTime) {
        firmware.tick(deltaTime);
    }

    // =================================================================================================
    // Test-only fault injection and instrumentation
    // =================================================================================================

    /**
     * Make the device report this status, as if it had the fault. It stays until {@link #clearFault()}.
     *
     * <p>Not {@code FAULT_BAD_READ}: the device never reports that. It is the status the SDK driver sets
     * itself when it catches a corrupt read (a failed CRC, a NaN, an impossible jump), so a test that wants
     * it uses {@link #corruptNextBulkRead()} and lets the real driver catch it.
     *
     * @param status what {@code getDeviceStatus()} should decode to after the next read
     * @throws IllegalArgumentException for {@code FAULT_BAD_READ}
     */
    public void forceFault(DeviceStatus status) {
        if (status == DeviceStatus.FAULT_BAD_READ) {
            throw new IllegalArgumentException("FAULT_BAD_READ is set by the SDK driver on a corrupt read, not"
                    + " reported by the device; use corruptNextBulkRead() to give the driver one to catch");
        }
        firmware.forcedStatusBits = Firmware.statusBits(status);
    }

    /**
     * Make the next bulk read (the one {@code update()} makes) arrive corrupt, as a bad I2C transfer does:
     * every data byte 0xFF - a status and encoder counts of -1, and NaN for every pose and velocity float -
     * and, on firmware 3, a CRC byte that does not match. The real driver's own checks then decide what
     * happens: with CRC detection it discards the whole read and sets {@code FAULT_BAD_READ}. With its
     * default local test and a scope that holds pose or velocity registers (the default scope does, and the
     * fixed read of firmware 1 and 2 always does), it refuses the NaN floats, keeps their last good values,
     * lets the integer registers it does not check through, and sets {@code FAULT_BAD_READ}. With the local
     * test and a scope of integer registers only, the driver checks nothing and the garbage gets through
     * with no fault; so it does with detection {@code NONE}. PLACEHOLDER: all-0xFF is one kind of corrupt
     * read, chosen because the driver's checks catch it in the default setup; a real bus fault can look like
     * anything.
     */
    public void corruptNextBulkRead() {
        firmware.corruptNextBulkRead = true;
    }

    /** Stop forcing a status; the device reports READY or CALIBRATING again. */
    public void clearFault() {
        firmware.forcedStatusBits = -1;
    }

    /**
     * {@return how many I2C reads the driver has made} Real hardware has no such counter; it exists so a
     * test can pin, say, one bulk read per {@code update()}.
     */
    public int busReadCount() {
        return firmware.reads;
    }

    // =================================================================================================
    // The emulated device
    // =================================================================================================

    /**
     * The Pinpoint on the far side of the bus: its register file, and what it does when one is written.
     * Registers are 4 bytes, little-endian, as the driver reads them; the bulk read is several of them back
     * to back. A read asked for one byte more than the register holds gets a CRC-8 appended on firmware 3,
     * the check the driver makes in CRC mode (polynomial 0x31, initial value 0x90, no final XOR - the
     * driver's own); older firmware has no CRC (see {@link #OLD_FIRMWARE_FIFTH_BYTE}).
     *
     * <p>A bulk-read scope is kept exactly as the driver writes it: its registers once each, in order. The
     * driver itself keeps the scope as it was passed, repeats and all, and asks for 4 bytes per entry, so a
     * scope with a repeated register, or one that names {@code BULK_READ} or {@code SET_BULK_READ}, fails
     * loud here: what a real unit sends back for one is not known.
     */
    private static final class Firmware extends FakeI2cDeviceSynchSimple {
        // Register addresses, as the driver's Register enum numbers them (SDK 12.0).
        static final int DEVICE_ID_REGISTER = 1;
        static final int DEVICE_VERSION = 2;
        static final int DEVICE_STATUS = 3;
        static final int DEVICE_CONTROL = 4;
        static final int LOOP_TIME = 5;
        static final int X_ENCODER_VALUE = 6;
        static final int Y_ENCODER_VALUE = 7;
        static final int X_POSITION = 8;
        static final int Y_POSITION = 9;
        static final int H_ORIENTATION = 10;
        static final int X_VELOCITY = 11;
        static final int Y_VELOCITY = 12;
        static final int H_VELOCITY = 13;
        static final int MM_PER_TICK = 14;          // holds ticks per mm, whatever its name says
        static final int X_POD_OFFSET = 15;
        static final int Y_POD_OFFSET = 16;
        static final int YAW_SCALAR = 17;
        static final int BULK_READ = 18;
        static final int QUATERNION_W = 19;
        static final int QUATERNION_X = 20;
        static final int QUATERNION_Y = 21;
        static final int QUATERNION_Z = 22;
        static final int PITCH = 23;
        static final int ROLL = 24;
        static final int SET_BULK_READ = 25;

        /** The fixed bulk read of firmware 1 and 2, in the order the driver unpacks it. */
        static final int[] FIXED_BULK_READ = {
                DEVICE_STATUS, LOOP_TIME, X_ENCODER_VALUE, Y_ENCODER_VALUE,
                X_POSITION, Y_POSITION, H_ORIENTATION, X_VELOCITY, Y_VELOCITY, H_VELOCITY};

        // DEVICE_CONTROL bits, as the driver's DeviceControl enum writes them.
        static final int RECALIBRATE_IMU = 1;
        static final int RESET_POS_AND_IMU = 2;
        static final int SET_Y_ENCODER_REVERSED = 4;
        static final int SET_Y_ENCODER_FORWARD = 8;
        static final int SET_X_ENCODER_REVERSED = 16;
        static final int SET_X_ENCODER_FORWARD = 32;

        // DEVICE_STATUS bits, as the driver's DeviceStatus enum decodes them.
        static final int STATUS_READY = 1;
        static final int STATUS_CALIBRATING = 2;

        /**
         * The byte firmware 1 and 2 send after a register's four, where firmware 3 sends a CRC. The driver
         * always reads five bytes and always checks the fifth, whatever its error-detection mode, and a
         * failed check sets its status to {@code FAULT_BAD_READ} until the next bulk read overwrites it.
         * NEEDS MEASUREMENT: zero is a stand-in; what a real v1 or v2 unit sends is not known, so neither is
         * whether a real one shows that status after, say, {@code getDeviceVersion()}.
         */
        static final byte OLD_FIRMWARE_FIFTH_BYTE = 0;

        /** How close to zero a calibration's remaining time counts as finished, seconds: one nanosecond. */
        static final double CALIBRATION_END_TOLERANCE_SECONDS = 1e-9;

        final int version;

        // The device's pose, in its own frame (X forward, Y left), mm and radians, held at full precision
        // and rounded to float only on the way out, as the firmware's registers are floats.
        double x;
        double y;
        double heading;
        double velocityX;
        double velocityY;
        double velocityHeading;
        private double xAtLastTick;
        private double yAtLastTick;
        private double headingAtLastTick;

        // The pod counts the encoder registers report, accumulated exactly and rounded only on the way out.
        // They already include the configured directions (see control()).
        double xTicks;
        double yTicks;

        // Configuration registers: the mount as the robot code describes it.
        float ticksPerMm = DEFAULT_TICKS_PER_MM;
        float xPodOffsetMm = 0f;
        float yPodOffsetMm = 0f;
        float yawScalar = 1f;
        int xDirection = 1;
        int yDirection = 1;
        int[] bulkScope = FIXED_BULK_READ.clone();

        // The mount as it really is on the simulated robot (setPhysicalPodMount). Not registers.
        double physicalXPodOffsetMm = 0.0;
        double physicalYPodOffsetMm = 0.0;
        int physicalXPodSign = 1;
        int physicalYPodSign = 1;
        double physicalTicksPerMm = DEFAULT_TICKS_PER_MM;

        double calibrationRemaining = 0.0;
        int forcedStatusBits = -1;
        boolean corruptNextBulkRead = false;
        int reads = 0;

        // Where the sim's drive plant last said the chassis was, in the plant's frame. Not a register.
        double chassisX;
        double chassisY;
        double chassisHeading;

        Firmware(int version) {
            if (version <= 0) {
                throw new IllegalArgumentException("firmware version must be positive, got " + version);
            }
            this.version = version;
        }

        /** The status register's bits for a {@link DeviceStatus}, as the driver's enum numbers them. */
        static int statusBits(DeviceStatus status) {
            switch (status) {
                case NOT_READY: return 0;
                case READY: return 1;
                case CALIBRATING: return 2;
                case FAULT_X_POD_NOT_DETECTED: return 4;
                case FAULT_Y_POD_NOT_DETECTED: return 8;
                case FAULT_NO_PODS_DETECTED: return 12;
                case FAULT_IMU_RUNAWAY: return 16;
                case FAULT_BAD_READ: return 32;
                default: throw new IllegalArgumentException("unknown status " + status);
            }
        }

        void moveChassisTo(double plantX, double plantY, double plantHeading) {
            // --- The world: the chassis moves, and the pods roll and count through the PHYSICAL mount. ---

            // The step, in the chassis's own frame at the start of it (chord forward, chord left), and the
            // turn, taken the short way round so a plant that wraps its heading at +-pi is not a jump.
            double dx = plantX - chassisX;
            double dy = plantY - chassisY;
            double cos = Math.cos(chassisHeading);
            double sin = Math.sin(chassisHeading);
            double chordForward = cos * dx + sin * dy;
            double chordLeft = -sin * dx + cos * dy;
            double turn = shortWayRound(plantHeading - chassisHeading);
            chassisX = plantX;
            chassisY = plantY;
            chassisHeading = plantHeading;

            // Distance travelled along the arc, forward and left in the chassis's own frame, that carries the
            // tracking centre along that chord while turning at a steady rate.
            double[] arc = arcFromChord(chordForward, chordLeft, turn);
            double arcForward = arc[0];
            double arcLeft = arc[1];

            // What each pod rolls. The X pod sits physicalXPodOffsetMm left of the tracking centre and
            // measures forward travel, so a CCW turn rolls it backwards; the Y pod sits physicalYPodOffsetMm
            // forward and measures leftward travel, so a CCW turn rolls it left (rigid-body velocity of a
            // point: v + omega x r).
            double xPodRolledMm = arcForward - physicalXPodOffsetMm * turn;
            double yPodRolledMm = arcLeft + physicalYPodOffsetMm * turn;

            // The counts the device sees: the pod's real direction and resolution, then the direction the
            // robot configured (see control()).
            double xCountChange = xDirection * physicalXPodSign * xPodRolledMm * physicalTicksPerMm;
            double yCountChange = yDirection * physicalYPodSign * yPodRolledMm * physicalTicksPerMm;
            xTicks += xCountChange;
            yTicks += yCountChange;

            if (calibrationRemaining > 0.0) {
                return;                                  // calibrating: the pose is not tracking (PLACEHOLDER)
            }

            // --- The firmware: counts and IMU back into a pose, through the CONFIGURED mount. ---

            double imuTurn = turn * yawScalar;
            double xPodMeasuredMm = xCountChange / ticksPerMm;
            double yPodMeasuredMm = yCountChange / ticksPerMm;
            // Take out the part of each pod's travel that is the robot turning about the configured centre.
            double measuredArcForward = xPodMeasuredMm + xPodOffsetMm * imuTurn;
            double measuredArcLeft = yPodMeasuredMm - yPodOffsetMm * imuTurn;
            double[] chord = chordFromArc(measuredArcForward, measuredArcLeft, imuTurn);

            // That chord, in the robot's frame, applied where the device thinks the robot is.
            double cosDevice = Math.cos(heading);
            double sinDevice = Math.sin(heading);
            x += cosDevice * chord[0] - sinDevice * chord[1];
            y += sinDevice * chord[0] + cosDevice * chord[1];
            heading += imuTurn;
        }

        /** An angle difference, radians, into (-pi, pi]: the short way round. */
        static double shortWayRound(double angleRad) {
            double wrapped = angleRad % (2.0 * Math.PI);              // (-2pi, 2pi)
            if (wrapped > Math.PI) {
                wrapped -= 2.0 * Math.PI;
            } else if (wrapped <= -Math.PI) {
                wrapped += 2.0 * Math.PI;
            }
            return wrapped;
        }

        /**
         * Arc to chord. A body that moves {@code arcForward} forward and {@code arcLeft} left in its own frame
         * while turning steadily by {@code turn} radians ends up displaced by this chord, forward and left in
         * its frame at the START of the move, millimetres. For a straight move (no turn) chord and arc are
         * the same. This is the standard constant-twist step:
         * chord = [[s, -c], [c, s]] * arc, with s = sin(turn)/turn and c = (1 - cos(turn))/turn.
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

        /** sin(a)/a, with its limit 1 at a = 0 (a Taylor series near zero, to avoid dividing by ~0). */
        private static double sinOverAngle(double a) {
            return Math.abs(a) < 1e-6 ? 1.0 - a * a / 6.0 : Math.sin(a) / a;
        }

        /** (1 - cos(a))/a, with its limit 0 at a = 0 (a Taylor series near zero). */
        private static double oneMinusCosOverAngle(double a) {
            return Math.abs(a) < 1e-6 ? a / 2.0 : (1.0 - Math.cos(a)) / a;
        }

        void tick(double deltaTime) {
            if (calibrationRemaining > 0.0) {
                // Stop within a nanosecond of zero: 0.25 s less five 0.05 s ticks is 1.4e-17 in doubles, and
                // that rounding must not keep the device calibrating for a whole extra tick.
                calibrationRemaining -= deltaTime;
                if (calibrationRemaining < CALIBRATION_END_TOLERANCE_SECONDS) {
                    calibrationRemaining = 0.0;
                }
            }
            if (deltaTime > 0.0) {
                velocityX = (x - xAtLastTick) / deltaTime;
                velocityY = (y - yAtLastTick) / deltaTime;
                velocityHeading = (heading - headingAtLastTick) / deltaTime;
            }
            xAtLastTick = x;
            yAtLastTick = y;
            headingAtLastTick = heading;
        }

        private int statusRegister() {
            if (forcedStatusBits >= 0) {
                return forcedStatusBits;
            }
            return calibrationRemaining > 0.0 ? STATUS_CALIBRATING : STATUS_READY;
        }

        private void startCalibration() {
            calibrationRemaining = CALIBRATION_SECONDS;
        }

        /** One register's four bytes, as the device would send them. */
        private byte[] register(int address) {
            switch (address) {
                case DEVICE_ID_REGISTER: return integer(DEVICE_ID);
                case DEVICE_VERSION: return integer(version);
                case DEVICE_STATUS: return integer(statusRegister());
                case DEVICE_CONTROL: return integer(0);
                case LOOP_TIME: return integer(LOOP_TIME_MICROSECONDS);
                case X_ENCODER_VALUE: return integer((int) Math.round(xTicks));
                case Y_ENCODER_VALUE: return integer((int) Math.round(yTicks));
                case X_POSITION: return real(x);
                case Y_POSITION: return real(y);
                case H_ORIENTATION: return real(heading);
                case X_VELOCITY: return real(velocityX);
                case Y_VELOCITY: return real(velocityY);
                case H_VELOCITY: return real(velocityHeading);
                case MM_PER_TICK: return real(ticksPerMm);
                case X_POD_OFFSET: return real(xPodOffsetMm);
                case Y_POD_OFFSET: return real(yPodOffsetMm);
                case YAW_SCALAR: return real(yawScalar);
                case QUATERNION_W:
                case QUATERNION_X:
                case QUATERNION_Y:
                case QUATERNION_Z:
                case PITCH:
                case ROLL:
                    return orientation(address);
                default:
                    throw new UnsupportedOperationException("FakeGoBildaPinpoint has no register " + address
                            + " to read; the SDK 12.0 driver never asks for one");
            }
        }

        /**
         * The quaternion, pitch and roll registers. PLACEHOLDER: taken to exist from firmware 3, the version
         * whose driver methods for them work; the driver itself reads them from older firmware when it has
         * not yet read the version, and what a v1 or v2 unit answers is not known, so the fake fails loud.
         */
        private byte[] orientation(int address) {
            if (version <= 2) {
                throw new UnsupportedOperationException("FakeGoBildaPinpoint: firmware " + version
                        + " has no register " + address + " (quaternion, pitch and roll arrived in firmware 3)");
            }
            switch (address) {
                case QUATERNION_W: return real(Math.cos(heading / 2.0));
                case QUATERNION_X: return real(0.0);          // flat floor: no roll or pitch
                case QUATERNION_Y: return real(0.0);
                case QUATERNION_Z: return real(Math.sin(heading / 2.0));
                case PITCH: return real(0.0);
                default: return real(0.0);                    // ROLL
            }
        }

        @Override
        public byte[] read(int address, int length) {
            reads++;
            byte[] data;
            if (address == BULK_READ) {
                int[] layout = version <= 2 ? FIXED_BULK_READ : bulkScope;
                ByteBuffer bulk = ByteBuffer.allocate(4 * layout.length);
                for (int register : layout) {
                    bulk.put(register(register));
                }
                data = bulk.array();
                if (corruptNextBulkRead) {
                    return corrupted(data, length);
                }
            } else {
                data = register(address);
            }
            if (length == data.length) {
                return data;
            }
            if (length == data.length + 1) {
                // The driver always asks for one byte more than the data and checks it as a CRC. Firmware 3
                // answers with the CRC. Firmware 1 and 2 have none, but an I2C read returns as many bytes as
                // were clocked, so the driver gets a byte that is not one (NEEDS MEASUREMENT, see the constant).
                byte[] withCrc = Arrays.copyOf(data, data.length + 1);
                withCrc[data.length] = version >= 3 ? crc8(data) : OLD_FIRMWARE_FIFTH_BYTE;
                return withCrc;
            }
            throw new UnsupportedOperationException("FakeGoBildaPinpoint: a read of register " + address + " for "
                    + length + " bytes; it holds " + data.length + " (or one more with a CRC); the SDK 12.0 driver"
                    + " never asks for any other length");
        }

        /**
         * {@link FakeGoBildaPinpoint#corruptNextBulkRead()}: every data byte 0xFF, and a CRC byte (firmware 3)
         * that is one bit off the right one, so it never matches. Firmware 1 and 2 bulk reads carry no CRC.
         */
        private byte[] corrupted(byte[] data, int length) {
            corruptNextBulkRead = false;
            byte[] garbage = new byte[data.length];
            Arrays.fill(garbage, (byte) 0xFF);
            if (length == data.length) {
                return garbage;
            }
            if (length == data.length + 1 && version >= 3) {
                byte[] withBadCrc = Arrays.copyOf(garbage, data.length + 1);
                withBadCrc[data.length] = (byte) (crc8(garbage) ^ 0x01);
                return withBadCrc;
            }
            throw new UnsupportedOperationException("FakeGoBildaPinpoint: a bulk read for " + length
                    + " bytes; it holds " + data.length + " (or one more with a CRC on firmware 3)");
        }

        @Override
        public void write(int address, byte[] data) {
            switch (address) {
                case DEVICE_CONTROL:
                    control(TypeConversion.byteArrayToInt(data, ByteOrder.LITTLE_ENDIAN));
                    return;
                // A written pose moves the last tick's pose with it, so the jump is not reported as velocity.
                case X_POSITION:
                    xAtLastTick += asFloat(data) - x;
                    x = asFloat(data);
                    return;
                case Y_POSITION:
                    yAtLastTick += asFloat(data) - y;
                    y = asFloat(data);
                    return;
                case H_ORIENTATION:
                    headingAtLastTick += asFloat(data) - heading;
                    heading = asFloat(data);
                    return;
                case MM_PER_TICK:
                    ticksPerMm = asFloat(data);
                    return;
                case X_POD_OFFSET:
                    xPodOffsetMm = asFloat(data);
                    return;
                case Y_POD_OFFSET:
                    yPodOffsetMm = asFloat(data);
                    return;
                case YAW_SCALAR:
                    yawScalar = asFloat(data);
                    return;
                case SET_BULK_READ:
                    int[] scope = new int[data.length];
                    for (int i = 0; i < data.length; i++) {
                        scope[i] = data[i];
                    }
                    bulkScope = scope;
                    return;
                default:
                    throw new UnsupportedOperationException("FakeGoBildaPinpoint: register " + address
                            + " is read-only on a Pinpoint; the SDK 12.0 driver never writes it");
            }
        }

        /**
         * A write to DEVICE_CONTROL. PLACEHOLDER, both: a reset zeroes the pose but not the pod counts (what a
         * real unit does to its encoder registers is not documented); and a direction setting applies to
         * the counts from then on, the ones already made keep their sign. That the reported counts include
         * the configured direction at all follows the driver's javadoc for {@code setEncoderDirections}
         * ("X pod should increase when the robot is moving forward").
         */
        private void control(int bits) {
            if ((bits & RESET_POS_AND_IMU) != 0) {
                x = 0.0;
                y = 0.0;
                heading = 0.0;
                xAtLastTick = 0.0;
                yAtLastTick = 0.0;
                headingAtLastTick = 0.0;
                velocityX = 0.0;
                velocityY = 0.0;
                velocityHeading = 0.0;
                startCalibration();
            }
            if ((bits & RECALIBRATE_IMU) != 0) {
                startCalibration();
            }
            if ((bits & SET_X_ENCODER_REVERSED) != 0) {
                xDirection = -1;
            }
            if ((bits & SET_X_ENCODER_FORWARD) != 0) {
                xDirection = 1;
            }
            if ((bits & SET_Y_ENCODER_REVERSED) != 0) {
                yDirection = -1;
            }
            if ((bits & SET_Y_ENCODER_FORWARD) != 0) {
                yDirection = 1;
            }
        }

        private static byte[] integer(int value) {
            return TypeConversion.intToByteArray(value, ByteOrder.LITTLE_ENDIAN);
        }

        private static byte[] real(double value) {
            return ByteBuffer.allocate(4).order(ByteOrder.LITTLE_ENDIAN).putFloat((float) value).array();
        }

        private static float asFloat(byte[] data) {
            return ByteBuffer.wrap(data).order(ByteOrder.LITTLE_ENDIAN).getFloat();
        }

        /** CRC-8, polynomial 0x31, initial value 0x90, no final XOR: the check the driver computes. */
        static byte crc8(byte[] data) {
            int crc = 0x90;
            for (byte b : data) {
                crc ^= b & 0xFF;
                for (int bit = 0; bit < 8; bit++) {
                    crc = (crc & 0x80) != 0 ? ((crc << 1) ^ 0x31) & 0xFF : (crc << 1) & 0xFF;
                }
            }
            return (byte) crc;
        }
    }
}
