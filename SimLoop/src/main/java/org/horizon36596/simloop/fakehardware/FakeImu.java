package org.horizon36596.simloop.fakehardware;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.ImuOrientationOnRobot;
import com.qualcomm.robotcore.hardware.QuaternionBasedImuHelper;

import org.firstinspires.ftc.robotcore.external.function.ThrowingSupplier;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

/**
 * Fake of the SDK's universal {@link IMU} — the interface a team gets back from
 * {@code hardwareMap.get(IMU.class, "imu")} on a Control Hub or Expansion Hub — driven by the drive plant's
 * heading. Register it in a {@link FakeHardwareMap} under the robot's IMU config name and robot code that
 * reads the IMU runs unchanged.
 *
 * <p><b>The frame arithmetic is the SDK's, not a re-derivation.</b> Every {@link IMU} method goes through the
 * SDK's own {@link QuaternionBasedImuHelper}, the class the SDK 12.0 {@code BHI260IMU} driver delegates the same
 * methods to. So the mounting remap from {@link IMU.Parameters}, the {@link #resetYaw()} offset, the
 * yaw/pitch/roll extraction, the wrapping of every angle into {@code [-180, 180)} degrees
 * ({@code [-pi, pi)} radians), and the unit conversion are what a real hub computes. What this fake supplies
 * is the other side of the helper: the raw device-frame quaternion and gyro rates a real sensor would report.
 *
 * <p><b>Physical versus declared mounting.</b> Two orientations are in play, as on a robot:
 * <ul>
 *   <li>The <b>physical</b> mounting — how the hub is really bolted to this simulated chassis — is fixed at
 *       construction. It decides the raw readings: the device quaternion is the one for which the SDK's
 *       helper, told the physical mounting, returns exactly the chassis heading, and the device gyro rates
 *       are the chassis rate taken back through that mounting's angular-velocity transform.</li>
 *   <li>The <b>declared</b> mounting is whatever robot code passes to {@link #initialize(IMU.Parameters)}
 *       (before that call, the SDK default: logo {@code UP}, USB {@code FORWARD}). The helper reads the raw
 *       values through it.</li>
 * </ul>
 * When the two match, yaw is the chassis heading and pitch and roll are zero. When they do not, the robot
 * reads what a mis-configured robot would: for example a hub that is really mounted logo-forward, USB-left,
 * but declared logo-up reads a constant phantom roll of 90 degrees; with the USB up or down instead, its
 * yaw/pitch/roll sit exactly at gimbal lock and read NaN.
 *
 * <p><b>Seam.</b> The harness hands the fake the chassis heading each tick with
 * {@link #setChassisHeading(double)}, in the drive plant's frame (radians, CCW-positive, not wrapped —
 * {@code MecanumDrivePlant.getHeading()} as it stands). The gyro rate is finite-differenced from those
 * injected headings across {@link #update(double)} ticks, the same pattern {@link FakeExternalEncoder} uses
 * for encoder velocity, so a harness that injects after {@code updateAll} reads the rate one tick behind the
 * heading. The simulated chassis is always flat, because a planar drive plant has no pitch or roll; any tilt
 * the robot reads comes from a declared mounting that does not match the physical one.
 *
 * <p><b>Power-on yaw.</b> A real {@code BHI260IMU} zeroes its yaw once, when the hardware map builds it, and
 * {@link #initialize(IMU.Parameters)} does not zero it again. This fake does the same: it is zeroed at
 * construction, when no heading has been injected yet, so yaw reads zero at plant heading zero.
 * A harness that seeds a start heading and wants the IMU to have powered up facing that way calls
 * {@link #resetYaw()} after seeding it — which is also what most team init code does.
 *
 * <p><b>Deterministic.</b> State changes only through {@link #setChassisHeading(double)},
 * {@link #initialize(IMU.Parameters)}, {@link #resetYaw()} and {@link #update(double)}; there is no wall-clock
 * read in any value it reports. Acquisition timestamps are simulated nanoseconds, the sum of every
 * {@code deltaTime} passed to {@link #update(double)}. One SDK detail to know about: the helper's
 * {@code resetYaw} starts an {@code ElapsedTime} to bound its retries when the device fails to answer. This
 * fake always answers on the first try, so that timer is constructed and never read.
 *
 * <p><b>The test JVM needs {@code unitTests.returnDefaultValues = true}.</b> The SDK's orientation types build
 * an {@code OpenGLMatrix}, whose constructor calls {@code android.opengl.Matrix.setIdentityM}; in a plain
 * Android unit test that call throws "not mocked". That is true of this fake's getters and equally of the
 * {@code new RevHubOrientationOnRobot(logo, usb)} in every team's init code. With the flag the call is a
 * no-op and the results are still exact, because the SDK fills and reads only the matrix's 3x3 rotation
 * block (SDK 12.0 bytecode; {@code FakeImuTest} pins the numbers).
 *
 * <p><b>Not modelled:</b> sensor noise, gyro drift, the device's own 16-bit quantization, the legacy
 * {@code BNO055IMU} interface, accelerometer and magnetometer readings, and any tilt of the chassis.
 */
public class FakeImu extends AbstractFakeDevice implements IMU {

    /** The tag the SDK helper would log a failed read under. This fake never fails a read. */
    private static final String LOG_TAG = "FakeImu";

    /**
     * How long the power-on yaw reset would keep retrying a failed read, in milliseconds. SPEC: the value the
     * SDK 12.0 {@code BHI260IMU} constructor passes. Never waited on here, because this fake's reads cannot
     * fail.
     */
    private static final int POWER_ON_RESET_TIMEOUT_MS = 500;

    /**
     * How long {@link #resetYaw()} would keep retrying a failed read, in milliseconds. SPEC: the value the SDK
     * 12.0 {@code BHI260IMU.resetYaw()} passes. Never waited on here, because this fake's reads cannot fail.
     */
    private static final int RESET_YAW_TIMEOUT_MS = 50;

    /** Nanoseconds in one second, for the simulated acquisition clock. SPEC (a unit definition). */
    private static final double NANOSECONDS_PER_SECOND = 1e9;

    // The physical mounting's fixed factors, computed once: raw = M^-1 * yaw * (O^-1 * M), and device gyro
    // rates = T^-1 applied to robot rates (see rawDeviceQuaternion and rawDeviceAngularVelocity).
    private final Quaternion physicalMountInverse;
    private final Quaternion physicalOffsetInverseTimesMount;
    private final Quaternion robotToDeviceRates;
    private final QuaternionBasedImuHelper helper;
    private final ThrowingSupplier<Quaternion, QuaternionBasedImuHelper.FailedToRetrieveQuaternionException>
            rawQuaternionSource = this::rawDeviceQuaternion;

    // Injected truth (the plant's heading) + the finite-differenced rate, in the plant's frame.
    private double chassisHeadingRadians = 0.0;
    private double previousChassisHeadingRadians = 0.0;
    // False until the first tick has recorded a previous heading, so a heading seeded before the first
    // update(dt) is not reported as a turn of (seed / dt).
    private boolean hasPreviousHeading = false;
    private double chassisYawRateRadiansPerSecond = 0.0;

    private long simulatedNanoseconds = 0L;

    /**
     * Builds an IMU physically mounted the way the SDK assumes by default: hub logo {@code UP}, USB ports
     * {@code FORWARD}. Robot code that declares that mounting (or declares nothing) reads yaw as the chassis
     * heading.
     */
    public FakeImu() {
        this(defaultMounting());
    }

    /**
     * Builds an IMU physically mounted as {@code physicalMounting} on the simulated chassis — usually a
     * {@link RevHubOrientationOnRobot}, exactly as a team would write it in its own {@link IMU.Parameters}.
     * Robot code that declares the same mounting reads yaw as the chassis heading; robot code that declares
     * a different one reads what that mistake reads on a real hub.
     *
     * @param physicalMounting how the IMU is really mounted on the simulated robot; a frame relation, no
     *                         units
     * @throws IllegalArgumentException if {@code physicalMounting} is null
     */
    public FakeImu(ImuOrientationOnRobot physicalMounting) {
        if (physicalMounting == null) {
            throw new IllegalArgumentException("physicalMounting must not be null");
        }
        Quaternion mount = physicalMounting.imuCoordinateSystemOrientationFromPerspectiveOfRobot();
        this.physicalMountInverse = mount.inverse();
        this.physicalOffsetInverseTimesMount = physicalMounting.imuRotationOffset().inverse().multiply(mount, 0L);
        this.robotToDeviceRates = physicalMounting.angularVelocityTransform().inverse();
        // As BHI260IMU's constructor does: the helper starts from the default declared mounting, and the
        // yaw is zeroed once, at power-on, against it.
        this.helper = new QuaternionBasedImuHelper(defaultMounting());
        helper.resetYaw(LOG_TAG, rawQuaternionSource, POWER_ON_RESET_TIMEOUT_MS);
    }

    /** The mounting {@code BHI260IMU} assumes until {@code initialize} says otherwise. SPEC (SDK 12.0). */
    private static ImuOrientationOnRobot defaultMounting() {
        return new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);
    }

    /**
     * Injects the chassis heading, which the harness calls each tick with the drive plant's heading. Reads
     * immediately; the next {@link #update(double)} finite-differences it into the gyro rate.
     *
     * @param plantHeadingRadians the chassis heading in the drive plant's frame, radians, CCW-positive seen
     *                            from above; wrapped or not, since the rate uses the shortest turn between
     *                            two headings
     * @throws IllegalArgumentException if {@code plantHeadingRadians} is not finite
     */
    public void setChassisHeading(double plantHeadingRadians) {
        if (!Double.isFinite(plantHeadingRadians)) {
            throw new IllegalArgumentException("injected chassis heading must be finite, got " + plantHeadingRadians);
        }
        this.chassisHeadingRadians = plantHeadingRadians;
    }

    /**
     * Advances the simulated clock and derives the gyro rate from the change in injected heading since the
     * previous tick.
     *
     * @param deltaTime the tick length, in seconds; a non-finite or non-positive value is treated as no time
     *                  passing
     */
    @Override
    public void update(double deltaTime) {
        if (!Double.isFinite(deltaTime) || deltaTime <= 0.0) {
            return;
        }
        simulatedNanoseconds += Math.round(deltaTime * NANOSECONDS_PER_SECOND);
        if (hasPreviousHeading) {
            double turnRadians = AngleUnit.normalizeRadians(chassisHeadingRadians - previousChassisHeadingRadians);
            chassisYawRateRadiansPerSecond = turnRadians / deltaTime;
        }
        previousChassisHeadingRadians = chassisHeadingRadians;
        hasPreviousHeading = true;
    }

    /**
     * The raw orientation the physical device reports: the quaternion {@code q} for which the helper's own
     * conversion, {@code M * (q * M^-1) * O} with {@code M} and {@code O} taken from the physical mounting,
     * is the chassis's pure yaw rotation. So {@code q = M^-1 * yaw * O^-1 * M}.
     */
    private Quaternion rawDeviceQuaternion() {
        long t = simulatedNanoseconds;
        double halfHeading = chassisHeadingRadians / 2.0;
        Quaternion chassisYaw = new Quaternion(
                (float) Math.cos(halfHeading), 0f, 0f, (float) Math.sin(halfHeading), t);
        return physicalMountInverse
                .multiply(chassisYaw, t)
                .multiply(physicalOffsetInverseTimesMount, t)
                .normalized();
    }

    /**
     * The raw gyro rates the physical device reports, in its own axes and in degrees per second (the unit
     * {@code BHI260IMU} reads them in): the chassis rate about robot {@code +z}, taken back through the
     * physical mounting's angular-velocity transform.
     */
    private AngularVelocity rawDeviceAngularVelocity() {
        VectorF robotRatesDegreesPerSecond = new VectorF(
                0f, 0f, (float) Math.toDegrees(chassisYawRateRadiansPerSecond));
        VectorF device = robotToDeviceRates.applyToVector(robotRatesDegreesPerSecond);
        return new AngularVelocity(AngleUnit.DEGREES, device.get(0), device.get(1), device.get(2),
                simulatedNanoseconds);
    }

    /**
     * Declares how the IMU is mounted, as robot code does in init: the device is read through
     * {@code parameters.imuOrientationOnRobot} from now on, and a later change to {@code parameters} has no
     * effect. Does not zero the yaw; call {@link #resetYaw()} for that.
     *
     * @param parameters the declared mounting
     * @return true: initialization of a simulated device always succeeds
     */
    @Override
    public boolean initialize(IMU.Parameters parameters) {
        helper.setImuOrientationOnRobot(parameters.imuOrientationOnRobot);
        return true;
    }

    /** Makes the current robot heading read as yaw zero, through the SDK helper's own reset. */
    @Override
    public void resetYaw() {
        helper.resetYaw(LOG_TAG, rawQuaternionSource, RESET_YAW_TIMEOUT_MS);
    }

    /**
     * {@return the robot's yaw, pitch and roll in degrees (intrinsic ZXY, as the SDK defines them), each in
     * {@code [-180, 180)}: yaw CCW-positive seen from above, zero where {@link #resetYaw()} was last called;
     * acquisition time in simulated nanoseconds}
     */
    @Override
    public YawPitchRollAngles getRobotYawPitchRollAngles() {
        return helper.getRobotYawPitchRollAngles(LOG_TAG, rawQuaternionSource);
    }

    /**
     * Returns the robot's orientation in any axes convention, through the SDK helper.
     *
     * @param reference intrinsic or extrinsic axes
     * @param order     the order the three rotations are applied in
     * @param angleUnit degrees or radians; every angle is wrapped to {@code [-180, 180)} degrees or
     *                  {@code [-pi, pi)} radians
     * @return the orientation relative to where {@link #resetYaw()} was last called, with acquisition time in
     *         simulated nanoseconds
     */
    @Override
    public Orientation getRobotOrientation(AxesReference reference, AxesOrder order, AngleUnit angleUnit) {
        return helper.getRobotOrientation(LOG_TAG, rawQuaternionSource, reference, order, angleUnit);
    }

    /**
     * {@return the robot's orientation as a unit quaternion relative to where {@link #resetYaw()} was last
     * called, with acquisition time in simulated nanoseconds}
     */
    @Override
    public Quaternion getRobotOrientationAsQuaternion() {
        return helper.getRobotOrientationAsQuaternion(LOG_TAG, rawQuaternionSource, true);
    }

    /**
     * Returns the robot's angular velocity in robot axes, through the SDK helper.
     *
     * @param angleUnit degrees or radians; the result is per second and not wrapped, since it is a rate
     * @return the rates about robot {@code x}, {@code y} and {@code z}; {@code zRotationRate} is CCW-positive
     *         seen from above. Acquisition time is in simulated nanoseconds
     */
    @Override
    public AngularVelocity getRobotAngularVelocity(AngleUnit angleUnit) {
        return helper.getRobotAngularVelocity(rawDeviceAngularVelocity(), angleUnit);
    }

    /** {@return {@code Lynx}, what {@code BHI260IMU} reports: the IMU is built into a REV hub} */
    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.Lynx;
    }
}
