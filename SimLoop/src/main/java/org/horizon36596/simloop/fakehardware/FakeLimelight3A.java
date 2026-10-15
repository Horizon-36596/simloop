package org.horizon36596.simloop.fakehardware;

import com.qualcomm.hardware.limelightvision.LLFieldMap;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.horizon36596.simloop.field.FieldFrameTransform;
import org.horizon36596.simloop.field.ObstacleBox;
import org.horizon36596.simloop.field.SeededRandom;
import org.horizon36596.simloop.vision.AprilTagLayout;
import org.horizon36596.simloop.vision.CameraLens;
import org.horizon36596.simloop.vision.CameraMount;
import org.horizon36596.simloop.vision.RigidTransform3;
import org.horizon36596.simloop.vision.SimTagCamera;
import org.horizon36596.simloop.vision.TagSighting;
import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import java.net.InetAddress;
import java.net.UnknownHostException;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Deque;
import java.util.List;

/**
 * A Limelight 3A for the sim: the SDK 12.0 {@link Limelight3A}, looked up with
 * {@code hardwareMap.get(Limelight3A.class, "limelight")}, that sees AprilTags from a team-supplied
 * {@link AprilTagLayout} instead of over USB. Robot code calls {@code start()}, {@code pipelineSwitch},
 * {@code updateRobotOrientation} and {@code getLatestResult()} exactly as it does on the robot.
 *
 * <p><b>The results are the SDK's own parsing of the device's JSON.</b> The real driver polls the
 * Limelight's {@code /results} endpoint and hands the JSON to {@code LLResult}'s constructor, which parses
 * {@code botpose}, {@code botpose_orb}, the {@code Fiducial} array and the rest. This fake builds that same
 * JSON from the sim's geometry and hands it to the same constructor, so every {@link LLResult} getter,
 * {@link LLResultTypes.FiducialResult} included, is the SDK's code reading field names the SDK 12.0.0
 * bytecode reads. The one change is time: {@link LLResult#getStaleness()},
 * {@link LLResult#getControlHubTimeStamp()} and {@link LLResult#getControlHubTimeStampNanos()} read the sim
 * clock instead of {@code System.currentTimeMillis()}, and {@link LLResult#getParseLatency()} reads 0.
 *
 * <p><b>Wiring</b>, in the sim harness (never in robot code):
 * <pre>
 *   AprilTagLayout layout = AprilTagLayout.builder()
 *           .addTag(1, 70.0, 30.0, 6.0, Math.PI, 4.0)       // your field, your numbers
 *           .build();
 *   FakeLimelight3A limelight = FakeLimelight3A.builder(layout,
 *           new CameraMount(7.0, 0.0, 9.0, 0.0, Math.toRadians(-15.0)), 54.5, 42.0).build();
 *   hardwareMap.register("limelight", limelight);
 *   // every tick, before hardwareMap.updateAll(dt):
 *   limelight.setRobotPose(fieldX, fieldY, fieldHeading);   // or setRobotPoseFromLocalizer(...)
 * </pre>
 *
 * <p><b>What happens on each {@link #update(double)}</b>, in this order:
 * <ol>
 *   <li>The sim clock advances by {@code deltaTime}. It is the only clock this class reads.</li>
 *   <li>The camera captures a frame when one is due (every tick, or at {@code frameRateHz}), from the pose
 *       last given to {@link #setRobotPose}, the yaw last given to {@link #updateRobotOrientation} and the
 *       pipeline last given to {@link #pipelineSwitch}. The camera captures whether or not the driver is
 *       started, as a powered Limelight does.</li>
 *   <li>A frame becomes the device's current result once its capture plus targeting latency has passed.</li>
 *   <li>If the driver is {@linkplain #start() started} and a poll is due (at the SDK's poll rate, 100 Hz
 *       unless {@link #setPollRateHz} changed it, at most once a tick), the current result is parsed into a
 *       new {@link LLResult} stamped with the sim time. As on the robot, a re-poll of an unchanged frame is
 *       a new result with a fresh stamp, so staleness measures time since the last poll, not frame age.</li>
 * </ol>
 *
 * <p><b>Start, stop and the first result</b> follow the SDK 12.0.0 driver's bytecode: before any poll,
 * {@link #getLatestResult()} returns an {@code LLResult} of an empty JSON object, never null, whose
 * {@code isValid()} is false. {@link #stop()} and {@link #pause()} stop polling and keep the last result,
 * which then grows stale; {@link #isConnected()} (the SDK's own method) turns false 250 ms of sim time
 * after the last poll.
 *
 * <p><b>Conventions in the JSON</b> (units as the Limelight uses them: metres and degrees; poses are
 * {@code [x, y, z, roll, pitch, yaw]}, angles decomposed as {@link RigidTransform3} documents):
 * <ul>
 *   <li>{@code botpose} (MegaTag1) and {@code botpose_orb} (MegaTag2) are in the layout's field frame.</li>
 *   <li>Camera space ({@code t6t_cs}) is the optical frame: {@code +x} right, {@code +y} down,
 *       {@code +z} out of the lens. Target space ({@code t6c_ts}, {@code t6r_ts}) is
 *       {@link AprilTagLayout.Tag#getFieldToTag()}'s frame.</li>
 *   <li>Robot space ({@code t6t_rs}, {@code t6c_rs}) is <b>{@code +x} forward, {@code +y} left, {@code +z}
 *       up</b>, the right-handed WPILib robot frame. Limelight's documentation describes robot space with
 *       {@code +y} to the right; which of the two a real 3A reports has not been checked against a device.
 *       {@code t6c_rs} gives the camera body (boresight along {@code +x}) with WPILib's pitch sign, positive
 *       nose-down.</li>
 *   <li>{@code tx} is positive right of the image centre, {@code ty} positive above it. There is no
 *       crosshair offset, so {@code txnc == tx} and {@code tync == ty}. The top-level {@code tx/ty/ta} are
 *       the largest-area tag's; the {@code Fiducial} array is in ascending tag id order.</li>
 * </ul>
 *
 * <p><b>MegaTag2.</b> {@code botpose_orb}'s yaw is exactly the last value given to
 * {@link #updateRobotOrientation(double)} (0 before the first call), and its position is where the robot
 * must be for each seen tag to sit where it is relative to the robot <em>if</em> the robot faced that yaw,
 * averaged over the seen tags. So a correct yaw gives the true position and a wrong yaw swings the answer
 * around the tags, as it does on a real robot.
 *
 * <p><b>Noise</b> (off unless {@link Builder#noise} is called): independent Gaussian draws, from a
 * {@link SeededRandom} so the same seed and the same sequence of calls give byte-identical results, added
 * to each tag's {@code tx} and {@code ty} (and the centre pixel with them), to {@code botpose}'s x, y and
 * yaw, and to {@code botpose_orb}'s x and y. Target poses ({@code t6*}), corners and area stay exact.
 *
 * <p><b>Not modelled:</b> any pipeline other than AprilTag (every pipeline index 0-9 behaves as an AprilTag
 * pipeline), crosshair calibration, lens distortion, motion blur, detection dropouts, tag-on-tag and
 * robot-body occlusion, tags that are not vertical, the {@code stddevs}, {@code botpose_wpiblue/wpired}
 * and python outputs, and the device-side capture schedule's phase. Methods that would configure the
 * device over the network ({@code uploadFieldmap}, {@code uploadPipeline}, {@code uploadPython},
 * {@code updatePythonInputs}, snapshots and calibration) throw {@link UnsupportedOperationException}: the
 * fake has nothing behind them, and a silent {@code true} would let a robot believe a field map was
 * uploaded when it was not. Give the fake its field through its {@link AprilTagLayout} instead.
 */
public class FakeLimelight3A extends Limelight3A implements FakeDevice {

    /** Pipelines a Limelight holds, indices 0 to this minus 1. SPEC: Limelight pipeline slots 0-9. */
    public static final int PIPELINE_COUNT = 10;

    /** The {@code pipelineType} every fake pipeline reports: an AprilTag (fiducial) pipeline. */
    public static final String PIPELINE_TYPE = "pipe_fiducial";

    /** The SDK driver's poll interval before {@link #setPollRateHz} is called, ms (SDK 12.0.0 bytecode). */
    private static final long DEFAULT_POLL_INTERVAL_MS = 10L;

    private static final double METRES_PER_INCH = 0.0254;
    private static final long NANOS_PER_MS = 1_000_000L;

    private final SimTagCamera camera;
    private final AprilTagLayout layout;
    private final String family;
    private final long framePeriodNanos;
    private final double captureLatencyMs;
    private final double targetingLatencyMs;
    private final long latencyNanos;
    private final SeededRandom noiseRandom;
    private final double txTyStdDevDegrees;
    private final double positionStdDevInches;
    private final double yawStdDevDegrees;
    private final double frameRateHz;

    private long simNanos;
    private boolean running;
    private long pollIntervalMs = DEFAULT_POLL_INTERVAL_MS;
    private long nextPollNanos;
    private long nextCaptureNanos;
    private boolean capturedOnce;
    private boolean polledOnce;
    private long lastPollMs;

    private double robotXInches;
    private double robotYInches;
    private double robotHeadingRad;
    private double suppliedYawDegrees;
    private int pipelineIndex;

    /** Captured frames waiting out their latency, oldest first. */
    private final Deque<Frame> inFlight = new ArrayDeque<Frame>();
    /** The newest frame whose latency has passed: what a poll of {@code /results} returns now. */
    private String currentFrameJson = "{}";
    private LLResult latestResult;

    private FakeLimelight3A(Builder b) {
        // The SDK constructor only stores these and builds a single-thread executor that starts no thread
        // until a task is scheduled. The serial number is read only by getConnectionInfo(), overridden here.
        super(null, b.name, loopback());
        // Shut that executor down now, through the SDK's own stop(): no poll thread can ever exist.
        super.stop();
        double fx = (b.widthPx / 2.0) / StrictMath.tan(StrictMath.toRadians(b.horizontalFovDegrees) / 2.0);
        double fy = (b.heightPx / 2.0) / StrictMath.tan(StrictMath.toRadians(b.verticalFovDegrees) / 2.0);
        CameraLens lens = new CameraLens(fx, fy, b.widthPx / 2.0, b.heightPx / 2.0, b.widthPx, b.heightPx);
        this.camera = new SimTagCamera(lens, b.mount, b.minRangeInches, b.maxRangeInches,
                b.maxIncidenceDegrees, b.occluders);
        this.layout = b.layout;
        this.family = b.family;
        this.frameRateHz = b.frameRateHz;
        this.framePeriodNanos = b.frameRateHz > 0.0 ? Math.round(1e9 / b.frameRateHz) : 0L;
        this.captureLatencyMs = b.captureLatencyMs;
        this.targetingLatencyMs = b.targetingLatencyMs;
        this.latencyNanos = Math.round((b.captureLatencyMs + b.targetingLatencyMs) * NANOS_PER_MS);
        this.noiseRandom = b.noiseEnabled ? new SeededRandom(b.noiseSeed) : null;
        this.txTyStdDevDegrees = b.txTyStdDevDegrees;
        this.positionStdDevInches = b.positionStdDevInches;
        this.yawStdDevDegrees = b.yawStdDevDegrees;
    }

    /** 127.0.0.1, built without a DNS lookup: if an SDK call ever slipped past this fake, it hits nothing. */
    private static InetAddress loopback() {
        try {
            return InetAddress.getByAddress("limelight-sim", new byte[] {127, 0, 0, 1});
        } catch (UnknownHostException impossible) {
            throw new IllegalStateException("a 4-byte address is always valid", impossible);
        }
    }

    /**
     * A builder for a fake Limelight. The four arguments have no defaults because every one of them changes
     * what the camera sees; take them from your own robot and your own field.
     *
     * @param layout               the tags on the field, team-supplied; {@link AprilTagLayout#empty()} for none
     * @param mount                where the camera is on the robot, robot frame; see {@link CameraMount}
     * @param horizontalFovDegrees full horizontal field of view, degrees, in {@code (0, 180)}
     * @param verticalFovDegrees   full vertical field of view, degrees, in {@code (0, 180)}
     * @return the builder
     */
    public static Builder builder(AprilTagLayout layout, CameraMount mount, double horizontalFovDegrees,
            double verticalFovDegrees) {
        return new Builder(layout, mount, horizontalFovDegrees, verticalFovDegrees);
    }

    // ------------------------------------------------------------------------------------ harness side

    /**
     * Where the robot is, for the frames captured from now on. Call every tick before
     * {@code FakeHardwareMap.updateAll}.
     *
     * @param fieldXInches    robot centre, field X, inches (the layout's frame)
     * @param fieldYInches    robot centre, field Y, inches
     * @param fieldHeadingRad robot heading, radians CCW from field {@code +X}
     * @throws IllegalArgumentException on a non-finite number
     */
    public void setRobotPose(double fieldXInches, double fieldYInches, double fieldHeadingRad) {
        if (!Double.isFinite(fieldXInches) || !Double.isFinite(fieldYInches) || !Double.isFinite(fieldHeadingRad)) {
            throw new IllegalArgumentException("robot pose must be finite, got (" + fieldXInches + ", "
                    + fieldYInches + ", " + fieldHeadingRad + ")");
        }
        this.robotXInches = fieldXInches;
        this.robotYInches = fieldYInches;
        this.robotHeadingRad = fieldHeadingRad;
    }

    /**
     * {@link #setRobotPose} from a pose in the localizer frame a {@code MecanumDrivePlant} reports, rotated
     * into the field frame by {@link FieldFrameTransform}.
     *
     * @param localizerXInches    localizer X, inches
     * @param localizerYInches    localizer Y, inches
     * @param localizerHeadingRad localizer heading, radians CCW
     */
    public void setRobotPoseFromLocalizer(double localizerXInches, double localizerYInches,
            double localizerHeadingRad) {
        setRobotPose(FieldFrameTransform.fieldXFromLocalizer(localizerXInches, localizerYInches),
                FieldFrameTransform.fieldYFromLocalizer(localizerXInches, localizerYInches),
                FieldFrameTransform.fieldHeadingFromLocalizer(localizerHeadingRad));
    }

    /**
     * Advances the sim clock and runs the capture, latency and poll steps the class comment lists.
     *
     * @param deltaTime the tick length, seconds; a non-finite or non-positive value advances nothing
     */
    @Override
    public void update(double deltaTime) {
        if (!(deltaTime > 0.0) || Double.isInfinite(deltaTime)) {
            return;
        }
        simNanos += Math.round(deltaTime * 1e9);

        if (framePeriodNanos == 0L || simNanos >= nextCaptureNanos) {
            inFlight.addLast(new Frame(simNanos + latencyNanos, buildFrameJson(simNanos)));
            if (framePeriodNanos > 0L) {
                // The schedule starts at the first capture; a tick longer than a frame skips frames.
                nextCaptureNanos = capturedOnce ? nextCaptureNanos + framePeriodNanos : simNanos + framePeriodNanos;
                if (nextCaptureNanos <= simNanos) {
                    nextCaptureNanos = simNanos + framePeriodNanos;
                }
            }
            capturedOnce = true;
        }
        while (!inFlight.isEmpty() && inFlight.peekFirst().availableAtNanos <= simNanos) {
            currentFrameJson = inFlight.pollFirst().json;
        }
        if (running && simNanos >= nextPollNanos) {
            latestResult = SimLLResult.parse(currentFrameJson, nowMs(), this);
            polledOnce = true;
            lastPollMs = nowMs();
            long intervalNanos = pollIntervalMs * NANOS_PER_MS;
            nextPollNanos += intervalNanos;
            if (nextPollNanos <= simNanos) {
                nextPollNanos = simNanos + intervalNanos;
            }
        }
    }

    /** {@return sim time since this device was built, nanoseconds: the sum of every update tick} */
    public long getSimTimeNanos() {
        return simNanos;
    }

    /** {@return the pipeline index the next captured frame will report} */
    public int getPipelineIndex() {
        return pipelineIndex;
    }

    /** {@return the yaw last sent with {@link #updateRobotOrientation(double)}, degrees; 0 before any} */
    public double getSuppliedYawDegrees() {
        return suppliedYawDegrees;
    }

    /** {@return the geometric camera behind this fake, for tests that want the noise-free truth} */
    public SimTagCamera getTagCamera() {
        return camera;
    }

    /** {@return the tag layout this fake sees} */
    public AprilTagLayout getLayout() {
        return layout;
    }

    private long nowMs() {
        return simNanos / NANOS_PER_MS;
    }

    // ------------------------------------------------------------------------ the SDK surface, on sim time

    /** Starts polling on the next {@link #update(double)}; does nothing if already running (SDK semantics). */
    @Override
    public void start() {
        if (!running) {
            running = true;
            nextPollNanos = simNanos;
        }
    }

    /** Stops polling and keeps the last result, as the SDK's {@code pause()} does. */
    @Override
    public void pause() {
        running = false;
    }

    /** Stops polling and keeps the last result, as the SDK's {@code stop()} does. */
    @Override
    public void stop() {
        running = false;
    }

    /** {@return true between {@link #start()} and {@link #stop()} or {@link #pause()}} */
    @Override
    public boolean isRunning() {
        return running;
    }

    /**
     * The SDK's rule exactly: ignored while running; otherwise the rate is clamped to 1-250 Hz and the
     * interval is {@code 1000 / hz} milliseconds, integer division.
     *
     * @param pollRateHz polls per second
     */
    @Override
    public void setPollRateHz(int pollRateHz) {
        if (running) {
            return;
        }
        pollIntervalMs = 1000 / Math.max(1, Math.min(pollRateHz, 250));
    }

    /**
     * Sim milliseconds since the last poll. Before the first poll this is {@link Long#MAX_VALUE}: the SDK
     * measures from a last-update time of 0 on the wall clock, which is decades ago, and this keeps
     * {@link #isConnected()} false until the fake has actually been polled, as on the robot.
     *
     * @return milliseconds, sim time
     */
    @Override
    public long getTimeSinceLastUpdate() {
        return polledOnce ? nowMs() - lastPollMs : Long.MAX_VALUE;
    }

    /**
     * The last polled result. Before the first poll, an empty result ({@code isValid()} false) is made on
     * the first call and kept, exactly as the SDK driver does; it is never null.
     *
     * @return the result
     */
    @Override
    public LLResult getLatestResult() {
        if (latestResult == null) {
            latestResult = SimLLResult.parse("{}", nowMs(), this);
        }
        return latestResult;
    }

    /**
     * The device status the SDK parses from {@code /status}: name, pipeline index and type, frame rate (0
     * when frames are captured every tick) and {@code finalYaw}, the MegaTag2 yaw. Temperature, CPU and the
     * rest read 0.
     *
     * @return the status
     */
    @Override
    public LLStatus getStatus() {
        try {
            JSONObject json = new JSONObject();
            json.put("name", getDeviceName());
            json.put("pipelineIndex", pipelineIndex);
            json.put("pipelineType", PIPELINE_TYPE);
            json.put("fps", frameRateHz);
            json.put("finalYaw", suppliedYawDegrees);
            return new LLStatus(json) { };
        } catch (JSONException e) {
            throw jsonBug(e);
        }
    }

    /**
     * Switches pipeline for the frames captured from the next tick on.
     *
     * @param index the pipeline, 0 to {@link #PIPELINE_COUNT} - 1
     * @return true when switched; false, and no change, for an index outside that range (assumed to be
     *         what the device's REST call answers; not checked against a device)
     */
    @Override
    public boolean pipelineSwitch(int index) {
        if (index < 0 || index >= PIPELINE_COUNT) {
            return false;
        }
        pipelineIndex = index;
        return true;
    }

    /** {@return true: there is no pipeline file to reload, and the fake's pipelines never change} */
    @Override
    public boolean reloadPipeline() {
        return true;
    }

    /**
     * The robot's field yaw for MegaTag2, used by frames captured from the next tick on.
     *
     * @param yawDegrees the yaw, degrees CCW from field {@code +X}, as the robot would send it
     * @return true; false, and no change, for a non-finite yaw (the SDK's JSON encode would reject it)
     */
    @Override
    public boolean updateRobotOrientation(double yawDegrees) {
        if (!Double.isFinite(yawDegrees)) {
            return false;
        }
        suppliedYawDegrees = yawDegrees;
        return true;
    }

    /** {@return {@code "sim"}; the SDK prints a USB serial number the fake does not have} */
    @Override
    public String getConnectionInfo() {
        return "sim";
    }

    /**
     * Not modelled; throws.
     *
     * @param name unused
     * @return never returns
     */
    @Override
    public boolean captureSnapshot(String name) {
        throw notModelled("captureSnapshot");
    }

    /**
     * Not modelled; throws.
     *
     * @return never returns
     */
    @Override
    public boolean deleteSnapshots() {
        throw notModelled("deleteSnapshots");
    }

    /**
     * Not modelled; throws.
     *
     * @param name unused
     * @return never returns
     */
    @Override
    public boolean deleteSnapshot(String name) {
        throw notModelled("deleteSnapshot");
    }

    /**
     * Not modelled; throws.
     *
     * @param a unused
     * @param b unused
     * @param c unused
     * @param d unused
     * @param e unused
     * @param f unused
     * @param g unused
     * @param h unused
     * @return never returns
     */
    @Override
    public boolean updatePythonInputs(double a, double b, double c, double d, double e, double f, double g,
            double h) {
        throw notModelled("updatePythonInputs");
    }

    /**
     * Not modelled; throws.
     *
     * @param inputs unused
     * @return never returns
     */
    @Override
    public boolean updatePythonInputs(double[] inputs) {
        throw notModelled("updatePythonInputs");
    }

    /**
     * Not modelled; throws.
     *
     * @param pipeline unused
     * @param index unused
     * @return never returns
     */
    @Override
    public boolean uploadPipeline(String pipeline, Integer index) {
        throw notModelled("uploadPipeline");
    }

    /**
     * Not modelled; throws. Give the fake its field as an {@link AprilTagLayout} instead.
     *
     * @param fieldMap unused
     * @param index    unused
     * @return never returns
     */
    @Override
    public boolean uploadFieldmap(LLFieldMap fieldMap, Integer index) {
        throw notModelled("uploadFieldmap (give the fake an AprilTagLayout instead)");
    }

    /**
     * Not modelled; throws.
     *
     * @param code unused
     * @param index unused
     * @return never returns
     */
    @Override
    public boolean uploadPython(String code, Integer index) {
        throw notModelled("uploadPython");
    }

    /**
     * Not modelled; throws.
     *
     * @return never returns
     */
    @Override
    public LLResultTypes.CalibrationResult getCalDefault() {
        throw notModelled("getCalDefault");
    }

    /**
     * Not modelled; throws.
     *
     * @return never returns
     */
    @Override
    public LLResultTypes.CalibrationResult getCalFile() {
        throw notModelled("getCalFile");
    }

    /**
     * Not modelled; throws.
     *
     * @return never returns
     */
    @Override
    public LLResultTypes.CalibrationResult getCalEEPROM() {
        throw notModelled("getCalEEPROM");
    }

    /**
     * Not modelled; throws.
     *
     * @return never returns
     */
    @Override
    public LLResultTypes.CalibrationResult getCalLatest() {
        throw notModelled("getCalLatest");
    }

    private static UnsupportedOperationException notModelled(String what) {
        return new UnsupportedOperationException("FakeLimelight3A does not model " + what
                + ": there is no device behind it. See the FakeLimelight3A class comment for what it models.");
    }

    // ------------------------------------------------------------------------------- building a frame

    /**
     * The {@code /results} JSON the device would send for a frame captured now. Android's
     * {@code JSONException} is a checked exception, so the one catch is here rather than on every put; it
     * can only fire on a non-finite number, which the checks on every input rule out.
     */
    private String buildFrameJson(long captureNanos) {
        try {
            return frameJson(captureNanos);
        } catch (JSONException e) {
            throw jsonBug(e);
        }
    }

    private static IllegalStateException jsonBug(JSONException e) {
        return new IllegalStateException("FakeLimelight3A built JSON the parser rejected; this is a bug", e);
    }

    private String frameJson(long captureNanos) throws JSONException {
        List<TagSighting> seen = camera.observe(layout, robotXInches, robotYInches, robotHeadingRad);
        JSONObject json = new JSONObject();
        json.put("pID", pipelineIndex);
        json.put("pipelineType", PIPELINE_TYPE);
        json.put("ts", captureNanos / (double) NANOS_PER_MS);
        json.put("cl", captureLatencyMs);
        json.put("tl", targetingLatencyMs);
        RigidTransform3 cameraInRobot = RigidTransform3.fromRollPitchYaw(camera.getMount().getForwardInches(),
                camera.getMount().getLeftInches(), camera.getMount().getUpInches(), 0.0,
                camera.getMount().getPitchRad(), camera.getMount().getYawRad());
        json.put("t6c_rs", poseArray(cameraInRobot));

        RigidTransform3 fieldToRobot = RigidTransform3.planar(robotXInches, robotYInches, robotHeadingRad);
        JSONArray fiducials = new JSONArray();
        TagSighting primary = null;
        double primaryTx = 0.0;
        double primaryTy = 0.0;
        double sumDistanceInches = 0.0;
        double sumArea = 0.0;
        double mt2X = 0.0;
        double mt2Y = 0.0;
        double mt2Z = 0.0;
        double yawRad = StrictMath.toRadians(suppliedYawDegrees);
        double cosYaw = StrictMath.cos(yawRad);
        double sinYaw = StrictMath.sin(yawRad);
        CameraLens lens = camera.getLens();
        for (TagSighting s : seen) {
            double tx = s.getTxDegrees() + gaussian(txTyStdDevDegrees);
            double ty = s.getTyDegrees() + gaussian(txTyStdDevDegrees);
            JSONObject f = new JSONObject();
            f.put("fID", s.getId());
            f.put("fam", family);
            f.put("tx", tx);
            f.put("ty", ty);
            f.put("tx_nocross", tx);
            f.put("ty_nocross", ty);
            f.put("txp", lens.getCentreXPx() + lens.getFocalXPx() * StrictMath.tan(StrictMath.toRadians(tx)));
            f.put("typ", lens.getCentreYPx() - lens.getFocalYPx() * StrictMath.tan(StrictMath.toRadians(ty)));
            f.put("ta", s.getAreaPercent());
            JSONArray corners = new JSONArray();
            for (double[] corner : s.getCornersPx()) {
                corners.put(new JSONArray().put(corner[0]).put(corner[1]));
            }
            f.put("pts", corners);
            f.put("t6t_cs", poseArray(s.getCameraToTag()));
            f.put("t6c_ts", poseArray(s.getCameraToTag().inverse()));
            f.put("t6t_rs", poseArray(s.getRobotToTag()));
            f.put("t6r_ts", poseArray(s.getRobotToTag().inverse()));
            f.put("t6r_fs", poseArray(fieldToRobot));
            fiducials.put(f);

            if (primary == null || s.getAreaPercent() > primary.getAreaPercent()) {
                primary = s;
                primaryTx = tx;
                primaryTy = ty;
            }
            sumDistanceInches += s.getRangeInches();
            sumArea += s.getAreaPercent();
            // MegaTag2: the robot sits wherever puts this tag at its robot-frame offset, turned by the yaw
            // the robot supplied rather than by the heading vision would have solved for.
            RigidTransform3 r = s.getRobotToTag();
            AprilTagLayout.Tag tag = s.getTag();
            mt2X += tag.getXInches() - (cosYaw * r.getXInches() - sinYaw * r.getYInches());
            mt2Y += tag.getYInches() - (sinYaw * r.getXInches() + cosYaw * r.getYInches());
            mt2Z += tag.getZInches() - r.getZInches();
        }
        json.put("Fiducial", fiducials);
        int count = seen.size();
        json.put("v", count > 0 ? 1 : 0);
        json.put("botpose_tagcount", count);
        if (count == 0) {
            double[] zeros = new double[6];
            json.put("botpose", toJson(zeros));
            json.put("botpose_orb", toJson(zeros));
            return json.toString();
        }
        json.put("tx", primaryTx);
        json.put("ty", primaryTy);
        json.put("txnc", primaryTx);
        json.put("tync", primaryTy);
        json.put("ta", primary.getAreaPercent());
        json.put("botpose_avgdist", sumDistanceInches / count * METRES_PER_INCH);
        json.put("botpose_avgarea", sumArea / count);
        json.put("botpose_span", spanInches(seen) * METRES_PER_INCH);

        double mt1X = robotXInches + gaussian(positionStdDevInches);
        double mt1Y = robotYInches + gaussian(positionStdDevInches);
        double mt1Yaw = wrapDegrees(StrictMath.toDegrees(robotHeadingRad) + gaussian(yawStdDevDegrees));
        json.put("botpose", toJson(new double[] {
                mt1X * METRES_PER_INCH, mt1Y * METRES_PER_INCH, 0.0, 0.0, 0.0, mt1Yaw}));
        double orbX = mt2X / count + gaussian(positionStdDevInches);
        double orbY = mt2Y / count + gaussian(positionStdDevInches);
        json.put("botpose_orb", toJson(new double[] {
                orbX * METRES_PER_INCH, orbY * METRES_PER_INCH, mt2Z / count * METRES_PER_INCH,
                0.0, 0.0, suppliedYawDegrees}));
        return json.toString();
    }

    /** The largest distance between two seen tags' centres, inches; 0 for one tag. */
    private static double spanInches(List<TagSighting> seen) {
        double span = 0.0;
        for (TagSighting a : seen) {
            for (TagSighting b : seen) {
                double dx = a.getTag().getXInches() - b.getTag().getXInches();
                double dy = a.getTag().getYInches() - b.getTag().getYInches();
                double dz = a.getTag().getZInches() - b.getTag().getZInches();
                span = Math.max(span, StrictMath.sqrt(dx * dx + dy * dy + dz * dz));
            }
        }
        return span;
    }

    /** A pose as Limelight sends it: metres, then roll, pitch, yaw in degrees. */
    private static JSONArray poseArray(RigidTransform3 t) throws JSONException {
        return toJson(new double[] {
                t.getXInches() * METRES_PER_INCH,
                t.getYInches() * METRES_PER_INCH,
                t.getZInches() * METRES_PER_INCH,
                StrictMath.toDegrees(t.getRollRad()),
                StrictMath.toDegrees(t.getPitchRad()),
                StrictMath.toDegrees(t.getYawRad()),
        });
    }

    private static JSONArray toJson(double[] values) throws JSONException {
        JSONArray array = new JSONArray();
        for (double v : values) {
            array.put(v);
        }
        return array;
    }

    /** Degrees into [-180, 180). */
    private static double wrapDegrees(double degrees) {
        return degrees - 360.0 * StrictMath.floor((degrees + 180.0) / 360.0);
    }

    /** One zero-mean Gaussian draw of this standard deviation, or exactly 0 with noise off (no draw). */
    private double gaussian(double stdDev) {
        if (noiseRandom == null || stdDev == 0.0) {
            return 0.0;
        }
        // Box-Muller, cosine branch only: one normal per two uniform draws, u1 in (0, 1] so log is finite.
        double u1 = 1.0 - noiseRandom.nextFraction();
        double u2 = noiseRandom.nextFraction();
        return stdDev * StrictMath.sqrt(-2.0 * StrictMath.log(u1)) * StrictMath.cos(2.0 * Math.PI * u2);
    }

    /** A captured frame and the sim time its latency has passed. */
    private static final class Frame {
        final long availableAtNanos;
        final String json;

        Frame(long availableAtNanos, String json) {
            this.availableAtNanos = availableAtNanos;
            this.json = json;
        }
    }

    /**
     * The SDK's {@link LLResult}, parsing the SDK's way, with its three wall-clock readings moved onto the
     * sim clock and its parse latency (a wall-clock {@code nanoTime} difference) reported as 0.
     */
    private static final class SimLLResult extends LLResult {

        private final long stampMs;
        private final FakeLimelight3A device;

        private SimLLResult(JSONObject json, long stampMs, FakeLimelight3A device) throws JSONException {
            super(json);
            this.stampMs = stampMs;
            this.device = device;
        }

        /** A fresh parse of the frame's JSON text, as each SDK poll parses a fresh response body. */
        static SimLLResult parse(String json, long stampMs, FakeLimelight3A device) {
            try {
                return new SimLLResult(new JSONObject(json), stampMs, device);
            } catch (JSONException e) {
                throw jsonBug(e);
            }
        }

        @Override
        public long getControlHubTimeStamp() {
            return stampMs;
        }

        @Override
        public long getControlHubTimeStampNanos() {
            return stampMs * NANOS_PER_MS;          // the SDK's own conversion: millisecond resolution
        }

        @Override
        public long getStaleness() {
            return device.nowMs() - stampMs;
        }

        @Override
        public double getParseLatency() {
            return 0.0;
        }
    }

    /**
     * Configures a {@link FakeLimelight3A}. Every setting has a stated default except the four
     * {@link FakeLimelight3A#builder} arguments.
     */
    public static final class Builder {

        private final AprilTagLayout layout;
        private final CameraMount mount;
        private final double horizontalFovDegrees;
        private final double verticalFovDegrees;
        private String name = "limelight";
        private int widthPx = 640;
        private int heightPx = 480;
        private double minRangeInches = 0.0;
        private double maxRangeInches = Double.POSITIVE_INFINITY;
        private double maxIncidenceDegrees = 90.0;
        private List<ObstacleBox> occluders = Collections.emptyList();
        private double frameRateHz = 0.0;
        private double captureLatencyMs = 0.0;
        private double targetingLatencyMs = 0.0;
        private String family = "36h11";
        private boolean noiseEnabled;
        private int noiseSeed;
        private double txTyStdDevDegrees;
        private double positionStdDevInches;
        private double yawStdDevDegrees;

        private Builder(AprilTagLayout layout, CameraMount mount, double horizontalFovDegrees,
                double verticalFovDegrees) {
            if (layout == null || mount == null) {
                throw new IllegalArgumentException("a fake Limelight needs a tag layout and a camera mount");
            }
            requireFov(horizontalFovDegrees, "horizontal");
            requireFov(verticalFovDegrees, "vertical");
            this.layout = layout;
            this.mount = mount;
            this.horizontalFovDegrees = horizontalFovDegrees;
            this.verticalFovDegrees = verticalFovDegrees;
        }

        private static void requireFov(double degrees, String which) {
            if (!(degrees > 0.0 && degrees < 180.0)) {
                throw new IllegalArgumentException(which + " FOV must be in (0, 180) degrees, got " + degrees);
            }
        }

        /**
         * The name the device reports. Default {@code "limelight"}.
         *
         * @param deviceName what {@code getDeviceName()} reports
         * @return this builder
         */
        public Builder name(String deviceName) {
            if (deviceName == null) {
                throw new IllegalArgumentException("device name must not be null");
            }
            this.name = deviceName;
            return this;
        }

        /**
         * The detector image size. Default 640 x 480, a PLACEHOLDER (a common Limelight 3A AprilTag
         * resolution, not checked against a datasheet here). It changes only the pixel outputs
         * ({@code txp}, {@code typ}, {@code pts}, {@code ta}), never the angles or poses.
         *
         * @param width  image width, pixels (&gt; 0)
         * @param height image height, pixels (&gt; 0)
         * @return this builder
         */
        public Builder imageSizePx(int width, int height) {
            if (width <= 0 || height <= 0) {
                throw new IllegalArgumentException("image size must be positive, got " + width + "x" + height);
            }
            this.widthPx = width;
            this.heightPx = height;
            return this;
        }

        /**
         * Detection range, lens to tag centre. Default 0 to infinity: no cutoff.
         *
         * @param minInches nearest a tag is seen, inches
         * @param maxInches farthest a tag is seen, inches
         * @return this builder
         */
        public Builder rangeInches(double minInches, double maxInches) {
            this.minRangeInches = minInches;
            this.maxRangeInches = maxInches;
            return this;
        }

        /**
         * Tags seen at this angle or more off their face normal are not detected. Default 90: only tags
         * seen from behind or edge-on are rejected.
         *
         * @param degrees the limit, in {@code (0, 90]}
         * @return this builder
         */
        public Builder maxIncidenceDegrees(double degrees) {
            this.maxIncidenceDegrees = degrees;
            return this;
        }

        /**
         * Boxes that block the line of sight to a tag. Default none.
         *
         * @param boxes field-frame boxes; copied
         * @return this builder
         */
        public Builder occluders(List<ObstacleBox> boxes) {
            if (boxes == null) {
                throw new IllegalArgumentException("occluder list must not be null");
            }
            this.occluders = new ArrayList<ObstacleBox>(boxes);
            return this;
        }

        /**
         * How often the camera captures, in sim time. Default 0, meaning once per {@code update} tick.
         *
         * @param hz frames per second, &ge; 0
         * @return this builder
         */
        public Builder frameRateHz(double hz) {
            if (!(hz >= 0.0) || Double.isInfinite(hz)) {
                throw new IllegalArgumentException("frame rate must be finite and >= 0, got " + hz);
            }
            this.frameRateHz = hz;
            return this;
        }

        /**
         * Latency between a frame's capture and the device offering its result, reported as {@code cl}
         * and {@code tl}. Default 0 and 0: a frame is offered on the tick it is captured.
         *
         * @param captureMs   capture latency, ms, &ge; 0
         * @param targetingMs targeting (processing) latency, ms, &ge; 0
         * @return this builder
         */
        public Builder latencyMs(double captureMs, double targetingMs) {
            if (!(captureMs >= 0.0) || !(targetingMs >= 0.0) || Double.isInfinite(captureMs)
                    || Double.isInfinite(targetingMs)) {
                throw new IllegalArgumentException("latencies must be finite and >= 0, got " + captureMs
                        + " and " + targetingMs);
            }
            this.captureLatencyMs = captureMs;
            this.targetingLatencyMs = targetingMs;
            return this;
        }

        /**
         * The tag family string each fiducial reports ({@code getFamily()}). Default {@code "36h11"}.
         *
         * @param tagFamily the family label
         * @return this builder
         */
        public Builder family(String tagFamily) {
            if (tagFamily == null) {
                throw new IllegalArgumentException("family must not be null");
            }
            this.family = tagFamily;
            return this;
        }

        /**
         * Turns on the seeded Gaussian noise the class comment describes. Off by default.
         *
         * @param seed                 the {@link SeededRandom} seed; same seed, same noise
         * @param txTyStdDevDegrees    standard deviation added to each tag's tx and ty, degrees, &ge; 0
         * @param positionStdDevInches standard deviation added to botpose and botpose_orb x and y, inches
         * @param yawStdDevDegrees     standard deviation added to botpose yaw, degrees, &ge; 0
         * @return this builder
         */
        public Builder noise(int seed, double txTyStdDevDegrees, double positionStdDevInches,
                double yawStdDevDegrees) {
            requireStdDev(txTyStdDevDegrees);
            requireStdDev(positionStdDevInches);
            requireStdDev(yawStdDevDegrees);
            this.noiseEnabled = true;
            this.noiseSeed = seed;
            this.txTyStdDevDegrees = txTyStdDevDegrees;
            this.positionStdDevInches = positionStdDevInches;
            this.yawStdDevDegrees = yawStdDevDegrees;
            return this;
        }

        private static void requireStdDev(double value) {
            if (!(value >= 0.0) || Double.isInfinite(value)) {
                throw new IllegalArgumentException("a standard deviation must be finite and >= 0, got " + value);
            }
        }

        /** {@return the fake, stopped, at sim time 0, pipeline 0, robot at the field origin facing +X} */
        public FakeLimelight3A build() {
            return new FakeLimelight3A(this);
        }
    }
}
