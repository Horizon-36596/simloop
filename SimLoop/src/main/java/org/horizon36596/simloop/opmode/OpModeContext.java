package org.horizon36596.simloop.opmode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.internal.opmode.OpModeServices;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.horizon36596.simloop.sim.FakeTimer;

/**
 * What the FTC runtime does to an OpMode around its code, done here instead: inject the public fields,
 * hand over the latest gamepad data before each piece of user code, and keep {@code time} and
 * {@code getRuntime()} on the sim clock. Shared by both kinds of OpMode, so they see the same world.
 *
 * <p><b>Sim runtime.</b> {@code OpMode.time} is written straight from the sim clock and is exact.
 * {@code getRuntime()} cannot be: it is {@code (System.nanoTime() - startTime) / 1e9}, and a team's OpMode
 * cannot be made to override it. So before each piece of user code the private {@code startTime} is moved so
 * that {@code getRuntime()} reads the sim runtime at that instant; during the code it then also counts the
 * real time the code has taken so far in this tick, typically microseconds. The SDK reads the clock for that,
 * not this class (which asks the SDK's own {@code resetRuntime()} to stamp it, and never reads a clock
 * itself). An OpMode that calls {@code resetRuntime()} restarts the sim runtime from that tick.
 */
final class OpModeContext {

    private final OpMode opMode;
    private final FakeTimer timer;

    /** The drivers' gamepads as the Driver Station holds them; the OpMode gets copies. */
    final Gamepad driverStationGamepad1 = new Gamepad();

    /** The operator's gamepad as the Driver Station holds it. */
    final Gamepad driverStationGamepad2 = new Gamepad();

    /** Sim time, in seconds, at which {@code getRuntime()} reads zero. */
    private double runtimeOriginSeconds;

    /** The {@code startTime} this class last wrote, so a {@code resetRuntime()} by the OpMode can be seen. */
    private long anchoredStartTime;

    OpModeContext(OpMode opMode, FakeTimer timer) {
        this.opMode = opMode;
        this.timer = timer;
    }

    /**
     * Injects everything the SDK injects before INIT, the way {@code OpModeInternal.internalInit()} does:
     * hardware map, telemetry, fresh gamepads with edge detection reset and the 0.5 trigger threshold,
     * not started, not stopped, and a services object for {@code requestOpModeStop()}.
     *
     * <p>Telemetry goes through {@link TelemetryRewiring}, so copies the OpMode made of the SDK's own
     * telemetry while it was constructed are pointed at {@code telemetry} too. Each SDK telemetry object is
     * then set to send on every update, so that one the rewiring could not reach reports every use to
     * {@code services} rather than being throttled on the wall clock.
     *
     * @return what the rewiring did, for the run's warnings
     */
    TelemetryRewiring inject(HardwareMap hardwareMap, RecordingTelemetry telemetry, OpModeServices services) {
        opMode.hardwareMap = hardwareMap;
        TelemetryRewiring rewiring = TelemetryRewiring.install(opMode, telemetry);
        for (Telemetry sdkTelemetry : rewiring.sdkTelemetry()) {
            sdkTelemetry.setMsTransmissionInterval(0);
        }
        opMode.gamepad1 = new Gamepad();
        opMode.gamepad2 = new Gamepad();
        opMode.gamepad1.resetEdgeDetection();
        opMode.gamepad2.resetEdgeDetection();
        opMode.gamepad1.setTriggerThreshold(0.5f);
        opMode.gamepad2.setTriggerThreshold(0.5f);
        SdkOpModeAccess.setStarted(opMode, false);
        SdkOpModeAccess.setStopRequested(opMode, false);
        SdkOpModeAccess.setServices(opMode, services);
        return rewiring;
    }

    /** Makes {@code getRuntime()} and {@code time} count from the sim clock's current reading: INIT. */
    void startRuntimeHere() {
        runtimeOriginSeconds = timer.time();
    }

    /** PLAY, the way {@code OpModeInternal.internalStart()} does it: reset edge detection, then started. */
    void markStarted() {
        opMode.gamepad1.resetEdgeDetection();
        opMode.gamepad2.resetEdgeDetection();
        SdkOpModeAccess.setStopRequested(opMode, false);
        SdkOpModeAccess.setStarted(opMode, true);
    }

    /** STOP: what {@code isStopRequested()} reads becomes true. */
    void markStopRequested() {
        SdkOpModeAccess.setStopRequested(opMode, true);
    }

    /**
     * Runs before every piece of user code: the latest gamepad data is copied in (which is what updates the
     * SDK's edge detection), and {@code time} and {@code getRuntime()} are set to the sim runtime.
     */
    void beforeUserCode() {
        opMode.gamepad1.copy(driverStationGamepad1);
        opMode.gamepad2.copy(driverStationGamepad2);
        opMode.time = anchorRuntime();
    }

    /**
     * Runs when user code held in a wait across ticks carries on, as an iterative OpMode's {@code init()} does
     * after a sleep: {@code getRuntime()} reads the sim runtime again, as it keeps following the real clock on
     * the robot. Gamepads and {@code time} stay as they were, because the SDK updates those only before a call.
     */
    void resumeUserCode() {
        afterUserCode(); // keep a resetRuntime() the code made before it waited
        anchorRuntime();
    }

    /**
     * Moves the SDK's private {@code startTime} so that {@code getRuntime()} reads the sim runtime now.
     *
     * @return the sim runtime, in seconds since INIT or the OpMode's last {@code resetRuntime()}
     */
    private double anchorRuntime() {
        double runtimeSeconds = timer.time() - runtimeOriginSeconds;
        opMode.resetRuntime();
        anchoredStartTime = SdkOpModeAccess.startTime(opMode) - Math.round(runtimeSeconds * 1e9);
        SdkOpModeAccess.setStartTime(opMode, anchoredStartTime);
        return runtimeSeconds;
    }

    /** Runs after every piece of user code: an OpMode that called {@code resetRuntime()} restarts the count. */
    void afterUserCode() {
        if (SdkOpModeAccess.startTime(opMode) != anchoredStartTime) {
            runtimeOriginSeconds = timer.time();
        }
    }
}
