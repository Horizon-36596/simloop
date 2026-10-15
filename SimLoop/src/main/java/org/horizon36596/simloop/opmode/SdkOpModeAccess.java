package org.horizon36596.simloop.opmode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.internal.opmode.OpModeServices;

import java.lang.reflect.Field;

/**
 * The four pieces of an SDK {@link OpMode}'s state that the FTC runtime sets and team code cannot: whether
 * PLAY has been pressed, whether STOP has been requested, the services object {@code requestOpModeStop()}
 * reports to, and the start of {@code getRuntime()}.
 *
 * <p><b>Why reflection.</b> On the robot, {@code OpModeManagerImpl} lives in the SDK's own package and writes
 * these directly. {@code isStarted}, {@code stopRequested} and {@code internalOpModeServices} are
 * package-private fields of {@code OpModeInternal}, and {@code startTime} is a private field of
 * {@code OpMode}. Reflection is the only way to set them without putting a class of ours inside
 * {@code com.qualcomm.robotcore.eventloop.opmode}, which would collide with the SDK the moment this module is
 * packaged into an app. The names were read from the RobotCore 11.2.1 and 12.0.0 bytecode ({@code javap -p}),
 * which agree; if an SDK release renames one, every run fails at once with the field's name in the message
 * rather than half-working.
 *
 * <p>The fields that are {@code public} ({@code hardwareMap}, {@code telemetry}, {@code gamepad1},
 * {@code gamepad2}, {@code time}) are written directly by the runner and do not appear here.
 */
final class SdkOpModeAccess {

    private static final String OP_MODE_INTERNAL = "com.qualcomm.robotcore.eventloop.opmode.OpModeInternal";

    private static final Field IS_STARTED = field(OP_MODE_INTERNAL, "isStarted");
    private static final Field STOP_REQUESTED = field(OP_MODE_INTERNAL, "stopRequested");
    private static final Field SERVICES = field(OP_MODE_INTERNAL, "internalOpModeServices");
    private static final Field START_TIME = field(OpMode.class.getName(), "startTime");

    private SdkOpModeAccess() {
    }

    /** Sets what {@code isStarted()} reads: true once PLAY has been pressed. */
    static void setStarted(OpMode opMode, boolean started) {
        write(IS_STARTED, opMode, started);
    }

    /** Sets what {@code isStopRequested()} reads: true once STOP has been pressed or the OpMode asked to stop. */
    static void setStopRequested(OpMode opMode, boolean stopRequested) {
        write(STOP_REQUESTED, opMode, stopRequested);
    }

    /**
     * Installs the object {@code requestOpModeStop()} calls. Without one, that SDK method dereferences a null
     * and an OpMode that ends itself would crash instead.
     */
    static void setServices(OpMode opMode, OpModeServices services) {
        write(SERVICES, opMode, services);
    }

    /** {@return the raw {@code startTime} the SDK's {@code getRuntime()} subtracts from, in nanoseconds} */
    static long startTime(OpMode opMode) {
        try {
            return START_TIME.getLong(opMode);
        } catch (IllegalAccessException e) {
            throw new IllegalStateException("SimLoop could not read OpMode.startTime", e);
        }
    }

    /** Overwrites the raw {@code startTime}, in the same nanosecond units the SDK stores it in. */
    static void setStartTime(OpMode opMode, long startTimeNanos) {
        try {
            START_TIME.setLong(opMode, startTimeNanos);
        } catch (IllegalAccessException e) {
            throw new IllegalStateException("SimLoop could not write OpMode.startTime", e);
        }
    }

    private static void write(Field field, OpMode opMode, Object value) {
        try {
            field.set(opMode, value);
        } catch (IllegalAccessException e) {
            throw new IllegalStateException("SimLoop could not write the SDK's " + field.getName(), e);
        }
    }

    private static Field field(String className, String name) {
        try {
            Field field = Class.forName(className).getDeclaredField(name);
            field.setAccessible(true);
            return field;
        } catch (ReflectiveOperationException | RuntimeException e) {
            throw new IllegalStateException("SimLoop's OpMode runner expects the FTC SDK's " + className + "."
                    + name + " (present in RobotCore 11.2.1 and 12.0.0). This SDK does not have it, so the"
                    + " runner cannot drive an OpMode's lifecycle on it.", e);
        }
    }
}
