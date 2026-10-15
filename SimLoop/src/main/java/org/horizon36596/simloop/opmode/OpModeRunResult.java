package org.horizon36596.simloop.opmode;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * What an {@link OpModeRunner} run did, for a test to assert on. What the robot did is read from the fake
 * hardware and plants the test built, or from the RLOG of a logged run; this holds only what the runner
 * itself knows.
 */
public final class OpModeRunResult {

    private final int ticks;
    private final int endedAtTick;
    private final double finalTimeSeconds;
    private final RecordingTelemetry telemetry;
    private final List<String> warnings;

    OpModeRunResult(int ticks, int endedAtTick, double finalTimeSeconds, RecordingTelemetry telemetry,
                    List<String> warnings) {
        this.ticks = ticks;
        this.endedAtTick = endedAtTick;
        this.finalTimeSeconds = finalTimeSeconds;
        this.telemetry = telemetry;
        this.warnings = Collections.unmodifiableList(new ArrayList<>(warnings));
    }

    /** {@return how many ticks ran, every phase included: {@code 1 + initLoopTicks + activeTicks + 1}} */
    public int ticks() {
        return ticks;
    }

    /**
     * {@return true if the OpMode finished before STOP was pressed}: a {@code LinearOpMode} whose
     * {@code runOpMode()} returned, or any OpMode that called {@code requestOpModeStop()} or
     * {@code terminateOpModeNow()}. The world kept ticking after it, as a real robot keeps coasting.
     */
    public boolean endedBeforeStop() {
        return endedAtTick >= 0;
    }

    /** {@return the tick, counting from 0 at INIT, on which the OpMode finished early, or -1 if it did not} */
    public int endedAtTick() {
        return endedAtTick;
    }

    /** {@return the sim clock's reading when the run finished, in seconds} */
    public double finalTimeSeconds() {
        return finalTimeSeconds;
    }

    /** {@return everything the OpMode sent to telemetry} */
    public RecordingTelemetry telemetry() {
        return telemetry;
    }

    /**
     * {@return what the run could not simulate faithfully, one sentence each, in the order found; empty when
     * nothing was}: telemetry the OpMode copied before INIT that the runner could not reach, and threads the
     * OpMode started itself. A run with warnings still ran every tick; a test that must not have any asserts
     * this is empty.
     */
    public List<String> warnings() {
        return warnings;
    }
}
