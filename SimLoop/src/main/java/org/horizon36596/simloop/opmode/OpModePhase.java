package org.horizon36596.simloop.opmode;

/**
 * Where an {@link OpModeRunner} run is in the Driver Station's INIT / PLAY / STOP sequence. Each sim tick
 * belongs to exactly one phase, and a run always has the same shape: one {@link #INIT} tick, then
 * {@link #INIT_LOOP} ticks, then {@link #ACTIVE} ticks, then one {@link #STOP} tick.
 */
public enum OpModePhase {

    /**
     * The tick on which INIT is pressed. An iterative OpMode's {@code init()} runs; a {@code LinearOpMode}'s
     * {@code runOpMode()} starts and runs until its first lifecycle check or {@code waitForStart()}.
     */
    INIT,

    /**
     * Ticks between INIT and PLAY. An iterative OpMode's {@code init_loop()} runs once per tick; a
     * {@code LinearOpMode} waiting in {@code waitForStart()} does nothing, and one running a
     * {@code while (opModeInInit())} loop runs one pass of it per tick.
     */
    INIT_LOOP,

    /**
     * Ticks after PLAY. The first one runs an iterative OpMode's {@code start()} and then its first
     * {@code loop()}; every later one runs {@code loop()}. A {@code LinearOpMode} returns from
     * {@code waitForStart()} on the first one and then runs one pass of its {@code while (opModeIsActive())}
     * loop per tick.
     */
    ACTIVE,

    /**
     * The tick on which STOP is pressed. An iterative OpMode's {@code stop()} runs; a {@code LinearOpMode}
     * sees {@code isStopRequested()} become true and must return from {@code runOpMode()}.
     */
    STOP
}
