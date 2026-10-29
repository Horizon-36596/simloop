package org.horizon36596.simloop.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.robocol.TelemetryMessage;

import org.firstinspires.ftc.robotcore.internal.opmode.OpModeServices;
import org.horizon36596.simloop.config.SimRobotConfig;
import org.horizon36596.simloop.fakehardware.FakeHardwareMap;
import org.horizon36596.simloop.fakehardware.FakeVoltageSensor;
import org.horizon36596.simloop.sim.FakeTimer;
import org.horizon36596.simloop.sim.ScenarioRunner;
import org.horizon36596.simloop.simclock.SimClockLoader;

import java.lang.reflect.Constructor;
import java.lang.reflect.InvocationTargetException;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * Runs a team's own OpMode, unchanged, against fake hardware: an iterative {@link OpMode} or a
 * {@link LinearOpMode}, headless, one fixed sim tick at a time, and deterministically.
 *
 * <pre>{@code
 * FakeHardwareMap hardwareMap = new FakeHardwareMap();
 * hardwareMap.register("left", new FakeMotor(2800, 10));
 * hardwareMap.register("right", new FakeMotor(2800, 10));
 *
 * OpModeRunResult result = new OpModeRunner(new MyTeleOp(), hardwareMap)
 *         .initLoopTicks(10)                  // 0.2 s between INIT and PLAY
 *         .activeTicks(150)                   // 3 s of driving at 50 Hz
 *         .gamepads((phase, tick, gamepad1, gamepad2) -> gamepad1.left_stick_y = -1f)
 *         .run();                             // or .runLogged(robotConfig, Paths.get("build/sim/x.rlog"))
 * }</pre>
 *
 * <h2>The shape of a run</h2>
 * One {@link OpModePhase#INIT} tick, {@code initLoopTicks} {@link OpModePhase#INIT_LOOP} ticks,
 * {@code activeTicks} {@link OpModePhase#ACTIVE} ticks and one {@link OpModePhase#STOP} tick. Each tick:
 * the {@link FakeTimer} advances by {@code tickSeconds}; the {@link GamepadScript} sets the gamepads; the
 * OpMode's code runs (see {@link OpModePhase} for which code, per phase); then the world moves - every fake
 * device in the hardware map, then whatever {@link #eachTick} adds, such as a drive plant. That is the real
 * order: code writes powers, and the robot responds before the next loop.
 *
 * <h2>What the OpMode is given</h2>
 * The SDK assigns an OpMode's fields after constructing it, and so does this: {@code hardwareMap} is the
 * fake map; {@code gamepad1} and {@code gamepad2} receive the script's gamepads the way the SDK delivers
 * Driver Station data, so {@code aWasPressed()} and friends work; {@code telemetry} is a
 * {@link RecordingTelemetry}; {@code time} and {@code getRuntime()} follow the sim clock; and
 * {@code requestOpModeStop()} and {@code terminateOpModeNow()} end the OpMode as they would on the robot.
 * An OpMode that reads {@code hardwareMap} in a field initializer sees null, exactly as on the robot.
 *
 * <h2>Telemetry copied before INIT</h2>
 * The SDK's constructor creates the OpMode's telemetry before any field initializer of the team's class
 * runs, so {@code Telemetry t = telemetry;} or a dashboard's
 * {@code new MultipleTelemetry(telemetry, dashboard.getTelemetry())} as a field captures the SDK's own
 * object, and no hook runs early enough to hand it the recording one instead. So the runner fixes the copies
 * afterwards, before INIT: every reference to the SDK's telemetry (or its {@code log()}) in the OpMode's own
 * fields, in the objects they hold (a {@code MultipleTelemetry}, a team's helper class) up to four objects
 * deep, and in the arrays, {@code List}s and {@code Map}s there, is replaced with the recording telemetry.
 * What that cannot fix - a copy in a {@code Set} or immutable list, in a {@code static} field, captured by a
 * lambda, deeper than four objects, or items added before INIT - is reported in
 * {@link OpModeRunResult#warnings()}: at INIT when the runner can see it, and otherwise the first time the
 * OpMode sends an update through the SDK's object, which the runner hears about, or at the end of the run if
 * the OpMode wrote to that object and never sent it.
 *
 * <h2>{@code OpMode.blackboard}</h2>
 * The SDK's blackboard is one static map for the whole JVM, which is how it carries data from an autonomous
 * to the TeleOp after it on a robot. In a test suite that would let one test's run leak into the next, so a
 * run clears it before INIT unless told to {@link #keepBlackboard(boolean) keep it}. It is not cleared after
 * the run: a test can read what the OpMode left, and the next run can keep it.
 *
 * <h2>Threads the OpMode starts</h2>
 * They are not stepped: they run in real time beside the sim. The runner notices them (see
 * {@link #failOnOpModeThreads(boolean)}) and says so in {@link OpModeRunResult#warnings()}.
 *
 * <h2>Two ways to hand over the OpMode</h2>
 * <ul>
 *   <li><b>{@code new OpModeRunner(opMode, hardwareMap)}</b> runs the instance the test built. Nothing about
 *       the OpMode's classes changes, so the limits below apply in full.</li>
 *   <li><b>{@link #forTeamClass(String, FakeHardwareMap)}</b> builds the OpMode itself, at the start of the
 *       run, inside a fresh {@link SimClockLoader} that moves the team's clock calls onto the sim clock:
 *       {@code ElapsedTime}, {@code System.nanoTime()}, {@code System.currentTimeMillis()},
 *       {@code Thread.sleep}, {@code TimeUnit.sleep}, Android's {@code SystemClock}, {@code sleep(ms)} with
 *       any duration, computed or not, and {@code idle()}. The price is class identity: the team's classes
 *       are loaded a second time for each run, so a team object the test built itself, or an earlier run
 *       left on the blackboard, cannot be cast to their types, and their statics start fresh on every run.
 *       {@code SimLoop/docs/unchanged-team-code.md} is the guide.</li>
 * </ul>
 *
 * <h2>Determinism and its edges</h2>
 * Nothing here reads a wall clock, and the thread the OpMode's code runs on only runs while the sim waits
 * for it, so the same OpMode, script and world give the same run every time. The limits, all of which are in
 * the OpMode's code rather than here, and all of which the module README lists:
 * <ul>
 *   <li>Run as an instance, {@code ElapsedTime}, {@code System.nanoTime()}, {@code System.currentTimeMillis()}
 *       and {@code Thread.sleep} read or wait on the real clock, and {@code sleep()} must be given a constant
 *       duration (a computed one fails with its file and line). {@link #forTeamClass} lifts all of these for
 *       the team's own packages.</li>
 *   <li>Either way, the SDK's own and other libraries' clock reads, the real date and time, threads the
 *       OpMode starts itself, and waits other than those above ({@code Object.wait}, {@code Thread.join},
 *       latches, futures, executors) are not moved. {@link #forTeamClass} names in
 *       {@link OpModeRunResult#warnings()} the ones it can see: the SDK's {@code Deadline} timer,
 *       {@code java.time}'s {@code now()} and {@code new Date()}, and known clock-reading libraries such as
 *       Pedro Pathing outside the team packages. On a thread the OpMode starts, a wait runs in real time (and
 *       with {@link #forTeamClass} a {@code Thread.sleep} there is refused and fails the run). On a
 *       {@code LinearOpMode}'s own thread, whose interrupt flag the runner keeps set, such a wait usually
 *       throws {@code InterruptedException} at once, taking no time; after the OpMode's code has caught one,
 *       a wait before its next lifecycle check runs in real time instead and moves no sim time.</li>
 *   <li>{@code getRuntime()} includes the few microseconds of real time the current tick's code has taken,
 *       so telemetry that shows it differs slightly on every run ({@code time} is exact).</li>
 *   <li>Runs share {@code OpMode.blackboard} and PsiKit's static {@code Logger} with every other run in the
 *       JVM. Logged runs take turns ({@link ScenarioRunner#run}), but OpMode tests must not run in parallel
 *       within one JVM.</li>
 * </ul>
 *
 * <p>A runner runs once, and so does the OpMode instance it was given, just as the SDK builds a fresh OpMode
 * for every INIT. Build both again to run again.
 */
public final class OpModeRunner {

    /** The default tick, in seconds: 50 Hz, about what a real FTC control loop manages. */
    public static final double DEFAULT_TICK_SECONDS = 0.02;

    /**
     * The name the runner gives the battery it adds to a map that has no voltage sensor: a Control Hub's own
     * voltage sensor is configured under the hub's name, which is {@code "Control Hub"} unless the team renamed it.
     */
    public static final String DEFAULT_VOLTAGE_SENSOR_NAME = "Control Hub";

    /**
     * The default for {@link #stuckTimeoutMillis(int)}: how long, in milliseconds of real time, an OpMode
     * may run within one tick before the run is failed as stuck.
     */
    public static final int DEFAULT_STUCK_TIMEOUT_MILLIS = 10_000;

    /** The OpMode this run runs: given to the constructor, or built by {@link #claimRun()} for a team class. */
    private OpMode opMode;
    private final FakeHardwareMap hardwareMap;
    /** For {@link #forTeamClass}: the class to build, and the loader its bytes come from; else null. */
    private final String teamClassName;
    private final ClassLoader teamClassParent;
    private List<String> teamPackages = SimClockLoader.DEFAULT_TEAM_PACKAGES;
    /** The loader the team class was built in, once the run has started; null for an instance run. */
    private SimClockLoader simClock;
    private FakeTimer timer = new FakeTimer();
    private int initLoopTicks;
    private int activeTicks;
    private double tickSeconds = DEFAULT_TICK_SECONDS;
    private GamepadScript gamepads = GamepadScript.NONE;
    private ScenarioRunner.ScenarioBody eachTick = deltaTimeSeconds -> { };
    private int stuckTimeoutMillis = DEFAULT_STUCK_TIMEOUT_MILLIS;
    private boolean keepBlackboard;
    private boolean failOnOpModeThreads;
    private boolean used;

    /**
     * Prepares a run of {@code opMode} against {@code hardwareMap}. Nothing runs until {@link #run()} or
     * {@link #runLogged(SimRobotConfig, Path)}.
     *
     * @param opMode      a freshly constructed OpMode, iterative or linear, exactly as the team wrote it
     * @param hardwareMap the fake devices, registered under the robot configuration's names; the runner
     *                    advances every one of them each tick
     */
    public OpModeRunner(OpMode opMode, FakeHardwareMap hardwareMap) {
        if (opMode == null || hardwareMap == null) {
            throw new IllegalArgumentException("OpModeRunner: opMode and hardwareMap must both be non-null");
        }
        this.opMode = opMode;
        this.hardwareMap = hardwareMap;
        this.teamClassName = null;
        this.teamClassParent = null;
    }

    private OpModeRunner(String teamClassName, ClassLoader teamClassParent, FakeHardwareMap hardwareMap) {
        if (teamClassName == null || teamClassName.isEmpty() || hardwareMap == null) {
            throw new IllegalArgumentException("OpModeRunner.forTeamClass: the class name and hardwareMap must both"
                    + " be given");
        }
        this.hardwareMap = hardwareMap;
        this.teamClassName = teamClassName;
        this.teamClassParent = teamClassParent;
    }

    /**
     * Prepares a run of the team's OpMode class {@code className}, built at the start of the run inside a
     * fresh {@link SimClockLoader}, so its unchanged {@code ElapsedTime}s, {@code System.nanoTime()} and
     * {@code System.currentTimeMillis()} read the sim clock, and its {@code Thread.sleep}, {@code sleep(ms)}
     * (any duration, computed or not) and {@code idle()} wait in sim time. Everything else about the run is as
     * for {@link #OpModeRunner(OpMode, FakeHardwareMap)}.
     *
     * <p>Only classes in the team packages are rewritten: {@link SimClockLoader#DEFAULT_TEAM_PACKAGES}
     * ({@code org.firstinspires.ftc.teamcode.} and {@code com.seattlesolvers.solverslib.}) unless
     * {@link #teamPackages(String...)} says otherwise. The OpMode must be in one of them.
     *
     * <p><b>Class identity.</b> The loader loads the team's classes a second time, so they are not the classes
     * the test itself compiled against: the OpMode cannot be cast to its own class (read what it did from the
     * fake hardware, its telemetry, {@code OpMode.blackboard} or reflection), a team object built with
     * {@code new} in the test and passed in throws {@code ClassCastException}, and every team static starts
     * fresh on each run, as on a robot after a restart. {@link #opMode()} returns the instance that ran.
     *
     * <p>A {@code Thread.sleep} on a thread the sim does not step - an iterative OpMode's, or one the OpMode
     * starts - is refused with the thread and the call site, and so is a {@code this::sleep} in a class first
     * loaded mid-run. The refusal is thrown where the call was made, and it also fails the run with an
     * {@link OpModeException} carrying that message: even when the thread that made the call dies of it
     * unwatched, and even when the team's code catches it, since the run then did not do what the robot
     * would. A loop that waits for a timer without a lifecycle check, {@code sleep()} or {@code idle()} inside
     * it cannot see sim time move, and fails after {@link #stuckTimeoutMillis(int)} with a message saying so.
     *
     * @param className   the OpMode's full class name, such as {@code "org.firstinspires.ftc.teamcode.MyAuto"}
     * @param hardwareMap the fake devices, registered under the robot configuration's names
     * @return a runner for that class
     * @throws IllegalArgumentException if the name is empty or the hardware map null; a name that cannot be
     *                                  found, or is not an OpMode, fails when the run starts
     */
    public static OpModeRunner forTeamClass(String className, FakeHardwareMap hardwareMap) {
        ClassLoader context = Thread.currentThread().getContextClassLoader();
        return new OpModeRunner(className, context != null ? context : OpModeRunner.class.getClassLoader(),
                hardwareMap);
    }

    /**
     * As {@link #forTeamClass(String, FakeHardwareMap)}, naming the class by its literal. Only the class's
     * name and loader are used: the run builds its own copy of it.
     *
     * @param teamClass   the OpMode's class, such as {@code MyAuto.class}
     * @param hardwareMap the fake devices, registered under the robot configuration's names
     * @return a runner for that class
     * @throws IllegalArgumentException if either is null
     */
    public static OpModeRunner forTeamClass(Class<? extends OpMode> teamClass, FakeHardwareMap hardwareMap) {
        if (teamClass == null) {
            throw new IllegalArgumentException("OpModeRunner.forTeamClass: teamClass must be non-null");
        }
        return new OpModeRunner(teamClass.getName(), teamClass.getClassLoader(), hardwareMap);
    }

    /**
     * Sets which classes a {@link #forTeamClass} run rewrites onto the sim clock, replacing the default
     * {@link SimClockLoader#DEFAULT_TEAM_PACKAGES}. Name every package of the team's own code, and any
     * library whose timers should follow the sim clock too.
     *
     * @param prefixes class-name prefixes, each ending in a dot, such as {@code "com.example.myteam."}
     * @return this runner
     * @throws IllegalStateException    if this runner was given an OpMode instance, whose classes are never
     *                                  rewritten
     * @throws IllegalArgumentException if a prefix is empty, lacks its final dot, or is inside the JDK, the FTC
     *                                  SDK, SimLoop, PsiKit or JUnit, which are never rewritten
     */
    public OpModeRunner teamPackages(String... prefixes) {
        if (teamClassName == null) {
            throw new IllegalStateException("OpModeRunner: teamPackages(...) is for a runner made with"
                    + " forTeamClass(...); an OpMode instance the test built is run as it is.");
        }
        this.teamPackages = SimClockLoader.checkedTeamPackages(Arrays.asList(requireNonNull(prefixes, "prefixes")));
        return this;
    }

    /**
     * {@return the OpMode this runner runs}: the instance it was given, or, for {@link #forTeamClass}, the
     * one it built inside its loader when the run started (null before that). Its class is the loaded copy,
     * so read its fields by reflection rather than by casting it to the team's class.
     */
    public OpMode opMode() {
        return opMode;
    }

    /**
     * Uses {@code timer} as the sim clock instead of a fresh one, so that anything else reading sim time
     * (a sim-backed timer the OpMode was handed, a plant) reads the same clock.
     *
     * @param timer the clock; it advances by the tick at the start of every tick
     * @return this runner
     */
    public OpModeRunner timer(FakeTimer timer) {
        this.timer = requireNonNull(timer, "timer");
        return this;
    }

    /**
     * Sets how many ticks pass between INIT and PLAY. Zero is allowed: PLAY straight after INIT.
     *
     * @param ticks number of {@link OpModePhase#INIT_LOOP} ticks, zero or more
     * @return this runner
     */
    public OpModeRunner initLoopTicks(int ticks) {
        this.initLoopTicks = requireNotNegative(ticks, "initLoopTicks");
        return this;
    }

    /**
     * Sets how many ticks pass between PLAY and STOP. Zero is allowed: STOP during init, and an iterative
     * OpMode's {@code start()} never runs, as on the robot.
     *
     * @param ticks number of {@link OpModePhase#ACTIVE} ticks, zero or more; 1500 at 50 Hz is a 30 s auto
     * @return this runner
     */
    public OpModeRunner activeTicks(int ticks) {
        this.activeTicks = requireNotNegative(ticks, "activeTicks");
        return this;
    }

    /**
     * Sets the fixed sim tick.
     *
     * @param seconds the tick length, in seconds; finite and greater than zero (default
     *                {@value #DEFAULT_TICK_SECONDS})
     * @return this runner
     */
    public OpModeRunner tickSeconds(double seconds) {
        if (!(seconds > 0.0) || Double.isInfinite(seconds)) {
            throw new IllegalArgumentException("OpModeRunner: tickSeconds must be finite and > 0, got " + seconds);
        }
        this.tickSeconds = seconds;
        return this;
    }

    /**
     * Sets what the drivers do. Default: {@link GamepadScript#NONE}.
     *
     * @param script called at the start of every tick, before the OpMode's code
     * @return this runner
     */
    public OpModeRunner gamepads(GamepadScript script) {
        this.gamepads = requireNonNull(script, "script");
        return this;
    }

    /**
     * Adds world behaviour that the fake devices do not model themselves, such as a drive plant turning
     * wheel speeds into a pose. It runs every tick, after the OpMode's code and after the hardware map's
     * devices have advanced.
     *
     * @param body the extra per-tick step, given the tick length in seconds
     * @return this runner
     */
    public OpModeRunner eachTick(ScenarioRunner.ScenarioBody body) {
        this.eachTick = requireNonNull(body, "body");
        return this;
    }

    /**
     * Sets how long the OpMode's code may run without handing control back before the run fails rather than
     * hangs: for an iterative OpMode, one call of {@code init()}, {@code init_loop()}, {@code start()},
     * {@code loop()} or {@code stop()}; for a {@code LinearOpMode}, one tick, or the time it takes to return
     * after STOP. It is counted in one-millisecond waits, not read from a clock, so it is at least this long
     * and has no effect on a healthy run. The failure names the phase and tick and carries the stuck thread's
     * stack; that thread is left behind as a daemon (an iterative OpMode's is interrupted first, so a stuck
     * {@code Thread.sleep} or blocking wait ends) and never runs OpMode code again.
     *
     * @param millis the limit, in milliseconds of real time; at least 1 (default
     *               {@value #DEFAULT_STUCK_TIMEOUT_MILLIS})
     * @return this runner
     */
    public OpModeRunner stuckTimeoutMillis(int millis) {
        if (millis < 1) {
            throw new IllegalArgumentException("OpModeRunner: stuckTimeoutMillis must be at least 1, got " + millis);
        }
        this.stuckTimeoutMillis = millis;
        return this;
    }

    /**
     * Keeps {@code OpMode.blackboard} as it is when the run starts, instead of clearing it before INIT (the
     * default), to hand data from one run to the next the way an autonomous hands its end pose to the TeleOp
     * after it on a robot: run the auto, then run the TeleOp with this set.
     *
     * @param keep true to keep whatever the blackboard holds; false (default) to start the run with it empty
     * @return this runner
     */
    public OpModeRunner keepBlackboard(boolean keep) {
        this.keepBlackboard = keep;
        return this;
    }

    /**
     * Makes a thread the OpMode starts itself fail the run, instead of only being reported in
     * {@link OpModeRunResult#warnings()} (the default). Such a thread runs in real time beside the sim, so
     * anything it does to the robot is not deterministic; a test that needs a run to be exact sets this.
     *
     * <p>The runner sees a thread the OpMode's code creates directly or through an executor it builds, once
     * that tick's code has handed control back, and one that dies of an uncaught exception however briefly it
     * lived. It misses a thread given a {@code ThreadGroup} of its own, one that starts and ends normally
     * within a single tick's code, and a pool thread that other code created first. (A sim-time wait refused
     * on any of these fails the run whatever this says; see {@link #forTeamClass}.)
     *
     * @param fail true to fail the run, on the tick the thread is first seen, with an {@link OpModeException}
     * @return this runner
     */
    public OpModeRunner failOnOpModeThreads(boolean fail) {
        this.failOnOpModeThreads = fail;
        return this;
    }

    /**
     * Runs the OpMode from INIT to STOP without writing a log.
     *
     * @return what the run did
     * @throws OpModeException if the OpMode threw, never handed control back, did not stop, made a call
     *                         {@link #forTeamClass} refused, or started a thread while
     *                         {@link #failOnOpModeThreads(boolean)} was set
     * @throws IllegalStateException if this runner has already run
     */
    public OpModeRunResult run() {
        claimRun();
        Run run = new Run();
        return run.execute(() -> {
            for (int tick = 0; tick < run.totalTicks; tick++) {
                timer.advance(tickSeconds);
                run.tick(tickSeconds);
            }
        });
    }

    /**
     * Runs the OpMode from INIT to STOP through {@link ScenarioRunner}, which writes every tick to a PsiKit
     * RLOG and refuses to return a log it cannot vouch for. Anything the OpMode passes to
     * {@code Logger.recordOutput} is in it, keyed under {@code RealOutputs/}.
     *
     * @param config   the robot's sim config, recorded in the log's metadata
     * @param rlogPath the log file to write; must end in {@code .rlog}
     * @return what the run did
     * @throws OpModeException if the OpMode threw, never handed control back, did not stop, made a call
     *                         {@link #forTeamClass} refused, or started a thread while
     *                         {@link #failOnOpModeThreads(boolean)} was set
     * @throws IllegalStateException if this runner has already run, or the log cannot be trusted (see
     *                               {@link ScenarioRunner#run})
     */
    public OpModeRunResult runLogged(SimRobotConfig config, Path rlogPath) {
        claimRun();
        Run run = new Run();
        return run.execute(() -> ScenarioRunner.run(opMode.getClass().getSimpleName(), config, timer,
                run.totalTicks, tickSeconds, rlogPath, run::tick));
    }

    /**
     * Marks this runner used, clears the blackboard unless it is kept, and for {@link #forTeamClass} builds
     * the team's OpMode inside a fresh {@link SimClockLoader} reading this run's timer, as the SDK builds an
     * OpMode when INIT is pressed.
     */
    private void claimRun() {
        if (used) {
            throw new IllegalStateException("OpModeRunner: this runner has already run. The SDK builds a"
                    + " fresh OpMode for every INIT; build a new OpMode and a new runner to run again.");
        }
        used = true;
        // Before a team OpMode is built, so its field initializers and constructor see this run's blackboard.
        if (!keepBlackboard) {
            OpMode.blackboard.clear();
        }
        if (teamClassName != null) {
            simClock = new SimClockLoader(teamClassParent, timer, teamPackages,
                    teamClassName.substring(teamClassName.lastIndexOf('.') + 1));
            opMode = buildTeamOpMode(simClock);
        }
    }

    private OpMode buildTeamOpMode(SimClockLoader loader) {
        String where = "OpModeRunner.forTeamClass(\"" + teamClassName + "\")";
        Class<?> loaded;
        try {
            loaded = Class.forName(teamClassName, false, loader);
        } catch (ClassNotFoundException e) {
            throw new IllegalArgumentException(where + ": SimLoop cannot find that class on the test classpath."
                    + " Check the spelling, and give the full name with its package.", e);
        }
        if (loaded.getClassLoader() != loader) {
            throw new IllegalArgumentException(where + ": that class is not in a package SimLoop rewrites ("
                    + String.join(", ", loader.teamPackages()) + "), so its timers would read the real clock."
                    + " Add its package with teamPackages(...).");
        }
        if (!OpMode.class.isAssignableFrom(loaded)) {
            throw new IllegalArgumentException(where + ": that class is not an OpMode or LinearOpMode.");
        }
        try {
            Constructor<?> constructor = loaded.getDeclaredConstructor();
            constructor.setAccessible(true);
            return (OpMode) constructor.newInstance();
        } catch (NoSuchMethodException e) {
            throw new IllegalArgumentException(where + ": the class needs a constructor that takes no arguments,"
                    + " as the FTC SDK does to build it.", e);
        } catch (InvocationTargetException e) {
            Throwable thrown = e.getCause();
            throw new OpModeException(loaded.getSimpleName() + "'s constructor threw while the OpMode was built"
                    + " for INIT: " + thrown + FailureHints.forFailure(thrown, false), OpModePhase.INIT, 0, thrown);
        } catch (InstantiationException | IllegalAccessException e) {
            throw new IllegalArgumentException(where + ": SimLoop cannot build that class (" + e + "). It must"
                    + " be a concrete class, as the FTC SDK requires.", e);
        }
    }

    private static <T> T requireNonNull(T value, String name) {
        if (value == null) {
            throw new IllegalArgumentException("OpModeRunner: " + name + " must be non-null");
        }
        return value;
    }

    private static int requireNotNegative(int ticks, String name) {
        if (ticks < 0) {
            throw new IllegalArgumentException("OpModeRunner: " + name + " must be zero or more, got " + ticks);
        }
        return ticks;
    }

    /**
     * A real robot's map always holds its hubs' voltage sensors, and robot code that compensates for the battery
     * reads one as {@code hardwareMap.voltageSensor.iterator().next()}, which throws
     * {@code NoSuchElementException} on a map with none. So a map with no voltage sensor gets a
     * {@link FakeVoltageSensor} at its nominal 12 V under {@link #DEFAULT_VOLTAGE_SENSOR_NAME} before the OpMode
     * sees it. It stays in the map after the run. A map that has a voltage sensor of its own is left alone, so a
     * test that wants another voltage registers one.
     */
    private void addBatteryIfMissing() {
        if (hardwareMap.voltageSensor.size() == 0) {
            hardwareMap.register(DEFAULT_VOLTAGE_SENSOR_NAME, new FakeVoltageSensor());
        }
    }

    /** One run's state: which tick it is on, the stepper driving the OpMode, and what the run noticed. */
    private final class Run {
        final int totalTicks = 1 + initLoopTicks + activeTicks + 1;
        final String opModeName = opMode.getClass().getSimpleName();
        final RecordingTelemetry telemetry = new RecordingTelemetry();
        final OpModeContext context = new OpModeContext(opMode, timer);
        final OpModeThreads threads = new OpModeThreads(opModeName);
        final OpModeStepper stepper;
        final TelemetryRewiring rewiring;
        /** The link from the sim-clock hooks to this run, for {@link #forTeamClass}; else null. */
        final SimClockBridge simClockBridge;
        int tick;
        int endedAtTick = -1;

        /** Where the tick in progress is, for a report made from the OpMode's side. */
        volatile String now = "before INIT";

        /** Where the OpMode first sent an update through the SDK's own telemetry, or null. */
        volatile String firstSdkTelemetrySend;

        Run() {
            stepper = opMode instanceof LinearOpMode
                    ? new LinearOpModeStepper((LinearOpMode) opMode, context, tickSeconds, stuckTimeoutMillis,
                            threads.group())
                    : new IterativeOpModeStepper(opMode, context, stuckTimeoutMillis, threads.group());
            addBatteryIfMissing();
            rewiring = context.inject(hardwareMap, telemetry, services());
            simClockBridge = simClock != null ? new SimClockBridge(opModeName, stepper) : null;
            if (simClock != null) {
                simClock.setScheduler(simClockBridge);
            }
        }

        /**
         * Runs {@code ticks}, then makes sure no OpMode thread outlives the run, whatever happened, and fails
         * the run on a refusal or a thread that the last tick's look could not yet see.
         */
        OpModeRunResult execute(Runnable ticks) {
            try {
                ticks.run();
            } catch (RuntimeException | Error failure) {
                try {
                    stepper.shutDown();
                } catch (RuntimeException alsoFailed) {
                    failure.addSuppressed(alsoFailed);
                }
                endSimClockRun();
                if (simClock != null) {
                    for (RuntimeException refusal : simClock.refusals()) {
                        if (!causes(failure, refusal)) {
                            failure.addSuppressed(refusal);
                        }
                    }
                }
                threads.close();
                throw failure;
            }
            try {
                stepper.shutDown();
                // With the sim clock still attached, so a helper that tries to wait now is refused for what it
                // is (a thread the sim does not step), not for a run that has ended.
                threads.awaitExit(stepper.thread());
            } finally {
                endSimClockRun();
            }
            try {
                int lastTick = totalTicks - 1;
                failOnRefusal(OpModePhase.STOP, lastTick);
                String late = threads.look(stepper.thread(), OpModePhase.STOP, lastTick);
                if (late != null && failOnOpModeThreads) {
                    throw startedAThread(late, OpModePhase.STOP, lastTick);
                }
            } catch (RuntimeException failure) {
                threads.close();
                throw failure;
            }
            List<String> warnings = new ArrayList<>(rewiring.warnings());
            String unrecorded = firstSdkTelemetrySend != null
                    ? " sent telemetry through the SDK's own telemetry object (" + firstSdkTelemetrySend + ")"
                    : rewiring.sdkTelemetryWrittenSinceInit()
                            ? " wrote telemetry to the SDK's own telemetry object and never sent it"
                            : null;
            if (unrecorded != null) {
                warnings.add(opModeName + unrecorded + ", which SimLoop does not record: a copy of telemetry made"
                        + " before INIT that the runner could not reach (a static field, a lambda, or deeper"
                        + " than " + TelemetryRewiring.MAX_DEPTH + " objects into a helper). Copy telemetry in"
                        + " init() or runOpMode() instead.");
            }
            warnings.addAll(threads.warnings());
            if (simClock != null) {
                warnings.addAll(simClock.warnings()); // real-clock reads the rewrite named but could not move
            }
            threads.close();
            return new OpModeRunResult(totalTicks, endedAtTick, timer.time(), telemetry, warnings);
        }

        /**
         * {@return true if {@code refusal} is {@code failure} or among its causes}: the run already fails with
         * it, so it need not be added again.
         */
        private boolean causes(Throwable failure, Throwable refusal) {
            Throwable cause = failure;
            for (int depth = 0; cause != null && depth < FailureHints.MAX_CAUSES; depth++, cause = cause.getCause()) {
                if (cause == refusal) {
                    return true;
                }
            }
            return false;
        }

        /**
         * Fails the run if the sim-clock loader has recorded a refusal: a wait it could not give sim time, or a
         * class it could not rewrite. The refusal itself was thrown where it happened, but a thread the OpMode
         * started dies of it unwatched, and team code can catch it, so without this a run that did not do
         * what the robot would do could pass.
         */
        private void failOnRefusal(OpModePhase phase, int atTick) {
            List<RuntimeException> refusals = simClock != null ? simClock.refusals() : null;
            if (refusals == null || refusals.isEmpty()) {
                return;
            }
            RuntimeException first = refusals.get(0);
            OpModeException failure = new OpModeException("SimLoop refused a call in " + opModeName + "'s code,"
                    + " so the run fails even though that code carried on (the refusal killed a thread the OpMode"
                    + " started, or a catch took it): " + first.getMessage(), phase, atTick, first);
            for (RuntimeException more : refusals.subList(1, refusals.size())) {
                failure.addSuppressed(more);
            }
            throw failure;
        }

        private OpModeException startedAThread(String name, OpModePhase phase, int atTick) {
            return new OpModeException(opModeName + " started a thread of its own, \"" + name + "\", during "
                    + phase + " (tick " + atTick + "). SimLoop does not step it, so the run is not"
                    + " deterministic, and failOnOpModeThreads is set. The thread is left running.", phase,
                    atTick, null);
        }

        /**
         * After shutDown: the run is over, so nothing is left to wait in sim time, and an OpMode thread still
         * running (the stepper gave up on it) is ended at its next clock read instead of spinning forever.
         */
        private void endSimClockRun() {
            if (simClock != null) {
                simClock.setScheduler(null);
                if (stepper.thread().isAlive()) {
                    simClock.abandon(stepper.thread());
                }
            }
        }

        void tick(double deltaTimeSeconds) {
            int thisTick = tick++;
            OpModePhase phase;
            int tickInPhase;
            if (thisTick == 0) {
                phase = OpModePhase.INIT;
                tickInPhase = 0;
            } else if (thisTick <= initLoopTicks) {
                phase = OpModePhase.INIT_LOOP;
                tickInPhase = thisTick - 1;
            } else if (thisTick <= initLoopTicks + activeTicks) {
                phase = OpModePhase.ACTIVE;
                tickInPhase = thisTick - 1 - initLoopTicks;
            } else {
                phase = OpModePhase.STOP;
                tickInPhase = 0;
            }
            if (thisTick == 0) {
                context.startRuntimeHere();
            }
            now = "first in " + phase + ", tick " + thisTick;
            gamepads.apply(phase, tickInPhase, context.driverStationGamepad1, context.driverStationGamepad2);
            stepper.runTick(phase, tickInPhase, thisTick);
            String started = threads.look(stepper.thread(), phase, thisTick);
            failOnRefusal(phase, thisTick); // first: a refused helper is the more telling of the two failures
            if (started != null && failOnOpModeThreads) {
                throw startedAThread(started, phase, thisTick);
            }
            if (phase != OpModePhase.STOP && endedAtTick < 0 && stepper.hasEnded()) {
                endedAtTick = thisTick;
            }
            hardwareMap.updateAll(deltaTimeSeconds);
            eachTick.tick(deltaTimeSeconds);
        }

        OpModeServices services() {
            return new OpModeServices() {
                /** Only the SDK's own telemetry sends here: the recording one never does. */
                @Override
                public void refreshUserTelemetry(TelemetryMessage message, double sendTimeSeconds) {
                    if (firstSdkTelemetrySend == null) {
                        firstSdkTelemetrySend = now;
                    }
                }

                @Override
                public void requestOpModeStop(OpMode opModeToStop) {
                    stepper.opModeAskedToStop();
                }
            };
        }
    }
}
