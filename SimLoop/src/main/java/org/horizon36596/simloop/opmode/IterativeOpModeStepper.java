package org.horizon36596.simloop.opmode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/**
 * Steps an iterative {@link OpMode} exactly as the SDK's {@code OpMode.internalRunOpMode()} does
 * (RobotCore 12.0.0 bytecode): {@code init()}; {@code init_loop()} until PLAY; {@code start()} and then
 * {@code loop()} straight after it; {@code loop()} until STOP; {@code stop()}. Before each of those the
 * latest gamepad data is copied in, and after each the SDK calls {@code telemetry.update()} for the OpMode,
 * so this does too.
 *
 * <h2>Its own thread, and the watchdog</h2>
 * Each call runs on a thread of the OpMode's own, as on the robot, while the sim thread waits for it; the two
 * never run at once, so a run is as deterministic as if it were one thread. The wait is what makes a stuck
 * OpMode fail instead of hang: if a call has not returned after {@code stuckTimeoutMillis} one-millisecond
 * waits (counted, not read from a clock), the run fails with the method, phase, tick and the thread's stack.
 * That thread cannot be stopped from outside, so it is interrupted (which ends a stuck {@code Thread.sleep}
 * or blocking wait), marked so that it runs no more OpMode code when it does return, and left as a daemon.
 *
 * <p>Breakpoints in an iterative OpMode still work: a debugger stops at them on any thread, and one that
 * suspends every thread (the default in IntelliJ and Android Studio) pauses the watchdog too.
 *
 * <h2>A wait in {@code init()}</h2>
 * The SDK calls {@code init()} on the OpMode's own thread and starts {@code init_loop()} only once it returns,
 * so on a robot {@code init()} may block: a library that sleeps while a sensor calibrates (Pedro Pathing's
 * Pinpoint localizer sleeps 500 ms in its constructor) just makes INIT take longer. Under
 * {@link OpModeRunner#forTeamClass} such a rewritten wait holds {@code init()} for whole ticks of sim time,
 * rounded up, through {@link #holdForSimSeconds}. While it holds:
 * <ul>
 *   <li>sim time moves and the devices and plants update, and no other OpMode method runs, so the tick on
 *       which {@code init()} returns runs nothing else and {@code init_loop()} starts on the tick after;</li>
 *   <li>gamepads and {@code time} are not updated, since the SDK updates them only before a call
 *       ({@code OpMode.internalPreUserCode()}), but {@code getRuntime()} follows the sim clock as it follows
 *       the real one on the robot;</li>
 *   <li>if PLAY comes first, {@code init_loop()} never runs and {@code start()} runs, with the first
 *       {@code loop()}, on the tick after {@code init()} returns, as {@code internalRunOpMode()} goes straight
 *       to {@code start()} once {@code isStarted} is set;</li>
 *   <li>if STOP comes first, the wait returns normally, because the SDK does not interrupt an iterative
 *       OpMode at STOP ({@code OpModeInternal.internalStop()} waits for its thread to finish). Sim time ends
 *       at STOP, so the rest of the wait passes with none; then {@code init()} finishes, {@code start()} runs
 *       if PLAY had come, and {@code stop()} runs.</li>
 * </ul>
 * A wait in {@code init_loop()}, {@code start()}, {@code loop()} or {@code stop()} is still refused (see
 * {@link SimClockBridge}).
 */
final class IterativeOpModeStepper implements OpModeStepper {

    /** The SDK's own force-stop exception, thrown by {@code terminateOpModeNow()}; matched by name. */
    static final String FORCE_STOP = "com.qualcomm.robotcore.eventloop.opmode.OpModeManagerImpl$ForceStopException";

    /** The name {@link #callInProgress} has while {@code init()} runs: the one call that may wait. */
    private static final String INIT = "init()";

    private final OpMode opMode;
    private final OpModeContext context;
    private final double tickSeconds;
    private final int maxWaits;
    private final CallThread thread;

    /** Guards the handoff between the sim thread and {@link #thread}. */
    private final Object lock = new Object();

    /** The call handed to the OpMode thread and not yet taken, or null. Guarded by {@link #lock}. */
    private Runnable pending;

    /** True once the call last handed over has returned or thrown. Guarded by {@link #lock}. */
    private boolean callDone;

    /** True while that call is held in a wait, until the sim resumes it on the next tick. Guarded by {@link #lock}. */
    private boolean parked;

    /** What that call threw, or null. Guarded by {@link #lock}. */
    private Throwable thrown;

    /** True once the OpMode thread must end: the run is over, or the thread was abandoned stuck. */
    private boolean exit;

    /** Set on the OpMode thread by {@code requestOpModeStop()}; read on the sim thread after the call. */
    private volatile boolean askedToStop;

    /** The call running on the OpMode thread, such as {@code "init()"}, or null between calls. */
    private volatile String callInProgress;

    /** STOP has come: a wait in progress, or one started after it, takes no more ticks. */
    private volatile boolean stopping;

    /** Sim thread only: the call held in a wait across ticks, or null. */
    private String heldCall;

    /** Sim thread only: PLAY has been pressed. */
    private boolean played;

    /** Sim thread only: {@code start()} has been called. */
    private boolean startCalled;

    /** Sim thread only: the OpMode has finished. */
    private boolean ended;

    /** Sim thread only: the OpMode thread got stuck and was left behind. */
    private boolean abandoned;

    /**
     * @param tickSeconds the length of one tick, in seconds of sim time, for rounding a wait in {@code init()}
     * @param maxWaits    the stuck timeout, in one-millisecond waits
     */
    IterativeOpModeStepper(OpMode opMode, OpModeContext context, double tickSeconds, int maxWaits,
                           ThreadGroup group) {
        this.opMode = opMode;
        this.context = context;
        this.tickSeconds = tickSeconds;
        this.maxWaits = maxWaits;
        this.thread = new CallThread(group);
    }

    /** The OpMode's own code called {@code requestOpModeStop()}: {@code stop()} runs when that call returns. */
    @Override
    public void opModeAskedToStop() {
        askedToStop = true;
    }

    @Override
    public void runTick(OpModePhase phase, int tickInPhase, int tick) {
        if (ended) {
            return;
        }
        switch (phase) {
            case INIT:
                call(INIT, phase, tick, opMode::init);
                break;
            case INIT_LOOP:
                if (!resumeHeldCall(phase, tick)) {
                    call("init_loop()", phase, tick, opMode::init_loop);
                }
                break;
            case ACTIVE:
                if (tickInPhase == 0) {
                    context.markStarted();
                    played = true;
                }
                if (resumeHeldCall(phase, tick)) {
                    break;
                }
                if (!startCalled) {
                    startCalled = true;
                    call("start()", phase, tick, opMode::start);
                }
                if (!ended) {
                    call("loop()", phase, tick, opMode::loop);
                }
                break;
            case STOP:
                // Before the held call resumes, so its wait ends now: there is no sim time after STOP.
                stopping = true;
                context.markStopRequested();
                resumeHeldCall(phase, tick);
                if (!ended && played && !startCalled) {
                    // PLAY came while init() waited: internalRunOpMode() still calls start() before stop().
                    startCalled = true;
                    call("start()", phase, tick, opMode::start);
                }
                if (!ended) {
                    stop(phase, tick);
                }
                break;
            default:
                throw new IllegalStateException("unknown phase " + phase);
        }
    }

    /** {@return true while {@code init()} is running on the OpMode thread, where a wait holds for sim time} */
    boolean inInit() {
        return INIT.equals(callInProgress);
    }

    /**
     * On the OpMode's own thread, in {@code init()}: holds it for enough ticks to cover {@code seconds} of sim
     * time, the way the sim-clock hooks ask for a rewritten {@code Thread.sleep}. Each tick the sim resumes it
     * instead of calling another OpMode method.
     *
     * @param seconds how long, in seconds of sim time
     * @return true if the wait returns normally: always, unless the run is being torn down, because the SDK
     *         does not interrupt an iterative OpMode at STOP (after STOP the rest of the wait takes no time)
     */
    boolean holdForSimSeconds(double seconds) {
        long ticks = LinearOpModeStepper.ticksFor(seconds, tickSeconds);
        for (long t = 0; t < ticks && !stopping; t++) {
            if (!park()) {
                return false;
            }
        }
        return true;
    }

    /**
     * On the OpMode thread: ends this tick for the call in progress and blocks until the sim resumes it.
     *
     * @return false if the OpMode thread must end instead (the run is being torn down)
     */
    private boolean park() {
        synchronized (lock) {
            parked = true;
            lock.notifyAll();
            while (parked && !exit) {
                try {
                    lock.wait();
                } catch (InterruptedException ignored) {
                    // Only the sim resumes a held call; an interrupt is not that.
                }
            }
            return !exit;
        }
    }

    /**
     * Resumes the call held in a wait, if there is one, for this tick.
     *
     * @return true if there was one: this tick belonged to it, whether or not it has now returned
     */
    private boolean resumeHeldCall(OpModePhase phase, int tick) {
        if (heldCall == null) {
            return false;
        }
        context.resumeUserCode();
        synchronized (lock) {
            parked = false;
            lock.notifyAll();
        }
        finishCall(heldCall, phase, tick);
        return true;
    }

    @Override
    public boolean hasEnded() {
        return ended;
    }

    @Override
    public Thread thread() {
        return thread;
    }

    /** Ends the OpMode thread, unless it was never started or was abandoned stuck. */
    @Override
    public void shutDown() {
        if (thread.getState() == Thread.State.NEW || abandoned) {
            return;
        }
        // The sim thread can arrive here interrupted (a test framework's timeout, or the interrupt that just
        // failed the call in progress); join() would then throw at once and leave the OpMode thread running
        // past the run. Set the flag aside for the join and put it back afterwards, for the caller to see.
        boolean callerInterrupted = Thread.interrupted();
        try {
            exitAndJoin();
        } finally {
            if (callerInterrupted) {
                Thread.currentThread().interrupt();
            }
        }
    }

    /** {@link #shutDown}, with the calling thread's interrupt flag clear. */
    private void exitAndJoin() {
        synchronized (lock) {
            exit = true;
            lock.notifyAll();
        }
        try {
            for (int waits = 0; thread.isAlive() && waits < maxWaits; waits++) {
                thread.join(1);
            }
        } catch (InterruptedException interrupted) {
            Thread.currentThread().interrupt();
        }
    }

    private void stop(OpModePhase phase, int tick) {
        context.markStopRequested();
        call("stop()", phase, tick, opMode::stop);
        ended = true;
    }

    private void call(String method, OpModePhase phase, int tick, Runnable userCode) {
        context.beforeUserCode();
        callInProgress = method;
        handOver(() -> {
            userCode.run();
            opMode.telemetry.update();
        });
        finishCall(method, phase, tick);
    }

    /**
     * Waits for the call on the OpMode thread to return or to be held in a wait, and once it has returned does
     * what follows a call: surface what it threw, and run {@code stop()} if it asked to stop.
     */
    private void finishCall(String method, OpModePhase phase, int tick) {
        Throwable failure;
        synchronized (lock) {
            awaitCallOrPark(method, phase, tick);
            if (parked) {
                heldCall = method;
                context.afterUserCode(); // a resetRuntime() before the wait counts from this tick, not the next
                return;
            }
            failure = thrown;
        }
        heldCall = null;
        callInProgress = null;
        if (failure != null) {
            if (FORCE_STOP.equals(failure.getClass().getName())) {
                ended = true; // terminateOpModeNow(): the SDK ends the OpMode on the spot, without stop()
                return;
            }
            throw new OpModeException(opMode.getClass().getSimpleName() + "." + method + " threw during "
                    + phase + " (tick " + tick + "): " + failure + FailureHints.forFailure(failure, false), phase,
                    tick, failure);
        }
        context.afterUserCode();
        if (askedToStop && !ended) {
            askedToStop = false;
            stop(phase, tick);
        }
    }

    /** Hands {@code work} to the OpMode thread, starting the thread on the first call. */
    private void handOver(Runnable work) {
        if (thread.getState() == Thread.State.NEW) {
            thread.start();
        }
        synchronized (lock) {
            pending = work;
            callDone = false;
            thrown = null;
            lock.notifyAll();
        }
    }

    /**
     * Waits, bounded, until the call on the OpMode thread has returned or is held in a wait. Caller holds the
     * lock.
     *
     * @throws OpModeException if neither happened within {@link #maxWaits} one-millisecond waits
     */
    private void awaitCallOrPark(String method, OpModePhase phase, int tick) {
        int waits = 0;
        while (!callDone && !parked) {
            if (waits++ >= maxWaits) {
                throw abandon(method, phase, tick);
            }
            try {
                lock.wait(1);
            } catch (InterruptedException interrupted) {
                Thread.currentThread().interrupt();
                throw new OpModeException("The thread running the sim was interrupted while "
                        + opMode.getClass().getSimpleName() + "." + method + " was running.", phase, tick,
                        interrupted);
            }
        }
    }

    /** Leaves the stuck OpMode thread behind, harmlessly, and says where it was. Caller holds the lock. */
    private OpModeException abandon(String method, OpModePhase phase, int tick) {
        Throwable whereItIs = new Throwable("where " + thread.getName() + " was when the sim gave up");
        whereItIs.setStackTrace(thread.getStackTrace());
        abandoned = true;
        ended = true;
        exit = true;
        thread.interrupt();
        return new OpModeException(opMode.getClass().getSimpleName() + "." + method + " ran for at least "
                + maxWaits + " ms of real time (stuckTimeoutMillis, counted in one-millisecond waits) without"
                + " returning during " + phase + ", so the sim cannot step it (tick "
                + tick + "). Its thread is a daemon and is left running, interrupted; the cause shows where it"
                + " was.", phase, tick, whereItIs);
    }

    /** The thread the OpMode's calls run on: takes one call at a time from the sim thread. */
    private final class CallThread extends Thread {

        CallThread(ThreadGroup group) {
            super(group, "SimLoop-OpMode-" + opMode.getClass().getSimpleName());
            setDaemon(true);
        }

        @Override
        public void run() {
            while (true) {
                Runnable work;
                synchronized (lock) {
                    while (pending == null && !exit) {
                        try {
                            lock.wait();
                        } catch (InterruptedException ignored) {
                            // Only the sim hands out calls; an interrupt is not one.
                        }
                    }
                    if (exit) {
                        return;
                    }
                    work = pending;
                    pending = null;
                }
                Throwable failure = null;
                try {
                    work.run();
                } catch (Throwable t) {
                    failure = t;
                }
                synchronized (lock) {
                    thrown = failure;
                    callDone = true;
                    lock.notifyAll();
                }
            }
        }
    }
}
