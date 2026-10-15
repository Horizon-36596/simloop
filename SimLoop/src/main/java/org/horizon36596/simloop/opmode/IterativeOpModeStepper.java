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
 */
final class IterativeOpModeStepper implements OpModeStepper {

    /** The SDK's own force-stop exception, thrown by {@code terminateOpModeNow()}; matched by name. */
    static final String FORCE_STOP = "com.qualcomm.robotcore.eventloop.opmode.OpModeManagerImpl$ForceStopException";

    private final OpMode opMode;
    private final OpModeContext context;
    private final int maxWaits;
    private final CallThread thread;

    /** Guards the handoff between the sim thread and {@link #thread}. */
    private final Object lock = new Object();

    /** The call handed to the OpMode thread and not yet taken, or null. Guarded by {@link #lock}. */
    private Runnable pending;

    /** True once the call last handed over has returned or thrown. Guarded by {@link #lock}. */
    private boolean callDone;

    /** What that call threw, or null. Guarded by {@link #lock}. */
    private Throwable thrown;

    /** True once the OpMode thread must end: the run is over, or the thread was abandoned stuck. */
    private boolean exit;

    /** Set on the OpMode thread by {@code requestOpModeStop()}; read on the sim thread after the call. */
    private volatile boolean askedToStop;

    /** Sim thread only: the OpMode has finished. */
    private boolean ended;

    /** Sim thread only: the OpMode thread got stuck and was left behind. */
    private boolean abandoned;

    IterativeOpModeStepper(OpMode opMode, OpModeContext context, int maxWaits, ThreadGroup group) {
        this.opMode = opMode;
        this.context = context;
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
                call("init()", phase, tick, opMode::init);
                break;
            case INIT_LOOP:
                call("init_loop()", phase, tick, opMode::init_loop);
                break;
            case ACTIVE:
                if (tickInPhase == 0) {
                    context.markStarted();
                    call("start()", phase, tick, opMode::start);
                }
                if (!ended) {
                    call("loop()", phase, tick, opMode::loop);
                }
                break;
            case STOP:
                stop(phase, tick);
                break;
            default:
                throw new IllegalStateException("unknown phase " + phase);
        }
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
        Throwable failure = runOnOpModeThread(method, phase, tick, () -> {
            userCode.run();
            opMode.telemetry.update();
        });
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

    /**
     * Hands {@code work} to the OpMode thread and waits, bounded, for it to finish.
     *
     * @return what it threw, or null
     * @throws OpModeException if it did not finish within {@link #maxWaits} one-millisecond waits
     */
    private Throwable runOnOpModeThread(String method, OpModePhase phase, int tick, Runnable work) {
        if (thread.getState() == Thread.State.NEW) {
            thread.start();
        }
        synchronized (lock) {
            pending = work;
            callDone = false;
            thrown = null;
            lock.notifyAll();
            int waits = 0;
            while (!callDone) {
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
            return thrown;
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
