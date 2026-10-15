package org.horizon36596.simloop.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.HashSet;
import java.util.Set;

/**
 * Steps a {@link LinearOpMode} in lockstep with the sim: its {@code runOpMode()} runs on a thread of its
 * own, but only ever while the sim thread is waiting for it, so the two never run at the same time and every
 * run takes the same path.
 *
 * <h2>Where the OpMode hands control back</h2>
 * A {@code LinearOpMode} is one long method, so a tick has to end somewhere inside it. It ends at the SDK
 * calls a linear OpMode already makes to ask about the match:
 * <ul>
 *   <li><b>A lifecycle check</b> ({@code opModeIsActive()}, {@code isStopRequested()},
 *       {@code opModeInInit()}, {@code isStarted()}) that this tick has already made at the same place in
 *       the code. So one pass through {@code while (opModeIsActive()) { ... }} is one tick, and so is one
 *       pass through {@code while (opModeIsActive() && !isStopRequested())}.</li>
 *   <li><b>{@code waitForStart()}</b>, which holds the OpMode, one tick at a time, until PLAY.</li>
 *   <li><b>{@code sleep(ms)}</b>, which holds it for {@code ceil(ms / tick)} ticks of sim time, so
 *       {@code sleep(1000)} at 50 Hz is exactly 50 ticks, whatever the laptop's speed.</li>
 * </ul>
 * After STOP none of these hold the OpMode any more: each check reports the stop, as on the robot, and the
 * OpMode is expected to return from {@code runOpMode()}.
 *
 * <h2>How, when the SDK's methods are final</h2>
 * {@code opModeIsActive()}, {@code isStopRequested()}, {@code isStarted()}, {@code sleep()} and
 * {@code idle()} are all {@code final} in {@code LinearOpMode}, so none of them can be overridden. What they
 * do call is the current thread: the checks call {@code Thread.currentThread().isInterrupted()}, and when
 * {@code sleep()}'s {@code Thread.sleep} or {@code waitForStart()}'s {@code wait()} is interrupted they call
 * {@code Thread.currentThread().interrupt()}. The OpMode's thread is an {@link OpModeThread}, which overrides
 * exactly those two methods, and it keeps its real interrupt flag set while OpMode code runs so that the
 * SDK's {@code Thread.sleep} and {@code wait()} return at once instead of waiting in real time. Each
 * override looks at the stack to see which SDK method called it; a call from anywhere else is answered as an
 * ordinary thread would answer it, except that {@code isInterrupted()} reports "stopping" rather than the
 * deliberately-set flag.
 *
 * <p>{@code sleep}'s duration is not on the stack, so it is read from the calling class's bytecode by
 * {@link SleepDurationReader}, which handles constant durations and refuses anything else loudly.
 *
 * <p>Waiting is bounded without reading a clock: the sim thread counts one-millisecond waits, the same
 * way {@code ScenarioRunner} bounds its wait for the log writer, so an OpMode that never hands control back
 * fails the run with its own stack trace instead of hanging the build.
 */
final class LinearOpModeStepper implements OpModeStepper {

    private static final String LINEAR_OP_MODE = LinearOpMode.class.getName();

    private final LinearOpMode opMode;
    private final OpModeContext context;
    private final double tickSeconds;
    private final int maxWaits;
    private final SleepDurationReader sleepReader = new SleepDurationReader();
    private final OpModeThread thread;

    /** Guards the handoff: exactly one of the two threads runs while this is not held by a waiter. */
    private final Object lock = new Object();

    /** True while the OpMode thread may run. Guarded by {@link #lock}. */
    private boolean opModeTurn;

    /** True once {@code runOpMode()} has returned or thrown. Guarded by {@link #lock}. */
    private boolean finished;

    /** What {@code runOpMode()} threw, if it was a failure. Guarded by {@link #lock}. */
    private Throwable failure;

    /** PLAY has been pressed. */
    private volatile boolean started;

    /** STOP has been pressed, the OpMode asked to stop, or the run is being torn down. */
    private volatile boolean stopping;

    /**
     * The sim has given up on the OpMode thread (it ran a whole stuck timeout without handing control back),
     * so {@link #shutDown} does not wait a second timeout for a thread that has already shown it will not
     * come back. Sim thread only.
     */
    private boolean gaveUp;

    /** How many one-millisecond waits shutDown() gives an OpMode thread it already gave up on to end. */
    private static final int GAVE_UP_GRACE_WAITS = 50;

    LinearOpModeStepper(LinearOpMode opMode, OpModeContext context, double tickSeconds, int maxWaits,
                        ThreadGroup group) {
        this.opMode = opMode;
        this.context = context;
        this.tickSeconds = tickSeconds;
        this.maxWaits = maxWaits;
        this.thread = new OpModeThread(group);
    }

    /** The OpMode's own code called {@code requestOpModeStop()}: from here on it sees the stop. */
    @Override
    public void opModeAskedToStop() {
        stopping = true;
        context.markStopRequested();
    }

    @Override
    public void runTick(OpModePhase phase, int tickInPhase, int tick) {
        switch (phase) {
            case INIT:
                synchronized (lock) {
                    opModeTurn = true;
                }
                context.beforeUserCode();
                thread.start();
                awaitTurnBack(phase, tick);
                break;
            case INIT_LOOP:
                giveTurn(phase, tick);
                break;
            case ACTIVE:
                if (tickInPhase == 0) {
                    context.markStarted();
                    started = true;
                }
                giveTurn(phase, tick);
                break;
            case STOP:
                stopping = true;
                context.markStopRequested();
                giveTurn(phase, tick);
                break;
            default:
                throw new IllegalStateException("unknown phase " + phase);
        }
    }

    @Override
    public boolean hasEnded() {
        synchronized (lock) {
            return finished;
        }
    }

    @Override
    public Thread thread() {
        return thread;
    }

    @Override
    public void shutDown() {
        // The sim thread can arrive here interrupted: a test framework's timeout, or the interrupt that just
        // failed the tick in progress. With the flag set, every bounded wait below would throw at once and the
        // OpMode thread would be left running past the run. So the flag is set aside for the waits and put
        // back afterwards, for the caller to see.
        boolean callerInterrupted = Thread.interrupted();
        try {
            stopAndJoin();
        } finally {
            if (callerInterrupted) {
                Thread.currentThread().interrupt();
            }
        }
    }

    /** {@link #shutDown}, with the calling thread's interrupt flag clear. */
    private void stopAndJoin() {
        if (thread.getState() == Thread.State.NEW) {
            return;
        }
        synchronized (lock) {
            if (!finished) {
                stopping = true;
                context.markStopRequested();
                opModeTurn = true;
                lock.notifyAll();
                // As the SDK does at STOP: a real interrupt wakes an OpMode blocked in a wait or sleep of the
                // JVM's (one that cleared its armed flag), so it can see the stop and return.
                thread.interrupt();
            }
            if (gaveUp) {
                // The failure already says the thread is left behind, so do not wait a second stuck timeout.
                // The interrupt above can still end it (a sleep or wait of the JVM's), so give that a moment,
                // then abandon it to the runner.
                for (int waits = 0; !finished && waits < GAVE_UP_GRACE_WAITS; waits++) {
                    waitOneMillisecond(OpModePhase.STOP, -1);
                }
                return;
            }
            int waits = 0;
            while (!finished) {
                if (waits++ >= maxWaits) {
                    throw notFinishing("did not return from runOpMode() after it was told to stop",
                            OpModePhase.STOP, -1);
                }
                waitOneMillisecond(OpModePhase.STOP, -1);
            }
        }
        // finished is set as the thread's last act, so this join is over almost at once; it is what makes
        // "no OpMode thread outlives its run" true the moment the run returns, not a moment later.
        try {
            for (int waits = 0; thread.isAlive() && waits < maxWaits; waits++) {
                thread.join(1);
            }
        } catch (InterruptedException interrupted) {
            Thread.currentThread().interrupt();
        }
    }

    /** Lets the OpMode run until it hands control back, then surfaces anything it threw. */
    private void giveTurn(OpModePhase phase, int tick) {
        synchronized (lock) {
            if (finished) {
                return;
            }
        }
        context.beforeUserCode();
        synchronized (lock) {
            opModeTurn = true;
            lock.notifyAll();
        }
        awaitTurnBack(phase, tick);
    }

    private void awaitTurnBack(OpModePhase phase, int tick) {
        synchronized (lock) {
            int waits = 0;
            while (opModeTurn && !finished) {
                if (waits++ >= maxWaits) {
                    throw notFinishing(stopping
                            ? "did not return from runOpMode() after STOP within at least " + maxWaits + " ms of"
                                    + " real time (stuckTimeoutMillis, counted in one-millisecond waits). Sim time"
                                    + " does not move after STOP, so a loop there that waits for a timer or for"
                                    + " sleep() or idle() to pass never ends: put opModeIsActive() in its"
                                    + " condition"
                            : "ran for at least " + maxWaits + " ms of real time (stuckTimeoutMillis, counted in"
                                    + " one-millisecond waits) without reaching a lifecycle check,"
                                    + " sleep(), idle() or waitForStart(), so the sim cannot step it. Sim time"
                                    + " only moves between ticks, so a loop that waits for a timer (ElapsedTime,"
                                    + " System.nanoTime()) with none of those calls inside it is a busy-wait that"
                                    + " never ends: check opModeIsActive() in its condition, or call idle() in"
                                    + " its body", phase, tick);
                }
                waitOneMillisecond(phase, tick);
            }
            if (failure != null) {
                throw new OpModeException(opMode.getClass().getSimpleName() + ".runOpMode() threw during "
                        + phase + " (tick " + tick + "): " + failure + FailureHints.forFailure(failure, true), phase,
                        tick, failure);
            }
        }
        context.afterUserCode();
    }

    /** Waits on {@link #lock}, which the caller holds, for at most one millisecond. */
    private void waitOneMillisecond(OpModePhase phase, int tick) {
        try {
            lock.wait(1);
        } catch (InterruptedException interrupted) {
            Thread.currentThread().interrupt();
            throw new OpModeException("The thread running the sim was interrupted while "
                    + opMode.getClass().getSimpleName() + " had its turn.", phase, tick, interrupted);
        }
    }

    private OpModeException notFinishing(String what, OpModePhase phase, int tick) {
        Throwable whereItIs = new Throwable("where " + thread.getName() + " was when the sim gave up");
        whereItIs.setStackTrace(thread.getStackTrace());
        stopping = true;
        gaveUp = true;
        return new OpModeException(opMode.getClass().getSimpleName() + " " + what + " (tick " + tick + "). Its"
                + " thread is a daemon and is left running; the cause shows where it was.", phase, tick, whereItIs);
    }

    /** {@return how many ticks a wait of {@code seconds} holds the OpMode for: enough to cover it, never fewer} */
    private long ticksFor(double seconds) {
        if (!(seconds > 0)) {
            return 0;
        }
        // The tiny subtraction stops floating-point noise turning an exact 1000 ms / 20 ms into 51 ticks.
        return (long) Math.ceil(seconds / tickSeconds - 1e-9);
    }

    /** {@return the thread {@code runOpMode()} runs on: the only thread whose waits the sim can step} */
    Thread opModeThread() {
        return thread;
    }

    /**
     * On the OpMode's own thread: holds it for enough ticks to cover {@code seconds} of sim time. The SDK's
     * {@code sleep()} comes here with the constant {@link SleepDurationReader} read, and the sim-clock hooks
     * with a duration only the running code knows ({@code Thread.sleep}, or a computed {@code sleep(ms)} in
     * rewritten code).
     *
     * @param seconds how long, in seconds of sim time
     * @return true if the wait ran its full length, false if STOP came before or during it
     */
    boolean holdForSimSeconds(double seconds) {
        long ticks = ticksFor(seconds);
        for (long t = 0; t < ticks && !stopping; t++) {
            thread.park();
        }
        return !stopping;
    }

    /**
     * On the OpMode's own thread, a rewritten {@code idle()} at {@code callSite}: ends the tick only if that
     * {@code idle()} has already run during it, as a repeated lifecycle check does, and never after STOP. The
     * SDK's {@code idle()} takes no time, so a first {@code idle()} after a {@code sleep()} adds no tick, and
     * {@code while (timer.seconds() < 2) idle();} still moves one tick a pass.
     *
     * @param callSite the team's frame that called {@code idle()}
     */
    void idleAt(String callSite) {
        String key = "idle@" + callSite;
        if (!stopping && !thread.checksThisTick.add(key)) {
            thread.park();
            thread.checksThisTick.add(key); // made in the new tick, as a lifecycle check that parked is
        }
    }

    /**
     * The SDK method a hook was called from: {@code direct} called the hook, {@code entry} is the
     * {@code LinearOpMode} method the OpMode's code called (they differ when, say, {@code opModeIsActive()}
     * calls {@code isStopRequested()}), and {@code caller} is the OpMode frame that called {@code entry}.
     */
    private static final class LifecycleCall {
        final String direct;
        final String entry;
        final StackTraceElement caller;

        private LifecycleCall(String direct, String entry, StackTraceElement caller) {
            this.direct = direct;
            this.entry = entry;
            this.caller = caller;
        }

        /** {@return the call, or null if the hook was not called by {@code LinearOpMode} itself} */
        static LifecycleCall find(StackTraceElement[] stack) {
            int i = 0;
            while (i < stack.length && stack[i].getClassName().startsWith(LinearOpModeStepper.class.getName())) {
                i++;
            }
            if (i >= stack.length || !LINEAR_OP_MODE.equals(stack[i].getClassName())) {
                return null;
            }
            String direct = stack[i].getMethodName();
            while (i + 1 < stack.length && LINEAR_OP_MODE.equals(stack[i + 1].getClassName())) {
                i++;
            }
            StackTraceElement caller = i + 1 < stack.length ? stack[i + 1] : stack[i];
            return new LifecycleCall(direct, stack[i].getMethodName(), caller);
        }

        /** Identifies this check at this place in the OpMode's code. */
        String key() {
            return entry + "/" + direct + "@" + caller;
        }
    }

    /**
     * The thread {@code runOpMode()} runs on. See the class javadoc for why it overrides
     * {@link #isInterrupted()} and {@link #interrupt()}, and why its interrupt flag is kept set.
     */
    private final class OpModeThread extends Thread {

        /** Lifecycle checks made since the OpMode last got its turn. OpMode thread only. */
        private final Set<String> checksThisTick = new HashSet<>();

        OpModeThread(ThreadGroup group) {
            super(group, "SimLoop-LinearOpMode-" + opMode.getClass().getSimpleName());
            setDaemon(true);
        }

        @Override
        public void run() {
            super.interrupt(); // arm: the SDK's Thread.sleep and wait() now return at once, into the hooks
            Throwable thrown = null;
            try {
                opMode.runOpMode();
            } catch (Throwable t) {
                thrown = t;
            }
            synchronized (lock) {
                if (thrown != null && !isNormalEnd(thrown)) {
                    failure = thrown;
                }
                finished = true;
                opModeTurn = false;
                lock.notifyAll();
            }
        }

        private boolean isNormalEnd(Throwable thrown) {
            return IterativeOpModeStepper.FORCE_STOP.equals(thrown.getClass().getName())
                    || (thrown instanceof InterruptedException && stopping);
        }

        /** Lifecycle checks end up here; see the class javadoc. */
        @Override
        public boolean isInterrupted() {
            if (Thread.currentThread() != this) {
                return super.isInterrupted();
            }
            LifecycleCall call = LifecycleCall.find(new Throwable().getStackTrace());
            if (call != null) {
                // Re-arm, in case the OpMode's own code cleared the flag (it caught the InterruptedException of
                // a Thread.sleep of its own): waitForStart() calls isStarted() just before its wait(), which
                // would otherwise block for real, with nothing left to wake it.
                super.interrupt();
            }
            if (call != null && !stopping && !checksThisTick.add(call.key())) {
                park();
                checksThisTick.add(call.key());
                if ("isStarted".equals(call.direct)) {
                    // isStarted() read its field before calling here, and PLAY may have been pressed while
                    // it was parked; answering for the field keeps opModeInInit() from one stale extra pass.
                    return stopping || started;
                }
            }
            return stopping;
        }

        /** {@code sleep()} and {@code waitForStart()} end up here; see the class javadoc. */
        @Override
        public void interrupt() {
            if (Thread.currentThread() != this) {
                super.interrupt();
                return;
            }
            try {
                LifecycleCall call = LifecycleCall.find(new Throwable().getStackTrace());
                if (call == null || stopping) {
                    return;
                }
                if ("sleep".equals(call.direct)) {
                    holdForSimSeconds(sleepReader.millis(call.caller, opMode.getClass().getClassLoader()) / 1000.0);
                } else if ("waitForStart".equals(call.direct)) {
                    while (!started && !stopping) {
                        park();
                    }
                }
            } finally {
                super.interrupt(); // re-arm, whatever happened
            }
        }

        /** Ends this tick's turn and blocks until the sim gives the OpMode the next one. */
        private void park() {
            if (stopping) {
                return;
            }
            Thread.interrupted(); // clear the armed flag, or wait() below would return at once
            try {
                synchronized (lock) {
                    opModeTurn = false;
                    lock.notifyAll();
                    while (!opModeTurn) {
                        try {
                            lock.wait();
                        } catch (InterruptedException ignored) {
                            // Only the sim hands out turns; an interrupt is not one.
                        }
                    }
                }
            } finally {
                checksThisTick.clear();
                super.interrupt();
            }
        }
    }
}
