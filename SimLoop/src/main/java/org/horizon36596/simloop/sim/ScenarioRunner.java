package org.horizon36596.simloop.sim;

import org.horizon36596.simloop.config.SimRobotConfig;

import org.psilynx.psikit.core.LogDataReceiver;
import org.psilynx.psikit.core.LogTable;
import org.psilynx.psikit.core.Logger;
import org.psilynx.psikit.core.rlog.RLOGWriter;

import java.io.IOException;
import java.io.UncheckedIOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.Collections;
import java.util.List;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.locks.ReentrantLock;

/**
 * Runs one headless sim scenario end-to-end: owns the entire PsiKit lifecycle (sim-harness §2-3) and the
 * fixed-timestep tick loop, so a test does not hand-write it (BACKLOG "scenario runner" / area B). Every
 * sim test up to this point (Batch 1.4's {@code DrivetrainSimTest}, Batch 2.2's {@code SlideSimTest})
 * re-typed the same ~20 lines: create the RLOG directory, reset/wire/start PsiKit's {@link Logger}, tick a
 * {@link FakeTimer} inside a {@code periodicBeforeUser()}/{@code periodicAfterUser()} bracket, call
 * {@link Logger#end()}, then re-check by hand that the writer thread didn't overflow or emit nothing. This
 * class is that boilerplate, written once.
 *
 * <p><b>What stays with the caller.</b> This class is season-agnostic core (architecture §3, domain R7): it
 * knows nothing about a specific robot's motors, plants, or subsystems. Building the fake hardware,
 * commanding the robot, advancing its plants, and deciding what to log each tick is scenario-specific
 * behavior the caller supplies as a {@link ScenarioBody}. The {@link SimRobotConfig} parameter is not read by this
 * class at all — it is recorded as PsiKit metadata purely so a human opening the RLOG in AdvantageScope (or
 * a future Phase-3 scorer) can see which robot config a run used, the same way {@code opModeName} is.
 *
 * <p><b>Trustworthy by construction.</b> {@link #run} throws if PsiKit's async writer queue overflowed
 * (BACKLOG B3 — cycles silently dropped) or if the resulting RLOG is missing or empty (BACKLOG B11) — a
 * scenario whose log cannot be trusted for replay is a defect the moment it happens, not something every
 * caller should have to re-check after the fact.
 *
 * <p><b>One logged run at a time per JVM.</b> PsiKit's {@link Logger} is one static object, so two runs
 * logging at once would write into each other's RLOG, and a run could return normally with a log that is
 * mostly the other's. {@link #run} therefore takes a JVM-wide lock: a second run started on another thread
 * waits for the first to finish, and one started from inside a run's own tick fails at once. Runs also share
 * {@code OpMode.blackboard}, which {@code OpModeRunner} clears at the start of each run, so tests that run
 * OpModes must not run in parallel within one JVM; Gradle's separate test JVMs are fine.
 */
public final class ScenarioRunner {

    /** Held for the whole of every {@link #run} and {@link #runUntil}: PsiKit's Logger can record only one run at a time. */
    private static final ReentrantLock ONE_LOGGED_RUN_AT_A_TIME = new ReentrantLock();

    private ScenarioRunner() {
    }

    /** One scenario's per-tick behavior: command the robot, advance its fake hardware/plants, log fields. */
    @FunctionalInterface
    public interface ScenarioBody {
        /**
         * Called once per sim tick, after {@link FakeTimer#advance(double)} and
         * {@link Logger#periodicBeforeUser()} and before {@link Logger#periodicAfterUser(long, long)}.
         *
         * @param deltaTimeSeconds the fixed tick step just advanced (same value passed to {@link #run})
         */
        void tick(double deltaTimeSeconds);
    }

    /**
     * Decides, before each tick, whether that tick runs. A fixed-length scenario is
     * {@link #fixedTickCount}; a live or replayed session runs until its input runs out.
     *
     * <p><b>Called exactly once per tick, in order</b>: with 0, then 1, then 2, and so on, each time before
     * that tick advances the clock, and never again after it has returned false. A gate may therefore do
     * the one-per-tick work of fetching that tick's input, as {@code sim.live.GamepadInputGate} does.
     */
    @FunctionalInterface
    public interface TickGate {
        /**
         * Called before tick {@code tickIndex} advances the clock or logs anything.
         *
         * @param tickIndex the tick about to run, counting from 0
         * @return true to run it; false to end the run here, with {@code tickIndex} ticks run
         */
        boolean shouldRunTick(int tickIndex);
    }

    /**
     * Runs after each tick has been logged, and may make the run wait. This is the one seam where a run can
     * be slowed to real time for a person watching it (the live-drive pacer). It is handed the sim time for
     * information only: nothing it does can reach the sim, because the next tick advances the clock by the
     * same fixed step whatever happened here. Every headless run uses {@link #NO_PACING}.
     */
    @FunctionalInterface
    public interface TickPacer {
        /**
         * Called once after each tick is logged; may wait before returning.
         *
         * @param ticksCompleted how many ticks have run so far, this one included
         * @param simSeconds     the sim clock after this tick, seconds
         */
        void afterTick(int ticksCompleted, double simSeconds);
    }

    /** No pacing at all: run as fast as the machine allows. What every headless scenario and replay uses. */
    public static final TickPacer NO_PACING = new TickPacer() {
        @Override
        public void afterTick(int ticksCompleted, double simSeconds) {
        }
    };

    /**
     * A gate that runs exactly {@code ticks} ticks, which is what {@link #run} uses.
     *
     * @param ticks how many ticks to run; must not be negative
     * @return the gate
     */
    public static TickGate fixedTickCount(final int ticks) {
        if (ticks < 0) {
            throw new IllegalArgumentException("ticks must not be negative, got " + ticks);
        }
        return new TickGate() {
            @Override
            public boolean shouldRunTick(int tickIndex) {
                return tickIndex < ticks;
            }
        };
    }

    /**
     * Runs {@code ticks} fixed-timestep sim ticks against {@code timer}, writing everything PsiKit logs to
     * {@code rlogPath}.
     *
     * @param opModeName       recorded as PsiKit metadata ("OpMode"), shown in AdvantageScope
     * @param config           the robot's sim config for this scenario; recorded as metadata only (see
     *                         class javadoc) — this class does not read a robot's motors/plants from it
     * @param timer            the deterministic clock (domain R5) this scenario runs on; the caller
     *                         constructs it (and must wire any sim-side infra to it, e.g.
     *                         {@code HeadlessRobotInitInfra}) BEFORE calling this method, since that wiring
     *                         has to happen before the robot under test is built
     * @param ticks            number of sim ticks to run
     * @param deltaTimeSeconds fixed per-tick time step (sim-harness §3); {@code timer} advances by this
     *                         once per tick, before {@code body.tick(deltaTimeSeconds)} runs
     * @param rlogPath         caller-chosen output file, e.g. {@code Paths.get("build", "sim", "run1.rlog")}
     * @param body             the scenario's per-tick behavior (see {@link ScenarioBody})
     * @throws IllegalArgumentException if any argument is unusable, checked before anything else happens:
     *                                  {@code ticks} not positive; {@code deltaTimeSeconds} not a finite
     *                                  number greater than zero; {@code config}, {@code timer},
     *                                  {@code rlogPath} or {@code body} null; or {@code rlogPath} not ending
     *                                  in {@code .rlog}
     * @throws IllegalStateException if this is called from inside another run's {@code body} on the same thread
     *                                (a run on another thread waits instead); if an old file at {@code rlogPath}
     *                                cannot be deleted; or if the RLOG this run produced cannot be trusted,
     *                                for any of four reasons: PsiKit's async writer queue overflowed
     *                                (BACKLOG B3); the file is missing or empty (BACKLOG B11); it decodes
     *                                to a different number of frames than ticks were run (BACKLOG B32); or
     *                                the writer thread stopped making progress while the producer waited
     *                                for it. All four are the same statement - the log is not a record of
     *                                the run - and none of them is affected by what the caller's own
     *                                trajectory checks found
     */
    public static void run(String opModeName, SimRobotConfig config, FakeTimer timer, int ticks,
            double deltaTimeSeconds, Path rlogPath, ScenarioBody body) {
        // ticks is the one argument runUntil does not see (it gets a gate instead), so it is checked here.
        if (ticks <= 0) {
            throw new IllegalArgumentException("ScenarioRunner: ticks must be at least 1, got " + ticks);
        }
        runUntil(opModeName, config, timer, deltaTimeSeconds, rlogPath, fixedTickCount(ticks), NO_PACING,
                Collections.<LogDataReceiver>emptyList(), body);
    }

    /**
     * Like {@link #run}, for a run whose length is not known in advance: a live session that ends when the
     * driver stops, or the replay of one. Everything {@link #run} guarantees about the RLOG holds here too,
     * measured against the number of ticks that actually ran. {@link #run} itself comes through here, so the
     * argument checks, the one-run-at-a-time guard and the delete of an old RLOG apply to both.
     *
     * <p><b>Two different "no ticks" refusals.</b> A bad argument is refused up front, before PsiKit or the
     * file system is touched, with {@link IllegalArgumentException} - for {@link #run}, that includes a
     * {@code ticks} below 1. A gate that says "stop" before tick 0 is only discovered once the run has
     * started, and is refused afterwards with {@link IllegalStateException}: the arguments were fine, but
     * the run produced no record of anything.
     *
     * @param opModeName       recorded as PsiKit metadata ("OpMode")
     * @param config           recorded as metadata only, as in {@link #run}
     * @param timer            the deterministic clock, wired to any sim-side infra before this is called
     * @param deltaTimeSeconds fixed per-tick step; {@code timer} advances by this once per tick
     * @param rlogPath         where the RLOG goes; must end in {@code .rlog}. A file already there is deleted
     * @param gate             asked before every tick whether it runs; the first "no" ends the run
     * @param pacer            called after every tick; {@link #NO_PACING} for anything headless
     * @param extraReceivers   PsiKit receivers to add beside the RLOG file, for example an
     *                         {@code RLOGServer} so AdvantageScope can watch a live session; empty for a
     *                         headless run. They see every frame the file does.
     * @param body             the scenario's per-tick behaviour
     * @return how many ticks ran, at least 1
     * @throws IllegalArgumentException if any argument is unusable, checked before anything else happens:
     *                                  any of them null; {@code deltaTimeSeconds} not a finite number
     *                                  greater than zero; or {@code rlogPath} not ending in {@code .rlog}
     * @throws IllegalStateException    if this is called from inside another run's {@code body} on the same
     *                                  thread (a run on another thread waits instead), or is interrupted
     *                                  while it waits; if an old file at {@code rlogPath} cannot be
     *                                  deleted; for the same four untrustworthy-RLOG reasons as
     *                                  {@link #run}; and when the gate ends
     *                                  the run before its first tick, because a run of nothing is not a
     *                                  record of anything
     */
    public static int runUntil(String opModeName, SimRobotConfig config, FakeTimer timer,
            double deltaTimeSeconds, Path rlogPath, TickGate gate, TickPacer pacer,
            List<? extends LogDataReceiver> extraReceivers, ScenarioBody body) {
        requireUsableArguments(config, timer, deltaTimeSeconds, rlogPath, gate, pacer, extraReceivers, body);

        // Two protections, for two different mistakes. The lock serialises runs from DIFFERENT threads (two
        // tests, or an OpMode's own thread), which would otherwise reset PsiKit's static Logger under each
        // other. The held-by-me check refuses a run started from inside THIS thread's own tick: the lock is
        // reentrant, so without the check that nested run would walk straight in and reset the Logger under
        // the outer run. The check comes first and never blocks, so re-entry cannot deadlock on the lock.
        if (ONE_LOGGED_RUN_AT_A_TIME.isHeldByCurrentThread()) {
            throw new IllegalStateException("ScenarioRunner: '" + opModeName + "' was started from inside another"
                    + " logged run's tick. PsiKit's Logger records one run at a time per JVM, so the two would"
                    + " write into one RLOG; run them one after the other.");
        }
        try {
            // Interruptibly: a run started from an OpMode's thread while the sim thread holds the lock would
            // otherwise wait for ever, as the sim thread waits for the OpMode. SimLoop keeps a LinearOpMode
            // thread's interrupt flag set, and STOP interrupts it, so such a run ends here instead.
            ONE_LOGGED_RUN_AT_A_TIME.lockInterruptibly();
        } catch (InterruptedException interrupted) {
            Thread.currentThread().interrupt();
            throw new IllegalStateException("ScenarioRunner: '" + opModeName + "' was interrupted while it waited"
                    + " for another logged run to finish. Start logged runs from the test's own thread, one after"
                    + " the other, not from an OpMode or a thread one started.", interrupted);
        }
        try {
            return runOnce(opModeName, config, timer, deltaTimeSeconds, rlogPath, gate, pacer, extraReceivers,
                    body);
        } finally {
            ONE_LOGGED_RUN_AT_A_TIME.unlock();
        }
    }

    /**
     * Rejects every argument {@link #runUntil} cannot use, before PsiKit is touched, so a bad call leaves the
     * logger, the clock and the file system exactly as they were.
     */
    private static void requireUsableArguments(SimRobotConfig config, FakeTimer timer, double deltaTimeSeconds,
            Path rlogPath, TickGate gate, TickPacer pacer, List<? extends LogDataReceiver> extraReceivers,
            ScenarioBody body) {
        if (config == null || timer == null || rlogPath == null || body == null) {
            throw new IllegalArgumentException("ScenarioRunner: config, timer, rlogPath and body must all be "
                    + "non-null (config=" + config + ", timer=" + timer + ", rlogPath=" + rlogPath
                    + ", body=" + body + ")");
        }
        if (gate == null || pacer == null || extraReceivers == null) {
            throw new IllegalArgumentException("ScenarioRunner: gate, pacer and extraReceivers must all be "
                    + "non-null; use fixedTickCount(n), NO_PACING and an empty list for a plain headless run");
        }
        if (!(deltaTimeSeconds > 0.0) || Double.isInfinite(deltaTimeSeconds)) {
            throw new IllegalArgumentException("ScenarioRunner: deltaTimeSeconds must be a finite number of "
                    + "seconds greater than zero, got " + deltaTimeSeconds);
        }
        // PsiKit's RLOGWriter appends ".rlog" to any name that lacks it, so "run1.log" would be written to
        // "run1.log.rlog" and every check after the run would look at a file that was never written.
        if (!rlogPath.getFileName().toString().endsWith(".rlog")) {
            throw new IllegalArgumentException("ScenarioRunner: rlogPath must end in .rlog, got " + rlogPath
                    + " (PsiKit would silently write to " + rlogPath.getFileName() + ".rlog instead)");
        }
    }

    /** The body of {@link #runUntil}, once its arguments are checked and the one-run guard is held. */
    private static int runOnce(String opModeName, SimRobotConfig config, FakeTimer timer,
            double deltaTimeSeconds, Path rlogPath, TickGate gate, TickPacer pacer,
            List<? extends LogDataReceiver> extraReceivers, ScenarioBody body) {
        Path dir = rlogPath.toAbsolutePath().getParent();
        String fileName = rlogPath.getFileName().toString();
        String baseName = fileName.substring(0, fileName.length() - ".rlog".length());

        try {
            Files.createDirectories(dir);
        } catch (IOException e) {
            throw new UncheckedIOException("ScenarioRunner: could not create RLOG directory " + dir, e);
        }

        // Reset PsiKit BEFORE the delete: Logger.reset() begins with end(), which closes any receiver still
        // attached from an earlier logging session - including a writer holding this very path open. Deleting
        // first could refuse a run whose handle the reset was about to release.
        Logger.reset();

        // Delete the previous run's file OURSELVES, and fail if that is impossible. PsiKit's
        // RLOGWriter.start() also deletes it, but ignores the result and then opens the file in APPEND mode
        // (checked by disassembling the pinned core-0.2.0-beta2 jar). On Windows a file that some stream
        // still holds open cannot be deleted, so a failed delete there used to mean this run's frames were
        // written after the old run's, and the file decoded to the wrong number of frames.
        try {
            Files.deleteIfExists(rlogPath);
        } catch (IOException e) {
            throw new IllegalStateException("ScenarioRunner: could not delete the old RLOG at " + rlogPath
                    + " before '" + opModeName + "' -- something still holds it open. PsiKit would append this"
                    + " run to the old file instead of replacing it, so the run is refused.", e);
        }

        Logger.setTimeSource(timer::time);
        Logger.recordMetadata("OpMode", opModeName);
        Logger.recordMetadata("SimRobotConfig", configName(config));
        // Two-arg ctor: the single-arg form prepends the Android path /sdcard/FIRST/PsiKit/ (jar behaviour);
        // (dir, name) keeps the RLOG in the JVM build dir for AdvantageScope replay.
        Logger.addDataReceiver(new RLOGWriter(dir.toString(), baseName));
        // Extra receivers go between the writer and the counter below, so a frame counts as written only once
        // they have had it too. A slow one (a live RLOGServer with a sluggish client) therefore slows the
        // producer down rather than letting PsiKit's queue overflow and drop frames - the right way round.
        for (LogDataReceiver extra : extraReceivers) {
            Logger.addDataReceiver(extra);
        }

        // Added AFTER the writer, so a frame is only counted once the writer has finished with it. That
        // ordering is what makes the count usable as backpressure: see WrittenFrameCounter.
        WrittenFrameCounter written = new WrittenFrameCounter();
        Logger.addDataReceiver(written);

        Logger.start();

        int ticks = 0;
        int firstDroppedTick = -1;
        // The tick loop is wrapped so that PsiKit is shut down on EVERY exit path, not only the happy one.
        // Logger is global static state holding an open RLOGWriter file handle; a throw out of the scenario
        // body or out of the pacer used to leave that handle open until some later Logger.reset() happened
        // to call end() for us. See the catch below for why the original exception is the one that escapes.
        try {
            for (int i = 0; gate.shouldRunTick(i); i++) {
                timer.advance(deltaTimeSeconds);
                Logger.periodicBeforeUser();
                body.tick(deltaTimeSeconds);
                Logger.periodicAfterUser(0, 0);

                // Latch the writer fault OURSELVES, every tick, because PsiKit's flag does not latch: every
                // successful enqueue resets it to false. So a queue that overflows in the middle of a run and
                // then recovers reports "no fault" at the end while the frames it refused are gone for good.
                // Reading the flag once after the loop - which is what this used to do - therefore only ever
                // catches a run whose very LAST tick also overflowed. See BACKLOG B32 for the disassembly.
                if (firstDroppedTick < 0 && Logger.getReceiverQueueFault()) {
                    firstDroppedTick = i;
                }

                paceProducerToWriter(i + 1, written);
                ticks = i + 1;
                pacer.afterTick(ticks, timer.time());
            }
        } catch (RuntimeException | Error failureDuringTheRun) {
            // Close the writer before the exception leaves, and let the ORIGINAL failure be the one the
            // caller sees: a shutdown that fails on top of an already-failing run is a consequence, not the
            // cause, and replacing the first exception with the second is how a real bug gets misdiagnosed.
            // The shutdown failure is still readable, as a suppressed exception on the original.
            //
            // One case this CANNOT fix, and it is worth knowing about: if PsiKit's receiver thread has
            // already died (an unchecked throw out of RLOGWriter.putTable kills it - its only handler
            // rethrows), then Logger.end()'s join returns at once and nothing is left alive to call the
            // writer's end(). The file handle stays open until the JVM exits. There is no PsiKit API that
            // reports a dead receiver thread, so this is recorded rather than handled (BACKLOG B34).
            try {
                shutDownPsiKit();
            } catch (RuntimeException | Error alsoFailedToShutDown) {
                failureDuringTheRun.addSuppressed(alsoFailedToShutDown);
            }
            throw failureDuringTheRun;
        }

        shutDownPsiKit();

        if (ticks == 0) {
            throw new IllegalStateException("ScenarioRunner: '" + opModeName + "' ran no ticks -- the gate said"
                    + " stop before tick 0 ran (the arguments were fine; the input ended at once), so " + rlogPath
                    + " is not a record of anything.");
        }
        if (firstDroppedTick >= 0) {
            throw new IllegalStateException("ScenarioRunner: PsiKit's async RLOG-writer queue overflowed "
                    + "during '" + opModeName + "', first at tick " + firstDroppedTick + " of " + ticks
                    + " -- cycles were silently dropped, so " + rlogPath + " is not trustworthy regardless"
                    + " of what it decodes to (BACKLOG B3, B32).");
        }
        long size;
        try {
            size = Files.exists(rlogPath) ? Files.size(rlogPath) : 0L;
        } catch (IOException e) {
            throw new UncheckedIOException("ScenarioRunner: could not read RLOG size at " + rlogPath, e);
        }
        if (size == 0) {
            throw new IllegalStateException("ScenarioRunner: '" + opModeName + "' wrote no bytes to " + rlogPath
                    + " -- PsiKit's async writer thread emitted nothing (BACKLOG B11).");
        }

        requireOneDecodedFramePerTick(opModeName, rlogPath, ticks);
        return ticks;
    }

    /**
     * Ends this run's logging session and hands PsiKit back as it was before the run: its own clock, no
     * receivers, no metadata.
     *
     * <p>{@code Logger.end()} alone is not enough. Disassembled from the pinned {@code core-0.2.0-beta2} jar,
     * it joins the writer thread, then builds a new receiver thread and re-adds every receiver to it, and it
     * never touches the time source. So after a run that only ended, the next caller in the same JVM that
     * starts PsiKit without its own {@code Logger.reset()} would find this run's {@code FakeTimer}, now
     * frozen, still installed as PsiKit's clock (every frame stamped the same time, and its log decoding as
     * one frame), and this run's writer still attached (restarting it deletes and rewrites this run's RLOG).
     * {@code Logger.reset()} begins with {@code end()} and then restores the default clock and a fresh,
     * empty receiver list, so calling it here covers both.
     */
    private static void shutDownPsiKit() {
        Logger.reset();
    }

    /**
     * The name recorded as the "SimRobotConfig" metadata: the class's simple name, or its full name when
     * the simple name is empty (an anonymous class, such as a test stub, has no simple name).
     */
    private static String configName(SimRobotConfig config) {
        String simpleName = config.getClass().getSimpleName();
        return simpleName.isEmpty() ? config.getClass().getName() : simpleName;
    }

    /**
     * How far the producer may get ahead of the writer before it waits, in frames.
     *
     * <p>PsiKit's receiver queue holds 500 and refuses anything past that, so the budget has to sit
     * comfortably under 500 rather than at it: the check below happens once per tick, and the writer can
     * lose ground between two checks. 300 leaves room for that without making the producer wait often.
     */
    private static final int MAX_FRAMES_AHEAD_OF_WRITER = 300;

    /**
     * How many one-millisecond waits the producer will spend on a single frame before it gives up, and
     * declares the writer stalled rather than waiting forever.
     *
     * <p>Counting sleeps rather than reading an elapsed time keeps this method clock-free (domain R5). A
     * {@code sleep(1)} lasts at least a millisecond and usually a little more, so 30,000 of them is at least
     * thirty seconds and probably rather more - far beyond any healthy writer, and short enough that a build
     * fails with a sentence explaining itself instead of hanging until somebody kills it.
     */
    private static final int MAX_WAITS_FOR_ONE_FRAME = 30_000;

    /**
     * Waits, if the writer has fallen too far behind, until it has caught up.
     *
     * <p><b>Why a simulator needs this and the real robot does not.</b> PsiKit's receiver queue holds 500
     * frames and its capacity is a hard-coded {@code private static final} with no setter. On the robot the
     * producer is a 50 Hz control loop, so the writer gets 20 ms per frame and the queue never fills. Here
     * the producer is a {@code for} loop with no clock in it at all: a tick costs whatever the scenario body
     * costs and nothing more, while the writer has to encode each frame and put it on disk. The producer
     * wins that race outright - not because it is scheduled unfairly, but because it is doing less work - so
     * the queue fills, {@code BlockingQueue.add} throws, and PsiKit drops frames. That is how a 1500-tick
     * run comes back holding 1312 of them.
     *
     * <p><b>Why this measures rather than guesses.</b> {@link Thread#yield()} was tried first and is not
     * enough; nor is any fixed sleep, which is either too slow for every run or too small for the worst one.
     * The queue's own depth is not reachable through PsiKit's API - but the receiver list is, so
     * {@link WrittenFrameCounter} is registered behind the writer and counts frames the writer has actually
     * finished. Ticks produced minus frames written is the depth.
     *
     * <p><b>Exactly the depth, until something is dropped.</b> A frame PsiKit refuses is never handed to any
     * receiver, so the counter can never account for it and the difference stays permanently inflated by the
     * number lost. The figure is therefore exact while the queue is healthy and an over-estimate once it is
     * not - which is the safe direction to be wrong in, since an over-estimate only makes the producer wait
     * more. It does mean this loop must not be the only thing standing between a stalled writer and a hung
     * build, which is what {@link #MAX_WAITS_FOR_ONE_FRAME} is for.
     *
     * <p><b>This does not affect determinism (domain R5).</b> Nothing in the sim reads a wall clock - sim
     * time is {@link FakeTimer}, advanced by a fixed step by the loop above - so how long a tick takes in
     * real seconds changes nothing about what is computed or logged. It changes only whether the frame
     * survives the trip to disk. The waiting is bounded by a count of sleeps rather than by an elapsed-time
     * reading, so this method does not read a clock either.
     *
     * @param ticksProduced how many ticks have been handed to the logger so far
     * @param written       the counter sitting behind the RLOG writer
     * @throws IllegalStateException if the writer stops making progress, or if the thread running the
     *                               scenario is interrupted while waiting - in both cases the log this run
     *                               would produce cannot be trusted, and hanging or quietly carrying on
     *                               unpaced are both worse than saying so
     */
    private static void paceProducerToWriter(int ticksProduced, WrittenFrameCounter written) {
        int waits = 0;
        while (ticksProduced - written.count() > MAX_FRAMES_AHEAD_OF_WRITER) {
            if (waits++ >= MAX_WAITS_FOR_ONE_FRAME) {
                throw new IllegalStateException("ScenarioRunner: the RLOG writer stopped making progress at "
                        + "tick " + ticksProduced + " -- it has finished " + written.count() + " frames and "
                        + "has not finished another in " + MAX_WAITS_FOR_ONE_FRAME + " waits. Something is "
                        + "wrong with the writer thread or the disk; the run is abandoned rather than hung.");
            }
            try {
                Thread.sleep(1);
            } catch (InterruptedException interrupted) {
                // Restore the flag for whoever is unwinding us, then stop. Returning instead would be worse
                // than it looks: the flag stays set, so EVERY later Thread.sleep in this run throws at once
                // and the pacing is silently off for the rest of the run - the exact failure this class was
                // written to remove, reintroduced by the error handling meant to tolerate it.
                Thread.currentThread().interrupt();
                throw new IllegalStateException("ScenarioRunner: interrupted at tick " + ticksProduced
                        + " while waiting for the RLOG writer to catch up; the run cannot be completed.",
                        interrupted);
            }
        }
    }

    /**
     * A {@link LogDataReceiver} that writes nothing and only counts.
     *
     * <p>It exists so the producer can see how far behind the RLOG writer is. PsiKit does not expose its
     * receiver queue's depth, but it does let a caller add receivers, and it hands every frame to each
     * receiver in turn - so a counter registered AFTER the writer increments only once the writer has
     * finished with that frame. Ticks produced minus this count is the queue depth.
     *
     * <p><b>That last sentence rests on PsiKit internals, so it was checked rather than assumed</b> (the
     * standing rule after BACKLOG B17). Disassembled from the pinned {@code core-0.2.0-beta2} jar:
     * {@code Logger} holds ONE static {@code receiverQueue}, and hands it to a single {@code ReceiverThread};
     * that thread holds the receivers in a {@code java.util.List} and walks it by index, calling
     * {@code putTable} on each in registration order, synchronously, on itself. There is no per-receiver
     * queue and no parallel dispatch, so a no-op counter cannot race ahead of the writer it sits behind.
     * {@code Logger.reset()} - which {@link #run} calls before wiring anything - constructs a brand-new
     * {@code ReceiverThread} with an empty receiver list and clears the queue, so receivers from an earlier
     * run in the same JVM cannot still be attached and inflate this count.
     *
     * <p>All of that is internal to a pinned version and none of it is a published contract. If PsiKit is
     * ever bumped, re-check it: if dispatch became parallel or per-receiver, this counter would drain
     * instantly, the pacing would measure nothing, and the truncation would come back quietly. The
     * completeness check in {@link #run} is the backstop that would still catch it.
     *
     * <p>The count is read from the producer thread and written from the receiver thread, which is why it
     * is an {@code AtomicInteger} rather than an {@code int}.
     */
    private static final class WrittenFrameCounter implements LogDataReceiver {
        private final AtomicInteger framesWritten = new AtomicInteger();

        @Override
        public void putTable(LogTable table) {
            framesWritten.incrementAndGet();
        }

        /** Frames the writer ahead of this one has finished. */
        int count() {
            return framesWritten.get();
        }
    }

    /**
     * Decodes the file that was just written and requires exactly one frame per tick.
     *
     * <p><b>Why the two guards above are not enough.</b> They check that the writer did not report a fault
     * and that it produced some bytes. Neither one says the log is COMPLETE, and an incomplete log is the
     * failure that actually costs something: a run can finish, pass both guards, and still decode to fewer
     * frames than it ran ticks, with the missing frames always at the tail. Measured in a season repo at
     * roughly one full suite run in three - 1484 of 1500 on one scenario, 596 of 600 on another - with
     * {@code Logger.getReceiverQueueFault()} false both times (BACKLOG B32).
     *
     * <p><b>Why this is worth a decode on every run.</b> Everything downstream treats an RLOG as the record
     * of what happened. {@code RunResult.isComplete()} is exact equality on frame count, so a truncated log
     * makes the loop's {@code runIsComplete} guardrail VIOLATE for a control law that did nothing wrong -
     * and the breakdown a human reads is then indistinguishable from a real regression. A scenario scored
     * on the ticks it happened to survive is also the cheapest way there is to win any loop exam. Turning a
     * silently wrong answer into a loud failure is worth more than the milliseconds the decode costs.
     *
     * <p><b>This does not fix the truncation, and is not meant to.</b> The cause is not established - see
     * B32, which has the disassembly of PsiKit's {@code Logger.end()} and {@code ReceiverThread.run()}
     * showing that both the normal and the interrupted path do drain the queue and do close the receivers,
     * so the obvious explanation is ruled out and the real one is still open. What this guarantees is
     * narrower and is the part callers depend on: if {@code run} returns, the file at {@code rlogPath} has
     * every tick in it.
     */
    private static void requireOneDecodedFramePerTick(String opModeName, Path rlogPath, int ticks) {
        int decodedFrames;
        try {
            decodedFrames = RlogDecodedCompare.decode(rlogPath).size();
        } catch (IOException | IllegalArgumentException e) {
            // Both of decode's failure modes: an unreadable file (IOException) and a missing or empty one
            // (IllegalArgumentException). Either way the log cannot be shown to be complete, which is the
            // same "not a record of the run" statement as every other check here, so the same exception.
            throw new IllegalStateException("ScenarioRunner: '" + opModeName + "' wrote " + rlogPath
                    + " but it could not be decoded back, so there is no way to tell whether the run is"
                    + " complete", e);
        }
        if (decodedFrames != ticks) {
            throw new IllegalStateException("ScenarioRunner: '" + opModeName + "' ran " + ticks
                    + " ticks but " + rlogPath + " decodes to " + decodedFrames + " frames"
                    + (decodedFrames < ticks ? " -- the log is truncated" : " -- the log has extra frames")
                    + ". PsiKit reported no writer fault, so this is BACKLOG B32. The run is NOT scoreable:"
                    + " anything measured from this file is measured over the ticks that survived, not the"
                    + " ticks that ran. Re-run it; it is deterministic, so a clean run is the same run.");
        }
    }
}
