package org.horizon36596.simloop.opmode;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/**
 * Notices threads an OpMode starts itself. SimLoop cannot step them, since they run in real time beside the
 * sim, but it can say that they exist.
 *
 * <p><b>How a thread is attributed to the OpMode.</b> Both kinds of OpMode run their code on a SimLoop thread
 * in a {@link ThreadGroup} of the run's own, and a new {@link Thread} joins the group of the thread that
 * creates it. So any thread the OpMode's code creates - {@code new Thread(...)}, an {@code Executors} pool
 * made there, the common {@code ForkJoinPool}'s first workers - lands in that group, and nothing else does:
 * not the RLOG writer, not other tests. After each tick's OpMode code, the group is listed. The group also
 * hears of every thread in it that dies of an uncaught exception (unless the thread has a handler of its
 * own), so a thread that dies that way is seen even if it lived for less than one tick.
 *
 * <p><b>What it misses.</b> A thread created with an explicit group of its own; a thread that starts and ends
 * normally within one tick's code; a pool thread created earlier by other code and merely reused.
 */
final class OpModeThreads {

    /**
     * How long, in one-millisecond waits, the threads the OpMode started are given at the end of the run to
     * finish after STOP before they are reported as still running.
     */
    static final int EXIT_GRACE_WAITS = 100;

    private final RunGroup group;
    private final String opModeName;

    /** Every thread seen so far, and where it was first seen. Sim thread only. */
    private final Map<Thread, String> seen = new LinkedHashMap<>();

    /** Threads of the group that died of an uncaught exception, and what it was. Guarded by itself. */
    private final Map<Thread, Throwable> died = new LinkedHashMap<>();

    OpModeThreads(String opModeName) {
        this.opModeName = opModeName;
        this.group = new RunGroup("SimLoop-OpMode-" + opModeName);
    }

    /** {@return the group the runner's OpMode thread must be created in} */
    ThreadGroup group() {
        return group;
    }

    /**
     * Lists the group, ignoring {@code own} (the runner's thread for the OpMode), and the threads of it that
     * have died of an uncaught exception since the last look.
     *
     * @return the name of the first thread not seen before, or null if there is none
     */
    String look(Thread own, OpModePhase phase, int tick) {
        String first = null;
        List<Thread> found = live();
        synchronized (died) {
            found.addAll(died.keySet());
        }
        for (Thread thread : found) {
            if (thread != own && !seen.containsKey(thread)) {
                seen.put(thread, "first seen in " + phase + ", tick " + tick);
                if (first == null) {
                    first = thread.getName();
                }
            }
        }
        return first;
    }

    /**
     * Waits, briefly, for every thread still in the group apart from {@code own} to finish. Waits are counted,
     * not timed, and shared: {@link #EXIT_GRACE_WAITS} in all.
     */
    void awaitExit(Thread own) {
        // The group is looked at afresh after every wait, so a thread started during the grace is waited for too.
        for (int waits = 0; waits < EXIT_GRACE_WAITS; waits++) {
            Thread waitingFor = null;
            for (Thread thread : live()) {
                if (thread != own && thread.isAlive()) {
                    waitingFor = thread;
                    break;
                }
            }
            if (waitingFor == null) {
                return;
            }
            try {
                waitingFor.join(1);
            } catch (InterruptedException interrupted) {
                Thread.currentThread().interrupt();
                return;
            }
        }
    }

    /** {@return one warning per thread the OpMode started, saying how it ended if it died of an exception} */
    List<String> warnings() {
        List<String> warnings = new ArrayList<>();
        for (Map.Entry<Thread, String> entry : seen.entrySet()) {
            Throwable death;
            synchronized (died) {
                death = died.get(entry.getKey());
            }
            warnings.add(opModeName + " started a thread of its own, \"" + entry.getKey().getName() + "\" ("
                    + entry.getValue() + "), which SimLoop does not step: it ran in real time beside the sim, so"
                    + " what it did is not deterministic."
                    + (death != null ? " It died of an uncaught " + death + "."
                            : entry.getKey().isAlive() ? " It was still running when the run ended." : ""));
        }
        return warnings;
    }

    /**
     * Lets the group go once nothing is left in it. On a Java 8 or Android runtime an empty group otherwise
     * stays listed under its parent for good; on newer JVMs this does nothing.
     */
    @SuppressWarnings("deprecation")
    void close() {
        try {
            if (live().isEmpty() && !group.isDestroyed()) {
                group.destroy();
            }
        } catch (RuntimeException alreadyGoneOrStillInUse) {
            // Harmless either way: the group is only bookkeeping.
        }
    }

    private List<Thread> live() {
        Thread[] threads = new Thread[group.activeCount() + 8];
        int count = group.enumerate(threads, true);
        while (count == threads.length) {
            threads = new Thread[threads.length * 2];
            count = group.enumerate(threads, true);
        }
        List<Thread> live = new ArrayList<>(count);
        for (int i = 0; i < count; i++) {
            if (threads[i].isAlive()) {
                live.add(threads[i]);
            }
        }
        return live;
    }

    /** The run's group: records each uncaught death, then reports it as any group does (to stderr). */
    private final class RunGroup extends ThreadGroup {

        RunGroup(String name) {
            super(name);
        }

        @Override
        public void uncaughtException(Thread thread, Throwable thrown) {
            synchronized (died) {
                died.put(thread, thrown);
            }
            super.uncaughtException(thread, thrown);
        }
    }
}
