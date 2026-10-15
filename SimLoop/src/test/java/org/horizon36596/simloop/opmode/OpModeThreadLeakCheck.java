package org.horizon36596.simloop.opmode;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

/**
 * The "no OpMode thread outlives its run" check, scoped to one test: make one as a field of the test class
 * (JUnit builds a fresh instance per test, so it is made just before the test runs) and call {@link #verify()}
 * from an {@code @AfterEach}.
 *
 * <p><b>Why scoped.</b> The check used to count every live OpMode thread in the JVM. One thread left running -
 * a run cut short by a timeout on a loaded machine, or a stuck-OpMode test whose assertion failed before it
 * let its spin go - then failed every later test in the JVM, about forty at once, and hid which test was the
 * real failure. Now a leaked thread fails the test that leaked it, and only that one: a thread already alive
 * when this check was made belongs to an earlier test, which has already failed for it.
 *
 * <p><b>Threads a test abandons on purpose.</b> A test that runs an OpMode the runner cannot stop registers
 * how to let it go with {@link #releaseAfter}. {@link #verify()} runs every release whether the test passed or
 * failed, so a failed assertion cannot leave a thread spinning, then gives the released threads time to end.
 * A test that registers nothing gets no grace: its threads must already be gone when the run returns, which is
 * the runner's contract.
 */
final class OpModeThreadLeakCheck {

    /** Name prefix of the thread a {@code LinearOpMode} runs on. */
    static final String LINEAR = "SimLoop-LinearOpMode-";

    /** Name prefix of the thread an iterative {@code OpMode} runs on. */
    static final String ITERATIVE = "SimLoop-OpMode-";

    /**
     * How long, in milliseconds of real time, threads this test abandoned are given to end once let go.
     * Generous on purpose: a loaded machine can starve a thread for a long time, and the whole wait is only
     * ever spent by a test that is failing anyway.
     */
    static final long END_GRACE_MILLIS = 10_000;

    private final String namePrefix;
    private final Set<Thread> aliveBefore;
    private final List<Runnable> releases = new ArrayList<>();

    /** @param namePrefix {@link #LINEAR} or {@link #ITERATIVE} */
    OpModeThreadLeakCheck(String namePrefix) {
        this.namePrefix = namePrefix;
        this.aliveBefore = new HashSet<>(alive(namePrefix));
    }

    /** Registers how to let a deliberately stuck OpMode go; {@link #verify()} runs it, pass or fail. */
    void releaseAfter(Runnable letGo) {
        releases.add(letGo);
    }

    /**
     * Waits, at most {@link #END_GRACE_MILLIS} in all, for every OpMode thread started since this check was
     * made to end. Returns early once they have; never fails by itself.
     */
    void awaitStartedThreadsEnd() throws InterruptedException {
        long deadline = System.nanoTime() + END_GRACE_MILLIS * 1_000_000L; // the test's own clock, not the sim's
        for (Thread thread : startedSince()) {
            long leftMillis = (deadline - System.nanoTime()) / 1_000_000L;
            if (leftMillis <= 0) {
                return;
            }
            thread.join(leftMillis);
        }
    }

    /** Fails with the names of the OpMode threads started since this check was made that are still alive. */
    void assertStartedThreadsEnded(String message) {
        List<String> leftover = new ArrayList<>();
        for (Thread thread : startedSince()) {
            leftover.add(thread.getName());
        }
        assertEquals(Collections.<String>emptyList(), leftover, message);
    }

    /** Lets every registered thread go, gives them time to end if there were any, then checks none is left. */
    void verify() throws InterruptedException {
        if (!releases.isEmpty()) {
            for (Runnable letGo : releases) {
                letGo.run();
            }
            awaitStartedThreadsEnd();
        }
        assertStartedThreadsEnded("an OpMode thread outlived its run");
    }

    private List<Thread> startedSince() {
        List<Thread> started = alive(namePrefix);
        started.removeAll(aliveBefore);
        return started;
    }

    private static List<Thread> alive(String namePrefix) {
        List<Thread> alive = new ArrayList<>();
        for (Thread thread : Thread.getAllStackTraces().keySet()) {
            if (thread.isAlive() && thread.getName().startsWith(namePrefix)) {
                alive.add(thread);
            }
        }
        return alive;
    }
}
