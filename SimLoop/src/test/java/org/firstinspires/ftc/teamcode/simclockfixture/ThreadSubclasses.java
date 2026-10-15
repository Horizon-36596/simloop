package org.firstinspires.ftc.teamcode.simclockfixture;

import java.util.concurrent.ForkJoinWorkerThread;

/** Thread.sleep reached through a team subclass of Thread and a JDK one, and a team static that hides it. */
public final class ThreadSubclasses {

    private ThreadSubclasses() {
    }

    /** A team Thread subclass whose own code calls the inherited static sleep: the Methodref owner is Worker. */
    public static class Worker extends Thread {
        /** Sleeps through the inherited Thread.sleep. */
        public static void nap(long ms) throws InterruptedException {
            sleep(ms);
        }
    }

    /** A team Thread subclass that declares its OWN static sleep(long), hiding Thread.sleep. */
    public static class HidingThread extends Thread {
        /** How many times the team's own sleep ran. */
        public static int calls;

        /** Never waits: this is the team's method, not Thread.sleep. */
        public static void sleep(long ms) {
            calls++;
        }
    }

    /** {@return how many times HidingThread's own sleep ran: 1, as Java's hiding rules say} */
    public static int callHiding() {
        HidingThread.calls = 0;
        HidingThread.sleep(5);
        return HidingThread.calls;
    }

    /** Thread.sleep with a JDK subclass of Thread as the qualifier, which javac keeps as the owner. */
    public static void viaJdkSubclass(long ms) throws InterruptedException {
        ForkJoinWorkerThread.sleep(ms);
    }
}
