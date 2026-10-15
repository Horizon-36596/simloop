package org.firstinspires.ftc.teamcode.simclockfixture;

/**
 * A team class with its own sleep(long) and idle() that are NOT LinearOpMode's: the rewriter must leave calls
 * to them alone, because it only moves waits whose owner is a LinearOpMode.
 */
public class LookalikeWaits {

    private long slept;
    private int idled;

    /** Adds to a counter; waits for nothing. */
    public void sleep(long millis) {
        slept += millis;
    }

    /** Adds to a counter; waits for nothing. */
    public void idle() {
        idled++;
    }

    /** @return "slept/idled" after one sleep(25) and one idle() */
    public static String exercise() {
        LookalikeWaits waits = new LookalikeWaits();
        waits.sleep(25);
        waits.idle();
        return waits.slept + "/" + waits.idled;
    }
}
