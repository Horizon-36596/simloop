package org.firstinspires.ftc.teamcode.simclockfixture;

/**
 * Clock reads inside a {@code tableswitch}, a {@code lookupswitch} and a string switch: the variable-length
 * instructions a damaged class file can give an impossible size. Used as a seed for the malformed-input tests.
 */
public final class ClockSwitches {

    private ClockSwitches() {
    }

    /** A dense switch: javac compiles it to a tableswitch. */
    public static long table(int k) {
        switch (k) {
            case 0: return System.nanoTime();
            case 1: return System.currentTimeMillis();
            case 2: return 2;
            case 3: return System.nanoTime() + 3;
            default: return -1;
        }
    }

    /** A sparse switch: javac compiles it to a lookupswitch. */
    public static long lookup(int k) {
        switch (k) {
            case -100000: return System.nanoTime();
            case 7: return 7;
            case 900000: return System.currentTimeMillis();
            default: return -1;
        }
    }

    /** A string switch: a lookupswitch on the hash, then a tableswitch. */
    public static long strings(String s) {
        switch (s) {
            case "nano": return System.nanoTime();
            case "milli": return System.currentTimeMillis();
            default: return -2;
        }
    }
}
