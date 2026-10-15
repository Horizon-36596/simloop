package org.horizon36596.simloop.simclock;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotSame;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.firstinspires.ftc.teamcode.simclockfixture.ClockProbe;
import org.firstinspires.ftc.teamcode.simclockfixture.ComputedSleepAuto;
import org.firstinspires.ftc.teamcode.simclockfixture.LapTimer;
import org.firstinspires.ftc.teamcode.simclockfixture.LookalikeWaits;
import org.firstinspires.ftc.teamcode.simclockfixture.PlainMath;
import org.firstinspires.ftc.teamcode.simclockfixture.SleepMethodReferenceAuto;
import org.firstinspires.ftc.teamcode.simclockfixture.ThreadSleepAuto;
import org.junit.jupiter.api.Test;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.UncheckedIOException;
import java.util.Arrays;

/**
 * Pins {@link ClassFileRewriter} on its own, one class file at a time: what it changes, what it leaves alone,
 * and how it refuses a file it cannot handle. Whether the result runs is {@link SimClockLoaderTest}'s job.
 *
 * <p>Invariant tier: a rewriter that silently mangled a class, or silently left a clock call behind, would
 * make a sim quietly wrong.
 */
class ClassFileRewriterTest {

    private final ClassFileRewriter rewriter = new ClassFileRewriter();

    /** Answers subclass questions the way the loader does, from the class files on the test classpath. */
    private static final ClassFileRewriter.Hierarchy HIERARCHY = (internalName, ancestor) -> {
        for (String current = internalName; current != null; ) {
            if (current.equals(ancestor)) {
                return true;
            }
            if (current.startsWith("java/")) {
                return false;
            }
            byte[] bytes = bytesOrNull(current);
            current = bytes == null ? null : ClassFileRewriter.superclassOf(current, bytes);
        }
        return false;
    };

    static byte[] bytesOf(Class<?> type) {
        byte[] bytes = bytesOrNull(type.getName().replace('.', '/'));
        if (bytes == null) {
            throw new AssertionError("no class file for " + type);
        }
        return bytes;
    }

    private static byte[] bytesOrNull(String internalName) {
        try (InputStream in = ClassFileRewriterTest.class.getClassLoader()
                .getResourceAsStream(internalName + ".class")) {
            if (in == null) {
                return null;
            }
            ByteArrayOutputStream out = new ByteArrayOutputStream();
            byte[] buffer = new byte[8192];
            for (int n = in.read(buffer); n > 0; n = in.read(buffer)) {
                out.write(buffer, 0, n);
            }
            return out.toByteArray();
        } catch (IOException e) {
            throw new UncheckedIOException(e);
        }
    }

    private ClassFileRewriter.Result rewrite(Class<?> type) {
        return rewriter.rewrite(type.getName(), bytesOf(type), HIERARCHY);
    }

    // ---- what changes, what does not --------------------------------------------------------------

    @Test
    void aClassWithNoClockCallsComesBackAsTheVerySameArray() {
        byte[] original = bytesOf(PlainMath.class);
        ClassFileRewriter.Result result = rewriter.rewrite(PlainMath.class.getName(), original, HIERARCHY);
        assertSame(original, result.bytes);
        assertEquals(0, result.callSites);
    }

    @Test
    void aTeamSleepAndIdleThatAreNotLinearOpModesAreLeftAlone() {
        byte[] original = bytesOf(LookalikeWaits.class);
        assertSame(original, rewriter.rewrite(LookalikeWaits.class.getName(), original, HIERARCHY).bytes);
    }

    @Test
    void theInputArrayIsNeverModified() {
        byte[] original = bytesOf(ThreadSleepAuto.class);
        byte[] copy = original.clone();
        ClassFileRewriter.Result result = rewriter.rewrite(ThreadSleepAuto.class.getName(), original, HIERARCHY);
        assertNotSame(original, result.bytes);
        assertTrue(Arrays.equals(copy, original), "the rewrite works on a copy");
    }

    @Test
    void everyClockCallInAClassIsCounted() {
        // A construction counts once (its new, not also its constructor call); new LapTimer() is not one.
        ClassFileRewriter.Result probe = rewrite(ClockProbe.class);
        assertEquals(6, probe.callSites, "nanoTime(), currentTimeMillis(), new ElapsedTime(), System::nanoTime,"
                + " ElapsedTime::new, Thread.sleep()");
        // sleep(ms * 2), pause's sleep(millis), sleep(fast ? ..), idle(), plus new ElapsedTime().
        assertEquals(5, rewrite(ComputedSleepAuto.class).callSites);
    }

    @Test
    void aSubclassOfElapsedTimeIsReparentedOntoSimClockElapsedTime() {
        ClassFileRewriter.Result result = rewrite(LapTimer.class);
        assertEquals(ClassFileRewriter.SIM_ELAPSED_TIME,
                ClassFileRewriter.superclassOf("LapTimer", result.bytes));
        assertEquals(ClassFileRewriter.ELAPSED_TIME, ClassFileRewriter.superclassOf("LapTimer", bytesOf(LapTimer.class)));
    }

    @Test
    void theRewrittenFileKeepsItsLengthApartFromTheAppendedPoolEntries() {
        // Nothing inside a method moves: only new constant-pool entries are added, before the methods.
        byte[] original = bytesOf(ThreadSleepAuto.class);
        byte[] rewritten = rewrite(ThreadSleepAuto.class).bytes;
        int addedEntries = u2(rewritten, 8) - u2(original, 8);
        assertTrue(addedEntries > 0 && addedEntries < 20, "added " + addedEntries + " pool entries");
        assertTrue(rewritten.length > original.length && rewritten.length - original.length < 400,
                "grew by " + (rewritten.length - original.length) + " bytes");
    }

    // ---- refusals ----------------------------------------------------------------------------------

    @Test
    void aMethodReferenceToSleepIsRefusedWithTheClassMethodAndLine() {
        IllegalStateException refused = assertThrows(IllegalStateException.class,
                () -> rewrite(SleepMethodReferenceAuto.class));
        String message = refused.getMessage();
        assertTrue(message.contains("::sleep"), message);
        assertTrue(message.contains(SleepMethodReferenceAuto.class.getName() + ".runOpMode (SleepMethodReferenceAuto.java:"),
                message);
        assertTrue(message.contains("ms -> sleep(ms)"), "says what to write instead: " + message);
    }

    // ---- files it cannot read ----------------------------------------------------------------------

    @Test
    void somethingThatIsNotAClassFileIsRefusedClearly() {
        IllegalArgumentException refused = assertThrows(IllegalArgumentException.class,
                () -> rewriter.rewrite("com.example.Junk", "not a class".getBytes(), HIERARCHY));
        assertTrue(refused.getMessage().contains("com.example.Junk"), refused.getMessage());
        assertTrue(refused.getMessage().contains("not a class file"), refused.getMessage());
    }

    @Test
    void aTruncatedClassFileIsRefusedClearly() {
        byte[] whole = bytesOf(ThreadSleepAuto.class);
        byte[] half = Arrays.copyOf(whole, whole.length / 2);
        IllegalArgumentException refused = assertThrows(IllegalArgumentException.class,
                () -> rewriter.rewrite("Half", half, HIERARCHY));
        assertTrue(refused.getMessage().startsWith("SimLoop cannot read the class file of Half"), refused.getMessage());
    }

    @Test
    void anUnknownConstantPoolTagIsRefusedClearly() {
        byte[] bytes = bytesOf(PlainMath.class);
        bytes[10] = 2; // the first entry's tag; 2 is not a tag JVMS defines
        IllegalArgumentException refused = assertThrows(IllegalArgumentException.class,
                () -> rewriter.rewrite("BadTag", bytes, HIERARCHY));
        assertTrue(refused.getMessage().contains("tag 2"), refused.getMessage());
    }

    @Test
    void aClassFileNewerThanTheReaderKnowsIsRefusedClearly() {
        byte[] bytes = bytesOf(PlainMath.class);
        bytes[6] = 0;
        bytes[7] = 99;
        IllegalArgumentException refused = assertThrows(IllegalArgumentException.class,
                () -> rewriter.rewrite("Future", bytes, HIERARCHY));
        assertTrue(refused.getMessage().contains("version 99"), refused.getMessage());
    }

    @Test
    void bytesAfterTheLastAttributeAreRefusedClearly() {
        byte[] whole = bytesOf(PlainMath.class);
        byte[] longer = Arrays.copyOf(whole, whole.length + 3);
        IllegalArgumentException refused = assertThrows(IllegalArgumentException.class,
                () -> rewriter.rewrite("Longer", longer, HIERARCHY));
        assertTrue(refused.getMessage().contains("3 bytes after its last attribute"), refused.getMessage());
    }

    private static int u2(byte[] bytes, int at) {
        return ((bytes[at] & 0xff) << 8) | (bytes[at + 1] & 0xff);
    }
}
