package org.horizon36596.simloop.simclock;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertInstanceOf;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTimeoutPreemptively;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assertions.fail;

import org.firstinspires.ftc.teamcode.simclockfixture.ClockReadsAuto;
import org.firstinspires.ftc.teamcode.simclockfixture.ClockSwitches;
import org.firstinspires.ftc.teamcode.simclockfixture.ComputedSleepAuto;
import org.firstinspires.ftc.teamcode.simclockfixture.ElapsedTimeAuto;
import org.firstinspires.ftc.teamcode.simclockfixture.LapTimer;
import org.firstinspires.ftc.teamcode.simclockfixture.MethodReferenceAuto;
import org.horizon36596.simloop.sim.FakeTimer;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.lang.reflect.Method;
import java.net.URL;
import java.net.URLClassLoader;
import java.nio.file.Files;
import java.nio.file.Path;
import java.time.Duration;
import java.util.Arrays;
import java.util.Map;
import java.util.Random;
import java.util.TreeMap;

/**
 * Pins what {@link ClassFileRewriter}'s class javadoc promises for damaged input: a file that is cut short,
 * is not a class file, or holds a count, length or instruction size that does not fit fails with an
 * {@link IllegalArgumentException} naming the class, quickly. It never hangs, never fills the heap and never
 * comes back silently corrupted. Also the constant-pool boundary (65535 entries) and a superclass cycle on the
 * classpath, which {@link SimClockLoader} must refuse rather than walk forever.
 *
 * <p>Every case runs under a preemptive timeout, so a hang is a failure, not a stuck build. The inputs are
 * built by {@link RawClassFile} or are mutations of the {@code simclockfixture} classes; no bytecode library.
 *
 * <p>Invariant tier: a loader that hangs inside {@code Class.forName} holds its lock and every other load
 * on it, and the test JVM with it.
 */
class MalformedClassFileTest {

    private static final String TEAM = "org/firstinspires/ftc/teamcode/simclockfixture/raw/";
    private static final Duration FAST = Duration.ofSeconds(5);
    private static final ClassFileRewriter.Hierarchy NO_HIERARCHY = (name, ancestor) -> name.equals(ancestor);

    private final ClassFileRewriter rewriter = new ClassFileRewriter();

    private void assertRefusedFast(String what, byte[] bytes) {
        IllegalArgumentException refused = assertTimeoutPreemptively(FAST,
                () -> assertThrows(IllegalArgumentException.class, () -> rewriter.rewrite(what, bytes, NO_HIERARCHY)),
                what + ": the rewriter did not finish within " + FAST);
        assertTrue(refused.getMessage().startsWith("SimLoop cannot read the class file of " + what),
                refused.getMessage());
    }

    /** One static method whose Code is {@code code}; nothing to rewrite in it. */
    private static byte[] classWithCode(byte[] code, byte[] codeAttributes) {
        RawClassFile c = new RawClassFile();
        c.method(0x09, "m", "()V", 4, 4, code, codeAttributes);
        return c.build(TEAM + "Code", "java/lang/Object");
    }

    // ---- a switch whose size is zero or negative ---------------------------------------------------

    @Test
    void aTableswitchWithHighBelowLowIsRefusedNotLoopedOnForever() {
        // tableswitch at code offset 0 (3 pad bytes), default 0, low 0, high -5: 1+3+12+4*(high-low+1) = 0.
        byte[] code = RawClassFile.bytes(0xaa, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0xff, 0xff, 0xff, 0xfb);
        assertRefusedFast("ZeroLengthTableswitch", classWithCode(code, new byte[] {0, 0}));
    }

    @Test
    void aLookupswitchWithNegativePairsIsRefusedNotLoopedOnForever() {
        // 4 nops, then a lookupswitch at offset 4 (3 pad bytes) with npairs -2: 12 + 8*(-2) = -4, back to 0.
        byte[] code = RawClassFile.bytes(0, 0, 0, 0, 0xab, 0, 0, 0, 0, 0, 0, 0, 0xff, 0xff, 0xff, 0xfe);
        assertRefusedFast("NegativeLookupswitch", classWithCode(code, new byte[] {0, 0}));
    }

    @Test
    void anInstructionThatRunsPastTheEndOfItsCodeIsRefused() {
        // invokestatic needs three bytes; the method's code is one byte long.
        assertRefusedFast("CutInstruction", classWithCode(RawClassFile.bytes(0xb8), new byte[] {0, 0}));
    }

    // ---- attribute lengths that do not advance ------------------------------------------------------

    @Test
    void aLineNumberTableWhoseLengthDoesNotAdvanceIsRefusedFast() {
        RawClassFile c = new RawClassFile();
        int lnt = c.utf8("LineNumberTable");
        ByteArrayOutputStream attrs = new ByteArrayOutputStream();
        writeU2(attrs, 0xffff);                             // 65535 attributes...
        writeU2(attrs, lnt);
        writeU4(attrs, -6);                                 // ...each of length -6, so the reader would stay put
        writeU2(attrs, 0xffff);                             // 65535 entries, all present
        attrs.write(new byte[4 * 0xffff], 0, 4 * 0xffff);
        c.method(0x09, "m", "()V", 1, 1, RawClassFile.bytes(0xb1), attrs.toByteArray());
        assertRefusedFast("RepeatedLineNumberTable", c.build(TEAM + "Lnt", "java/lang/Object"));
    }

    @Test
    void aLineNumberTableThatDisagreesWithItsLengthIsRefused() {
        RawClassFile c = new RawClassFile();
        int lnt = c.utf8("LineNumberTable");
        ByteArrayOutputStream attrs = new ByteArrayOutputStream();
        writeU2(attrs, 1);
        writeU2(attrs, lnt);
        writeU4(attrs, 6);                                  // room for one entry...
        writeU2(attrs, 9);                                  // ...claims nine
        writeU4(attrs, 0);
        c.method(0x09, "m", "()V", 1, 1, RawClassFile.bytes(0xb1), attrs.toByteArray());
        assertRefusedFast("LyingLineNumberTable", c.build(TEAM + "Lnt2", "java/lang/Object"));
    }

    @Test
    void aBootstrapMethodsAttributeWhoseLengthDoesNotAdvanceIsRefusedFast() {
        RawClassFile c = new RawClassFile();
        int bsm = c.utf8("BootstrapMethods");
        ByteArrayOutputStream attrs = new ByteArrayOutputStream();
        writeU2(attrs, 0xffff);                             // 65535 class attributes...
        writeU2(attrs, bsm);
        writeU4(attrs, -6);                                 // ...each re-read in place
        writeU2(attrs, 0xffff);                             // 65535 bootstrap methods of no arguments
        for (int i = 0; i < 0xffff; i++) {
            writeU2(attrs, 1);
            writeU2(attrs, 0);
        }
        c.classAttributesSection = attrs.toByteArray();
        assertRefusedFast("RepeatedBootstrapMethods", c.build(TEAM + "Bsm", "java/lang/Object"));
    }

    @Test
    void fieldAttributesWhoseLengthsDoNotAdvanceAreRefusedFast() {
        // Self-similar 8-byte units: read as a field (access 0, name ffff, descriptor fffa, 65535 attributes)
        // and, one unit on, as its attribute (length fffffffa = -6): 65535 fields x 65535 attributes unchecked.
        ByteArrayOutputStream fields = new ByteArrayOutputStream();
        writeU2(fields, 0xffff);
        byte[] unit = {0, 0, (byte) 0xff, (byte) 0xff, (byte) 0xff, (byte) 0xfa, (byte) 0xff, (byte) 0xff};
        for (int i = 0; i < 0x10001; i++) {
            fields.write(unit, 0, unit.length);
        }
        RawClassFile c = new RawClassFile();
        c.fieldsSection = fields.toByteArray();
        assertRefusedFast("QuadraticFields", c.build(TEAM + "Fields", "java/lang/Object"));
    }

    // ---- counts, tags, magic, descriptors -------------------------------------------------------------

    @Test
    void hugeCountsOnATinyFileAreRefusedFast() {
        byte[] header = RawClassFile.bytes(0xca, 0xfe, 0xba, 0xbe, 0, 0, 0, 52, 0xff, 0xff, 1, 0xff, 0xff);
        assertRefusedFast("HugePool", header);
        byte[] codeTooLong = classWithCode(RawClassFile.bytes(0xb1), new byte[] {0, 0});
        codeTooLong[indexOf(codeTooLong, RawClassFile.bytes(0, 0, 0, 1, 0xb1))] = 0x7f; // code_length 0x7f000001
        assertRefusedFast("HugeCodeLength", codeTooLong);
    }

    @Test
    void everyUndefinedConstantPoolTagIsRefused() {
        for (int tag : new int[] {0, 2, 13, 14, 21, 127, 255}) {
            byte[] bytes = classWithCode(RawClassFile.bytes(0xb1), new byte[] {0, 0});
            bytes[10] = (byte) tag;
            assertRefusedFast("Tag" + tag, bytes);
        }
    }

    @Test
    void aBadMagicAndAnEmptyFileAreRefused() {
        byte[] bytes = classWithCode(RawClassFile.bytes(0xb1), new byte[] {0, 0});
        bytes[0] = 0;
        assertRefusedFast("BadMagic", bytes);
        assertRefusedFast("Empty", new byte[0]);
    }

    @Test
    void aMethodrefWhoseDescriptorIsNotAMethodDescriptorIsADamagedFileNotARefusal() {
        // A damaged NameAndType used to come back as an IllegalStateException naming a nonsense constructor.
        RawClassFile c = new RawClassFile();
        int init = c.methodref("com/qualcomm/robotcore/util/ElapsedTime", "<init>", "Code");
        c.method(0x09, "m", "()V", 2, 0, RawClassFile.bytes(0xb7, init >> 8, init & 0xff, 0xb1), new byte[] {0, 0});
        assertRefusedFast("NotADescriptor", c.build(TEAM + "Desc", "java/lang/Object"));
    }

    // ---- truncation and fuzzing of real rewritable classes --------------------------------------------

    @Test
    void everyTruncationOfARewritableClassIsRefusedWithTheReadError() {
        byte[] whole = ClassFileRewriterTest.bytesOf(ElapsedTimeAuto.class);
        assertTimeoutPreemptively(Duration.ofSeconds(10), () -> {
            for (int length = 0; length < whole.length; length++) {
                byte[] cut = Arrays.copyOf(whole, length);
                try {
                    rewriter.rewrite("Cut" + length, cut, NO_HIERARCHY);
                    fail("a class cut to " + length + " of " + whole.length + " bytes was accepted");
                } catch (IllegalArgumentException expected) {
                    assertTrue(expected.getMessage().startsWith("SimLoop cannot read the class file of Cut" + length),
                            expected.getMessage());
                }
            }
        });
    }

    /**
     * Random byte mutations, from a fixed seed, of classes the rewriter changes. Allowed: a result, an
     * IllegalArgumentException (cannot read) or an IllegalStateException (read and refused). Anything else,
     * or a mutation that does not finish, is a failure naming the mutation.
     */
    @Test
    void randomMutationsNeverHangOrThrowAnythingButTheDocumentedExceptions() {
        byte[][] seeds = {
                ClassFileRewriterTest.bytesOf(ClockSwitches.class),
                ClassFileRewriterTest.bytesOf(ElapsedTimeAuto.class),
                ClassFileRewriterTest.bytesOf(MethodReferenceAuto.class),
                ClassFileRewriterTest.bytesOf(ComputedSleepAuto.class),
                ClassFileRewriterTest.bytesOf(ClockReadsAuto.class),
                ClassFileRewriterTest.bytesOf(LapTimer.class)};
        Random random = new Random(20260928L);
        Map<String, Integer> outcomes = new TreeMap<>();
        String[] current = {"none yet"};
        assertTimeoutPreemptively(Duration.ofSeconds(10), () -> {
            for (int run = 0; run < 5000; run++) {
                byte[] bytes = seeds[run % seeds.length].clone();
                StringBuilder how = new StringBuilder("seed " + run % seeds.length);
                for (int flip = 1 + random.nextInt(4); flip > 0; flip--) {
                    int at = 8 + random.nextInt(bytes.length - 8); // keep the magic and the version
                    int value = random.nextInt(256);
                    how.append(String.format(" @%d=%02x", at, value));
                    bytes[at] = (byte) value;
                }
                current[0] = how.toString();
                String kind;
                try {
                    rewriter.rewrite("Fuzz", bytes, NO_HIERARCHY);
                    kind = "ok";
                } catch (IllegalArgumentException | IllegalStateException documented) {
                    kind = documented.getClass().getSimpleName();
                } catch (RuntimeException | Error other) {
                    throw new AssertionError("mutation " + current[0] + " threw " + other, other);
                }
                outcomes.merge(kind, 1, Integer::sum);
            }
        }, () -> "a mutation did not finish: " + current[0]);
        assertTrue(outcomes.get("IllegalArgumentException") > 0, "the fuzz reached the reader's checks: " + outcomes);
    }

    // ---- a superclass cycle on the classpath ----------------------------------------------------------

    @Test
    void aSuperclassCycleOnTheClasspathFailsInsteadOfHangingTheLoader(@TempDir Path dir) throws Exception {
        write(dir, "A", new RawClassFile().build(TEAM + "A", TEAM + "B"));
        write(dir, "B", new RawClassFile().build(TEAM + "B", TEAM + "A"));
        RawClassFile caller = new RawClassFile();
        int sleep = caller.methodref(TEAM + "A", "sleep", "(J)V"); // is A a Thread? The walk meets the cycle.
        caller.method(0x09, "m", "()V", 2, 0,
                RawClassFile.bytes(0x09, 0xb8, sleep >> 8, sleep & 0xff, 0xb1), new byte[] {0, 0}); // lconst_0
        write(dir, "Caller", caller.build(TEAM + "Caller", "java/lang/Object"));
        try (URLClassLoader parent = new URLClassLoader(new URL[] {dir.toUri().toURL()},
                MalformedClassFileTest.class.getClassLoader())) {
            SimClockLoader loader = new SimClockLoader(parent, new FakeTimer(), SimClockLoader.DEFAULT_TEAM_PACKAGES);
            Throwable thrown = assertTimeoutPreemptively(FAST, () -> {
                try {
                    Class.forName(TEAM.replace('/', '.') + "Caller", false, loader);
                    return null;
                } catch (Throwable t) {
                    return t;
                }
            }, "SimClockLoader walked the A <-> B superclass cycle for longer than " + FAST);
            assertInstanceOf(IllegalArgumentException.class, thrown);
            assertTrue(thrown.getMessage().contains("superclass cycle"), thrown.getMessage());
        }
    }

    // ---- the constant-pool size limit -----------------------------------------------------------------

    /** A team class whose pool holds exactly {@code count - 1} entries and one System.nanoTime() call. */
    private static byte[] nanoTimeClassWithPoolCount(int count, String name, boolean longLast) {
        RawClassFile c = new RawClassFile();
        int nano = c.methodref("java/lang/System", "nanoTime", "()J");
        c.method(0x09, "f", "()J", 2, 0, RawClassFile.bytes(0xb8, nano >> 8, nano & 0xff, 0xad), new byte[] {0, 0});
        int self = c.cls(TEAM + name);
        int sup = c.cls("java/lang/Object");
        int reserve = longLast ? 2 : 0;
        for (int i = 0; c.poolCount() < count - reserve; i++) {
            c.utf8("pad" + i);
        }
        if (longLast) {
            c.longConst(0x0102030405060708L);
        }
        assertEquals(count, c.poolCount());
        return c.buildWith(self, sup);
    }

    @Test
    void aPoolWithRoomForExactlyTheSimClockNamesIsRewrittenAndRuns(@TempDir Path dir) throws Exception {
        for (boolean longLast : new boolean[] {false, true}) {
            String name = "Full" + longLast;
            byte[] bytes = nanoTimeClassWithPoolCount(65533, name, longLast); // + a Utf8 and a Class = 65535
            ClassFileRewriter.Result result = rewriter.rewrite(name, bytes, NO_HIERARCHY);
            assertEquals(0xffff, ((result.bytes[8] & 0xff) << 8) | (result.bytes[9] & 0xff));
            write(dir, name, bytes);
            try (URLClassLoader parent = new URLClassLoader(new URL[] {dir.toUri().toURL()},
                    MalformedClassFileTest.class.getClassLoader())) {
                FakeTimer clock = new FakeTimer();
                clock.advance(1.25);
                SimClockLoader loader = new SimClockLoader(parent, clock, SimClockLoader.DEFAULT_TEAM_PACKAGES);
                Method f = Class.forName(TEAM.replace('/', '.') + name, true, loader).getMethod("f");
                assertEquals(1_250_000_000L, f.invoke(null));
            }
        }
    }

    @Test
    void aPoolOneEntryTooFullIsRefusedSayingSo() {
        for (boolean longLast : new boolean[] {false, true}) {
            byte[] bytes = nanoTimeClassWithPoolCount(65534, "Over" + longLast, longLast);
            IllegalStateException refused = assertThrows(IllegalStateException.class,
                    () -> rewriter.rewrite("Over", bytes, NO_HIERARCHY));
            assertTrue(refused.getMessage().contains("constant pool is already full"), refused.getMessage());
        }
    }

    // ---- helpers ----------------------------------------------------------------------------------------

    private static void write(Path dir, String simpleName, byte[] bytes) throws IOException {
        Path file = dir.resolve(TEAM + simpleName + ".class");
        Files.createDirectories(file.getParent());
        Files.write(file, bytes);
    }

    private static void writeU2(ByteArrayOutputStream out, int v) {
        out.write(v >> 8);
        out.write(v);
    }

    private static void writeU4(ByteArrayOutputStream out, int v) {
        out.write(v >> 24);
        out.write(v >> 16);
        out.write(v >> 8);
        out.write(v);
    }

    private static int indexOf(byte[] hay, byte[] needle) {
        outer:
        for (int i = 0; i <= hay.length - needle.length; i++) {
            for (int j = 0; j < needle.length; j++) {
                if (hay[i + j] != needle[j]) {
                    continue outer;
                }
            }
            return i;
        }
        throw new AssertionError("pattern not found");
    }
}
