package org.horizon36596.simloop.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.horizon36596.simloop.simclock.JvmInstructions;

import java.io.DataInputStream;
import java.io.IOException;
import java.io.InputStream;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

/**
 * Finds out how long a {@code LinearOpMode.sleep(ms)} call asked for, by reading the calling class's
 * bytecode.
 *
 * <p><b>Why this exists at all.</b> The SDK declares {@code LinearOpMode.sleep(long)} {@code final}, and its
 * whole body is {@code Thread.sleep(ms)} (RobotCore 11.2.1 and 12.0.0 are byte-for-byte the same). Nothing
 * can override it, and by the time the runner regains control the argument is gone: it lived only in the
 * frame of a method that has already been entered. What is still known is where the call came from - the
 * stack trace names the calling class, method and source line. For the overwhelmingly common way sleep is
 * written, {@code sleep(1000)} or {@code sleep(SOME_CONSTANT)}, javac compiles the argument to a constant
 * loaded by the instruction immediately before the call, so reading that class file answers the question
 * exactly. This class does only that.
 *
 * <p><b>What it refuses, loudly.</b> A duration computed at run time ({@code sleep(ms)} inside a helper,
 * {@code sleep((long) (seconds * 1000))}), two sleeps with different constants on one source line, or a
 * class compiled without line numbers. Each is an {@link IllegalStateException} naming the file and line,
 * never a guess: a guessed sleep would be a sim that is quietly wrong.
 *
 * <p>The parser reads the class-file structure (JVMS §4) only as far as it needs: the constant pool, each
 * method's name, its {@code Code} attribute and that attribute's {@code LineNumberTable}. It walks the
 * instructions with {@link JvmInstructions}, the length table covering every opcode in JVMS §6.5 that it
 * shares with the sim-clock rewriter, and rejects anything else.
 */
final class SleepDurationReader {

    private static final int LCONST_0 = 0x09;
    private static final int LCONST_1 = 0x0a;
    private static final int LDC2_W = 0x14;
    private static final int TABLESWITCH = 0xaa;
    private static final int LOOKUPSWITCH = 0xab;
    private static final int IFEQ = 0x99;
    private static final int JSR = 0xa8;
    private static final int IFNULL = 0xc6;
    private static final int IFNONNULL = 0xc7;
    private static final int GOTO_W = 0xc8;
    private static final int JSR_W = 0xc9;

    /** Answers already worked out, keyed by call site, so each line's bytecode is read once per run. */
    private final Map<StackTraceElement, Long> answers = new HashMap<>();

    /**
     * {@return the constant duration, in milliseconds, that the {@code sleep} call at {@code callSite} passes}
     *
     * @param callSite the stack frame that called {@code LinearOpMode.sleep}
     * @param loader   the class loader that loaded the OpMode, used to find the caller's class file
     * @throws IllegalStateException if the duration is not a compile-time constant, if the line holds two
     *                               sleeps with different constants, or if the class file cannot be read
     */
    long millis(StackTraceElement callSite, ClassLoader loader) {
        Long known = answers.get(callSite);
        if (known != null) {
            return known;
        }
        long answer = readFromClassFile(callSite, loader);
        answers.put(callSite, answer);
        return answer;
    }

    private static long readFromClassFile(StackTraceElement site, ClassLoader loader) {
        String where = site.getClassName() + "." + site.getMethodName() + " ("
                + site.getFileName() + ":" + site.getLineNumber() + ")";
        if (site.getLineNumber() < 0) {
            throw new IllegalStateException("SimLoop cannot tell how long the sleep() in " + where + " is: the"
                    + " class was compiled without line numbers. Compile with javac's default -g:lines.");
        }
        ClassFile classFile;
        String resource = site.getClassName().replace('.', '/') + ".class";
        try (InputStream in = loader.getResourceAsStream(resource)) {
            if (in == null) {
                throw new IllegalStateException("SimLoop cannot find " + resource + " to read the sleep() in "
                        + where);
            }
            classFile = ClassFile.parse(new DataInputStream(in));
        } catch (IOException e) {
            throw new IllegalStateException("SimLoop could not read " + resource + " for the sleep() in " + where, e);
        }

        Set<Long> constants = new LinkedHashSet<>();
        boolean computed = false;
        for (ClassFile.Method method : classFile.methods) {
            if (!method.name.equals(site.getMethodName()) || method.code == null) {
                continue;
            }
            for (Long constantMillis : method.sleepCallsOnLine(classFile, site.getLineNumber(), loader)) {
                if (constantMillis == null) {
                    computed = true;
                } else {
                    constants.add(constantMillis);
                }
            }
        }
        if (computed) {
            throw new IllegalStateException("SimLoop cannot simulate the sleep() in " + where + ": its duration"
                    + " is computed while the OpMode runs, and LinearOpMode.sleep is final in the FTC SDK, so the"
                    + " sim never sees the value. Only a constant duration - sleep(500), or sleep(NAMED_CONSTANT)"
                    + " with a static final long or int - can be stepped in sim time. To wait a computed time,"
                    + " loop on opModeIsActive() against the sim clock instead of calling sleep().");
        }
        if (constants.isEmpty()) {
            throw new IllegalStateException("SimLoop cannot tell how long the sleep() in " + where + " is: no"
                    + " LinearOpMode.sleep call was found on that line, so sleep() was reached some other way -"
                    + " through an interface, a method reference or reflection - and the sim cannot read its"
                    + " duration. Call sleep(ms) directly on the OpMode instead.");
        }
        if (constants.size() != 1) {
            throw new IllegalStateException("SimLoop cannot tell how long the sleep() in " + where + " is: that"
                    + " line has sleep() calls with different durations " + constants + ". Put each sleep() on"
                    + " its own line.");
        }
        return constants.iterator().next();
    }

    /** The parts of a class file this reader needs, and nothing else. */
    private static final class ClassFile {
        /** Constant-pool entries by index: String for Utf8, Long for Long, int[] for two-index refs. */
        final Object[] pool;
        final List<Method> methods = new ArrayList<>();

        private ClassFile(Object[] pool) {
            this.pool = pool;
        }

        static ClassFile parse(DataInputStream in) throws IOException {
            if (in.readInt() != 0xCAFEBABE) {
                throw new IOException("not a class file");
            }
            in.readUnsignedShort(); // minor_version
            in.readUnsignedShort(); // major_version
            ClassFile classFile = new ClassFile(readConstantPool(in));
            in.readUnsignedShort(); // access_flags
            in.readUnsignedShort(); // this_class
            in.readUnsignedShort(); // super_class
            skipFully(in, 2 * in.readUnsignedShort()); // interfaces
            int fields = in.readUnsignedShort();
            for (int i = 0; i < fields; i++) {
                skipFully(in, 6);
                skipAttributes(in);
            }
            int methods = in.readUnsignedShort();
            for (int i = 0; i < methods; i++) {
                in.readUnsignedShort(); // access_flags
                String name = (String) classFile.pool[in.readUnsignedShort()];
                in.readUnsignedShort(); // descriptor_index
                classFile.methods.add(Method.read(name, in, classFile.pool));
            }
            return classFile;
        }

        private static Object[] readConstantPool(DataInputStream in) throws IOException {
            Object[] pool = new Object[in.readUnsignedShort()];
            for (int i = 1; i < pool.length; i++) {
                int tag = in.readUnsignedByte();
                switch (tag) {
                    case 1: pool[i] = in.readUTF(); break;                                    // Utf8
                    case 3: case 4: skipFully(in, 4); break;                                 // Integer, Float
                    case 5: pool[i] = in.readLong(); i++; break;                             // Long: 2 slots
                    case 6: skipFully(in, 8); i++; break;                                    // Double: 2 slots
                    case 7: case 8: case 16: case 19: case 20:                               // one index
                        pool[i] = new int[] {in.readUnsignedShort()};
                        break;
                    case 9: case 10: case 11: case 12: case 17: case 18:                     // two indexes
                        pool[i] = new int[] {in.readUnsignedShort(), in.readUnsignedShort()};
                        break;
                    case 15: skipFully(in, 3); break;                                        // MethodHandle
                    default: throw new IOException("unknown constant-pool tag " + tag);
                }
            }
            return pool;
        }

        /** Resolves a Methodref to {owner internal name, method name, descriptor}. */
        String[] methodRef(int index) {
            int[] ref = (int[]) pool[index];
            int[] nameAndType = (int[]) pool[ref[1]];
            return new String[] {
                    (String) pool[((int[]) pool[ref[0]])[0]],
                    (String) pool[nameAndType[0]],
                    (String) pool[nameAndType[1]]};
        }

        /** One method: its name, bytecode and line table. {@code code} is null for abstract methods. */
        static final class Method {
            final String name;
            byte[] code;
            /** Pairs of (start_pc, line_number), in class-file order. */
            int[] lineTable = new int[0];

            private Method(String name) {
                this.name = name;
            }

            static Method read(String name, DataInputStream in, Object[] pool) throws IOException {
                Method method = new Method(name);
                int attributes = in.readUnsignedShort();
                for (int a = 0; a < attributes; a++) {
                    String attributeName = (String) pool[in.readUnsignedShort()];
                    int length = in.readInt();
                    if (!"Code".equals(attributeName)) {
                        skipFully(in, length);
                        continue;
                    }
                    in.readUnsignedShort(); // max_stack
                    in.readUnsignedShort(); // max_locals
                    method.code = new byte[in.readInt()];
                    in.readFully(method.code);
                    skipFully(in, 8 * in.readUnsignedShort()); // exception_table
                    int codeAttributes = in.readUnsignedShort();
                    for (int c = 0; c < codeAttributes; c++) {
                        String codeAttributeName = (String) pool[in.readUnsignedShort()];
                        int codeAttributeLength = in.readInt();
                        if ("LineNumberTable".equals(codeAttributeName)) {
                            int entries = in.readUnsignedShort();
                            int[] table = Arrays.copyOf(method.lineTable, method.lineTable.length + 2 * entries);
                            for (int e = method.lineTable.length; e < table.length; e++) {
                                table[e] = in.readUnsignedShort();
                            }
                            method.lineTable = table;
                        } else {
                            skipFully(in, codeAttributeLength);
                        }
                    }
                }
                return method;
            }

            /** {@return the source line an instruction belongs to, or -1 if the table does not cover it} */
            int lineOf(int pc) {
                int bestStart = -1;
                int line = -1;
                for (int e = 0; e < lineTable.length; e += 2) {
                    if (lineTable[e] <= pc && lineTable[e] > bestStart) {
                        bestStart = lineTable[e];
                        line = lineTable[e + 1];
                    }
                }
                return line;
            }

            /**
             * {@return one entry per {@code LinearOpMode.sleep(long)} call on {@code line}: its constant
             * argument in milliseconds, or null where the argument is computed}
             */
            List<Long> sleepCallsOnLine(ClassFile classFile, int line, ClassLoader loader) {
                List<Long> calls = new ArrayList<>();
                boolean[] jumpedTo = branchTargets();
                int previousPc = -1;
                int pc = 0;
                while (pc < code.length) {
                    int opcode = code[pc] & 0xff;
                    if ((opcode == JvmInstructions.INVOKEVIRTUAL || opcode == JvmInstructions.INVOKESPECIAL)
                            && lineOf(pc) == line) {
                        String[] ref = classFile.methodRef(u2(pc + 1));
                        if ("sleep".equals(ref[1]) && "(J)V".equals(ref[2]) && isLinearOpMode(ref[0], loader)) {
                            // A call some branch jumps to is reached by more than one path, as in
                            // sleep(fast ? 100 : 1000): the instruction just before it is only one of the
                            // values it can be given, so the duration counts as computed.
                            calls.add(jumpedTo[pc] ? null : constantLoadedAt(classFile, previousPc));
                        }
                    }
                    previousPc = pc;
                    pc += instructionLength(pc);
                }
                return calls;
            }

            /** {@return for each bytecode offset, whether a jump or switch in this method can land there} */
            private boolean[] branchTargets() {
                boolean[] targets = new boolean[code.length];
                int pc = 0;
                while (pc < code.length) {
                    int opcode = code[pc] & 0xff;
                    if ((opcode >= IFEQ && opcode <= JSR) || opcode == IFNULL || opcode == IFNONNULL) {
                        mark(targets, pc + (short) u2(pc + 1));
                    } else if (opcode == GOTO_W || opcode == JSR_W) {
                        mark(targets, pc + s4(pc + 1));
                    } else if (opcode == TABLESWITCH || opcode == LOOKUPSWITCH) {
                        int operands = pc + 1 + ((4 - (pc + 1) % 4) % 4); // padded to a 4-byte boundary
                        mark(targets, pc + s4(operands)); // default
                        if (opcode == TABLESWITCH) {
                            int cases = s4(operands + 8) - s4(operands + 4) + 1;
                            for (int k = 0; k < cases; k++) {
                                mark(targets, pc + s4(operands + 12 + 4 * k));
                            }
                        } else {
                            int pairs = s4(operands + 4);
                            for (int k = 0; k < pairs; k++) {
                                mark(targets, pc + s4(operands + 8 + 8 * k + 4));
                            }
                        }
                    }
                    pc += instructionLength(pc);
                }
                return targets;
            }

            private static void mark(boolean[] targets, int pc) {
                if (pc >= 0 && pc < targets.length) {
                    targets[pc] = true;
                }
            }

            /** {@return the long constant the instruction at {@code pc} pushes, or null if it is not one} */
            private Long constantLoadedAt(ClassFile classFile, int pc) {
                if (pc < 0) {
                    return null;
                }
                int opcode = code[pc] & 0xff;
                if (opcode == LCONST_0) {
                    return 0L;
                }
                if (opcode == LCONST_1) {
                    return 1L;
                }
                if (opcode == LDC2_W && classFile.pool[u2(pc + 1)] instanceof Long) {
                    return (Long) classFile.pool[u2(pc + 1)];
                }
                return null;
            }

            private int instructionLength(int pc) {
                try {
                    return JvmInstructions.length(code, pc, 0);
                } catch (IllegalArgumentException unknown) {
                    throw new IllegalStateException(unknown.getMessage() + " in " + name, unknown);
                }
            }

            private int u2(int at) {
                return ((code[at] & 0xff) << 8) | (code[at + 1] & 0xff);
            }

            private int s4(int at) {
                return ((code[at] & 0xff) << 24) | ((code[at + 1] & 0xff) << 16)
                        | ((code[at + 2] & 0xff) << 8) | (code[at + 3] & 0xff);
            }
        }

        private static boolean isLinearOpMode(String internalName, ClassLoader loader) {
            try {
                return LinearOpMode.class.isAssignableFrom(
                        Class.forName(internalName.replace('/', '.'), false, loader));
            } catch (ClassNotFoundException | LinkageError e) {
                return false;
            }
        }

        private static void skipAttributes(DataInputStream in) throws IOException {
            int attributes = in.readUnsignedShort();
            for (int a = 0; a < attributes; a++) {
                in.readUnsignedShort();
                skipFully(in, in.readInt());
            }
        }

        private static void skipFully(DataInputStream in, int bytes) throws IOException {
            int remaining = bytes;
            while (remaining > 0) {
                int skipped = in.skipBytes(remaining);
                if (skipped <= 0) {
                    throw new IOException("class file ended early");
                }
                remaining -= skipped;
            }
        }
    }
}
