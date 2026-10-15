package org.horizon36596.simloop.simclock;

import java.util.Arrays;

/**
 * How many bytes each JVM instruction takes (JVMS §6.5), for code that walks a method's bytecode one
 * instruction at a time. SimLoop reads bytecode in two places - {@link ClassFileRewriter}, which moves a
 * team's clock calls onto sim time, and the OpMode runner's {@code SleepDurationReader}, which reads a
 * constant {@code sleep(ms)} argument - and both use this one table, so a mistake in it can only be made once.
 *
 * <p>This is plumbing for those two readers, not something a team's code needs.
 */
public final class JvmInstructions {

    /** {@code new}: creates an object; its operand is a two-byte constant-pool index of a class. */
    public static final int NEW = 0xbb;

    /** {@code invokevirtual}: calls an instance method; operand is a two-byte Methodref index. */
    public static final int INVOKEVIRTUAL = 0xb6;

    /** {@code invokespecial}: calls a constructor, a private method or a {@code super.} method. */
    public static final int INVOKESPECIAL = 0xb7;

    /** {@code invokestatic}: calls a static method; operand is a two-byte Methodref index. */
    public static final int INVOKESTATIC = 0xb8;

    /** {@code invokedynamic}: a lambda, method reference or string concatenation; four operand bytes. */
    public static final int INVOKEDYNAMIC = 0xba;

    /** {@code ldc}: pushes a constant; operand is a ONE-byte constant-pool index. */
    public static final int LDC = 0x12;

    /** {@code ldc_w}: pushes a constant; operand is a two-byte constant-pool index. */
    public static final int LDC_W = 0x13;

    private static final int TABLESWITCH = 0xaa;
    private static final int LOOKUPSWITCH = 0xab;
    private static final int WIDE = 0xc4;
    private static final int IINC = 0x84;

    /**
     * Lengths in bytes, by opcode, of every fixed-length instruction. Zero marks the three variable-length
     * ones ({@code tableswitch}, {@code lookupswitch}, {@code wide}), handled in {@link #length}, and every
     * byte value that is not an opcode at all.
     */
    private static final int[] FIXED_LENGTH = new int[256];

    static {
        fill(0x00, 0x0f, 1);        // nop .. dconst_1
        FIXED_LENGTH[0x10] = 2;     // bipush
        FIXED_LENGTH[0x11] = 3;     // sipush
        FIXED_LENGTH[0x12] = 2;     // ldc
        FIXED_LENGTH[0x13] = 3;     // ldc_w
        FIXED_LENGTH[0x14] = 3;     // ldc2_w
        fill(0x15, 0x19, 2);        // iload .. aload
        fill(0x1a, 0x35, 1);        // iload_0 .. saload
        fill(0x36, 0x3a, 2);        // istore .. astore
        fill(0x3b, 0x83, 1);        // istore_0 .. lxor
        FIXED_LENGTH[0x84] = 3;     // iinc
        fill(0x85, 0x98, 1);        // i2l .. dcmpg
        fill(0x99, 0xa8, 3);        // ifeq .. jsr
        FIXED_LENGTH[0xa9] = 2;     // ret
        fill(0xac, 0xb1, 1);        // ireturn .. return
        fill(0xb2, 0xb8, 3);        // getstatic .. invokestatic
        FIXED_LENGTH[0xb9] = 5;     // invokeinterface
        FIXED_LENGTH[0xba] = 5;     // invokedynamic
        FIXED_LENGTH[0xbb] = 3;     // new
        FIXED_LENGTH[0xbc] = 2;     // newarray
        FIXED_LENGTH[0xbd] = 3;     // anewarray
        FIXED_LENGTH[0xbe] = 1;     // arraylength
        FIXED_LENGTH[0xbf] = 1;     // athrow
        FIXED_LENGTH[0xc0] = 3;     // checkcast
        FIXED_LENGTH[0xc1] = 3;     // instanceof
        FIXED_LENGTH[0xc2] = 1;     // monitorenter
        FIXED_LENGTH[0xc3] = 1;     // monitorexit
        FIXED_LENGTH[0xc5] = 4;     // multianewarray
        FIXED_LENGTH[0xc6] = 3;     // ifnull
        FIXED_LENGTH[0xc7] = 3;     // ifnonnull
        FIXED_LENGTH[0xc8] = 5;     // goto_w
        FIXED_LENGTH[0xc9] = 5;     // jsr_w
    }

    private JvmInstructions() {
    }

    /**
     * {@return the length in bytes of the instruction at {@code pc}, operands included}
     *
     * @param bytes     the array holding the code: a method's code array, or a whole class file
     * @param pc        where the instruction starts, as an index into {@code bytes}
     * @param codeStart where the method's code array starts in {@code bytes} (0 when {@code bytes} is the code
     *                  array); the two switch instructions pad to a four-byte boundary counted from there
     * @throws IllegalArgumentException if the byte at {@code pc} is not a JVM opcode, or is a switch whose
     *                                  size is impossible (its high bound below its low one, a negative
     *                                  number of pairs, or more bytes than {@code bytes} holds)
     */
    public static int length(byte[] bytes, int pc, int codeStart) {
        int opcode = bytes[pc] & 0xff;
        if (opcode == TABLESWITCH || opcode == LOOKUPSWITCH) {
            int offsetInCode = pc - codeStart;
            int operands = pc + 1 + ((4 - (offsetInCode + 1) % 4) % 4); // padded to a 4-byte boundary
            // Worked out in long arithmetic and checked, so a damaged switch cannot come out as zero or
            // negative (a walker would then never move on) or overflow into a plausible small number.
            long length;
            if (opcode == TABLESWITCH) {
                int low = s4(bytes, operands + 4);
                int high = s4(bytes, operands + 8);
                length = high < low ? -1 : operands - pc + 12 + 4L * ((long) high - low + 1);
            } else {
                int pairs = s4(bytes, operands + 4);
                length = pairs < 0 ? -1 : operands - pc + 8 + 8L * pairs;
            }
            if (length <= 0 || pc + length > bytes.length) {
                throw new IllegalArgumentException(String.format("the %s at code offset %d has an impossible size",
                        opcode == TABLESWITCH ? "tableswitch" : "lookupswitch", offsetInCode));
            }
            return (int) length;
        }
        if (opcode == WIDE) {
            return (bytes[pc + 1] & 0xff) == IINC ? 6 : 4;
        }
        if (FIXED_LENGTH[opcode] == 0) {
            throw new IllegalArgumentException(String.format(
                    "SimLoop does not recognise opcode 0x%02x at code offset %d", opcode, pc - codeStart));
        }
        return FIXED_LENGTH[opcode];
    }

    private static int s4(byte[] bytes, int at) {
        return ((bytes[at] & 0xff) << 24) | ((bytes[at + 1] & 0xff) << 16)
                | ((bytes[at + 2] & 0xff) << 8) | (bytes[at + 3] & 0xff);
    }

    private static void fill(int from, int to, int length) {
        Arrays.fill(FIXED_LENGTH, from, to + 1, length);
    }
}
