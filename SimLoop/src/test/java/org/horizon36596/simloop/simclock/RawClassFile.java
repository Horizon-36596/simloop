package org.horizon36596.simloop.simclock;

import java.io.ByteArrayOutputStream;
import java.io.DataOutputStream;
import java.io.IOException;
import java.io.UncheckedIOException;
import java.util.ArrayList;
import java.util.List;

/**
 * A tiny hand assembler for class files, so malformed-input and constant-pool-boundary tests need no bytecode
 * library. Entries are appended in call order; {@link #build} writes header, pool, one class, its methods and
 * any raw trailing sections the test wants to corrupt.
 */
final class RawClassFile {
    private final ByteArrayOutputStream pool = new ByteArrayOutputStream();
    private final DataOutputStream poolOut = new DataOutputStream(pool);
    private int nextIndex = 1;
    final List<byte[]> methods = new ArrayList<>();
    byte[] fieldsSection = {0, 0};              // fields_count + fields
    byte[] classAttributesSection = {0, 0};     // attributes_count + attributes
    int major = 49;

    private int add(int slots, IOCall writer) {
        try {
            writer.run(poolOut);
        } catch (IOException e) {
            throw new UncheckedIOException(e);
        }
        int index = nextIndex;
        nextIndex += slots;
        return index;
    }

    private interface IOCall {
        void run(DataOutputStream out) throws IOException;
    }

    int poolCount() {
        return nextIndex;
    }

    int utf8(String s) {
        return add(1, o -> { o.writeByte(1); o.writeUTF(s); });
    }

    int cls(String internalName) {
        int n = utf8(internalName);
        return add(1, o -> { o.writeByte(7); o.writeShort(n); });
    }

    int nat(String name, String desc) {
        int n = utf8(name);
        int d = utf8(desc);
        return add(1, o -> { o.writeByte(12); o.writeShort(n); o.writeShort(d); });
    }

    int methodref(String owner, String name, String desc) {
        int c = cls(owner);
        int t = nat(name, desc);
        return add(1, o -> { o.writeByte(10); o.writeShort(c); o.writeShort(t); });
    }

    int longConst(long v) {
        return add(2, o -> { o.writeByte(5); o.writeLong(v); });
    }

    int raw(int slots, byte[] entry) {
        return add(slots, o -> o.write(entry));
    }

    /** A method with a Code attribute holding {@code code} and {@code codeAttributes} (count included). */
    void method(int access, String name, String desc, int maxStack, int maxLocals, byte[] code, byte[] codeAttributes) {
        int n = utf8(name);
        int d = utf8(desc);
        int codeName = utf8("Code");
        ByteArrayOutputStream m = new ByteArrayOutputStream();
        DataOutputStream o = new DataOutputStream(m);
        try {
            o.writeShort(access);
            o.writeShort(n);
            o.writeShort(d);
            o.writeShort(1);
            o.writeShort(codeName);
            o.writeInt(2 + 2 + 4 + code.length + 2 + codeAttributes.length);
            o.writeShort(maxStack);
            o.writeShort(maxLocals);
            o.writeInt(code.length);
            o.write(code);
            o.writeShort(0); // exception table
            o.write(codeAttributes);
        } catch (IOException e) {
            throw new UncheckedIOException(e);
        }
        methods.add(m.toByteArray());
    }

    byte[] build(String thisName, String superName) {
        int self = cls(thisName);
        int sup = cls(superName);
        return buildWith(self, sup);
    }

    byte[] buildWith(int self, int sup) {
        ByteArrayOutputStream out = new ByteArrayOutputStream();
        DataOutputStream o = new DataOutputStream(out);
        try {
            o.writeInt(0xCAFEBABE);
            o.writeShort(0);
            o.writeShort(major);
            o.writeShort(nextIndex);
            o.write(pool.toByteArray());
            o.writeShort(0x21); // public super
            o.writeShort(self);
            o.writeShort(sup);
            o.writeShort(0);    // interfaces
            o.write(fieldsSection);
            o.writeShort(methods.size());
            for (byte[] m : methods) {
                o.write(m);
            }
            o.write(classAttributesSection);
        } catch (IOException e) {
            throw new UncheckedIOException(e);
        }
        return out.toByteArray();
    }

    static byte[] bytes(int... values) {
        byte[] b = new byte[values.length];
        for (int i = 0; i < values.length; i++) {
            b[i] = (byte) values[i];
        }
        return b;
    }
}
