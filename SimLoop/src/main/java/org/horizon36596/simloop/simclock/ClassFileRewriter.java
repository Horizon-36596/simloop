package org.horizon36596.simloop.simclock;

import java.io.ByteArrayInputStream;
import java.io.ByteArrayOutputStream;
import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;

/**
 * Rewrites one compiled class so that its clock reads and waits use the sim clock instead of the real one.
 * Hand-written against the class-file format (JVMS §4) so SimLoop needs no bytecode library.
 *
 * <h2>What it changes</h2>
 * <ul>
 *   <li>{@code System.nanoTime()} and {@code System.currentTimeMillis()} call {@link SimClockHooks} instead,
 *       and so do Android's {@code SystemClock.uptimeMillis()}, {@code elapsedRealtime()},
 *       {@code elapsedRealtimeNanos()} and {@code sleep(ms)} (matched by name: SimLoop needs no Android
 *       classes for this).</li>
 *   <li>{@code Thread.sleep(long)} and {@code Thread.sleep(long, int)} call {@code SimClockHooks.threadSleep},
 *       unless the class the call names declares a static {@code sleep} of its own that hides Thread's.</li>
 *   <li>{@code unit.sleep(n)} on a {@code java.util.concurrent.TimeUnit} calls
 *       {@code SimClockHooks.timeUnitSleep(unit, n)}.</li>
 *   <li>{@code new ElapsedTime(...)}, with any of its three constructors, makes a {@link SimClockElapsedTime};
 *       a class that {@code extends ElapsedTime} extends {@code SimClockElapsedTime} instead.</li>
 *   <li>{@code sleep(ms)} and {@code idle()} called on a {@code LinearOpMode} call
 *       {@code SimClockHooks.opModeSleep} and {@code SimClockHooks.opModeIdle}.</li>
 *   <li>Method references and lambdas that name any of those ({@code System::nanoTime},
 *       {@code ElapsedTime::new}) follow, because they point at the same constant-pool entries.</li>
 * </ul>
 * A method reference to a {@code LinearOpMode}'s {@code sleep} or {@code idle} ({@code this::sleep}), or to a
 * {@code TimeUnit}'s {@code sleep}, cannot follow - it would need a different kind of handle - so it is refused
 * with the class, method and line.
 *
 * <h2>What it names but leaves on the real clock</h2>
 * A few clock reads have no sim-time answer the rewrite can give: the FTC SDK's own {@code ElapsedTime}
 * subclass {@code Deadline} (constructed, or extended by a team class), and the real date and time
 * ({@code java.time}'s {@code now()} methods and {@code Clock.system...()}, {@code new Date()},
 * {@code Calendar.getInstance()}). Each is left alone and listed in {@link Result#realClockUses} with its
 * place, so the run can warn about it.
 *
 * <h2>Why it never has to recompute anything</h2>
 * Every change keeps every instruction the same length, so no code offset moves: branch targets, exception
 * tables, line tables and the {@code StackMapTable} frames the verifier checks all stay valid untouched. The
 * changes are of three kinds only:
 * <ol>
 *   <li><b>Constant-pool entries edited in place.</b> The {@code Methodref} for {@code System.nanoTime} is
 *       pointed at {@code SimClockHooks.nanoTime}; every instruction and method handle that used it follows.</li>
 *   <li><b>Constant-pool entries appended</b> for the names those edits need, which moves only what comes
 *       after the pool, never inside a method.</li>
 *   <li><b>Two-byte operands patched</b> in the code, plus one opcode byte: {@code new ElapsedTime} becomes
 *       {@code new SimClockElapsedTime} (the {@code Class} entry itself is shared with casts and field types,
 *       so it cannot be edited in place), and {@code invokevirtual MyAuto.sleep(J)V} becomes
 *       {@code invokestatic SimClockHooks.opModeSleep(LinearOpMode, long)}. Both leave the operand stack
 *       exactly as it was (the receiver becomes the first argument), which is why no frame changes.</li>
 * </ol>
 *
 * <p>A class that needs none of this comes back as the very same array. A file that is not a class file,
 * is cut short, is damaged (a count, length or instruction size that does not fit the file, a method
 * reference whose descriptor is not a method's), or uses a constant-pool entry this reader does not know
 * fails with an {@link IllegalArgumentException} naming the class. Every such check is made before the
 * reader moves on, so damaged input fails in time proportional to its size and never hangs or fills the
 * heap. A class it can read but must refuse fails with an {@link IllegalStateException} saying what to
 * change.
 */
final class ClassFileRewriter {

    static final String SYSTEM = "java/lang/System";
    static final String THREAD = "java/lang/Thread";
    static final String ELAPSED_TIME = "com/qualcomm/robotcore/util/ElapsedTime";
    static final String LINEAR_OP_MODE = "com/qualcomm/robotcore/eventloop/opmode/LinearOpMode";
    static final String TIME_UNIT = "java/util/concurrent/TimeUnit";
    /** Android's clock, named only as a string: SimLoop's main code imports nothing from Android. */
    static final String SYSTEM_CLOCK = "android/os/SystemClock";
    /** The FTC SDK's only ElapsedTime subclass (RobotCore 12.0.0), which is SDK code and never rewritten. */
    static final String DEADLINE = "org/firstinspires/ftc/robotcore/internal/system/Deadline";
    static final String HOOKS = SimClockHooks.class.getName().replace('.', '/');
    static final String SIM_ELAPSED_TIME = SimClockElapsedTime.class.getName().replace('.', '/');

    private static final String OP_MODE_SLEEP_DESCRIPTOR = "(L" + LINEAR_OP_MODE + ";J)V";
    private static final String OP_MODE_IDLE_DESCRIPTOR = "(L" + LINEAR_OP_MODE + ";)V";
    private static final String TIME_UNIT_SLEEP_DESCRIPTOR = "(L" + TIME_UNIT + ";J)V";

    /** RobotCore 12.0.0's three {@code ElapsedTime} constructors, which {@link SimClockElapsedTime} repeats. */
    private static final List<String> ELAPSED_TIME_CONSTRUCTORS = Arrays.asList(
            "()V", "(J)V", "(L" + ELAPSED_TIME + "$Resolution;)V");

    private static final int ACC_STATIC = 0x0008;

    /** The newest class-file version this reader has been checked against: 69 is Java 25. */
    private static final int NEWEST_MAJOR_VERSION = 69;

    /** Answers questions about the class hierarchy of a call's owner without loading a class. */
    interface Hierarchy {
        /**
         * @param internalName a class name as the class file spells it ({@code com/example/MyAuto})
         * @param ancestor     the class it might be or extend, spelled the same way
         * @return true if it is that class or extends it
         */
        boolean isOrExtends(String internalName, String ancestor);

        /**
         * {@return true if {@code internalName}, or a class between it and {@code ancestor} (not
         * {@code ancestor} itself), declares a static method {@code name} {@code descriptor}}: one that hides
         * the ancestor's, so a call qualified by {@code internalName} runs it and not the ancestor's.
         * Default: none does.
         *
         * @param internalName the owner a call names, which {@link #isOrExtends} said extends {@code ancestor}
         * @param ancestor     the class whose static method might be hidden
         * @param name         the method's name
         * @param descriptor   the method's descriptor
         */
        default boolean declaresBelow(String internalName, String ancestor, String name, String descriptor) {
            return false;
        }
    }

    /** What one rewrite did. */
    static final class Result {
        /** The class file to define: the input array itself when nothing needed changing. */
        final byte[] bytes;

        /** How many instructions and method handles now use the sim clock. */
        final int callSites;

        /**
         * One sentence for each place the class reads a clock the rewrite leaves real (see the class javadoc),
         * starting with the place ({@code com.example.MyAuto.runOpMode (MyAuto.java:12)}); empty for most.
         */
        final List<String> realClockUses;

        private Result(byte[] bytes, int callSites, List<String> realClockUses) {
            this.bytes = bytes;
            this.callSites = callSites;
            this.realClockUses = realClockUses;
        }
    }

    /**
     * Rewrites one class.
     *
     * @param className the class's binary name ({@code com.example.MyAuto}), used in messages
     * @param original  the class file's bytes; never modified
     * @param hierarchy answers subclass questions for the owners of calls
     * @return the rewritten bytes and a count of what changed
     * @throws IllegalArgumentException if the bytes are not a class file this reader understands
     * @throws IllegalStateException    if the class uses a clock call that cannot be moved to sim time
     */
    Result rewrite(String className, byte[] original, Hierarchy hierarchy) {
        try {
            return new Rewrite(className, new ParsedClass(original), hierarchy).run();
        } catch (IndexOutOfBoundsException truncated) {
            throw new IllegalArgumentException("SimLoop cannot read the class file of " + className + ": it ends"
                    + " early or an entry points outside it (" + truncated.getMessage() + ")", truncated);
        } catch (IllegalArgumentException malformed) {
            throw new IllegalArgumentException("SimLoop cannot read the class file of " + className + ": "
                    + malformed.getMessage(), malformed);
        }
    }

    /**
     * {@return the internal name of the superclass {@code bytes} declares, or null for {@code java/lang/Object}},
     * read without loading anything.
     *
     * @param className the class's name, for messages
     * @param bytes     its class file
     * @throws IllegalArgumentException if the bytes are not a class file this reader understands
     */
    static String superclassOf(String className, byte[] bytes) {
        try {
            requireClassFileHeader(bytes);
            ConstantPool pool = new ConstantPool(bytes);
            int superClass = u2(bytes, pool.end + 4);
            return superClass == 0 ? null : pool.className(superClass);
        } catch (IndexOutOfBoundsException | IllegalArgumentException unreadable) {
            throw new IllegalArgumentException("SimLoop cannot read the class file of " + className
                    + " to see what it extends: " + unreadable.getMessage(), unreadable);
        }
    }

    /**
     * {@return true if the class file {@code bytes} declares a static method {@code name} {@code descriptor}},
     * read without loading anything.
     *
     * @param className  the class's name, for messages
     * @param bytes      its class file
     * @param name       the method's name
     * @param descriptor the method's descriptor
     * @throws IllegalArgumentException if the bytes are not a class file this reader understands
     */
    static boolean declaresStaticMethod(String className, byte[] bytes, String name, String descriptor) {
        try {
            requireClassFileHeader(bytes);
            ConstantPool pool = new ConstantPool(bytes);
            int pos = pool.end + 6;                         // access_flags, this_class, super_class
            pos += 2 + 2 * u2(bytes, pos);                  // interfaces
            int fields = u2(bytes, pos);
            pos += 2;
            for (int f = 0; f < fields; f++) {
                pos = ParsedClass.skipAttributes(bytes, pos + 6);
            }
            int methods = u2(bytes, pos);
            pos += 2;
            for (int m = 0; m < methods; m++) {
                boolean isStatic = (u2(bytes, pos) & ACC_STATIC) != 0;
                if (isStatic && name.equals(pool.utf8(u2(bytes, pos + 2)))
                        && descriptor.equals(pool.utf8(u2(bytes, pos + 4)))) {
                    return true;
                }
                pos = ParsedClass.skipAttributes(bytes, pos + 6);
            }
            return false;
        } catch (IndexOutOfBoundsException | IllegalArgumentException unreadable) {
            throw new IllegalArgumentException("SimLoop cannot read the class file of " + className
                    + " to see what it declares: " + unreadable.getMessage(), unreadable);
        }
    }

    /** One rewrite in progress: the parsed input, the edits decided so far, and the new pool entries. */
    private static final class Rewrite {
        private final String className;
        private final ParsedClass in;
        private final ConstantPool pool;
        private final Hierarchy hierarchy;

        /** Constant-pool index of an edited Methodref -> its new {class index, name-and-type index}. */
        private final Map<Integer, int[]> editedMethodrefs = new LinkedHashMap<>();

        /** Byte offset in the input -> new value, for every patched byte after the constant pool. */
        private final Map<Integer, Integer> patchedBytes = new HashMap<>();

        /** Constant-pool index of a Methodref that reads a real clock -> what it reads, for the notes. */
        private final Map<Integer, String> realClockRefs = new LinkedHashMap<>();

        /** The notes for {@link Result#realClockUses}, once each, in the order found. */
        private final List<String> realClockUses = new ArrayList<>();

        /** The {@link #realClockRefs} some instruction calls. */
        private final Set<Integer> calledRealClockRefs = new HashSet<>();

        private int callSites;

        Rewrite(String className, ParsedClass in, Hierarchy hierarchy) {
            this.className = className;
            this.in = in;
            this.pool = in.pool;
            this.hierarchy = hierarchy;
        }

        Result run() {
            refuseMethodHandlesToOpModeWaits();
            editClockMethodrefs();
            reparentElapsedTimeSubclass();
            for (MethodCode method : in.methods) {
                patchCode(method);
            }
            countMethodHandlesThatFollowEdits();
            noteRealClockRefsNoCallUses();
            if (editedMethodrefs.isEmpty() && patchedBytes.isEmpty()) {
                return new Result(in.bytes, 0, realClockUses);
            }
            return new Result(assemble(), callSites, realClockUses);
        }

        // ---- step 1: refuse what cannot be moved -------------------------------------------------------

        /**
         * {@code this::sleep} compiles to a method handle that invokes {@code sleep} virtually, and a virtual
         * handle cannot be pointed at a static hook without changing its kind and the lambda's shape.
         */
        private void refuseMethodHandlesToOpModeWaits() {
            for (int index = 1; index < pool.count; index++) {
                if (pool.tag[index] != ConstantPool.METHOD_HANDLE) {
                    continue;
                }
                int ref = pool.second[index];
                if (pool.tag[ref] == ConstantPool.METHODREF && virtualWaitHookFor(ref) != null) {
                    String method = pool.memberName(ref);
                    boolean timeUnit = TIME_UNIT.equals(pool.memberOwner(ref));
                    String lambda = timeUnit ? "n -> unit.sleep(n)"
                            : "sleep".equals(method) ? "ms -> sleep(ms)" : "() -> idle()";
                    throw new IllegalStateException("SimLoop cannot move the method reference ::" + method
                            + " in " + usesOfHandle(index) + " onto sim time: a reference to "
                            + (timeUnit ? "TimeUnit." : "LinearOpMode.") + method + " cannot be redirected the way"
                            + " a call can. Write it as a lambda (" + lambda + ") and it will be.");
                }
            }
        }

        /** {@return where method handle {@code handleIndex} is used: "Class.method (File.java:12)", or the class} */
        private String usesOfHandle(int handleIndex) {
            List<String> places = new ArrayList<>();
            for (MethodCode method : in.methods) {
                for (int pc = method.codeStart; pc < method.codeEnd(); pc = method.next(in.bytes, pc)) {
                    if (usesHandle(pc, handleIndex)) {
                        places.add(className + "." + method.name + " (" + in.sourceFile + ":"
                                + method.lineOf(pc - method.codeStart) + ")");
                    }
                }
            }
            return places.isEmpty() ? className : String.join(", ", places);
        }

        private boolean usesHandle(int pc, int handleIndex) {
            int opcode = in.bytes[pc] & 0xff;
            if (opcode == JvmInstructions.INVOKEDYNAMIC) {
                int bootstrap = pool.first[u2(in.bytes, pc + 1)];
                return bootstrap < in.bootstrapArguments.size()
                        && contains(in.bootstrapArguments.get(bootstrap), handleIndex);
            }
            if (opcode == JvmInstructions.LDC) {
                return (in.bytes[pc + 1] & 0xff) == handleIndex;
            }
            return opcode == JvmInstructions.LDC_W && u2(in.bytes, pc + 1) == handleIndex;
        }

        // ---- step 2: Methodrefs edited in place ------------------------------------------------------

        /** Points the static clock calls and ElapsedTime's constructors at the sim versions. */
        private void editClockMethodrefs() {
            for (int ref = 1; ref < pool.count; ref++) {
                if (pool.tag[ref] != ConstantPool.METHODREF) {
                    continue;
                }
                String owner = pool.memberOwner(ref);
                String name = pool.memberName(ref);
                String descriptor = pool.memberDescriptor(ref);
                if (!descriptor.startsWith("(")) {
                    // A damaged file, not a class to refuse: say so rather than name a nonsense constructor.
                    throw new IllegalArgumentException("constant-pool entry " + ref + " is a Methodref whose"
                            + " descriptor \"" + descriptor + "\" is not a method descriptor");
                }
                String systemClockHook = SYSTEM_CLOCK.equals(owner) ? systemClockHookFor(name, descriptor) : null;
                if (SYSTEM.equals(owner) && isClockRead(name, descriptor)) {
                    editedMethodrefs.put(ref, new int[] {pool.classRef(HOOKS), pool.second[ref]});
                } else if (systemClockHook != null) {
                    editedMethodrefs.put(ref, new int[] {pool.classRef(HOOKS), pool.nameAndType(systemClockHook,
                            descriptor)});
                } else if ("sleep".equals(name) && isThreadSleep(descriptor) && hierarchy.isOrExtends(owner, THREAD)
                        && !hierarchy.declaresBelow(owner, THREAD, name, descriptor)) {
                    // Thread.sleep, however it is qualified (Worker.sleep, ForkJoinWorkerThread.sleep), unless
                    // the qualifier hides it with a static sleep of its own, which is the team's code to run.
                    editedMethodrefs.put(ref, new int[] {pool.classRef(HOOKS), pool.nameAndType("threadSleep",
                            descriptor)});
                } else if (realClockRead(owner, name, descriptor) != null) {
                    realClockRefs.put(ref, realClockRead(owner, name, descriptor));
                } else if (ELAPSED_TIME.equals(owner) && "<init>".equals(name)) {
                    if (!ELAPSED_TIME_CONSTRUCTORS.contains(descriptor)) {
                        throw new IllegalStateException("SimLoop cannot move " + className + " onto sim time: it"
                                + " calls an ElapsedTime constructor " + descriptor + " that the RobotCore 12.0.0"
                                + " ElapsedTime, and so SimClockElapsedTime, does not have.");
                    }
                    editedMethodrefs.put(ref, new int[] {pool.classRef(SIM_ELAPSED_TIME), pool.second[ref]});
                }
            }
        }

        private static boolean isClockRead(String name, String descriptor) {
            return "()J".equals(descriptor) && ("nanoTime".equals(name) || "currentTimeMillis".equals(name));
        }

        private static boolean isThreadSleep(String descriptor) {
            return "(J)V".equals(descriptor) || "(JI)V".equals(descriptor);
        }

        /**
         * {@return what a call of {@code owner.name descriptor} does with a clock the rewrite leaves real, as
         * the end of a sentence that starts with its place; or null for any other call}
         */
        private static String realClockRead(String owner, String name, String descriptor) {
            if (DEADLINE.equals(owner) && "<init>".equals(name)) {
                return "constructs the FTC SDK's Deadline timer (" + DEADLINE.replace('/', '.') + "), which keeps"
                        + " the real clock: SimLoop never rewrites the SDK, so in the sim it expires by laptop time,"
                        + " not sim time. Use an ElapsedTime instead.";
            }
            // now(Clock) reads the clock it is given, which may be a fixed one; now() and now(ZoneId) read the
            // real one.
            boolean now = owner.startsWith("java/time/") && "now".equals(name)
                    && !descriptor.startsWith("(Ljava/time/Clock;)");
            boolean systemClock = "java/time/Clock".equals(owner) && name.startsWith("system");
            boolean newDate = "java/util/Date".equals(owner) && "<init>".equals(name) && "()V".equals(descriptor);
            boolean calendar = "java/util/Calendar".equals(owner) && "getInstance".equals(name);
            if (!now && !systemClock && !newDate && !calendar) {
                return null;
            }
            String call = newDate ? "new java.util.Date()" : owner.replace('/', '.') + "." + name + "()";
            return "reads the real date and time (" + call + "), which SimLoop does not move: it is different on"
                    + " every run. System.currentTimeMillis() reads the sim clock, from a fixed date.";
        }

        /** {@return the hook for static {@code android.os.SystemClock.name descriptor}, or null if none} */
        private static String systemClockHookFor(String name, String descriptor) {
            if ("()J".equals(descriptor)) {
                if ("uptimeMillis".equals(name) || "elapsedRealtime".equals(name)
                        || "elapsedRealtimeNanos".equals(name)) {
                    return name;                            // SimClockHooks has a method of each name
                }
                return null;
            }
            return "sleep".equals(name) && "(J)V".equals(descriptor) ? "systemClockSleep" : null;
        }

        // ---- step 3: the superclass -------------------------------------------------------------------

        private void reparentElapsedTimeSubclass() {
            if (in.superClass != 0 && ELAPSED_TIME.equals(pool.className(in.superClass))) {
                patchU2(in.superClassOffset, pool.classRef(SIM_ELAPSED_TIME));
                callSites++;
            } else if (in.superClass != 0 && DEADLINE.equals(pool.className(in.superClass))) {
                noteRealClock(className + " (" + in.sourceFile + ")", "extends the FTC SDK's Deadline timer ("
                        + DEADLINE.replace('/', '.') + "), which keeps the real clock: SimLoop never rewrites the"
                        + " SDK, so in the sim it expires by laptop time, not sim time. Extend ElapsedTime"
                        + " instead.");
            }
        }

        private void noteRealClock(String place, String what) {
            String note = place + " " + what;
            if (!realClockUses.contains(note)) {
                realClockUses.add(note);
            }
        }

        /** A real-clock read no instruction calls (only a method handle, {@code Instant::now}): name the class. */
        private void noteRealClockRefsNoCallUses() {
            for (Map.Entry<Integer, String> read : realClockRefs.entrySet()) {
                if (!calledRealClockRefs.contains(read.getKey())) {
                    noteRealClock(className, read.getValue());
                }
            }
        }

        // ---- step 4: same-width patches in each method's code ----------------------------------------

        private void patchCode(MethodCode method) {
            for (int pc = method.codeStart; pc < method.codeEnd(); pc = method.next(in.bytes, pc)) {
                int opcode = in.bytes[pc] & 0xff;
                if (opcode == JvmInstructions.NEW) {
                    patchNewElapsedTime(pc);
                } else if (opcode == JvmInstructions.INVOKEVIRTUAL || opcode == JvmInstructions.INVOKESPECIAL) {
                    patchVirtualWait(pc); // an edited ElapsedTime <init> call is counted with its new, not here
                } else if (opcode == JvmInstructions.INVOKESTATIC) {
                    countIfEdited(u2(in.bytes, pc + 1));
                }
                if (opcode == JvmInstructions.INVOKESTATIC || opcode == JvmInstructions.INVOKESPECIAL
                        || opcode == JvmInstructions.INVOKEVIRTUAL) {
                    int ref = u2(in.bytes, pc + 1);
                    String read = realClockRefs.get(ref);
                    if (read != null) {
                        calledRealClockRefs.add(ref);
                        noteRealClock(className + "." + method.name + " (" + in.sourceFile + ":"
                                + method.lineOf(pc - method.codeStart) + ")", read);
                    }
                }
            }
        }

        /** {@code new ElapsedTime} -> {@code new SimClockElapsedTime}: two operand bytes. */
        private void patchNewElapsedTime(int pc) {
            if (ELAPSED_TIME.equals(pool.className(u2(in.bytes, pc + 1)))) {
                patchU2(pc + 1, pool.classRef(SIM_ELAPSED_TIME));
                callSites++;
            }
        }

        /**
         * {@code invokevirtual MyAuto.sleep(J)V} -> {@code invokestatic SimClockHooks.opModeSleep(LinearOpMode, J)V},
         * and {@code invokevirtual TimeUnit.sleep(J)V} ->
         * {@code invokestatic SimClockHooks.timeUnitSleep(TimeUnit, J)V}: one opcode byte and two operand bytes.
         * The stack holds (receiver, arguments) either way.
         */
        private void patchVirtualWait(int pc) {
            int ref = u2(in.bytes, pc + 1);
            if (pool.tag[ref] != ConstantPool.METHODREF) {
                return; // an InterfaceMethodref, which is never a LinearOpMode or TimeUnit method
            }
            String[] hook = virtualWaitHookFor(ref);
            if (hook != null) {
                patchedBytes.put(pc, JvmInstructions.INVOKESTATIC);
                patchU2(pc + 1, pool.methodref(HOOKS, hook[0], hook[1]));
                callSites++;
            }
        }

        /**
         * {@return the {name, descriptor} of the hook for instance-method Methodref {@code ref}, or null if it is
         * not a wait the rewrite moves}
         */
        private String[] virtualWaitHookFor(int ref) {
            String name = pool.memberName(ref);
            String descriptor = pool.memberDescriptor(ref);
            if (TIME_UNIT.equals(pool.memberOwner(ref))) {
                return "sleep".equals(name) && "(J)V".equals(descriptor)
                        ? new String[] {"timeUnitSleep", TIME_UNIT_SLEEP_DESCRIPTOR} : null;
            }
            boolean sleep = "sleep".equals(name) && "(J)V".equals(descriptor);
            boolean idle = "idle".equals(name) && "()V".equals(descriptor);
            if ((!sleep && !idle) || !hierarchy.isOrExtends(pool.memberOwner(ref), LINEAR_OP_MODE)) {
                return null;
            }
            return sleep ? new String[] {"opModeSleep", OP_MODE_SLEEP_DESCRIPTOR}
                    : new String[] {"opModeIdle", OP_MODE_IDLE_DESCRIPTOR};
        }

        private void countIfEdited(int ref) {
            if (editedMethodrefs.containsKey(ref)) {
                callSites++;
            }
        }

        /** {@code System::nanoTime}, {@code ElapsedTime::new}: the handle points at an edited Methodref. */
        private void countMethodHandlesThatFollowEdits() {
            for (int index = 1; index < pool.count; index++) {
                if (pool.tag[index] == ConstantPool.METHOD_HANDLE && editedMethodrefs.containsKey(pool.second[index])) {
                    callSites++;
                }
            }
        }

        private void patchU2(int offset, int value) {
            patchedBytes.put(offset, value >> 8);
            patchedBytes.put(offset + 1, value & 0xff);
        }

        // ---- step 5: put the new class file together -------------------------------------------------

        /** [header and pool, Methodrefs edited] [appended pool entries] [the rest, patched]. */
        private byte[] assemble() {
            byte[] appended = pool.appendedBytes();
            if (pool.nextIndex > 0xffff) {
                throw new IllegalStateException("SimLoop cannot move " + className + " onto sim time: its constant"
                        + " pool is already full (65535 entries), so there is no room for the sim-clock names.");
            }
            byte[] out = new byte[in.bytes.length + appended.length];
            System.arraycopy(in.bytes, 0, out, 0, pool.end);
            System.arraycopy(appended, 0, out, pool.end, appended.length);
            System.arraycopy(in.bytes, pool.end, out, pool.end + appended.length, in.bytes.length - pool.end);
            out[8] = (byte) (pool.nextIndex >> 8);
            out[9] = (byte) pool.nextIndex;
            for (Map.Entry<Integer, int[]> edit : editedMethodrefs.entrySet()) {
                int at = pool.offset[edit.getKey()];
                out[at + 1] = (byte) (edit.getValue()[0] >> 8);
                out[at + 2] = (byte) edit.getValue()[0];
                out[at + 3] = (byte) (edit.getValue()[1] >> 8);
                out[at + 4] = (byte) edit.getValue()[1];
            }
            for (Map.Entry<Integer, Integer> patch : patchedBytes.entrySet()) {
                out[patch.getKey() + appended.length] = (byte) (int) patch.getValue(); // all lie after the pool
            }
            return out;
        }
    }

    // ---- reading the class file ---------------------------------------------------------------------

    /** The parts of a class file the rewrite reads, with the byte offsets it patches. */
    private static final class ParsedClass {
        final byte[] bytes;
        final ConstantPool pool;
        final int superClassOffset;
        final int superClass;
        final List<MethodCode> methods = new ArrayList<>();
        /** For each BootstrapMethods entry, the constant-pool indexes of its arguments. */
        final List<int[]> bootstrapArguments = new ArrayList<>();
        String sourceFile = "unknown source";

        ParsedClass(byte[] bytes) {
            this.bytes = bytes;
            requireClassFileHeader(bytes);
            pool = new ConstantPool(bytes);
            int pos = pool.end;
            pos += 2;                                   // access_flags
            pos += 2;                                   // this_class
            superClassOffset = pos;
            superClass = u2(bytes, pos);
            pos += 2;
            pos += 2 + 2 * u2(bytes, pos);              // interfaces
            int fields = u2(bytes, pos);
            pos += 2;
            for (int f = 0; f < fields; f++) {
                pos = skipAttributes(bytes, pos + 6);   // access, name, descriptor, then attributes
            }
            int methodCount = u2(bytes, pos);
            pos += 2;
            for (int m = 0; m < methodCount; m++) {
                pos = readMethod(pos);
            }
            readClassAttributes(pos);
        }

        private int readMethod(int pos) {
            String name = pool.utf8(u2(bytes, pos + 2));
            int attributes = u2(bytes, pos + 6);
            pos += 8;
            for (int a = 0; a < attributes; a++) {
                int length = attributeLength(bytes, pos);
                if ("Code".equals(pool.utf8(u2(bytes, pos)))) {
                    methods.add(MethodCode.read(name, bytes, pos + 6, length, pool));
                }
                pos += 6 + length;
            }
            return pos;
        }

        private void readClassAttributes(int pos) {
            int attributes = u2(bytes, pos);
            pos += 2;
            for (int a = 0; a < attributes; a++) {
                String name = pool.utf8(u2(bytes, pos));
                int length = attributeLength(bytes, pos);
                int body = pos + 6;
                if ("SourceFile".equals(name)) {
                    sourceFile = pool.utf8(u2(bytes, body));
                } else if ("BootstrapMethods".equals(name) && readBootstrapMethods(body) != body + length) {
                    throw new IllegalArgumentException("its BootstrapMethods attribute does not fill the " + length
                            + " bytes it claims");
                }
                pos = body + length;
            }
            if (pos != bytes.length) {
                throw new IllegalArgumentException("it has " + (bytes.length - pos) + " bytes after its last"
                        + " attribute, so it is not a class file SimLoop can read");
            }
        }

        /** {@return the offset just past the BootstrapMethods entries starting at {@code pos}} */
        private int readBootstrapMethods(int pos) {
            int count = u2(bytes, pos);
            pos += 2;
            for (int b = 0; b < count; b++) {
                int arguments = u2(bytes, pos + 2);
                int[] indexes = new int[arguments];
                for (int i = 0; i < arguments; i++) {
                    indexes[i] = u2(bytes, pos + 4 + 2 * i);
                }
                bootstrapArguments.add(indexes);
                pos += 4 + 2 * arguments;
            }
            return pos;
        }

        private static int skipAttributes(byte[] bytes, int pos) {
            int attributes = u2(bytes, pos);
            pos += 2;
            for (int a = 0; a < attributes; a++) {
                pos += 6 + attributeLength(bytes, pos);
            }
            return pos;
        }
    }

    /** Where one method's bytecode is in the class file, and its line table for messages. */
    private static final class MethodCode {
        final String name;
        final int codeStart;
        final int codeLength;
        /** Pairs of (start_pc, line_number). */
        final int[] lineTable;

        private MethodCode(String name, int codeStart, int codeLength, int[] lineTable) {
            this.name = name;
            this.codeStart = codeStart;
            this.codeLength = codeLength;
            this.lineTable = lineTable;
        }

        /**
         * Reads a Code attribute whose body starts at {@code body} (just after its name and length) and is
         * {@code attributeLength} bytes long.
         */
        static MethodCode read(String name, byte[] bytes, int body, int attributeLength, ConstantPool pool) {
            int codeLength = u4(bytes, body + 4);
            int codeStart = body + 8;
            // max_stack, max_locals and code_length (8 bytes) come before the code, and the exception-table
            // and attribute counts (4 bytes) after it, all inside the attribute.
            if (codeLength <= 0 || 8L + codeLength + 4 > attributeLength) {
                throw new IllegalArgumentException("the code of " + name + " runs past the end of its Code"
                        + " attribute");
            }
            int pos = codeStart + codeLength;
            pos += 2 + 8 * u2(bytes, pos);                  // exception_table
            int attributes = u2(bytes, pos);
            pos += 2;
            int[] lines = new int[0];
            for (int a = 0; a < attributes; a++) {
                int length = attributeLength(bytes, pos);
                if ("LineNumberTable".equals(pool.utf8(u2(bytes, pos)))) {
                    int entries = u2(bytes, pos + 6);
                    if (length != 2 + 4 * entries) {
                        throw new IllegalArgumentException("a LineNumberTable of " + name + " claims " + entries
                                + " entries in " + length + " bytes");
                    }
                    int[] more = Arrays.copyOf(lines, lines.length + 2 * entries);
                    for (int e = 0; e < 2 * entries; e++) {
                        more[lines.length + e] = u2(bytes, pos + 8 + 2 * e);
                    }
                    lines = more;
                }
                pos += 6 + length;
            }
            return new MethodCode(name, codeStart, codeLength, lines);
        }

        int codeEnd() {
            return codeStart + codeLength;
        }

        /**
         * {@return where the instruction after the one at {@code pc} starts}
         *
         * @throws IllegalArgumentException if that instruction runs past the end of this method's code, or its
         *                                  opcode or size is impossible
         */
        int next(byte[] bytes, int pc) {
            int next = pc + JvmInstructions.length(bytes, pc, codeStart);
            if (next > codeEnd()) {
                throw new IllegalArgumentException("an instruction of " + name + " at code offset "
                        + (pc - codeStart) + " runs past the end of its code");
            }
            return next;
        }

        /** {@return the source line of the instruction at {@code pcInCode}, or -1 if unknown} */
        int lineOf(int pcInCode) {
            int bestStart = -1;
            int line = -1;
            for (int e = 0; e < lineTable.length; e += 2) {
                if (lineTable[e] <= pcInCode && lineTable[e] > bestStart) {
                    bestStart = lineTable[e];
                    line = lineTable[e + 1];
                }
            }
            return line;
        }
    }

    /**
     * The constant pool as read (tags, offsets, the one or two indexes each entry holds, the text of each
     * Utf8), plus the entries a rewrite appends to it.
     */
    private static final class ConstantPool {
        static final int UTF8 = 1;
        static final int CLASS = 7;
        static final int METHODREF = 10;
        static final int NAME_AND_TYPE = 12;
        static final int METHOD_HANDLE = 15;

        /** One past the last index read from the file: entries 1 .. count-1 are the file's own. */
        final int count;
        final int[] tag;
        final int[] offset;
        /** The first index an entry holds (class name, owner class, kind of a handle, bootstrap index). */
        final int[] first;
        /** The second index an entry holds (name-and-type, descriptor, handle's reference). */
        final int[] second;
        final String[] utf8;
        /** Byte offset just past the pool. */
        final int end;

        /** The next index an appended entry gets. */
        int nextIndex;
        private final ByteArrayOutputStream appended = new ByteArrayOutputStream();
        private final DataOutputStream appendedOut = new DataOutputStream(appended);
        private final Map<String, Integer> appendedIndex = new HashMap<>();

        ConstantPool(byte[] bytes) {
            count = u2(bytes, 8);
            nextIndex = count;
            tag = new int[count];
            offset = new int[count];
            first = new int[count];
            second = new int[count];
            utf8 = new String[count];
            int pos = 10;
            for (int i = 1; i < count; i++) {
                offset[i] = pos;
                tag[i] = bytes[pos] & 0xff;
                pos = readEntry(bytes, i, pos);
                if (tag[i] == 5 || tag[i] == 6) {
                    i++;                                     // Long and Double take two slots
                }
            }
            end = pos;
        }

        private int readEntry(byte[] bytes, int i, int pos) {
            switch (tag[i]) {
                case UTF8:
                    int length = u2(bytes, pos + 1);
                    utf8[i] = decodeUtf8(bytes, pos + 1, length);
                    return pos + 3 + length;
                case 3: case 4:                              // Integer, Float
                    return pos + 5;
                case 5: case 6:                              // Long, Double
                    return pos + 9;
                case CLASS: case 8: case 16: case 19: case 20: // Class, String, MethodType, Module, Package
                    first[i] = u2(bytes, pos + 1);
                    return pos + 3;
                case 9: case METHODREF: case 11: case NAME_AND_TYPE: case 17: case 18:
                    first[i] = u2(bytes, pos + 1);           // Field/Method/InterfaceMethodref, NameAndType,
                    second[i] = u2(bytes, pos + 3);          // Dynamic, InvokeDynamic
                    return pos + 5;
                case METHOD_HANDLE:
                    first[i] = bytes[pos + 1] & 0xff;        // reference kind
                    second[i] = u2(bytes, pos + 2);          // reference index
                    return pos + 4;
                default:
                    throw new IllegalArgumentException("constant-pool entry " + i + " has tag " + tag[i]
                            + ", which is not one JVMS §4.4 defines");
            }
        }

        private static String decodeUtf8(byte[] bytes, int at, int length) {
            try {
                return new DataInputStream(new ByteArrayInputStream(bytes, at, length + 2)).readUTF();
            } catch (IOException e) {
                throw new IllegalArgumentException("a Utf8 constant at byte " + at + " is not valid modified UTF-8");
            }
        }

        String utf8(int index) {
            return utf8[checked(index, UTF8)];
        }

        String className(int classIndex) {
            return utf8(first[checked(classIndex, CLASS)]);
        }

        String memberOwner(int ref) {
            return className(first[ref]);
        }

        String memberName(int ref) {
            return utf8(first[checked(second[ref], NAME_AND_TYPE)]);
        }

        String memberDescriptor(int ref) {
            return utf8(second[checked(second[ref], NAME_AND_TYPE)]);
        }

        private int checked(int index, int expectedTag) {
            if (index <= 0 || index >= count || tag[index] != expectedTag) {
                throw new IllegalArgumentException("constant-pool index " + index + " should be an entry with tag "
                        + expectedTag + " and is not");
            }
            return index;
        }

        // Finding or appending entries. An existing entry is reused, so a rewrite adds as little as it can.

        int utf8Index(String text) {
            for (int i = 1; i < count; i++) {
                if (tag[i] == UTF8 && text.equals(utf8[i])) {
                    return i;
                }
            }
            return append("U" + text, out -> {
                out.writeByte(UTF8);
                out.writeUTF(text);
            });
        }

        int classRef(String internalName) {
            for (int i = 1; i < count; i++) {
                if (tag[i] == CLASS && internalName.equals(utf8[first[i]])) {
                    return i;
                }
            }
            int name = utf8Index(internalName);
            return append("C" + internalName, out -> {
                out.writeByte(CLASS);
                out.writeShort(name);
            });
        }

        int nameAndType(String name, String descriptor) {
            for (int i = 1; i < count; i++) {
                if (tag[i] == NAME_AND_TYPE && name.equals(utf8[first[i]]) && descriptor.equals(utf8[second[i]])) {
                    return i;
                }
            }
            int nameIndex = utf8Index(name);
            int descriptorIndex = utf8Index(descriptor);
            return append("N" + name + ":" + descriptor, out -> {
                out.writeByte(NAME_AND_TYPE);
                out.writeShort(nameIndex);
                out.writeShort(descriptorIndex);
            });
        }

        int methodref(String owner, String name, String descriptor) {
            int classIndex = classRef(owner);
            int nameAndTypeIndex = nameAndType(name, descriptor);
            return append("M" + owner + "." + name + ":" + descriptor, out -> {
                out.writeByte(METHODREF);
                out.writeShort(classIndex);
                out.writeShort(nameAndTypeIndex);
            });
        }

        byte[] appendedBytes() {
            return appended.toByteArray();
        }

        private interface EntryWriter {
            void write(DataOutputStream out) throws IOException;
        }

        private int append(String key, EntryWriter writer) {
            Integer known = appendedIndex.get(key);
            if (known != null) {
                return known;
            }
            try {
                writer.write(appendedOut);
            } catch (IOException cannotHappen) {
                throw new IllegalStateException(cannotHappen); // a ByteArrayOutputStream does not fail
            }
            int index = nextIndex++;
            appendedIndex.put(key, index);
            return index;
        }
    }

    /** Checks the magic number and that the class-file version is one this reader knows. */
    private static void requireClassFileHeader(byte[] bytes) {
        if (bytes.length < 10 || u4(bytes, 0) != 0xCAFEBABE) {
            throw new IllegalArgumentException("it is not a class file (it does not start with 0xCAFEBABE)");
        }
        int major = u2(bytes, 6);
        if (major > NEWEST_MAJOR_VERSION) {
            throw new IllegalArgumentException("it was compiled for class-file version " + major + ", newer than"
                    + " SimLoop's reader knows (" + NEWEST_MAJOR_VERSION + ", Java 25). Compile team code for"
                    + " Java 8, as the FTC SDK requires.");
        }
    }

    // ---- byte helpers ---------------------------------------------------------------------------------

    /**
     * {@return the length of the attribute whose header starts at {@code pos}}
     *
     * @throws IllegalArgumentException if the attribute does not fit in the file: a damaged length that would
     *                                  otherwise make a reader stand still or walk off into the rest of it
     */
    private static int attributeLength(byte[] bytes, int pos) {
        int length = u4(bytes, pos + 2);
        if (length < 0 || (long) pos + 6 + length > bytes.length) {
            throw new IllegalArgumentException("an attribute at byte " + pos + " claims " + (length & 0xffffffffL)
                    + " bytes, more than the file holds");
        }
        return length;
    }

    private static int u2(byte[] bytes, int at) {
        return ((bytes[at] & 0xff) << 8) | (bytes[at + 1] & 0xff);
    }

    private static int u4(byte[] bytes, int at) {
        return ((bytes[at] & 0xff) << 24) | ((bytes[at + 1] & 0xff) << 16)
                | ((bytes[at + 2] & 0xff) << 8) | (bytes[at + 3] & 0xff);
    }

    private static boolean contains(int[] values, int value) {
        for (int v : values) {
            if (v == value) {
                return true;
            }
        }
        return false;
    }
}
