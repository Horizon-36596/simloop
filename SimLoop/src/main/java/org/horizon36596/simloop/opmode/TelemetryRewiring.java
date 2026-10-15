package org.horizon36596.simloop.opmode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.opmode.TelemetryImpl;

import java.lang.reflect.Field;
import java.lang.reflect.Modifier;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.IdentityHashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;

/**
 * Points the copies of the SDK's own telemetry that an OpMode made while it was being constructed at the
 * runner's {@link RecordingTelemetry}, so that what they are sent is recorded.
 *
 * <p><b>Why it is needed, and why after construction.</b> The SDK's {@code OpModeInternal} constructor sets
 * {@code telemetry = new TelemetryImpl(this)} before any subclass field initializer runs, and nothing in that
 * constructor can be intercepted: there is no hook that runs between it and the subclass's initializers. So
 * {@code Telemetry t = telemetry;} or {@code new MultipleTelemetry(telemetry, dashboard.getTelemetry())} as a
 * field always captures the SDK's object. On the robot that is fine: that object is the one the Driver
 * Station shows. Here the runner hands the OpMode a {@code RecordingTelemetry} instead, so each such copy is
 * put right afterwards, by reflection, before INIT.
 *
 * <p><b>What is rewired.</b> Every reference to an SDK {@code TelemetryImpl}, and to that object's
 * {@link Telemetry#log()}, found in:
 * <ul>
 *   <li>the OpMode's own instance fields (those declared by the team's classes, not the SDK's);</li>
 *   <li>the instance fields of objects those fields hold, such as a dashboard's {@code MultipleTelemetry} or
 *       a team's own helper class, up to {@value #MAX_DEPTH} objects deep;</li>
 *   <li>the elements of arrays and {@link List}s, and the values of {@link Map}s, held there.</li>
 * </ul>
 * Only an object {@code TelemetryImpl} identifies as the SDK's, or its log, is ever replaced, and only by the
 * recording telemetry or its log, so nothing else in the OpMode changes. Objects of JDK, Android, Kotlin and
 * FTC SDK classes, and enums, are not walked into.
 *
 * <p><b>What cannot be rewired</b>, each reported as a warning in the run's result: a copy in an immutable
 * collection or a {@link Set}; in a field the JVM will not let reflection write (a lambda's captured value on
 * a desktop JVM); and items or lines the OpMode added before INIT, which live inside the SDK's object and
 * have no counterpart to move to. Copies the walk cannot reach at all (a {@code static} field, deeper than
 * {@value #MAX_DEPTH} objects, inside a JDK class) are caught later instead: {@link OpModeRunner} counts every
 * update the SDK's object sends and warns if there were any, and warns too if the run ends with something
 * written to that object and never sent.
 */
final class TelemetryRewiring {

    /** How many objects deep, below the OpMode's own fields, the walk looks for copies. */
    static final int MAX_DEPTH = 4;

    /** A bound on the objects walked, so a huge object graph cannot stall INIT. */
    private static final int MAX_OBJECTS = 10_000;

    /** Package prefixes whose objects are not walked into: the platform, the language runtime, the SDK. */
    private static final String[] NOT_WALKED = {
            "java.", "javax.", "jdk.", "sun.", "com.sun.", "android.", "androidx.", "dalvik.", "kotlin.",
            "com.qualcomm.", "org.firstinspires.",
    };

    private final Telemetry replacement;
    private final Map<Object, Object> replacements = new IdentityHashMap<>();
    private final Set<Object> visited = Collections.newSetFromMap(new IdentityHashMap<Object, Boolean>());
    private final List<Slot> slots = new ArrayList<>();
    private final Map<Collection<?>, String> positionless = new IdentityHashMap<>();
    private final List<String> rewired = new ArrayList<>();
    private final List<String> warnings = new ArrayList<>();

    /** The SDK telemetry objects that already held items, lines or log entries at INIT. */
    private final Set<Object> hadContentAtInit = Collections.newSetFromMap(new IdentityHashMap<Object, Boolean>());

    private TelemetryRewiring(Telemetry replacement) {
        this.replacement = replacement;
    }

    /**
     * Installs {@code replacement} as {@code opMode.telemetry} and rewires every copy of the SDK's telemetry
     * the OpMode made while it was constructed. If the OpMode's constructor replaced {@code telemetry} with an
     * object of its own (say, a {@code MultipleTelemetry}), that object stays, and the SDK's telemetry inside
     * it is rewired instead: which is what the OpMode's code then sees on the robot too.
     *
     * @return what was done, for the run's warnings
     */
    static TelemetryRewiring install(OpMode opMode, RecordingTelemetry replacement) {
        TelemetryRewiring rewiring = new TelemetryRewiring(replacement);
        Telemetry constructed = opMode.telemetry;
        if (constructed instanceof TelemetryImpl) {
            rewiring.found((TelemetryImpl) constructed);
            opMode.telemetry = replacement;
        } else if (constructed != null) {
            rewiring.visitValue(constructed, "telemetry", 0);
        } else {
            opMode.telemetry = replacement;
        }
        rewiring.visitFields(opMode, opMode.getClass().getSimpleName(), 0);
        rewiring.replaceAll();
        return rewiring;
    }

    /** {@return where a copy was replaced, one path per copy, such as {@code "MyTeleOp.dash.telemetryList[0]"}} */
    List<String> rewired() {
        return Collections.unmodifiableList(rewired);
    }

    /** {@return the copies that could not be replaced, and why, one sentence each} */
    List<String> warnings() {
        return Collections.unmodifiableList(warnings);
    }

    /** {@return the SDK telemetry objects found, so the runner can make each one report every update} */
    List<Telemetry> sdkTelemetry() {
        List<Telemetry> found = new ArrayList<>();
        for (Object original : replacements.keySet()) {
            if (original instanceof TelemetryImpl) {
                found.add((Telemetry) original);
            }
        }
        return found;
    }

    /**
     * {@return true if an SDK telemetry object that was empty at INIT now holds items, lines or log entries}:
     * the OpMode wrote through a copy the walk could not reach and never sent it, which an iterative OpMode
     * relying on the automatic {@code telemetry.update()} does, since that update goes to the recording
     * telemetry. Call it after the run.
     */
    boolean sdkTelemetryWrittenSinceInit() {
        for (Object original : replacements.keySet()) {
            if (original instanceof TelemetryImpl && !hadContentAtInit.contains(original)
                    && hasContent((TelemetryImpl) original)) {
                return true;
            }
        }
        return false;
    }

    // ---- walking ---------------------------------------------------------------------------------------

    private void found(TelemetryImpl sdkTelemetry) {
        if (replacements.containsKey(sdkTelemetry)) {
            return;
        }
        replacements.put(sdkTelemetry, replacement);
        replacements.put(sdkTelemetry.log(), replacement.log());
        if (hasContent(sdkTelemetry)) {
            hadContentAtInit.add(sdkTelemetry);
            warnings.add("The OpMode added telemetry items, lines or log entries before INIT (in a field initializer"
                    + " or constructor). They belong to the SDK's own telemetry object and are not recorded; add"
                    + " them in init() or runOpMode() instead.");
        }
    }

    private void visitFields(Object owner, String path, int depth) {
        for (Class<?> type = owner.getClass(); type != null && isWalked(type); type = type.getSuperclass()) {
            for (Field field : type.getDeclaredFields()) {
                if (Modifier.isStatic(field.getModifiers()) || field.isSynthetic() || field.getType().isPrimitive()) {
                    continue;
                }
                try {
                    field.setAccessible(true);
                    Object value = field.get(owner);
                    String where = path + "." + field.getName();
                    slots.add(new FieldSlot(owner, field, value, where));
                    visitValue(value, where, depth + 1);
                } catch (IllegalAccessException | RuntimeException unreadable) {
                    // Not readable, so not something the OpMode's code reads telemetry through either.
                }
            }
        }
    }

    private void visitValue(Object value, String path, int depth) {
        if (value instanceof TelemetryImpl) {
            found((TelemetryImpl) value);
            return;
        }
        if (value == null || value == replacement || depth > MAX_DEPTH || visited.size() >= MAX_OBJECTS
                || !visited.add(value)) {
            return;
        }
        if (value instanceof Object[]) {
            Object[] array = (Object[]) value;
            for (int i = 0; i < array.length; i++) {
                slots.add(new ArraySlot(array, i, path + "[" + i + "]"));
                visitValue(array[i], path + "[" + i + "]", depth + 1);
            }
        } else if (value instanceof List) {
            List<?> list = (List<?>) value;
            for (int i = 0; i < list.size(); i++) {
                slots.add(new ListSlot(list, i, path + "[" + i + "]"));
                visitValue(list.get(i), path + "[" + i + "]", depth + 1);
            }
        } else if (value instanceof Map) {
            for (Map.Entry<?, ?> entry : ((Map<?, ?>) value).entrySet()) {
                String where = path + "[" + entry.getKey() + "]";
                slots.add(new MapSlot(entry, where));
                visitValue(entry.getValue(), where, depth + 1);
            }
        } else if (value instanceof Collection) {
            positionless.put((Collection<?>) value, path);
            for (Object element : (Collection<?>) value) {
                visitValue(element, path + "[...]", depth + 1);
            }
        } else if (isWalked(value.getClass())) {
            visitFields(value, path, depth);
        }
    }

    private void replaceAll() {
        for (Slot slot : slots) {
            Object original = slot.current();
            Object replacementValue = replacements.get(original);
            if (original == null || replacementValue == null) {
                continue;
            }
            try {
                slot.set(original, replacementValue);
                rewired.add(slot.where);
            } catch (Exception refused) {
                warnings.add("The OpMode holds a copy of the SDK's telemetry at " + slot.where + " that SimLoop"
                        + " could not replace (" + refused + "), so what is sent through it is not recorded. Copy"
                        + " telemetry in init() or runOpMode() instead of before INIT.");
            }
        }
        for (Map.Entry<Collection<?>, String> collection : positionless.entrySet()) {
            for (Object element : collection.getKey()) {
                if (replacements.containsKey(element)) {
                    warnings.add("The OpMode holds a copy of the SDK's telemetry in a "
                            + collection.getKey().getClass().getSimpleName() + " at " + collection.getValue()
                            + ", which has no position SimLoop could put the recording telemetry in, so what is"
                            + " sent through it is not recorded. Copy telemetry in init() or runOpMode() instead"
                            + " of before INIT.");
                    break;
                }
            }
        }
    }

    private static boolean isWalked(Class<?> type) {
        if (type.isArray() || type.isEnum() || (type.getSuperclass() != null && type.getSuperclass().isEnum())) {
            return false;
        }
        String name = type.getName();
        for (String prefix : NOT_WALKED) {
            if (name.startsWith(prefix)) {
                return false;
            }
        }
        return true;
    }

    /** True if the SDK telemetry already holds items, lines or log entries: added before INIT. */
    private static boolean hasContent(TelemetryImpl sdkTelemetry) {
        try {
            Field lines = TelemetryImpl.class.getDeclaredField("lines");
            lines.setAccessible(true);
            Object container = lines.get(sdkTelemetry);
            if (container instanceof Iterable && ((Iterable<?>) container).iterator().hasNext()) {
                return true;
            }
            Object log = sdkTelemetry.log();
            Field entries = log.getClass().getDeclaredField("entries");
            entries.setAccessible(true);
            Object logged = entries.get(log);
            return logged instanceof Collection && !((Collection<?>) logged).isEmpty();
        } catch (ReflectiveOperationException | RuntimeException unknownSdk) {
            return false; // an SDK laid out differently: the check is a courtesy, not a guarantee
        }
    }

    // ---- where a copy can live -------------------------------------------------------------------------

    /** One place a reference can be held, and how to overwrite it. */
    private abstract static class Slot {
        final String where;

        Slot(String where) {
            this.where = where;
        }

        abstract Object current();

        /** Overwrites {@code original}, which this slot held when it was found, with {@code value}. */
        abstract void set(Object original, Object value) throws Exception;
    }

    private static final class FieldSlot extends Slot {
        private final Object owner;
        private final Field field;
        private final Object value;

        FieldSlot(Object owner, Field field, Object value, String where) {
            super(where);
            this.owner = owner;
            this.field = field;
            this.value = value;
        }

        @Override
        Object current() {
            return value;
        }

        @Override
        void set(Object original, Object replacement) throws IllegalAccessException {
            field.set(owner, replacement);
        }
    }

    private static final class ArraySlot extends Slot {
        private final Object[] array;
        private final int index;

        ArraySlot(Object[] array, int index, String where) {
            super(where);
            this.array = array;
            this.index = index;
        }

        @Override
        Object current() {
            return array[index];
        }

        @Override
        void set(Object original, Object replacement) {
            array[index] = replacement;
        }
    }

    private static final class ListSlot extends Slot {
        private final List<?> list;
        private final int index;

        ListSlot(List<?> list, int index, String where) {
            super(where);
            this.list = list;
            this.index = index;
        }

        @Override
        Object current() {
            return list.get(index);
        }

        @Override
        @SuppressWarnings("unchecked")
        void set(Object original, Object replacement) {
            ((List<Object>) list).set(index, replacement);
        }
    }

    private static final class MapSlot extends Slot {
        private final Map.Entry<?, ?> entry;

        MapSlot(Map.Entry<?, ?> entry, String where) {
            super(where);
            this.entry = entry;
        }

        @Override
        Object current() {
            return entry.getValue();
        }

        @Override
        @SuppressWarnings("unchecked")
        void set(Object original, Object replacement) {
            ((Map.Entry<Object, Object>) entry).setValue(replacement);
        }
    }
}
