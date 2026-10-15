package org.horizon36596.simloop.simclock;

import org.horizon36596.simloop.sim.FakeTimer;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.HashSet;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.atomic.AtomicInteger;

/**
 * A class loader that loads a team's classes with their clock calls moved onto one sim clock, so code that
 * uses {@code ElapsedTime}, {@code System.nanoTime()}, {@code Thread.sleep}, {@code TimeUnit.sleep},
 * Android's {@code SystemClock} or a computed {@code sleep(ms)} runs unchanged in a SimLoop test.
 * {@code OpModeRunner.forTeamClass(...)} makes one per run; most tests never touch this class directly.
 *
 * <h2>What it loads, and how</h2>
 * <ul>
 *   <li><b>The team's classes</b> - those whose names start with one of the {@link #teamPackages()} - it
 *       loads itself, child-first, rewritten by {@link ClassFileRewriter}. It reads their bytes through the
 *       parent's {@code getResourceAsStream}, so it needs no classpath setup, JVM flag or build change: it
 *       works wherever the team's compiled classes are on the test classpath (Gradle, the IDE, a console
 *       launcher).</li>
 *   <li><b>{@link SimClockHooks} and {@link SimClockElapsedTime}</b> it also defines itself, unrewritten, so
 *       each loader's copies read that loader's clock and two sims in one JVM stay apart.</li>
 *   <li><b>Everything else</b> is the parent's, parent-first as usual: the JDK, the FTC SDK, SimLoop, PsiKit,
 *       JUnit and any library outside the team packages. So {@code LinearOpMode}, {@code FakeHardwareMap}
 *       and {@code FakeTimer} are the same classes inside and outside, and an OpMode loaded here is an
 *       {@code OpMode} to the runner. Those namespaces are never rewritten, whatever the team packages say.</li>
 * </ul>
 *
 * <h2>What it tells the run</h2>
 * Every refusal, whether a hook's or a rewrite's, is recorded before it is thrown ({@link #refusals()}), so the
 * run fails on it even when the team's code catches it. Clock reads it leaves real but can see - the SDK's
 * {@code Deadline}, the real date and time, a known clock-reading library outside the team packages - are
 * collected as {@link #warnings()}.
 *
 * <h2>The price: a second copy of each team class</h2>
 * A team class loaded here is a different class from the same-named one the test itself sees, and its
 * statics are fresh. A team object the test builds with {@code new} and hands in, or that another run's
 * loader made, cannot be cast to the loaded copy's type ({@code ClassCastException}), and a static the test
 * sets is not the one the loaded code reads. Share data through SDK or SimLoop types (the OpMode's own
 * fields, {@code OpMode.blackboard}, telemetry, the fake hardware), or build the team object inside the
 * loader with {@link #loadClass}.
 */
public final class SimClockLoader extends ClassLoader {

    /**
     * The packages rewritten when a test names none: the FTC {@code TeamCode} package and SolversLib, which
     * many teams vendor as source into {@code TeamCode} and whose {@code Timing} and {@code WaitCommand}
     * read the wall clock.
     */
    public static final List<String> DEFAULT_TEAM_PACKAGES = Collections.unmodifiableList(Arrays.asList(
            "org.firstinspires.ftc.teamcode.",
            "com.seattlesolvers.solverslib."));

    /**
     * Pedro Pathing's package prefix, to add to the team packages when its follower, timers, PID controllers
     * and localizers should read the sim clock:
     * {@code teamPackages("org.firstinspires.ftc.teamcode.", "com.seattlesolvers.solverslib.", PEDRO_PATHING)}.
     * Not a default, because a rewritten library's objects cannot be handed from one run to the next (see
     * {@code SimLoop/docs/unchanged-team-code.md}).
     */
    public static final String PEDRO_PATHING = "com.pedropathing.";

    /**
     * Libraries known to read the wall clock inside their own code, by top-level package prefix, with the
     * name a warning uses. A run warns once per library it sees team code use without it in the team
     * packages, and refuses a team package that names only part of one.
     */
    private static final String[][] CLOCK_LIBRARIES = {
            {PEDRO_PATHING, "Pedro Pathing", "its Follower.update(), Timer, PIDController and PID-style"
                    + " controllers, the two- and three-wheel and fusion localizers, and the follower's hold"
                    + " timeout (isBusy())"},
            {"com.acmerobotics.roadrunner.", "Road Runner", "its actions, trajectory followers and timeouts"}};

    /** Never rewritten and always the parent's, whatever the team packages say. */
    private static final List<String> NEVER_REWRITTEN = Collections.unmodifiableList(Arrays.asList(
            "java.", "javax.", "jdk.", "sun.", "com.sun.",
            "android.", "androidx.", "dalvik.", "kotlin.",
            "com.qualcomm.", "org.firstinspires.ftc.robotcore.", "org.firstinspires.ftc.robotserver.",
            "org.firstinspires.ftc.ftccommon.", "org.firstinspires.ftc.onbotjava.", "org.firstinspires.ftc.vision.",
            "org.firstinspires.ftc.robotcontroller.", "org.firstinspires.inspection.", "com.google.blocks.",
            "org.horizon36596.simloop.", "org.psilynx.",
            "org.junit.", "org.opentest4j.", "org.apiguardian."));

    /** How many different refusals {@link #refusals()} keeps: enough to report, bounded for a looping helper. */
    private static final int MAX_REFUSALS_KEPT = 10;

    /**
     * Defined here, unrewritten, so their statics belong to this loader. Keep both free of nested classes and
     * lambdas: those would come from the parent, in another runtime package, and could not reach their
     * private members.
     */
    private static final List<String> DEFINED_PER_LOADER = Collections.unmodifiableList(Arrays.asList(
            SimClockHooks.class.getName(), SimClockElapsedTime.class.getName()));

    private final FakeTimer clock;
    private final List<String> teamPackages;
    private final String runName;
    private final ClassFileRewriter rewriter = new ClassFileRewriter();
    private final Map<String, Boolean> subclassAnswers = new ConcurrentHashMap<>();
    private final ClassFileRewriter.Hierarchy hierarchy = new ClassFileRewriter.Hierarchy() {
        @Override
        public boolean isOrExtends(String internalName, String ancestor) {
            return SimClockLoader.this.isOrExtends(internalName, ancestor);
        }

        @Override
        public boolean declaresBelow(String internalName, String ancestor, String name, String descriptor) {
            return SimClockLoader.this.declaresBelow(internalName, ancestor, name, descriptor);
        }
    };
    private final List<String> rewrittenClasses = Collections.synchronizedList(new ArrayList<String>());
    private volatile SimClockScheduler scheduler;
    private volatile Thread abandoned;
    /** The first {@link #MAX_REFUSALS_KEPT} refusals, each with a different message. Guarded by itself. */
    private final List<RuntimeException> refusals = new ArrayList<>();
    /** Real-clock reads the rewrites noted, and clock-reading libraries seen, once each. Guarded by itself. */
    private final Set<String> warnings = new LinkedHashSet<>();
    /** Libraries of {@link #CLOCK_LIBRARIES} already warned about. Guarded by {@link #warnings}. */
    private final Set<String> librariesWarnedAbout = new HashSet<>();
    private final AtomicInteger rewrittenCallSites = new AtomicInteger();

    /**
     * Makes a loader for one sim.
     *
     * @param parent       where everything that is not a team class comes from, and where the team classes'
     *                     bytes are read from: normally the test's own class loader
     * @param clock        the sim clock every rewritten call reads
     * @param teamPackages the name prefixes of the classes to rewrite, each ending in a dot, such as
     *                     {@code "com.example.myteam."}; see {@link #DEFAULT_TEAM_PACKAGES}
     * @throws IllegalArgumentException for the reasons {@link #checkedTeamPackages} gives, or a null argument
     */
    public SimClockLoader(ClassLoader parent, FakeTimer clock, List<String> teamPackages) {
        this(parent, clock, teamPackages, null);
    }

    /**
     * Makes a loader for one run, named for it: {@link #toString()} then says whose classes it holds, which
     * tells two loaders' copies of a class apart in a message.
     *
     * @param parent       as for {@link #SimClockLoader(ClassLoader, FakeTimer, List)}
     * @param clock        as for {@link #SimClockLoader(ClassLoader, FakeTimer, List)}
     * @param teamPackages as for {@link #SimClockLoader(ClassLoader, FakeTimer, List)}
     * @param runName      what the run is, such as the OpMode's simple class name; null for none
     * @throws IllegalArgumentException for the reasons {@link #checkedTeamPackages} gives, or a null argument
     *                                  other than {@code runName}
     */
    public SimClockLoader(ClassLoader parent, FakeTimer clock, List<String> teamPackages, String runName) {
        super(requireNonNull(parent, "parent"));
        this.clock = requireNonNull(clock, "clock");
        this.teamPackages = checkedTeamPackages(teamPackages);
        this.runName = runName;
    }

    /** {@return "SimClockLoader for the run of MyAuto": which run's copies of the team classes these are} */
    @Override
    public String toString() {
        return "SimClockLoader for " + (runName != null ? "the run of " + runName : "a sim");
    }

    /**
     * {@return an unmodifiable copy of {@code teamPackages}, once each prefix is checked}
     *
     * @param teamPackages the name prefixes to rewrite
     * @throws IllegalArgumentException if the list is null or empty, or a prefix is empty, lacks its final
     *                                  dot, lies inside a namespace that is never rewritten (the JDK, the SDK,
     *                                  SimLoop, PsiKit, JUnit), takes one in ({@code "org."}, {@code "com."}),
     *                                  or names only part of a library that reads the clock, such as
     *                                  {@code "com.pedropathing.utils."} for {@link #PEDRO_PATHING}
     */
    public static List<String> checkedTeamPackages(List<String> teamPackages) {
        requireNonNull(teamPackages, "teamPackages");
        if (teamPackages.isEmpty()) {
            throw new IllegalArgumentException("SimClockLoader: give at least one team package to rewrite");
        }
        for (String prefix : teamPackages) {
            if (prefix == null || prefix.isEmpty() || !prefix.endsWith(".")) {
                throw new IllegalArgumentException("SimClockLoader: a team package is a name prefix ending in a"
                        + " dot, such as \"com.example.myteam.\"; got \"" + prefix + "\"");
            }
            String protectedPrefix = neverRewrittenPrefixOf(prefix);
            if (protectedPrefix != null) {
                throw new IllegalArgumentException("SimClockLoader: \"" + prefix + "\" is inside \"" + protectedPrefix
                        + "\", which SimLoop never rewrites (the JDK, the FTC SDK, SimLoop, PsiKit and JUnit"
                        + " always keep the real classes)");
            }
            for (String never : NEVER_REWRITTEN) {
                if (never.startsWith(prefix)) {
                    throw new IllegalArgumentException("SimClockLoader: \"" + prefix + "\" would take in \""
                            + never + "\", which SimLoop never rewrites (the JDK, the FTC SDK, SimLoop, PsiKit and"
                            + " JUnit always keep the real classes), and every library under it. Name your own"
                            + " packages instead, such as \"com.example.myteam.\".");
                }
            }
            for (String[] library : CLOCK_LIBRARIES) {
                if (prefix.startsWith(library[0]) && !prefix.equals(library[0])) {
                    throw new IllegalArgumentException("SimClockLoader: \"" + prefix + "\" is only part of "
                            + library[1] + " (\"" + library[0] + "\"). Rewriting part of a library gives it two"
                            + " copies of those classes, one per clock, since the rest of the library keeps"
                            + " reaching the real-clock copy, or fails with a LinkageError. List the whole library"
                            + " by its top-level prefix, \"" + library[0] + "\".");
                }
            }
        }
        return Collections.unmodifiableList(new ArrayList<>(teamPackages));
    }

    /** {@return the sim clock every rewritten call in this loader reads} */
    public FakeTimer clock() {
        return clock;
    }

    /** {@return the name prefixes of the classes this loader rewrites} */
    public List<String> teamPackages() {
        return teamPackages;
    }

    /**
     * Sets the run that rewritten waits ({@code Thread.sleep}, {@code sleep(ms)}, {@code idle()}) hand their
     * waiting to. The OpMode runner sets it when a run starts and clears it when the run ends.
     *
     * @param scheduler the run's scheduler, or null when no run is going (waits then fail, saying so)
     */
    public void setScheduler(SimClockScheduler scheduler) {
        this.scheduler = scheduler;
    }

    /** {@return the scheduler of the run in progress, or null if none} */
    public SimClockScheduler scheduler() {
        return scheduler;
    }

    /**
     * Marks the OpMode thread its run gave up on: one still running team code after the run ended, usually a
     * loop waiting for a timer with nothing in it that ends a tick. This loader's clock never moves again, so
     * such a loop would spin forever; from now on a clock read on that thread throws instead, which ends it.
     * The OpMode runner calls this when it leaves such a thread behind.
     *
     * @param thread the thread to stop answering, or null for none
     */
    public void abandon(Thread thread) {
        this.abandoned = thread;
    }

    /** {@return the thread {@link #abandon} marked, or null} */
    public Thread abandonedThread() {
        return abandoned;
    }

    /**
     * Records a refusal, just before it is thrown: a rewritten wait this loader's run cannot give sim time
     * (a {@code Thread.sleep} on a thread the sim does not step, a wait with no run going), or a team class
     * it cannot rewrite (a {@code this::sleep} in a class first loaded mid-run). The exception alone could be
     * caught by the team's code, or kill a thread no one watches; the record is what lets the run fail
     * anyway, as documented. The hooks and this loader call it; a team's code never does.
     *
     * @param refusal the exception about to be thrown, whose message names the call, its place and the reason
     */
    public void refused(RuntimeException refusal) {
        synchronized (refusals) {
            if (refusals.size() >= MAX_REFUSALS_KEPT) {
                return;
            }
            for (RuntimeException kept : refusals) {
                if (String.valueOf(kept.getMessage()).equals(String.valueOf(refusal.getMessage()))) {
                    return;
                }
            }
            refusals.add(refusal);
        }
    }

    /**
     * {@return what this loader's classes do on the real clock that the run should warn about, once each}: a
     * team class that constructs or extends the SDK's {@code Deadline}, or reads the real date and time
     * (each with the class, method and line), and a library known to read the clock inside its own code
     * that team code uses without its package among the team packages
     */
    public List<String> warnings() {
        synchronized (warnings) {
            return Collections.unmodifiableList(new ArrayList<>(warnings));
        }
    }

    /** {@return the refusals recorded so far, first first, once per message and at most ten} */
    public List<RuntimeException> refusals() {
        synchronized (refusals) {
            return Collections.unmodifiableList(new ArrayList<>(refusals));
        }
    }

    /**
     * {@return true if a class of this name would be loaded here and rewritten}
     *
     * @param className a binary class name, such as {@code com.example.myteam.MyAuto}
     */
    public boolean isTeamClass(String className) {
        return neverRewrittenPrefixOf(className) == null && startsWithAny(className, teamPackages);
    }

    /** {@return the names of the classes defined here with at least one clock call moved, in load order} */
    public List<String> rewrittenClasses() {
        synchronized (rewrittenClasses) {
            return Collections.unmodifiableList(new ArrayList<>(rewrittenClasses));
        }
    }

    /** {@return how many calls, constructions and method references the rewrites have moved so far} */
    public int rewrittenCallSites() {
        return rewrittenCallSites.get();
    }

    /**
     * Loads a team class here, rewritten; any other class from the parent.
     *
     * @throws ClassNotFoundException   if a team class's bytes cannot be found through the parent, or any other
     *                                  class cannot be loaded by the parent
     * @throws IllegalArgumentException if a team class file cannot be read
     * @throws IllegalStateException    if a team class uses a clock call that cannot be moved to sim time
     */
    @Override
    protected Class<?> loadClass(String name, boolean resolve) throws ClassNotFoundException {
        boolean perLoader = DEFINED_PER_LOADER.contains(name);
        if (!perLoader && !isTeamClass(name)) {
            warnIfClockLibrary(name);
            return super.loadClass(name, resolve);
        }
        // One lock for the whole loader, as a loader that is not parallel-capable has: getClassLoadingLock and
        // registerAsParallelCapable are not in every Android SDK this module compiles against.
        synchronized (this) {
            Class<?> loaded = findLoadedClass(name);
            if (loaded == null) {
                loaded = define(name, perLoader);
            }
            if (resolve) {
                resolveClass(loaded);
            }
            return loaded;
        }
    }

    private Class<?> define(String name, boolean perLoader) throws ClassNotFoundException {
        String internalName = name.replace('.', '/');
        byte[] bytes = classFileBytes(internalName);
        if (bytes == null) {
            throw new ClassNotFoundException(name + " is in a team package SimLoop rewrites ("
                    + String.join(", ", teamPackages) + "), but the test class loader has no " + internalName
                    + ".class to read");
        }
        if (!perLoader) {
            ClassFileRewriter.Result result;
            try {
                result = rewriter.rewrite(name, bytes, hierarchy);
            } catch (RuntimeException refusedOrUnreadable) {
                // A class first loaded mid-run surfaces this at the call that needed it, where team code can
                // catch it; recorded, it still fails the run.
                refused(refusedOrUnreadable);
                throw refusedOrUnreadable;
            }
            if (result.bytes != bytes) {
                rewrittenClasses.add(name);
                rewrittenCallSites.addAndGet(result.callSites);
            }
            synchronized (warnings) {
                warnings.addAll(result.realClockUses);
            }
            bytes = result.bytes;
        }
        return defineClass(name, bytes, 0, bytes.length);
    }

    /** {@return the bytes of {@code internalName}'s class file, read through the parent, or null if none} */
    private byte[] classFileBytes(String internalName) {
        try (InputStream in = getParent().getResourceAsStream(internalName + ".class")) {
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
            throw new IllegalStateException("SimLoop could not read " + internalName + ".class", e);
        }
    }

    /**
     * Walks {@code super_class} entries through the class files, loading nothing, so asking about a class
     * in the middle of loading another can never start a circular load.
     *
     * @throws IllegalArgumentException if the class files on the classpath make a superclass cycle (stale or
     *                                  damaged files; the JVM would report a ClassCircularityError)
     */
    private boolean isOrExtends(String internalName, String ancestor) {
        String key = internalName + " < " + ancestor;
        Boolean known = subclassAnswers.get(key);
        if (known != null) {
            return known;
        }
        boolean answer = false;
        String current = internalName;
        List<String> walked = new ArrayList<>();
        while (current != null) {
            if (walked.contains(current)) {
                walked.add(current);
                throw new IllegalArgumentException("SimLoop cannot tell what " + internalName.replace('/', '.')
                        + " extends: the class files on the classpath make a superclass cycle ("
                        + String.join(" extends ", walked).replace('/', '.') + "). Rebuild: some are stale or"
                        + " damaged.");
            }
            walked.add(current);
            if (current.equals(ancestor)) {
                answer = true;
                break;
            }
            if (current.startsWith("java/")) {
                // The JDK's classes extend only the JDK's, and are safe to load (without initialising):
                // ForkJoinWorkerThread.sleep is Thread.sleep.
                answer = ancestor.startsWith("java/") && jdkIsOrExtends(current, ancestor);
                break;
            }
            if (current.startsWith("[")) {
                break; // an array extends only Object
            }
            byte[] bytes = classFileBytes(current);
            current = bytes == null ? null : ClassFileRewriter.superclassOf(current, bytes);
        }
        subclassAnswers.put(key, answer);
        return answer;
    }

    /** A team class links {@code name}, which the parent loads: warn once if it is a clock-reading library's. */
    private void warnIfClockLibrary(String name) {
        for (String[] library : CLOCK_LIBRARIES) {
            if (!name.startsWith(library[0])) {
                continue;
            }
            synchronized (warnings) {
                if (librariesWarnedAbout.add(library[0])) {
                    List<String> suggested = new ArrayList<>(teamPackages);
                    suggested.add(library[0]);
                    warnings.add("The team code uses " + library[1] + " (" + name + "), which reads the real clock"
                            + " in " + library[2] + ". It is not in the team packages (" + String.join(", ",
                            teamPackages) + "), so those keep laptop time, and a run that steps them can differ"
                            + " from the robot and from the run before. (A run that uses only its data types, such"
                            + " as a pose, can ignore this.) To put it on sim time, add its package:"
                            + " teamPackages(\"" + String.join("\", \"", suggested) + "\")."
                            + " SimLoop/docs/unchanged-team-code.md says what that costs.");
                }
            }
            return;
        }
    }

    /** {@return true if JDK class {@code internalName} is or extends JDK class {@code ancestor}} */
    private boolean jdkIsOrExtends(String internalName, String ancestor) {
        try {
            Class<?> type = Class.forName(internalName.replace('/', '.'), false, getParent());
            return Class.forName(ancestor.replace('/', '.'), false, getParent()).isAssignableFrom(type);
        } catch (ClassNotFoundException | LinkageError notAClassOfThisJdk) {
            return false;
        }
    }

    /**
     * Whether {@code internalName} or a class between it and {@code ancestor} declares the static method
     * {@code name} {@code descriptor}, hiding the ancestor's. The JDK's own classes hide none of the methods
     * asked about, so the walk stops at the first of them.
     */
    private boolean declaresBelow(String internalName, String ancestor, String name, String descriptor) {
        List<String> walked = new ArrayList<>();
        for (String current = internalName; current != null && !current.equals(ancestor)
                && !current.startsWith("java/") && !walked.contains(current); ) {
            walked.add(current);
            byte[] bytes = classFileBytes(current);
            if (bytes == null) {
                return false;
            }
            if (ClassFileRewriter.declaresStaticMethod(current, bytes, name, descriptor)) {
                return true;
            }
            current = ClassFileRewriter.superclassOf(current, bytes);
        }
        return false;
    }

    private static String neverRewrittenPrefixOf(String name) {
        for (String prefix : NEVER_REWRITTEN) {
            if (name.startsWith(prefix)) {
                return prefix;
            }
        }
        return null;
    }

    private static boolean startsWithAny(String name, List<String> prefixes) {
        for (String prefix : prefixes) {
            if (name.startsWith(prefix)) {
                return true;
            }
        }
        return false;
    }

    private static <T> T requireNonNull(T value, String name) {
        if (value == null) {
            throw new IllegalArgumentException("SimClockLoader: " + name + " must be non-null");
        }
        return value;
    }
}
