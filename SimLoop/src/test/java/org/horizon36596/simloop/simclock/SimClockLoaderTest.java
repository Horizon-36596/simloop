package org.horizon36596.simloop.simclock;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotSame;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.simclockfixture.ClockProbe;
import org.firstinspires.ftc.teamcode.simclockfixture.LapTimer;
import org.firstinspires.ftc.teamcode.simclockfixture.LookalikeWaits;
import org.firstinspires.ftc.teamcode.simclockfixture.PlainMath;
import org.horizon36596.simloop.sim.FakeTimer;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

import java.lang.reflect.InvocationTargetException;
import java.net.URL;
import java.net.URLClassLoader;
import java.nio.file.Files;
import java.nio.file.Path;
import java.lang.reflect.Method;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.function.LongSupplier;
import java.util.function.Supplier;

/**
 * Pins {@link SimClockLoader}: which classes it rewrites and which it leaves to the parent, that a rewritten
 * class loads, verifies and reads its own loader's sim clock, and that each failure says what went wrong.
 * The fixtures are ordinary team classes in {@code org.firstinspires.ftc.teamcode.simclockfixture}, called
 * here by reflection because the loader's copy of a class is not the test's.
 *
 * <p>Invariant tier: these are the loader's promises to a team whose code it runs.
 */
class SimClockLoaderTest {

    private static final ClassLoader PARENT = SimClockLoaderTest.class.getClassLoader();
    private static final String PROBE = ClockProbe.class.getName();

    private static SimClockLoader loader(FakeTimer clock) {
        return new SimClockLoader(PARENT, clock, SimClockLoader.DEFAULT_TEAM_PACKAGES);
    }

    /** Calls a static no-argument method of the loader's copy of {@code className}, unwrapping what it throws. */
    private static Object call(ClassLoader loader, String className, String method) throws Throwable {
        Method m = Class.forName(className, true, loader).getMethod(method);
        try {
            return m.invoke(null);
        } catch (InvocationTargetException e) {
            throw e.getCause();
        }
    }

    @Test
    void androidSystemClockReadsAreSimTimeWithoutLoadingAnyAndroidClass(@TempDir Path dir) throws Throwable {
        // Assembled by hand so it names every SystemClock read, elapsedRealtimeNanos (API 17) included, whatever
        // android.jar the tests compile against. Rewritten, it never loads android.os.SystemClock.
        String name = "org/firstinspires/ftc/teamcode/simclockfixture/raw/SystemClockReads";
        RawClassFile c = new RawClassFile();
        String[] reads = {"uptimeMillis", "elapsedRealtime", "elapsedRealtimeNanos"};
        for (String read : reads) {
            int ref = c.methodref("android/os/SystemClock", read, "()J");
            c.method(0x09, read, "()J", 2, 0, RawClassFile.bytes(0xb8, ref >> 8, ref & 0xff, 0xad), new byte[] {0, 0});
        }
        Path file = dir.resolve(name + ".class");
        Files.createDirectories(file.getParent());
        Files.write(file, c.build(name, "java/lang/Object"));
        FakeTimer clock = new FakeTimer();
        clock.advance(1.5);
        try (URLClassLoader parent = new URLClassLoader(new URL[] {dir.toUri().toURL()}, PARENT)) {
            SimClockLoader loader = new SimClockLoader(parent, clock, SimClockLoader.DEFAULT_TEAM_PACKAGES);
            String className = name.replace('/', '.');
            assertEquals(1500L, call(loader, className, "uptimeMillis"), "sim milliseconds");
            assertEquals(1500L, call(loader, className, "elapsedRealtime"), "the same clock");
            assertEquals(1_500_000_000L, call(loader, className, "elapsedRealtimeNanos"), "sim nanoseconds");
            assertEquals(3, loader.rewrittenCallSites());
        }
    }

    // ---- what is rewritten, and what is the parent's -------------------------------------------------

    @Test
    void aTeamClassIsDefinedByTheLoaderAndEverythingElseIsTheParents() throws Exception {
        SimClockLoader loader = loader(new FakeTimer());

        Class<?> probe = Class.forName(PROBE, false, loader);
        assertSame(loader, probe.getClassLoader(), "a team class is loaded child-first");
        assertNotSame(ClockProbe.class, probe, "so it is a second copy of the class");

        for (Class<?> shared : Arrays.asList(String.class, System.class, Thread.class, ElapsedTime.class,
                LinearOpMode.class, FakeTimer.class, Test.class, SimClockLoader.class)) {
            assertSame(shared, Class.forName(shared.getName(), false, loader), shared + " stays the parent's");
        }
    }

    @Test
    void theHooksAndTheTimerClassAreDefinedAgainInEachLoader() throws Exception {
        SimClockLoader loader = loader(new FakeTimer());
        for (Class<?> perLoader : Arrays.asList(SimClockHooks.class, SimClockElapsedTime.class)) {
            Class<?> copy = Class.forName(perLoader.getName(), false, loader);
            assertSame(loader, copy.getClassLoader(), perLoader.getSimpleName());
            assertNotSame(perLoader, copy);
        }
    }

    @Test
    void theJdkTheSdkAndSimLoopAreNeverRewrittenWhateverThePackagesSay() {
        SimClockLoader loader = new SimClockLoader(PARENT, new FakeTimer(), SimClockLoader.DEFAULT_TEAM_PACKAGES);
        assertTrue(loader.isTeamClass(PROBE));
        for (String name : Arrays.asList("com.qualcomm.robotcore.util.ElapsedTime",
                "org.firstinspires.ftc.robotcore.external.Telemetry", "org.horizon36596.simloop.sim.FakeTimer",
                "org.psilynx.psikit.core.Logger", "org.junit.jupiter.api.Test", "com.sun.Anything",
                "org.firstinspires.inspection.InspectionState",
                "com.google.blocks.ftcrobotcontroller.util.HardwareUtil",
                "org.firstinspires.ftc.robotcontroller.external.samples.BasicOpMode_Linear")) {
            assertFalse(loader.isTeamClass(name), name + " is never rewritten");
        }
        for (String inside : Arrays.asList("java.util.", "com.qualcomm.robotcore.", "org.horizon36596.simloop.opmode.",
                "org.firstinspires.inspection.", "com.google.blocks.", "org.firstinspires.ftc.robotcontroller.")) {
            IllegalArgumentException refused = assertThrows(IllegalArgumentException.class,
                    () -> new SimClockLoader(PARENT, new FakeTimer(), Collections.singletonList(inside)));
            assertTrue(refused.getMessage().contains("never rewrites"), refused.getMessage());
        }
    }

    @Test
    void aTeamPackageThatWouldTakeInANamespaceNeverRewrittenIsRefused() {
        // "org." would sweep in the SDK's own packages, and gson, jackson and org.json with them.
        for (String broad : Arrays.asList("org.", "com.", "org.firstinspires.", "org.firstinspires.ftc.",
                "com.google.")) {
            IllegalArgumentException refused = assertThrows(IllegalArgumentException.class,
                    () -> SimClockLoader.checkedTeamPackages(Collections.singletonList(broad)), broad);
            assertTrue(refused.getMessage().contains("\"" + broad + "\" would take in"), refused.getMessage());
            assertTrue(refused.getMessage().contains("Name your own packages instead"), refused.getMessage());
        }
    }

    @Test
    void aTeamPackageThatIsOnlyPartOfAClockReadingLibraryIsRefused() {
        // Rewriting com.pedropathing.utils. alone gives Pedro two Timers on two clocks, or a LinkageError.
        IllegalArgumentException refused = assertThrows(IllegalArgumentException.class,
                () -> SimClockLoader.checkedTeamPackages(Arrays.asList("org.firstinspires.ftc.teamcode.",
                        "com.pedropathing.utils.")));
        assertTrue(refused.getMessage().contains("only part of Pedro Pathing (\"com.pedropathing.\")"),
                refused.getMessage());
        assertEquals(Arrays.asList("org.firstinspires.ftc.teamcode.", SimClockLoader.PEDRO_PATHING),
                SimClockLoader.checkedTeamPackages(Arrays.asList("org.firstinspires.ftc.teamcode.",
                        SimClockLoader.PEDRO_PATHING)), "the whole library is fine");
    }

    @Test
    void aClassOutsideTheTeamPackagesIsTheParentsAndUntouched() throws Throwable {
        SimClockLoader loader = new SimClockLoader(PARENT, new FakeTimer(),
                Collections.singletonList("com.example.nothing."));
        assertSame(ClockProbe.class, Class.forName(PROBE, false, loader));
        assertTrue(loader.rewrittenClasses().isEmpty());

        // And a library outside the packages, called from rewritten team code, still reads the real clock.
        FakeTimer clock = new FakeTimer();
        SimClockLoader team = loader(clock);
        long before = System.nanoTime();
        long library = (Long) call(team, PROBE, "libraryNanoTime");
        long after = System.nanoTime();
        assertTrue(before <= library && library <= after, "the untouched library read the real clock: " + library);
        assertEquals(0L, call(team, PROBE, "nanoTime"));
    }

    @Test
    void theSolversLibPackageIsRewrittenByDefault() throws Throwable {
        FakeTimer clock = new FakeTimer();
        clock.advance(3.0);
        assertEquals(3_000_000_000L, call(loader(clock), PROBE, "solversLibNanoTime"));
    }

    @Test
    void onlyClassesWithAClockCallAreCountedAsRewritten() throws Throwable {
        SimClockLoader loader = loader(new FakeTimer());
        assertEquals(9.0, (Double) Class.forName(PlainMath.class.getName(), true, loader)
                .getMethod("square", double.class).invoke(null, 3.0), 0.0);
        assertEquals("25/1", call(loader, LookalikeWaits.class.getName(), "exercise"),
                "a team's own sleep(long) and idle() still run");
        assertEquals(Collections.emptyList(), loader.rewrittenClasses());

        call(loader, PROBE, "nanoTime");
        List<String> rewritten = loader.rewrittenClasses();
        assertTrue(rewritten.contains(PROBE), rewritten.toString());
        assertFalse(rewritten.contains(PlainMath.class.getName()), rewritten.toString());
        assertFalse(rewritten.contains(LookalikeWaits.class.getName()), rewritten.toString());
        // ClockProbe's six, plus LapTimer's superclass if verifying ClockProbe has loaded LapTimer already.
        int expected = 6 + (rewritten.contains(LapTimer.class.getName()) ? 1 : 0);
        assertEquals(expected, loader.rewrittenCallSites());
    }

    // ---- a rewritten class runs on its loader's clock -------------------------------------------------

    @Test
    void nanoTimeAndCurrentTimeMillisReadTheSimClock() throws Throwable {
        FakeTimer clock = new FakeTimer();
        SimClockLoader loader = loader(clock);
        assertEquals(0L, call(loader, PROBE, "nanoTime"));
        assertEquals(SimClockHooks.SIM_EPOCH_MILLIS, call(loader, PROBE, "currentTimeMillis"));
        clock.advance(1.5);
        assertEquals(1_500_000_000L, call(loader, PROBE, "nanoTime"));
        assertEquals(SimClockHooks.SIM_EPOCH_MILLIS + 1500, call(loader, PROBE, "currentTimeMillis"));
    }

    @Test
    void elapsedTimeAndATeamSubclassOfItReadTheSimClock() throws Throwable {
        FakeTimer clock = new FakeTimer();
        SimClockLoader loader = loader(clock);
        ElapsedTime timer = (ElapsedTime) call(loader, PROBE, "newTimer");
        ElapsedTime lapTimer = (ElapsedTime) call(loader, PROBE, "newLapTimer");
        clock.advance(0.75);

        assertEquals(0.75, timer.seconds(), 1e-9);
        assertEquals(750.0, lapTimer.milliseconds(), 1e-6);
        assertEquals(SimClockElapsedTime.class.getName(), timer.getClass().getName());
        assertSame(loader, timer.getClass().getClassLoader(), "this loader's own SimClockElapsedTime");
        assertEquals(SimClockElapsedTime.class.getName(), lapTimer.getClass().getSuperclass().getName());
    }

    @Test
    @SuppressWarnings("unchecked")
    void methodReferencesToTheClockAndToElapsedTimeFollowTheRewrite() throws Throwable {
        FakeTimer clock = new FakeTimer();
        SimClockLoader loader = loader(clock);
        LongSupplier nanoClock = (LongSupplier) call(loader, PROBE, "nanoClock");
        Supplier<ElapsedTime> factory = (Supplier<ElapsedTime>) call(loader, PROBE, "timerFactory");
        ElapsedTime made = factory.get();
        clock.advance(0.2);

        assertEquals(200_000_000L, nanoClock.getAsLong());
        assertEquals(SimClockElapsedTime.class.getName(), made.getClass().getName());
        assertEquals(0.2, made.seconds(), 1e-9);
    }

    @Test
    void twoLoadersHaveTwoClocksAndTwoSetsOfStatics() throws Throwable {
        FakeTimer oneClock = new FakeTimer();
        FakeTimer twoClock = new FakeTimer();
        SimClockLoader one = loader(oneClock);
        SimClockLoader two = loader(twoClock);
        oneClock.advance(1.0);
        twoClock.advance(2.0);

        assertEquals(1_000_000_000L, call(one, PROBE, "nanoTime"));
        assertEquals(2_000_000_000L, call(two, PROBE, "nanoTime"));
        assertEquals(1, call(one, PROBE, "count"));
        assertEquals(2, call(one, PROBE, "count"));
        assertEquals(1, call(two, PROBE, "count"), "the second loader's copy has its own statics");
        assertEquals(1, ClockProbe.count(), "and so does the test's own copy");
    }

    // ---- failures that say what went wrong ------------------------------------------------------------

    @Test
    void theHooksOutsideASimClockLoaderHaveNoClockAndSaySo() {
        IllegalStateException noClock = assertThrows(IllegalStateException.class, SimClockHooks::nanoTime);
        assertTrue(noClock.getMessage().contains("not by a SimClockLoader"), noClock.getMessage());
        assertTrue(noClock.getMessage().contains("will not fall back to the real one"), noClock.getMessage());
        assertThrows(IllegalStateException.class, SimClockHooks::currentTimeMillis);
        IllegalStateException noTimer = assertThrows(IllegalStateException.class, SimClockElapsedTime::new);
        assertTrue(noTimer.getMessage().contains("not by a SimClockLoader"), noTimer.getMessage());
    }

    @Test
    void aSleepWithNoRunGoingFailsWithTheCallSite() {
        SimClockLoader loader = loader(new FakeTimer());
        IllegalStateException refused = assertThrows(IllegalStateException.class, () -> call(loader, PROBE, "pause"));
        String message = refused.getMessage();
        assertTrue(message.startsWith("SimLoop cannot run Thread.sleep(10) at " + PROBE + ".pause(ClockProbe.java:"),
                message);
        assertTrue(message.contains("no OpMode run is using this sim clock"), message);
    }

    @Test
    void aSleepOnAThreadTheSchedulerRefusesFailsWithItsReason() {
        SimClockLoader loader = loader(new FakeTimer());
        loader.setScheduler(new SimClockScheduler() {
            @Override
            public String whyThreadCannotWait(Thread thread) {
                return "not on " + thread.getName() + ".";
            }

            @Override
            public boolean holdFor(double seconds) {
                throw new AssertionError("a refused thread must not be held");
            }

            @Override
            public void idleAt(String callSite) {
                throw new AssertionError("a refused thread must not be held");
            }
        });
        IllegalStateException refused = assertThrows(IllegalStateException.class, () -> call(loader, PROBE, "pause"));
        assertTrue(refused.getMessage().endsWith("on sim time: not on " + Thread.currentThread().getName() + "."),
                refused.getMessage());
    }

    @Test
    void aTeamClassWhoseBytesCannotBeFoundIsAClearClassNotFound() {
        SimClockLoader loader = loader(new FakeTimer());
        ClassNotFoundException missing = assertThrows(ClassNotFoundException.class,
                () -> Class.forName("org.firstinspires.ftc.teamcode.NoSuchAuto", false, loader));
        assertTrue(missing.getMessage().contains("has no org/firstinspires/ftc/teamcode/NoSuchAuto.class"),
                missing.getMessage());
    }

    @Test
    void aLoaderNamedForItsRunSaysWhoseClassesItHolds() {
        SimClockLoader named = new SimClockLoader(PARENT, new FakeTimer(), SimClockLoader.DEFAULT_TEAM_PACKAGES,
                "MyAuto");
        assertTrue(named.toString().equals("SimClockLoader for the run of MyAuto"), named.toString());
        assertTrue(loader(new FakeTimer()).toString().equals("SimClockLoader for a sim"));
    }

    @Test
    void badTeamPackagesAreRefused() {
        for (List<String> bad : Arrays.asList(Collections.<String>emptyList(), Collections.singletonList(""),
                Collections.singletonList("com.example.team"), Collections.<String>singletonList(null))) {
            assertThrows(IllegalArgumentException.class, () -> new SimClockLoader(PARENT, new FakeTimer(), bad),
                    String.valueOf(bad));
        }
        assertThrows(IllegalArgumentException.class,
                () -> new SimClockLoader(PARENT, null, SimClockLoader.DEFAULT_TEAM_PACKAGES));
    }
}
