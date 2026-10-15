package org.horizon36596.simloop.sim;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Objects;
import java.util.function.LongSupplier;
import java.util.function.Supplier;

/**
 * An FTC SDK {@link ElapsedTime} that reads the simulation clock ({@link FakeTimer}) instead of the wall
 * clock. Hand one to code that expects an {@code ElapsedTime} and, in the sim, a 250 ms wait lasts exactly
 * 250 ms of simulated time on every run, however fast or slow the laptop ticks.
 *
 * <p><b>Why it exists.</b> The SDK's {@code ElapsedTime} reads {@code System.nanoTime()}. A sim run steps its
 * {@code FakeTimer} by a fixed amount per tick, and a tick takes far less real time than it simulates, so
 * a real {@code ElapsedTime} in team code stretches every timed step and makes two runs of the same
 * scenario differ. The porting guide, {@code SimLoop/docs/sim-safe-timers.md}, covers this and the other
 * wall-clock readers ({@code LinearOpMode.sleep}, SolversLib's {@code WaitCommand},
 * {@code System.currentTimeMillis()}). For team code that cannot be changed at all,
 * {@code OpModeRunner.forTeamClass(...)} rewrites its {@code new ElapsedTime()} to make a
 * {@link org.horizon36596.simloop.simclock.SimClockElapsedTime} instead, which reads the run's clock with no
 * {@code FakeTimer} passed in; this class is for the timers a test builds and hands over itself.
 *
 * <p><b>What is identical to the real one.</b> Everything except where "now" comes from. The SDK routes
 * every reading through one protected method, {@code nsNow()} (checked against the RobotCore 11.2.1 and
 * 12.0.0 bytecode, which are identical), and this class overrides only that. So {@link #seconds()},
 * {@link #milliseconds()}, {@link #nanoseconds()}, {@link #time()}, {@link #time(java.util.concurrent.TimeUnit)},
 * {@link #now(java.util.concurrent.TimeUnit)}, {@link #reset()}, {@link #startTime()},
 * {@link #startTimeNanoseconds()}, {@link #getResolution()} and {@link #toString()} are the SDK's own code,
 * running on sim time. The one method not to call in a JVM test is {@link #log(String)}: it writes to
 * Android's logcat, which does not exist off the robot, exactly as with a real {@code ElapsedTime}.
 *
 * <p><b>Sim time in nanoseconds.</b> {@code FakeTimer} keeps seconds as a {@code double}. This class reads
 * it as {@link #nanoTimeOf(FakeTimer)}: the seconds times 10<sup>9</sup>, rounded to the nearest
 * nanosecond. Rounding (rather than truncating) is what makes fifty 20 ms ticks read exactly
 * 1,000,000,000 ns, even though the {@code double} sum of fifty 0.02 s steps is very slightly off 1.0 s.
 *
 * <p><b>Resetting the sim clock.</b> {@link FakeTimer#reset()} moves the sim clock back to zero. A timer
 * that was started before that reset then reads a negative elapsed time, because its start is later than
 * "now" - the same arithmetic a real {@code ElapsedTime} would do. Call {@link #reset()} on it (or make a
 * new one) after resetting the {@code FakeTimer}.
 *
 * <p>Not thread-safe beyond what {@code FakeTimer} is: read it from the thread that advances the clock.
 */
public final class SimElapsedTime extends ElapsedTime {

    /** Nanoseconds in one second, as a {@code double}, for converting {@code FakeTimer}'s seconds. */
    private static final double NANOS_PER_SECOND = 1e9;

    /**
     * The sim clock this timer reads. {@code null} only while the SDK's constructor runs: it calls
     * {@code reset()}, and so {@link #nsNow()}, before this class has stored it (see {@link #nsNow()}).
     */
    private final FakeTimer timer;

    /**
     * Starts a timer at the sim clock's current time, counting in seconds - the same as
     * {@code new ElapsedTime()}.
     *
     * @param timer the sim clock to read; must not be null
     * @throws NullPointerException if {@code timer} is null
     */
    public SimElapsedTime(FakeTimer timer) {
        super();
        this.timer = requireTimer(timer);
        reset();
    }

    /**
     * Starts a timer at the sim clock's current time, with {@link #time()} and {@link #startTime()} in the
     * given unit - the same as {@code new ElapsedTime(resolution)}.
     *
     * @param timer      the sim clock to read; must not be null
     * @param resolution the unit {@link #time()} and {@link #startTime()} report in: SECONDS or MILLISECONDS
     * @throws NullPointerException if {@code timer} or {@code resolution} is null
     */
    public SimElapsedTime(FakeTimer timer, Resolution resolution) {
        super(resolution);
        this.timer = requireTimer(timer);
        reset();
    }

    /**
     * Makes a timer that counts from a given moment on the sim clock, counting in seconds - the same as
     * {@code new ElapsedTime(startTime)}. Useful to resume a timer from a moment already past.
     *
     * @param timer                the sim clock to read; must not be null
     * @param startTimeNanoseconds the moment to count from, in nanoseconds on the sim clock (the same scale
     *                             as {@link #nanoTimeOf(FakeTimer)}, so 0 is when the {@code FakeTimer} read
     *                             0.0 s)
     * @throws NullPointerException if {@code timer} is null
     */
    public SimElapsedTime(FakeTimer timer, long startTimeNanoseconds) {
        super(startTimeNanoseconds);
        this.timer = requireTimer(timer);
    }

    /**
     * The sim clock's current time, in nanoseconds. This is the one method the SDK's {@code ElapsedTime}
     * reads "now" through; overriding it is the whole of this class.
     *
     * <p>It returns 0 while the SDK's constructor is still running, before the clock has been stored.
     * Nothing can observe that value: every constructor here calls {@link #reset()} (or sets the start time
     * explicitly) once the clock is stored.
     *
     * @return the sim clock's time in nanoseconds since the {@code FakeTimer} read 0.0 s
     */
    @Override
    protected long nsNow() {
        return timer == null ? 0L : nanoTimeOf(timer);
    }

    /**
     * Reads a sim clock in nanoseconds: its seconds times 10<sup>9</sup>, rounded to the nearest nanosecond.
     * This is the sim-side value for a team's own {@code nanoTime()} seam.
     *
     * @param timer the sim clock to read; must not be null
     * @return the clock's time in nanoseconds since it read 0.0 s
     * @throws NullPointerException if {@code timer} is null
     */
    public static long nanoTimeOf(FakeTimer timer) {
        return Math.round(requireTimer(timer).time() * NANOS_PER_SECOND);
    }

    /**
     * A nanosecond clock that reads the sim clock - the sim-side value for team code that takes its time
     * from a {@code LongSupplier}. On the robot the same code is handed {@code System::nanoTime}.
     *
     * @param timer the sim clock to read; must not be null
     * @return a supplier that returns {@link #nanoTimeOf(FakeTimer)} each time it is called
     * @throws NullPointerException if {@code timer} is null
     */
    public static LongSupplier nanoClock(FakeTimer timer) {
        FakeTimer clock = requireTimer(timer);
        return () -> nanoTimeOf(clock);
    }

    /**
     * A factory of fresh sim timers - the sim-side value for team code that makes its timers through a
     * {@code Supplier<ElapsedTime>}. On the robot the same code is handed {@code ElapsedTime::new}.
     *
     * @param timer the sim clock every timer it makes will read; must not be null
     * @return a supplier that returns a new {@code SimElapsedTime}, started at the sim clock's current time
     *         and counting in seconds, each time it is called
     * @throws NullPointerException if {@code timer} is null
     */
    public static Supplier<ElapsedTime> factory(FakeTimer timer) {
        FakeTimer clock = requireTimer(timer);
        return () -> new SimElapsedTime(clock);
    }

    /** Null check with a message that names the fix. */
    private static FakeTimer requireTimer(FakeTimer timer) {
        return Objects.requireNonNull(timer,
                "SimElapsedTime needs the sim's FakeTimer - pass the same one the scenario advances");
    }
}
