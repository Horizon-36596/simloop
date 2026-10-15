package org.horizon36596.simloop.viz;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.horizon36596.simloop.config.DrivetrainSimConfig;
import org.horizon36596.simloop.config.SimRobotConfig;
import org.horizon36596.simloop.loop.RunResult;
import org.horizon36596.simloop.sim.FakeTimer;
import org.horizon36596.simloop.sim.ScenarioRunner;

import org.junit.jupiter.api.Test;
import org.psilynx.psikit.core.Logger;
import org.psilynx.psikit.core.wpi.math.Pose2d;
import org.psilynx.psikit.core.wpi.math.Rotation2d;
import org.psilynx.psikit.core.wpi.math.Translation3d;

import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.Paths;

/**
 * A five-second scenario that fires three shots at a raised goal while strafing, and writes
 * {@code build/sim/shot-arcs-demo.rlog} for AdvantageScope, so the intended-against-actual arcs can be
 * looked at in a replay.
 *
 * <p>The three shots fail in three different ways, which is the point of drawing intended against
 * actual:
 * <ol>
 *   <li>flywheel 4% under its setpoint — short and low;</li>
 *   <li>hood settled 3° steep — high and short;</li>
 *   <li>launch exactly as commanded, but the solver that aimed it ignores drag — short by drag alone.</li>
 * </ol>
 * Every actual arc also carries drag, because a real ball does. The intended arcs carry none, because the
 * solver in this example assumes none.
 *
 * <p>The aim point is synthetic: a goal mouth at (-12.75, -13.4) in on the field plan and 59.5 in up. It is
 * an arbitrary target chosen to give arcs of a typical length; it is not any game's scoring geometry.
 */
class ShotArcDemoTest {

    private static final double DT = 0.02;
    private static final int TICKS = 250;
    private static final int[] FIRE_TICKS = {30, 100, 170};

    private static final double AIM_X = -12.75;
    private static final double AIM_Y = -13.4;
    private static final double AIM_Z = 59.5;

    private static final double RELEASE_HEIGHT = 14.0;
    private static final double PITCH = Math.toRadians(70.0);
    private static final double DRAG_PER_INCH = 0.0015;

    private static final SimRobotConfig ROBOT = new SimRobotConfig() {
        @Override public String[] driveMotorNames() { return new String[] {"fl", "fr", "bl", "br"}; }
        @Override public String odometryName() { return "odo"; }
        @Override public DrivetrainSimConfig drivetrain() {
            return new DrivetrainSimConfig() {
                @Override public double trackWidth() { return 14.0; }
                @Override public double wheelBase() { return 14.0; }
                @Override public double wheelRadius() { return 2.0; }
                @Override public double ticksPerInch() { return 100.0; }
                @Override public double maxVelocityTicksPerSecond() { return 2000.0; }
                @Override public double maxAccel() { return 8.0; }
                @Override public double fieldHalfWidth() { return 70.674; }
                @Override public double fieldHalfHeight() { return 70.674; }
            };
        }
    };

    /** Robot x along the strafe, inches: -45 to +20 at y = -60 over the run. */
    private static double robotX(int tick) {
        return -45.0 + 65.0 * tick / TICKS;
    }

    private static final double ROBOT_Y = -60.0;

    @Test
    void threeShotsLogIntendedAndActualArcs() throws IOException {
        FakeTimer timer = new FakeTimer();
        ShotArcRecorder shots = new ShotArcRecorder(3);
        ShotArc[] fullActual = new ShotArc[1];
        int[] firedAt = {-1};
        double[] intendedEndError = new double[FIRE_TICKS.length];
        double[] actualMiss = new double[FIRE_TICKS.length];
        int[] shotIndex = {0};

        Path rlog = Paths.get("build", "sim", "shot-arcs-demo.rlog");
        ScenarioRunner.run("ShotArcDemo", ROBOT, timer, TICKS, DT, rlog, deltaTime -> {
            int tick = (int) Math.round(timer.time() / DT);
            double rx = robotX(tick);
            double bearing = Math.atan2(AIM_Y - ROBOT_Y, AIM_X - rx);

            int k = shotIndex[0];
            if (k < FIRE_TICKS.length && tick == FIRE_TICKS[k]) {
                double horizontal = Math.hypot(AIM_X - rx, AIM_Y - ROBOT_Y);
                double speed = ShotArc.speedToReach(horizontal, AIM_Z - RELEASE_HEIGHT, PITCH);
                ShotArc.Launch commanded = new ShotArc.Launch(rx, ROBOT_Y, RELEASE_HEIGHT, speed, bearing, PITCH);
                ShotArc.Launch flown = k == 0 ? commanded.withSpeed(speed * 0.96)
                        : k == 1 ? commanded.withPitch(PITCH + Math.toRadians(3.0))
                        : commanded;
                ShotArc intended = ShotArc.ballistic(commanded, 0.0, AIM_Z, DT);
                fullActual[0] = ShotArc.ballistic(flown, DRAG_PER_INCH, 0.0, DT);
                intendedEndError[k] = intended.closestApproachInches(AIM_X, AIM_Y, AIM_Z);
                actualMiss[k] = fullActual[0].closestApproachInches(AIM_X, AIM_Y, AIM_Z);
                shots.fire(intended, AIM_X, AIM_Y, AIM_Z);
                firedAt[0] = tick;
                shotIndex[0]++;
            }

            // Grow the newest actual arc one sample per tick, so a replay shows the ball in flight.
            Translation3d[] ball = new Translation3d[0];
            if (firedAt[0] >= 0) {
                int flown = tick - firedAt[0] + 1;
                ShotArc soFar = fullActual[0].prefix(flown);
                shots.setActual(soFar);
                if (flown < fullActual[0].size()) {
                    int at = soFar.size() - 1;
                    ball = ShotArc.ofPoints(soFar.x(at), soFar.y(at), soFar.z(at)).toTranslations();
                }
            }
            shots.record("Shots");
            Logger.recordOutput("Shots/ballInFlight", ball);
            Logger.recordOutput("Robot/fieldPose2d",
                    new Pose2d(rx * 0.0254, ROBOT_Y * 0.0254, new Rotation2d(bearing)));
        });

        for (int k = 0; k < FIRE_TICKS.length; k++) {
            assertTrue(intendedEndError[k] < 0.5, "intended arc " + k + " should pass through the aim point");
            assertTrue(actualMiss[k] > 1.0 && actualMiss[k] < 30.0,
                    "actual arc " + k + " should miss by a visible but plausible amount, got " + actualMiss[k]);
            System.out.printf("shot %d: intended end error %.2f in, actual miss %.2f in%n", k, intendedEndError[k], actualMiss[k]);
        }

        RunResult run = RunResult.fromRlog("ShotArcDemo", rlog, TICKS);
        assertEquals(3.0, run.finalValue("RealOutputs/Shots/shotCount"), 0.0);
        assertTrue(run.hasKey("RealOutputs/Shots/intended/2"));
        assertEquals(actualMiss[0], run.finalValue("RealOutputs/Shots/missInches/0"), 1e-6,
                "the logged miss is the full actual arc's miss once the shot has landed");
    }
}
