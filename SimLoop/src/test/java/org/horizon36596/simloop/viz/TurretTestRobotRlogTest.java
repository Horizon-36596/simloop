package org.horizon36596.simloop.viz;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.horizon36596.simloop.config.DrivetrainSimConfig;
import org.horizon36596.simloop.config.SimRobotConfig;
import org.horizon36596.simloop.sim.FakeTimer;
import org.horizon36596.simloop.sim.RlogDecodedCompare;
import org.horizon36596.simloop.sim.ScenarioRunner;
import org.junit.jupiter.api.Test;
import org.psilynx.psikit.core.Logger;
import org.psilynx.psikit.core.wpi.math.Pose2d;
import org.psilynx.psikit.core.wpi.math.Rotation2d;

import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.List;
import java.util.Map;
import java.util.Set;

/**
 * SimLoop's turret test robot, driven through a scenario and logged to an RLOG a human can open:
 * {@code SimLoop/build/sim/turret-test-robot.rlog}. Open it in AdvantageScope's 3D Field tab to watch
 * {@link TurretSketch} work: {@code docs/onboarding/advantagescope-mechanisms.md}, "The turret".
 *
 * <p><b>The scenario.</b> The robot drives a 30 in circle round the field centre while its turret tracks a
 * fixed goal and its hood sweeps from 30° to 70°. The turret is a real actuator, not a number: it lags its command
 * (first order, 0.15 s) and cannot turn faster than 4 rad/s, and its encoder reports that lagging position
 * quantized to 8192 counts per turn. So what the turret <b>reports</b> and what it was <b>told</b> genuinely
 * differ, which is what makes clause 1 testable.
 *
 * <p>This robot is not any season's: it is defined entirely in {@link TurretTestRobot}. Determinism (domain R5):
 * the clock is a {@link FakeTimer}, and the trigonometry is {@link StrictMath}.
 */
class TurretTestRobotRlogTest {

    private static final int TICKS = 400;              // 8 s at 50 Hz: the circle, and two hood sweeps
    private static final double DT = 0.02;

    private static final double CIRCLE_RADIUS_INCHES = 30.0;
    /** 0.8 rad/s for 8 s is 6.4 rad: one whole lap, so the goal passes all the way round the turret. */
    private static final double CIRCLE_RATE_RAD_PER_S = 0.8;
    private static final double GOAL_X_INCHES = 48.0;
    private static final double GOAL_Y_INCHES = 48.0;

    private static final double TURRET_TIME_CONSTANT_S = 0.15;
    private static final double TURRET_MAX_RATE_RAD_PER_S = 4.0;
    private static final double ENCODER_RADIANS_PER_COUNT = 2.0 * Math.PI / 8192.0;
    private static final double HOOD_TIME_CONSTANT_S = 0.2;

    /** Recorded as metadata only; ScenarioRunner never reads a robot's hardware from it. */
    private static final SimRobotConfig TEST_ROBOT_CONFIG = new SimRobotConfig() {
        @Override public String[] driveMotorNames() { return new String[] {"fl", "fr", "bl", "br"}; }
        @Override public String odometryName() { return "odo"; }
        @Override public DrivetrainSimConfig drivetrain() {
            return new DrivetrainSimConfig() {
                @Override public double trackWidth() { return 14.0; }
                @Override public double wheelBase() { return 12.0; }
                @Override public double wheelRadius() { return 2.0; }
                @Override public double ticksPerInch() { return 100.0; }
                @Override public double maxVelocityTicksPerSecond() { return 1000.0; }
                @Override public double maxAccel() { return 2.0; }
                @Override public double fieldHalfWidth() { return 72.0; }
                @Override public double fieldHalfHeight() { return 72.0; }
            };
        }
    };

    /**
     * Contract clause 1: the drawing is at the angle the turret REPORTS, every frame, and the test would catch a
     * drawing that used the command instead, because the two differ by more than a tolerance on some frames.
     * The report stays inside the turret's travel here (it chases a command already clamped to it), so the
     * hard-stop clamp is not exercised by this scenario; {@code TurretSketchTest} pins that one directly.
     * Invariant tier.
     */
    @Test
    void theTurretIsDrawnAtTheAngleItReportsNotTheAngleItWasTold() throws IOException {
        List<Map<String, String>> frames = RlogDecodedCompare.decode(run("turret-test-robot"));
        double largestLag = 0.0;
        int frameIndex = 0;
        for (Map<String, String> frame : frames) {
            double reported = Double.parseDouble(frame.get("RealOutputs/" + TurretTestRobot.YAW_STATE_KEY));
            double told = Double.parseDouble(frame.get("RealOutputs/TestRobot/Turret/commandedRad"));
            double drawn = Double.parseDouble(frame.get("RealOutputs/TestRobot/Turret3d/yawRadians"));
            assertEquals(reported, drawn, 1.0e-12, "frame " + frameIndex + ": the drawing must follow the report");
            largestLag = Math.max(largestLag, Math.abs(told - reported));
            frameIndex++;
        }
        // No frame-count check here: ScenarioRunner.run already refuses a log without one frame per tick.
        assertTrue(largestLag > 0.05, "the report never lagged the command by more than " + largestLag
                + " rad, so this run could not tell a drawing of the report from a drawing of the command");
    }

    /** Domain R5: the same scenario, run twice, decodes to identical logs. Invariant tier. */
    @Test
    void theSameScenarioDrawsTheSameThingTwice() throws IOException {
        Path first = run("turret-test-robot-determinism-a");
        Path second = run("turret-test-robot-determinism-b");
        List<RlogDecodedCompare.Mismatch> mismatches = RlogDecodedCompare.compare(first, second);
        assertTrue(mismatches.isEmpty(), RlogDecodedCompare.summarise(mismatches));
    }

    /**
     * Everything the 3D Field tab needs for the drawing is in the log, under the names the docs give, including
     * both joints' {@code stateKey}s from {@code turret-test-robot.mechanism.json}.
     */
    @Test
    void theLogCarriesTheRobotPoseTheGhostTheHoodAndTheAim() throws IOException {
        Set<String> keys = RlogDecodedCompare.decodedKeys(run("turret-test-robot-keys"));
        for (String key : new String[] {
                "RealOutputs/TestRobot/pose2d", "RealOutputs/TestRobot/Turret3d/Ghost",
                "RealOutputs/TestRobot/Turret3d/Aim", "RealOutputs/TestRobot/Turret3d/aimElevationRadians",
                "RealOutputs/" + TurretTestRobot.YAW_STATE_KEY, "RealOutputs/" + TurretTestRobot.HOOD_STATE_KEY}) {
            assertTrue(keys.contains(key), "missing " + key);
        }
        assertTrue(keys.stream().anyMatch(k -> k.startsWith("RealOutputs/TestRobot/Turret3d/Mechanism/")),
                "the hood's Mechanism2d must be logged under TestRobot/Turret3d/Mechanism");
    }

    /** One run of the scenario, logged to {@code build/sim/<name>.rlog}. Each test uses its own file. */
    private static Path run(String name) {
        FakeTimer timer = new FakeTimer();
        TurretSketch turret = new TurretSketch(TurretTestRobot.create());
        double[] turretActual = {0.0};
        double[] hoodActualDeg = {TurretTestRobot.HOOD_DRAWN_POSE_DEG};

        Path rlog = Paths.get("build", "sim", name + ".rlog");
        ScenarioRunner.run("TurretTestRobot", TEST_ROBOT_CONFIG, timer, TICKS, DT, rlog, deltaTime -> {
            double t = timer.time();

            // The robot: a circle round the field centre, facing along it.
            double angle = CIRCLE_RATE_RAD_PER_S * t;
            double x = CIRCLE_RADIUS_INCHES * StrictMath.cos(angle);
            double y = CIRCLE_RADIUS_INCHES * StrictMath.sin(angle);
            double heading = angle + Math.PI / 2.0;

            // The turret is told to point at the goal, robot-relative and within its travel.
            double toGoal = StrictMath.atan2(GOAL_Y_INCHES - y, GOAL_X_INCHES - x) - heading;
            double told = Math.max(TurretTestRobot.YAW_LIMIT_CW_RADIANS,
                    Math.min(TurretTestRobot.YAW_LIMIT_CCW_RADIANS, wrap(toGoal)));

            // It gets there first-order, and no faster than its rate limit.
            double step = (told - turretActual[0]) * (1.0 - StrictMath.exp(-deltaTime / TURRET_TIME_CONSTANT_S));
            double maxStep = TURRET_MAX_RATE_RAD_PER_S * deltaTime;
            turretActual[0] += Math.max(-maxStep, Math.min(maxStep, step));
            double reported = Math.rint(turretActual[0] / ENCODER_RADIANS_PER_COUNT) * ENCODER_RADIANS_PER_COUNT;

            // The hood sweeps 30..70 degrees, lagging too.
            double hoodToldDeg = 50.0 + 20.0 * StrictMath.sin(2.0 * Math.PI * 0.25 * t);
            hoodActualDeg[0] += (hoodToldDeg - hoodActualDeg[0])
                    * (1.0 - StrictMath.exp(-deltaTime / HOOD_TIME_CONSTANT_S));

            Logger.recordOutput("TestRobot/pose2d", new Pose2d(
                    x * 0.0254, y * 0.0254, Rotation2d.fromRadians(heading)));
            Logger.recordOutput("TestRobot/Turret/commandedRad", told);
            Logger.recordOutput(TurretTestRobot.YAW_STATE_KEY, reported);
            // The hood's stateKey is its JOINT angle, radians from the drawn pose, as every REVOLUTE stateKey is
            // (docs/reference/mechanism-ir.md). Degrees from horizontal is logged beside it as a diagnostic.
            double hoodJointRad = TurretTestRobot.hoodJointRadiansFor(hoodActualDeg[0]);
            Logger.recordOutput(TurretTestRobot.HOOD_STATE_KEY, hoodJointRad);
            Logger.recordOutput("TestRobot/Hood/angleDeg", hoodActualDeg[0]);

            turret.setRobotFieldPose(x, y, heading);
            turret.setYawRadians(reported);
            turret.setJointAngleRadians(TurretTestRobot.HOOD_JOINT, hoodJointRad);
            turret.record(TurretTestRobot.TURRET_LOG_KEY);
        });
        return rlog.toAbsolutePath();
    }

    /** An angle wrapped into (-pi, pi]. */
    private static double wrap(double radians) {
        double wrapped = radians % (2.0 * Math.PI);
        if (wrapped > Math.PI) {
            wrapped -= 2.0 * Math.PI;
        } else if (wrapped <= -Math.PI) {
            wrapped += 2.0 * Math.PI;
        }
        return wrapped;
    }
}
