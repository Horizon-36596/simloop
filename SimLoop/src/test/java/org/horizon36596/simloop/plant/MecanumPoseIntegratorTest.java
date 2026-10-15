package org.horizon36596.simloop.plant;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.horizon36596.simloop.config.DrivetrainSimConfig;

import org.junit.jupiter.api.Test;

/**
 * The mecanum forward-kinematics pose integrator: wheel order and signs, rotation rate, integration at a
 * non-zero heading, the field clamp on every side, and what it refuses to accept.
 *
 * <p>Wheel order everywhere is {@code [frontLeft, frontRight, backLeft, backRight]}; a positive wheel
 * speed drives that corner of the chassis forward. Poses are in the localizer frame: {@code +x} forward at
 * heading 0, {@code +y} left, heading CCW-positive radians.
 */
class MecanumPoseIntegratorTest {

    private static final double DT = 0.02;

    /** Test geometry: 12 in by 12 in, so the rotation lever arm (wheelBase + trackWidth) / 2 is 12 in. */
    private static final double WHEEL_BASE = 12.0;
    private static final double TRACK_WIDTH = 12.0;
    private static final double LEVER_ARM = (WHEEL_BASE + TRACK_WIDTH) / 2.0;

    private static DrivetrainSimConfig config(final double fieldHalf) {
        return config(WHEEL_BASE, TRACK_WIDTH, fieldHalf, fieldHalf, 9300.0, 9300.0);
    }

    private static DrivetrainSimConfig config(final double wheelBase, final double trackWidth,
                                              final double fieldHalfWidth, final double fieldHalfHeight,
                                              final double maxVelocity, final double maxLateralVelocity) {
        return new DrivetrainSimConfig() {
            @Override public double trackWidth() { return trackWidth; }
            @Override public double wheelBase() { return wheelBase; }
            @Override public double wheelRadius() { return 2.0; }
            @Override public double ticksPerInch() { return 100.0; }
            @Override public double maxVelocityTicksPerSecond() { return maxVelocity; }
            @Override public double maxLateralVelocityTicksPerSecond() { return maxLateralVelocity; }
            @Override public double maxAccel() { return 1.8; }
            @Override public double fieldHalfWidth() { return fieldHalfWidth; }
            @Override public double fieldHalfHeight() { return fieldHalfHeight; }
        };
    }

    /** Runs the integrator for {@code seconds} of sim time at the given wheel speeds (in/s), in DT ticks. */
    private static void drive(MecanumPoseIntegrator integrator, double seconds,
                              double frontLeft, double frontRight, double backLeft, double backRight) {
        int ticks = (int) Math.round(seconds / DT);
        for (int i = 0; i < ticks; i++) {
            integrator.integrate(frontLeft, frontRight, backLeft, backRight, DT);
        }
    }

    @Test
    void equalForwardWheelSpeedsAdvancePlusXAtHeadingZero() {
        MecanumPoseIntegrator integrator = new MecanumPoseIntegrator(config(72.0));
        for (int i = 0; i < 50; i++) {
            integrator.integrate(20.0, 20.0, 20.0, 20.0, DT); // 20 in/s forward
        }
        assertTrue(integrator.getX() > 0.0, "forward command should advance +x");
        assertEquals(0.0, integrator.getY(), 1e-9, "no strafe -> y stays 0");
        assertEquals(0.0, integrator.getHeading(), 1e-9, "no rotation -> heading stays 0");
    }

    @Test
    void poseIsClampedToFieldBounds() {
        MecanumPoseIntegrator integrator = new MecanumPoseIntegrator(config(10.0));
        for (int i = 0; i < 1000; i++) {
            integrator.integrate(100.0, 100.0, 100.0, 100.0, DT); // drive far past the bound
        }
        assertEquals(10.0, integrator.getX(), 1e-9, "x clamped to the field bound");
    }

    @Test
    void oppositeSideSpeedsProduceRotation() {
        MecanumPoseIntegrator integrator = new MecanumPoseIntegrator(config(72.0));
        for (int i = 0; i < 10; i++) {
            integrator.integrate(-10.0, 10.0, -10.0, 10.0, DT); // right side forward -> CCW turn
        }
        assertTrue(integrator.getHeading() > 0.0, "should rotate CCW (+heading)");
    }

    // ---- Rotation rate and integration at a non-zero heading ----

    @Test
    void turnInPlaceRotatesAtWheelSpeedOverTheLeverArm() {
        // Left side back at v, right side forward at v: omega = v / ((wheelBase + trackWidth) / 2).
        // Over one second that is exactly the heading, and a turn in place must not translate.
        double v = 10.0;
        MecanumPoseIntegrator integrator = new MecanumPoseIntegrator(config(72.0));
        drive(integrator, 1.0, -v, v, -v, v);

        assertEquals(v / LEVER_ARM, integrator.getHeading(), 1e-9, "heading after 1 s of turning in place");
        assertEquals(0.0, integrator.getX(), 1e-9, "turning in place must not move x");
        assertEquals(0.0, integrator.getY(), 1e-9, "turning in place must not move y");
    }

    @Test
    void forwardAtAQuarterTurnMovesAlongLocalizerPlusY() {
        // Facing localizer +y (heading pi/2), robot-forward IS localizer +y.
        double v = 20.0;
        MecanumPoseIntegrator integrator = new MecanumPoseIntegrator(config(72.0));
        integrator.setPose(0.0, 0.0, Math.PI / 2.0);
        drive(integrator, 1.0, v, v, v, v);

        assertEquals(0.0, integrator.getX(), 1e-9, "forward at heading pi/2 must not move x");
        assertEquals(v, integrator.getY(), 1e-9, "forward at heading pi/2 moves +y at v for 1 s");
        assertEquals(Math.PI / 2.0, integrator.getHeading(), 1e-12, "no rotation commanded");
    }

    @Test
    void strafeLeftAtAQuarterTurnMovesAlongLocalizerMinusX() {
        // Strafe-left wheel pattern: FL back, FR forward, BL forward, BR back -> robot-frame vy = +v.
        // Facing localizer +y, robot-left is localizer -x.
        double v = 20.0;
        MecanumPoseIntegrator integrator = new MecanumPoseIntegrator(config(72.0));
        integrator.setPose(0.0, 0.0, Math.PI / 2.0);
        drive(integrator, 1.0, -v, v, v, -v);

        assertEquals(-v, integrator.getX(), 1e-9, "strafe left at heading pi/2 moves -x at v for 1 s");
        assertEquals(0.0, integrator.getY(), 1e-9, "strafe left at heading pi/2 must not move y");
        assertEquals(Math.PI / 2.0, integrator.getHeading(), 1e-12, "no rotation commanded");
    }

    @Test
    void driveAndTurnTogetherFollowsTheTrueArc() {
        // Constant forward speed v and turn rate w trace a circle of radius v / w. After time T the
        // exact pose is x = (v/w) sin(wT), y = (v/w)(1 - cos(wT)). Integrating with the heading at the
        // START of each tick lags that arc by about v*dt/2 (0.2 in here); integrating with the heading at
        // the middle of each tick is within a thousandth of an inch.
        double v = 20.0;
        double w = Math.PI / 2.0;           // a quarter turn per second
        double sideOffset = w * LEVER_ARM;  // left side slower, right side faster, by this much
        MecanumPoseIntegrator integrator = new MecanumPoseIntegrator(config(72.0));
        drive(integrator, 1.0, v - sideOffset, v + sideOffset, v - sideOffset, v + sideOffset);

        double radius = v / w;
        assertEquals(w, integrator.getHeading(), 1e-9, "heading after 1 s");
        assertEquals(radius * Math.sin(w), integrator.getX(), 2e-3, "x on the true arc");
        assertEquals(radius * (1.0 - Math.cos(w)), integrator.getY(), 2e-3, "y on the true arc");
    }

    // ---- The field clamp, on every side, on a field that is not square ----

    /**
     * A 60 in wide (field X, right-left) by 100 in long (field Y, forward-back) field. In the localizer
     * frame, localizer x is field +Y and localizer y is field -X, so localizer x is bounded by the
     * half-height (50) and localizer y by the half-width (30).
     */
    private static MecanumPoseIntegrator nonSquareField() {
        return new MecanumPoseIntegrator(config(WHEEL_BASE, TRACK_WIDTH, 30.0, 50.0, 9300.0, 9300.0));
    }

    @Test
    void nonSquareFieldClampsLocalizerXByTheHalfHeight() {
        MecanumPoseIntegrator forward = nonSquareField();
        drive(forward, 20.0, 20.0, 20.0, 20.0, 20.0);
        assertEquals(50.0, forward.getX(), 1e-9, "forward is field +Y, bounded by fieldHalfHeight");

        MecanumPoseIntegrator backward = nonSquareField();
        drive(backward, 20.0, -20.0, -20.0, -20.0, -20.0);
        assertEquals(-50.0, backward.getX(), 1e-9, "backward is field -Y, bounded by fieldHalfHeight");
    }

    @Test
    void nonSquareFieldClampsLocalizerYByTheHalfWidth() {
        MecanumPoseIntegrator left = nonSquareField();
        drive(left, 20.0, -20.0, 20.0, 20.0, -20.0);
        assertEquals(30.0, left.getY(), 1e-9, "left is field -X, bounded by fieldHalfWidth");

        MecanumPoseIntegrator right = nonSquareField();
        drive(right, 20.0, 20.0, -20.0, -20.0, 20.0);
        assertEquals(-30.0, right.getY(), 1e-9, "right is field +X, bounded by fieldHalfWidth");
    }

    // ---- Inputs it refuses, and ticks it ignores ----

    @Test
    void constructorRejectsGeometryThatWouldPoisonThePose() {
        // Zero lever arm: omega = 0/0 = NaN, and a NaN pose can never be clamped back.
        assertThrows(IllegalArgumentException.class,
                () -> new MecanumPoseIntegrator(config(0.0, 0.0, 72.0, 72.0, 9300.0, 9300.0)));
        assertThrows(IllegalArgumentException.class,
                () -> new MecanumPoseIntegrator(config(-12.0, 12.0, 72.0, 72.0, 9300.0, 9300.0)));
        assertThrows(IllegalArgumentException.class,
                () -> new MecanumPoseIntegrator(config(12.0, Double.NaN, 72.0, 72.0, 9300.0, 9300.0)));
        // A negative field bound makes clamp(min > max) teleport the robot every tick.
        assertThrows(IllegalArgumentException.class,
                () -> new MecanumPoseIntegrator(config(12.0, 12.0, -72.0, 72.0, 9300.0, 9300.0)));
        assertThrows(IllegalArgumentException.class,
                () -> new MecanumPoseIntegrator(config(12.0, 12.0, 72.0, 0.0, 9300.0, 9300.0)));
        assertThrows(IllegalArgumentException.class,
                () -> new MecanumPoseIntegrator(config(12.0, 12.0, Double.POSITIVE_INFINITY, 72.0, 9300.0, 9300.0)));
        // NaN slips a plain `<= 0` check, so the velocity ceilings must be checked for finiteness too.
        assertThrows(IllegalArgumentException.class,
                () -> new MecanumPoseIntegrator(config(12.0, 12.0, 72.0, 72.0, Double.NaN, 9300.0)));
        assertThrows(IllegalArgumentException.class,
                () -> new MecanumPoseIntegrator(config(12.0, 12.0, 72.0, 72.0, 9300.0, Double.NaN)));
        assertThrows(IllegalArgumentException.class,
                () -> new MecanumPoseIntegrator(config(12.0, 12.0, 72.0, 72.0, Double.POSITIVE_INFINITY, 9300.0)));
    }

    @Test
    void aTickThatIsNotForwardInTimeIsIgnored() {
        MecanumPoseIntegrator integrator = new MecanumPoseIntegrator(config(72.0));
        integrator.setPose(1.0, 2.0, 0.3);
        integrator.integrate(20.0, 20.0, 20.0, 20.0, -DT);
        integrator.integrate(20.0, 20.0, 20.0, 20.0, 0.0);
        integrator.integrate(20.0, 20.0, 20.0, 20.0, Double.NaN);
        integrator.integrate(20.0, 20.0, 20.0, 20.0, Double.POSITIVE_INFINITY);

        assertEquals(1.0, integrator.getX(), 0.0, "a non-positive or non-finite tick must not move x");
        assertEquals(2.0, integrator.getY(), 0.0, "a non-positive or non-finite tick must not move y");
        assertEquals(0.3, integrator.getHeading(), 0.0, "a non-positive or non-finite tick must not turn");
    }

    @Test
    void aNonFiniteWheelSpeedIsRejected() {
        MecanumPoseIntegrator integrator = new MecanumPoseIntegrator(config(72.0));
        assertThrows(IllegalArgumentException.class,
                () -> integrator.integrate(Double.NaN, 20.0, 20.0, 20.0, DT));
        assertThrows(IllegalArgumentException.class,
                () -> integrator.integrate(20.0, 20.0, 20.0, Double.NEGATIVE_INFINITY, DT));
        assertEquals(0.0, integrator.getX(), 0.0, "a rejected tick must leave the pose untouched");
        assertEquals(0.0, integrator.getHeading(), 0.0, "a rejected tick must leave the pose untouched");
    }
}
