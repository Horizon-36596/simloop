package org.horizon36596.simloop.viz;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;

import org.junit.jupiter.api.Test;
import org.psilynx.psikit.core.wpi.math.Pose3d;
import org.psilynx.psikit.core.wpi.math.Translation3d;

/**
 * {@link ArticulatedRobotModel}: the poses it logs must put an asset authored "relative to the joint origin,
 * robot axes, every joint at zero" back where the CAD had it, and move it the way the joint says.
 *
 * <p>Each test checks one property of that contract with hand-computable numbers: inches in, metres out;
 * a 90 degree turn moves (1, 0, 0) to a place you can name without a calculator.
 */
class ArticulatedRobotModelTest {

    private static final double M = 0.0254;
    private static final double EPS = 1e-9;
    private static final RobotVector3 X = new RobotVector3(1, 0, 0);
    private static final RobotVector3 Y = new RobotVector3(0, 1, 0);
    private static final RobotVector3 Z = new RobotVector3(0, 0, 1);

    /** Where a point authored at {@code local} (inches, relative to the joint origin) is drawn, inches. */
    private static double[] drawn(Pose3d pose, double x, double y, double z) {
        Translation3d p = new Translation3d(x * M, y * M, z * M).rotateBy(pose.getRotation()).plus(pose.getTranslation());
        return new double[] {p.getX() / M, p.getY() / M, p.getZ() / M};
    }

    /** A turret on the robot, a hood on the turret, a camera riding on the turret. */
    private static ArticulatedRobotModel turretHoodCamera() {
        return ArticulatedRobotModel.builder()
                .revolute("turret", ArticulatedRobotModel.ROBOT, new RobotVector3(0, 0, 10), Z)
                .revolute("hood", 0, new RobotVector3(4, 0, 12), Y)
                .fixed("camera", 0, new RobotVector3(6, 0, 11))
                .build();
    }

    @Test
    void atZeroEveryComponentIsDrawnWhereTheCadPutIt() {
        Pose3d[] poses = turretHoodCamera().poses(0, 0, 0);
        assertArrayEquals(new double[] {0, 0, 10}, drawn(poses[0], 0, 0, 0), EPS);
        assertArrayEquals(new double[] {4, 0, 12}, drawn(poses[1], 0, 0, 0), EPS);
        assertArrayEquals(new double[] {6, 0, 11}, drawn(poses[2], 0, 0, 0), EPS);
        // and a point of the mesh 1 in ahead of the hood hinge is 1 in ahead of it, unrotated
        assertArrayEquals(new double[] {5, 0, 12}, drawn(poses[1], 1, 0, 0), EPS);
    }

    @Test
    void theJointOriginItselfNeverMovesWithItsOwnJoint() {
        // A point on an axle stays on the axle however far the axle turns.
        ArticulatedRobotModel model = turretHoodCamera();
        for (double hood : new double[] {-1.0, 0.3, 2.5}) {
            assertArrayEquals(new double[] {4, 0, 12}, drawn(model.poses(0, hood, 0)[1], 0, 0, 0), EPS);
        }
    }

    @Test
    void aRevoluteJointTurnsItsMeshRightHandedAboutItsAxis() {
        // +90 deg about +Y takes +X to -Z (WPILib positive pitch: nose down).
        Pose3d hood = turretHoodCamera().poses(0, StrictMath.PI / 2, 0)[1];
        assertArrayEquals(new double[] {4, 0, 11}, drawn(hood, 1, 0, 0), EPS);
        // +90 deg about +Z takes +X to +Y.
        Pose3d turret = turretHoodCamera().poses(StrictMath.PI / 2, 0, 0)[0];
        assertArrayEquals(new double[] {0, 1, 10}, drawn(turret, 1, 0, 0), EPS);
    }

    @Test
    void childrenRideAlongWithTheirParent() {
        // Turret +90 deg: the hood hinge at (4, 0) swings to (0, 4), the camera at (6, 0) to (0, 6), and the
        // camera now looks along +Y.
        Pose3d[] poses = turretHoodCamera().poses(StrictMath.PI / 2, 0, 0);
        assertArrayEquals(new double[] {0, 4, 12}, drawn(poses[1], 0, 0, 0), EPS);
        assertArrayEquals(new double[] {0, 6, 11}, drawn(poses[2], 0, 0, 0), EPS);
        assertArrayEquals(new double[] {0, 7, 11}, drawn(poses[2], 1, 0, 0), EPS);
    }

    @Test
    void aChildTurnsAboutItsOwnAxisCarriedByItsParent() {
        // Turret +90 deg, hood +90 deg: the hood's axle now points along -X (it was +Y), so the hood's
        // mesh point 1 in ahead of the hinge (now pointing +Y) pitches down to 1 in below the hinge.
        Pose3d hood = turretHoodCamera().poses(StrictMath.PI / 2, StrictMath.PI / 2, 0)[1];
        assertArrayEquals(new double[] {0, 4, 11}, drawn(hood, 1, 0, 0), EPS);
    }

    @Test
    void aPrismaticJointSlidesAlongItsDirectionInInches() {
        ArticulatedRobotModel lift = ArticulatedRobotModel.builder()
                .prismatic("slide", ArticulatedRobotModel.ROBOT, new RobotVector3(-3, 0, 2), Z)
                .revolute("wrist", 0, new RobotVector3(-3, 0, 8), Y)
                .build();
        Pose3d[] poses = lift.poses(5.0, 0.0);
        assertArrayEquals(new double[] {-3, 0, 7}, drawn(poses[0], 0, 0, 0), EPS);
        assertArrayEquals(new double[] {-3, 0, 13}, drawn(poses[1], 0, 0, 0), EPS);
    }

    @Test
    void outputIsMetresAndIdentityRotationAtZero() {
        Pose3d camera = turretHoodCamera().poses(0, 0, 0)[2];
        assertEquals(6 * M, camera.getTranslation().getX(), EPS);
        assertEquals(11 * M, camera.getTranslation().getZ(), EPS);
        assertEquals(1.0, camera.getRotation().getQuaternion().getW(), EPS);
    }

    @Test
    void sameInputsGiveIdenticalPoses() {
        ArticulatedRobotModel model = turretHoodCamera();
        Pose3d a = model.poses(0.7, -0.2, 0)[2];
        Pose3d b = model.poses(0.7, -0.2, 0)[2];
        assertEquals(a.getTranslation().getX(), b.getTranslation().getX(), 0.0);
        assertEquals(a.getRotation().getQuaternion().getZ(), b.getRotation().getQuaternion().getZ(), 0.0);
    }

    @Test
    void namesAndTreeAreReadable() {
        ArticulatedRobotModel model = turretHoodCamera();
        assertEquals(3, model.size());
        assertEquals(1, model.indexOf("hood"));
        assertEquals(0, model.getParent(2));
        assertEquals(ArticulatedRobotModel.ROBOT, model.getParent(0));
        assertEquals(ArticulatedRobotModel.JointType.FIXED, model.getJointType(2));
        assertEquals(4.0, model.getJointOrigin(1).x, EPS);
        assertEquals(1.0, model.getJointAxis(1).y, EPS);
    }

    @Test
    void badModelsAndBadInputsAreRefused() {
        assertThrows(IllegalArgumentException.class, () -> ArticulatedRobotModel.builder()
                .fixed("a", ArticulatedRobotModel.ROBOT, RobotVector3.origin())
                .fixed("a", ArticulatedRobotModel.ROBOT, RobotVector3.origin()));
        assertThrows(IllegalArgumentException.class, () -> ArticulatedRobotModel.builder()
                .fixed("child", 0, RobotVector3.origin()));
        assertThrows(IllegalArgumentException.class, () -> ArticulatedRobotModel.builder()
                .revolute("bent", ArticulatedRobotModel.ROBOT, RobotVector3.origin(), new RobotVector3(1, 1, 0)));
        assertThrows(IllegalStateException.class, () -> ArticulatedRobotModel.builder().build());
        ArticulatedRobotModel model = turretHoodCamera();
        assertThrows(IllegalArgumentException.class, () -> model.poses(0, 0));
        assertThrows(IllegalArgumentException.class, () -> model.poses(0, Double.NaN, 0));
        assertThrows(IllegalArgumentException.class, () -> model.indexOf("flywheel"));
    }
}
