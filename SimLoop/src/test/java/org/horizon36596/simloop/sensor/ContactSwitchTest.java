package org.horizon36596.simloop.sensor;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.horizon36596.simloop.fakehardware.FakeDigitalChannel;
import org.horizon36596.simloop.fakehardware.FakeHardwareMap;
import org.horizon36596.simloop.fakehardware.FakeTouchSensor;
import org.horizon36596.simloop.fakehardware.SwitchPolarity;
import org.horizon36596.simloop.field.FieldWorld;
import org.horizon36596.simloop.field.FootprintContact;
import org.horizon36596.simloop.field.GamePiece;
import org.horizon36596.simloop.field.GamePieceTracker;
import org.horizon36596.simloop.field.ObstacleBox;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * Pins {@link ContactSwitch}: a switch's actuator is pressed exactly while it touches the wall, an
 * obstacle at its height, or a loose piece, and robot code reads that through a {@code TouchSensor} or a
 * raw {@code DigitalChannel}. INVARIANT tier (domain R9) for the contact rule and determinism.
 *
 * <p>The robot is 18 in square, so its front face is 9 in ahead of its centre, and the bumper switch's
 * lever sticks 0.25 in out past it.
 */
class ContactSwitchTest {

    private static final FieldWorld FIELD = FieldWorld.builder().perimeter(72.0, 72.0).build();
    private static final List<GamePiece> NO_PIECES = Collections.emptyList();
    private static final double BALL_RADIUS = 2.5;

    private static ContactSwitch bumper() {
        return new ContactSwitch("bumper", 9.125, 0.0, 0.25, 4.0, 2.0);
    }

    // ---- walls ----

    @Test
    void invariant_aRobotStoppedFlushAgainstTheWallPressesItsBumperSwitch() {
        assertEquals(ContactSwitch.Contact.WALL, bumper().check(FIELD, NO_PIECES, 63.0, 0.0, 0.0),
                "front face at 72, lever to 72.25");
    }

    @Test
    void invariant_aRobotAnInchShortOfTheWallDoesNot() {
        assertEquals(ContactSwitch.Contact.NONE, bumper().check(FIELD, NO_PIECES, 62.0, 0.0, 0.0));
    }

    @Test
    void theLeverJustTouchingTheWallCountsAsPressed() {
        assertTrue(bumper().isPressed(FIELD, NO_PIECES, 62.75, 0.0, 0.0), "lever tip exactly at 72");
        assertFalse(bumper().isPressed(FIELD, NO_PIECES, 62.7, 0.0, 0.0));
    }

    @Test
    void theSwitchTurnsWithTheRobot() {
        ContactSwitch bumper = bumper();
        assertTrue(bumper.isPressed(FIELD, NO_PIECES, 0.0, 63.0, Math.PI / 2.0), "facing +Y against the far wall");
        assertTrue(bumper.isPressed(FIELD, NO_PIECES, -63.0, 0.0, Math.PI), "facing -X");
        assertFalse(bumper.isPressed(FIELD, NO_PIECES, 63.0, 0.0, Math.PI), "backed into the wall, the front switch is free");
        assertTrue(bumper.isPressed(FIELD, NO_PIECES, 0.0, -63.0, -Math.PI / 2.0), "facing -Y");
    }

    @Test
    void aCornerOfARotatedActuatorReachesTheWallFirst() {
        // A 4 in wide pad turned 45 degrees reaches (0.25 + 4) / 2 / sqrt(2) = 1.5026 in further along X than
        // its centre, whose own X is robotX + 9.125 * cos 45.
        ContactSwitch bumper = bumper();
        double reach = (0.25 / 2.0 + 4.0 / 2.0) * Math.cos(Math.PI / 4.0);
        double touchingX = 72.0 - reach - 9.125 * Math.cos(Math.PI / 4.0);
        assertTrue(bumper.isPressed(FIELD, NO_PIECES, touchingX + 1e-9, 0.0, Math.PI / 4.0));
        assertFalse(bumper.isPressed(FIELD, NO_PIECES, touchingX - 1e-6, 0.0, Math.PI / 4.0));
    }

    @Test
    void anUnboundedWorldHasNoWallToPress() {
        assertFalse(bumper().isPressed(FieldWorld.unbounded(), NO_PIECES, 100.0, 0.0, 0.0));
    }

    // ---- obstacles ----

    @Test
    void invariant_anObstacleAtTheSwitchesHeightPressesIt() {
        FieldWorld world = FieldWorld.builder().perimeter(72.0, 72.0)
                .obstacle(new ObstacleBox("goal", 20.0, 30.0, -10.0, 10.0, 0.0, 20.0)).build();

        assertEquals(ContactSwitch.Contact.OBSTACLE, bumper().check(world, NO_PIECES, 10.85, 0.0, 0.0));
        assertEquals(ContactSwitch.Contact.OBSTACLE, bumper().check(world, NO_PIECES, 10.75, 0.0, 0.0),
                "lever tip exactly on the face: touching counts");
        assertEquals(ContactSwitch.Contact.NONE, bumper().check(world, NO_PIECES, 10.5, 0.0, 0.0));
    }

    @Test
    void anObstacleAboveOrBelowTheSwitchDoesNotPressIt() {
        FieldWorld world = FieldWorld.builder().perimeter(72.0, 72.0)
                .obstacle(new ObstacleBox("hanging goal", 20.0, 30.0, -10.0, 10.0, 5.0, 20.0)).build();

        assertEquals(ContactSwitch.Contact.NONE, bumper().check(world, NO_PIECES, 12.0, 0.0, 0.0),
                "the lever at 2 in passes under a box that starts at 5 in");
    }

    @Test
    void aRobotPushedOutOfAnObstacleByFootprintContactEndsWithItsSwitchPressed() {
        // The harness pattern: drive into a structure, let FootprintContact push the chassis back out flush,
        // then read the switch. A lever that sticks out past the bumper is pressed; that is why it must.
        ObstacleBox goal = new ObstacleBox("goal", 20.0, 30.0, -10.0, 10.0, 0.0, 20.0);
        FieldWorld world = FieldWorld.builder().perimeter(72.0, 72.0).obstacle(goal).build();
        double robotX = 13.0;                                         // front face 2 in inside the goal
        FootprintContact.Push push = FootprintContact.pushOutOf(robotX, 0.0, 0.0, 18.0, 18.0, goal);
        robotX += push.getDxInches();

        assertEquals(11.0, robotX, 1e-9, "pushed back until the front face is on the goal's face");
        assertTrue(bumper().isPressed(world, NO_PIECES, robotX, 0.0, 0.0));
        ContactSwitch flush = new ContactSwitch("flush", 9.0 - 0.125, 0.0, 0.25, 4.0, 2.0);
        assertTrue(flush.isPressed(world, NO_PIECES, robotX, 0.0, 0.0),
                "a lever flush with the bumper is exactly touching, which also counts");
    }

    @Test
    void theWallIsReportedBeforeAnObstacleTouchingAtTheSameTime() {
        FieldWorld world = FieldWorld.builder().perimeter(72.0, 72.0)
                .obstacle(new ObstacleBox("corner post", 70.0, 72.0, -3.0, 3.0, 0.0, 10.0)).build();

        assertEquals(ContactSwitch.Contact.WALL, bumper().check(world, NO_PIECES, 63.0, 0.0, 0.0));
    }

    // ---- pieces ----

    @Test
    void invariant_aLooseBallAgainstTheLeverPressesItAndOneJustAwayDoesNot() {
        // The lever is at 2 in, 0.5 in below a 2.5 in ball's centre, where the ball's slice has radius
        // sqrt(2.5^2 - 0.5^2) = 2.449 in. The lever's tip is at 9.25 in.
        GamePieceTracker touching = new GamePieceTracker(3);
        touching.addLoosePiece("ball", 11.5, 0.0, BALL_RADIUS);
        assertEquals(ContactSwitch.Contact.PIECE, bumper().check(FIELD, touching.getPieces(), 0.0, 0.0, 0.0));

        GamePieceTracker away = new GamePieceTracker(3);
        away.addLoosePiece("ball", 11.75, 0.0, BALL_RADIUS);
        assertEquals(ContactSwitch.Contact.NONE, bumper().check(FIELD, away.getPieces(), 0.0, 0.0, 0.0));
    }

    @Test
    void aHeldPieceDoesNotPressAFieldSwitchButCanDriveOneThroughASource() {
        GamePieceTracker tracker = new GamePieceTracker(3);
        tracker.teleportRobot(0.0, 0.0, 0.0);
        tracker.addHeldPiece("ball", BALL_RADIUS);
        ContactSwitch inside = new ContactSwitch("hopper floor", 0.0, 0.0, 6.0, 6.0, 1.0);
        assertFalse(inside.isPressed(FIELD, tracker.getPieces(), 0.0, 0.0, 0.0), "held pieces are not on the field");

        FakeTouchSensor hopperSwitch = new FakeTouchSensor();
        hopperSwitch.setPressedSource(() -> tracker.getHeldCount() > 0);
        hopperSwitch.update(0.02);
        assertTrue(hopperSwitch.isPressed(), "the season drives a held-piece switch from the tracker");
    }

    // ---- driving the fakes ----

    @Test
    void invariant_driveIntoTheWallAndTheTouchSensorGoesDownExactlyWhenTheLeverArrives() {
        // Drive at 20 in/s on 20 ms ticks toward the +X wall, clamping the chassis at the wall the way a
        // drive plant's perimeter clamp does, and read the switch the way robot code does.
        FakeHardwareMap hardwareMap = new FakeHardwareMap();
        FakeTouchSensor fake = new FakeTouchSensor();
        hardwareMap.register("bumper", fake);
        TouchSensor robotView = hardwareMap.get(TouchSensor.class, "bumper");
        ContactSwitch bumper = bumper();

        double x = 50.0;
        int firstPressedTick = -1;
        for (int tick = 0; tick < 100; tick++) {
            x = Math.min(x + 20.0 * 0.02, 72.0 - 9.0);
            bumper.apply(fake, FIELD, NO_PIECES, x, 0.0, 0.0);
            if (robotView.isPressed() && firstPressedTick < 0) {
                firstPressedTick = tick;
                assertTrue(x + 9.25 >= 72.0, "pressed only once the lever reached the wall, at x = " + x);
            }
            if (firstPressedTick < 0) {
                assertTrue(x + 9.25 < 72.0, "released while the lever was clear, at x = " + x);
            }
        }
        assertTrue(firstPressedTick > 0, "the robot reached the wall");
        assertTrue(robotView.isPressed(), "and stays pressed against it");
    }

    @Test
    void invariant_aRevTouchSensorReadAsADigitalChannelGoesLowWhenPressed() {
        FakeDigitalChannel channel = new FakeDigitalChannel();
        DigitalChannel robotView = channel;

        bumper().apply(channel, SwitchPolarity.PRESSED_IS_LOW, FIELD, NO_PIECES, 63.0, 0.0, 0.0);
        assertFalse(robotView.getState(), "pressed: LOW");

        bumper().apply(channel, SwitchPolarity.PRESSED_IS_LOW, FIELD, NO_PIECES, 0.0, 0.0, 0.0);
        assertTrue(robotView.getState(), "released: HIGH");

        bumper().apply(channel, SwitchPolarity.PRESSED_IS_HIGH, FIELD, NO_PIECES, 63.0, 0.0, 0.0);
        assertTrue(robotView.getState(), "the other wiring: pressed is HIGH");
    }

    @Test
    void badArgumentsAreRefused() {
        assertThrows(IllegalArgumentException.class, () -> new ContactSwitch(" ", 0.0, 0.0, 1.0, 1.0, 1.0));
        assertThrows(IllegalArgumentException.class, () -> new ContactSwitch("s", 0.0, 0.0, 0.0, 1.0, 1.0));
        assertThrows(IllegalArgumentException.class, () -> new ContactSwitch("s", 0.0, 0.0, 1.0, -1.0, 1.0));
        assertThrows(IllegalArgumentException.class, () -> new ContactSwitch("s", 0.0, 0.0, 1.0, 1.0, -1.0));
        assertThrows(IllegalArgumentException.class, () -> new ContactSwitch("s", Double.NaN, 0.0, 1.0, 1.0, 1.0));
        ContactSwitch bumper = bumper();
        assertThrows(IllegalArgumentException.class, () -> bumper.check(null, NO_PIECES, 0.0, 0.0, 0.0));
        assertThrows(IllegalArgumentException.class, () -> bumper.check(FIELD, null, 0.0, 0.0, 0.0));
        assertThrows(IllegalArgumentException.class, () -> bumper.check(FIELD, NO_PIECES, 0.0, 0.0, Double.NaN));
        assertThrows(IllegalArgumentException.class,
                () -> bumper.apply((FakeTouchSensor) null, FIELD, NO_PIECES, 0.0, 0.0, 0.0));
        assertThrows(IllegalArgumentException.class,
                () -> bumper.apply(new FakeDigitalChannel(), null, FIELD, NO_PIECES, 0.0, 0.0, 0.0));
    }

    @Test
    void invariant_theSamePathGivesTheSameContactsEveryRun() {
        assertEquals(sweep(), sweep());
    }

    private static List<ContactSwitch.Contact> sweep() {
        FieldWorld world = FieldWorld.builder().perimeter(72.0, 72.0)
                .obstacle(new ObstacleBox("goal", 20.0, 30.0, -10.0, 10.0, 0.0, 20.0)).build();
        GamePieceTracker tracker = new GamePieceTracker(3, world);
        tracker.addLoosePiece("ball", -30.0, 40.0, BALL_RADIUS);
        List<ContactSwitch.Contact> contacts = new ArrayList<>();
        ContactSwitch bumper = bumper();
        for (int step = 0; step < 720; step++) {
            double t = step * 0.01;
            contacts.add(bumper.check(world, tracker.getPieces(), 60.0 * StrictMath.cos(t), 60.0 * StrictMath.sin(3 * t),
                    t * 2.0));
        }
        return contacts;
    }
}
