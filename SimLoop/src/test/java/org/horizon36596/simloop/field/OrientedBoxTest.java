package org.horizon36596.simloop.field;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

/**
 * INVARIANT-tier (domain R9) — the Contract: <b>two boxes overlap exactly when no line separates their shadows,
 * whatever way either is turned, and the horizontal push out of an overlap is the shortest sideways move that
 * frees them.</b> This is the test every robot-versus-field and robot-versus-robot contact rests on, so a wrong
 * answer here is a robot that drives through a wall or stops in mid-air.
 *
 * <p>Inches, field frame, no clock (domain R5).
 */
class OrientedBoxTest {

    private static final OrientedBox BAR = OrientedBox.of(new ObstacleBox("bar", 24.0, 25.0, -19.4, 19.4, 0.0, 2.0));

    @Test
    void nonsenseIsRefused() {
        double[][] identity = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
        assertThrows(IllegalArgumentException.class, () -> new OrientedBox(" ", 0, 0, 0, identity, 1, 1, 1));
        assertThrows(IllegalArgumentException.class, () -> new OrientedBox("b", Double.NaN, 0, 0, identity, 1, 1, 1));
        assertThrows(IllegalArgumentException.class, () -> new OrientedBox("b", 0, 0, 0, identity, 0, 1, 1));
        assertThrows(IllegalArgumentException.class,
                () -> new OrientedBox("b", 0, 0, 0, new double[][] {{1, 0, 0}, {1, 0, 0}, {0, 0, 1}}, 1, 1, 1),
                "two parallel axes are not a box");
        assertThrows(IllegalArgumentException.class,
                () -> new OrientedBox("b", 0, 0, 0, new double[][] {{2, 0, 0}, {0, 1, 0}, {0, 0, 1}}, 1, 1, 1),
                "an axis must be unit length");
    }

    @Test
    void axisAlignedBoxesOverlapOnlyWhenEveryRangeOverlaps() {
        OrientedBox a = OrientedBox.axisAligned("a", 0, 0, 1, 1, 1, 1);
        assertTrue(OrientedBox.overlap(a, OrientedBox.axisAligned("b", 1.5, 0, 1, 1, 1, 1)));
        assertFalse(OrientedBox.overlap(a, OrientedBox.axisAligned("c", 2.0, 0, 1, 1, 1, 1)), "touching is apart");
        assertFalse(OrientedBox.overlap(a, OrientedBox.axisAligned("d", 1.5, 0, 3.5, 1, 1, 1)), "above it");
    }

    @Test
    void aBoxTurnedFortyFiveDegreesClearsACornerItsBoundingBoxWouldHit() {
        // A 2 in square turned 45 degrees reaches 1.414 in along X and Y but only along the diagonals. Its
        // bounding box would overlap a box whose corner sits at (1.2, 1.2); the turned square does not.
        OrientedBox turned = OrientedBox.yawed("turned", 0, 0, 1, StrictMath.PI / 4, 1, 1, 1);
        OrientedBox corner = OrientedBox.axisAligned("corner", 2.2, 2.2, 1, 1, 1, 1);
        assertFalse(OrientedBox.overlap(turned, corner));
        assertTrue(OrientedBox.overlap(turned, OrientedBox.axisAligned("edge", 2.3, 0, 1, 1, 1, 1)),
                "but its point at (1.414, 0) does reach a box starting at x = 1.3");
    }

    @Test
    void anEdgeAgainstAnEdgeIsCaughtByTheCrossProductAxes() {
        // Two long bars crossed in an X at different heights, each tilted toward the other. Their face normals
        // alone cannot tell; only an edge-edge axis separates them.
        double tilt = StrictMath.toRadians(30);
        double c = StrictMath.cos(tilt);
        double s = StrictMath.sin(tilt);
        OrientedBox low = new OrientedBox("low", 0, 0, 0,
                new double[][] {{1, 0, 0}, {0, c, s}, {0, -s, c}}, 10, 0.5, 0.5);
        OrientedBox high = new OrientedBox("high", 0, 0, 1.6,
                new double[][] {{0, 1, 0}, {-c, 0, s}, {-s, 0, -c}}, 10, 0.5, 0.5);
        assertFalse(OrientedBox.overlap(low, high));
        OrientedBox lower = new OrientedBox("lower", 0, 0, 1.0,
                new double[][] {{0, 1, 0}, {-c, 0, s}, {-s, 0, -c}}, 10, 0.5, 0.5);
        assertTrue(OrientedBox.overlap(low, lower));
    }

    @Test
    void theHorizontalPushIsTheShortestSidewaysWayOut() {
        // A robot-sized box 2 in into the bar from the inside goes back 2 in along -X.
        OrientedBox robot = OrientedBox.axisAligned("chassis", 17.0 - 2.0 + 0.0, 0, 4, 9, 9, 4);
        FootprintContact.Push push = OrientedBox.axisAligned("chassis", 17.0, 0, 4, 9, 9, 4).horizontalPushOutOf(BAR);
        assertNotNull(push);
        assertEquals(-2.0, push.getDxInches(), 1e-9);
        assertEquals(0.0, push.getDyInches(), 1e-9);
        assertEquals(2.0, push.getDepthInches(), 1e-9);
        assertNull(robot.horizontalPushOutOf(BAR), "15 in out is touching, not inside");
    }

    @Test
    void aBoxAboveALowBarPassesOverIt() {
        // An intake overhang whose underside is 2.83 in up passes over a 2 in bar: no push at all.
        OrientedBox overhang = OrientedBox.axisAligned("overhang", 24.5, 0, 3.97, 2, 8, 1.14);
        assertNull(overhang.horizontalPushOutOf(BAR));
    }

    @Test
    void aTurnedRobotIsPushedAlongTheWayThatFreesItShortest() {
        // A 10 x 10 box turned 30 degrees with one corner 0.5 in into the bar's face.
        double yaw = StrictMath.toRadians(30);
        double reach = 5.0 * (StrictMath.cos(yaw) + StrictMath.sin(yaw));     // its shadow's half width on X
        OrientedBox robot = OrientedBox.yawed("chassis", 24.0 - reach + 0.5, 0, 4, yaw, 5, 5, 4);
        FootprintContact.Push push = robot.horizontalPushOutOf(BAR);
        assertNotNull(push);
        assertEquals(0.5, push.getDepthInches(), 1e-9);
        assertEquals(-0.5, push.getDxInches(), 1e-9);
        // A hair further than the push, so floating-point rounding cannot leave them exactly touching.
        OrientedBox moved = OrientedBox.yawed("chassis", robot.getCentreX() + push.getDxInches() - 1e-9,
                robot.getCentreY() + push.getDyInches(), 4, yaw, 5, 5, 4);
        assertFalse(OrientedBox.overlap(moved, BAR), "after the push they are apart");
    }

    @Test
    void placingOnARobotTurnsAndMovesTheBoxButKeepsItsHeight() {
        OrientedBox front = OrientedBox.axisAligned("front", 8, 0, 4, 1, 8, 1);
        OrientedBox placed = front.placedOnRobot(10, 20, StrictMath.PI / 2);   // facing field +Y
        assertEquals(10.0, placed.getCentreX(), 1e-9);
        assertEquals(28.0, placed.getCentreY(), 1e-9);
        assertEquals(4.0, placed.getCentreZ(), 1e-9);
        assertArrayEquals(new double[] {0, 1, 0}, placed.axis(0), 1e-12);
        assertEquals(3.0, placed.getMinZ(), 1e-12);
        assertEquals(5.0, placed.getMaxZ(), 1e-12);
    }

    @Test
    void theClosestPointIsOnTheSurfaceOutsideAndThePointItselfInside() {
        OrientedBox box = OrientedBox.yawed("b", 0, 0, 1, StrictMath.PI / 2, 2, 1, 1);  // long along field Y
        assertArrayEquals(new double[] {1, 0, 1}, box.closestPoint(5, 0, 1), 1e-12);
        assertArrayEquals(new double[] {0, 2, 2}, box.closestPoint(0, 9, 9), 1e-12);
        assertArrayEquals(new double[] {0.5, 0.5, 0.5}, box.closestPoint(0.5, 0.5, 0.5), 1e-12);
    }

    @Test
    void aWallContactBringsTheFurthestCornerBackAndAPartInsideHasNone() {
        OrientedBox part = OrientedBox.axisAligned("p", 68, 0, 1, 3, 1, 1);    // reaches x = 71
        RobotContact contact = RobotContact.withPerimeter(part, 70, 70);
        assertNotNull(contact);
        assertEquals(RobotContact.PERIMETER, contact.getStructureName());
        assertEquals(-1.0, contact.getPushXInches(), 1e-9);
        assertEquals(0.0, contact.getPushYInches(), 1e-9);
        assertEquals("p@perimeter", contact.label());
        assertNull(RobotContact.withPerimeter(OrientedBox.axisAligned("q", 0, 0, 1, 3, 1, 1), 70, 70));
    }

    @Test
    void aStandingRobotIsASolidBoxFromTheFloorUp() {
        StaticRobot partner = new StaticRobot("partner", -40, 61.8, -StrictMath.PI / 2, 17.8, 16.96, 15);
        OrientedBox box = partner.toBox();
        assertEquals(0.0, box.getMinZ(), 1e-12);
        assertEquals(15.0, box.getMaxZ(), 1e-12);
        assertArrayEquals(new double[] {-40 + 8.48, 61.8 - 8.9, 0}, box.closestPoint(-20, 40, 0), 1e-9);
        // A chassis 0.5 in into the partner's front face (y = 61.8 - 8.9), its own half width 8.48 along Y.
        RobotContact contact = RobotContact.between(OrientedBox.axisAligned("chassis", -40, 61.8 - 8.9 - 8.48 + 0.5,
                4, 8.9, 8.48, 4), box);
        assertNotNull(contact);
        assertEquals("chassis@partner", contact.label());
        assertEquals(-0.5, contact.getPushYInches(), 1e-9);
    }
}
