package org.horizon36596.simloop.field;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertThrows;

import org.junit.jupiter.api.Test;

/** {@link FootprintContact}: an 18 in square robot against a 1 in wide bar standing at x = 24..25. */
class FootprintContactTest {

    private static final ObstacleBox BAR = new ObstacleBox("bar", 24.0, 25.0, -19.4, 19.4, 0.0, 2.0);

    @Test
    void nonsenseArgumentsAreRefusedRatherThanAnsweredWithNaN() {
        assertThrows(IllegalArgumentException.class,
                () -> FootprintContact.pushOutOf(Double.NaN, 0.0, 0.0, 18.0, 18.0, BAR));
        assertThrows(IllegalArgumentException.class,
                () -> FootprintContact.pushOutOf(0.0, 0.0, Double.POSITIVE_INFINITY, 18.0, 18.0, BAR));
        assertThrows(IllegalArgumentException.class,
                () -> FootprintContact.pushOutOf(0.0, 0.0, 0.0, 0.0, 18.0, BAR), "a footprint with no length");
        assertThrows(IllegalArgumentException.class,
                () -> FootprintContact.pushOutOf(0.0, 0.0, 0.0, 18.0, -1.0, BAR), "a negative width");
        assertThrows(IllegalArgumentException.class,
                () -> FootprintContact.pushOutOf(0.0, 0.0, 0.0, 18.0, 18.0, null), "no box");
    }

    @Test
    void aRobotClearOfTheBarIsNotPushed() {
        assertNull(FootprintContact.pushOutOf(10.0, 0.0, 0.0, 18.0, 18.0, BAR));
    }

    @Test
    void aRobotTouchingTheBarExactlyIsNotPushed() {
        assertNull(FootprintContact.pushOutOf(15.0, 0.0, 0.0, 18.0, 18.0, BAR));
    }

    @Test
    void aRobotTwoInchesIntoTheBarFromTheInsideIsPushedTwoInchesBack() {
        FootprintContact.Push push = FootprintContact.pushOutOf(17.0, 0.0, 0.0, 18.0, 18.0, BAR);
        assertNotNull(push);
        assertEquals(-2.0, push.getDxInches(), 1e-9);
        assertEquals(0.0, push.getDyInches(), 1e-9);
        assertEquals(2.0, push.getDepthInches(), 1e-9);
    }

    @Test
    void aRobotComingFromTheOutsideIsPushedOutwards() {
        FootprintContact.Push push = FootprintContact.pushOutOf(33.5, 0.0, 0.0, 18.0, 18.0, BAR);
        assertNotNull(push);
        assertEquals(0.5, push.getDxInches(), 1e-9);
    }

    @Test
    void aRobotOverlappingTheEndOfTheBarIsPushedTheShortWay() {
        // Centre 1 in past the bar's end along Y: 8 in of overlap along Y, only 1 in along X.
        FootprintContact.Push push = FootprintContact.pushOutOf(16.0, 20.4, 0.0, 18.0, 18.0, BAR);
        assertNotNull(push);
        assertEquals(-1.0, push.getDxInches(), 1e-9);
        assertEquals(0.0, push.getDyInches(), 1e-9);
    }

    @Test
    void aRotatedRobotReachesFurtherAlongTheDiagonal() {
        // At 45 degrees the 18 in square's half-extent along X is 9 * sqrt(2) = 12.73 in, so a centre
        // 12 in short of the bar's face is 0.73 in into it, where the unrotated robot would be clear.
        assertNull(FootprintContact.pushOutOf(12.0, 0.0, 0.0, 18.0, 18.0, BAR));
        FootprintContact.Push push = FootprintContact.pushOutOf(12.0, 0.0, StrictMath.PI / 4.0, 18.0, 18.0, BAR);
        assertNotNull(push);
        assertEquals(9.0 * StrictMath.sqrt(2.0) - 12.0, push.getDepthInches(), 1e-9);
    }

    @Test
    void applyingThePushLeavesTheRobotTouchingAndNoLongerOverlapping() {
        double x = 13.0;
        double y = 5.0;
        double heading = 0.3;
        FootprintContact.Push push = FootprintContact.pushOutOf(x, y, heading, 18.0, 18.0, BAR);
        assertNotNull(push);
        assertNull(FootprintContact.pushOutOf(x + push.getDxInches() * 1.000001, y + push.getDyInches() * 1.000001,
                heading, 18.0, 18.0, BAR));
    }
}
