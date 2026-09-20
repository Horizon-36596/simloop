package org.horizon36596.simloop.field;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

/**
 * Pins the localizer &harr; field rotation (conventions §7, BACKLOG B37). Every assertion here is a
 * number a human can check on a field drawing rather than a restatement of the implementation.
 */
class FieldFrameTransformTest {

    private static final double EPSILON = 1e-9;

    @Test
    void robotForwardBecomesFieldForward() {
        // A robot 10 in "forward" in the localizer frame is 10 in "up the field" in the field frame.
        double localizerX = 10.0;
        double localizerY = 0.0;

        assertEquals(0.0, FieldFrameTransform.fieldXFromLocalizer(localizerX, localizerY), EPSILON,
                "forward carries no field-right component");
        assertEquals(10.0, FieldFrameTransform.fieldYFromLocalizer(localizerX, localizerY), EPSILON,
                "forward is field forward, which is +Y");
    }

    @Test
    void robotLeftBecomesFieldLeftWhichIsNegativeX() {
        // The robot's left, seen from behind it looking down the field, is the field's -X.
        double localizerX = 0.0;
        double localizerY = 10.0;

        assertEquals(-10.0, FieldFrameTransform.fieldXFromLocalizer(localizerX, localizerY), EPSILON,
                "left is -X, not +X: this sign is the whole bug B37 was about");
        assertEquals(0.0, FieldFrameTransform.fieldYFromLocalizer(localizerX, localizerY), EPSILON);
    }

    @Test
    void headingZeroInTheLocalizerFrameIsNinetyDegreesInTheFieldFrame() {
        assertEquals(Math.PI / 2.0, FieldFrameTransform.fieldHeadingFromLocalizer(0.0), EPSILON,
                "a robot facing its own forward is facing field +Y, which is 90 degrees from field +X");
    }

    @Test
    void theInverseDirectionIsPinnedToLiteralsToo() {
        // Deliberately NOT a round trip. A round trip passes for any self-consistent pair of inverses,
        // including a pair that rotates the wrong way in both directions, so each inverse method needs
        // its own literal expectation.
        assertEquals(10.0, FieldFrameTransform.localizerXFromField(0.0, 10.0), EPSILON,
                "10 in up the field is 10 in of robot-forward");
        assertEquals(0.0, FieldFrameTransform.localizerYFromField(0.0, 10.0), EPSILON,
                "10 in up the field is no robot-left at all");

        assertEquals(0.0, FieldFrameTransform.localizerXFromField(-10.0, 0.0), EPSILON,
                "10 in to the field's left is no robot-forward");
        assertEquals(10.0, FieldFrameTransform.localizerYFromField(-10.0, 0.0), EPSILON,
                "10 in to the field's left is 10 in of robot-left");

        assertEquals(0.0, FieldFrameTransform.localizerHeadingFromField(Math.PI / 2.0), EPSILON,
                "facing up the field is the robot's own heading 0");
    }

    @Test
    void theRotationIsExactlyInvertible() {
        // Deliberately not a round number and not on an axis, so a transposed pair or a dropped sign
        // cannot pass by coincidence.
        double localizerX = 13.75;
        double localizerY = -41.5;
        double localizerHeading = 0.6;

        double fieldX = FieldFrameTransform.fieldXFromLocalizer(localizerX, localizerY);
        double fieldY = FieldFrameTransform.fieldYFromLocalizer(localizerX, localizerY);
        double fieldHeading = FieldFrameTransform.fieldHeadingFromLocalizer(localizerHeading);

        assertEquals(localizerX, FieldFrameTransform.localizerXFromField(fieldX, fieldY), EPSILON);
        assertEquals(localizerY, FieldFrameTransform.localizerYFromField(fieldX, fieldY), EPSILON);
        assertEquals(localizerHeading, FieldFrameTransform.localizerHeadingFromField(fieldHeading), EPSILON);
    }

    @Test
    void theOriginIsSharedSoThereIsNoTranslation() {
        // If anyone ever adds a translation, this is the test that fails first.
        assertEquals(0.0, FieldFrameTransform.fieldXFromLocalizer(0.0, 0.0), EPSILON);
        assertEquals(0.0, FieldFrameTransform.fieldYFromLocalizer(0.0, 0.0), EPSILON);
        assertEquals(0.0, FieldFrameTransform.localizerXFromField(0.0, 0.0), EPSILON);
        assertEquals(0.0, FieldFrameTransform.localizerYFromField(0.0, 0.0), EPSILON);
    }

    @Test
    void oneQuarterTurnLandsOnTheOneExpectedPoint() {
        // A point off both axes, asserted against literals, so exactly one mapping passes: a sign flip,
        // an axis swap, a reflection and the identity all fail this.
        //
        // An earlier version of this test rotated four times and checked it came back to the start. That
        // test could not fail: four +90 degree turns is the identity, but so is four -90 degree turns,
        // so is the identity map itself, and so is any reflection (which has order two). Four turns
        // proves nothing about direction.
        double localizerX = 7.0;
        double localizerY = 3.0;

        assertEquals(-3.0, FieldFrameTransform.fieldXFromLocalizer(localizerX, localizerY), EPSILON,
                "+90 degrees sends (7, 3) to (-3, 7); -90 degrees would send it to (3, -7)");
        assertEquals(7.0, FieldFrameTransform.fieldYFromLocalizer(localizerX, localizerY), EPSILON);
    }

    @Test
    void aHeadingRateIsTheSameNumberInBothFramesButAPositionIsNot() {
        // Pins the claim the javadoc makes: the frames differ by a CONSTANT offset, so a difference
        // between two headings survives the rotation untouched, even though the headings themselves do
        // not. Someone "fixing" the heading methods to scale or wrap would break this.
        double headingBefore = 0.30;
        double headingAfter = 0.44;
        double localizerRate = headingAfter - headingBefore;

        double fieldRate = FieldFrameTransform.fieldHeadingFromLocalizer(headingAfter)
                - FieldFrameTransform.fieldHeadingFromLocalizer(headingBefore);

        assertEquals(localizerRate, fieldRate, EPSILON, "a heading rate is not rotated");

        // A localizer-frame linear velocity, by contrast, does rotate — same methods as a position,
        // because it is a vector in the same axes. 12 in/s of robot-forward is 12 in/s up the field.
        double velocityForwardInchesPerSecond = 12.0;
        assertEquals(0.0,
                FieldFrameTransform.fieldXFromLocalizer(velocityForwardInchesPerSecond, 0.0), EPSILON);
        assertEquals(12.0,
                FieldFrameTransform.fieldYFromLocalizer(velocityForwardInchesPerSecond, 0.0), EPSILON);
    }
}
