package org.horizon36596.simloop.vision;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertThrows;

import java.util.ArrayList;
import java.util.List;

import org.junit.jupiter.api.Test;

/**
 * INVARIANT-tier (domain R9): the team-supplied tag layout. Every layout here is made up; SimLoop ships
 * no field geometry.
 */
class AprilTagLayoutTest {

    private static final double EPS = 1e-12;

    @Test
    void anEmptyLayoutHasNoTags() {
        assertEquals(0, AprilTagLayout.empty().size());
        assertEquals(0, AprilTagLayout.empty().getTags().size());
        assertSame(AprilTagLayout.empty(), AprilTagLayout.builder().build());
    }

    @Test
    void tagsComeBackInAscendingIdOrderWhateverOrderTheyWereAdded() {
        AprilTagLayout layout = AprilTagLayout.builder()
                .addTag(9, 0, 0, 5, 0, 4)
                .addTag(2, 10, 0, 5, 0, 4)
                .addTag(5, 20, 0, 5, 0, 4)
                .build();
        List<Integer> ids = new ArrayList<>();
        for (AprilTagLayout.Tag tag : layout.getTags()) {
            ids.add(tag.getId());
        }
        assertEquals(List.of(2, 5, 9), ids);
        assertEquals(20.0, layout.getTag(5).getXInches(), 0.0);
        assertNull(layout.getTag(3));
    }

    @Test
    void aRepeatedIdOrABadSizeIsRejected() {
        AprilTagLayout.Builder builder = AprilTagLayout.builder().addTag(1, 0, 0, 5, 0, 4);
        assertThrows(IllegalArgumentException.class, () -> builder.addTag(1, 5, 5, 5, 0, 4));
        assertThrows(IllegalArgumentException.class, () -> AprilTagLayout.builder().addTag(2, 0, 0, 5, 0, 0));
        assertThrows(IllegalArgumentException.class,
                () -> AprilTagLayout.builder().addTag(2, Double.NaN, 0, 5, 0, 4));
    }

    @Test
    void theTagFrameIsLimelightTargetSpace() {
        // A tag facing -X (on a +X wall, looking back at the field centre). A viewer in front of it looks
        // along +X, so their right is -Y.
        AprilTagLayout.Tag tag = AprilTagLayout.builder().addTag(1, 70, 12, 8, Math.PI, 4).build().getTag(1);
        RigidTransform3 t = tag.getFieldToTag();
        double[] origin = t.apply(0, 0, 0);
        assertEquals(70.0, origin[0], EPS);
        assertEquals(12.0, origin[1], EPS);
        assertEquals(8.0, origin[2], EPS);
        // +X_tag: the viewer's right = field -Y.
        assertEquals(0.0, t.rotation(0, 0), EPS);
        assertEquals(-1.0, t.rotation(1, 0), EPS);
        // +Y_tag: down.
        assertEquals(-1.0, t.rotation(2, 1), EPS);
        // +Z_tag: away from the viewer, into the tag = field +X = minus the face normal.
        assertEquals(1.0, t.rotation(0, 2), EPS);
    }
}
