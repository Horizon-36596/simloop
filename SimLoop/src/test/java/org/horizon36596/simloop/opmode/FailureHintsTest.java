package org.horizon36596.simloop.opmode;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assertions.assertTimeoutPreemptively;

import org.junit.jupiter.api.Test;

import java.time.Duration;

/**
 * Pins the hints SimLoop appends to what an OpMode threw, and that looking for them always ends.
 *
 * <p>Invariant tier: a failure message must never hang the run that reports it.
 */
class FailureHintsTest {

    @Test
    void aCastOfAClassToItselfDeepInTheCausesGetsTheReloadHint() {
        ClassCastException cast = new ClassCastException("class a.B cannot be cast to class a.B");
        String hint = FailureHints.forFailure(new RuntimeException(new IllegalStateException(cast)), false);
        assertTrue(hint.contains("loads the team's classes afresh"), hint);
    }

    @Test
    void aCycleOfCausesEndsWithNoHint() {
        RuntimeException a = new RuntimeException("a");
        RuntimeException b = new RuntimeException("b");
        a.initCause(b);
        b.initCause(a);
        String hint = assertTimeoutPreemptively(Duration.ofSeconds(5), () -> FailureHints.forFailure(a, false));
        assertEquals("", hint);
    }
}
