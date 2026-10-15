package org.horizon36596.simloop;

import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assertions.fail;

import java.io.IOException;
import java.io.UncheckedIOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.Map;
import java.util.stream.Stream;

import org.junit.jupiter.api.Test;

/**
 * INVARIANT-tier (domain R9, hard rule R7) — the separability guard for the extractable core: NO file
 * under {@code org.horizon36596.simloop.*} may reference {@code org.firstinspires.ftc.teamcode.*} (season
 * coupling), {@code com.seattlesolvers.solverslib.*} (SolversLib — wired only by the season glue, this
 * module's CLAUDE.md rule 2), or {@code com.pedropathing.*} (Pedro Pathing is a {@code TeamCode}-only
 * dependency, so the core stays pathing-library-agnostic as well as season-agnostic — architecture §3),
 * whether via an {@code import} or a fully-qualified inline name. The one-way
 * dependency is already compiler-enforced (this module's build.gradle declares no such dependency, so a
 * forbidden reference would not compile); this test pins the invariant explicitly so it stays a red test —
 * and travels with the module on {@code git subtree split --prefix=SimLoop} (architecture §3).
 *
 * <p>Scans the module's own {@code src/main/java} sources by text (skipping comment lines so a doc that
 * merely names a forbidden package is not a false positive), so it holds regardless of what is on the
 * compile classpath. Fails loud if it cannot locate the source root (so it can never pass vacuously).
 */
class SeparabilityTest {

    private static final String[] FORBIDDEN_PACKAGE_PREFIXES = {
            "org.firstinspires.ftc.teamcode.",   // season code (R7)
            "com.seattlesolvers.solverslib.",    // SolversLib — season glue wires it, not the core (rule 2)
            "com.pedropathing.",                 // Pedro Pathing — a TeamCode-only dependency (architecture §3)
    };

    /**
     * The only lines allowed to name a forbidden package, by file (relative to {@code src/main/java}): the
     * sim-clock loader's default list of team packages to rewrite, and its constant for Pedro Pathing's
     * prefix, which a test adds to that list to rewrite Pedro. They are string data naming what to rewrite,
     * not a reference to any class there, and the loader never loads a class of its own from them.
     */
    private static final Map<String, List<String>> NAMED_EXEMPTIONS = Collections.singletonMap(
            Paths.get("org", "horizon36596", "simloop", "simclock", "SimClockLoader.java").toString(),
            Arrays.asList(
                    "\"org.firstinspires.ftc.teamcode.\",",
                    "\"com.seattlesolvers.solverslib.\"));",
                    "public static final String PEDRO_PATHING = \"com.pedropathing.\";"));

    @Test
    void noSimLoopSourceReferencesTeamCodeSolversLibOrPedroPathing() {
        Path sourceRoot = locateMainSourceRoot();

        List<String> violations = new ArrayList<>();
        int[] scanned = {0};

        try (Stream<Path> files = Files.walk(sourceRoot)) {
            files.filter(p -> p.toString().endsWith(".java")).forEach(p -> {
                scanned[0]++;
                try {
                    for (String line : Files.readAllLines(p)) {
                        String trimmed = line.trim();
                        // Skip comment lines so a Javadoc/comment naming a forbidden package isn't flagged.
                        if (trimmed.startsWith("//") || trimmed.startsWith("*") || trimmed.startsWith("/*")) {
                            continue;
                        }
                        List<String> exempt = NAMED_EXEMPTIONS.get(sourceRoot.relativize(p).toString());
                        if (exempt != null && exempt.contains(trimmed)) {
                            continue;
                        }
                        for (String forbidden : FORBIDDEN_PACKAGE_PREFIXES) {
                            // Catch both `import <pkg>.X;` and a fully-qualified inline `<pkg>.X` reference.
                            if (line.contains(forbidden)) {
                                violations.add(sourceRoot.relativize(p) + ": " + trimmed);
                            }
                        }
                    }
                } catch (IOException e) {
                    throw new UncheckedIOException(e);
                }
            });
        } catch (IOException e) {
            throw new UncheckedIOException(e);
        }

        assertTrue(scanned[0] > 0, "separability scan found no .java files under " + sourceRoot
                + " — the source root resolved wrong; the test would pass vacuously otherwise");

        if (!violations.isEmpty()) {
            fail("SimLoop core must not reference season/SolversLib types (R7 / module rule 2). Violations:\n  "
                    + String.join("\n  ", violations));
        }
    }

    /** Resolve {@code src/main/java} relative to the module dir (JVM working dir for :SimLoop tests),
     *  with a fallback for a repo-root working dir. Fails the test loud if neither exists. */
    private static Path locateMainSourceRoot() {
        Path[] candidates = {
                Paths.get("src", "main", "java"),
                Paths.get("SimLoop", "src", "main", "java"),
        };
        for (Path candidate : candidates) {
            if (Files.isDirectory(candidate)) {
                return candidate.toAbsolutePath();
            }
        }
        throw new AssertionError("could not locate SimLoop src/main/java from working dir "
                + Paths.get("").toAbsolutePath() + " — tried " + java.util.Arrays.toString(candidates));
    }
}
