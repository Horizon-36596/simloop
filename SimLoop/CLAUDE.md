# SimLoop module — CLAUDE.md

> This file scopes guidance to the `SimLoop` module subtree only, and travels with the module. It must NOT give repo-wide robot-development instructions — the root CLAUDE.md of whichever repository holds this module governs robot development.

## What this module is
The season-agnostic, deterministic simulation core from Horizon (FTC 36596): fake FTC devices and `FakeHardwareMap` (`fakehardware`), the physics behind them (`plant`), the interfaces a season implements to hand its numbers in (`config`), the clock and the scenario tick loop with PsiKit wiring (`sim`), a runner for a team's unchanged OpModes (`opmode`), sim time for a team's own timer code (`simclock`), game-piece and field simulation (`field`), a simulated camera (`vision`), plant fitting from robot logs (`fit`), mechanism drawings for AdvantageScope (`viz`), and scoring runs from their logs (`loop`). Package root: `org.horizon36596.simloop`. One line per package is in `README.md`. The keyboard window for live driving is the separate plain-Java module in `desktop/`.

## Hard rules for this module
1. **No season imports, no season facts.** Nothing here may import `org.firstinspires.ftc.teamcode.*` or any season-specific class, and no class, javadoc or comment may name a season, its game or its game pieces. If you need a season fact (subsystem set, drivetrain geometry, field, mechanism limits), take it through an interface/config that the season glue implements — the core never reaches out. `SeparabilityTest` enforces the import half.
2. **Depends only on** the FTC SDK hardware interfaces + PsiKit core. No SolversLib dependency in core — a season's own `TeamCode` wires SolversLib, and this library never sees it. Adding a dependency is a human decision.
3. **Deterministic & pure.** Plant `update(deltaTime)` mutates state; getters are pure reads. No wall-clock; the clock is injected (`FakeTimer`). Iteration order is fixed (no iterating a `HashMap`/`HashSet` where order reaches the output).
4. **Fakes replicate the real SDK exactly.** Every fake device matches the real FTC SDK class it stands in for, every public method, including ones nothing calls yet. Check claims against the real SDK classes (`javap -c -p`), not memory.
5. **Self-contained.** This module's tests, README, `LICENSE`, `NOTICE.md`, `PUBLISHING.md` and the `maven-publish` wiring at the bottom of `build.gradle` all live inside this directory, so the module carries everything it needs wherever it is checked out. Adding something the library needs to the repository root instead is how that stops being true.
6. **This is published, so the public API is a promise.** Other teams compile against it. Renaming a public method, changing a signature, or bumping the FTC SDK version is a MAJOR bump and a line in `PUBLISHING.md`'s version scheme, not a tidy-up. The SDK is an `api` dependency, so consumers compile against it *through* us; changing its version changes theirs.
7. **Clear and straightforward beats clever.** Javadoc on every public class, method and field, with units and frame on every number.

## Where it is developed
This repository is the source of truth. A team that vendors this module as source pins it to one commit here, may change it locally while it iterates, and sends those changes back as pull requests. Team- or robot-specific code never goes in this module.

## References in the comments that do not resolve here

Comments and javadoc throughout this module cite things like `BACKLOG B18`, `architecture §3`,
`conventions §7`, `domain R5`, `sim-harness §2` and `docs/briefs/...`. Those are entries in the development
notes of the repository this library was first written in, and they are deliberately not copied here —
they are a maintainer's audit trail, not documentation, and they would rot the moment they were duplicated.

**Nothing a reader needs is only in them.** Every one of those citations sits beside the explanation it
supports: the comment says what is true and why, and the citation records where the decision was argued.
Read the comment and ignore the pointer. The documentation a user needs is `docs/`, and the reasoning a
maintainer needs is in the comment itself.

## Where the rest of the specification lives
`docs/` in this directory — a page per package, with units and frame on every number, plus what this
library deliberately does not do. `PUBLISHING.md` for releases. `README.md` for installing it.

## Build and test
`./gradlew :SimLoop:testDebugUnitTest`, and `./gradlew :SimLoop:desktop:test` for the keyboard window. The release unit-test variant is switched off in `build.gradle` on purpose (two test JVMs in one module both write `build/sim/*.rlog` and corrupt each other); do not re-enable it.
