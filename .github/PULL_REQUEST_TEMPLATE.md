## What

<!-- What does this change? One or two sentences. -->

## Why

<!-- The problem it fixes or the need it meets. Link an issue if there is one: "Fixes #12". -->

## How I tested it

<!-- The commands you ran and what they showed. For a fidelity fix, the real-robot numbers you compared against. -->

- [ ] `./gradlew test` passes on my machine
- [ ] New or changed behaviour has a test
- [ ] If I touched docs or public javadoc: `./gradlew :SimLoop:docsSite` passes

## Checklist

- [ ] No robot-specific or season-specific code (SimLoop stays season-agnostic)
- [ ] Every new number says its unit, and its frame if it is a pose
- [ ] Any public API change is called out above
- [ ] If an AI agent wrote some of this, a human has read the change and run the tests
