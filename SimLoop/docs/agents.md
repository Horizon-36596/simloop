# Using SimLoop with an AI agent

SimLoop ships a toolkit for teams who want to program with an AI coding agent: a context file that
teaches any agent how the library works, four skills for the jobs that come up, prompts to start from,
and notes on spending less.

It lives in [`simloop-agents/`](https://github.com/Horizon-36596/simloop/tree/main/simloop-agents) in the
repository. Copy the folder into your FTC project.

The toolkit is optional Markdown. No build file reads it, and removing it does not affect the sim.

## Before using it

Someone on the team needs to understand and review the code the agent changes. Ask it to explain
the units, tick order, hardware names and assertions, then compare those with your robot. A passing
simulation checks the model you supplied; it does not validate the physical robot.

The [toolkit README](https://github.com/Horizon-36596/simloop/blob/main/simloop-agents/README.md)
has the setup instructions and suggested workflow.

## Which tool

The instructions use the [`AGENTS.md`](https://agents.md/) format. Check how your coding tool loads
project instructions and confirm that it has read this file. The skills and prompts are plain
Markdown, so you can also provide them directly.

Horizon uses **Claude Code**; its paths are the ones the team has exercised most.

## What is in the folder

| File | What it is |
|---|---|
| `AGENTS.md` | The library overview, API inventory, simulation rules and test structure. Copy it to your repository root and add your robot's details. |
| `skills/` | Four procedures — write a sim test, add a mechanism, debug a failing sim, review sim code. Written for any agent; drop into `.claude/skills/` unmodified. |
| `prompts/` | Copy-paste starting points, including one for learning and one for auditing your tests before a competition. |
| `pointers/` | One-line files for the tools that need a nudge, and for a project that already has its own instructions file. They point at `AGENTS.md` rather than copying it. |
| `token-savers.md` | Spending less. Techniques first, because they are free and work better than the tools. |

## The rules an agent gets wrong

These are what `AGENTS.md` exists to prevent. Each one produces code that compiles, and most of them
produce a test that passes:

- **Inventing an API.** A plausible method name half-remembered from another library. The surface is
  small and completely listed; [what it does not do](limits.md) is the page to check first.
- **The log-key prefix.** `Logger.recordOutput("Slide/x", v)` reads back as `"RealOutputs/Slide/x"`.
- **Tick order.** `periodic()` then `advancePhysics()`. Reversed lags every response one tick and still
  passes.
- **The two frames.** Convert a pose when it crosses between `plant` and `field`; the wrong frame
  can produce incorrect geometry without throwing. See [`field`](packages/field.md).
- **The wall clock.** Robot decisions must use sim time so repeated runs can be compared.
- **Assertions that cannot fail.** Asserting a slide stayed within the end stops the plant clamps to
  every tick tests the model, not the robot.

## Check that the assertion can detect a failure

**Make every test fail once, on purpose, before you trust it.**

Temporarily break the behaviour the test is meant to check: remove the command, use an unreachable
target or shorten the run. Confirm that the relevant assertion fails, then restore the code. If the
test still passes, it is not checking the behaviour you intended.
