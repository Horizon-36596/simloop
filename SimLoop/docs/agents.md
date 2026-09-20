# Using SimLoop with an AI agent

SimLoop ships a toolkit for teams who want to program with an AI coding agent: a context file that
teaches any agent how the library works, four skills for the jobs that come up, prompts to start from,
and notes on spending less.

It lives in [`simloop-agents/`](https://github.com/Horizon-36596/simloop/tree/main/simloop-agents) in the
repository. Copy the folder into your FTC project.

!!! info "Optional, and inert"
    The folder is Markdown and nothing else. No Gradle file reads it, nothing imports it, and no part of
    SimLoop depends on it existing. If you want nothing to do with AI, delete the folder and nothing
    changes. Everything else in this documentation is written for a human and stays that way.

## Before the rest

Three things, in short. The full version — which is longer, and is the canonical copy of this
argument — is in
[`simloop-agents/README.md`](https://github.com/Horizon-36596/simloop/blob/main/simloop-agents/README.md).
Read it before a team commits to this.

- **It does not replace understanding your own code.** An agent will write a subsystem you do not
  understand, and it will compile, and it will probably pass in the simulator. The architecture still
  has to live in somebody's head, and an agent makes it easy to skip the part where it gets there.
- **It is not a way around learning to program.** Reviewing code you did not write and spotting the
  confident wrong answer are real skills with their own discipline. This is a second thing to learn,
  not a way to avoid the first.
- **It does genuinely lower the door.** A student who has never written Java can describe a mechanism in
  plain English and get a working, tested simulation of it. That is the opportunity, and it is why the
  folder exists.

## Which tool

Any of them. [`AGENTS.md`](https://agents.md/) is an open format that Cursor, Codex, Gemini CLI,
Copilot's coding agent, Zed, Windsurf, Jules, Junie and Aider read from a repository root without being
configured to. The skills and prompts are plain Markdown and work anywhere you can paste text.

We use **Claude Code**, so the Claude-specific paths are the best-tested route. That is a statement of
what has actually been exercised, not a claim that the others are worse.

## What is in the folder

| File | What it is |
|---|---|
| `AGENTS.md` | The one that matters. What SimLoop is, the nine rules that stop an agent writing code that compiles and is wrong, the complete public API, and the shape of every test. Copy it to your repository root. |
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
- **The two frames.** A pose crossing between `plant` and `field` unrotated gives a confident wrong
  answer rather than throwing. See [`field`](packages/field.md).
- **The wall clock.** Any real-time read destroys determinism, which is the whole point.
- **Assertions that cannot fail.** Asserting a slide stayed within the end stops the plant clamps to
  every tick tests the model, not the robot.

## The one habit worth keeping

**Make every test fail once, on purpose, before you trust it.**

An agent writes tests that pass — that is what it is optimising for. A test that passes because it
cannot fail looks exactly like a test that passes because the robot works, and the difference matters
most at the moment you are relying on it. Break the thing, watch it go red, put it back.
