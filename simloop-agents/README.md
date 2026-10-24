# Programming SimLoop with an AI agent

A toolkit for teams who want to use an AI coding agent on their robot code: a context file that teaches
any agent how SimLoop works, four skills for the jobs that come up, prompts to start from, and notes on
not spending more than you need to.

The toolkit is optional Markdown. No build file reads it, and removing it does not affect SimLoop.

## Review what the agent changes

Someone on the team needs to understand the robot code and be able to debug it without the agent.
Ask for an explanation of the hardware names, units, tick order and assertions before accepting a
change. Compare the model's constants with your measurements. A passing simulation checks behaviour
against that model; it does not prove that the real robot will behave the same way.

Use the learning prompt when a student needs an explanation. Use the review procedure to check a
change, and make the relevant assertion fail once before trusting a new test.

## Which tool

The instructions use the `AGENTS.md` format. Confirm that your coding tool has loaded them; tools
can differ in how they find project instructions. You can also ask it to read the file directly.
Skills and prompts are plain Markdown and can be supplied in the same way.

Horizon uses Claude Code and has exercised its paths most. The setup below includes those paths.

## Setting it up

Two steps for most tools, and two more if you use Claude Code.

**0. Get this folder into your FTC project first.** Copy the whole of `simloop-agents/` from the SimLoop
repository into the directory that holds `TeamCode/`, the same way you copied `simloop-starter/`. Every
path below is written relative to that. You can delete it again afterwards if you would rather keep only
the files you used — nothing depends on it staying.

**1. Copy `AGENTS.md` to the root of your FTC project** — beside `build.gradle`, not inside this folder.
That is where agents look for it.

```
your-ftc-project/
  AGENTS.md            <- step 1 puts a copy here
  TeamCode/
  simloop-starter/
  simloop-agents/      <- step 0 puts the folder here
  build.gradle
```

**2. Fill in the "About this project" section at the bottom of it.** Your subsystems, your units, where
things live. This is the step people skip and it is the step that does most of the work — an agent that
knows your robot writes noticeably better code than one that has to guess. The *Set the project up for
agents* prompt in [`prompts/`](prompts/README.md) will do the first draft by reading your code, and you
correct it.

After copying the file, confirm that your tool has loaded it. If it needs a different instructions
file, use the appropriate pointer from [`pointers/`](pointers/README.md), or ask it to read
`AGENTS.md` explicitly.

**3. Claude Code only: copy [`skills/`](skills) into `.claude/skills/`** in your project root:

```
your-ftc-project/.claude/skills/write-a-sim-test/SKILL.md
your-ftc-project/.claude/skills/add-a-mechanism/SKILL.md
your-ftc-project/.claude/skills/debug-a-failing-sim/SKILL.md
your-ftc-project/.claude/skills/review-sim-code/SKILL.md
```

They are picked up automatically — no install step — and because they live in your repository, everyone
who clones it gets them.

**On any other tool the same files are just procedures.** Paste the one you want into the conversation,
or tell the agent to read `simloop-agents/skills/<name>/SKILL.md`. Nothing in them is Claude-specific
except the folder they can be dropped into.

**4. If your project has a `CLAUDE.md`**, add an explicit pointer to `AGENTS.md` there.
[`pointers/CLAUDE.md`](pointers/CLAUDE.md) contains the import line. Confirm in the session that the
agent has loaded both the SimLoop guidance and your robot-specific instructions.

## What is in here

| File | What it is |
|---|---|
| [`AGENTS.md`](AGENTS.md) | The library overview, API inventory, simulation rules and test structure. Copy it to your repository root and add your robot's details. |
| [`skills/`](skills) | Four procedures: write a sim test, add a mechanism, debug a failing sim, review sim code. Written for any agent; drop straight into `.claude/skills/` unmodified. |
| [`prompts/`](prompts/README.md) | Copy-paste starting points, including one for learning and one for auditing your tests before a competition. |
| [`pointers/`](pointers) | One-line files for the tools that need a nudge, and for a project that already has its own instructions file. Deliberately thin — they point at the one copy rather than duplicating it. |
| [`token-savers.md`](token-savers.md) | Ways to keep context focused and compare usage without assuming fixed savings. |

## One thing you will need that is not in here

The skills all end by telling you to open a `.rlog` in **AdvantageScope** and look at the curve. That is
a separate, free application and you install it yourself — it is not part of SimLoop and not part of
this folder. Get it from [docs.advantagescope.org](https://docs.advantagescope.org/); the logs live under
`build/sim/` and you open one by dragging the file in.

This matters more than it sounds. An agent can tell you a test passed. Only you can see that the motion
looks nothing like the real mechanism.

## The one habit worth keeping

**Make every test fail once, on purpose, before you trust it.**

An agent writes tests that pass. That is what it is optimising for, and a test that passes because it
cannot fail looks exactly like a test that passes because the robot works. The difference matters most
at the moment you are relying on it.

Break the thing. Watch it go red. Put it back. Ten seconds, and it is the single practice that separates
a simulation you can trust from one that is quietly lying to you.
