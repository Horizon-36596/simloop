# Programming SimLoop with an AI agent

A toolkit for teams who want to use an AI coding agent on their robot code: a context file that teaches
any agent how SimLoop works, four skills for the jobs that come up, prompts to start from, and notes on
not spending more than you need to.

**Optional. Entirely.** This folder is Markdown and nothing else. No Gradle file reads it, no test
imports it, no part of SimLoop depends on it existing. A team that wants nothing to do with AI can
delete this folder and notice no difference at all — and that is the point, not a disclaimer.

---

## Read this before the rest

### It does not replace understanding your own code

An agent will happily write a subsystem you do not understand. It will compile. It will probably even
work in the simulator. And then something goes wrong at a competition, at 9:40am, with no Wi-Fi, and the
only thing that gets the robot back on the field is somebody on the team who knows how the code is put
together.

That person cannot be conjured on the morning. **The architecture of your robot code has to live in
somebody's head**, and an agent is very good at letting you skip the part where it gets there. That is
the real risk, and it is worth naming plainly rather than burying in a footnote.

Everything in this folder is built to push the other way. The skills make the agent explain what it did
and why. The prompts ask it to show you a test failing before you trust it. There is a prompt whose
whole job is to make it teach you and then check whether you followed. Use them.

A fair test of whether you are using this well: **could you explain, to another team, why your code is
shaped the way it is?** If not, you have output but you do not have understanding, and the agent has
been doing your learning for you.

### It is not a way around learning to program

*FIRST* exists to teach people things, and a team that ships a robot nobody understands has won
nothing worth having.

But "learn to program by hand" and "learn to program with an agent" are not the same skill and neither
one is fake. Reviewing code you did not write, spotting the confident wrong answer, knowing when the
tool is out of its depth, writing a specification precise enough to be built from — those are real
skills, they are hard, and they are the ones a student will actually use in five years. This folder
treats working with an agent as **a second thing to learn**, with its own discipline, not as a way to
avoid the first.

If your team is learning to program by hand, keep doing that. This is for the teams who want the other
path available too, or who want both.

### It does genuinely lower the door

Worth saying, because the two paragraphs above are cautious and this part is the actual opportunity: a
student who has never written Java can now describe a mechanism in plain English and get a working,
tested simulation of it. A team with one programmer and six builders can have seven people contributing
to the code. Someone who would have bounced off a semicolon in week one can be useful in week one, and
can learn the language afterwards because they now have a reason to.

That is a real gain, and it is why this folder exists.

---

## Which tool

**Any of them.** `AGENTS.md` is an open format that Cursor, Codex, Gemini CLI, Copilot's coding agent,
Zed, Windsurf, Jules, Junie and Aider read from your repository root without being configured to. The
skills and prompts are plain Markdown and work anywhere you can paste text.

If you want a recommendation: we use **Claude Code**, and this folder was built while using it, so the
Claude-specific paths (`.claude/skills/`, `CLAUDE.md`) are the best-tested route. That is a preference
and an honest statement of what we have actually exercised — not a claim that the others are worse. Use
what your team has access to. Nothing here locks you in, and moving between tools costs you a file copy.

---

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

**For most tools, that is the whole setup.** Cursor, Codex, Gemini CLI, Zed, Windsurf, Jules, Junie,
Aider and Copilot's coding agent read a root `AGENTS.md` on their own, so you are done — go and use the
[prompts](prompts/README.md). The two steps below exist because Claude Code needs a little more wiring,
not because it gets more attention.

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

**4. Claude Code only: if your project already has a `CLAUDE.md`**, add one import line to it. See
[`pointers/CLAUDE.md`](pointers/CLAUDE.md) for why: Claude Code reads a root `AGENTS.md` on its own, but
only when no `CLAUDE.md` exists, so a project with both and no import silently loses everything in
`AGENTS.md`.

## What is in here

| File | What it is |
|---|---|
| [`AGENTS.md`](AGENTS.md) | The one that matters. What SimLoop is, the nine rules that stop an agent writing code that compiles and is wrong, the complete public API, and the shape of every test. Copy it to your repository root. |
| [`skills/`](skills) | Four procedures: write a sim test, add a mechanism, debug a failing sim, review sim code. Written for any agent; drop straight into `.claude/skills/` unmodified. |
| [`prompts/`](prompts/README.md) | Copy-paste starting points, including one for learning and one for auditing your tests before a competition. |
| [`pointers/`](pointers) | One-line files for the tools that need a nudge, and for a project that already has its own instructions file. Deliberately thin — they point at the one copy rather than duplicating it. |
| [`token-savers.md`](token-savers.md) | Spending less. Techniques first, because they are free and work better than the tools. |

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
