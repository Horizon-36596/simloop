# Spending less on agents

Every one of these tools charges for the text going in and the text coming out. A team on a free tier
runs out; a team paying for it would rather not pay twice. None of this is specific to SimLoop, and all
of it is public.

Read the techniques first. They are free, they work in every tool, and they save more than any of the
tools below.

## The techniques

**Most of what an agent reads, it did not need.** That is where the money goes — not in the answer, in
the context that produced it.

### Point at the thing, do not paste the repository

"Read `TeamCode/.../Slide.java` and fix the overshoot" costs a fraction of "here is my project, fix the
overshoot", and the answer is better, because the agent is not choosing between forty files.

### Start a new conversation per task

A long conversation resends its whole history with every message. Ten small tasks in one conversation
can cost several times ten small conversations. When the subject changes, start over.

### Put the standing facts in a file, not in every message

That is what `AGENTS.md` in this folder is for. Written once, read automatically, never retyped. Re-typing
your conventions at the top of each request is the same tokens over and over.

### Ask for the shape of the answer you want

"One line per finding, no preamble" genuinely costs less than letting a model write four paragraphs of
throat-clearing first. Output is the expensive half per token.

### Do not make an agent search for something you already know

If you know the file, say the file. If you know the line, say the line. A five-second look on your part
removes ten tool calls on its.

### Let it fail fast

Give the agent the exact error text. An agent that has to reproduce a failure itself spends a build, a
test run, and a lot of reading to arrive where your clipboard already was.

## The tools

### RTK (Rust Token Killer)

<https://github.com/rtk-ai/rtk>

A CLI proxy. The agent runs `git status`, RTK runs the real command, and hands back a compressed version
of the output instead of the raw wall of text. A single Rust binary with no dependencies; on Claude Code
it installs a hook that rewrites eligible shell commands transparently, so nothing in your workflow
changes.

**What it claims:** 60–90% fewer tokens on common dev commands.

**What is contested:** an independent benchmark published by JetBrains measured it as *more* expensive
on real agent work at low reasoning effort and about neutral at high effort. The compression is real;
whether it saves you money depends on how much of your session is actually shell output.

So: worth trying, worth **measuring on your own usage**, not worth treating as a guaranteed saving. It
also only intercepts shell commands — an agent's built-in file-reading and search tools do not go
through it, and on a typical FTC session those are most of the traffic.

### Your tool's own context controls

Cheaper than any proxy and already installed. Most agents have a way to clear or compact the
conversation; in Claude Code that is `/clear` between tasks and `/compact` when one task genuinely needs
the history. Learn the two commands for whichever tool you use. This is the single biggest lever on the
list.

### A smaller model for the small jobs

Renaming things, writing javadoc, formatting a table, summarising a log — these do not need the most
capable model available. Most tools let you switch. The difference in cost between tiers is large and
the difference in quality on a mechanical task is not.

---

## Measure before you optimise

Both of the above sell a number. Yours will be different. Run a normal week, look at what your tool
reports you actually spent, and change one thing at a time.

An FTC team's bill is usually not dominated by clever tooling — it is dominated by long conversations
that should have been several short ones. Fix that first and you may not need any of this.
