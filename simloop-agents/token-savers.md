# Spending less on agents

Usage depends on the tool, model, plan and amount of context processed. Use your tool's own usage
report to compare changes; this guide makes no percentage-savings promise.

## Give the agent a bounded job

Name the file, behaviour and success condition. For example:

> Read `TeamCode/.../Slide.java`. Find why the slide overshoots, preserve its hardware names and
> units, and verify the change with the existing slide test.

Include the exact error and relevant test output. Keep the full log available, but start with the
failure and its surrounding lines rather than pasting every successful build step.

## Keep the facts easy to find

Put shared project facts in `AGENTS.md`: dependency versions, hardware names, units, tick order and
required checks. Confirm that the tool has loaded it. Link to the source of a fact instead of keeping
several copies that can disagree.

When you already know the relevant file or symbol, name it. When you do not, let the agent inspect
the dependencies it needs; an artificially narrow view can produce an incomplete fix.

## Keep the context relevant

Use a separate conversation for an unrelated task when the earlier history no longer helps. Keep
an ongoing fix in the same conversation while its decisions and evidence are still useful. A reset
that forces the agent to rediscover the project can waste work too.

Before handing work to a fresh conversation, save the changed files, current hypothesis, exact
commands and results, and what remains unresolved. A short handoff is more useful than a transcript
of every attempt.

## Ask for the output you need

For a code review, ask for actionable findings with file locations and evidence. For a completed
change, ask what changed, which checks ran and any remaining limitation. Shortening the report can
help without removing the investigation or required verification.

## Measure changes on comparable work

If you try a different model, context setting or output-compression tool, compare it on similar
jobs. Record usage, retries and whether the result was correct. Smaller output alone does not show
that the whole task cost less.

Change one thing at a time. Keep the configuration that completes the work reliably within your
team's budget, using the current pricing and limits shown by your provider.
