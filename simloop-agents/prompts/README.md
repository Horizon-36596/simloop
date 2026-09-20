# Prompts

Copy one of these, paste it into whatever agent you use, and fill in the `<angle brackets>`. They work
in any tool. They assume the agent can read your repository and run `./gradlew`; if yours cannot, paste
the relevant file in with the prompt.

These are starting points, not incantations. The reason they are longer than "write me a test" is that
every extra sentence is a decision the agent would otherwise make for you, silently and probably wrong.

---

## Set the project up for agents

Run this once, in the root of your FTC project, after copying `simloop-agents/` in.

```
Read simloop-agents/README.md and set this project up for AI-assisted development with SimLoop. Copy simloop-agents/AGENTS.md to the repository root, then fill in the "About this project" section at the bottom of it by actually reading my code: list my subsystems one line each with the units each one uses, say where my robot code lives, where my sim config lives if I have one, and where my tests live. Do not invent a subsystem I do not have, and leave a line blank rather than guessing at it.

Then tell me which of the other files in simloop-agents/ apply to the tool I am using, and stop. Do not copy anything else in without telling me what it does first.
```

---

## My first simulated test

```
I want my first SimLoop simulation test. The subsystem is <SubsystemName>, in <path/to/Subsystem.java>. Read it before you write anything, and tell me what units its position is in and what devices it asks hardwareMap for.

The behaviour I want to pin down is: <describe it in one sentence - "the slide reaches scoring height in under two seconds and does not overshoot by more than an inch">.

Follow simloop-agents/skills/write-a-sim-test/SKILL.md. Before you finish, break the subsystem on purpose so the test goes red, show me that it went red, then put it back. I want to see the test fail once before I believe it.
```

---

## Add a mechanism to the simulation

```
Add <MechanismName> to my SimLoop simulation so it can be tested without a robot. The subsystem is at <path/to/Mechanism.java>.

Follow simloop-agents/skills/add-a-mechanism/SKILL.md. Read the subsystem first and tell me the four things that skill asks for - device names, what it commands, what it reads, what units - before writing any code, and wait for me to confirm them.

Here is what I know about the real mechanism: <travel range, weight, motor, gear ratio, anything you have measured. Say "I do not know" for the rest - that is a real answer>.

Label every constant you write with where it came from. If you estimated it, say estimated. I would rather have an honest guess I can go and measure than a number that looks authoritative.
```

---

## Something is failing and I do not know why

```
This SimLoop test is failing and I do not know why. Here is the exact output:

<paste the whole failure, including the stack trace>

Follow simloop-agents/skills/debug-a-failing-sim/SKILL.md. Work through the signature table first, and tell me which row you think it is and why before changing anything.

Do not change my robot code to make the test pass unless you are certain the robot code is genuinely wrong - and if you think it is, say what would change on the real robot and let me decide.
```

---

## Review before I merge

```
Review the SimLoop test and sim-harness changes in <this branch / these files> against simloop-agents/skills/review-sim-code/SKILL.md.

The question I care about most: for every assertion in there, name a change to my robot code that would make it fail. If you cannot name one for an assertion, tell me - that assertion is not testing anything and I would rather know now.

Report one line per finding. No praise, no summary of what the code does. If it is clean, say so in one line.
```

---

## Explain this to me, I am learning

Use this one often. It is the difference between an agent that builds your robot and an agent that
teaches you to.

```
Explain <the thing - the test you just wrote, the frame conversion, the first-order plant model> to me as if I am an FTC student who can program but has never used a simulator.

Tell me what would go wrong on the real robot if this were wrong, because that is what I actually need to understand. Then ask me two questions to check whether I followed it, and wait for my answers before telling me if they were right.
```

---

## Before a competition

```
Look at every SimLoop test in this project and tell me honestly which of them would catch a real problem and which of them are passing no matter what the robot does. For each one, give me a single line: the test name, and either the specific bug it would catch or the reason it cannot fail.

Do not fix anything yet. I want the list first.
```
