<!-- Optional. Goes at .github/copilot-instructions.md in your FTC project.

     Use this pointer if the tool or project needs an explicit reference to AGENTS.md. If this
     destination already exists, add the instruction without replacing the team's current guidance.
     Confirm that the shared instructions are loaded in the session. -->

This project uses **SimLoop** to run its robot code against fake hardware in JVM unit tests.

Read `AGENTS.md` in the root of this repository and follow it. It has the rules that matter: the log-key
prefix, the tick order, the two coordinate frames, the units, and what SimLoop deliberately does not do.

Read it before writing a SimLoop test, and check the project's pinned version before calling an API.
