<!-- Optional. Goes at .github/copilot-instructions.md in your FTC project.

     You probably do not need this. Copilot's coding agent reads a root AGENTS.md, and Copilot Chat in
     VS Code reads it as well - recent versions pick up a workspace-root AGENTS.md on their own,
     alongside this file rather than instead of it.

     It is here for two cases: a Copilot version or editor that predates that, and a project that
     already has a .github/copilot-instructions.md and wants it to point at the same place. It is a
     pointer, not a second copy, so there is nothing in it to drift. -->

This project uses **SimLoop** to run its robot code against fake hardware in JVM unit tests.

Read `AGENTS.md` in the root of this repository and follow it. It has the rules that matter: the log-key
prefix, the tick order, the two coordinate frames, the units, and what SimLoop deliberately does not do.

Do not write a SimLoop test without reading it first. The mistakes it warns about all compile, and most
of them pass.
