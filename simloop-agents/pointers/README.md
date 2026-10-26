# Point your tool at the shared instructions

Keep the SimLoop guidance in the root `AGENTS.md` of your FTC project. These files are examples for
tools that use another project-instructions file. Confirm in the session that the agent has loaded
the shared guidance; do not assume that copying a filename was enough.

| Example | Where it belongs in your FTC project | What to copy |
|---|---|---|
| [`CLAUDE.md`](CLAUDE.md) | Root `CLAUDE.md` | The `@AGENTS.md` import line |
| [`copilot-instructions.md`](copilot-instructions.md) | `.github/copilot-instructions.md` | The instruction to read root `AGENTS.md` |

If the destination already exists, add the pointer to it and keep the team's current instructions.
Do not replace the file with this example. Check your tool's current instructions mechanism if it
uses a different path or import syntax.

After setup, ask the agent to identify your pinned SimLoop version, the units your mechanisms use,
and the rule about `RealOutputs/` log keys. If it has not loaded those facts, fix the pointer before
asking it to write a test.
