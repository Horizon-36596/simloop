<!-- Goes at the ROOT of your FTC project, beside build.gradle.

     You only need this file if your project ALREADY has a CLAUDE.md. Claude Code reads a root
     AGENTS.md by itself when there is no CLAUDE.md, and stops looking at AGENTS.md the moment one
     exists - so a project with both, and no import line, silently loses everything in AGENTS.md.

     If you already have a CLAUDE.md, do not replace it. Add the one @-import line below to it. The
     import is the point: the SimLoop guidance stays in one file, and your own instructions stay
     yours. Two copies of the same guidance drift, and the stale one is the one an agent reads. -->

@AGENTS.md
