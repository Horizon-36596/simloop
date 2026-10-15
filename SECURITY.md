# Security

## Reporting a problem

**Please do not open a public issue for a security problem.** Report it privately instead:

1. Go to this repository's **Security** tab.
2. Click **Report a vulnerability**.
3. Describe what you found, how to reproduce it, and what it could affect.

That opens a private advisory that only you and the maintainers can see. GitHub explains the steps in
[Privately reporting a security vulnerability](https://docs.github.com/en/code-security/security-advisories/guidance-on-reporting-and-writing-information-about-vulnerabilities/privately-reporting-a-security-vulnerability).

Things worth reporting include a secret or credential committed to this repository, a way for a pull
request to run code with this repository's permissions, or a published artifact that does not match
its source.

## What to expect

SimLoop is maintained by an FTC team, so this is best effort. We aim to reply within a week, and we
will tell you what we plan to do and when a fix is out. We are happy to credit you in the advisory.

## Supported versions

SimLoop is an alpha. Only the latest alpha release gets fixes. `v0.1.0-beta1` is superseded and will
not be patched.

## Scope

SimLoop runs in JVM unit tests on a developer's machine or in CI. It never runs on a robot and opens no
network connections. A problem in a dependency (the FTC SDK, PsiKit) belongs with that project, though
we would still like to know about it.
