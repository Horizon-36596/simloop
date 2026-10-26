# Publishing SimLoop

How a release is cut, what it is verified against, and what would have to change to put it on Maven
Central. The build wiring this describes lives at the bottom of `SimLoop/build.gradle`.

## Coordinates

| | Value |
|---|---|
| Group (in the POM) | `org.horizon36596` |
| Artifact (in the POM) | `SimLoop` |
| Version (in the POM) | `0.2.0-alpha.1` |
| **What a consumer actually writes** | **`com.github.Horizon-36596:simloop:v0.2.0-alpha.1`** |
| Packaging | `aar`, plus sources and javadoc jars, plus the keyboard window jar (classifier `desktop`) |

**JitPack ignores every one of those first three values and derives its own.** The group becomes
`com.github.<owner>`, the artifact becomes the **repository** name, and the version is the git tag
verbatim, leading `v` included. A consumer writes `com.github.Horizon-36596:simloop:v0.2.0-alpha.1`;
JitPack's served POM uses that coordinate too. The declared `org.horizon36596:SimLoop:0.2.0-alpha.1`
coordinate belongs to `publishToMavenLocal` and a possible Maven Central publication.

That is JitPack's **single-module** spelling, and this build gets it because `jitpack.yml` installs
exactly one module. A build that published two would get the **multi-module** spelling,
`com.github.<owner>.<repo>:<module>` — which is what this file used to claim was already happening.
It was wrong, and it stayed wrong right up until a real build could be read: on 2026-09-19 the first
JitPack build of `v0.1.0-beta1` served
`com/github/Horizon-36596/simloop/v0.1.0-beta1/simloop-v0.1.0-beta1.aar`, and the two-dot spelling
404s. **Do not publish a second module** without changing every coordinate in this repository at the
same time. That is why the keyboard window (`:SimLoop:desktop`) ships as a classifier jar of this one
publication, `com.github.Horizon-36596:simloop:<tag>:desktop`, and not as a module of its own.

## Version scheme

`MAJOR.MINOR.PATCH`, with an `-alpha.N` suffix while the API is still moving. The current release is
`0.2.0-alpha.1`, the first public alpha. `0.1.0-beta1` came before it and is superseded; it gets no fixes.

- **`-alpha.N`** says the API may change between alpha versions without a major bump. That is the true
  state today and should stay until teams outside Horizon have used it for a season.
- **Before 1.0, MINOR carries what MAJOR would.** `0.2.0-alpha.1` moved the FTC SDK from 11.2.1 to
  12.0.0, which the rule below makes a MAJOR change; under `0.x` that is a MINOR bump, and the release
  notes say so prominently. A consumer's project must be on SDK 12.0.
- **MINOR** for new capability and for any source-compatible change.
- **MAJOR** for anything a consumer has to edit code for — a renamed method, a changed signature, a
  removed class, or a bumped FTC SDK version, because the SDK is `api` and a consumer compiles against
  it through us.

### Behaviour changes since `0.1.0-beta1` was tagged

Signature-compatible, so no MAJOR bump under the rule above, but each one changes what existing consumer
code *does* and belongs in the `0.2.0-alpha.1` release notes. All from the 2026-09-17 adversarial review,
except where a row says it was overtaken.

| What changed | What a consumer sees |
|---|---|
| `FakeMotor.setMode(RUN_TO_POSITION)` | Overtaken in `0.2.0-alpha.1`: `RUN_TO_POSITION` is now modelled, the SDK side exactly and the hub firmware as a labelled approximation ([docs/run-to-position.md](docs/run-to-position.md)). For a while between the two releases it threw `UnsupportedOperationException`; it no longer does. |
| `FakeMotor.setVelocity`, `setMotorDisable`, `STOP_AND_RESET_ENCODER` | Now actually affect the shaft, the encoder count and the reported current. A test that passed because these were no-ops can legitimately go red. |
| `FakeServo.setPosition` with `REVERSE` + `scaleRange` | Direction is applied before scaling, matching `ServoImpl`. `getScaledPosition()` returns a different number than before for that combination — the old one was a mirror image. `scaleRange` now clips out-of-range endpoints rather than throwing. |
| `EnvelopeGuardrails` guardrail names | `withinBound`, `loopTimeUnderBudget` and `safeStopAtEnd` now embed their thresholds in `Guardrail.name()`. Any persisted last-best score from an earlier version will read as a different envelope in `Gate` and must be re-baselined. Non-finite and negative thresholds are now rejected at construction. |
| `RunResult.fromRlog` | `RealOutputs/Console` and `RealOutputs/Logger/QueuedCycles` are no longer scoreable, so `keys()` and `frames()` no longer contain them. |

`SimLoop/build.gradle` declares the source version. Installation snippets, the starter, agent guides
and the site footer repeat it. Review all current-version references during a release; keep references
to older tags where they explain compatibility or historical checks.

## Cutting a release

1. **Green first.** `./gradlew :SimLoop:testDebugUnitTest` — all of it, not the tests you touched.
   Then `./gradlew :SimLoop:apiDocs`, which must report **zero warnings**: the API reference is published
   and read by people and agents who have nothing else, so a missing unit is a release defect there in a
   way it is not in an IDE.
2. **Update the approved version** in `SimLoop/build.gradle` and its current-version references.
   Search the root and module READMEs, `SimLoop/docs/`, `SimLoop/mkdocs.yml`, `simloop-starter/` and
   `simloop-agents/`. Review each match rather than replacing historical tags blindly. Run
   `./gradlew :SimLoop:docsSite`; its version check verifies the footer, and its example check verifies
   the first-test snippet. These checks do not replace reviewing installation coordinates.
3. **Verify the artifact locally before anyone can consume it** — the section below says how, and it is
   the part people skip.
4. **Commit, tag, push the tag.** The tag is the version, e.g. `v0.2.0-alpha.1`.
5. **Build that tag on JitPack yourself, and read the log**, at
   `https://jitpack.io/#Horizon-36596/simloop`. Before giving teams a new coordinate, verify:
   - **Whether the coordinate is right at all.** JitPack derives the artifact id from the module, and
     the only way to be certain which spelling it serves is to see it serve one.
   - **Whether JitPack's environment builds the tag.** `jitpack.yml` pins `openjdk17`. Check the new
     build's log rather than assuming a local build proves the hosted environment works.

   Check repository visibility and JitPack access before relying on a new hosted build. The owner
   decides any visibility or hosting change separately.
6. **Tell the teams the coordinate and the two repository lines**, not just the coordinate. The
   PsiKit one is the thing that bites — see below.

Tagging and pushing leave the machine, so they are a human's call, not an agent's.

## Verifying a release locally

```bash
./gradlew :SimLoop:publishToMavenLocal
```

writes the AAR, the sources jar, the POM and the Gradle module metadata into `~/.m2/repository/`.

Then check the POM lists **`RobotCore`, `Hardware` and PsiKit `core` at `compile` scope**. The current
build does not depend on PsiKit `ftc`. If a required dependency moves to `runtime`, a consumer can
lose the SDK types or logging API it needs while compiling tests.

The real check is to compile something against it from outside this repo: a throwaway Android library
module whose only declared dependency is `org.horizon36596:SimLoop:<version>`, with `mavenLocal()`
first in its repositories, containing one class that does this much —

```java
FakeHardwareMap map = new FakeHardwareMap();
FakeMotor motor = new FakeMotor(1000.0, 2.0);
map.register("fl", motor);
DcMotorEx asSdkType = motor;            // names an SDK type: this is the line that fails on runtime scope
Logger.recordOutput("Demo/time", 0.0);  // names a PsiKit type, same reason
```

That was run for `0.1.0-beta1` and passed, and the same project with the dependencies switched back to
`implementation` failed with `cannot access HardwareMap` — the scope claim above is measured, not
assumed.

On 2026-10-24, JitPack's [alpha build-status API](https://jitpack.io/api/builds/com.github.Horizon-36596/simloop/v0.2.0-alpha.1)
reported `ok` for tag commit `9116406` with JDK 17. Its
[served POM](https://jitpack.io/com/github/Horizon-36596/simloop/v0.2.0-alpha.1/simloop-v0.2.0-alpha.1.pom)
returned HTTP 200 and listed those three dependencies at compile scope. This confirms the existing
tag's hosted metadata; it does not validate unpublished source or guarantee a future build's access.

## Why the library is a subproject and not the root project

This repository holds one library, and that library is a Gradle **subproject**. Both the `SimLoop/` path
prefix and the `:SimLoop` task prefix that appear throughout this file and the workflows are therefore
correct, and they are meant to stay:

- The Android Gradle Plugin's library plugin is applied to a subproject, which is the layout it expects
  and the layout every FTC project already uses. A root project that is itself an Android library is a
  shape nothing else in this ecosystem has.
The coordinate does **not** depend on it — an earlier version of this file said it did. JitPack names
the artifact after the repository while the build publishes one module, whatever the directory layout
is. What would rename the artifact is publishing a second module (see Coordinates, above).

The root of the repository holds what belongs to the repository rather than to the library:
`settings.gradle`, `build.gradle`, `gradlew` and the wrapper, `jitpack.yml`, `LICENSE`, `NOTICE.md`,
`README.md` and
`.github/workflows/`. Everything else is under `SimLoop/` and travels with the module.

## What consumers have to add that a POM cannot tell them

**PsiKit is not on Maven Central.** It is served from `https://repo.dairy.foundation/releases`. A POM
records what a dependency is and never where to find it, so a consumer who adds JitPack and nothing
else fails at resolution with `Could not find org.psilynx.psikit:core`, which looks like our bug. The
README's install section leads with this for that reason.

## Hosting the documentation site

The site is at `SimLoop/docs/`, it builds with one command, and CI builds it on every push so it can
never become something that only builds on one laptop:

```powershell
./gradlew :SimLoop:docsSite
```

That renders the markdown, checks that every internal link resolves, generates the API reference with
doclint fatal, checks that the version stamped in the site footer matches the version this build
publishes, and drops the API reference inside the site at `/javadoc/`. It writes to
`SimLoop/build/docs/site`. Its one dependency is pinned in `SimLoop/docs-requirements.txt`:

```powershell
pip install -r SimLoop/docs-requirements.txt
```

`.github/workflows/docs-publish.yml` is configured to publish on a `v*` tag. Existing tags do not prove
that the workflow deployed successfully; check its latest run and the actual site. The settings below
span this repository and the shared-domain landing repository. The `test.yml` workflow also builds the
site and uploads an artifact without deploying it.

### Turning it on — four steps, all of them a human's

The site does not live on its own domain. It lives at **`libraries.horizon36596.org/simloop/`**, a path
under a domain that belongs to a *different* repository — Horizon's organisation Pages site — so that a
second library later gets `libraries.horizon36596.org/<its-name>/` and needs no DNS of its own.

That is the whole reason the steps below look the way they do, and it is the one thing to get right:
**the custom domain is set on the landing repository, never on this one.** GitHub's rule is that a
project site with no custom domain of its own is served under the organisation site's domain, at the
project's repository name — *"if the custom domain for your user site is `www.octocat.com`, and you have
a project site with no custom domain configured that is published from a repository called
`octo-project`, the GitHub Pages site for that repository will be available at
`www.octocat.com/octo-project`."* Setting a custom domain **here** would override that and take the
domain root, which is the opposite of what is wanted.

The path is the repository name, and it is case-sensitive. This repository is `simloop`, lower case, so
the path is `/simloop`. It was renamed from `SimLoop` on 2026-09-19 for exactly that reason, before any
tag existed — which also moved the published coordinate to `com.github.Horizon-36596:simloop`, since
JitPack takes the artifact id straight from the repository name.

The landing repository was created on 2026-09-19. Check the current DNS, Pages settings and latest
deployment before repeating setup steps. A successful local build does not show that the project
site is deployed.

**1. Create the landing repository.** ✅ **Done 2026-09-19** — `Horizon-36596/Horizon-36596.github.io`,
public, with the landing page at its root and Pages serving `main` / `(root)`. The name is not a choice:
GitHub recognises `<owner>.github.io` as the organisation site and nothing else. The copy under
`docs/landing-site/` in this repository is the source those files were taken from; that repository is
where they are edited now.

**2. Add the DNS record.** One record, at whoever hosts `horizon36596.org`:

| Field | Value |
|---|---|
| **Type** | `CNAME` |
| **Name** / Host | `libraries` |
| **Value** / Target / Points to | `horizon-36596.github.io` |
| **TTL** | leave it at the default, or 3600 |
| **Proxy status** (Cloudflare only) | **DNS only** — grey cloud, not orange |

Three things about that record, each of which has cost somebody an afternoon:

- The **Name** is `libraries`, not `libraries.horizon36596.org`. Most registrars append the domain for
  you, and a full name here becomes `libraries.horizon36596.org.horizon36596.org`. A few registrars do
  want the full name — if yours shows existing records with their full names, match what you see.
- The **Value** is the organisation's Pages host, `horizon-36596.github.io`, and it is **not** a
  repository: there is no `/simloop` on the end. GitHub's own instruction is that the record "should
  always point to `<user>.github.io` or `<organization>.github.io`, excluding the repository name". Some
  registrars require a trailing dot (`horizon-36596.github.io.`); GitHub does not mention one, so if the
  field rejects the value without it, that is your registrar's convention, not GitHub's.
- **On Cloudflare, turn the proxy off** until GitHub has issued the certificate. *Unverified:* GitHub's
  own documentation says nothing about proxies, but this is a common report — a proxied record hides the
  real origin from the certificate check. If the certificate does issue with the proxy on, leave it on.

**3. Point the landing repository at the domain.** In `Horizon-36596.github.io` → Settings → Pages:
Source **Deploy from a branch**, branch `main`, folder `/ (root)`; then Custom domain →
`libraries.horizon36596.org` → **Save**, and tick **Enforce HTTPS** once the certificate has issued.

**4. Enable Pages on THIS repository, with no custom domain.** ✅ **Done 2026-09-19** — Source is
**GitHub Actions** and the Custom domain field is empty. Not "Deploy from a branch": `docs-publish.yml`
deploys the built artifact directly and there is no `gh-pages` branch to point at. That empty Custom
domain field is what makes this site answer at `/simloop` under the landing domain rather than trying to
own a domain of its own, so **leave it empty** — filling it in is how this arrangement breaks.

**The two repositories have opposite rules about `CNAME` files, and both are right.** The landing
repository publishes **from a branch**, and for that path a custom domain *is* stored as a `CNAME` file
in the publishing source — GitHub committed one to `Horizon-36596.github.io` by itself the moment the
domain was saved, containing `libraries.horizon36596.org`. **Leave it there.** Deleting it as tidy-up,
on the strength of the paragraph below, drops the domain for every library at once.

This repository is the other case. There is deliberately **no `CNAME` file in this repository**, and one
should not be added back. It was
deleted on 2026-09-19 along with the old single-domain plan. Two reasons it would be worse than useless:
a `CNAME` file is how a repository claims a domain root, which is the behaviour being avoided here; and
on this publish path it would not work anyway — GitHub's documentation is explicit that **"if you are
publishing from a custom GitHub Actions workflow, any CNAME file is ignored and is not required."**
(*Troubleshooting custom domains and GitHub Pages*, under "CNAME errors", checked 2026-09-19.)

**5. Let tags deploy.** ✅ **Done 2026-09-19** — and it is the step nobody predicts. Turning Pages on
creates a `github-pages` *environment* whose deployment branch policy allows **the default branch only**.
A tag is not a branch, so the first `v*` push failed at `deploy`, minutes after the same workflow had
worked by hand from `main`:

```text
Tag "v0.1.0-beta1" is not allowed to deploy to github-pages due to environment protection rules.
```

That message is clear, but it is only visible in the right place. The rejection happens before any step
runs, so it is an **annotation on the run**, not a line in a step log — the job has zero steps, and
`gh run view --log-failed` answers `log not found` and stops there. Read it in the Actions tab, or:

```powershell
gh api repos/Horizon-36596/simloop/check-runs/<job id>/annotations
```

One policy entry fixes it:

```powershell
gh api -X POST repos/Horizon-36596/simloop/environments/github-pages/deployment-branch-policies -f 'name=v*' -f 'type=tag'
```

In the UI: Settings → Environments → `github-pages` → Deployment branches and tags → Add rule → ref type
**Tag**, pattern `v*`. **The next library will need this too.**

With those done, pushing a `v*` tag publishes the site, and so does running the workflow by hand from the
Actions tab — which is how the site first got there on 2026-09-19, before any tag existed.

### Checking it worked

GitHub's documentation gives one number worth knowing: **"It can take up to an hour for your site to
become available over HTTPS after you configure your custom domain."** DNS propagation is on top of that
and depends on your registrar. Both are waiting, not failure.

- `https://libraries.horizon36596.org/` loads the landing page, over HTTPS, with no certificate warning.
  If this one fails, nothing below it can work — the domain is not attached yet. The github.io addresses
  keep working throughout; they are the same site, and comparing the two is how you tell a DNS problem
  from a site problem.
- `https://libraries.horizon36596.org/simloop/` loads the documentation site. If the landing page works
  but this path returns 404, check this repository's Pages configuration, visibility, workflow result
  and published artifact. Also confirm that no separate custom domain is set on this project site.
- The footer of every page reads the version you just tagged.
- `https://libraries.horizon36596.org/simloop/api-reference/` loads the API guide page, and its button
  opens `https://libraries.horizon36596.org/simloop/javadoc/`. Check that both routes exist in the
  artifact. If only Javadoc is missing, verify that the deployment used `docsSite`, which copies it,
  rather than MkDocs alone.

## What a Maven Central release would additionally need

Not done, and not needed for an alpha with a handful of teams. For when it is:

- A verified namespace on Sonatype Central for `org.horizon36596`, which means proving control of
  `horizon36596.org` with a DNS TXT record. That is the reason the declared group is a domain rather
  than a `com.github.*` one: the namespace is claimable at the moment it is wanted. An
  `io.github.horizon-36596` group is the fallback that needs no domain.
- **GPG signing** of every artifact, with the key published to a keyserver. This is the real work: a
  `signing` block, and a key that is not in the repository and not in an agent's reach.
- ~~A **javadoc jar**.~~ **Done, 2026-09-17.** Every release carries `SimLoop-<version>-javadoc.jar`,
  built by `./gradlew :SimLoop:apiDocs` - the real javadoc tool with `-Xdoclint:all` and `-Werror`, so a
  missing comment, a missing `@param`/`@return` or a broken `{@link}` fails the build rather than printing
  a line. It reports zero warnings today, and that is the state a release is expected to be in.

  It is **not** AGP's `withJavadocJar()`, which was tried first and rejected the same day: AGP's javadoc
  task runs Dokka, which predates the inline `{@return ...}` tag this module's accessors use and emitted
  it as literal text, braces and all, on every getter page. `SimLoop/build.gradle` says this at the
  publishing block. The jar shipped in the POM and the HTML on the documentation site are now the same
  build, which is also why there is nothing to keep in sync.
- A `repositories { maven { url = ...; credentials { ... } } }` block, with credentials read from an
  environment variable or `~/.gradle/gradle.properties` and never from a file in the repository.
- Central versions are **immutable**. Treat published alpha tags as immutable too: correct a mistake
  with a new version so cached artifacts and source keep referring to the same code.

The POM already carries everything else Central validates — name, description, url, licence, developer
and SCM — so this is a known and bounded list, not an unknown.

## Decisions a human has made

Settled 2026-09-17. The answers and the reasoning behind each are in
`docs/progress/decisions/L.md`; this is the short form, because these are the facts the rest of this
file assumes.

- **The licence is BSD 3-Clause, `Copyright (c) 2026, Horizon (FTC 36596)`.** From 2026-09-17 it was
  a copyleft licence; it went back to BSD 3-Clause on 2026-10-14 for the public alpha, so that any team
  can use, change and ship it, and publish its own robot code under any licence. `v0.1.0-beta1` was
  released under the copyleft licence; `v0.2.0-alpha.1` is the first release under BSD 3-Clause. Contributions come in
  under the same licence (GitHub's terms), with no CLA and no sign-off. Depended-on third-party code
  keeps its own licence; the SDK's `RobotCore` and `Hardware` stay BSD, and PsiKit is BSD 3-Clause.
- **A person cuts a release, and that person is the repository owner.** The release *is* the tag: pushing
  `v<version>` is what declares a commit usable, and everything downstream reacts to it. No workflow
  creates a tag. This is the same rule root `CLAUDE.md` applies to push, PR and merge — the act that
  leaves the machine is a human's.
- **The original plan tied the documentation site to public source.** Published tags now exist and the alpha's JitPack metadata is available. Check current access before
  the next release; do not infer visibility, publication or artifact access from the original plan.
- **The library is a Gradle subproject, not the root project.** The reasoning is above,
  under "Why the library is a subproject and not the root project". Publishing one module is what
  preserves the JitPack artifact name; the directory name alone does not determine it.
- **Documentation is not versioned** until the library is past alpha. One site, no version selector, with
  the version stamped on every page.
- **Bugs go to GitHub Issues, questions to GitHub Discussions,** both on `Horizon-36596/simloop`.
  Access follows the repository's current visibility and permissions.

### Settled on 2026-09-17: the package and the Maven group

- **The Java package is `org.horizon36596.simloop` and the Maven group is `org.horizon36596`.** Settled
  at the only moment such a thing is free: nothing published, no tag cut, and no team outside Horizon
  holding a copy. Changing either one after the first team has it costs that team an edit to every import
  line, which is why this is a decision rather than a preference. `org.horizon36596` is the reverse-DNS of
  a domain Horizon controls, which is also what a Maven Central namespace verification asks for.
- **The published coordinate did not change with it.** JitPack derives its own group and artifact from
  the repository address, not from this file, so `com.github.Horizon-36596:simloop` is what a consumer
  writes either way. The declared group matters for `publishToMavenLocal` and for any future Maven
  Central release.
