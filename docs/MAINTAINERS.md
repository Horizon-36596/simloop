# For maintainers

Notes for Horizon (FTC 36596) on running this repository in public. Contributors want
[`CONTRIBUTING.md`](../CONTRIBUTING.md) instead.

## Settings to apply the moment the repository is public

A private repository on the free plan cannot have branch rules, so these wait until it is public. They
are all in one script, which an admin runs once, with the owner's OK:

```bash
bash tools/maintainers/apply-repo-settings.sh --yes
```

It refuses to run while the repository is private. What it sets:

| Setting | Value |
|---|---|
| Ruleset `checks` on `main` | Pull request required (0 approvals). Required checks `test`, `leak-gate`, `secret-scan`, on an up-to-date branch. No force push, no deletion. **No bypass for anyone**, the owner included, so the leak gate can never be skipped. |
| Ruleset `review` on `main` | 1 approving review, from a code owner. Stale approvals dismissed on a new push. Repository admins may bypass it, but only through a pull request. |
| Discussions | On. The issue chooser sends questions there. |
| Private vulnerability reporting | On. `SECURITY.md` and the Code of Conduct point to it. |
| Secret scanning | On, with push protection. |
| Actions | Default token read-only. Pages built by Actions (already the case; the script just confirms it). |
| Labels | Creates `fidelity`, which the "Sim doesn't match my robot" form uses. |

GitHub applies both rulesets together. An outside contributor's pull request needs every check and
an approval from the code owner (`.github/CODEOWNERS`: `@featurescript`). Horizon's own pull requests,
including ones an agent opens through the owner's `gh` login, still need every check but not a second
person. Nobody can push straight to `main`.

A ruleset cannot name one user as a bypass actor. The API accepts only apps, organisation admins,
repository roles, teams and deploy keys. So the `review` bypass is the repository **Admin** role, which
covers every repository admin, not only `featurescript`. To narrow it, lower the other admins to Write
or Maintain in Settings, Collaborators.

## Steps when the repository goes public

1. Flip the visibility to public.
2. Run `bash tools/maintainers/apply-repo-settings.sh --yes`.
3. **Watch the first CI run before merging anything.** Actions are blocked on this private repository
   by the organisation's billing limit, so no workflow has run on PR #2 (alpha prep) or will run on the
   sync PR until the repository is public, when Actions become free. Re-run the checks on PR #2 and on
   the sync PR, and merge only when `test`, `docs`, `leak-gate` and `secret-scan` are green. The
   `secret-scan` job has never run anywhere yet.

## People

- **Outside contributors** fork and open a pull request. They need no access at all.
- **Trusted regular contributors** can be added as outside collaborators on this repository only
  (Settings, Collaborators), with the Triage or Write role. Never add them to the organisation.
- **Fork pull requests** run CI with a read-only token and no secrets (`pull_request`, never
  `pull_request_target`). GitHub asks a maintainer to approve the first run for a first-time
  contributor; check the diff touches nothing under `.github/` you did not expect before approving.

## The leak gate

`tools/leak-gate/` runs on every pull request. Its rules are public, so they name only public things.
The private robot repository runs its own private blocklist before anything is upstreamed here; that
is the check for our own class names. If the public gate fails on something safe, add an exact
`file|line` exception with a reason to `allowlist.txt`, never a looser rule. The gate prints
exceptions that no longer match anything, so remove those.

## Reviews

Alpha means best effort, but try to give every pull request a first reply within a week, even if it
is only "thanks, we will look at this after our next event". Label easy issues `good first issue`; it
is the first place new contributors look.
