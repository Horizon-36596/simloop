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
| Ruleset on `main` | Pull request required. 1 approving review, from a code owner. Stale approvals dismissed on a new push. Required checks: `test`, `leak-gate`, `secret-scan`. No force push, no deletion. No bypass list, so admins follow the rules too. |
| Discussions | On. The issue chooser sends questions there. |
| Private vulnerability reporting | On. `SECURITY.md` and the Code of Conduct point to it. |
| Secret scanning | On, with push protection. |
| Actions | Default token read-only. Pages built by Actions (already the case; the script just confirms it). |
| Labels | Creates `fidelity`, which the "Sim doesn't match my robot" form uses. |

Before running it, fix the code owner. `.github/CODEOWNERS` names one account, `@featurescript`,
because the organisation has no teams yet. With "review from a code owner" required, that person's own
pull requests could never be approved. Create a team (for example `simloop-maintainers`) with two or
more people in it, and change CODEOWNERS to `* @Horizon-36596/simloop-maintainers` first.

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
