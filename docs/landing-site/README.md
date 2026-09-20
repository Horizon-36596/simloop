# The landing site for `libraries.horizon36596.org`

These files are **not part of the SimLoop library** and nothing builds them. They live here so that the
page which fronts the domain is version-controlled somewhere rather than existing only in a browser tab.

They belong in a different repository: **[`Horizon-36596/Horizon-36596.github.io`](https://github.com/Horizon-36596/Horizon-36596.github.io)**,
public, at its root. That name is not a choice — GitHub recognises `<owner>.github.io` as the
organisation's Pages site and nothing else does the job.

**That repository now exists and is serving these files**, since 2026-09-19, at
<https://horizon-36596.github.io/> — and at `libraries.horizon36596.org` once the DNS record is added.
**Edit them there, not here.** This copy is the one they were created in and is kept so the page is
version-controlled somewhere a reader of this repository will find it; it is not what is served, and
nothing syncs the two.

## Why a separate repository at all

Because the documentation for every Horizon library is meant to live under one domain, at a path:

```
libraries.horizon36596.org/            <- these files
libraries.horizon36596.org/simloop/    <- the SimLoop repository's Pages site
libraries.horizon36596.org/<next>/     <- whatever comes next, no DNS needed
```

GitHub gives that for free, but only in one direction: a project site with **no custom domain of its
own** is served under the organisation site's domain, at the project's repository name. So the domain is
attached to *this* landing repository, and every library repository leaves its Custom domain field
empty. The full procedure, with the DNS record written out, is in
[`SimLoop/PUBLISHING.md`](../../SimLoop/PUBLISHING.md) under "Turning it on".

**The path is the repository name, and it is case-sensitive.** That is why the SimLoop repository is
named `simloop` in lower case.

## Installing it (already done — this is the record of how)

Copy `index.html`, `404.html` and `horizon-mark.svg` to the root of `Horizon-36596.github.io`, commit,
and set that repository's Pages source to **Deploy from a branch → `main` → `/ (root)`**. There is no
build step, no dependency and no framework — it is three files.

## The `CNAME` file in that repository is load-bearing

`Horizon-36596.github.io` publishes from a branch, and on that path GitHub stores the custom domain as a
`CNAME` file at the root of the publishing source — it committed one itself when the domain was saved.
**Do not delete it.** Every library's documentation is served under that domain, so removing it takes
them all down at once.

The library repositories are the opposite case and must have no `CNAME` file, because they publish from
a GitHub Actions workflow, where a `CNAME` file is ignored and a custom domain would make the repository
claim a domain root instead of a path under this one. Same file name, opposite rule, decided by how the
site is published.

## Adding a library to the list

One `<a class="library">` block in `index.html`, copied from the SimLoop one. The tags are plain text:
status, language, licence, package root. Nothing here is generated, so nothing here can go stale without
someone choosing not to update it.

## Keeping it consistent with the docs

The colours, the three type faces and the amber→orange→crimson rule under the heading are the same
tokens as `SimLoop/docs/stylesheets/horizon.css`, which took them from the team website's
`tailwind.config.ts`. Three files now hold those values and they are meant to stay identical — if one
drifts, it is wrong rather than different.

Dark only, with no toggle, for the same reason the documentation site is: the team site declares
`color-scheme: dark` and ships no light mode.
