# Live-site documentation port, 2026-10-24

This branch targets `main` at `7e06448`, the source used by the SimLoop Pages workflow. It ports the
applicable documentation work from #26 without importing the pending vision implementation from #24.

The vision page keeps the geometry-only API in this source: no distortion, synthetic-frame renderer,
robot-body occluders or tilted-tag methods are advertised. The projection example now handles empty
sightings and a missing plane intersection, and states the height and coordinate frame.

The user-facing workflow is the simulation library plus a separate AdvantageScope window. The
standalone application around SimLoop is not ready and is not part of this release.

The remaining changes cover installation, the verified slide example and plot, API landing routing,
logging/model explanations, agent setup and narrow-screen readability. No runtime API, dependency,
version, tag, deployment or repository visibility is changed.

The live `/simloop/` and `/simloop/javadoc/` routes returned 404 during the review. The Pages API also
returned 404 for this repository. Account payment/spending-limit restrictions prevented hosted CI
jobs from starting on #26. A merged source PR alone does not restore Pages or demonstrate a deployment.

## Verification

- `:SimLoop:docsSite --offline` passes, including API doclint, version and tutorial-source checks.
- The slide example passes on `main`: one test, no failures, errors or skips. Its Java source blob is
  identical on `main` and the vision branch used to record the plotted values.
- The leak gate passes all 10 rules over 393 files.
- The rendered crawl checks 29 guide pages, 2,196 local links and 59 image references with no missing
  files, anchors, images or alt text. The guide also passes the 320 px page-width check.
