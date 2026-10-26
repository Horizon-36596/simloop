# Documentation review, 2026-10-24

## Source checked

GitHub was checked before editing. `main` was `7e06448`; the queued work was vision #24, streaming
RLOG reads #21, scenario cleanup #22, docs analytics #23 and branding #25. The integration comment on
#24 records a combined local run at `eb91fad`. It does not confirm a publication time.

This documentation branch is based on `feat/vision-fidelity` at `4b40ef9`. It supplements the vision
documentation already in #24. The `v0.2.0-alpha.1` tag resolves to `9116406` and does not contain the
new distortion, occlusion, tilted-tag or synthetic-frame classes. Installation docs must not imply
that those classes resolve from the pinned alpha coordinate.

## Changes made

- Marked the source/tag boundary on the home page, feature list, vision page, agent index and README.
- Linked installation to the starter at the pinned tag and explained first-run dependency downloads.
- Clarified that the installed starter tests and the slide tutorial are different examples.
- Added vision output selection, channel order, native OpenCV setup, processor initialization,
  calibration requirements, Mat cleanup and deterministic frame-sequence guidance.
- Explained projection height, robot/field coordinates, empty sightings and a missing plane intersection.
- Added logging ownership and corrected the distinction between model time and I/O waits.
- Expanded nondeterminism troubleshooting beyond wall-clock reads.
- Replaced broad claims about automatic tool support and repeated agent advice with explicit setup and
  review steps. Simplified the toolkit README without changing its workflow.
- Added a plot of a real run of the documented slide test, its exported values and regeneration tools.

## Verification

- `:SimLoop:docsSite --offline` passes, including strict MkDocs, version check and API doclint.
- The documented slide example passes: one test, no failures, errors or skips.
- Its RLOG was decoded with `RlogTimeSeries`: 150 frames, final position 11.999820053 in, peak
  13.150056536 in. These pass the tutorial's 0.25 in final tolerance and 14 in peak bound.
- The tutorial's executable Java matches the maintained test source; the only omitted block is the
  class-level documentation comment.
- Local file and anchor crawl passed for all 29 rendered guide pages before the final additions;
  the final pass is recorded in the PR. Generated Javadoc is checked by `docsSite`.

The plot describes simulated values from the example's placeholder model, not measured robot motion.
The OpenCV fragment requires the consuming team's processor and native library; it is not a standalone
test, and no end-to-end OpenCV execution is claimed for this patch.

## Pending work already covered elsewhere

#21 adds the long-log streaming documentation. #22 owns the logger-reset fix. #23 owns the counter and
privacy page; #25 owns the logo assets. Keep their documentation with their code when merging. A
published tag containing #24 must be chosen separately; this patch changes no version or release.
