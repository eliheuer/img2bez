# Known problems / future work

The current map of where traced output still falls short of a designer's
outline, in rough priority order. Each item is measured, not speculative.

## Tracer geometry

* **Corner vertices sit slightly inside the ink.** Across a 2,000-glyph
  corpus comparison, traced corner points average +0.4 units into the ink
  (median +0.24; a third are off by more than 1 unit), while smooth points
  are unbiased. Anti-aliasing rounds every corner and the traced vertex
  lands inside the true one; every downstream pass inherits the error
  (it can tilt an adjacent line enough to defeat the refine passes).
  Fix: derive each corner as the intersection of its two adjacent edge
  fits instead of trusting the traced vertex.
* **Gentle optical bowing gets flattened.** A barely-curved edge (the
  nearly-flat outside of a B or D bowl) reads as a straight line at
  working resolutions when its chord stays near an axis, so the designed
  curve becomes a line. The axial-tangent veto in `cleanup::straighten`
  protects brackets whose neighbor confirms the tangent, but near-axial
  chords don't qualify.
* **Long shallow diagonals over-segment.** The strokes of w/v/x/y can
  keep a spurious mid-diagonal smooth point where references draw one
  line (or one designed bowed midpoint). Related: the site head's
  `point` class (designed on-curve points on diagonals and spines) is
  under-trained — only a few hundred labels corpus-wide — so those zones
  currently fall back to the procedural gate.
* **Tiny cubics cannot hold G1 in integer coordinates.** A ~17-unit
  cubic with ~6-unit handles rounds to a few degrees of tangent
  mismatch at `--grid 2`; exact at `--grid 0`. Cosmetic, known class.

## Learned components (opt-in)

* **Head features are not resolution-invariant.** The site head's
  span/sigma features are raw pixels, so a model trained at one render
  size misjudges the same feature at another (chamfer pairs collapse
  below ~800 px). A crop-height normalization made it worse (crop noise
  beats resolution bias — a documented negative result); the real fix
  is a pixels-per-em estimate at trace time.
* **Point budget is a whole-contour decision.** A learned keep/merge
  head for redundant points is not viable on local features: the tracer
  keeps ~89% of candidate points at sites where reference designers
  keep ~27%, but a local classifier trained on that signal degenerates
  to merge-everything. Needs sequence-level modeling with segment-level
  labels.

## Harness

* The structural gate's baseline (`eval-harness/baselines/current.tsv`)
  must be re-baselined whenever the default pipeline or the harness
  renderer changes; a stale baseline reads as phantom regressions.
* The gate renders at reduced resolution, so learned-head A/B results
  there can differ from full-resolution behavior; check both (the
  `structure-compare` tool traces at full size).
