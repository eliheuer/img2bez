# Autotracing for type design: research notes and pipeline design

*June 2026. Background research and design rationale for the img2bez
sub-pixel ("fit") tracing pipeline.*

## Goal

Produce production-ready type design outlines from glyph images:
outlines that need minimal manual editing, stay as close as possible to
the source shapes, and follow professional drawing conventions. The
quality bar is what you would see in a well-drawn font source (e.g.
a hand-drawn reference grotesque), evaluated with `./eval-harness/render-specimen.sh` against
reference UFOs.

## Research

### Drawing conventions (from widely-taught type-drawing guides)

Published type-drawing guides give the rules a designer
follows; the tracer should produce structure that already obeys them:

- **Minimum points.** Use as few points as the shape needs; extra
  points make editing painful.
- **Points at extrema.** Anchor points sit at the highest, lowest,
  leftmost, and rightmost positions of every curve, with handles
  leaving exactly horizontally or vertically.
- **Points at inflections.** Where a curve changes directional bias
  (an `s` spine, a bowl flowing into a leg), place a point with
  handles extending in opposite directions.
- **Corners may have angled handles**; everything else should be H/V.
- **Balanced tension.** The line connecting two on-curve points often
  parallels the line connecting their handles.

### Near-optimal cubic fitting (Raph Levien / kurbo)

<https://raphlinus.github.io/curves/2021/03/11/bezier-fitting.html>

Kurbo's `fit_to_bezpath_opt` implements a near-optimal cubic fit:
endpoint tangents are taken from the source, and the two free handle
lengths are solved by matching the signed area (Green's theorem) and
first moment of the source curve, reducing the search to a quartic
whose roots cover all candidate cubics — no local-minimum traps. The
`_opt` variant also chooses globally optimal subdivision points,
giving the minimum number of segments for a target accuracy. img2bez
uses this as the fallback fitter; sources implement the
`ParamCurveFit` trait.

### G2 continuity / harmonization (the equal-curvature editor rule)

<https://gist.github.com/simoncozens/3c5d304ae2c14894393c6284df91be5b>

Font editors "harmonize" a smooth join between two cubics
(`a0 a1 a2 a3` and `b0 b1 b2 b3`, `a3 = b0`) for equal curvature
(G2) on both sides:

1. Intersect the handle lines `a1–a2` and `b1–b2` at `d`.
2. `p0 = |a1a2| / |a2d|`, `p1 = |db1| / |b1b2|`, `p = sqrt(p0 · p1)`.
3. Move the on-curve point to `t = p / (p + 1)` of the way along
   `a2 → b1`.

This is the curvature-comb-equalizing rule used by the Harmonize
tools in mainstream font editors. img2bez applies it (with a displacement
cap) at every smooth join after G1 alignment.

### Why potrace-style pipelines fall short for type

A potrace-shaped pipeline thresholds to binary, traces pixel edges,
approximates with a polygon, classifies corners with the *alpha*
parameter, and fits curves per section. img2bez was originally built
that way; it now traces the grayscale directly (see below). Two
structural problems motivated the change:

1. **Thresholding discards the anti-aliasing.** AA carries ~0.1 px of
   boundary position information; binary pixel data quantizes it to
   ±0.5 px. Every downstream decision (straight vs. curve, corner
   vs. smooth, extremum location) then needs noise-tolerant
   heuristics that are fragile precisely at the places type cares
   about (flat extrema, shallow joins, overshoot regions).
2. **Alpha conflates deviation with sharpness.** Potrace's alpha
   measures how far a polygon vertex deviates from its neighbors'
   chord; on a large smooth arc with long polygon facets this is
   large even though the turn is shallow, producing false corners at
   the flattest part of an `o` — exactly where a designer wants a
   single smooth extremum point.

## The fit pipeline

Implemented in `src/vectorize/subpixel.rs` and
`src/vectorize/fit.rs`; routed from `src/lib.rs`. Inputs are
expected to carry anti-aliasing at the ink boundary (rendered glyphs,
grayscale scans).

### 1. Sub-pixel boundary extraction (`subpixel.rs`)

Marching squares over the *grayscale* image at the threshold
iso-level, with linear interpolation along grid edges. Saddle cells
are disambiguated with the center average. Crossings are chained into
closed loops via edge keys; keys are sorted so extraction is fully
deterministic (a contour's start point must not depend on hash
order — sub-pixel shifts flip marginal classifications). Verified
accuracy on a synthetic AA disk: < 0.35 px.

Routing guards in `lib.rs`:

- `downscale_if_blocked`: nearest-neighbor-upscaled sources quantize
  the AA into uniform 2×2 blocks; the image is downscaled back to its
  true resolution first. The detector requires essentially *all*
  blocks uniform — a normal render's flat background already makes
  most blocks uniform.

### 2. Resampling and adaptive smoothing

The boundary is resampled at 1 px arc-length spacing and smoothed
with a small Gaussian (σ = 1.2 samples). If the smoothed boundary
still looks noisy — mean |turn| per sample above 0.10 rad, or a
corner detected more often than once per 30 samples (impossible for
real type) — σ escalates ×1.8 up to four times. Clean AA renders
never escalate; degraded sources get progressively stronger
smoothing.

### 3. Curvature-based structure detection

All features come from the turn-angle signal of the smoothed
polyline, mirroring where a designer puts points:

- **Corners**: turn concentrated in a ±1-sample window (≥ 12°) that
  also dominates the surrounding ±5-sample window (≥ 55%). This
  separates a sharp 20° serif join (impulse) from a tight smooth arc
  (distributed turn).
- **Straight runs**: maximal runs whose chord deviation stays under
  `max(0.15 px, 0.005 × chord)`. A line's deviation is flat noise; an
  arc's grows quadratically with chord length, so a *linear* envelope
  separates them at every scale. Runs start scanning from a corner so
  a run wrapping the array seam isn't split. Run ends snap to nearby
  splits (nearest within 8 samples — the corner-rounding zone that
  smoothing creates).
- **Tangent points**: straight-run endpoints; the adjacent curve's
  end tangent is constrained to the line direction.
- **Extrema**: derivative sign changes of x/y with plateau collapsing,
  ≥ 1 px prominence (a scan that runs into a structural boundary
  while still receding counts as prominent), localized by a
  least-squares parabola vertex over ±14 samples and clamped clear of
  the standoff zones. Extrema may sit close to corners (5 samples) —
  real designs put bowl extrema within ~10 px of crossbar corners —
  but not close to straight runs (8 samples), where they would be
  edge noise on the line itself.
- **Inflections**: persistent sign changes of smoothed curvature,
  with a standoff so they never crowd other points.
- **Chamfers**: a short (≤ 16-sample) corner-to-corner section that is
  straight within 1.2 px is emitted as a line. Straight-run detection
  cannot see these: the corner-rounding zones at both ends leave no
  clean interior samples.

### 4. Constrained fitting

Each curved section is fitted with a Schneider-style least-squares
cubic whose end tangent *directions* are fixed (exact H/V at extrema,
the line direction at tangent points, axis-snapped free directions at
corners within 15°), solving only the two handle lengths, with Newton
reparameterization. Fixing tangents is what makes the fit robust to a
split point sitting a few pixels off the true extremum — and the
handles come out exactly H/V by construction.

If the single cubic misses tolerance, the section is split at its
dominant curvature-sign change (a designer's inflection point — e.g.
an `R` bowl flowing into the leg crotch) and both halves are fitted
with a shared tangent at the split. Only if that also fails does
kurbo's `fit_to_bezpath_opt` subdivide freely.

### 5. Cleanup passes

- **Corner-sliver reconstruction**: a tiny curve (≤ 12 px) squeezed
  between two lines is a rasterization-rounded corner (e.g. the inner
  crotch of a `v`); the lines are extended to their sharp
  intersection, as a designer would draw it.
- **G1 alignment**: at smooth joins, handles are rotated to a shared
  direction (the axis at extrema, the line direction at line joins,
  the bisector elsewhere).
- **G2 harmonization**: the equal-curvature rule above, two passes,
  with the on-curve displacement capped at 2 px.

The existing post-processing (winding correction, grid snapping, H/V
line snapping) runs unchanged downstream.

## Raster-loss refinement

The fit stages fit curves to the extracted iso-contour — a
boundary-sample objective. `vectorize/refine.rs` adds an optional
refinement (on by default, `--no-refine` to disable) that re-scores
candidate outlines directly against the source grayscale, borrowing
the central idea of differentiable vector graphics rasterization
([diffvg](https://people.csail.mit.edu/tzumao/diffvg/)): near the
outline, a pixel's ink coverage is approximately a linear ramp in its
signed distance to the path, so an area-based loss over the boundary
band is smooth in the curve parameters. Because the initial outline is
already sub-pixel accurate, no autodiff is needed — handle lengths are
solved by golden-section coordinate descent (including the
mean/difference directions, since arc fitting produces a narrow loss
valley along alpha + beta ≈ const).

Three passes run:

- **Junction flats**: this reference (like many designs) draws a tiny
  axis-aligned flat — typically 8 font units — at every stroke
  junction: the crotch of `v`, the saddles between `m`'s arches, all
  four crossings of `x`. Anti-aliasing plus smoothing round the wedge
  tip off, so the fit places a single shallow vertex (or a couple of
  junk micro-segments) there instead. For each sharp valley whose
  depth axis is near-vertical or near-horizontal, three structures
  compete on the raster band loss: the fit as it stands, a sharp
  vertex pushed to its loss-minimizing depth, and an axis-aligned flat
  whose depth (and, for bare vertices, width) is optimized. The flat
  is applied only when it beats both rivals by a clear margin (factor
  0.8) — a genuinely sharp corner ties with the sharp candidate and
  can never pass. The candidate search range is bounded by walking the
  wedge's bisector in the coverage field until the ink flips, so the
  judged band never punches out the far side of a thin stroke.
- **Merge**: adjacent same-turning cubics whose shared joint was
  created by fit subdivision are replaced by one cubic when the
  optimized single cubic reproduces the raster well in absolute terms
  (band loss ≤ 0.04 — measured good merges score 0.01–0.04, bad ones
  0.06+), or loses almost nothing against the RE-optimized pair. The
  fair pair baseline matters: comparing against the pair as fitted
  makes every merge look good simply because one side was optimized.
- **Polish**: a cubic's handle lengths are re-optimized only when its
  current fit is clearly poor (loss ≥ 0.02), optimization improves it
  by ≥ 30%, and the result genuinely fits (loss ≤ 0.02). The dead-band
  is what prevents regressions: marginal raster gains are not worth
  perturbing handles the contour fit already placed well, and
  mis-segmented sections (where no handle length fixes the fit) are
  left for structural work instead of shuffled.

Merge and polish preserve type-design structure by construction:
handle directions never change (H/V handles stay exact), on-curve
points never move, and corner/extremum/tangent joints are never
touched. The junction-flat pass is the deliberate exception — it
exists to fix structure — and only ever rebuilds the two or three
segments at a junction it has beaten on the raster score.

One implementation detail proved decisive for the junction judge: the
signed distance to a polyline must take its sign from the
angle-bisecting pseudo-normal when the nearest feature is a vertex.
The naive side-of-line test misclassifies pixels in a sharp vertex's
outer cone (the two adjacent windows disagree, and which one wins the
nearest-window race is arbitrary), which inflated the loss of every
V-shaped candidate and made flats look better than they were.

The merge/polish passes lifted S 0.884→0.927, C 0.908→0.945,
s 0.943→0.977, U 0.937→0.964, and brought `2` from the one
stress-gate failure (23 points vs 19) to a pass at 0.912. Junction
flats then lifted 29 more glyphs with zero regressions — v, z, M, V
to exact reference structure (1.000), m 0.891→0.923, n 0.932→0.956,
b/d 0.939→0.975, 8 0.905→0.941 — for a basic-Latin mean of 0.956 →
0.967.

## Evaluation

`./eval-harness/render-specimen.sh --no-build --no-open --text X -w DIR` renders a
specimen and writes `DIR/structural_summary.txt` with per-glyph
score, segment counts, H/V-handle ratio, and matched reference
points. `IMG2BEZ_DEBUG_FIT=1` prints each contour's split list,
line sections, chosen smoothing sigma, and constrained-fit errors.

Current results live in the README's "Tracing quality" table.

## Known limitations / future work

- `7` (0.857) and `y` (0.854) remain the weakest glyphs: both draw a
  short straight segment where the trace runs the adjacent curve
  through (`7`'s diagonal ends, `y`'s inner tail) — a line/curve
  boundary placement problem, not a junction problem.
- The harmonization pass enforces G2 only pairwise at joins; a global
  curvature-energy polish (or Euler-spiral/hyperbezier intermediate
  representation) could further smooth long multi-segment curves.
- Grid snapping happens after harmonization and can perturb G2 by a
  unit or two; re-harmonizing after snapping (with on-curve points
  pinned) would recover it.
- The `lowres4x` profile (4× nearest upscale) still over-segments
  after two rounds of block downscaling.
