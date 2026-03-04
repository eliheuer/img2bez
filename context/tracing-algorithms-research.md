# Tracing Algorithm Reference

Background research on raster-to-vector tracing algorithms. img2bez uses a Potrace-style pipeline (dual-grid decomposition, optimal polygon via DP, alpha-based corner detection) with kurbo's `fit_to_bezpath_opt` for curve fitting.

## Potrace (Peter Selinger)

**Paper:** [Potrace: a polygon-based tracing algorithm](https://potrace.sourceforge.net/potrace.pdf) (2003)

The gold standard for bitmap tracing. img2bez reimplements its core pipeline in Rust.

### Pipeline

**Stage 1 — Path Decomposition (dual grid)**

Works on the dual grid where vertices are at pixel corners, not centers. A straight edge of N pixels produces 2 vertices, not N. Contours trace between pixels along exact boundaries. See `src/vectorize/decompose.rs`.

**Stage 2 — Optimal Polygon (DP)**

The key innovation. Three substages:

1. `calc_sums` — Prefix sums (x, y, x^2, xy, y^2) for O(1) line-fit statistics over any subrange.
2. `calc_lon` — For each vertex, find the farthest reachable vertex via a straight line within 0.5px. Uses constraint propagation with angular corridor tracking.
3. `bestpolygon` — DP minimizing segment count first, then total penalty (RMS perpendicular distance). Produces the globally optimal polygon approximation.
4. `adjust_vertices` — Sub-pixel refinement via quadratic optimization within +/-0.5px of each vertex.

See `src/vectorize/polygon.rs` and `context/potrace-algorithm-details.md` for pseudocode.

**Stage 3 — Corner Detection and Curves**

Computes an alpha parameter per vertex based on triangle geometry. Alpha >= alphamax (default 1.0) marks a corner. Smooth vertices get cubic bezier control points placed along tangent directions.

**Stage 4 — Curve Merging (opticurve)**

img2bez replaces this stage with kurbo's `fit_to_bezpath_opt`, which produces near-optimal cubic fits with better curve quality.

### Why Potrace produces tight outlines

1. Pixel-edge coordinates — geometrically exact boundaries
2. Global polygon optimization — DP finds the optimal approximation, not a greedy one
3. Sub-pixel vertex refinement — mathematically optimal vertex placement
4. The polygon is the intermediate representation — curves are fit to the polygon, not raw pixels

## Autotrace (Martin Weber)

**Source:** [github.com/autotrace/autotrace](https://github.com/autotrace/autotrace)

Uses the Schneider algorithm (Graphics Gems, 1990) for curve fitting:

1. **Pixel outline extraction** — follows border pixels
2. **Corner detection** — angle-based, using `corner_surround` neighbors
3. **Schneider curve fitting** — chord-length parameterization, least-squares bezier fit, Newton-Raphson reparameterization, recursive subdivision at worst-fit points
4. **Line/curve classification** — replaces near-straight curves with line segments

Compared to Potrace: local corner detection instead of global polygon, Schneider fitting instead of polygon-then-fit, more parameters, generally produces more control points.

## VTracer (Vision Cortex)

**Source:** [github.com/visioncortex/vtracer](https://github.com/visioncortex/vtracer)

Rust-based, O(n) linear time:

1. **Color clustering** — groups pixels by color similarity
2. **Path walking** — traces cluster outlines
3. **Staircase removal** — signed-area penalty for subpath replacement
4. **Corner-preserving smoothing** — 4-point subdivision with angle-based corner exclusion
5. **Splice detection and curve fitting** — inflection and angle tests to split curves

Favors fidelity over simplification. No global optimization. Good for multi-color images.

## Schneider Algorithm (Graphics Gems, 1990)

The foundational curve fitting algorithm used by Autotrace and many other tools:

- Chord-length parameterization assigns t values proportional to arc length
- Endpoint tangent estimation from neighboring points
- Least-squares bezier fit solves a 2x2 system for control point scale factors
- Newton-Raphson reparameterization improves parameter estimates (up to 4 iterations)
- Recursive subdivision at worst-fit point if error exceeds tolerance

Implementation: [Graphics Gems FitCurves.c](https://www.realtimerendering.com/resources/GraphicsGems/gems/FitCurves.c)

Enhanced version by Campbell Barton (Blender): [curve-fit-nd](https://github.com/ideasman42/curve-fit-nd) — adds circle fitting, offset fitting, and top-down re-fitting mode.

## Raph Levien's Bezier Fitting (kurbo)

Used in img2bez via `kurbo::fit_to_bezpath_opt`:

- L2 error metric (not max distance)
- Solves a quartic polynomial directly instead of iterating
- Assumes tangent angles are given (G1 constraints)
- Area and moment matching for control point placement
- Error scales as O(n^-6) — very tight convergence
- Monoid-based prefix sums for O(1) range queries

**Blog post:** [Simplifying Bezier Paths](https://raphlinus.github.io/curves/2023/04/18/bezpath-simplify.html)

## References

### Libraries

| Library | Language | Use |
|---------|----------|-----|
| [kurbo](https://crates.io/crates/kurbo) | Rust | Bezier math, curve fitting |
| [norad](https://crates.io/crates/norad) | Rust | UFO read/write |
| [imageproc](https://crates.io/crates/imageproc) | Rust | Image processing, thresholding |
| [vtracer](https://github.com/visioncortex/vtracer) | Rust | Alternative bitmap tracer |
| [svg2glif](https://crates.io/crates/svg2glif) | Rust | SVG to GLIF conversion |

### Papers

- [Potrace algorithm](https://potrace.sourceforge.net/potrace.pdf) — Polygon-based tracing (2003)
- [Bezier Splatting](https://arxiv.org/abs/2503.16424) — 30-150x faster differentiable rendering (NeurIPS 2025)
- [StarVector](https://arxiv.org/abs/2312.11556) — Image-to-SVG foundation model
- [DeepVecFont-v2](https://arxiv.org/abs/2303.14585) — Neural vector font generation (CVPR 2023)
- [Differentiable Variable Fonts](https://arxiv.org/abs/2510.07638) — Gradient-based variable font optimization
