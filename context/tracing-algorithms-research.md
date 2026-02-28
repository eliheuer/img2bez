# Raster-to-Vector Tracing: Algorithm Research

Research conducted 2026-02-22 to inform img2bez architecture improvements.

## 1. Potrace (Peter Selinger) — The Gold Standard

**Paper:** [Potrace: a polygon-based tracing algorithm](https://potrace.sourceforge.net/potrace.pdf) (2003)
**Source mirror:** [github.com/skyrpex/potrace](https://github.com/skyrpex/potrace)

### The Pipeline (5 stages)

**Stage 1 — Path Decomposition (contour extraction)**

Potrace works on the *dual grid* — vertices are at pixel **corners** (integer coordinates), not pixel centers. A vertex exists where four adjacent pixels are not all the same color. An edge connects two vertices at distance 1 if the edge separates a black pixel (on the left) from a white pixel (on the right). This means the initial contour is a rectilinear path that traces *between* pixels along their exact boundaries. The path consists entirely of horizontal and vertical unit-length steps.

Small contours below `turdsize` (default 2 pixels) are discarded as speckle noise.

**Stage 2 — Optimal Polygon Approximation (the key innovation)**

This is what makes Potrace special. It does NOT just detect corners and split. Instead:

1. **`calc_sums`**: Precomputes cumulative sums of x, y, x^2, xy, y^2 for every prefix of the path. This allows O(1) computation of least-squares line-fitting statistics over any sub-range [i,j].

2. **`calc_lon` (longest straight subpath)**: For each vertex i, computes the farthest vertex j such that the subpath i...j can be approximated by a straight line within a tolerance of 0.5 pixels. The straightness test uses a constraint-propagation approach: as you extend from i toward j, you maintain two bounding constraint vectors (via cross products). When all four cardinal directions have appeared or the constraints are violated, the straight run ends. This runs in O(n^2) worst case.

3. **`bestpolygon` (dynamic programming)**: Finds the *globally optimal* polygon approximation. The DP minimizes a *lexicographic* objective: first minimize the number of segments, then among all polygons with that minimum segment count, minimize total penalty. The penalty for segment [i,j] is based on the sum of squared perpendicular distances from original path points to the fitted line, weighted by segment length. This is computable in O(1) per segment using the precomputed sums.

4. **`adjust_vertices`**: The DP gives integer vertex positions (on pixel corners). This step refines each vertex position by solving a small quadratic optimization: find the point within a +/-0.5 pixel box that minimizes deviation from the two adjacent polygon edges. This gives sub-pixel vertex placement.

**Stage 3 — Corner Detection and Curve Fitting**

For each polygon vertex, Potrace computes an alpha parameter based on the geometry of the angle. The formula is essentially: `alpha = (4/3) * (ratio of adjacent segment lengths)`, clamped and compared against `alphamax` (default 1.0). This means:

- **Sharp angles between short segments** are corners
- **Sharp angles between very long segments** are also corners (the length ratio matters)
- Everything else gets smooth Bezier treatment

For smooth vertices, control points are placed along the tangent direction at distance `alpha * d` from the vertex, where d is related to the polygon edge length. The alpha value controls how "round" the join is.

For corners, two straight line segments meet at the vertex.

**Stage 4 — Curve Optimization (optional)**

Adjacent Bezier segments can be merged if:
- They are both curves (not lines, not corners)
- They curve in the same direction (both convex or both concave)
- Total direction change is < 179 degrees
- The merged curve stays within `opttolerance` (default 0.2) of the polygon

This reduces segment count while barely affecting visual quality.

### Why Potrace Produces Tight Outlines

1. **Pixel-edge coordinates**: The initial contour is geometrically exact — it traces the boundary *between* pixels, not through pixel centers
2. **Global polygon optimization**: The DP finds the globally optimal polygon, not a greedy/local approximation
3. **Sub-pixel vertex refinement**: Vertices are placed at the mathematically optimal position within each pixel boundary
4. **The polygon is the intermediate representation**: Curves are fit to the polygon, not to the raw pixel path. The polygon already captures the essential geometry with minimal vertices

---

## 2. Autotrace (Martin Weber) — Schneider-based Approach

**Source:** [github.com/autotrace/autotrace](https://github.com/autotrace/autotrace)
**Key files:** `src/pxl-outline.c`, `src/fit.c`, `src/curve.c`

### The Pipeline

**Stage 1 — Pixel Outline Extraction** (`pxl-outline.c`)

Extracts pixel boundary contours. Like imageproc's `find_contours`, this follows the border pixels.

**Stage 2 — Corner Detection** (`fit.c: find_corners`)

- For each point, computes difference vectors from `corner_surround` (typically 4) neighboring points
- Calculates the angle between incoming and outgoing vectors
- Marks as corner if angle < `corner_threshold` (100 degrees) OR < `corner_always_threshold` (60 degrees)
- Removes adjacent corners to avoid degenerate 2-point curves

**Stage 3 — Knee Removal and Filtering**

- Removes "knee" points (right-angle staircase artifacts)
- Applies smoothing filters to reduce digitization noise

**Stage 4 — Cubic Bezier Fitting** (`fit.c: fit_with_least_squares`)

Uses the **Schneider algorithm** (Philip J. Schneider, Graphics Gems, 1990):

1. **Chord-length parameterization**: Assign parameter t_i to each point proportional to cumulative arc length
2. **Endpoint tangent estimation**: Compute tangent direction at each endpoint from neighboring points
3. **Least-squares Bezier fit**: Solve a 2x2 linear system (via Bernstein polynomial basis) for the two interior control point scale factors alpha_1 and alpha_2. Control points are placed along the endpoint tangents: `P1 = alpha_1 * tangent_1 + P0`
4. **Error measurement**: Compute max Euclidean distance from each point to the fitted curve
5. **Newton-Raphson reparameterization**: If error is moderate, adjust parameter values to better match the curve (up to 4 iterations)
6. **Recursive subdivision**: If error is still too high, split at the worst-fit point and recursively fit both halves

**Stage 5 — Line/Curve Classification** (`fit.c: spline_linear_enough`)

After fitting, checks if each spline is essentially straight (all points within `line_threshold` of the chord). If so, replaces with a line segment.

### Key Differences from Potrace

- **Local corner detection** instead of global polygon optimization
- **Schneider curve fitting** (iterative least-squares) instead of polygon-then-fit
- No intermediate polygon representation
- More parameters to tune (corner_surround, corner_threshold, tangent_surround, etc.)
- Generally produces more control points and less clean outlines than Potrace
- Better suited for organic/complex shapes where the polygon model is too rigid

---

## 3. VTracer (Vision Cortex) — Rust-based, Linear Time

**Docs:** [visioncortex.org/vtracer-docs](https://www.visioncortex.org/vtracer-docs/)
**Source:** [github.com/visioncortex/vtracer](https://github.com/visioncortex/vtracer)

### The Pipeline

**Stage 1 — Hierarchical Color Clustering**

Groups pixels by color similarity into clusters. Each cluster is traced independently.

**Stage 2 — Path Walking**

A directional walker traces the outline of each cluster, consolidating consecutive steps in the same direction.

**Stage 3 — Staircase Removal**

Uses **signed area** calculations to determine which vertices to keep when replacing pixel staircases with cleaner edges. The penalty for replacing a subpath with a single edge is proportional to h * b (height * base length). Uses greedy replacement until tolerance is exceeded.

**Stage 4 — Corner-Preserving Smoothing**

Applies a **4-point subdivision scheme** (interpolating smoothing), but with corner preservation: at each vertex, computes the absolute angle difference. If it exceeds a threshold, that vertex is treated as a corner and excluded from the smoothing calculation. The smoothing runs for a configurable number of iterations (default 10).

**Stage 5 — Splice Point Detection and Curve Fitting**

Before curve fitting, identifies "splice points" where curves should be cut:
- **Point of inflection test**: track signs of angle differences along the path; sign changes indicate inflection points
- **Angle displacement test**: large absolute angle differences indicate corners

Subpaths between splice points are fed to a Bezier curve fitter.

### Key Differences

- **O(n) linear** vs Potrace's O(n^2) polygon optimization
- **Favors fidelity over simplification** — produces curves that more closely follow the original pixels
- Handles multi-color images natively via clustering
- No global optimization — uses greedy local decisions
- Produces different interpretations of ambiguous corners

---

## 4. Other Notable Approaches

### FontForge

FontForge does NOT have its own tracing algorithm. It wraps either Potrace or Autotrace as external tools. The user's choice determines which algorithm is used.

### Schneider Algorithm (Graphics Gems, 1990)

The foundational curve fitting algorithm used by Autotrace and many other tools. Implementation at [Graphics Gems repository](https://www.realtimerendering.com/resources/GraphicsGems/gems/FitCurves.c).

### curve-fit-nd (Campbell Barton / Blender)

An enhanced Schneider implementation ([github.com/ideasman42/curve-fit-nd](https://github.com/ideasman42/curve-fit-nd)) with improvements:
- Circle fitting as fallback for circular arcs
- Offset fitting for better handle length calculation
- "Re-fitting" mode: start with every point as a knot, iteratively remove the one with least error (top-down vs. Schneider's bottom-up subdivision)
- Corner detection via the re-fitting process itself

### Raph Levien's Bezier Fitting (kurbo)

The approach used in kurbo's `fit_to_bezpath_opt`:
- Uses L2 error metric (not max distance)
- Solves a quartic polynomial directly instead of iterating
- Assumes tangent angles are given (G1 constraints)
- Uses area and moment matching for control point placement
- Non-iterative: finds all candidate solutions simultaneously

Path simplification:
- Error scales as O(n^6) — very tight convergence
- Uses monoid-based prefix sums for O(1) range queries on area/moment
- Adaptive subdivision finds the longest segment that fits within tolerance

---

## 5. Architectural Analysis of img2bez vs Potrace

### Gap 1: Pixel Centers vs Pixel Edges (THE BIGGEST ISSUE)

**img2bez:** Uses `imageproc::find_contours` which returns **pixel center coordinates** (integer x,y positions of boundary pixels). A straight vertical edge of 10 pixels produces 10 points at pixel centers.

**Potrace:** Works on the **dual grid** where vertices are at pixel **corners** (the intersections between pixels). A straight vertical edge of 10 pixels produces 2 vertices — one at each end.

**Impact:** img2bez starts with an inherently noisy representation. Every straight edge is a staircase of per-pixel points. The subsequent smoothing, RDP, and curve fitting are all working to recover information that was lost by choosing the wrong coordinate system.

### Gap 2: No Intermediate Polygon / No Global Optimization

**img2bez:** pixel-center contour → corner detection → split → smooth → RDP → fit beziers (all local/greedy)

**Potrace:** pixel-edge contour → calc_lon → bestpolygon (DP) → adjust_vertices → classify corners → generate beziers (globally optimal polygon)

### Gap 3: Smoothing Destroys Information

img2bez applies neighbor-averaging smoothing before curve fitting. This uniformly blurs the contour — rounding corners and pulling curves. Potrace has no smoothing; the polygon optimization handles noise implicitly.

### Gap 4: RDP is Wrong for Curve Preparation

RDP minimizes max perpendicular distance — it's a polyline simplifier, not a curve-fitting preprocessor. It can remove inflection points and curvature maxima that the Bezier fitter needs.

### Gap 5: Two-Pass Fitting is a Bandaid

img2bez fits curves twice (polyline → curves → simplified curves) to compensate for noisy input. With clean input from a polygon, a single pass suffices.

---

## 6. Implementation Plan: Potrace-Style Pipeline for img2bez

### New modules to create

1. **`src/potrace/decompose.rs`** — Pixel-edge contour extraction on the dual grid
2. **`src/potrace/polygon.rs`** — calc_sums, calc_lon, bestpolygon, adjust_vertices
3. **`src/potrace/curve.rs`** — Corner classification (alpha) and Bezier curve generation

### Key data structures

```
PixelPath {
    points: Vec<(i32, i32)>,   // pixel-corner coordinates
    is_outer: bool,
}

Polygon {
    vertices: Vec<(f64, f64)>,  // sub-pixel refined positions
    is_corner: Vec<bool>,       // corner classification per vertex
}
```

### The new pipeline

```
Binary image
  → pixel-edge path decomposition (dual grid)
  → calc_sums (prefix sums for O(1) line-fit stats)
  → calc_lon (longest straight subpath per vertex)
  → bestpolygon (DP: min segments, min penalty)
  → adjust_vertices (sub-pixel refinement)
  → scale to font units (Y-flip, target height)
  → classify corners (alpha parameter)
  → generate bezier curves
  → existing cleanup pipeline (direction, extrema, grid snap, etc.)
```

### Reference: Potrace source structure

Key functions in `potrace/src/trace.c`:
- `calc_sums()` — prefix sums
- `calc_lon()` — longest straight subpath
- `bestpolygon()` — DP optimal polygon
- `adjust_vertices()` — sub-pixel refinement
- `smooth()` — Bezier curve generation with alpha-based corner detection
- `opticurve()` — optional curve merging optimization

The C source is ~1500 lines for the core algorithm. A Rust reimplementation would be similar in size.
