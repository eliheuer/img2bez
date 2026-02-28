# Potrace Algorithm — Precise Implementation Details

Derived from the C source code (`trace.c`). Reference for Rust reimplementation.

## Data Structures

```
// Prefix sum accumulator
struct Sums {
    x: f64,    // sum of x coordinates
    y: f64,    // sum of y coordinates
    x2: f64,   // sum of x^2
    xy: f64,   // sum of x*y
    y2: f64,   // sum of y^2
}

// Working path data
struct PathData {
    pt: Vec<(i32, i32)>,       // path points (pixel-corner coords)
    lon: Vec<usize>,           // longest straight subpath endpoint per vertex
    x0: i32, y0: i32,         // origin for sums (= pt[0])
    sums: Vec<Sums>,          // prefix sums [len+1]
    po: Vec<usize>,           // polygon vertex indices [m]
    // ... curve output
}
```

## Stage 0: `calc_sums` — Prefix Sum Computation

Precompute prefix sums so that sums over any subrange [i..j] are O(1).

```
x0 = pt[0].x
y0 = pt[0].y
sums[0] = Sums { x:0, y:0, x2:0, xy:0, y2:0 }

for i in 0..n:
    x = pt[i].x - x0
    y = pt[i].y - y0
    sums[i+1].x  = sums[i].x  + x
    sums[i+1].y  = sums[i].y  + y
    sums[i+1].x2 = sums[i].x2 + x*x
    sums[i+1].xy = sums[i].xy + x*y
    sums[i+1].y2 = sums[i].y2 + y*y
```

**Cyclic range query** from i to j (where j may wrap around):
```
r = 1 if j < i else 0
x  = sums[j+1].x  - sums[i].x  + r * sums[n].x
y  = sums[j+1].y  - sums[i].y  + r * sums[n].y
x2 = sums[j+1].x2 - sums[i].x2 + r * sums[n].x2
... same for xy, y2
k  = j+1 - i + r*n    // point count
```

## Stage 1: `calc_lon` — Longest Straight Subpath

For each vertex i, find lon[i] = farthest index reachable by a straight line staying within 0.5 units of all intermediate points.

### Step 1: Build `nc` (next corner) array
```
k = 0
for i from n-1 down to 0:
    if pt[i].x != pt[k].x AND pt[i].y != pt[k].y:
        k = i + 1
    nc[i] = k
```

### Step 2: Constraint propagation

For each start point i, walk forward maintaining two constraint vectors that define a cone of valid line directions.

- Track 4 cardinal directions used (indices via `dir = (3 + 3*sign(dx) + sign(dy)) / 2`)
- If all 4 directions seen → path must turn, set pivk[i]
- Compute cur = pt[k] - pt[i]
- Check constraint violations via cross products
- Update constraints using offset vectors (accounting for ±0.5 unit tolerance):
  ```
  off.x = cur.x + (if cur.y>=0 && (cur.y>0 || cur.x<0) { 1 } else { -1 })
  off.y = cur.y + (if cur.x<=0 && (cur.x<0 || cur.y<0) { 1 } else { -1 })
  if xprod(constraint[0], off) >= 0: constraint[0] = off
  // symmetric for constraint[1]
  ```

### Step 3: Clean up lon from pivk
Propagate backward to ensure monotonicity, fix wraparound.

## Stage 2: `bestpolygon` — Optimal Polygon via DP

### Penalty function `penalty3(i, j)`

Measures how well a straight line from point i to j fits intermediate points:
```
// Line direction perpendicular
px = (pt[i].x + pt[j].x) / 2.0 - x0
py = (pt[i].y + pt[j].y) / 2.0 - y0
ey =  (pt[j].x - pt[i].x)
ex = -(pt[j].y - pt[i].y)

// Variance projected onto normal
a = (x2 - 2*x*px)/k + px*px
b = (xy - x*py - y*px)/k + px*py
c = (y2 - 2*y*py)/k + py*py

return sqrt(ex*ex*a + 2*ex*ey*b + ey*ey*c)
```

### DP recurrence

1. Compute clip0[i] (farthest reachable from i) and clip1[j] (earliest that can reach j)
2. Compute seg0[j], seg1[j] (valid ranges for j-th vertex)
3. DP: minimize penalty with minimum segment count
4. Trace backward through prev[] pointers

## Stage 3: `adjust_vertices` — Sub-pixel Vertex Refinement

For each polygon vertex, move to optimal position within ±0.5 pixel:

1. Compute optimal line through each segment via eigenvalue analysis of covariance matrix:
   ```
   ctr = centroid of points in segment
   lambda2 = (a + c + sqrt((a-c)^2 + 4*b^2)) / 2   // larger eigenvalue
   dir = eigenvector for lambda2
   ```

2. Form 3x3 quadratic form Q for each segment (distance² from line)
3. Sum Q matrices of two adjacent segments
4. Solve 2x2 linear system for minimum
5. Constrain to ±0.5 pixel box (check edges and corners if outside)

## Stage 4: `smooth` — Corner Detection and Curve Generation

### Alpha parameter (corner/smooth classification)

For three consecutive vertices i, j, k:
```
p4 = midpoint(vertex[k], vertex[j])    // the "knot" point

// dpara = cross product of (j-i, k-i) = twice signed area of triangle
// ddenom uses dorth_infty (90° rotation snapped to cardinal)
dd = |dpara(vertex[i], vertex[j], vertex[k]) / ddenom(vertex[i], vertex[k])|
alpha = if dd > 1 { (1 - 1/dd) / 0.75 } else { 0 }
```

### Decision and control points

- **alpha >= alphamax** (default 1.0) → **Corner**: tag=CORNER, c[j] = [_, vertex[j], p4]
- **alpha < alphamax** → **Curve**: clamp alpha to [0.55, 1.0], then:
  ```
  c[j][0] = interval(0.5 + 0.5*alpha, vertex[i], vertex[j])  // CP1
  c[j][1] = interval(0.5 + 0.5*alpha, vertex[k], vertex[j])  // CP2
  c[j][2] = p4                                                 // endpoint
  ```

Where `interval(t, a, b) = a + t*(b-a)`. Alpha=1 puts CP at vertex (max curvature). Alpha=0.55 puts CP at 0.775 of the way.

## Stage 5: `opticurve` — Curve Merging Optimization

DP that tries to merge consecutive curve segments into single Beziers.

### Merge feasibility (`opti_penalty`)
1. All intermediate segments must have same convexity, no corners
2. Total bend < 179°
3. Compute intersection of endpoint tangent lines
4. Place new control points using computed alpha
5. Check each intermediate vertex is within `opttolerance` of new curve

### DP
Minimize segment count first, then penalty as tiebreaker. Trace backward for result.

## Key Implementation Notes

1. **Cyclic mod:** `mod(a, n)` must handle negatives correctly: `n-1-(-1-a)%n` for a<0
2. **`xprod`** is integer cross product: `p1.x*p2.y - p1.y*p2.x`
3. **INFTY** sentinel = 10000000 in calc_lon
4. **Alpha clamping:** raw alpha clamped to [0.55, 1.0] for curves
5. **`dorth_infty(p0, p2)`** = 90° rotation of direction snapped to cardinal: `(-sign(dy), sign(dx))`
6. **`dpara(p0, p1, p2)`** = cross product of (p1-p0, p2-p0)
7. **`ddenom(p0, p2)`** = `dorth.y*(p2.x-p0.x) - dorth.x*(p2.y-p0.y)` where dorth=dorth_infty(p0,p2)
8. **`interval(t, a, b)`** = `a + t*(b-a)` componentwise
9. **`tangent()`** solves quadratic for parameter where Bezier is tangent to a direction
10. **Area formula** in opticurve: `0.3 * alpha * (4-alpha)` approximates Bezier area relative to chord
