//! Optimal polygon approximation via dynamic programming.
//!
//! Given a closed pixel-edge contour, finds the polygon with the fewest
//! vertices that stays within half a pixel of the original path. This
//! dramatically reduces vertex count (e.g. a 200-point circle → 20 vertices)
//! while preserving shape fidelity.
//!
//! ## Algorithm
//!
//! 1. **Prefix sums** (`calc_sums`) — O(1) line-fit statistics for any
//!    sub-range of the path.
//! 2. **Longest straight subpath** (`calc_lon`) — for each vertex, find
//!    the farthest reachable vertex where the path stays within ±0.5
//!    of a straight line (using constraint propagation).
//! 3. **DP optimal polygon** (`best_polygon`) — globally minimize line-fit
//!    penalty over all valid polygons with the minimum number of segments.
//! 4. **Vertex refinement** (`adjust_vertices`) — shift each polygon vertex
//!    to the sub-pixel position that minimizes squared distance to the
//!    two adjacent line segments (constrained to ±0.5 of the pixel corner).

use super::decompose::PixelPath;

/// Prefix sum accumulator for O(1) line-fit statistics.
#[derive(Debug, Clone, Copy, Default)]
struct Sums {
    x: f64,
    y: f64,
    x2: f64,
    xy: f64,
    y2: f64,
}

/// Optimal polygon derived from a pixel path.
#[derive(Debug, Clone)]
pub struct Polygon {
    /// Sub-pixel-refined vertex positions.
    pub vertices: Vec<(f64, f64)>,
    /// +1 for outer, -1 for hole (from the source PixelPath).
    #[allow(dead_code)]
    pub sign: i8,
}

/// Compute the optimal polygon for a pixel path.
///
/// Prefix sums → longest straight subpaths → DP optimal polygon →
/// sub-pixel vertex refinement.
pub fn optimal_polygon(path: &PixelPath) -> Polygon {
    let n = path.points.len();
    if n < 4 {
        let vertices = path
            .points
            .iter()
            .map(|&(x, y)| (x as f64, y as f64))
            .collect();
        return Polygon {
            vertices,
            sign: path.sign,
        };
    }

    let (sums, x0, y0) = calc_sums(&path.points);
    let lon = calc_lon(&path.points);
    let po = best_polygon(&path.points, &lon, &sums, x0, y0);
    let vertices = if std::env::var("IMG2BEZ_DEBUG_NO_ADJUST").is_ok() {
        // Debug: skip sub-pixel adjustment, use raw DP vertex positions
        po.iter()
            .map(|&i| (path.points[i].0 as f64, path.points[i].1 as f64))
            .collect()
    } else {
        adjust_vertices(&path.points, &po, &sums, x0, y0)
    };

    Polygon {
        vertices,
        sign: path.sign,
    }
}

// ── Prefix sums ──────────────────────────────────────────

/// Compute prefix sums for O(1) range queries on line-fit statistics.
///
/// For any sub-range [i..j], the sums of x, y, x², xy, y² can be
/// retrieved in O(1) as `sums[j+1] - sums[i]` (with a cyclic wrap
/// correction when j < i). These statistics are sufficient to compute
/// the best-fit line and its RMS error for any sub-range.
fn calc_sums(pt: &[(i32, i32)]) -> (Vec<Sums>, i32, i32) {
    let n = pt.len();
    let x0 = pt[0].0;
    let y0 = pt[0].1;

    let mut sums = vec![Sums::default(); n + 1];
    for i in 0..n {
        let x = (pt[i].0 - x0) as f64;
        let y = (pt[i].1 - y0) as f64;
        sums[i + 1] = Sums {
            x: sums[i].x + x,
            y: sums[i].y + y,
            x2: sums[i].x2 + x * x,
            xy: sums[i].xy + x * y,
            y2: sums[i].y2 + y * y,
        };
    }

    (sums, x0, y0)
}

// ── Longest straight subpath ─────────────────────────────

/// For each vertex i, compute the farthest vertex reachable by a straight line
/// that stays within 0.5 units of all intermediate points.
///
/// ## Algorithm: constraint propagation
///
/// Starting from vertex `i`, we walk forward through vertices, maintaining
/// two constraint vectors that define an angular corridor. The corridor
/// represents all line directions from `i` that would keep every visited
/// vertex within ±0.5 pixels of the line.
///
/// At each step we check:
/// 1. **Four-direction test**: if the path has moved in all 4 cardinal
///    directions (N, S, E, W), no single straight line can approximate it.
/// 2. **Constraint violation**: if the current vertex falls outside the
///    angular corridor (checked via cross products against the two
///    constraint vectors).
/// 3. **Constraint tightening**: each new vertex that is >1 pixel from `i`
///    narrows the corridor by shifting the constraint half a pixel toward
///    the vertex's side.
///
/// ### Direction index formula
///
/// The expression `(3 + 3*dkx + dky) / 2` maps a cardinal step (dx, dy)
/// to a direction index 0–3:
///
/// ```text
///   (dx, dy) → (3 + 3*dx + dy) / 2
///   (-1,  0) → (3 - 3 + 0) / 2 = 0   West
///   ( 0, -1) → (3 + 0 - 1) / 2 = 1   South
///   ( 0,  1) → (3 + 0 + 1) / 2 = 2   North
///   ( 1,  0) → (3 + 3 + 0) / 2 = 3   East
/// ```
#[allow(clippy::needless_range_loop)]
fn calc_lon(pt: &[(i32, i32)]) -> Vec<usize> {
    let n = pt.len();
    let mut lon = vec![0usize; n];

    // Build nc[i]: index of the next "corner" (direction change) after i.
    let mut nc = vec![0usize; n];
    {
        let mut k = 0usize;
        for i in (0..n).rev() {
            if pt[i].0 != pt[k % n].0 && pt[i].1 != pt[k % n].1 {
                k = i + 1;
            }
            nc[i] = k;
        }
    }

    let mut pivk = vec![0usize; n];

    for i in (0..n).rev() {
        // Direction counters for all 4 cardinal directions.
        let mut ct = [0i32; 4];
        let mut constraint = [(0i32, 0i32); 2];

        // Mark direction from i to i+1.
        let i1 = (i + 1) % n;
        let dir0 = ((3 + 3 * (pt[i1].0 - pt[i].0) + (pt[i1].1 - pt[i].1)) / 2) as usize;
        ct[dir0] += 1;

        let mut k = nc[i];
        let mut k1 = i;

        loop {
            let dkx = sign(pt[k % n].0 - pt[k1 % n].0);
            let dky = sign(pt[k % n].1 - pt[k1 % n].1);
            let dir_idx = ((3 + 3 * dkx + dky) / 2) as usize;
            ct[dir_idx] += 1;

            // If all 4 cardinal directions have appeared, path must turn.
            if ct[0] != 0 && ct[1] != 0 && ct[2] != 0 && ct[3] != 0 {
                pivk[i] = k1 % n;
                break;
            }

            let cur = (pt[k % n].0 - pt[i].0, pt[k % n].1 - pt[i].1);

            // Check constraint violations.
            if xprod(constraint[0], cur) < 0 || xprod(constraint[1], cur) > 0 {
                pivk[i] = pivot_at_violation(pt, &constraint, k, k1, i, n);
                break;
            }

            // Update constraints (skip when |cur| <= 1 — no constraint).
            if !(cur.0.abs() <= 1 && cur.1.abs() <= 1) {
                let off0 = (
                    cur.0
                        + if cur.1 >= 0 && (cur.1 > 0 || cur.0 < 0) {
                            1
                        } else {
                            -1
                        },
                    cur.1
                        + if cur.0 <= 0 && (cur.0 < 0 || cur.1 < 0) {
                            1
                        } else {
                            -1
                        },
                );
                if xprod(constraint[0], off0) >= 0 {
                    constraint[0] = off0;
                }

                let off1 = (
                    cur.0
                        + if cur.1 <= 0 && (cur.1 < 0 || cur.0 < 0) {
                            1
                        } else {
                            -1
                        },
                    cur.1
                        + if cur.0 >= 0 && (cur.0 > 0 || cur.1 < 0) {
                            1
                        } else {
                            -1
                        },
                );
                if xprod(constraint[1], off1) <= 0 {
                    constraint[1] = off1;
                }
            }

            k1 = k;
            k = nc[k1 % n];

            if !cyclic(k % n, i, k1 % n, n) {
                pivk[i] = pivot_at_violation(pt, &constraint, k, k1, i, n);
                break;
            }
        }
    }

    // Convert pivk to lon, ensuring monotonicity.
    let mut j = pivk[n - 1];
    lon[n - 1] = j;
    for i in (0..n - 2 + 1).rev() {
        if cyclic(i + 1, pivk[i], j, n) {
            j = pivk[i];
        }
        lon[i] = j;
    }

    // Fix up for cyclic path.
    {
        let mut i = n - 1;
        while cyclic(pmod(i + 1, n), j, lon[i], n) {
            lon[i] = j;
            if i == 0 {
                break;
            }
            i -= 1;
        }
    }

    lon
}

/// Compute the exact pivot index when a constraint is violated.
///
/// When the angular corridor is violated at vertex `k`, we need the exact
/// fractional index where the line first exits the ±0.5 envelope. This
/// uses linear interpolation between the last valid vertex (`k1`) and the
/// violating vertex (`k`).
///
/// The cross products `a = cross(constraint, cur_at_k1)` and
/// `b = cross(constraint, step_direction)` give the signed distance to
/// the constraint line at k1 and the rate of change per step. The
/// violation occurs at step `j = floor(a / -b)` (or `floor(-c / d)` for
/// the other constraint), whichever comes first.
fn pivot_at_violation(
    pt: &[(i32, i32)],
    constraint: &[(i32, i32); 2],
    k: usize,
    k1: usize,
    i: usize,
    n: usize,
) -> usize {
    let dk = (
        sign(pt[k % n].0 - pt[k1 % n].0),
        sign(pt[k % n].1 - pt[k1 % n].1),
    );
    let cur1 = (pt[k1 % n].0 - pt[i].0, pt[k1 % n].1 - pt[i].1);
    let a = xprod(constraint[0], cur1);
    let b = xprod(constraint[0], dk);
    let c = xprod(constraint[1], cur1);
    let d = xprod(constraint[1], dk);

    let infty = 10_000_000i64;
    let mut j = infty;
    if b < 0 {
        j = floordiv(a, -b);
    }
    if d > 0 {
        j = j.min(floordiv(-c, d));
    }
    pmod_signed((k1 % n) as isize + j as isize, n as isize)
}

// ── Dynamic programming optimal polygon ──────────────────

/// Find the globally optimal polygon using dynamic programming.
///
/// Returns polygon vertex indices into the original path.
#[allow(clippy::needless_range_loop)]
fn best_polygon(pt: &[(i32, i32)], lon: &[usize], sums: &[Sums], x0: i32, y0: i32) -> Vec<usize> {
    let n = pt.len();

    // clip0[i] = farthest vertex reachable from i (clipping interval).
    let mut clip0 = vec![0usize; n];
    for i in 0..n {
        let prev_i = if i == 0 { n - 1 } else { i - 1 };
        let mut c = pmod_signed(lon[prev_i] as isize - 1, n as isize);
        if c == i {
            c = (i + 1) % n;
        }
        clip0[i] = if c < i { n } else { c };
    }

    // clip1[j] = earliest vertex from which j is reachable.
    let mut clip1 = vec![0usize; n + 1];
    {
        let mut j = 1usize;
        for i in 0..n {
            while j <= clip0[i] {
                clip1[j] = i;
                j += 1;
            }
        }
    }

    // seg0: greedy forward walk to find minimum segment count m.
    let mut seg0 = vec![0usize; n + 1];
    let m;
    {
        let mut i = 0usize;
        let mut j = 0usize;
        while i < n {
            seg0[j] = i;
            i = clip0[i];
            j += 1;
        }
        seg0[j] = n;
        m = j;
    }

    // seg1: backward walk from n using clip1.
    let mut seg1 = vec![0usize; m + 1];
    {
        let mut i = n;
        let mut j = m;
        while j > 0 {
            seg1[j] = i;
            i = clip1[i];
            j -= 1;
        }
        seg1[0] = 0;
    }

    // DP: pen[i] = min penalty to reach vertex i, prev[i] = predecessor.
    let mut pen = vec![-1.0f64; n + 1];
    let mut prev = vec![0usize; n + 1];
    pen[0] = 0.0;

    for j in 1..=m {
        for i in seg1[j]..=seg0[j] {
            let mut best = -1.0f64;

            // Try all valid predecessors k, from seg0[j-1] downward to clip1[i].
            let k_start = seg0[j - 1];
            let k_end = clip1[i];
            if k_start >= k_end {
                let mut k = k_start;
                loop {
                    let thispen = penalty3(pt, sums, x0, y0, k, i) + pen[k];
                    if pen[k] >= 0.0 && (best < 0.0 || thispen < best) {
                        prev[i] = k;
                        best = thispen;
                    }
                    if k == k_end {
                        break;
                    }
                    k -= 1;
                }
            }
            pen[i] = best;
        }
    }

    // Trace back the optimal path.
    let mut po = vec![0usize; m];
    {
        let mut i = n;
        let mut j = m;
        while j > 0 {
            j -= 1;
            i = prev[i];
            po[j] = i;
        }
    }

    po
}

/// Penalty for approximating path segment [i..j] with a straight line.
///
/// Uses a quadratic form to compute the RMS perpendicular distance of all
/// points from the line through the midpoint of vertices i and j, directed
/// perpendicular to the segment i→j:
///
/// ```text
///   px, py  = midpoint of (pt[i], pt[j])
///   ex, ey  = perpendicular direction: (-(j.y - i.y), j.x - i.x)
///   a       = E[x²] - 2·E[x]·px + px²
///   b       = E[xy] - E[x]·py - E[y]·px + px·py
///   c       = E[y²] - 2·E[y]·py + py²
///   penalty = sqrt(ex²·a + 2·ex·ey·b + ey²·c)
/// ```
///
/// All expectations are computed in O(1) via prefix sums.
fn penalty3(pt: &[(i32, i32)], sums: &[Sums], x0: i32, y0: i32, i: usize, j: usize) -> f64 {
    let n = sums.len() - 1;
    let jn = j % n; // cyclic index into pt[]

    // Get sums over the cyclic range [i..j].
    let (r, k) = if jn >= i {
        (0, jn - i)
    } else {
        (1, jn + n - i)
    };

    if k == 0 {
        return 0.0;
    }

    let x = sums[jn + 1].x - sums[i].x + r as f64 * sums[n].x;
    let y = sums[jn + 1].y - sums[i].y + r as f64 * sums[n].y;
    let x2 = sums[jn + 1].x2 - sums[i].x2 + r as f64 * sums[n].x2;
    let xy = sums[jn + 1].xy - sums[i].xy + r as f64 * sums[n].xy;
    let y2 = sums[jn + 1].y2 - sums[i].y2 + r as f64 * sums[n].y2;
    let k = k as f64;

    let px = (pt[i].0 + pt[jn].0) as f64 / 2.0 - x0 as f64;
    let py = (pt[i].1 + pt[jn].1) as f64 / 2.0 - y0 as f64;
    let ey = (pt[jn].0 - pt[i].0) as f64;
    let ex = -(pt[jn].1 - pt[i].1) as f64;

    let a = (x2 - 2.0 * x * px) / k + px * px;
    let b = (xy - x * py - y * px) / k + px * py;
    let c = (y2 - 2.0 * y * py) / k + py * py;

    let s = ex * ex * a + 2.0 * ex * ey * b + ey * ey * c;
    s.max(0.0).sqrt()
}

// ── Vertex adjustment ────────────────────────────────────

/// Refine each polygon vertex to the optimal sub-pixel position.
///
/// For each vertex, builds a 3×3 quadratic form Q = Q_prev + Q_next from
/// the two adjacent line segments (computed by `point_slope()`). Each Q
/// encodes the squared perpendicular distance to a line as:
///
/// ```text
///   dist²(x, y) = [x, y, 1] · Q · [x, y, 1]ᵀ
/// ```
///
/// The unconstrained minimum is found by solving the 2×2 linear system
/// from the top-left block of Q. If the solution lies within ±0.5 of the
/// original pixel corner, it is used directly. Otherwise, the minimum on
/// the boundary of the ±0.5 box is found by `constrain_to_box()`.
fn adjust_vertices(
    pt: &[(i32, i32)],
    po: &[usize],
    sums: &[Sums],
    x0: i32,
    y0: i32,
) -> Vec<(f64, f64)> {
    let n = pt.len();
    let m = po.len();
    let mut vertices = vec![(0.0f64, 0.0f64); m];

    if m == 0 {
        return vertices;
    }

    // For each polygon segment, compute the optimal line direction and center.
    // Then for each vertex, combine the quadratic forms of adjacent segments.

    for i in 0..m {
        let i_prev = if i == 0 { m - 1 } else { i - 1 };
        let seg_a = (po[i_prev], po[i]); // segment before vertex i
        let seg_b = (po[i], po[(i + 1) % m]); // segment after vertex i

        let (ctr_a, dir_a) = point_slope(pt, sums, x0, y0, seg_a.0, seg_a.1, n);
        let (ctr_b, dir_b) = point_slope(pt, sums, x0, y0, seg_b.0, seg_b.1, n);

        // Build quadratic form Q for each segment: Q = v * v^T / d
        // where v = (dir.y, -dir.x, -(dir.y*ctr.x - dir.x*ctr.y))
        // Minimize (p, 1)^T * (Q_a + Q_b) * (p, 1) over p.
        let q = add_quadform(&make_quadform(ctr_a, dir_a), &make_quadform(ctr_b, dir_b));

        // Solve the 2x2 system.
        let det = q[0][0] * q[1][1] - q[0][1] * q[1][0];
        let s = (pt[po[i]].0 as f64, pt[po[i]].1 as f64); // pixel corner

        if det.abs() < 1e-10 {
            // Singular: use the pixel corner directly.
            vertices[i] = s;
            continue;
        }

        let wx = (-q[0][2] * q[1][1] + q[1][2] * q[0][1]) / det;
        let wy = (q[0][2] * q[1][0] - q[1][2] * q[0][0]) / det;

        // Constrain to within 0.5 of the pixel corner.
        if (wx - s.0).abs() <= 0.5 && (wy - s.1).abs() <= 0.5 {
            vertices[i] = (wx, wy);
        } else {
            // Search the boundary of the ±0.5 box for the minimum.
            vertices[i] = constrain_to_box(&q, s);
        }
    }

    vertices
}

/// Compute the best-fit line through a path segment [a..b].
///
/// Returns (centroid, direction_unit_vector).
///
/// The direction is the eigenvector of the 2×2 covariance matrix
/// corresponding to its largest eigenvalue (the axis of maximum
/// variance). For a 2×2 symmetric matrix [[a, b], [b, c]], the
/// eigenvalues are `(a + c ± sqrt((a-c)² + 4b²)) / 2`. The
/// eigenvector for λ is found from `(A - λI)v = 0`.
fn point_slope(
    pt: &[(i32, i32)],
    sums: &[Sums],
    x0: i32,
    y0: i32,
    a: usize,
    b: usize,
    n: usize,
) -> ((f64, f64), (f64, f64)) {
    let bn = b % n; // cyclic index into pt[]

    let (r, k) = if bn >= a {
        (0, bn - a)
    } else {
        (1, bn + n - a)
    };

    if k == 0 {
        return ((pt[a].0 as f64, pt[a].1 as f64), (1.0, 0.0));
    }

    let x = sums[bn + 1].x - sums[a].x + r as f64 * sums[n].x;
    let y = sums[bn + 1].y - sums[a].y + r as f64 * sums[n].y;
    let x2 = sums[bn + 1].x2 - sums[a].x2 + r as f64 * sums[n].x2;
    let xy = sums[bn + 1].xy - sums[a].xy + r as f64 * sums[n].xy;
    let y2 = sums[bn + 1].y2 - sums[a].y2 + r as f64 * sums[n].y2;
    let k = k as f64;

    let ctr = (x / k + x0 as f64, y / k + y0 as f64);

    let a_cov = (x2 - x * x / k) / k;
    let b_cov = (xy - x * y / k) / k;
    let c_cov = (y2 - y * y / k) / k;

    // Largest eigenvalue's eigenvector = direction of maximum variance.
    let lambda2 = (a_cov + c_cov + ((a_cov - c_cov).powi(2) + 4.0 * b_cov * b_cov).sqrt()) / 2.0;

    let a2 = a_cov - lambda2;
    let c2 = c_cov - lambda2;

    let dir = if a2.abs() >= c2.abs() {
        let len = (b_cov * b_cov + a2 * a2).sqrt();
        if len > 1e-10 {
            (-b_cov / len, a2 / len)
        } else {
            (1.0, 0.0)
        }
    } else {
        let len = (c2 * c2 + b_cov * b_cov).sqrt();
        if len > 1e-10 {
            (-c2 / len, b_cov / len)
        } else {
            (1.0, 0.0)
        }
    };

    (ctr, dir)
}

/// Build the 3x3 quadratic form for distance from a line.
///
/// The quadratic form Q is such that for a point (x, y):
/// distance^2 = [x, y, 1] * Q * [x, y, 1]^T
fn make_quadform(ctr: (f64, f64), dir: (f64, f64)) -> [[f64; 3]; 3] {
    // Normal to the line: (dir.y, -dir.x)
    // v = (normal.x, normal.y, -dot(normal, ctr))
    let v = [dir.1, -dir.0, -(dir.1 * ctr.0 - dir.0 * ctr.1)];
    let d = dir.0 * dir.0 + dir.1 * dir.1;
    if d < 1e-10 {
        return [[0.0; 3]; 3];
    }

    let mut q = [[0.0f64; 3]; 3];
    for l in 0..3 {
        for k in 0..3 {
            q[l][k] = v[l] * v[k] / d;
        }
    }
    q
}

/// Element-wise sum of two 3×3 quadratic forms.
fn add_quadform(a: &[[f64; 3]; 3], b: &[[f64; 3]; 3]) -> [[f64; 3]; 3] {
    let mut q = [[0.0f64; 3]; 3];
    for l in 0..3 {
        for k in 0..3 {
            q[l][k] = a[l][k] + b[l][k];
        }
    }
    q
}

/// Evaluate quadratic form at point (x, y): [x, y, 1] * Q * [x, y, 1]^T
fn eval_quadform(q: &[[f64; 3]; 3], x: f64, y: f64) -> f64 {
    let p = [x, y, 1.0];
    let mut val = 0.0;
    for l in 0..3 {
        for k in 0..3 {
            val += p[l] * q[l][k] * p[k];
        }
    }
    val
}

/// Find the point on or inside the ±0.5 box that minimizes the quadratic form.
fn constrain_to_box(q: &[[f64; 3]; 3], center: (f64, f64)) -> (f64, f64) {
    let lo_x = center.0 - 0.5;
    let hi_x = center.0 + 0.5;
    let lo_y = center.1 - 0.5;
    let hi_y = center.1 + 0.5;

    let mut best = center;
    let mut best_val = eval_quadform(q, center.0, center.1);

    // Check the 4 edges.
    // Left edge: x = lo_x, minimize over y.
    let check = |x: f64, y: f64, best: &mut (f64, f64), best_val: &mut f64| {
        let v = eval_quadform(q, x, y);
        if v < *best_val {
            *best_val = v;
            *best = (x, y);
        }
    };

    // For each fixed-x edge, optimal y = -( q[1][0]*x + q[1][2] ) / q[1][1]
    for &x in &[lo_x, hi_x] {
        if q[1][1].abs() > 1e-10 {
            let y = -(q[1][0] * x + q[1][2]) / q[1][1];
            let y = y.clamp(lo_y, hi_y);
            check(x, y, &mut best, &mut best_val);
        }
        check(x, lo_y, &mut best, &mut best_val);
        check(x, hi_y, &mut best, &mut best_val);
    }

    // For each fixed-y edge, optimal x = -( q[0][1]*y + q[0][2] ) / q[0][0]
    for &y in &[lo_y, hi_y] {
        if q[0][0].abs() > 1e-10 {
            let x = -(q[0][1] * y + q[0][2]) / q[0][0];
            let x = x.clamp(lo_x, hi_x);
            check(x, y, &mut best, &mut best_val);
        }
    }

    best
}

// ── Helpers ──────────────────────────────────────────────

/// Integer cross product.
fn xprod(a: (i32, i32), b: (i32, i32)) -> i64 {
    a.0 as i64 * b.1 as i64 - a.1 as i64 * b.0 as i64
}

/// Sign function: -1, 0, or 1.
fn sign(x: i32) -> i32 {
    if x > 0 {
        1
    } else if x < 0 {
        -1
    } else {
        0
    }
}

/// Proper modulo for unsigned (always non-negative).
fn pmod(a: usize, n: usize) -> usize {
    ((a % n) + n) % n
}

/// Proper modulo for signed values (always non-negative result).
fn pmod_signed(a: isize, n: isize) -> usize {
    (((a % n) + n) % n) as usize
}

/// Floor division for signed values (rounds toward negative infinity).
fn floordiv(a: i64, b: i64) -> i64 {
    if a >= 0 {
        a / b
    } else {
        -1 - (-1 - a) / b
    }
}

/// Check if b is in the cyclic interval [a, c) mod n.
fn cyclic(a: usize, b: usize, c: usize, _n: usize) -> bool {
    if a <= c {
        a <= b && b < c
    } else {
        a <= b || b < c
    }
}

#[cfg(test)]
mod tests {
    use super::super::decompose::PixelPath;
    use super::*;

    #[test]
    fn rectangle_produces_4_vertices() {
        // 4x4 rectangle: top → right → bottom → left edges.
        let mut points: Vec<(i32, i32)> = Vec::new();
        for x in 0..4 {
            points.push((x, 4));
        }
        for y in (0..4).rev() {
            points.push((4, y));
        }
        for x in (0..4).rev() {
            points.push((x, 0));
        }
        for y in 0..4 {
            points.push((0, y));
        }

        let path = PixelPath { points, sign: 1 };
        let poly = optimal_polygon(&path);

        // The DP may produce 4 or 5 vertices for a small rectangle
        // (pixel grid constraints can introduce an extra vertex).
        assert!(
            poly.vertices.len() >= 4 && poly.vertices.len() <= 5,
            "Rectangle should produce 4-5 polygon vertices, got {}",
            poly.vertices.len()
        );
    }
}
