//! Optimal polygon approximation via dynamic programming.
//!
//! 1. `calc_sums` — prefix sums for O(1) line-fit statistics
//! 2. `calc_lon` — longest straight subpath per vertex
//! 3. `best_polygon` — DP for globally optimal polygon
//! 4. `adjust_vertices` — sub-pixel vertex refinement

#![allow(clippy::needless_range_loop)]

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
#[allow(dead_code)]
pub struct Polygon {
    /// Sub-pixel-refined vertex positions.
    pub vertices: Vec<(f64, f64)>,
    /// +1 for outer, -1 for hole (from the source PixelPath).
    pub sign: i8,
}

/// Compute the optimal polygon for a pixel path.
///
/// Prefix sums → longest straight subpaths → DP optimal polygon →
/// sub-pixel vertex refinement.
pub fn optimal_polygon(path: &PixelPath) -> Polygon {
    let n = path.points.len();
    if n < 4 {
        let vertices = path.points.iter().map(|&(x, y)| (x as f64, y as f64)).collect();
        return Polygon { vertices, sign: path.sign };
    }

    let (sums, x0, y0) = calc_sums(&path.points);
    let lon = calc_lon(&path.points);
    let po = best_polygon(&path.points, &lon, &sums, x0, y0);
    let vertices = adjust_vertices(&path.points, &po, &sums, x0, y0);

    Polygon {
        vertices,
        sign: path.sign,
    }
}

// ── Prefix sums ──────────────────────────────────────────

/// Compute prefix sums for O(1) range queries on line-fit statistics.
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
    let infty = 10_000_000i64;

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
                // goto foundk
                break;
            }

            let cur = (pt[k % n].0 - pt[i].0, pt[k % n].1 - pt[i].1);

            // Check constraint violations.
            if xprod(constraint[0], cur) < 0 || xprod(constraint[1], cur) > 0 {
                // goto constraint_viol — but handled inline below.
                // Compute exact violation point using linear interpolation.
                let dk = (
                    sign(pt[k % n].0 - pt[k1 % n].0),
                    sign(pt[k % n].1 - pt[k1 % n].1),
                );
                let cur1 = (pt[k1 % n].0 - pt[i].0, pt[k1 % n].1 - pt[i].1);
                let a = xprod(constraint[0], cur1);
                let b = xprod(constraint[0], dk);
                let c = xprod(constraint[1], cur1);
                let d = xprod(constraint[1], dk);

                let mut j = infty;
                if b < 0 {
                    j = floordiv(a, -b);
                }
                if d > 0 {
                    j = j.min(floordiv(-c, d));
                }
                pivk[i] = pmod_signed((k1 % n) as isize + j as isize, n as isize);
                // goto foundk
                break;
            }

            // Update constraints (skip when |cur| <= 1 — no constraint).
            if !(cur.0.abs() <= 1 && cur.1.abs() <= 1) {
                let off0 = (
                    cur.0 + if cur.1 >= 0 && (cur.1 > 0 || cur.0 < 0) { 1 } else { -1 },
                    cur.1 + if cur.0 <= 0 && (cur.0 < 0 || cur.1 < 0) { 1 } else { -1 },
                );
                if xprod(constraint[0], off0) >= 0 {
                    constraint[0] = off0;
                }

                let off1 = (
                    cur.0 + if cur.1 <= 0 && (cur.1 < 0 || cur.0 < 0) { 1 } else { -1 },
                    cur.1 + if cur.0 >= 0 && (cur.0 > 0 || cur.1 < 0) { 1 } else { -1 },
                );
                if xprod(constraint[1], off1) <= 0 {
                    constraint[1] = off1;
                }
            }

            k1 = k;
            k = nc[k1 % n];

            if !cyclic(k % n, i, k1 % n, n) {
                // Went all the way around without constraint violation.
                // Same constraint_viol logic applies here.
                let dk = (
                    sign(pt[k % n].0 - pt[k1 % n].0),
                    sign(pt[k % n].1 - pt[k1 % n].1),
                );
                let cur1 = (pt[k1 % n].0 - pt[i].0, pt[k1 % n].1 - pt[i].1);
                let a = xprod(constraint[0], cur1);
                let b = xprod(constraint[0], dk);
                let c = xprod(constraint[1], cur1);
                let d = xprod(constraint[1], dk);

                let mut j = infty;
                if b < 0 {
                    j = floordiv(a, -b);
                }
                if d > 0 {
                    j = j.min(floordiv(-c, d));
                }
                pivk[i] = pmod_signed((k1 % n) as isize + j as isize, n as isize);
                break;
            }
        }
        // foundk:
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

// ── Dynamic programming optimal polygon ──────────────────

/// Find the globally optimal polygon using dynamic programming.
///
/// Returns polygon vertex indices into the original path.
fn best_polygon(
    pt: &[(i32, i32)],
    lon: &[usize],
    sums: &[Sums],
    x0: i32,
    y0: i32,
) -> Vec<usize> {
    let n = pt.len();

    // clip0[i] = farthest vertex reachable from i (clipping interval).
    // C: c = mod(lon[mod(i-1,n)]-1, n); if c==i then c=mod(i+1,n); if c<i then clip0=n else clip0=c.
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
/// Returns the RMS distance of points from the best-fit line through
/// the midpoint of i and j, projected onto the perpendicular direction.
fn penalty3(
    pt: &[(i32, i32)],
    sums: &[Sums],
    x0: i32,
    y0: i32,
    i: usize,
    j: usize,
) -> f64 {
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
/// For each vertex, solves a quadratic optimization: minimize the sum of
/// squared distances to the two adjacent line segments, constrained to
/// lie within ±0.5 of the original pixel corner.
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
        let q = add_quadform(
            &make_quadform(ctr_a, dir_a),
            &make_quadform(ctr_b, dir_b),
        );

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
        return (
            (pt[a].0 as f64, pt[a].1 as f64),
            (1.0, 0.0),
        );
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
    use super::*;

    #[test]
    fn test_calc_lon_rectangle() {
        // A simple 4x4 rectangle on the pixel grid (pixel-corner coordinates).
        // Goes: right along top, down the right side, left along bottom, up the left side.
        let mut pt: Vec<(i32, i32)> = Vec::new();
        // Top edge: (0,4) -> (4,4)
        for x in 0..4 { pt.push((x, 4)); }
        // Right edge: (4,4) -> (4,0)
        for y in (0..4).rev() { pt.push((4, y)); }
        // Bottom edge: (4,0) -> (0,0)
        for x in (0..4).rev() { pt.push((x, 0)); }
        // Left edge: (0,0) -> (0,4)
        for y in 0..4 { pt.push((0, y)); }

        let n = pt.len(); // Should be 16
        println!("Rectangle test: n={}", n);
        println!("Points:");
        for (i, p) in pt.iter().enumerate() {
            println!("  pt[{:2}] = ({}, {})", i, p.0, p.1);
        }

        // ── nc (next corner) ──
        // Recompute nc here so we can print it.
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
        println!("\nnc (next corner after each vertex):");
        for (i, &c) in nc.iter().enumerate() {
            println!("  nc[{:2}] = {:2}  (pt[{}] = {:?})", i, c, c % n, pt[c % n]);
        }

        // ── calc_lon ──
        let lon = calc_lon(&pt);
        println!("\nlon (farthest reachable vertex from each vertex):");
        for (i, &l) in lon.iter().enumerate() {
            println!("  lon[{:2}] = {:2}  (from {:?} can reach {:?})", i, l, pt[i], pt[l % n]);
        }

        // ── Expected analysis ──
        println!("\n--- Expected lon analysis ---");
        println!("For a 4x4 rectangle with 16 vertices:");
        println!("Each edge has 4 vertices. From the start of an edge,");
        println!("a straight line should reach the end of that edge (3 steps)");
        println!("plus possibly the next corner vertex.");
        println!();
        println!("Edge vertices:          corners at indices 0, 4, 8, 12");
        println!("Top edge:    pt[0..4]   = (0,4)..(3,4)");
        println!("Right edge:  pt[4..8]   = (4,3)..(4,0)");
        println!("Bottom edge: pt[8..12]  = (3,0)..(0,0)");
        println!("Left edge:   pt[12..16] = (0,1)..(0,3)");
        println!();
        println!("From corner pt[0]=(0,4), the straight line goes along the top edge.");
        println!("It should reach at least pt[3]=(3,4), and potentially pt[4]=(4,3)");
        println!("depending on the half-pixel tolerance.");

        // ── calc_sums + best_polygon ──
        let (sums, x0, y0) = calc_sums(&pt);
        println!("\nPrefix sums: x0={}, y0={}", x0, y0);

        let po = best_polygon(&pt, &lon, &sums, x0, y0);
        println!("\nOptimal polygon: m={} vertices", po.len());
        println!("po indices: {:?}", &po);
        println!("po coordinates:");
        for (j, &idx) in po.iter().enumerate() {
            println!("  po[{}] = pt[{}] = {:?}", j, idx, pt[idx]);
        }

        // ── Verify ──
        println!("\n--- Verification ---");
        assert_eq!(n, 16, "Expected 16 points for a 4x4 rectangle");

        // For a rectangle, we expect the optimal polygon to have exactly 4 vertices
        // at or near the 4 corners.
        assert!(
            po.len() >= 4,
            "Rectangle should have at least 4 polygon vertices, got {}",
            po.len()
        );

        // Ideally exactly 4 for a clean rectangle
        if po.len() == 4 {
            println!("PASS: Got exactly 4 polygon vertices (ideal for a rectangle)");
        } else {
            println!(
                "NOTE: Got {} polygon vertices (expected 4 for a clean rectangle)",
                po.len()
            );
        }

        // Also run full adjust_vertices to see final positions
        let vertices = adjust_vertices(&pt, &po, &sums, x0, y0);
        println!("\nAdjusted vertices:");
        for (j, v) in vertices.iter().enumerate() {
            println!("  v[{}] = ({:.3}, {:.3})  (from pt[{}] = {:?})", j, v.0, v.1, po[j], pt[po[j]]);
        }
    }
}
