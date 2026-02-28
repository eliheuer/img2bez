//! Corner detection and minimal Bezier curve fitting from an optimal polygon.
//!
//! Uses the alpha parameter to classify polygon vertices as corners
//! or smooth, then fits minimal cubic Beziers through smooth sections
//! using kurbo's `fit_to_bezpath` (two-pass fitting for clean output
//! with minimal points).

use kurbo::{fit_to_bezpath_opt, simplify::SimplifyBezPath, BezPath, PathEl, Point};

use super::polygon::Polygon;

/// Parameters for curve generation.
pub struct CurveParams {
    /// Maximum alpha for smooth curves. Vertices with alpha >= this
    /// are corners. Default: 1.0.
    pub alphamax: f64,
    /// Curve fitting accuracy (in pixel-corner coordinates).
    /// Smaller = more points, closer fit. ~0.5 is good for type.
    pub accuracy: f64,
    /// Laplacian smoothing iterations applied to polygon vertices
    /// before curve fitting. Removes pixel staircase noise.
    /// 0 = no smoothing. 3 = good default.
    pub smooth_iterations: usize,
}

impl Default for CurveParams {
    fn default() -> Self {
        Self {
            alphamax: 1.0,
            accuracy: 0.5,
            smooth_iterations: 3,
        }
    }
}

/// Convert a polygon to a BezPath with minimal cubic Beziers.
///
/// 1. Classify each vertex as corner or smooth via the alpha parameter.
/// 2. Split at corners.
/// 3. Fit minimal cubics through each smooth section using kurbo.
/// 4. Connect with line segments at corners.
pub fn polygon_to_bezpath(poly: &Polygon, params: &CurveParams) -> BezPath {
    let m = poly.vertices.len();
    if m < 3 {
        return BezPath::new();
    }

    let v = &poly.vertices;

    // Compute alpha for each vertex.
    let alphas: Vec<f64> = (0..m)
        .map(|j| {
            let i = if j == 0 { m - 1 } else { j - 1 };
            let k = (j + 1) % m;
            compute_alpha(v[i], v[j], v[k])
        })
        .collect();

    // Find corner indices.
    let corners: Vec<usize> = (0..m)
        .filter(|&j| alphas[j] >= params.alphamax)
        .collect();

    if corners.is_empty() {
        // All smooth: smooth cyclically then fit entire closed curve.
        let smoothed = smooth_closed(v, params.smooth_iterations);
        return fit_smooth_closed(&smoothed, params.accuracy);
    }

    let mut path = BezPath::new();
    let first = v[corners[0]];
    path.move_to(Point::new(first.0, first.1));

    let num_corners = corners.len();
    for ci in 0..num_corners {
        let start = corners[ci];
        let end = corners[(ci + 1) % num_corners];

        // Extract the vertices between these two corners (inclusive).
        let segment = extract_cyclic(v, start, end, m);

        if segment.len() <= 2 {
            // Straight line to next corner.
            let p = v[end];
            path.line_to(Point::new(p.0, p.1));
        } else {
            // Smooth interior points (corners stay fixed), then fit.
            let smoothed = smooth_segment(&segment, params.smooth_iterations);
            let fitted = fit_segment(&smoothed, params.accuracy);
            for el in fitted.elements().iter().skip(1) {
                path.push(*el);
            }
        }
    }

    path.push(PathEl::ClosePath);
    path
}

// ── Curve fitting ────────────────────────────────────────

/// Fit a smooth closed curve (no corners at all).
fn fit_smooth_closed(vertices: &[(f64, f64)], accuracy: f64) -> BezPath {
    let polyline = points_to_path(vertices, true);
    two_pass_fit(&polyline, accuracy)
}

/// Fit minimal cubics through an open segment between two corners.
fn fit_segment(points: &[(f64, f64)], accuracy: f64) -> BezPath {
    let polyline = points_to_path(points, false);
    two_pass_fit(&polyline, accuracy)
}

/// Two-pass fitting: polyline → curves → minimal curves.
///
/// The second pass dramatically reduces points because smooth curves
/// simplify far better than noisy polylines.
fn two_pass_fit(path: &BezPath, accuracy: f64) -> BezPath {
    let pass1 = fit_to_bezpath_opt(
        &SimplifyBezPath::new(path.elements().iter().copied()),
        accuracy,
    );
    fit_to_bezpath_opt(
        &SimplifyBezPath::new(pass1.elements().iter().copied()),
        accuracy,
    )
}

/// Convert (x, y) tuples to a line-segment BezPath.
fn points_to_path(points: &[(f64, f64)], closed: bool) -> BezPath {
    let mut path = BezPath::new();
    if let Some(&(x, y)) = points.first() {
        path.move_to(Point::new(x, y));
        for &(x, y) in &points[1..] {
            path.line_to(Point::new(x, y));
        }
        if closed {
            path.push(PathEl::ClosePath);
        }
    }
    path
}

// ── Polygon smoothing ────────────────────────────────────

/// Laplacian smoothing for an open polyline segment.
/// Endpoints (corners) are kept fixed; interior points are averaged
/// with their neighbors for `iterations` passes.
fn smooth_segment(points: &[(f64, f64)], iterations: usize) -> Vec<(f64, f64)> {
    if iterations == 0 || points.len() <= 2 {
        return points.to_vec();
    }
    let mut pts = points.to_vec();
    let n = pts.len();
    for _ in 0..iterations {
        let prev = pts.clone();
        for i in 1..n - 1 {
            pts[i].0 = (prev[i - 1].0 + prev[i].0 + prev[i + 1].0) / 3.0;
            pts[i].1 = (prev[i - 1].1 + prev[i].1 + prev[i + 1].1) / 3.0;
        }
    }
    pts
}

/// Laplacian smoothing for a closed polyline (no corners).
/// All points are averaged cyclically with their neighbors.
fn smooth_closed(points: &[(f64, f64)], iterations: usize) -> Vec<(f64, f64)> {
    if iterations == 0 || points.len() < 3 {
        return points.to_vec();
    }
    let mut pts = points.to_vec();
    let n = pts.len();
    for _ in 0..iterations {
        let prev = pts.clone();
        for i in 0..n {
            let p = if i == 0 { n - 1 } else { i - 1 };
            let nx = (i + 1) % n;
            pts[i].0 = (prev[p].0 + prev[i].0 + prev[nx].0) / 3.0;
            pts[i].1 = (prev[p].1 + prev[i].1 + prev[nx].1) / 3.0;
        }
    }
    pts
}

// ── Helpers ──────────────────────────────────────────────

/// Extract a cyclic sub-sequence from `start` to `end` (inclusive).
///
/// When `start == end` (single corner case), returns the full cycle:
/// all `total` vertices starting from `start` plus wrapping back to `start`.
fn extract_cyclic(
    points: &[(f64, f64)],
    start: usize,
    end: usize,
    total: usize,
) -> Vec<(f64, f64)> {
    let mut result = Vec::new();
    let mut i = start;
    let mut first = true;
    loop {
        result.push(points[i]);
        if i == end && !first {
            break;
        }
        first = false;
        i = (i + 1) % total;
    }
    result
}

/// Compute the alpha (smoothness) parameter for vertex j with neighbors i and k.
///
/// Alpha is based on the cross product and perpendicular distance of j
/// from the line i..k, normalized by the orthogonal distance.
/// Higher alpha = more "round" (smoother curve). Lower alpha = sharper.
fn compute_alpha(vi: (f64, f64), vj: (f64, f64), vk: (f64, f64)) -> f64 {
    // dpara: cross product of (j-i, k-i) = twice signed area of triangle ijk
    let dpara = (vj.0 - vi.0) * (vk.1 - vi.1) - (vj.1 - vi.1) * (vk.0 - vi.0);

    // ddenom: uses dorth_infty (90° rotation snapped to cardinal direction)
    let dorth = dorth_infty(vi, vk);
    let ddenom = dorth.1 * (vk.0 - vi.0) - dorth.0 * (vk.1 - vi.1);

    if ddenom.abs() < 1e-10 {
        return 4.0 / 3.0; // degenerate: treat as very smooth
    }

    let dd = (dpara / ddenom).abs();

    let alpha = if dd > 1.0 { 1.0 - 1.0 / dd } else { 0.0 };
    alpha / 0.75
}

/// 90-degree CCW rotation of direction from p0 to p2, snapped to cardinal.
fn dorth_infty(p0: (f64, f64), p2: (f64, f64)) -> (f64, f64) {
    let dx = p2.0 - p0.0;
    let dy = p2.1 - p0.1;
    (-fsign(dy), fsign(dx))
}

/// Sign function for f64.
fn fsign(x: f64) -> f64 {
    if x > 0.0 {
        1.0
    } else if x < 0.0 {
        -1.0
    } else {
        0.0
    }
}
