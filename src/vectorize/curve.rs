//! Corner and extrema detection with minimal Bezier curve fitting from an optimal polygon.
//!
//! Splits the polygon at both corners (via alpha parameter) and extrema
//! (where x or y changes direction), then fits minimal cubic Beziers
//! through each short section. Because sections run extrema-to-extrema,
//! handles naturally align H/V and point counts stay minimal.

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

/// Minimum absolute protrusion (in polygon-coordinate units) for a
/// vertex to count as a segment extremum. Filters out sub-pixel noise.
/// Also uses a relative threshold (5% of segment diagonal) for small features.
const MIN_PROTRUSION_ABS: f64 = 2.0;
const MIN_PROTRUSION_REL: f64 = 0.05;

/// Find the bounding-box extrema of an entire contour (no-corners case).
///
/// Returns the indices of the single topmost, bottommost, leftmost,
/// rightmost vertices — exactly 4 (or fewer if coincident).
fn find_contour_extrema(vertices: &[(f64, f64)]) -> Vec<usize> {
    let m = vertices.len();
    if m < 4 {
        return vec![];
    }
    let mut min_x = 0;
    let mut max_x = 0;
    let mut min_y = 0;
    let mut max_y = 0;
    for j in 1..m {
        if vertices[j].0 < vertices[min_x].0 { min_x = j; }
        if vertices[j].0 > vertices[max_x].0 { max_x = j; }
        if vertices[j].1 < vertices[min_y].1 { min_y = j; }
        if vertices[j].1 > vertices[max_y].1 { max_y = j; }
    }
    let mut extrema = vec![min_x, max_x, min_y, max_y];
    extrema.sort_unstable();
    extrema.dedup();
    extrema
}

/// Find the most extreme interior vertex per axis within a segment.
///
/// Between two split points (corners), finds the single vertex that
/// protrudes furthest in each axis direction beyond both endpoints.
/// Only includes vertices that protrude by at least `MIN_PROTRUSION`.
fn find_segment_extrema(
    vertices: &[(f64, f64)],
    start: usize,
    end: usize,
    total: usize,
) -> Vec<usize> {
    let (sx, sy) = vertices[start];
    let (ex, ey) = vertices[end];
    let baseline_min_x = sx.min(ex);
    let baseline_max_x = sx.max(ex);
    let baseline_min_y = sy.min(ey);
    let baseline_max_y = sy.max(ey);

    // Dynamic protrusion threshold: use the smaller of the absolute
    // minimum and a percentage of the segment diagonal. This lets
    // small features (like the tail of an 'a') register extrema
    // that would be filtered out by a fixed absolute threshold.
    let diag = ((ex - sx).powi(2) + (ey - sy).powi(2)).sqrt();
    let min_protrusion = MIN_PROTRUSION_ABS.min(diag * MIN_PROTRUSION_REL).max(0.5);

    let mut min_x = baseline_min_x;
    let mut max_x = baseline_max_x;
    let mut min_y = baseline_min_y;
    let mut max_y = baseline_max_y;
    let mut min_x_idx: Option<usize> = None;
    let mut max_x_idx: Option<usize> = None;
    let mut min_y_idx: Option<usize> = None;
    let mut max_y_idx: Option<usize> = None;

    let mut i = (start + 1) % total;
    while i != end {
        let (x, y) = vertices[i];
        if x < min_x { min_x = x; min_x_idx = Some(i); }
        if x > max_x { max_x = x; max_x_idx = Some(i); }
        if y < min_y { min_y = y; min_y_idx = Some(i); }
        if y > max_y { max_y = y; max_y_idx = Some(i); }
        i = (i + 1) % total;
    }

    let mut extrema = Vec::new();
    if let Some(i) = min_x_idx {
        if baseline_min_x - min_x >= min_protrusion { extrema.push(i); }
    }
    if let Some(i) = max_x_idx {
        if max_x - baseline_max_x >= min_protrusion { extrema.push(i); }
    }
    if let Some(i) = min_y_idx {
        if baseline_min_y - min_y >= min_protrusion { extrema.push(i); }
    }
    if let Some(i) = max_y_idx {
        if max_y - baseline_max_y >= min_protrusion { extrema.push(i); }
    }

    extrema.sort_unstable();
    extrema.dedup();
    extrema
}

/// Convert a polygon to a BezPath with minimal cubic Beziers.
///
/// 1. Classify each vertex as corner or smooth via the alpha parameter.
/// 2. Find significant extrema (per-segment or global bounding box).
/// 3. Merge corners + extrema into split points.
/// 4. Split contour at all split points.
/// 5. Smooth + fit each section with a single-pass curve fit.
pub fn polygon_to_bezpath(poly: &Polygon, params: &CurveParams) -> BezPath {
    let m = poly.vertices.len();
    if m < 3 {
        return BezPath::new();
    }

    let v = &poly.vertices;

    // DEBUG: output polygon as straight lines (no curve fitting)
    if std::env::var("IMG2BEZ_DEBUG_POLYGON").is_ok() {
        let mut path = BezPath::new();
        path.move_to(Point::new(v[0].0, v[0].1));
        for &(x, y) in &v[1..] {
            path.line_to(Point::new(x, y));
        }
        path.push(PathEl::ClosePath);
        return path;
    }

    // Compute alpha for each vertex.
    let alphas: Vec<f64> = (0..m)
        .map(|j| {
            let i = if j == 0 { m - 1 } else { j - 1 };
            let k = (j + 1) % m;
            compute_alpha(v[i], v[j], v[k])
        })
        .collect();

    // Find corner indices (sharp turns).
    let corners: Vec<usize> = (0..m)
        .filter(|&j| alphas[j] >= params.alphamax)
        .collect();

    // Find curvature transitions: where the polygon changes from
    // straight (alpha ≈ 0) to curved or vice versa. These are
    // the natural split points at stem/crossbar boundaries.
    let transitions = find_curvature_transitions(&alphas, 0.3);

    // Build split points: corners + transitions.
    // Corners are always kept (structurally important, even when close together).
    // Transitions that fall within 3 vertices of a corner are dropped
    // (they're redundant with the corner), but transitions far from
    // corners are kept for stem/crossbar boundaries.
    let corner_set: std::collections::HashSet<usize> = corners.iter().copied().collect();
    let filtered_transitions: Vec<usize> = transitions
        .iter()
        .copied()
        .filter(|&t| {
            // Keep this transition only if it's > 3 vertices from any corner.
            !corners.iter().any(|&c| {
                let d = if t > c { t - c } else { c - t };
                let d = d.min(m - d); // cyclic distance
                d <= 3
            })
        })
        .collect();
    let mut base_splits: Vec<usize> = corners.iter().copied()
        .chain(filtered_transitions.iter().copied())
        .collect();
    base_splits.sort_unstable();
    base_splits.dedup();
    // Merge only non-corner transitions that cluster together.
    base_splits = merge_nearby_preserve_corners(base_splits, 3, m, &corner_set);

    let split_points = if base_splits.is_empty() {
        // No corners or transitions: use global bounding-box extrema.
        find_contour_extrema(v)
    } else {
        // Per-segment: find extrema only in curved sections (not straight).
        let mut pts = base_splits.clone();
        let ns = base_splits.len();
        for si in 0..ns {
            let start = base_splits[si];
            let end = base_splits[(si + 1) % ns];
            let segment = extract_cyclic(v, start, end, m);
            if segment.len() > 2 {
                let smoothed = laplacian_smooth(&segment, params.smooth_iterations, false);
                let dev = collinear_deviation(&smoothed);
                let s0 = smoothed[0];
                let sn = smoothed[smoothed.len() - 1];
                let slen = ((sn.0 - s0.0).powi(2) + (sn.1 - s0.1).powi(2)).sqrt();
                let tol = (slen * 0.02).max(3.0);
                if dev > tol {
                    pts.extend(find_segment_extrema(v, start, end, m));
                }
            }
        }
        pts.sort_unstable();
        pts.dedup();
        pts
    };

    // DEBUG: print split analysis when IMG2BEZ_DEBUG_SPLITS is set
    if std::env::var("IMG2BEZ_DEBUG_SPLITS").is_ok() {
        eprintln!("=== polygon_to_bezpath debug (contour with {} vertices) ===", m);
        eprintln!("  Total polygon vertices: {}", m);
        eprintln!("  Corners ({}):", corners.len());
        for &ci in &corners {
            eprintln!("    corner[{}] = ({:.1}, {:.1})  alpha={:.3}", ci, v[ci].0, v[ci].1, alphas[ci]);
        }
        eprintln!("  Transitions ({}):", transitions.len());
        for &ti in &transitions {
            eprintln!("    transition[{}] = ({:.1}, {:.1})  alpha={:.3}", ti, v[ti].0, v[ti].1, alphas[ti]);
        }
        eprintln!("  Final split_points ({}):", split_points.len());
        for &si in &split_points {
            let label = if corners.contains(&si) {
                "corner"
            } else if transitions.contains(&si) {
                "transition"
            } else {
                "extremum"
            };
            eprintln!("    split[{}] = ({:.1}, {:.1})  [{}]", si, v[si].0, v[si].1, label);
        }
        eprintln!("===");
    }


    if split_points.is_empty() {
        // No corners or extrema: smooth cyclically then fit entire closed curve.
        let smoothed = laplacian_smooth(v, params.smooth_iterations, true);
        return fit_smooth_closed(&smoothed, params.accuracy);
    }

    let mut path = BezPath::new();
    let first = v[split_points[0]];
    path.move_to(Point::new(first.0, first.1));

    let num_splits = split_points.len();
    for si in 0..num_splits {
        let start = split_points[si];
        let end = split_points[(si + 1) % num_splits];

        // Extract the vertices between these two split points (inclusive).
        let segment = extract_cyclic(v, start, end, m);

        if segment.len() <= 2 {
            let p = v[end];
            path.line_to(Point::new(p.0, p.1));
        } else {
            // Smooth first, then decide: line or curve.
            let smoothed = laplacian_smooth(&segment, params.smooth_iterations, false);
            let max_dev = collinear_deviation(&smoothed);
            // Relative threshold: a long segment (stem, crossbar) tolerates
            // more absolute deviation than a short one. Minimum 3 pixels.
            let p0 = smoothed[0];
            let pn = smoothed[smoothed.len() - 1];
            let seg_len = ((pn.0 - p0.0).powi(2) + (pn.1 - p0.1).powi(2)).sqrt();
            let line_tol = (seg_len * 0.02).max(3.0);
            if max_dev <= line_tol {
                // Straight section (stem, crossbar, etc.)
                let p = v[end];
                path.line_to(Point::new(p.0, p.1));
            } else {
                let fitted = fit_single_cubic(&smoothed);
                for el in fitted.elements().iter().skip(1) {
                    path.push(*el);
                }
            }
        }
    }

    path.push(PathEl::ClosePath);

    // Snap nearly-H/V lines to exact H/V, and merge consecutive
    // collinear lines into single lines.
    snap_and_merge_lines(&mut path);

    path
}

// ── Curve fitting ────────────────────────────────────────

/// Fit a smooth closed curve (no corners or extrema at all).
fn fit_smooth_closed(vertices: &[(f64, f64)], accuracy: f64) -> BezPath {
    let polyline = points_to_path(vertices, true);
    fit_to_bezpath_opt(
        &SimplifyBezPath::new(polyline.elements().iter().copied()),
        accuracy,
    )
}

/// Fit a single cubic through a segment between two split points.
///
/// Each section (extrema-to-extrema or extrema-to-corner) is a
/// quarter-curve that should always be exactly one cubic Bezier.
/// Handle directions come from the polygon tangent at each endpoint;
/// handle lengths are optimized to minimize distance to the polyline.
fn fit_single_cubic(points: &[(f64, f64)]) -> BezPath {
    let n = points.len();
    let p0 = Point::new(points[0].0, points[0].1);
    let p3 = Point::new(points[n - 1].0, points[n - 1].1);

    if n <= 2 {
        let mut path = BezPath::new();
        path.move_to(p0);
        path.line_to(p3);
        return path;
    }

    // Tangent at start: direction from first to second point.
    let t0 = Point::new(
        points[1].0 - points[0].0,
        points[1].1 - points[0].1,
    );
    // Tangent at end: direction from second-to-last to last point.
    let t1 = Point::new(
        points[n - 1].0 - points[n - 2].0,
        points[n - 1].1 - points[n - 2].1,
    );

    let t0_len = (t0.x * t0.x + t0.y * t0.y).sqrt();
    let t1_len = (t1.x * t1.x + t1.y * t1.y).sqrt();

    if t0_len < 1e-10 || t1_len < 1e-10 {
        let mut path = BezPath::new();
        path.move_to(p0);
        path.line_to(p3);
        return path;
    }

    // Unit tangents, snapped to H/V when close.
    // Per type-design convention (ohnotype.co/blog/drawing-vectors),
    // handles at extrema should be H/V. Snap within ~30° of an axis.
    let u0 = snap_tangent_hv(Point::new(t0.x / t0_len, t0.y / t0_len));
    let u1 = snap_tangent_hv(Point::new(t1.x / t1_len, t1.y / t1_len));

    // Optimize handle lengths to best fit the interior polyline points.
    // Try a range of scales and pick the one with least max deviation.
    let chord = ((p3.x - p0.x).powi(2) + (p3.y - p0.y).powi(2)).sqrt();
    let mut best_a = chord / 3.0;
    let mut best_b = chord / 3.0;
    let mut best_err = f64::MAX;

    for &scale in &[0.2, 0.25, 0.3, 0.35, 0.4, 0.45, 0.5] {
        let a = chord * scale;
        let b = chord * scale;
        let p1 = Point::new(p0.x + u0.x * a, p0.y + u0.y * a);
        let p2 = Point::new(p3.x - u1.x * b, p3.y - u1.y * b);
        let err = max_polyline_deviation(p0, p1, p2, p3, points);
        if err < best_err {
            best_err = err;
            best_a = a;
            best_b = b;
        }
    }

    let cp1 = Point::new(p0.x + u0.x * best_a, p0.y + u0.y * best_a);
    let cp2 = Point::new(p3.x - u1.x * best_b, p3.y - u1.y * best_b);

    let mut path = BezPath::new();
    path.move_to(p0);
    path.curve_to(cp1, cp2, p3);
    path
}

/// Maximum deviation between a cubic and a polyline, sampled at polyline points.
fn max_polyline_deviation(
    p0: Point, p1: Point, p2: Point, p3: Point,
    polyline: &[(f64, f64)],
) -> f64 {
    let n = polyline.len();
    if n <= 2 {
        return 0.0;
    }
    let mut max_d = 0.0_f64;
    for i in 1..n - 1 {
        // Approximate parameter t by arc-length fraction.
        let t = i as f64 / (n - 1) as f64;
        let mt = 1.0 - t;
        // De Casteljau evaluation.
        let bx = mt * mt * mt * p0.x
            + 3.0 * mt * mt * t * p1.x
            + 3.0 * mt * t * t * p2.x
            + t * t * t * p3.x;
        let by = mt * mt * mt * p0.y
            + 3.0 * mt * mt * t * p1.y
            + 3.0 * mt * t * t * p2.y
            + t * t * t * p3.y;
        let dx = bx - polyline[i].0;
        let dy = by - polyline[i].1;
        max_d = max_d.max((dx * dx + dy * dy).sqrt());
    }
    max_d
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

/// Laplacian smoothing: each interior point is replaced by the average
/// of itself and its two neighbors for `iterations` passes.
///
/// - `closed = false`: endpoints are kept fixed (open segment between corners).
/// - `closed = true`: all points are averaged cyclically (no corners).
fn laplacian_smooth(points: &[(f64, f64)], iterations: usize, closed: bool) -> Vec<(f64, f64)> {
    if iterations == 0 || points.len() < 3 {
        return points.to_vec();
    }
    let mut pts = points.to_vec();
    let n = pts.len();
    let start = if closed { 0 } else { 1 };
    let end = if closed { n } else { n - 1 };
    for _ in 0..iterations {
        let prev = pts.clone();
        for i in start..end {
            let p = if i == 0 { n - 1 } else { i - 1 };
            let nx = (i + 1) % n;
            pts[i].0 = (prev[p].0 + prev[i].0 + prev[nx].0) / 3.0;
            pts[i].1 = (prev[p].1 + prev[i].1 + prev[nx].1) / 3.0;
        }
    }
    pts
}

// ── Helpers ──────────────────────────────────────────────

/// Find vertices at straight-to-curve transitions.
///
/// Smooths the alpha values over a 5-vertex window first, then finds
/// where the smoothed alpha crosses the threshold. This filters out
/// single-vertex noise and only fires at real structural boundaries
/// (where stems/crossbars meet curves).
fn find_curvature_transitions(alphas: &[f64], threshold: f64) -> Vec<usize> {
    let m = alphas.len();
    if m < 5 {
        return vec![];
    }

    // Smooth alphas with a 5-vertex cyclic moving average.
    let smoothed: Vec<f64> = (0..m)
        .map(|j| {
            let mut sum = 0.0;
            for k in 0..5 {
                sum += alphas[(j + m - 2 + k) % m];
            }
            sum / 5.0
        })
        .collect();

    let mut transitions = Vec::new();
    for j in 0..m {
        let prev = if j == 0 { m - 1 } else { j - 1 };
        let prev_straight = smoothed[prev] < threshold;
        let curr_straight = smoothed[j] < threshold;
        if prev_straight != curr_straight {
            transitions.push(j);
        }
    }
    transitions
}

/// Snap a unit tangent vector to exact H or V if it's within ~30°.
///
/// tan(30°) ≈ 0.577. If the minor component is less than 0.577 of the
/// major component, the tangent is within 30° of an axis — snap it.
/// This ensures handles point H/V at extrema and curve-to-line junctions,
/// matching type-design convention.
fn snap_tangent_hv(u: Point) -> Point {
    let ax = u.x.abs();
    let ay = u.y.abs();
    if ax < 1e-10 && ay < 1e-10 {
        return u;
    }
    // tan(30°) ≈ 0.577
    if ax > ay && ay / ax < 0.577 {
        // Nearly horizontal: snap to pure horizontal.
        Point::new(u.x.signum(), 0.0)
    } else if ay > ax && ax / ay < 0.577 {
        // Nearly vertical: snap to pure vertical.
        Point::new(0.0, u.y.signum())
    } else {
        u // Genuinely diagonal — keep as-is (corner/inflection).
    }
}

/// Snap nearly-H/V lines to exact H/V and merge consecutive collinear lines.
///
/// For each LineTo, if the x-delta is much smaller than the y-delta,
/// it's a vertical line — snap x to match. Vice versa for horizontal.
/// Then merge consecutive lines going the same direction into one.
fn snap_and_merge_lines(path: &mut BezPath) {
    let els = path.elements().to_vec();
    let mut out = BezPath::new();
    // HV snap ratio: if one axis delta is < 25% of the other, snap it.
    // Catches stems and crossbars that are clearly H/V in the source
    // image but zigzag slightly in the polygon approximation.
    let hv_ratio = 0.25;

    // First pass: snap nearly-H/V lines.
    let mut snapped: Vec<PathEl> = Vec::new();
    let mut cursor = Point::new(0.0, 0.0);
    for &el in &els {
        match el {
            PathEl::MoveTo(p) => {
                cursor = p;
                snapped.push(el);
            }
            PathEl::LineTo(p) => {
                let dx = (p.x - cursor.x).abs();
                let dy = (p.y - cursor.y).abs();
                let snapped_p = if dx > 0.0 && dy > 0.0 {
                    if dx / dy < hv_ratio {
                        // Nearly vertical: snap x to cursor x.
                        Point::new(cursor.x, p.y)
                    } else if dy / dx < hv_ratio {
                        // Nearly horizontal: snap y to cursor y.
                        Point::new(p.x, cursor.y)
                    } else {
                        p
                    }
                } else {
                    p
                };
                cursor = snapped_p;
                snapped.push(PathEl::LineTo(snapped_p));
            }
            PathEl::CurveTo(c1, c2, p) => {
                cursor = p;
                snapped.push(PathEl::CurveTo(c1, c2, p));
            }
            other => snapped.push(other),
        }
    }

    // Second pass: merge consecutive collinear lines.
    let mut i = 0;
    while i < snapped.len() {
        match snapped[i] {
            PathEl::LineTo(p1) => {
                // Look ahead for more LineTos in the same direction.
                let mut end = p1;
                let prev = cursor_before(&snapped, i);
                let is_h = (p1.y - prev.y).abs() < 1e-6;
                let is_v = (p1.x - prev.x).abs() < 1e-6;
                if is_h || is_v {
                    while i + 1 < snapped.len() {
                        if let PathEl::LineTo(p2) = snapped[i + 1] {
                            let same_dir = if is_h {
                                (p2.y - end.y).abs() < 1e-6
                            } else {
                                (p2.x - end.x).abs() < 1e-6
                            };
                            if same_dir {
                                end = p2;
                                i += 1;
                            } else {
                                break;
                            }
                        } else {
                            break;
                        }
                    }
                }
                out.push(PathEl::LineTo(end));
                cursor = end;
            }
            PathEl::MoveTo(p) => {
                out.push(PathEl::MoveTo(p));
                cursor = p;
            }
            PathEl::CurveTo(c1, c2, p) => {
                out.push(PathEl::CurveTo(c1, c2, p));
                cursor = p;
            }
            other => out.push(other),
        }
        i += 1;
    }

    *path = out;
}

/// Get the cursor position before element at index `i`.
fn cursor_before(els: &[PathEl], i: usize) -> Point {
    for j in (0..i).rev() {
        match els[j] {
            PathEl::MoveTo(p) | PathEl::LineTo(p) | PathEl::CurveTo(_, _, p) => return p,
            _ => {}
        }
    }
    Point::new(0.0, 0.0)
}

/// Merge non-corner split-point indices that are within `min_gap` of each other.
/// Corners are always kept (they represent real structural features).
/// Non-corner transitions too close to a kept point are dropped.
fn merge_nearby_preserve_corners(
    sorted: Vec<usize>,
    min_gap: usize,
    total: usize,
    corners: &std::collections::HashSet<usize>,
) -> Vec<usize> {
    if sorted.len() <= 1 {
        return sorted;
    }
    let mut result = vec![sorted[0]];
    for &idx in &sorted[1..] {
        let gap = idx - result.last().unwrap();
        if corners.contains(&idx) {
            // Always keep corners, even if close to previous point.
            result.push(idx);
        } else if gap > min_gap {
            result.push(idx);
        }
        // else: non-corner too close to previous — drop it.
    }
    // Check wrap-around: only drop last if it's NOT a corner.
    if result.len() > 1 {
        let last = *result.last().unwrap();
        let first = result[0];
        if total - last + first <= min_gap && !corners.contains(&last) {
            result.pop();
        }
    }
    result
}

/// Maximum deviation of interior points from the line connecting
/// first and last point. Returns 0.0 for ≤2 points.
/// Used to detect straight sections (stems, crossbars).
fn collinear_deviation(points: &[(f64, f64)]) -> f64 {
    let n = points.len();
    if n <= 2 {
        return 0.0;
    }
    let (ax, ay) = points[0];
    let (bx, by) = points[n - 1];
    let dx = bx - ax;
    let dy = by - ay;
    let len = (dx * dx + dy * dy).sqrt();
    if len < 1e-10 {
        return points.iter()
            .map(|&(px, py)| ((px - ax).powi(2) + (py - ay).powi(2)).sqrt())
            .fold(0.0_f64, f64::max);
    }
    let mut max_d = 0.0_f64;
    for &(px, py) in &points[1..n - 1] {
        let cross = ((px - ax) * dy - (py - ay) * dx).abs();
        max_d = max_d.max(cross / len);
    }
    max_d
}

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
/// Measures how far vertex j deviates from the line i→k, normalized by
/// the cardinal-snapped perpendicular distance. Higher alpha = smoother
/// (more "round"), lower alpha = sharper corner.
///
/// The raw alpha ranges [0, 1). Dividing by 0.75 rescales so that the
/// default `alphamax = 1.0` corresponds to a 3/4-pixel deviation
/// threshold — a good balance between preserving intentional corners
/// and smoothing pixel staircase artifacts.
fn compute_alpha(vi: (f64, f64), vj: (f64, f64), vk: (f64, f64)) -> f64 {
    // Cross product of (j-i, k-i) = twice signed area of triangle ijk.
    let dpara = (vj.0 - vi.0) * (vk.1 - vi.1) - (vj.1 - vi.1) * (vk.0 - vi.0);

    // Denominator: perpendicular distance using cardinal-snapped direction.
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
