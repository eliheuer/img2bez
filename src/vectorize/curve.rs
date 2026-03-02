//! Corner and extrema detection with minimal Bezier curve fitting from an optimal polygon.
//!
//! Splits the polygon at both corners (via alpha parameter) and extrema
//! (where x or y changes direction), then fits minimal cubic Beziers
//! through each short section. Because sections run extrema-to-extrema,
//! handles naturally align H/V and point counts stay minimal.

use kurbo::{
    fit_to_bezpath_opt, simplify::SimplifyBezPath, BezPath, PathEl, Point,
};

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
    let transitions = find_curvature_transitions(&alphas, 0.37);

    // Build split points: corners + ALL transitions.
    // Corners and transitions serve different structural purposes:
    // corners mark sharp angular changes, transitions mark where
    // straight sections meet curved sections. Both are needed even
    // when they're near each other (e.g., a chamfered corner adjacent
    // to a stem/curve boundary).
    let corner_set: std::collections::HashSet<usize> = corners.iter().copied().collect();
    let mut base_splits: Vec<usize> = corners.iter().copied()
        .chain(transitions.iter().copied())
        .collect();
    base_splits.sort_unstable();
    base_splits.dedup();
    // Merge only non-corner transitions that are at the same vertex.
    base_splits = merge_nearby_preserve_corners(base_splits, 1, m, &corner_set);

    let mut split_points = if base_splits.is_empty() {
        // No corners or transitions: use global bounding-box extrema.
        find_contour_extrema(v)
    } else {
        // Per-segment: find extrema only in curved sections (not straight).
        let mut pts = base_splits.clone();
        let mut extrema_set: std::collections::HashSet<usize> = std::collections::HashSet::new();
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
                    let seg_ext = find_segment_extrema(v, start, end, m);
                    for &e in &seg_ext {
                        extrema_set.insert(e);
                    }
                    pts.extend(seg_ext);
                }
            }
        }
        pts.sort_unstable();
        pts.dedup();

        // Remove transitions that create very short sections.
        // When a curvature transition fires close to a corner, the tiny
        // section between them adds an unnecessary on-curve point. The
        // corner already marks the structural boundary; the transition
        // is redundant and creates a no-man's-land mini-segment.
        // Only transitions (not corners or extrema) can be removed.
        let keep_set: std::collections::HashSet<usize> = corner_set.iter()
            .copied()
            .chain(extrema_set.iter().copied())
            .collect();
        let min_section_len: f64 = 90.0;
        loop {
            let ns = pts.len();
            if ns < 3 { break; }
            let mut to_remove = None;
            for si in 0..ns {
                let start = pts[si];
                let end = pts[(si + 1) % ns];
                let dist = ((v[end].0 - v[start].0).powi(2)
                    + (v[end].1 - v[start].1).powi(2))
                .sqrt();
                if dist < min_section_len {
                    // Prefer removing the endpoint that is a transition
                    if !keep_set.contains(&end) {
                        to_remove = Some((si + 1) % ns);
                        break;
                    } else if !keep_set.contains(&start) {
                        to_remove = Some(si);
                        break;
                    }
                }
            }
            match to_remove {
                Some(idx) => { pts.remove(idx); }
                None => break,
            }
        }

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

    // Two-pass fitting: first classify all sections, then fit curves
    // with tangent constraints from adjacent line sections.
    let mut num_splits = split_points.len();

    // Pass 1: classify each section as line or curve.
    struct SectionInfo {
        is_line: bool,
        smoothed: Vec<(f64, f64)>,
    }

    let mut sections: Vec<SectionInfo> = Vec::with_capacity(num_splits);
    let debug_splits = std::env::var("IMG2BEZ_DEBUG_SPLITS").is_ok();

    for si in 0..num_splits {
        let start = split_points[si];
        let end = split_points[(si + 1) % num_splits];
        let segment = extract_cyclic(v, start, end, m);

        if segment.len() <= 2 {
            sections.push(SectionInfo {
                is_line: true,
                smoothed: segment,
            });
        } else {
            let smoothed = laplacian_smooth(&segment, params.smooth_iterations, false);
            let max_dev = collinear_deviation(&smoothed);
            let p0 = smoothed[0];
            let pn = *smoothed.last().unwrap();
            let seg_len = ((pn.0 - p0.0).powi(2) + (pn.1 - p0.1).powi(2)).sqrt();
            // For short sections (< 100 polygon units), use a more
            // permissive absolute tolerance. Bitmap tracing creates small
            // curved artifacts at junctions that should be straight lines.
            let line_tol = if seg_len < 100.0 {
                (seg_len * 0.25).max(3.0)
            } else {
                (seg_len * 0.02).max(3.0)
            };
            let is_line = max_dev <= line_tol;
            if debug_splits {
                eprintln!("  Section[{}] ({:.1},{:.1})->({:.1},{:.1}) len={:.1} pts={} dev={:.2} tol={:.2} => {}",
                    si, p0.0, p0.1, pn.0, pn.1, seg_len, segment.len(), max_dev, line_tol,
                    if is_line { "LINE" } else { "CURVE" });
            }
            sections.push(SectionInfo {
                is_line,
                smoothed,
            });
        }
    }

    // Pass 1.5: Split long CURVE sections containing a straight run.
    //
    // With high alphamax (e.g. 1.0) or after short-section removal,
    // a single huge section may include both straight and curved portions
    // that a single cubic can't represent. Find the longest straight run
    // anywhere in the section (prefix, suffix, or internal) and split
    // it off as a LINE — potentially creating up to 3 sub-sections.
    {
        let min_section_chord = 500.0_f64;
        let min_straight_chord = 200.0_f64;
        let straight_tol = 3.0_f64;
        let min_curve_pts: usize = 4;

        let mut new_sections: Vec<SectionInfo> = Vec::new();
        let mut new_split_points: Vec<usize> = Vec::new();

        for si in 0..sections.len() {
            if sections[si].is_line {
                new_split_points.push(split_points[si]);
                new_sections.push(SectionInfo {
                    is_line: true,
                    smoothed: std::mem::take(&mut sections[si].smoothed),
                });
                continue;
            }

            let sm = &sections[si].smoothed;
            let n = sm.len();
            let p0 = sm[0];
            let pn = sm[n - 1];
            let chord = ((pn.0 - p0.0).powi(2) + (pn.1 - p0.1).powi(2)).sqrt();

            if chord < min_section_chord || n < min_curve_pts + 4 {
                new_split_points.push(split_points[si]);
                new_sections.push(SectionInfo {
                    is_line: false,
                    smoothed: std::mem::take(&mut sections[si].smoothed),
                });
                continue;
            }

            // Find the longest straight run anywhere in the section.
            let mut best_start = 0usize;
            let mut best_end = 0usize;
            let mut best_chord = 0.0_f64;

            for i in 0..n.saturating_sub(2) {
                let mut end = i + 2;
                while end < n && collinear_deviation(&sm[i..=end]) <= straight_tol {
                    end += 1;
                }
                let actual_end = end - 1;
                if actual_end > i + 1 {
                    let c = ((sm[actual_end].0 - sm[i].0).powi(2)
                           + (sm[actual_end].1 - sm[i].1).powi(2)).sqrt();
                    if c > best_chord {
                        best_chord = c;
                        best_start = i;
                        best_end = actual_end;
                    }
                }
            }

            if best_chord < min_straight_chord {
                new_split_points.push(split_points[si]);
                new_sections.push(SectionInfo {
                    is_line: false,
                    smoothed: std::mem::take(&mut sections[si].smoothed),
                });
                continue;
            }

            // Back off by 1 vertex from each end that borders a curve
            // so the curve fitter has room for a smooth transition.
            let adj_start = if best_start > 0 { best_start + 1 } else { best_start };
            let adj_end = if best_end < n - 1 { best_end - 1 } else { best_end };

            // Re-check chord after back-off
            let adj_chord = ((sm[adj_end].0 - sm[adj_start].0).powi(2)
                           + (sm[adj_end].1 - sm[adj_start].1).powi(2)).sqrt();
            if adj_chord < min_straight_chord {
                new_split_points.push(split_points[si]);
                new_sections.push(SectionInfo {
                    is_line: false,
                    smoothed: std::mem::take(&mut sections[si].smoothed),
                });
                continue;
            }

            let poly_start = split_points[si];
            let old_smoothed = std::mem::take(&mut sections[si].smoothed);

            if debug_splits {
                eprintln!("  Pass 1.5: Section[{}] straight run at smoothed[{}..{}], chord={:.1}",
                    si, adj_start, adj_end, adj_chord);
            }

            if adj_start <= 1 {
                // Prefix: LINE then CURVE
                if n - adj_end >= min_curve_pts {
                    new_split_points.push(poly_start);
                    new_sections.push(SectionInfo {
                        is_line: true,
                        smoothed: old_smoothed[0..=adj_end].to_vec(),
                    });
                    new_split_points.push((poly_start + adj_end) % m);
                    new_sections.push(SectionInfo {
                        is_line: false,
                        smoothed: old_smoothed[adj_end..].to_vec(),
                    });
                } else {
                    new_split_points.push(poly_start);
                    new_sections.push(SectionInfo { is_line: false, smoothed: old_smoothed });
                }
            } else if adj_end >= n - 2 {
                // Suffix: CURVE then LINE
                if adj_start + 1 >= min_curve_pts {
                    new_split_points.push(poly_start);
                    new_sections.push(SectionInfo {
                        is_line: false,
                        smoothed: old_smoothed[0..=adj_start].to_vec(),
                    });
                    new_split_points.push((poly_start + adj_start) % m);
                    new_sections.push(SectionInfo {
                        is_line: true,
                        smoothed: old_smoothed[adj_start..].to_vec(),
                    });
                } else {
                    new_split_points.push(poly_start);
                    new_sections.push(SectionInfo { is_line: false, smoothed: old_smoothed });
                }
            } else {
                // Internal: CURVE + LINE + CURVE
                let first_ok = adj_start + 1 >= min_curve_pts;
                let last_ok = n - adj_end >= min_curve_pts;
                if first_ok && last_ok {
                    new_split_points.push(poly_start);
                    new_sections.push(SectionInfo {
                        is_line: false,
                        smoothed: old_smoothed[0..=adj_start].to_vec(),
                    });
                    new_split_points.push((poly_start + adj_start) % m);
                    new_sections.push(SectionInfo {
                        is_line: true,
                        smoothed: old_smoothed[adj_start..=adj_end].to_vec(),
                    });
                    new_split_points.push((poly_start + adj_end) % m);
                    new_sections.push(SectionInfo {
                        is_line: false,
                        smoothed: old_smoothed[adj_end..].to_vec(),
                    });
                } else {
                    new_split_points.push(poly_start);
                    new_sections.push(SectionInfo { is_line: false, smoothed: old_smoothed });
                }
            }
        }

        sections = new_sections;
        split_points = new_split_points;
        num_splits = split_points.len();
    }

    // Pass 2: emit path elements.
    let mut path = BezPath::new();
    let first = v[split_points[0]];
    path.move_to(Point::new(first.0, first.1));

    for si in 0..num_splits {
        let end_idx = split_points[(si + 1) % num_splits];

        if sections[si].is_line {
            let p = v[end_idx];
            path.line_to(Point::new(p.0, p.1));
        } else {
            let cubic = fit_single_cubic(&sections[si].smoothed, None, None);
            for el in cubic.elements().iter().skip(1) {
                path.push(*el);
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
///
/// Optional tangent overrides (`t0_override`, `t1_override`) constrain
/// the start/end handle direction to match an adjacent line section,
/// ensuring G1 continuity at curve-to-line junctions.
fn fit_single_cubic(
    points: &[(f64, f64)],
    t0_override: Option<Point>,
    t1_override: Option<Point>,
) -> BezPath {
    let n = points.len();
    let p0 = Point::new(points[0].0, points[0].1);
    let p3 = Point::new(points[n - 1].0, points[n - 1].1);

    if n <= 2 {
        let mut path = BezPath::new();
        path.move_to(p0);
        path.line_to(p3);
        return path;
    }

    // Tangent at start/end: use override from adjacent line if available,
    // otherwise estimate from polygon points using a wider window (up to
    // 1/3 of the section) for stable direction estimation.
    let u0 = if let Some(dir) = t0_override {
        snap_tangent_hv(dir)
    } else {
        let k = ((n / 3).max(1)).min(5);
        let t0 = Point::new(
            points[k].0 - points[0].0,
            points[k].1 - points[0].1,
        );
        let t0_len = (t0.x * t0.x + t0.y * t0.y).sqrt();
        if t0_len < 1e-10 {
            let mut path = BezPath::new();
            path.move_to(p0);
            path.line_to(p3);
            return path;
        }
        snap_tangent_hv(Point::new(t0.x / t0_len, t0.y / t0_len))
    };
    let u1 = if let Some(dir) = t1_override {
        snap_tangent_hv(dir)
    } else {
        let k = ((n / 3).max(1)).min(5);
        let t1 = Point::new(
            points[n - 1].0 - points[n - 1 - k].0,
            points[n - 1].1 - points[n - 1 - k].1,
        );
        let t1_len = (t1.x * t1.x + t1.y * t1.y).sqrt();
        if t1_len < 1e-10 {
            let mut path = BezPath::new();
            path.move_to(p0);
            path.line_to(p3);
            return path;
        }
        snap_tangent_hv(Point::new(t1.x / t1_len, t1.y / t1_len))
    };

    // Optimize handle lengths to best fit the interior polyline points.
    // Grid search over asymmetric (sa, sb) scale factors, then multi-pass
    // refinement. Minimizes max nearest-point distance (Hausdorff).
    // Hard constraint: handles must be within MAX_HANDLE_RATIO of each other.
    let chord = ((p3.x - p0.x).powi(2) + (p3.y - p0.y).powi(2)).sqrt();
    let mut best_sa = 0.33_f64;
    let mut best_sb = 0.33_f64;
    let mut best_err = f64::MAX;

    const MAX_HANDLE_RATIO: f64 = 1.5;
    let ratio_ok = |sa: f64, sb: f64| -> bool {
        sa.max(sb) / sa.min(sb) <= MAX_HANDLE_RATIO
    };

    // Coarse grid: 9 values from 0.10 to 0.70
    let scales: &[f64] = &[0.10, 0.175, 0.25, 0.325, 0.40, 0.475, 0.55, 0.625, 0.70];
    for &sa in scales {
        for &sb in scales {
            if !ratio_ok(sa, sb) { continue; }
            let a = chord * sa;
            let b = chord * sb;
            let p1 = Point::new(p0.x + u0.x * a, p0.y + u0.y * a);
            let p2 = Point::new(p3.x - u1.x * b, p3.y - u1.y * b);
            let err = max_polyline_deviation(p0, p1, p2, p3, points);
            if err < best_err {
                best_err = err;
                best_sa = sa;
                best_sb = sb;
            }
        }
    }

    // Three-pass refinement: medium → fine → ultra-fine
    for &(step, range) in &[(0.008, 0.05), (0.002, 0.012), (0.0005, 0.003)] {
        let sa_lo = (best_sa - range).max(0.05);
        let sa_hi = (best_sa + range).min(0.80);
        let sb_lo = (best_sb - range).max(0.05);
        let sb_hi = (best_sb + range).min(0.80);
        let mut sa = sa_lo;
        while sa <= sa_hi + 1e-9 {
            let mut sb = sb_lo;
            while sb <= sb_hi + 1e-9 {
                if ratio_ok(sa, sb) {
                    let a = chord * sa;
                    let b = chord * sb;
                    let p1 = Point::new(p0.x + u0.x * a, p0.y + u0.y * a);
                    let p2 = Point::new(p3.x - u1.x * b, p3.y - u1.y * b);
                    let err = max_polyline_deviation(p0, p1, p2, p3, points);
                    if err < best_err {
                        best_err = err;
                        best_sa = sa;
                        best_sb = sb;
                    }
                }
                sb += step;
            }
            sa += step;
        }
    }

    let mut cp1 = Point::new(p0.x + u0.x * chord * best_sa, p0.y + u0.y * chord * best_sa);
    let mut cp2 = Point::new(p3.x - u1.x * chord * best_sb, p3.y - u1.y * chord * best_sb);

    // Magic triangle check: handles must not cross each other's extension
    // lines. The triangle is formed by p0, p3, and the intersection of
    // the two handle directions. Each handle must stay on its side.
    // Clamp handle lengths so they don't extend past the intersection point.
    clamp_to_magic_triangle(p0, &mut cp1, &mut cp2, p3);

    let mut path = BezPath::new();
    path.move_to(p0);
    path.curve_to(cp1, cp2, p3);
    path
}

/// Clamp control points so they stay within the magic triangle.
///
/// The magic triangle is formed by the two on-curve points and the
/// intersection of the two handle direction lines. Each handle must
/// not extend past this intersection point (measured along its own
/// direction). If a handle overshoots, it is shortened to 95% of the
/// distance to the intersection.
fn clamp_to_magic_triangle(p0: Point, cp1: &mut Point, cp2: &mut Point, p3: Point) {
    let u0 = Point::new(cp1.x - p0.x, cp1.y - p0.y);
    let u1 = Point::new(cp2.x - p3.x, cp2.y - p3.y);

    let len0 = (u0.x * u0.x + u0.y * u0.y).sqrt();
    let len1 = (u1.x * u1.x + u1.y * u1.y).sqrt();
    if len0 < 1e-10 || len1 < 1e-10 {
        return;
    }

    // Find intersection of the two handle rays using parametric form:
    // p0 + t * u0 = p3 + s * u1
    // Solve for t: cross(u0, u1) * t = cross(p3-p0, u1)
    let cross_uv = u0.x * u1.y - u0.y * u1.x;
    if cross_uv.abs() < 1e-10 {
        return; // parallel handles, no triangle
    }

    let dp = Point::new(p3.x - p0.x, p3.y - p0.y);
    let t = (dp.x * u1.y - dp.y * u1.x) / cross_uv;
    let s = (dp.x * u0.y - dp.y * u0.x) / cross_uv;

    // t is the parameter for ray from p0 along u0.
    // s is the parameter for ray from p3 along u1.
    // Both must be positive for a valid triangle.
    // Handle extends from 0 to 1 in its parameter (len0/len1 normalized).
    // If t < 1 (handle overshoots intersection), clamp to 95% of t.
    if t > 0.0 {
        let max_len = t * len0 * 0.95;
        if len0 > max_len {
            let scale = max_len / len0;
            *cp1 = Point::new(p0.x + u0.x * scale, p0.y + u0.y * scale);
        }
    }
    if s > 0.0 {
        let max_len = s * len1 * 0.95;
        if len1 > max_len {
            let scale = max_len / len1;
            *cp2 = Point::new(p3.x + u1.x * scale, p3.y + u1.y * scale);
        }
    }
}

/// Maximum geometric distance from interior polyline points to the cubic curve.
/// Uses nearest-point (one-sided Hausdorff) rather than parametric correspondence,
/// which gives more accurate error measurement for handle optimization.
fn max_polyline_deviation(
    p0: Point, p1: Point, p2: Point, p3: Point,
    polyline: &[(f64, f64)],
) -> f64 {
    let n = polyline.len();
    if n <= 2 {
        return 0.0;
    }

    // Sample the cubic densely.
    const NUM_SAMPLES: usize = 96;
    let mut samples = [(0.0_f64, 0.0_f64); NUM_SAMPLES + 1];
    for i in 0..=NUM_SAMPLES {
        let t = i as f64 / NUM_SAMPLES as f64;
        let mt = 1.0 - t;
        samples[i] = (
            mt * mt * mt * p0.x
                + 3.0 * mt * mt * t * p1.x
                + 3.0 * mt * t * t * p2.x
                + t * t * t * p3.x,
            mt * mt * mt * p0.y
                + 3.0 * mt * mt * t * p1.y
                + 3.0 * mt * t * t * p2.y
                + t * t * t * p3.y,
        );
    }

    // For each interior polyline point, find minimum distance to any curve sample.
    let mut max_d = 0.0_f64;
    for i in 1..n - 1 {
        let px = polyline[i].0;
        let py = polyline[i].1;
        let mut min_d_sq = f64::MAX;
        for &(cx, cy) in &samples {
            let d_sq = (cx - px) * (cx - px) + (cy - py) * (cy - py);
            if d_sq < min_d_sq {
                min_d_sq = d_sq;
            }
        }
        max_d = max_d.max(min_d_sq.sqrt());
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

/// Snap a unit tangent vector to exact H or V if it's within ~40°.
///
/// tan(40°) ≈ 0.839. If the minor component is less than 0.839 of the
/// major component, the tangent is within 40° of an axis — snap it.
/// This ensures handles point H/V at extrema and curve-to-line junctions,
/// matching type-design convention (ohnotype.co/blog/drawing-vectors).
/// 40° is aggressive but appropriate: in type design, nearly all handles
/// should be H/V; genuinely diagonal handles are rare (mainly inflections).
fn snap_tangent_hv(u: Point) -> Point {
    let ax = u.x.abs();
    let ay = u.y.abs();
    if ax < 1e-10 && ay < 1e-10 {
        return u;
    }
    // tan(40°) ≈ 0.839
    if ax > ay && ay / ax < 0.839 {
        // Nearly horizontal: snap to pure horizontal.
        Point::new(u.x.signum(), 0.0)
    } else if ay > ax && ax / ay < 0.839 {
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
    // Merges both H/V lines and nearly-collinear diagonal lines.
    let collinear_tol = 1.5; // max perpendicular deviation to merge
    let mut i = 0;
    while i < snapped.len() {
        match snapped[i] {
            PathEl::LineTo(p1) => {
                let start = cursor_before(&snapped, i);
                let mut end = p1;
                // Greedily extend: try to merge the next LineTo if
                // all intermediate points stay collinear with start→end.
                while i + 1 < snapped.len() {
                    if let PathEl::LineTo(p2) = snapped[i + 1] {
                        // Check if 'end' (current middle point) stays
                        // close to the line from 'start' to 'p2'.
                        let dev = point_line_dist(end, start, p2);
                        if dev < collinear_tol {
                            end = p2;
                            i += 1;
                        } else {
                            break;
                        }
                    } else {
                        break;
                    }
                }
                out.push(PathEl::LineTo(end));
            }
            PathEl::MoveTo(p) => {
                out.push(PathEl::MoveTo(p));
            }
            PathEl::CurveTo(c1, c2, p) => {
                out.push(PathEl::CurveTo(c1, c2, p));
            }
            other => out.push(other),
        }
        i += 1;
    }

    *path = out;
}

/// DEAD CODE — kept for reference but not called.
/// Extract near-straight H/V prefixes from cubic curves as line segments.
/// Disabled: splitting cubics degrades H/V handle alignment and IoU.
#[allow(dead_code)]
fn extract_hv_stems(path: &mut BezPath) {
    use std::f64::consts::{FRAC_PI_2, PI};
    let els = path.elements().to_vec();
    let mut out: Vec<PathEl> = Vec::new();
    let hv_threshold = 10.0_f64.to_radians();
    let min_line_len = 30.0;
    let max_deviation = 2.0; // max perpendicular distance from H/V line

    let mut cursor = Point::ZERO;
    let mut prev_is_hv_line = false;
    let mut prev_line_dir: Option<(f64, f64)> = None; // unit direction of prev line

    for &el in &els {
        match el {
            PathEl::MoveTo(p) => {
                cursor = p;
                prev_is_hv_line = false;
                prev_line_dir = None;
                out.push(el);
            }
            PathEl::LineTo(p) => {
                let dx = p.x - cursor.x;
                let dy = p.y - cursor.y;
                let len = (dx * dx + dy * dy).sqrt();
                let angle = dy.atan2(dx).abs();
                let is_hv = angle < hv_threshold
                    || (PI - angle) < hv_threshold
                    || (angle - FRAC_PI_2).abs() < hv_threshold;
                prev_is_hv_line = is_hv && len > 20.0;
                prev_line_dir = if len > 1e-10 { Some((dx/len, dy/len)) } else { None };
                cursor = p;
                out.push(el);
            }
            PathEl::CurveTo(c1, c2, p3) => {
                let p0 = cursor;

                // Check if first handle direction is H/V.
                let h_dx = c1.x - p0.x;
                let h_dy = c1.y - p0.y;
                let h_len = (h_dx * h_dx + h_dy * h_dy).sqrt();
                let h_angle = h_dy.atan2(h_dx).abs();
                let handle_is_hv = h_len > 1e-10
                    && (h_angle < hv_threshold
                        || (PI - h_angle) < hv_threshold
                        || (h_angle - FRAC_PI_2).abs() < hv_threshold);

                // Check if handle direction matches previous line direction.
                let dir_matches = if let Some((pdx, pdy)) = prev_line_dir {
                    if h_len > 1e-10 {
                        let dot = (h_dx/h_len)*pdx + (h_dy/h_len)*pdy;
                        dot.abs() > 0.94 // within ~20°
                    } else { false }
                } else { false };

                if prev_is_hv_line && handle_is_hv && dir_matches {
                    // Find the t value where the curve first deviates from
                    // the handle direction by more than max_deviation.
                    let h_unit = if h_len > 1e-10 {
                        (h_dx / h_len, h_dy / h_len)
                    } else { (0.0, 0.0) };

                    // Sample the cubic at fine t intervals.
                    let mut best_t = 0.0;
                    let steps = 100;
                    for i in 1..=steps {
                        let t = i as f64 / steps as f64;
                        let pt = eval_cubic(p0, c1, c2, p3, t);
                        // Perpendicular distance from the H/V line through p0.
                        let dx = pt.x - p0.x;
                        let dy = pt.y - p0.y;
                        let perp = (dx * h_unit.1 - dy * h_unit.0).abs();
                        if perp > max_deviation {
                            break;
                        }
                        // Also check the line length along the direction.
                        let along = dx * h_unit.0 + dy * h_unit.1;
                        if along > 0.0 {
                            best_t = t;
                        }
                    }

                    // Check if the extracted line is long enough.
                    if best_t > 0.01 {
                        let split_pt = eval_cubic(p0, c1, c2, p3, best_t);
                        let line_dx = split_pt.x - p0.x;
                        let line_dy = split_pt.y - p0.y;
                        let line_len = (line_dx*line_dx + line_dy*line_dy).sqrt();

                        if line_len >= min_line_len {
                            // de Casteljau split at best_t.
                            let (_, c2_right, c1_right, p3_right) =
                                de_casteljau_split(p0, c1, c2, p3, best_t);

                            // Snap the split point to H/V from p0.
                            let snapped_split = if (split_pt.x - p0.x).abs() < max_deviation {
                                Point::new(p0.x, split_pt.y) // vertical
                            } else if (split_pt.y - p0.y).abs() < max_deviation {
                                Point::new(split_pt.x, p0.y) // horizontal
                            } else {
                                split_pt
                            };

                            out.push(PathEl::LineTo(snapped_split));
                            out.push(PathEl::CurveTo(c1_right, c2_right, p3_right));
                            cursor = p3;
                            prev_is_hv_line = false;
                            prev_line_dir = None;
                            continue;
                        }
                    }
                }

                // No split — emit as-is.
                cursor = p3;
                prev_is_hv_line = false;
                prev_line_dir = None;
                out.push(el);
            }
            other => {
                prev_is_hv_line = false;
                prev_line_dir = None;
                out.push(other);
            }
        }
    }

    *path = BezPath::from_vec(out);
}

#[allow(dead_code)]
fn eval_cubic(p0: Point, p1: Point, p2: Point, p3: Point, t: f64) -> Point {
    let mt = 1.0 - t;
    let mt2 = mt * mt;
    let t2 = t * t;
    Point::new(
        mt2 * mt * p0.x + 3.0 * mt2 * t * p1.x + 3.0 * mt * t2 * p2.x + t2 * t * p3.x,
        mt2 * mt * p0.y + 3.0 * mt2 * t * p1.y + 3.0 * mt * t2 * p2.y + t2 * t * p3.y,
    )
}

#[allow(dead_code)]
fn de_casteljau_split(
    p0: Point, p1: Point, p2: Point, p3: Point, t: f64,
) -> (Point, Point, Point, Point) {
    // Level 1
    let q0 = Point::new(p0.x + t*(p1.x-p0.x), p0.y + t*(p1.y-p0.y));
    let q1 = Point::new(p1.x + t*(p2.x-p1.x), p1.y + t*(p2.y-p1.y));
    let q2 = Point::new(p2.x + t*(p3.x-p2.x), p2.y + t*(p3.y-p2.y));
    // Level 2
    let r0 = Point::new(q0.x + t*(q1.x-q0.x), q0.y + t*(q1.y-q0.y));
    let r1 = Point::new(q1.x + t*(q2.x-q1.x), q1.y + t*(q2.y-q1.y));
    // Right sub-curve: control points are (split_pt, r1, q2, p3)
    // where split_pt = eval_cubic(t)
    // Return: (split_pt, r1, q2, p3) but we reorganize for the caller
    // Caller needs: (split_pt, c2_right=r1, c1_right_unused, p3_right=p3)
    // Actually: right sub-curve is (split_pt, r1, q2, p3)
    // So CurveTo(r1, q2, p3) starting from split_pt
    let _split = Point::new(r0.x + t*(r1.x-r0.x), r0.y + t*(r1.y-r0.y));
    // Return (split_pt, c1_right, c2_right, p3)
    (_split, r1, q2, p3)
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

/// Perpendicular distance from point `p` to the line through `a` and `b`.
fn point_line_dist(p: Point, a: Point, b: Point) -> f64 {
    let dx = b.x - a.x;
    let dy = b.y - a.y;
    let len = (dx * dx + dy * dy).sqrt();
    if len < 1e-10 {
        return ((p.x - a.x).powi(2) + (p.y - a.y).powi(2)).sqrt();
    }
    ((p.x - a.x) * dy - (p.y - a.y) * dx).abs() / len
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
