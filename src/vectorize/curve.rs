//! Corner and extrema detection with minimal Bezier curve fitting from an optimal polygon.
//!
//! Splits the polygon at both corners (via alpha parameter) and extrema
//! (where x or y changes direction), then fits minimal cubic Beziers
//! through each short section. Because sections run extrema-to-extrema,
//! handles naturally align H/V and point counts stay minimal.

use kurbo::{fit_to_bezpath_opt, simplify::SimplifyBezPath, BezPath, PathEl, Point};

use super::polygon::Polygon;

// ── Named constants ──────────────────────────────────────

/// Smoothed-alpha threshold for detecting straight-to-curve transitions.
/// Below this value the polygon region is considered straight (a stem or
/// crossbar); above it the region is curved.
const CURVATURE_TRANSITION_THRESHOLD: f64 = 0.37;

/// Minimum chord length (in polygon-coordinate units) between adjacent
/// split points. Sections shorter than this are candidates for removal
/// to avoid creating tiny no-man's-land segments.
const MIN_SECTION_CHORD: f64 = 90.0;

/// Section chord length below which a more permissive straightness
/// tolerance is applied. Bitmap tracing creates small curved artifacts
/// at junctions that should be straight lines.
const SHORT_SECTION_THRESHOLD: f64 = 100.0;

/// Permissive straightness tolerance (as fraction of chord length) for
/// sections shorter than `SHORT_SECTION_THRESHOLD`.
const SHORT_SECTION_TOLERANCE: f64 = 0.25;

/// Minimum chord for a curve section to be eligible for Pass 1.5
/// (splitting long curves that contain a straight run).
const PASS15_MIN_SECTION_CHORD: f64 = 500.0;

/// Minimum chord length for a straight run extracted by Pass 1.5.
const PASS15_MIN_STRAIGHT_CHORD: f64 = 200.0;

/// Maximum perpendicular deviation (in polygon units) for a run of
/// points to be considered straight during Pass 1.5 extraction.
const PASS15_STRAIGHT_TOLERANCE: f64 = 3.0;

/// Minimum number of smoothed points for a curve sub-section created
/// by Pass 1.5. Fewer points than this can't produce a reasonable cubic.
const PASS15_MIN_CURVE_POINTS: usize = 4;

/// Maximum number of smoothed points before a sub-section is promoted
/// from curve to line in Pass 1.5. Avoids awkward short curves at
/// chamfer-to-stem transitions.
const PASS15_SHORT_SUBSECTION: usize = 7;

/// Maximum ratio between the two handle lengths of a fitted cubic.
/// Prevents lopsided curves where one handle dominates.
const MAX_HANDLE_RATIO: f64 = 1.5;

/// Number of samples along a cubic for Hausdorff error measurement.
/// Higher = more accurate error but slower grid search.
const CURVE_DEVIATION_SAMPLES: usize = 96;

/// H/V snap threshold for line segments: if one axis delta is less than
/// this fraction of the other, snap the minor axis to zero.
const HV_LINE_SNAP_RATIO: f64 = 0.25;

/// Maximum perpendicular deviation for merging consecutive collinear
/// line segments into a single line.
const COLLINEAR_MERGE_TOLERANCE: f64 = 1.5;

/// tan(40deg) — threshold for snapping tangent vectors to H/V. If the
/// minor component is less than this fraction of the major component,
/// the tangent is within 40deg of an axis and gets snapped.
const TAN_40_DEG: f64 = 0.839;

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
        if vertices[j].0 < vertices[min_x].0 {
            min_x = j;
        }
        if vertices[j].0 > vertices[max_x].0 {
            max_x = j;
        }
        if vertices[j].1 < vertices[min_y].1 {
            min_y = j;
        }
        if vertices[j].1 > vertices[max_y].1 {
            max_y = j;
        }
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
        if x < min_x {
            min_x = x;
            min_x_idx = Some(i);
        }
        if x > max_x {
            max_x = x;
            max_x_idx = Some(i);
        }
        if y < min_y {
            min_y = y;
            min_y_idx = Some(i);
        }
        if y > max_y {
            max_y = y;
            max_y_idx = Some(i);
        }
        i = (i + 1) % total;
    }

    let mut extrema = Vec::new();
    if let Some(i) = min_x_idx {
        if baseline_min_x - min_x >= min_protrusion {
            extrema.push(i);
        }
    }
    if let Some(i) = max_x_idx {
        if max_x - baseline_max_x >= min_protrusion {
            extrema.push(i);
        }
    }
    if let Some(i) = min_y_idx {
        if baseline_min_y - min_y >= min_protrusion {
            extrema.push(i);
        }
    }
    if let Some(i) = max_y_idx {
        if max_y - baseline_max_y >= min_protrusion {
            extrema.push(i);
        }
    }

    extrema.sort_unstable();
    extrema.dedup();
    extrema
}

/// Convert a polygon to a BezPath with minimal cubic Beziers.
///
/// ## Multi-pass pipeline
///
/// 1. **Corner + transition detection** — classify each vertex as corner
///    (alpha >= alphamax) or curvature transition (straight ↔ curved).
/// 2. **Extrema insertion** — find per-segment bounding-box extrema in
///    curved sections (or global extrema if no corners exist).
/// 3. **Short-section removal** — drop transitions that create tiny
///    sections between corners.
/// 4. **Pass 1: classify** — each section between split points is labeled
///    LINE or CURVE based on collinear deviation.
/// 5. **Pass 1.5: split long curves** — oversized curve sections containing
///    a straight run are split into up to 3 sub-sections (see
///    `split_long_curve_sections()`).
/// 6. **Pass 2: emit** — lines become `LineTo`, curves get a single cubic
///    fit via `fit_single_cubic()`, then H/V snap + collinear merge.
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
    let corners: Vec<usize> = (0..m).filter(|&j| alphas[j] >= params.alphamax).collect();

    // Find curvature transitions: where the polygon changes from
    // straight (alpha ≈ 0) to curved or vice versa. These are
    // the natural split points at stem/crossbar boundaries.
    let transitions = find_curvature_transitions(&alphas, CURVATURE_TRANSITION_THRESHOLD);

    // Build split points: corners + ALL transitions.
    // Corners and transitions serve different structural purposes:
    // corners mark sharp angular changes, transitions mark where
    // straight sections meet curved sections. Both are needed even
    // when they're near each other (e.g., a chamfered corner adjacent
    // to a stem/curve boundary).
    let corner_set: std::collections::HashSet<usize> = corners.iter().copied().collect();
    let mut base_splits: Vec<usize> = corners
        .iter()
        .copied()
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
        let keep_set: std::collections::HashSet<usize> = corner_set
            .iter()
            .copied()
            .chain(extrema_set.iter().copied())
            .collect();
        let min_section_len: f64 = MIN_SECTION_CHORD;
        loop {
            let ns = pts.len();
            if ns < 3 {
                break;
            }
            let mut to_remove = None;
            for si in 0..ns {
                let start = pts[si];
                let end = pts[(si + 1) % ns];
                let dist =
                    ((v[end].0 - v[start].0).powi(2) + (v[end].1 - v[start].1).powi(2)).sqrt();
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
                Some(idx) => {
                    pts.remove(idx);
                }
                None => break,
            }
        }

        pts
    };

    // DEBUG: print split analysis when IMG2BEZ_DEBUG_SPLITS is set
    if std::env::var("IMG2BEZ_DEBUG_SPLITS").is_ok() {
        eprintln!(
            "=== polygon_to_bezpath debug (contour with {} vertices) ===",
            m
        );
        eprintln!("  Total polygon vertices: {}", m);
        eprintln!("  Corners ({}):", corners.len());
        for &ci in &corners {
            eprintln!(
                "    corner[{}] = ({:.1}, {:.1})  alpha={:.3}",
                ci, v[ci].0, v[ci].1, alphas[ci]
            );
        }
        eprintln!("  Transitions ({}):", transitions.len());
        for &ti in &transitions {
            eprintln!(
                "    transition[{}] = ({:.1}, {:.1})  alpha={:.3}",
                ti, v[ti].0, v[ti].1, alphas[ti]
            );
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
            eprintln!(
                "    split[{}] = ({:.1}, {:.1})  [{}]",
                si, v[si].0, v[si].1, label
            );
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
    let mut sections: Vec<SectionInfo> = Vec::with_capacity(num_splits);
    let debug_splits = std::env::var("IMG2BEZ_DEBUG_SPLITS").is_ok();

    for si in 0..num_splits {
        let start = split_points[si];
        let end = split_points[(si + 1) % num_splits];
        let segment = extract_cyclic(v, start, end, m);

        if segment.len() <= 2 {
            sections.push(SectionInfo {
                is_line: true,
                raw: segment.clone(),
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
            let line_tol = if seg_len < SHORT_SECTION_THRESHOLD {
                (seg_len * SHORT_SECTION_TOLERANCE).max(3.0)
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
                raw: segment,
                smoothed,
            });
        }
    }

    // Pass 1.5: split long curve sections that contain a straight run.
    split_long_curve_sections(&mut sections, &mut split_points, m, debug_splits);
    num_splits = split_points.len();

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
            let cubic = fit_single_cubic(&sections[si].smoothed, &sections[si].raw, None, None);
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

// ── Section classification ────────────────────────────────

/// A contour section between two split points, classified as line or curve.
struct SectionInfo {
    /// Whether this section should be emitted as a straight line.
    is_line: bool,
    /// Raw (unsmoothed) polygon vertices for this section.
    raw: Vec<(f64, f64)>,
    /// Laplacian-smoothed polygon vertices (used for tangent estimation).
    smoothed: Vec<(f64, f64)>,
}

/// Split long CURVE sections that contain an internal straight run.
///
/// With high alphamax (e.g. 1.0) or after short-section removal, a single
/// huge section may include both straight and curved portions that a single
/// cubic can't represent. This pass finds the longest straight run anywhere
/// in each oversized curve section (prefix, suffix, or internal) and splits
/// it off as a LINE — potentially creating up to 3 sub-sections.
fn split_long_curve_sections(
    sections: &mut Vec<SectionInfo>,
    split_points: &mut Vec<usize>,
    m: usize,
    debug: bool,
) {
    let mut new_sections: Vec<SectionInfo> = Vec::new();
    let mut new_split_points: Vec<usize> = Vec::new();

    for si in 0..sections.len() {
        if sections[si].is_line {
            new_split_points.push(split_points[si]);
            new_sections.push(SectionInfo {
                is_line: true,
                raw: std::mem::take(&mut sections[si].raw),
                smoothed: std::mem::take(&mut sections[si].smoothed),
            });
            continue;
        }

        let sm = &sections[si].smoothed;
        let n = sm.len();
        let p0 = sm[0];
        let pn = sm[n - 1];
        let chord = ((pn.0 - p0.0).powi(2) + (pn.1 - p0.1).powi(2)).sqrt();

        if chord < PASS15_MIN_SECTION_CHORD || n < PASS15_MIN_CURVE_POINTS + 4 {
            new_split_points.push(split_points[si]);
            new_sections.push(SectionInfo {
                is_line: false,
                raw: std::mem::take(&mut sections[si].raw),
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
            while end < n && collinear_deviation(&sm[i..=end]) <= PASS15_STRAIGHT_TOLERANCE {
                end += 1;
            }
            let actual_end = end - 1;
            if actual_end > i + 1 {
                let c = ((sm[actual_end].0 - sm[i].0).powi(2)
                    + (sm[actual_end].1 - sm[i].1).powi(2))
                .sqrt();
                if c > best_chord {
                    best_chord = c;
                    best_start = i;
                    best_end = actual_end;
                }
            }
        }

        if best_chord < PASS15_MIN_STRAIGHT_CHORD {
            new_split_points.push(split_points[si]);
            new_sections.push(SectionInfo {
                is_line: false,
                raw: std::mem::take(&mut sections[si].raw),
                smoothed: std::mem::take(&mut sections[si].smoothed),
            });
            continue;
        }

        // Back off by 1 vertex from each end that borders a curve
        // so the curve fitter has room for a smooth transition.
        let adj_start = if best_start > 0 {
            best_start + 1
        } else {
            best_start
        };
        let adj_end = if best_end < n - 1 {
            best_end - 1
        } else {
            best_end
        };

        // Re-check chord after back-off.
        let adj_chord = ((sm[adj_end].0 - sm[adj_start].0).powi(2)
            + (sm[adj_end].1 - sm[adj_start].1).powi(2))
        .sqrt();
        if adj_chord < PASS15_MIN_STRAIGHT_CHORD {
            new_split_points.push(split_points[si]);
            new_sections.push(SectionInfo {
                is_line: false,
                raw: std::mem::take(&mut sections[si].raw),
                smoothed: std::mem::take(&mut sections[si].smoothed),
            });
            continue;
        }

        let poly_start = split_points[si];
        let old_smoothed = std::mem::take(&mut sections[si].smoothed);
        let old_raw = std::mem::take(&mut sections[si].raw);

        if debug {
            eprintln!(
                "  Pass 1.5: Section[{}] straight run at smoothed[{}..{}], chord={:.1}",
                si, adj_start, adj_end, adj_chord
            );
        }

        if adj_start <= 1 {
            // Prefix: LINE then CURVE.
            if n - adj_end >= PASS15_MIN_CURVE_POINTS {
                new_split_points.push(poly_start);
                new_sections.push(SectionInfo {
                    is_line: true,
                    raw: old_raw[0..=adj_end].to_vec(),
                    smoothed: old_smoothed[0..=adj_end].to_vec(),
                });
                new_split_points.push((poly_start + adj_end) % m);
                new_sections.push(SectionInfo {
                    is_line: false,
                    raw: old_raw[adj_end..].to_vec(),
                    smoothed: old_smoothed[adj_end..].to_vec(),
                });
            } else {
                new_split_points.push(poly_start);
                new_sections.push(SectionInfo {
                    is_line: false,
                    raw: old_raw,
                    smoothed: old_smoothed,
                });
            }
        } else if adj_end >= n - 2 {
            // Suffix: CURVE then LINE.
            if adj_start + 1 >= PASS15_MIN_CURVE_POINTS {
                new_split_points.push(poly_start);
                new_sections.push(SectionInfo {
                    is_line: false,
                    raw: old_raw[0..=adj_start].to_vec(),
                    smoothed: old_smoothed[0..=adj_start].to_vec(),
                });
                new_split_points.push((poly_start + adj_start) % m);
                new_sections.push(SectionInfo {
                    is_line: true,
                    raw: old_raw[adj_start..].to_vec(),
                    smoothed: old_smoothed[adj_start..].to_vec(),
                });
            } else {
                new_split_points.push(poly_start);
                new_sections.push(SectionInfo {
                    is_line: false,
                    raw: old_raw,
                    smoothed: old_smoothed,
                });
            }
        } else {
            // Internal: up to 3 sub-sections (LINE/CURVE + LINE + LINE/CURVE).
            // If the first or last portion is very short, emit it as a LINE
            // instead of a CURVE. This avoids awkward short curves at
            // chamfer-to-stem transitions.
            let first_is_short = adj_start < PASS15_SHORT_SUBSECTION;
            let last_is_short = n - adj_end <= PASS15_SHORT_SUBSECTION;
            let first_ok = first_is_short || adj_start + 1 >= PASS15_MIN_CURVE_POINTS;
            let last_ok = last_is_short || n - adj_end >= PASS15_MIN_CURVE_POINTS;
            if first_ok && last_ok {
                new_split_points.push(poly_start);
                new_sections.push(SectionInfo {
                    is_line: first_is_short,
                    raw: old_raw[0..=adj_start].to_vec(),
                    smoothed: old_smoothed[0..=adj_start].to_vec(),
                });
                new_split_points.push((poly_start + adj_start) % m);
                new_sections.push(SectionInfo {
                    is_line: true,
                    raw: old_raw[adj_start..=adj_end].to_vec(),
                    smoothed: old_smoothed[adj_start..=adj_end].to_vec(),
                });
                new_split_points.push((poly_start + adj_end) % m);
                new_sections.push(SectionInfo {
                    is_line: last_is_short,
                    raw: old_raw[adj_end..].to_vec(),
                    smoothed: old_smoothed[adj_end..].to_vec(),
                });
            } else {
                new_split_points.push(poly_start);
                new_sections.push(SectionInfo {
                    is_line: false,
                    raw: old_raw,
                    smoothed: old_smoothed,
                });
            }
        }
    }

    *sections = new_sections;
    *split_points = new_split_points;
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
/// Handle directions come from the smoothed polygon tangent at each
/// endpoint; handle lengths are optimized to minimize distance to the
/// raw (unsmoothed) polyline.
///
/// `points` — smoothed polygon segment (used for tangent estimation).
/// `raw_points` — raw polygon segment (used for error measurement in
///   the grid search). Because Laplacian smoothing shrinks curves toward
///   the chord, fitting against the raw polygon produces longer handles
///   that better match the source shape's actual curvature.
///
/// Optional tangent overrides (`t0_override`, `t1_override`) constrain
/// the start/end handle direction to match an adjacent line section,
/// ensuring G1 continuity at curve-to-line junctions.
fn fit_single_cubic(
    points: &[(f64, f64)],
    raw_points: &[(f64, f64)],
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
    // otherwise estimate from smoothed polygon points.
    let u0_raw = t0_override.or_else(|| estimate_tangent(points, true));
    let u1_raw = t1_override.or_else(|| estimate_tangent(points, false));

    let (u0_raw, u1_raw) = match (u0_raw, u1_raw) {
        (Some(a), Some(b)) => (a, b),
        _ => {
            let mut path = BezPath::new();
            path.move_to(p0);
            path.line_to(p3);
            return path;
        }
    };

    // Snap tangents to H/V if within threshold.
    let u0 = snap_tangent_hv(u0_raw);
    let u1 = snap_tangent_hv(u1_raw);

    // Optimize handle lengths to best fit the raw (unsmoothed) polyline.
    // Grid search over asymmetric (sa, sb) scale factors, then multi-pass
    // refinement. Minimizes max nearest-point distance (Hausdorff).
    // Hard constraint: handles must be within MAX_HANDLE_RATIO of each other.
    let chord = ((p3.x - p0.x).powi(2) + (p3.y - p0.y).powi(2)).sqrt();

    let ratio_ok = |sa: f64, sb: f64| -> bool { sa.max(sb) / sa.min(sb) <= MAX_HANDLE_RATIO };

    // Coarse grid: 9 values from 0.10 to 0.70
    let scales: &[f64] = &[0.10, 0.175, 0.25, 0.325, 0.40, 0.475, 0.55, 0.625, 0.70];
    let mut best_sa = 0.33_f64;
    let mut best_sb = 0.33_f64;
    let mut best_err = f64::MAX;

    for &sa in scales {
        for &sb in scales {
            if !ratio_ok(sa, sb) {
                continue;
            }
            let a = chord * sa;
            let b = chord * sb;
            let p1 = Point::new(p0.x + u0.x * a, p0.y + u0.y * a);
            let p2 = Point::new(p3.x - u1.x * b, p3.y - u1.y * b);
            let err = max_polyline_deviation(p0, p1, p2, p3, raw_points);
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
                    let err = max_polyline_deviation(p0, p1, p2, p3, raw_points);
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

    // Build final control points and apply magic triangle clamping.
    let a = chord * best_sa;
    let b = chord * best_sb;
    let mut cp1 = Point::new(p0.x + u0.x * a, p0.y + u0.y * a);
    let mut cp2 = Point::new(p3.x - u1.x * b, p3.y - u1.y * b);
    clamp_to_magic_triangle(p0, &mut cp1, &mut cp2, p3);

    if std::env::var("IMG2BEZ_DEBUG_FIT").is_ok() {
        let h1 = ((cp1.x - p0.x).powi(2) + (cp1.y - p0.y).powi(2)).sqrt();
        let h2 = ((cp2.x - p3.x).powi(2) + (cp2.y - p3.y).powi(2)).sqrt();
        let ratio = if h1.min(h2) > 1e-6 {
            h1.max(h2) / h1.min(h2)
        } else {
            0.0
        };
        eprintln!(
            "  fit_single_cubic: p0=({:.1},{:.1}) p3=({:.1},{:.1}) chord={:.1}",
            p0.x, p0.y, p3.x, p3.y, chord
        );
        eprintln!(
            "    u0=({:.3},{:.3}) u1=({:.3},{:.3})",
            u0.x, u0.y, u1.x, u1.y
        );
        eprintln!(
            "    sa={:.4} sb={:.4} h1={:.1} ({:.0}%) h2={:.1} ({:.0}%) ratio={:.2} err={:.2}",
            best_sa,
            best_sb,
            h1,
            h1 / chord * 100.0,
            h2,
            h2 / chord * 100.0,
            ratio,
            best_err
        );
    }

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
    p0: Point,
    p1: Point,
    p2: Point,
    p3: Point,
    polyline: &[(f64, f64)],
) -> f64 {
    let n = polyline.len();
    if n <= 2 {
        return 0.0;
    }

    // Sample the cubic densely.
    let mut samples = [(0.0_f64, 0.0_f64); CURVE_DEVIATION_SAMPLES + 1];
    for (i, sample) in samples.iter_mut().enumerate() {
        let t = i as f64 / CURVE_DEVIATION_SAMPLES as f64;
        let mt = 1.0 - t;
        *sample = (
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
    for &(px, py) in &polyline[1..n - 1] {
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

/// Estimate the unit tangent direction at one end of a smoothed polygon section.
///
/// Uses a window of up to 1/3 of the section (clamped to 5 vertices) for
/// stable direction estimation. Returns `None` if the section is degenerate
/// (all points coincide).
///
/// `from_start = true`: tangent at the first point (outgoing direction).
/// `from_start = false`: tangent at the last point (incoming direction).
fn estimate_tangent(points: &[(f64, f64)], from_start: bool) -> Option<Point> {
    let n = points.len();
    let k = (n / 3).clamp(1, 5);
    let (dx, dy) = if from_start {
        (points[k].0 - points[0].0, points[k].1 - points[0].1)
    } else {
        (
            points[n - 1].0 - points[n - 1 - k].0,
            points[n - 1].1 - points[n - 1 - k].1,
        )
    };
    let len = (dx * dx + dy * dy).sqrt();
    if len < 1e-10 {
        return None;
    }
    Some(Point::new(dx / len, dy / len))
}

/// Build candidate tangent directions for curve fitting.
///
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
    if ax > ay && ay / ax < TAN_40_DEG {
        // Nearly horizontal: snap to pure horizontal.
        Point::new(u.x.signum(), 0.0)
    } else if ay > ax && ax / ay < TAN_40_DEG {
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
    let hv_ratio = HV_LINE_SNAP_RATIO;

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
    let collinear_tol = COLLINEAR_MERGE_TOLERANCE;
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
        return points
            .iter()
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
