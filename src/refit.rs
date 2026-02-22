//! Extrema-anchored refitting with H/V handle constraints.
//!
//! After the initial curve fitting (which approximates the shape), this module
//! refits using only the essential anchor points (corners + extrema) with
//! type-design-correct handle directions:
//! - At Y-extrema (top/bottom of curves): handles are horizontal
//! - At X-extrema (left/right of curves): handles are vertical
//! - At corners: handles follow the curve's natural tangent

use kurbo::{BezPath, CubicBez, ParamCurve, PathEl, Point, Vec2};

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

/// Refit an open segment (between two corners) using extrema as anchor points.
///
/// The input `fitted` is a BezPath from the initial curve fitter (MoveTo + CurveTo(s)).
/// The output has on-curve points only at corners and extrema, with H/V handles
/// at extrema.
pub fn refit_segment(fitted: &BezPath, min_depth: f64) -> BezPath {
    let segments = collect_cubics(fitted);
    if segments.is_empty() {
        return fitted.clone();
    }

    let start = segments[0].p0;
    let end = segments.last().unwrap().p3;

    // Find extrema on the fitted curve
    let extrema = find_extrema(&segments, min_depth);

    if extrema.is_empty() {
        // No extrema: fit a single cubic from start to end using original tangents
        let d0 = (segments[0].p1 - segments[0].p0).normalize();
        let last = segments.last().unwrap();
        let d3 = (last.p3 - last.p2).normalize();
        let samples = sample_all(&segments, 50);
        let cubic = fit_constrained(start, d0, end, d3, &samples);
        let mut result = BezPath::new();
        result.move_to(start);
        result.push(PathEl::CurveTo(cubic.p1, cubic.p2, cubic.p3));
        return result;
    }

    // Build anchor list: [start_corner, ...extrema..., end_corner]
    let d_start = (segments[0].p1 - segments[0].p0).normalize();
    let last_seg = segments.last().unwrap();
    let d_end = (last_seg.p3 - last_seg.p2).normalize();

    let mut anchors: Vec<Anchor> = Vec::new();

    // Start corner (unconstrained tangent from fitted curve)
    anchors.push(Anchor {
        point: start,
        tangent_out: d_start,
        tangent_in: Vec2::ZERO, // not used for first anchor
    });

    // Extrema (H/V constrained)
    for ex in &extrema {
        let (t_out, t_in) = match ex.axis {
            Axis::Y => {
                // Y-extremum: tangent is horizontal
                let dir = Vec2::new(ex.tangent_sign, 0.0);
                (dir, dir)
            }
            Axis::X => {
                // X-extremum: tangent is vertical
                let dir = Vec2::new(0.0, ex.tangent_sign);
                (dir, dir)
            }
        };
        anchors.push(Anchor {
            point: ex.point,
            tangent_out: t_out,
            tangent_in: t_in,
        });
    }

    // End corner (unconstrained tangent from fitted curve)
    anchors.push(Anchor {
        point: end,
        tangent_out: Vec2::ZERO, // not used for last anchor
        tangent_in: d_end,
    });

    // Fit constrained cubics between consecutive anchors
    build_path_from_anchors(&anchors, &segments)
}

/// Refit a closed path (no corners, e.g., an O shape) using extrema as anchors.
pub fn refit_closed(fitted: &BezPath, min_depth: f64) -> BezPath {
    let segments = collect_cubics(fitted);
    if segments.is_empty() {
        return fitted.clone();
    }

    let extrema = find_extrema(&segments, min_depth);
    if extrema.len() < 2 {
        return fitted.clone(); // Not enough structure to refit
    }

    // Build anchor list from extrema only (all H/V constrained)
    let mut anchors: Vec<Anchor> = Vec::new();
    for ex in &extrema {
        let (t_out, t_in) = match ex.axis {
            Axis::Y => {
                let dir = Vec2::new(ex.tangent_sign, 0.0);
                (dir, dir)
            }
            Axis::X => {
                let dir = Vec2::new(0.0, ex.tangent_sign);
                (dir, dir)
            }
        };
        anchors.push(Anchor {
            point: ex.point,
            tangent_out: t_out,
            tangent_in: t_in,
        });
    }

    build_closed_path_from_anchors(&anchors, &segments)
}

// ---------------------------------------------------------------------------
// Internal types
// ---------------------------------------------------------------------------

#[derive(Debug, Clone, Copy)]
enum Axis {
    X, // X-extremum: handle should be vertical
    Y, // Y-extremum: handle should be horizontal
}

#[derive(Debug, Clone, Copy)]
struct ExtremumInfo {
    point: Point,
    axis: Axis,
    tangent_sign: f64, // +1 or -1 along the free axis
    seg_idx: usize,    // which cubic segment
    t: f64,            // t within that segment
}

struct Anchor {
    point: Point,
    tangent_out: Vec2, // outgoing tangent direction (toward next anchor)
    tangent_in: Vec2,  // incoming tangent direction (from previous anchor)
}

// ---------------------------------------------------------------------------
// Extrema finding
// ---------------------------------------------------------------------------

fn find_extrema(segments: &[CubicBez], min_depth: f64) -> Vec<ExtremumInfo> {
    let mut extrema = Vec::new();

    for (seg_idx, cubic) in segments.iter().enumerate() {
        for axis in 0..2 {
            let (a, b, c, d) = if axis == 0 {
                (cubic.p0.x, cubic.p1.x, cubic.p2.x, cubic.p3.x)
            } else {
                (cubic.p0.y, cubic.p1.y, cubic.p2.y, cubic.p3.y)
            };

            let v0 = if axis == 0 { cubic.p0.x } else { cubic.p0.y };
            let v3 = if axis == 0 { cubic.p3.x } else { cubic.p3.y };

            // Derivative coefficients: Da*t^2 + Db*t + Dc = 0
            let da = -3.0 * a + 9.0 * b - 9.0 * c + 3.0 * d;
            let db = 6.0 * a - 12.0 * b + 6.0 * c;
            let dc = -3.0 * a + 3.0 * b;

            let mut ts = Vec::new();
            if da.abs() < 1e-10 {
                if db.abs() > 1e-10 {
                    ts.push(-dc / db);
                }
            } else {
                let disc = db * db - 4.0 * da * dc;
                if disc >= 0.0 {
                    let sq = disc.sqrt();
                    ts.push((-db + sq) / (2.0 * da));
                    ts.push((-db - sq) / (2.0 * da));
                }
            }

            for t in ts {
                if t <= 0.02 || t >= 0.98 {
                    continue;
                }

                let pt = cubic.eval(t);
                let vt = if axis == 0 { pt.x } else { pt.y };
                let max_ep = v0.max(v3);
                let min_ep = v0.min(v3);

                // Skip shallow extrema
                let depth = if vt > max_ep {
                    vt - max_ep
                } else if vt < min_ep {
                    min_ep - vt
                } else {
                    0.0
                };
                if depth < min_depth {
                    continue;
                }

                // Determine tangent sign from curve direction at the extremum
                let dt = 0.001;
                let pt_before = cubic.eval((t - dt).max(0.0));
                let pt_after = cubic.eval((t + dt).min(1.0));

                let tangent_sign = if axis == 0 {
                    // X-extremum: tangent is vertical, check Y direction
                    if pt_after.y > pt_before.y {
                        1.0
                    } else {
                        -1.0
                    }
                } else {
                    // Y-extremum: tangent is horizontal, check X direction
                    if pt_after.x > pt_before.x {
                        1.0
                    } else {
                        -1.0
                    }
                };

                extrema.push(ExtremumInfo {
                    point: pt,
                    axis: if axis == 0 { Axis::X } else { Axis::Y },
                    tangent_sign,
                    seg_idx,
                    t,
                });
            }
        }
    }

    // Sort by position along the path
    extrema.sort_by(|a, b| {
        a.seg_idx
            .cmp(&b.seg_idx)
            .then(a.t.partial_cmp(&b.t).unwrap())
    });

    // Remove extrema that are very close together (within 10 font units)
    dedup_close_extrema(&mut extrema, 10.0);

    extrema
}

fn dedup_close_extrema(extrema: &mut Vec<ExtremumInfo>, min_dist: f64) {
    let mut i = 0;
    while i + 1 < extrema.len() {
        let d = ((extrema[i].point.x - extrema[i + 1].point.x).powi(2)
            + (extrema[i].point.y - extrema[i + 1].point.y).powi(2))
        .sqrt();
        if d < min_dist {
            // Keep the one with the larger depth (more significant extremum)
            extrema.remove(i + 1);
        } else {
            i += 1;
        }
    }
}

// ---------------------------------------------------------------------------
// Constrained cubic fitting
// ---------------------------------------------------------------------------

/// Fit a single cubic bezier between two points with constrained tangent directions.
///
/// P1 = p0 + alpha * d0 (handle from start)
/// P2 = p3 - beta * d3 (handle into end)
///
/// Optimizes alpha and beta to minimize max distance to sample points.
fn fit_constrained(
    p0: Point,
    d0: Vec2,
    p3: Point,
    d3: Vec2,
    samples: &[Point],
) -> CubicBez {
    let chord = ((p3.x - p0.x).powi(2) + (p3.y - p0.y).powi(2)).sqrt();
    if chord < 1.0 || samples.is_empty() {
        let alpha = chord / 3.0;
        let beta = chord / 3.0;
        return CubicBez::new(
            p0,
            pt_add(p0, d0, alpha),
            pt_sub(p3, d3, beta),
            p3,
        );
    }

    // Coarse grid search
    let steps = 25;
    let mut best_err = f64::MAX;
    let mut best_alpha = chord / 3.0;
    let mut best_beta = chord / 3.0;

    for ai in 1..=steps {
        let alpha = chord * ai as f64 / (steps + 1) as f64;
        for bi in 1..=steps {
            let beta = chord * bi as f64 / (steps + 1) as f64;

            let cubic = CubicBez::new(
                p0,
                pt_add(p0, d0, alpha),
                pt_sub(p3, d3, beta),
                p3,
            );

            let err = max_error(&cubic, samples);
            if err < best_err {
                best_err = err;
                best_alpha = alpha;
                best_beta = beta;
            }
        }
    }

    // Refine with finer grid around best values
    let step_size = chord / (steps + 1) as f64;
    let refine_steps = 10;
    let ra = best_alpha;
    let rb = best_beta;

    for ai in 0..=refine_steps {
        let alpha = (ra - step_size + 2.0 * step_size * ai as f64 / refine_steps as f64).max(1.0);
        for bi in 0..=refine_steps {
            let beta =
                (rb - step_size + 2.0 * step_size * bi as f64 / refine_steps as f64).max(1.0);

            let cubic = CubicBez::new(
                p0,
                pt_add(p0, d0, alpha),
                pt_sub(p3, d3, beta),
                p3,
            );

            let err = max_error(&cubic, samples);
            if err < best_err {
                best_err = err;
                best_alpha = alpha;
                best_beta = beta;
            }
        }
    }

    CubicBez::new(
        p0,
        pt_add(p0, d0, best_alpha),
        pt_sub(p3, d3, best_beta),
        p3,
    )
}

fn pt_add(p: Point, d: Vec2, scale: f64) -> Point {
    Point::new(p.x + d.x * scale, p.y + d.y * scale)
}

fn pt_sub(p: Point, d: Vec2, scale: f64) -> Point {
    Point::new(p.x - d.x * scale, p.y - d.y * scale)
}

/// Max distance from any sample point to the nearest point on the cubic.
fn max_error(cubic: &CubicBez, samples: &[Point]) -> f64 {
    let mut max_err: f64 = 0.0;
    for sample in samples {
        let mut min_dist = f64::MAX;
        for k in 0..=40 {
            let t = k as f64 / 40.0;
            let pt = cubic.eval(t);
            let dist = ((pt.x - sample.x).powi(2) + (pt.y - sample.y).powi(2)).sqrt();
            min_dist = min_dist.min(dist);
        }
        max_err = max_err.max(min_dist);
    }
    max_err
}

// ---------------------------------------------------------------------------
// Path assembly
// ---------------------------------------------------------------------------

fn build_path_from_anchors(anchors: &[Anchor], segments: &[CubicBez]) -> BezPath {
    let mut result = BezPath::new();
    if anchors.is_empty() {
        return result;
    }

    result.move_to(anchors[0].point);

    for i in 0..anchors.len() - 1 {
        let p0 = anchors[i].point;
        let p3 = anchors[i + 1].point;
        let d0 = anchors[i].tangent_out;
        let d3 = anchors[i + 1].tangent_in;

        let samples = sample_between(segments, p0, p3, 30);
        let cubic = fit_constrained(p0, d0, p3, d3, &samples);
        result.push(PathEl::CurveTo(cubic.p1, cubic.p2, cubic.p3));
    }

    result
}

fn build_closed_path_from_anchors(anchors: &[Anchor], segments: &[CubicBez]) -> BezPath {
    let mut result = BezPath::new();
    let n = anchors.len();
    if n < 2 {
        return result;
    }

    result.move_to(anchors[0].point);

    for i in 0..n {
        let next = (i + 1) % n;
        let p0 = anchors[i].point;
        let p3 = anchors[next].point;
        let d0 = anchors[i].tangent_out;
        let d3 = anchors[next].tangent_in;

        let samples = sample_between_wrapped(segments, p0, p3, 30);
        let cubic = fit_constrained(p0, d0, p3, d3, &samples);
        result.push(PathEl::CurveTo(cubic.p1, cubic.p2, cubic.p3));
    }

    result.push(PathEl::ClosePath);
    result
}

// ---------------------------------------------------------------------------
// Sampling helpers
// ---------------------------------------------------------------------------

/// Collect all CurveTo segments from a BezPath as CubicBez values.
fn collect_cubics(path: &BezPath) -> Vec<CubicBez> {
    let mut segments = Vec::new();
    let mut current = Point::ZERO;

    for el in path.elements() {
        match *el {
            PathEl::MoveTo(p) => {
                current = p;
            }
            PathEl::LineTo(p) => {
                // Treat as a degenerate cubic
                segments.push(CubicBez::new(current, current, p, p));
                current = p;
            }
            PathEl::CurveTo(p1, p2, p3) => {
                segments.push(CubicBez::new(current, p1, p2, p3));
                current = p3;
            }
            _ => {}
        }
    }
    segments
}

/// Sample points along all segments.
fn sample_all(segments: &[CubicBez], total_samples: usize) -> Vec<Point> {
    if segments.is_empty() {
        return vec![];
    }
    let per_seg = (total_samples / segments.len()).max(5);
    let mut samples = Vec::new();
    for seg in segments {
        for j in 1..per_seg {
            let t = j as f64 / per_seg as f64;
            samples.push(seg.eval(t));
        }
    }
    samples
}

/// Sample points from the path between two known points (by nearest-point matching).
fn sample_between(segments: &[CubicBez], start: Point, end: Point, n: usize) -> Vec<Point> {
    // Sample the entire path densely
    let all = sample_all_with_positions(segments, 10);

    // Find nearest sample to start
    let start_idx = nearest_idx(&all, start);
    // Find nearest sample to end, searching forward from start
    let end_idx = nearest_idx_from(&all, end, start_idx);

    if end_idx <= start_idx + 1 {
        return vec![];
    }

    // Subsample evenly between start and end
    let range = &all[start_idx..=end_idx];
    let step = range.len() as f64 / (n + 1) as f64;
    (1..=n)
        .map(|i| range[(i as f64 * step) as usize].0)
        .collect()
}

/// Same as sample_between but handles wrapping for closed paths.
fn sample_between_wrapped(
    segments: &[CubicBez],
    start: Point,
    end: Point,
    n: usize,
) -> Vec<Point> {
    let all = sample_all_with_positions(segments, 10);
    if all.is_empty() {
        return vec![];
    }

    let start_idx = nearest_idx(&all, start);
    let end_idx = nearest_idx(&all, end);

    let total = all.len();
    let mut indices: Vec<usize> = Vec::new();

    if end_idx > start_idx {
        // Normal order
        for i in start_idx..=end_idx {
            indices.push(i);
        }
    } else {
        // Wrapped
        for i in start_idx..total {
            indices.push(i);
        }
        for i in 0..=end_idx {
            indices.push(i);
        }
    }

    if indices.len() < 2 {
        return vec![];
    }

    let step = indices.len() as f64 / (n + 1) as f64;
    (1..=n)
        .filter_map(|i| {
            let idx = (i as f64 * step) as usize;
            indices.get(idx).map(|&j| all[j].0)
        })
        .collect()
}

/// Sample all segments densely, returning (Point, global_param) pairs.
fn sample_all_with_positions(
    segments: &[CubicBez],
    per_seg: usize,
) -> Vec<(Point, f64)> {
    let mut samples = Vec::new();
    let n = segments.len();
    if n == 0 {
        return samples;
    }

    for (seg_idx, seg) in segments.iter().enumerate() {
        for j in 0..=per_seg {
            let t = j as f64 / per_seg as f64;
            let global = seg_idx as f64 + t;
            samples.push((seg.eval(t), global));
        }
    }
    samples
}

fn nearest_idx(samples: &[(Point, f64)], target: Point) -> usize {
    samples
        .iter()
        .enumerate()
        .min_by(|(_, a), (_, b)| {
            dist_sq(a.0, target)
                .partial_cmp(&dist_sq(b.0, target))
                .unwrap()
        })
        .map(|(i, _)| i)
        .unwrap_or(0)
}

fn nearest_idx_from(samples: &[(Point, f64)], target: Point, from: usize) -> usize {
    samples[from..]
        .iter()
        .enumerate()
        .min_by(|(_, a), (_, b)| {
            dist_sq(a.0, target)
                .partial_cmp(&dist_sq(b.0, target))
                .unwrap()
        })
        .map(|(i, _)| i + from)
        .unwrap_or(samples.len() - 1)
}

fn dist_sq(a: Point, b: Point) -> f64 {
    (a.x - b.x).powi(2) + (a.y - b.y).powi(2)
}
