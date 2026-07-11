// Copyright 2026 the img2bez Authors
// SPDX-License-Identifier: Apache-2.0 OR MIT

//! Remove redundant on-curve points the fitter left on a smooth arc: drop a
//! point when ONE cubic reproduces the original two-curve path within a tight
//! tolerance.
//!
//! Guards against erasing real structure: the point must be smooth (corners
//! stay), a TRUE local extremum stays (a near-axis tangent alone is not
//! enough — stale labels and saddles may merge), and the two arcs must bend
//! the same way (no merging across an inflection). A short line joined
//! smoothly to a cubic counts as curve material and may absorb.

use kurbo::{BezPath, CubicBez, ParamCurve, PathEl, PathSeg, Point, Vec2};

/// Max distance the single cubic may sit from the original two-curve path.
const MAX_DEVIATION: f64 = 4.0;
/// A join counts as smooth when its tangent break is under this (degrees).
const SMOOTH_MAX_BREAK_DEG: f64 = 12.0;
/// A tangent within this of horizontal/vertical marks an extremum to keep
/// (clean profile).
const EXTREMUM_AXIS_DEG: f64 = 12.0;
/// Photo profile only: breaks larger than this are hard corners that stay;
/// shallower ones are left to the deviation test (soft-scan edge noise).
const PHOTO_CORNER_DEG: f64 = 55.0;
/// Max length (font units) of a line absorbable across a smooth joint —
/// fitter stubs, not designed flats (longer or corner-bounded).
const SHORT_LINE_ABSORB_MAX: f64 = 40.0;
/// Larger budget for a non-axis-aligned line off a hard corner (taper flank:
/// one curve up to its apex). Axis-aligned lines are designed flats.
const TIP_FLANK_ABSORB_MAX: f64 = 100.0;
const TIP_FLANK_BREAK_DEG: f64 = 40.0;
/// A flank candidate within this of horizontal/vertical is a designed flat:
/// designed flats trace fractions of a degree off-axis, real flanks run many.
const TIP_FLANK_AXIS_DEG: f64 = 2.0;
/// Slack (font units) neighbors may protrude past a near-axis point before it
/// stops counting as a true local extremum.
const EXTREMUM_SHADOW_EPS: f64 = 0.25;
/// Grid resolution for the handle-length fit.
const GRID: usize = 24;
/// Max combined chord-projected handle reach (chord fraction; past 1.0 the
/// handles cross). Same invariant/value as `refine::HANDLE_REACH_MAX`.
const HANDLE_REACH_MAX: f64 = 0.9;
/// Samples per cubic when measuring path deviation.
const SAMPLES: usize = 20;

/// Strict extremum keeping (clean/wild profiles): near-axis points are never
/// merged.
pub const KEEP_ALL_EXTREMA: f64 = -1.0;

/// Drop redundant on-curve points from every contour of `path`.
///
/// `extremum_keep_dev` (font units): a near-axis point is kept only when
/// merging across it would deviate by more than this. [`KEEP_ALL_EXTREMA`]
/// keeps them all (clean default); photo passes a small positive value so
/// soft-scan stem wobble collapses while genuine extrema stay.
pub fn remove_redundant_points(
    path: &BezPath,
    extremum_keep_dev: f64,
) -> BezPath {
    let mut out = BezPath::new();
    for contour in split_contours(path) {
        append_contour(&mut out, &simplify_contour(contour, extremum_keep_dev));
    }
    out
}

/// Simplify one closed contour's segments, dropping points until none qualify.
fn simplify_contour(
    mut segs: Vec<PathSeg>,
    extremum_keep_dev: f64,
) -> Vec<PathSeg> {
    // Photo profile: lift short LINE segments to cubics so the cubic-only
    // merge can collapse a soft scan's wobbly stem. Clean/wild keep lines.
    if extremum_keep_dev >= 0.0 {
        for s in segs.iter_mut() {
            if let PathSeg::Line(l) = *s {
                *s = PathSeg::Cubic(CubicBez::new(
                    l.p0,
                    l.p0.lerp(l.p1, 1.0 / 3.0),
                    l.p0.lerp(l.p1, 2.0 / 3.0),
                    l.p1,
                ));
            }
        }
    }
    loop {
        let n = segs.len();
        if n < 3 {
            break;
        }
        let mut removed = false;
        for i in 0..n {
            let prev = (i + n - 1) % n;
            // Lift a short smooth-joined line (fitter stub) to a cubic so the
            // merge test can absorb it; only the merged result is committed.
            let hard_corner_at = |j: usize| -> bool {
                // Tangent break at the joint before segment j.
                let pj = (j + n - 1) % n;
                let (in_d, out_d) =
                    (seg_end_dir(&segs[pj]), seg_start_dir(&segs[j]));
                match (in_d, out_d) {
                    (Some(a), Some(b)) => {
                        a.dot(b) < TIP_FLANK_BREAK_DEG.to_radians().cos()
                    }
                    _ => false,
                }
            };
            let lift = |s: PathSeg, far_hard: bool| -> Option<CubicBez> {
                match s {
                    PathSeg::Cubic(c) => Some(c),
                    PathSeg::Line(l) => {
                        let v = l.p1 - l.p0;
                        let cap = if far_hard && !flat_axial(v) {
                            TIP_FLANK_ABSORB_MAX
                        } else {
                            SHORT_LINE_ABSORB_MAX
                        };
                        (v.hypot() <= cap).then(|| {
                            CubicBez::new(
                                l.p0,
                                l.p0.lerp(l.p1, 1.0 / 3.0),
                                l.p0.lerp(l.p1, 2.0 / 3.0),
                                l.p1,
                            )
                        })
                    }
                    _ => None,
                }
            };
            // Far ends: the joint before prev, and the joint after i.
            let (Some(a), Some(b)) = (
                lift(segs[prev], hard_corner_at(prev)),
                lift(segs[i], hard_corner_at((i + 1) % n)),
            ) else {
                continue;
            };
            // At least one side must be a real cubic — two stub lines are a
            // designed polyline zone, not curve material.
            if matches!(
                (segs[prev], segs[i]),
                (PathSeg::Line(_), PathSeg::Line(_))
            ) {
                continue;
            }
            if let Some(merged) = try_merge(a, b, extremum_keep_dev) {
                segs[prev] = PathSeg::Cubic(merged);
                segs.remove(i);
                removed = true;
                break;
            }
        }
        if !removed {
            break;
        }
    }
    segs
}

/// If the shared point of cubics `a`→`b` is redundant, return the single cubic
/// that reproduces their path; otherwise `None`.
fn try_merge(
    a: CubicBez,
    b: CubicBez,
    extremum_keep_dev: f64,
) -> Option<CubicBez> {
    let p = a.p3; // shared on-curve point
    let in_dir = norm(p - a.p2);
    let out_dir = norm(b.p1 - p);
    if in_dir == Vec2::ZERO || out_dir == Vec2::ZERO {
        return None;
    }
    let photo = extremum_keep_dev >= 0.0;
    let dbg = std::env::var("IMG2BEZ_DEBUG_SIMPLIFY").is_ok();
    // Clean bails on any break past SMOOTH_MAX_BREAK_DEG; photo bails only on
    // a hard corner and lets the deviation test judge shallow scan noise.
    let break_deg = if photo {
        PHOTO_CORNER_DEG
    } else {
        SMOOTH_MAX_BREAK_DEG
    };
    if in_dir.dot(out_dir) < break_deg.to_radians().cos() {
        if dbg {
            eprintln!(
                "    simplify-cand at ({:.1},{:.1}) REJECT break={:.1}°",
                p.x,
                p.y,
                in_dir.dot(out_dir).clamp(-1.0, 1.0).acos().to_degrees()
            );
        }
        return None;
    }
    // Clean never merges across an inflection; photo lets deviation decide.
    if !photo && turn_sign(a) * turn_sign(b) < 0.0 {
        if dbg {
            eprintln!(
                "    simplify-cand at ({:.1},{:.1}) REJECT inflection",
                p.x, p.y
            );
        }
        return None;
    }

    let (s0, s1) = (a.p0, b.p3);
    let u0 = norm(a.p1 - s0);
    let u1 = norm(b.p2 - s1);
    if u0 == Vec2::ZERO || u1 == Vec2::ZERO {
        return None;
    }
    let target: Vec<Point> = sample(a).into_iter().chain(sample(b)).collect();

    // Fit handle lengths so the single cubic hugs the original path;
    // crossed-handle candidates (which read as kinks) are excluded.
    let l0 = (a.p3 - a.p0).hypot();
    let l1 = (b.p3 - b.p0).hypot();
    let hi = (l0 + l1) * 1.1;
    let chord_v = s1 - s0;
    let chord = chord_v.hypot();
    if chord < 1e-9 {
        return None;
    }
    let e = chord_v * (1.0 / chord);
    let reach_limit = chord * HANDLE_REACH_MAX;
    // Magic triangle bound: no handle past the tangent intersection.
    let tri = crate::model::geom::handle_triangle(s0, u0, s1, u1);
    let mut best: Option<(f64, CubicBez)> = None;
    for i in 1..=GRID {
        let la = hi * i as f64 / GRID as f64;
        for j in 1..=GRID {
            let lb = hi * j as f64 / GRID as f64;
            if la * u0.dot(e) - lb * u1.dot(e) > reach_limit {
                continue;
            }
            if let Some((t, s)) = tri
                && (la > t || lb > s)
            {
                continue;
            }
            let cand = CubicBez::new(s0, s0 + u0 * la, s1 + u1 * lb, s1);
            let dev = max_dev(&cand, &target);
            if best.as_ref().is_none_or(|(d, _)| dev < *d) {
                best = Some((dev, cand));
            }
        }
    }
    // Photo fairs out merges within `extremum_keep_dev`; clean keeps every
    // near-axis point and merges the rest within MAX_DEVIATION. Which
    // pass-throughs to keep is intent, not geometry (docs/input-adaptation.md).
    let limit = if photo {
        extremum_keep_dev
    } else {
        MAX_DEVIATION
    };
    if std::env::var("IMG2BEZ_DEBUG_SIMPLIFY").is_ok() {
        eprintln!(
            "    simplify-cand at ({:.1},{:.1}) dev={:.2} limit={:.1} near_axis={} true_ext={}",
            p.x,
            p.y,
            best.as_ref().map(|(d, _)| *d).unwrap_or(f64::NAN),
            limit,
            near_axis(in_dir) || near_axis(out_dir),
            is_true_extremum(p, &target),
        );
    }
    let protected = !photo
        && (near_axis(in_dir) || near_axis(out_dir))
        && is_true_extremum(p, &target);
    let merged = matches!(best, Some((dev, _)) if dev <= limit) && !protected;
    if crate::ml::mldata::enabled() {
        crate::ml::mldata::log(
            "point_merge",
            Some((p.x, p.y)),
            &[
                ("dev", best.as_ref().map(|(d, _)| *d).unwrap_or(f64::NAN)),
                ("limit", limit),
                (
                    "near_axis",
                    if near_axis(in_dir) || near_axis(out_dir) {
                        1.0
                    } else {
                        0.0
                    },
                ),
                (
                    "true_extremum",
                    if is_true_extremum(p, &target) {
                        1.0
                    } else {
                        0.0
                    },
                ),
            ],
            if merged { "merge" } else { "keep" },
        );
    }
    match best {
        Some((dev, cand)) if dev <= limit && !protected => Some(cand),
        _ => None,
    }
}

/// Is `p` genuinely the locally extreme point of its neighborhood in x or y?
/// True only when every sampled neighbor stays on one side in that axis;
/// stale extremum labels and saddles fail and merge freely.
fn is_true_extremum(p: Point, target: &[Point]) -> bool {
    let eps = EXTREMUM_SHADOW_EPS;
    let x_min = target.iter().all(|t| t.x >= p.x - eps);
    let x_max = target.iter().all(|t| t.x <= p.x + eps);
    let y_min = target.iter().all(|t| t.y >= p.y - eps);
    let y_max = target.iter().all(|t| t.y <= p.y + eps);
    x_min || x_max || y_min || y_max
}

/// Max distance from any target point to the `cand` curve (point-to-segment
/// on the sampled polyline, so sample spacing doesn't leak into the metric).
fn max_dev(cand: &CubicBez, target: &[Point]) -> f64 {
    let poly = sample(*cand);
    target
        .iter()
        .map(|t| {
            poly.windows(2)
                .map(|w| dist_to_segment(*t, w[0], w[1]))
                .fold(f64::INFINITY, f64::min)
        })
        .fold(0.0, f64::max)
}

/// Distance from point `p` to segment `a`–`b`.
fn dist_to_segment(p: Point, a: Point, b: Point) -> f64 {
    let ab = b - a;
    let len2 = ab.dot(ab);
    let t = if len2 < 1e-12 {
        0.0
    } else {
        ((p - a).dot(ab) / len2).clamp(0.0, 1.0)
    };
    (p - (a + ab * t)).hypot()
}

fn sample(c: CubicBez) -> Vec<Point> {
    (0..=SAMPLES)
        .map(|k| c.eval(k as f64 / SAMPLES as f64))
        .collect()
}

/// Sign of a cubic's net turning (left vs right bend).
fn turn_sign(c: CubicBez) -> f64 {
    let t0 = norm(c.p1 - c.p0);
    let t1 = norm(c.p3 - c.p2);
    (t0.cross(t1)).signum()
}

/// Normalize, or the zero vector for a degenerate input.
fn norm(v: Vec2) -> Vec2 {
    let l = v.hypot();
    if l > 1e-9 { v / l } else { Vec2::ZERO }
}

/// True if `d` points within `EXTREMUM_AXIS_DEG` of an axis.
fn near_axis(d: Vec2) -> bool {
    let tol = EXTREMUM_AXIS_DEG.to_radians().sin();
    d.x.abs() < tol || d.y.abs() < tol
}

/// True if `v` is within [`TIP_FLANK_AXIS_DEG`] of an axis (designed flat).
fn flat_axial(v: Vec2) -> bool {
    let n = norm(v);
    if n == Vec2::ZERO {
        return false;
    }
    let tol = TIP_FLANK_AXIS_DEG.to_radians().sin();
    n.x.abs() < tol || n.y.abs() < tol
}

/// Direction leaving a segment's start, or None when degenerate.
fn seg_start_dir(s: &PathSeg) -> Option<Vec2> {
    let v = match *s {
        PathSeg::Line(l) => l.p1 - l.p0,
        PathSeg::Cubic(c) => c.p1 - c.p0,
        PathSeg::Quad(q) => q.p1 - q.p0,
    };
    (v.hypot() > 1e-9).then(|| v.normalize())
}

/// Direction arriving at a segment's end, or None when degenerate.
fn seg_end_dir(s: &PathSeg) -> Option<Vec2> {
    let v = match *s {
        PathSeg::Line(l) => l.p1 - l.p0,
        PathSeg::Cubic(c) => c.p3 - c.p2,
        PathSeg::Quad(q) => q.p2 - q.p1,
    };
    (v.hypot() > 1e-9).then(|| v.normalize())
}

/// Split a path into per-contour segment lists.
fn split_contours(path: &BezPath) -> Vec<Vec<PathSeg>> {
    let mut contours = Vec::new();
    let mut cur: Vec<PathSeg> = Vec::new();
    let mut start = Point::ZERO;
    let mut at = Point::ZERO;
    for el in path.elements() {
        match *el {
            PathEl::MoveTo(p) => {
                if !cur.is_empty() {
                    contours.push(std::mem::take(&mut cur));
                }
                start = p;
                at = p;
            }
            PathEl::LineTo(p) => {
                cur.push(PathSeg::Line(kurbo::Line::new(at, p)));
                at = p;
            }
            PathEl::QuadTo(c, p) => {
                cur.push(PathSeg::Quad(kurbo::QuadBez::new(at, c, p)));
                at = p;
            }
            PathEl::CurveTo(c1, c2, p) => {
                cur.push(PathSeg::Cubic(CubicBez::new(at, c1, c2, p)));
                at = p;
            }
            PathEl::ClosePath => {
                if (at - start).hypot() > 1e-9 {
                    cur.push(PathSeg::Line(kurbo::Line::new(at, start)));
                    at = start;
                }
            }
        }
    }
    if !cur.is_empty() {
        contours.push(cur);
    }
    contours
}

/// Append a closed contour (segment list) to `out`.
fn append_contour(out: &mut BezPath, segs: &[PathSeg]) {
    let Some(first) = segs.first() else { return };
    out.move_to(first.start());
    for seg in segs {
        match seg {
            PathSeg::Line(l) => out.line_to(l.p1),
            PathSeg::Quad(q) => out.quad_to(q.p1, q.p2),
            PathSeg::Cubic(c) => out.curve_to(c.p1, c.p2, c.p3),
        }
    }
    out.close_path();
}

#[cfg(test)]
mod tests {
    use super::*;

    fn curves(path: &BezPath) -> usize {
        path.elements()
            .iter()
            .filter(|el| matches!(el, PathEl::CurveTo(..)))
            .count()
    }

    /// A convex cubic split in two collapses back to one cubic.
    #[test]
    fn drops_redundant_arc_point() {
        let orig = CubicBez::new(
            Point::new(0.0, 0.0),
            Point::new(40.0, 10.0),
            Point::new(80.0, 40.0),
            Point::new(100.0, 100.0),
        );
        let left = orig.subsegment(0.0..0.5);
        let right = orig.subsegment(0.5..1.0);
        let mut path = BezPath::new();
        path.move_to(left.p0);
        path.curve_to(left.p1, left.p2, left.p3);
        path.curve_to(right.p1, right.p2, right.p3);
        path.close_path();
        let out = remove_redundant_points(&path, KEEP_ALL_EXTREMA);
        assert!(
            curves(&out) < curves(&path),
            "a redundant curve should merge"
        );
    }

    /// A point with a vertical tangent (an extremum) is kept.
    #[test]
    fn keeps_extremum_point() {
        let mut path = BezPath::new();
        path.move_to(Point::new(0.0, 0.0));
        // Arc up to a rightmost point (vertical tangent), then back.
        path.curve_to(
            Point::new(40.0, 0.0),
            Point::new(80.0, 40.0),
            Point::new(80.0, 80.0),
        );
        path.curve_to(
            Point::new(80.0, 120.0),
            Point::new(40.0, 160.0),
            Point::new(0.0, 160.0),
        );
        path.close_path();
        let out = remove_redundant_points(&path, KEEP_ALL_EXTREMA);
        assert_eq!(curves(&out), curves(&path), "extremum must be kept");
    }
}
