// Copyright 2026 the img2bez Authors
// SPDX-License-Identifier: Apache-2.0 OR MIT

//! Flatten near-straight curves (AA wobble the fitter read as faint curves)
//! into lines and merge near-collinear lines. Real curves and corners stay.

use kurbo::{BezPath, Line, ParamCurve, PathEl, PathSeg, Point, QuadBez};

/// A curve is "straight" when both control points sit within this distance
/// (font units) of its chord — a fraction of a pixel of AA wobble, not a curve.
const FLATTEN_MAX_OFFSET: f64 = 3.0;
/// An end tangent within this many degrees of horizontal/vertical marks a
/// designed axial tangent (a shallow bracket easing into an extremum).
const AXIAL_TANGENT_MAX_DEG: f64 = 10.0;
/// A chord further than this from every axis cannot represent an axial
/// tangent, so flattening onto it would kink the join.
const CHORD_OFF_AXIS_MIN_DEG: f64 = 2.0;
/// Below this chord length (font units) the axial-tangent veto does not
/// apply: designed brackets are longer; junction debris must keep flattening
/// or it survives as reach-violating micro-cubics.
const AXIAL_VETO_MIN_CHORD: f64 = 30.0;
/// Two lines merge when the turn between them is under this (degrees) — a
/// straight run with a sub-pixel kink, not an intentional corner.
const COLLINEAR_MAX_TURN_DEG: f64 = 4.0;

/// Flatten near-straight curves and merge collinear lines across every contour.
pub fn flatten_straight_runs(path: &BezPath) -> BezPath {
    let mut out = BezPath::new();
    for contour in split_contours(path) {
        let segs = merge_collinear(flatten(contour));
        append_contour(&mut out, &segs);
    }
    out
}

/// Replace each near-straight cubic/quad with a line.
fn flatten(segs: Vec<PathSeg>) -> Vec<PathSeg> {
    let n = segs.len();
    (0..n)
        .map(|i| {
            let seg = segs[i];
            let (a, c_first, c_last, b) = match seg {
                PathSeg::Cubic(c) if straight(c.p0, &[c.p1, c.p2], c.p3) => {
                    (c.p0, c.p1, c.p2, c.p3)
                }
                PathSeg::Quad(q) if straight(q.p0, &[q.p1], q.p2) => {
                    (q.p0, q.p1, q.p1, q.p2)
                }
                other => return other,
            };
            let prev_in = seg_end_tangent(&segs[(i + n - 1) % n]);
            let next_out = seg_start_tangent(&segs[(i + 1) % n]);
            if keeps_axial_tangent(a, c_first, c_last, b, prev_in, next_out) {
                return seg;
            }
            PathSeg::Line(Line::new(a, b))
        })
        .collect()
}

/// True if every control point lies within `FLATTEN_MAX_OFFSET` of the chord.
fn straight(a: Point, controls: &[Point], b: Point) -> bool {
    controls
        .iter()
        .all(|c| dist_to_line(*c, a, b) <= FLATTEN_MAX_OFFSET)
}

/// Which axis a vector aligns with inside [`AXIAL_TANGENT_MAX_DEG`]:
/// `Some(true)` horizontal, `Some(false)` vertical, `None` neither.
fn axis_of(v: kurbo::Vec2) -> Option<bool> {
    if v.hypot() < 1e-9 {
        return None;
    }
    let ang = v.y.atan2(v.x).abs().to_degrees(); // 0..180
    let to_h = ang.min(180.0 - ang);
    let to_v = (90.0 - ang).abs();
    if to_h < AXIAL_TANGENT_MAX_DEG && to_h <= to_v {
        Some(true)
    } else if to_v < AXIAL_TANGENT_MAX_DEG {
        Some(false)
    } else {
        None
    }
}

/// Degrees from the nearest axis (horizontal or vertical), in 0..=45.
fn off_axis_deg(v: kurbo::Vec2) -> f64 {
    let ang = v.y.atan2(v.x).abs().to_degrees(); // 0..180
    let to_h = ang.min(180.0 - ang);
    (to_h).min((90.0 - ang).abs())
}

/// A near-straight curve stays a CURVE when an end tangent is axis-aligned,
/// the neighbor meets that end on the same axis, and the chord is not axial
/// — a designed shallow bracket easing into an axial extremum. AA-wobble
/// runs fail the neighbor test, so genuine runs still flatten.
fn keeps_axial_tangent(
    a: Point,
    c_first: Point,
    c_last: Point,
    b: Point,
    prev_in: kurbo::Vec2,
    next_out: kurbo::Vec2,
) -> bool {
    let chord = b - a;
    if chord.hypot() < AXIAL_VETO_MIN_CHORD
        || off_axis_deg(chord) < CHORD_OFF_AXIS_MIN_DEG
    {
        return false;
    }
    let ends = [(c_first - a, prev_in), (b - c_last, next_out)];
    ends.into_iter().any(|(t, neighbor)| {
        matches!((axis_of(t), axis_of(neighbor)), (Some(x), Some(y)) if x == y)
    })
}

/// Direction a segment leaves its start point.
fn seg_start_tangent(seg: &PathSeg) -> kurbo::Vec2 {
    match *seg {
        PathSeg::Line(l) => l.p1 - l.p0,
        PathSeg::Quad(q) => {
            let t = q.p1 - q.p0;
            if t.hypot() > 1e-9 { t } else { q.p2 - q.p0 }
        }
        PathSeg::Cubic(c) => {
            let t = c.p1 - c.p0;
            if t.hypot() > 1e-9 { t } else { c.p2 - c.p0 }
        }
    }
}

/// Direction a segment arrives at its end point.
fn seg_end_tangent(seg: &PathSeg) -> kurbo::Vec2 {
    match *seg {
        PathSeg::Line(l) => l.p1 - l.p0,
        PathSeg::Quad(q) => {
            let t = q.p2 - q.p1;
            if t.hypot() > 1e-9 { t } else { q.p2 - q.p0 }
        }
        PathSeg::Cubic(c) => {
            let t = c.p3 - c.p2;
            if t.hypot() > 1e-9 { t } else { c.p3 - c.p1 }
        }
    }
}

/// Merge consecutive lines that meet at a negligible angle, dropping the joint.
fn merge_collinear(mut segs: Vec<PathSeg>) -> Vec<PathSeg> {
    let cos_min = COLLINEAR_MAX_TURN_DEG.to_radians().cos();
    loop {
        let n = segs.len();
        if n < 2 {
            break;
        }
        let mut merged = false;
        for i in 0..n {
            let j = (i + 1) % n;
            let (PathSeg::Line(a), PathSeg::Line(b)) = (segs[i], segs[j])
            else {
                continue;
            };
            let (u, v) = (a.p1 - a.p0, b.p1 - b.p0);
            if u.hypot() < 1e-9 || v.hypot() < 1e-9 {
                continue;
            }
            if u.normalize().dot(v.normalize()) >= cos_min {
                segs[i] = PathSeg::Line(Line::new(a.p0, b.p1));
                segs.remove(j);
                merged = true;
                break;
            }
        }
        if !merged {
            break;
        }
    }
    segs
}

/// Distance from `p` to the infinite line through `a`–`b`.
fn dist_to_line(p: Point, a: Point, b: Point) -> f64 {
    let ab = b - a;
    let len = ab.hypot();
    if len < 1e-9 {
        return (p - a).hypot();
    }
    (ab.cross(p - a) / len).abs()
}

/// Split a path into per-contour segment lists (closing each with a line).
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
                cur.push(PathSeg::Line(Line::new(at, p)));
                at = p;
            }
            PathEl::QuadTo(c, p) => {
                cur.push(PathSeg::Quad(QuadBez::new(at, c, p)));
                at = p;
            }
            PathEl::CurveTo(c1, c2, p) => {
                cur.push(PathSeg::Cubic(kurbo::CubicBez::new(at, c1, c2, p)));
                at = p;
            }
            PathEl::ClosePath => {
                if (at - start).hypot() > 1e-9 {
                    cur.push(PathSeg::Line(Line::new(at, start)));
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

    fn lines(path: &BezPath) -> usize {
        path.elements()
            .iter()
            .filter(|el| matches!(el, PathEl::LineTo(_)))
            .count()
    }

    #[test]
    fn flattens_and_merges_a_wobbled_straight_run() {
        // A straight diagonal broken into line / faint-curve / line.
        let mut path = BezPath::new();
        path.move_to(Point::new(0.0, 0.0));
        path.line_to(Point::new(90.0, 322.0));
        path.curve_to(
            Point::new(93.0, 331.0),
            Point::new(100.0, 349.0),
            Point::new(100.0, 358.0),
        );
        path.line_to(Point::new(128.0, 456.0));
        path.line_to(Point::new(0.0, 456.0));
        path.close_path();
        let out = flatten_straight_runs(&path);
        // The diagonal run collapses to one line (plus the bottom + closing).
        assert!(
            lines(&out) <= 3,
            "straight run should merge: {}",
            lines(&out)
        );
    }

    #[test]
    fn keeps_a_real_curve() {
        let mut path = BezPath::new();
        path.move_to(Point::new(0.0, 0.0));
        path.curve_to(
            Point::new(0.0, 55.0),
            Point::new(45.0, 100.0),
            Point::new(100.0, 100.0),
        );
        path.line_to(Point::new(0.0, 100.0));
        path.close_path();
        let out = flatten_straight_runs(&path);
        assert!(
            out.elements()
                .iter()
                .any(|el| matches!(el, PathEl::CurveTo(..))),
            "a real curve must be kept"
        );
    }
}
