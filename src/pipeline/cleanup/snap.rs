// Copyright 2026 the img2bez Authors
// SPDX-License-Identifier: Apache-2.0 OR MIT

//! Grid snapping and H/V handle correction.

use std::f64::consts::{FRAC_PI_2, PI};

use kurbo::{BezPath, PathEl, Point, Vec2};

/// Max curve movement (font units) allowed when rounding handles to
/// integers; only exceeded in pathological cases, where floats are kept.
const ROUND_MAX_DEVIATION: f64 = 1.5;

/// Snap on-curve points to a grid. Off-curve control points are NOT snapped,
/// preserving curve shape.
pub fn to_grid(path: &BezPath, grid: f64) -> BezPath {
    let snap = |p: Point| -> Point {
        Point::new((p.x / grid).round() * grid, (p.y / grid).round() * grid)
    };
    BezPath::from_vec(
        path.elements()
            .iter()
            .map(|el| match *el {
                PathEl::MoveTo(p) => PathEl::MoveTo(snap(p)),
                PathEl::LineTo(p) => PathEl::LineTo(snap(p)),
                PathEl::CurveTo(a, b, p) => PathEl::CurveTo(a, b, snap(p)),
                PathEl::QuadTo(a, p) => PathEl::QuadTo(a, snap(p)),
                PathEl::ClosePath => PathEl::ClosePath,
            })
            .collect(),
    )
}

/// Round off-curve control points to integers. Exact H/V handles stay H/V
/// (only the free axis moves); a segment whose shape would shift more than
/// `ROUND_MAX_DEVIATION` keeps its floats.
pub fn round_handles(path: &BezPath) -> BezPath {
    let round = |p: Point| Point::new(p.x.round(), p.y.round());
    let mut out = BezPath::new();
    let mut cur = Point::ZERO;

    for el in path.elements() {
        match *el {
            PathEl::MoveTo(p) => {
                out.move_to(p);
                cur = p;
            }
            PathEl::LineTo(p) => {
                out.line_to(p);
                cur = p;
            }
            PathEl::CurveTo(c1, c2, p) => {
                let (r1, r2) = (round(c1), round(c2));
                if curve_shift(cur, c1, c2, p, r1, r2) <= ROUND_MAX_DEVIATION {
                    out.curve_to(r1, r2, p);
                } else {
                    out.curve_to(c1, c2, p);
                }
                cur = p;
            }
            PathEl::QuadTo(c, p) => {
                let rc = round(c);
                if curve_shift(cur, c, c, p, rc, rc) <= ROUND_MAX_DEVIATION {
                    out.quad_to(rc, p);
                } else {
                    out.quad_to(c, p);
                }
                cur = p;
            }
            PathEl::ClosePath => out.close_path(),
        }
    }

    out
}

/// Max distance between an original cubic and the same cubic with rounded
/// control points, sampled uniformly.
fn curve_shift(
    a: Point,
    c1: Point,
    c2: Point,
    b: Point,
    r1: Point,
    r2: Point,
) -> f64 {
    const N: usize = 16;
    let eval = |c1: Point, c2: Point, t: f64| -> Vec2 {
        let mt = 1.0 - t;
        a.to_vec2() * (mt * mt * mt)
            + c1.to_vec2() * (3.0 * mt * mt * t)
            + c2.to_vec2() * (3.0 * mt * t * t)
            + b.to_vec2() * (t * t * t)
    };
    (0..=N)
        .map(|k| {
            let t = k as f64 / N as f64;
            (eval(c1, c2, t) - eval(r1, r2, t)).hypot()
        })
        .fold(0.0, f64::max)
}

/// A join whose tangent break exceeds this (degrees) reads as a corner
/// anchor; its handles get the tighter [`CORNER_HV_SNAP_DEG`] band.
const CORNER_ANCHOR_BREAK_DEG: f64 = 25.0;

/// H/V snap band (degrees) for corner-anchored handles. Much tighter than
/// the general threshold: axial stem handles still land on axis, but a
/// leaning stroke-end handle keeps its lean (axis-forcing reshapes a taper).
pub const CORNER_HV_SNAP_DEG: f64 = 9.0;

/// On-curve points whose join breaks above `CORNER_ANCHOR_BREAK_DEG` — the
/// anchors [`hv_handles`] treats with the tighter corner snap band.
pub fn corner_anchor_points(path: &BezPath) -> Vec<Point> {
    let mut out = Vec::new();
    // Per contour: collect (in_dir, anchor, out_dir) around each on-curve.
    let mut contour: Vec<PathEl> = Vec::new();
    let flush = |els: &[PathEl], out: &mut Vec<Point>| {
        let mut prev = Point::ZERO;
        let mut anchors: Vec<Point> = Vec::new();
        let mut in_ref: Vec<Point> = Vec::new(); // point before anchor
        for el in els {
            match *el {
                PathEl::MoveTo(p) => {
                    anchors.push(p);
                    in_ref.push(p); // self: zero vin skips it below
                    prev = p;
                }
                PathEl::LineTo(p) => {
                    anchors.push(p);
                    in_ref.push(prev);
                    prev = p;
                }
                PathEl::CurveTo(_, c2, p) => {
                    anchors.push(p);
                    in_ref.push(c2);
                    prev = p;
                }
                PathEl::QuadTo(c, p) => {
                    anchors.push(p);
                    in_ref.push(c);
                    prev = p;
                }
                PathEl::ClosePath => {}
            }
        }
        let n = anchors.len();
        if n < 3 {
            return;
        }
        // Outgoing reference: first off-curve (or endpoint) of the next seg.
        let mut seg_first: Vec<Point> = Vec::new();
        for el in els {
            match *el {
                PathEl::MoveTo(_) => {}
                PathEl::LineTo(p) => seg_first.push(p),
                PathEl::CurveTo(c1, _, _) => seg_first.push(c1),
                PathEl::QuadTo(c, _) => seg_first.push(c),
                PathEl::ClosePath => {}
            }
        }
        // The wrap to anchors[0] via close is approximated with anchors[0].
        for i in 0..n {
            let a = anchors[i];
            let vin = a - in_ref[i];
            let out_pt = if i < seg_first.len() {
                seg_first[i]
            } else {
                anchors[0]
            };
            let vout = out_pt - a;
            if vin.hypot() < 1e-9 || vout.hypot() < 1e-9 {
                continue;
            }
            let cosang = vin.dot(vout) / (vin.hypot() * vout.hypot());
            if cosang < CORNER_ANCHOR_BREAK_DEG.to_radians().cos() {
                out.push(a);
            }
        }
        // The MoveTo anchor's zero-length vin skips it above — acceptable:
        // a true corner there is still caught via the hard break at close.
    };
    for el in path.elements() {
        if matches!(el, PathEl::MoveTo(_)) && !contour.is_empty() {
            flush(&contour, &mut out);
            contour.clear();
        }
        contour.push(*el);
    }
    if !contour.is_empty() {
        flush(&contour, &mut out);
    }
    out
}

/// Snap handles within `threshold_deg` of horizontal/vertical to exact H/V.
/// Exceptions: anchors in `skip` (smooth inflections — flattening bows the
/// curve) are untouched; anchors in `corners` use the tighter
/// [`CORNER_HV_SNAP_DEG`] band.
pub fn hv_handles(
    path: &BezPath,
    threshold_deg: f64,
    skip: &[Point],
    corners: &[Point],
) -> BezPath {
    let threshold = threshold_deg.to_radians();
    let corner_threshold = CORNER_HV_SNAP_DEG.to_radians();
    let is_skipped =
        |anchor: Point| skip.iter().any(|s| (*s - anchor).hypot() < 1.0);
    let thresh_for = |anchor: Point| -> f64 {
        if corners.iter().any(|c| (*c - anchor).hypot() < 1.0) {
            corner_threshold
        } else {
            threshold
        }
    };
    let lines = line_dirs(path);
    let line_at = |anchor: Point| -> Option<Vec2> {
        lines
            .iter()
            .find(|(a, _)| (*a - anchor).hypot() < 1.0)
            .map(|(_, u)| *u)
    };
    let mut elements: Vec<PathEl> = path.elements().to_vec();
    let mut prev = Point::ZERO;

    for el in &mut elements {
        match el {
            PathEl::MoveTo(p) | PathEl::LineTo(p) => {
                prev = *p;
            }
            PathEl::CurveTo(c1, c2, p3) => {
                if !is_skipped(prev) {
                    *c1 =
                        snap_handle(*c1, prev, thresh_for(prev), line_at(prev));
                }
                if !is_skipped(*p3) {
                    *c2 = snap_handle(*c2, *p3, thresh_for(*p3), line_at(*p3));
                }
                prev = *p3;
            }
            PathEl::QuadTo(_, p) => prev = *p,
            PathEl::ClosePath => {}
        }
    }

    BezPath::from_vec(elements)
}

/// Direction of the line segment(s) adjoining each on-curve point. A curve
/// handle at such an anchor is a tangent-point candidate whose continuity is
/// against the line, not the axes.
fn line_dirs(path: &BezPath) -> Vec<(Point, Vec2)> {
    let mut out = Vec::new();
    let mut start = Point::ZERO;
    let mut prev = Point::ZERO;
    let push = |a: Point, b: Point, out: &mut Vec<(Point, Vec2)>| {
        let v = b - a;
        if v.hypot() > 1e-9 {
            let u = v / v.hypot();
            out.push((a, u));
            out.push((b, u));
        }
    };
    for el in path.elements() {
        match *el {
            PathEl::MoveTo(p) => {
                start = p;
                prev = p;
            }
            PathEl::LineTo(p) => {
                push(prev, p, &mut out);
                prev = p;
            }
            PathEl::CurveTo(_, _, p) | PathEl::QuadTo(_, p) => prev = p,
            PathEl::ClosePath => {
                push(prev, start, &mut out);
                prev = start;
            }
        }
    }
    out
}

/// Snap a handle to exact H/V within `threshold` (radians) of an axis.
/// Tangent points come first: a handle within `threshold` of an adjoining
/// line's axis is projected onto the line (exact G1) instead — axis-snapping
/// against a diagonal line turns a smooth join into a kink.
fn snap_handle(
    handle: Point,
    anchor: Point,
    threshold: f64,
    line: Option<Vec2>,
) -> Point {
    let dx = handle.x - anchor.x;
    let dy = handle.y - anchor.y;
    if dx * dx + dy * dy < 1e-12 {
        return handle; // coincident — nothing to snap
    }

    if let Some(u) = line {
        let v = handle - anchor;
        let off_axis = v.cross(u).atan2(v.dot(u)).abs(); // 0..PI
        if off_axis.min(PI - off_axis) < threshold {
            return anchor + u * v.dot(u);
        }
    }

    let angle = dy.atan2(dx).abs(); // range: 0..PI

    // Near horizontal (0 deg or 180 deg): lock y to anchor.
    if angle < threshold || (PI - angle) < threshold {
        return Point::new(handle.x, anchor.y);
    }
    // Near vertical (90 deg): lock x to anchor.
    if (angle - FRAC_PI_2).abs() < threshold {
        return Point::new(anchor.x, handle.y);
    }
    handle
}

#[cfg(test)]
mod tests {
    use super::*;
    use kurbo::{BezPath, PathEl, Point};

    #[test]
    fn grid_snap_rounds_oncurve() {
        let mut path = BezPath::new();
        path.move_to(Point::new(10.3, 20.7));
        path.line_to(Point::new(50.9, 30.1));
        path.line_to(Point::new(41.4, 61.6));
        path.push(PathEl::ClosePath);

        let snapped = to_grid(&path, 2.0);
        for el in snapped.elements() {
            match el {
                PathEl::MoveTo(p) | PathEl::LineTo(p) => {
                    assert_eq!(p.x % 2.0, 0.0, "x={} not a multiple of 2", p.x);
                    assert_eq!(p.y % 2.0, 0.0, "y={} not a multiple of 2", p.y);
                }
                _ => {}
            }
        }
    }
}
