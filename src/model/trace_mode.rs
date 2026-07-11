// Copyright 2026 the img2bez Authors
// SPDX-License-Identifier: Apache-2.0 OR MIT

//! Output-constraint trace modes (see [`TraceMode`]): re-shape the finished
//! outline under a design constraint — every point smooth (organic, all-curve)
//! or every segment a straight line (polygonal). These run as a post-pass on
//! the finished [`Outline`], so they compose with any tracing settings.

use kurbo::{CubicBez, PathEl, PathSeg, Point};

use crate::model::config::TraceMode;
use crate::model::outline::{Contour, Outline, OutlinePoint, PointKind};

/// Flattening tolerance (font units) for `LineOnly`.
const LINE_TOL: f64 = 1.0;

/// Apply the output-shape constraint of `mode` to `outline`.
pub fn apply(outline: Outline, mode: TraceMode) -> Outline {
    match mode {
        TraceMode::Default => outline,
        TraceMode::Smooth => Outline {
            contours: outline.contours.iter().map(smoothify).collect(),
        },
        TraceMode::LineOnly => Outline {
            contours: outline.contours.iter().map(linify).collect(),
        },
    }
}

/// The contour's segments as cubics (lines and quadratics raised).
fn cubic_segs(c: &Contour) -> Vec<[Point; 4]> {
    let bez = Outline {
        contours: vec![c.clone()],
    }
    .to_bezpaths()
    .pop()
    .unwrap_or_default();
    bez.segments()
        .map(|seg| {
            let cb = match seg {
                PathSeg::Cubic(c) => c,
                PathSeg::Quad(q) => q.raise(),
                PathSeg::Line(l) => CubicBez::new(
                    l.p0,
                    l.p0.lerp(l.p1, 1.0 / 3.0),
                    l.p0.lerp(l.p1, 2.0 / 3.0),
                    l.p1,
                ),
            };
            [cb.p0, cb.p1, cb.p2, cb.p3]
        })
        .collect()
}

/// Make every on-curve point a smooth cubic join: at each vertex align both
/// handles to the averaged tangent (G1), preserving handle lengths; lines
/// become straight cubics that flow into their neighbours.
fn smoothify(c: &Contour) -> Contour {
    let mut segs = cubic_segs(c);
    let n = segs.len();
    if n < 2 {
        return c.clone();
    }
    for i in 0..n {
        let prev = (i + n - 1) % n;
        let p = segs[i][0]; // the vertex (== segs[prev][3])
        let din = p - segs[prev][2]; // incoming tangent direction at p
        let dout = segs[i][1] - p; // outgoing tangent direction at p
        let (lin, lout) = (din.hypot(), dout.hypot());
        if lin < 1e-6 || lout < 1e-6 {
            continue;
        }
        let t = din / lin + dout / lout;
        let tl = t.hypot();
        if tl < 1e-9 {
            continue; // opposing tangents (a true cusp) — leave it
        }
        let dir = t / tl;
        segs[prev][2] = p - dir * lin;
        segs[i][1] = p + dir * lout;
    }

    // Rebuild as a UFO ring of smooth curve points; the closing segment's
    // handles precede point 0.
    let mut points: Vec<OutlinePoint> = Vec::with_capacity(n * 3);
    points.push(on(segs[0][0]));
    for (i, s) in segs.iter().enumerate() {
        points.push(off(s[1]));
        points.push(off(s[2]));
        if i < n - 1 {
            points.push(on(s[3]));
        }
    }
    Contour { points }
}

/// Flatten the contour's curves to straight line segments at `LINE_TOL`,
/// producing an all-line polygon with no off-curve points.
fn linify(c: &Contour) -> Contour {
    let bez = Outline {
        contours: vec![c.clone()],
    }
    .to_bezpaths()
    .pop()
    .unwrap_or_default();

    let mut verts: Vec<Point> = Vec::new();
    kurbo::flatten(bez.elements().iter().copied(), LINE_TOL, |el| match el {
        PathEl::MoveTo(p) | PathEl::LineTo(p) => verts.push(p),
        _ => {}
    });
    // Drop a closing point coincident with the start (the ring is implicit).
    let closes = verts.len() > 1
        && matches!((verts.first(), verts.last()),
            (Some(f), Some(l)) if (*f - *l).hypot() < 1e-6);
    if closes {
        verts.pop();
    }
    let points = verts
        .into_iter()
        .map(|p| OutlinePoint {
            x: p.x,
            y: p.y,
            kind: PointKind::Line,
            smooth: false,
        })
        .collect();
    Contour { points }
}

fn on(p: Point) -> OutlinePoint {
    OutlinePoint {
        x: p.x,
        y: p.y,
        kind: PointKind::Curve,
        smooth: true,
    }
}

fn off(p: Point) -> OutlinePoint {
    OutlinePoint {
        x: p.x,
        y: p.y,
        kind: PointKind::OffCurve,
        smooth: false,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use kurbo::BezPath;

    fn square() -> Outline {
        let mut p = BezPath::new();
        p.move_to((0.0, 0.0));
        p.line_to((100.0, 0.0));
        p.line_to((100.0, 100.0));
        p.line_to((0.0, 100.0));
        p.close_path();
        Outline::from_bezpaths(&[p])
    }

    #[test]
    fn smooth_makes_every_point_a_smooth_curve() {
        let out = apply(square(), TraceMode::Smooth);
        let pts = &out.contours[0].points;
        // every on-curve point is a smooth curve point; the rest are off-curve.
        for p in pts {
            match p.kind {
                PointKind::Curve => {
                    assert!(p.smooth, "on-curve must be smooth")
                }
                PointKind::OffCurve => {}
                k => panic!("unexpected kind {k:?}"),
            }
        }
        // 4 corners -> 4 smooth on-curve points, each with 2 handles = 12.
        let on = pts.iter().filter(|p| p.kind == PointKind::Curve).count();
        assert_eq!(on, 4);
        assert_eq!(pts.len(), 12);
    }

    #[test]
    fn line_only_has_no_off_curve_points() {
        // A circle-ish outline so there are real curves to flatten.
        let mut p = BezPath::new();
        p.move_to((50.0, 0.0));
        p.curve_to((78.0, 0.0), (100.0, 22.0), (100.0, 50.0));
        p.curve_to((100.0, 78.0), (78.0, 100.0), (50.0, 100.0));
        p.curve_to((22.0, 100.0), (0.0, 78.0), (0.0, 50.0));
        p.curve_to((0.0, 22.0), (22.0, 0.0), (50.0, 0.0));
        p.close_path();
        let out = apply(Outline::from_bezpaths(&[p]), TraceMode::LineOnly);
        let pts = &out.contours[0].points;
        assert!(pts.iter().all(|p| p.kind == PointKind::Line));
        // a flattened quarter-circle yields several segments, not 4.
        assert!(pts.len() > 8, "got {} line points", pts.len());
    }

    #[test]
    fn default_is_unchanged() {
        let sq = square();
        assert_eq!(apply(sq.clone(), TraceMode::Default), sq);
    }
}
