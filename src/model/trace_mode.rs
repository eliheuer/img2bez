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
pub fn apply(outline: Outline, mode: TraceMode, keep_corner_deg: f64) -> Outline {
    match mode {
        TraceMode::Default => outline,
        TraceMode::Smooth => Outline {
            contours: outline
                .contours
                .iter()
                .map(|c| smoothify(c, keep_corner_deg))
                .collect(),
        },
        TraceMode::SmoothG2 => Outline {
            contours: outline.contours.iter().map(spline_g2).collect(),
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

/// Make on-curve points smooth cubic joins: at each vertex align both
/// handles to the averaged tangent (G1), preserving handle lengths;
/// lines become straight cubics that flow into their neighbours. A
/// vertex whose direction turns more than `keep_corner_deg` keeps its
/// corner: taper tips and hard junctions reverse direction, and
/// forcing them smooth inverts the outline into loops.
fn smoothify(c: &Contour, keep_corner_deg: f64) -> Contour {
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
        let cos_turn = (din / lin).dot(dout / lout);
        if cos_turn < (keep_corner_deg.to_radians()).cos() {
            continue; // turns harder than the threshold — keep the corner
        }
        let t = din / lin + dout / lout;
        let tl = t.hypot();
        if tl < 1e-9 {
            continue; // exactly opposing tangents — leave it
        }
        let dir = t / tl;
        // Attenuate handle length as the turn sharpens: long handles
        // along the (unstable) averaged direction of a near-reversal
        // invert the outline into loops. cos(turn/2) is 1 for a
        // straight-through join and 0 at a full reversal, so tips
        // become small rounded caps and stay smooth.
        let scale = ((1.0 + cos_turn.clamp(-1.0, 1.0)) * 0.5).sqrt();
        segs[prev][2] = p - dir * (lin * scale);
        segs[i][1] = p + dir * (lout * scale);
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

/// Re-spline the contour as a periodic natural cubic spline through
/// its on-curve points: C2 (curvature-continuous) everywhere, every
/// point smooth, one cubic bezier per knot interval. The spline
/// interpolates the knots exactly; deviation between knots is bounded
/// by knot density, so tight fits stay faithful.
fn spline_g2(c: &Contour) -> Contour {
    let segs = cubic_segs(c);
    // knots: the on-curve ring, deduped of coincident neighbours
    let mut knots: Vec<Point> = Vec::with_capacity(segs.len());
    for s in &segs {
        if knots.last().map_or(true, |l: &Point| (*l - s[0]).hypot() > 1e-6) {
            knots.push(s[0]);
        }
    }
    if knots.len() > 1
        && (knots[0] - *knots.last().unwrap()).hypot() <= 1e-6
    {
        knots.pop();
    }
    let n = knots.len();
    if n < 3 {
        return smoothify(c, 180.0);
    }

    // chord-length steps, cyclic
    let h: Vec<f64> = (0..n)
        .map(|i| (knots[(i + 1) % n] - knots[i]).hypot().max(1e-6))
        .collect();

    // periodic natural cubic spline second derivatives, per axis
    let solve = |vals: &dyn Fn(usize) -> f64| -> Vec<f64> {
        // cyclic tridiagonal via Sherman-Morrison
        let a: Vec<f64> = (0..n).map(|i| h[(i + n - 1) % n]).collect(); // sub
        let b: Vec<f64> = (0..n).map(|i| 2.0 * (h[(i + n - 1) % n] + h[i])).collect();
        let c_: Vec<f64> = h.clone(); // super
        let d: Vec<f64> = (0..n)
            .map(|i| {
                let prev = (i + n - 1) % n;
                let next = (i + 1) % n;
                6.0 * ((vals(next) - vals(i)) / h[i] - (vals(i) - vals(prev)) / h[prev])
            })
            .collect();
        // Solve cyclic system B m = d where B has corners a[0], c[n-1].
        let tri = |b: &[f64], d: &[f64]| -> Vec<f64> {
            let mut cp = vec![0.0; n];
            let mut dp = vec![0.0; n];
            cp[0] = c_[0] / b[0];
            dp[0] = d[0] / b[0];
            for i in 1..n {
                let m = b[i] - a[i] * cp[i - 1];
                cp[i] = c_[i] / m;
                dp[i] = (d[i] - a[i] * dp[i - 1]) / m;
            }
            let mut x = vec![0.0; n];
            x[n - 1] = dp[n - 1];
            for i in (0..n - 1).rev() {
                x[i] = dp[i] - cp[i] * x[i + 1];
            }
            x
        };
        let gamma = -b[0];
        let mut bb = b.clone();
        bb[0] = b[0] - gamma;
        bb[n - 1] = b[n - 1] - a[0] * c_[n - 1] / gamma;
        // u vector: gamma at 0, c[n-1] at n-1
        let mut u = vec![0.0; n];
        u[0] = gamma;
        u[n - 1] = c_[n - 1];
        // The cyclic terms couple rows 0 and n-1; classic S-M solve:
        let x = tri(&bb, &d);
        let z = tri(&bb, &u);
        let fact = (x[0] + a[0] * x[n - 1] / gamma)
            / (1.0 + z[0] + a[0] * z[n - 1] / gamma);
        (0..n).map(|i| x[i] - fact * z[i]).collect()
    };
    let xs: Vec<f64> = knots.iter().map(|p| p.x).collect();
    let ys: Vec<f64> = knots.iter().map(|p| p.y).collect();
    let mx = solve(&|i| xs[i]);
    let my = solve(&|i| ys[i]);

    // emit one cubic per interval from endpoint first derivatives
    let mut points: Vec<OutlinePoint> = Vec::with_capacity(n * 3);
    points.push(on(knots[0]));
    for i in 0..n {
        let j = (i + 1) % n;
        let hi = h[i];
        let p0 = knots[i];
        let p1 = knots[j];
        let d0 = kurbo::Vec2::new(
            (p1.x - p0.x) / hi - hi * (2.0 * mx[i] + mx[j]) / 6.0,
            (p1.y - p0.y) / hi - hi * (2.0 * my[i] + my[j]) / 6.0,
        );
        let d1 = kurbo::Vec2::new(
            (p1.x - p0.x) / hi + hi * (mx[i] + 2.0 * mx[j]) / 6.0,
            (p1.y - p0.y) / hi + hi * (my[i] + 2.0 * my[j]) / 6.0,
        );
        points.push(off(p0 + d0 * (hi / 3.0)));
        points.push(off(p1 - d1 * (hi / 3.0)));
        if i < n - 1 {
            points.push(on(p1));
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
        let out = apply(square(), TraceMode::Smooth, 180.0);
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
        let out = apply(Outline::from_bezpaths(&[p]), TraceMode::LineOnly, 180.0);
        let pts = &out.contours[0].points;
        assert!(pts.iter().all(|p| p.kind == PointKind::Line));
        // a flattened quarter-circle yields several segments, not 4.
        assert!(pts.len() > 8, "got {} line points", pts.len());
    }

    #[test]
    fn default_is_unchanged() {
        let sq = square();
        assert_eq!(apply(sq.clone(), TraceMode::Default, 180.0), sq);
    }
}
