// Copyright 2026 the img2bez Authors
// SPDX-License-Identifier: Apache-2.0 OR MIT

//! Place an on-curve point at strong curve inflections. Splitting by
//! subdivision reproduces the curve exactly, so this is a pure structure
//! improvement; only *genuine* inflections (well inside the segment, both
//! lobes sweeping a real turn) are split.

use kurbo::{BezPath, CubicBez, ParamCurve, ParamCurveDeriv, PathEl, Point};

/// Min curve-parameter distance from either endpoint — a sign change crowded
/// against an end is a fit artifact, not a feature.
const T_MARGIN: f64 = 0.15;

/// Min tangent turn (degrees) per flanking lobe: spine esses sweep ~80°,
/// transition esses (best drawn as one cubic) ~30-45°, wiggles under 10°.
const MIN_LOBE_TURN_DEG: f64 = 55.0;

/// Skip segments shorter than this chord (font units).
const MIN_CHORD: f64 = 40.0;

/// Min absolute distance (font units) of the split from either endpoint;
/// T_MARGIN alone still hugs a corner on a short cubic.
const MIN_SPLIT_DIST: f64 = 28.0;

/// Insert an on-curve point at each cubic's strongest genuine inflection.
///
/// Runs after H/V snapping so the deliberately diagonal inflection handles
/// are not flattened. `grid` (0 = off) rounds the new point, co-translating
/// its two adjacent handles by the same delta so they stay collinear.
pub fn split_inflections(path: &BezPath, grid: f64) -> BezPath {
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
                let cubic = CubicBez::new(cur, c1, c2, p);
                if let Some(t) = inflection_split_param(&cubic) {
                    let left = cubic.subsegment(0.0..t);
                    let right = cubic.subsegment(t..1.0);
                    let q = left.p3;
                    let delta = snap_to_grid(q, grid) - q;
                    let q_snapped = q + delta;
                    // Co-translate the flanking handles; outer handles stay.
                    out.curve_to(left.p1, left.p2 + delta, q_snapped);
                    out.curve_to(right.p1 + delta, right.p2, right.p3);
                } else {
                    out.curve_to(c1, c2, p);
                }
                cur = p;
            }
            PathEl::QuadTo(c, p) => {
                out.quad_to(c, p);
                cur = p;
            }
            PathEl::ClosePath => out.close_path(),
        }
    }

    out
}

/// On-curve points that are smooth inflections: collinear handles, and the
/// neighbouring on-curve points straddle the tangent line.
///
/// H/V snapping must leave these handles alone — an inflection tangent is
/// often near-axis but meant to be diagonal; flattening it bows the curve.
pub fn smooth_inflection_points(path: &BezPath) -> Vec<Point> {
    let segs: Vec<kurbo::PathSeg> = path.segments().collect();
    let m = segs.len();
    if m < 2 {
        return Vec::new();
    }
    let mut out = Vec::new();
    for i in 0..m {
        let prev = segs[i];
        let next = segs[(i + 1) % m];
        // Both sides must be cubics (lines carry no handle to protect).
        let (kurbo::PathSeg::Cubic(a), kurbo::PathSeg::Cubic(b)) = (prev, next)
        else {
            continue;
        };
        let p = a.p3; // shared on-curve point
        let h_in = p - a.p2; // tangent arriving at P
        let h_out = b.p1 - p; // tangent leaving P
        if h_in.hypot() < 1e-6 || h_out.hypot() < 1e-6 {
            continue;
        }
        // Smooth join: arriving and leaving tangents nearly parallel.
        let cos = h_in.normalize().dot(h_out.normalize());
        if cos < SMOOTH_COS {
            continue;
        }
        // Inflection: the flanking on-curve points straddle the tangent line.
        let normal = kurbo::Vec2::new(-h_out.y, h_out.x);
        let side_prev = (a.p0 - p).dot(normal);
        let side_next = (b.p3 - p).dot(normal);
        if side_prev * side_next < 0.0 {
            out.push(p);
        }
    }
    out
}

/// Cosine threshold for treating a join as smooth (~14° of tangent break).
const SMOOTH_COS: f64 = 0.97;

/// Round a point to the grid, or leave it untouched when grid snapping is off.
fn snap_to_grid(p: Point, grid: f64) -> Point {
    if grid <= 0.0 {
        return p;
    }
    Point::new((p.x / grid).round() * grid, (p.y / grid).round() * grid)
}

/// Return the parameter of the cubic's strongest genuine inflection, or
/// `None` if it carries no inflection worth a point.
fn inflection_split_param(c: &CubicBez) -> Option<f64> {
    if (c.p3 - c.p0).hypot() < MIN_CHORD {
        return None;
    }
    let min_turn = MIN_LOBE_TURN_DEG.to_radians();

    inflection_params(c)
        .into_iter()
        .filter(|&t| t > T_MARGIN && t < 1.0 - T_MARGIN)
        // Also clear of both endpoints in absolute distance.
        .filter(|&t| {
            let q = c.eval(t);
            (q - c.p0).hypot() >= MIN_SPLIT_DIST
                && (q - c.p3).hypot() >= MIN_SPLIT_DIST
        })
        .map(|t| (t, lobe_turn(c, 0.0, t).min(lobe_turn(c, t, 1.0))))
        .filter(|&(_, weakest)| weakest >= min_turn)
        // Anchor the most pronounced inflection (largest weakest-lobe turn).
        .max_by(|a, b| {
            a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal)
        })
        .map(|(t, _)| t)
}

/// Parameters in (0, 1) where signed curvature changes sign: the condition
/// `x'·y'' − y'·x'' = 0` is exactly quadratic in `t`, fitted from 3 samples.
fn inflection_params(c: &CubicBez) -> Vec<f64> {
    let d1 = c.deriv();
    let d2 = d1.deriv();
    let cross = |t: f64| {
        let a = d1.eval(t).to_vec2();
        let b = d2.eval(t).to_vec2();
        a.x * b.y - a.y * b.x
    };
    solve_quadratic_via_samples(cross)
}

/// Fit `f` (known to be quadratic in `t`) at t = 0, ½, 1 and return its real
/// roots in (0, 1). `f` is exactly quadratic, so three samples are exact.
fn solve_quadratic_via_samples(f: impl Fn(f64) -> f64) -> Vec<f64> {
    let f0 = f(0.0);
    let fh = f(0.5);
    let f1 = f(1.0);
    // f(t) = c2 t² + c1 t + c0 through (0,f0),(½,fh),(1,f1).
    let c0 = f0;
    let c2 = 2.0 * f0 - 4.0 * fh + 2.0 * f1;
    let c1 = f1 - f0 - c2;

    let mut roots = Vec::new();
    if c2.abs() < 1e-9 {
        if c1.abs() > 1e-12 {
            roots.push(-c0 / c1);
        }
    } else {
        let disc = c1 * c1 - 4.0 * c2 * c0;
        if disc >= 0.0 {
            let s = disc.sqrt();
            roots.push((-c1 + s) / (2.0 * c2));
            roots.push((-c1 - s) / (2.0 * c2));
        }
    }
    roots.retain(|&t| t > 0.0 && t < 1.0);
    roots
}

/// Total absolute tangent turning (radians) of the cubic over `[t0, t1]`.
fn lobe_turn(c: &CubicBez, t0: f64, t1: f64) -> f64 {
    const STEPS: usize = 32;
    let deriv = c.deriv();
    let mut total = 0.0;
    let mut prev = {
        let v = deriv.eval(t0).to_vec2();
        v.atan2()
    };
    for i in 1..=STEPS {
        let t = t0 + (t1 - t0) * i as f64 / STEPS as f64;
        let v = deriv.eval(t).to_vec2();
        let ang = v.atan2();
        let mut delta = ang - prev;
        while delta > std::f64::consts::PI {
            delta -= std::f64::consts::TAU;
        }
        while delta < -std::f64::consts::PI {
            delta += std::f64::consts::TAU;
        }
        total += delta.abs();
        prev = ang;
    }
    total
}

#[cfg(test)]
mod tests {
    use super::*;

    fn count_oncurve(path: &BezPath) -> usize {
        path.elements()
            .iter()
            .filter(|el| {
                matches!(
                    el,
                    PathEl::MoveTo(_)
                        | PathEl::LineTo(_)
                        | PathEl::CurveTo(..)
                        | PathEl::QuadTo(..)
                )
            })
            .count()
    }

    #[test]
    fn splits_strong_s_inflection() {
        // A long S-shaped cubic carrying an inflection.
        let mut path = BezPath::new();
        path.move_to(Point::new(300.0, 250.0));
        path.curve_to(
            Point::new(300.0, 150.0),
            Point::new(0.0, 350.0),
            Point::new(0.0, 0.0),
        );
        let out = split_inflections(&path, 0.0);
        assert_eq!(
            count_oncurve(&out),
            3,
            "the S cubic should gain one on-curve point at its inflection"
        );
    }

    #[test]
    fn leaves_plain_arc_alone() {
        // A quarter-circle-ish arc: curves one way only, no inflection.
        let mut path = BezPath::new();
        path.move_to(Point::new(100.0, 0.0));
        path.curve_to(
            Point::new(100.0, 55.0),
            Point::new(55.0, 100.0),
            Point::new(0.0, 100.0),
        );
        let out = split_inflections(&path, 0.0);
        assert_eq!(count_oncurve(&out), 2, "a plain arc must not be split");
    }

    #[test]
    fn leaves_tiny_wiggle_alone() {
        // A short segment with a faint inflection: below the chord/turn gates.
        let mut path = BezPath::new();
        path.move_to(Point::new(20.0, 10.0));
        path.curve_to(
            Point::new(15.0, 8.0),
            Point::new(5.0, 12.0),
            Point::new(0.0, 10.0),
        );
        let out = split_inflections(&path, 0.0);
        assert_eq!(count_oncurve(&out), 2, "a tiny wiggle must not be split");
    }

    #[test]
    fn split_preserves_curve_geometry() {
        let cubic = CubicBez::new(
            Point::new(300.0, 250.0),
            Point::new(300.0, 150.0),
            Point::new(0.0, 350.0),
            Point::new(0.0, 0.0),
        );
        let t = inflection_split_param(&cubic).expect("should find inflection");
        let left = cubic.subsegment(0.0..t);
        let right = cubic.subsegment(t..1.0);
        // The two halves must trace the same points as the original.
        for i in 0..=10 {
            let s = i as f64 / 10.0;
            let orig = cubic.eval(s);
            let rebuilt = if s <= t {
                left.eval(s / t)
            } else {
                right.eval((s - t) / (1.0 - t))
            };
            assert!(
                (orig - rebuilt).hypot() < 1e-6,
                "split must reproduce the curve at s={s}"
            );
        }
    }
}
