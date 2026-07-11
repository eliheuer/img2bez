// Copyright 2026 the img2bez Authors
// SPDX-License-Identifier: Apache-2.0 OR MIT

//! Even out lopsided handle lengths without changing the curve shape:
//! lengths move toward each other along their existing directions (on-curve
//! points, directions, and smoothness preserved) within a small deviation
//! budget.

use kurbo::{BezPath, PathEl, Point, Vec2};

/// Only touch a segment whose span-normalized (geom::handle_spans) handle
/// ratio exceeds this: balanced quadrants ~1.1, mildly inverted pairs ~1.7.
const RATIO_TRIGGER: f64 = 1.35;
/// Shape budget (absolute floor / chord fraction). Raster-blind pass: must
/// stay small or it repaints ink-hugging curves.
const MAX_DEVIATION_ABS: f64 = 2.5;
const MAX_DEVIATION_FRAC: f64 = 0.01;
/// Skip segments shorter than this chord — too small to matter.
const MIN_CHORD: f64 = 30.0;

/// Handles shorter than this (font units) are below integer-rounding
/// resolution; evening them trades real G1 for cosmetic proportion.
const MIN_HANDLE: f64 = 8.0;
/// Grid resolution for the handle-length search.
const GRID: usize = 28;
/// Max combined chord-projected handle reach (chord fraction; past 1.0 the
/// handles cross). Same invariant/value as `refine::HANDLE_REACH_MAX`.
const HANDLE_REACH_MAX: f64 = 0.9;
/// Samples used to measure curve deviation.
const SAMPLES: usize = 24;

/// Enforce the well-drawn handle invariants on every cubic: the magic
/// triangle (no handle past the tangent intersection) and no crossing
/// (reach ≤ `HANDLE_REACH_MAX`). Only lengths shrink, so smoothness survives.
///
/// Must be the LAST geometric pass: snap/inflection rotate handle directions
/// and can push a legal handle outside its new triangle. No deviation budget
/// — a handle outside the triangle is a bulge or cusp by construction.
pub fn cap_handles(path: &BezPath) -> BezPath {
    let mut out = BezPath::new();
    let mut cur = Point::ZERO;
    for el in path.elements() {
        match *el {
            PathEl::CurveTo(c1, c2, p) => {
                let (mut h1, mut h2) = (c1 - cur, c2 - p);
                let (l1, l2) = (h1.hypot(), h2.hypot());
                let chord_v = p - cur;
                let chord = chord_v.hypot();
                if l1 > 1e-9 && l2 > 1e-9 && chord > 1e-9 {
                    if let Some((t, s)) = crate::model::geom::handle_triangle(
                        cur,
                        h1 / l1,
                        p,
                        h2 / l2,
                    ) {
                        if l1 > t {
                            h1 *= t / l1;
                        }
                        if l2 > s {
                            h2 *= s / l2;
                        }
                    }
                    let e = chord_v / chord;
                    let reach = h1.dot(e) - h2.dot(e);
                    let limit = chord * HANDLE_REACH_MAX;
                    if reach > limit {
                        let k = limit / reach;
                        h1 *= k;
                        h2 *= k;
                    }
                }
                out.curve_to(cur + h1, p + h2, p);
                cur = p;
            }
            other => {
                out.push(other);
                if let Some(p) = other.end_point() {
                    cur = p;
                }
            }
        }
    }
    out
}

/// Even out lopsided handle lengths across every cubic in the path.
pub fn even_handles(path: &BezPath) -> BezPath {
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
                let (nc1, nc2) = even_segment(cur, c1, c2, p);
                out.curve_to(nc1, nc2, p);
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

/// Return possibly-rebalanced control points for one cubic.
fn even_segment(a: Point, c1: Point, c2: Point, b: Point) -> (Point, Point) {
    let v0 = c1 - a;
    let v1 = c2 - b;
    let (l0, l1) = (v0.hypot(), v1.hypot());
    // Both handles sub-resolution = tiny cubic, integer noise, leave it.
    // ONE degenerate handle on a real chord is the target case; the search
    // floor keeps outputs at or above MIN_HANDLE.
    if (b - a).hypot() < MIN_CHORD || (l0 < MIN_HANDLE && l1 < MIN_HANDLE) {
        return (c1, c2);
    }
    // A zero-length handle has no direction; normalizing it would emit NaN
    // (and the deviation metric's f64::max folds NaN away silently).
    if l0 < 1e-9 || l1 < 1e-9 {
        return (c1, c2);
    }
    let (u0, u1) = (v0 / l0, v1 / l1);
    // "Even" is judged span-normalized (geom::handle_spans): a tall
    // counter's long-vertical/short-horizontal pair is already balanced.
    let spans = crate::model::geom::handle_spans(a, u0, b, u1);
    let norm_ratio = |la: f64, lb: f64| -> f64 {
        let (na, nb) = match spans {
            Some((s0, s1)) => (la / s0, lb / s1),
            None => (la, lb),
        };
        na.max(nb) / na.min(nb)
    };
    if norm_ratio(l0, l1) < RATIO_TRIGGER {
        return (c1, c2);
    }
    let orig = sample(a, c1, c2, b);
    let dev = |la: f64, lb: f64| -> f64 {
        let cand = sample(a, a + u0 * la, b + u1 * lb, b);
        orig.iter()
            .zip(&cand)
            .map(|(o, c)| (*o - *c).hypot())
            .fold(0.0, f64::max)
    };

    // Search length pairs for the most even (then least-deviating) candidate
    // within the shape budget.
    let budget = MAX_DEVIATION_ABS.max((b - a).hypot() * MAX_DEVIATION_FRAC);
    let chord_v = b - a;
    let chord = chord_v.hypot();
    let e = chord_v * (1.0 / chord);
    let reach_limit = chord * HANDLE_REACH_MAX;
    // Original pair stays the fallback; every searched candidate must
    // respect the reach cap.
    let lo = (l0.min(l1) * 0.5).max(MIN_HANDLE);
    let hi = l0.max(l1) * 1.3;
    if hi <= lo {
        return (c1, c2);
    }
    // Magic triangle bound: no handle past the tangent intersection.
    let tri = crate::model::geom::handle_triangle(a, u0, b, u1);
    let mut best = (l0, l1);
    let mut best_ratio = norm_ratio(l0, l1);
    let mut best_dev = 0.0;
    for i in 0..=GRID {
        let la = lo + (hi - lo) * i as f64 / GRID as f64;
        for j in 0..=GRID {
            let lb = lo + (hi - lo) * j as f64 / GRID as f64;
            if la * u0.dot(e) - lb * u1.dot(e) > reach_limit {
                continue;
            }
            if let Some((t, s)) = tri
                && (la > t || lb > s)
            {
                continue;
            }
            let d = dev(la, lb);
            if d > budget {
                continue;
            }
            let ratio = norm_ratio(la, lb);
            if ratio < best_ratio - 1e-6
                || (ratio <= best_ratio + 1e-6 && d < best_dev)
            {
                best = (la, lb);
                best_ratio = ratio;
                best_dev = d;
            }
        }
    }

    (a + u0 * best.0, b + u1 * best.1)
}

/// Sample a cubic at `SAMPLES + 1` uniform parameters.
fn sample(a: Point, c1: Point, c2: Point, b: Point) -> Vec<Point> {
    (0..=SAMPLES)
        .map(|k| {
            let t = k as f64 / SAMPLES as f64;
            let mt = 1.0 - t;
            let v: Vec2 = a.to_vec2() * (mt * mt * mt)
                + c1.to_vec2() * (3.0 * mt * mt * t)
                + c2.to_vec2() * (3.0 * mt * t * t)
                + b.to_vec2() * (t * t * t);
            v.to_point()
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn evens_lopsided_handle_within_tolerance() {
        // Quarter-arc with a ~9:1 handle split; both handles stay inside the
        // magic triangle so evening is allowed to act.
        let a = Point::new(0.0, 0.0);
        let c1 = Point::new(90.0, 0.0);
        let c2 = Point::new(100.0, 90.0);
        let b = Point::new(100.0, 100.0);
        let (nc1, nc2) = even_segment(a, c1, c2, b);
        let l0 = (nc1 - a).hypot();
        let l1 = (nc2 - b).hypot();
        let before = (c1 - a).hypot() / (c2 - b).hypot();
        let after = l0.max(l1) / l0.min(l1);
        assert!(
            after < before,
            "ratio should improve: {before:.1} -> {after:.1}"
        );
        // Directions preserved.
        assert!((nc1 - a).normalize().dot((c1 - a).normalize()) > 0.999);
        assert!((nc2 - b).normalize().dot((c2 - b).normalize()) > 0.999);
    }

    #[test]
    fn leaves_even_handles_alone() {
        let a = Point::new(0.0, 0.0);
        let c1 = Point::new(0.0, 55.0);
        let c2 = Point::new(45.0, 100.0);
        let b = Point::new(100.0, 100.0);
        let (nc1, nc2) = even_segment(a, c1, c2, b);
        assert_eq!((nc1, nc2), (c1, c2), "near-even handles must be untouched");
    }
}
