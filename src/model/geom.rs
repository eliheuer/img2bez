// Copyright 2026 the img2bez Authors
// SPDX-License-Identifier: Apache-2.0 OR MIT

//! Shared geometry utilities.

use kurbo::{BezPath, PathEl, Point, Vec2};

/// Signed area via the shoelace formula over on-curve points only: correct
/// winding for direction detection (positive = CCW, negative = CW), though it
/// underestimates the true area of curved segments.
#[inline]
pub fn signed_area(path: &BezPath) -> f64 {
    debug_assert!(!path.elements().is_empty());
    let mut area = 0.0;
    let mut first = Point::ZERO;
    let mut current = Point::ZERO;
    for el in path.elements() {
        match *el {
            PathEl::MoveTo(p) => {
                first = p;
                current = p;
            }
            PathEl::LineTo(p)
            | PathEl::CurveTo(_, _, p)
            | PathEl::QuadTo(_, p) => {
                area += current.x * p.y - p.x * current.y;
                current = p;
            }
            PathEl::ClosePath => {
                area += current.x * first.y - first.x * current.y;
            }
        }
    }
    area / 2.0
}

/// A straight line `a`→`b` expressed as the degenerate cubic
/// `[a, a + (b-a)/3, a + 2(b-a)/3, b]` used by the curve-fitting pipelines.
#[inline]
pub fn line_seg(a: Point, b: Point) -> [Point; 4] {
    [a, a.lerp(b, 1.0 / 3.0), a.lerp(b, 2.0 / 3.0), b]
}

/// The "magic triangle" of a curve segment: the two on-curve nodes plus the
/// intersection of the handle elongations; a well-drawn cubic keeps each
/// handle inside it. Given unit handle directions `u0`/`u1` (node toward
/// off-curve), returns each node's distance to the intersection — the maximum
/// legal handle lengths. `None` when the rays don't converge ahead of both
/// nodes (parallel tangents or an inflected segment — the rule doesn't apply).
#[inline]
pub(crate) fn handle_triangle(
    a: Point,
    u0: Vec2,
    b: Point,
    u1: Vec2,
) -> Option<(f64, f64)> {
    let denom = u0.cross(u1);
    if denom.abs() < 1e-9 {
        return None;
    }
    let d = b - a;
    let t = d.cross(u1) / denom;
    let s = d.cross(u0) / denom;
    (t > 0.0 && s > 0.0).then_some((t, s))
}

/// Minimum fraction of the chord a handle's span must cover for the
/// proportion rule to apply; below it (handle near-perpendicular to the
/// chord) the span carries no aspect information.
const SPAN_MIN_FRAC: f64 = 0.15;

/// Chord extent along each handle direction — the stretch of curve each
/// handle must cover. The type-design proportion rule: a handle's length
/// relates to its span, not to the other handle, so "balanced" means
/// `l0/s0 == l1/s1`, not `l0 == l1`. `u0`/`u1` are unit handle directions
/// (node toward off-curve). `None` when either span is under
/// [`SPAN_MIN_FRAC`] of the chord.
#[inline]
pub(crate) fn handle_spans(
    a: Point,
    u0: Vec2,
    b: Point,
    u1: Vec2,
) -> Option<(f64, f64)> {
    let d = b - a;
    let chord = d.hypot();
    if chord < 1e-9 {
        return None;
    }
    let s0 = d.dot(u0).abs();
    let s1 = d.dot(u1).abs();
    (s0 > chord * SPAN_MIN_FRAC && s1 > chord * SPAN_MIN_FRAC)
        .then_some((s0, s1))
}

#[cfg(test)]
mod tests {
    use super::*;
    use kurbo::{BezPath, PathEl, Point};

    #[test]
    fn signed_area_detects_winding() {
        // CCW square: (0,0) → (100,0) → (100,100) → (0,100) → close
        let mut ccw = BezPath::new();
        ccw.move_to(Point::new(0.0, 0.0));
        ccw.line_to(Point::new(100.0, 0.0));
        ccw.line_to(Point::new(100.0, 100.0));
        ccw.line_to(Point::new(0.0, 100.0));
        ccw.push(PathEl::ClosePath);
        assert!(
            signed_area(&ccw) > 0.0,
            "CCW square should have positive area"
        );

        // CW square: reverse
        let mut cw = BezPath::new();
        cw.move_to(Point::new(0.0, 0.0));
        cw.line_to(Point::new(0.0, 100.0));
        cw.line_to(Point::new(100.0, 100.0));
        cw.line_to(Point::new(100.0, 0.0));
        cw.push(PathEl::ClosePath);
        assert!(
            signed_area(&cw) < 0.0,
            "CW square should have negative area"
        );
    }
}
