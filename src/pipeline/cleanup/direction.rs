// Copyright 2026 the img2bez Authors
// SPDX-License-Identifier: Apache-2.0 OR MIT

//! Contour winding correction: outer contours CCW, counters CW — the
//! cubic-outline convention UFO sources expect (compilers flip for quads).

use kurbo::{BezPath, PathEl, Point, flatten};

use crate::model::geom::signed_area;

/// Ensure outer contours are CCW, counters are CW. Nesting depth comes from
/// point-in-polygon testing: even depth = outer (CCW), odd = hole (CW).
pub fn fix_directions(paths: &[BezPath]) -> Vec<BezPath> {
    if paths.is_empty() {
        return vec![];
    }

    let polygons: Vec<Vec<Point>> =
        paths.iter().map(|p| flatten_to_polygon(p, 1.0)).collect();
    let areas: Vec<f64> = paths.iter().map(signed_area).collect();

    paths
        .iter()
        .enumerate()
        .map(|(i, path)| {
            // Majority vote over vertices tolerates points on a shared
            // boundary. The centroid is NOT a safe test point: a ring's
            // outer centroid falls inside its counter.
            let depth = (0..paths.len())
                .filter(|&j| {
                    j != i
                        && polygon_contains_majority(&polygons[i], &polygons[j])
                })
                .count();

            let should_be_ccw = depth % 2 == 0;
            let is_ccw = areas[i] > 0.0;
            if should_be_ccw != is_ccw {
                reverse_path(path)
            } else {
                path.clone()
            }
        })
        .collect()
}

/// Reverse a BezPath's winding direction.
fn reverse_path(path: &BezPath) -> BezPath {
    debug_assert!(!path.elements().is_empty());
    let mut first = Point::ZERO;
    let mut segments: Vec<(Point, Seg)> = Vec::new();
    let mut closed = false;

    for el in path.elements() {
        match *el {
            PathEl::MoveTo(p) => first = p,
            PathEl::LineTo(p) => {
                segments.push((p, Seg::Line));
            }
            PathEl::CurveTo(a, b, p) => {
                segments.push((p, Seg::Curve(a, b)));
            }
            PathEl::QuadTo(a, p) => {
                segments.push((p, Seg::Quad(a)));
            }
            PathEl::ClosePath => closed = true,
        }
    }

    let mut output = BezPath::new();
    if segments.is_empty() {
        output.move_to(first);
        if closed {
            output.push(PathEl::ClosePath);
        }
        return output;
    }

    output.move_to(segments[segments.len() - 1].0);

    for i in (0..segments.len()).rev() {
        let target = if i == 0 { first } else { segments[i - 1].0 };
        match segments[i].1 {
            Seg::Line => output.line_to(target),
            Seg::Curve(a, b) => {
                output.push(PathEl::CurveTo(b, a, target));
            }
            Seg::Quad(a) => {
                output.push(PathEl::QuadTo(a, target));
            }
        }
    }

    if closed {
        output.push(PathEl::ClosePath);
    }
    output
}

/// Flatten a BezPath to a polyline for point-in-polygon testing (on-curve
/// points alone fail when long cubics deviate far from their chord).
fn flatten_to_polygon(path: &BezPath, tolerance: f64) -> Vec<Point> {
    let mut points = Vec::new();
    flatten(path.elements().iter().copied(), tolerance, |el| match el {
        PathEl::MoveTo(p) | PathEl::LineTo(p) => points.push(p),
        _ => {}
    });
    points
}

/// True if more than half of `inner`'s vertices lie inside `outer` — valid
/// for non-crossing contours, where the vote only absorbs shared-boundary
/// vertices.
fn polygon_contains_majority(inner: &[Point], outer: &[Point]) -> bool {
    if inner.is_empty() {
        return false;
    }
    let inside = inner
        .iter()
        .filter(|&&p| point_in_polygon(p, outer))
        .count();
    inside * 2 > inner.len()
}

/// Ray-casting point-in-polygon test.
fn point_in_polygon(point: Point, polygon: &[Point]) -> bool {
    let n = polygon.len();
    if n < 3 {
        return false;
    }
    let mut inside = false;
    let mut j = n - 1;
    for i in 0..n {
        let pi = polygon[i];
        let pj = polygon[j];
        if ((pi.y > point.y) != (pj.y > point.y))
            && (point.x
                < (pj.x - pi.x) * (point.y - pi.y) / (pj.y - pi.y) + pi.x)
        {
            inside = !inside;
        }
        j = i;
    }
    inside
}

/// A path segment's type and control points, stored so `reverse_path` can
/// emit each segment backwards (cubics swap c1↔c2; the quad control stays).
#[derive(Clone, Copy)]
enum Seg {
    Line,
    Curve(Point, Point),
    Quad(Point),
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::model::geom::signed_area;
    use kurbo::{BezPath, PathEl, Point};

    #[test]
    fn ccw_contour_unchanged() {
        // CCW square: (0,0) → (100,0) → (100,100) → (0,100) → close
        let mut path = BezPath::new();
        path.move_to(Point::new(0.0, 0.0));
        path.line_to(Point::new(100.0, 0.0));
        path.line_to(Point::new(100.0, 100.0));
        path.line_to(Point::new(0.0, 100.0));
        path.push(PathEl::ClosePath);
        let area_before = signed_area(&path);
        assert!(area_before > 0.0, "Input should be CCW (positive area)");

        let result = fix_directions(&[path]);
        assert_eq!(result.len(), 1);
        let area_after = signed_area(&result[0]);
        assert!(
            area_after > 0.0,
            "Output should still be CCW (positive area)"
        );
    }

    #[test]
    fn nested_contours_correct_winding() {
        // Outer CCW square (centroid at 100,100)
        let mut outer = BezPath::new();
        outer.move_to(Point::new(0.0, 0.0));
        outer.line_to(Point::new(200.0, 0.0));
        outer.line_to(Point::new(200.0, 200.0));
        outer.line_to(Point::new(0.0, 200.0));
        outer.push(PathEl::ClosePath);
        // Inner CW square, offset so centroids differ (centroid at 150,150)
        let mut inner = BezPath::new();
        inner.move_to(Point::new(120.0, 120.0));
        inner.line_to(Point::new(120.0, 180.0));
        inner.line_to(Point::new(180.0, 180.0));
        inner.line_to(Point::new(180.0, 120.0));
        inner.push(PathEl::ClosePath);

        // Already correct: outer CCW, inner CW
        let result = fix_directions(&[outer, inner]);
        assert!(signed_area(&result[0]) > 0.0, "Outer should be CCW");
        assert!(signed_area(&result[1]) < 0.0, "Inner should be CW");

        // Swap windings (both wrong) and verify fix_directions corrects them
        let mut outer_wrong = BezPath::new();
        outer_wrong.move_to(Point::new(0.0, 0.0));
        outer_wrong.line_to(Point::new(0.0, 200.0));
        outer_wrong.line_to(Point::new(200.0, 200.0));
        outer_wrong.line_to(Point::new(200.0, 0.0));
        outer_wrong.push(PathEl::ClosePath);
        let mut inner_wrong = BezPath::new();
        inner_wrong.move_to(Point::new(120.0, 120.0));
        inner_wrong.line_to(Point::new(180.0, 120.0));
        inner_wrong.line_to(Point::new(180.0, 180.0));
        inner_wrong.line_to(Point::new(120.0, 180.0));
        inner_wrong.push(PathEl::ClosePath);

        let result2 = fix_directions(&[outer_wrong, inner_wrong]);
        assert!(signed_area(&result2[0]) > 0.0, "Fixed outer should be CCW");
        assert!(signed_area(&result2[1]) < 0.0, "Fixed inner should be CW");
    }
}
