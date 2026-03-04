//! Contour winding direction correction.
//!
//! Ensures outer contours wind CCW and counters (holes) wind CW,
//! which is the standard convention for TrueType/OpenType fonts.

use kurbo::{BezPath, PathEl, Point, flatten};

use crate::geom::signed_area;

/// Ensure outer contours are CCW, counters are CW.
///
/// Determines nesting by point-in-polygon testing: contours nested
/// inside an even number of others are outer (CCW), odd = hole (CW).
/// This correctly handles multiple independent outer contours (e.g.
/// separate letters in a logo) as well as nested counters.
pub fn fix_directions(paths: &[BezPath]) -> Vec<BezPath> {
    if paths.is_empty() {
        return vec![];
    }

    let polygons: Vec<Vec<Point>> = paths.iter().map(|p| flatten_to_polygon(p, 1.0)).collect();
    let areas: Vec<f64> = paths.iter().map(signed_area).collect();

    paths
        .iter()
        .enumerate()
        .map(|(i, path)| {
            // Use the centroid as the test point — it's reliably inside
            // the contour, unlike the first point which may sit on a
            // shared boundary (e.g. O counter extremum = O outer extremum).
            let test_point = centroid(&polygons[i]);

            // Count how many OTHER contours contain this point.
            let depth = (0..paths.len())
                .filter(|&j| j != i && point_in_polygon(test_point, &polygons[j]))
                .count();

            // Even depth = outer (CCW, area > 0), odd depth = hole (CW, area < 0).
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

/// Flatten a BezPath to a polyline for accurate point-in-polygon testing.
///
/// Using only on-curve points fails when long cubics deviate far from
/// the chord between endpoints.  Flattening within a tolerance gives a
/// polygon that faithfully follows the actual curve.
fn flatten_to_polygon(path: &BezPath, tolerance: f64) -> Vec<Point> {
    let mut points = Vec::new();
    flatten(path.elements().iter().copied(), tolerance, |el| match el {
        PathEl::MoveTo(p) | PathEl::LineTo(p) => points.push(p),
        _ => {}
    });
    points
}

/// Centroid (mean of all points) of a polygon.
fn centroid(points: &[Point]) -> Point {
    if points.is_empty() {
        return Point::ZERO;
    }
    let n = points.len() as f64;
    let sum_x: f64 = points.iter().map(|p| p.x).sum();
    let sum_y: f64 = points.iter().map(|p| p.y).sum();
    Point::new(sum_x / n, sum_y / n)
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
            && (point.x < (pj.x - pi.x) * (point.y - pi.y) / (pj.y - pi.y) + pi.x)
        {
            inside = !inside;
        }
        j = i;
    }
    inside
}

/// A path segment's type and control points (used for path reversal).
///
/// When reversing a path, line segments simply swap endpoints. Cubic
/// curves swap endpoints and reverse their control point order (c1↔c2).
/// Quadratic curves swap endpoints; the single control point stays.
#[derive(Clone, Copy)]
enum Seg {
    Line,
    Curve(Point, Point),
    Quad(Point),
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geom::signed_area;
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

        // Now swap windings (both wrong) and verify fix_directions corrects them
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
