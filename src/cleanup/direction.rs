//! Contour winding direction correction.
//!
//! Ensures outer contours wind CCW and counters (holes) wind CW,
//! which is the standard convention for TrueType/OpenType fonts.

use kurbo::{BezPath, PathEl, Point};

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

    let polygons: Vec<Vec<Point>> = paths.iter().map(on_curve_points).collect();
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

/// Signed area via shoelace (on-curve points only).
fn signed_area(path: &BezPath) -> f64 {
    let mut area = 0.0;
    let mut first = Point::ZERO;
    let mut current = Point::ZERO;

    for el in path.elements() {
        match *el {
            PathEl::MoveTo(p) => {
                first = p;
                current = p;
            }
            PathEl::LineTo(p) | PathEl::CurveTo(_, _, p) | PathEl::QuadTo(_, p) => {
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

/// Reverse a BezPath's winding direction.
fn reverse_path(path: &BezPath) -> BezPath {
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

    output.move_to(segments.last().unwrap().0);

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

/// Extract on-curve points from a BezPath as a polygon.
fn on_curve_points(path: &BezPath) -> Vec<Point> {
    path.elements()
        .iter()
        .filter_map(|el| match *el {
            PathEl::MoveTo(p)
            | PathEl::LineTo(p)
            | PathEl::CurveTo(_, _, p)
            | PathEl::QuadTo(_, p) => Some(p),
            PathEl::ClosePath => None,
        })
        .collect()
}

/// Centroid (mean of all points) of a polygon.
fn centroid(points: &[Point]) -> Point {
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

#[derive(Clone, Copy)]
enum Seg {
    Line,
    Curve(Point, Point),
    Quad(Point),
}
