//! Shared geometry utilities.

use kurbo::{BezPath, PathEl, Point};

/// Signed area of a BezPath via the shoelace formula (on-curve points only).
///
/// Positive = counter-clockwise, negative = clockwise.
pub fn signed_area(path: &BezPath) -> f64 {
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
