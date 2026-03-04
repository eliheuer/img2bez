//! Shared geometry utilities.

use kurbo::{BezPath, PathEl, Point};

/// Signed area of a BezPath via the shoelace formula (on-curve points only).
///
/// Positive = counter-clockwise, negative = clockwise.
///
/// Uses only on-curve points (ignoring off-curve control points), which
/// gives the correct winding for direction detection even though it
/// slightly underestimates the true area of curved segments.
#[inline]
pub fn signed_area(path: &BezPath) -> f64 {
    debug_assert!(!path.elements().is_empty());
    // Shoelace formula: sum of (x_i * y_{i+1} - x_{i+1} * y_i) / 2
    // for each pair of consecutive on-curve points, including the
    // closing edge from the last point back to the first.
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
        assert!(signed_area(&ccw) > 0.0, "CCW square should have positive area");

        // CW square: reverse
        let mut cw = BezPath::new();
        cw.move_to(Point::new(0.0, 0.0));
        cw.line_to(Point::new(0.0, 100.0));
        cw.line_to(Point::new(100.0, 100.0));
        cw.line_to(Point::new(100.0, 0.0));
        cw.push(PathEl::ClosePath);
        assert!(signed_area(&cw) < 0.0, "CW square should have negative area");
    }
}
