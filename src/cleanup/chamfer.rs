//! Chamfer insertion at line-line corners.
//!
//! Replaces sharp corners with 45-degree bevels, which is
//! the defining visual feature of Virtua Grotesk.

use kurbo::{BezPath, PathEl, Point, Vec2};

/// Skip chamfering when edges are nearly collinear.
///
/// cos(18°) ≈ 0.951. Corners whose edges form an angle of less than
/// ~18° from straight (i.e., cos > 0.95) are too shallow to chamfer
/// meaningfully and would produce barely-visible bevels.
const COLLINEAR_THRESHOLD: f64 = 0.95;

/// Add 45° chamfers at line-line corners.
pub fn chamfer(path: &BezPath, size: f64, min_edge: f64) -> BezPath {
    let mut first = Point::ZERO;
    let mut points: Vec<(Point, bool)> = Vec::new();
    let mut closed = false;

    for el in path.elements() {
        match *el {
            PathEl::MoveTo(p) => first = p,
            PathEl::LineTo(p) => points.push((p, true)),
            PathEl::CurveTo(_, _, p) | PathEl::QuadTo(_, p) => points.push((p, false)),
            PathEl::ClosePath => closed = true,
        }
    }

    if points.is_empty() {
        return path.clone();
    }

    let mut all = vec![(first, true)];
    all.extend_from_slice(&points);
    let n = all.len();

    let mut new_points: Vec<Point> = Vec::new();
    for i in 0..n {
        let (point, is_line) = all[i];
        let (prev, _) = all[(i + n - 1) % n];
        let (next, next_line) = all[(i + 1) % n];

        if !is_line || !next_line {
            new_points.push(point);
            continue;
        }

        let v_in = Vec2::new(point.x - prev.x, point.y - prev.y);
        let v_out = Vec2::new(next.x - point.x, next.y - point.y);
        let len_in = v_in.hypot();
        let len_out = v_out.hypot();

        if len_in < min_edge || len_out < min_edge {
            new_points.push(point);
            continue;
        }

        let dot = v_in.x * v_out.x + v_in.y * v_out.y;
        let cos = dot / (len_in * len_out);
        if cos.abs() > COLLINEAR_THRESHOLD {
            new_points.push(point);
            continue;
        }

        new_points.push(Point::new(
            point.x - size * v_in.x / len_in,
            point.y - size * v_in.y / len_in,
        ));
        new_points.push(Point::new(
            point.x + size * v_out.x / len_out,
            point.y + size * v_out.y / len_out,
        ));
    }

    let mut output = BezPath::new();
    if let Some(&p) = new_points.first() {
        output.move_to(p);
        for &p in &new_points[1..] {
            output.line_to(p);
        }
        if closed {
            output.push(PathEl::ClosePath);
        }
    }
    output
}

#[cfg(test)]
mod tests {
    use super::*;
    use kurbo::{BezPath, PathEl, Point};

    #[test]
    fn right_angle_gets_chamfered() {
        // Square with edges > min_edge
        let mut path = BezPath::new();
        path.move_to(Point::new(0.0, 0.0));
        path.line_to(Point::new(100.0, 0.0));
        path.line_to(Point::new(100.0, 100.0));
        path.line_to(Point::new(0.0, 100.0));
        path.push(PathEl::ClosePath);

        let input_points = path
            .elements()
            .iter()
            .filter(|el| matches!(el, PathEl::MoveTo(_) | PathEl::LineTo(_)))
            .count();

        let result = chamfer(&path, 10.0, 5.0);
        let output_points = result
            .elements()
            .iter()
            .filter(|el| matches!(el, PathEl::MoveTo(_) | PathEl::LineTo(_)))
            .count();

        assert!(
            output_points > input_points,
            "Chamfered square should have more points ({}) than input ({})",
            output_points,
            input_points
        );
    }
}
