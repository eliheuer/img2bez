//! Path simplification: redundant segment removal.
//!
//! Converts near-straight curves to lines, merges collinear
//! line segments, and removes segments too short to matter.

use kurbo::{BezPath, PathEl, Point, Vec2};

/// Convert curves where both handles hug the chord.
pub fn curves_to_lines(path: &BezPath, tolerance: f64) -> BezPath {
    let mut output = BezPath::new();
    let mut current = Point::ZERO;

    for el in path.elements() {
        match *el {
            PathEl::MoveTo(p) => {
                output.move_to(p);
                current = p;
            }
            PathEl::CurveTo(a, b, p) => {
                let d1 = point_to_line_dist(a, current, p);
                let d2 = point_to_line_dist(b, current, p);
                if d1 < tolerance && d2 < tolerance {
                    output.line_to(p);
                } else {
                    output.push(PathEl::CurveTo(a, b, p));
                }
                current = p;
            }
            PathEl::LineTo(p) => {
                output.line_to(p);
                current = p;
            }
            other => output.push(other),
        }
    }
    output
}

/// Merge consecutive collinear line segments.
pub fn merge_collinear(path: &BezPath, tolerance: f64) -> BezPath {
    let mut output = BezPath::new();
    let mut anchor = Point::ZERO;
    let mut pending: Option<Point> = None;

    for el in path.elements() {
        match *el {
            PathEl::MoveTo(p) => {
                flush(&mut output, &mut pending);
                output.move_to(p);
                anchor = p;
            }
            PathEl::LineTo(p) => match pending {
                Some(mid) if point_to_line_dist(mid, anchor, p) < tolerance => {
                    pending = Some(p);
                }
                Some(mid) => {
                    output.line_to(mid);
                    anchor = mid;
                    pending = Some(p);
                }
                None => {
                    pending = Some(p);
                }
            },
            PathEl::CurveTo(a, b, p) => {
                flush(&mut output, &mut pending);
                output.push(PathEl::CurveTo(a, b, p));
                anchor = p;
            }
            PathEl::QuadTo(a, p) => {
                flush(&mut output, &mut pending);
                output.push(PathEl::QuadTo(a, p));
                anchor = p;
            }
            PathEl::ClosePath => {
                flush(&mut output, &mut pending);
                output.push(PathEl::ClosePath);
            }
        }
    }
    flush(&mut output, &mut pending);
    output
}

/// Remove segments shorter than `tolerance`.
pub fn remove_tiny(path: &BezPath, tolerance: f64) -> BezPath {
    let mut output = BezPath::new();
    let mut current = Point::ZERO;
    let tolerance_sq = tolerance * tolerance;

    for el in path.elements() {
        match *el {
            PathEl::MoveTo(p) => {
                output.move_to(p);
                current = p;
            }
            PathEl::LineTo(p) => {
                if dist_sq(current, p) >= tolerance_sq {
                    output.line_to(p);
                    current = p;
                }
            }
            PathEl::CurveTo(a, b, p) => {
                if dist_sq(current, p) >= tolerance_sq {
                    output.push(PathEl::CurveTo(a, b, p));
                    current = p;
                }
            }
            PathEl::QuadTo(a, p) => {
                if dist_sq(current, p) >= tolerance_sq {
                    output.push(PathEl::QuadTo(a, p));
                    current = p;
                }
            }
            PathEl::ClosePath => {
                output.push(PathEl::ClosePath);
            }
        }
    }
    output
}

/// Distance from point P to line through A→B.
fn point_to_line_dist(p: Point, a: Point, b: Point) -> f64 {
    let ab = Vec2::new(b.x - a.x, b.y - a.y);
    let ap = Vec2::new(p.x - a.x, p.y - a.y);
    let len_sq = ab.x * ab.x + ab.y * ab.y;
    if len_sq < 1e-10 {
        return ap.hypot();
    }
    (ab.x * ap.y - ab.y * ap.x).abs() / len_sq.sqrt()
}

fn flush(output: &mut BezPath, pending: &mut Option<Point>) {
    if let Some(p) = pending.take() {
        output.line_to(p);
    }
}

fn dist_sq(a: Point, b: Point) -> f64 {
    (a.x - b.x).powi(2) + (a.y - b.y).powi(2)
}
