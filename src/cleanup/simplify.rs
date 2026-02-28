//! Path simplification: redundant segment removal.
//!
//! Converts near-straight curves to lines, merges collinear
//! line segments, and removes segments too short to matter.

use kurbo::{BezPath, CubicBez, ParamCurve, PathEl, Point, Vec2};

/// Number of sample points along a curve for straightness checks.
const CURVE_SAMPLES: usize = 8;

/// Convert near-straight curves to line segments.
///
/// Samples 8 points along the cubic and converts to a line if the maximum
/// deviation from the chord is within tolerance.  This catches cases the
/// old handle-distance check missed (handles off-chord but curve still
/// visually straight).
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
                if curve_is_straight(current, a, b, p, tolerance) {
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

/// Check if a cubic curve deviates from its chord by less than `tolerance`.
fn curve_is_straight(p0: Point, p1: Point, p2: Point, p3: Point, tolerance: f64) -> bool {
    let cubic = CubicBez::new(p0, p1, p2, p3);
    for i in 1..CURVE_SAMPLES {
        let t = i as f64 / CURVE_SAMPLES as f64;
        let pt = cubic.eval(t);
        if point_to_line_dist(pt, p0, p3) > tolerance {
            return false;
        }
    }
    true
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
