use crate::config::TracingConfig;
use crate::contour::RawContour;

/// A contour with corner annotations on each point.
#[derive(Debug, Clone)]
pub struct AnnotatedContour {
    pub points: Vec<AnnotatedPoint>,
}

/// A point with a corner/smooth classification.
#[derive(Debug, Clone, Copy)]
pub struct AnnotatedPoint {
    pub x: f64,
    pub y: f64,
    pub is_corner: bool,
}

/// Detect corners in a set of raw contours.
///
/// Uses angle-based detection: at each point, the turning angle is computed
/// using neighboring points at distance `corner_window`. If the angle exceeds
/// `corner_angle_threshold`, the point is marked as a corner.
pub fn detect(contours: &[RawContour], config: &TracingConfig) -> Vec<AnnotatedContour> {
    contours
        .iter()
        .map(|c| {
            let points = annotate_corners(
                &c.points,
                config.corner_angle_threshold,
                config.corner_window,
            );
            AnnotatedContour { points }
        })
        .collect()
}

fn annotate_corners(points: &[(f64, f64)], threshold: f64, window: usize) -> Vec<AnnotatedPoint> {
    let n = points.len();
    if n < 3 {
        return points
            .iter()
            .map(|&(x, y)| AnnotatedPoint {
                x,
                y,
                is_corner: false,
            })
            .collect();
    }

    let w = window.min(n / 3).max(1);

    points
        .iter()
        .enumerate()
        .map(|(i, &(x, y))| {
            let prev = points[(i + n - w) % n];
            let next = points[(i + w) % n];

            let v_in = (x - prev.0, y - prev.1);
            let v_out = (next.0 - x, next.1 - y);

            let angle = angle_between(v_in, v_out);

            AnnotatedPoint {
                x,
                y,
                is_corner: angle > threshold,
            }
        })
        .collect()
}

/// Unsigned angle between two vectors, in radians [0, pi].
fn angle_between(a: (f64, f64), b: (f64, f64)) -> f64 {
    let dot = a.0 * b.0 + a.1 * b.1;
    let cross = a.0 * b.1 - a.1 * b.0;
    cross.atan2(dot).abs()
}
