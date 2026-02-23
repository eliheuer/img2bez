use crate::config::TracingConfig;
use crate::trace::RawContour;

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

/// Annotate corners in a set of raw contours.
///
/// At each point, the turning angle is computed using neighboring
/// points at distance `corner_window`. If the angle exceeds
/// `corner_angle_threshold`, the point is marked as a corner.
pub fn annotate(contours: &[RawContour], config: &TracingConfig) -> Vec<AnnotatedContour> {
    contours
        .iter()
        .map(|contour| {
            let (points, angles) = annotate_corners(
                &contour.points,
                config.corner_angle_threshold,
                config.corner_window,
            );
            let points = suppress_corners(points, &angles);
            AnnotatedContour { points }
        })
        .collect()
}

/// Annotate each point as corner/smooth and return the detection angles.
fn annotate_corners(
    points: &[(f64, f64)],
    threshold: f64,
    window: usize,
) -> (Vec<AnnotatedPoint>, Vec<f64>) {
    let n = points.len();
    if n < 3 {
        let pts = points
            .iter()
            .map(|&(x, y)| AnnotatedPoint {
                x,
                y,
                is_corner: false,
            })
            .collect();
        return (pts, vec![0.0; n]);
    }

    let window_size = window.min(n / 3).max(1);

    let mut annotated = Vec::with_capacity(n);
    let mut angles = Vec::with_capacity(n);

    for (i, &(x, y)) in points.iter().enumerate() {
        let prev = points[(i + n - window_size) % n];
        let next = points[(i + window_size) % n];

        let v_in = (x - prev.0, y - prev.1);
        let v_out = (next.0 - x, next.1 - y);

        let angle = angle_between(v_in, v_out);
        angles.push(angle);

        annotated.push(AnnotatedPoint {
            x,
            y,
            is_corner: angle > threshold,
        });
    }

    (annotated, angles)
}

/// Non-maximum suppression: for each run of consecutive corners,
/// keep only the sharpest one (largest turning angle).
///
/// Uses the same windowed angles from detection for consistent ranking.
fn suppress_corners(points: Vec<AnnotatedPoint>, angles: &[f64]) -> Vec<AnnotatedPoint> {
    let n = points.len();
    if n < 3 {
        return points;
    }

    let mut result = points;

    let mut i = 0;
    while i < n {
        if !result[i].is_corner {
            i += 1;
            continue;
        }

        // Found start of a corner run. Find its extent.
        let run_start = i;
        let mut run_end = i;
        while run_end + 1 < n && result[run_end + 1].is_corner {
            run_end += 1;
        }

        // If run is more than one corner, keep only the sharpest.
        if run_end > run_start {
            let mut best = run_start;
            for j in run_start..=run_end {
                if angles[j] > angles[best] {
                    best = j;
                }
            }
            for (j, point) in result.iter_mut().enumerate().take(run_end + 1).skip(run_start) {
                if j != best {
                    point.is_corner = false;
                }
            }
        }

        i = run_end + 1;
    }

    result
}

/// Unsigned angle between two vectors, in radians [0, pi].
fn angle_between(a: (f64, f64), b: (f64, f64)) -> f64 {
    let dot = a.0 * b.0 + a.1 * b.1;
    let cross = a.0 * b.1 - a.1 * b.0;
    cross.atan2(dot).abs()
}
