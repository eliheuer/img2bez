//! Curve fitting: annotated pixel contours → BezPaths.
//!
//! Per contour:
//! 1. Scale pixel coordinates to font units (Y-flip)
//! 2. Split at detected corners
//! 3. RDP-simplify each smooth segment
//! 4. Two-pass cubic bezier fitting via kurbo

use geo::{LineString, Simplify};
use kurbo::{fit_to_bezpath_opt, simplify::SimplifyBezPath, BezPath, PathEl, Point};

use crate::config::TracingConfig;
use crate::corners::AnnotatedContour;

/// Fit cubic curves to annotated contours.
///
/// Returns one `BezPath` per contour, in font units.
pub fn curves(
    contours: &[AnnotatedContour],
    image_height: u32,
    config: &TracingConfig,
) -> Vec<BezPath> {
    contours
        .iter()
        .map(|contour| fit_one(contour, image_height, config))
        .collect()
}

fn fit_one(contour: &AnnotatedContour, image_height: u32, config: &TracingConfig) -> BezPath {
    let n = contour.points.len();
    if n < 3 {
        return BezPath::new();
    }

    let scaled: Vec<(f64, f64)> = contour
        .points
        .iter()
        .map(|p| scale_point(p.x, p.y, image_height, config))
        .collect();

    let corners: Vec<usize> = contour
        .points
        .iter()
        .enumerate()
        .filter(|(_, p)| p.is_corner)
        .map(|(i, _)| i)
        .collect();

    if corners.is_empty() {
        return fit_smooth_closed(&scaled, config);
    }

    let mut path = BezPath::new();
    let first = scaled[corners[0]];
    path.move_to(Point::new(first.0, first.1));

    let num_corners = corners.len();
    for ci in 0..num_corners {
        let start = corners[ci];
        let end = corners[(ci + 1) % num_corners];
        let segment = extract_segment(&scaled, start, end, n);

        if segment.len() <= 2 {
            let p = scaled[end];
            path.line_to(Point::new(p.0, p.1));
            continue;
        }

        let smoothed = smooth_segment(&segment, config.smooth_iterations);
        let simplified = rdp_simplify(&smoothed, config.rdp_epsilon);
        let fitted = fit_segment(&simplified, config.fit_accuracy);

        for el in fitted.elements().iter().skip(1) {
            path.push(*el);
        }
    }

    path.push(PathEl::ClosePath);
    path
}

// ── Two-pass fitting ─────────────────────────────────────

/// Fit a smooth closed contour (no corners).
fn fit_smooth_closed(points: &[(f64, f64)], config: &TracingConfig) -> BezPath {
    let smoothed = smooth_closed(points, config.smooth_iterations);
    let simplified = rdp_simplify(&smoothed, config.rdp_epsilon);
    let path = points_to_path(&simplified, true);
    two_pass_fit(&path, config.fit_accuracy)
}

/// Fit cubics to an open segment between two corners.
fn fit_segment(points: &[(f64, f64)], accuracy: f64) -> BezPath {
    let path = points_to_path(points, false);
    two_pass_fit(&path, accuracy)
}

/// Two-pass fitting: polyline → curves → minimal curves.
///
/// Smooth curves simplify far better than noisy pixel
/// polylines, so a second pass dramatically reduces points.
fn two_pass_fit(path: &BezPath, accuracy: f64) -> BezPath {
    let pass1 = fit_to_bezpath_opt(
        &SimplifyBezPath::new(path.elements().iter().copied()),
        accuracy,
    );
    fit_to_bezpath_opt(
        &SimplifyBezPath::new(pass1.elements().iter().copied()),
        accuracy,
    )
}

/// Convert (x, y) tuples to a line-segment BezPath.
fn points_to_path(points: &[(f64, f64)], closed: bool) -> BezPath {
    let mut path = BezPath::new();
    if let Some(&(x, y)) = points.first() {
        path.move_to(Point::new(x, y));
        for &(x, y) in &points[1..] {
            path.line_to(Point::new(x, y));
        }
        if closed {
            path.push(PathEl::ClosePath);
        }
    }
    path
}

// ── Helpers ──────────────────────────────────────────────

/// Extract a cyclic sub-sequence from `start` to `end`.
fn extract_segment(
    points: &[(f64, f64)],
    start: usize,
    end: usize,
    total: usize,
) -> Vec<(f64, f64)> {
    let mut result = Vec::new();
    let mut i = start;
    loop {
        result.push(points[i]);
        if i == end {
            break;
        }
        i = (i + 1) % total;
    }
    result
}

/// RDP polyline simplification.
fn rdp_simplify(points: &[(f64, f64)], epsilon: f64) -> Vec<(f64, f64)> {
    if points.len() <= 2 || epsilon <= 0.0 {
        return points.to_vec();
    }
    LineString::from(points.to_vec())
        .simplify(&epsilon)
        .into_inner()
        .into_iter()
        .map(|coord| (coord.x, coord.y))
        .collect()
}

/// Smooth a closed polyline with neighbor averaging.
fn smooth_closed(points: &[(f64, f64)], iterations: usize) -> Vec<(f64, f64)> {
    if iterations == 0 || points.len() < 3 {
        return points.to_vec();
    }
    let mut smoothed = points.to_vec();
    let n = smoothed.len();
    for _ in 0..iterations {
        let previous = smoothed.clone();
        for i in 0..n {
            let a = previous[(i + n - 1) % n];
            let b = previous[i];
            let c = previous[(i + 1) % n];
            smoothed[i] = ((a.0 + b.0 + c.0) / 3.0, (a.1 + b.1 + c.1) / 3.0);
        }
    }
    smoothed
}

/// Smooth an open polyline, preserving endpoints.
fn smooth_segment(points: &[(f64, f64)], iterations: usize) -> Vec<(f64, f64)> {
    if iterations == 0 || points.len() < 3 {
        return points.to_vec();
    }
    let mut smoothed = points.to_vec();
    let n = smoothed.len();
    for _ in 0..iterations {
        let previous = smoothed.clone();
        for i in 1..n - 1 {
            let a = previous[i - 1];
            let b = previous[i];
            let c = previous[i + 1];
            smoothed[i] = ((a.0 + b.0 + c.0) / 3.0, (a.1 + b.1 + c.1) / 3.0);
        }
    }
    smoothed
}

/// Scale pixel coordinates to font units with Y-flip.
fn scale_point(px_x: f64, px_y: f64, image_height: u32, config: &TracingConfig) -> (f64, f64) {
    let height = image_height as f64;
    let scale = config.target_height / height;
    let x = px_x * scale;
    let y = (height - px_y) * scale + config.y_offset;
    (x, y)
}
