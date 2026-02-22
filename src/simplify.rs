use geo::{LineString, Simplify};
use kurbo::{fit_to_bezpath_opt, simplify::SimplifyBezPath, BezPath, PathEl, Point};

use crate::config::TracingConfig;
use crate::corners::AnnotatedContour;

/// Fit cubic bezier curves to annotated contours.
///
/// The pipeline:
/// 1. Scale pixel coordinates to font units (with Y-flip)
/// 2. Split each contour at detected corners
/// 3. RDP-simplify each smooth segment
/// 4. Fit optimal cubic beziers via kurbo
///
/// `image_height` is the source image height in pixels, used for Y-flip.
/// Returns one `BezPath` per input contour.
pub fn fit(
    contours: &[AnnotatedContour],
    image_height: u32,
    config: &TracingConfig,
) -> Vec<BezPath> {
    contours
        .iter()
        .map(|c| fit_one(c, image_height, config))
        .collect()
}

fn fit_one(contour: &AnnotatedContour, image_height: u32, config: &TracingConfig) -> BezPath {
    let n = contour.points.len();
    if n < 3 {
        return BezPath::new();
    }

    // Step 1: Scale all points to font units
    let scaled: Vec<(f64, f64)> = contour
        .points
        .iter()
        .map(|p| scale_point(p.x, p.y, image_height, config))
        .collect();

    // Step 2: Find corner indices
    let corner_indices: Vec<usize> = contour
        .points
        .iter()
        .enumerate()
        .filter(|(_, p)| p.is_corner)
        .map(|(i, _)| i)
        .collect();

    // No corners: fit the entire contour as one smooth closed curve
    if corner_indices.is_empty() {
        return fit_smooth_closed(&scaled, config);
    }

    // Step 3: Build BezPath by walking corner-to-corner segments
    let mut path = BezPath::new();
    let first = scaled[corner_indices[0]];
    path.move_to(Point::new(first.0, first.1));

    let nc = corner_indices.len();
    for ci in 0..nc {
        let start_idx = corner_indices[ci];
        let end_idx = corner_indices[(ci + 1) % nc];

        // Extract the segment of points between these two corners
        let segment = extract_segment(&scaled, start_idx, end_idx, n);

        if segment.len() <= 2 {
            // Straight line to next corner
            let p = scaled[end_idx];
            path.line_to(Point::new(p.0, p.1));
        } else {
            // Smooth, RDP simplify, then fit curves
            let smoothed = smooth_segment(&segment, config.smooth_iterations);
            let simplified = rdp_simplify(&smoothed, config.rdp_epsilon);
            let fitted = fit_segment(&simplified, config.fit_accuracy);

            // Append fitted elements (skip the MoveTo)
            for el in fitted.elements().iter().skip(1) {
                path.push(*el);
            }
        }
    }

    path.push(PathEl::ClosePath);
    path
}

/// Fit a smooth closed contour (no corners detected).
fn fit_smooth_closed(points: &[(f64, f64)], config: &TracingConfig) -> BezPath {
    let smoothed = smooth_closed(points, config.smooth_iterations);
    let simplified = rdp_simplify(&smoothed, config.rdp_epsilon);

    // Build a BezPath of line segments, then simplify with kurbo
    let mut line_path = BezPath::new();
    if let Some(&first) = simplified.first() {
        line_path.move_to(Point::new(first.0, first.1));
        for &(x, y) in &simplified[1..] {
            line_path.line_to(Point::new(x, y));
        }
        line_path.push(PathEl::ClosePath);
    }

    // Two-pass fitting: first pass converts noisy polyline to smooth curves,
    // second pass re-simplifies the smooth curves to minimum segments.
    let sbp = SimplifyBezPath::new(line_path.elements().iter().copied());
    let first_pass = fit_to_bezpath_opt(&sbp, config.fit_accuracy);

    let sbp2 = SimplifyBezPath::new(first_pass.elements().iter().copied());
    fit_to_bezpath_opt(&sbp2, config.fit_accuracy)
}

/// Fit cubic beziers to an open segment (between two corners).
///
/// Uses two-pass fitting: first pass fits curves to the noisy polyline,
/// second pass re-simplifies those smooth curves to the minimum number
/// of segments. This works because smooth curves simplify far better
/// than noisy pixel polylines.
fn fit_segment(points: &[(f64, f64)], accuracy: f64) -> BezPath {
    let mut line_path = BezPath::new();
    if let Some(&first) = points.first() {
        line_path.move_to(Point::new(first.0, first.1));
        for &(x, y) in &points[1..] {
            line_path.line_to(Point::new(x, y));
        }
    }

    // Pass 1: polyline → curves (may be many segments due to pixel noise)
    let sbp = SimplifyBezPath::new(line_path.elements().iter().copied());
    let first_pass = fit_to_bezpath_opt(&sbp, accuracy);

    // Pass 2: smooth curves → minimal curves
    let sbp2 = SimplifyBezPath::new(first_pass.elements().iter().copied());
    fit_to_bezpath_opt(&sbp2, accuracy)
}

/// Extract a cyclic sub-sequence of points from `start` to `end` (inclusive).
fn extract_segment(
    points: &[(f64, f64)],
    start: usize,
    end: usize,
    total: usize,
) -> Vec<(f64, f64)> {
    let mut segment = Vec::new();
    let mut i = start;
    loop {
        segment.push(points[i]);
        if i == end {
            break;
        }
        i = (i + 1) % total;
    }
    segment
}

/// Simplify a polyline using the Ramer-Douglas-Peucker algorithm.
fn rdp_simplify(points: &[(f64, f64)], epsilon: f64) -> Vec<(f64, f64)> {
    if points.len() <= 2 || epsilon <= 0.0 {
        return points.to_vec();
    }

    let line_string: LineString<f64> =
        LineString::from(points.iter().map(|&(x, y)| (x, y)).collect::<Vec<_>>());

    let simplified = line_string.simplify(&epsilon);

    simplified
        .into_inner()
        .into_iter()
        .map(|c| (c.x, c.y))
        .collect()
}

/// Smooth a closed polyline with iterative neighbor averaging.
/// Removes pixel staircase noise while preserving overall shape.
fn smooth_closed(points: &[(f64, f64)], iterations: usize) -> Vec<(f64, f64)> {
    if iterations == 0 || points.len() < 3 {
        return points.to_vec();
    }
    let mut pts = points.to_vec();
    let n = pts.len();
    for _ in 0..iterations {
        let prev = pts.clone();
        for i in 0..n {
            let p = prev[(i + n - 1) % n];
            let c = prev[i];
            let nx = prev[(i + 1) % n];
            pts[i] = ((p.0 + c.0 + nx.0) / 3.0, (p.1 + c.1 + nx.1) / 3.0);
        }
    }
    pts
}

/// Smooth an open polyline segment, preserving the endpoints (corners).
fn smooth_segment(points: &[(f64, f64)], iterations: usize) -> Vec<(f64, f64)> {
    if iterations == 0 || points.len() < 3 {
        return points.to_vec();
    }
    let mut pts = points.to_vec();
    let n = pts.len();
    for _ in 0..iterations {
        let prev = pts.clone();
        for i in 1..n - 1 {
            pts[i] = (
                (prev[i - 1].0 + prev[i].0 + prev[i + 1].0) / 3.0,
                (prev[i - 1].1 + prev[i].1 + prev[i + 1].1) / 3.0,
            );
        }
    }
    pts
}

/// Scale a pixel coordinate to font units with Y-flip.
///
/// Pixel coords: (0,0) = top-left, y increases downward.
/// Font coords: y increases upward.
///
/// The image height is scaled to `target_height` font units,
/// then shifted by `y_offset` to align the baseline.
fn scale_point(px_x: f64, px_y: f64, image_height: u32, config: &TracingConfig) -> (f64, f64) {
    let ih = image_height as f64;
    let scale = config.target_height / ih;
    let x = px_x * scale;
    let y = (ih - px_y) * scale + config.y_offset;
    (x, y)
}
