use image::GrayImage;
use imageproc::contours::{find_contours, BorderType};

use crate::config::TracingConfig;
use crate::error::TraceError;

/// A contour extracted from the bitmap, in pixel coordinates.
#[derive(Debug, Clone)]
pub struct RawContour {
    /// Points in pixel coordinates (y=0 is top of image).
    pub points: Vec<(f64, f64)>,
    /// Whether this is an outer contour or a hole.
    pub is_outer: bool,
}

/// Extract contours from a binary image.
///
/// Returns contours sorted with outer contours first, then their holes.
/// Each contour is a closed polyline of pixel coordinates.
pub fn extract(gray: &GrayImage, config: &TracingConfig) -> Result<Vec<RawContour>, TraceError> {
    let ip_contours = find_contours::<i32>(gray);

    let mut result: Vec<RawContour> = ip_contours
        .iter()
        .filter(|contour| contour.points.len() >= 3)
        .map(|contour| {
            let points = contour.points.iter().map(|p| (p.x as f64, p.y as f64)).collect();

            RawContour {
                points,
                is_outer: contour.border_type == BorderType::Outer,
            }
        })
        .collect();

    // Filter by minimum area
    result.retain(|contour| polygon_area(&contour.points).abs() > config.min_contour_area);

    Ok(result)
}

/// Signed area via shoelace formula. Positive = CCW, negative = CW.
fn polygon_area(points: &[(f64, f64)]) -> f64 {
    let n = points.len();
    if n < 3 {
        return 0.0;
    }
    (0..n)
        .map(|i| {
            let j = (i + 1) % n;
            points[i].0 * points[j].1 - points[j].0 * points[i].1
        })
        .sum::<f64>()
        / 2.0
}
