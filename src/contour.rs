use image::GrayImage;
use imageproc::contours::{find_contours, BorderType};

use crate::config::TracingConfig;
use crate::error::TraceError;

/// A contour extracted from the bitmap, in pixel coordinates.
#[derive(Debug, Clone)]
pub struct RawContour {
    /// Points in pixel coordinates (y=0 is top of image).
    pub points: Vec<(f64, f64)>,
    /// Whether this is an outer contour or a hole (counter).
    pub is_outer: bool,
    /// Index of the parent contour in the returned Vec (for nesting hierarchy).
    pub parent: Option<usize>,
}

/// Detect contours in a binary image.
///
/// Returns contours sorted with outer contours first, then their holes.
/// Each contour is a closed polyline of pixel coordinates.
pub fn detect(gray: &GrayImage, config: &TracingConfig) -> Result<Vec<RawContour>, TraceError> {
    let ip_contours = find_contours::<i32>(gray);

    let mut result: Vec<RawContour> = ip_contours
        .iter()
        .filter(|c| c.points.len() >= 3)
        .map(|c| {
            let points = c.points.iter().map(|p| (p.x as f64, p.y as f64)).collect();

            RawContour {
                points,
                is_outer: c.border_type == BorderType::Outer,
                parent: c.parent,
            }
        })
        .collect();

    // Filter by minimum area
    result.retain(|c| polygon_area(&c.points).abs() > config.min_contour_area);

    Ok(result)
}

/// Signed area via shoelace formula. Positive = CCW, negative = CW.
fn polygon_area(pts: &[(f64, f64)]) -> f64 {
    let n = pts.len();
    if n < 3 {
        return 0.0;
    }
    (0..n)
        .map(|i| {
            let j = (i + 1) % n;
            pts[i].0 * pts[j].1 - pts[j].0 * pts[i].1
        })
        .sum::<f64>()
        / 2.0
}
