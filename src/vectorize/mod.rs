//! Vectorization pipeline: bitmap → bezier contours.
//!
//! 1. Pixel-edge contour extraction (dual grid)
//! 2. Optimal polygon approximation (DP)
//! 3. Sub-pixel vertex refinement
//! 4. Alpha-based corner detection and Bezier curve generation

pub mod decompose;
pub mod polygon;
pub mod curve;

use image::GrayImage;
use kurbo::{Affine, BezPath, Vec2};
use rayon::prelude::*;

use crate::config::TracingConfig;

/// Run the full vectorization pipeline: binary image → font-unit BezPaths.
pub fn trace(gray: &GrayImage, config: &TracingConfig) -> Vec<BezPath> {
    let (image_width, image_height) = gray.dimensions();
    let height = image_height as f64;
    let scale = config.target_height / height;

    // Stage 1: Extract pixel-edge contours on the dual grid.
    let min_area = (config.min_contour_area / (scale * scale)).max(2.0) as usize;
    let mut pixel_paths = decompose::decompose(gray, min_area);

    // Discard contours that span nearly the full image — these are
    // image-frame artifacts, not real glyph contours.
    let iw = image_width as i32;
    let ih = image_height as i32;
    pixel_paths.retain(|p| !is_frame_contour(p, iw, ih));

    // Stage 2-3: Optimal polygon + vertex refinement for each contour.
    let polygons: Vec<polygon::Polygon> = pixel_paths
        .iter()
        .map(polygon::optimal_polygon)
        .collect();

    // Stage 4: Classify corners + fit minimal cubics through smooth sections.
    // Convert fit_accuracy from font units to pixel-corner coordinates.
    // Curve fitting is the bottleneck (~95% of trace time), so we
    // parallelize across contours with rayon.
    let params = curve::CurveParams {
        alphamax: config.alphamax,
        accuracy: config.fit_accuracy / scale,
        smooth_iterations: config.smooth_iterations,
    };
    let transform = Affine::scale(scale)
        * Affine::translate(Vec2::new(0.0, config.y_offset / scale));

    let paths: Vec<BezPath> = polygons
        .par_iter()
        .map(|poly| {
            let mut path = curve::polygon_to_bezpath(poly, &params);
            path.apply_affine(transform);
            path
        })
        .collect();

    paths
}

/// Check if a pixel path is the image frame (bounding box covers >90% of image).
fn is_frame_contour(path: &decompose::PixelPath, img_w: i32, img_h: i32) -> bool {
    if path.points.len() < 3 {
        return false;
    }
    let (mut min_x, mut min_y) = (i32::MAX, i32::MAX);
    let (mut max_x, mut max_y) = (i32::MIN, i32::MIN);
    for &(x, y) in &path.points {
        min_x = min_x.min(x);
        min_y = min_y.min(y);
        max_x = max_x.max(x);
        max_y = max_y.max(y);
    }
    let pw = max_x - min_x;
    let ph = max_y - min_y;
    pw as f64 > img_w as f64 * 0.9 && ph as f64 > img_h as f64 * 0.9
}
