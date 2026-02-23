//! Post-processing pipeline for traced bezier contours.
//!
//! Transforms raw fitted curves into font-ready outlines:
//! contour direction, extrema insertion, grid snapping,
//! H/V handle correction, and optional chamfers.

mod chamfer;
mod direction;
mod extrema;
mod simplify;
mod snap;

use kurbo::BezPath;

use crate::config::TracingConfig;

/// Apply all post-processing steps to traced contours.
pub fn process(paths: &[BezPath], config: &TracingConfig) -> Vec<BezPath> {
    let mut result = paths.to_vec();

    if config.fix_direction {
        result = direction::fix_directions(&result);
    }

    if config.add_extrema {
        let depth = config.min_extrema_depth;
        result = result
            .iter()
            .map(|p| extrema::insert_extrema(p, depth))
            .collect();
    }

    if config.grid > 0 {
        let grid = config.grid as f64;
        result = result
            .iter()
            .map(|p| snap::to_grid(p, grid))
            .collect();
    }

    // Snap handles within 15° of H/V to exact H/V.
    result = result
        .iter()
        .map(|p| snap::hv_handles(p, 15.0))
        .collect();

    // Snap nearly-horizontal/vertical line and curve endpoints to exact H/V.
    let hv_threshold = if config.grid > 0 {
        config.grid as f64 * 2.0
    } else {
        4.0
    };
    result = result
        .iter()
        .map(|p| snap::hv_lines(p, hv_threshold))
        .collect();

    // Convert near-straight curves to lines.
    let line_tolerance = if config.grid > 0 {
        (config.grid as f64 * 3.0).max(6.0)
    } else {
        6.0
    };
    result = result
        .iter()
        .map(|p| simplify::curves_to_lines(p, line_tolerance))
        .collect();

    // Merge collinear lines, remove tiny segments.
    let tight = if config.grid > 0 {
        config.grid as f64
    } else {
        4.0
    };
    result = result
        .iter()
        .map(|p| {
            let p = simplify::merge_collinear(p, tight);
            simplify::remove_tiny(&p, tight)
        })
        .collect();

    if config.chamfer_size > 0.0 {
        let size = config.chamfer_size;
        let min = config.chamfer_min_edge;
        result = result
            .iter()
            .map(|p| chamfer::chamfer(p, size, min))
            .collect();
    }

    result
}
