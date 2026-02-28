//! Post-processing pipeline for traced bezier contours.
//!
//! With extrema-based splitting in the curve fitting stage, most
//! cleanup is no longer needed. This pipeline handles only:
//! contour direction, grid snapping, residual H/V handle correction,
//! and optional chamfers.

mod chamfer;
mod direction;
mod extrema;
mod simplify;
mod snap;

use kurbo::BezPath;

use crate::config::TracingConfig;

/// Apply post-processing steps to traced contours.
///
/// Four steps: fix direction → grid snap → H/V handles → chamfer.
pub fn process(paths: &[BezPath], config: &TracingConfig) -> Vec<BezPath> {
    let mut result = paths.to_vec();

    if config.fix_direction {
        result = direction::fix_directions(&result);
    }

    if config.grid > 0 {
        let grid = config.grid as f64;
        result = result
            .iter()
            .map(|p| snap::to_grid(p, grid))
            .collect();
    }

    // Snap nearly-H/V line endpoints after grid snapping.
    // Grid snap may leave line endpoints a few units off H/V.
    let hv_line_tol = if config.grid > 0 {
        config.grid as f64 * 1.5
    } else {
        3.0
    };
    result = result
        .iter()
        .map(|p| snap::hv_lines(p, hv_line_tol))
        .collect();

    // One pass of H/V handle snapping catches grid-snap residuals.
    result = result
        .iter()
        .map(|p| snap::hv_handles(p, 25.0))
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
