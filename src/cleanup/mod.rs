//! Post-processing pipeline for traced bezier contours.
//!
//! With extrema-based splitting in the curve fitting stage, most
//! cleanup is no longer needed. This pipeline handles only:
//! contour direction, grid snapping, residual H/V handle correction,
//! and optional chamfers.

mod chamfer;
mod direction;
mod snap;

use kurbo::BezPath;

use crate::config::TracingConfig;

/// Maximum angle (in degrees) from exact H/V before a bezier handle is
/// snapped to exact horizontal or vertical. 15deg catches residual
/// off-axis handles introduced by grid snapping.
const HV_HANDLE_SNAP_THRESHOLD_DEG: f64 = 15.0;

/// Apply post-processing steps to traced contours.
///
/// Three steps: fix direction → grid snap → H/V handles → chamfer.
pub fn process(paths: &[BezPath], config: &TracingConfig) -> Vec<BezPath> {
    let mut result = paths.to_vec();

    if config.fix_direction {
        result = direction::fix_directions(&result);
    }

    if config.grid > 0 {
        let grid = config.grid as f64;
        result = result.iter().map(|p| snap::to_grid(p, grid)).collect();
    }

    // One pass of H/V handle snapping catches grid-snap residuals.
    result = result
        .iter()
        .map(|p| snap::hv_handles(p, HV_HANDLE_SNAP_THRESHOLD_DEG))
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
