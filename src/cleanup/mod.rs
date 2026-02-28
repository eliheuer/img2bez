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

    // Snap handles within 25° of H/V to exact H/V.
    result = result
        .iter()
        .map(|p| snap::hv_handles(p, 25.0))
        .collect();

    // Force remaining off-axis handles to H/V if curve distortion
    // stays within the grid tolerance.
    let hv_force_tol = if config.grid > 0 {
        config.grid as f64 * 4.0
    } else {
        8.0
    };
    result = result
        .iter()
        .map(|p| snap::force_hv_handles(p, hv_force_tol))
        .collect();

    // Convert degenerate curves (both handles hug the chord) to lines.
    // Must run before hv_lines so newly-created lines get H/V snapped.
    let c2l_tolerance = if config.grid > 0 {
        config.grid as f64 * 3.0
    } else {
        5.0
    };
    result = result
        .iter()
        .map(|p| simplify::curves_to_lines(p, c2l_tolerance))
        .collect();

    // Snap nearly-H/V line and curve endpoints to exact H/V.
    let hv_threshold = if config.grid > 0 {
        config.grid as f64 * 1.5
    } else {
        3.0
    };
    result = result
        .iter()
        .map(|p| snap::hv_lines(p, hv_threshold))
        .collect();

    // Re-snap handles: hv_lines may have moved endpoints, invalidating
    // handle alignments set earlier.
    result = result
        .iter()
        .map(|p| snap::hv_handles(p, 25.0))
        .collect();

    // Merge collinear lines, remove tiny segments.
    let merge_tol = if config.grid > 0 {
        config.grid as f64 * 2.0
    } else {
        4.0
    };
    result = result
        .iter()
        .map(|p| {
            let p = simplify::merge_collinear(p, merge_tol);
            simplify::remove_tiny(&p, merge_tol)
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
