// Copyright 2026 the img2bez Authors
// SPDX-License-Identifier: Apache-2.0 OR MIT

//! Post-processing pipeline for traced bezier contours: light structural
//! passes after the fitter's heavy shaping. See [`process`] for the order.

mod chamfer;
mod direction;
mod even;
mod inflection;
mod simplify;
mod snap;
mod straighten;

use kurbo::BezPath;

use crate::model::config::TraceOptions;

/// Max angle (degrees) from exact H/V before a handle is snapped; catches
/// residual off-axis handles introduced by grid snapping.
const HV_HANDLE_SNAP_THRESHOLD_DEG: f64 = 15.0;

/// Photo-profile fairing tolerance (font units): features whose merge
/// deviates less than this are soft-scan edge noise and fair out; more is
/// real structure. Clean/wild keep all features (KEEP_ALL_EXTREMA).
const PHOTO_FAIR_DEV: f64 = 3.0;

/// Apply post-processing steps to traced contours.
///
/// In order: fix direction → straighten runs → simplify (merge redundant
/// points; deviation-fair on the photo profile) → grid snap → H/V handle
/// snap → inflection anchoring → chamfer → handle evening → handle cap →
/// integer handle rounding.
pub fn process(paths: &[BezPath], config: &TraceOptions) -> Vec<BezPath> {
    let mut result = paths.to_vec();

    if config.fix_direction {
        result = direction::fix_directions(&result);
    }

    result = result
        .iter()
        .map(straighten::flatten_straight_runs)
        .collect();

    let fair_dev = if config.profile == crate::model::config::Profile::Photo {
        // IMG2BEZ_PHOTO_FAIR_DEV overrides the default for offline tuning
        // (no-op in wasm, where env vars are unavailable).
        std::env::var("IMG2BEZ_PHOTO_FAIR_DEV")
            .ok()
            .and_then(|s| s.parse().ok())
            .unwrap_or(PHOTO_FAIR_DEV)
    } else {
        simplify::KEEP_ALL_EXTREMA
    };
    result = result
        .iter()
        .map(|p| simplify::remove_redundant_points(p, fair_dev))
        .collect();

    if config.grid > 0 {
        let grid = config.grid as f64;
        // Two-tier dyadic self-labeling snap when a coarser structure grid is
        // set (e.g. 8 over a fine grid of 2); otherwise a single-grid snap.
        result = if config.structure_grid > config.grid {
            let structure = config.structure_grid as f64;
            result
                .iter()
                .map(|p| snap::to_two_tier_grid(p, grid, structure))
                .collect()
        } else {
            result.iter().map(|p| snap::to_grid(p, grid)).collect()
        };
    }

    // H/V snap spares smooth inflection points (their near-axis tangents are
    // meant to stay diagonal) and corner anchors (forcing a stroke-end
    // corner handle onto an axis visibly reshapes a taper).
    result = result
        .iter()
        .map(|p| {
            let keep = inflection::smooth_inflection_points(p);
            let corners = snap::corner_anchor_points(p);
            snap::hv_handles(p, HV_HANDLE_SNAP_THRESHOLD_DEG, &keep, &corners)
        })
        .collect();

    // Inflection anchoring runs after H/V snap so its off-axis handles
    // survive; the new point is grid-snapped here.
    let grid = config.grid.max(0) as f64;
    result = result
        .iter()
        .map(|p| inflection::split_inflections(p, grid))
        .collect();

    if config.chamfer_size > 0.0 {
        let size = config.chamfer_size;
        let min = config.chamfer_min_edge;
        result = result
            .iter()
            .map(|p| chamfer::chamfer(p, size, min))
            .collect();
    }

    result = result.iter().map(even::even_handles).collect();

    // Last geometric pass: clamp handles the stages above pushed outside
    // their magic triangle (H/V snap and inflection anchoring rotate handle
    // directions).
    result = result.iter().map(even::cap_handles).collect();

    if config.grid > 0 {
        result = result.iter().map(snap::round_handles).collect();
    }

    result
}
