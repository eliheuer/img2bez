// Copyright 2026 the img2bez Authors
// SPDX-License-Identifier: Apache-2.0 OR MIT

//! Vectorization pipeline: bitmap → bezier contours. Sub-pixel
//! iso-contour extraction (`subpixel`), curvature-based segmentation and
//! constrained fitting (`fit`), optional raster-loss refinement
//! (`refine`). For master sets, `joint` replaces the per-image structural
//! decisions with one plan decided across all masters.

pub mod fit;
pub mod joint;
pub mod refine;
pub mod subpixel;

use image::GrayImage;
use kurbo::{Affine, BezPath};
#[cfg(feature = "parallel")]
use rayon::prelude::*;

use crate::model::config::TraceOptions;

/// Fraction of image dimensions a contour's bounding box must span to be
/// a possible image-frame artifact.
const FRAME_CONTOUR_THRESHOLD: f64 = 0.9;
/// A frame-sized contour is discarded only when it also HUGS the border
/// (this fraction of points within FRAME_BORDER_PX of the edge): a scan's
/// frame line runs along the border, a tightly cropped glyph doesn't.
const FRAME_HUG_FRACTION: f64 = 0.5;
const FRAME_BORDER_PX: f64 = 2.5;

/// Extract the glyph's iso-contours: marching-squares boundaries at the
/// threshold level, with image-frame artifacts discarded. Shared by the
/// single-image pipeline and the joint masters pipeline.
pub(crate) fn glyph_contours(
    gray: &GrayImage,
    threshold: u8,
    config: &TraceOptions,
    min_area: f64,
) -> Vec<subpixel::SubpixelContour> {
    let (image_width, image_height) = gray.dimensions();
    let mut contours = subpixel::extract_iso_contours(
        gray,
        threshold,
        config.invert,
        min_area,
    );
    let iw = image_width as f64;
    let ih = image_height as f64;
    contours.retain(|c| {
        let (x0, y0, x1, y1) = c.bbox();
        let frame_sized = x1 - x0 > iw * FRAME_CONTOUR_THRESHOLD
            && y1 - y0 > ih * FRAME_CONTOUR_THRESHOLD;
        if !frame_sized {
            return true;
        }
        let near_border = c
            .points
            .iter()
            .filter(|p| {
                p.0 <= FRAME_BORDER_PX
                    || p.1 <= FRAME_BORDER_PX
                    || p.0 >= iw - FRAME_BORDER_PX
                    || p.1 >= ih - FRAME_BORDER_PX
            })
            .count();
        (near_border as f64) < c.points.len() as f64 * FRAME_HUG_FRACTION
    });
    contours
}

/// Trace a grayscale image into fitted bezier paths. Paths are returned
/// in neutral em space (y-up, `0..em_height`); placement is applied later
/// by `crate::placement`.
pub fn trace_subpixel(
    gray: &GrayImage,
    threshold: u8,
    config: &TraceOptions,
) -> Vec<BezPath> {
    let (_image_width, image_height) = gray.dimensions();
    let height = image_height as f64;
    let scale = config.em_height / height;
    let min_area = (config.min_contour_area / (scale * scale)).max(2.0);

    let contours = glyph_contours(gray, threshold, config, min_area);

    // Fitting accuracy in pixels: type quality favors minimal points over
    // pixel-perfect tracking, so allow a couple of pixels of deviation.
    let accuracy = (config.fit_accuracy / scale).clamp(0.5, 3.0);
    // Scale only — neutral em space, no baseline shift.
    let transform = Affine::scale(scale);

    // Optional raster-loss refinement target (see `refine`).
    let raster = config
        .refine_raster
        .then(|| refine::RasterTarget::new(gray, config.invert, 1.0 / scale));
    let raster = raster.as_ref();

    #[cfg(feature = "parallel")]
    let paths: Vec<BezPath> = contours
        .par_iter()
        .map(|c| {
            let mut p = fit::trace_contour(
                c,
                accuracy,
                config.smoothing,
                config.corner_threshold_deg,
                config.corner_smear,
                config.soft_source,
                raster,
            );
            p.apply_affine(transform);
            p
        })
        .collect();

    #[cfg(not(feature = "parallel"))]
    let paths: Vec<BezPath> = contours
        .iter()
        .map(|c| {
            let mut p = fit::trace_contour(
                c,
                accuracy,
                config.smoothing,
                config.corner_threshold_deg,
                config.corner_smear,
                config.soft_source,
                raster,
            );
            p.apply_affine(transform);
            p
        })
        .collect();

    paths
}
