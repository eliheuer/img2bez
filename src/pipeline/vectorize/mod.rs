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

    // Optional raster-loss refinement target (see `refine`).
    let raster = config
        .refine_raster
        .then(|| refine::RasterTarget::new(gray, config.invert, 1.0 / scale));

    fit_contours(&contours, image_height, config, raster.as_ref())
}

/// Fit contours (from any source: image extraction or a signed field)
/// into bezier paths in neutral em space. `image_height` sets the
/// px-to-em scale exactly as in [`trace_subpixel`].
pub(crate) fn fit_contours(
    contours: &[subpixel::SubpixelContour],
    image_height: u32,
    config: &TraceOptions,
    raster: Option<&refine::RasterTarget>,
) -> Vec<BezPath> {
    let height = image_height as f64;
    let scale = config.em_height / height;
    // SmoothG2 is realized here, not as a post-pass: a C2 spline is
    // only faithful when its knots are dense, so it splines the raw
    // iso-contour resampled by arc length instead of the economical
    // fitter's sparse structural points.
    if config.mode == crate::model::config::TraceMode::SmoothG2 {
        let accuracy_px = (config.fit_accuracy / scale).clamp(0.5, 3.0);
        let spacing = (accuracy_px * 4.0).max(3.0);
        let transform = Affine::scale(scale);
        return contours
            .iter()
            .filter_map(|c| {
                let mut p = spline_g2_contour(&c.points, spacing)?;
                p.apply_affine(transform);
                Some(p)
            })
            .collect();
    }
    // Fitting accuracy in pixels: type quality favors minimal points over
    // pixel-perfect tracking, so allow a couple of pixels of deviation.
    let accuracy = (config.fit_accuracy / scale).clamp(0.5, 3.0);
    // Scale only — neutral em space, no baseline shift.
    let transform = Affine::scale(scale);

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

/// Resample a closed polyline at uniform arc-length `spacing` and fit
/// a periodic natural cubic spline through the samples: C2 curvature
/// continuity, every point smooth, and fidelity pinned by the sample
/// density. Returns None for degenerate contours.
fn spline_g2_contour(points: &[(f64, f64)], spacing: f64) -> Option<BezPath> {
    use kurbo::{Point, Vec2};
    if points.len() < 4 {
        return None;
    }
    // perimeter and arc-length resampling
    let n_in = points.len();
    let seg_len: Vec<f64> = (0..n_in)
        .map(|i| {
            let a = points[i];
            let b = points[(i + 1) % n_in];
            ((b.0 - a.0).powi(2) + (b.1 - a.1).powi(2)).sqrt()
        })
        .collect();
    let perimeter: f64 = seg_len.iter().sum();
    if perimeter < spacing * 3.0 {
        return None;
    }
    let n = ((perimeter / spacing).round() as usize).max(8);
    let step = perimeter / n as f64;
    let mut knots: Vec<Point> = Vec::with_capacity(n);
    let mut seg = 0usize;
    let mut into = 0.0f64;
    let mut acc = 0.0f64;
    for _ in 0..n {
        while acc > seg_len[seg] - into {
            acc -= seg_len[seg] - into;
            into = 0.0;
            seg = (seg + 1) % n_in;
        }
        into += acc;
        acc = step;
        let a = points[seg];
        let b = points[(seg + 1) % n_in];
        let t = if seg_len[seg] > 1e-9 { into / seg_len[seg] } else { 0.0 };
        knots.push(Point::new(a.0 + (b.0 - a.0) * t, a.1 + (b.1 - a.1) * t));
    }

    // periodic natural cubic spline (cyclic tridiagonal, Sherman-Morrison)
    let m = knots.len();
    let h: Vec<f64> = (0..m)
        .map(|i| (knots[(i + 1) % m] - knots[i]).hypot().max(1e-9))
        .collect();
    let solve = |vals: &dyn Fn(usize) -> f64| -> Vec<f64> {
        let a: Vec<f64> = (0..m).map(|i| h[(i + m - 1) % m]).collect();
        let b: Vec<f64> = (0..m).map(|i| 2.0 * (h[(i + m - 1) % m] + h[i])).collect();
        let c: Vec<f64> = h.clone();
        let d: Vec<f64> = (0..m)
            .map(|i| {
                let prev = (i + m - 1) % m;
                let next = (i + 1) % m;
                6.0 * ((vals(next) - vals(i)) / h[i] - (vals(i) - vals(prev)) / h[prev])
            })
            .collect();
        let tri = |bb: &[f64], dd: &[f64]| -> Vec<f64> {
            let mut cp = vec![0.0; m];
            let mut dp = vec![0.0; m];
            cp[0] = c[0] / bb[0];
            dp[0] = dd[0] / bb[0];
            for i in 1..m {
                let w = bb[i] - a[i] * cp[i - 1];
                cp[i] = c[i] / w;
                dp[i] = (dd[i] - a[i] * dp[i - 1]) / w;
            }
            let mut x = vec![0.0; m];
            x[m - 1] = dp[m - 1];
            for i in (0..m - 1).rev() {
                x[i] = dp[i] - cp[i] * x[i + 1];
            }
            x
        };
        let gamma = -b[0];
        let mut bb = b.clone();
        bb[0] = b[0] - gamma;
        bb[m - 1] = b[m - 1] - a[0] * c[m - 1] / gamma;
        let mut u = vec![0.0; m];
        u[0] = gamma;
        u[m - 1] = c[m - 1];
        let x = tri(&bb, &d);
        let z = tri(&bb, &u);
        let fact = (x[0] + a[0] * x[m - 1] / gamma)
            / (1.0 + z[0] + a[0] * z[m - 1] / gamma);
        (0..m).map(|i| x[i] - fact * z[i]).collect()
    };
    let mx = solve(&|i| knots[i].x);
    let my = solve(&|i| knots[i].y);

    let mut path = BezPath::new();
    path.move_to(knots[0]);
    for i in 0..m {
        let j = (i + 1) % m;
        let hi = h[i];
        let p0 = knots[i];
        let p1 = knots[j];
        let d0 = Vec2::new(
            (p1.x - p0.x) / hi - hi * (2.0 * mx[i] + mx[j]) / 6.0,
            (p1.y - p0.y) / hi - hi * (2.0 * my[i] + my[j]) / 6.0,
        );
        let d1 = Vec2::new(
            (p1.x - p0.x) / hi + hi * (mx[i] + 2.0 * mx[j]) / 6.0,
            (p1.y - p0.y) / hi + hi * (my[i] + 2.0 * my[j]) / 6.0,
        );
        path.curve_to(p0 + d0 * (hi / 3.0), p1 - d1 * (hi / 3.0), p1);
    }
    path.close_path();
    Some(path)
}
