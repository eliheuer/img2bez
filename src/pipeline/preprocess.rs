// Copyright 2026 the img2bez Authors
// SPDX-License-Identifier: Apache-2.0 OR MIT

//! Image-level preparation for tracing: resolution recovery, source-class
//! pre-blur, threshold resolution, and the glue that runs the prepared
//! pixels through the sub-pixel pipeline into an [`Outline`].

use kurbo::{BezPath, PathEl};
use std::time::Instant;

use crate::io::bitmap;
use crate::ml::image_stats;
use crate::model::config::{ThresholdMethod, TraceOptions};
use crate::model::error::TraceError;
use crate::model::outline::Outline;
use crate::model::trace_mode;
use crate::pipeline::{cleanup, vectorize};

/// Prepare a raw grayscale image and trace it with the sub-pixel pipeline.
pub(crate) fn trace_luma(
    raw: image::GrayImage,
    config: &TraceOptions,
    t_start: Option<Instant>,
) -> Result<Outline, TraceError> {
    let (raw, threshold, (upscaled, soft)) = preprocess_luma(raw, config);
    let mut config = config.clone();
    config.corner_smear = upscaled;
    config.soft_source = soft;
    let mut outline =
        trace_subpixel_pipeline(raw, threshold, &config, t_start)?;
    outline.normalize_starts(config.rtl_start);
    Ok(outline)
}

/// Image-level preparation shared by every trace: recover true resolution,
/// apply the source-class pre-blur, resolve the ink threshold. The masters
/// flow preprocesses each master with this, then traces the set jointly.
pub(crate) fn preprocess_luma(
    raw: image::GrayImage,
    config: &TraceOptions,
) -> (image::GrayImage, u8, (bool, bool)) {
    // Nearest-neighbor upscaled sources quantize the anti-aliasing into
    // uniform blocks; recover the true resolution before tracing.
    let mut raw = raw;
    while let Some(downscaled) = downscale_if_blocked(&raw) {
        if config.verbose {
            eprintln!(
                "  Detect      uniform 2x2 blocks; downscaling to {}x{}",
                downscaled.width(),
                downscaled.height()
            );
        }
        raw = downscaled;
    }
    // Smooth-upscale small glyphs so the sub-pixel tracer sees anti-aliased
    // ramps, not stair-stepped pixel blocks.
    let mut upscaled_lowres = false;
    if let Some(upscaled) = upscale_if_lowres(&raw) {
        if config.verbose {
            eprintln!(
                "  Detect      low-res glyph; smooth-upscaling to {}x{}",
                upscaled.width(),
                upscaled.height()
            );
        }
        raw = upscaled;
        upscaled_lowres = true;
    }
    // Pre-blur clears soft-scan edge texture before contour extraction.
    // Explicit `pre_blur` wins; otherwise Photo profile (forced or, with
    // `auto_pre_blur`, classified soft) gets a resolution-scaled blur.
    let pre_blur = if config.pre_blur > 0.0 {
        config.pre_blur
    } else if config.profile.applies_pre_blur() || config.auto_pre_blur {
        let stats = image_stats::measure(&raw);
        let photo = config.profile.applies_pre_blur()
            || (config.auto_pre_blur && stats.classify().applies_pre_blur());
        if photo {
            photo_pre_blur_sigma(stats.extent_px)
        } else {
            0.0
        }
    } else {
        0.0
    };
    if pre_blur > 0.0 {
        if config.verbose {
            let how = if config.profile.applies_pre_blur() {
                "photo"
            } else {
                "auto"
            };
            eprintln!("  Pre-blur    sigma {pre_blur:.1} ({how})");
        }
        raw = image::imageops::blur(&raw, pre_blur as f32);
    }
    let threshold = bitmap::resolve_threshold(&raw, config);
    (raw, threshold, (upscaled_lowres, pre_blur > 0.0))
}

/// Resolution-scaled pre-blur sigma (px) for Photo handling: ~1.6% of ink
/// extent, clamped — enough to erase soft-scan edge wobble (spurious stem
/// extrema) while real structure survives; more over-blurs terminals.
fn photo_pre_blur_sigma(extent_px: u32) -> f64 {
    const BLUR_FRACTION: f64 = 0.016;
    const BLUR_MIN: f64 = 2.0;
    const BLUR_MAX: f64 = 16.0;
    (extent_px as f64 * BLUR_FRACTION).clamp(BLUR_MIN, BLUR_MAX)
}

/// If the image is composed of uniform 2x2 pixel blocks (a nearest-
/// neighbor 2x upscale), return the half-resolution original.
fn downscale_if_blocked(img: &image::GrayImage) -> Option<image::GrayImage> {
    let (w, h) = img.dimensions();
    if w < 64 || h < 64 || w % 2 != 0 || h % 2 != 0 {
        return None;
    }
    let mut uniform = 0u64;
    let total = (w / 2) as u64 * (h / 2) as u64;
    for by in 0..h / 2 {
        for bx in 0..w / 2 {
            let p = |dx: u32, dy: u32| {
                img.get_pixel(bx * 2 + dx, by * 2 + dy).0[0] as i32
            };
            let v = [p(0, 0), p(1, 0), p(0, 1), p(1, 1)];
            let min = *v.iter().min().unwrap();
            let max = *v.iter().max().unwrap();
            if max - min <= 2 {
                uniform += 1;
            }
        }
    }
    // A genuine nearest-neighbor upscale has (almost) EVERY block uniform;
    // anything less means real AA boundaries.
    if uniform < total - total / 2000 {
        return None;
    }
    Some(image::GrayImage::from_fn(w / 2, h / 2, |x, y| {
        *img.get_pixel(x * 2, y * 2)
    }))
}

/// Glyph extent (px) below which a source counts as low-resolution; sits
/// well under normal font renders so they never trigger smoothing.
const LOWRES_MAX_EXTENT: u32 = 200;
/// Target glyph extent (px) to smooth-upscale a low-res source toward.
const LOWRES_TARGET_EXTENT: u32 = 400;
/// Hard cap on the smoothing upscale factor (guards tiny / blank inputs).
const LOWRES_MAX_FACTOR: f64 = 8.0;

/// Background-relative ink bbox `(min_x, min_y, max_x, max_y)`, or `None` if
/// no ink. Background sampled from the four corners; ink differs by >40
/// levels. Deliberately separate from `image_stats::ink_extent` and the
/// threshold-based `placement::ink_bounds`.
fn ink_bbox(img: &image::GrayImage) -> Option<(u32, u32, u32, u32)> {
    let (w, h) = img.dimensions();
    if w == 0 || h == 0 {
        return None;
    }
    let at = |x: u32, y: u32| img.get_pixel(x, y).0[0] as i32;
    let bg = (at(0, 0) + at(w - 1, 0) + at(0, h - 1) + at(w - 1, h - 1)) / 4;
    let (mut min_x, mut min_y, mut max_x, mut max_y) = (w, h, 0u32, 0u32);
    let mut found = false;
    for y in 0..h {
        for x in 0..w {
            if (at(x, y) - bg).abs() > 40 {
                min_x = min_x.min(x);
                min_y = min_y.min(y);
                max_x = max_x.max(x);
                max_y = max_y.max(y);
                found = true;
            }
        }
    }
    found.then_some((min_x, min_y, max_x, max_y))
}

/// Smooth-upscale (CatmullRom) a glyph whose extent is below
/// [`LOWRES_MAX_EXTENT`] toward [`LOWRES_TARGET_EXTENT`]; `None` otherwise.
fn upscale_if_lowres(img: &image::GrayImage) -> Option<image::GrayImage> {
    let (min_x, min_y, max_x, max_y) = ink_bbox(img)?;
    let extent = (max_x - min_x).max(max_y - min_y);
    if extent == 0 || extent >= LOWRES_MAX_EXTENT {
        return None;
    }
    let factor = (LOWRES_TARGET_EXTENT as f64 / extent as f64)
        .ceil()
        .min(LOWRES_MAX_FACTOR);
    if factor < 2.0 {
        return None;
    }
    let (w, h) = img.dimensions();
    let nw = ((w as f64) * factor).round().max(1.0) as u32;
    let nh = ((h as f64) * factor).round().max(1.0) as u32;
    Some(image::imageops::resize(
        img,
        nw,
        nh,
        image::imageops::FilterType::CatmullRom,
    ))
}

fn trace_subpixel_pipeline(
    raw: image::GrayImage,
    threshold: u8,
    config: &TraceOptions,
    t_start: Option<Instant>,
) -> Result<Outline, TraceError> {
    let dims = raw.dimensions();
    log_load(dims, "sub-pixel iso", config);
    let curves = vectorize::trace_subpixel(&raw, threshold, config);
    finish_trace(curves, dims, config, t_start)
}

fn log_load(dims: (u32, u32), pipeline: &str, config: &TraceOptions) {
    if config.verbose {
        let threshold_name = match config.threshold {
            ThresholdMethod::Otsu => "Otsu".to_string(),
            ThresholdMethod::Fixed(t) => format!("fixed {}", t),
        };
        eprintln!(
            "  Load        {}x{} px, {} threshold, {} pipeline",
            dims.0, dims.1, threshold_name, pipeline
        );
    }
}

fn finish_trace(
    curves: Vec<BezPath>,
    dims: (u32, u32),
    config: &TraceOptions,
    t_start: Option<Instant>,
) -> Result<Outline, TraceError> {
    let h = dims.1;
    let scale = config.em_height / h as f64;
    if curves.is_empty() {
        return Err(TraceError::NoContours);
    }
    let (raw_curves, raw_lines) = count_segments(&curves);
    if config.verbose {
        eprintln!(
            "  Trace       {} contours \u{2192} {} curves + {} lines  (scale \u{00d7}{:.2})",
            curves.len(),
            raw_curves,
            raw_lines,
            scale,
        );
    }

    // ── Post-processing ─────────────────────────────
    let paths = if std::env::var("IMG2BEZ_DEBUG_NO_CLEANUP").is_ok() {
        if config.verbose {
            eprintln!("  Debug       skipping cleanup");
        }
        curves.clone()
    } else {
        cleanup::process(&curves, config)
    };

    let mut steps: Vec<&str> = Vec::new();
    if config.fix_direction {
        steps.push("direction");
    }
    if config.grid > 0 {
        steps.push("grid snap");
    }
    steps.push("H/V snap");
    if config.chamfer_size > 0.0 {
        steps.push("chamfer");
    }
    if config.verbose {
        eprintln!("  Clean       {}", steps.join(" \u{00b7} "));
        let (on, off) = count_points(&paths);
        let elapsed = t_start.map(|s| s.elapsed().as_millis()).unwrap_or(0);
        eprintln!(
            "  Result      {} contours \u{00b7} {} points ({} on-curve)  ({}ms)",
            paths.len(),
            on + off,
            on,
            elapsed,
        );
    }

    Ok(trace_mode::apply(
        Outline::from_bezpaths(&paths),
        config.mode,
    ))
}

/// Count (curves, lines) segments in a set of BezPaths.
fn count_segments(paths: &[BezPath]) -> (usize, usize) {
    let mut curves = 0;
    let mut lines = 0;
    for path in paths {
        for el in path.elements() {
            match el {
                PathEl::CurveTo(..) => curves += 1,
                PathEl::LineTo(_) => lines += 1,
                _ => {}
            }
        }
    }
    (curves, lines)
}

/// Count (on-curve, off-curve) points for UFO output.
fn count_points(paths: &[BezPath]) -> (usize, usize) {
    let mut on = 0usize;
    let mut off = 0usize;
    for path in paths {
        for el in path.elements() {
            match el {
                PathEl::CurveTo(..) => {
                    on += 1;
                    off += 2;
                }
                PathEl::LineTo(_) => {
                    on += 1;
                }
                PathEl::QuadTo(..) => {
                    on += 1;
                    off += 1;
                }
                _ => {}
            }
        }
    }
    (on, off)
}
