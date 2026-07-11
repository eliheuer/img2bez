// Copyright 2026 the img2bez Authors
// SPDX-License-Identifier: Apache-2.0 OR MIT

//! Trace raster glyph images into font-ready cubic bezier outlines.
//!
//! [`trace`] turns image bytes into an [`Outline`]: a serde-serializable,
//! UFO-faithful point model (point types and `smooth` flags) in neutral em
//! space — y-up, with no font metrics applied. [`place`] then positions it
//! in a font (baseline, sidebearings, advance). Everything downstream is a
//! view onto the [`Outline`]: kurbo `BezPath`s, GLIF, SVG, JSON.
//!
//! Tracing is deterministic: the same image bytes and options always
//! produce the same outline, down to byte-identical GLIF output. The
//! learned decision heads (tiny embedded tree ensembles at a few pipeline
//! gates) are opt-in via [`set_corner_head`] / [`set_site_head`] (or the
//! `IMG2BEZ_CORNER_HEAD` / `IMG2BEZ_SITE_HEAD` env vars) and equally
//! deterministic; the default pipeline is fully procedural.
//!
//! # Example
//!
//! ```no_run
//! use img2bez::{trace, place, TraceOptions, FontMetrics};
//!
//! let bytes = std::fs::read("glyph.png")?;
//! let outline = trace(&bytes, &TraceOptions::default())?;
//! // `outline` is the shape in neutral em space; read it directly
//! // (outline.contours / outline.to_bezpaths() / serde_json), or place it:
//! let placed = place(&outline, &FontMetrics::default());
//! # Ok::<(), Box<dyn std::error::Error>>(())
//! ```
//!
//! # Feature flags
//!
//! | Feature | Default | Effect |
//! |---------|---------|--------|
//! | `ufo` | yes | UFO reading/writing via [norad] (`ufo`, `masters`) |
//! | `parallel` | yes | Per-contour parallelism via rayon |
//! | `render` | yes | PNG rendering / raster comparison via tiny-skia |
//! | `cli` | yes | The `img2bez` binary (implies `ufo` + `render`) |
//! | `experimental-refit` | no | Master-compatibility refit (unstable) |
//!
//! # Debug environment variables
//!
//! `IMG2BEZ_DEBUG_FIT` (pipeline splits and fit errors),
//! `IMG2BEZ_DEBUG_FLATS` (junction-flat decisions),
//! `IMG2BEZ_DEBUG_NO_CLEANUP` (skip all post-processing),
//! `IMG2BEZ_DEBUG_PIXELDIFF` (save a 1:1 pixel diff image),
//! `IMG2BEZ_DEBUG_JOINT` / `IMG2BEZ_DEBUG_JOINT_EXTREMA` (joint masters),
//! `IMG2BEZ_PHOTO_FAIR_DEV` (photo fairing-threshold override).
//!
//! Two logging variables cause **file writes** when set:
//! `IMG2BEZ_LOG_DECISIONS=<path>` appends a JSONL record of each internal
//! pipeline decision (with `IMG2BEZ_LOG_TAG` labeling the rows) — training
//! data collection, off unless the variable is present in the process
//! environment.

#![warn(missing_docs)]
#![allow(clippy::many_single_char_names)]

mod font;
mod io;
mod ml;
mod model;
mod pipeline;

pub use font::compat;
#[cfg(feature = "ufo")]
pub use font::masters;
/// Experimental master-compatibility refit. Off by default; enable the
/// `experimental-refit` feature. Not production-ready (see the module docs).
#[cfg(feature = "experimental-refit")]
pub use font::refit;
#[cfg(feature = "ufo")]
pub use io::ufo;
pub use ml::judge;
pub use pipeline::placement;

// CLI/eval-harness support: public for the binary target, doc-hidden,
// not covered by semver.
#[cfg(feature = "ufo")]
#[doc(hidden)]
pub use io::eval;

// CLI rendering support (comparison PNGs): public for the binary target,
// doc-hidden, not covered by semver.
#[cfg(feature = "render")]
#[doc(hidden)]
pub use io::render;

/// Re-export of [kurbo](https://docs.rs/kurbo), the curve geometry library —
/// the version [`Outline::to_bezpaths`] uses. kurbo enters the public API only
/// through that adapter; the `Outline` model itself is plain `f64`, so the
/// core type is independent of the kurbo version.
pub use kurbo;

/// Re-export of [image](https://docs.rs/image), the raster codec library:
/// `image::GrayImage` appears in the [`placement`] and [`ImageStats`] APIs,
/// so callers must use the version this crate was built with.
pub use image;

/// Re-export of [norad](https://docs.rs/norad), the UFO reader/writer:
/// `norad::Font` and `norad::Glyph` appear in the [`ufo`] module's API, so
/// callers must use the version this crate was built with.
#[cfg(feature = "ufo")]
pub use norad;

pub use font::compat::{CompatibilityReport, ContourReport, make_compatible};
pub use font::masters_flow::{trace_masters, trace_place_masters};
#[cfg(feature = "ufo")]
pub use io::ufo::{ReferenceMetrics, read_reference};
pub use ml::heads::{set_corner_head, set_site_head};
pub use ml::image_stats::{ImageStats, measure};
pub use model::config::{
    FontMetrics, Profile, Style, ThresholdMethod, TraceMode, TraceOptions,
};
pub use model::error::TraceError;
pub use model::glyph::{Advance, Glyph};
pub use model::outline::{Contour, Outline, OutlinePoint, PointKind};
pub use pipeline::placement::{
    PlacedGlyph, PlacementOptions, PlacementReport, Sidebearings, SourceBox,
    TargetBand, place,
};

use std::path::Path;
use std::time::Instant;

use io::bitmap;
use pipeline::preprocess::trace_luma;

/// Trace image bytes into a font-ready [`Outline`].
///
/// The image is traced with the sub-pixel pipeline (marching-squares
/// iso-contours at the threshold level + type-quality curve fitting). Inputs
/// are expected to carry anti-aliasing at the ink boundary (rendered glyphs,
/// grayscale scans).
///
/// The result is in neutral em space — y-up, the image height mapped to
/// `opts.em_height`, no baseline shift, sidebearings, or advance applied.
/// Use [`place`] to position it into a font, or read the [`Outline`] directly.
///
/// # Errors
///
/// [`TraceError::ImageLoad`] if the bytes do not decode as a supported image
/// (PNG, JPEG, BMP); [`TraceError::NoContours`] if thresholding finds no ink.
pub fn trace(
    image_bytes: &[u8],
    opts: &TraceOptions,
) -> Result<Outline, TraceError> {
    let t_start = trace_timer_now();
    let raw = bitmap::load_luma_bytes(image_bytes)?;
    trace_luma(raw, opts, t_start)
}

/// Measure no-reference [`ImageStats`] for raw image bytes.
///
/// Decodes like [`trace`]. Reports the input's character without tracing —
/// e.g. which profile it classifies as ([`ImageStats::classify`]).
///
/// # Errors
///
/// [`TraceError::ImageLoad`] if the bytes do not decode as a supported image.
pub fn measure_image(image_bytes: &[u8]) -> Result<ImageStats, TraceError> {
    Ok(measure(&bitmap::load_luma_bytes(image_bytes)?))
}

/// Trace an image file into a font-ready [`Outline`] (see [`trace`]).
///
/// # Errors
///
/// As [`trace`], with [`TraceError::Io`] additionally covering an unreadable
/// path.
pub fn trace_file(
    image_path: &Path,
    opts: &TraceOptions,
) -> Result<Outline, TraceError> {
    let t_start = trace_timer_now();
    let raw = bitmap::load_luma(image_path)?;
    trace_luma(raw, opts, t_start)
}

/// Whether a codepoint belongs to a right-to-left script (Arabic or
/// Hebrew blocks, including presentation forms). Drives the contour
/// start-point rule: RTL glyphs start contours bottom-right.
pub fn is_rtl_codepoint(c: char) -> bool {
    matches!(u32::from(c),
        0x0590..=0x05FF          // Hebrew
        | 0x0600..=0x06FF        // Arabic
        | 0x0750..=0x077F        // Arabic Supplement
        | 0x08A0..=0x08FF        // Arabic Extended-A
        | 0xFB1D..=0xFB4F        // Hebrew presentation forms
        | 0xFB50..=0xFDFF        // Arabic presentation forms A
        | 0xFE70..=0xFEFF) // Arabic presentation forms B
}

/// Normalize contour start points consistently across an interpolation-
/// compatible master set: the start is chosen on `outlines[0]` (see
/// [`Contour::normalize_start`]) and every master rotates by the same point
/// index, so the start rule can never break interpolation. Contours whose
/// point counts differ across masters are left untouched (already
/// incompatible; [`make_compatible`]'s report shows it).
pub fn normalize_master_starts(outlines: &mut [Outline], rtl: bool) {
    if outlines.is_empty() {
        return;
    }
    let n_contours = outlines[0].contours.len();
    for c in 0..n_contours {
        if outlines.iter().any(|o| {
            o.contours.get(c).map(|k| k.points.len())
                != Some(outlines[0].contours[c].points.len())
        }) {
            continue;
        }
        // Same selection rule as Contour::normalize_start, on master 0.
        let mut best: Option<(usize, f64, f64)> = None;
        for (i, p) in outlines[0].contours[c].points.iter().enumerate() {
            if p.kind == PointKind::OffCurve {
                continue;
            }
            let better = match best {
                None => true,
                Some((_, by, bx)) => {
                    p.y < by - 1e-9
                        || (p.y < by + 1e-9
                            && if rtl {
                                p.x > bx + 1e-9
                            } else {
                                p.x < bx - 1e-9
                            })
                }
            };
            if better {
                best = Some((i, p.y, p.x));
            }
        }
        if let Some((i, _, _)) = best {
            for o in outlines.iter_mut() {
                o.contours[c].points.rotate_left(i);
            }
        }
    }
}

/// Trace image bytes, place the outline with font metrics, and assemble a
/// UFO-faithful [`Glyph`].
///
/// This is the high-level entry point for editors, agent tools, and web
/// bindings that want the same structured output the GLIF/JSON writers use.
///
/// ```no_run
/// use img2bez::{trace_glyph, FontMetrics, TraceOptions};
///
/// let bytes = std::fs::read("A.png")?;
/// let glyph = trace_glyph(
///     &bytes,
///     "A",
///     &['A'],
///     &TraceOptions::default(),
///     &FontMetrics::default(),
/// )?;
/// println!("{} points", glyph.outline.contours[0].points.len());
/// # Ok::<(), Box<dyn std::error::Error>>(())
/// ```
///
/// # Errors
///
/// As [`trace`].
pub fn trace_glyph(
    image_bytes: &[u8],
    name: impl Into<String>,
    codepoints: &[char],
    opts: &TraceOptions,
    metrics: &FontMetrics,
) -> Result<Glyph, TraceError> {
    // Derive the RTL start rule from the codepoints so callers don't have to.
    let mut opts = opts.clone();
    opts.rtl_start |= codepoints.iter().copied().any(is_rtl_codepoint);
    let outline = trace(image_bytes, &opts)?;
    Ok(glyph_from_outline(name, codepoints, &outline, metrics))
}

/// Trace an image file and assemble a placed [`Glyph`] (see [`trace_glyph`]).
///
/// # Errors
///
/// As [`trace_file`].
pub fn trace_glyph_path(
    image_path: &Path,
    name: impl Into<String>,
    codepoints: &[char],
    opts: &TraceOptions,
    metrics: &FontMetrics,
) -> Result<Glyph, TraceError> {
    let mut opts = opts.clone();
    opts.rtl_start |= codepoints.iter().copied().any(is_rtl_codepoint);
    let outline = trace_file(image_path, &opts)?;
    Ok(glyph_from_outline(name, codepoints, &outline, metrics))
}

/// Place an existing neutral outline and assemble a UFO-faithful [`Glyph`].
/// Free-function form of [`Glyph::from_outline`].
pub fn glyph_from_outline(
    name: impl Into<String>,
    codepoints: &[char],
    outline: &Outline,
    metrics: &FontMetrics,
) -> Glyph {
    Glyph::from_outline(name, codepoints, outline, metrics)
}

/// Assemble a UFO-faithful [`Glyph`] from an already placed outline.
/// Free-function form of [`Glyph::from_placed`].
pub fn glyph_from_placed(
    name: impl Into<String>,
    codepoints: &[char],
    placed: &PlacedGlyph,
    metrics: &FontMetrics,
) -> Glyph {
    Glyph::from_placed(name, codepoints, placed, metrics)
}

/// Per-image placement geometry: image dims, ink bounds (px), px scale.
pub(crate) type PlacementGeom = ((u32, u32), kurbo::Rect, f64);

/// Pre-trace half of [`trace_place`] / [`trace_place_masters`]: detect the
/// ink box, derive the em height that fits the source box to the target band
/// (so tracing runs at font scale), and return the adjusted options plus the
/// geometry [`placement::position`] needs.
pub(crate) fn placement_prework(
    raw: &image::GrayImage,
    opts: &TraceOptions,
    placement: &PlacementOptions,
) -> Result<(TraceOptions, PlacementGeom), TraceError> {
    let (w, h) = raw.dimensions();
    let threshold = bitmap::resolve_threshold(raw, opts);

    let ink = placement::ink_bounds(raw, threshold, opts.invert);
    let ink_px = ink.unwrap_or(kurbo::Rect::new(0.0, 0.0, w as f64, h as f64));

    // Source-box height as a fraction of image height: resolution-invariant.
    let source_h_px = match placement.source {
        SourceBox::Canvas => h as f64,
        SourceBox::InkBounds => ink.ok_or(TraceError::NoContours)?.height(),
        SourceBox::Custom(r) => r.height(),
    };
    if source_h_px <= 0.0 || h == 0 {
        return Err(TraceError::NoContours);
    }
    let source_frac = source_h_px / h as f64;
    // Scale the image so the source box exactly fills the target band height.
    let em_height = placement.vertical.height() / source_frac;

    let mut topts = opts.clone();
    topts.em_height = em_height;
    let scale = em_height / h as f64; // font units per source pixel
    Ok((topts, ((w, h), ink_px, scale)))
}

/// Trace an image and place it deterministically from explicit
/// [`PlacementOptions`].
///
/// Unlike `trace` + `place` (which assume the image is framed to the font's
/// vertical extent), this detects the source box, computes the scale that fits
/// it to the target band *before* tracing — so the cleanup heuristics run at
/// font scale regardless of image padding — then positions the result and
/// returns a [`PlacementReport`]. This is the headless path for generated
/// rasters, where padding is arbitrary.
///
/// # Errors
///
/// As [`trace`]; also [`TraceError::NoContours`] when an ink-bounds source box
/// is requested and the image has no ink to measure.
pub fn trace_place(
    image_bytes: &[u8],
    opts: &TraceOptions,
    placement: &PlacementOptions,
) -> Result<(PlacedGlyph, PlacementReport), TraceError> {
    let t_start = trace_timer_now();
    let raw = bitmap::load_luma_bytes(image_bytes)?;
    let (topts, (dims, ink_px, scale)) =
        placement_prework(&raw, opts, placement)?;
    let outline = trace_luma(raw, &topts, t_start)?;
    Ok(placement::position(
        &outline, placement, dims, ink_px, scale,
    ))
}

fn trace_timer_now() -> Option<Instant> {
    #[cfg(target_arch = "wasm32")]
    {
        None
    }
    #[cfg(not(target_arch = "wasm32"))]
    {
        Some(Instant::now())
    }
}

/// Convenience: trace an image file, place it, and write it into a UFO.
///
/// # Errors
///
/// As [`trace_file`]; plus the `ufo` error variants ([`TraceError::Norad`],
/// [`TraceError::NoradWrite`]) when the UFO cannot be opened or written.
#[cfg(feature = "ufo")]
pub fn trace_into_ufo(
    image_path: &Path,
    glyph_name: &str,
    codepoints: &[char],
    ufo_path: &Path,
    opts: &TraceOptions,
    metrics: &FontMetrics,
) -> Result<(), TraceError> {
    let outline = trace_file(image_path, opts)?;
    let placed = place(&outline, metrics);
    write_into_ufo(glyph_name, codepoints, &placed, ufo_path, metrics)
}

/// Write an already-placed glyph into a UFO (created if missing). The
/// UFO-writing half of [`trace_into_ufo`], also used by the multi-master path.
///
/// # Errors
///
/// [`TraceError::Norad`] / [`TraceError::NoradWrite`] when the UFO cannot be
/// opened or written.
#[cfg(feature = "ufo")]
pub fn write_into_ufo(
    glyph_name: &str,
    codepoints: &[char],
    placed: &PlacedGlyph,
    ufo_path: &Path,
    metrics: &FontMetrics,
) -> Result<(), TraceError> {
    let glyph = ufo::to_glyph(glyph_name, placed, codepoints)?;
    let mut font = ufo::open_or_create_font(ufo_path, metrics)?;
    font.default_layer_mut().insert_glyph(glyph);
    font.save(ufo_path)?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use image::GrayImage;
    use kurbo::PathEl;

    #[test]
    fn classify_crisp_bilevel_is_wild() {
        // Hard-edged bilevel art classifies Wild and is never auto-blurred
        // (the gate-safety property).
        let mut img = GrayImage::from_pixel(120, 120, image::Luma([255]));
        for y in 30..90 {
            for x in 30..90 {
                img.put_pixel(x, y, image::Luma([0]));
            }
        }
        assert_eq!(measure(&img).classify(), Profile::Wild);
    }

    #[test]
    fn classify_soft_low_contrast_is_photo() {
        // A smooth low-contrast ramp (the soft-scan signature) classifies
        // Photo.
        let mut img = GrayImage::new(120, 120);
        for y in 0..120 {
            for x in 0..120 {
                let v = 90.0 + 70.0 * x as f64 / 119.0; // mid-gray, linear ramp
                img.put_pixel(x, y, image::Luma([v as u8]));
            }
        }
        assert_eq!(measure(&img).classify(), Profile::Photo);
    }

    #[test]
    fn trace_rectangle_bitmap() {
        // A 40×40 white image with a dark 20×20 rectangle (ink = dark).
        let mut img = GrayImage::from_pixel(40, 40, image::Luma([255]));
        for y in 10..30 {
            for x in 10..30 {
                img.put_pixel(x, y, image::Luma([0]));
            }
        }
        let config = TraceOptions {
            em_height: 100.0,
            ..TraceOptions::default()
        };
        let paths = pipeline::vectorize::trace_subpixel(&img, 128, &config);
        assert_eq!(paths.len(), 1, "Expected 1 contour, got {}", paths.len());
        let elements = paths[0].elements();
        assert!(!elements.is_empty(), "Path should not be empty");
        assert!(
            matches!(elements[0], PathEl::MoveTo(_)),
            "First element should be MoveTo"
        );
        assert!(
            matches!(elements.last().unwrap(), PathEl::ClosePath),
            "Last element should be ClosePath"
        );
    }
}
