//! img2bez: bitmap image → font-ready bezier contours.
//!
//! Traces bitmap glyphs into clean cubic bezier paths
//! suitable for font editors and compilation pipelines.
//!
//! # Example
//!
//! ```no_run
//! use img2bez::{trace, TracingConfig};
//! use std::path::Path;
//!
//! let config = TracingConfig::default();
//! let result = trace(Path::new("glyph.png"), &config)?;
//! // result.paths contains Vec<kurbo::BezPath>
//! # Ok::<(), img2bez::TraceError>(())
//! ```

#![forbid(unsafe_code)]

mod bitmap;
mod config;
mod geom;
mod metrics;
mod cleanup;
mod vectorize;

pub mod error;

#[cfg(feature = "ufo")]
pub mod ufo;

#[cfg(feature = "ufo")]
pub mod eval;

pub mod render;

// Re-export kurbo so downstream users get the same version
// used by TraceResult.paths (Vec<kurbo::BezPath>).
pub use kurbo;

pub use config::{ThresholdMethod, TracingConfig};
pub use error::TraceError;

use kurbo::{BezPath, PathEl};
use std::path::Path;
use std::time::Instant;

use geom::signed_area;

/// Whether a contour is an outer boundary or a hole.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ContourType {
    Outer,
    Counter,
}

/// The result of tracing: font-ready bezier contours.
#[derive(Debug, Clone)]
pub struct TraceResult {
    /// Traced contours as kurbo BezPaths.
    pub paths: Vec<BezPath>,
    /// Classification of each contour.
    pub contour_types: Vec<ContourType>,
    /// Computed advance width in font units.
    pub advance_width: f64,
    /// Translation (dx, dy) applied by the reposition step.
    /// Needed to map font coordinates back to source image coordinates.
    pub reposition_shift: (f64, f64),
}

/// Full pipeline: image path → font-ready bezier contours.
///
/// Pipeline: pixel-edge contour extraction on the dual grid,
/// optimal polygon approximation via DP, and alpha-based curve generation.
pub fn trace(image_path: &Path, config: &TracingConfig) -> Result<TraceResult, TraceError> {
    let t_start = Instant::now();

    // ── Load & threshold ──────────────────────────────────
    let gray = bitmap::load_and_threshold(image_path, config)?;
    let (w, h) = gray.dimensions();
    let scale = config.target_height / h as f64;
    let threshold_name = match config.threshold {
        ThresholdMethod::Otsu => "Otsu".to_string(),
        ThresholdMethod::Fixed(t) => format!("fixed {}", t),
    };
    eprintln!("  Load        {}x{} px, {} threshold", w, h, threshold_name);

    // ── Vectorize ─────────────────────────────────────────
    let curves = vectorize::trace(&gray, config);
    if curves.is_empty() {
        return Err(TraceError::NoContours);
    }
    let (raw_curves, raw_lines) = count_segments(&curves);
    eprintln!(
        "  Trace       {} contours \u{2192} {} curves + {} lines  (scale \u{00d7}{:.2})",
        curves.len(), raw_curves, raw_lines, scale,
    );

    // ── Post-processing ───────────────────────────────────
    let paths = if std::env::var("IMG2BEZ_DEBUG_NO_CLEANUP").is_ok() {
        eprintln!("  Debug       skipping cleanup");
        curves.clone()
    } else {
        cleanup::process(&curves, config)
    };

    let contour_types: Vec<ContourType> = paths
        .iter()
        .map(|path| {
            if signed_area(path) >= 0.0 {
                ContourType::Outer
            } else {
                ContourType::Counter
            }
        })
        .collect();
    let mut steps: Vec<&str> = Vec::new();
    if config.fix_direction { steps.push("direction"); }
    if config.grid > 0 { steps.push("grid snap"); }
    steps.push("H/V snap");
    if config.chamfer_size > 0.0 { steps.push("chamfer"); }
    eprintln!("  Clean       {}", steps.join(" \u{00b7} "));

    // ── Metrics ───────────────────────────────────────────
    let (paths, reposition_shift) = metrics::reposition(&paths, config.lsb, config.grid);
    let advance_width = config
        .advance_width
        .unwrap_or_else(|| metrics::advance_from_bounds(&paths, config.rsb));

    let n_outer = contour_types.iter().filter(|t| **t == ContourType::Outer).count();
    let n_counter = contour_types.len() - n_outer;
    let (on, off) = count_points(&paths);
    let elapsed = t_start.elapsed().as_millis();
    eprintln!(
        "  Result      {} contours ({} outer, {} counter) \u{00b7} {} points ({} on-curve) \u{00b7} width {}  ({}ms)",
        contour_types.len(), n_outer, n_counter,
        on + off, on, advance_width as i64, elapsed,
    );

    Ok(TraceResult {
        paths,
        contour_types,
        advance_width,
        reposition_shift,
    })
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
                PathEl::CurveTo(..) => { on += 1; off += 2; }
                PathEl::LineTo(_) => { on += 1; }
                PathEl::QuadTo(..) => { on += 1; off += 1; }
                _ => {}
            }
        }
    }
    (on, off)
}

/// Convenience: trace and write directly into a UFO.
#[cfg(feature = "ufo")]
pub fn trace_into_ufo(
    image_path: &Path,
    glyph_name: &str,
    ufo_path: &Path,
    config: &TracingConfig,
) -> Result<(), TraceError> {
    let result = trace(image_path, config)?;
    let glyph = ufo::to_glyph(glyph_name, &result, config)?;
    let mut font = norad::Font::load(ufo_path)?;
    font.default_layer_mut().insert_glyph(glyph);
    font.save(ufo_path)?;
    Ok(())
}
