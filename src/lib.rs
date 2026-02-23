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
mod contour;
mod corners;
mod postprocess;
mod simplify;

pub mod error;

#[cfg(feature = "ufo")]
pub mod output;

// Re-export kurbo so downstream users get the same version
// used by TraceResult.paths (Vec<kurbo::BezPath>).
pub use kurbo;

pub use config::{ThresholdMethod, TracingConfig};
pub use error::TraceError;

use kurbo::{Affine, BezPath, Shape, Vec2};
use std::path::Path;

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
}

/// Full pipeline: image path → font-ready bezier contours.
pub fn trace(image_path: &Path, config: &TracingConfig) -> Result<TraceResult, TraceError> {
    let gray = bitmap::load_and_threshold(image_path, config)?;
    let (_, image_height) = gray.dimensions();

    let raw = contour::detect(&gray, config)?;
    if raw.is_empty() {
        return Err(TraceError::NoContours);
    }

    let contour_types: Vec<ContourType> = raw
        .iter()
        .map(|c| {
            if c.is_outer {
                ContourType::Outer
            } else {
                ContourType::Counter
            }
        })
        .collect();

    let annotated = corners::detect(&raw, config);
    let curves = simplify::fit(&annotated, image_height, config);
    let paths = postprocess::process(&curves, config);
    let paths = reposition(&paths, config.lsb);

    let advance_width = config
        .advance_width
        .unwrap_or_else(|| advance_from_bounds(&paths, config.rsb));

    Ok(TraceResult {
        paths,
        contour_types,
        advance_width,
    })
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
    let glyph = output::ufo::to_glyph(glyph_name, &result, config)?;
    let mut font = norad::Font::load(ufo_path)?;
    font.default_layer_mut().insert_glyph(glyph);
    font.save(ufo_path)?;
    Ok(())
}

// ── Internal helpers ─────────────────────────────────────

/// Shift paths so bottom sits on baseline, left at LSB.
fn reposition(paths: &[BezPath], lsb: f64) -> Vec<BezPath> {
    let mut min_x = f64::MAX;
    let mut min_y = f64::MAX;
    for path in paths {
        let bb = path.bounding_box();
        min_x = min_x.min(bb.x0);
        min_y = min_y.min(bb.y0);
    }
    if min_x == f64::MAX {
        return paths.to_vec();
    }
    let dx = lsb - min_x;
    let dy = -min_y;
    paths.iter().map(|p| translate(p, dx, dy)).collect()
}

fn advance_from_bounds(paths: &[BezPath], rsb: f64) -> f64 {
    let mut max_x = f64::MIN;
    for path in paths {
        max_x = max_x.max(path.bounding_box().x1);
    }
    if max_x == f64::MIN {
        0.0
    } else {
        max_x + rsb
    }
}

fn translate(path: &BezPath, dx: f64, dy: f64) -> BezPath {
    let mut p = path.clone();
    p.apply_affine(Affine::translate(Vec2::new(dx, dy)));
    p
}
