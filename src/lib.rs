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
mod corners;
mod fit;
mod metrics;
mod cleanup;
mod trace;

pub mod error;

#[cfg(feature = "ufo")]
pub mod ufo;

// Re-export kurbo so downstream users get the same version
// used by TraceResult.paths (Vec<kurbo::BezPath>).
pub use kurbo;

pub use config::{ThresholdMethod, TracingConfig};
pub use error::TraceError;

use kurbo::BezPath;
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

    let raw = trace::extract(&gray, config)?;
    if raw.is_empty() {
        return Err(TraceError::NoContours);
    }

    let contour_types: Vec<ContourType> = raw
        .iter()
        .map(|contour| {
            if contour.is_outer {
                ContourType::Outer
            } else {
                ContourType::Counter
            }
        })
        .collect();

    let annotated = corners::annotate(&raw, config);
    let curves = fit::curves(&annotated, image_height, config);
    let paths = cleanup::process(&curves, config);
    let paths = metrics::reposition(&paths, config.lsb);

    let advance_width = config
        .advance_width
        .unwrap_or_else(|| metrics::advance_from_bounds(&paths, config.rsb));

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
    let glyph = ufo::to_glyph(glyph_name, &result, config)?;
    let mut font = norad::Font::load(ufo_path)?;
    font.default_layer_mut().insert_glyph(glyph);
    font.save(ufo_path)?;
    Ok(())
}
