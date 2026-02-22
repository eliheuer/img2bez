pub mod bitmap;
pub mod config;
pub mod contour;
pub mod corners;
pub mod error;
pub mod postprocess;
pub mod refit;
pub mod simplify;

#[cfg(feature = "ufo")]
pub mod output;

pub use config::TracingConfig;
pub use error::TraceError;

use kurbo::Shape;
use std::path::Path;

/// Whether a contour is an outer boundary or an inner counter (hole).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ContourType {
    Outer,
    Counter,
}

/// The result of tracing an image: font-ready bezier contours.
///
/// This is format-independent. Convert to norad (UFO), fontir, or
/// use the `kurbo::BezPath`s directly in your application.
#[derive(Debug, Clone)]
pub struct TraceResult {
    /// The traced contours as kurbo BezPaths.
    pub paths: Vec<kurbo::BezPath>,
    /// Classification of each contour (outer or counter).
    pub contour_types: Vec<ContourType>,
    /// Computed advance width.
    pub advance_width: f64,
}

/// Full pipeline: image path -> font-ready bezier contours.
pub fn trace(image_path: &Path, config: &TracingConfig) -> Result<TraceResult, TraceError> {
    // 1. Load and threshold
    let gray = bitmap::load_and_threshold(image_path, config)?;
    let (_, image_height) = gray.dimensions();

    // 2. Detect pixel contours
    let raw = contour::detect(&gray, config)?;
    if raw.is_empty() {
        return Err(TraceError::NoContours);
    }

    // Remember contour types before we lose that info
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

    // 3. Detect corners
    let annotated = corners::detect(&raw, config);

    // 4. Fit curves (scales to font units internally)
    let curves = simplify::fit(&annotated, image_height, config);

    // 5. Post-process
    let paths = postprocess::process(&curves, config);

    // 6. Compute advance width
    let advance_width = config.advance_width.unwrap_or_else(|| {
        let mut min_x = f64::MAX;
        let mut max_x = f64::MIN;
        for path in &paths {
            let bbox = path.bounding_box();
            min_x = min_x.min(bbox.x0);
            max_x = max_x.max(bbox.x1);
        }
        if min_x > max_x {
            0.0
        } else {
            (max_x - min_x) + config.lsb + config.rsb
        }
    });

    Ok(TraceResult {
        paths,
        contour_types,
        advance_width,
    })
}

/// Convenience: trace an image and write the glyph directly into a UFO.
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
