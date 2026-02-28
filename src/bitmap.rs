use std::path::Path;

use image::{GrayImage, ImageReader};
use imageproc::contrast::otsu_level;

use crate::config::{ThresholdMethod, TracingConfig};
use crate::error::TraceError;

/// Load an image and convert it to a binary (black/white) GrayImage.
///
/// Foreground (glyph) pixels are 255, background pixels are 0.
pub fn load_and_threshold(path: &Path, config: &TracingConfig) -> Result<GrayImage, TraceError> {
    let img = ImageReader::open(path)
        .map_err(|e| TraceError::ImageLoad(e.to_string()))?
        .decode()
        .map_err(|e| TraceError::ImageLoad(e.to_string()))?
        .into_luma8();

    let threshold = match config.threshold {
        ThresholdMethod::Fixed(t) => t,
        ThresholdMethod::Otsu => {
            let t = otsu_level(&img);
            eprintln!("  Threshold   Otsu = {}", t);
            t
        }
    };

    let mut binary =
        imageproc::contrast::threshold(&img, threshold, imageproc::contrast::ThresholdType::BinaryInverted);

    if config.invert {
        for pixel in binary.pixels_mut() {
            pixel.0[0] = 255 - pixel.0[0];
        }
    }

    // Debug: save thresholded bitmap
    if std::env::var("IMG2BEZ_DEBUG_BITMAP").is_ok() {
        binary.save("debug_threshold.png").ok();
        eprintln!("  Debug       saved debug_threshold.png");
    }

    Ok(binary)
}
