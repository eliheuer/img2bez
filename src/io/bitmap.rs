// Copyright 2026 the img2bez Authors
// SPDX-License-Identifier: Apache-2.0 OR MIT

//! Grayscale image loading and binarization threshold selection.

use std::path::Path;

use image::{GrayImage, ImageReader};
use imageproc::contrast::otsu_level;

use crate::model::config::{ThresholdMethod, TraceOptions};
use crate::model::error::TraceError;

/// Load an image as raw (un-thresholded) grayscale.
pub fn load_luma(path: &Path) -> Result<GrayImage, TraceError> {
    Ok(ImageReader::open(path)?.decode()?.into_luma8())
}

/// Decode an image from memory as raw (un-thresholded) grayscale.
pub fn load_luma_bytes(bytes: &[u8]) -> Result<GrayImage, TraceError> {
    Ok(image::load_from_memory(bytes)?.into_luma8())
}

/// Resolve the binarization threshold level for an image.
pub fn resolve_threshold(img: &GrayImage, config: &TraceOptions) -> u8 {
    match config.threshold {
        ThresholdMethod::Fixed(t) => t,
        ThresholdMethod::Otsu => {
            let t = otsu_level(img);
            if config.verbose {
                eprintln!("  Threshold   Otsu = {}", t);
            }
            t
        }
    }
}
