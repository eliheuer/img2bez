// Copyright 2026 the img2bez Authors
// SPDX-License-Identifier: Apache-2.0 OR MIT

//! Error types for the tracing pipeline: every stage maps its failures into
//! the single [`TraceError`] enum.

use thiserror::Error;

/// Errors that can occur during image tracing.
#[derive(Error, Debug)]
#[non_exhaustive]
pub enum TraceError {
    /// The source image bytes could not be decoded.
    #[error("failed to decode image: {0}")]
    ImageLoad(#[from] image::ImageError),

    /// The source image file could not be read.
    #[error("failed to read image file: {0}")]
    Io(#[from] std::io::Error),

    /// Thresholding found no ink at all.
    #[error("no contours found in image")]
    NoContours,

    /// A master set is not interpolation-compatible (and could not be made
    /// so); the message describes the first mismatch.
    #[error("masters are not compatible: {0}")]
    MastersIncompatible(String),

    /// A vertical fit spec (`zone:zone`) could not be resolved against the
    /// font's metrics.
    #[cfg(feature = "ufo")]
    #[error("invalid fit spec: {0}")]
    InvalidFitSpec(String),

    /// The target UFO could not be loaded.
    #[cfg(feature = "ufo")]
    #[error("norad error: {0}")]
    Norad(#[from] norad::error::FontLoadError),

    /// The target UFO could not be written.
    #[cfg(feature = "ufo")]
    #[error("norad write error: {0}")]
    NoradWrite(#[from] norad::error::FontWriteError),

    /// A reference `.glif` could not be loaded.
    #[cfg(feature = "ufo")]
    #[error("failed to load .glif file: {0}")]
    GlifLoad(#[from] norad::error::GlifLoadError),

    /// A UFO contour could not be converted to a kurbo path.
    #[cfg(feature = "ufo")]
    #[error("contour conversion failed: {0}")]
    ContourConvert(String),

    /// A designspace could not be read or written.
    #[cfg(feature = "ufo")]
    #[error("designspace error: {0}")]
    Designspace(String),

    /// A reference glyph named for metric copying was not found in the UFO.
    #[cfg(feature = "ufo")]
    #[error("glyph {glyph:?} not found in {ufo}")]
    #[non_exhaustive]
    ReferenceGlyphNotFound {
        /// The UFO path that was searched.
        ufo: String,
        /// The glyph name that was requested.
        glyph: String,
    },
}
