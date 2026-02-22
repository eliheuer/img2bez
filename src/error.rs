use thiserror::Error;

#[derive(Error, Debug)]
pub enum TraceError {
    #[error("failed to load image: {0}")]
    ImageLoad(String),

    #[error("no contours found in image")]
    NoContours,

    #[error("empty contour")]
    EmptyContour,

    #[error("invalid path: {0}")]
    InvalidPath(String),

    #[cfg(feature = "ufo")]
    #[error("norad error: {0}")]
    Norad(#[from] norad::error::FontLoadError),

    #[cfg(feature = "ufo")]
    #[error("norad write error: {0}")]
    NoradWrite(#[from] norad::error::FontWriteError),
}
