/// All tracing parameters in one struct.
///
/// Use `TracingConfig::default()` for sensible defaults,
/// then override specific fields:
///
/// ```
/// use img2bez::TracingConfig;
///
/// let config = TracingConfig {
///     grid: 2,
///     chamfer_size: 16.0,
///     ..TracingConfig::default()
/// };
/// ```
#[derive(Debug, Clone, PartialEq)]
pub struct TracingConfig {
    // -- Bitmap stage --
    /// Threshold method for converting to binary.
    pub threshold: ThresholdMethod,
    /// If true, invert the image (swap foreground/background) before tracing.
    pub invert: bool,

    // -- Contour stage --
    /// Minimum contour area in pixels (filter speckles).
    pub min_contour_area: f64,

    // -- Curve fitting --
    /// Accuracy for kurbo fit_to_bezpath_opt (Frechet distance tolerance
    /// in font units). Smaller = more points, closer fit.
    /// 4.0 is good for type design (clean curves, few points).
    pub fit_accuracy: f64,
    /// Smoothing iterations applied to polyline points before curve fitting.
    /// Removes pixel staircase noise. 0 = no smoothing.
    pub smooth_iterations: usize,
    /// Maximum alpha for smooth curves (corner detection threshold).
    /// Vertices with alpha >= this become corners.
    /// Lower = more corners = shorter smooth sections = tighter fits.
    /// Range: 0.0 (everything is a corner) to 1.34 (nothing is a corner).
    /// Default: 1.0. Try 0.6–0.8 for geometric type.
    pub alphamax: f64,

    // -- Post-processing --
    /// Grid size for coordinate snapping. 0 = no snapping.
    pub grid: i32,
    /// Whether to correct contour direction (CCW outer, CW counter).
    pub fix_direction: bool,
    /// Chamfer size. 0 = no chamfers. 16 = Virtua Grotesk Regular.
    pub chamfer_size: f64,
    /// Minimum edge length to chamfer (edges shorter than this are skipped).
    pub chamfer_min_edge: f64,

    // -- Scaling / output --
    /// Advance width. If None, computed from contour bounding box + sidebearings.
    pub advance_width: Option<f64>,
    /// Left sidebearing. Used when advance_width is None.
    pub lsb: f64,
    /// Right sidebearing. Used when advance_width is None.
    pub rsb: f64,
    /// Target height in font units (ascender - descender, typically).
    /// The traced contour is scaled so the image height maps to this value.
    pub target_height: f64,
    /// Y offset after scaling (to align baseline).
    /// Typically the descender value (negative).
    pub y_offset: f64,
    /// Unicode codepoints to assign (used by output backends).
    pub codepoints: Vec<char>,
}

/// Threshold method for converting a grayscale image to binary.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ThresholdMethod {
    /// Fixed brightness threshold (0-255).
    Fixed(u8),
    /// Otsu's method (automatic).
    Otsu,
}

impl Default for TracingConfig {
    fn default() -> Self {
        Self {
            threshold: ThresholdMethod::Otsu, // adaptive, works for most scans
            invert: false,
            min_contour_area: 100.0, // ~10×10 px: filters dust/noise
            fit_accuracy: 4.0,       // font units; 2–4 is good for type
            smooth_iterations: 3,    // removes pixel staircase; >5 loses detail
            alphamax: 1.0,           // ~45° corner threshold; 0.6 for geometric type
            grid: 0,                 // off by default; set 2 for VG
            fix_direction: true,     // required for correct font rendering
            chamfer_size: 0.0,       // off; 16 for Virtua Grotesk style
            chamfer_min_edge: 40.0,  // don't chamfer edges shorter than this
            advance_width: None,     // auto-compute from bbox + sidebearings
            lsb: 50.0,               // left sidebearing in font units
            rsb: 50.0,               // right sidebearing in font units
            target_height: 1000.0,   // UPM-relative; 1088 for VG (asc-desc)
            y_offset: 0.0,           // set to descender (e.g. -256) to align baseline
            codepoints: vec![],
        }
    }
}
