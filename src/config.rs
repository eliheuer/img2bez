/// All tracing parameters in one struct.
/// Designed to be serializable (for saving presets) and
/// adjustable at runtime (for editor sliders).
#[derive(Debug, Clone)]
pub struct TracingConfig {
    // -- Bitmap stage --
    /// Threshold method for converting to binary.
    pub threshold: ThresholdMethod,
    /// If true, invert the image (swap foreground/background) before tracing.
    pub invert: bool,

    // -- Contour stage --
    /// Minimum contour area in pixels (filter speckles).
    pub min_contour_area: f64,

    // -- Corner detection --
    /// Angle change threshold for corner detection (radians).
    /// Points where the polyline turns more than this are marked as corners.
    /// Lower = more corners. ~0.5 (30 deg) for geometric type. ~1.0 (57 deg) for organic shapes.
    pub corner_angle_threshold: f64,
    /// Window size for angle computation (number of neighboring points).
    /// Larger = smoother angle estimates, fewer false corners.
    pub corner_window: usize,

    // -- Curve fitting --
    /// Accuracy for kurbo fit_to_bezpath_opt (Frechet distance tolerance
    /// in font units). Smaller = more points, closer fit.
    pub fit_accuracy: f64,
    /// RDP simplification epsilon (in pixel coordinates, before scaling).
    pub rdp_epsilon: f64,

    // -- Post-processing --
    /// Grid size for coordinate snapping. 0 = no snapping.
    pub grid: i32,
    /// Whether to insert points at curve extrema.
    pub add_extrema: bool,
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
#[derive(Debug, Clone, Copy)]
pub enum ThresholdMethod {
    /// Fixed brightness threshold (0-255).
    Fixed(u8),
    /// Otsu's method (automatic).
    Otsu,
}

impl Default for TracingConfig {
    fn default() -> Self {
        Self {
            threshold: ThresholdMethod::Otsu,
            invert: false,
            min_contour_area: 100.0,
            corner_angle_threshold: 0.5,
            corner_window: 5,
            fit_accuracy: 1.0,
            rdp_epsilon: 2.0,
            grid: 0,
            add_extrema: true,
            fix_direction: true,
            chamfer_size: 0.0,
            chamfer_min_edge: 40.0,
            advance_width: None,
            lsb: 50.0,
            rsb: 50.0,
            target_height: 1000.0,
            y_offset: 0.0,
            codepoints: vec![],
        }
    }
}
