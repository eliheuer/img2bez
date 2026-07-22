// Copyright 2026 the img2bez Authors
// SPDX-License-Identifier: Apache-2.0 OR MIT

/// Source-class tuning preset. The single source of truth for profile defaults,
/// shared by the CLI `--profile` flag and the wasm bindings so both produce
/// identical output for the same image. Today a profile only sets the
/// curve-fit accuracy; other levers may follow.
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq)]
#[non_exhaustive]
pub enum Profile {
    /// Unknown-source raster (a different design at a different resolution):
    /// looser fit, fewer and cleaner points. The default.
    #[default]
    Wild,
    /// Clean, high-resolution source (e.g. a font render): tighter fit.
    Clean,
    /// Soft photographic scan of printed type (an old-style proof, a faint
    /// letterpress page): wild-like fit plus an image pre-blur that clears the
    /// edge texture which would otherwise over-segment the trace. Auto-detected
    /// from the image in the default flow, or forced with `--profile photo`.
    Photo,
}

/// An output-shape constraint applied to the finished outline. Use these when
/// the design calls for a specific point style regardless of what the source
/// suggests.
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq)]
#[non_exhaustive]
pub enum TraceMode {
    /// Corners, straight lines, and curves as detected — the normal trace.
    #[default]
    Default,
    /// Every on-curve point is a smooth point and every segment a cubic: a
    /// tangent-continuous, all-curves outline for organic forms (no corners,
    /// no straight lines). Built by re-splining through the structural points.
    Smooth,
    /// Every segment is a straight line and there are no off-curve points — a
    /// polygonal outline (the curves flattened to a tolerance).
    LineOnly,
}

impl Profile {
    /// Curve-fitting accuracy (font units) for this source class. An explicit
    /// accuracy override always takes precedence over this.
    pub fn fit_accuracy(self) -> f64 {
        match self {
            Profile::Clean => 2.0,
            Profile::Wild | Profile::Photo => 4.0,
        }
    }

    /// Whether this profile clears soft-scan edge texture with an image
    /// pre-blur before contour extraction (the `Photo` handling). The blur
    /// amount is resolution-scaled from the image, not fixed by the profile.
    pub fn applies_pre_blur(self) -> bool {
        matches!(self, Profile::Photo)
    }

    /// Parse a profile by name, never failing: unknown or empty input falls
    /// back to the default (`Wild`). Use the [`FromStr`](std::str::FromStr)
    /// impl (`name.parse()`) when a typo should be an error instead.
    pub fn from_name_lossy(name: &str) -> Self {
        name.parse().unwrap_or_default()
    }
}

impl std::str::FromStr for Profile {
    type Err = String;

    /// Parse a profile name (case-insensitive): `wild`, `clean`, or `photo`.
    fn from_str(s: &str) -> Result<Self, Self::Err> {
        match s.trim().to_ascii_lowercase().as_str() {
            "wild" => Ok(Profile::Wild),
            "clean" => Ok(Profile::Clean),
            "photo" => Ok(Profile::Photo),
            other => Err(format!(
                "unknown profile {other:?}; valid names are \
                 \"wild\", \"clean\", and \"photo\""
            )),
        }
    }
}

/// The drawing style of the source letterform: design-specific tuning layered
/// on top of the base settings. **Declared**, not auto-detected — style is
/// design intent, not a measurable property of the image. Orthogonal to
/// [`Profile`] (input quality) and [`TraceMode`] (output constraint), so they
/// combine freely.
///
/// **Reserved / experimental — currently a no-op.** Hidden from the
/// documented API until styles carry real settings: today every style traces
/// identically to `Basic`. The plumbing stays so bindings keep working.
#[doc(hidden)]
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq)]
#[allow(clippy::enum_variant_names)] // `OldStyle` is the correct type term
#[non_exhaustive]
pub enum Style {
    /// No style-specific tuning — the base settings only.
    #[default]
    Basic,
    /// Geometric grotesk sans: monolinear, with chamfered (beveled) corners.
    Grotesk,
    /// Humanist old-style: organic curves, moderate stroke contrast, serifs.
    OldStyle,
    /// Geometric sans: pure circles and straight lines, sharp joins.
    Geometric,
    /// Brush script: flowing, smooth strokes.
    Brush,
    /// Broad/pointed-nib calligraphy: high, angled stroke contrast.
    Nib,
    /// Qalam: Arabic reed-pen calligraphy.
    Qalam,
}

impl Style {
    /// Parse a style by name, never failing: unknown or empty input falls
    /// back to `Basic`. Use the [`FromStr`](std::str::FromStr) impl
    /// (`name.parse()`) when a typo should be an error instead.
    pub fn from_name_lossy(name: &str) -> Self {
        name.parse().unwrap_or_default()
    }

    /// Canonical lowercase name (for reports, logs, and round-tripping).
    pub fn name(self) -> &'static str {
        match self {
            Style::Basic => "basic",
            Style::Grotesk => "grotesk",
            Style::OldStyle => "old-style",
            Style::Geometric => "geometric",
            Style::Brush => "brush",
            Style::Nib => "nib",
            Style::Qalam => "qalam",
        }
    }

    /// Layer this style's tuning onto `opts`, on top of the base/profile.
    /// Styles without settings yet trace identically to `Basic`.
    pub fn apply(self, _opts: &mut TraceOptions) {
        match self {
            Style::Basic => {}
            // Named slots: no tuned settings yet.
            Style::Grotesk
            | Style::OldStyle
            | Style::Geometric
            | Style::Brush
            | Style::Nib
            | Style::Qalam => {}
        }
    }
}

impl std::str::FromStr for Style {
    type Err = String;

    /// Parse a style name (case-insensitive): `basic`, `grotesk`,
    /// `old-style` (or `oldstyle`), `geometric`, `brush`, `nib`, or `qalam`.
    fn from_str(s: &str) -> Result<Self, Self::Err> {
        match s.trim().to_ascii_lowercase().as_str() {
            "basic" => Ok(Style::Basic),
            "grotesk" => Ok(Style::Grotesk),
            "old-style" | "oldstyle" => Ok(Style::OldStyle),
            "geometric" => Ok(Style::Geometric),
            "brush" => Ok(Style::Brush),
            "nib" => Ok(Style::Nib),
            "qalam" => Ok(Style::Qalam),
            other => Err(format!(
                "unknown style {other:?}; valid names are \"basic\", \
                 \"grotesk\", \"old-style\", \"geometric\", \"brush\", \
                 \"nib\", and \"qalam\""
            )),
        }
    }
}

/// Options that control the *shape* of the traced outline — everything from
/// the bitmap threshold through curve fitting and cleanup. Holds nothing
/// about where the glyph sits in a font: baseline, sidebearings, and advance
/// live in [`FontMetrics`], applied separately by [`crate::place`] — so an
/// editor can trace once and place the result into its own coordinate system.
///
/// ```
/// use img2bez::TraceOptions;
///
/// let mut opts = TraceOptions::default();
/// opts.grid = 2;
/// ```
#[derive(Debug, Clone, PartialEq)]
#[non_exhaustive]
pub struct TraceOptions {
    /// Source-class preset this options set came from (via `for_profile`);
    /// drives profile-specific handling like the `Photo` pre-blur.
    /// Default `Wild`.
    pub profile: Profile,

    // -- Bitmap stage --
    /// Threshold method for converting to binary.
    pub threshold: ThresholdMethod,
    /// If true, invert the image (swap foreground/background) before tracing.
    pub invert: bool,

    // -- Contour stage --
    /// Minimum contour area in pixels (filter speckles).
    pub min_contour_area: f64,

    /// Right-to-left start points: contours start at their bottom-RIGHT
    /// on-curve point instead of bottom-left, per RTL writing direction
    /// (Arabic, Hebrew). [`crate::trace_glyph`] sets this automatically
    /// from the glyph's codepoints; set it directly when tracing without
    /// codepoint context.
    pub rtl_start: bool,

    // -- Curve fitting --
    /// Curve-fit tolerance in font units: the maximum deviation the
    /// section fitters accept before splitting. Smaller = more points,
    /// closer fit.
    pub fit_accuracy: f64,

    // -- Normalization --
    /// The font-unit height the source image maps to — its full
    /// ascender-to-descender extent. **Not** the units-per-em (a font metric,
    /// in [`FontMetrics`]): the two differ when ascenders/descenders reach
    /// past the em box (default 1088 = 832 − (−256) on a 1024 em). The
    /// cleanup heuristics are tuned for the default; for a different scale,
    /// trace at the default and rescale via [`crate::Outline::scaled`].
    pub em_height: f64,

    // -- Post-processing --
    /// Grid size for coordinate snapping. 0 = no snapping.
    pub grid: i32,
    /// Coarse structure grid for the dyadic self-labeling snap (0 = off, the
    /// default). When set (e.g. 8), on-curve points snap to this grid where
    /// they land close to it (structure) and fall back to `grid` only where
    /// snapping to the structure grid would move them too far (an optical
    /// correction). The result is self-labeling: a point on `grid` but off
    /// `structure_grid` can only be a deliberate correction.
    ///
    /// TODO: first-pass heuristic — a fixed distance tolerance of one `grid`
    /// step. Review and tune against real traced projects before trusting the
    /// structure/correction split. A human places corrections by design
    /// intent; this places them by distance to the structure grid, which is a
    /// crude proxy and will misclassify some points.
    pub structure_grid: i32,
    /// Whether to correct contour direction (CCW outer, CW counter).
    pub fix_direction: bool,
    /// Chamfer size in font units. 0 = no chamfers.
    pub chamfer_size: f64,
    /// Minimum edge length to chamfer (edges shorter than this are skipped).
    pub chamfer_min_edge: f64,

    // -- Input-adaptive levers --
    /// Gaussian blur (sigma, px) applied to the *image* before contour
    /// extraction (0 = off). Unlike `smoothing` (which smooths the extracted
    /// polyline), this smooths the raster itself, so the iso-contour is drawn
    /// through a clean edge — the lever for rough, textured, or low-contrast
    /// sources like a photo of printed type, where the boundary itself is
    /// noisy.
    pub pre_blur: f64,
    /// When `pre_blur` is 0 (no explicit blur), auto-detect soft, low-contrast
    /// sources from the image statistics and apply a resolution-scaled pre-blur
    /// (default true). Crisp renders and bilevel art sit far outside the soft
    /// regime and are left untouched. An explicit `pre_blur > 0` always wins.
    pub auto_pre_blur: bool,
    /// Multiplier on the pre-fit Gaussian smoothing applied to the boundary
    /// before feature detection (1.0 = default). Raise it for noisy, blurry, or
    /// upscaled sources to stop the tracer chasing wobble into spurious points;
    /// lower it to track a clean, crisp source more tightly.
    pub smoothing: f64,
    /// Corner-detection turn-angle threshold in degrees (default 12). A vertex
    /// is a corner when the boundary turns more sharply than this. Raise it so
    /// only hard corners survive (fewer points on soft/degraded sources); lower
    /// it to keep gentler corners on clean high-contrast designs.
    pub corner_threshold_deg: f64,
    /// Output-shape constraint applied to the finished outline (default: none).
    pub mode: TraceMode,

    // -- Tracing mode --
    /// If true (default), refine the traced output against the source
    /// raster: handle lengths are re-optimized on a coverage loss and
    /// fitter-created curve subdivisions are merged when one cubic
    /// reproduces the image nearly as well. Structure constraints (points
    /// at extrema, H/V handles) are preserved.
    pub refine_raster: bool,

    // -- Diagnostics --
    /// Print progress to stderr. Libraries default to `false`;
    /// set `true` in CLI binaries to restore step-by-step output.
    pub verbose: bool,

    /// Set by preprocessing when the source was smooth-upscaled from low
    /// resolution: corners in the upscaled image are known to be smeared
    /// over many samples, so corner detection may accept cusp-tight turns
    /// on window evidence alone. Never set on native-resolution sources,
    /// where a distributed tight turn is a designed smooth curve.
    pub(crate) corner_smear: bool,
    /// Set by preprocessing when the source classified as a soft
    /// photographic scan (the photo pre-blur was applied): tight
    /// small-radius turns there are ink-wear artifacts to keep smooth,
    /// not designed vertices.
    pub(crate) soft_source: bool,
}

/// Where a traced outline sits in a font: the units-per-em it reports, the
/// vertical metrics that position the baseline, and the sidebearings/advance
/// that position it horizontally. Consumed by [`crate::place`].
#[derive(Debug, Clone, Copy, PartialEq)]
#[non_exhaustive]
pub struct FontMetrics {
    /// The font's units-per-em (the em-square design grid). Independent of
    /// the ascender/descender, which may reach past it (default: 1024 em with
    /// 1088 of vertical reach). Written to the UFO's `unitsPerEm` and the
    /// glyph JSON.
    pub units_per_em: f64,
    /// Ascender in font units (top of the tallest letters, above the baseline).
    pub ascender: f64,
    /// Descender in font units (typically negative). The traced outline is
    /// shifted so its em-box bottom lands here, putting the baseline at y=0.
    pub descender: f64,
    /// Left sidebearing: the leftmost ink is moved to this x.
    pub lsb: f64,
    /// Right sidebearing: added past the rightmost ink to set the advance,
    /// unless `advance_width` is given.
    pub rsb: f64,
    /// Explicit advance width. `None` computes it from the ink bounds + `rsb`.
    pub advance_width: Option<f64>,
}

/// Threshold method for converting a grayscale image to binary.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum ThresholdMethod {
    /// Fixed brightness threshold (0-255).
    Fixed(u8),
    /// Otsu's method (automatic).
    Otsu,
}

impl Default for TraceOptions {
    fn default() -> Self {
        Self {
            rtl_start: false,
            profile: Profile::Wild,
            threshold: ThresholdMethod::Otsu,
            invert: false,
            min_contour_area: 100.0, // ~10×10 px: filters dust/noise
            fit_accuracy: 2.0,
            em_height: 1088.0,
            grid: 2,
            structure_grid: 0,
            fix_direction: true,
            chamfer_size: 0.0,
            chamfer_min_edge: 40.0,
            pre_blur: 0.0,
            auto_pre_blur: true,
            smoothing: 1.0,
            corner_threshold_deg: 12.0,
            mode: TraceMode::Default,

            refine_raster: true,
            verbose: false,
            corner_smear: false,
            soft_source: false,
        }
    }
}

impl Default for FontMetrics {
    /// A 1024-unit em with ascender 832, descender −256, and 64-unit
    /// sidebearings — the metrics the tracer is tuned for.
    fn default() -> Self {
        Self {
            units_per_em: 1024.0,
            ascender: 832.0,
            descender: -256.0,
            lsb: 64.0,
            rsb: 64.0,
            advance_width: None,
        }
    }
}

impl TraceOptions {
    /// Start from the defaults for a source-class [`Profile`] (sets the fit
    /// accuracy). Chain the `with_*` methods to override individual knobs.
    pub fn for_profile(profile: Profile) -> Self {
        Self {
            profile,
            fit_accuracy: profile.fit_accuracy(),
            ..Self::default()
        }
    }

    /// Set the curve-fit accuracy (font units; smaller = more points).
    pub fn with_accuracy(mut self, accuracy: f64) -> Self {
        self.fit_accuracy = accuracy;
        self
    }

    /// Set the coordinate snapping grid (0 = no snapping).
    pub fn with_grid(mut self, grid: i32) -> Self {
        self.grid = grid;
        self
    }

    /// Set the coarse structure grid for the dyadic self-labeling snap
    /// (0 = off). See [`TraceOptions::structure_grid`]. Typically 8, paired
    /// with a `grid` of 2.
    pub fn with_structure_grid(mut self, structure_grid: i32) -> Self {
        self.structure_grid = structure_grid;
        self
    }

    /// Set the automatic line-corner chamfer size in font units.
    ///
    /// Keep this at `0.0` for curved glyphs unless you have reviewed the
    /// result; chamfering is intended for line-based sharp forms.
    pub fn with_chamfer(mut self, size: f64) -> Self {
        self.chamfer_size = size;
        self
    }

    /// Set the minimum edge length eligible for automatic chamfering.
    pub fn with_chamfer_min_edge(mut self, min_edge: f64) -> Self {
        self.chamfer_min_edge = min_edge;
        self
    }

    /// Set the font-unit height the source image maps to (see the field docs).
    pub fn with_em_height(mut self, em_height: f64) -> Self {
        self.em_height = em_height;
        self
    }

    /// Set the binarization threshold method.
    pub fn with_threshold(mut self, threshold: ThresholdMethod) -> Self {
        self.threshold = threshold;
        self
    }

    /// Swap foreground/background before tracing.
    pub fn with_invert(mut self, invert: bool) -> Self {
        self.invert = invert;
        self
    }

    /// Enable or disable raster-loss refinement.
    pub fn with_refine(mut self, refine: bool) -> Self {
        self.refine_raster = refine;
        self
    }
}

impl FontMetrics {
    /// Build metrics from a vertical em (ascender/descender), with default
    /// sidebearings and auto-computed advance.
    pub fn new(ascender: f64, descender: f64) -> Self {
        Self {
            ascender,
            descender,
            ..Self::default()
        }
    }

    /// Build metrics from a target image height and vertical offset.
    ///
    /// This mirrors the CLI flags: `target_height` is `ascender - descender`,
    /// and `y_offset` is usually the descender.
    pub fn from_target_height(target_height: f64, y_offset: f64) -> Self {
        Self::new(target_height + y_offset, y_offset)
    }

    /// Set the units-per-em (the em-square design grid).
    pub fn with_units_per_em(mut self, units_per_em: f64) -> Self {
        self.units_per_em = units_per_em;
        self
    }

    /// Set both sidebearings.
    pub fn with_sidebearings(mut self, lsb: f64, rsb: f64) -> Self {
        self.lsb = lsb;
        self.rsb = rsb;
        self
    }

    /// Set an explicit advance width (overrides the `rsb`-based computation).
    pub fn with_advance(mut self, advance: f64) -> Self {
        self.advance_width = Some(advance);
        self
    }

    /// Return the font's units-per-em value.
    pub fn units_per_em(&self) -> f64 {
        self.units_per_em
    }
}
