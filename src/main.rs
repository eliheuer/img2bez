// Copyright 2026 the img2bez Authors
// SPDX-License-Identifier: Apache-2.0 OR MIT

//! CLI entry point for the img2bez raster-to-vector tracer.
//!
//! Reads a raster image, traces it to cubic bezier contours, and inserts
//! the resulting drawing into a UFO font source.

use clap::{Args, Parser, Subcommand, ValueEnum};
use img2bez::{FontMetrics, TraceOptions};
use std::path::PathBuf;

/// Tuning preset selected by the kind of raster source being traced.
///
/// `wild` (the default) loosens the fit for fewer, cleaner points on
/// unknown-source rasters; `clean` is the tighter fit for a high-res font
/// render you want tracked exactly — neutral on clean fonts, better on
/// unknown sources.
///
/// CLI surface for the library's [`img2bez::Profile`], kept thin (just the
/// clap `ValueEnum` derive) so the accuracy mapping lives in the library,
/// shared with the wasm bindings.
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq, ValueEnum)]
enum Profile {
    /// Unknown-source raster. Looser fit. Default.
    #[default]
    Wild,
    /// Clean, high-resolution source (e.g. a font render). Tighter fit.
    Clean,
    /// Soft photographic scan of printed type. Forces the image pre-blur that
    /// clears edge texture (auto-detected in the default flow).
    Photo,
}

impl From<Profile> for img2bez::Profile {
    fn from(p: Profile) -> Self {
        match p {
            Profile::Wild => img2bez::Profile::Wild,
            Profile::Clean => img2bez::Profile::Clean,
            Profile::Photo => img2bez::Profile::Photo,
        }
    }
}

impl Profile {
    /// Default curve-fitting accuracy (font units) for this source class;
    /// an explicit `--accuracy` overrides it. Delegates to the library.
    fn fit_accuracy(self) -> f64 {
        img2bez::Profile::from(self).fit_accuracy()
    }
}

#[derive(Parser)]
#[command(
    name = "img2bez",
    about = "Bitmap image to font-ready bezier contours"
)]
struct Cli {
    /// Input image path (PNG, JPEG, BMP). Required for single-glyph mode
    /// (not the subcommands).
    #[arg(short, long)]
    input: Option<PathBuf>,

    /// Output UFO path (will insert/replace the glyph). Required for
    /// single-glyph mode (not the subcommands).
    #[arg(short, long)]
    output: Option<PathBuf>,

    /// Subcommand. Omit for the single-glyph trace (the flags below).
    #[command(subcommand)]
    command: Option<Command>,

    /// Glyph name (single-glyph mode)
    #[arg(short, long)]
    name: Option<String>,

    /// Unicode codepoint (hex, e.g. "003F" for ?)
    #[arg(short = 'u', long)]
    unicode: Option<String>,

    /// Advance width (auto-computed if omitted)
    #[arg(short = 'w', long)]
    width: Option<f64>,

    /// Grid size for coordinate snapping (2 = even integers for a
    /// power-of-2 grid; 0 = off)
    #[arg(long, default_value = "2")]
    grid: i32,

    /// Coarse structure grid for the dyadic self-labeling snap (e.g. 8;
    /// 0 = off). On-curve points snap here where close (structure) and drop
    /// to --grid only where snapping here would distort (a correction).
    #[arg(long, default_value = "0")]
    structure_grid: i32,

    /// Chamfer size (0 = off)
    #[arg(long, default_value = "0")]
    chamfer: f64,

    /// Source-class tuning preset (wild = unknown raster, default; clean = font
    /// render; photo = soft scan of printed type, forces the de-texture blur)
    #[arg(long, value_enum, default_value_t = Profile::Wild, global = true)]
    profile: Profile,

    /// Curve fitting accuracy in font units (overrides the profile default)
    #[arg(long, global = true)]
    accuracy: Option<f64>,

    /// Image pre-blur sigma (px, 0 = off). Blurs the raster before contour
    /// extraction — the lever for rough/textured/low-contrast sources like a
    /// photo of printed type, where the edge itself is noisy.
    #[arg(long, global = true)]
    pre_blur: Option<f64>,

    /// Disable automatic pre-blur. By default, soft/low-contrast sources are
    /// detected from the image statistics and given a resolution-scaled blur;
    /// this turns that off (an explicit --pre-blur still applies).
    #[arg(long, global = true)]
    no_auto_pre_blur: bool,

    /// Pre-fit smoothing multiplier (1.0 = default). Raise for noisy, blurry,
    /// or upscaled sources; lower to track a clean source tightly.
    #[arg(long, global = true)]
    smoothing: Option<f64>,

    /// Corner turn-angle threshold in degrees (default 12). Raise so only hard
    /// corners survive on degraded sources; lower for clean high-contrast art.
    #[arg(long, global = true)]
    corner_threshold: Option<f64>,

    /// Output-shape constraint: `default`; `smooth` (every point a smooth curve
    /// point — organic, all-curves); or `line` (all straight lines, no curves).
    #[arg(long, value_enum, default_value_t = Mode::Default, global = true)]
    mode: Mode,

    /// Drawing style of the source: `basic` (default), `grotesk`,
    /// `old-style`, `geometric`, `brush`, `nib`, or `qalam`. Layers
    /// design-specific tuning on top of the base settings.
    // Hidden until styles carry real settings (every style currently traces
    // identically); the flag still parses so existing invocations keep working.
    #[arg(long, value_enum, default_value_t = Style::Basic, global = true, hide = true)]
    style: Style,

    /// Append a JSONL record of each trace (image features + settings +
    /// output stats) to PATH. Set the `IMG2BEZ_LOG` env var instead to log
    /// every run without the flag.
    #[arg(long, value_name = "PATH", global = true)]
    log: Option<PathBuf>,

    /// Target height in font units (ascender - descender; 1088 for 1024 UPM)
    #[arg(long, default_value = "1088")]
    target_height: f64,

    /// Y offset after scaling (typically descender; -256 for 1024 UPM)
    #[arg(long, default_value = "-256", allow_hyphen_values = true)]
    y_offset: f64,

    /// Invert the image before tracing
    #[arg(long)]
    invert: bool,

    /// Fixed brightness threshold (0-255). Overrides Otsu auto-detection.
    #[arg(long)]
    threshold: Option<u8>,

    /// Reference .glif file for quality evaluation
    #[arg(long)]
    reference: Option<PathBuf>,

    /// Disable raster-loss refinement (handle polish + curve merging
    /// scored against the source image)
    #[arg(long)]
    no_refine: bool,

    /// Output format. `ufo` writes/updates a UFO at --output; `glif`, `json`,
    /// and `svg` write the serialized outline to --output (use `-` for stdout).
    #[arg(long, value_enum, default_value_t = OutputFormat::Ufo)]
    format: OutputFormat,

    /// Placement source box: `canvas` (image framed to the font's vertical
    /// extent — the default) or `ink` (detect the ink and fit it to --fit-y,
    /// for generated/padded images).
    #[arg(long, value_enum, default_value_t = FitSource::Canvas)]
    fit_source: FitSource,

    /// Target vertical band in font units as `ymin:ymax` (e.g. "-256:768" for
    /// descender-to-cap). Required with `--fit-source ink`.
    #[arg(long, allow_hyphen_values = true)]
    fit_y: Option<String>,

    /// Left sidebearing in font units (with `--fit-source ink`; default 64).
    #[arg(long, allow_hyphen_values = true)]
    lsb: Option<f64>,

    /// Right sidebearing in font units (with `--fit-source ink`; default 64).
    #[arg(long, allow_hyphen_values = true)]
    rsb: Option<f64>,

    /// Copy the advance width from an existing UFO glyph, as `UFO:glyph`
    /// (e.g. "Font.ufo:germandbls"). Pins the advance; `--lsb` still positions
    /// the ink. Overrides `--width`. Requires `--fit-source ink`.
    #[arg(long, value_name = "UFO:GLYPH")]
    metrics_from: Option<String>,

    /// Copy the target vertical band from an existing UFO glyph's ink bounds,
    /// as `UFO:glyph` (e.g. "Font.ufo:b"). An alternative to `--fit-y`.
    #[arg(long, value_name = "UFO:GLYPH")]
    fit_y_from: Option<String>,

    /// Copy the sidebearings (lsb and rsb) from an existing UFO glyph, as
    /// `UFO:glyph` (e.g. "Font.ufo:B"). An alternative to `--lsb`/`--rsb`.
    #[arg(long, value_name = "UFO:GLYPH")]
    sidebearings_from: Option<String>,
}

/// Subcommands for the multi-master variable-font workflow.
#[derive(Subcommand)]
enum Command {
    /// Trace one image per master of a glyph, reconcile them into
    /// interpolation-compatible outlines, and write each into its master UFO
    /// (or return JSON). Masters and metrics come from the `.designspace`.
    Masters(MastersArgs),
    /// Scaffold a new variable font: a `.designspace` plus an empty UFO
    /// per master, with the given metrics. Run once, then add glyphs with
    /// `masters`.
    NewFont(NewFontArgs),
    /// Print no-reference image statistics (resolution, edge sharpness, noise,
    /// bilevel-ness) as JSON — the features for input-adaptive settings.
    Stats(StatsArgs),
}

/// `img2bez stats` arguments.
#[derive(Args)]
struct StatsArgs {
    /// Image to measure (PNG/JPEG/BMP).
    image: PathBuf,
}

/// `img2bez masters` arguments.
#[derive(Args)]
struct MastersArgs {
    /// The font's `.designspace` (defines the masters and their metrics).
    designspace: PathBuf,

    /// Glyph name to write.
    #[arg(long)]
    glyph: String,

    /// Unicode codepoint (hex), e.g. "00DF".
    #[arg(long)]
    unicode: Option<String>,

    /// A directory of images named by master — `<stem>.png` where `<stem>` is
    /// the master's UFO filename (e.g. `MyFont-Bold.png`) or its style name
    /// (`Bold.png`). Scales to any number of masters with one flag.
    #[arg(long, value_name = "DIR")]
    images: Option<PathBuf>,

    /// One image per master, as `MasterName=path` (repeatable). Overrides
    /// `--images` for that master; use for a few masters or odd paths.
    #[arg(long, value_name = "NAME=IMAGE")]
    image: Vec<String>,

    /// Vertical fit band as `zone:zone`, where a zone is `descender`,
    /// `baseline`, `xheight`, `cap`, `ascender`, or a raw number. Resolved from
    /// the font's metrics. Example: `descender:cap`.
    #[arg(long, default_value = "descender:cap", allow_hyphen_values = true)]
    fit: String,

    /// Left sidebearing in font units (default 64).
    #[arg(long, allow_hyphen_values = true)]
    lsb: Option<f64>,

    /// Right sidebearing in font units (default 64).
    #[arg(long, allow_hyphen_values = true)]
    rsb: Option<f64>,

    /// `ufo` (default) writes each master UFO; `json` returns the reconciled
    /// outlines + report on stdout and writes nothing.
    #[arg(long, value_enum, default_value_t = MastersFormat::Ufo)]
    format: MastersFormat,

    /// Also write the structured report (resolved fit, advances, sidebearings,
    /// bounds, point counts, inserted points, low-confidence, warnings) as JSON
    /// to this path. Works in write mode too, for the agent loop.
    #[arg(long, value_name = "PATH")]
    report: Option<PathBuf>,

    /// Exit non-zero if reconcile had to guess any point correspondences
    /// (low confidence), so an automated run regenerates instead of shipping.
    #[arg(long)]
    fail_on_low_confidence: bool,

    /// When the glyph already exists in a master, keep its advance width and
    /// left sidebearing instead of computing them (for replacing glyphs).
    #[arg(long)]
    preserve_existing_metrics: bool,
}

/// Output mode for `img2bez masters`.
#[derive(Copy, Clone, Debug, PartialEq, Eq, ValueEnum)]
enum MastersFormat {
    /// Write each master UFO.
    Ufo,
    /// Return the reconciled outlines + report as JSON; write nothing.
    Json,
}

/// `img2bez new-font` arguments.
#[derive(Args)]
struct NewFontArgs {
    /// Path to create the `.designspace` at (its directory is created).
    designspace: PathBuf,

    /// Family name.
    #[arg(long)]
    family: String,

    /// A master as `Name:axisTag=pos` (repeatable), e.g. `Regular:wght=400`.
    /// The first axis tag seen defines the (single) axis.
    #[arg(long, value_name = "NAME:AXIS=POS")]
    master: Vec<String>,

    /// Units per em.
    #[arg(long, default_value = "1000")]
    upm: f64,

    /// Ascender (font units).
    #[arg(long, default_value = "800", allow_hyphen_values = true)]
    ascender: f64,

    /// Descender (font units, typically negative).
    #[arg(long, default_value = "-200", allow_hyphen_values = true)]
    descender: f64,

    /// Cap height (font units).
    #[arg(long = "cap-height", default_value = "700")]
    cap_height: f64,

    /// X-height (font units).
    #[arg(long = "x-height", default_value = "500")]
    x_height: f64,

    /// Overwrite if the designspace already exists.
    #[arg(long)]
    force: bool,
}

/// Split a `UFO:glyph` reference spec on its last colon (UFO paths don't
/// contain colons on the platforms we target, and glyph names never do).
fn parse_ref_spec(spec: &str) -> Result<(PathBuf, String), String> {
    spec.rsplit_once(':')
        .filter(|(ufo, glyph)| !ufo.is_empty() && !glyph.is_empty())
        .map(|(ufo, glyph)| (PathBuf::from(ufo), glyph.to_string()))
        .ok_or_else(|| {
            format!(
                "expected \"UFO:glyph\" (e.g. \"Font.ufo:b\"), got {spec:?}"
            )
        })
}

/// Output-shape constraint (maps to [`img2bez::TraceMode`]).
#[derive(Copy, Clone, Debug, PartialEq, Eq, ValueEnum)]
enum Mode {
    /// Corners, lines, and curves as detected.
    Default,
    /// Every on-curve point is a smooth curve point (organic, all-curves).
    Smooth,
    /// Every segment is a straight line (no off-curve points).
    Line,
}

impl From<Mode> for img2bez::TraceMode {
    fn from(m: Mode) -> Self {
        match m {
            Mode::Default => img2bez::TraceMode::Default,
            Mode::Smooth => img2bez::TraceMode::Smooth,
            Mode::Line => img2bez::TraceMode::LineOnly,
        }
    }
}

/// Drawing style of the source letterform (maps to [`img2bez::Style`]). Layers
/// design-specific tuning on top of the base; declared, not auto-detected.
#[derive(Copy, Clone, Debug, PartialEq, Eq, ValueEnum)]
#[allow(clippy::enum_variant_names)] // `OldStyle` is the correct type term
enum Style {
    /// No style tuning — base settings only.
    Basic,
    /// Geometric grotesk (chamfered corners).
    Grotesk,
    /// Humanist old-style serif.
    OldStyle,
    /// Geometric sans (circles and lines).
    Geometric,
    /// Brush script.
    Brush,
    /// Broad/pointed-nib calligraphy.
    Nib,
    /// Qalam (Arabic reed pen).
    Qalam,
}

impl From<Style> for img2bez::Style {
    fn from(s: Style) -> Self {
        match s {
            Style::Basic => img2bez::Style::Basic,
            Style::Grotesk => img2bez::Style::Grotesk,
            Style::OldStyle => img2bez::Style::OldStyle,
            Style::Geometric => img2bez::Style::Geometric,
            Style::Brush => img2bez::Style::Brush,
            Style::Nib => img2bez::Style::Nib,
            Style::Qalam => img2bez::Style::Qalam,
        }
    }
}

/// Placement source box.
#[derive(Copy, Clone, Debug, PartialEq, Eq, ValueEnum)]
enum FitSource {
    /// The whole image canvas (image framed to the font's vertical extent).
    Canvas,
    /// Detect the ink and fit it to the target band.
    Ink,
}

/// What the CLI emits to `--output`.
#[derive(Copy, Clone, Debug, PartialEq, Eq, ValueEnum)]
enum OutputFormat {
    /// A UFO font source (the glyph is inserted/replaced).
    Ufo,
    /// UFO GLIF XML for the single glyph.
    Glif,
    /// The canonical outline as JSON.
    Json,
    /// An SVG `<path>` `d` string.
    Svg,
}

/// Write `text` to `path`, or to stdout when `path` is `-`.
fn write_text(path: &std::path::Path, text: &str) -> std::io::Result<()> {
    use std::io::Write;
    if path.as_os_str() == "-" {
        let mut out = std::io::stdout().lock();
        out.write_all(text.as_bytes())?;
        out.write_all(b"\n")?;
    } else {
        std::fs::write(path, text)?;
    }
    Ok(())
}

/// One master's traced + placed result, ready to write or serialize.
struct PlacedMaster {
    name: String,
    ufo_path: PathBuf,
    outline: img2bez::Outline,
    advance: f64,
    metrics: FontMetrics,
    fit: (f64, f64),
    lsb: f64,
    rsb: f64,
    bounds: kurbo::Rect,
    out_of_target: bool,
    /// Effective trace profile (forced via `--profile`, or auto-detected),
    /// plus the input stats behind the detection.
    profile: img2bez::Profile,
    sharpness: f64,
    bilevelness: f64,
}

/// Per-master entry in the structured report.
#[derive(serde::Serialize)]
#[serde(rename_all = "camelCase")]
struct MasterReport {
    name: String,
    ufo: String,
    written: bool,
    /// Resolved vertical band `[y_min, y_max]` in font units.
    fit: [f64; 2],
    advance: f64,
    lsb: f64,
    rsb: f64,
    /// Final glyph bounds `[x_min, y_min, x_max, y_max]`.
    bounds: [f64; 4],
    points: usize,
    inserted_points: usize,
    out_of_target: bool,
    /// Trace profile used (`wild`/`clean`/`photo`; forced or auto-detected),
    /// and the input stats behind it.
    profile: String,
    sharpness: f64,
    bilevelness: f64,
}

/// The machine-readable result of a `masters` run (written by `--report`).
#[derive(serde::Serialize)]
#[serde(rename_all = "camelCase")]
struct MastersReport {
    glyph: String,
    unicodes: Vec<String>,
    fit_spec: String,
    compatible: bool,
    inserted_points: usize,
    low_confidence: bool,
    masters: Vec<MasterReport>,
    warnings: Vec<String>,
}

/// Build [`TraceOptions`] from the shared CLI tuning flags; used by both the
/// single-glyph command and `masters`. Callers set `em_height` and `verbose`.
fn opts_from_cli(cli: &Cli) -> TraceOptions {
    let mut opts = TraceOptions::default();
    opts.profile = cli.profile.into();
    opts.fit_accuracy = cli.accuracy.unwrap_or(cli.profile.fit_accuracy());
    opts.grid = cli.grid;
    opts.structure_grid = cli.structure_grid;
    opts.chamfer_size = cli.chamfer;
    opts.invert = cli.invert;
    opts.threshold = match cli.threshold {
        Some(t) => img2bez::ThresholdMethod::Fixed(t),
        None => img2bez::ThresholdMethod::Otsu,
    };
    opts.refine_raster = !cli.no_refine;
    opts.auto_pre_blur = !cli.no_auto_pre_blur;
    // RTL scripts start contours bottom-right (derived from --unicode).
    opts.rtl_start = cli
        .unicode
        .as_deref()
        .and_then(|h| {
            u32::from_str_radix(
                h.trim_start_matches("U+").trim_start_matches("u+").trim(),
                16,
            )
            .ok()
        })
        .and_then(char::from_u32)
        .is_some_and(img2bez::is_rtl_codepoint);
    if let Some(b) = cli.pre_blur {
        opts.pre_blur = b;
    }
    if let Some(s) = cli.smoothing {
        opts.smoothing = s;
    }
    if let Some(c) = cli.corner_threshold {
        opts.corner_threshold_deg = c;
    }
    opts.mode = cli.mode.into();
    // Layer the drawing-style tuning on top of the base/profile.
    img2bez::Style::from(cli.style).apply(&mut opts);
    opts
}

/// Lowercase name of a [`img2bez::Profile`] for the report and logs.
fn profile_name(p: img2bez::Profile) -> &'static str {
    match p {
        img2bez::Profile::Wild => "wild",
        img2bez::Profile::Clean => "clean",
        img2bez::Profile::Photo => "photo",
        _ => "unknown",
    }
}

/// Lowercase name of a [`img2bez::TraceMode`] for the trace log.
fn mode_name(m: img2bez::TraceMode) -> &'static str {
    match m {
        img2bez::TraceMode::Default => "default",
        img2bez::TraceMode::Smooth => "smooth",
        img2bez::TraceMode::LineOnly => "line",
        _ => "unknown",
    }
}

/// The data-collection log path: `--log` if given, else the `IMG2BEZ_LOG`
/// environment variable. `None` disables logging (the default).
fn effective_log_path(cli: &Cli) -> Option<PathBuf> {
    cli.log
        .clone()
        .or_else(|| std::env::var_os("IMG2BEZ_LOG").map(PathBuf::from))
}

/// Stable 64-bit FNV-1a hash of the image bytes (hex): identifies a source
/// image across runs without pulling in a crypto dependency.
fn fnv1a_hex(bytes: &[u8]) -> String {
    let mut h: u64 = 0xcbf2_9ce4_8422_2325;
    for &b in bytes {
        h ^= b as u64;
        h = h.wrapping_mul(0x0000_0100_0000_01b3);
    }
    format!("{h:016x}")
}

/// The settings actually applied to a trace, for the data-collection log.
#[derive(serde::Serialize)]
#[serde(rename_all = "camelCase")]
struct TraceLogSettings<'a> {
    /// Forced/declared profile (`--profile`).
    profile: &'a str,
    /// Class the image statistics detected (`ImageStats::classify`).
    auto_profile: &'a str,
    /// Profile whose pre-blur actually applied (forced, else auto-detected).
    effective_profile: &'a str,
    auto_pre_blur: bool,
    accuracy: f64,
    pre_blur: f64,
    smoothing: f64,
    corner_threshold: f64,
    mode: &'a str,
    style: &'a str,
    grid: i32,
    refine: bool,
}

/// Output-shape summary of a trace, for the data-collection log.
#[derive(serde::Serialize)]
#[serde(rename_all = "camelCase")]
struct TraceLogOutput {
    contours: usize,
    on_curve: usize,
    off_curve: usize,
    /// Reference-free judge's verdict (repro IoU + structural regularizers).
    judge: img2bez::judge::Judgement,
}

/// One JSONL line in the data-collection log: an image's no-reference
/// features, the settings used, and the resulting point counts.
#[derive(serde::Serialize)]
#[serde(rename_all = "camelCase")]
struct TraceLogRecord<'a> {
    /// Record schema version (bump when fields change).
    schema: u32,
    /// Unix seconds.
    ts: u64,
    /// `trace` (single glyph) or `masters`.
    command: &'a str,
    glyph: &'a str,
    #[serde(skip_serializing_if = "Option::is_none")]
    unicode: Option<&'a str>,
    image: &'a str,
    image_hash: String,
    image_bytes: usize,
    features: &'a img2bez::ImageStats,
    settings: TraceLogSettings<'a>,
    output: TraceLogOutput,
    /// Reserved for a future quality/acceptance label. Null for now.
    #[serde(skip_serializing_if = "Option::is_none")]
    label: Option<&'a str>,
}

/// Append `record` as one JSON line to `path`, creating parent dirs and the
/// file as needed.
fn append_jsonl<T: serde::Serialize>(
    path: &std::path::Path,
    record: &T,
) -> std::io::Result<()> {
    use std::io::Write;
    let line = serde_json::to_string(record).map_err(std::io::Error::other)?;
    if let Some(dir) = path.parent().filter(|d| !d.as_os_str().is_empty()) {
        std::fs::create_dir_all(dir)?;
    }
    let mut f = std::fs::OpenOptions::new()
        .create(true)
        .append(true)
        .open(path)?;
    writeln!(f, "{line}")
}

/// Append one trace to the data-collection log (best-effort: a logging
/// failure warns to stderr but never fails the trace).
#[allow(clippy::too_many_arguments)]
fn log_trace(
    log_path: &std::path::Path,
    command: &str,
    glyph: &str,
    unicode: Option<&str>,
    image_path: &str,
    bytes: &[u8],
    stats: &img2bez::ImageStats,
    opts: &TraceOptions,
    style_name: &str,
    outline: &img2bez::Outline,
) {
    use std::time::{SystemTime, UNIX_EPOCH};
    // Effective profile: forced pre-blur profile wins, else the auto-detected
    // class when auto-pre-blur is on, else the forced profile.
    let detected = stats.classify();
    let effective = if opts.profile.applies_pre_blur() {
        opts.profile
    } else if opts.auto_pre_blur && detected.applies_pre_blur() {
        detected
    } else {
        opts.profile
    };
    let (mut on_curve, mut off_curve) = (0usize, 0usize);
    for c in &outline.contours {
        for p in &c.points {
            if p.kind == img2bez::PointKind::OffCurve {
                off_curve += 1;
            } else {
                on_curve += 1;
            }
        }
    }
    let record = TraceLogRecord {
        schema: 1,
        ts: SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .map(|d| d.as_secs())
            .unwrap_or(0),
        command,
        glyph,
        unicode,
        image: image_path,
        image_hash: fnv1a_hex(bytes),
        image_bytes: bytes.len(),
        features: stats,
        settings: TraceLogSettings {
            profile: profile_name(opts.profile),
            auto_profile: profile_name(detected),
            effective_profile: profile_name(effective),
            auto_pre_blur: opts.auto_pre_blur,
            accuracy: opts.fit_accuracy,
            pre_blur: opts.pre_blur,
            smoothing: opts.smoothing,
            corner_threshold: opts.corner_threshold_deg,
            mode: mode_name(opts.mode),
            style: style_name,
            grid: opts.grid,
            refine: opts.refine_raster,
        },
        output: TraceLogOutput {
            contours: outline.contours.len(),
            on_curve,
            off_curve,
            judge: img2bez::judge::judge_source(outline, bytes, opts)
                .unwrap_or_else(|_| img2bez::judge::structure(outline)),
        },
        label: None,
    };
    if let Err(e) = append_jsonl(log_path, &record) {
        eprintln!("  warning     trace log ({}): {e}", log_path.display());
    }
}

/// `masters` subcommand: trace one image per master, reconcile into an
/// interpolation-compatible set, then write each master UFO or return JSON.
/// Masters + metrics come from the designspace.
fn run_masters(
    cli: &Cli,
    args: &MastersArgs,
) -> Result<(), Box<dyn std::error::Error>> {
    let mut opts = opts_from_cli(cli);
    opts.verbose = false;

    let masters = img2bez::masters::read_masters(&args.designspace)?;
    if masters.len() < 2 {
        return Err(format!(
            "{} defines {} master(s); need at least 2",
            args.designspace.display(),
            masters.len()
        )
        .into());
    }
    let image_map = parse_image_map(&args.image)?;
    let codepoints = parse_unicode(args.unicode.as_deref())?;
    let unicodes: Vec<String> = codepoints
        .iter()
        .map(|c| format!("{:04X}", *c as u32))
        .collect();
    let writing = args.format == MastersFormat::Ufo;

    if writing {
        let unicode_str = args
            .unicode
            .as_deref()
            .map(|u| format!(" (U+{})", u.to_uppercase()))
            .unwrap_or_default();
        eprintln!();
        eprintln!(
            "  img2bez masters \u{00b7} {}{}  ({} masters)",
            args.glyph,
            unicode_str,
            masters.len()
        );
    }

    let default_lsb = args.lsb.unwrap_or(64.0);
    let default_rsb = args.rsb.unwrap_or(64.0);

    // Resolve each master's image, band, and placement, then trace the set
    // jointly (see img2bez::trace_place_masters) and ink-fit each to its band.
    struct MasterPrep {
        img: PathBuf,
        metrics: FontMetrics,
        fit: (f64, f64),
        bytes: Vec<u8>,
        stats: img2bez::ImageStats,
        effective_profile: img2bez::Profile,
        placement: img2bez::PlacementOptions,
    }
    let mut preps: Vec<MasterPrep> = Vec::new();
    for m in &masters {
        let img = resolve_image(m, &image_map, args.images.as_deref())?;
        let zones = img2bez::masters::read_zones(&m.ufo_path)?;
        let (ymin, ymax) = zones.resolve_fit(&args.fit)?;

        // Preserve an existing glyph's advance + lsb when asked and present.
        let sidebearings = match args
            .preserve_existing_metrics
            .then(|| img2bez::read_reference(&m.ufo_path, &args.glyph))
        {
            Some(Ok(rm)) => {
                let lsb = rm.sidebearings().map_or(default_lsb, |(l, _)| l);
                img2bez::Sidebearings::FixedAdvance {
                    width: rm.advance_width,
                    lsb,
                }
            }
            _ => img2bez::Sidebearings::Explicit {
                lsb: default_lsb,
                rsb: default_rsb,
            },
        };

        let mut placement = img2bez::PlacementOptions::ink_fit(
            img2bez::TargetBand::Custom {
                y_min: ymin,
                y_max: ymax,
            },
            sidebearings,
        );
        placement.grid = cli.grid;
        let bytes = std::fs::read(&img)?;
        // Record the input's character and the profile actually used, so the
        // report shows how each master image was treated.
        let stats = img2bez::measure_image(&bytes)?;
        let detected = stats.classify();
        let effective_profile = if opts.profile.applies_pre_blur() {
            opts.profile
        } else if opts.auto_pre_blur && detected.applies_pre_blur() {
            detected
        } else {
            opts.profile
        };
        preps.push(MasterPrep {
            img,
            metrics: zones.metrics(),
            fit: (ymin, ymax),
            bytes,
            stats,
            effective_profile,
            placement,
        });
    }

    let inputs: Vec<(&[u8], &img2bez::PlacementOptions)> = preps
        .iter()
        .map(|p| (p.bytes.as_slice(), &p.placement))
        .collect();
    let (placed_set, creport) = img2bez::trace_place_masters(&inputs, &opts)?;
    if !creport.compatible {
        return Err(format!(
            "masters are not compatible: {}",
            creport.note.as_deref().unwrap_or("unknown reason")
        )
        .into());
    }
    // Joint tracing without lossy resolution = compatible by construction;
    // low_confidence means the alignment had to guess somewhere.
    let already = !creport.low_confidence && creport.inserted_points == 0;

    let mut placed: Vec<PlacedMaster> = Vec::new();
    for (m, (prep, (pl, prep_rep))) in
        masters.iter().zip(preps.iter().zip(&placed_set))
    {
        if let Some(log_path) = effective_log_path(cli) {
            log_trace(
                &log_path,
                "masters",
                &args.glyph,
                args.unicode.as_deref(),
                &prep.img.display().to_string(),
                &prep.bytes,
                &prep.stats,
                &opts,
                img2bez::Style::from(cli.style).name(),
                &pl.outline,
            );
        }
        placed.push(PlacedMaster {
            name: m.name.clone(),
            ufo_path: m.ufo_path.clone(),
            outline: pl.outline.clone(),
            advance: pl.advance_width,
            metrics: prep.metrics,
            fit: prep.fit,
            lsb: prep_rep.lsb,
            rsb: prep_rep.rsb,
            bounds: prep_rep.final_bounds,
            out_of_target: prep_rep.out_of_target,
            profile: prep.effective_profile,
            sharpness: prep.stats.sharpness,
            bilevelness: prep.stats.bilevelness,
        });
    }

    // Per-master inserted-point counts (summed across contours).
    let inserted_for = |idx: usize| -> usize {
        creport
            .contours
            .iter()
            .map(|c| c.inserted_per_master.get(idx).copied().unwrap_or(0))
            .sum()
    };

    // Warnings.
    let mut warnings: Vec<String> = Vec::new();
    if creport.low_confidence {
        warnings.push(
            "the masters disagreed structurally and the alignment had to \
             resolve it (low confidence); review the result or regenerate \
             the outlier master"
                .into(),
        );
    }
    for p in &placed {
        if p.out_of_target {
            warnings.push(format!(
                "master {:?} outline exceeds the target band [{:.0}, {:.0}]",
                p.name, p.fit.0, p.fit.1
            ));
        }
    }
    if placed.iter().any(|p| p.fit != placed[0].fit) {
        warnings.push(format!(
            "masters resolve --fit {:?} to different bands (their metrics \
             disagree): {:?}",
            args.fit,
            placed.iter().map(|p| p.fit).collect::<Vec<_>>()
        ));
    }

    // Build the structured report.
    let masters_report = MastersReport {
        glyph: args.glyph.clone(),
        unicodes: unicodes.clone(),
        fit_spec: args.fit.clone(),
        compatible: creport.compatible,
        inserted_points: creport.inserted_points,
        low_confidence: creport.low_confidence,
        masters: placed
            .iter()
            .enumerate()
            .map(|(i, p)| MasterReport {
                name: p.name.clone(),
                ufo: p.ufo_path.display().to_string(),
                written: writing,
                fit: [p.fit.0, p.fit.1],
                advance: p.advance,
                lsb: p.lsb,
                rsb: p.rsb,
                bounds: [p.bounds.x0, p.bounds.y0, p.bounds.x1, p.bounds.y1],
                points: p.outline.contours.iter().map(|c| c.points.len()).sum(),
                inserted_points: inserted_for(i),
                out_of_target: p.out_of_target,
                profile: profile_name(p.profile).to_string(),
                sharpness: p.sharpness,
                bilevelness: p.bilevelness,
            })
            .collect(),
        warnings: warnings.clone(),
    };
    let write_report = |r: &MastersReport| -> std::io::Result<()> {
        if let Some(path) = &args.report {
            std::fs::write(path, serde_json::to_string_pretty(r).unwrap())?;
        }
        Ok(())
    };

    // JSON compute mode: emit report + outlines, write no UFOs.
    if !writing {
        #[derive(serde::Serialize)]
        struct MasterOut<'a> {
            name: &'a str,
            outline: &'a img2bez::Outline,
        }
        #[derive(serde::Serialize)]
        struct Out<'a> {
            report: &'a MastersReport,
            masters: Vec<MasterOut<'a>>,
        }
        let out = Out {
            report: &masters_report,
            masters: placed
                .iter()
                .map(|p| MasterOut {
                    name: &p.name,
                    outline: &p.outline,
                })
                .collect(),
        };
        println!("{}", serde_json::to_string_pretty(&out)?);
        write_report(&masters_report)?;
        if args.fail_on_low_confidence && creport.low_confidence {
            return Err(
                "low-confidence reconcile (--fail-on-low-confidence)".into()
            );
        }
        return Ok(());
    }

    // Write mode.
    eprintln!(
        "  Fit         {} \u{2192} [{:.0}, {:.0}]",
        args.fit, placed[0].fit.0, placed[0].fit.1
    );
    eprintln!(
        "  Compat      {}",
        if already {
            "already compatible by construction".to_string()
        } else {
            format!(
                "reconciled to a shared structure ({} points inserted)",
                creport.inserted_points
            )
        }
    );
    for w in &warnings {
        eprintln!("  warning     {w}");
    }
    // Strict mode: don't write low-confidence masters; report and fail.
    if args.fail_on_low_confidence && creport.low_confidence {
        write_report(&masters_report)?;
        return Err(format!(
            "low-confidence reconcile ({} points inserted); not written \
             (--fail-on-low-confidence) — regenerate the outlier master",
            creport.inserted_points
        )
        .into());
    }
    let pts: usize = placed[0]
        .outline
        .contours
        .iter()
        .map(|c| c.points.len())
        .sum();
    eprintln!(
        "  Result      {} contours \u{00b7} {} points/master",
        placed[0].outline.contours.len(),
        pts
    );
    eprintln!();
    for p in &placed {
        let pl = img2bez::PlacedGlyph::new(p.outline.clone(), p.advance);
        img2bez::write_into_ufo(
            &args.glyph,
            &codepoints,
            &pl,
            &p.ufo_path,
            &p.metrics,
        )?;
        eprintln!(
            "  \u{2713} {}  (adv {:.0}, lsb {:.0}, rsb {:.0})",
            p.ufo_path.display(),
            p.advance,
            p.lsb,
            p.rsb
        );
    }
    write_report(&masters_report)?;
    if let Some(path) = &args.report {
        eprintln!("  Report      {}", path.display());
    }
    eprintln!();
    Ok(())
}

/// `new-font` subcommand: scaffold a designspace + an empty UFO per master.
fn run_new_font(args: &NewFontArgs) -> Result<(), Box<dyn std::error::Error>> {
    if args.master.is_empty() {
        return Err(
            "at least one --master is required (e.g. Regular:wght=400)".into(),
        );
    }
    let masters = parse_master_specs(&args.master)?;
    let zones = img2bez::masters::VerticalZones {
        units_per_em: args.upm,
        descender: args.descender,
        baseline: 0.0,
        x_height: args.x_height,
        cap_height: args.cap_height,
        ascender: args.ascender,
    };
    img2bez::masters::create_font(
        &args.designspace,
        &args.family,
        &masters,
        &zones,
        args.force,
    )?;
    eprintln!();
    eprintln!(
        "  Created {} with {} master(s): {}",
        args.designspace.display(),
        masters.len(),
        masters
            .iter()
            .map(|(n, t, p)| format!("{n} ({t}={p})"))
            .collect::<Vec<_>>()
            .join(", ")
    );
    eprintln!();
    Ok(())
}

/// Resolve the image for one master: an explicit `--image Name=path` wins;
/// otherwise look in the `--images` directory for `<ufo-stem>.<ext>`, then
/// `<stylename>.<ext>`.
fn resolve_image(
    master: &img2bez::masters::MasterRef,
    image_map: &std::collections::HashMap<String, PathBuf>,
    images_dir: Option<&std::path::Path>,
) -> Result<PathBuf, String> {
    if let Some(p) = image_map.get(&master.name) {
        return Ok(p.clone());
    }
    if let Some(dir) = images_dir {
        let stem = master
            .ufo_path
            .file_stem()
            .and_then(|s| s.to_str())
            .unwrap_or("");
        let exts = ["png", "jpg", "jpeg", "bmp", "PNG", "JPG", "JPEG"];
        for key in [stem, master.name.as_str()] {
            if key.is_empty() {
                continue;
            }
            for ext in exts {
                let candidate = dir.join(format!("{key}.{ext}"));
                if candidate.exists() {
                    return Ok(candidate);
                }
            }
        }
        return Err(format!(
            "no image for master {:?} in {}: expected {}.png or {}.png",
            master.name,
            dir.display(),
            stem,
            master.name
        ));
    }
    Err(format!(
        "no image for master {:?}: pass --image {}=<path> or --images <dir>",
        master.name, master.name
    ))
}

/// `stats` subcommand: print the image's no-reference statistics as JSON.
fn run_stats(args: &StatsArgs) -> Result<(), Box<dyn std::error::Error>> {
    let img = image::open(&args.image)?.into_luma8();
    let stats = img2bez::measure(&img);
    println!("{}", serde_json::to_string_pretty(&stats)?);
    Ok(())
}

/// Parse `--image NAME=path` pairs into a name → path map.
fn parse_image_map(
    images: &[String],
) -> Result<std::collections::HashMap<String, PathBuf>, String> {
    let mut map = std::collections::HashMap::new();
    for spec in images {
        let (name, path) = spec.split_once('=').ok_or_else(|| {
            format!("--image must be NAME=path, got {spec:?}")
        })?;
        map.insert(name.trim().to_string(), PathBuf::from(path.trim()));
    }
    Ok(map)
}

/// Parse an optional hex codepoint (with or without a `U+` prefix) into a
/// (0 or 1) char list.
fn parse_unicode(hex: Option<&str>) -> Result<Vec<char>, String> {
    match hex {
        Some(u) => {
            let cleaned =
                u.trim().trim_start_matches("U+").trim_start_matches("u+");
            let cp = u32::from_str_radix(cleaned, 16).map_err(|_| {
                format!("invalid --unicode hex: {u:?} (expected e.g. 0041 or U+0041)")
            })?;
            Ok(char::from_u32(cp).into_iter().collect())
        }
        None => Ok(Vec::new()),
    }
}

/// Parse `Name:axisTag=pos` master specs into `(name, tag, position)`.
fn parse_master_specs(
    specs: &[String],
) -> Result<Vec<(String, String, f64)>, String> {
    specs
        .iter()
        .map(|s| {
            let (name, axis) = s.split_once(':').ok_or_else(|| {
                format!("--master must be Name:axis=pos, got {s:?}")
            })?;
            let (tag, pos) = axis.split_once('=').ok_or_else(|| {
                format!("--master must be Name:axis=pos, got {s:?}")
            })?;
            let pos: f64 = pos
                .trim()
                .parse()
                .map_err(|_| format!("invalid axis position in {s:?}"))?;
            Ok((name.trim().to_string(), tag.trim().to_string(), pos))
        })
        .collect()
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let cli = Cli::parse();

    // Subcommands take over the whole run.
    if let Some(command) = &cli.command {
        return match command {
            Command::Masters(args) => run_masters(&cli, args),
            Command::NewFont(args) => run_new_font(args),
            Command::Stats(args) => run_stats(args),
        };
    }

    // Single-glyph mode.
    let name = cli
        .name
        .clone()
        .ok_or("--name <glyph> is required (or use a subcommand)")?;
    let input = cli.input.clone().ok_or("--input <image> is required")?;
    let output = cli.output.clone().ok_or("--output <path> is required")?;
    let codepoints: Vec<char> = parse_unicode(cli.unicode.as_deref())?;

    let mut opts = opts_from_cli(&cli);
    opts.em_height = cli.target_height;
    opts.verbose = true;

    let mut metrics =
        FontMetrics::from_target_height(cli.target_height, cli.y_offset);
    metrics.advance_width = cli.width;

    // Header
    let unicode_str = cli
        .unicode
        .as_deref()
        .map(|u| format!(" (U+{})", u.to_uppercase()))
        .unwrap_or_default();
    eprintln!();
    eprintln!("  img2bez \u{00b7} {}{}", name, unicode_str);
    eprintln!();

    // Framed path (trace + place by metrics) or, with `--fit-source ink`,
    // deterministic ink-bounds placement. `comparison_dx` maps the placed
    // outline back to source space for the overlay (framed path only).
    let (placed, comparison_dx) = match cli.fit_source {
        FitSource::Canvas => {
            let outline = img2bez::trace_file(&input, &opts)?;
            let min_x = outline
                .contours
                .iter()
                .flat_map(|c| &c.points)
                .filter(|p| p.kind != img2bez::PointKind::OffCurve)
                .map(|p| p.x)
                .fold(f64::MAX, f64::min);
            (
                img2bez::place(&outline, &metrics),
                Some(metrics.lsb - min_x),
            )
        }
        FitSource::Ink => {
            // Vertical band: --fit-y-from wins over an explicit --fit-y.
            let (ymin, ymax) = if let Some(spec) = cli.fit_y_from.as_deref() {
                let (ufo, glyph) = parse_ref_spec(spec)?;
                img2bez::read_reference(&ufo, &glyph)?
                    .vertical_band()
                    .ok_or_else(|| {
                        format!("reference glyph {glyph:?} has no outline")
                    })?
            } else {
                let spec = cli.fit_y.as_deref().ok_or(
                    "--fit-y <ymin:ymax> (or --fit-y-from) is required with --fit-source ink",
                )?;
                spec.split_once(':')
                    .and_then(|(a, b)| {
                        Some((
                            a.trim().parse::<f64>().ok()?,
                            b.trim().parse::<f64>().ok()?,
                        ))
                    })
                    .ok_or("--fit-y must be \"ymin:ymax\", e.g. \"-256:768\"")?
            };

            // Sidebearings: --sidebearings-from wins over explicit
            // --lsb/--rsb; --metrics-from pins the advance (FixedAdvance).
            let (ref_lsb, ref_rsb) = match cli.sidebearings_from.as_deref() {
                Some(spec) => {
                    let (ufo, glyph) = parse_ref_spec(spec)?;
                    let (l, r) = img2bez::read_reference(&ufo, &glyph)?
                        .sidebearings()
                        .ok_or_else(|| {
                            format!("reference glyph {glyph:?} has no outline")
                        })?;
                    (Some(l), Some(r))
                }
                None => (None, None),
            };
            let lsb = ref_lsb.or(cli.lsb).unwrap_or(64.0);
            let rsb = ref_rsb.or(cli.rsb).unwrap_or(64.0);

            let sidebearings = if let Some(spec) = cli.metrics_from.as_deref() {
                let (ufo, glyph) = parse_ref_spec(spec)?;
                let width =
                    img2bez::read_reference(&ufo, &glyph)?.advance_width;
                img2bez::Sidebearings::FixedAdvance { width, lsb }
            } else {
                img2bez::Sidebearings::Explicit { lsb, rsb }
            };

            let mut placement = img2bez::PlacementOptions::ink_fit(
                img2bez::TargetBand::Custom {
                    y_min: ymin,
                    y_max: ymax,
                },
                sidebearings,
            );
            placement.grid = cli.grid;
            let bytes = std::fs::read(&input)?;
            let (placed, report) =
                img2bez::trace_place(&bytes, &opts, &placement)?;
            let b = report.final_bounds;
            eprintln!(
                "  Place       ink-fit  bounds x=[{:.0}, {:.0}] y=[{:.0}, {:.0}]  adv {:.0}  lsb {:.0} rsb {:.0}",
                b.x0,
                b.x1,
                b.y0,
                b.y1,
                report.advance_width,
                report.lsb,
                report.rsb
            );
            if report.out_of_target {
                eprintln!(
                    "  warning     placed outline exceeds the target band"
                );
            }
            (placed, None)
        }
    };

    // Reference-free judge: every run carries its own quality signal.
    match std::fs::read(&input) {
        Ok(bytes) => {
            match img2bez::judge::judge_source(&placed.outline, &bytes, &opts) {
                Ok(j) => eprintln!(
                    "  Judge       wild {:.3}  (iou {}  hv {:.2}  micro {}  parsimony {:.2})",
                    j.wild,
                    j.repro_iou.map_or("-".into(), |v| format!("{v:.3}")),
                    j.hv_frac,
                    j.micro_segs,
                    j.parsimony,
                ),
                Err(e) => eprintln!("  warning     judge: {e}"),
            }
        }
        Err(e) => eprintln!("  warning     judge: {e}"),
    }

    // Data-collection log, opt-in via --log or IMG2BEZ_LOG; reads and
    // measures the source only when logging is on.
    if let Some(log_path) = effective_log_path(&cli) {
        match std::fs::read(&input)
            .map(|b| (img2bez::measure_image(&b).ok(), b))
        {
            Ok((Some(stats), bytes)) => log_trace(
                &log_path,
                "trace",
                &name,
                cli.unicode.as_deref(),
                &input.display().to_string(),
                &bytes,
                &stats,
                &opts,
                img2bez::Style::from(cli.style).name(),
                &placed.outline,
            ),
            Ok((None, _)) => eprintln!(
                "  warning     trace log: could not measure {}",
                input.display()
            ),
            Err(e) => eprintln!(
                "  warning     trace log: could not read {}: {e}",
                input.display()
            ),
        }
    }

    match cli.format {
        OutputFormat::Ufo => {
            let glyph = img2bez::ufo::to_glyph(&name, &placed, &codepoints)?;
            let mut font =
                img2bez::ufo::open_or_create_font(&output, &metrics)?;
            font.default_layer_mut().insert_glyph(glyph);
            font.save(&output)?;
        }
        OutputFormat::Glif => {
            let glyph = img2bez::glyph_from_placed(
                &name,
                &codepoints,
                &placed,
                &metrics,
            );
            write_text(&output, &glyph.to_glif())?;
        }
        OutputFormat::Json => {
            let glyph = img2bez::glyph_from_placed(
                &name,
                &codepoints,
                &placed,
                &metrics,
            );
            write_text(&output, &serde_json::to_string_pretty(&glyph)?)?;
        }
        OutputFormat::Svg => {
            write_text(&output, &placed.outline.to_svg_path())?;
        }
    }

    // Footer
    eprintln!();
    eprintln!("  \u{2713} {}", output.display());

    // Optional reference comparison
    if let Some(ref_path) = &cli.reference {
        let traced = img2bez::eval::from_outline(&placed.outline);
        let reference = img2bez::eval::load_glif(ref_path)?;

        // Raster comparison (primary metric: visual similarity)
        let output_dir =
            output.parent().unwrap_or_else(|| std::path::Path::new("."));
        let raster = img2bez::render::raster_compare(
            &traced.paths,
            &reference.paths,
            output_dir,
            &name,
        )?;
        eprintln!();
        eprintln!(
            "  Raster IoU  {:.1}%  (overlap={} traced={} ref={})",
            raster.iou * 100.0,
            raster.overlap_px,
            raster.traced_px,
            raster.ref_px
        );
        eprintln!("  Diff        {}", raster.diff_path.display());

        // Geometric metrics (secondary)
        let report = img2bez::eval::evaluate(
            &traced,
            &reference,
            cli.grid,
            &ref_path.display().to_string(),
        );
        eprint!("{}", report);
    }

    // Visual comparison (framed path only; ink-fit reports bounds instead).
    if let Some(dx) = comparison_dx {
        let comparison_path = output
            .parent()
            .unwrap_or_else(|| std::path::Path::new("."))
            .join(format!("{}_comparison.png", name));
        let font_scale = cli.target_height / {
            let img = image::open(&input)?.into_luma8();
            img.height() as f64
        };
        img2bez::render::render_comparison(
            &input,
            &placed.outline.to_bezpaths(),
            &comparison_path,
            font_scale,
            metrics.descender,
            (dx, 0.0),
            true,
        )?;
        eprintln!("  Compare     {}", comparison_path.display());
    }

    eprintln!();

    Ok(())
}
