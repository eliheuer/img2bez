// Copyright 2026 the img2bez Authors
// SPDX-License-Identifier: Apache-2.0 OR MIT

//! Browser/WASM entry point for img2bez: trace image bytes to UFO GLIF XML.
//!
//! This is a thin wrapper over `img2bez::trace_glyph` so the blog demo (and
//! any other web host) can trace a raster fully client-side. The JSON and GLIF
//! outputs share one canonical glyph model: each on-curve point's type
//! (curve/line/qcurve) and its `smooth` flag, plus the off-curve handles.

use serde::Deserialize;
use wasm_bindgen::prelude::*;

/// Tracing options from JS. All optional; every default comes from the library
/// (`TraceOptions::default()` plus the shared `Profile`), exactly as the CLI
/// builds it — so a bare `{ "glyph": "a" }` traces identically to the CLI.
#[derive(Default, Deserialize)]
#[serde(rename_all = "camelCase", default)]
struct Config {
    glyph: Option<String>,
    /// Hex codepoint, e.g. "0061" for 'a'. Sets the glyph's unicode.
    unicode: Option<String>,
    /// Source-class preset, mirrors the CLI `--profile`: "wild" (default,
    /// unknown raster), "clean" (font render), or "photo" (soft scan of printed
    /// type — applies a de-texture pre-blur). Sets the fit accuracy.
    profile: Option<String>,
    /// Output-shape mode: "default", "smooth" (all-curves), or "line".
    mode: Option<String>,
    /// Drawing style: "basic" (default), "grotesk", "old-style", "geometric",
    /// "brush", "nib", "qalam". Layers design-specific tuning on the base.
    style: Option<String>,
    width: Option<f64>,
    target_height: Option<f64>,
    y_offset: Option<f64>,
    grid: Option<i32>,
    /// Optional line-corner chamfer size in font units. Defaults to 0.
    chamfer: Option<f64>,
    /// Minimum edge length eligible for automatic chamfering.
    chamfer_min_edge: Option<f64>,
    /// Curve-fit accuracy in font units; overrides the profile default.
    accuracy: Option<f64>,
    /// Enable/disable raster-loss refinement. Defaults to true.
    refine: Option<bool>,
    invert: Option<bool>,
    /// Fixed brightness threshold 0-255; omitted = Otsu auto-detect.
    threshold: Option<u8>,
    /// Experimental: enable the learned corner gate (default off; the
    /// CLI equivalent is IMG2BEZ_CORNER_HEAD=1).
    corner_head: Option<bool>,
    /// Experimental: enable the learned 4-class site head (zone
    /// classifier). The better of the two learned paths.
    site_head: Option<bool>,

    // --- Placement (only read by `tracePlaceToJson`) ---
    /// Source box: "canvas" (frame the whole image, default) or "ink" (detect
    /// ink and fit it to the target band — for generated/padded images).
    fit_source: Option<String>,
    /// Target band bottom in font units (e.g. -256 for the descender).
    fit_y_min: Option<f64>,
    /// Target band top in font units (e.g. 768 for cap height).
    fit_y_max: Option<f64>,
    /// Left sidebearing in font units (default 64).
    lsb: Option<f64>,
    /// Right sidebearing in font units (default 64).
    rsb: Option<f64>,
}

#[wasm_bindgen(start)]
pub fn start() {
    console_error_panic_hook::set_once();
}

/// Trace image bytes, place the result, and assemble the UFO-faithful glyph.
/// Shared by all the `traceTo*` entry points.
fn trace_glyph(
    image_bytes: &[u8],
    config_json: &str,
) -> Result<img2bez::Glyph, JsValue> {
    if image_bytes.is_empty() {
        return Err(JsValue::from_str("image bytes are empty"));
    }
    let cfg: Config = serde_json::from_str(config_json)
        .map_err(|e| JsValue::from_str(&format!("config: {e}")))?;
    img2bez::set_corner_head(cfg.corner_head);
    img2bez::set_site_head(cfg.site_head);
    let (mut opts, metrics) = build_config(&cfg);
    opts.rtl_start = cfg
        .unicode
        .as_deref()
        .and_then(|h| u32::from_str_radix(h.trim(), 16).ok())
        .and_then(char::from_u32)
        .is_some_and(img2bez::is_rtl_codepoint);
    let name = cfg.glyph.clone().unwrap_or_else(|| "a".to_string());
    let codepoints: Vec<char> = cfg
        .unicode
        .as_deref()
        .and_then(|h| u32::from_str_radix(h.trim(), 16).ok())
        .and_then(char::from_u32)
        .into_iter()
        .collect();
    img2bez::trace_glyph(image_bytes, name, &codepoints, &opts, &metrics)
        .map_err(|e| JsValue::from_str(&format!("trace: {e}")))
}

/// Trace PNG/JPEG/BMP image bytes to UFO GLIF XML. `config_json` is a JSON
/// object (see `Config`); pass `"{}"` for all defaults.
#[wasm_bindgen(js_name = traceToGlif)]
pub fn trace_to_glif(
    image_bytes: &[u8],
    config_json: &str,
) -> Result<String, JsValue> {
    Ok(trace_glyph(image_bytes, config_json)?.to_glif())
}

/// Trace image bytes to a UFO-faithful JSON glyph: `{ name, unicodes, advance,
/// unitsPerEm, outline: { contours: [{ points: [{ x, y, type?, smooth? }] }] } }`.
/// The structured form web editors and agents consume directly (no XML parsing).
#[wasm_bindgen(js_name = traceToJson)]
pub fn trace_to_json(
    image_bytes: &[u8],
    config_json: &str,
) -> Result<String, JsValue> {
    let glyph = trace_glyph(image_bytes, config_json)?;
    serde_json::to_string(&glyph)
        .map_err(|e| JsValue::from_str(&format!("json: {e}")))
}

/// Trace image bytes and judge the result against the source: returns
/// `{ "glyph": <Glyph>, "judge": <Judgement> }` as JSON. The judge is the
/// reference-free quality score (reproduction IoU + structural regularizers,
/// same scale as the CLI's `Judge` line) — the number a tuning loop moves.
#[wasm_bindgen(js_name = traceWithJudge)]
pub fn trace_with_judge(
    image_bytes: &[u8],
    config_json: &str,
) -> Result<String, JsValue> {
    if image_bytes.is_empty() {
        return Err(JsValue::from_str("image bytes are empty"));
    }
    let cfg: Config = serde_json::from_str(config_json)
        .map_err(|e| JsValue::from_str(&format!("config: {e}")))?;
    img2bez::set_corner_head(cfg.corner_head);
    img2bez::set_site_head(cfg.site_head);
    let (mut opts, metrics) = build_config(&cfg);
    opts.rtl_start = cfg
        .unicode
        .as_deref()
        .and_then(|h| u32::from_str_radix(h.trim(), 16).ok())
        .and_then(char::from_u32)
        .is_some_and(img2bez::is_rtl_codepoint);
    let name = cfg.glyph.clone().unwrap_or_else(|| "a".to_string());
    let codepoints: Vec<char> = cfg
        .unicode
        .as_deref()
        .and_then(|h| u32::from_str_radix(h.trim(), 16).ok())
        .and_then(char::from_u32)
        .into_iter()
        .collect();
    // Trace via the placement path so the exact glyph-units -> source-pixel
    // affine (report.scale / report.translation) travels with the outline:
    // the demo overlays the trace on the image with it, instead of guessing
    // an alignment from bounding boxes.
    let placement = build_placement(&cfg, opts.grid)?;
    let (placed, report) = img2bez::trace_place(image_bytes, &opts, &placement)
        .map_err(|e| JsValue::from_str(&format!("trace: {e}")))?;
    let glyph =
        img2bez::glyph_from_placed(name, &codepoints, &placed, &metrics);
    let judge = img2bez::judge::judge_source(&glyph.outline, image_bytes, &opts)
        .map_err(|e| JsValue::from_str(&format!("judge: {e}")))?;
    let out =
        serde_json::json!({ "glyph": glyph, "judge": judge, "report": report });
    serde_json::to_string(&out)
        .map_err(|e| JsValue::from_str(&format!("json: {e}")))
}

/// Trace image bytes to an SVG `<path>` `d` string (y-up; flip for a y-down
/// SVG viewport).
#[wasm_bindgen(js_name = traceToSvg)]
pub fn trace_to_svg(
    image_bytes: &[u8],
    config_json: &str,
) -> Result<String, JsValue> {
    Ok(trace_glyph(image_bytes, config_json)?.outline.to_svg_path())
}

/// Trace, then place deterministically from explicit placement intent, and
/// return `{ "glyph": <Glyph>, "report": <PlacementReport> }` as JSON. This is
/// the headless path for generated/padded rasters: with `fitSource: "ink"` the
/// ink box is fitted to `[fitYMin, fitYMax]` regardless of image margins.
/// Mirrors the CLI `--fit-source ink` exactly (same library call).
#[wasm_bindgen(js_name = tracePlaceToJson)]
pub fn trace_place_to_json(
    image_bytes: &[u8],
    config_json: &str,
) -> Result<String, JsValue> {
    if image_bytes.is_empty() {
        return Err(JsValue::from_str("image bytes are empty"));
    }
    let cfg: Config = serde_json::from_str(config_json)
        .map_err(|e| JsValue::from_str(&format!("config: {e}")))?;
    img2bez::set_corner_head(cfg.corner_head);
    img2bez::set_site_head(cfg.site_head);
    let (mut opts, metrics) = build_config(&cfg);
    opts.verbose = false;

    let name = cfg.glyph.clone().unwrap_or_else(|| "a".to_string());
    let codepoints: Vec<char> = cfg
        .unicode
        .as_deref()
        .and_then(|h| u32::from_str_radix(h.trim(), 16).ok())
        .and_then(char::from_u32)
        .into_iter()
        .collect();

    let placement = build_placement(&cfg, opts.grid)?;
    let (placed, report) = img2bez::trace_place(image_bytes, &opts, &placement)
        .map_err(|e| JsValue::from_str(&format!("trace: {e}")))?;
    let glyph =
        img2bez::glyph_from_placed(name, &codepoints, &placed, &metrics);

    #[derive(serde::Serialize)]
    struct Out {
        glyph: img2bez::Glyph,
        report: img2bez::PlacementReport,
    }
    serde_json::to_string(&Out { glyph, report })
        .map_err(|e| JsValue::from_str(&format!("json: {e}")))
}

/// Build `PlacementOptions` from the config. `fitSource: "ink"` requires
/// `fitYMin`/`fitYMax`; "canvas" falls back to the metrics frame
/// (descender..ascender) so it matches the framed `trace` path.
fn build_placement(
    cfg: &Config,
    grid: i32,
) -> Result<img2bez::PlacementOptions, JsValue> {
    use img2bez::{Sidebearings, SourceBox, TargetBand};
    let lsb = cfg.lsb.unwrap_or(64.0);
    let rsb = cfg.rsb.unwrap_or(64.0);
    let sidebearings = Sidebearings::Explicit { lsb, rsb };

    let source = match cfg.fit_source.as_deref().unwrap_or("canvas") {
        "ink" => SourceBox::InkBounds,
        "canvas" => SourceBox::Canvas,
        other => {
            return Err(JsValue::from_str(&format!(
                "fitSource must be \"ink\" or \"canvas\", got {other:?}"
            )));
        }
    };

    let (y_min, y_max) = match (cfg.fit_y_min, cfg.fit_y_max) {
        (Some(lo), Some(hi)) => (lo, hi),
        _ if source == SourceBox::InkBounds => {
            return Err(JsValue::from_str(
                "fitSource \"ink\" requires fitYMin and fitYMax",
            ));
        }
        // Canvas with no band given: use the default render frame.
        _ => (-256.0, 832.0),
    };

    let mut placement = img2bez::PlacementOptions::ink_fit(
        TargetBand::Custom { y_min, y_max },
        sidebearings,
    );
    placement.source = source;
    placement.grid = grid;
    Ok(placement)
}

/// Build `(TraceOptions, FontMetrics)` from the JSON config, starting from the
/// library defaults — the same source of truth the CLI uses — and overriding
/// only the fields JS actually provided. The profile supplies the fit accuracy
/// (wild by default), exactly as the CLI's `--profile` does.
fn build_config(cfg: &Config) -> (img2bez::TraceOptions, img2bez::FontMetrics) {
    let profile = img2bez::Profile::from_name_lossy(
        cfg.profile.as_deref().unwrap_or("wild"),
    );

    let mut opts = img2bez::TraceOptions::for_profile(profile);
    opts.verbose = false;
    if let Some(accuracy) = cfg.accuracy {
        opts.fit_accuracy = accuracy.max(0.1);
    }
    if let Some(h) = cfg.target_height {
        opts.em_height = h.max(1.0);
    }
    if let Some(g) = cfg.grid {
        opts.grid = g.max(0);
    }
    if let Some(chamfer) = cfg.chamfer {
        opts.chamfer_size = chamfer.max(0.0);
    }
    if let Some(min_edge) = cfg.chamfer_min_edge {
        opts.chamfer_min_edge = min_edge.max(0.0);
    }
    if let Some(refine) = cfg.refine {
        opts.refine_raster = refine;
    }
    if let Some(inv) = cfg.invert {
        opts.invert = inv;
    }
    if let Some(t) = cfg.threshold {
        opts.threshold = img2bez::ThresholdMethod::Fixed(t);
    }
    if let Some(m) = cfg.mode.as_deref() {
        opts.mode = match m {
            "smooth" => img2bez::TraceMode::Smooth,
            "line" => img2bez::TraceMode::LineOnly,
            _ => img2bez::TraceMode::Default,
        };
    }
    // Layer the drawing-style tuning on top (mirrors the CLI --style).
    img2bez::Style::from_name_lossy(cfg.style.as_deref().unwrap_or("basic"))
        .apply(&mut opts);

    // Mirror the CLI: targetHeight is ascender - descender, yOffset is the
    // descender, and ascender = targetHeight + yOffset. The source UPM stays
    // at the FontMetrics default unless the Rust caller changes it.
    let target_height = cfg.target_height.map(|h| h.max(1.0)).unwrap_or(1088.0);
    let y_offset = cfg.y_offset.unwrap_or(-256.0);
    let mut metrics =
        img2bez::FontMetrics::from_target_height(target_height, y_offset);
    metrics.advance_width = cfg.width;

    (opts, metrics)
}
