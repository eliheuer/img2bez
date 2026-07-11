// Copyright 2026 the img2bez Authors
// SPDX-License-Identifier: Apache-2.0 OR MIT

//! Convert a placed [`crate::Outline`] to UFO glyph / font format via `norad`.
//!
//! The point stream (types + the `smooth` flag) is produced by [`crate::Outline`], so
//! this module just maps the canonical model onto `norad`'s types.

use std::path::Path;

use kurbo::Shape;
use norad::{Contour, ContourPoint, Glyph, PointType};

use crate::model::config::FontMetrics;
use crate::model::error::TraceError;
use crate::model::outline::{self, PointKind};
use crate::placement::PlacedGlyph;

/// Metrics read from an existing UFO glyph, so a new glyph can copy a
/// neighbor's design decisions (advance, vertical fit, sidebearings) in
/// font-editor terms rather than as raw numbers. See [`read_reference`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ReferenceMetrics {
    /// The reference glyph's advance width.
    pub advance_width: f64,
    /// Tight ink bounds in font units, or `None` if the glyph has no
    /// contours.
    pub bounds: Option<kurbo::Rect>,
}

impl ReferenceMetrics {
    /// The reference ink's vertical band `(y_min, y_max)` — a `--fit-y-from`
    /// source (e.g. copy a 'b''s baseline-to-ascender extent).
    pub fn vertical_band(&self) -> Option<(f64, f64)> {
        self.bounds.map(|r| (r.y0, r.y1))
    }

    /// The reference glyph's `(lsb, rsb)` — a `--sidebearings-from` source.
    pub fn sidebearings(&self) -> Option<(f64, f64)> {
        self.bounds.map(|r| (r.x0, self.advance_width - r.x1))
    }
}

/// Read [`ReferenceMetrics`] from glyph `glyph_name` in the UFO at `path`.
///
/// The bounds are the tight outline bounding box (via kurbo), so copied
/// vertical fits and sidebearings match what a font editor would report.
pub fn read_reference(
    path: &Path,
    glyph_name: &str,
) -> Result<ReferenceMetrics, TraceError> {
    let font = norad::Font::load(path)?;
    let glyph = font.get_glyph(glyph_name).ok_or_else(|| {
        TraceError::ReferenceGlyphNotFound {
            ufo: path.display().to_string(),
            glyph: glyph_name.to_string(),
        }
    })?;

    let mut bbox: Option<kurbo::Rect> = None;
    for contour in &glyph.contours {
        let path = contour
            .to_kurbo()
            .map_err(|e| TraceError::ContourConvert(e.to_string()))?;
        let b = path.bounding_box();
        bbox = Some(bbox.map_or(b, |acc| acc.union(b)));
    }

    Ok(ReferenceMetrics {
        advance_width: glyph.width,
        bounds: bbox,
    })
}

/// Load the UFO at `path`, or create a fresh one if it does not exist. A new
/// font gets font info from the metrics (units-per-em, ascender, descender)
/// and a family name from the file stem.
pub fn open_or_create_font(
    path: &Path,
    metrics: &FontMetrics,
) -> Result<norad::Font, TraceError> {
    if path.exists() {
        return Ok(norad::Font::load(path)?);
    }
    let mut font = norad::Font::new();
    let upm = metrics.units_per_em.max(1.0).round() as u32;
    font.font_info.units_per_em = Some(upm.into());
    font.font_info.ascender = Some(metrics.ascender.round());
    font.font_info.descender = Some(metrics.descender.round());
    font.font_info.family_name = Some(
        path.file_stem()
            .and_then(|stem| stem.to_str())
            .unwrap_or("Untitled")
            .to_string(),
    );
    font.font_info.style_name = Some("Regular".to_string());
    Ok(font)
}

/// Convert a placed glyph to a `norad::Glyph`.
pub fn to_glyph(
    name: &str,
    placed: &PlacedGlyph,
    codepoints: &[char],
) -> Result<Glyph, TraceError> {
    let mut glyph = Glyph::new(name);
    glyph.width = placed.advance_width;

    for &codepoint in codepoints {
        glyph.codepoints.insert(codepoint);
    }
    for contour in &placed.outline.contours {
        if contour.points.is_empty() {
            continue;
        }
        glyph.contours.push(to_contour(contour));
    }

    Ok(glyph)
}

/// Convert a canonical [`outline::Contour`] to a `norad::Contour`.
/// Glue for [`to_glyph`]; callers convert whole glyphs, not lone contours.
pub(crate) fn to_contour(contour: &outline::Contour) -> Contour {
    let points = contour
        .points
        .iter()
        .map(|p| {
            let typ = match p.kind {
                PointKind::Move => PointType::Move,
                PointKind::Line => PointType::Line,
                PointKind::Curve => PointType::Curve,
                PointKind::QCurve => PointType::QCurve,
                PointKind::OffCurve => PointType::OffCurve,
            };
            ContourPoint::new(p.x, p.y, typ, p.smooth, None, None)
        })
        .collect();
    Contour::new(points, None)
}
