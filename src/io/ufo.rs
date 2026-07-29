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


/// Insert or replace ONE glyph in an on-disk UFO by writing only its `.glif`
/// (plus, for a brand-new glyph, the `glyphs/contents.plist` entry). Never
/// re-serializes the rest of the font — a full `norad::Font::save` reformats
/// every file in its own style (observed: 638-file diff in a live repo).
///
/// # Errors
///
/// [`TraceError::UfoWrite`] if the UFO has no readable
/// `glyphs/contents.plist` (fresh fonts should go through a full save) or
/// the plist/glif write fails.
pub fn write_glyph_surgical(
    ufo_path: &Path,
    glyph: &Glyph,
) -> Result<(), TraceError> {
    let glyphs_dir = ufo_path.join("glyphs");
    let contents_path = glyphs_dir.join("contents.plist");
    let mut contents: plist::Dictionary = plist::from_file(&contents_path)
        .map_err(|e| TraceError::UfoWrite(format!(
            "read {}: {e}", contents_path.display())))?;
    let name = glyph.name().to_string();
    let file_name = match contents.get(&name).and_then(|v| v.as_string()) {
        Some(existing) => existing.to_string(),
        None => {
            let fname = glif_file_name(&name, &contents);
            contents.insert(name, plist::Value::String(fname.clone()));
            plist::to_file_xml(&contents_path, &contents).map_err(|e| {
                TraceError::UfoWrite(format!(
                    "write {}: {e}", contents_path.display()))
            })?;
            fname
        }
    };
    glyph
        .save(glyphs_dir.join(&file_name))
        .map_err(|e| TraceError::UfoWrite(format!("write {file_name}: {e}")))
}

/// UFO3-style glif file name: each ASCII uppercase letter gets a trailing
/// underscore; unsafe characters become underscores. Uniqueness against the
/// existing contents values is enforced with a numeric suffix. (Simplified
/// from the UFO spec's full user-name-to-file-name algorithm; sufficient for
/// glyph names img2bez produces.)
fn glif_file_name(name: &str, contents: &plist::Dictionary) -> String {
    let mut stem = String::with_capacity(name.len() + 4);
    for ch in name.chars() {
        if ch.is_ascii_uppercase() {
            stem.push(ch);
            stem.push('_');
        } else if ch.is_ascii_alphanumeric() || matches!(ch, '.' | '_' | '-') {
            stem.push(ch);
        } else {
            stem.push('_');
        }
    }
    let taken: std::collections::HashSet<&str> = contents
        .values()
        .filter_map(|v| v.as_string())
        .collect();
    let mut candidate = format!("{stem}.glif");
    let mut n = 1;
    while taken.contains(candidate.as_str()) {
        candidate = format!("{stem}.{n:03}.glif");
        n += 1;
    }
    candidate
}


#[cfg(test)]
mod surgical_tests {
    use super::*;
    use crate::model::outline::{Contour, Outline, OutlinePoint, PointKind};

    fn square(x0: f64, y0: f64, s: f64, ccw: bool) -> Contour {
        let mut pts = vec![
            (x0, y0),
            (x0 + s, y0),
            (x0 + s, y0 + s),
            (x0, y0 + s),
        ];
        if !ccw {
            pts.reverse();
        }
        Contour {
            points: pts
                .into_iter()
                .map(|(x, y)| OutlinePoint {
                    x,
                    y,
                    kind: PointKind::Line,
                    smooth: false,
                })
                .collect(),
        }
    }

    #[test]
    fn fix_directions_orients_by_nesting() {
        // outer CW (wrong), hole CCW (wrong), island inside hole CW (wrong)
        let mut o = Outline {
            contours: vec![
                square(0.0, 0.0, 100.0, false),
                square(10.0, 10.0, 80.0, true),
                square(20.0, 20.0, 40.0, false),
            ],
        };
        o.fix_directions();
        assert!(o.contours[0].signed_area() > 0.0, "outer must be CCW");
        assert!(o.contours[1].signed_area() < 0.0, "hole must be CW");
        assert!(o.contours[2].signed_area() > 0.0, "island must be CCW");
    }

    #[test]
    fn reverse_moves_segment_types_and_keeps_smooth() {
        use PointKind::*;
        // line into A, cubic (2 offs) into B: after reversal the cubic
        // arrives at A and the line at B.
        let mk = |x: f64, y: f64, kind, smooth| OutlinePoint { x, y, kind, smooth };
        let mut c = Contour {
            points: vec![
                mk(0.0, 0.0, Line, false),        // A
                mk(1.0, 0.0, OffCurve, false),
                mk(2.0, 0.0, OffCurve, false),
                mk(3.0, 0.0, Curve, true),        // B
            ],
        };
        c.reverse();
        let kinds: Vec<PointKind> =
            c.points.iter().map(|p| p.kind).collect();
        assert_eq!(kinds.iter().filter(|k| **k == OffCurve).count(), 2);
        let a = c.points.iter().find(|p| p.x == 0.0 && p.y == 0.0).unwrap();
        let b = c.points.iter().find(|p| p.x == 3.0 && p.y == 0.0).unwrap();
        assert_eq!(a.kind, Curve, "cubic now arrives at A");
        assert_eq!(b.kind, Line, "line now arrives at B");
        assert!(b.smooth, "smooth stays with its point");
    }

    #[test]
    fn surgical_write_touches_only_glyph_files() {
        let dir = tempfile::tempdir().unwrap();
        let ufo = dir.path().join("T.ufo");
        let mut font = norad::Font::new();
        font.font_info.family_name = Some("T".into());
        let mut g = Glyph::new("a");
        g.width = 500.0;
        font.default_layer_mut().insert_glyph(g);
        font.save(&ufo).unwrap();
        let fontinfo = ufo.join("fontinfo.plist");
        let before = std::fs::read(&fontinfo).unwrap();
        let mtime = std::fs::metadata(&fontinfo).unwrap().modified().unwrap();

        let mut g2 = Glyph::new("b");
        g2.width = 640.0;
        write_glyph_surgical(&ufo, &g2).unwrap();

        assert_eq!(std::fs::read(&fontinfo).unwrap(), before);
        assert_eq!(
            std::fs::metadata(&fontinfo).unwrap().modified().unwrap(),
            mtime,
            "fontinfo must not be rewritten"
        );
        let contents: plist::Dictionary =
            plist::from_file(ufo.join("glyphs/contents.plist")).unwrap();
        assert!(contents.contains_key("b"));
        let f = norad::Font::load(&ufo).unwrap();
        assert!(f.default_layer().get_glyph("b").is_some());
    }
}
