// Copyright 2026 the img2bez Authors
// SPDX-License-Identifier: Apache-2.0 OR MIT

//! Variable-font master support (`ufo` feature): read a `.designspace`'s
//! masters and vertical zones, resolve semantic fit specs, and scaffold a
//! new font (designspace + empty master UFOs).

use std::path::{Path, PathBuf};

use crate::model::config::FontMetrics;
use crate::model::error::TraceError;

/// A master discovered in a designspace: its style name and resolved UFO path.
#[derive(Debug, Clone)]
pub struct MasterRef {
    /// The master's name (its `stylename`, e.g. "Bold"), used to map images.
    pub name: String,
    /// The master UFO path, resolved relative to the designspace's directory.
    pub ufo_path: PathBuf,
}

/// A font's vertical zones in font units, for resolving `--fit`.
#[derive(Debug, Clone, Copy)]
pub struct VerticalZones {
    /// Units per em.
    pub units_per_em: f64,
    /// Descender (typically negative).
    pub descender: f64,
    /// Baseline — always 0, the origin.
    pub baseline: f64,
    /// X-height.
    pub x_height: f64,
    /// Cap height.
    pub cap_height: f64,
    /// Ascender.
    pub ascender: f64,
}

impl VerticalZones {
    /// `FontMetrics` for placement/glyph assembly (UPM + ascender/descender).
    pub fn metrics(&self) -> FontMetrics {
        FontMetrics::new(self.ascender, self.descender)
            .with_units_per_em(self.units_per_em)
    }

    /// Resolve one zone name (or a raw number) to a font-unit coordinate.
    fn coord(&self, name: &str) -> Result<f64, String> {
        match name.trim().to_ascii_lowercase().as_str() {
            "descender" | "desc" => Ok(self.descender),
            "baseline" | "base" => Ok(self.baseline),
            "xheight" | "x-height" | "x" => Ok(self.x_height),
            "cap" | "capheight" | "cap-height" => Ok(self.cap_height),
            "ascender" | "asc" => Ok(self.ascender),
            other => other
                .parse::<f64>()
                .map_err(|_| format!("unknown fit zone {other:?}")),
        }
    }

    /// Resolve a fit spec like `descender:cap` to a `(y_min, y_max)` band.
    ///
    /// # Errors
    ///
    /// [`TraceError::InvalidFitSpec`] when
    /// either zone name is unknown or the spec is not `zone:zone`.
    pub fn resolve_fit(&self, spec: &str) -> Result<(f64, f64), TraceError> {
        let invalid = TraceError::InvalidFitSpec;
        let (a, b) = spec.split_once(':').ok_or_else(|| {
            invalid(format!(
                "expected 'zone:zone' (e.g. descender:cap), got {spec:?}"
            ))
        })?;
        Ok((
            self.coord(a).map_err(invalid)?,
            self.coord(b).map_err(invalid)?,
        ))
    }
}

/// Read a designspace's masters (style name → UFO path), in document order.
pub fn read_masters(designspace: &Path) -> Result<Vec<MasterRef>, TraceError> {
    let doc = norad::designspace::DesignSpaceDocument::load(designspace)
        .map_err(|e| TraceError::Designspace(e.to_string()))?;
    let dir = designspace.parent().unwrap_or_else(|| Path::new("."));
    Ok(doc
        .sources
        .iter()
        .map(|s| {
            let name = s
                .stylename
                .clone()
                .or_else(|| s.name.clone())
                .unwrap_or_else(|| s.filename.clone());
            MasterRef {
                name,
                ufo_path: dir.join(&s.filename),
            }
        })
        .collect())
}

/// Read a master UFO's vertical zones (and a `FontMetrics`). Missing fields
/// fall back to em-proportional defaults so an under-specified UFO still works.
pub fn read_zones(ufo: &Path) -> Result<VerticalZones, TraceError> {
    let font = norad::Font::load(ufo)?;
    let fi = &font.font_info;
    let upm: f64 = fi.units_per_em.map(|v| v.as_f64()).unwrap_or(1000.0);
    Ok(VerticalZones {
        units_per_em: upm,
        descender: fi.descender.unwrap_or(-0.2 * upm),
        baseline: 0.0,
        x_height: fi.x_height.unwrap_or(0.5 * upm),
        cap_height: fi.cap_height.unwrap_or(0.7 * upm),
        ascender: fi.ascender.unwrap_or(0.8 * upm),
    })
}

/// Scaffold a new variable font: write a `.designspace` plus one empty UFO per
/// master (each with `zones` as its metrics). `masters` is `(style name, axis
/// tag, position)`; the first master's axis tag defines the single axis and the
/// first position is the default. Errors if the designspace exists unless
/// `force`.
pub fn create_font(
    designspace: &Path,
    family: &str,
    masters: &[(String, String, f64)],
    zones: &VerticalZones,
    force: bool,
) -> Result<(), TraceError> {
    use norad::designspace::{Axis, DesignSpaceDocument, Dimension, Source};

    if masters.is_empty() {
        return Err(TraceError::Designspace("no masters given".into()));
    }
    if designspace.exists() && !force {
        return Err(TraceError::Designspace(format!(
            "{} already exists (use --force to overwrite)",
            designspace.display()
        )));
    }
    let dir = designspace.parent().unwrap_or_else(|| Path::new("."));
    std::fs::create_dir_all(dir)
        .map_err(|e| TraceError::Designspace(e.to_string()))?;

    let tag = masters[0].1.clone();
    let axis_name = axis_name_for(&tag);
    let positions: Vec<f64> = masters.iter().map(|m| m.2).collect();
    let min = positions.iter().copied().fold(f64::INFINITY, f64::min);
    let max = positions.iter().copied().fold(f64::NEG_INFINITY, f64::max);

    let mut doc = DesignSpaceDocument {
        format: 5.0,
        ..Default::default()
    };
    doc.axes.push(Axis {
        name: axis_name.clone(),
        tag: tag.clone(),
        default: positions[0] as f32,
        minimum: Some(min as f32),
        maximum: Some(max as f32),
        ..Default::default()
    });

    let upm = norad::fontinfo::NonNegativeIntegerOrFloat::try_from(
        zones.units_per_em,
    )
    .map_err(|_| TraceError::Designspace("units-per-em must be > 0".into()))?;

    for (name, _tag, pos) in masters {
        let filename = format!("{}-{}.ufo", slug(family), slug(name));
        let ufo_path = dir.join(&filename);
        let mut font = norad::Font::new();
        font.font_info.family_name = Some(family.to_string());
        font.font_info.style_name = Some(name.clone());
        font.font_info.units_per_em = Some(upm);
        font.font_info.ascender = Some(zones.ascender);
        font.font_info.descender = Some(zones.descender);
        font.font_info.cap_height = Some(zones.cap_height);
        font.font_info.x_height = Some(zones.x_height);
        font.save(&ufo_path)?;

        doc.sources.push(Source {
            filename,
            stylename: Some(name.clone()),
            name: Some(format!("{family} {name}")),
            location: vec![Dimension {
                name: axis_name.clone(),
                xvalue: Some(*pos as f32),
                ..Default::default()
            }],
            ..Default::default()
        });
    }

    doc.save(designspace)
        .map_err(|e| TraceError::Designspace(e.to_string()))?;
    Ok(())
}

/// Human axis name for a known tag (else the tag, upper-cased).
fn axis_name_for(tag: &str) -> String {
    match tag {
        "wght" => "Weight".into(),
        "wdth" => "Width".into(),
        "opsz" => "Optical Size".into(),
        "ital" => "Italic".into(),
        "slnt" => "Slant".into(),
        other => other.to_uppercase(),
    }
}

/// A filesystem-safe token from a name (alphanumerics kept, others dropped).
fn slug(s: &str) -> String {
    let t: String = s.chars().filter(|c| c.is_alphanumeric()).collect();
    if t.is_empty() { "Font".into() } else { t }
}
