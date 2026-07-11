// Copyright 2026 the img2bez Authors
// SPDX-License-Identifier: Apache-2.0 OR MIT

//! A traced glyph in a UFO-faithful shape.
//!
//! [`Outline`] is pure geometry; a [`Glyph`] wraps it with the glyph-level
//! metadata a font needs — name, unicode codepoints, advance — laid out to
//! mirror a UFO `.glif`. This is the single glyph-level output type that the
//! JSON, GLIF, and UFO writers all serialize, so they can't drift apart.
//!
//! UFO is [img2bez's primary target format](https://unifiedfontobject.org/);
//! the JSON is a direct projection of the GLIF structure (name, zero-or-more
//! `<unicode hex>`, `<advance>`, `<outline>` of contours of points).

use serde::{Deserialize, Serialize};

use crate::model::config::FontMetrics;
use crate::model::outline::Outline;
use crate::pipeline::placement::{PlacedGlyph, place};

/// A glyph's advance, mirroring UFO `<advance width="…" height="…"/>`. `height`
/// is omitted in JSON when zero (the usual case for horizontal text).
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Advance {
    /// Horizontal advance width in font units.
    pub width: f64,
    /// Vertical advance height in font units (0 for horizontal fonts).
    #[serde(default, skip_serializing_if = "is_zero")]
    pub height: f64,
}

fn is_zero(v: &f64) -> bool {
    *v == 0.0
}

/// A traced glyph: an [`Outline`] plus the glyph-level metadata a font carries.
///
/// The JSON form mirrors a UFO `.glif`:
///
/// ```json
/// {
///   "name": "Q",
///   "unicodes": ["0051"],
///   "advance": { "width": 896 },
///   "unitsPerEm": 1024,
///   "outline": { "contours": [ { "points": [
///     { "x": 450, "y": 752, "type": "curve", "smooth": true },
///     { "x": 210, "y": 752 }
///   ] } ] }
/// }
/// ```
///
/// ## Stability
///
/// This JSON shape is the **stable schema for the `0.1.x` series**: tools
/// and agents may depend on it — within `0.1.x` keys are not renamed,
/// removed, or added (these structs have public fields, so even an additive
/// change is a Rust semver break and comes with a `0.x` minor-version bump).
/// There is no `schema` version field — the model mirrors UFO `.glif`,
/// which has none; callers needing an explicit version should wrap this in
/// their own envelope.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct Glyph {
    /// Glyph name.
    pub name: String,
    /// Unicode codepoints as uppercase hex strings (UFO `<unicode hex=…>`); a
    /// glyph may map zero, one, or several. Omitted in JSON when empty.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub unicodes: Vec<String>,
    /// The glyph's advance.
    pub advance: Advance,
    /// The font's units-per-em. Font-level in UFO; carried here as context for
    /// a single traced glyph so a consumer knows its coordinate scale.
    pub units_per_em: f64,
    /// The glyph's contours.
    pub outline: Outline,
}

impl Glyph {
    /// Assemble a glyph from a placed outline and its metadata. `codepoints`
    /// are converted to UFO-style uppercase hex strings.
    pub fn new(
        name: impl Into<String>,
        codepoints: &[char],
        advance_width: f64,
        units_per_em: f64,
        outline: Outline,
    ) -> Self {
        Glyph {
            name: name.into(),
            unicodes: codepoints
                .iter()
                .map(|c| format!("{:04X}", *c as u32))
                .collect(),
            advance: Advance {
                width: advance_width,
                height: 0.0,
            },
            units_per_em,
            outline,
        }
    }

    /// Place an existing neutral outline with the font metrics and assemble
    /// a UFO-faithful glyph. Also available as the free function
    /// [`crate::glyph_from_outline`].
    pub fn from_outline(
        name: impl Into<String>,
        codepoints: &[char],
        outline: &Outline,
        metrics: &FontMetrics,
    ) -> Glyph {
        let placed = place(outline, metrics);
        Glyph::from_placed(name, codepoints, &placed, metrics)
    }

    /// Assemble a UFO-faithful glyph from an already placed outline. Also
    /// available as the free function [`crate::glyph_from_placed`].
    pub fn from_placed(
        name: impl Into<String>,
        codepoints: &[char],
        placed: &PlacedGlyph,
        metrics: &FontMetrics,
    ) -> Glyph {
        Glyph::new(
            name,
            codepoints,
            placed.advance_width,
            metrics.units_per_em(),
            placed.outline.clone(),
        )
    }

    /// Serialize to UFO GLIF XML.
    pub fn to_glif(&self) -> String {
        crate::io::glif::write_glif(self)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::model::outline::{Contour, OutlinePoint, PointKind};

    fn sample() -> Glyph {
        let outline = Outline {
            contours: vec![Contour {
                points: vec![
                    OutlinePoint {
                        x: 0.0,
                        y: 0.0,
                        kind: PointKind::Line,
                        smooth: false,
                    },
                    OutlinePoint {
                        x: 10.0,
                        y: 0.0,
                        kind: PointKind::OffCurve,
                        smooth: false,
                    },
                    OutlinePoint {
                        x: 10.0,
                        y: 10.0,
                        kind: PointKind::OffCurve,
                        smooth: false,
                    },
                    OutlinePoint {
                        x: 0.0,
                        y: 10.0,
                        kind: PointKind::Curve,
                        smooth: true,
                    },
                ],
            }],
        };
        Glyph::new("Q", &['Q'], 896.0, 1024.0, outline)
    }

    #[test]
    fn json_mirrors_ufo_point_model() {
        let json = serde_json::to_string(&sample()).unwrap();
        // Codepoints as a hex array, advance object, nested outline.
        assert!(json.contains(r#""unicodes":["0051"]"#), "{json}");
        assert!(json.contains(r#""advance":{"width":896.0}"#), "{json}");
        assert!(json.contains(r#""outline":{"contours""#), "{json}");
        // On-curve carries `type`; off-curve omits it (UFO).
        assert!(json.contains(r#""type":"curve""#), "{json}");
        assert!(json.contains(r#""smooth":true"#), "{json}");
        // The two off-curve points serialize with no `type`/`smooth`.
        assert!(json.contains(r#"{"x":10.0,"y":0.0}"#), "{json}");
    }

    #[test]
    fn json_round_trips() {
        let g = sample();
        let back: Glyph =
            serde_json::from_str(&serde_json::to_string(&g).unwrap()).unwrap();
        assert_eq!(g, back);
    }
}
