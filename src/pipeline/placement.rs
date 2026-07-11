// Copyright 2026 the img2bez Authors
// SPDX-License-Identifier: Apache-2.0 OR MIT

//! Deterministic placement: map traced contours into a font's coordinate
//! system from explicit, caller-supplied intent — source box, vertical
//! target band, sidebearings/advance. img2bez never guesses typography from
//! a codepoint; this is what makes an arbitrarily padded raster place
//! repeatably (see [`crate::trace_place`]).

use image::GrayImage;
use kurbo::{Rect, Shape};
use serde::Serialize;

/// Serde adapter: serialize a [`kurbo::Rect`] as `[x0, y0, x1, y1]` — the
/// shape downstream tooling parses (kurbo's own impl is a named-field
/// struct). Every serialized `Rect` field must opt in via `#[serde(with)]`.
mod rect_array {
    use kurbo::Rect;
    use serde::{Deserialize, Deserializer, Serialize, Serializer};

    pub fn serialize<S: Serializer>(r: &Rect, s: S) -> Result<S::Ok, S::Error> {
        [r.x0, r.y0, r.x1, r.y1].serialize(s)
    }

    /// Kept so a future `Deserialize` derive reads the same 4-array shape.
    #[allow(dead_code)]
    pub fn deserialize<'de, D: Deserializer<'de>>(
        d: D,
    ) -> Result<Rect, D::Error> {
        let [x0, y0, x1, y1] = <[f64; 4]>::deserialize(d)?;
        Ok(Rect::new(x0, y0, x1, y1))
    }
}

use crate::model::config::FontMetrics;
use crate::model::outline::{Contour, Outline, OutlinePoint, PointKind};

/// Which part of the image is the glyph, in pixels.
#[derive(Debug, Clone, Copy, PartialEq)]
#[non_exhaustive]
pub enum SourceBox {
    /// The whole image canvas — correct when the image is framed exactly to
    /// the font's vertical extent (the plain trace+place assumption).
    Canvas,
    /// The detected ink bounding box — correct for arbitrarily padded images.
    InkBounds,
    /// An explicit image-space rectangle in pixels (y-down).
    Custom(Rect),
}

/// The font-unit vertical band the source content is scaled to fill.
#[derive(Debug, Clone, Copy, PartialEq)]
#[non_exhaustive]
pub enum TargetBand {
    /// Baseline (0) up to the cap height.
    BaselineToCap(f64),
    /// Baseline (0) up to the x-height.
    BaselineToXHeight(f64),
    /// Baseline (0) up to the ascender.
    BaselineToAscender(f64),
    /// Descender up to ascender (a descending glyph fills the whole band).
    DescenderToAscender {
        /// Descender in font units (typically negative).
        descender: f64,
        /// Ascender in font units.
        ascender: f64,
    },
    /// An explicit font-unit band `[y_min, y_max]`.
    Custom {
        /// Bottom of the band in font units.
        y_min: f64,
        /// Top of the band in font units.
        y_max: f64,
    },
}

impl TargetBand {
    /// The band as `(y_min, y_max)` in font units.
    pub fn bounds(&self) -> (f64, f64) {
        match *self {
            TargetBand::BaselineToCap(h) => (0.0, h),
            TargetBand::BaselineToXHeight(h) => (0.0, h),
            TargetBand::BaselineToAscender(h) => (0.0, h),
            TargetBand::DescenderToAscender {
                descender,
                ascender,
            } => (descender, ascender),
            TargetBand::Custom { y_min, y_max } => (y_min, y_max),
        }
    }

    /// The band height in font units.
    pub fn height(&self) -> f64 {
        let (lo, hi) = self.bounds();
        hi - lo
    }
}

/// How the horizontal metrics (sidebearings and advance) are set. img2bez never
/// infers these — the caller states intent.
#[derive(Debug, Clone, Copy, PartialEq)]
#[non_exhaustive]
pub enum Sidebearings {
    /// Leftmost ink at `lsb`; advance is the rightmost ink plus `rsb`.
    Explicit {
        /// Left sidebearing in font units.
        lsb: f64,
        /// Right sidebearing in font units.
        rsb: f64,
    },
    /// Fixed advance `width`, leftmost ink at `lsb`.
    FixedAdvance {
        /// Fixed advance width in font units.
        width: f64,
        /// Left sidebearing in font units.
        lsb: f64,
    },
    /// Center the ink horizontally within a fixed `advance`.
    Center {
        /// Fixed advance width in font units.
        advance: f64,
    },
}

/// A full placement intent: where the glyph is in the image, what font band it
/// fills, its horizontal metrics, and the snapping grid.
#[derive(Debug, Clone, Copy, PartialEq)]
#[non_exhaustive]
pub struct PlacementOptions {
    /// What part of the image is the glyph.
    pub source: SourceBox,
    /// The font-unit vertical band the ink is scaled to fill.
    pub vertical: TargetBand,
    /// Horizontal metrics.
    pub sidebearings: Sidebearings,
    /// Coordinate snap grid (0 = off).
    pub grid: i32,
}

impl PlacementOptions {
    /// Fit detected ink into `vertical`, with the given sidebearings. The most
    /// common shape for placing a generated glyph image.
    pub fn ink_fit(vertical: TargetBand, sidebearings: Sidebearings) -> Self {
        PlacementOptions {
            source: SourceBox::InkBounds,
            vertical,
            sidebearings,
            grid: 2,
        }
    }
}

/// What placement did, for headless review. All bounds are font units except
/// `ink_bounds_px`.
#[derive(Debug, Clone, Copy, PartialEq, Serialize)]
#[serde(rename_all = "camelCase")]
#[non_exhaustive]
pub struct PlacementReport {
    /// Source image dimensions in pixels.
    pub image_size: (u32, u32),
    /// Detected ink bbox in pixels (y-down); serializes as `[x0,y0,x1,y1]`.
    #[serde(with = "rect_array")]
    pub ink_bounds_px: Rect,
    /// Uniform scale applied (font units per source pixel).
    pub scale: f64,
    /// Translation applied after scaling `(dx, dy)` in font units.
    pub translation: (f64, f64),
    /// Final glyph bounds in font units; serializes as `[x0,y0,x1,y1]`.
    #[serde(with = "rect_array")]
    pub final_bounds: Rect,
    /// Advance width in font units.
    pub advance_width: f64,
    /// Resolved left sidebearing.
    pub lsb: f64,
    /// Resolved right sidebearing.
    pub rsb: f64,
    /// True if the placed outline reaches outside the target vertical band.
    pub out_of_target: bool,
}

/// Ink bounding box in pixels (y-down), or `None` with no ink. Uses the
/// tracer's marching-squares cutoff (`iso = threshold + 0.5`) so the box
/// agrees with what gets traced, even when Otsu lands on 0. Deliberately
/// separate from the pre-threshold `preprocess::ink_bbox` /
/// `image_stats::ink_extent` scans.
pub fn ink_bounds(
    img: &GrayImage,
    threshold: u8,
    invert: bool,
) -> Option<Rect> {
    let (w, h) = img.dimensions();
    let iso = threshold as f64 + 0.5;
    let (mut x0, mut y0, mut x1, mut y1) = (w, h, 0u32, 0u32);
    let mut found = false;
    for y in 0..h {
        for x in 0..w {
            let v = img.get_pixel(x, y).0[0] as f64;
            let is_ink = if invert { v > iso } else { v < iso };
            if is_ink {
                x0 = x0.min(x);
                y0 = y0.min(y);
                x1 = x1.max(x);
                y1 = y1.max(y);
                found = true;
            }
        }
    }
    if !found {
        return None;
    }
    // +1 on the maxima so the box spans the full extent of the last ink pixel.
    Some(Rect::new(
        x0 as f64,
        y0 as f64,
        x1 as f64 + 1.0,
        y1 as f64 + 1.0,
    ))
}

/// Position a traced outline (already scaled to the target band height) into
/// font space per `opts`; `image_size`/`ink_px`/`scale` go in the report.
pub fn position(
    outline: &Outline,
    opts: &PlacementOptions,
    image_size: (u32, u32),
    ink_px: Rect,
    scale: f64,
) -> (PlacedGlyph, PlacementReport) {
    let Some(bounds) = outline.bounds() else {
        let (y_min_t, _) = opts.vertical.bounds();
        return (
            PlacedGlyph {
                outline: outline.clone(),
                advance_width: 0.0,
            },
            PlacementReport {
                image_size,
                ink_bounds_px: ink_px,
                scale,
                translation: (0.0, 0.0),
                final_bounds: Rect::new(0.0, y_min_t, 0.0, y_min_t),
                advance_width: 0.0,
                lsb: 0.0,
                rsb: 0.0,
                out_of_target: false,
            },
        );
    };

    let (min_x, min_y, max_x) = (bounds.x0, bounds.y0, bounds.x1);
    let (y_min_t, y_max_t) = opts.vertical.bounds();
    let g = opts.grid.max(0) as f64;
    let round = |v: f64| if g > 0.0 { (v / g).round() * g } else { v };

    // Vertical: ink bottom → band y_min (scale already sized ink to band).
    let dy = round(y_min_t - min_y);

    let ink_w = max_x - min_x;
    let (dx, advance) = match opts.sidebearings {
        Sidebearings::Explicit { lsb, rsb } => {
            let dx = round(lsb - min_x);
            (dx, (max_x + dx) + rsb)
        }
        Sidebearings::FixedAdvance { width, lsb } => {
            (round(lsb - min_x), width)
        }
        Sidebearings::Center { advance } => {
            (round((advance - ink_w) / 2.0 - min_x), advance)
        }
    };

    let placed = outline.translated(dx, dy);
    let final_bounds = placed.bounds().unwrap_or(Rect::ZERO);
    // Sidebearings as actually resolved after the (rounded) translate.
    let lsb = min_x + dx;
    let rsb = advance - (max_x + dx);
    let out_of_target =
        final_bounds.y0 < y_min_t - 0.5 || final_bounds.y1 > y_max_t + 0.5;

    (
        PlacedGlyph {
            outline: placed,
            advance_width: advance,
        },
        PlacementReport {
            image_size,
            ink_bounds_px: ink_px,
            scale,
            translation: (dx, dy),
            final_bounds,
            advance_width: advance,
            lsb,
            rsb,
            out_of_target,
        },
    )
}

// ── Metrics-based placement ─────────────────────────────────────────────────
// [`place`] takes the neutral em-space outline from [`crate::trace`] and
// applies font metrics: baseline to y=0 (assumes a full-em source render),
// leftmost ink to the LSB, and computes the advance.

/// A placed glyph: an [`Outline`] positioned in font space, with its advance.
#[derive(Debug, Clone, PartialEq)]
#[non_exhaustive]
pub struct PlacedGlyph {
    /// The positioned outline (baseline at y=0, leftmost ink at the LSB).
    pub outline: Outline,
    /// Advance width in font units.
    pub advance_width: f64,
}

impl PlacedGlyph {
    /// Assemble a placed outline directly (the struct is `non_exhaustive`).
    pub fn new(outline: Outline, advance_width: f64) -> Self {
        Self {
            outline,
            advance_width,
        }
    }
}

/// Position a traced outline into a font using its metrics: shift by
/// `(lsb - min_x, descender)` (baseline lands at y=0); advance is
/// `metrics.advance_width` if set, else rightmost ink plus RSB.
pub fn place(outline: &Outline, metrics: &FontMetrics) -> PlacedGlyph {
    // On-curve extents only (control points don't define sidebearings).
    let mut min_x = f64::MAX;
    for contour in &outline.contours {
        for p in &contour.points {
            if p.kind != PointKind::OffCurve {
                min_x = min_x.min(p.x);
            }
        }
    }
    if min_x == f64::MAX {
        return PlacedGlyph {
            outline: outline.clone(),
            advance_width: metrics.advance_width.unwrap_or(0.0),
        };
    }

    let dx = metrics.lsb - min_x;
    let dy = metrics.descender;
    let placed = translate(outline, dx, dy);

    let advance_width = metrics
        .advance_width
        .unwrap_or_else(|| advance_from_bounds(&placed, metrics.rsb));

    PlacedGlyph {
        outline: placed,
        advance_width,
    }
}

/// Advance width from the outline's tight bounds plus the right sidebearing.
fn advance_from_bounds(outline: &Outline, rsb: f64) -> f64 {
    let max_x = outline
        .to_bezpaths()
        .iter()
        .map(|p| p.bounding_box().x1)
        .fold(f64::MIN, f64::max);
    if max_x == f64::MIN { 0.0 } else { max_x + rsb }
}

/// Translate every point in the outline by `(dx, dy)`.
fn translate(outline: &Outline, dx: f64, dy: f64) -> Outline {
    let contours = outline
        .contours
        .iter()
        .map(|c| Contour {
            points: c
                .points
                .iter()
                .map(|p| OutlinePoint {
                    x: p.x + dx,
                    y: p.y + dy,
                    ..*p
                })
                .collect(),
        })
        .collect();
    Outline { contours }
}
