// Copyright 2026 the img2bez Authors
// SPDX-License-Identifier: Apache-2.0 OR MIT

//! Reference-free trace quality: the judge.
//!
//! Scores a trace from the outline and source image alone, combining
//! **reproduction** ([`reproduction_iou`], `render` feature: IoU against the
//! thresholded input) and **structure** ([`structure`]: kink smoothness, H/V
//! handle fraction, micro-segment penalty, point parsimony). Both halves
//! matter: a wobble-chasing trace reproduces pixels perfectly with 3x the
//! points a designer would use. Port of the eval harness's `score_wild.py`;
//! definitions and weights must match. Constants are font units, ~1000 em.

use serde::Serialize;

use crate::model::outline::{Outline, PointKind};

/// On-curve chord below this (font units) is a vestigial micro-segment.
const MICRO_LEN: f64 = 14.0;
/// Target on-curve points per 1000 font units of chordal perimeter
/// (designer-grade traces ~4.3; noisy over-traces 6.5+).
const TARGET_DENSITY: f64 = 5.0;
/// Handle offset within this many font units of an axis counts as H/V.
const HV_TOL: f64 = 0.5;
/// A join's tangent break is an accidental kink when too big for grid
/// rounding, too small for a designed corner; each costs KINK_PENALTY.
const KINK_TOL_DEG: f64 = 3.0;
const KINK_CORNER_DEG: f64 = 45.0;
const KINK_PENALTY: f64 = 0.15;
/// Render height (px) for the reproduction-IoU comparison.
#[cfg(feature = "render")]
const RASTER_H: u32 = 512;
/// Flattening samples per cubic segment when rasterizing.
#[cfg(feature = "render")]
const CURVE_SAMPLES: usize = 16;

/// Combined-score weights: reproduction vs structure, and within structure.
#[cfg(feature = "render")]
const W_REPRO: f64 = 0.5;
#[cfg(feature = "render")]
const W_STRUCT: f64 = 0.5;
const W_HV: f64 = 0.30;
const W_SMOOTH: f64 = 0.30;
const W_MICRO: f64 = 0.15;
const W_PARSIMONY: f64 = 0.25;

/// The judge's verdict on one trace. Serializes for reports and the trace log.
#[derive(Debug, Clone, PartialEq, Serialize)]
#[serde(rename_all = "camelCase")]
#[non_exhaustive]
pub struct Judgement {
    /// Reproduction IoU (`render` feature); `None` without a source image.
    pub repro_iou: Option<f64>,
    /// Fraction of curve handles leaving exactly horizontal or vertical.
    pub hv_frac: f64,
    /// Count of vestigial micro-segments (on-curve chords under 14 units).
    pub micro_segs: usize,
    /// Count of accidental kinks (tangent breaks in the 3..45 degree band).
    pub kinked_joins: usize,
    /// 1 minus the per-kink penalty, floored at 0 — the lumpiness score.
    pub smooth: f64,
    /// 1 minus the micro-segment fraction of on-curve points.
    pub micro_clean: f64,
    /// On-curve points per 1000 units of chordal perimeter.
    pub density: f64,
    /// Point-economy score: 1.0 at or under the target density.
    pub parsimony: f64,
    /// Weighted structure score (H/V + micro + parsimony).
    pub structure: f64,
    /// Combined score in [0, 1]; equals `structure` when repro is missing.
    pub wild: f64,
}

/// Judge a trace from its outline alone (no source image): the structure half
/// of the score. `wild` equals `structure` in the result.
pub fn structure(outline: &Outline) -> Judgement {
    let (mut hv_aligned, mut hv_total) = (0usize, 0usize);
    let mut oncurves = 0usize;
    let mut micro = 0usize;
    let mut kinks = 0usize;
    let mut perimeter = 0.0f64;

    for contour in &outline.contours {
        let pts = &contour.points;
        let n = pts.len();
        if n == 0 {
            continue;
        }
        // H/V handles: each cubic's incoming off-curve pair vs their anchors.
        for (i, p) in pts.iter().enumerate() {
            if p.kind != PointKind::Curve {
                continue;
            }
            let mut controls = Vec::new();
            let mut prev_on = None;
            for k in 1..n {
                let q = &pts[(i + n - k) % n];
                if q.kind == PointKind::OffCurve {
                    controls.push(q);
                } else {
                    prev_on = Some(q);
                    break;
                }
            }
            if prev_on.is_none() || controls.len() < 2 {
                continue;
            }
            let prev_on = prev_on.unwrap();
            // Walked backwards, so controls[last] is c1 (nearest prev_on).
            let c1 = controls[controls.len() - 1];
            let c2 = controls[controls.len() - 2];
            hv_total += 2;
            if is_hv(prev_on.x, prev_on.y, c1.x, c1.y) {
                hv_aligned += 1;
            }
            if is_hv(p.x, p.y, c2.x, c2.y) {
                hv_aligned += 1;
            }
        }
        // Accidental kinks: tangent break between incoming and outgoing
        // directions at every join.
        for (i, p) in pts.iter().enumerate() {
            if p.kind == PointKind::OffCurve {
                continue;
            }
            let prev = &pts[(i + n - 1) % n];
            let next = &pts[(i + 1) % n];
            let (ix, iy) = (p.x - prev.x, p.y - prev.y);
            let (ox, oy) = (next.x - p.x, next.y - p.y);
            let (li, lo) =
                ((ix * ix + iy * iy).sqrt(), (ox * ox + oy * oy).sqrt());
            if li < 1e-9 || lo < 1e-9 {
                continue;
            }
            let cos = ((ix * ox + iy * oy) / (li * lo)).clamp(-1.0, 1.0);
            let theta = cos.acos().to_degrees();
            if theta > KINK_TOL_DEG && theta < KINK_CORNER_DEG {
                kinks += 1;
            }
        }
        // Micro segments + chordal perimeter over on-curve points.
        let on: Vec<(f64, f64)> = pts
            .iter()
            .filter(|p| p.kind != PointKind::OffCurve)
            .map(|p| (p.x, p.y))
            .collect();
        oncurves += on.len();
        for i in 0..on.len() {
            let a = on[i];
            let b = on[(i + 1) % on.len()];
            let d = ((b.0 - a.0).powi(2) + (b.1 - a.1).powi(2)).sqrt();
            perimeter += d;
            if d < MICRO_LEN {
                micro += 1;
            }
        }
    }

    let hv_frac = if hv_total > 0 {
        hv_aligned as f64 / hv_total as f64
    } else {
        1.0
    };
    let density = if perimeter > 0.0 {
        oncurves as f64 / (perimeter / 1000.0)
    } else {
        0.0
    };
    let micro_clean = if oncurves > 0 {
        (1.0 - micro as f64 / oncurves as f64).max(0.0)
    } else {
        1.0
    };
    let parsimony = if density > 0.0 {
        (TARGET_DENSITY / density).min(1.0)
    } else {
        1.0
    };
    let smooth = (1.0 - kinks as f64 * KINK_PENALTY).max(0.0);
    let structure = W_HV * hv_frac
        + W_SMOOTH * smooth
        + W_MICRO * micro_clean
        + W_PARSIMONY * parsimony;

    Judgement {
        repro_iou: None,
        hv_frac,
        micro_segs: micro,
        kinked_joins: kinks,
        smooth,
        micro_clean,
        density,
        parsimony,
        structure,
        wild: structure,
    }
}

/// True when the a→b offset is within `HV_TOL` of an axis.
fn is_hv(ax: f64, ay: f64, bx: f64, by: f64) -> bool {
    (ax - bx).abs() <= HV_TOL || (ay - by).abs() <= HV_TOL
}

/// Judge a trace against its source image: structure plus reproduction IoU.
/// `threshold` is the trace's resolved ink cut; darker pixels are glyph.
#[cfg(feature = "render")]
pub fn judge(
    outline: &Outline,
    source: &image::GrayImage,
    threshold: u8,
) -> Judgement {
    let mut j = structure(outline);
    if let Some(iou) = reproduction_iou(outline, source, threshold) {
        j.repro_iou = Some(iou);
        j.wild = W_REPRO * iou + W_STRUCT * j.structure;
    }
    j
}

/// Judge a trace against its source image bytes: decodes and resolves the
/// ink threshold the same way [`crate::trace`] does, then runs [`judge`].
///
/// # Errors
///
/// [`crate::TraceError::ImageLoad`] if the bytes do not decode.
#[cfg(feature = "render")]
pub fn judge_source(
    outline: &Outline,
    image_bytes: &[u8],
    opts: &crate::TraceOptions,
) -> Result<Judgement, crate::TraceError> {
    let gray = crate::io::bitmap::load_luma_bytes(image_bytes)?;
    let threshold = crate::io::bitmap::resolve_threshold(&gray, opts);
    Ok(judge(outline, &gray, threshold))
}

/// Bbox-normalized IoU between the rasterized trace and the thresholded
/// source glyph. `None` when either mask is empty.
#[cfg(feature = "render")]
pub fn reproduction_iou(
    outline: &Outline,
    source: &image::GrayImage,
    threshold: u8,
) -> Option<f64> {
    let tmask = trace_mask(outline)?;
    let smask = source_mask(source, threshold)?;
    // Nearest-sample the trace mask onto the source frame and take IoU.
    let (sw, sh) = (smask.width, smask.height);
    let mut inter = 0u64;
    let mut union = 0u64;
    for y in 0..sh {
        for x in 0..sw {
            let s = smask.get(x, y);
            let tx = (x as f64 + 0.5) / sw as f64 * tmask.width as f64;
            let ty = (y as f64 + 0.5) / sh as f64 * tmask.height as f64;
            let t = tmask.get(
                (tx as u32).min(tmask.width - 1),
                (ty as u32).min(tmask.height - 1),
            );
            if s && t {
                inter += 1;
            }
            if s || t {
                union += 1;
            }
        }
    }
    (union > 0).then(|| inter as f64 / union as f64)
}

/// A 1-bit mask.
#[cfg(feature = "render")]
struct Mask {
    width: u32,
    height: u32,
    bits: Vec<bool>,
}

#[cfg(feature = "render")]
impl Mask {
    fn get(&self, x: u32, y: u32) -> bool {
        self.bits[(y * self.width + x) as usize]
    }
}

/// Rasterize the outline to a bbox-cropped 1-bit mask at `RASTER_H` height,
/// even-odd filled, y flipped to image orientation.
#[cfg(feature = "render")]
fn trace_mask(outline: &Outline) -> Option<Mask> {
    use tiny_skia::{FillRule, Paint, PathBuilder, Pixmap, Transform};

    // Flatten contours to polygons and find the union bbox.
    let mut polys: Vec<Vec<(f64, f64)>> = Vec::new();
    for contour in &outline.contours {
        if contour.points.len() < 3 {
            continue;
        }
        let poly = flatten_contour(contour);
        if poly.len() >= 3 {
            polys.push(poly);
        }
    }
    if polys.is_empty() {
        return None;
    }
    let (mut minx, mut miny, mut maxx, mut maxy) = (
        f64::INFINITY,
        f64::INFINITY,
        f64::NEG_INFINITY,
        f64::NEG_INFINITY,
    );
    for p in polys.iter().flatten() {
        minx = minx.min(p.0);
        miny = miny.min(p.1);
        maxx = maxx.max(p.0);
        maxy = maxy.max(p.1);
    }
    let (wu, hu) = (maxx - minx, maxy - miny);
    if wu <= 0.0 || hu <= 0.0 {
        return None;
    }
    let scale = RASTER_H as f64 / hu;
    let w = ((wu * scale).round() as u32).max(1);
    let h = RASTER_H;

    let mut pb = PathBuilder::new();
    for poly in &polys {
        let to_px = |&(x, y): &(f64, f64)| {
            (((x - minx) * scale) as f32, ((maxy - y) * scale) as f32)
        };
        let (x0, y0) = to_px(&poly[0]);
        pb.move_to(x0, y0);
        for p in &poly[1..] {
            let (x, y) = to_px(p);
            pb.line_to(x, y);
        }
        pb.close();
    }
    let path = pb.finish()?;
    let mut pixmap = Pixmap::new(w, h)?;
    let mut paint = Paint::default();
    paint.set_color_rgba8(255, 255, 255, 255);
    paint.anti_alias = false;
    pixmap.fill_path(
        &path,
        &paint,
        FillRule::EvenOdd,
        Transform::identity(),
        None,
    );

    let bits: Vec<bool> =
        pixmap.pixels().iter().map(|p| p.alpha() > 0).collect();
    Some(Mask {
        width: w,
        height: h,
        bits,
    })
}

/// Flatten one contour (UFO point order) to a y-up polygon; cubics sampled at
/// `CURVE_SAMPLES`.
#[cfg(feature = "render")]
fn flatten_contour(
    contour: &crate::model::outline::Contour,
) -> Vec<(f64, f64)> {
    let pts = &contour.points;
    let n = pts.len();
    let start = pts
        .iter()
        .position(|p| p.kind != PointKind::OffCurve)
        .unwrap_or(0);
    let seq: Vec<_> = (0..n).map(|k| &pts[(start + k) % n]).collect();
    let mut poly: Vec<(f64, f64)> = vec![(seq[0].x, seq[0].y)];
    let mut i = 1usize;
    while i <= seq.len() {
        let p = seq[i % seq.len()];
        if p.kind != PointKind::OffCurve {
            poly.push((p.x, p.y));
            i += 1;
        } else {
            let c1 = seq[i % seq.len()];
            let c2 = seq[(i + 1) % seq.len()];
            let end = seq[(i + 2) % seq.len()];
            let p0 = *poly.last().unwrap();
            for s in 1..=CURVE_SAMPLES {
                let t = s as f64 / CURVE_SAMPLES as f64;
                let u = 1.0 - t;
                let x = u * u * u * p0.0
                    + 3.0 * u * u * t * c1.x
                    + 3.0 * u * t * t * c2.x
                    + t * t * t * end.x;
                let y = u * u * u * p0.1
                    + 3.0 * u * u * t * c1.y
                    + 3.0 * u * t * t * c2.y
                    + t * t * t * end.y;
                poly.push((x, y));
            }
            i += 3;
        }
    }
    poly
}

/// Glyph mask from the source raster: pixels darker than the threshold,
/// cropped to their bounding box. `None` if the image has no ink.
#[cfg(feature = "render")]
fn source_mask(img: &image::GrayImage, threshold: u8) -> Option<Mask> {
    let (w, h) = img.dimensions();
    // Recover a usable cut from the histogram extremes when the resolved
    // threshold is degenerate (hard bilevel images can report 0).
    let t = if threshold == 0 || threshold == 255 {
        let (mut lo, mut hi) = (255u8, 0u8);
        for p in img.pixels() {
            lo = lo.min(p.0[0]);
            hi = hi.max(p.0[0]);
        }
        if hi > lo {
            ((lo as u16 + hi as u16) / 2) as u8
        } else {
            128
        }
    } else {
        threshold
    };
    let ink = |x: u32, y: u32| img.get_pixel(x, y).0[0] < t;
    let (mut x0, mut y0, mut x1, mut y1) = (w, h, 0u32, 0u32);
    let mut found = false;
    for y in 0..h {
        for x in 0..w {
            if ink(x, y) {
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
    let (mw, mh) = (x1 - x0 + 1, y1 - y0 + 1);
    let mut bits = Vec::with_capacity((mw * mh) as usize);
    for y in y0..=y1 {
        for x in x0..=x1 {
            bits.push(ink(x, y));
        }
    }
    Some(Mask {
        width: mw,
        height: mh,
        bits,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::model::outline::Outline;

    fn square(side: f64) -> Outline {
        let mut p = kurbo::BezPath::new();
        p.move_to((0.0, 0.0));
        p.line_to((side, 0.0));
        p.line_to((side, side));
        p.line_to((0.0, side));
        p.close_path();
        Outline::from_bezpaths(&[p])
    }

    #[test]
    fn all_line_square_scores_full_structure() {
        let j = structure(&square(400.0));
        assert_eq!(j.micro_segs, 0);
        assert!((j.hv_frac - 1.0).abs() < 1e-12);
        assert!((j.parsimony - 1.0).abs() < 1e-12);
        assert!((j.structure - 1.0).abs() < 1e-12);
        assert_eq!(j.wild, j.structure);
    }

    #[test]
    fn tiny_square_is_all_micro_segments() {
        // Every 10-unit side is under MICRO_LEN.
        let j = structure(&square(10.0));
        assert_eq!(j.micro_segs, 4);
        assert!(j.micro_clean.abs() < 1e-12);
        assert!(j.structure < 0.65);
    }

    #[cfg(feature = "render")]
    #[test]
    fn reproduction_iou_of_matching_square_is_high() {
        let img = image::GrayImage::from_fn(100, 100, |x, y| {
            let ink = (20..80).contains(&x) && (20..80).contains(&y);
            image::Luma([if ink { 0 } else { 255 }])
        });
        let iou = reproduction_iou(&square(600.0), &img, 128).unwrap();
        assert!(iou > 0.98, "iou {iou}");
    }
}
