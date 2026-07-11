// Copyright 2026 the img2bez Authors
// SPDX-License-Identifier: Apache-2.0 OR MIT

//! Raster-loss refinement: re-scores candidate outlines directly against
//! the source grayscale via the prefilter approximation (coverage is a
//! linear ramp in signed distance near the outline), golden-section
//! search on the band loss. `refine_contour` lists the passes; the
//! structural passes (1-4) only rebuild segments they beat on the raster
//! score, and 5-7 never change handle directions, move on-curve points,
//! or touch design joints.

use image::GrayImage;
use kurbo::{Point, Vec2};

use super::fit::{FittedContour, SplitKind};

mod flats;
mod lines;
mod merge;
mod polish;
mod verify;
mod welds;

use self::flats::junction_flats;
use self::lines::{collapse_corner_micro_lines, smooth_inflection_lines};
use self::merge::merge_pass;
use self::polish::{polish_pass, rebalance_pass, slide_pass};
use self::verify::verify_coverage;
use self::welds::weld_convex_tips;

/// Half-width (px) of the pixel band around the outline scored by the loss.
const BAND_RADIUS: f64 = 2.0;

/// Handle-length search range (fractions of the chord): above degenerate,
/// below loops, with margin for squarish-bowl tension (~0.9).
const MIN_HANDLE_FRAC: f64 = 0.02;
const MAX_HANDLE_FRAC: f64 = 1.1;

/// Golden-section iterations per line search (interval shrinks to ~4e-4
/// of the range) and coordinate-descent sweeps.
const GOLDEN_ITERS: usize = 16;
const DESCENT_SWEEPS: usize = 2;

/// Handle-balance regularization: the band loss basin is broad along an
/// arc, so the raster optimum can land on a lopsided split (~6:1) that
/// reads as a kink. The balanced optimum is adopted within REL x the
/// optimum (+ FLOOR, capped at CEIL); genuinely asymmetric ink costs far
/// more and is left alone. Bimodal in the spread, so all-or-nothing;
/// optima within MIN_SPREAD px of equal are untouched.
const BALANCE_SLACK_REL: f64 = 2.0;
const BALANCE_SLACK_FLOOR: f64 = 4e-3;
const BALANCE_SLACK_CEIL: f64 = 0.09;
const BALANCE_MIN_SPREAD: f64 = 1.0;

/// Max combined chord-projected handle reach as a fraction of the chord
/// (1.0 = control points cross); applied after balancing, both scaled
/// equally. Loose because tight bowls genuinely reach ~0.9.
const HANDLE_REACH_MAX: f64 = 0.9;

/// The source image as an ink-coverage field, sampled in the same y-up
/// pixel space the contours live in.
pub(crate) struct RasterTarget {
    /// Row-major (y-down) coverage in [0, 1]; 1 = full ink.
    cov: Vec<f32>,
    w: usize,
    h: usize,
    /// Pixels per font unit (image height / em height); font-unit size
    /// gates multiply by this so sources of any resolution agree.
    pub(crate) px_per_unit: f64,
}

impl RasterTarget {
    pub(crate) fn new(img: &GrayImage, invert: bool, px_per_unit: f64) -> Self {
        let (w, h) = img.dimensions();
        let (mut lo, mut hi) = (255u8, 0u8);
        for p in img.pixels() {
            lo = lo.min(p.0[0]);
            hi = hi.max(p.0[0]);
        }
        let range = ((hi as f32) - (lo as f32)).max(1.0);
        let cov = img
            .pixels()
            .map(|p| {
                // Dark = ink by default (matching the threshold convention).
                let c = ((hi as f32) - (p.0[0] as f32)) / range;
                if invert { 1.0 - c } else { c }
            })
            .collect();
        Self {
            cov,
            w: w as usize,
            h: h as usize,
            px_per_unit: px_per_unit.max(1e-6),
        }
    }

    /// Bilinear coverage at a y-up point (pixel centers at integer + 0.5).
    pub(super) fn coverage(&self, x: f64, y_up: f64) -> f64 {
        let u = x - 0.5;
        let v = (self.h as f64 - y_up) - 0.5;
        let x0 = u.floor();
        let y0 = v.floor();
        let fx = u - x0;
        let fy = v - y0;
        let at = |ix: f64, iy: f64| -> f64 {
            let ix = (ix.max(0.0) as usize).min(self.w - 1);
            let iy = (iy.max(0.0) as usize).min(self.h - 1);
            self.cov[iy * self.w + ix] as f64
        };
        let c00 = at(x0, y0);
        let c10 = at(x0 + 1.0, y0);
        let c01 = at(x0, y0 + 1.0);
        let c11 = at(x0 + 1.0, y0 + 1.0);
        c00 * (1.0 - fx) * (1.0 - fy)
            + c10 * fx * (1.0 - fy)
            + c01 * (1.0 - fx) * fy
            + c11 * fx * fy
    }
}

/// A pixel-center sample in the loss band.
struct BandPt {
    p: Point,
    /// Source coverage at this pixel.
    src: f64,
    /// Signed distance to the FIXED neighbor geometry; the loss takes
    /// whichever of neighbor/candidate is closer. ±infinity when no
    /// neighbor is nearby.
    fixed_sd: f64,
}

/// Refine a fitted contour against the source raster. See module docs.
pub(super) fn refine_contour(
    mut contour: FittedContour,
    rt: &RasterTarget,
) -> FittedContour {
    if contour.segs.len() < 2 {
        return contour;
    }
    let debug = std::env::var("IMG2BEZ_DEBUG_FIT").is_ok();
    let ink_left = match ink_is_left(rt, &contour) {
        Some(side) => side,
        None => return contour, // boundary band carries no usable gradient
    };

    // Joints modified by the structural rebuild passes; their flanking
    // segments are eligible for rescue-tier polish (a rebuild can leave a
    // wall at high loss that demonstrably improves when re-optimized).
    let mut touched: Vec<Point> = Vec::new();

    // Pass 1: rebuild junction valleys as tiny axis-aligned flats where
    // the raster prefers them.
    contour = junction_flats(contour, rt, ink_left, debug);

    // Pass 2: re-express inflection-bridging straight runs as one smooth
    // inflection point flanked by two curves.
    contour = smooth_inflection_lines(contour, rt, ink_left, debug);

    // Pass 3: collapse a vestigial micro-line at a sharp line-line corner
    // into the single true vertex.
    contour = collapse_corner_micro_lines(contour, rt, ink_left, debug);

    // Pass 4: weld a convex tip's doubled corners into one apex vertex.
    contour = weld_convex_tips(contour, rt, ink_left, debug);

    // Approximate "touched" as every Corner joint: rescue-worthy walls
    // end at hard corners; organic misfits (which rescue must not touch)
    // sit at smooth joints.
    for i in 0..contour.segs.len() {
        if contour.joint_kind[i] == SplitKind::Corner {
            touched.push(contour.segs[i][0]);
        }
    }

    // Pass 5: merge fitter-created joints where one cubic suffices.
    contour = merge_pass(contour, rt, ink_left, debug);

    // Pass 6: polish handle lengths of every remaining cubic.
    contour = polish_pass(contour, rt, ink_left, &touched, debug);

    // Pass 7: re-even lopsided handles against the raster.
    contour = rebalance_pass(contour, rt, ink_left, debug);

    // Pass 8: slide a line↔curve boundary back out of a line that
    // swallowed the curve's tail. After the handle passes: the tell (a
    // degenerate joint-side handle) only manifests once polish has run,
    // and the slide re-optimizes the curve itself.
    contour = slide_pass(contour, rt, ink_left, debug);

    // Pass 9: verify coverage; sustained buried/floating drift triggers a
    // re-fit with angular freedom on corner-anchored handle ends.
    contour = verify_coverage(contour, rt, ink_left, debug);

    contour
}

/// Intersection `a + s*da = b + u*db` with s, u > 0 (a true wedge tip
/// ahead of both rays), None when parallel or behind.
fn ray_intersection(a: Point, da: Vec2, b: Point, db: Vec2) -> Option<Point> {
    let det = da.x * (-db.y) - da.y * (-db.x);
    if det.abs() < 1e-9 {
        return None;
    }
    let r = b - a;
    let s = (r.x * (-db.y) - r.y * (-db.x)) / det;
    let u = (da.x * r.y - da.y * r.x) / det;
    if s <= 0.0 || u <= 0.0 {
        return None;
    }
    Some(a + da * s)
}

/// Which side of the travel direction the ink is on (sampled both sides
/// of the initial outline); None when indistinguishable.
fn ink_is_left(rt: &RasterTarget, contour: &FittedContour) -> Option<bool> {
    let mut left = 0.0;
    let mut right = 0.0;
    for seg in &contour.segs {
        for k in 1..4 {
            let t = k as f64 * 0.25;
            let p = cubic_pt(seg, t);
            let d = cubic_deriv(seg, t);
            if d.hypot() < 1e-9 {
                continue;
            }
            let nrm = Vec2::new(-d.y, d.x) * (1.0 / d.hypot());
            for off in [0.75, 1.5] {
                left += rt.coverage(p.x + nrm.x * off, p.y + nrm.y * off);
                right += rt.coverage(p.x - nrm.x * off, p.y - nrm.y * off);
            }
        }
    }
    if (left - right).abs() < 1e-6 {
        return None;
    }
    Some(left > right)
}

/// Sampled polyline for each segment (2 points for lines).
fn all_polylines(contour: &FittedContour) -> Vec<Vec<Point>> {
    contour
        .segs
        .iter()
        .zip(&contour.is_line)
        .map(|(seg, &line)| {
            if line {
                vec![seg[0], seg[3]]
            } else {
                sample_cubic(seg)
            }
        })
        .collect()
}

fn cubic_pt(seg: &[Point; 4], t: f64) -> Point {
    let mt = 1.0 - t;
    let b0 = mt * mt * mt;
    let b1 = 3.0 * mt * mt * t;
    let b2 = 3.0 * mt * t * t;
    let b3 = t * t * t;
    Point::new(
        b0 * seg[0].x + b1 * seg[1].x + b2 * seg[2].x + b3 * seg[3].x,
        b0 * seg[0].y + b1 * seg[1].y + b2 * seg[2].y + b3 * seg[3].y,
    )
}

fn cubic_deriv(seg: &[Point; 4], t: f64) -> Vec2 {
    let mt = 1.0 - t;
    (seg[1] - seg[0]) * (3.0 * mt * mt)
        + (seg[2] - seg[1]) * (6.0 * mt * t)
        + (seg[3] - seg[2]) * (3.0 * t * t)
}

fn sample_cubic(seg: &[Point; 4]) -> Vec<Point> {
    let chord = (seg[3] - seg[0]).hypot();
    let n = ((chord / 3.0) as usize).clamp(8, 64);
    (0..=n)
        .map(|k| cubic_pt(seg, k as f64 / n as f64))
        .collect()
}

/// Total signed turn (radians) along a polyline.
fn poly_turn(poly: &[Point]) -> f64 {
    let mut s = 0.0;
    for w in poly.windows(3) {
        let u = w[1] - w[0];
        let v = w[2] - w[1];
        s += u.cross(v).atan2(u.dot(v));
    }
    s
}

/// Signed distance (positive left of travel) from `p` to a polyline.
/// When the nearest feature is a vertex, the side-of-line test
/// misclassifies points in the vertex's outer cone, so the sign comes
/// from the angle-bisecting pseudo-normal of the adjacent segments.
fn signed_dist(p: Point, poly: &[Point]) -> f64 {
    let mut best_d2 = f64::INFINITY;
    let mut best_sign = 1.0;
    for (j, w) in poly.windows(2).enumerate() {
        let a = w[0];
        let ab = w[1] - a;
        let len2 = ab.dot(ab);
        if len2 < 1e-18 {
            continue;
        }
        let t = ((p - a).dot(ab) / len2).clamp(0.0, 1.0);
        let proj = a + ab * t;
        let d2 = (p - proj).hypot2();
        if d2 >= best_d2 {
            continue;
        }
        best_d2 = d2;
        best_sign = if t <= 1e-9 || t >= 1.0 - 1e-9 {
            let v = if t <= 1e-9 { j } else { j + 1 };
            let mut pseudo = Vec2::ZERO;
            // Walk outward past duplicate points (joint seams repeat the
            // shared vertex).
            let mut b = v;
            while b > 0 {
                b -= 1;
                let dir = poly[v] - poly[b];
                if dir.hypot() > 1e-9 {
                    let dir = dir.normalize();
                    pseudo += Vec2::new(-dir.y, dir.x);
                    break;
                }
            }
            let mut f = v;
            while f + 1 < poly.len() {
                f += 1;
                let dir = poly[f] - poly[v];
                if dir.hypot() > 1e-9 {
                    let dir = dir.normalize();
                    pseudo += Vec2::new(-dir.y, dir.x);
                    break;
                }
            }
            if (p - poly[v]).dot(pseudo) >= 0.0 {
                1.0
            } else {
                -1.0
            }
        } else if ab.cross(p - a) >= 0.0 {
            1.0
        } else {
            -1.0
        };
    }
    best_sign * best_d2.sqrt()
}

/// Pixel-center samples within BAND_RADIUS of `region`. Membership comes
/// from the INITIAL geometry and is reused across all candidate
/// evaluations, so losses are directly comparable.
fn collect_band(
    rt: &RasterTarget,
    region: &[Point],
    fixed: &[&Vec<Point>],
) -> Vec<BandPt> {
    let mut x0 = f64::INFINITY;
    let mut y0 = f64::INFINITY;
    let mut x1 = f64::NEG_INFINITY;
    let mut y1 = f64::NEG_INFINITY;
    for q in region {
        x0 = x0.min(q.x);
        y0 = y0.min(q.y);
        x1 = x1.max(q.x);
        y1 = y1.max(q.y);
    }
    let ix0 = ((x0 - BAND_RADIUS).floor() as i64).max(0);
    let iy0 = ((y0 - BAND_RADIUS).floor() as i64).max(0);
    let ix1 = ((x1 + BAND_RADIUS).ceil() as i64).min(rt.w as i64 - 1);
    let iy1 = ((y1 + BAND_RADIUS).ceil() as i64).min(rt.h as i64 - 1);
    if ix1 < ix0 || iy1 < iy0 {
        return Vec::new();
    }
    let bw = (ix1 - ix0 + 1) as usize;
    let bh = (iy1 - iy0 + 1) as usize;

    // Cheap prefilter: stamp squares around the region points, then test
    // only stamped pixels precisely. Avoids a full bbox × polyline scan.
    let stamp_r = BAND_RADIUS.ceil() as i64 + 2;
    let mut mask = vec![false; bw * bh];
    for q in region {
        let cx = q.x.floor() as i64;
        let cy = q.y.floor() as i64;
        for gy in (cy - stamp_r).max(iy0)..=(cy + stamp_r).min(iy1) {
            for gx in (cx - stamp_r).max(ix0)..=(cx + stamp_r).min(ix1) {
                mask[(gy - iy0) as usize * bw + (gx - ix0) as usize] = true;
            }
        }
    }

    let mut band = Vec::new();
    for gy in iy0..=iy1 {
        for gx in ix0..=ix1 {
            if !mask[(gy - iy0) as usize * bw + (gx - ix0) as usize] {
                continue;
            }
            let p = Point::new(gx as f64 + 0.5, gy as f64 + 0.5);
            let sd = signed_dist(p, region);
            if sd.abs() > BAND_RADIUS {
                continue;
            }
            let mut fixed_sd = f64::INFINITY;
            for f in fixed {
                let s = signed_dist(p, f);
                if s.abs() < fixed_sd.abs() {
                    fixed_sd = s;
                }
            }
            band.push(BandPt {
                p,
                src: rt.coverage(p.x, p.y),
                fixed_sd,
            });
        }
    }
    band
}

/// Mean squared coverage error of a candidate polyline over the band,
/// with coverage predicted by the linear-ramp (prefilter) approximation.
fn band_loss(band: &[BandPt], candidate: &[Point], ink_left: bool) -> f64 {
    if band.is_empty() {
        return 0.0;
    }
    let mut s = 0.0;
    for bp in band {
        let sd_c = signed_dist(bp.p, candidate);
        let sd = if bp.fixed_sd.abs() < sd_c.abs() {
            bp.fixed_sd
        } else {
            sd_c
        };
        let toward_ink = if ink_left { sd } else { -sd };
        let pred = (0.5 + toward_ink).clamp(0.0, 1.0);
        let e = pred - bp.src;
        s += e * e;
    }
    s / band.len() as f64
}

/// Band-loss-minimizing handle lengths for a cubic with fixed endpoints
/// and handle directions, by golden-section coordinate descent.
/// `prefix`/`suffix` are fixed polylines evaluated as part of the
/// candidate; pass empty slices when the cubic stands alone.
#[allow(clippy::too_many_arguments)] // geometric context, not config
fn optimize_handles(
    a: Point,
    u0: Vec2,
    u1: Vec2,
    b: Point,
    init: (f64, f64),
    band: &[BandPt],
    ink_left: bool,
    prefix: &[Point],
    suffix: &[Point],
) -> ([Point; 4], f64) {
    let chord = (b - a).hypot();
    let lo = chord * MIN_HANDLE_FRAC;
    let hi = chord * MAX_HANDLE_FRAC;
    // Magic triangle: neither handle may reach past the
    // tangent-elongation intersection, where one exists.
    let (hi_a, hi_b) = match crate::model::geom::handle_triangle(a, u0, b, u1) {
        Some((t, s)) => (hi.min(t).max(lo), hi.min(s).max(lo)),
        None => (hi, hi),
    };
    let eval = |alpha: f64, beta: f64| -> f64 {
        let seg = [a, a + u0 * alpha, b + u1 * beta, b];
        let pts = sample_cubic(&seg);
        if prefix.is_empty() && suffix.is_empty() {
            return band_loss(band, &pts, ink_left);
        }
        // Duplicate seam points are harmless: signed_dist skips
        // zero-length windows.
        let mut cand =
            Vec::with_capacity(prefix.len() + pts.len() + suffix.len());
        cand.extend_from_slice(prefix);
        cand.extend_from_slice(&pts);
        cand.extend_from_slice(suffix);
        band_loss(band, &cand, ink_left)
    };
    let mut alpha = init.0.clamp(lo, hi_a);
    let mut beta = init.1.clamp(lo, hi_b);
    for _ in 0..DESCENT_SWEEPS {
        // On arcs the loss valley runs along alpha + beta ≈ const, where
        // axis-aligned descent stalls — search mean/difference first.
        let d = (alpha - beta) * 0.5;
        let m = golden_min(
            |m| eval((m + d).clamp(lo, hi_a), (m - d).clamp(lo, hi_b)),
            lo,
            hi,
        );
        alpha = (m + d).clamp(lo, hi_a);
        beta = (m - d).clamp(lo, hi_b);
        let m = (alpha + beta) * 0.5;
        let span = hi - lo;
        let d = golden_min(
            |d| eval((m + d).clamp(lo, hi_a), (m - d).clamp(lo, hi_b)),
            -span,
            span,
        );
        beta = (m - d).clamp(lo, hi_b);
        alpha = golden_min(|x| eval(x, beta), lo, hi_a);
        beta = golden_min(|x| eval(alpha, x), lo, hi_b);
    }

    // Rebalance toward span-proportional handles (the kappa rule): take
    // the best split on the proportional ray if its loss stays within
    // slack of the raster optimum (see BALANCE_SLACK_*).
    let l_opt = eval(alpha, beta);
    let (w0, w1) = match crate::model::geom::handle_spans(a, u0, b, u1) {
        Some((s0, s1)) => {
            let m = s0.max(s1);
            (s0 / m, s1 / m)
        }
        None => (1.0, 1.0),
    };
    if (alpha / w0 - beta / w1).abs() > BALANCE_MIN_SPREAD {
        let target = (l_opt * BALANCE_SLACK_REL + BALANCE_SLACK_FLOOR)
            .min(BALANCE_SLACK_CEIL);
        let ray = |k: f64| -> (f64, f64) {
            ((k * w0).clamp(lo, hi_a), (k * w1).clamp(lo, hi_b))
        };
        let k = golden_min(
            |k| {
                let (x, y) = ray(k);
                eval(x, y)
            },
            lo,
            (hi_a / w0).min(hi_b / w1),
        );
        let (x, y) = ray(k);
        if eval(x, y) <= target {
            alpha = x;
            beta = y;
        }
    }

    // If the combined chord-projected reach exceeds the limit, scale both
    // handles down, preserving the balance ratio.
    if chord > 1e-9 {
        let e = (b - a) / chord;
        let reach = alpha * u0.dot(e) - beta * u1.dot(e);
        let limit = chord * HANDLE_REACH_MAX;
        if reach > limit {
            let s = limit / reach;
            alpha *= s;
            beta *= s;
        }
    }

    // Return the true raster optimum loss (not the rebalanced loss) so merge
    // and polish accept/reject decisions compare fits, not handle hygiene.
    let seg = [a, a + u0 * alpha, b + u1 * beta, b];
    (seg, l_opt)
}

/// Golden-section search for the minimum of `f` on [lo, hi].
fn golden_min(mut f: impl FnMut(f64) -> f64, mut lo: f64, mut hi: f64) -> f64 {
    const INV_PHI: f64 = 0.618_033_988_749_894_8;
    let mut x1 = hi - (hi - lo) * INV_PHI;
    let mut x2 = lo + (hi - lo) * INV_PHI;
    let mut f1 = f(x1);
    let mut f2 = f(x2);
    for _ in 0..GOLDEN_ITERS {
        if f1 <= f2 {
            hi = x2;
            x2 = x1;
            f2 = f1;
            x1 = hi - (hi - lo) * INV_PHI;
            f1 = f(x1);
        } else {
            lo = x1;
            x1 = x2;
            f1 = f2;
            x2 = lo + (hi - lo) * INV_PHI;
            f2 = f(x2);
        }
    }
    if f1 <= f2 { x1 } else { x2 }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::model::geom::line_seg;
    use image::Luma;

    fn disk_image(w: u32, h: u32, cx: f64, cy: f64, r: f64) -> GrayImage {
        GrayImage::from_fn(w, h, |x, y| {
            let d = ((x as f64 + 0.5 - cx).powi(2)
                + (y as f64 + 0.5 - cy).powi(2))
            .sqrt();
            let v = ((d - r + 0.5).clamp(0.0, 1.0) * 255.0) as u8;
            Luma([v])
        })
    }

    #[test]
    fn coverage_matches_disk_interior_and_exterior() {
        let img = disk_image(64, 64, 32.0, 32.0, 20.0);
        let rt = RasterTarget::new(&img, false, 1.0);
        // y-up center of the disk.
        assert!(rt.coverage(32.0, 32.0) > 0.99, "disk center should be ink");
        assert!(rt.coverage(2.0, 2.0) < 0.01, "far corner is background");
        // On the boundary the AA ramp should give ~half coverage.
        let edge = rt.coverage(32.0 + 20.0, 32.0);
        assert!(
            (0.25..=0.75).contains(&edge),
            "boundary coverage {edge} should be mid-ramp"
        );
    }

    #[test]
    fn signed_dist_sign_follows_travel_direction() {
        let poly = vec![Point::new(0.0, 0.0), Point::new(10.0, 0.0)];
        assert!(signed_dist(Point::new(5.0, 1.0), &poly) > 0.0);
        assert!(signed_dist(Point::new(5.0, -1.0), &poly) < 0.0);
        assert!((signed_dist(Point::new(5.0, 2.0), &poly) - 2.0).abs() < 1e-9);
    }

    /// An inked block whose top edge dips into a notch, box-filtered to
    /// mimic renderer AA. `flat_half` is the notch tip's half-width:
    /// 0 = sharp V tip, > 0 = flat-bottomed.
    fn notch_image(w: u32, h: u32, flat_half: f64, slope: f64) -> GrayImage {
        let g =
            |x: f64| 20.0 + slope * (0.0_f64).max((x - 32.0).abs() - flat_half);
        GrayImage::from_fn(w, h, |px, py| {
            let mut hits = 0;
            for sx in 0..8 {
                for sy in 0..8 {
                    let x = px as f64 - 0.5 + (sx as f64 + 0.5) / 4.0;
                    let y_up = (h - py) as f64 - 1.5 + (sy as f64 + 0.5) / 4.0;
                    if (4.0..60.0).contains(&x)
                        && y_up > 4.0
                        && y_up < g(x).min(88.0)
                    {
                        hits += 1;
                    }
                }
            }
            Luma([255 - (hits * 255 / 64) as u8])
        })
    }

    /// A closed all-lines contour (CCW in y-up, ink on the left) for the
    /// notched block, with a single vertex at `tip`.
    fn notch_contour(xr: f64, xl: f64, tip: Point) -> FittedContour {
        let pts = [
            Point::new(4.0, 4.0),
            Point::new(60.0, 4.0),
            Point::new(60.0, 88.0),
            Point::new(xr, 88.0),
            tip,
            Point::new(xl, 88.0),
            Point::new(4.0, 88.0),
        ];
        let n = pts.len();
        FittedContour {
            segs: (0..n).map(|j| line_seg(pts[j], pts[(j + 1) % n])).collect(),
            is_line: vec![true; n],
            joint_kind: vec![SplitKind::Corner; n],
        }
    }

    #[test]
    fn junction_flat_recovers_flat_bottomed_notch() {
        // True tip: flat from (29, 20) to (35, 20); walls slope 3. The
        // trace's vertex sits 1 px shallow of the flat.
        let img = notch_image(64, 96, 3.0, 3.0);
        let rt = RasterTarget::new(&img, false, 1.0);
        let c = notch_contour(57.7, 6.3, Point::new(32.0, 21.0));
        let ink_left = ink_is_left(&rt, &c).unwrap();
        let out = junction_flats(c, &rt, ink_left, false);
        assert_eq!(out.segs.len(), 8, "the vertex should become a flat");
        let flat = (0..8)
            .find(|&j| {
                let v = out.segs[j][3] - out.segs[j][0];
                out.is_line[j]
                    && v.hypot() < 10.0
                    && v.y.abs() < 1e-9
                    && out.segs[j][0].y < 50.0
            })
            .expect("an axis-aligned tiny flat should exist near the tip");
        let a = out.segs[flat][0];
        let b = out.segs[flat][3];
        assert!(
            (a.y - 20.0).abs() < 1.5,
            "flat level {:.2} should be near the true tip 20",
            a.y
        );
        let width = (b - a).hypot();
        assert!(
            (3.5..=9.0).contains(&width),
            "flat width {width:.2} should be near the true 6"
        );
    }

    #[test]
    fn junction_flat_leaves_sharp_notch_alone() {
        // Same notch but with a genuinely sharp tip, traced accurately.
        let img = notch_image(64, 96, 0.0, 3.0);
        let rt = RasterTarget::new(&img, false, 1.0);
        let c = notch_contour(54.7, 9.3, Point::new(32.0, 20.3));
        let ink_left = ink_is_left(&rt, &c).unwrap();
        let out = junction_flats(c, &rt, ink_left, false);
        assert_eq!(out.segs.len(), 7, "a sharp tip must stay a single vertex");
    }

    #[test]
    fn optimize_recovers_circle_handle_length() {
        // Quarter arc of a r=20 disk: the optimal cubic handle length is
        // ~0.5523 * r; the optimizer should land near it from a poor seed.
        let img = disk_image(64, 64, 32.0, 32.0, 20.0);
        let rt = RasterTarget::new(&img, false, 1.0);
        // CCW quarter in y-up coords: ink on the left of travel.
        let a = Point::new(52.0, 32.0);
        let b = Point::new(32.0, 52.0);
        let u0 = Vec2::new(0.0, 1.0);
        let u1 = Vec2::new(1.0, 0.0);
        let k = 0.5523 * 20.0;
        let true_seg = [a, a + u0 * k, b + u1 * k, b];
        let band = collect_band(&rt, &sample_cubic(&true_seg), &[]);
        assert!(band.len() > 50, "band should cover the arc");
        let (seg, loss) =
            optimize_handles(a, u0, u1, b, (3.0, 3.0), &band, true, &[], &[]);
        let alpha = (seg[1] - seg[0]).hypot();
        let beta = (seg[2] - seg[3]).hypot();
        assert!(
            (alpha - k).abs() < 1.2 && (beta - k).abs() < 1.2,
            "handles ({alpha:.2}, {beta:.2}) should approach {k:.2}, loss {loss:.5}"
        );
    }

    #[test]
    fn optimize_balances_handles_when_raster_permits() {
        // On a symmetric arc lopsided splits fit nearly as well as the
        // balanced one; rebalancing should still return near-equal handles
        // from a badly lopsided seed.
        let img = disk_image(64, 64, 32.0, 32.0, 20.0);
        let rt = RasterTarget::new(&img, false, 1.0);
        let a = Point::new(52.0, 32.0);
        let b = Point::new(32.0, 52.0);
        let u0 = Vec2::new(0.0, 1.0);
        let u1 = Vec2::new(1.0, 0.0);
        let k = 0.5523 * 20.0;
        let true_seg = [a, a + u0 * k, b + u1 * k, b];
        let band = collect_band(&rt, &sample_cubic(&true_seg), &[]);
        // Wildly unequal seed: a near-zero handle against a near-maximal one.
        let (seg, _) =
            optimize_handles(a, u0, u1, b, (0.5, 25.0), &band, true, &[], &[]);
        let alpha = (seg[1] - seg[0]).hypot();
        let beta = (seg[2] - seg[3]).hypot();
        let ratio = alpha.max(beta) / alpha.min(beta).max(1e-6);
        assert!(
            ratio < 1.35,
            "handles ({alpha:.2}, {beta:.2}) should be near-balanced, ratio {ratio:.2}"
        );
    }
}
