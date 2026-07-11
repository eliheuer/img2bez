// Copyright 2026 the img2bez Authors
// SPDX-License-Identifier: Apache-2.0 OR MIT

//! Cheap, deterministic, no-reference image statistics for input-adaptive
//! settings selection. [`ImageStats::classify`] picks a source-class profile
//! from them — today that drives the automatic pre-blur for soft scans.

use image::GrayImage;
use serde::Serialize;

use crate::model::config::Profile;

/// No-reference statistics of an input image.
#[derive(Debug, Clone, Copy, PartialEq, Serialize)]
#[serde(rename_all = "camelCase")]
#[non_exhaustive]
pub struct ImageStats {
    /// The glyph's larger ink extent, in pixels — the effective resolution.
    pub extent_px: u32,
    /// Edge sharpness: variance of the Laplacian (higher = sharper edges).
    /// Note this is inflated by noise; pair it with `noise`.
    pub sharpness: f64,
    /// Estimated Gaussian noise sigma (Immerkær estimator, 0–255 scale).
    pub noise: f64,
    /// Fraction of pixels at the extremes (≤16 or ≥239): ~1.0 means bilevel /
    /// no anti-aliasing, low means a smooth grayscale ramp at the edges.
    pub bilevelness: f64,
}

impl ImageStats {
    /// Classify the input into the trace [`Profile`] whose preset fits it.
    ///
    /// Soft/photographic needs low sharpness AND low bilevelness — a crisp
    /// low-res render also reads low-bilevel but stays sharp, and must not
    /// count as soft. Everything else falls through to [`Profile::Wild`].
    pub fn classify(&self) -> Profile {
        const SOFT_SHARPNESS_MAX: f64 = 80.0;
        const SOFT_BILEVEL_MAX: f64 = 0.30;
        if self.sharpness < SOFT_SHARPNESS_MAX
            && self.bilevelness < SOFT_BILEVEL_MAX
        {
            Profile::Photo
        } else {
            Profile::Wild
        }
    }
}

/// Measure the no-reference statistics of `img`.
pub fn measure(img: &GrayImage) -> ImageStats {
    ImageStats {
        extent_px: ink_extent(img),
        sharpness: laplacian_variance(img),
        noise: immerkaer_noise(img),
        bilevelness: bilevelness(img),
    }
}

/// Larger side of the ink bounding box (corner-sampled background; ink
/// differs by >40 levels). Deliberately separate from `preprocess::ink_bbox`
/// and the threshold-based `placement::ink_bounds`.
fn ink_extent(img: &GrayImage) -> u32 {
    let (w, h) = img.dimensions();
    if w == 0 || h == 0 {
        return 0;
    }
    let at = |x: u32, y: u32| img.get_pixel(x, y).0[0] as i32;
    let bg = (at(0, 0) + at(w - 1, 0) + at(0, h - 1) + at(w - 1, h - 1)) / 4;
    let (mut x0, mut y0, mut x1, mut y1) = (w, h, 0u32, 0u32);
    let mut found = false;
    for y in 0..h {
        for x in 0..w {
            if (at(x, y) - bg).abs() > 40 {
                x0 = x0.min(x);
                y0 = y0.min(y);
                x1 = x1.max(x);
                y1 = y1.max(y);
                found = true;
            }
        }
    }
    if !found {
        return 0;
    }
    (x1 - x0).max(y1 - y0) + 1
}

/// Variance of the 4-neighbor Laplacian — the classic no-reference
/// sharpness measure.
fn laplacian_variance(img: &GrayImage) -> f64 {
    let (w, h) = img.dimensions();
    if w < 3 || h < 3 {
        return 0.0;
    }
    let at = |x: u32, y: u32| img.get_pixel(x, y).0[0] as f64;
    let mut sum = 0.0;
    let mut sum_sq = 0.0;
    let mut n = 0.0;
    for y in 1..h - 1 {
        for x in 1..w - 1 {
            let l = 4.0 * at(x, y)
                - at(x - 1, y)
                - at(x + 1, y)
                - at(x, y - 1)
                - at(x, y + 1);
            sum += l;
            sum_sq += l * l;
            n += 1.0;
        }
    }
    if n == 0.0 {
        return 0.0;
    }
    (sum_sq / n) - (sum / n).powi(2)
}

/// Gaussian-noise sigma estimate (Immerkær, 1996): a mask that cancels
/// smooth/linear gradients but passes noise; robust to real structure.
fn immerkaer_noise(img: &GrayImage) -> f64 {
    let (w, h) = img.dimensions();
    if w < 3 || h < 3 {
        return 0.0;
    }
    let at = |x: u32, y: u32| img.get_pixel(x, y).0[0] as f64;
    let mut sum_abs = 0.0;
    let mut n = 0.0;
    for y in 1..h - 1 {
        for x in 1..w - 1 {
            // [ 1 -2  1 ; -2  4 -2 ;  1 -2  1 ]
            let r = at(x - 1, y - 1) - 2.0 * at(x, y - 1) + at(x + 1, y - 1)
                - 2.0 * at(x - 1, y)
                + 4.0 * at(x, y)
                - 2.0 * at(x + 1, y)
                + at(x - 1, y + 1)
                - 2.0 * at(x, y + 1)
                + at(x + 1, y + 1);
            sum_abs += r.abs();
            n += 1.0;
        }
    }
    if n == 0.0 {
        return 0.0;
    }
    // sigma = sqrt(pi/2) / 6 * mean(|response|)
    (std::f64::consts::PI / 2.0).sqrt() / 6.0 * (sum_abs / n)
}

/// Fraction of pixels at the tonal extremes (≤16 or ≥239). High = the image is
/// (near-)bilevel with no anti-aliasing; low = soft grayscale edges.
fn bilevelness(img: &GrayImage) -> f64 {
    let total = (img.width() as f64) * (img.height() as f64);
    if total == 0.0 {
        return 0.0;
    }
    let extreme = img
        .pixels()
        .filter(|p| p.0[0] <= 16 || p.0[0] >= 239)
        .count();
    extreme as f64 / total
}

#[cfg(test)]
mod tests {
    use super::*;
    use image::Luma;

    /// A vertical step edge at column `edge`, optionally blurred by averaging.
    fn step(w: u32, h: u32, edge: u32, blur: bool) -> GrayImage {
        GrayImage::from_fn(w, h, |x, _| {
            if blur {
                // 3-wide ramp around the edge.
                let v = if x + 1 < edge {
                    0
                } else if x > edge + 1 {
                    255
                } else {
                    128
                };
                Luma([v])
            } else {
                Luma([if x < edge { 0 } else { 255 }])
            }
        })
    }

    #[test]
    fn sharper_edge_has_higher_laplacian_variance() {
        let sharp = laplacian_variance(&step(64, 64, 32, false));
        let blurred = laplacian_variance(&step(64, 64, 32, true));
        assert!(sharp > blurred, "sharp {sharp} vs blurred {blurred}");
    }

    #[test]
    fn bilevel_image_reads_as_bilevel() {
        let bw = step(40, 40, 20, false); // pure 0/255
        assert!(bilevelness(&bw) > 0.99, "{}", bilevelness(&bw));
        // A mid-gray field is not bilevel.
        let gray = GrayImage::from_pixel(40, 40, Luma([128]));
        assert!(bilevelness(&gray) < 0.01);
    }

    #[test]
    fn noise_estimate_rises_with_noise() {
        // Flat field: ~0 noise.
        let flat = GrayImage::from_pixel(48, 48, Luma([128]));
        let clean = immerkaer_noise(&flat);
        // Add a deterministic checkerboard "noise".
        let noisy = GrayImage::from_fn(48, 48, |x, y| {
            Luma([if (x + y) % 2 == 0 { 118 } else { 138 }])
        });
        let est = immerkaer_noise(&noisy);
        assert!(clean < 1.0, "flat noise {clean}");
        assert!(est > clean, "noisy {est} vs clean {clean}");
    }

    #[test]
    fn ink_extent_measures_the_mark() {
        // 100x100 white with a full-width dark bar: extent is the larger side.
        let img = GrayImage::from_fn(100, 100, |_, y| {
            Luma([if (30..70).contains(&y) { 0 } else { 255 }])
        });
        assert!(ink_extent(&img) >= 99);
    }
}
