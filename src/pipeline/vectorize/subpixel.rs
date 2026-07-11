// Copyright 2026 the img2bez Authors
// SPDX-License-Identifier: Apache-2.0 OR MIT

//! Sub-pixel boundary extraction: marching squares at the threshold
//! iso-level with linear interpolation along grid edges. Anti-aliased
//! renders carry ~0.1px of boundary position that binary thresholding
//! destroys; keeping it makes downstream classification crisp.

use image::GrayImage;

/// A closed sub-pixel contour in y-up pixel coordinates ((0, 0) =
/// bottom-left); the first point is not repeated at the end.
#[derive(Debug, Clone)]
pub struct SubpixelContour {
    pub points: Vec<(f64, f64)>,
}

impl SubpixelContour {
    /// Twice the signed area (shoelace). Positive = CCW in y-up coords.
    pub fn signed_area2(&self) -> f64 {
        let n = self.points.len();
        let mut sum = 0.0;
        for i in 0..n {
            let (x0, y0) = self.points[i];
            let (x1, y1) = self.points[(i + 1) % n];
            sum += x0 * y1 - x1 * y0;
        }
        sum
    }

    /// Axis-aligned bounding box as `(min_x, min_y, max_x, max_y)`.
    pub fn bbox(&self) -> (f64, f64, f64, f64) {
        let mut x0 = f64::INFINITY;
        let mut y0 = f64::INFINITY;
        let mut x1 = f64::NEG_INFINITY;
        let mut y1 = f64::NEG_INFINITY;
        for &(x, y) in &self.points {
            x0 = x0.min(x);
            y0 = y0.min(y);
            x1 = x1.max(x);
            y1 = y1.max(y);
        }
        (x0, y0, x1, y1)
    }
}

/// Identify a crossing by the grid edge it lies on. Grid points are pixel
/// centers with a 1-sample padded background border, shifted +1 so
/// indices are non-negative; `horizontal` edges run (x,y)->(x+1,y),
/// vertical (x,y)->(x,y+1).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
struct EdgeKey {
    x: i32,
    y: i32,
    horizontal: bool,
}

/// Extract closed iso-contours of the glyph boundary. `threshold` is the
/// iso-level (luma <= threshold is ink); `invert` flips ink/background.
/// Contours under 8 points or |area| < `min_area_px` are noise, dropped.
pub fn extract_iso_contours(
    img: &GrayImage,
    threshold: u8,
    invert: bool,
    min_area_px: f64,
) -> Vec<SubpixelContour> {
    let (w, h) = img.dimensions();
    let w = w as i32;
    let h = h as i32;
    let iso = threshold as f64 + 0.5;

    // Signed field: positive inside ink. Padded border is background.
    let field = |x: i32, y: i32| -> f64 {
        if x < 0 || x >= w || y < 0 || y >= h {
            return if invert { 0.0 - iso } else { iso - 255.0 };
        }
        let luma = img.get_pixel(x as u32, y as u32).0[0] as f64;
        if invert { luma - iso } else { iso - luma }
    };

    // Sub-pixel crossing point on an edge, in image coordinates (y-down,
    // pixel centers at integer + 0.5).
    let crossing = |key: EdgeKey| -> (f64, f64) {
        let ax = key.x - 1;
        let ay = key.y - 1;
        let (bx, by) = if key.horizontal {
            (ax + 1, ay)
        } else {
            (ax, ay + 1)
        };
        let fa = field(ax, ay);
        let fb = field(bx, by);
        let s = if (fa - fb).abs() < 1e-12 {
            0.5
        } else {
            (fa / (fa - fb)).clamp(0.001, 0.999)
        };
        (
            ax as f64 + 0.5 + s * (bx - ax) as f64,
            ay as f64 + 0.5 + s * (by - ay) as f64,
        )
    };

    // March all cells (including the padded ring). Each crossing edge is
    // shared by exactly two cells, so every edge key accumulates exactly
    // two neighbors, forming closed loops.
    let mut links: std::collections::HashMap<EdgeKey, Vec<EdgeKey>> =
        std::collections::HashMap::new();
    let mut add_segment = |a: EdgeKey, b: EdgeKey| {
        links.entry(a).or_default().push(b);
        links.entry(b).or_default().push(a);
    };

    for cy in -1..h {
        for cx in -1..w {
            // Cell corners (grid samples): tl, tr, bl, br in image space.
            let f_tl = field(cx, cy);
            let f_tr = field(cx + 1, cy);
            let f_bl = field(cx, cy + 1);
            let f_br = field(cx + 1, cy + 1);
            let code = ((f_tl > 0.0) as u8)
                | (((f_tr > 0.0) as u8) << 1)
                | (((f_bl > 0.0) as u8) << 2)
                | (((f_br > 0.0) as u8) << 3);
            if code == 0 || code == 15 {
                continue;
            }
            // Edge keys in padded (+1) index space.
            let top = EdgeKey {
                x: cx + 1,
                y: cy + 1,
                horizontal: true,
            };
            let bottom = EdgeKey {
                x: cx + 1,
                y: cy + 2,
                horizontal: true,
            };
            let left = EdgeKey {
                x: cx + 1,
                y: cy + 1,
                horizontal: false,
            };
            let right = EdgeKey {
                x: cx + 2,
                y: cy + 1,
                horizontal: false,
            };
            match code {
                1 | 14 => add_segment(top, left),
                2 | 13 => add_segment(top, right),
                3 | 12 => add_segment(left, right),
                4 | 11 => add_segment(left, bottom),
                8 | 7 => add_segment(right, bottom),
                5 | 10 => add_segment(top, bottom),
                6 | 9 => {
                    // Ambiguous saddle: an inside center connects the
                    // inside corners, so the boundary isolates the
                    // outside pair (and vice versa).
                    let center = (f_tl + f_tr + f_bl + f_br) * 0.25;
                    let connected = center > 0.0;
                    let isolate_tl_br = (code == 6) == connected;
                    if isolate_tl_br {
                        add_segment(top, left);
                        add_segment(right, bottom);
                    } else {
                        add_segment(top, right);
                        add_segment(left, bottom);
                    }
                }
                _ => unreachable!(),
            }
        }
    }

    // Chain edges into closed loops. Keys are sorted so discovery and
    // starting points are deterministic — HashMap order would shift the
    // polyline origin per run and flip marginal classifications.
    let mut visited: std::collections::HashSet<EdgeKey> =
        std::collections::HashSet::new();
    let mut contours = Vec::new();
    let mut keys: Vec<EdgeKey> = links.keys().copied().collect();
    keys.sort_unstable();
    for start in keys {
        if visited.contains(&start) {
            continue;
        }
        let neighbors = &links[&start];
        if neighbors.len() != 2 {
            // Degenerate connectivity (shouldn't happen); skip defensively.
            visited.insert(start);
            continue;
        }
        let mut loop_keys = vec![start];
        visited.insert(start);
        let mut prev = start;
        let mut current = neighbors[0];
        while current != start {
            visited.insert(current);
            loop_keys.push(current);
            let next_links = match links.get(&current) {
                Some(l) if l.len() == 2 => l,
                _ => break,
            };
            let next = if next_links[0] == prev {
                next_links[1]
            } else {
                next_links[0]
            };
            prev = current;
            current = next;
        }
        if loop_keys.len() < 8 {
            continue;
        }
        // Convert to y-up image coordinates.
        let points: Vec<(f64, f64)> = loop_keys
            .iter()
            .map(|&k| {
                let (x, y_down) = crossing(k);
                (x, h as f64 - y_down)
            })
            .collect();
        let contour = SubpixelContour { points };
        if contour.signed_area2().abs() * 0.5 >= min_area_px {
            contours.push(contour);
        }
    }
    contours
}

#[cfg(test)]
mod tests {
    use super::*;
    use image::Luma;

    fn disk_image(w: u32, h: u32, cx: f64, cy: f64, r: f64) -> GrayImage {
        // Anti-aliased disk sampled at pixel centers (x + 0.5, y + 0.5),
        // matching the extractor's convention.
        GrayImage::from_fn(w, h, |x, y| {
            let d = ((x as f64 + 0.5 - cx).powi(2)
                + (y as f64 + 0.5 - cy).powi(2))
            .sqrt();
            let v = ((d - r + 0.5).clamp(0.0, 1.0) * 255.0) as u8;
            Luma([v])
        })
    }

    #[test]
    fn disk_iso_contour_radius_accuracy() {
        let img = disk_image(64, 64, 32.0, 32.0, 20.0);
        let contours = extract_iso_contours(&img, 127, false, 16.0);
        assert_eq!(contours.len(), 1, "expected one contour");
        let c = &contours[0];
        // Every point should be within ~0.35px of the true radius.
        for &(x, y) in &c.points {
            let d = ((x - 32.0).powi(2) + (y - (64.0 - 32.0)).powi(2)).sqrt();
            assert!(
                (d - 20.0).abs() < 0.35,
                "boundary point at radius {d:.3}, expected 20.0"
            );
        }
    }

    #[test]
    fn rectangle_touching_border_is_closed() {
        // Ink rectangle flush with the image border must still close.
        let img = GrayImage::from_fn(20, 20, |x, _y| {
            if x < 10 { Luma([0]) } else { Luma([255]) }
        });
        let contours = extract_iso_contours(&img, 127, false, 4.0);
        assert_eq!(contours.len(), 1);
        assert!(contours[0].points.len() >= 8);
    }

    #[test]
    fn hole_produces_second_contour() {
        // Ring: ink between r=8 and r=16.
        let img = GrayImage::from_fn(64, 64, |x, y| {
            let d =
                ((x as f64 - 32.0).powi(2) + (y as f64 - 32.0).powi(2)).sqrt();
            if d > 8.0 && d < 16.0 {
                Luma([0])
            } else {
                Luma([255])
            }
        });
        let contours = extract_iso_contours(&img, 127, false, 16.0);
        assert_eq!(contours.len(), 2, "expected outer + hole contours");
    }
}
