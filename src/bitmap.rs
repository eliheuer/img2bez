use std::path::Path;

use image::{GrayImage, ImageReader};
use imageproc::contrast::otsu_level;

use crate::config::{ThresholdMethod, TracingConfig};
use crate::error::TraceError;

/// Load an image and convert it to a binary (black/white) GrayImage.
///
/// Foreground (glyph) pixels are 255, background pixels are 0.
pub fn load_and_threshold(path: &Path, config: &TracingConfig) -> Result<GrayImage, TraceError> {
    let img = ImageReader::open(path)
        .map_err(|e| TraceError::ImageLoad(e.to_string()))?
        .decode()
        .map_err(|e| TraceError::ImageLoad(e.to_string()))?
        .into_luma8();

    let threshold = match config.threshold {
        ThresholdMethod::Fixed(t) => t,
        ThresholdMethod::Otsu => {
            let t = otsu_level(&img);
            if config.verbose {
                eprintln!("  Threshold   Otsu = {}", t);
            }
            t
        }
    };

    // BinaryInverted: pixels BELOW the threshold become 255 (foreground).
    // This matches the convention that dark pixels in scanned glyph images
    // are the glyph body (foreground) and light pixels are background.
    let mut binary = imageproc::contrast::threshold(
        &img,
        threshold,
        imageproc::contrast::ThresholdType::BinaryInverted,
    );

    if config.invert {
        for pixel in binary.pixels_mut() {
            pixel.0[0] = 255 - pixel.0[0];
        }
    }

    // Fill small interior holes: background (0) regions not reachable from the
    // image border that are smaller than min_contour_area are noise from paper
    // texture / threshold artifacts. Large holes (real counters in O, e, 8…)
    // are preserved.
    fill_small_holes(&mut binary, config.min_contour_area as u32);

    // Debug: save thresholded bitmap
    if std::env::var("IMG2BEZ_DEBUG_BITMAP").is_ok() {
        binary.save("debug_threshold.png").ok();
        eprintln!("  Debug       saved debug_threshold.png");
    }

    Ok(binary)
}

/// Find all interior background regions (holes) and fill those smaller than
/// `max_hole_area` pixels. Large holes (real counters in O, e, 8…) are kept.
fn fill_small_holes(binary: &mut GrayImage, max_hole_area: u32) {
    let (w, h) = binary.dimensions();
    if w == 0 || h == 0 {
        return;
    }

    // Label each background pixel as either "border-connected" (0) or a hole ID (1..N).
    // hole_id 0 = reachable from border = true background.
    let size = (w * h) as usize;
    let mut hole_id = vec![0u32; size]; // 0 = unassigned
    let mut hole_sizes: Vec<u32> = vec![0]; // hole_sizes[0] unused (border)
    let mut next_id = 1u32;

    // Mark all foreground pixels so we skip them.
    for y in 0..h {
        for x in 0..w {
            if binary.get_pixel(x, y)[0] != 0 {
                hole_id[(y * w + x) as usize] = u32::MAX; // foreground sentinel
            }
        }
    }

    // Flood-fill each connected background region.
    for start_y in 0..h {
        for start_x in 0..w {
            let start_idx = (start_y * w + start_x) as usize;
            if hole_id[start_idx] != 0 {
                continue; // already labeled or foreground
            }

            // Check if this region touches the border.
            let id = next_id;
            next_id += 1;
            hole_sizes.push(0);

            let mut touches_border = false;
            let mut stack = vec![(start_x, start_y)];
            hole_id[start_idx] = id;

            while let Some((x, y)) = stack.pop() {
                hole_sizes[id as usize] += 1;
                if x == 0 || x == w - 1 || y == 0 || y == h - 1 {
                    touches_border = true;
                }
                for (dx, dy) in [(-1i32, 0), (1, 0), (0, -1i32), (0, 1)] {
                    let nx = x as i32 + dx;
                    let ny = y as i32 + dy;
                    if nx >= 0 && nx < w as i32 && ny >= 0 && ny < h as i32 {
                        let (nx, ny) = (nx as u32, ny as u32);
                        let idx = (ny * w + nx) as usize;
                        if hole_id[idx] == 0 {
                            hole_id[idx] = id;
                            stack.push((nx, ny));
                        }
                    }
                }
            }

            // Border-connected regions are true background — mark them safe.
            if touches_border {
                hole_sizes[id as usize] = u32::MAX; // never fill
            }
        }
    }

    // Fill small interior holes.
    for y in 0..h {
        for x in 0..w {
            let idx = (y * w + x) as usize;
            let id = hole_id[idx];
            if id != u32::MAX && id > 0 && hole_sizes[id as usize] < max_hole_area {
                binary.put_pixel(x, y, image::Luma([255u8]));
            }
        }
    }
}
