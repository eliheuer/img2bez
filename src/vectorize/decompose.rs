//! Pixel-edge contour extraction on the dual grid.
//!
//! Contours are traced on the pixel-corner grid (between pixels) rather
//! than through pixel centers. A straight edge of N pixels produces 2
//! points, not N. Uses XOR fill to handle contour nesting (outer vs hole).

use image::GrayImage;

/// A closed path on the pixel-corner grid.
#[derive(Debug, Clone)]
pub struct PixelPath {
    /// Points in pixel-corner coordinates (y-up).
    /// (0,0) = bottom-left corner of image. (w,h) = top-right.
    pub points: Vec<(i32, i32)>,
    /// +1 for outer boundary, -1 for hole.
    pub sign: i8,
}

/// Efficient bitmap for contour tracing.
///
/// Stores the image as a flat boolean array with y-up coordinates
/// (y=0 at bottom, increasing upward).
struct Bitmap {
    data: Vec<bool>,
    width: i32,
    height: i32,
}

impl Bitmap {
    /// Create from a binary GrayImage (non-zero = foreground).
    fn from_gray(img: &GrayImage) -> Self {
        let (w, h) = img.dimensions();
        let width = w as i32;
        let height = h as i32;

        // Store row-major, but flip y so row 0 is the bottom of the image.
        let mut data = vec![false; (w * h) as usize];
        for iy in 0..h {
            for ix in 0..w {
                let set = img.get_pixel(ix, iy).0[0] > 0;
                // Flip y: row 0 at bottom
                let py = height - 1 - iy as i32;
                data[(py * width + ix as i32) as usize] = set;
            }
        }

        Bitmap {
            data,
            width,
            height,
        }
    }

    /// Get pixel at (x, y) in y-up coordinates. Out-of-bounds = false.
    fn get(&self, x: i32, y: i32) -> bool {
        if x < 0 || x >= self.width || y < 0 || y >= self.height {
            return false;
        }
        self.data[(y * self.width + x) as usize]
    }

    /// XOR all pixels in row y from column x to the right edge.
    fn xor_row_from(&mut self, x: i32, y: i32) {
        if y < 0 || y >= self.height {
            return;
        }
        let x_start = x.max(0);
        for xi in x_start..self.width {
            let idx = (y * self.width + xi) as usize;
            self.data[idx] ^= true;
        }
    }
}

/// Extract closed contours from a binary image on the dual (pixel-corner) grid.
///
/// Scans in raster order, traces each boundary, and XORs the interior
/// to handle nesting.
///
/// `min_area`: discard contours smaller than this (in pixels).
pub fn decompose(gray: &GrayImage, min_area: usize) -> Vec<PixelPath> {
    let mut bm = Bitmap::from_gray(gray);
    // Keep a copy of the original bitmap for sign detection.
    let orig = bm.data.clone();
    let mut paths = Vec::new();

    // Scan bottom-to-top, left-to-right (y increases upward).
    for y in 0..bm.height {
        for x in 0..bm.width {
            if bm.get(x, y) {
                // Determine sign using the ORIGINAL bitmap (not the XOR'd working copy).
                // If pixel below is not set in original → outer boundary.
                let below_set = if y > 0 && y - 1 < bm.height && x >= 0 && x < bm.width {
                    orig[((y - 1) * bm.width + x) as usize]
                } else {
                    false
                };
                let sign: i8 = if !below_set { 1 } else { -1 };

                let path = find_path(&bm, x, y, sign);
                xor_fill(&mut bm, &path);
                paths.push(path);
            }
        }
    }

    // Filter tiny contours.
    paths.retain(|p| path_area(&p.points).abs() >= min_area as f64);

    paths
}

/// Trace one closed contour starting from pixel (x0, y0).
///
/// Follows the boundary on the dual grid, keeping foreground on the left.
/// Uses the "right" turn policy for ambiguous (diagonal) crossings.
fn find_path(bm: &Bitmap, x0: i32, y0: i32, sign: i8) -> PixelPath {
    let mut points = Vec::new();
    let mut x = x0;
    let mut y = y0;
    let mut dx: i32 = 0;
    let mut dy: i32 = sign as i32; // +1 (up) for outer, -1 (down) for hole

    loop {
        points.push((x, y));

        // Pixel offset formulas for the dual grid.
        //
        // The path walks on pixel corners. At each corner (x, y) with
        // direction (dx, dy), we check the two pixels that share the
        // edge we're about to cross:
        //
        //   c = pixel to the RIGHT of the path direction
        //   d = pixel to the LEFT  (should be foreground for normal traversal)
        //
        // The formula `(dx + dy - 1) / 2` (integer division) maps each
        // cardinal direction to the correct pixel offset:
        //
        //   dir (dx,dy)  │ c_offset_x=(dx+dy-1)/2  c_offset_y=(dy-dx-1)/2
        //   ─────────────┼─────────────────────────────────────────────────
        //   Up    (0, 1) │  0                       -1
        //   Down  (0,-1) │ -1                        0
        //   Right (1, 0) │  0                        0
        //   Left  (-1,0) │ -1                       -1
        //
        //   dir (dx,dy)  │ d_offset_x=(dx-dy-1)/2  d_offset_y=(dy+dx-1)/2
        //   ─────────────┼─────────────────────────────────────────────────
        //   Up    (0, 1) │ -1                        0
        //   Down  (0,-1) │  0                       -1
        //   Right (1, 0) │  0                        0  (same row)
        //   Left  (-1,0) │ -1                       -1
        let c = bm.get(x + (dx + dy - 1) / 2, y + (dy - dx - 1) / 2);
        let d = bm.get(x + (dx - dy - 1) / 2, y + (dy + dx - 1) / 2);

        // Turn policy decision tree:
        //
        //   c  d  │ action
        //   ──────┼─────────────────────────────────────
        //   1  0  │ Ambiguous (diagonal crossing): turn RIGHT
        //   1  1  │ Both set: turn LEFT (keep foreground on left)
        //   0  0  │ Both unset: turn RIGHT (find foreground)
        //   0  1  │ Normal: go STRAIGHT (foreground on left, bg on right)
        if c && !d {
            let tmp = dx;
            dx = -dy;
            dy = tmp;
        } else if c {
            let tmp = dx;
            dx = dy;
            dy = -tmp;
        } else if !d {
            let tmp = dx;
            dx = -dy;
            dy = tmp;
        }

        x += dx;
        y += dy;

        if x == x0 && y == y0 {
            break;
        }
    }

    PixelPath { points, sign }
}

/// XOR-fill the interior of a path.
///
/// For each vertical step in the path, toggle all pixels from that column
/// to the right edge of the row. Pairs of toggles cancel outside the
/// contour, leaving only the interior flipped.
fn xor_fill(bm: &mut Bitmap, path: &PixelPath) {
    let n = path.points.len();
    if n == 0 {
        return;
    }

    let mut y_prev = path.points[n - 1].1;

    for k in 0..n {
        let (x, y) = path.points[k];
        if y != y_prev {
            // Vertical step: toggle row at the lower y value.
            let row = y.min(y_prev);
            bm.xor_row_from(x, row);
        }
        y_prev = y;
    }
}

/// Signed area of a pixel-corner path via shoelace formula.
fn path_area(points: &[(i32, i32)]) -> f64 {
    let n = points.len();
    if n < 3 {
        return 0.0;
    }
    let mut area: i64 = 0;
    for i in 0..n {
        let j = (i + 1) % n;
        area += points[i].0 as i64 * points[j].1 as i64 - points[j].0 as i64 * points[i].1 as i64;
    }
    area as f64 / 2.0
}
