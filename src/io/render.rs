// Copyright 2026 the img2bez Authors
// SPDX-License-Identifier: Apache-2.0 OR MIT

//! Raster rendering of traced BezPaths for visual comparison.
//!
//! Converts kurbo `BezPath`s back to pixels via tiny-skia, producing a
//! three-panel comparison PNG against the source bitmap
//! ([`render_comparison`]) and a pixel-level IoU diff against a
//! reference outline ([`raster_compare`]).

use std::path::Path;

use kurbo::{BezPath, PathEl};
use tiny_skia::PremultipliedColorU8;

// ── Comparison panel layout ──────────────────────────────

/// Width of each comparison panel in pixels.
const PANEL_WIDTH: u32 = 800;
/// Height of each comparison panel in pixels.
const PANEL_HEIGHT: u32 = 800;

/// Padding around the content area within each panel.
const PANEL_PADDING: u32 = 20;

/// Width of the grey separator bar between panels.
const PANEL_SEPARATOR: u32 = 2;

// ── Raster comparison layout ─────────────────────────────

/// Square canvas size for raster IoU comparison rendering.
const RASTER_CANVAS_SIZE: u32 = 600;

/// Padding around the glyph in the raster comparison canvas.
const RASTER_PADDING: f64 = 20.0;

/// Opaque premultiplied color; with alpha=255 `from_rgba` cannot fail.
#[inline]
fn opaque(r: u8, g: u8, b: u8) -> PremultipliedColorU8 {
    PremultipliedColorU8::from_rgba(r, g, b, 255)
        .expect("alpha=255 is always valid")
}

/// Convert a kurbo `BezPath` to a `tiny_skia::Path`.
fn kurbo_to_tinyskia(
    bezpath: &BezPath,
    transform: tiny_skia::Transform,
) -> Option<tiny_skia::Path> {
    let mut pb = tiny_skia::PathBuilder::new();
    for el in bezpath.elements() {
        match *el {
            PathEl::MoveTo(p) => {
                let (x, y) = transform_point(p.x, p.y, transform);
                pb.move_to(x, y);
            }
            PathEl::LineTo(p) => {
                let (x, y) = transform_point(p.x, p.y, transform);
                pb.line_to(x, y);
            }
            PathEl::QuadTo(c, p) => {
                let (cx, cy) = transform_point(c.x, c.y, transform);
                let (px, py) = transform_point(p.x, p.y, transform);
                pb.quad_to(cx, cy, px, py);
            }
            PathEl::CurveTo(c1, c2, p) => {
                let (c1x, c1y) = transform_point(c1.x, c1.y, transform);
                let (c2x, c2y) = transform_point(c2.x, c2.y, transform);
                let (px, py) = transform_point(p.x, p.y, transform);
                pb.cubic_to(c1x, c1y, c2x, c2y, px, py);
            }
            PathEl::ClosePath => pb.close(),
        }
    }
    pb.finish()
}

/// Apply an affine transform to a point, narrowing f64 → f32.
#[inline]
fn transform_point(x: f64, y: f64, t: tiny_skia::Transform) -> (f32, f32) {
    let x = x as f32;
    let y = y as f32;
    (t.sx * x + t.kx * y + t.tx, t.ky * x + t.sy * y + t.ty)
}

/// Draw bezier handles and point markers over a panel: gray handles, green
/// on-curve points, purple off-curve points.
fn draw_handles_and_points(
    pixmap: &mut tiny_skia::Pixmap,
    paths: &[BezPath],
    transform: tiny_skia::Transform,
) {
    let mut handle_paint = tiny_skia::Paint::default();
    handle_paint.set_color(tiny_skia::Color::from_rgba8(144, 144, 144, 255));
    handle_paint.anti_alias = true;
    let stroke = tiny_skia::Stroke {
        width: 2.0,
        ..Default::default()
    };

    let mut on_pts: Vec<(f32, f32)> = Vec::new();
    let mut off_pts: Vec<(f32, f32)> = Vec::new();

    for path in paths {
        let mut prev: Option<(f32, f32)> = None;
        for el in path.elements() {
            match *el {
                PathEl::MoveTo(p) | PathEl::LineTo(p) => {
                    let q = transform_point(p.x, p.y, transform);
                    on_pts.push(q);
                    prev = Some(q);
                }
                PathEl::CurveTo(c1, c2, p) => {
                    let pc1 = transform_point(c1.x, c1.y, transform);
                    let pc2 = transform_point(c2.x, c2.y, transform);
                    let pp = transform_point(p.x, p.y, transform);
                    if let Some(pv) = prev {
                        stroke_line(pixmap, pv, pc1, &handle_paint, &stroke);
                    }
                    stroke_line(pixmap, pp, pc2, &handle_paint, &stroke);
                    off_pts.push(pc1);
                    off_pts.push(pc2);
                    on_pts.push(pp);
                    prev = Some(pp);
                }
                PathEl::QuadTo(c, p) => {
                    let pc = transform_point(c.x, c.y, transform);
                    let pp = transform_point(p.x, p.y, transform);
                    if let Some(pv) = prev {
                        stroke_line(pixmap, pv, pc, &handle_paint, &stroke);
                    }
                    stroke_line(pixmap, pp, pc, &handle_paint, &stroke);
                    off_pts.push(pc);
                    on_pts.push(pp);
                    prev = Some(pp);
                }
                PathEl::ClosePath => {}
            }
        }
    }

    for &(x, y) in &off_pts {
        fill_dot(
            pixmap,
            x,
            y,
            6.0,
            tiny_skia::Color::from_rgba8(139, 108, 255, 255),
        );
    }
    for &(x, y) in &on_pts {
        fill_dot(
            pixmap,
            x,
            y,
            7.0,
            tiny_skia::Color::from_rgba8(102, 238, 136, 255),
        );
    }
}

/// Stroke a single line segment between two already-transformed points.
fn stroke_line(
    pixmap: &mut tiny_skia::Pixmap,
    a: (f32, f32),
    b: (f32, f32),
    paint: &tiny_skia::Paint,
    stroke: &tiny_skia::Stroke,
) {
    let mut pb = tiny_skia::PathBuilder::new();
    pb.move_to(a.0, a.1);
    pb.line_to(b.0, b.1);
    if let Some(path) = pb.finish() {
        pixmap.stroke_path(
            &path,
            paint,
            stroke,
            tiny_skia::Transform::identity(),
            None,
        );
    }
}

/// Draw a hollow point marker: dark fill with a colored ring outline.
fn fill_dot(
    pixmap: &mut tiny_skia::Pixmap,
    cx: f32,
    cy: f32,
    r: f32,
    color: tiny_skia::Color,
) {
    let mut pb = tiny_skia::PathBuilder::new();
    pb.push_circle(cx, cy, r);
    if let Some(path) = pb.finish() {
        let mut fill = tiny_skia::Paint::default();
        fill.set_color(tiny_skia::Color::from_rgba8(22, 22, 22, 255));
        fill.anti_alias = true;
        pixmap.fill_path(
            &path,
            &fill,
            tiny_skia::FillRule::Winding,
            tiny_skia::Transform::identity(),
            None,
        );
        let mut ring = tiny_skia::Paint::default();
        ring.set_color(color);
        ring.anti_alias = true;
        let stroke = tiny_skia::Stroke {
            width: 2.6,
            ..Default::default()
        };
        pixmap.stroke_path(
            &path,
            &ring,
            &stroke,
            tiny_skia::Transform::identity(),
            None,
        );
    }
}

/// Encode a pixmap to PNG bytes.
fn encode_png(pixmap: &tiny_skia::Pixmap) -> Result<Vec<u8>, std::io::Error> {
    let mut buf = Vec::new();
    let mut encoder =
        png::Encoder::new(&mut buf, pixmap.width(), pixmap.height());
    encoder.set_color(png::ColorType::Rgba);
    encoder.set_depth(png::BitDepth::Eight);
    let mut writer = encoder
        .write_header()
        .map_err(|e| std::io::Error::other(e.to_string()))?;
    writer
        .write_image_data(pixmap.data())
        .map_err(|e| std::io::Error::other(e.to_string()))?;
    drop(writer);
    Ok(buf)
}

/// Render BezPaths as filled black on a white square pixmap: one combined
/// EvenOdd fill, no anti-aliasing, for crisp pixel comparison.
fn render_paths_to_pixmap(
    paths: &[BezPath],
    size: u32,
    transform: tiny_skia::Transform,
) -> tiny_skia::Pixmap {
    let mut pm =
        tiny_skia::Pixmap::new(size, size).expect("non-zero canvas size");
    pm.fill(tiny_skia::Color::WHITE);
    let mut paint = tiny_skia::Paint::default();
    paint.set_color(tiny_skia::Color::BLACK);
    paint.anti_alias = false;

    let mut combined = BezPath::new();
    for p in paths {
        for el in p.elements() {
            combined.push(*el);
        }
    }
    if let Some(sk) = kurbo_to_tinyskia(&combined, transform) {
        pm.fill_path(
            &sk,
            &paint,
            tiny_skia::FillRule::EvenOdd,
            tiny_skia::Transform::identity(),
            None,
        );
    }
    pm
}

/// Render a three-panel comparison PNG to `output_path`.
///
/// Panels, left to right:
/// 1. The source image as-is (grayscale, letterboxed).
/// 2. Structure view: the traced outline with its handles and point
///    markers.
/// 3. Overlay: the trace filled in red over a dimmed copy of the
///    source.
///
/// The trace is mapped back into source-pixel space by inverting the
/// pipeline's placement: `font_scale` is target height over image
/// height, `y_offset` is the Y offset applied during tracing
/// (typically the descender), and `reposition_shift` is the (dx, dy)
/// translation applied by the reposition step.
///
/// With `IMG2BEZ_DEBUG_PIXELDIFF` set, also writes a 1:1 pixel-diff
/// image next to `output_path` and prints overlap counts.
pub fn render_comparison(
    source_image: &Path,
    traced_paths: &[BezPath],
    output_path: &Path,
    font_scale: f64,
    y_offset: f64,
    reposition_shift: (f64, f64),
    verbose: bool,
) -> Result<(), std::io::Error> {
    let panel_h: u32 = PANEL_HEIGHT;
    let panel_w: u32 = PANEL_WIDTH;
    let padding: u32 = PANEL_PADDING;
    let separator: u32 = PANEL_SEPARATOR;

    let content_w = (panel_w - padding * 2) as f32;
    let content_h = (panel_h - padding * 2) as f32;

    // The thresholded binary is only used by the debug pixel diff below;
    // the panels show the grayscale as-is.
    let img = image::open(source_image)
        .map_err(|e| {
            std::io::Error::new(std::io::ErrorKind::NotFound, e.to_string())
        })?
        .into_luma8();
    let threshold = imageproc::contrast::otsu_level(&img);
    let binary = imageproc::contrast::threshold(
        &img,
        threshold,
        imageproc::contrast::ThresholdType::BinaryInverted,
    );
    let (src_w, src_h) = (binary.width(), binary.height());

    // Compute shared scale: fit source image into panel content area
    let img_scale = (content_w / src_w as f32).min(content_h / src_h as f32);
    let rendered_w = (src_w as f32 * img_scale) as u32;
    let rendered_h = (src_h as f32 * img_scale) as u32;
    let ox = padding + (content_w as u32 - rendered_w) / 2;
    let oy = padding + (content_h as u32 - rendered_h) / 2;

    // ── Left panel: the source image as-is (grayscale, anti-aliased) ──
    // Show the real input: img2bez traces from the gray anti-aliased pixels.
    let resized = image::imageops::resize(
        &img,
        rendered_w,
        rendered_h,
        image::imageops::FilterType::Triangle,
    );
    // Letterbox with the source's own background color (corner sample) so the
    // padding blends with the image.
    let bg = img.get_pixel(0, 0).0[0];
    let mut source_panel =
        tiny_skia::Pixmap::new(panel_w, panel_h).expect("non-zero panel size");
    source_panel.fill(tiny_skia::Color::from_rgba8(bg, bg, bg, 255));
    for y in 0..rendered_h {
        for x in 0..rendered_w {
            let luma = resized.get_pixel(x, y).0[0];
            let dst_x = ox + x;
            let dst_y = oy + y;
            if dst_x < panel_w && dst_y < panel_h {
                let pm = opaque(luma, luma, luma);
                source_panel.pixels_mut()[(dst_y * panel_w + dst_x) as usize] =
                    pm;
            }
        }
    }

    // ── Right panel: traced paths at the same scale ──
    // Invert the pipeline's placement (px → font units, then the reposition
    // shift) to map font coords back to source pixels, then Y-down panel
    // coords:
    //   x_panel = s * (x_font - dx) + ox
    //   y_panel = -s * y_font + oy + rendered_h + s * (dy + y_offset)
    // where s = img_scale / font_scale.
    let (dx, dy) = reposition_shift;
    let s = img_scale as f64 / font_scale;
    if verbose {
        eprintln!(
            "  Render      src={}x{} img_scale={:.4} font_scale={:.4} s={:.6}",
            src_w, src_h, img_scale, font_scale, s
        );
        eprintln!(
            "  Render      rendered={}x{} ox={} oy={} padding={}",
            rendered_w, rendered_h, ox, oy, padding
        );
        eprintln!(
            "  Render      dx={:.1} dy={:.1} y_offset={:.1}",
            dx, dy, y_offset
        );
    }
    let tx = (ox as f64 - s * dx) as f32;
    let ty = (oy as f64 + rendered_h as f64 + s * (dy + y_offset)) as f32;
    if verbose {
        eprintln!("  Render      tx={:.2} ty={:.2}", tx, ty);
    }
    let transform = tiny_skia::Transform {
        sx: s as f32,
        kx: 0.0,
        ky: 0.0,
        sy: -(s as f32), // flip Y
        tx,
        ty,
    };

    let mut traced_panel =
        tiny_skia::Pixmap::new(panel_w, panel_h).expect("non-zero panel size");
    traced_panel.fill(tiny_skia::Color::from_rgba8(22, 22, 22, 255));

    // Combine all contours into a single path so EvenOdd fill rule
    // correctly cuts holes for counter contours.
    let mut combined = BezPath::new();
    for path in traced_paths {
        for el in path.elements() {
            combined.push(*el);
        }
    }
    // Structure view: a faint dark-gray fill so the shape reads, a dark-gray
    // outline stroke, then the handles and points drawn on top.
    let mut fill_paint = tiny_skia::Paint::default();
    fill_paint.set_color(tiny_skia::Color::from_rgba8(184, 184, 184, 32));
    fill_paint.anti_alias = true;
    let mut outline_paint = tiny_skia::Paint::default();
    outline_paint.set_color(tiny_skia::Color::from_rgba8(201, 201, 201, 255));
    outline_paint.anti_alias = true;
    let outline_stroke = tiny_skia::Stroke {
        width: 2.0,
        ..Default::default()
    };
    if let Some(sk_path) = kurbo_to_tinyskia(&combined, transform) {
        traced_panel.fill_path(
            &sk_path,
            &fill_paint,
            tiny_skia::FillRule::EvenOdd,
            tiny_skia::Transform::identity(),
            None,
        );
        traced_panel.stroke_path(
            &sk_path,
            &outline_paint,
            &outline_stroke,
            tiny_skia::Transform::identity(),
            None,
        );
    }
    draw_handles_and_points(&mut traced_panel, traced_paths, transform);

    // ── Third panel: overlay (traced in red on a light-gray card) ──
    // Gray card so the red trace stands out; the source is remapped so its
    // white paper blends into the card while dark strokes stay visible.
    let card = 200u8;
    let mut overlay_panel =
        tiny_skia::Pixmap::new(panel_w, panel_h).expect("non-zero panel size");
    overlay_panel.fill(tiny_skia::Color::from_rgba8(card, card, card, 255));
    for y in 0..rendered_h {
        for x in 0..rendered_w {
            let luma = resized.get_pixel(x, y).0[0];
            let v = (70u32 + luma as u32 * (card as u32 - 70) / 255) as u8;
            let dst_x = ox + x;
            let dst_y = oy + y;
            if dst_x < panel_w && dst_y < panel_h {
                overlay_panel.pixels_mut()
                    [(dst_y * panel_w + dst_x) as usize] = opaque(v, v, v);
            }
        }
    }
    let mut red_paint = tiny_skia::Paint::default();
    // Primary red (#ff4a3d), kept semi-transparent for the overlay.
    red_paint.set_color(tiny_skia::Color::from_rgba8(255, 74, 61, 185));
    red_paint.anti_alias = true;
    if let Some(sk_path) = kurbo_to_tinyskia(&combined, transform) {
        overlay_panel.fill_path(
            &sk_path,
            &red_paint,
            tiny_skia::FillRule::EvenOdd,
            tiny_skia::Transform::identity(),
            None,
        );
    }

    // ── Composite ──
    let total_w = panel_w * 3 + separator * 2;
    let mut final_pixmap =
        tiny_skia::Pixmap::new(total_w, panel_h).expect("non-zero panel size");
    final_pixmap.fill(tiny_skia::Color::from_rgba8(12, 12, 12, 255));
    for y in 0..panel_h {
        for x in 0..panel_w {
            let idx = (y * panel_w + x) as usize;
            final_pixmap.pixels_mut()[(y * total_w + x) as usize] =
                source_panel.pixels()[idx];
            final_pixmap.pixels_mut()
                [(y * total_w + panel_w + separator + x) as usize] =
                traced_panel.pixels()[idx];
            final_pixmap.pixels_mut()
                [(y * total_w + (panel_w + separator) * 2 + x) as usize] =
                overlay_panel.pixels()[idx];
        }
    }

    let png_data = encode_png(&final_pixmap)?;
    std::fs::write(output_path, png_data)?;

    // ── 1:1 pixel diff (debug) ──
    // Render contour at source resolution, compare with binary pixel-by-pixel.
    if std::env::var("IMG2BEZ_DEBUG_PIXELDIFF").is_ok() {
        let (dx, dy) = reposition_shift;
        let s1 = 1.0 / font_scale; // maps font units to source pixels
        let t1x1 = tiny_skia::Transform {
            sx: s1 as f32,
            kx: 0.0,
            ky: 0.0,
            sy: -(s1 as f32),
            tx: (-(s1 * dx)) as f32,
            ty: (src_h as f64 + s1 * (dy + y_offset)) as f32,
        };
        let mut contour_pm =
            tiny_skia::Pixmap::new(src_w, src_h).expect("non-zero image size");
        contour_pm.fill(tiny_skia::Color::WHITE);
        let mut black = tiny_skia::Paint::default();
        black.set_color(tiny_skia::Color::BLACK);
        black.anti_alias = false;
        if let Some(sk_path) = kurbo_to_tinyskia(&combined, t1x1) {
            contour_pm.fill_path(
                &sk_path,
                &black,
                tiny_skia::FillRule::EvenOdd,
                tiny_skia::Transform::identity(),
                None,
            );
        }
        // Compare: count pixels that differ
        let mut source_only = 0u64; // black in source, white in contour
        let mut contour_only = 0u64; // white in source, black in contour
        let mut both = 0u64; // both black
        for y in 0..src_h {
            for x in 0..src_w {
                let src_px = binary.get_pixel(x, y).0[0]; // 0=glyph, 255=bg
                let cnt_px = contour_pm.pixels()[(y * src_w + x) as usize];
                let cnt_is_black = cnt_px.red() < 128;
                let src_is_black = src_px < 128;
                match (src_is_black, cnt_is_black) {
                    (true, true) => both += 1,
                    (true, false) => source_only += 1,
                    (false, true) => contour_only += 1,
                    (false, false) => {}
                }
            }
        }
        eprintln!(
            "  PixelDiff   overlap={} source_only={} contour_only={} total_src={} total_cnt={}",
            both,
            source_only,
            contour_only,
            both + source_only,
            both + contour_only
        );
        // Save the diff image: green=overlap, red=contour-only, blue=source-only
        let mut diff_pm =
            tiny_skia::Pixmap::new(src_w, src_h).expect("non-zero image size");
        diff_pm.fill(tiny_skia::Color::WHITE);
        for y in 0..src_h {
            for x in 0..src_w {
                let src_px = binary.get_pixel(x, y).0[0];
                let cnt_px = contour_pm.pixels()[(y * src_w + x) as usize];
                let cnt_is_black = cnt_px.red() < 128;
                let src_is_black = src_px < 128;
                let color = match (src_is_black, cnt_is_black) {
                    (true, true) => opaque(0, 128, 0),
                    (true, false) => opaque(0, 0, 255),
                    (false, true) => opaque(255, 0, 0),
                    (false, false) => continue,
                };
                diff_pm.pixels_mut()[(y * src_w + x) as usize] = color;
            }
        }
        let stem = output_path
            .file_stem()
            .and_then(|s| s.to_str())
            .unwrap_or("output");
        let diff_path =
            output_path.with_file_name(format!("{}_pixeldiff.png", stem));
        std::fs::write(&diff_path, encode_png(&diff_pm)?)?;
        eprintln!("  PixelDiff   saved {}", diff_path.display());
    }

    Ok(())
}

/// Result of raster-based comparison between traced and reference outlines.
pub struct RasterCompare {
    /// Intersection over union of filled regions (0.0–1.0).
    pub iou: f64,
    /// Pixels filled in traced output.
    pub traced_px: u64,
    /// Pixels filled in reference.
    pub ref_px: u64,
    /// Pixels filled in both.
    pub overlap_px: u64,
    /// Path to the saved diff image.
    pub diff_path: std::path::PathBuf,
}

/// Render both traced and reference BezPaths and compare pixel-by-pixel.
///
/// Both are rendered filled black at the same scale, aligned by bounding
/// box center. Returns IoU and saves a diff image:
///   green = both filled, red = traced only, blue = reference only.
pub fn raster_compare(
    traced_paths: &[BezPath],
    reference_paths: &[BezPath],
    output_dir: &Path,
    glyph_name: &str,
) -> Result<RasterCompare, std::io::Error> {
    use kurbo::Shape;

    let canvas_size: u32 = RASTER_CANVAS_SIZE;
    let padding: f64 = RASTER_PADDING;

    // Union bounding box of both path sets.
    let t_bbox = traced_paths
        .iter()
        .map(|p| p.bounding_box())
        .reduce(|a, b| a.union(b));
    let r_bbox = reference_paths
        .iter()
        .map(|p| p.bounding_box())
        .reduce(|a, b| a.union(b));

    let (t_bbox, r_bbox) = match (t_bbox, r_bbox) {
        (Some(t), Some(r)) => (t, r),
        _ => {
            let diff_path =
                output_dir.join(format!("{}_raster_diff.png", glyph_name));
            return Ok(RasterCompare {
                iou: 0.0,
                traced_px: 0,
                ref_px: 0,
                overlap_px: 0,
                diff_path,
            });
        }
    };

    // Align by bounding box center: translate traced to match reference center.
    let t_cx = t_bbox.x0 + t_bbox.width() / 2.0;
    let t_cy = t_bbox.y0 + t_bbox.height() / 2.0;
    let r_cx = r_bbox.x0 + r_bbox.width() / 2.0;
    let r_cy = r_bbox.y0 + r_bbox.height() / 2.0;
    let dx = r_cx - t_cx;
    let dy = r_cy - t_cy;

    // Reference bbox (plus padding) is the viewport; traced is aligned to it.
    let view_bbox = r_bbox;
    let content = canvas_size as f64 - padding * 2.0;
    let scale = content / view_bbox.width().max(view_bbox.height());

    // Reference transform: font coords → canvas coords (Y-flipped).
    let ref_tx = padding - view_bbox.x0 * scale;
    let ref_ty = padding + (view_bbox.y0 + view_bbox.height()) * scale;
    let ref_transform = tiny_skia::Transform {
        sx: scale as f32,
        kx: 0.0,
        ky: 0.0,
        sy: -(scale as f32),
        tx: ref_tx as f32,
        ty: ref_ty as f32,
    };

    // Traced transform: same as reference but with center-alignment offset.
    let traced_transform = tiny_skia::Transform {
        sx: scale as f32,
        kx: 0.0,
        ky: 0.0,
        sy: -(scale as f32),
        tx: (ref_tx + dx * scale) as f32,
        ty: (ref_ty - dy * scale) as f32,
    };

    // Render traced (center-aligned) and reference.
    let traced_pm =
        render_paths_to_pixmap(traced_paths, canvas_size, traced_transform);
    let ref_pm =
        render_paths_to_pixmap(reference_paths, canvas_size, ref_transform);

    // Compare pixels.
    let total = (canvas_size * canvas_size) as usize;
    let mut overlap: u64 = 0;
    let mut traced_only: u64 = 0;
    let mut ref_only: u64 = 0;

    let mut diff_pm = tiny_skia::Pixmap::new(canvas_size, canvas_size)
        .expect("non-zero canvas size");
    diff_pm.fill(tiny_skia::Color::from_rgba8(32, 32, 32, 255));

    for i in 0..total {
        let t_black = traced_pm.pixels()[i].red() < 128;
        let r_black = ref_pm.pixels()[i].red() < 128;
        let color = match (t_black, r_black) {
            (true, true) => {
                overlap += 1;
                opaque(0, 180, 0)
            }
            (true, false) => {
                traced_only += 1;
                opaque(220, 60, 60)
            }
            (false, true) => {
                ref_only += 1;
                opaque(60, 60, 220)
            }
            (false, false) => continue,
        };
        diff_pm.pixels_mut()[i] = color;
    }

    let traced_px = overlap + traced_only;
    let ref_px = overlap + ref_only;
    let union_px = traced_px + ref_only;
    let iou = if union_px > 0 {
        overlap as f64 / union_px as f64
    } else {
        1.0
    };

    let diff_path = output_dir.join(format!("{}_raster_diff.png", glyph_name));
    std::fs::write(&diff_path, encode_png(&diff_pm)?)?;

    Ok(RasterCompare {
        iou,
        traced_px,
        ref_px,
        overlap_px: overlap,
        diff_path,
    })
}
