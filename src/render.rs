//! Raster rendering of traced BezPaths for visual comparison.
//!
//! Converts kurbo BezPaths back to pixels via tiny-skia and produces
//! side-by-side comparison PNGs against the source bitmap.

use std::path::Path;

use kurbo::{BezPath, PathEl};

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

/// Apply transform manually to a point (f64 → f32).
fn transform_point(x: f64, y: f64, t: tiny_skia::Transform) -> (f32, f32) {
    let x = x as f32;
    let y = y as f32;
    (
        t.sx * x + t.kx * y + t.tx,
        t.ky * x + t.sy * y + t.ty,
    )
}

/// Encode a pixmap to PNG bytes.
fn encode_png(pixmap: &tiny_skia::Pixmap) -> Vec<u8> {
    let mut buf = Vec::new();
    let mut encoder = png::Encoder::new(&mut buf, pixmap.width(), pixmap.height());
    encoder.set_color(png::ColorType::Rgba);
    encoder.set_depth(png::BitDepth::Eight);
    let mut writer = encoder.write_header().unwrap();
    writer.write_image_data(pixmap.data()).unwrap();
    drop(writer);
    buf
}

/// Render a side-by-side comparison PNG.
///
/// Left panel: source bitmap (thresholded to B/W).
/// Right panel: traced contours re-rasterized at the same scale.
///
/// `font_scale` = target_height / image_height (the scale used during tracing).
/// `y_offset` = the Y offset used during tracing (typically the descender).
/// `reposition_shift` = (dx, dy) translation applied by the reposition step.
pub fn render_comparison(
    source_image: &Path,
    traced_paths: &[BezPath],
    output_path: &Path,
    font_scale: f64,
    y_offset: f64,
    reposition_shift: (f64, f64),
) -> Result<(), std::io::Error> {
    let panel_h: u32 = 800;
    let panel_w: u32 = 800;
    let padding: u32 = 20;
    let separator: u32 = 2;

    let content_w = (panel_w - padding * 2) as f32;
    let content_h = (panel_h - padding * 2) as f32;

    // Load and threshold source image
    let img = image::open(source_image)
        .expect("failed to open source image")
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

    // ── Left panel: thresholded source ──
    let resized = image::imageops::resize(
        &binary,
        rendered_w,
        rendered_h,
        image::imageops::FilterType::Nearest,
    );
    let mut source_panel = tiny_skia::Pixmap::new(panel_w, panel_h).unwrap();
    source_panel.fill(tiny_skia::Color::WHITE);
    for y in 0..rendered_h {
        for x in 0..rendered_w {
            let luma = resized.get_pixel(x, y).0[0];
            let dst_x = ox + x;
            let dst_y = oy + y;
            if dst_x < panel_w && dst_y < panel_h {
                let pm =
                    tiny_skia::PremultipliedColorU8::from_rgba(luma, luma, luma, 255).unwrap();
                source_panel.pixels_mut()[(dst_y * panel_w + dst_x) as usize] = pm;
            }
        }
    }

    // ── Right panel: traced paths at the same scale ──
    //
    // The tracing pipeline transforms pixel coords (Y-up) → font units:
    //   x_font_old = x_px * font_scale
    //   y_font_old = y_px * font_scale + y_offset
    //
    // Then reposition shifts by (dx, dy):
    //   x_font = x_font_old + dx
    //   y_font = y_font_old + dy
    //
    // To map font coords back to source image pixel coords (Y-up):
    //   x_px = (x_font - dx) / font_scale
    //   y_px = (y_font - dy - y_offset) / font_scale
    //
    // Then to panel coords (Y-down display):
    //   x_panel = ox + x_px * img_scale
    //   y_panel = oy + (src_h - y_px) * img_scale
    //
    // Combining:
    //   x_panel = s * (x_font - dx) + ox
    //   y_panel = -s * (y_font - dy - y_offset) + oy + rendered_h
    //           = -s * y_font + oy + rendered_h + s * (dy + y_offset)
    //
    // where s = img_scale / font_scale
    let (dx, dy) = reposition_shift;
    let s = img_scale as f64 / font_scale;
    eprintln!("  Render      src={}x{} img_scale={:.4} font_scale={:.4} s={:.6}", src_w, src_h, img_scale, font_scale, s);
    eprintln!("  Render      rendered={}x{} ox={} oy={} padding={}", rendered_w, rendered_h, ox, oy, padding);
    eprintln!("  Render      dx={:.1} dy={:.1} y_offset={:.1}", dx, dy, y_offset);
    let tx = (ox as f64 - s * dx) as f32;
    let ty = (oy as f64 + rendered_h as f64 + s * (dy + y_offset)) as f32;
    eprintln!("  Render      tx={:.2} ty={:.2}", tx, ty);
    // Verify: map font point (dx, -dy + y_offset) back — should give (ox, oy+rendered_h) i.e. bottom-left
    // Actually map the pre-reposition origin (0,0) in pixel-corner coords:
    //   font coords = (0*scale + dx, 0*scale + y_offset + dy)
    //   display = (s*(dx - dx) + ox, -s*(y_offset+dy - dy - y_offset) + oy + rendered_h)
    //           = (ox, oy + rendered_h) ✓
    let transform = tiny_skia::Transform {
        sx: s as f32,
        kx: 0.0,
        ky: 0.0,
        sy: -(s as f32), // flip Y
        tx,
        ty,
    };

    let mut traced_panel = tiny_skia::Pixmap::new(panel_w, panel_h).unwrap();
    traced_panel.fill(tiny_skia::Color::WHITE);
    let mut paint = tiny_skia::Paint::default();
    paint.set_color(tiny_skia::Color::BLACK);
    paint.anti_alias = true;

    // Combine all contours into a single path so EvenOdd fill rule
    // correctly cuts holes for counter contours.
    let mut combined = BezPath::new();
    for path in traced_paths {
        for el in path.elements() {
            combined.push(*el);
        }
    }
    if let Some(sk_path) = kurbo_to_tinyskia(&combined, transform) {
        traced_panel.fill_path(
            &sk_path,
            &paint,
            tiny_skia::FillRule::EvenOdd,
            tiny_skia::Transform::identity(),
            None,
        );
    }

    // ── Third panel: overlay (traced in red on source) ──
    let mut overlay_panel = source_panel.clone();
    let mut red_paint = tiny_skia::Paint::default();
    red_paint.set_color(tiny_skia::Color::from_rgba8(255, 0, 0, 128));
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
    let mut final_pixmap = tiny_skia::Pixmap::new(total_w, panel_h).unwrap();
    final_pixmap.fill(tiny_skia::Color::from_rgba8(200, 200, 200, 255));
    for y in 0..panel_h {
        for x in 0..panel_w {
            let idx = (y * panel_w + x) as usize;
            final_pixmap.pixels_mut()[(y * total_w + x) as usize] = source_panel.pixels()[idx];
            final_pixmap.pixels_mut()[(y * total_w + panel_w + separator + x) as usize] =
                traced_panel.pixels()[idx];
            final_pixmap.pixels_mut()[(y * total_w + (panel_w + separator) * 2 + x) as usize] =
                overlay_panel.pixels()[idx];
        }
    }

    let png_data = encode_png(&final_pixmap);
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
        let mut contour_pm = tiny_skia::Pixmap::new(src_w, src_h).unwrap();
        contour_pm.fill(tiny_skia::Color::WHITE);
        let mut black = tiny_skia::Paint::default();
        black.set_color(tiny_skia::Color::BLACK);
        black.anti_alias = false;
        if let Some(sk_path) = kurbo_to_tinyskia(&combined, t1x1) {
            contour_pm.fill_path(
                &sk_path, &black, tiny_skia::FillRule::EvenOdd,
                tiny_skia::Transform::identity(), None,
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
        eprintln!("  PixelDiff   overlap={} source_only={} contour_only={} total_src={} total_cnt={}",
            both, source_only, contour_only, both + source_only, both + contour_only);
        // Save the diff image: green=overlap, red=contour-only, blue=source-only
        let mut diff_pm = tiny_skia::Pixmap::new(src_w, src_h).unwrap();
        diff_pm.fill(tiny_skia::Color::WHITE);
        for y in 0..src_h {
            for x in 0..src_w {
                let src_px = binary.get_pixel(x, y).0[0];
                let cnt_px = contour_pm.pixels()[(y * src_w + x) as usize];
                let cnt_is_black = cnt_px.red() < 128;
                let src_is_black = src_px < 128;
                let color = match (src_is_black, cnt_is_black) {
                    (true, true) => tiny_skia::PremultipliedColorU8::from_rgba(0, 128, 0, 255).unwrap(),
                    (true, false) => tiny_skia::PremultipliedColorU8::from_rgba(0, 0, 255, 255).unwrap(),
                    (false, true) => tiny_skia::PremultipliedColorU8::from_rgba(255, 0, 0, 255).unwrap(),
                    (false, false) => continue,
                };
                diff_pm.pixels_mut()[(y * src_w + x) as usize] = color;
            }
        }
        let diff_path = output_path.with_file_name(
            format!("{}_pixeldiff.png", output_path.file_stem().unwrap().to_str().unwrap())
        );
        std::fs::write(&diff_path, encode_png(&diff_pm))?;
        eprintln!("  PixelDiff   saved {}", diff_path.display());
    }

    Ok(())
}
