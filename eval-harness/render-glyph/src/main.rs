//! Render a single UFO glyph to a PNG, in the coordinate space img2bez expects.
//!
//! The output image spans the full ascender-to-descender height of the font,
//! with the glyph drawn black on white. This matches an img2bez trace invoked
//! with `--target-height (ascender - descender)` and `--y-offset descender`.
//!
//! This is the Rust replacement for the old `render_glyph.py` drawbot-skia
//! renderer. It is a dev-only eval-harness tool (never published).
//!
//! Usage:
//!   render-glyph <ufo_path> <glyph_name> <output_png> [--height 700]
//!
//! Prints, for shell/Python callers to capture:
//!   target_height=<float>
//!   y_offset=<float>
//!   advance_width=<float>
//!   px_height=<int>
//!   px_width=<int>
//!   x_shift_font=<float>

use std::process::exit;

use kurbo::{BezPath, PathEl, Shape};
use norad::Font;
use tiny_skia::{Color, FillRule, Paint, PathBuilder, Pixmap, PremultipliedColorU8, Transform};

fn die(msg: impl AsRef<str>) -> ! {
    eprintln!("render-glyph: {}", msg.as_ref());
    exit(1);
}

struct Args {
    ufo_path: String,
    glyph_name: String,
    output_png: String,
    height: u32,
}

fn parse_args() -> Args {
    let mut positional: Vec<String> = Vec::new();
    let mut height: u32 = 700;
    let mut it = std::env::args().skip(1);
    while let Some(arg) = it.next() {
        match arg.as_str() {
            "--height" => {
                let v = it.next().unwrap_or_else(|| die("--height needs a value"));
                height = v
                    .parse()
                    .unwrap_or_else(|_| die(format!("invalid --height: {v}")));
            }
            "-h" | "--help" => {
                println!("usage: render-glyph <ufo_path> <glyph_name> <output_png> [--height N]");
                exit(0);
            }
            other => positional.push(other.to_string()),
        }
    }
    if positional.len() != 3 {
        die("expected <ufo_path> <glyph_name> <output_png>");
    }
    Args {
        ufo_path: positional[0].clone(),
        glyph_name: positional[1].clone(),
        output_png: positional[2].clone(),
        height,
    }
}

fn main() {
    let args = parse_args();

    let font = Font::load(&args.ufo_path)
        .unwrap_or_else(|e| die(format!("failed to load {}: {e}", args.ufo_path)));

    // Font metrics, with the same fallbacks the Python renderer used.
    let upm = font
        .font_info
        .units_per_em
        .map(|v| v.as_f64())
        .unwrap_or(1000.0);
    let ascender = font.font_info.ascender.unwrap_or(800.0);
    let descender = font.font_info.descender.unwrap_or(-200.0);
    let total_height = ascender - descender;
    if total_height <= 0.0 {
        die(format!("non-positive font height ({total_height})"));
    }

    let glyph = font
        .get_glyph(&args.glyph_name)
        .unwrap_or_else(|| die(format!("glyph '{}' not found", args.glyph_name)));

    let advance_width = if glyph.width != 0.0 { glyph.width } else { upm };
    let scale = args.height as f64 / total_height;

    // Combine every contour into one path (font units, y-up).
    let mut path = BezPath::new();
    for contour in &glyph.contours {
        match contour.to_kurbo() {
            Ok(bp) => path.extend(bp),
            Err(e) => die(format!("contour to kurbo failed: {e}")),
        }
    }

    // Tight bounds (curve extrema included), matching ufoLib2 getBounds().
    let (min_x, min_y, max_y) = if path.elements().is_empty() {
        (0.0, descender, ascender)
    } else {
        let b = path.bounding_box();
        (b.x0, b.y0, b.y1)
    };

    // Negative-LSB glyphs (j, f, ...) extend left of x=0; widen the canvas and
    // shift the drawing right so the whole glyph is visible.
    let x_shift_font = (-min_x).max(0.0);
    let px_x_offset = (x_shift_font * scale + 0.5) as i64;
    let px_width = (((advance_width * scale) as i64) + px_x_offset).max(1) as u32;
    let px_height = args.height;

    let mut pixmap = Pixmap::new(px_width, px_height)
        .unwrap_or_else(|| die(format!("invalid canvas size {px_width}x{px_height}")));
    pixmap.fill(Color::WHITE);

    // Font space (y-up, baseline at 0) -> pixel space (y-down, origin top-left):
    //   pixel_x = (font_x + x_shift_font) * scale
    //   pixel_y = px_height - (font_y - descender) * scale
    let transform = Transform {
        sx: scale as f32,
        kx: 0.0,
        ky: 0.0,
        sy: -(scale as f32),
        tx: (x_shift_font * scale) as f32,
        ty: (px_height as f64 + descender * scale) as f32,
    };

    if !path.elements().is_empty() {
        let mut pb = PathBuilder::new();
        for el in path.elements() {
            match *el {
                PathEl::MoveTo(p) => pb.move_to(p.x as f32, p.y as f32),
                PathEl::LineTo(p) => pb.line_to(p.x as f32, p.y as f32),
                PathEl::QuadTo(c, p) => pb.quad_to(c.x as f32, c.y as f32, p.x as f32, p.y as f32),
                PathEl::CurveTo(c1, c2, p) => pb.cubic_to(
                    c1.x as f32,
                    c1.y as f32,
                    c2.x as f32,
                    c2.y as f32,
                    p.x as f32,
                    p.y as f32,
                ),
                PathEl::ClosePath => pb.close(),
            }
        }
        if let Some(sk_path) = pb.finish() {
            let mut paint = Paint::default();
            paint.set_color(Color::BLACK);
            paint.anti_alias = true;
            // Font glyphs use non-zero winding, like drawbot's default fill.
            pixmap.fill_path(&sk_path, &paint, FillRule::Winding, transform, None);
        }
    }

    // Whiten anti-aliased pixel rows that bleed outside the glyph's y bounds.
    // AA can smear 1-2 px past the true bbox (e.g. below the baseline for T, l,
    // h, n), which img2bez would otherwise read as phantom CURVE sections. A 2px
    // margin keeps the legitimate top/bottom edges.
    let clip_bottom = (px_height as f64 - (min_y - descender) * scale) as i64 + 2;
    let clip_top = (px_height as f64 - (max_y - descender) * scale) as i64 - 2;
    if clip_bottom < px_height as i64 || clip_top > 0 {
        let white = PremultipliedColorU8::from_rgba(255, 255, 255, 255)
            .expect("opaque white is a valid premultiplied color");
        let w = px_width as usize;
        let pixels = pixmap.pixels_mut();
        for row in 0..px_height as i64 {
            if row < clip_top || row > clip_bottom {
                let start = row as usize * w;
                for px in &mut pixels[start..start + w] {
                    *px = white;
                }
            }
        }
    }

    pixmap
        .save_png(&args.output_png)
        .unwrap_or_else(|e| die(format!("failed to write {}: {e}", args.output_png)));

    // key=value lines consumed by render_glyph.py and the eval-harness shells.
    println!("target_height={total_height}");
    println!("y_offset={descender}");
    println!("advance_width={advance_width}");
    println!("px_height={px_height}");
    println!("px_width={px_width}");
    println!("x_shift_font={x_shift_font}");
}
