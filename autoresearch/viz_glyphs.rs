/// viz_glyphs.rs — bezier structure comparison images for img2bez autoresearch.
///
/// Usage (via designbot):
///   designbot --render viz_glyphs.rs --output /dev/null \
///       -- <traced.ufo> <reference.ufo> <work_dir> [glyph ...]
///
/// Each output image is split left/right:
///   left  — reference glyph (gray)
///   right — traced glyph  (blue outline, red on-curve dots, cyan off-curve squares)
///
/// Output files: <work_dir>/viz_uni<XXXX>.png
use designbot::norad;
use designbot::kurbo::{BezPath, PathEl, Point};
use designbot::prelude::*;

const W: f64 = 900.0;
const H: f64 = 500.0;
const PAD: f64 = 40.0;

fn glyph_bbox(glyph: &norad::Glyph) -> (f64, f64, f64, f64) {
    let mut min_x = f64::MAX;
    let mut min_y = f64::MAX;
    let mut max_x = f64::MIN;
    let mut max_y = f64::MIN;
    for contour in &glyph.contours {
        for pt in &contour.points {
            let (x, y) = (pt.x as f64, pt.y as f64);
            min_x = min_x.min(x);
            min_y = min_y.min(y);
            max_x = max_x.max(x);
            max_y = max_y.max(y);
        }
    }
    if min_x == f64::MAX {
        (0.0, 0.0, 100.0, 100.0)
    } else {
        (min_x, min_y, max_x, max_y)
    }
}

/// Convert a norad contour to a kurbo BezPath (without using contour.to_kurbo()
/// to avoid a kurbo version mismatch between norad and designbot).
fn contour_to_bezpath(contour: &norad::Contour) -> BezPath {
    let pts = &contour.points;
    let n = pts.len();
    if n == 0 {
        return BezPath::new();
    }

    // Find the first on-curve point as the start.
    let start_idx = pts
        .iter()
        .position(|p| !matches!(p.typ, norad::PointType::OffCurve))
        .unwrap_or(0);

    let mut path = BezPath::new();
    let sp = &pts[start_idx];
    path.move_to(Point::new(sp.x as f64, sp.y as f64));

    let mut i = (start_idx + 1) % n;
    while i != start_idx {
        let p = &pts[i];
        match p.typ {
            norad::PointType::Line => {
                path.line_to(Point::new(p.x as f64, p.y as f64));
                i = (i + 1) % n;
            }
            norad::PointType::Curve => {
                let p1 = &pts[(i + n - 2) % n];
                let p2 = &pts[(i + n - 1) % n];
                path.curve_to(
                    Point::new(p1.x as f64, p1.y as f64),
                    Point::new(p2.x as f64, p2.y as f64),
                    Point::new(p.x as f64, p.y as f64),
                );
                i = (i + 1) % n;
            }
            _ => {
                i = (i + 1) % n;
            }
        }
    }
    path.close_path();
    path
}

fn seg_counts(glyph: &norad::Glyph) -> (usize, usize) {
    let mut curves = 0usize;
    let mut lines = 0usize;
    for contour in &glyph.contours {
        for pt in &contour.points {
            match pt.typ {
                norad::PointType::Curve | norad::PointType::QCurve => curves += 1,
                norad::PointType::Line => lines += 1,
                _ => {}
            }
        }
    }
    (curves, lines)
}

/// Draw a glyph's bezier structure into `ctx`.
/// Glyph coordinates are Y-up; screen coordinates are Y-down.
fn draw_structure(
    ctx: &mut Canvas,
    glyph: &norad::Glyph,
    panel_x: f64,
    x0: f64,
    y0: f64,
    gw: f64,
    scale: f64,
    outline_col: Color,
    on_col: Color,
    off_col: Color,
) {
    let panel_w = W / 2.0;
    let glyph_screen_w = gw * scale;
    let x_off = panel_x + (panel_w - glyph_screen_w) / 2.0 - x0 * scale;
    // Place glyph bottom at PAD from canvas bottom.
    let y_off = H - PAD + y0 * scale;

    // Font→screen: x_s = x_off + x_f*scale,  y_s = y_off - y_f*scale
    let to_screen = |xf: f64, yf: f64| -> (f64, f64) {
        (x_off + xf * scale, y_off - yf * scale)
    };

    ctx.save();

    // Draw outline.
    for contour in &glyph.contours {
        let bez = contour_to_bezpath(contour);
        let mut screen_path = BezPath::new();
        for el in bez.iter() {
            match el {
                PathEl::MoveTo(p) => {
                    let (sx, sy) = to_screen(p.x, p.y);
                    screen_path.move_to(Point::new(sx, sy));
                }
                PathEl::LineTo(p) => {
                    let (sx, sy) = to_screen(p.x, p.y);
                    screen_path.line_to(Point::new(sx, sy));
                }
                PathEl::CurveTo(p1, p2, p) => {
                    let (sx1, sy1) = to_screen(p1.x, p1.y);
                    let (sx2, sy2) = to_screen(p2.x, p2.y);
                    let (sx, sy) = to_screen(p.x, p.y);
                    screen_path.curve_to(
                        Point::new(sx1, sy1),
                        Point::new(sx2, sy2),
                        Point::new(sx, sy),
                    );
                }
                PathEl::ClosePath => {
                    screen_path.close_path();
                }
                _ => {}
            }
        }
        ctx.no_fill();
        ctx.stroke(outline_col);
        ctx.stroke_width(2.0);
        ctx.draw_path(screen_path);
    }

    // Draw handle lines.
    ctx.stroke(off_col);
    ctx.stroke_width(1.0);
    ctx.no_fill();
    for contour in &glyph.contours {
        let pts = &contour.points;
        let n = pts.len();
        for i in 0..n {
            let pt = &pts[i];
            if matches!(pt.typ, norad::PointType::Curve | norad::PointType::QCurve) {
                let prev = &pts[(i + n - 1) % n];
                if matches!(prev.typ, norad::PointType::OffCurve) {
                    let (x1, y1) = to_screen(pt.x as f64, pt.y as f64);
                    let (x2, y2) = to_screen(prev.x as f64, prev.y as f64);
                    ctx.line(x1, y1, x2, y2);
                }
                let prev2 = &pts[(i + n - 2) % n];
                if !matches!(prev2.typ, norad::PointType::OffCurve) {
                    let prev1 = &pts[(i + n - 1) % n];
                    if matches!(prev1.typ, norad::PointType::OffCurve) {
                        let (x1, y1) = to_screen(pt.x as f64, pt.y as f64);
                        let (x2, y2) = to_screen(prev1.x as f64, prev1.y as f64);
                        ctx.line(x1, y1, x2, y2);
                    }
                }
            }
        }
    }

    // Draw points.
    let r_on = 16.0 / scale.max(0.1);
    let r_off = 10.0 / scale.max(0.1);
    for contour in &glyph.contours {
        for pt in &contour.points {
            let (sx, sy) = to_screen(pt.x as f64, pt.y as f64);
            if matches!(pt.typ, norad::PointType::OffCurve) {
                ctx.fill(off_col);
                ctx.stroke(Color::white());
                ctx.stroke_width(1.5 / scale.max(0.1));
                ctx.rect(sx - r_off / 2.0, sy - r_off / 2.0, r_off, r_off);
            } else if !matches!(pt.typ, norad::PointType::Move) {
                ctx.fill(on_col);
                ctx.stroke(Color::white());
                ctx.stroke_width(1.5 / scale.max(0.1));
                ctx.oval(sx - r_on / 2.0, sy - r_on / 2.0, r_on, r_on);
            }
        }
    }

    ctx.restore();
}

fn render_glyph_viz(
    name: &str,
    traced: &norad::Glyph,
    reference: &norad::Glyph,
    out_path: &str,
) {
    let (x0, y0, x1, y1) = glyph_bbox(traced);
    let gw = (x1 - x0).max(1.0);
    let gh = (y1 - y0).max(1.0);
    let scale = ((W / 2.0 - 2.0 * PAD) / gw).min((H - 2.0 * PAD) / gh);

    let (tc, tl) = seg_counts(traced);
    let (rc, rl) = seg_counts(reference);

    let mut ctx = Canvas::new(W, H);
    ctx.background(Color::from_floats(0.97, 0.97, 0.97, 1.0));

    ctx.fill(Color::black());
    ctx.font_size(22.0);
    ctx.text(name, 20.0, 35.0);
    ctx.fill(Color::from_floats(0.4, 0.4, 0.4, 1.0));
    ctx.font_size(16.0);
    ctx.text(
        &format!("ref {}c+{}l   traced {}c+{}l", rc, rl, tc, tl),
        80.0,
        35.0,
    );

    // Left panel — reference.
    ctx.fill(Color::from_floats(0.93, 0.93, 0.93, 1.0));
    ctx.no_stroke();
    ctx.rect(0.0, 0.0, W / 2.0, H);
    ctx.fill(Color::from_floats(0.5, 0.5, 0.5, 1.0));
    ctx.font_size(13.0);
    ctx.text_align(TextAlign::Center);
    ctx.text(&format!("reference  ({}c+{}l)", rc, rl), W / 4.0, H - 16.0);

    draw_structure(
        &mut ctx, reference, 0.0, x0, y0, gw, scale,
        Color::from_floats(0.6, 0.6, 0.6, 1.0),
        Color::from_floats(0.55, 0.55, 0.55, 1.0),
        Color::from_floats(0.75, 0.75, 0.75, 1.0),
    );

    // Right panel — traced.
    ctx.fill(Color::from_floats(0.35, 0.35, 0.35, 1.0));
    ctx.font_size(13.0);
    ctx.text_align(TextAlign::Center);
    ctx.text(&format!("traced  ({}c+{}l)", tc, tl), 3.0 * W / 4.0, H - 16.0);

    draw_structure(
        &mut ctx, traced, W / 2.0, x0, y0, gw, scale,
        Color::from_floats(0.1, 0.2, 0.85, 1.0),
        Color::from_floats(0.85, 0.15, 0.1, 1.0),
        Color::from_floats(0.1, 0.5, 0.9, 1.0),
    );

    let renderer = Renderer::new(W as u32, H as u32);
    renderer.render_to_png(&ctx, out_path).expect("render failed");
    println!("  viz {} → {}", name, out_path);
}

fn get_glyph<'a>(font: &'a norad::Font, name: &str) -> Option<&'a norad::Glyph> {
    font.default_layer().iter().find(|g| g.name().as_str() == name)
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    // designbot forwards extra args directly; arg[0] is the binary name.
    let user_args: Vec<&str> = args[1..].iter().map(|s| s.as_str()).collect();

    if user_args.len() < 3 {
        eprintln!(
            "Usage: designbot --render viz_glyphs.rs --output /dev/null \
             -- <traced.ufo> <reference.ufo> <work_dir> [glyph ...]"
        );
        std::process::exit(1);
    }

    let traced_ufo_path = user_args[0];
    let ref_ufo_path = user_args[1];
    let work_dir = user_args[2];
    let glyph_filter: Vec<&str> = user_args[3..].to_vec();

    let traced_font = norad::Font::load(traced_ufo_path).expect("failed to load traced UFO");
    let ref_font = norad::Font::load(ref_ufo_path).expect("failed to load reference UFO");

    let names: Vec<String> = if glyph_filter.is_empty() {
        let mut ns: Vec<String> = traced_font
            .default_layer()
            .iter()
            .filter_map(|g| {
                let n = g.name().to_string();
                if get_glyph(&ref_font, &n).is_some() { Some(n) } else { None }
            })
            .collect();
        ns.sort();
        ns
    } else {
        glyph_filter.iter().map(|s| s.to_string()).collect()
    };

    for name in &names {
        let traced_glyph = match get_glyph(&traced_font, name) {
            Some(g) => g,
            None => { eprintln!("  skip {} (not in traced UFO)", name); continue; }
        };
        let ref_glyph = match get_glyph(&ref_font, name) {
            Some(g) => g,
            None => { eprintln!("  skip {} (not in reference UFO)", name); continue; }
        };

        let file_key = if name.chars().count() == 1 {
            format!("uni{:04X}", name.chars().next().unwrap() as u32)
        } else {
            name.clone()
        };
        let out_path = format!("{}/viz_{}.png", work_dir, file_key);
        render_glyph_viz(name, traced_glyph, ref_glyph, &out_path);
    }
}
