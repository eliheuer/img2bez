//! Compare traced outline structure against the source font's outlines.
//!
//! For each requested glyph of a source UFO:
//!   1. flatten components and render the glyph black-on-white at --height
//!      (same framing as the render-glyph harness tool),
//!   2. trace the render in-process with the img2bez library at CLI-default
//!      options,
//!   3. draw a side-by-side structure sheet (source left, traced right:
//!      silhouette, outline, on-curve smooth/corner markers, handles),
//!   4. emit structure stats + handle-invariant scans to summary.tsv.
//!
//! The source font is ground truth for what the trace SHOULD be, so every
//! structural difference the sheet shows is a lead on a tracer improvement
//! (or a documented not-a-defect, like merging redundant collinear points).
//!
//! Usage:
//!   structure-compare <ufo_path> [--glyphs a-z,A-Z] [--height 1024] [--out DIR]

use std::collections::BTreeSet;
use std::fmt::Write as _;
use std::path::{Path, PathBuf};
use std::process::exit;

use designbot::prelude::*;
use kurbo::{Affine, BezPath, Point, Shape};
use norad::Font;

/// designbot pins a different kurbo major than this crate; same shape,
/// different nominal type — rebuild element-by-element.
fn db_path(path: &BezPath) -> designbot::prelude::BezPath {
    let mut out = designbot::prelude::BezPath::new();
    for el in path.elements() {
        match *el {
            kurbo::PathEl::MoveTo(p) => out.move_to((p.x, p.y)),
            kurbo::PathEl::LineTo(p) => out.line_to((p.x, p.y)),
            kurbo::PathEl::QuadTo(c, p) => out.quad_to((c.x, c.y), (p.x, p.y)),
            kurbo::PathEl::CurveTo(c1, c2, p) => {
                out.curve_to((c1.x, c1.y), (c2.x, c2.y), (p.x, p.y))
            }
            kurbo::PathEl::ClosePath => out.close_path(),
        }
    }
    out
}

const PANEL: f64 = 900.0;
const PAD: f64 = 80.0;
/// Text margin from the sheet edges (identical on all four sides).
const MARGIN: f64 = 40.0;
/// The one font size used for all sheet text.
const FONT_SIZE: f64 = 20.0;
/// Line spacing for the two-line header/footer blocks.
const LINE: f64 = 31.0;
/// Header band: top margin + two text lines + breathing room.
const HEADER: f64 = MARGIN + 2.0 * LINE + 28.0;
/// Footer band: two text lines + bottom margin.
const FOOTER: f64 = 2.0 * LINE + MARGIN + 6.0;
/// The one text color (light gray).
const TEXT: (u8, u8, u8) = (185, 185, 185);
/// Monospace family for all sheet text.
const MONO: &str = "Menlo";

/// Kink threshold (degrees): a smooth-flagged join whose tangents disagree
/// by more than this is counted as a kink. Integer coordinates make ~1 deg
/// noise unavoidable; 2 deg keeps the count meaningful.
const KINK_SMOOTH_DEG: f64 = 2.0;

/// Corner-flagged joins with a tangent break in [5, 25) degrees read as
/// "shallow corners" — usually a tangent point the tracer failed to smooth.
const SHALLOW_CORNER_LO: f64 = 5.0;
const SHALLOW_CORNER_HI: f64 = 25.0;

/// Magic-triangle ratio above which a handle counts as violating (matches
/// the scan_triangle.py scratchpad scanner; slack for integer rounding).
const TRIANGLE_TOL: f64 = 1.05;

/// Combined chord-projected handle reach fraction above which a cubic
/// counts as crossing-prone (pipeline enforces 0.9; 0.95 allows rounding).
const REACH_TOL: f64 = 0.95;

fn die(msg: impl AsRef<str>) -> ! {
    eprintln!("structure-compare: {}", msg.as_ref());
    exit(1);
}

// ---------------------------------------------------------------------------
// Shared outline model: both norad source glyphs and img2bez traces are
// reduced to this so every stat and scan runs identically on both sides.

#[derive(Clone, Copy, PartialEq)]
enum Kind {
    Off,
    Line,
    Curve,
    Qcurve,
    Move,
}

#[derive(Clone, Copy)]
struct Pt {
    x: f64,
    y: f64,
    kind: Kind,
    smooth: bool,
}

impl Pt {
    fn on_curve(&self) -> bool {
        self.kind != Kind::Off
    }
    fn point(&self) -> Point {
        Point::new(self.x, self.y)
    }
}

type GContour = Vec<Pt>;

/// One segment of a cyclic contour: the on-curve it starts at, the
/// off-curves in between, and the on-curve it ends at.
struct Seg {
    a: Pt,
    ctrl: Vec<Pt>,
    b: Pt,
}

fn segments(contour: &GContour) -> Vec<Seg> {
    let ons: Vec<usize> = (0..contour.len())
        .filter(|&i| contour[i].on_curve())
        .collect();
    if ons.is_empty() {
        return Vec::new();
    }
    let n = contour.len();
    let mut segs = Vec::with_capacity(ons.len());
    for (k, &i) in ons.iter().enumerate() {
        let j = ons[(k + 1) % ons.len()];
        let mut ctrl = Vec::new();
        let mut m = (i + 1) % n;
        while m != j {
            ctrl.push(contour[m]);
            m = (m + 1) % n;
        }
        segs.push(Seg {
            a: contour[i],
            ctrl,
            b: contour[j],
        });
    }
    segs
}

fn to_bezpath(contours: &[GContour]) -> BezPath {
    let mut path = BezPath::new();
    for c in contours {
        let segs = segments(c);
        if segs.is_empty() {
            continue;
        }
        path.move_to(segs[0].a.point());
        for s in &segs {
            match s.ctrl.len() {
                0 => path.line_to(s.b.point()),
                2 => path.curve_to(
                    s.ctrl[0].point(),
                    s.ctrl[1].point(),
                    s.b.point(),
                ),
                1 => path.quad_to(s.ctrl[0].point(), s.b.point()),
                // Degenerate (TrueType off-curve runs): polyline through.
                _ => {
                    for p in s.ctrl.iter().chain(std::iter::once(&s.b)) {
                        path.line_to(p.point());
                    }
                }
            }
        }
        path.close_path();
    }
    path
}

// ---------------------------------------------------------------------------
// Stats and scans

#[derive(Default)]
struct Stats {
    contours: usize,
    oncurves: usize,
    cubics: usize,
    lines: usize,
    kinks_smooth: usize, // smooth flag, tangents disagree > KINK_SMOOTH_DEG
    shallow_corners: usize, // corner flag, break in [5, 25) deg
    triangle_viol: usize,
    reach_viol: usize,
}

fn analyze(contours: &[GContour]) -> Stats {
    let mut st = Stats {
        contours: contours.len(),
        ..Stats::default()
    };
    for c in contours {
        let segs = segments(c);
        st.oncurves += segs.len();
        for (k, s) in segs.iter().enumerate() {
            match s.ctrl.len() {
                0 => st.lines += 1,
                2 => {
                    st.cubics += 1;
                    scan_cubic(s, &mut st);
                }
                _ => st.cubics += 1, // quads counted with curves
            }
            // Join at s.b: incoming tangent from this segment, outgoing
            // from the next.
            let nxt = &segs[(k + 1) % segs.len()];
            let vin = s.b.point()
                - s.ctrl.last().map(|p| p.point()).unwrap_or(s.a.point());
            let vout = nxt
                .ctrl
                .first()
                .map(|p| p.point())
                .unwrap_or(nxt.b.point())
                - s.b.point();
            if vin.hypot() < 1e-9 || vout.hypot() < 1e-9 {
                continue;
            }
            let dtheta = vin.cross(vout).atan2(vin.dot(vout)).abs().to_degrees();
            if s.b.smooth {
                if dtheta > KINK_SMOOTH_DEG {
                    st.kinks_smooth += 1;
                }
            } else if (SHALLOW_CORNER_LO..SHALLOW_CORNER_HI).contains(&dtheta) {
                st.shallow_corners += 1;
            }
        }
    }
    st
}

/// Magic-triangle + reach checks for one cubic (mirrors the pipeline's
/// invariants; violations in TRACED output are bugs, in SOURCE output they
/// are just design facts and still worth seeing).
fn scan_cubic(s: &Seg, st: &mut Stats) {
    let (a, b) = (s.a.point(), s.b.point());
    let (h1, h2) = (s.ctrl[0].point() - a, s.ctrl[1].point() - b);
    let (l1, l2) = (h1.hypot(), h2.hypot());
    let chord = b - a;
    if chord.hypot() < 1e-9 {
        return;
    }
    if l1 > 3.0 && l2 > 3.0 {
        let (u0, u1) = (h1 / l1, h2 / l2);
        let denom = u0.cross(u1);
        if denom.abs() > 1e-9 {
            let t = chord.cross(u1) / denom;
            let s_ = chord.cross(u0) / denom;
            if t > 0.0 && s_ > 0.0 && (l1 > t * TRIANGLE_TOL || l2 > s_ * TRIANGLE_TOL)
            {
                st.triangle_viol += 1;
            }
        }
    }
    let e = chord / chord.hypot();
    if h1.dot(e) + (-h2).dot(e) > chord.hypot() * REACH_TOL {
        st.reach_viol += 1;
    }
}

// ---------------------------------------------------------------------------
// Source side: norad glyph -> GContours, components flattened

fn source_contours(font: &Font, name: &str) -> Result<Vec<GContour>, String> {
    let glyph = font
        .get_glyph(name)
        .ok_or_else(|| format!("glyph '{name}' not found"))?;
    let mut out = Vec::new();
    collect(font, glyph, Affine::IDENTITY, 0, &mut out)?;
    Ok(out)
}

fn collect(
    font: &Font,
    glyph: &norad::Glyph,
    xf: Affine,
    depth: usize,
    out: &mut Vec<GContour>,
) -> Result<(), String> {
    if depth > 8 {
        return Err("component nesting too deep (cycle?)".into());
    }
    for contour in &glyph.contours {
        let mut pts = Vec::with_capacity(contour.points.len());
        for p in &contour.points {
            let q = xf * Point::new(p.x, p.y);
            pts.push(Pt {
                x: q.x,
                y: q.y,
                kind: match p.typ {
                    norad::PointType::Move => Kind::Move,
                    norad::PointType::Line => Kind::Line,
                    norad::PointType::OffCurve => Kind::Off,
                    norad::PointType::Curve => Kind::Curve,
                    norad::PointType::QCurve => Kind::Qcurve,
                },
                smooth: p.smooth,
            });
        }
        out.push(pts);
    }
    for comp in &glyph.components {
        let base = font
            .get_glyph(&comp.base)
            .ok_or_else(|| format!("missing component base '{}'", comp.base))?;
        let t = comp.transform;
        let child = Affine::new([
            t.x_scale, t.xy_scale, t.yx_scale, t.y_scale, t.x_offset,
            t.y_offset,
        ]);
        collect(font, base, xf * child, depth + 1, out)?;
    }
    Ok(())
}

// ---------------------------------------------------------------------------
// Traced side: img2bez Outline -> GContours

fn traced_contours(outline: &img2bez::Outline) -> Vec<GContour> {
    outline
        .contours
        .iter()
        .map(|c| {
            c.points
                .iter()
                .map(|p| Pt {
                    x: p.x,
                    y: p.y,
                    kind: match p.kind {
                        img2bez::PointKind::Move => Kind::Move,
                        img2bez::PointKind::Line => Kind::Line,
                        img2bez::PointKind::Curve => Kind::Curve,
                        img2bez::PointKind::QCurve => Kind::Qcurve,
                        img2bez::PointKind::OffCurve => Kind::Off,
                    },
                    smooth: p.smooth,
                })
                .collect()
        })
        .collect()
}

// ---------------------------------------------------------------------------
// Rendering the trace input (tiny-skia, same framing as render-glyph)

struct FontBox {
    ascender: f64,
    descender: f64,
}

fn render_input(
    path: &BezPath,
    fb: &FontBox,
    height: u32,
) -> Result<Vec<u8>, String> {
    use tiny_skia::{
        Color, FillRule, Paint, PathBuilder, Pixmap, PremultipliedColorU8,
        Transform,
    };
    let total_height = fb.ascender - fb.descender;
    if total_height <= 0.0 {
        return Err(format!("non-positive font height ({total_height})"));
    }
    if path.elements().is_empty() {
        return Err("empty outline".into());
    }
    let scale = height as f64 / total_height;
    let b = path.bounding_box();
    let (min_x, min_y, max_y) = (b.x0, b.y0, b.y1);
    let x_shift_font = (-min_x).max(0.0);
    let px_width =
        (((b.x1 + x_shift_font) * scale) as i64 + 8).max(16) as u32;
    let px_height = height;

    let mut pixmap = Pixmap::new(px_width, px_height)
        .ok_or_else(|| format!("invalid canvas {px_width}x{px_height}"))?;
    pixmap.fill(Color::WHITE);

    let transform = Transform {
        sx: scale as f32,
        kx: 0.0,
        ky: 0.0,
        sy: -(scale as f32),
        tx: ((x_shift_font * scale) + 4.0) as f32,
        ty: (px_height as f64 + fb.descender * scale) as f32,
    };

    let mut pb = PathBuilder::new();
    for el in path.elements() {
        match *el {
            kurbo::PathEl::MoveTo(p) => pb.move_to(p.x as f32, p.y as f32),
            kurbo::PathEl::LineTo(p) => pb.line_to(p.x as f32, p.y as f32),
            kurbo::PathEl::QuadTo(c, p) => {
                pb.quad_to(c.x as f32, c.y as f32, p.x as f32, p.y as f32)
            }
            kurbo::PathEl::CurveTo(c1, c2, p) => pb.cubic_to(
                c1.x as f32,
                c1.y as f32,
                c2.x as f32,
                c2.y as f32,
                p.x as f32,
                p.y as f32,
            ),
            kurbo::PathEl::ClosePath => pb.close(),
        }
    }
    let sk_path = pb.finish().ok_or("path build failed")?;
    let mut paint = Paint::default();
    paint.set_color(Color::BLACK);
    paint.anti_alias = true;
    pixmap.fill_path(&sk_path, &paint, FillRule::Winding, transform, None);

    // Whiten AA rows bleeding past the glyph's true y-bounds (see
    // render-glyph: phantom curve sections otherwise).
    let clip_bottom =
        (px_height as f64 - (min_y - fb.descender) * scale) as i64 + 2;
    let clip_top =
        (px_height as f64 - (max_y - fb.descender) * scale) as i64 - 2;
    if clip_bottom < px_height as i64 || clip_top > 0 {
        let white = PremultipliedColorU8::from_rgba(255, 255, 255, 255)
            .expect("opaque white");
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
    pixmap.encode_png().map_err(|e| e.to_string())
}

// ---------------------------------------------------------------------------
// Sheet drawing (designbot; canvas is y-down)

fn draw_panel(ctx: &mut Canvas, ox: f64, oy: f64, contours: &[GContour]) {
    let path = to_bezpath(contours);
    if path.elements().is_empty() {
        return;
    }
    let b = path.bounding_box();
    let avail = PANEL - 2.0 * PAD;
    let s = avail / (b.width().max(b.height()));
    let cx = ox + PAD + (avail - b.width() * s) / 2.0 - s * b.x0;
    let cy = oy + PAD + (avail - b.height() * s) / 2.0 + s * b.y1;
    let xf = Affine::new([s, 0.0, 0.0, -s, cx, cy]);
    let tp = |p: Point| xf * p;

    // Silhouette + outline (blog-demo dark theme: faint ink, light path).
    ctx.fill(Color::rgb(42, 42, 42));
    ctx.no_stroke();
    ctx.draw_path(db_path(&(xf * path.clone())));
    ctx.no_fill();
    ctx.stroke(Color::rgb(240, 240, 240));
    ctx.stroke_width(3.5);
    ctx.draw_path(db_path(&(xf * path)));

    // Handles under the point markers.
    for c in contours {
        for seg in segments(c) {
            if seg.ctrl.len() != 2 {
                continue;
            }
            for (ctrlp, anchor) in
                [(seg.ctrl[0], seg.a), (seg.ctrl[1], seg.b)]
            {
                let h = tp(ctrlp.point());
                let a = tp(anchor.point());
                ctx.no_fill();
                ctx.stroke(Color::rgb(115, 115, 145));
                ctx.stroke_width(1.6);
                ctx.line(a.x, a.y, h.x, h.y);
                ctx.stroke(Color::rgb(135, 125, 240));
                ctx.stroke_width(2.4);
                ctx.oval(h.x - 4.5, h.y - 4.5, 9.0, 9.0);
            }
        }
    }
    // On-curve markers: circle = smooth, square = corner.
    for c in contours {
        for p in c.iter().filter(|p| p.on_curve()) {
            let q = tp(p.point());
            ctx.no_fill();
            if p.smooth {
                ctx.stroke(Color::rgb(60, 200, 110));
                ctx.stroke_width(3.5);
                ctx.oval(q.x - 8.0, q.y - 8.0, 16.0, 16.0);
            } else {
                ctx.stroke(Color::rgb(240, 150, 30));
                ctx.stroke_width(3.5);
                ctx.rect(q.x - 7.0, q.y - 7.0, 14.0, 14.0);
            }
        }
    }

}

/// Everything a sheet needs to identify itself when shared with no other
/// context: which glyph, which source file, which tracer build, which
/// settings — plus the per-side stats.
struct SheetMeta<'a> {
    glyph: &'a str,
    codepoint: Option<char>,
    ufo_path: &'a Path,
    height: u32,
    date: &'a str,
    note: &'a str,
}

fn stats_line(label: &str, st: &Stats) -> String {
    format!(
        "{label}  {} points · {} curves · {} lines · {} contours",
        st.oncurves, st.cubics, st.lines, st.contours
    )
}

fn draw_sheet(
    out_png: &Path,
    meta: &SheetMeta,
    src: (&[GContour], &Stats),
    trc: (&[GContour], &Stats),
) -> Result<(), String> {
    let (w, h) = (PANEL * 2.0, HEADER + PANEL + FOOTER);
    let mut ctx = Canvas::new(w, h);
    ctx.background(Color::rgb(13, 13, 13));
    draw_panel(&mut ctx, 0.0, HEADER, src.0);
    draw_panel(&mut ctx, PANEL, HEADER, trc.0);

    // All text: one face, one size, one color, one margin.
    ctx.no_stroke();
    ctx.font(MONO);
    ctx.font_size(FONT_SIZE);
    ctx.fill(Color::rgb(TEXT.0, TEXT.1, TEXT.2));
    // text() positions the TOP of the text box (measured: ink starts
    // ~0.1em below y, ends ~1.0em below y), so equal visual margins put
    // the top line's y just above the margin and the bottom line's y a
    // full em above it.
    let h1 = MARGIN - FONT_SIZE * 0.1;
    let h2 = h1 + LINE;
    let f2 = h - MARGIN - FONT_SIZE;
    let f1 = f2 - LINE;

    // Header: glyph identity left, tool/build/date right; source path and
    // trace settings on the second line.
    let cp = meta
        .codepoint
        .map(|c| format!("U+{:04X}", c as u32))
        .unwrap_or_else(|| "(no codepoint)".into());
    let font_name = meta
        .ufo_path
        .file_stem()
        .map(|s| s.to_string_lossy().into_owned())
        .unwrap_or_default();
    let home = std::env::var("HOME").unwrap_or_default();
    let shown_path = meta.ufo_path.display().to_string();
    let shown_path = if !home.is_empty() && shown_path.starts_with(&home) {
        format!("~{}", &shown_path[home.len()..])
    } else {
        shown_path
    };
    ctx.text(&format!("{}  {}  {}", meta.glyph, cp, font_name), MARGIN, h1);
    ctx.text(&shown_path, MARGIN, h2);
    ctx.text_align(TextAlign::Right);
    ctx.text(
        &format!(
            "structure-compare · img2bez@{} · {}",
            env!("IMG2BEZ_GIT_REV"),
            meta.date
        ),
        w - MARGIN,
        h1,
    );
    ctx.text(
        &format!(
            "render {}px · trace wild · grid 2 · em 1088{}",
            meta.height,
            if meta.note.is_empty() {
                String::new()
            } else {
                format!(" · {}", meta.note)
            }
        ),
        w - MARGIN,
        h2,
    );
    ctx.text_align(TextAlign::Left);

    // Footer: source stats left-aligned, traced stats right-aligned — the
    // same left/right structure as the header.
    let gap = trc.1.oncurves as i64 - src.1.oncurves as i64;
    ctx.text(&stats_line("SOURCE", src.1), MARGIN, f1);
    ctx.text(&format!("kinks {}", src.1.kinks_smooth), MARGIN, f2);
    ctx.text_align(TextAlign::Right);
    ctx.text(&stats_line("TRACED", trc.1), w - MARGIN, f1);
    ctx.text(
        &format!(
            "gap {gap:+} · kinks {} · shallow {} · triangle {} · reach {}",
            trc.1.kinks_smooth,
            trc.1.shallow_corners,
            trc.1.triangle_viol,
            trc.1.reach_viol
        ),
        w - MARGIN,
        f2,
    );
    ctx.text_align(TextAlign::Left);

    let renderer = Renderer::new(w as u32, h as u32);
    renderer
        .render_to_png(&ctx, out_png.to_str().ok_or("bad out path")?)
        .map_err(|e| e.to_string())
}

// ---------------------------------------------------------------------------

struct Args {
    ufo_path: PathBuf,
    glyphs: Vec<String>,
    height: u32,
    out_dir: PathBuf,
}

fn parse_glyph_spec(spec: &str) -> Vec<String> {
    let mut names = Vec::new();
    for token in spec.split(',') {
        let token = token.trim();
        if token.is_empty() {
            continue;
        }
        let chars: Vec<char> = token.chars().collect();
        if chars.len() == 3 && chars[1] == '-' {
            let (lo, hi) = (chars[0] as u32, chars[2] as u32);
            if lo <= hi {
                for c in lo..=hi {
                    if let Some(c) = char::from_u32(c) {
                        names.push(c.to_string());
                    }
                }
                continue;
            }
        }
        names.push(token.to_string());
    }
    names
}

fn parse_args() -> Args {
    let mut positional = Vec::new();
    let mut glyphs = "a-z,A-Z".to_string();
    let mut height: u32 = 1024;
    let mut out_dir: Option<PathBuf> = None;
    let mut it = std::env::args().skip(1);
    while let Some(arg) = it.next() {
        match arg.as_str() {
            "--glyphs" => {
                glyphs = it.next().unwrap_or_else(|| die("--glyphs needs a value"))
            }
            "--height" => {
                let v = it.next().unwrap_or_else(|| die("--height needs a value"));
                height = v
                    .parse()
                    .unwrap_or_else(|_| die(format!("invalid --height: {v}")));
            }
            "--out" => {
                out_dir = Some(PathBuf::from(
                    it.next().unwrap_or_else(|| die("--out needs a value")),
                ))
            }
            "-h" | "--help" => {
                println!(
                    "usage: structure-compare <ufo_path> [--glyphs a-z,A-Z] \
                     [--height 1024] [--out DIR]\n\n\
                     Sheets land in --out, or by default in\n\
                     ~/Desktop/structure-compare/<font-name>/"
                );
                exit(0);
            }
            other => positional.push(other.to_string()),
        }
    }
    if positional.len() != 1 {
        die("expected exactly one <ufo_path>");
    }
    // Canonical path: the sheet prints it as provenance, and a relative
    // "Font.ufo" would be useless in a shared image.
    let ufo_path = PathBuf::from(&positional[0]);
    let ufo_path = std::fs::canonicalize(&ufo_path).unwrap_or(ufo_path);
    // Default output: a per-font folder on the Desktop — the tool's job is
    // producing images a human flips through, so put them where Eli looks.
    let out_dir = out_dir.unwrap_or_else(|| {
        let stem = ufo_path
            .file_stem()
            .map(|s| s.to_string_lossy().into_owned())
            .unwrap_or_else(|| "font".into());
        std::env::var_os("HOME")
            .map(PathBuf::from)
            .unwrap_or_else(|| PathBuf::from("."))
            .join("Desktop")
            .join("structure-compare")
            .join(stem)
    });
    Args {
        ufo_path,
        glyphs: parse_glyph_spec(&glyphs),
        height,
        out_dir,
    }
}

/// Case-collision-proof file key: uniXXXX for single-codepoint glyphs,
/// otherwise the name with an upper/lower disambiguator suffix.
fn file_key(glyph: &norad::Glyph, name: &str) -> String {
    if let Some(c) = glyph.codepoints.iter().next() {
        return format!("uni{:04X}", c as u32);
    }
    let suffix = if name.chars().any(|c| c.is_uppercase()) {
        ".upper"
    } else {
        ""
    };
    format!("{name}{suffix}")
}

fn trace_options(height: u32) -> img2bez::TraceOptions {
    // Mirror the CLI's single-glyph defaults (src/main.rs opts_from_cli):
    // wild profile, Otsu, grid 2, refine + auto-pre-blur on, and em_height
    // equal to the CLI's default --target-height so grid snap and cleanup
    // run at the same scale the eval harness uses.
    let mut opts = img2bez::TraceOptions::default();
    opts.profile = img2bez::Profile::Wild;
    opts.fit_accuracy = img2bez::Profile::Wild.fit_accuracy();
    opts.grid = 2;
    opts.em_height = 1088.0;
    let _ = height; // input px height is independent of em_height
    opts
}

fn main() {
    let args = parse_args();
    std::fs::create_dir_all(args.out_dir.join("render"))
        .unwrap_or_else(|e| die(format!("cannot create out dir: {e}")));

    let font = Font::load(&args.ufo_path).unwrap_or_else(|e| {
        die(format!("failed to load {}: {e}", args.ufo_path.display()))
    });
    let fb = FontBox {
        ascender: font.font_info.ascender.unwrap_or(800.0),
        descender: font.font_info.descender.unwrap_or(-200.0),
    };
    let opts = trace_options(args.height);
    let date = std::process::Command::new("date")
        .arg("+%Y-%m-%d")
        .output()
        .ok()
        .map(|o| String::from_utf8_lossy(&o.stdout).trim().to_string())
        .unwrap_or_default();

    let mut tsv = String::from(
        "glyph\tstatus\tsrc_pts\tsrc_c\tsrc_l\tsrc_contours\ttrc_pts\ttrc_c\t\
         trc_l\ttrc_contours\tgap\ttrc_kinks\ttrc_shallow\ttrc_triangle\t\
         trc_reach\tsrc_kinks\tnote\n",
    );
    let mut rows: Vec<(usize, String)> = Vec::new();
    let mut seen_keys = BTreeSet::new();
    let (mut ok, mut failed) = (0usize, 0usize);

    for name in &args.glyphs {
        let mut note = String::new();
        let src = match source_contours(&font, name) {
            Ok(c) if !c.is_empty() => c,
            Ok(_) => {
                writeln!(tsv, "{name}\tempty\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t").ok();
                failed += 1;
                continue;
            }
            Err(e) => {
                writeln!(tsv, "{name}\tno-source\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t{e}")
                    .ok();
                failed += 1;
                continue;
            }
        };
        let glyph = font.get_glyph(name.as_str()).expect("checked above");
        if !glyph.components.is_empty() {
            note = "component-flattened".into();
        }
        let key = file_key(glyph, name);
        if !seen_keys.insert(key.clone()) {
            die(format!("file key collision: {key}"));
        }

        let src_stats = analyze(&src);
        let src_path = to_bezpath(&src);

        let png = match render_input(&src_path, &fb, args.height) {
            Ok(p) => p,
            Err(e) => {
                writeln!(
                    tsv,
                    "{name}\trender-failed\t{}\t{}\t{}\t{}\t\t\t\t\t\t\t\t\t\t\t{e}",
                    src_stats.oncurves,
                    src_stats.cubics,
                    src_stats.lines,
                    src_stats.contours
                )
                .ok();
                failed += 1;
                continue;
            }
        };
        std::fs::write(args.out_dir.join("render").join(format!("{key}.png")), &png)
            .unwrap_or_else(|e| die(format!("write render png: {e}")));

        let outline = match img2bez::trace(&png, &opts) {
            Ok(o) => o,
            Err(e) => {
                writeln!(
                    tsv,
                    "{name}\ttrace-failed\t{}\t{}\t{}\t{}\t\t\t\t\t\t\t\t\t\t\t{e:?}",
                    src_stats.oncurves,
                    src_stats.cubics,
                    src_stats.lines,
                    src_stats.contours
                )
                .ok();
                failed += 1;
                continue;
            }
        };
        let trc = traced_contours(&outline);
        let trc_stats = analyze(&trc);
        let gap = trc_stats.oncurves.abs_diff(src_stats.oncurves);

        let sheet = args.out_dir.join(format!("{key}-structure.png"));
        let meta = SheetMeta {
            glyph: name,
            codepoint: glyph.codepoints.iter().next(),
            ufo_path: &args.ufo_path,
            height: args.height,
            date: &date,
            note: &note,
        };
        if let Err(e) =
            draw_sheet(&sheet, &meta, (&src, &src_stats), (&trc, &trc_stats))
        {
            note = format!("sheet-failed: {e}");
        }

        writeln!(
            tsv,
            "{name}\tok\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{note}",
            src_stats.oncurves,
            src_stats.cubics,
            src_stats.lines,
            src_stats.contours,
            trc_stats.oncurves,
            trc_stats.cubics,
            trc_stats.lines,
            trc_stats.contours,
            gap,
            trc_stats.kinks_smooth,
            trc_stats.shallow_corners,
            trc_stats.triangle_viol,
            trc_stats.reach_viol,
            src_stats.kinks_smooth,
        )
        .ok();
        rows.push((
            gap * 100
                + trc_stats.kinks_smooth * 10
                + trc_stats.triangle_viol * 10
                + trc_stats.reach_viol * 10
                + trc_stats.shallow_corners,
            format!(
                "{name:>4}  src {:>3}pts {:>2}c+{:>2}l | trc {:>3}pts {:>2}c+{:>2}l | \
                 gap {gap}  kinks {}  shallow {}  tri {}  reach {}",
                src_stats.oncurves,
                src_stats.cubics,
                src_stats.lines,
                trc_stats.oncurves,
                trc_stats.cubics,
                trc_stats.lines,
                trc_stats.kinks_smooth,
                trc_stats.shallow_corners,
                trc_stats.triangle_viol,
                trc_stats.reach_viol,
            ),
        ));
        ok += 1;
    }

    std::fs::write(args.out_dir.join("summary.tsv"), &tsv)
        .unwrap_or_else(|e| die(format!("write summary.tsv: {e}")));

    rows.sort_by(|a, b| b.0.cmp(&a.0));
    println!("structure-compare: {ok} compared, {failed} failed/skipped");
    println!("worst first (score = 100*gap + 10*(kinks+tri+reach) + shallow):");
    for (_, line) in &rows {
        println!("  {line}");
    }
    println!("sheets + summary.tsv in {}", args.out_dir.display());
}
