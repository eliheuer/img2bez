# img2bez: Custom Rust Autotracer Architecture

## Design Goal

A Rust crate that takes a bitmap image of a glyph and outputs font-ready bezier contours (`Vec<kurbo::BezPath>`) — no intermediate SVG, no Python, every stage tunable for geometric type design.

**Dual use**: library (`use img2bez`) for a font editor, plus a thin CLI binary.

**Format-independent core**: The library's primary output is `Vec<kurbo::BezPath>`. Output backends (UFO via norad, fontir, etc.) are optional modules. Your font editor can consume the `BezPath`s directly without going through any file format.

---

## Crate Structure

```
img2bez/
├── Cargo.toml
├── src/
│   ├── lib.rs              # re-exports, top-level trace() function
│   ├── config.rs           # TracingConfig — all parameters in one struct
│   ├── bitmap.rs           # image loading, threshold, preprocessing
│   ├── contour.rs          # pixel contour detection, hierarchy
│   ├── corners.rs          # corner detection (the hard problem)
│   ├── simplify.rs         # RDP + kurbo curve fitting
│   ├── postprocess.rs      # grid snap, extrema, direction, chamfers
│   ├── output/
│   │   ├── mod.rs          # output backend trait/common types
│   │   └── ufo.rs          # norad Contour/Glyph output (behind "ufo" feature)
│   └── bin/
│       └── main.rs         # CLI wrapper (clap)
```

### Cargo.toml

```toml
[package]
name = "img2bez"
version = "0.1.0"
edition = "2021"
description = "Bitmap image to font-ready bezier contours"
license = "MIT"

[lib]
name = "img2bez"
path = "src/lib.rs"

[[bin]]
name = "img2bez"
path = "src/bin/main.rs"
required-features = ["cli"]

[dependencies]
image = "0.25"
imageproc = "0.25"
kurbo = "0.13"
geo = "0.28"
thiserror = "2"

# Optional: output backends
norad = { version = "0.18", features = ["kurbo"], optional = true }

# Optional: CLI
clap = { version = "4", features = ["derive"], optional = true }

[features]
default = ["ufo", "cli"]
ufo = ["norad"]       # UFO .glif output via norad
cli = ["clap", "ufo"] # CLI binary (includes UFO output)
```

Feature gates:
- **Your font editor** depends on `img2bez` with `default-features = false` — gets the core pipeline only, no clap, no norad. Consumes `Vec<kurbo::BezPath>` directly.
- **Standalone CLI** uses default features — includes norad for UFO output and clap for arg parsing.
- **Other tools** can opt into just `ufo` without `cli` if they want the norad bridge as a library.

---

## Pipeline Overview

```
                          TracingConfig
                              │
    ┌─────────┐    ┌─────────▼──────────┐    ┌──────────┐
    │  PNG/   │    │   bitmap.rs         │    │ contour  │
    │  JPEG   │───▶│   load + threshold  │───▶│ .rs      │
    │  BMP    │    │   → GrayImage       │    │ find     │
    └─────────┘    └────────────────────-┘    │ contours │
                                               └────┬─────┘
                                                    │
                                          Vec<RawContour>
                                          (pixel polylines with
                                           Outer/Hole labels)
                                                    │
                                               ┌────▼─────┐
                                               │ corners  │
                                               │ .rs      │
                                               │ detect   │
                                               │ corners  │
                                               └────┬─────┘
                                                    │
                                          Vec<AnnotatedContour>
                                          (polylines with corner
                                           marks at each point)
                                                    │
                                               ┌────▼─────┐
                                               │ simplify │
                                               │ .rs      │
                                               │ RDP →    │
                                               │ kurbo    │
                                               │ fit      │
                                               └────┬─────┘
                                                    │
                                            Vec<kurbo::BezPath>
                                            (smooth cubic beziers)
                                                    │
                                               ┌────▼─────────┐
                                               │ postprocess  │
                                               │ .rs          │
                                               │ grid snap    │
                                               │ extrema      │
                                               │ direction    │
                                               │ chamfers     │
                                               └────┬─────────┘
                                                    │
                                            Vec<kurbo::BezPath>
                                            (font-ready)
                                                    │
                                               ┌────▼─────┐
                                               │ ufo.rs   │
                                               │ BezPath  │
                                               │ → norad  │
                                               │ Contour  │
                                               └────┬─────┘
                                                    │
                                              norad::Glyph
                                              (ready to save)
```

---

## The Public API

### lib.rs — Top-Level

```rust
pub mod bitmap;
pub mod config;
pub mod contour;
pub mod corners;
pub mod postprocess;
pub mod simplify;

#[cfg(feature = "ufo")]
pub mod output;

pub use config::TracingConfig;

/// The core result type: font-ready bezier contours.
/// This is format-independent — convert to norad, fontir, or use directly.
pub struct TraceResult {
    /// The traced contours as kurbo BezPaths
    pub paths: Vec<kurbo::BezPath>,
    /// Whether each contour is an outer contour or a hole (counter)
    pub contour_types: Vec<ContourType>,
    /// Computed advance width (from bounding box + sidebearings)
    pub advance_width: f64,
}

pub enum ContourType {
    Outer,
    Counter,
}

/// Full pipeline: image path → font-ready bezier contours
pub fn trace(
    image_path: &Path,
    config: &TracingConfig,
) -> Result<TraceResult, TraceError> {
    let gray = bitmap::load_and_threshold(image_path, config)?;
    let raw = contour::detect(&gray)?;
    let annotated = corners::detect(&raw, config);
    let curves = simplify::fit(&annotated, config);
    let processed = postprocess::process(&curves, config);
    Ok(TraceResult::from_paths(processed, config))
}

/// Convenience: trace and write directly into a UFO
#[cfg(feature = "ufo")]
pub fn trace_into_ufo(
    image_path: &Path,
    glyph_name: &str,
    ufo_path: &Path,
    config: &TracingConfig,
) -> Result<(), TraceError> {
    let result = trace(image_path, config)?;
    let glyph = output::ufo::to_glyph(glyph_name, &result, config)?;
    let mut font = norad::Font::load(ufo_path)?;
    font.default_layer_mut().insert_glyph(glyph);
    font.save(ufo_path)?;
    Ok(())
}
```

The core `trace()` returns `TraceResult` containing `Vec<kurbo::BezPath>` — no dependency on any font format. The font editor consumes these directly. The `trace_into_ufo()` convenience function is only available with the `ufo` feature.

### For the Font Editor — Stage-by-Stage Access

```rust
// The editor can run stages independently (no "ufo" feature needed):

// 1. Show the thresholded bitmap
let gray = img2bez::bitmap::load_and_threshold(path, &config)?;

// 2. Show detected contours overlaid on the image
let raw_contours = img2bez::contour::detect(&gray)?;

// 3. Let user adjust corner detection, show corners as dots
let annotated = img2bez::corners::detect(&raw_contours, &config);

// 4. Show fitted curves — these are kurbo::BezPaths, render with vello
let curves = img2bez::simplify::fit(&annotated, &config);

// 5. Show post-processed outlines
let processed = img2bez::postprocess::process(&curves, &config);

// 6. Use the BezPaths directly in the editor's data model
for path in &processed {
    editor.add_contour_from_bezpath(path);
}
```

This is the real advantage of library-first. The editor can show the user intermediate results at every stage, let them tweak `TracingConfig` sliders, and re-run from any midpoint. And since the output is `kurbo::BezPath` — the same type vello renders — you can display traced contours immediately.

---

## Module Details

### config.rs

```rust
/// All tracing parameters in one struct.
/// Designed to be serializable (for saving presets) and
/// adjustable at runtime (for editor sliders).
#[derive(Debug, Clone)]
pub struct TracingConfig {
    // -- Bitmap stage --
    /// Threshold method for converting to binary
    pub threshold: ThresholdMethod,

    // -- Contour stage --
    /// Minimum contour area in pixels (filter speckles)
    pub min_contour_area: f64,

    // -- Corner detection --
    /// Angle change threshold for corner detection (radians).
    /// Points where the polyline turns more than this are marked as corners.
    /// Lower = more corners. ~0.5 (30°) is a good starting point for
    /// geometric type. ~1.0 (57°) for more organic shapes.
    pub corner_angle_threshold: f64,

    /// Window size for angle computation (number of neighboring points).
    /// Larger = smoother angle estimates, fewer false corners.
    pub corner_window: usize,

    // -- Curve fitting --
    /// Accuracy for kurbo fit_to_bezpath_opt (Frechet distance tolerance
    /// in font units). Smaller = more points, closer fit.
    /// 1.0 is a good starting point for UPM 1024.
    pub fit_accuracy: f64,

    /// RDP simplification epsilon (in pixel coordinates, before scaling).
    /// Applied before curve fitting to reduce the polyline.
    pub rdp_epsilon: f64,

    // -- Post-processing --
    /// Grid size for coordinate snapping. 0 = no snapping.
    pub grid: i32,

    /// Whether to insert points at curve extrema
    pub add_extrema: bool,

    /// Whether to correct contour direction (CCW outer, CW counter)
    pub fix_direction: bool,

    /// Chamfer size. 0 = no chamfers. 16 = Virtua Grotesk Regular.
    pub chamfer_size: f64,

    /// Minimum edge length to chamfer (edges shorter than this are skipped).
    /// Typically chamfer_size * 2.5 or so.
    pub chamfer_min_edge: f64,

    // -- Output --
    /// Advance width. If None, computed from contour bounding box + sidebearings.
    pub advance_width: Option<f64>,

    /// Left sidebearing. Used when advance_width is None.
    pub lsb: f64,

    /// Right sidebearing. Used when advance_width is None.
    pub rsb: f64,

    /// Target height in font units (ascender - descender, typically).
    /// The traced contour is scaled so the image height maps to this value.
    pub target_height: f64,

    /// Y offset after scaling (to align baseline).
    /// If the glyph sits on the baseline, this is typically the descender
    /// value (negative), so the bottom of the image maps to the descender.
    pub y_offset: f64,

    /// Unicode codepoints to assign
    pub codepoints: Vec<char>,
}

pub enum ThresholdMethod {
    /// Fixed brightness threshold (0-255)
    Fixed(u8),
    /// Otsu's method (automatic)
    Otsu,
}

impl Default for TracingConfig {
    fn default() -> Self {
        Self {
            threshold: ThresholdMethod::Otsu,
            min_contour_area: 100.0,
            corner_angle_threshold: 0.5,    // ~30 degrees
            corner_window: 5,
            fit_accuracy: 1.0,
            rdp_epsilon: 2.0,
            grid: 2,
            add_extrema: true,
            fix_direction: true,
            chamfer_size: 0.0,              // off by default
            chamfer_min_edge: 40.0,
            advance_width: None,
            lsb: 50.0,
            rsb: 50.0,
            target_height: 1088.0,          // 832 - (-256) for VG
            y_offset: -256.0,              // descender
            codepoints: vec![],
        }
    }
}

impl TracingConfig {
    /// Preset for Virtua Grotesk Regular
    pub fn virtua_grotesk_regular() -> Self {
        Self {
            grid: 2,
            chamfer_size: 16.0,
            chamfer_min_edge: 40.0,
            target_height: 1088.0,
            y_offset: -256.0,
            corner_angle_threshold: 0.4,    // tighter for geometric
            fit_accuracy: 0.5,              // tighter for clean curves
            ..Default::default()
        }
    }

    /// Preset for Virtua Grotesk Bold
    pub fn virtua_grotesk_bold() -> Self {
        Self {
            chamfer_size: 16.0,             // adjust if Bold chamfers differ
            ..Self::virtua_grotesk_regular()
        }
    }
}
```

### bitmap.rs

```rust
use image::{GrayImage, ImageReader};
use imageproc::contrast::otsu_level;

pub fn load_and_threshold(
    path: &Path,
    config: &TracingConfig,
) -> Result<GrayImage, TraceError> {
    let img = ImageReader::open(path)?.decode()?.into_luma8();

    let threshold = match config.threshold {
        ThresholdMethod::Fixed(t) => t,
        ThresholdMethod::Otsu => otsu_level(&img),
    };

    // Binary image: 255 = foreground (glyph), 0 = background
    let binary = imageproc::contrast::threshold(&img, threshold);
    Ok(binary)
}
```

Short module. The main design decision is which pixels are "foreground." Convention: white glyph on black background, or auto-detect based on border pixel color (if the border is mostly white, invert).

### contour.rs

```rust
use imageproc::contours::{find_contours, BorderType, Contour as IpContour};

/// Intermediate type: a polyline in pixel coordinates with classification
pub struct RawContour {
    /// Points in pixel coordinates (y=0 is top of image)
    pub points: Vec<(f64, f64)>,
    /// Is this an outer contour or a hole (counter)?
    pub is_outer: bool,
    /// Index of the parent contour (for nested shapes)
    pub parent: Option<usize>,
}

pub fn detect(gray: &GrayImage) -> Result<Vec<RawContour>, TraceError> {
    let ip_contours: Vec<IpContour<i32>> = find_contours::<i32>(gray);

    let (w, h) = gray.dimensions();

    let mut result: Vec<RawContour> = ip_contours
        .iter()
        .filter(|c| c.points.len() >= 3)
        .map(|c| {
            let points = c.points.iter()
                .map(|p| (p.x as f64, p.y as f64))
                .collect();

            RawContour {
                points,
                is_outer: c.border_type == BorderType::Outer,
                parent: c.parent,
            }
        })
        .collect();

    // Filter by minimum area
    result.retain(|c| polygon_area(&c.points).abs() > MIN_AREA);

    Ok(result)
}

fn polygon_area(pts: &[(f64, f64)]) -> f64 {
    // Shoelace formula
    let n = pts.len();
    (0..n).map(|i| {
        let j = (i + 1) % n;
        pts[i].0 * pts[j].1 - pts[j].0 * pts[i].1
    }).sum::<f64>() / 2.0
}
```

### corners.rs — The Hard Problem

This is where you'd spend the most iteration time. The basic idea:

```rust
/// A polyline point annotated with whether it's a corner or smooth
pub struct AnnotatedContour {
    pub points: Vec<AnnotatedPoint>,
    pub is_outer: bool,
    pub parent: Option<usize>,
}

pub struct AnnotatedPoint {
    pub x: f64,
    pub y: f64,
    pub is_corner: bool,
}

pub fn detect(
    contours: &[RawContour],
    config: &TracingConfig,
) -> Vec<AnnotatedContour> {
    contours.iter().map(|c| {
        let annotated_points = annotate_corners(
            &c.points,
            config.corner_angle_threshold,
            config.corner_window,
        );
        AnnotatedContour {
            points: annotated_points,
            is_outer: c.is_outer,
            parent: c.parent,
        }
    }).collect()
}

fn annotate_corners(
    points: &[(f64, f64)],
    threshold: f64,
    window: usize,
) -> Vec<AnnotatedPoint> {
    let n = points.len();
    points.iter().enumerate().map(|(i, &(x, y))| {
        // Compute turning angle using neighbors at distance `window`
        let prev = points[(i + n - window) % n];
        let next = points[(i + window) % n];

        let v_in = (x - prev.0, y - prev.1);
        let v_out = (next.0 - x, next.1 - y);

        let angle = angle_between(v_in, v_out);

        AnnotatedPoint {
            x,
            y,
            is_corner: angle > threshold,
        }
    }).collect()
}

fn angle_between(a: (f64, f64), b: (f64, f64)) -> f64 {
    let dot = a.0 * b.0 + a.1 * b.1;
    let cross = a.0 * b.1 - a.1 * b.0;
    cross.atan2(dot).abs()
}
```

**Why this is hard and where you'd iterate:**

The `window` parameter controls how far you look ahead/behind to compute the turning angle. Too small (1-2) and every pixel staircase step looks like a corner. Too large (20+) and real corners get smoothed out. The right value depends on image resolution.

**Strategies to try:**

1. **Multi-scale**: Compute turning angles at several window sizes. A real corner shows up at all scales; noise only shows at small scales.

2. **Curvature-based**: Instead of raw angle, compute discrete curvature (angle / arc length). Corners have high curvature regardless of resolution.

3. **Template matching for geometric fonts**: VG corners are almost always 90° (before chamfer). You could look specifically for right-angle turns and 45° turns, rejecting everything else. This is "cheating" in the best way — using prior knowledge about your design system.

4. **User-assisted**: In the font editor, show the detected corners as dots. Let the user click to add/remove corners, then re-fit. This is probably the practical endgame — automated detection gets you 90% of the way, the user fixes the last 10%.

### simplify.rs

```rust
use geo::Simplify;
use kurbo::{BezPath, PathEl, Point, simplify::SimplifyBezPath, fit_to_bezpath_opt};

pub fn fit(
    contours: &[AnnotatedContour],
    config: &TracingConfig,
) -> Vec<BezPath> {
    contours.iter().map(|c| fit_one(c, config)).collect()
}

fn fit_one(contour: &AnnotatedContour, config: &TracingConfig) -> BezPath {
    // Step 1: Scale pixel coords → font units
    let scaled: Vec<(f64, f64)> = contour.points.iter().map(|p| {
        scale_point(p.x, p.y, config)
    }).collect();

    // Step 2: Split at corners into segments
    let corner_indices: Vec<usize> = contour.points.iter()
        .enumerate()
        .filter(|(_, p)| p.is_corner)
        .map(|(i, _)| i)
        .collect();

    if corner_indices.is_empty() {
        // No corners: fit the entire contour as one smooth curve
        return fit_smooth_contour(&scaled, config.fit_accuracy);
    }

    // Step 3: Build BezPath with lines at corners, curves between
    let mut path = BezPath::new();
    let first = scaled[corner_indices[0]];
    path.move_to(Point::new(first.0, first.1));

    for i in 0..corner_indices.len() {
        let start = corner_indices[i];
        let end = corner_indices[(i + 1) % corner_indices.len()];

        // Extract the segment between these two corners
        let segment = extract_segment(&scaled, start, end);

        if segment.len() <= 2 {
            // Short segment: just a straight line to the next corner
            let p = scaled[end];
            path.line_to(Point::new(p.0, p.1));
        } else {
            // Fit a curve through this segment
            let sub_path = fit_segment_to_cubics(&segment, config.fit_accuracy);
            // Append the curve elements (skip the initial MoveTo)
            for el in sub_path.elements().iter().skip(1) {
                path.push(*el);
            }
        }
    }

    path.close_path();
    path
}

fn fit_smooth_contour(points: &[(f64, f64)], accuracy: f64) -> BezPath {
    // Convert polyline to a BezPath of line segments
    let mut line_path = BezPath::new();
    if let Some(&first) = points.first() {
        line_path.move_to(Point::new(first.0, first.1));
        for &(x, y) in &points[1..] {
            line_path.line_to(Point::new(x, y));
        }
        line_path.close_path();
    }

    // Simplify using kurbo's optimal algorithm
    let simple = SimplifyBezPath::new(line_path.elements().iter().copied());
    fit_to_bezpath_opt(&simple, accuracy)
}

fn fit_segment_to_cubics(points: &[(f64, f64)], accuracy: f64) -> BezPath {
    // Same idea but for an open segment between two corners
    let mut line_path = BezPath::new();
    if let Some(&first) = points.first() {
        line_path.move_to(Point::new(first.0, first.1));
        for &(x, y) in &points[1..] {
            line_path.line_to(Point::new(x, y));
        }
    }
    let simple = SimplifyBezPath::new(line_path.elements().iter().copied());
    fit_to_bezpath_opt(&simple, accuracy)
}

fn scale_point(px_x: f64, px_y: f64, config: &TracingConfig) -> (f64, f64) {
    // Pixel coords: (0,0) = top-left, y increases downward
    // Font coords: y increases upward, baseline at y=0
    //
    // Scale so image height → target_height, then shift by y_offset
    let scale = config.target_height / IMAGE_HEIGHT;  // IMAGE_HEIGHT from contour detection
    let x = px_x * scale;
    let y = (IMAGE_HEIGHT - px_y) * scale + config.y_offset;  // flip Y
    (x, y)
}
```

**The key insight**: Split at detected corners, then fit curves to each smooth segment independently. Corners become `line_to` junctions. Smooth arcs between corners get kurbo's optimal cubic fitting. This is much better than running the simplifier on the entire jagged polyline, because:

- Corners are preserved exactly (not smoothed over)
- Each smooth arc gets fit independently (better accuracy per segment)
- Point count stays minimal (one curve per smooth arc, line segments at corners)

### postprocess.rs

```rust
use kurbo::{BezPath, PathEl, Point, CubicBez};

/// Apply all post-processing steps to a set of traced contours
pub fn process(paths: &[BezPath], config: &TracingConfig) -> Vec<BezPath> {
    let mut result: Vec<BezPath> = paths.to_vec();

    if config.add_extrema {
        result = result.iter().map(insert_extrema).collect();
    }

    if config.grid > 0 {
        result = result.iter().map(|p| snap_to_grid(p, config.grid)).collect();
    }

    if config.fix_direction {
        result = fix_contour_directions(&result);
    }

    if config.chamfer_size > 0.0 {
        result = result.iter()
            .map(|p| add_chamfers(p, config.chamfer_size, config.chamfer_min_edge))
            .collect();
    }

    result
}

fn snap_to_grid(path: &BezPath, grid: i32) -> BezPath {
    let g = grid as f64;
    BezPath::from_vec(
        path.elements().iter().map(|el| match el {
            PathEl::MoveTo(p) => PathEl::MoveTo(snap_pt(*p, g)),
            PathEl::LineTo(p) => PathEl::LineTo(snap_pt(*p, g)),
            PathEl::CurveTo(p1, p2, p3) => PathEl::CurveTo(
                snap_pt(*p1, g), snap_pt(*p2, g), snap_pt(*p3, g)
            ),
            PathEl::ClosePath => PathEl::ClosePath,
            other => *other,
        }).collect()
    )
}

fn snap_pt(p: Point, grid: f64) -> Point {
    Point::new(
        (p.x / grid).round() * grid,
        (p.y / grid).round() * grid,
    )
}

fn path_signed_area(path: &BezPath) -> f64 {
    // Flatten to line segments, compute signed area via shoelace
    let mut area = 0.0;
    let mut first = Point::ZERO;
    let mut current = Point::ZERO;

    for el in path.elements() {
        match el {
            PathEl::MoveTo(p) => { first = *p; current = *p; }
            PathEl::LineTo(p) => {
                area += current.x * p.y - p.x * current.y;
                current = *p;
            }
            PathEl::CurveTo(_, _, p) => {
                // Approximate: use on-curve points only
                // (for area sign detection this is sufficient)
                area += current.x * p.y - p.x * current.y;
                current = *p;
            }
            PathEl::ClosePath => {
                area += current.x * first.y - first.x * current.y;
            }
            _ => {}
        }
    }
    area / 2.0
}

fn fix_contour_directions(paths: &[BezPath]) -> Vec<BezPath> {
    // Outer contours (positive area after correction) should be CCW (positive)
    // Hole contours (contained within outers) should be CW (negative)
    //
    // Simple heuristic: largest contour is outer, rest are holes.
    // For proper nesting, use the parent info from contour detection.
    paths.iter().enumerate().map(|(i, path)| {
        let area = path_signed_area(path);
        let should_be_positive = i == 0; // first = outer (TODO: use hierarchy)

        if (area > 0.0) != should_be_positive {
            reverse_bezpath(path)
        } else {
            path.clone()
        }
    }).collect()
}

fn reverse_bezpath(path: &BezPath) -> BezPath {
    // Reverse a closed BezPath by reversing element order and
    // swapping control point order in curves
    // (kurbo may have a built-in for this — check BezPath::reverse())
    todo!("implement or find in kurbo")
}

fn insert_extrema(path: &BezPath) -> BezPath {
    // Walk each CurveTo, find t values where dx/dt=0 or dy/dt=0,
    // split the curve at those t values using CubicBez::subdivide()
    todo!("implement using kurbo::CubicBez")
}

fn add_chamfers(path: &BezPath, size: f64, min_edge: f64) -> BezPath {
    // Walk the path, at each LineTo→LineTo junction where the
    // turning angle is significant, replace the corner with two
    // LineTo points offset by `size` along each edge
    todo!("implement — see the algorithm in img2bez-pipeline.md")
}
```

### output/ufo.rs (behind `#[cfg(feature = "ufo")]`)

```rust
use kurbo::{BezPath, PathEl, Point};
use norad::{Contour, ContourPoint, Glyph, PointType};
use crate::{TraceResult, TracingConfig, TraceError};

/// Convert a TraceResult to a norad Glyph
pub fn to_glyph(
    name: &str,
    result: &TraceResult,
    config: &TracingConfig,
) -> Result<Glyph, TraceError> {
    let mut glyph = Glyph::new(name);
    glyph.width = result.advance_width;

    // Set codepoints
    for &cp in &config.codepoints {
        glyph.codepoints.insert(cp);
    }

    // Convert each BezPath to a norad Contour
    for path in &result.paths {
        let contour = bezpath_to_contour(path)?;
        glyph.contours.push(contour);
    }

    Ok(glyph)
}

fn bezpath_to_contour(path: &BezPath) -> Result<Contour, TraceError> {
    let elements = path.elements();
    let mut points = Vec::new();

    // UFO convention for closed contours: the points list doesn't have
    // an explicit MoveTo — the first point's type indicates the segment
    // type of the *closing* segment (the implicit segment from last
    // point back to first point).
    //
    // For a contour that's all lines: every point is PointType::Line.
    // For a contour ending with a curve back to start: the first point
    // is PointType::Curve (or Line), and the last off-curves belong
    // to the closing segment.

    let mut skip_first_move = true;

    for el in elements {
        match el {
            PathEl::MoveTo(p) => {
                if skip_first_move {
                    skip_first_move = false;
                    // We'll handle the first point's type after
                    // we know what the last segment type is
                    continue;
                }
            }
            PathEl::LineTo(p) => {
                points.push(ContourPoint::new(
                    p.x, p.y, PointType::Line, false, None, None,
                ));
            }
            PathEl::CurveTo(p1, p2, p3) => {
                points.push(ContourPoint::new(
                    p1.x, p1.y, PointType::OffCurve, false, None, None,
                ));
                points.push(ContourPoint::new(
                    p2.x, p2.y, PointType::OffCurve, false, None, None,
                ));
                points.push(ContourPoint::new(
                    p3.x, p3.y, PointType::Curve, true, None, None,
                ));
            }
            PathEl::ClosePath => {
                // The closing segment is implicit in UFO.
                // The first point needs to be re-typed based on what
                // the closing segment is (line or curve).
            }
            _ => {}
        }
    }

    // Now handle the first point. In kurbo, the first element is
    // MoveTo(first_point), and the closing segment goes from the
    // last on-curve point back to first_point. The type of that
    // closing segment determines the first point's type.
    if let Some(PathEl::MoveTo(p)) = elements.first() {
        // Look at the last segment type to determine the closing type.
        // If the path ends with LineTo → ClosePath, the close is a line.
        // If the path ends with CurveTo → ClosePath, the close is a curve.
        let last_segment_is_line = matches!(
            elements.iter().rev().find(|e| !matches!(e, PathEl::ClosePath)),
            Some(PathEl::LineTo(_)) | Some(PathEl::MoveTo(_))
        );

        let typ = if last_segment_is_line {
            PointType::Line
        } else {
            PointType::Curve
        };

        // Insert the first point at the beginning
        points.insert(0, ContourPoint::new(
            p.x, p.y, typ, false, None, None,
        ));
    }

    Ok(Contour::new(points, None))
}

fn compute_width_from_bounds(paths: &[BezPath], lsb: f64, rsb: f64) -> f64 {
    let mut min_x = f64::MAX;
    let mut max_x = f64::MIN;
    for path in paths {
        let bbox = path.bounding_box();
        min_x = min_x.min(bbox.x0);
        max_x = max_x.max(bbox.x1);
    }
    // Shift contours so left edge = lsb, then width = max_x + rsb
    (max_x - min_x) + lsb + rsb
}
```

### bin/main.rs — CLI

```rust
use clap::Parser;
use img2bez::{trace_into_ufo, TracingConfig};
use std::path::PathBuf;

#[derive(Parser)]
#[command(name = "img2bez", about = "Bitmap image to font-ready bezier contours")]
struct Cli {
    /// Input image path (PNG, JPEG, BMP)
    #[arg(short, long)]
    input: PathBuf,

    /// Output UFO path (will insert/replace the glyph)
    #[arg(short, long)]
    output: PathBuf,

    /// Glyph name
    #[arg(short, long)]
    name: String,

    /// Unicode codepoint (e.g. "003F" for ?)
    #[arg(short = 'u', long)]
    unicode: Option<String>,

    /// Advance width (auto-computed from bounds if omitted)
    #[arg(short = 'w', long)]
    width: Option<f64>,

    /// Grid size for coordinate snapping (default: 2)
    #[arg(long, default_value = "2")]
    grid: i32,

    /// Chamfer size (0 = off, 16 = VG Regular)
    #[arg(long, default_value = "0")]
    chamfer: f64,

    /// Curve fitting accuracy in font units (smaller = more precise)
    #[arg(long, default_value = "1.0")]
    accuracy: f64,

    /// Corner detection angle threshold in degrees
    #[arg(long, default_value = "30")]
    corner_threshold: f64,

    /// Use Virtua Grotesk preset
    #[arg(long)]
    vg: bool,
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let cli = Cli::parse();

    let mut config = if cli.vg {
        TracingConfig::virtua_grotesk_regular()
    } else {
        TracingConfig::default()
    };

    config.grid = cli.grid;
    config.chamfer_size = cli.chamfer;
    config.fit_accuracy = cli.accuracy;
    config.corner_angle_threshold = cli.corner_threshold.to_radians();
    config.advance_width = cli.width;

    if let Some(ref u) = cli.unicode {
        let cp = u32::from_str_radix(u, 16)?;
        if let Some(c) = char::from_u32(cp) {
            config.codepoints = vec![c];
        }
    }

    trace_into_ufo(&cli.input, &cli.name, &cli.output, &config)?;

    println!("Traced '{}' into {}", cli.name, cli.output.display());
    Ok(())
}
```

**Usage:**
```bash
# Standalone: trace image into a UFO
img2bez -i question.png -o sources/VirtuaGrotesk-Regular.ufo -n question -u 003F --vg

# As a library in your font editor: use img2bez::trace() directly,
# get Vec<kurbo::BezPath> back, no file I/O needed
```

---

## What You're Actually Building vs What Exists

| Piece | Exists in ecosystem | You write |
|-------|-------------------|-----------|
| Image loading | `image` crate | ~10 lines wrapping it |
| Thresholding | `imageproc::contrast` | ~5 lines |
| Contour detection | `imageproc::contours` | ~30 lines wrapping + filtering |
| **Corner detection** | **Nothing suitable** | **~100 lines, the core R&D** |
| RDP simplification | `geo::Simplify` | ~10 lines wrapping |
| **Curve fitting** | `kurbo::fit_to_bezpath_opt` | **~80 lines** (split-at-corners + fit) |
| Grid snapping | Nothing | ~15 lines |
| Extrema insertion | Nothing in Rust | ~60 lines |
| Direction fixing | Nothing in Rust | ~30 lines |
| Chamfer insertion | Nothing | ~50 lines |
| **BezPath→norad Contour** | **Missing bridge** | **~60 lines** (ufo feature) |
| CLI | `clap` | ~50 lines (cli feature) |

**Total custom code: ~500 lines.** The estimate holds up. The ecosystem does the heavy math (curve fitting, contour detection). You write the glue, the corner detection, and the font-specific post-processing. The core library (no features) is ~400 lines; the norad bridge and CLI add ~100 more.

**The one piece that's genuinely R&D**: corner detection. Everything else is mechanical. Budget half your time there.

---

## Risks and Mitigations

**Risk: kurbo's SimplifyBezPath chokes on pixel staircases**
Mitigation: RDP first (via `geo`) to reduce 2000 pixel points to ~50 significant points, then kurbo fits curves through those. If that's still too noisy, try bilateral filtering on the source image before thresholding.

**Risk: corner detection is never quite right**
Mitigation: Make it interactive in the editor. Auto-detect gets 90%, user clicks to fix the rest. The library API supports this — the editor calls `corners::detect()`, shows results, user edits, then calls `simplify::fit()` with the corrected corners.

**Risk: norad contour point ordering is tricky**
Mitigation: The UFO "last point first" convention for closed contours is subtle. The `bezpath_to_contour()` function above handles it. Write a test that round-trips: load an existing VG glyph with norad, convert to kurbo BezPath, convert back to norad Contour, save, and verify the .glif is byte-identical.

**Risk: chamfers interact with curve fitting**
Mitigation: Chamfers are post-processing — they only apply to line-line junctions, never to curves. The pipeline order (fit curves → snap grid → add extrema → fix direction → add chamfers) ensures chamfers see clean line segments.
