# img2bez

Trace bitmap glyph images to bezier contours and insert them directly into UFO font sources.

Designed for agentic AI type design pipelines — takes scanned or rendered letterforms and produces cubic outlines ready for editing or compilation into fonts.

Built on the [Linebender](https://linebender.org) ecosystem: [kurbo](https://crates.io/crates/kurbo) for bezier curve math and optimal curve fitting via [`fit_to_bezpath_opt`](https://docs.rs/kurbo/latest/kurbo/fn.fit_to_bezpath_opt.html), and [norad](https://crates.io/crates/norad) for UFO reading/writing.

## Installation

```bash
cargo install --git https://github.com/eliheuer/img2bez
```

Or to use as a library, add to your `Cargo.toml`:

```toml
[dependencies]
img2bez = { git = "https://github.com/eliheuer/img2bez" }
```

## Quick start

```bash
# Basic usage: trace a glyph image and insert it into a UFO source
img2bez --input glyph.png \
  --output MyFont.ufo \    # target UFO (must already exist)
  --name A \               # glyph name in the UFO
  --unicode 0041           # Unicode codepoint in hex

# With font metrics: scale and position the outline to match your UPM
img2bez --input glyph.png \
  --output MyFont.ufo \
  --name A \
  --unicode 0041 \
  --target-height 1024 \    # ascender - descender in font units
  --y-offset -256 \         # shift down by the descender value
  --grid 2                  # snap coordinates to a 2-unit grid

# Compare the traced output against a reference glyph
img2bez --input glyph.png \
  --output MyFont.ufo \
  --name A \
  --unicode 0041 \
  --reference MyFont.ufo/glyphs/A_.glif
```

## CLI options

| Flag | Default | Description |
|------|---------|-------------|
| `-i, --input` | required | Input image (PNG, JPEG, BMP) |
| `-o, --output` | required | Output UFO path |
| `-n, --name` | required | Glyph name |
| `-u, --unicode` | — | Unicode codepoint (hex, e.g. `0041`) |
| `-w, --width` | auto | Advance width (auto-computed from bbox if omitted) |
| `--target-height` | 1000 | Target height in font units (ascender - descender) |
| `--y-offset` | 0 | Y offset after scaling (typically the descender) |
| `--grid` | 0 | Grid size for coordinate snapping (0 = off) |
| `--accuracy` | 4.0 | Curve fitting accuracy in font units (smaller = tighter) |
| `--alphamax` | 1.0 | Corner detection (0.6-0.8 for geometric type, 1.0 for organic) |
| `--smooth` | 3 | Polygon smoothing iterations (0 = off) |
| `--chamfer` | 0 | Chamfer size (0 = off) |
| `--threshold` | Otsu | Fixed brightness threshold (0-255), overrides Otsu |
| `--invert` | false | Invert image before tracing |
| `--reference` | — | Reference `.glif` for quality evaluation |

## Library usage

```rust
use img2bez::{trace, TracingConfig};
use std::path::Path;

let config = TracingConfig {
    target_height: 1088.0,
    y_offset: -256.0,
    grid: 2,
    ..TracingConfig::default()
};

let result = trace(Path::new("glyph.png"), &config)?;
// result.paths: Vec<kurbo::BezPath>
// result.advance_width: f64
// result.contour_types: Vec<ContourType>
```

## Pipeline overview

```
 Input image
      |
 1. Threshold (Otsu or fixed) ─── bitmap.rs
      |
 2. Pixel-edge contour extraction (dual grid) ─── vectorize/decompose.rs
      |
 3. Optimal polygon approximation (DP) ─── vectorize/polygon.rs
      |
 4. Sub-pixel vertex refinement ─── vectorize/polygon.rs
      |
 5. Corner detection + curve fitting ─── vectorize/curve.rs
      |
 6. Post-processing ─── cleanup/
      ├── Contour direction (CCW outer, CW counter)
      ├── Grid snapping
      ├── H/V handle correction
      └── Optional chamfer insertion
      |
 7. Reposition + advance width ─── metrics.rs
      |
 8. UFO output ─── ufo.rs
```

## Features

- **Potrace-style pipeline**: decompose, polygon, curve — adapted for font outlines
- **Extrema-aware splitting**: curves are split at bounding-box extrema so handles naturally align H/V
- **Grid snapping**: on-curve points snap to an integer grid while off-curve handles preserve curve accuracy
- **Rayon parallelism**: contours are fitted in parallel
- **Raster IoU evaluation**: compare traced output against a reference with pixel-level accuracy

## Cargo features

| Feature | Default | Description |
|---------|---------|-------------|
| `ufo` | yes | UFO output via norad |
| `cli` | yes | Command-line binary (implies `ufo`) |

## Debug environment variables

Prefix your command with `VAR=1` to enable debug output for a single run:

```bash
IMG2BEZ_DEBUG_SPLITS=1 img2bez --input glyph.png --output MyFont.ufo --name A
```

| Variable | Effect |
|----------|--------|
| `IMG2BEZ_DEBUG_BITMAP` | Save thresholded bitmap as `debug_threshold.png` |
| `IMG2BEZ_DEBUG_PIXELS` | Print raw pixel contour stats |
| `IMG2BEZ_DEBUG_RAW_CONTOUR` | Skip polygon optimization, use raw pixel points |
| `IMG2BEZ_DEBUG_NO_ADJUST` | Skip sub-pixel vertex refinement |
| `IMG2BEZ_DEBUG_POLYGON` | Output polygon as straight lines (no curve fitting) |
| `IMG2BEZ_DEBUG_SPLITS` | Print split-point analysis per contour |
| `IMG2BEZ_DEBUG_FIT` | Print per-section curve fitting details |
| `IMG2BEZ_DEBUG_NO_CLEANUP` | Skip all post-processing |
| `IMG2BEZ_DEBUG_PIXELDIFF` | Save 1:1 pixel diff image |

## License

MIT
