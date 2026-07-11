# img2bez

[![CI](https://github.com/eliheuer/img2bez/actions/workflows/ci.yml/badge.svg)](https://github.com/eliheuer/img2bez/actions/workflows/ci.yml)
[![Apache 2.0 or MIT license.](https://img.shields.io/badge/license-Apache--2.0_OR_MIT-blue.svg)](#license)
[![Linebender Zulip chat.](https://img.shields.io/badge/Linebender-Zulip-blue?logo=Zulip)](https://xi.zulipchat.com)

img2bez traces raster glyph images into bézier outlines and writes them
directly into UFO font sources.

Most autotracers reproduce a glyph's silhouette and leave the outline
structure to a human. img2bez aims for the structure a font source
needs: points at extrema with horizontal or vertical handles, straight
segments as lines, points at inflections, smooth joins harmonized
toward G2, and the minimum number of points that render the shape.
Traces run in milliseconds on a CPU. Built on
[kurbo](https://crates.io/crates/kurbo) and
[norad](https://crates.io/crates/norad) from the
[Linebender](https://linebender.org) ecosystem.

Read the [blog post](https://elih.net/blog/img2bez) for the design
rationale and an interactive in-browser demo.

![img2bez: a raster glyph image being traced into a bézier outline with smooth points, corner points, and handles.](https://raw.githubusercontent.com/eliheuer/img2bez/main/docs/images/share-card.png)

## Install

Not yet published to crates.io. Install from GitHub for now:

```bash
cargo install --git https://github.com/eliheuer/img2bez            # the CLI
cargo add --git https://github.com/eliheuer/img2bez img2bez        # as a library
```

## Quick start

```bash
# Trace a glyph image into a UFO source (created if it doesn't exist)
img2bez --input glyph.png --output MyFont.ufo --name A --unicode 0041

# With font metrics and grid snapping
img2bez --input glyph.png --output MyFont.ufo --name A --unicode 0041 \
  --target-height 1024 --y-offset -256 --grid 2
```

`img2bez --help` is the authoritative flag list, including the `masters`
(interpolation-compatible variable-font masters) and `new-font`
subcommands. Key flags: `--profile wild|clean|photo` (source-class
preset), `--accuracy` (fit tolerance, font units), `--mode smooth|line`
(shape constraint), `--format ufo|glif|json|svg`.

## Library usage

```rust
use std::path::Path;

use img2bez::{trace_glyph_path, FontMetrics, Profile, TraceOptions};

let opts = TraceOptions::for_profile(Profile::Clean)
    .with_em_height(1088.0)
    .with_grid(2);
let metrics = FontMetrics::from_target_height(1088.0, -256.0)
    .with_advance(600.0);

let glyph = trace_glyph_path(
    Path::new("glyph.png"), "A", &['A'], &opts, &metrics,
)?;

// The same Glyph model serializes to GLIF, JSON, or SVG.
std::fs::write("A.glif", glyph.to_glif())?;

# Ok::<(), Box<dyn std::error::Error>>(())
```

Designed to embed: `trace_glyph` takes image bytes and returns a
`Glyph` with no filesystem access; output is deterministic (cache by
input hash); options are per call; a lean build
(`--no-default-features --features ufo`) drops the CLI and renderer
dependencies. [WASM bindings](wasm/) run the identical pipeline in the
browser; [mcp/](mcp/) exposes it to AI agents as an MCP server.

## How it works

```
 Input image (grayscale)
      |
 1. Sub-pixel boundary extraction — marching squares at the
    threshold iso-level; the anti-aliasing carries ~0.1px accuracy
 2. Structure detection — corners, straights, tangent points,
    extrema, inflections from the curvature signal
 3. Constrained cubic fitting — tangent directions fixed (exactly
    H/V at extrema), handle lengths solved, candidate structures
    scored against the source pixels
 4. Font cleanup — contour direction, grid snapping, H/V handle
    correction, G2 harmonization, corner reconstruction
      |
 UFO / GLIF / JSON / SVG
```

Optional learned models (tiny decision heads, a few KB each, plain Rust
inference) can replace individual structural judgment calls. They only
choose between structures the procedural pipeline proposes — all
drawing stays procedural — and they are **off by default**
(`IMG2BEZ_SITE_HEAD=1`, or `siteHead` in the wasm config). The
[demo](https://elih.net/blog/img2bez) has a live rules-vs-learned
toggle.

## Tracing quality

The eval harness traces clean renders of a reference font and scores
the result against the original UFO outlines (point count and
placement, line/curve structure, H/V handles). Across basic Latin
(a–z, A–Z, 0–9): mean structural score **0.964**, 12 glyphs exact,
62/62 pass. Try your own images in the
[demo](https://elih.net/blog/img2bez).

![A structure-compare sheet: the source font's outline structure (left) next to img2bez's trace of a rendering of it (right) — identical point counts, smooth points, corners, and handles.](https://raw.githubusercontent.com/eliheuer/img2bez/main/docs/images/structure-compare-a.png)

*A `structure-compare` sheet: the source font's outlines (left), the
trace of a 1024px rendering of them (right) — same 27 points, same
structure.*

<details>
<summary>Per-glyph scores</summary>

| Glyph | Score | Glyph | Score | Glyph | Score | Glyph | Score |
|-------|-------|-------|-------|-------|-------|-------|-------|
| i | 1.000 | j | 0.987 | 6 | 0.967 | D | 0.946 |
| o | 1.000 | q | 0.987 | 9 | 0.967 | S | 0.943 |
| v | 1.000 | z | 0.986 | B | 0.965 | b | 0.939 |
| E | 1.000 | P | 0.986 | g | 0.964 | p | 0.938 |
| F | 1.000 | w | 0.982 | U | 0.964 | l | 0.934 |
| H | 1.000 | J | 0.982 | n | 0.960 | 2 | 0.934 |
| I | 1.000 | W | 0.982 | h | 0.958 | Y | 0.931 |
| L | 1.000 | x | 0.981 | u | 0.958 | s | 0.927 |
| O | 1.000 | R | 0.981 | X | 0.958 | m | 0.922 |
| T | 1.000 | C | 0.978 | 5 | 0.958 | y | 0.918 |
| V | 1.000 | a | 0.972 | d | 0.957 | 0 | 0.915 |
| 1 | 1.000 | 4 | 0.971 | Q | 0.957 | e | 0.909 |
| 8 | 0.993 | A | 0.969 | c | 0.954 | r | 0.863 |
| f | 0.991 | 3 | 0.968 | N | 0.950 | 7 | 0.779 |
| t | 0.991 | k | 0.967 | Z | 0.950 |  |  |
| M | 0.990 | K | 0.967 | G | 0.948 |  |  |

</details>

## Cargo features

| Feature | Default | Description |
|---------|---------|-------------|
| `ufo` | yes | UFO output via norad |
| `cli` | yes | The `img2bez` binary (implies `ufo` and `render`) |
| `parallel` | yes | Per-contour parallelism via rayon |
| `render` | yes | PNG rendering and raster IoU via tiny-skia |

<details>
<summary>Debug environment variables</summary>

Prefix a command with `VAR=1` for a single run, e.g.
`IMG2BEZ_DEBUG_FIT=1 img2bez ...`:

| Variable | Effect |
|----------|--------|
| `IMG2BEZ_DEBUG_FIT` | Pipeline splits, fit errors, refine decisions |
| `IMG2BEZ_DEBUG_CORNERS` | Corner decisions; site-head zone scores |
| `IMG2BEZ_DEBUG_FLATS` | Junction-flat decisions |
| `IMG2BEZ_DEBUG_SIMPLIFY` | Point-merge accept/reject reasons |
| `IMG2BEZ_DEBUG_TANGENT` / `_WELD` / `_ARCCHAIN` | Tangent, weld, arc-chain decisions |
| `IMG2BEZ_DEBUG_NO_CLEANUP` | Skip all post-processing |
| `IMG2BEZ_DEBUG_PIXELDIFF` | Save a 1:1 pixel diff image |
| `IMG2BEZ_DEBUG_JOINT` / `_JOINT_EXTREMA` | Joint masters decisions |
| `IMG2BEZ_PHOTO_FAIR_DEV` | Photo-profile fairing threshold override |
| `IMG2BEZ_LOG` / `IMG2BEZ_LOG_DECISIONS` | Append JSONL trace/decision logs to a path (**causes file writes**) |

`IMG2BEZ_CORNER_HEAD=1` / `IMG2BEZ_SITE_HEAD=1` are opt-ins, not debug
switches: they enable the learned heads (the default pipeline is fully
procedural).

</details>

## Development

The tracer is developed against a measurable loop: render a reference
font, trace it back, score the structural match. The harness and the
`structure-compare` QA tool (side-by-side sheets like the one above,
for any UFO) live in
[eval-harness/](https://github.com/eliheuer/img2bez/tree/main/eval-harness)
(repository only, not in the published crate). Known gaps:
[docs/known-problems.md](https://github.com/eliheuer/img2bez/blob/main/docs/known-problems.md).

## Minimum supported Rust version (MSRV)

img2bez has been verified to compile with **Rust 1.88** and later.

## License

Licensed under either of

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or
  <http://www.apache.org/licenses/LICENSE-2.0>)
- MIT license ([LICENSE-MIT](LICENSE-MIT) or
  <http://opensource.org/licenses/MIT>)

at your option.

## Contribution

Unless you explicitly state otherwise, any contribution intentionally
submitted for inclusion in the work by you, as defined in the Apache-2.0
license, shall be licensed as above, without any additional terms or
conditions.
