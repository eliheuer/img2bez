# Image-to-Bezier Pipeline: Research & Implementation Plan

## The Problem

Drawing glyph outlines by having an LLM write bezier coordinates from scratch is too slow, token-heavy, and imprecise. We need a two-step pipeline:

1. **Autotrace**: Convert a reference image to vector contours (fast, mechanical)
2. **Adapt**: Edit those contours to match VG's style, proportions, weight, and grid (font-aware, can be partially automated)

This document covers every viable approach, from "works today in 5 minutes" to "custom Rust tool from scratch."

---

## Part 1: Autotracing — Image to Raw Vectors

### Option A: VTracer (Recommended Starting Point)

**What**: Rust-based bitmap-to-SVG tracer. MIT licensed, actively maintained (Feb 2026).

**Why it's best for us**:
- Rust-native (matches our fontc/designbot toolchain)
- MIT license (no GPL concerns)
- Python bindings available: `pip install vtracer`
- Tunable curve quality via `splice_threshold` and `corner_threshold`
- `--colormode bw --mode spline` produces cubic beziers suitable for fonts

**Quick test**:
```bash
pip install vtracer
```
```python
import vtracer
vtracer.convert_image_to_svg_py(
    "reference_glyph.png",
    "traced.svg",
    colormode="bw",
    mode="spline",
    filter_speckle=4,
    corner_threshold=60,
    splice_threshold=45
)
```

**Limitations**: Output is SVG only (no direct UFO). Python API only accepts file paths, not in-memory images. No font awareness (contour direction, extrema, grid).

### Option B: Potrace (Battle-Tested Fallback)

**What**: The classic bitmap-to-vector tracer (2003). GPL v2+. Used by FontForge, Inkscape, everywhere.

**Why consider it**:
- Proven track record for font tracing (FontForge's default tracer)
- Python bindings: `pip install pypotrace`
- Outputs cubic beziers (compatible with UFO/CFF)
- Fine-tunable: `alphamax` for corner detection, `opttolerance` for curve joining

**Key parameters for glyph tracing**:
```python
import potrace

# Create bitmap from image, then:
path = potrace.Bitmap(bitmap).trace(
    alphamax=1.0,    # corner detection (0=all corners, >1.33=all smooth)
    opttolerance=0.2, # curve segment joining tolerance
    turdsize=2        # speckle suppression
)
# path.curves_tree contains the traced bezier paths
```

**Limitations**: GPL license. Input must be binary (PBM/BMP). Less compact output than VTracer.

### Option C: img2bez — Custom Rust Autotracer (The Ambitious Route)

Build from scratch using the linebender ecosystem. This gives maximum control over the pipeline. See `context/img2bez-architecture.md` for the full architecture.

**The stack**:
```
image (0.25)        — load PNG/JPEG, convert to grayscale
imageproc (0.25)    — threshold (Otsu), morphological cleanup, find_contours()
geo (0.28)          — RDP polyline simplification
kurbo (0.13)        — cubic bezier curve fitting (Raph Levien's algorithms)
norad (0.18)        — (optional) write to UFO .glif files
```

**Pipeline**:
```
PNG → grayscale → threshold → find_contours() → pixel polylines
    → scale to font units + flip Y
    → RDP simplification (reduce 1000s of points to key corners)
    → kurbo::SimplifyBezPath → fit_to_bezpath_opt() (near-optimal cubic fitting)
    → Vec<kurbo::BezPath> (core output — format-independent)
    → optional: norad::Contour → write .glif
```

**Key advantage**: `kurbo::fit_to_bezpath_opt()` uses Raph Levien's O(n^-6) error convergence algorithm — mathematically superior to every other curve fitter available. Produces near-minimal point counts with excellent curve quality.

**The missing bridge** (norad has `Contour::to_kurbo()` but no `from_kurbo()`):
```rust
fn bezpath_to_contour(path: &kurbo::BezPath) -> norad::Contour {
    let mut points = Vec::new();
    for el in path.elements() {
        match el {
            PathEl::MoveTo(p) => points.push(
                ContourPoint::new(p.x, p.y, PointType::Line, false, None, None)),
            PathEl::LineTo(p) => points.push(
                ContourPoint::new(p.x, p.y, PointType::Line, false, None, None)),
            PathEl::CurveTo(p1, p2, p3) => {
                points.push(ContourPoint::new(p1.x, p1.y, PointType::OffCurve, false, None, None));
                points.push(ContourPoint::new(p2.x, p2.y, PointType::OffCurve, false, None, None));
                points.push(ContourPoint::new(p3.x, p3.y, PointType::Curve, true, None, None));
            }
            PathEl::ClosePath => {} // norad contours are implicitly closed
            _ => {}
        }
    }
    Contour::new(points, None, None)
}
```

**Effort estimate**: ~500 lines of Rust for a minimal working tool. Most of the hard math is in kurbo. The imageproc `find_contours()` gives you the pixel-level polylines with `Outer`/`Hole` classification (maps directly to UFO contour direction).

**Cargo.toml**:
```toml
[dependencies]
image = "0.25"
imageproc = "0.25"
kurbo = "0.13"
norad = "0.18"
geo = "0.28"
```

### Comparison

| | VTracer | Potrace | Custom Rust |
|---|---|---|---|
| **Setup time** | 5 min | 10 min | 2-3 days |
| **Output quality** | Good, tunable | Good, proven | Best (kurbo) |
| **Output format** | SVG (need conversion) | SVG/EPS (need conversion) | kurbo::BezPath (+ optional UFO) |
| **Font awareness** | None | None | Full (custom) |
| **License** | MIT | GPL v2 | Yours |
| **Point economy** | Decent | Moderate | Best |

**Recommendation**: Start with VTracer for immediate results. Build img2bez as a weekend project for production quality.

---

## Part 2: SVG-to-UFO Conversion

If using VTracer or Potrace (SVG output), you need to convert to GLIF format.

### Python Path (fontTools + ufoLib2)

```python
import ufoLib2
from fontTools.svgLib import SVGPath
from fontTools.pens.pointPen import SegmentToPointPen
from fontTools.pens.transformPen import TransformPen
from fontTools.misc.transform import Transform

# Parse SVG
svg = SVGPath("traced_glyph.svg")

# Open UFO, create glyph
font = ufoLib2.Font.open('sources/VirtuaGrotesk-Regular.ufo')
glyph = font.newGlyph("question")
glyph.width = 600
glyph.unicodes = [0x003F]

# Draw SVG into glyph with coordinate transform
# SVG is top-down, font is bottom-up. Scale to match UPM.
pen = glyph.getPen()
t = Transform().translate(0, 832).scale(1, -1)  # flip Y, shift to ascender
transform_pen = TransformPen(pen, t)
svg.draw(transform_pen)

font.save()
```

### Rust Path (svg2glif crate)

The `svg2glif` crate (by Santhosh Thottingal, Dec 2025) handles SVG→GLIF conversion with proper coordinate system transformation built in. ~187 microseconds per conversion.

```rust
// Cargo.toml: svg2glif = "0.1"
```

---

## Part 3: Post-Processing — Making Traced Outlines Font-Ready

This is where the "adapt to VG style" happens. Each step can be automated.

### Step 1: Grid Snap (coordinates to multiples of 2)

```python
from fontTools.pens.roundingPen import RoundingPen

def snap(v):
    return round(v / 2) * 2

pen = glyph.getPen()
grid_pen = RoundingPen(pen, roundFunc=snap)
# Replay traced outline through grid_pen
```

### Step 2: Add Extrema Points

Font outlines need on-curve points at every H/V extreme of every curve. Without these, hinting breaks and rasterization suffers.

```python
from fontTools.misc.bezierTools import splitCubicAtT
import math

def find_cubic_extrema_t(p0, p1, p2, p3):
    """Find t values where cubic has H/V extrema."""
    ts = []
    for axis in (0, 1):
        a, b, c, d = p0[axis], p1[axis], p2[axis], p3[axis]
        A = -3*a + 9*b - 9*c + 3*d
        B = 6*a - 12*b + 6*c
        C = -3*a + 3*b
        if abs(A) < 1e-10:
            if abs(B) > 1e-10:
                t = -C / B
                if 0.01 < t < 0.99:
                    ts.append(t)
        else:
            disc = B*B - 4*A*C
            if disc >= 0:
                sq = math.sqrt(disc)
                for t in [(-B + sq)/(2*A), (-B - sq)/(2*A)]:
                    if 0.01 < t < 0.99:
                        ts.append(t)
    return sorted(set(ts))
```

### Step 3: Fix Contour Direction

UFO convention: outer contours CCW (positive area), counters CW (negative area).

```python
from fontTools.pens.reverseContourPen import ReverseContourPen

def contour_area(points):
    """Signed area via shoelace. Positive=CCW, Negative=CW."""
    on_curve = [(p.x, p.y) for p in points if p.type is not None]
    area = sum(
        on_curve[i][0] * on_curve[(i+1) % len(on_curve)][1] -
        on_curve[(i+1) % len(on_curve)][0] * on_curve[i][1]
        for i in range(len(on_curve))
    ) / 2.0
    return area
```

### Step 4: Add 16-Unit Chamfers

The signature VG feature. Every sharp line-line corner gets a 45-degree bevel.

```python
def add_chamfer(corner, prev_pt, next_pt, size=16):
    """Replace a corner with two chamfer points."""
    import math
    dx_in = corner[0] - prev_pt[0]
    dy_in = corner[1] - prev_pt[1]
    len_in = math.hypot(dx_in, dy_in)

    dx_out = next_pt[0] - corner[0]
    dy_out = next_pt[1] - corner[1]
    len_out = math.hypot(dx_out, dy_out)

    if len_in < size * 2 or len_out < size * 2:
        return [corner]  # too short to chamfer

    # Cut points
    before = (
        round((corner[0] - size * dx_in / len_in) / 2) * 2,
        round((corner[1] - size * dy_in / len_in) / 2) * 2,
    )
    after = (
        round((corner[0] + size * dx_out / len_out) / 2) * 2,
        round((corner[1] + size * dy_out / len_out) / 2) * 2,
    )
    return [before, after]
```

Verified against actual VG glyphs: the "I" glyph has 8 points (4 corners x 2 chamfer points), matching the expected pattern.

### Step 5: Simplify / Remove Redundant Points

Autotraced outlines typically have 5-10x more points than needed. Target the economy of existing VG glyphs: "A" has 24 points, "O" has 24 points, "I" has 8 points.

Options:
- `beziers.py` library: `BezierPath.tidy()` does extrema + simplify + cleanup in one call
- kurbo `SimplifyBezPath` (Rust): near-optimal simplification
- Custom `ContourFilterPen`: remove zero-length segments, merge collinear lines

---

## Part 4: Generating the Bold Master

The hardest problem. Both masters must have identical structure (contours, point count, point types). Only coordinates and widths differ.

### VG's Weight Strategy: Counter Reduction

The design philosophy says: outer contours often stay **identical** between Regular and Bold. Weight comes from the inner counter shrinking inward.

For round forms (O, C, e):
- Keep outer contour the same
- Offset inner counter inward

For straight stems (I, H, E):
- Widen stems (Regular ~96 units → Bold ~160 units)
- Keep outer height the same

### Approach 1: Trace Both Weights Independently, Then Match Structure

1. Create reference images at both weights
2. Autotrace both
3. **Problem**: the two traces will have different point counts and structures
4. **Solution**: Define the topology once (from the Regular trace), then use DiffVG optimization to fit that topology to the Bold target image

### Approach 2: Trace Regular, Derive Bold Algorithmically

1. Autotrace the Regular weight only
2. For each contour, apply VG's counter-reduction rules:
   - Outer contours: keep identical (copy coordinates)
   - Inner counters: offset inward by `(bold_stem - regular_stem) / 2`
   - Adjust advance width proportionally (Bold/Regular width ratio ~1.06)
3. This guarantees structural compatibility by construction

### Approach 3: DiffVG/Bezier Splatting Optimization (AI-Assisted)

Use differentiable rendering to optimize control point positions:

1. Define the contour topology (from your Regular trace)
2. Initialize Bold coordinates = Regular coordinates
3. Render via differentiable rasterizer (DiffVG or Bezier Splatting)
4. Compare against Bold target image
5. Backpropagate gradients to adjust coordinates
6. Repeat until converged

**Bezier Splatting** (NeurIPS 2025) is 30x faster forward / 150x faster backward than DiffVG. Available at [github.com/WeixiongLin/Bezier-Splatting](https://github.com/WeixiongLin/Bezier-Splatting).

This preserves topology (same contour count, same point count) while optimizing coordinates to match a target — exactly what we need for compatible masters.

---

## Part 5: AI/ML Approaches Worth Exploring

### Tier 1: Use Today

**StarVector-1B** — Vision-language model that generates SVG from images. Trained on 1.8M font samples. Runs on a single consumer GPU (~8GB VRAM).
- [HuggingFace: starvector-1b-im2svg](https://huggingface.co/starvector/starvector-1b-im2svg)
- Input: rendered glyph image → Output: SVG code
- Fast (seconds per glyph)
- Quality: rough but useful as a starting point, especially for complex shapes

**DiffVG** — Differentiable vector graphics renderer. Not a generator — it's an optimizer. Define initial bezier paths, render them, compare to target image, backprop to adjust control points.
- [github.com/BachiLi/diffvg](https://github.com/BachiLi/diffvg)
- Best use: refine autotraced outlines against a reference image
- Requires GPU, PyTorch

**LIVE** — Layer-wise image vectorization. Progressively adds bezier paths to reconstruct a target image. Good for single-glyph vectorization.
- [github.com/ma-xu/LIVE](https://github.com/ma-xu/LIVE)
- Minutes per glyph, single GPU

### Tier 2: Worth Setting Up

**DeepVecFont-v2** (CVPR 2023) — Transformer-based vector font generation. State-of-the-art for neural vector font generation.
- [github.com/yizhiwang96/deepvecfont-v2](https://github.com/yizhiwang96/deepvecfont-v2)
- Requires training data (100+ fonts converted to SFD format)

**Chat2SVG Stage 3** (CVPR 2025) — VAE-based path optimizer + DiffVG coordinate refinement. The optimization stage alone could be extracted for font glyph refinement.
- [github.com/kingnobro/Chat2SVG](https://github.com/kingnobro/Chat2SVG)

### Tier 3: Research Interest

**OmniSVG** (NeurIPS 2025) — Built on Qwen2.5-VL, generates SVG from text or images.
- [github.com/OmniSVG/OmniSVG](https://github.com/OmniSVG/OmniSVG)

**VecFusion** (CVPR 2024, Adobe) — Cascaded diffusion for vector fonts. Best quality but Adobe research code.

**Differentiable Variable Fonts** (Oct 2025) — Makes variable font interpolation differentiable in PyTorch. Could optimize axis values to match target images. Future direction: inverting to create masters.
- [arxiv.org/abs/2510.07638](https://arxiv.org/abs/2510.07638)

### What Doesn't Work Well

- **Claude/GPT writing coordinates from scratch**: Too slow, too many tokens, insufficient precision for bezier control points. LLMs lack spatial reasoning for numerical coordinates.
- **SAM for glyph extraction**: Overkill. Simple thresholding beats it for clean typographic images.
- **ControlNet/diffusion models alone**: Generate rasters, not vectors. Add an extra conversion step and introduce imprecision.
- **Fine-tuning LLMs on SVG data**: ACCV 2024 tried this with GPT-3.5 — results were worse than baseline for simple Latin letterforms. LLMs can learn SVG structure but lack precision for clean geometric shapes.

---

## Part 6: Recommended Implementation Plan

### Phase 1: Quick Win (1 hour)

Get the basic pipeline working with existing tools:

1. `pip install vtracer ufoLib2`
2. Prepare a high-contrast B/W glyph image (white glyph on black, or black on white, 1000+ px tall)
3. Trace with vtracer (`colormode=bw, mode=spline`)
4. Convert SVG to GLIF using fontTools `SVGPath`
5. Write into UFO with ufoLib2
6. Build and check

This won't produce production-quality results, but it proves the pipeline and shows you exactly what post-processing is needed.

### Phase 2: Post-Processing Script (1 day)

Build `scripts/img2bez.py` that chains the full pipeline:

```
trace → parse SVG → scale to UPM → grid snap → add extrema →
fix contour direction → simplify → add chamfers → write to UFO
```

All the fontTools pieces exist. The script is ~200 lines of Python gluing them together. Use the code snippets from Part 3 above.

### Phase 3: Bold Master Generation (1-2 days)

Add the Bold derivation step:
- Copy Regular topology
- Apply counter-reduction offsets
- Adjust advance widths
- Validate compatibility with `fontTools.varLib.interpolatable`

### Phase 4: img2bez — Custom Rust Crate (weekend project)

If the Python pipeline's tracing quality isn't good enough, build img2bez:

```
image + imageproc → contour detection
kurbo → optimal curve fitting (fit_to_bezpath_opt)
norad → optional UFO output (or use BezPaths directly in your editor)
```

This bypasses the SVG intermediate format entirely and uses kurbo's superior curve fitting. The `imageproc::contours::find_contours()` function gives you pixel-level polylines with Outer/Hole classification. Kurbo's `SimplifyBezPath` + `fit_to_bezpath_opt()` produces near-optimal cubic bezier fits. Core output is `Vec<kurbo::BezPath>` — format-independent, usable in any linebender-ecosystem tool.

### Phase 5: DiffVG Refinement (optional, for polish)

For glyphs that need extra precision:
1. Start with autotraced outline
2. Use DiffVG/Bezier Splatting to optimize control points against the reference image
3. This is especially useful for the Bold master — define topology from Regular, optimize coordinates against Bold target

---

## Key Resources

### Tools & Libraries

| Tool | Type | Install | Use For |
|------|------|---------|---------|
| [vtracer](https://github.com/visioncortex/vtracer) | Rust/Python | `pip install vtracer` | Autotracing |
| [potrace](https://potrace.sourceforge.net/) | C/Python | `pip install pypotrace` | Autotracing (alt) |
| [fontTools](https://github.com/fonttools/fonttools) | Python | installed | SVG parsing, transforms, bezier math |
| [ufoLib2](https://github.com/fonttools/ufoLib2) | Python | installed | UFO read/write |
| [kurbo](https://github.com/linebender/kurbo) | Rust | `cargo add kurbo` | Curve fitting (best-in-class) |
| [norad](https://github.com/linebender/norad) | Rust | `cargo add norad` | UFO read/write from Rust |
| [imageproc](https://github.com/image-rs/imageproc) | Rust | `cargo add imageproc` | Contour detection |
| [DiffVG](https://github.com/BachiLi/diffvg) | Python/CUDA | build from source | Differentiable rendering |
| [Bezier Splatting](https://github.com/WeixiongLin/Bezier-Splatting) | Python/CUDA | build from source | Fast differentiable rendering |
| [StarVector](https://huggingface.co/starvector/starvector-1b-im2svg) | Python | HuggingFace | Image→SVG via AI |
| [beziers.py](https://github.com/simoncozens/beziers.py) | Python | `pip install beziers` | Path offset, tidy, extrema |
| [svg2glif](https://crates.io/crates/svg2glif) | Rust | `cargo add svg2glif` | SVG→GLIF conversion |

### Papers

- [Bezier Splatting](https://arxiv.org/abs/2503.16424) — 30-150x faster than DiffVG (NeurIPS 2025)
- [StarVector](https://arxiv.org/abs/2312.11556) — Image-to-SVG foundation model
- [DeepVecFont-v2](https://arxiv.org/abs/2303.14585) — Best neural vector font generation (CVPR 2023)
- [VecFusion](https://arxiv.org/abs/2312.10540) — Cascaded diffusion for vector fonts (CVPR 2024)
- [Differentiable Variable Fonts](https://arxiv.org/abs/2510.07638) — Gradient-based variable font optimization
- [Chat2SVG](https://arxiv.org/abs/2411.16602) — LLM + DiffVG hybrid pipeline (CVPR 2025)
- [Raph Levien: Simplifying Bezier Paths](https://raphlinus.github.io/curves/2023/04/18/bezpath-simplify.html) — The math behind kurbo's simplifier

### Collections

- [Typography Research Collection](https://github.com/IShengFang/TypographyResearchCollection) — Comprehensive index of font/typography AI papers
- [PyTorch-SVGRender](https://github.com/ximinng/PyTorch-SVGRender) — Unified library for DiffVG, LIVE, and other SVG renderers
