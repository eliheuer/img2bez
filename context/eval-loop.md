# Reference-Comparison Evaluation Loop

img2bez includes a `--reference` flag that compares traced output against a hand-drawn reference `.glif` file. This creates a tight feedback loop: trace, measure, identify weaknesses, fix, re-measure.

## Running It

```bash
img2bez --input glyph.png \
  --output test.ufo \
  --name a \
  --unicode 0061 \
  --target-height 735 \
  --y-offset -172 \
  --grid 2 \
  --accuracy 2 \
  --alphamax 0.8 \
  --reference path/to/reference.glif
```

The output UFO must already exist. The repo includes `test.ufo` with empty A–Z glyphs.

**Scale calibration is critical.** The `--target-height` must be calibrated so the traced output matches the reference glyph size. If the Scale metric shows a ratio far from 1.0x, adjust `--target-height` first — this is the single most impactful parameter.

## Reading the Report

```
  Eval vs ~/path/to/reference.glif

  Scale         529x611 vs 528x608  (1.00x)                  0.993
  Shape         Hausdorff 14.3   mean 5.1    (normalized)      0.643
  Points        29 vs 29 on-curve  (x1.00)                   1.000
  Segments      12c+15l vs 12c+15l  (0.56 vs 0.56)            1.000
  H/V handles   24/24  (100%)                                 1.000
  Grid (2)      29/29  (100%)                                 1.000
  Contours      2 vs 2                                       1.000

  Overall       0.892
```

### Metrics

| Metric | Weight | What it measures | What improves it |
|--------|--------|-----------------|-----------------|
| **Scale** | 15% | Bounding box size match. 1.0x = perfect. | `--target-height`, `--y-offset` |
| **Shape** | 30% | Outline similarity. Hausdorff < 5 = excellent, 5–15 = good. | `--accuracy`, source image quality |
| **Points** | 10% | On-curve point count ratio. | `--accuracy`, `--alphamax` |
| **Segments** | 10% | Curve-to-line proportion match. | `--accuracy`, curve-to-line thresholds |
| **H/V handles** | 15% | Off-curve points axis-aligned with anchors. | Automatic (cleanup pipeline) |
| **Grid** | 10% | On-curve points on the coordinate grid. | `--grid` |
| **Contours** | 10% | 1.0 if contour count matches, 0.0 otherwise. | Min contour area filtering |

### Score interpretation

- **> 0.90** — Good structural match
- **0.75–0.90** — One or two metrics dragging it down, check Scale first
- **0.50–0.75** — Significant issues (wrong scale, structural mismatch)
- **< 0.50** — Something fundamentally wrong

## The Improvement Loop

1. **Calibrate scale** — adjust `--target-height` until Scale ratio is ~1.0x
2. **Check structure** — Points, Segments, Contours should match closely. Tune `--alphamax` (0.6–1.0) and `--accuracy` (0.5–8.0)
3. **Shape quality** — Hausdorff below 10 is good. Below 5 requires near-perfect source images
4. **Quality metrics** — H/V handles and Grid are handled automatically by cleanup

## Source

The evaluation module is in `src/eval.rs`. The raster comparison (IoU) is in `src/render.rs`. Both are gated behind the `ufo` feature (enabled by default).
