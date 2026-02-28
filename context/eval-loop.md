# Reference-Comparison Evaluation Loop

## Overview

img2bez includes a `--reference` flag that compares traced output against a hand-drawn reference `.glif` file. This creates a tight feedback loop for tuning the tracing algorithm: trace → measure → identify weaknesses → fix → re-measure.

The reference glyphs live in the Virtua Grotesk sources at `~/GH/repos/virtua-grotesk/sources/VirtuaGrotesk-Regular.ufo/glyphs/`.

## Running It

```bash
# Basic usage: trace + compare
cargo run --release -- \
  -i ~/Desktop/a-U+0061.png \
  -o /tmp/eval-test.ufo \
  -n a -u 0061 \
  --target-height 735 --y-offset -172 --grid 2 \
  --accuracy 2 --alphamax 0.8 \
  --reference ~/GH/repos/virtua-grotesk/sources/VirtuaGrotesk-Regular.ufo/glyphs/a.glif
```

**Important: Scale calibration.** The `--target-height` must be calibrated so the traced output matches the reference glyph size. If the Scale metric shows a ratio far from 1.0x, adjust `--target-height`. For the current test images rendered from Virtua Grotesk, `--target-height 735 --y-offset -172` produces correct scale.

The output UFO must already exist (a minimal UFO with `metainfo.plist`, `layercontents.plist`, and `glyphs/contents.plist` is enough). Create one with:

```bash
mkdir -p /tmp/eval-test.ufo/glyphs

cat > /tmp/eval-test.ufo/metainfo.plist << 'EOF'
<?xml version="1.0" encoding="UTF-8"?>
<!DOCTYPE plist PUBLIC "-//Apple//DTD PLIST 1.0//EN" "http://www.apple.com/DTDs/PropertyList-1.0.dtd">
<plist version="1.0">
<dict>
	<key>creator</key>
	<string>org.robofab.ufoLib</string>
	<key>formatVersion</key>
	<integer>3</integer>
</dict>
</plist>
EOF

cat > /tmp/eval-test.ufo/layercontents.plist << 'EOF'
<?xml version="1.0" encoding="UTF-8"?>
<!DOCTYPE plist PUBLIC "-//Apple//DTD PLIST 1.0//EN" "http://www.apple.com/DTDs/PropertyList-1.0.dtd">
<plist version="1.0">
<array>
	<array>
		<string>public.default</string>
		<string>glyphs</string>
	</array>
</array>
</plist>
EOF

cat > /tmp/eval-test.ufo/glyphs/contents.plist << 'EOF'
<?xml version="1.0" encoding="UTF-8"?>
<!DOCTYPE plist PUBLIC "-//Apple//DTD PLIST 1.0//EN" "http://www.apple.com/DTDs/PropertyList-1.0.dtd">
<plist version="1.0">
<dict>
</dict>
</plist>
EOF
```

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
| **Scale** | 15% | Size match between traced and reference bounding boxes. 1.0x = perfect. | Adjust `--target-height` and `--y-offset` |
| **Shape** | 30% | How closely the traced outline matches the reference shape (after normalizing scale). Hausdorff < 5 = excellent, 5-15 = good, > 15 = fair. | Better curve fitting accuracy, polygon approximation, source image quality |
| **Points** | 10% | On-curve point count ratio. Penalizes both over- and under-counting. | `--accuracy`, `--alphamax` |
| **Segments** | 10% | Proportion of curves vs lines. Compares line fraction. | Curve-to-line thresholds, `--accuracy` |
| **H/V handles** | 15% | Percentage of off-curve points axis-aligned with their anchor. | H/V snap pipeline (automatic) |
| **Grid** | 10% | Percentage of on-curve points on the coordinate grid. | `--grid` flag |
| **Contours** | 10% | Binary: 1.0 if contour count matches, 0.0 otherwise. | Decompose filtering, min contour area |

### Score interpretation

- **> 0.90** — Good structural match, remaining differences are from source bitmap
- **0.75–0.90** — One or two metrics dragging it down, check Scale first
- **0.50–0.75** — Significant issues (wrong scale, structural mismatch)
- **< 0.50** — Something fundamentally wrong (inverted image, wrong parameters)

## Key Parameters

```bash
# Corner detection: lower = more corners = tighter fits for geometric type
--alphamax 0.8    # good for geometric type (default: 1.0)

# Curve fitting accuracy in font units: lower = more faithful but more points
--accuracy 2      # good balance (default: 4.0)

# Scale: MUST be calibrated per source image
--target-height 735   # for current test images
--y-offset -172       # baseline offset

# Grid snap (use even values)
--grid 2
```

## The Improvement Loop

1. **Calibrate scale first** — run with `--reference` and check the Scale metric. Adjust `--target-height` until Scale ratio ≈ 1.0x. This is the single most impactful parameter.

2. **Check structural metrics** — Points, Segments, Contours should match closely. Adjust `--alphamax` (0.6-1.0) and `--accuracy` (0.5-8.0).

3. **Shape quality** — Hausdorff below 10 is good. Remaining distance is often from the source bitmap itself (rendering differences, anti-aliasing, etc). Below 5 requires near-perfect source images.

4. **Quality metrics** — H/V handles and Grid are handled automatically by the cleanup pipeline.

## Source

The evaluation module lives in `src/eval.rs`. The CLI integration is in `src/main.rs` (the `--reference` arg). Both are gated behind the `ufo` feature (enabled by default).
