# Degradation corpus & input-adaptive settings

Foundation for choosing trace settings automatically from the kind of input
image (the "input adaptation" plan). The idea: you can only tune a
settings-selector if you can *measure* the choice, and that needs a corpus with
known answers.

## The trick: degradation augmentation

Render a known glyph clean, push it through controlled degradations, and the
*correct* outline is still known — the reference glyph it came from. That gives
`(degraded image → known outline)` pairs across the whole space of input
quality, for free.

`degrade.py` applies, in realistic capture order, any of: `--downscale`,
`--blur`, `--noise`, `--jpeg`, `--bilevel`, `--dither`, `--nn-upscale`.

```sh
# clean render -> a "wild" low-res, blurry, noisy capture
python degrade.py a-clean.png a-wild.png --downscale 110 --blur 1.3 --noise 8
```

## Sweep recipe (with reference)

img2bez scores a trace against a reference glyph with `--reference`, so a sweep
is: render → degrade → trace the degraded image at candidate settings → read the
`Overall` / `Raster IoU` score vs the known outline.

```sh
render_glyph.py reference.ufo a a-clean.png           # known answer = a.glif
degrade.py a-clean.png a-wild.png --downscale 110 --blur 1.3 --noise 8
for acc in 2 4 6 8; do
  img2bez --input a-wild.png --output /tmp/o.ufo --name a \
    --reference reference.ufo/glyphs/a.glif --accuracy $acc
done
```

## No-reference features (`img2bez stats`)

What a selector reads at production time (no reference available):

```sh
img2bez stats a-wild.png
# { "extentPx": 64, "sharpness": 359, "noise": 2.0, "bilevelness": 0.76 }
```

(`extentPx` = ink resolution, `sharpness` = variance of Laplacian,
`noise` = Immerkær sigma, `bilevelness` = fraction of extreme pixels.) On
clean vs degraded `a` these read 392/791/0.5/0.99 vs 64/359/2.0/0.76 — they
discriminate the input class cleanly.

## Findings (2026-06)

**Degraded inputs over-segment, and `fit_accuracy` can't fix it.** A clean `a`
traces to ~27 points, a down/blur/noise `a` to ~77–95 — the tracer tracks the
noise as wobble — and `--accuracy 1`→`10` only moves that 95→96. So the
profile system needs more levers than fit tolerance.

**The new levers (now exposed):** `--smoothing` (multiplier on the pre-fit
Gaussian sigma, `TraceOptions::smoothing`) and `--corner-threshold` (degrees,
`TraceOptions::corner_threshold_deg`). Sweeping them:

- *Blurry but high-resolution* `a` (shape survives): the score has a clear
  optimum — `--smoothing 2.5` gives 0.817 vs 0.789 at the default and 0.780 when
  over-smoothed (4.0). A real, recoverable gain, and the curve is unimodal, so a
  selector can map to it.
- *Severely low-res* `a` (64px, destroyed): smoothing trades point count for
  fidelity and the combined score doesn't improve — past a point the information
  is just gone. There's a fidelity floor no setting beats.

So the levers recover *moderate* degradation (the common AI/scan case) but not
destroyed input. Defaults (`smoothing 1.0`, `corner 12`) are unchanged, so the
structural gate stays green (`glyphs_worsened: 0`).

**Next:** sweep `{accuracy, smoothing, corner}` across the corpus tiers, map the
[`img2bez stats`](#no-reference-features-img2bez-stats) features to the winning
settings (e.g. low `sharpness` → higher `smoothing`), and validate the resulting
`auto` profile against the gate. Real AI-generated inputs are the best test set.
