# Input Adaptation: good default traces across image types

Working plan + research log for making `img2bez` produce good traces, **on
default settings**, across very different input images (clean renders, crisp
bilevel art, soft photographic scans of printed type, noisy AI rasters).

Related: [`eval-harness/CORPUS.md`](../eval-harness/CORPUS.md) (degradation
corpus), the blog post §6 "Tuning for Different Inputs", and the shipped levers
`--pre-blur` / `--smoothing` / `--corner-threshold` / `--mode`.

Status: **research done, Phase 1 not yet started.** Last updated 2026-06-28.

---

## 1. Problem

A single fixed default cannot be optimal for inputs that differ by orders of
magnitude in edge sharpness and contrast. Two reference images we want to trace
**well on defaults**:

- **`G.png`** — crisp, near-bilevel letter. *Already fine* (16 points).
- **`os-001.png`** — soft, low-contrast photo of an old-style printed `a`
  (paper grain, uneven ink, soft edges). *Fails badly* on defaults: 69 points,
  the boundary chases ink texture into spurious points.

Goal: both good on defaults, without regressing the clean reference font (the
structural gate must stay `glyphs_worsened: 0`).

---

## 2. Research findings (2026-06-28)

### 2.1 The input classes separate by orders of magnitude on cheap features

`img2bez stats` (variance-of-Laplacian sharpness, bilevelness, Immerkär noise,
ink extent):

| image | extent px | sharpness | bilevelness | default trace |
|---|---|---|---|---|
| VG render `a` (gate ref) | 392 | 790.8 | 0.992 | 27 pt (good) |
| VG render `n` | 382 | 619.5 | 0.995 | good |
| VG render `s` | 392 | 810.2 | 0.993 | good |
| `G.png` (crisp) | 782 | 669.3 | 0.996 | 16 pt (good) |
| **`os-001.png` (soft photo)** | 489 | **2.7** | **0.019** | **69 pt (bad)** |

The "clean regime" (renders + crisp art) clusters at sharpness **620–810**,
bilevel **0.99+**. `os-001` sits at sharpness **2.7**, bilevel **0.019** — a
~250× / ~50× outlier. **The signal that says "this needs help" is not subtle.**

### 2.2 The fix is mechanical and verified

`os-001` point count vs lever:

```
default            69 pt
pre-blur 2         36 pt
pre-blur 4         32 pt
pre-blur 6         28 pt   <- clean, verified a good old-style `a` visually
pre-blur 8         27 pt
pre-blur 12        22 pt
smoothing 4.0      33 pt   <- weaker than pre-blur
```

**Image pre-blur is the right lever** for soft/textured edges: it removes edge
texture *before* the iso-contour is drawn. Polyline smoothing (the existing
adaptive stage) only cleans the boundary *after* the iso-contour already
wandered, so it caps out weaker (33 vs 28). `pre-blur ~6` at extent 489 ≈
**~1.2% of ink extent**.

### 2.3 What does NOT work: blanket resolution-normalization

Downscaling everything to a working resolution (area/LANCZOS) was tested as a
corpus-free "just normalize" default. It **regresses the clean regime** and is
non-monotonic:

```
VG-a   native 27 pt  -> ext300 37 pt,  ext220 73 pt   (REGRESSED)
os-001 native 69 pt  -> ext300 28 pt,  ext220 64 pt   (finicky)
```

Downscaling a clean render adds points (resampling ringing). **Ruled out as a
default.** Adaptation must be *targeted parameter selection*, not a blanket
transform.

### 2.4 Why the existing adaptive smoothing undershoots on `os-001`

`fit.rs` already escalates the pre-fit Gaussian sigma "until the boundary reads
as clean type geometry," but the break criterion is `mean_turn` + corner-count
(it looks for *sharp* features). `os-001`'s failure is *low-amplitude meander*
of a soft textured edge — gentle wandering, not sharp turns — so the criterion
is satisfied early and never escalates, yet the cubic fitter still spends ~69
points tracking the meander within the fit tolerance. The escalation is also
capped (`sigma *= 1.8`, 4 iterations) and is a polyline smooth (weaker lever).

---

## 3. Strategy

### 3.1 Core principle (unchanged from the blog)

Any learned component stays a **selector** (picks settings) or a **judge**
(scores a result). It **never draws the outline**. The deterministic tracer
stays the inner loop, so `img2bez` stays fast, deterministic, embeddable.

### 3.2 "Procedural vs ML" is a false binary at inference time

A tiny learned selector (logistic regression / GBM / 2-layer MLP over ~6 scalar
features) is a few **KB** of weights and a ~20-line matmul. Ship it as hardcoded
weights + Rust matmul: zero dependencies, deterministic, compiles to WASM, runs
in microseconds. That is *as* procedural and embeddable as a hand-written
threshold. **"Too big to include" is not the constraint** — size is a non-issue;
the cost is training infrastructure + the corpus.

So the real choice is **hand-tuned thresholds vs corpus-learned weights, both
shipped as deterministic Rust**. A heavyweight NN runtime (ONNX/candle/ort + a
conv net) is *not* needed for this problem and should be avoided.

### 3.3 Selector vs Judge (the more interesting axis)

- **Selector**: `features -> preset`. Predicts. Cheapest. Given the huge class
  separation, threshold rules already classify ~correctly.
- **Judge**: `(image, traced outline) -> quality score`. Evaluates the *actual*
  result. Enables **try-N-presets-and-keep-the-best** (search > blind predict)
  and, crucially, **reference-free QA at production time** (the eval harness
  today needs a hand-drawn reference). A judge can start *procedural*: raster IoU
  between the re-rendered outline and the thresholded input + structural sanity
  (point economy, H/V handles, contour count).

### 3.4 The corpus is the real asset

The degradation corpus auto-labels training data: render a known glyph, degrade
it many ways, trace each degraded image under every preset, score against the
**known source** — the best-scoring preset is the label, for free, at scale.
The corpus can (a) calibrate threshold rules, (b) train a selector, or (c)
generate judge labels. Build it regardless of the model decision.

### 3.5 The mode/preset model (the organizing concept)

The end state is a set of **modes/presets**, each a bundle of settings tuned for
one input class (clean render, soft photographic scan, crisp bilevel, noisy AI
raster, low-res sprite), plus optional output-shape modes (smooth/line). In
**default mode** the tracer *detects the input class and applies the matching
preset automatically* — the Phase 1 auto-pre-blur is the first, narrow instance
of this (detect "soft" → apply the soft handling). Explicit `--profile`/`--mode`
overrides the detection.

So the work is two coupled questions:
1. **What settings does each input class need?** (define the presets)
2. **How do we detect the class from the image?** (the selector)

`os-001` is the running test case for the **soft-scan preset**: pushing its
quality further (the tail/terminal de-clustering) is how we develop the preset
and learn what distinguishes "needs soft mode" from the default — not a one-off
hand-tune. Each new preset gets its own canonical test image(s).

### 3.6 Recommendation: layered, shipped incrementally

Start procedural (selection is easy here); reserve learning for the judge and
for when presets multiply. Each phase below is independently shippable.

---

## 4. Checklist

### Phase 0 — Placement offset fix (DONE)
- [x] `runebender-web` `image_trace.rs` honors `xOffset` as LSB so the trace
      aligns with the background image (regression from the API port).
- [x] Verified locally at `localhost:8770`.
- [x] Redeploy to runebender.org (commit `runebender-web` + re-run
      `build-cloud-editor` + push `runebender-dot-org`). Live 2026-06-28.

### Phase 1 — Gate-safe auto pre-blur (procedural v1)  ✅ DONE (2026-06-28)
- [x] `auto_pre_blur_sigma()` in `lib.rs::trace_luma` (after upscale/downscale,
      before threshold), only when `pre_blur == 0` and `auto_pre_blur` is on.
      Trigger: `sharpness < 80 AND bilevelness < 0.30`; blur =
      `clamp(0.025 * extent_px, 2, 16)` (os-001 → sigma ~12).
- [x] Never overrides an explicit `--pre-blur`. New `auto_pre_blur` field
      (default true) + CLI `--no-auto-pre-blur` to disable.
- [x] Gate: both `run_structural_gate.sh` checks pass — `glyphs_worsened: 0`
      and `PASS: trace stress gate`.
- [x] `os-001` 69 → **28 pt**; `G` unchanged (16 pt); VG renders + demo-a
      unchanged; verified none of the gate glyphs trip the trigger.
- [x] Unit tests: crisp-bilevel → no blur (safety), soft-ramp → blur.

**Calibration lessons (important for Phase 3):**
- **bilevelness alone is not enough.** A *low-resolution* clean render also reads
  low bilevel (anti-aliased edge pixels dominate when the glyph is small): the
  stress gate's `aen-lowres2x.png` is bilevel 0.168 / sharpness 119. A
  bilevel-only trigger fired on it and **failed the trace-stress gate**.
- **sharpness is the separator.** Soft scan (os-001) = sharpness ~3; low-res
  render = sharpness ~120; clean render = 600+. Requiring `sharpness < 80`
  *and* `bilevelness < 0.30` catches the soft scan and excludes the low-res
  render. Both gates pass.
- Real soft scans have *soft* (low-gradient) texture, so low sharpness is the
  right signal. Synthetic grain (injected high-amplitude noise) reads sharp
  (~130) and would be missed — but that is not representative of real input;
  do not raise the sharpness threshold for it without real samples.
- Thresholds (80, 0.30) are calibrated on os-001 + the gate composites. Replace
  with corpus-derived values in Phase 3.
- Decision: **baked in, always-on** (field default true). Fires only on soft
  inputs, so `clean`/`wild` both get it safely; no separate `auto` profile.

### Phase 2 — Degradation corpus + reference-free judge  (in progress)
- [x] **Reference-free judge** already exists: `eval-harness/score_wild.py`.
      `wild = 0.5*repro_iou + 0.5*structure`, where `structure` =
      `0.5*hv_frac + 0.3*micro_clean + 0.2*parsimony` (all no-reference). The
      micro/parsimony terms catch over-segmentation that pixel-IoU alone
      rewards. Validated on os-001: clean 23-pt trace scores **0.895** vs the
      messy 69-pt trace **0.762**, *even though* the messy one has higher
      repro_iou (0.96 vs 0.88) — the structure terms correctly override.
- [x] **Search+judge harness**: `select_preset.py` (since removed;
  superseded by the img2bez-data fontcorpus loop) — try presets
      {raw, default, clean}, judge each, pick the winner; prints `img2bez stats`
      features alongside, so each row is a `(features -> winning preset)` label.
- [x] **Validated detection**: across os-001 / G / VG-a / demo-a (all real
      inputs), `default` (the tracer's own auto-detection) wins or ties for the
      judge-best preset. os-001 → soft (default) wins by a wide margin; clean
      regime → default == raw (auto inert). The detection already matches the
      judge.
- [x] **Degradation corpus runner**: `eval-harness/build_corpus.py` renders
      known glyphs, degrades them across 8 tiers (clean/soft/soft_hard/blur/
      lowres/noisy/bilevel/nn2x), traces each under every preset, and scores
      both reference-free (judge) and reference-based. Added `degrade.py
      --contrast LO:HI` to synthesize the low-bilevel soft class. 6 glyphs × 8
      tiers × 3 presets = 144 traces; full table → `corpus-work/corpus.tsv`.
      Judge↔reference winner agreement **37/48 (77%)**; Pearson over 144 traces
      **0.32** (dragged down by extreme tiers — lowres at 57px, nn2x).

      **Critical finding — the synthetic corpus can't reproduce os-001's class.**
      On every synthetic tier the judge prefers `raw` (no auto-blur), because
      smooth blur+contrast produces an *already-clean* boundary: raw traces the
      soft tier at 14–20 pts, and auto-blur only over-blurs it slightly. But
      real os-001 traces at 69 pts under raw and 23 under default — because its
      problem is **edge texture** (ink grain), which a smooth Gaussian
      degradation does not create. So:
        - The auto-pre-blur's real job is **removing texture, not softness**;
          sharpness/bilevelness are proxies that happen to fire on real soft
          scans (which are textured) but also on smooth-soft (which doesn't need
          it). Real soft scans are usually textured, so the rule is net-positive
          in practice, but it is calibrated on a proxy.
        - Faithfully synthesizing soft texture is hard: injected noise reads
          *sharp* (high Laplacian variance), unlike real soft grain (os-001
          sharpness ≈ 3). **The corpus needs real soft scans for this class**, or
          a paper/ink-grain degradation model. The synthetic tiers are valid for
          the other classes (lowres/bilevel/noisy/nn2x/blur).
- [x] Trust check (compact): on softened VG glyphs (a/e/n/s, reference known),
      traced under all presets, the judge's best preset matches the
      reference-based best on **3/4** (the 4th is a raw≈clean tie), and
      Pearson(wild, reference_score) = **0.61** over 12 traces. The judge tracks
      real quality well enough to drive selection.
- [ ] Scale the trust check to the full degradation corpus (more glyphs/tiers)
      and tune the judge weights if the correlation needs tightening.

**Phase 2 findings:**
- **Search+judge > a fixed rule.** On *already-soft* synthetic glyphs the fixed
  auto-pre-blur over-blurs (adds blur to an already-blurred image); both the
  judge and the reference then prefer `raw`/`clean` over `default`. A
  judge-driven selection picks correctly per-image (raw there, default on
  os-001). So: calibrate the fast runtime rule **offline** with the corpus+judge;
  don't ship the N-trace search in the hot path.
- **Phase 3 calibration item:** scale the auto-pre-blur *down* when the input is
  already very soft (low sharpness), instead of a fixed fraction of extent — the
  over-blur the corpus exposed. os-001 (real, textured) is unaffected; this is
  about not double-blurring an already-soft scan.

### Phase 3 — Calibrate rules / define presets  (in progress)
- [x] **Mode system formalized.** The soft handling is now a named, selectable
      preset: `Profile::Photo` (wild-like fit + a resolution-scaled image
      pre-blur). Detection is factored into `ImageStats::classify()` (features →
      `Profile`): soft (sharpness < 80 AND bilevelness < 0.30) → `Photo`, else
      `Wild`. `TraceOptions.profile` records the choice. Default flow
      auto-classifies and applies `Photo` when soft; `--profile photo` forces it
      on any image; `--no-auto-pre-blur` disables the auto path. CLI + wasm (via
      `for_profile`/`from_name`) both expose it; gate green, judge on os-001
      unchanged (0.895). `select_preset.py` gained a `photo` candidate.

- [x] **Soft-photo blur curve retuned from real images (2026-06-29).** Swept the
      14 Desktop glyph images with `label_sweep.py` (since removed; superseded
      by the img2bez-data fontcorpus loop) (sweep settings,
      judge each, render a contact sheet, save a dataset label). The old curve
      `extent × 0.025` (sigma 12 on os-001) over-blurred and melted the terminals;
      `× 0.016` (sigma ~8) keeps them crisp (os-001 repro_iou 0.88 → 0.92, wild
      0.895 → 0.903, structure held). Only touches the auto-detected photo class,
      so the clean path is byte-identical and the gate stayed green (0 worsened).
      Calibrated on os-001 (the only photo-class sample so far); widen with more
      soft scans. The default now handles os-001 with no override.
- [x] **Judge bug fixed.** `score_wild._source_mask` returned an empty mask when
      img2bez reported an extreme threshold (0 on hard-bilevel/no-AA images),
      giving a bogus repro_iou of 0 (H scored 0.500 for a perfect trace). Now
      recovers a histogram-midpoint cut. H 0.500 → 0.997.
- [ ] **Open: no-AA bilevel over-segmentation.** Crisp bilevel images with no
      anti-aliasing (high sharpness AND bilevelness ~0.997) make the sub-pixel
      tracer stair-step on horizontal/vertical edges. `n` (slab serifs) traces at
      37 on-curve points under `auto` vs 25 with a small pre-blur (pb4) that
      synthesizes the AA the pipeline expects. Real, but glyph-dependent (`e`/`c`
      at 0.997 trace fine at 10–12 pts; blanket blur melts them), so it needs a
      narrow rule calibrated on more no-AA samples — exactly what the dataset is
      for. Labeled `n`/`teat-a` accordingly. Do NOT add a blanket bilevel blur.
- [ ] The judge's structure term (`hv_frac`) underrates legitimately diagonal
      glyphs (`w`, `S`): it rewards blur that rounds strokes into more near-H/V
      handles even when blur visibly melts the form. Use the contact sheet
      (human-in-loop) as ground truth for those; don't drive labels off the
      judge alone. Consider a diagonal-aware structure term later.

- [ ] Use the corpus to set Phase 1 thresholds + blur curve from data, not by
      hand. Sweep `{pre_blur, smoothing, corner, accuracy}` per input tier.
- [ ] **Structural de-clustering.** On os-001 the residual over-segmentation is
      *structural*, not fit-tolerance: a soft near-straight stroke spawns a
      stack of spurious x/y-extrema (loosening `--accuracy` does nothing; only
      blur removes them). Blur is a blunt fix. A targeted fix — suppress an
      extremum whose deviation from the chord is below a fraction of stroke
      width, and merge on-curve points closer than ~N units — would clean the
      remaining terminal/tail clusters without over-blurring fine features.
- [ ] Define named presets (e.g. `clean`, `wild`, `photo`/old-style, `sprite`)
      and the feature→preset map. Keep declared-style separate from
      image-quality (drawing style is intent, not measurable from one raster).

### Phase 4 — Tiny learned selector/judge (only if rules plateau)

**Decision (2026-06-29): the synthetic corpus can't train the selector; collect
real data instead.** Generated `(features -> best preset)` labels at scale (12
glyphs × 8 tiers, 96 samples) and the labels are unusable: **0 photo, 84 wild,
12 clean.** The synthetic degradations never make `photo` win (they're smooth,
so they don't over-segment like a real textured scan — the Phase 2 finding,
quantified), and `wild` vs `clean` is not feature-separable (sharp 1239 vs 1262,
bilevel 0.74 vs 0.65; clean wins scatter near-randomly). A classifier on this
learns "almost always wild" — strictly worse than the rule, which already fires
`photo` correctly on os-001 via the texture signature. **The bottleneck is
labeled data, not the model.** Even with good data, the class selector is
near-redundant with the rule; the real ML value would be Stage-2 continuous
lever prediction (optimal blur *amount* per image), which needs the same data.

So Phase 4 becomes a **real-data collection pipeline** (the rule stays the
runtime selector meanwhile):
- [x] **Trace log.** `img2bez --log PATH` (or the `IMG2BEZ_LOG` env var, the
      headless way) appends one JSONL record per trace — image features
      (`ImageStats`), settings used (forced/auto/effective profile, accuracy,
      pre-blur, smoothing, corner, mode, style, refine), and output point counts
      — plus an FNV-1a `imageHash` so the settings tried on one image group
      together. Single-glyph and per-master. Opt-in; default traces unchanged
      (gate green). Schema v1, a `label` field reserved for acceptance.
- [x] **Dataset inspector.** `eval-harness/tracelog.py [LOG] [--per-image]`
      summarizes coverage: class balance over unique images, feature ranges,
      settings variety, a readiness gauge (needs the photo class represented),
      and per-image the settings tried in order (**last ≈ accepted** — the weak
      label the real workflow produces for free).
- [ ] **Wire `IMG2BEZ_LOG` into the font-repo workflow** so Codex accumulates a
      dataset as it traces (AGENTS.md). Optionally capture an explicit accept
      signal in Runebender (kept trace = positive) to label beyond last-wins.
- [ ] **Once the log has enough photo-class images**, distill a tiny model
      (small MLP over features — hardcoded weights + Rust matmul, deterministic,
      dependency-free, WASM-compatible) and benchmark vs the rule; adopt only if
      it clearly wins. Start with the Stage-2 lever regressor, not the
      near-redundant class selector.

### Phase 5 — Surface the levers + auto everywhere  (in progress)

The real workflow is **headless-first**: an agent (Codex) drives the
`img2bez masters` CLI and reads the JSON report; Runebender is the visual
review/correction surface. So the surfacing splits in two:

- [x] **Headless / CLI (the agent path).** The trace-tuning flags
      (`--profile {wild,clean,photo}`, `--pre-blur`, `--smoothing`,
      `--corner-threshold`, `--mode`, `--no-auto-pre-blur`) are now `global`, so
      they work after any subcommand: `img2bez masters <ds> --glyph X
      --profile photo …`. One `opts_from_cli` builds the options for both the
      single-glyph command and `masters`, so they never drift. The `masters`
      `--report` JSON now records, per master, the effective `profile` (forced
      or auto-detected) plus `sharpness`/`bilevelness` — so the agent sees how
      each image was classified and can re-run with a forced profile if a trace
      looks off. (`img2bez::measure_image(bytes)` exposes the stats.) Validated
      on the 00DF germandbls example: both crisp renders classify `wild`, 48 pt,
      compatible. Gate green.
- [ ] **Runebender trace UI (human review).** Expose `profile` + the levers in
      `wasmTraceConfig` (`Runebender.vue` currently passes only
      width/targetHeight/xOffset/yOffset/grid/accuracy/invert/threshold) + a
      small control, then redeploy.
- [ ] Plumb the `photo` profile through wasm/MCP config (wasm `Config.profile`
      already takes a string via `Profile::from_name`, so `"photo"` works today;
      confirm + expose in the MCP tool schema).

---

## 5. Open questions / risks

- Overfitting Phase 1 to a single image (`os-001`). Mitigation: more soft test
  images before committing thresholds; corpus calibration in Phase 3.
- Sharpness/bilevel are content- and resolution-dependent; absolute thresholds
  are fragile. The huge clean-regime margin makes a conservative threshold safe
  *now*, but Phase 3 should replace hand thresholds with corpus-derived ones.
- Drawing style (inktraps, terminal shape) is design intent, not image quality —
  declare it (preset), don't infer it from one raster.

## 6. Reproduce the research

```sh
BIN=./target/release/img2bez
$BIN stats ~/Desktop/os-001.png            # sharpness 2.7, bilevel 0.019
$BIN stats ~/Desktop/G.png                 # sharpness 669, bilevel 0.996
# point-count sweep
$BIN --input ~/Desktop/os-001.png --name a --format json --output - | <count on-curve>
$BIN --input ~/Desktop/os-001.png --name a --pre-blur 6 --format json --output -
```
