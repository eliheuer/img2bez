# img2bez autoresearch

You are running an autonomous overnight research loop to improve img2bez — a
bitmap-to-Bezier tracing tool for font glyphs.

Your goal is to maximize **mean_iou** (mean raster IoU across all reference
glyphs, higher is better). The secondary metric **mean_score** (weighted vector
quality) should also improve or hold steady.

Do not stop. Run experiments continuously until manually interrupted.

---

## Reference font

The current reference is **Virtua Grotesk Regular** (52 glyphs: A–Z, a–z).
Baseline established 2026-03-20: **mean_iou 90.39%**, mean_score 0.804.

To switch to a different font for a research session:
```bash
ln -sf /path/to/other.ufo autoresearch/reference.ufo
# Or override per-run:
REFERENCE_UFO=/path/to/other.ufo ./autoresearch/run_experiment.sh
```

---

## Setup ritual (do this once at the start of a session)

1. Read these files for full context:
   - `autoresearch/program.md` (this file)
   - `README.md`
   - `src/lib.rs`
   - `src/eval.rs`
   - `src/vectorize/curve.rs`
   - `src/vectorize/polygon.rs`

2. Create a branch: `git checkout -b autoresearch/$(date +%b%d | tr A-Z a-z)`

3. Run the baseline experiment (default parameters, no extra flags):
   ```
   ./autoresearch/run_experiment.sh > run.log 2>&1
   ```
   Parse `run.log` for `mean_iou` and `mean_score`.

4. Initialize `autoresearch/results.tsv` (untracked by git):
   ```
   commit	mean_iou	mean_score	glyphs_ok	status	description
   <hash>	<value>	<value>	<N>	baseline	default parameters
   ```

---

## Experiment loop (repeat forever)

### Step 1 — Propose a hypothesis

Examine the current code and metrics. Ask:
- Which glyphs had the lowest IoU? What do they have in common?
- Is the polygon too coarse (losing shape detail) or too fine (noisy)?
- Are corners detected correctly?
- Are smooth curves overshooting or undershooting?
- Are there systematic biases in the traced outlines vs. reference?

Then choose ONE change to make. The change can be:

**A. Parameter experiment** (fast, no recompile):
- Try different default values for: `alphamax`, `fit_accuracy`, `smooth_iterations`,
  `min_contour_area`, algorithm constants in `curve.rs`/`polygon.rs`
- Run: `./autoresearch/run_experiment.sh --alphamax 0.8 --accuracy 3.0`

**B. Source code experiment** (slower, requires recompile):
- Modify a constant, heuristic, or algorithm in `src/`
- Run: `./autoresearch/run_experiment.sh` (script rebuilds automatically)
- Scope: `src/vectorize/`, `src/cleanup/`, `src/config.rs`, `src/lib.rs`

### Step 2 — Run the experiment

```
./autoresearch/run_experiment.sh [extra flags] > run.log 2>&1
```

The script:
1. Builds `img2bez` (release mode)
2. For each glyph in `autoresearch/reference.ufo`:
   - Renders to PNG via `render_glyph.py` (drawbot-skia)
   - Runs img2bez with `--reference` to compare traced vs. original
   - Collects raster IoU and vector quality score
3. Reports `mean_iou`, `mean_score`, `glyphs_ok`, `glyphs_failed`

Parse results:
```
grep "mean_iou:\|mean_score:\|glyphs_ok:\|glyphs_failed:" run.log
```

### Step 3 — Decide: keep or discard

**Keep** if `mean_iou` improved by any amount:
```
git add -p   # stage only src/ changes (not run.log or work/)
git commit -m "autoresearch: <one-line description>"
```
Log to `results.tsv`: status=`keep`

**Discard** if `mean_iou` is equal or worse:
```
git checkout -- src/   # revert source changes only
```
Log to `results.tsv`: status=`discard`

**Crash** if the experiment failed to produce metrics (build error, all glyphs
failed, NaN in output):
```
git checkout -- src/   # revert
```
Log to `results.tsv`: status=`crash`. If the crash is a trivial bug you
introduced, fix it and retry. If it's a pre-existing issue, move on.

### Step 4 — Log results

Append a row to `autoresearch/results.tsv`:
```
<git_hash>	<mean_iou>	<mean_score>	<glyphs_ok>	<keep|discard|crash>	<description>
```

Then go back to Step 1 and propose a new hypothesis.

---

## What you can and cannot change

**CAN change:**
- Any file under `src/` (Rust source code)
- Algorithm constants, thresholds, heuristics
- The tracing pipeline logic

**CANNOT change:**
- `autoresearch/run_experiment.sh` (the evaluation harness)
- `autoresearch/render_glyph.py` (the rendering script)
- `autoresearch/reference.ufo/` (the ground truth)
- `autoresearch/program.md` (these instructions)

**SHOULD NOT change:**
- `Cargo.toml` dependencies (no new crates)
- Public API signatures in `lib.rs` (keep CLI working)

---

## Simplicity criterion

A gain of 0.1% IoU with 50 extra lines of code is not worth it.
A gain of 0.5% IoU with a 3-line constant change is definitely worth it.
Deleting code that achieves the same or better results is always a win.

When choosing between equally-performing approaches, prefer the simpler one.

---

## Where to look for improvements

Start by reading the per-glyph results in `autoresearch/work/*.log` for the
glyphs with the lowest IoU scores. Patterns to look for:

**Polygon approximation** (`src/vectorize/polygon.rs`)
- `min_contour_area`: too low → dust noise traced; too high → small counters lost
- Smoothing iterations: too few → staircase artifacts; too many → lost detail
- Sub-pixel vertex refinement: are corners placed at the right sub-pixel location?

**Curve fitting** (`src/vectorize/curve.rs`)
- `CURVATURE_TRANSITION_THRESHOLD`: affects straight vs. curved detection
- `MIN_SECTION_CHORD`: too large → merges distinct curve sections
- `alphamax` default: 1.0 may be too permissive for geometric letterforms
- `MAX_HANDLE_RATIO`: affects how aggressively handle lengths are balanced

**Corner detection** (`src/vectorize/curve.rs`)
- Are sharp corners (like in A, V, W) being detected correctly?
- Are smooth joins (like in O, S, C) being kept smooth?

**Post-processing** (`src/cleanup/`)
- H/V handle snapping: 15° threshold may be too loose or too tight
- Grid snapping: affects on-curve point placement

**Comparison images** (`autoresearch/work/*_comparison.png`):
Look at the visual diffs to build intuition about what's going wrong.
Each glyph also has a `work/<glyph>.log` with the full img2bez output.

---

## Results TSV format

File: `autoresearch/results.tsv` (not tracked by git)

Columns (tab-separated):
```
commit  mean_iou  mean_score  glyphs_ok  status  description
```

Example:
```
a839910	82.45	0.741	26	baseline	default parameters
b1c2d3e	83.12	0.748	26	keep	lower alphamax default to 0.85
f4e5d6c	82.98	0.745	26	discard	increasing smooth iterations hurt O and G
```

---

## Raster vs. vector comparison

Both comparisons matter but raster IoU is the primary metric:

**Raster IoU** (primary):
- Renders both the traced and reference glyphs to bitmaps at the same resolution
- Computes pixel-level intersection-over-union
- Directly measures visual accuracy
- Saved as `autoresearch/work/<glyph>_comparison.png` and `<glyph>_diff.png`

**Vector score** (secondary, 0.0–1.0):
- Scale match (0.15 weight): traced and reference have similar bounding box
- Shape distance (0.30): Hausdorff + mean point-to-boundary distance
- Point count (0.10): traced has similar number of on-curve points as reference
- Segment mix (0.10): similar ratio of curves to lines
- H/V handles (0.15): off-curve handles align to horizontal/vertical axes
- Grid alignment (0.10): on-curve points land on the grid
- Contour count (0.10): same number of contours (important for counter detection)

The per-component scores appear in `work/<glyph>.log` and help diagnose which
aspect of quality is failing.

---

## NEVER STOP

Keep running experiments until you are manually interrupted (Ctrl+C or process
kill). If you hit a dead end on one direction, pivot to a different part of
the code. There is always something to try.
