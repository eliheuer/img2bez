# Focused Structural Eval Harness Loop

This loop is for improving `img2bez` toward editable font outlines, not just
raster overlap. The current reference UFO is:

```text
eval-harness/reference.ufo -> <your local checkout of the reference UFO>
(an untracked symlink; point it at the hand-drawn UFO the gate scores against)
```

## Focus Glyphs

The first structural set lives in `eval-harness/focused_glyphs.txt`:

```text
ampersand a e s R O S n
```

These cover counters, bowls, curves, diagonals, straight stems, terminals, and
the `n` arch structure.

## Run

```sh
cd img2bez
./eval-harness/setup.sh
./eval-harness/run_structural_loop.sh
```

The loop uses the repo-local `./.venv-eval-harness` environment created by
`eval-harness/ensure_venv.sh`; it does not depend on user-specific `~/Py`
paths.

You can pass normal `img2bez` flags through:

```sh
./eval-harness/run_structural_loop.sh --accuracy 2
```

## Outputs

The loop writes:

- `eval-harness/work/structural_report.tsv`
- `eval-harness/work/structural_summary.txt`
- `eval-harness/work/structure_<glyph>.svg`
- the existing raster comparison images from `run_experiment.sh`

Set `RUN_LABEL=<name>` to archive reports, logs, and overlays in
`eval-harness/runs/<name>` so they survive later runs:

```sh
RUN_LABEL=baseline ./eval-harness/run_structural_loop.sh
RUN_LABEL=accuracy2 ./eval-harness/run_structural_loop.sh --accuracy 2
```

The SVG overlays draw the reference outline in green and traced outline in red.
On-curve points are marked in the same colors. This makes it easier to inspect
whether the tracer found the same editable structure as the reference UFO.

## Shareable Specimen Images

The composite stress gate also renders a 2048x2048 DrawBot specimen image for
visual review. It uses the repo-local venv and the
`eliheuer/drawbot-skia` fork from `eval-harness/requirements.txt`.

```sh
./eval-harness/render-specimen.sh
./eval-harness/render-specimen.sh --text aRE
./eval-harness/render-specimen.sh --chars "ش"
```

The specimen has two rows:

- input/reference: outline structure, rendered preview, red overlay
- output/trace: outline structure, rendered preview, red overlay

Edit `eval-harness/render_specimen.py` to refine the Swiss grid, typography,
colors, labels, and technical footer, then rerun the command above.

## Current Structural Goal

Improve `mean_structural_score` and the weakest glyphs without letting raster
IoU collapse. When a change improves the structural score but hurts IoU, inspect
the overlays before deciding whether to keep it.

The point-structure target follows the same practical drawing rules described
in OH no Type's "Drawing Vectors": use as few points as possible, place smooth
on-curve points at extrema, add inflection points where curves change direction,
keep handles horizontal or vertical wherever possible, and reserve clustered or
diagonal-handle points for true corners and terminals.

## Step-Through Process

1. Run the focused loop.
2. Open the weakest `structure_<glyph>.svg` overlay.
3. Compare the source GLIF to the traced GLIF.
4. Identify which stage is wrong:
   - lost extrema
   - false corner cluster
   - true straight span emitted as curves
   - noisy extra on-curves
   - bad H/V handle placement
5. Change one algorithm step.
6. Rerun the focused loop and compare `structural_summary.txt`.

For one-glyph inspection with split debug:

```sh
./eval-harness/inspect_glyph.sh s s-default
./eval-harness/inspect_glyph.sh s s-accuracy2 --accuracy 2
./eval-harness/inspect_glyph.sh n n-default
```

This writes a persistent folder under `eval-harness/inspect/<label>` with the
input PNG, traced UFO, split debug log, structural report, and overlay.

Compare archived runs:

```sh
python3 eval-harness/compare_structural_runs.py baseline accuracy2
```

The comparison report is the main gate for settings and algorithm changes. A
single glyph improvement is useful evidence, but the default should not change
unless the focused set improves without hiding a large regression in one of the
reference structures.

Run the focused structural gate:

```sh
python3 eval-harness/check_structural_gate.py transition050 \
  --require-improved ampersand:0.01 \
  --require-improved s:0.05 \
  --require-not-worse n \
  --require-not-worse O
```

This gate checks the focused reference set against the archived baseline. It
allows small tradeoffs, but requires the mean structural score to improve,
requires several glyphs to move forward, protects `n` and `O` from regression,
and makes the ampersand/`s` improvements explicit.

To test the current checkout instead of an existing archive:

```sh
RUN_LABEL=current-transition050 ./eval-harness/run_structural_gate.sh
```

This runs the focused loop, archives the current result under
`eval-harness/runs/<RUN_LABEL>`, then applies the focused structural gate to
that exact archive. The script uses `eval-harness/baselines/current.tsv` as the
regression baseline, so a candidate must be at least as good as the current
known-good structure. Use `compare_structural_runs.py` to inspect possible
improvements beyond that floor.

## Current Findings

> **Historical (pre sub-pixel-only).** The findings below were recorded
> while the legacy binary pipeline (`curve.rs`/`polygon.rs`) was still in
> the tree. That pipeline and its constants (`CURVATURE_TRANSITION_THRESHOLD`,
> `FALSE_CORNER_GENUINE_ALPHA`, …) have since been removed; the sub-pixel
> pipeline (`subpixel`/`fit`/`refine`) is now the only path. Kept as
> a record of past experiments — do not tune the named constants, they no
> longer exist.

- Baseline focused loop: mean IoU `93.77%`, mean vector score `0.824`,
  mean structural score `0.859`.
- Raising `CURVATURE_TRANSITION_THRESHOLD` from `0.37` to `0.50` is the first
  applied structural improvement. Focused mean IoU improves from `93.77%` to
  `94.70%`, mean vector score from `0.824` to `0.859`, and mean structural
  score from `0.859` to `0.893`. The strongest gains are `s`, `e`, `R`, and
  `ampersand`; `a` is the only focused glyph that regresses structurally.
- Raising `FALSE_CORNER_GENUINE_ALPHA` globally is not enough. `1.10` improves
  `s` relative to the original baseline but damages `ampersand`, `a`, `e`,
  `S`, and `n`. `1.00` passes the old gate but is still worse than the
  `transition050` checkpoint. Future work should use a narrower cluster rule,
  not a broader global alpha threshold.
- Raising `CURVATURE_TRANSITION_THRESHOLD` globally from `0.50` to `0.55`
  improves `S` and slightly improves `ampersand`, but regresses `a`, `R`, and
  `n`. A contour-complexity rule is safer: use `0.55` only for contours with at
  least 100 polygon vertices. This improves `S` from `0.850` to `0.878` while
  leaving the rest of the focused set unchanged against `transition050`.
- `extrema-deviation-guard` adds post-fit validation without changing trace
  parameters: meaningful cubic extrema are promoted to on-curve points, severe
  long-section cubic divergence is split/refit with bounded recursion, and
  nearly-straight fitted cubics are demoted back to lines. The focused gate
  passes with mean structural score `0.898` (from current `0.896`), improving
  `ampersand`, `a`, and `s` with no protected-glyph regressions.
