#!/usr/bin/env python3
"""Score a trace of an UNKNOWN-source raster image (no ground-truth UFO).

The known-source loop (run_structural_loop.sh) renders a reference glyph, traces
it, and compares the result *structurally* to the original outline. Wild images
have no original, so "is this trace good?" needs a reference-free answer.

This scores two things, both without ground truth:

  reproduction  Render the trace back to a bitmap at the input's resolution and
                take IoU against the (thresholded) input. This is the diffvg
                trick the refiner already uses, exposed via img2bez's built-in
                IMG2BEZ_DEBUG_PIXELDIFF instrumentation. High = matches pixels.

  structure     Regularizers that encode the type-design rules from the blog
                with no reference needed:
                  hv_frac    fraction of curve handles leaving exactly H or V
                  micro      penalty for vestigial micro-segments (the foot
                             cluster that over-segments terminals/junctions)
                  parsimony  penalty for too many points per unit of outline

High reproduction with poor structure is exactly the failure mode that the
desktop 'a' shows: it matches the pixels but litters the terminal with points.
The combined score balances the two, and every sub-metric is printed so an
eval-harness loop (or a human) can see which lever moved.

Usage:
  ./score_wild.py IMAGE [IMAGE ...] [--glyph NAME] [--bin img2bez]
  ./score_wild.py --dir sources/wild
  # pass extra img2bez flags after `--`, e.g. tuning a candidate profile:
  ./score_wild.py sources/wild/a.png -- --accuracy 3.0 --no-refine
"""
from __future__ import annotations

import argparse
import re
import subprocess
import sys
import tempfile
from pathlib import Path

from PIL import Image, ImageChops, ImageDraw

import structural_report as sr  # reuse the glif parser + H/V + stats machinery

# --- tuning constants (font units; the loop is expected to sweep these) ------
MICRO_LEN = 14.0          # on-curve segments shorter than this are vestigial
TARGET_DENSITY = 5.0      # target on-curve points per 1000 units of perimeter
KINK_TOL_DEG = 3.0        # tangent break below this is grid rounding
KINK_CORNER_DEG = 45.0    # break above this is a designed corner
KINK_PENALTY = 0.15       # smoothness cost per accidental kink
RASTER_H = 512            # render height for the reproduction-IoU comparison
CURVE_SAMPLES = 16        # flattening samples per cubic segment
# combined-score weights (reproduction vs structure, and within structure)
W_REPRO, W_STRUCT = 0.5, 0.5
W_HV, W_SMOOTH, W_MICRO, W_PARSIMONY = 0.30, 0.30, 0.15, 0.25

THRESHOLD_RE = re.compile(r"Threshold\s+\w+\s*=\s*(\d+)")


def trace(image: Path, glyph: str, binary: str, extra: list[str]) -> tuple[int, Path]:
    """Trace `image`, returning (threshold_used, traced_ufo_path)."""
    out = Path(tempfile.mkdtemp(prefix="wild_")) / f"{glyph}.ufo"
    cmd = [binary, "--input", str(image), "--output", str(out), "--name", glyph, *extra]
    proc = subprocess.run(cmd, capture_output=True, text=True, env=_env())
    if proc.returncode != 0:
        raise RuntimeError(f"img2bez failed on {image}:\n{proc.stderr}")
    m = THRESHOLD_RE.search(proc.stderr)
    threshold = int(m.group(1)) if m else 128
    return threshold, out


def _flatten(contour) -> list[tuple[float, float]]:
    """Flatten a glif contour to a y-up polygon (cubic curves sampled)."""
    pts = contour.points
    n = len(pts)
    # rotate so we start on an on-curve point
    start = next((i for i, p in enumerate(pts) if p.oncurve), 0)
    seq = [pts[(start + k) % n] for k in range(n)]
    poly: list[tuple[float, float]] = [(seq[0].x, seq[0].y)]
    i = 1
    while i <= len(seq):
        p = seq[i % len(seq)]
        if p.oncurve:
            poly.append((p.x, p.y))
            i += 1
        else:  # two off-curves then an on-curve (cubic)
            c1 = seq[i % len(seq)]
            c2 = seq[(i + 1) % len(seq)]
            end = seq[(i + 2) % len(seq)]
            p0 = poly[-1]
            for s in range(1, CURVE_SAMPLES + 1):
                t = s / CURVE_SAMPLES
                u = 1 - t
                x = u**3 * p0[0] + 3*u*u*t*c1.x + 3*u*t*t*c2.x + t**3*end.x
                y = u**3 * p0[1] + 3*u*u*t*c1.y + 3*u*t*t*c2.y + t**3*end.y
                poly.append((x, y))
            i += 3
    return poly


def _trace_mask(glyph) -> Image.Image | None:
    """Rasterize a traced glyph to a 1-bit mask (even-odd via XOR), y flipped."""
    polys = [_flatten(c) for c in glyph.contours if len(c.points) >= 3]
    polys = [p for p in polys if len(p) >= 3]
    if not polys:
        return None
    xs = [x for p in polys for x, _ in p]
    ys = [y for p in polys for _, y in p]
    minx, maxx, miny, maxy = min(xs), max(xs), min(ys), max(ys)
    w_units, h_units = maxx - minx, maxy - miny
    if w_units <= 0 or h_units <= 0:
        return None
    scale = RASTER_H / h_units
    w = max(1, round(w_units * scale))
    h = RASTER_H
    mask = Image.new("1", (w, h), 0)
    for poly in polys:  # even-odd: XOR each contour's fill
        layer = Image.new("1", (w, h), 0)
        d = ImageDraw.Draw(layer)
        # font units are y-up; image is y-down -> flip y
        d.polygon([((x - minx) * scale, (maxy - y) * scale) for x, y in poly], fill=1)
        mask = ImageChops.logical_xor(mask, layer)
    return mask


def _source_mask(image: Path, threshold: int) -> Image.Image:
    """Glyph mask from the input raster, cropped to the glyph bbox."""
    im = Image.open(image).convert("L")
    t = threshold
    if not 0 < t < 255:
        # img2bez can report an extreme/degenerate threshold (e.g. 0) on hard
        # bilevel images with no anti-aliasing. A literal `v < 0` cut yields an
        # empty mask and a bogus repro_iou of 0, so recover a usable cut from the
        # histogram midpoint between the darkest and lightest pixels.
        lo, hi = im.getextrema()
        t = (lo + hi) // 2 if hi > lo else 128
        t = min(254, max(1, t))
    # glyph = pixels darker than the cut (dark-on-light)
    mask = im.point(lambda v: 255 if v < t else 0, mode="L").convert("1")
    bbox = mask.getbbox()
    return mask.crop(bbox) if bbox else mask


def reproduction_iou(image: Path, ufo: Path, glyph: str, threshold: int) -> float:
    """Bbox-normalized IoU between the traced shape and the input glyph mask."""
    glif = sr.resolve_glif(ufo, glyph)
    g = sr.load_glif(glif, glyph)
    tmask = _trace_mask(g)
    if tmask is None:
        return 0.0
    smask = _source_mask(image, threshold)
    tbox = tmask.getbbox()
    if tbox:
        tmask = tmask.crop(tbox)
    # normalize both to the same frame, then IoU
    tmask = tmask.resize(smask.size)
    inter = ImageChops.logical_and(smask, tmask)
    union = ImageChops.logical_or(smask, tmask)
    ic = sum(inter.point(lambda v: 1 if v else 0).getdata())
    uc = sum(union.point(lambda v: 1 if v else 0).getdata())
    return ic / uc if uc else 0.0


def structure(ufo: Path, glyph: str) -> dict[str, float]:
    """Reference-free structural metrics from the traced glif."""
    glif = sr.resolve_glif(ufo, glyph)
    if glif is None:
        raise RuntimeError(f"no traced glif for {glyph} in {ufo}")
    g = sr.load_glif(glif, glyph)
    st = sr.stats(g)

    hv_frac = st.hv_aligned / st.hv_total if st.hv_total else 1.0

    # Accidental kinks: a tangent break at a join too big for grid rounding
    # and too small to be a designed corner is always an accident.
    import math as _math
    kinks = 0
    for contour in g.contours:
        pts = contour.points
        n = len(pts)
        for i, p in enumerate(pts):
            if not p.oncurve:
                continue
            prev, nxt = pts[(i - 1) % n], pts[(i + 1) % n]
            v_in = (p.x - prev.x, p.y - prev.y)
            v_out = (nxt.x - p.x, nxt.y - p.y)
            li, lo = _math.hypot(*v_in), _math.hypot(*v_out)
            if li < 1e-9 or lo < 1e-9:
                continue
            cos = max(-1.0, min(1.0, (v_in[0]*v_out[0] + v_in[1]*v_out[1]) / (li*lo)))
            theta = _math.degrees(_math.acos(cos))
            if KINK_TOL_DEG < theta < KINK_CORNER_DEG:
                kinks += 1
    smooth = max(0.0, 1.0 - kinks * KINK_PENALTY)

    perimeter = 0.0
    micro = 0
    for contour in g.contours:
        on = [p for p in contour.points if p.oncurve]
        for i in range(len(on)):
            d = sr.distance(on[i], on[(i + 1) % len(on)])
            perimeter += d
            if d < MICRO_LEN:
                micro += 1

    oncurves = st.oncurves
    density = oncurves / (perimeter / 1000.0) if perimeter else 0.0
    micro_clean = max(0.0, 1.0 - micro / oncurves) if oncurves else 1.0
    parsimony = min(1.0, TARGET_DENSITY / density) if density else 1.0
    struct = (
        W_HV * hv_frac
        + W_SMOOTH * smooth
        + W_MICRO * micro_clean
        + W_PARSIMONY * parsimony
    )
    return {
        "oncurves": oncurves,
        "hv_frac": hv_frac,
        "kinked_joins": kinks,
        "smooth": smooth,
        "micro_segs": micro,
        "micro_clean": micro_clean,
        "density": density,
        "parsimony": parsimony,
        "structure": struct,
    }


def score_image(image: Path, glyph: str, binary: str, extra: list[str]) -> dict[str, float]:
    threshold, ufo = trace(image, glyph, binary, extra)
    repro_iou = reproduction_iou(image, ufo, glyph, threshold)
    s = structure(ufo, glyph)
    wild = W_REPRO * repro_iou + W_STRUCT * s["structure"]
    return {"image": image.name, "repro_iou": repro_iou, **s, "wild": wild}


def _env() -> dict:
    import os
    return dict(os.environ)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("images", nargs="*", type=Path)
    ap.add_argument("--dir", type=Path, help="score every image in this directory")
    ap.add_argument("--glyph", default=None, help="glyph name (default: file stem)")
    ap.add_argument("--bin", default="img2bez", help="img2bez binary")
    # Everything after a literal `--` is passed verbatim to img2bez (including
    # valued flags like `--accuracy 4.0`), so split before argparse sees it.
    argv = sys.argv[1:]
    extra: list[str] = []
    if "--" in argv:
        cut = argv.index("--")
        argv, extra = argv[:cut], argv[cut + 1:]
    args = ap.parse_args(argv)

    images = list(args.images)
    if args.dir:
        images += sorted(p for p in args.dir.iterdir() if p.suffix.lower() in {".png", ".jpg", ".jpeg", ".bmp"})
    if not images:
        ap.error("no images given (pass paths or --dir)")

    rows = []
    for img in images:
        glyph = args.glyph or re.sub(r"\W", "_", img.stem) or "glyph"
        try:
            rows.append(score_image(img, glyph, args.bin, extra))
        except RuntimeError as e:
            print(f"  SKIP {img.name}: {e}", file=sys.stderr)

    if not rows:
        return 1
    hdr = ["image", "wild", "repro_iou", "hv_frac", "smooth", "kinked_joins", "micro_segs", "density", "parsimony", "oncurves"]
    print("\t".join(hdr))
    for r in rows:
        print("\t".join(
            f"{r[k]:.3f}" if isinstance(r[k], float) else str(r[k]) for k in hdr
        ))
    if len(rows) > 1:
        mean = sum(r["wild"] for r in rows) / len(rows)
        print(f"# mean wild score: {mean:.3f}  over {len(rows)} images", file=sys.stderr)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
