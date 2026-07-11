#!/usr/bin/env python3
"""Degradation corpus: render known glyphs, degrade them across the input
space, trace each under every preset, and score both ways.

For each (glyph, tier, preset) it records the no-reference features
(`img2bez stats`), the reference-free `wild` judge score (score_wild.py), and
the reference-based structural score (structural_report.compare_glyphs, possible
because the source glyph is known). From that it reports:

  - judge<->reference agreement: does the judge's best preset match the
    reference's best preset per (glyph, tier)? Plus Pearson(wild, ref).
  - the feature->preset map: per tier, the mean stats and which preset wins —
    the data a detection rule (or learned selector) is calibrated against.

Usage:
  ./build_corpus.py [--glyphs a e n o s g] [--bin ../target/release/img2bez]
                    [--ref reference.ufo] [--out corpus-work]
"""
from __future__ import annotations

import argparse
import json
import shutil
import statistics as stats_mod
import subprocess
import sys
from pathlib import Path

import score_wild as sw
import structural_report as sr
from render_glyph import render_glyph

# Degradation tiers spanning the input space. Each maps to degrade.py flags.
TIERS: dict[str, list[str]] = {
    "clean": [],                                              # the render itself
    "soft": ["--blur", "4", "--contrast", "70:185"],          # os-001-like soft scan
    "soft_hard": ["--blur", "6", "--contrast", "55:170", "--noise", "5"],
    "blur": ["--blur", "3"],                                  # blurred, high contrast
    "lowres": ["--downscale", "96"],                          # small sprite
    "noisy": ["--noise", "14", "--jpeg", "40"],               # sensor + codec
    "bilevel": ["--bilevel", "128"],                          # crisp B/W (G-like)
    "nn2x": ["--downscale", "150", "--nn-upscale", "2"],      # NN upscale
}

PRESETS: dict[str, list[str]] = {
    "raw": ["--no-auto-pre-blur"],
    "default": [],
    "clean": ["--profile", "clean", "--no-auto-pre-blur"],
}

TOL = 18.0  # match tolerance for the reference comparison (gate default)


def stats_of(image: Path, binary: str) -> dict:
    out = subprocess.run([binary, "stats", str(image)], capture_output=True, text=True)
    return json.loads(out.stdout) if out.returncode == 0 and out.stdout.strip() else {}


def score_one(image: Path, glyph: str, binary: str, flags: list[str], refg) -> dict:
    """One trace -> (wild judge score, reference-based score, oncurves)."""
    thr, ufo = sw.trace(image, glyph, binary, flags)
    repro = sw.reproduction_iou(image, ufo, glyph, thr)
    struct = sw.structure(ufo, glyph)
    wild = sw.W_REPRO * repro + sw.W_STRUCT * struct["structure"]
    traced = sr.load_glif(sr.resolve_glif(ufo, glyph), glyph)
    traced = sr.align_to_reference(traced, refg)
    ref = sr.compare_glyphs(glyph, refg, traced, TOL)["score"]
    return {"wild": wild, "ref": ref, "oncurves": struct["oncurves"]}


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--glyphs", nargs="+", default=["a", "e", "n", "o", "s", "g"])
    ap.add_argument("--bin", default="../target/release/img2bez")
    ap.add_argument("--ref", type=Path, default=Path("reference.ufo"))
    ap.add_argument("--out", type=Path, default=Path("corpus-work"))
    args = ap.parse_args()
    args.out.mkdir(exist_ok=True)

    rows: list[dict] = []
    for glyph in args.glyphs:
        refg = sr.load_glif(sr.resolve_glif(args.ref, glyph), glyph)
        clean = args.out / f"{glyph}_render.png"
        render_glyph(args.ref, glyph, clean, px_height=700)
        for tier, dflags in TIERS.items():
            img = args.out / f"{glyph}_{tier}.png"
            if dflags:
                subprocess.run([sys.executable, "degrade.py", str(clean), str(img), *dflags],
                               capture_output=True, text=True)
            else:
                shutil.copy(clean, img)
            st = stats_of(img, args.bin)
            scored = {}
            for name, pflags in PRESETS.items():
                try:
                    scored[name] = score_one(img, glyph, args.bin, pflags, refg)
                except Exception as e:  # noqa: BLE001
                    print(f"  skip {glyph}/{tier}/{name}: {e}", file=sys.stderr)
            if len(scored) != len(PRESETS):
                continue
            jbest = max(scored, key=lambda n: scored[n]["wild"])
            rbest = max(scored, key=lambda n: scored[n]["ref"])
            rows.append({
                "glyph": glyph, "tier": tier,
                "sharp": st.get("sharpness", 0.0), "bilevel": st.get("bilevelness", 0.0),
                "noise": st.get("noise", 0.0), "extent": st.get("extentPx", 0),
                "jbest": jbest, "rbest": rbest, "agree": jbest == rbest,
                **{f"wild_{n}": scored[n]["wild"] for n in PRESETS},
                **{f"ref_{n}": scored[n]["ref"] for n in PRESETS},
                **{f"pts_{n}": scored[n]["oncurves"] for n in PRESETS},
            })

    if not rows:
        print("no rows", file=sys.stderr)
        return 1

    # Full table -> TSV
    tsv = args.out / "corpus.tsv"
    cols = list(rows[0].keys())
    tsv.write_text("\t".join(cols) + "\n" + "\n".join(
        "\t".join(f"{r[c]:.3f}" if isinstance(r[c], float) else str(r[c]) for c in cols) for r in rows
    ))

    # judge<->reference agreement + correlation
    agree = sum(r["agree"] for r in rows)
    pairs = [(r[f"wild_{n}"], r[f"ref_{n}"]) for r in rows for n in PRESETS]
    ws = [p[0] for p in pairs]; rs = [p[1] for p in pairs]
    mw, mr = stats_mod.mean(ws), stats_mod.mean(rs)
    cov = sum((w - mw) * (r - mr) for w, r in pairs)
    dw = sum((w - mw) ** 2 for w in ws) ** 0.5
    dr = sum((r - mr) ** 2 for r in rs) ** 0.5
    pear = cov / (dw * dr) if dw * dr else 0.0

    # feature -> winning-preset map, per tier
    print(f"\n=== feature -> winning preset (per tier, mean over {len(args.glyphs)} glyphs) ===")
    print(f"{'tier':10s} {'sharp':>7s} {'bilevel':>8s} {'extent':>7s}  judge-winner   default-matches-judge?")
    for tier in TIERS:
        trows = [r for r in rows if r["tier"] == tier]
        if not trows:
            continue
        msharp = stats_mod.mean(r["sharp"] for r in trows)
        mbi = stats_mod.mean(r["bilevel"] for r in trows)
        mext = stats_mod.mean(r["extent"] for r in trows)
        # most common judge-winner this tier
        winners = [r["jbest"] for r in trows]
        win = max(set(winners), key=winners.count)
        # does default tie/beat the judge winner on wild score, on average?
        ddef = stats_mod.mean(r["wild_default"] for r in trows)
        dwin = stats_mod.mean(r[f"wild_{win}"] for r in trows)
        match = "yes" if ddef >= dwin - 0.005 else f"no (default {ddef:.3f} < {win} {dwin:.3f})"
        print(f"{tier:10s} {msharp:7.0f} {mbi:8.3f} {mext:7.0f}  {win:12s}   {match}")

    print(f"\n=== judge vs reference ===")
    print(f"per-(glyph,tier) winner agreement: {agree}/{len(rows)} ({100*agree/len(rows):.0f}%)")
    print(f"Pearson(wild, reference_score) over {len(pairs)} traces: {pear:.3f}")
    print(f"\nfull table: {tsv}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
