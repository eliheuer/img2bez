#!/usr/bin/env python3
"""Inspect the img2bez trace-collection log (the real-data pipeline).

Each trace run with `--log PATH` (or the `IMG2BEZ_LOG` env var) appends one
JSONL record: the image's no-reference features, the settings used, and the
output point counts. Over the Codex/Runebender workflow this accumulates real
`(image features -> settings -> output)` examples. This tool summarizes the
dataset so you can see its coverage and when there is enough of the hard class
(soft/photo scans) to train an input-adaptive selector on.

Why this exists and not a synthetic-corpus trainer: the degradation corpus can't
reproduce the soft-textured (photo) class, so a selector trained on it just
relearns "always wild". Real logged traces are the way to get that class. See
docs/input-adaptation.md and the selector-data finding.

Usage:
    tracelog.py [LOG] [--per-image]
    LOG defaults to $IMG2BEZ_LOG.
"""

import argparse
import json
import os
import sys
from collections import Counter, defaultdict

# Mirror of ImageStats::classify (src/imagestats.rs): the soft/photo class is
# low edge sharpness AND low bilevelness; everything else is wild. Kept in sync
# by hand — if the Rust thresholds move, move these too.
SOFT_SHARPNESS_MAX = 80.0
SOFT_BILEVEL_MAX = 0.30


def classify(sharpness, bilevelness):
    if sharpness < SOFT_SHARPNESS_MAX and bilevelness < SOFT_BILEVEL_MAX:
        return "photo"
    return "wild"


def load(path):
    rows = []
    with open(path) as f:
        for ln in f:
            ln = ln.strip()
            if ln:
                rows.append(json.loads(ln))
    return rows


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument(
        "log",
        nargs="?",
        default=os.environ.get("IMG2BEZ_LOG"),
        help="JSONL trace log (default: $IMG2BEZ_LOG)",
    )
    ap.add_argument(
        "--per-image",
        action="store_true",
        help="list the settings tried on each image (last ~ accepted)",
    )
    args = ap.parse_args()
    if not args.log:
        sys.exit("no log path: pass one or set IMG2BEZ_LOG")
    if not os.path.exists(args.log):
        sys.exit(f"no such log: {args.log}")

    rows = load(args.log)
    if not rows:
        print("empty log")
        return

    by_img = defaultdict(list)
    for r in rows:
        by_img[r["imageHash"]].append(r)

    # Class balance over UNIQUE images (the detected class of each source).
    classes = Counter()
    for rs in by_img.values():
        f = rs[0]["features"]
        classes[classify(f["sharpness"], f["bilevelness"])] += 1

    print(f"records: {len(rows)}   unique images: {len(by_img)}")
    print(f"detected class (unique images): {dict(classes)}")

    print("feature coverage (min .. max over all records):")
    for k, label in [
        ("sharpness", "sharpness"),
        ("bilevelness", "bilevel"),
        ("extentPx", "extent_px"),
        ("noise", "noise"),
    ]:
        vs = [r["features"][k] for r in rows]
        print(f"  {label:11} {min(vs):10.3f} .. {max(vs):.3f}")

    variety = Counter()
    for r in rows:
        s = r["settings"]
        variety[(s["profile"], s["mode"], s["style"])] += 1
    print("settings used (profile, mode, style -> count):")
    for k, c in variety.most_common():
        print(f"  {k[0]}/{k[1]}/{k[2]:8} -> {c}")

    # Readiness gauge: a learned selector needs the hard class represented and
    # enough distinct images. These thresholds are a starting heuristic.
    photo = classes.get("photo", 0)
    ready = photo >= 20 and len(by_img) >= 80
    print(
        f"\nreadiness: {len(by_img)} images, {photo} photo-class -> "
        + (
            "enough to start probing a selector."
            if ready
            else "keep collecting (the bottleneck is real soft/photo samples)."
        )
    )

    if args.per_image:
        print("\nper-image settings tried (-> order; last ~ accepted):")
        for h, rs in by_img.items():
            f = rs[0]["features"]
            cls = classify(f["sharpness"], f["bilevelness"])
            tried = [
                f"{r['settings']['effectiveProfile']}/{r['settings']['mode']}"
                f"/{r['settings']['style']}/acc{r['settings']['accuracy']:g}"
                f"={r['output']['onCurve']}+{r['output']['offCurve']}"
                for r in rs
            ]
            g = rs[0]["glyph"]
            print(f"  {h[:8]} {g:2} [{cls:5}] x{len(rs)}: {' -> '.join(tried)}")


if __name__ == "__main__":
    main()
