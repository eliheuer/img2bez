#!/usr/bin/env python3
"""Pass/fail gate for focused structural tracing experiments."""

from __future__ import annotations

import argparse
import csv
from pathlib import Path


DEFAULT_GLYPHS = ["ampersand", "a", "e", "s", "R", "O", "S", "n"]


def main() -> int:
    args = parse_args()
    baseline = load_report(resolve_report(args.baseline))
    candidate = load_report(resolve_report(args.candidate))
    glyphs = resolve_glyphs(args)

    missing = [glyph for glyph in glyphs if glyph not in baseline or glyph not in candidate]
    if missing:
        print("FAIL: missing glyph reports: " + ", ".join(missing))
        return 1

    deltas = {glyph: float(candidate[glyph]["score"]) - float(baseline[glyph]["score"]) for glyph in glyphs}
    tolerated = tolerated_raster_improved_regressions(args, baseline, candidate, deltas)
    effective_deltas = {
        glyph: (0.0 if glyph in tolerated else delta)
        for glyph, delta in deltas.items()
    }
    mean_delta = sum(effective_deltas.values()) / len(effective_deltas)
    improved = [glyph for glyph, delta in effective_deltas.items() if delta > 0]
    worsened = [glyph for glyph, delta in effective_deltas.items() if delta < 0]

    failures = []
    if mean_delta < args.min_mean_delta:
        failures.append(f"mean delta {mean_delta:+.3f} < required {args.min_mean_delta:+.3f}")
    if len(improved) < args.min_improved:
        failures.append(f"improved glyphs {len(improved)} < required {args.min_improved}")
    if len(worsened) > args.max_worsened:
        failures.append(f"worsened glyphs {len(worsened)} > allowed {args.max_worsened}")
    for requirement in args.require_improved:
        glyph, min_delta = parse_delta_requirement(requirement)
        actual = deltas.get(glyph)
        if actual is None:
            failures.append(f"required glyph {glyph!r} is not in report")
        elif actual < min_delta:
            failures.append(f"{glyph} delta {actual:+.3f} < required {min_delta:+.3f}")
    for requirement in args.require_not_worse:
        glyph, max_loss = parse_delta_requirement(requirement, default=0.0)
        actual = deltas.get(glyph)
        if actual is None:
            failures.append(f"required glyph {glyph!r} is not in report")
        elif actual < -max_loss:
            if glyph not in tolerated:
                failures.append(f"{glyph} delta {actual:+.3f} is worse than allowed {-max_loss:+.3f}")
    for requirement in args.max_oncurve_ratio:
        glyph, max_ratio = parse_delta_requirement(requirement)
        row = candidate.get(glyph)
        if row is None:
            failures.append(f"on-curve ratio glyph {glyph!r} is not in report")
            continue
        ratio = parse_count_ratio(row.get("oncurves", ""))
        if ratio is None:
            failures.append(f"{glyph} has unparsable oncurve count {row.get('oncurves')!r}")
        elif ratio > max_ratio:
            failures.append(f"{glyph} oncurve ratio {ratio:.3f} > allowed {max_ratio:.3f}")

    status = "PASS" if not failures else "FAIL"
    print(f"{status}: focused structural gate")
    print(f"mean_score_delta: {mean_delta:+.3f}")
    print(f"glyphs_improved: {len(improved)} ({' '.join(improved)})")
    print(f"glyphs_worsened: {len(worsened)} ({' '.join(worsened)})")
    if tolerated:
        print(f"raster_tolerated: {' '.join(sorted(tolerated))}")
    for glyph in glyphs:
        print(f"  {glyph:10s} {deltas[glyph]:+.3f}")
    for failure in failures:
        print("fail: " + failure)
    return 1 if failures else 0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--baseline",
        default="baseline",
        help="Baseline run label, run directory, or structural_report.tsv",
    )
    parser.add_argument("candidate", help="Candidate run label, run directory, or structural_report.tsv")
    parser.add_argument("--glyph-file", default="eval-harness/focused_glyphs.txt")
    parser.add_argument("--min-mean-delta", type=float, default=0.02)
    parser.add_argument("--min-improved", type=int, default=4)
    parser.add_argument("--max-worsened", type=int, default=2)
    parser.add_argument(
        "--require-improved",
        action="append",
        default=[],
        metavar="GLYPH[:MIN_DELTA]",
        help="Require a glyph to improve by at least MIN_DELTA (default 0.001)",
    )
    parser.add_argument(
        "--require-not-worse",
        action="append",
        default=[],
        metavar="GLYPH[:MAX_LOSS]",
        help="Require a glyph to avoid regression larger than MAX_LOSS (default 0.0)",
    )
    parser.add_argument(
        "--max-oncurve-ratio",
        action="append",
        default=[],
        metavar="GLYPH:RATIO",
        help="Fail if traced/reference on-curve count exceeds RATIO for a glyph.",
    )
    parser.add_argument(
        "--allow-raster-improved-regression",
        action="append",
        default=[],
        metavar="GLYPH:MAX_SCORE_LOSS:MIN_IOU_GAIN",
        help=(
            "Treat a glyph's score regression as neutral when raster IoU improves "
            "by at least MIN_IOU_GAIN percentage points and the score loss is at "
            "most MAX_SCORE_LOSS."
        ),
    )
    return parser.parse_args()


def resolve_report(value: str) -> Path:
    path = Path(value)
    candidates = [
        path / "structural_report.tsv",
        path,
        Path("eval-harness") / "runs" / value / "structural_report.tsv",
    ]
    for candidate in candidates:
        if candidate.is_file():
            return candidate
    raise SystemExit(f"ERROR: structural report not found for {value}")


def load_report(path: Path) -> dict[str, dict[str, str]]:
    with open(path, "r", encoding="utf-8", newline="") as f:
        rows = list(csv.DictReader(f, delimiter="\t"))
    return {row["glyph"]: row for row in rows if row.get("status") == "ok"}


def resolve_glyphs(args: argparse.Namespace) -> list[str]:
    path = Path(args.glyph_file)
    if not path.is_file():
        return DEFAULT_GLYPHS
    glyphs = []
    with open(path, "r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if line and not line.startswith("#"):
                glyphs.append(line)
    return glyphs or DEFAULT_GLYPHS


def parse_delta_requirement(value: str, *, default: float = 0.001) -> tuple[str, float]:
    if ":" not in value:
        return value, default
    glyph, min_delta = value.split(":", 1)
    return glyph, float(min_delta)


def parse_count_ratio(value: str) -> float | None:
    if "/" not in value:
        return None
    traced, reference = value.split("/", 1)
    try:
        traced_count = int(traced)
        reference_count = int(reference)
    except ValueError:
        return None
    if reference_count <= 0:
        return 1.0 if traced_count == 0 else None
    return traced_count / reference_count


def tolerated_raster_improved_regressions(
    args: argparse.Namespace,
    baseline: dict[str, dict[str, str]],
    candidate: dict[str, dict[str, str]],
    deltas: dict[str, float],
) -> set[str]:
    tolerated: set[str] = set()
    for requirement in args.allow_raster_improved_regression:
        parts = requirement.split(":")
        if len(parts) != 3:
            raise SystemExit(
                "ERROR: --allow-raster-improved-regression must be GLYPH:MAX_SCORE_LOSS:MIN_IOU_GAIN"
            )
        glyph, max_loss_text, min_iou_gain_text = parts
        if glyph not in baseline or glyph not in candidate:
            continue
        max_loss = float(max_loss_text)
        min_iou_gain = float(min_iou_gain_text)
        delta = deltas.get(glyph, 0.0)
        if delta >= 0 or delta < -max_loss:
            continue
        baseline_iou = parse_float(baseline[glyph].get("raster_iou"))
        candidate_iou = parse_float(candidate[glyph].get("raster_iou"))
        if baseline_iou is None or candidate_iou is None:
            continue
        if candidate_iou - baseline_iou >= min_iou_gain:
            tolerated.add(glyph)
    return tolerated


def parse_float(value: str | None) -> float | None:
    if value is None or value == "":
        return None
    try:
        return float(value)
    except ValueError:
        return None


if __name__ == "__main__":
    raise SystemExit(main())
