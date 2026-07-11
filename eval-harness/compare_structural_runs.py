#!/usr/bin/env python3
"""Compare two archived structural reports.

Use after `RUN_LABEL=<name> ./eval-harness/run_structural_loop.sh ...`.
The output is intentionally small: glyph score delta, matched-reference delta,
extra-point delta, line-mix delta, and H/V handle delta.
"""

from __future__ import annotations

import argparse
import csv
from pathlib import Path


def main() -> int:
    args = parse_args()
    before = load_report(resolve_report(args.before))
    after = load_report(resolve_report(args.after))

    glyphs = sorted(set(before) | set(after))
    print(f"before: {args.before}")
    print(f"after:  {args.after}")
    print("")
    print(
        "\t".join(
            [
                "glyph",
                "score",
                "matched",
                "extra",
                "line_mix",
                "hv",
                "oncurves",
                "segments",
            ]
        )
    )

    score_deltas = []
    for glyph in glyphs:
        b = before.get(glyph)
        a = after.get(glyph)
        if not b or not a:
            print(f"{glyph}\tmissing\tmissing\tmissing\tmissing\tmissing\tmissing\tmissing")
            continue

        score_deltas.append(delta_float(a["score"], b["score"]))
        print(
            "\t".join(
                [
                    glyph,
                    fmt_delta(delta_float(a["score"], b["score"])),
                    fmt_ratio_delta(a["ref_oncurves_matched"], b["ref_oncurves_matched"]),
                    fmt_delta(delta_int(a["traced_unmatched"], b["traced_unmatched"]), invert=True),
                    fmt_delta(delta_float(a["line_mix"], b["line_mix"])),
                    fmt_delta(delta_float(a["hv"], b["hv"])),
                    f"{b['oncurves']} -> {a['oncurves']}",
                    f"{b['segments']} -> {a['segments']}",
                ]
            )
        )

    if score_deltas:
        mean_delta = sum(score_deltas) / len(score_deltas)
        improved = sum(1 for d in score_deltas if d > 0)
        worsened = sum(1 for d in score_deltas if d < 0)
        print("")
        print(f"mean_score_delta: {mean_delta:+.3f}")
        print(f"glyphs_improved: {improved}")
        print(f"glyphs_worsened: {worsened}")

    return 0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("before", help="Run label, run directory, or structural_report.tsv")
    parser.add_argument("after", help="Run label, run directory, or structural_report.tsv")
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


def delta_float(after: str, before: str) -> float:
    return float(after) - float(before)


def delta_int(after: str, before: str) -> int:
    return int(after) - int(before)


def fmt_delta(value: float | int, *, invert: bool = False) -> str:
    if invert:
        value = -value
    if isinstance(value, int):
        return f"{value:+d}"
    return f"{value:+.3f}"


def fmt_ratio_delta(after: str, before: str) -> str:
    after_num, _ = parse_ratio(after)
    before_num, _ = parse_ratio(before)
    return f"{after_num - before_num:+d}"


def parse_ratio(value: str) -> tuple[int, int]:
    left, right = value.split("/", 1)
    return int(left), int(right)


if __name__ == "__main__":
    raise SystemExit(main())
