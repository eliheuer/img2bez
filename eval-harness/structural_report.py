#!/usr/bin/env python3
"""Compare traced GLIF structure against reference UFO GLIF structure.

This is intentionally more type-design focused than raster IoU. It asks whether
the traced outline has a similar editable point model: contour count, on-curve
count, line/curve mix, H/V handles, and on-curve placement near the reference.
"""

from __future__ import annotations

import argparse
import math
import os
import plistlib
import re
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path


DEFAULT_GLYPHS = ["ampersand", "a", "e", "s", "R", "O", "S", "n"]
DIAGNOSTICS_RE = re.compile(
    r"Validate\s+extrema_fixed=(?P<extrema>\d+)\s+divergence_splits=(?P<splits>\d+)"
    r"(?:\s+tangent_overrides=(?P<strong>\d+)/(?P<clean>\d+)/(?P<visible>\d+))?"
    r"(?:\s+tangent_rejects=(?P<rejects>\d+))?"
    r"(?:\s+oversegment_removed=(?P<oversegment>\d+))?"
    r"(?:\s+final_divergences=(?P<final>\d+))?"
    r"(?:\s+final_repairs=(?P<repair>\d+))?"
)
RASTER_IOU_RE = re.compile(r"Raster IoU\s+(?P<iou>\d+(?:\.\d+)?)%")
EVAL_SCORE_RE = re.compile(r"Overall\s+(?P<score>\d+(?:\.\d+)?)")


@dataclass
class Point:
    x: float
    y: float
    typ: str | None
    smooth: bool

    @property
    def oncurve(self) -> bool:
        return self.typ is not None


@dataclass
class Contour:
    points: list[Point]


@dataclass
class Glyph:
    name: str
    contours: list[Contour]


@dataclass
class Stats:
    contours: int
    oncurves: int
    offcurves: int
    lines: int
    curves: int
    smooth: int
    hv_aligned: int
    hv_total: int


def main() -> int:
    args = parse_args()
    glyphs = resolve_glyphs(args)
    out_dir = Path(args.output_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    rows = []
    for name in glyphs:
        diagnostics = read_trace_diagnostics(out_dir, name)
        ref_path = resolve_glif(Path(args.reference_ufo), name)
        traced_path = resolve_glif(Path(args.traced_ufo), name)
        if not ref_path or not traced_path:
            rows.append({
                "glyph": name,
                "status": "missing",
                "score": 0.0,
                **diagnostics,
                "note": "missing reference or traced glif",
            })
            continue

        reference = load_glif(ref_path, name)
        traced = load_glif(traced_path, name)
        aligned_traced = align_to_reference(traced, reference)
        row = compare_glyphs(name, reference, aligned_traced, args.match_tolerance)
        row.update(diagnostics)
        rows.append(row)
        write_overlay_svg(out_dir / f"structure_{safe_name(name)}.svg", reference, aligned_traced)

    write_tsv(out_dir / "structural_report.tsv", rows)
    write_summary(out_dir / "structural_summary.txt", rows)
    print_summary(rows)
    return 0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--reference-ufo", required=True)
    parser.add_argument("--traced-ufo", required=True)
    parser.add_argument("--glyph-file")
    parser.add_argument("--glyphs", nargs="*")
    parser.add_argument("--output-dir", default="eval-harness/work")
    parser.add_argument("--match-tolerance", type=float, default=18.0)
    return parser.parse_args()


def resolve_glyphs(args: argparse.Namespace) -> list[str]:
    if args.glyphs:
        return args.glyphs
    if args.glyph_file:
        glyphs = []
        with open(args.glyph_file, "r", encoding="utf-8") as f:
            for line in f:
                line = line.strip()
                if line and not line.startswith("#"):
                    glyphs.append(line)
        return glyphs
    return DEFAULT_GLYPHS


def resolve_glif(ufo: Path, glyph_name: str) -> Path | None:
    for glyphs_dir in ("glyphs", "glyphs.public.default"):
        contents_path = ufo / glyphs_dir / "contents.plist"
        if not contents_path.exists():
            continue
        with open(contents_path, "rb") as f:
            contents = plistlib.load(f)
        filename = contents.get(glyph_name)
        if filename:
            path = ufo / glyphs_dir / filename
            return path if path.exists() else None
    return None


def read_trace_diagnostics(out_dir: Path, glyph_name: str) -> dict[str, int]:
    path = out_dir / f"{glyph_log_name(glyph_name)}.log"
    diagnostics = {
        "raster_iou": 0.0,
        "eval_score": 0.0,
        "missed_extrema_fixed": 0,
        "high_deviation_splits": 0,
        "strong_tangent_overrides": 0,
        "clean_tangent_overrides": 0,
        "visible_tangent_overrides": 0,
        "rejected_tangent_near_misses": 0,
        "oversegmented_splits_removed": 0,
        "final_outline_divergences": 0,
        "final_outline_repairs": 0,
    }
    if not path.exists():
        return diagnostics
    text = path.read_text(encoding="utf-8", errors="replace")
    if iou_match := RASTER_IOU_RE.search(text):
        diagnostics["raster_iou"] = float(iou_match.group("iou"))
    if eval_match := EVAL_SCORE_RE.search(text):
        diagnostics["eval_score"] = float(eval_match.group("score"))
    match = DIAGNOSTICS_RE.search(text)
    if not match:
        return diagnostics
    diagnostics["missed_extrema_fixed"] = int(match.group("extrema"))
    diagnostics["high_deviation_splits"] = int(match.group("splits"))
    diagnostics["strong_tangent_overrides"] = int(match.group("strong") or 0)
    diagnostics["clean_tangent_overrides"] = int(match.group("clean") or 0)
    diagnostics["visible_tangent_overrides"] = int(match.group("visible") or 0)
    diagnostics["rejected_tangent_near_misses"] = int(match.group("rejects") or 0)
    diagnostics["oversegmented_splits_removed"] = int(match.group("oversegment") or 0)
    diagnostics["final_outline_divergences"] = int(match.group("final") or 0)
    diagnostics["final_outline_repairs"] = int(match.group("repair") or 0)
    return diagnostics


def glyph_log_name(name: str) -> str:
    if len(name) == 1:
        return f"uni{ord(name):04X}"
    return safe_name(name)


def load_glif(path: Path, name: str) -> Glyph:
    root = ET.parse(path).getroot()
    contours = []
    outline = root.find("outline")
    if outline is None:
        return Glyph(name, contours)
    for contour_el in outline.findall("contour"):
        points = []
        for point_el in contour_el.findall("point"):
            points.append(Point(
                x=float(point_el.attrib["x"]),
                y=float(point_el.attrib["y"]),
                typ=point_el.attrib.get("type"),
                smooth=point_el.attrib.get("smooth") == "yes",
            ))
        if points:
            contours.append(Contour(points))
    return Glyph(name, contours)


def compare_glyphs(name: str, reference: Glyph, traced: Glyph, tolerance: float) -> dict[str, object]:
    ref_stats = stats(reference)
    trace_stats = stats(traced)
    match = match_oncurves(reference, traced, tolerance)

    contour_score = exact_ratio(trace_stats.contours, ref_stats.contours)
    point_score = ratio_score(trace_stats.oncurves, ref_stats.oncurves)
    segment_score = ratio_score(
        trace_stats.lines + trace_stats.curves,
        ref_stats.lines + ref_stats.curves,
    )
    line_mix_score = fraction_score(
        trace_stats.lines,
        trace_stats.lines + trace_stats.curves,
        ref_stats.lines,
        ref_stats.lines + ref_stats.curves,
    )
    hv_score = trace_stats.hv_aligned / trace_stats.hv_total if trace_stats.hv_total else 1.0
    ref_match_score = match["reference_matched"] / ref_stats.oncurves if ref_stats.oncurves else 1.0
    extra_score = 1.0 - (match["traced_unmatched"] / trace_stats.oncurves) if trace_stats.oncurves else 1.0
    extra_score = max(0.0, extra_score)

    score = (
        ref_match_score * 0.25
        + line_mix_score * 0.20
        + hv_score * 0.20
        + point_score * 0.15
        + contour_score * 0.10
        + segment_score * 0.10
    )

    return {
        "glyph": name,
        "status": "ok",
        "score": score,
        "contours": f"{trace_stats.contours}/{ref_stats.contours}",
        "oncurves": f"{trace_stats.oncurves}/{ref_stats.oncurves}",
        "segments": f"{trace_stats.curves}c+{trace_stats.lines}l/{ref_stats.curves}c+{ref_stats.lines}l",
        "line_mix": line_mix_score,
        "hv": hv_score,
        "ref_oncurves_matched": f"{match['reference_matched']}/{ref_stats.oncurves}",
        "traced_unmatched": match["traced_unmatched"],
        "mean_ref_distance": match["mean_reference_distance"],
        "note": "",
    }


def stats(glyph: Glyph) -> Stats:
    oncurves = offcurves = lines = curves = smooth = 0
    hv_aligned = hv_total = 0
    for contour in glyph.contours:
        pts = contour.points
        for p in pts:
            if p.oncurve:
                oncurves += 1
                if p.typ == "line":
                    lines += 1
                elif p.typ in ("curve", "qcurve"):
                    curves += 1
                if p.smooth:
                    smooth += 1
            else:
                offcurves += 1
        aligned, total = hv_handles(pts)
        hv_aligned += aligned
        hv_total += total
    return Stats(len(glyph.contours), oncurves, offcurves, lines, curves, smooth, hv_aligned, hv_total)


def hv_handles(points: list[Point]) -> tuple[int, int]:
    aligned = total = 0
    for i, point in enumerate(points):
        if point.typ != "curve":
            continue
        prev_idx = previous_oncurve_index(points, i)
        controls = previous_offcurves(points, i)
        if prev_idx is None or len(controls) < 2:
            continue
        prev_on = points[prev_idx]
        c1, c2 = controls[-2], controls[-1]
        # A handle anchored at a diagonal-line tangent point cannot be H/V
        # without kinking the join (its direction is owned by the line), so
        # such handles are excluded from the H/V measure. The hand-drawn
        # reference outlines keep these handles on the line, not on an axis.
        if not adjoins_diagonal_line(points, prev_idx):
            total += 1
            if is_hv(prev_on, c1):
                aligned += 1
        if not adjoins_diagonal_line(points, i):
            total += 1
            if is_hv(point, c2):
                aligned += 1
    return aligned, total


def previous_oncurve(points: list[Point], index: int) -> Point | None:
    idx = previous_oncurve_index(points, index)
    return None if idx is None else points[idx]


def previous_oncurve_index(points: list[Point], index: int) -> int | None:
    n = len(points)
    for step in range(1, n + 1):
        j = (index - step) % n
        if points[j].oncurve:
            return j
    return None


def adjoins_diagonal_line(points: list[Point], index: int) -> bool:
    """True if the on-curve point at `index` is an endpoint of a diagonal
    (non-axis-aligned) line segment."""
    n = len(points)
    p = points[index]
    if p.typ == "line":
        j = previous_oncurve_index(points, index)
        if j is not None and not is_hv(points[j], p):
            return True
    for step in range(1, n + 1):
        m = (index + step) % n
        if points[m].oncurve:
            if points[m].typ == "line" and not is_hv(p, points[m]):
                return True
            break
    return False


def previous_offcurves(points: list[Point], index: int) -> list[Point]:
    n = len(points)
    controls = []
    for step in range(1, n + 1):
        p = points[(index - step) % n]
        if p.oncurve:
            break
        controls.append(p)
    return list(reversed(controls))


def is_hv(a: Point, b: Point) -> bool:
    return abs(a.x - b.x) <= 0.5 or abs(a.y - b.y) <= 0.5


def match_oncurves(reference: Glyph, traced: Glyph, tolerance: float) -> dict[str, float]:
    ref_points = all_oncurves(reference)
    traced_points = all_oncurves(traced)
    matched_trace_indices = set()
    matched_ref = 0
    distances = []

    for ref in ref_points:
        best_idx = None
        best_dist = math.inf
        for idx, traced in enumerate(traced_points):
            dist = distance(ref, traced)
            if dist < best_dist:
                best_idx = idx
                best_dist = dist
        distances.append(best_dist if math.isfinite(best_dist) else tolerance * 4)
        if best_idx is not None and best_dist <= tolerance:
            matched_ref += 1
            matched_trace_indices.add(best_idx)

    return {
        "reference_matched": matched_ref,
        "traced_unmatched": max(0, len(traced_points) - len(matched_trace_indices)),
        "mean_reference_distance": sum(distances) / len(distances) if distances else 0.0,
    }


def align_to_reference(traced: Glyph, reference: Glyph) -> Glyph:
    traced_bbox = glyph_bbox(traced)
    ref_bbox = glyph_bbox(reference)
    if traced_bbox is None or ref_bbox is None:
        return traced

    tx0, ty0, tx1, ty1 = traced_bbox
    rx0, ry0, rx1, ry1 = ref_bbox
    traced_diag = math.hypot(tx1 - tx0, ty1 - ty0)
    ref_diag = math.hypot(rx1 - rx0, ry1 - ry0)
    if traced_diag <= 0.0 or ref_diag <= 0.0:
        return traced

    scale = ref_diag / traced_diag
    tcx = tx0 + (tx1 - tx0) / 2.0
    tcy = ty0 + (ty1 - ty0) / 2.0
    rcx = rx0 + (rx1 - rx0) / 2.0
    rcy = ry0 + (ry1 - ry0) / 2.0

    contours = []
    for contour in traced.contours:
        points = []
        for point in contour.points:
            points.append(Point(
                x=(point.x - tcx) * scale + rcx,
                y=(point.y - tcy) * scale + rcy,
                typ=point.typ,
                smooth=point.smooth,
            ))
        contours.append(Contour(points))
    return Glyph(traced.name, contours)


def glyph_bbox(glyph: Glyph) -> tuple[float, float, float, float] | None:
    points = all_points_for_glyph(glyph)
    if not points:
        return None
    return (
        min(p.x for p in points),
        min(p.y for p in points),
        max(p.x for p in points),
        max(p.y for p in points),
    )


def all_oncurves(glyph: Glyph) -> list[Point]:
    return [p for contour in glyph.contours for p in contour.points if p.oncurve]


def distance(a: Point, b: Point) -> float:
    return math.hypot(a.x - b.x, a.y - b.y)


def exact_ratio(value: int, reference: int) -> float:
    if reference == 0:
        return 1.0 if value == 0 else 0.0
    return 1.0 if value == reference else max(0.0, 1.0 - abs(value - reference) / reference)


def ratio_score(value: int, reference: int) -> float:
    if reference == 0:
        return 1.0 if value == 0 else 0.0
    return max(0.0, 1.0 - abs((value / reference) - 1.0))


def fraction_score(value: int, total: int, ref_value: int, ref_total: int) -> float:
    frac = value / total if total else 0.0
    ref_frac = ref_value / ref_total if ref_total else 0.0
    return max(0.0, 1.0 - abs(frac - ref_frac))


def write_tsv(path: Path, rows: list[dict[str, object]]) -> None:
    fields = [
        "glyph",
        "status",
        "score",
        "raster_iou",
        "eval_score",
        "contours",
        "oncurves",
        "segments",
        "line_mix",
        "hv",
        "ref_oncurves_matched",
        "traced_unmatched",
        "mean_ref_distance",
        "missed_extrema_fixed",
        "high_deviation_splits",
        "strong_tangent_overrides",
        "clean_tangent_overrides",
        "visible_tangent_overrides",
        "rejected_tangent_near_misses",
        "oversegmented_splits_removed",
        "final_outline_divergences",
        "final_outline_repairs",
        "note",
    ]
    with open(path, "w", encoding="utf-8") as f:
        f.write("\t".join(fields) + "\n")
        for row in rows:
            f.write("\t".join(format_value(row.get(field, "")) for field in fields) + "\n")


def write_summary(path: Path, rows: list[dict[str, object]]) -> None:
    ok = [row for row in rows if row.get("status") == "ok"]
    mean = sum(float(row["score"]) for row in ok) / len(ok) if ok else 0.0
    with open(path, "w", encoding="utf-8") as f:
        f.write(f"mean_structural_score: {mean:.3f}\n")
        f.write(f"glyphs_ok: {len(ok)}\n")
        f.write(f"glyphs_failed: {len(rows) - len(ok)}\n")
        f.write("\n")
        for row in sorted(ok, key=lambda r: float(r["score"])):
            f.write(
                f"{row['glyph']}: score={float(row['score']):.3f} "
                f"iou={float(row.get('raster_iou', 0.0)):.1f}% "
                f"oncurves={row['oncurves']} segments={row['segments']} "
                f"hv={float(row['hv']):.3f} matched={row['ref_oncurves_matched']} "
                f"diag={row.get('missed_extrema_fixed', 0)}/{row.get('high_deviation_splits', 0)} "
                f"tangent={row.get('strong_tangent_overrides', 0)}/{row.get('clean_tangent_overrides', 0)}/{row.get('visible_tangent_overrides', 0)} "
                f"rejects={row.get('rejected_tangent_near_misses', 0)} "
                f"overseg={row.get('oversegmented_splits_removed', 0)} "
                f"final={row.get('final_outline_divergences', 0)} "
                f"repairs={row.get('final_outline_repairs', 0)}\n"
            )


def print_summary(rows: list[dict[str, object]]) -> None:
    ok = [row for row in rows if row.get("status") == "ok"]
    mean = sum(float(row["score"]) for row in ok) / len(ok) if ok else 0.0
    print(f"mean_structural_score: {mean:.3f}")
    for row in sorted(ok, key=lambda r: float(r["score"])):
        print(
            f"  {row['glyph']:<10} score={float(row['score']):.3f} "
            f"iou={float(row.get('raster_iou', 0.0)):.1f}% "
            f"oncurves={row['oncurves']:<7} segments={row['segments']:<12} "
            f"hv={float(row['hv']):.3f} matched={row['ref_oncurves_matched']} "
            f"diag={row.get('missed_extrema_fixed', 0)}/{row.get('high_deviation_splits', 0)} "
            f"tangent={row.get('strong_tangent_overrides', 0)}/{row.get('clean_tangent_overrides', 0)}/{row.get('visible_tangent_overrides', 0)} "
            f"rejects={row.get('rejected_tangent_near_misses', 0)} "
            f"overseg={row.get('oversegmented_splits_removed', 0)} "
            f"final={row.get('final_outline_divergences', 0)} "
            f"repairs={row.get('final_outline_repairs', 0)}"
        )


def format_value(value: object) -> str:
    if isinstance(value, float):
        return f"{value:.3f}"
    return str(value)


def write_overlay_svg(path: Path, reference: Glyph, traced: Glyph) -> None:
    all_points = [p for glyph in (reference, traced) for p in all_points_for_glyph(glyph)]
    if not all_points:
        return
    min_x = min(p.x for p in all_points) - 40
    max_x = max(p.x for p in all_points) + 40
    min_y = min(p.y for p in all_points) - 40
    max_y = max(p.y for p in all_points) + 40
    width = max_x - min_x
    height = max_y - min_y
    view_box = f"{min_x:.1f} {-max_y:.1f} {width:.1f} {height:.1f}"

    ref_path = svg_path(reference)
    traced_path = svg_path(traced)
    ref_points = svg_points(reference, "#0a8f5a")
    traced_points = svg_points(traced, "#c4311c")

    svg = f'''<svg xmlns="http://www.w3.org/2000/svg" viewBox="{view_box}" width="1200" height="1200">
  <rect x="{min_x:.1f}" y="{-max_y:.1f}" width="{width:.1f}" height="{height:.1f}" fill="#101010"/>
  <g transform="scale(1,-1)">
    <path d="{ref_path}" fill="none" stroke="#66ee88" stroke-width="3" opacity="0.85"/>
    <path d="{traced_path}" fill="none" stroke="#ff684f" stroke-width="2" opacity="0.9"/>
    {ref_points}
    {traced_points}
  </g>
</svg>
'''
    path.write_text(svg, encoding="utf-8")


def svg_path(glyph: Glyph) -> str:
    return " ".join(contour_to_path(contour.points) for contour in glyph.contours)


def contour_to_path(points: list[Point]) -> str:
    first_idx = next((idx for idx, point in enumerate(points) if point.oncurve), None)
    if first_idx is None:
        return ""
    first = points[first_idx]
    commands = [f"M {first.x:.1f} {first.y:.1f}"]
    offcurves = []
    n = len(points)
    for step in range(1, n + 1):
        point = points[(first_idx + step) % n]
        if not point.oncurve:
            offcurves.append(point)
            continue
        if len(offcurves) >= 2:
            c1, c2 = offcurves[-2], offcurves[-1]
            commands.append(f"C {c1.x:.1f} {c1.y:.1f} {c2.x:.1f} {c2.y:.1f} {point.x:.1f} {point.y:.1f}")
        elif len(offcurves) == 1:
            c = offcurves[0]
            commands.append(f"Q {c.x:.1f} {c.y:.1f} {point.x:.1f} {point.y:.1f}")
        else:
            commands.append(f"L {point.x:.1f} {point.y:.1f}")
        offcurves = []
    commands.append("Z")
    return " ".join(commands)


def svg_points(glyph: Glyph, color: str) -> str:
    parts = []
    for point in all_oncurves(glyph):
        parts.append(f'<circle cx="{point.x:.1f}" cy="{point.y:.1f}" r="5" fill="{color}"/>')
    return "\n    ".join(parts)


def all_points_for_glyph(glyph: Glyph) -> list[Point]:
    return [p for contour in glyph.contours for p in contour.points]


def safe_name(name: str) -> str:
    return "ampersand" if name == "ampersand" else name.replace("/", "_")


if __name__ == "__main__":
    raise SystemExit(main())
