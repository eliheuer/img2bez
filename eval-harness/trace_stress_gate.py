#!/usr/bin/env python3
"""Trace composite source images and fail obvious point-count errors.

The focused structural gate compares one rendered glyph at a time against the
reference UFO. That misses app-style debugging images where several glyphs are
traced into one outline. This stress gate creates those composite sources from
the same reference UFO and checks whether the trace is far denser or far sparser
than the source glyph structure implies.
"""

from __future__ import annotations

import argparse
import copy
import plistlib
import re
import shutil
import os
import subprocess
import sys
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path

from PIL import Image, ImageFilter

SCRIPT_DIR = Path(__file__).resolve().parent
REPO_DIR = SCRIPT_DIR.parent
DEFAULT_REFERENCE_UFO = SCRIPT_DIR / "reference.ufo"
DEFAULT_WORK_DIR = SCRIPT_DIR / "stress-work"
DIAGNOSTICS_RE = re.compile(
    r"Validate\s+.*?final_divergences=(?P<final>\d+)"
    r"(?:\s+final_repairs=(?P<repair>\d+))?",
    re.S,
)
RASTER_IOU_RE = re.compile(r"Raster IoU\s+(?P<iou>\d+(?:\.\d+)?)%")
OVERALL_RE = re.compile(r"Overall\s+(?P<score>\d+(?:\.\d+)?)")

sys.path.insert(0, str(SCRIPT_DIR))
from render_glyph import render_glyph  # noqa: E402


@dataclass
class ContourStats:
    oncurves: int
    offcurves: int
    lines: int
    curves: int
    bbox: tuple[float, float, float, float]


@dataclass
class GlyphStats:
    contours: list[ContourStats]

    @property
    def oncurves(self) -> int:
        return sum(contour.oncurves for contour in self.contours)

    @property
    def max_contour_oncurves(self) -> int:
        return max((contour.oncurves for contour in self.contours), default=0)

    @property
    def lines(self) -> int:
        return sum(contour.lines for contour in self.contours)

    @property
    def curves(self) -> int:
        return sum(contour.curves for contour in self.contours)


def main() -> int:
    args = parse_args()
    work_dir = Path(args.work_dir)
    if work_dir.exists():
        shutil.rmtree(work_dir)
    work_dir.mkdir(parents=True)

    reference_ufo = Path(args.reference_ufo)
    text = args.text
    print(f"stress_text: {text}")
    print(f"reference_ufo: {reference_ufo}")
    input_glyphs = resolve_input_glyph_names(reference_ufo, text)

    rendered = render_source_images(reference_ufo, input_glyphs, work_dir, args.render_height)
    if args.source_image:
        source_png = Path(args.source_image)
        if not source_png.is_file():
            print(f"FAIL: source image not found: {source_png}")
            return 1
    else:
        source_png = compose_source_image(
            rendered,
            work_dir / f"{text_file_key(text)}-{args.profile}.png",
            args.padding,
            args.profile,
        )
    output_ufo = prepare_output_ufo(reference_ufo, work_dir / "output.ufo")
    glyph_name = stress_glyph_name(text)
    reference_glif = write_composite_reference_glif(
        reference_ufo,
        rendered,
        work_dir / f"{glyph_name}_reference.glif",
    )
    reference_composite_ufo = prepare_composite_reference_ufo(
        reference_ufo,
        reference_glif,
        glyph_name,
        work_dir / "reference-composite.ufo",
    )

    # IMG2BEZ_BIN env overrides (matches run_experiment.sh): test a
    # scratch build while another process owns target/release.
    img2bez_bin = Path(os.environ.get("IMG2BEZ_BIN", args.img2bez_bin))
    if not img2bez_bin.is_absolute():
        img2bez_bin = REPO_DIR / img2bez_bin

    if args.build:
        subprocess.run(["cargo", "build", "--release"], cwd=img2bez_bin.parent.parent.parent, check=True)

    cmd = [
        str(img2bez_bin),
        "--input",
        str(source_png),
        "--output",
        str(output_ufo),
        "--name",
        glyph_name,
        "--width",
        str(sum(item["advance_width"] for item in rendered)),
        "--target-height",
        str(rendered[0]["target_height"]),
        "--y-offset",
        str(rendered[0]["y_offset"]),
        "--reference",
        str(reference_glif),
        *args.trace_args,
    ]
    trace_log = work_dir / f"{glyph_name}.log"
    with trace_log.open("w", encoding="utf-8") as log:
        result = subprocess.run(cmd, cwd=REPO_DIR, text=True, stdout=log, stderr=log)

    if result.returncode != 0:
        print(f"FAIL: trace command exited {result.returncode}; see {trace_log}")
        return 1

    traced = load_glif_stats(resolve_glif(output_ufo, glyph_name))
    reference = combine_reference_stats(reference_ufo, input_glyphs)
    diagnostics = read_trace_diagnostics(trace_log)
    reference_metrics = read_reference_metrics(trace_log)
    total_ratio = safe_ratio(traced.oncurves, reference.oncurves)
    contour_ratio = safe_ratio(traced.max_contour_oncurves, reference.max_contour_oncurves)
    glyph_slices = slice_stats_by_glyph(rendered, reference_ufo, traced)
    structural_artifacts = write_composite_structure_report(
        reference_composite_ufo,
        output_ufo,
        glyph_name,
        work_dir,
    )
    specimen_image = write_specimen_image(
        reference_composite_ufo,
        output_ufo,
        glyph_name,
        source_png,
        structural_artifacts["report"] if structural_artifacts else None,
        work_dir,
        text,
    )

    print(f"source_png: {source_png}")
    print(f"trace_log: {trace_log}")
    print(f"reference_glif: {reference_glif}")
    if structural_artifacts:
        print(f"structure_overlay: {structural_artifacts['overlay']}")
        print(f"structure_report: {structural_artifacts['report']}")
        print(f"structure_summary: {structural_artifacts['summary']}")
    if specimen_image:
        print(f"specimen_image: {specimen_image}")
    print(f"reference_oncurves: {reference.oncurves}")
    print(f"traced_oncurves: {traced.oncurves}")
    print(f"oncurve_ratio: {total_ratio:.3f}")
    print(f"reference_max_contour_oncurves: {reference.max_contour_oncurves}")
    print(f"traced_max_contour_oncurves: {traced.max_contour_oncurves}")
    print(f"max_contour_ratio: {contour_ratio:.3f}")
    if reference_metrics["raster_iou"] is not None:
        print(f"raster_iou: {reference_metrics['raster_iou']:.1f}%")
    if reference_metrics["eval_score"] is not None:
        print(f"eval_score: {reference_metrics['eval_score']:.3f}")
    print(f"final_outline_divergences: {diagnostics['final_outline_divergences']}")
    print(f"final_outline_repairs: {diagnostics['final_outline_repairs']}")
    print("densest_traced_contours:")
    for index, contour in densest_contours(traced, limit=5):
        min_x, min_y, max_x, max_y = contour.bbox
        print(
            f"  contour={index} oncurves={contour.oncurves} "
            f"offcurves={contour.offcurves} "
            f"bbox=({min_x:.1f},{min_y:.1f},{max_x:.1f},{max_y:.1f})"
        )
    print("per_glyph_slices:")
    for row in glyph_slices:
        min_x, max_x = row["x_range"]
        print(
            f"  glyph={row['glyph']} x=({min_x:.1f},{max_x:.1f}) "
            f"ref_contours={row['reference_contours']} "
            f"traced_contours={row['traced_contours']} "
            f"ref_oncurves={row['reference_oncurves']} "
            f"traced_oncurves={row['traced_oncurves']} "
            f"ratio={row['oncurve_ratio']:.3f} "
            f"ref_lines={row['reference_lines']} "
            f"traced_lines={row['traced_lines']} "
            f"ref_curves={row['reference_curves']} "
            f"traced_curves={row['traced_curves']} "
            f"offcurves={row['traced_offcurves']}"
        )

    failures = []
    if total_ratio < args.min_total_ratio:
        failures.append(
            f"total oncurve ratio {total_ratio:.3f} < {args.min_total_ratio:.3f}"
        )
    if total_ratio > args.max_total_ratio:
        failures.append(
            f"total oncurve ratio {total_ratio:.3f} > {args.max_total_ratio:.3f}"
        )
    if contour_ratio < args.min_contour_ratio:
        failures.append(
            f"max contour ratio {contour_ratio:.3f} < {args.min_contour_ratio:.3f}"
        )
    if contour_ratio > args.max_contour_ratio:
        failures.append(
            f"max contour ratio {contour_ratio:.3f} > {args.max_contour_ratio:.3f}"
        )
    if diagnostics["final_outline_repairs"] < args.min_final_repairs:
        failures.append(
            "final outline repairs "
            f"{diagnostics['final_outline_repairs']} < {args.min_final_repairs}"
        )
    if (
        args.min_raster_iou is not None
        and reference_metrics["raster_iou"] is not None
        and reference_metrics["raster_iou"] < args.min_raster_iou
    ):
        failures.append(
            f"raster IoU {reference_metrics['raster_iou']:.1f}% < {args.min_raster_iou:.1f}%"
        )
    if (
        args.min_eval_score is not None
        and reference_metrics["eval_score"] is not None
        and reference_metrics["eval_score"] < args.min_eval_score
    ):
        failures.append(
            "eval score "
            f"{reference_metrics['eval_score']:.3f} < {args.min_eval_score:.3f}"
        )
    if (
        args.max_final_divergences is not None
        and diagnostics["final_outline_divergences"] > args.max_final_divergences
    ):
        failures.append(
            "final outline divergences "
            f"{diagnostics['final_outline_divergences']} > {args.max_final_divergences}"
        )
    glyph_ratio_requirements = parse_glyph_ratio_requirements(args.glyph_oncurve_ratio)
    glyph_delta_requirements = parse_glyph_delta_requirements(args.glyph_segment_delta)
    glyph_slices_by_name = {row["glyph"]: row for row in glyph_slices}
    for glyph, min_ratio, max_ratio in glyph_ratio_requirements:
        row = glyph_slices_by_name.get(glyph)
        if row is None:
            failures.append(f"glyph {glyph!r} was not present in traced text {text!r}")
            continue
        ratio = float(row["oncurve_ratio"])
        if ratio < min_ratio:
            failures.append(
                f"{glyph} oncurve ratio {ratio:.3f} < {min_ratio:.3f}"
            )
        if ratio > max_ratio:
            failures.append(
                f"{glyph} oncurve ratio {ratio:.3f} > {max_ratio:.3f}"
            )
    for glyph, segment_kind, max_delta in glyph_delta_requirements:
        row = glyph_slices_by_name.get(glyph)
        if row is None:
            failures.append(f"glyph {glyph!r} was not present in traced text {text!r}")
            continue
        if segment_kind == "lines":
            reference_key = "reference_lines"
            traced_key = "traced_lines"
        elif segment_kind == "curves":
            reference_key = "reference_curves"
            traced_key = "traced_curves"
        else:
            reference_key = "reference_oncurves"
            traced_key = "traced_oncurves"
        delta = abs(int(row[traced_key]) - int(row[reference_key]))
        if delta > max_delta:
            failures.append(
                f"{glyph} {segment_kind} delta {delta} > allowed {max_delta} "
                f"({row[traced_key]}/{row[reference_key]})"
            )

    if failures:
        print("FAIL: trace stress gate")
        for failure in failures:
            print("fail: " + failure)
        return 1

    print("PASS: trace stress gate")
    return 0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--reference-ufo", default=str(DEFAULT_REFERENCE_UFO))
    parser.add_argument("--work-dir", default=str(DEFAULT_WORK_DIR))
    parser.add_argument("--text", default="aen")
    parser.add_argument(
        "--source-image",
        help="Optional existing image to trace instead of rendering --text.",
    )
    parser.add_argument("--render-height", type=int, default=700)
    parser.add_argument("--padding", type=int, default=70)
    parser.add_argument(
        "--profile",
        choices=["clean", "screen-gray", "lowres2x", "lowres4x", "soft-gray"],
        default="clean",
        help="Generated source-image stress profile.",
    )
    parser.add_argument("--min-total-ratio", type=float, default=0.75)
    parser.add_argument("--max-total-ratio", type=float, default=1.20)
    parser.add_argument("--min-contour-ratio", type=float, default=0.75)
    parser.add_argument("--max-contour-ratio", type=float, default=1.65)
    parser.add_argument(
        "--min-final-repairs",
        type=int,
        default=0,
        help="Fail unless the trace reports at least this many final outline repairs.",
    )
    parser.add_argument(
        "--max-final-divergences",
        type=int,
        help="Fail if the trace reports more than this many final outline divergences.",
    )
    parser.add_argument(
        "--min-raster-iou",
        type=float,
        help="Fail if the composite trace/reference raster IoU is below this percent.",
    )
    parser.add_argument(
        "--min-eval-score",
        type=float,
        help="Fail if the composite vector evaluation score is below this value.",
    )
    parser.add_argument(
        "--glyph-oncurve-ratio",
        action="append",
        default=[],
        metavar="GLYPH:MIN:MAX",
        help=(
            "Optional per-glyph on-curve ratio guard for composite traces, "
            "for example R:0.85:1.15. May be passed more than once."
        ),
    )
    parser.add_argument(
        "--glyph-segment-delta",
        action="append",
        default=[],
        metavar="GLYPH:KIND:MAX_DELTA",
        help=(
            "Optional per-glyph segment-count guard for type-design structure. "
            "KIND is lines, curves, or oncurves. Example: R:lines:2."
        ),
    )
    parser.add_argument(
        "--img2bez-bin",
        default=str(REPO_DIR / "target" / "release" / "img2bez"),
        help="Path to the img2bez binary to test. Useful for historical worktrees.",
    )
    parser.add_argument("--build", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("trace_args", nargs=argparse.REMAINDER)
    args = parser.parse_args()
    if args.trace_args and args.trace_args[0] == "--":
        args.trace_args = args.trace_args[1:]
    return args


def parse_glyph_ratio_requirements(values: list[str]) -> list[tuple[str, float, float]]:
    requirements = []
    for value in values:
        parts = value.split(":")
        if len(parts) != 3:
            raise ValueError(
                f"--glyph-oncurve-ratio expects GLYPH:MIN:MAX, got {value!r}"
            )
        glyph, min_ratio, max_ratio = parts
        requirements.append((glyph, float(min_ratio), float(max_ratio)))
    return requirements


def parse_glyph_delta_requirements(values: list[str]) -> list[tuple[str, str, int]]:
    requirements = []
    for value in values:
        parts = value.split(":")
        if len(parts) != 3:
            raise ValueError(
                f"--glyph-segment-delta expects GLYPH:KIND:MAX_DELTA, got {value!r}"
            )
        glyph, kind, max_delta = parts
        if kind not in {"lines", "curves", "oncurves"}:
            raise ValueError(
                f"--glyph-segment-delta KIND must be lines, curves, or oncurves; got {kind!r}"
            )
        requirements.append((glyph, kind, int(max_delta)))
    return requirements


def read_trace_diagnostics(trace_log: Path) -> dict[str, int]:
    diagnostics = {
        "final_outline_divergences": 0,
        "final_outline_repairs": 0,
    }
    text = trace_log.read_text(encoding="utf-8", errors="replace")
    match = DIAGNOSTICS_RE.search(text)
    if not match:
        return diagnostics
    diagnostics["final_outline_divergences"] = int(match.group("final"))
    diagnostics["final_outline_repairs"] = int(match.group("repair") or 0)
    return diagnostics


def read_reference_metrics(trace_log: Path) -> dict[str, float | None]:
    text = trace_log.read_text(encoding="utf-8", errors="replace")
    raster_iou = None
    eval_score = None
    if match := RASTER_IOU_RE.search(text):
        raster_iou = float(match.group("iou"))
    if match := OVERALL_RE.search(text):
        eval_score = float(match.group("score"))
    return {"raster_iou": raster_iou, "eval_score": eval_score}


def resolve_input_glyph_names(reference_ufo: Path, text: str) -> list[dict[str, str]]:
    if not text:
        raise ValueError("--text must not be empty")
    available = available_glyph_names(reference_ufo)
    unicode_map = glyph_names_by_unicode(reference_ufo)
    resolved = []
    for char in text:
        glyph_name = resolve_character_glyph_name(char, available, unicode_map)
        resolved.append({"char": char, "glyph_name": glyph_name})
    return resolved


def resolve_character_glyph_name(
    char: str,
    available: set[str],
    unicode_map: dict[int, str],
) -> str:
    codepoint = ord(char)
    candidates = [
        char,
        f"uni{codepoint:04X}" if codepoint <= 0xFFFF else f"u{codepoint:05X}",
        f"uni{codepoint:04x}" if codepoint <= 0xFFFF else f"u{codepoint:05x}",
    ]
    for candidate in candidates:
        if candidate in available:
            return candidate
    if codepoint in unicode_map:
        return unicode_map[codepoint]
    raise FileNotFoundError(
        f"no glyph found for character {char!r} (U+{codepoint:04X}); "
        f"tried {', '.join(candidates)} and UFO unicode mappings"
    )


def available_glyph_names(reference_ufo: Path) -> set[str]:
    names = set()
    for glyphs_dir_name in ("glyphs", "glyphs.public.default"):
        contents_path = reference_ufo / glyphs_dir_name / "contents.plist"
        if not contents_path.exists():
            continue
        with contents_path.open("rb") as f:
            names.update(plistlib.load(f).keys())
    return names


def glyph_names_by_unicode(reference_ufo: Path) -> dict[int, str]:
    by_unicode = {}
    for glyphs_dir_name in ("glyphs", "glyphs.public.default"):
        glyphs_dir = reference_ufo / glyphs_dir_name
        contents_path = glyphs_dir / "contents.plist"
        if not contents_path.exists():
            continue
        with contents_path.open("rb") as f:
            contents = plistlib.load(f)
        for glyph_name, filename in contents.items():
            glif_path = glyphs_dir / filename
            if not glif_path.exists():
                continue
            try:
                root = ET.parse(glif_path).getroot()
            except ET.ParseError:
                continue
            for unicode_el in root.findall("unicode"):
                hex_value = unicode_el.attrib.get("hex")
                if not hex_value:
                    continue
                try:
                    by_unicode.setdefault(int(hex_value, 16), glyph_name)
                except ValueError:
                    continue
    return by_unicode


def render_source_images(
    reference_ufo: Path,
    input_glyphs: list[dict[str, str]],
    work_dir: Path,
    render_height: int,
) -> list[dict[str, object]]:
    rendered = []
    for index, item in enumerate(input_glyphs):
        char = item["char"]
        glyph_name = item["glyph_name"]
        glyph_png = work_dir / f"{index}_{glyph_file_key(glyph_name)}.png"
        metrics = render_glyph(str(reference_ufo), glyph_name, str(glyph_png), render_height)
        metrics["path"] = glyph_png
        metrics["char"] = char
        metrics["glyph_name"] = glyph_name
        rendered.append(metrics)
    return rendered


def write_composite_reference_glif(
    reference_ufo: Path,
    rendered: list[dict[str, object]],
    output_glif: Path,
) -> Path:
    glyph_el = ET.Element(
        "glyph",
        {"name": output_glif.stem.removesuffix("_reference"), "format": "2"},
    )
    ET.SubElement(
        glyph_el,
        "advance",
        {"width": str(int(sum(item["advance_width"] for item in rendered)))},
    )
    outline_el = ET.SubElement(glyph_el, "outline")

    x_offset = 0.0
    for metrics in rendered:
        source_root = ET.parse(resolve_glif(reference_ufo, str(metrics["glyph_name"]))).getroot()
        source_outline = source_root.find("outline")
        if source_outline is not None:
            for contour_el in source_outline.findall("contour"):
                shifted = copy.deepcopy(contour_el)
                for point in shifted.findall("point"):
                    point.set("x", format_float(float(point.attrib["x"]) + x_offset))
                outline_el.append(shifted)
        x_offset += float(metrics["advance_width"])

    ET.ElementTree(glyph_el).write(output_glif, encoding="utf-8", xml_declaration=True)
    return output_glif


def format_float(value: float) -> str:
    if abs(value - round(value)) < 1e-6:
        return str(int(round(value)))
    return f"{value:.3f}".rstrip("0").rstrip(".")


def compose_source_image(
    rendered: list[dict[str, object]],
    output_png: Path,
    padding: int,
    profile: str,
) -> Path:
    images = [Image.open(item["path"]).convert("RGB") for item in rendered]
    width = sum(image.width for image in images) + padding * 2
    height = max(image.height for image in images) + padding * 2
    background = "white" if profile == "clean" else (132, 132, 132)
    composite = Image.new("RGB", (width, height), background)
    x = padding
    for image in images:
        if profile in {"screen-gray", "lowres2x", "lowres4x", "soft-gray"}:
            image = image_on_background(image, background)
        composite.paste(image, (x, padding))
        x += image.width
    if profile in {"lowres2x", "lowres4x"}:
        factor = 2 if profile == "lowres2x" else 4
        low = composite.resize(
            (max(1, width // factor), max(1, height // factor)),
            Image.Resampling.BILINEAR,
        )
        composite = low.resize((width, height), Image.Resampling.NEAREST)
    elif profile == "soft-gray":
        composite = composite.filter(ImageFilter.GaussianBlur(radius=0.65))
    composite.save(output_png)
    return output_png


def image_on_background(image: Image.Image, background: str | tuple[int, int, int]) -> Image.Image:
    """Replace white glyph-render background with the profile background."""
    bg = Image.new("RGB", image.size, background)
    # Rendered glyph images are black-on-white. Use inverse luminance as alpha.
    mask = Image.eval(image.convert("L"), lambda value: 255 - value)
    black = Image.new("RGB", image.size, "black")
    bg.paste(black, mask=mask)
    return bg


def prepare_output_ufo(reference_ufo: Path, output_ufo: Path) -> Path:
    if output_ufo.is_symlink():
        output_ufo.unlink()
    elif output_ufo.exists():
        shutil.rmtree(output_ufo)
    shutil.copytree(reference_ufo, output_ufo)
    glyphs_dir = output_ufo / "glyphs"
    for glif_path in glyphs_dir.glob("*.glif"):
        glif_path.unlink()
    with (glyphs_dir / "contents.plist").open("wb") as f:
        plistlib.dump({}, f)
    return output_ufo


def prepare_composite_reference_ufo(
    reference_ufo: Path,
    reference_glif: Path,
    glyph_name: str,
    output_ufo: Path,
) -> Path:
    if output_ufo.is_symlink():
        output_ufo.unlink()
    elif output_ufo.exists():
        shutil.rmtree(output_ufo)
    shutil.copytree(reference_ufo, output_ufo)
    glyphs_dir = output_ufo / "glyphs"
    for glif_path in glyphs_dir.glob("*.glif"):
        glif_path.unlink()
    filename = f"{glyph_file_key(glyph_name)}.glif"
    shutil.copyfile(reference_glif, glyphs_dir / filename)
    with (glyphs_dir / "contents.plist").open("wb") as f:
        plistlib.dump({glyph_name: filename}, f)
    return output_ufo


def write_composite_structure_report(
    reference_ufo: Path,
    traced_ufo: Path,
    glyph_name: str,
    work_dir: Path,
) -> dict[str, Path] | None:
    glyph_file = work_dir / f"{glyph_name}_glyphs.txt"
    glyph_file.write_text(f"{glyph_name}\n", encoding="utf-8")
    result = subprocess.run(
        [
            sys.executable,
            str(SCRIPT_DIR / "structural_report.py"),
            "--reference-ufo",
            str(reference_ufo),
            "--traced-ufo",
            str(traced_ufo),
            "--glyph-file",
            str(glyph_file),
            "--output-dir",
            str(work_dir),
        ],
        cwd=REPO_DIR,
        text=True,
        capture_output=True,
    )
    log_path = work_dir / f"{glyph_name}_structure_report.log"
    log_path.write_text(result.stdout + result.stderr, encoding="utf-8")
    if result.returncode != 0:
        print(f"WARN: structural report failed; see {log_path}", file=sys.stderr)
        return None
    return {
        "overlay": work_dir / f"structure_{glyph_file_key(glyph_name)}.svg",
        "report": work_dir / "structural_report.tsv",
        "summary": work_dir / "structural_summary.txt",
    }


def write_specimen_image(
    reference_ufo: Path,
    traced_ufo: Path,
    glyph_name: str,
    source_png: Path,
    metrics_path: Path | None,
    work_dir: Path,
    text: str,
) -> Path | None:
    output = work_dir / f"specimen_{glyph_file_key(glyph_name)}.png"
    cmd = [
        sys.executable,
        str(SCRIPT_DIR / "render_specimen.py"),
        "--reference-ufo",
        str(reference_ufo),
        "--traced-ufo",
        str(traced_ufo),
        "--glyph",
        glyph_name,
        "--source-image",
        str(source_png),
        "--output",
        str(output),
        "--input-text",
        text,
    ]
    if metrics_path is not None:
        cmd.extend(["--metrics", str(metrics_path)])
    result = subprocess.run(
        cmd,
        cwd=REPO_DIR,
        text=True,
        capture_output=True,
    )
    log_path = work_dir / f"{glyph_name}_specimen.log"
    log_path.write_text(result.stdout + result.stderr, encoding="utf-8")
    if result.returncode != 0:
        print(f"WARN: specimen render failed; see {log_path}", file=sys.stderr)
        return None
    return output


def combine_reference_stats(reference_ufo: Path, input_glyphs: list[dict[str, str]]) -> GlyphStats:
    contours = []
    for item in input_glyphs:
        contours.extend(load_glif_stats(resolve_glif(reference_ufo, item["glyph_name"])).contours)
    return GlyphStats(contours)


def slice_stats_by_glyph(
    rendered: list[dict[str, object]],
    reference_ufo: Path,
    traced: GlyphStats,
) -> list[dict[str, object]]:
    rows = []
    x_offset = 0.0
    for metrics in rendered:
        char = str(metrics["char"])
        glyph_name = str(metrics["glyph_name"])
        advance = float(metrics["advance_width"])
        x_min = x_offset
        x_max = x_offset + advance
        reference = load_glif_stats(resolve_glif(reference_ufo, glyph_name))
        assigned = [
            contour
            for contour in traced.contours
            if x_min <= contour_center_x(contour) < x_max
        ]
        traced_stats = GlyphStats(assigned)
        rows.append(
            {
                "glyph": char,
                "x_range": (x_min, x_max),
                "reference_contours": len(reference.contours),
                "traced_contours": len(traced_stats.contours),
                "reference_oncurves": reference.oncurves,
                "traced_oncurves": traced_stats.oncurves,
                "traced_offcurves": sum(contour.offcurves for contour in assigned),
                "reference_lines": reference.lines,
                "traced_lines": traced_stats.lines,
                "reference_curves": reference.curves,
                "traced_curves": traced_stats.curves,
                "oncurve_ratio": safe_ratio(traced_stats.oncurves, reference.oncurves),
            }
        )
        x_offset = x_max
    return rows


def contour_center_x(contour: ContourStats) -> float:
    min_x, _, max_x, _ = contour.bbox
    return (min_x + max_x) / 2.0


def resolve_glif(ufo: Path, glyph_name: str) -> Path:
    for glyphs_dir_name in ("glyphs", "glyphs.public.default"):
        contents_path = ufo / glyphs_dir_name / "contents.plist"
        if not contents_path.exists():
            continue
        with contents_path.open("rb") as f:
            contents = plistlib.load(f)
        filename = contents.get(glyph_name)
        if filename:
            return ufo / glyphs_dir_name / filename
    raise FileNotFoundError(f"glyph {glyph_name!r} not found in {ufo}")


def load_glif_stats(glif_path: Path) -> GlyphStats:
    root = ET.parse(glif_path).getroot()
    outline = root.find("outline")
    if outline is None:
        return GlyphStats([])

    contours = []
    for contour_el in outline.findall("contour"):
        points = contour_el.findall("point")
        coords = [
            (float(point.attrib["x"]), float(point.attrib["y"]))
            for point in points
        ]
        if coords:
            xs = [point[0] for point in coords]
            ys = [point[1] for point in coords]
            bbox = (min(xs), min(ys), max(xs), max(ys))
        else:
            bbox = (0.0, 0.0, 0.0, 0.0)
        oncurves = sum(1 for point in points if point.attrib.get("type"))
        lines = sum(1 for point in points if point.attrib.get("type") == "line")
        curves = sum(1 for point in points if point.attrib.get("type") == "curve")
        contours.append(
            ContourStats(
                oncurves=oncurves,
                offcurves=len(points) - oncurves,
                lines=lines,
                curves=curves,
                bbox=bbox,
            )
        )
    return GlyphStats(contours)


def safe_ratio(actual: int, expected: int) -> float:
    if expected <= 0:
        return 1.0 if actual == 0 else float("inf")
    return actual / expected


def densest_contours(glyph: GlyphStats, limit: int) -> list[tuple[int, ContourStats]]:
    return sorted(
        enumerate(glyph.contours),
        key=lambda item: item[1].oncurves,
        reverse=True,
    )[:limit]


def glyph_file_key(glyph_name: str) -> str:
    if len(glyph_name) == 1:
        return f"uni{ord(glyph_name):04X}"
    return glyph_name


def stress_glyph_name(text: str) -> str:
    key = text_file_key(text)
    return f"stress_{key}"


def text_file_key(text: str) -> str:
    if re.fullmatch(r"[A-Za-z0-9_.-]+", text):
        return text
    return "_".join(f"u{ord(char):04X}" for char in text)


if __name__ == "__main__":
    raise SystemExit(main())
