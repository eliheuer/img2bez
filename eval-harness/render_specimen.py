#!/usr/bin/env python3
"""Render a 2048x2048 img2bez eval-harness specimen with drawbot-skia.

The specimen is designed as a social/shareable debugging sheet:

    input/reference row:
      structure view, filled preview, dark overlay

    output/traced row:
      structure view, filled preview, dark overlay

The renderer intentionally uses ``drawbot_skia.drawbot`` for all drawing so it
matches the Runebender DrawBot stack. Pillow is only used to read source image
dimensions and to build the red raster-difference overlay used for visual QA.
"""

from __future__ import annotations

import argparse
import csv
import math
import plistlib
import subprocess
import tempfile
import textwrap
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Iterable

PAGE = 2048
MARGIN = 64
GAP = 24
HEADER_H = 172
FOOTER_H = 288
LABEL_H = 58
HEADER_INSET = 36
HEADER_TEXT_SIZE = 42
HEADER_LINE_GAP = 62
TECH_SIZE = 30
FOOTER_SIZE = 30
FOOTER_LINE_GAP = 36
SHOW_METRIC_LINES = False
GLYPH_PADDING = 18
STRUCTURE_OUTLINE_WIDTH = 2.2
HANDLE_LINE_WIDTH = STRUCTURE_OUTLINE_WIDTH

BG = "#0c0c0c"
PANEL = "#161616"
PANEL_DARK = "#161616"
OVERLAY_BG = "#161616"
RASTER_GRAY = "#b8b8b8"
GRID_MINOR = "#262626"
GRID_MAJOR = "#3b3b3b"
CELL_GRID_MINOR = 24
CELL_GRID_MAJOR = 96
CELL_GRID_ALPHA = 0.48
GRID_GREEN = "#66ee88"
STROKE = "#4a4a4a"
TEXT = "#efefef"
HANDLE_LINE = "#909090"
GREEN = "#66ee88"
GREEN_DARK = "#18bf73"
RED = "#ff4a34"
ORANGE = "#ff9d18"
PURPLE = "#8b6cff"
YELLOW = "#ffdd3b"
WHITE_LINE = "#c9c9c9"
BLACK = "#030303"

REPO_URL = "github.com/eliheuer/img2bez"
REPO_DIR = Path(__file__).resolve().parent.parent
VIRTUA_GROTESK_FONT = REPO_DIR / "eval-harness" / "assets" / "fonts" / "VirtuaGrotesk-wght.ttf"
VIRTUA_GROTESK = str(VIRTUA_GROTESK_FONT)

HEADLINE_FONT = [VIRTUA_GROTESK, "Virtua Grotesk", "Helvetica", "Arial"]
HEADLINE_BOLD_FONT = [VIRTUA_GROTESK, "Virtua Grotesk", "Helvetica-Bold", "Helvetica Bold", "Arial-BoldMT", "Arial Bold"]
SPECIMEN_TEXT_FONT = [
    VIRTUA_GROTESK,
    "Virtua Grotesk",
    "SFMono-Semibold",
    "SFMono-Regular",
    "Menlo-Bold",
    "Menlo-Regular",
    "Menlo",
    "Monaco",
    "Courier",
]
MONO_FONT = SPECIMEN_TEXT_FONT


@dataclass
class Point:
    x: float
    y: float
    typ: str | None
    smooth: bool

    @property
    def oncurve(self) -> bool:
        return self.typ is not None and self.typ != "offcurve"


@dataclass
class Contour:
    points: list[Point]


@dataclass
class Glyph:
    name: str
    contours: list[Contour]
    width: float


@dataclass(frozen=True)
class FontMetrics:
    units_per_em: float
    ascender: float
    descender: float
    cap_height: float
    x_height: float


@dataclass(frozen=True)
class Box:
    x: float
    y: float
    w: float
    h: float

    @property
    def x1(self) -> float:
        return self.x + self.w

    @property
    def y1(self) -> float:
        return self.y + self.h

    def inset(self, amount: float) -> "Box":
        return Box(self.x + amount, self.y + amount, self.w - amount * 2, self.h - amount * 2)

    def label_area(self) -> "Box":
        return Box(self.x, self.y + self.h - LABEL_H, self.w, LABEL_H)

    def content_area(self) -> "Box":
        return Box(self.x, self.y, self.w, self.h - LABEL_H)


Transform = Callable[[Point], tuple[float, float]]


def main() -> int:
    args = parse_args()
    try:
        import drawbot_skia.drawbot as db
    except ImportError as error:
        raise SystemExit(
            "Error: drawbot-skia is not installed. Run ./eval-harness/setup.sh "
            "to install the eliheuer/drawbot-skia fork into ./.venv-eval-harness."
        ) from error

    reference_ufo = Path(args.reference_ufo)
    traced_ufo = Path(args.traced_ufo)
    reference = load_glyph(reference_ufo, args.glyph)
    traced = load_glyph(traced_ufo, args.glyph)
    metrics = read_metrics(Path(args.metrics)) if args.metrics else {}
    font_metrics = load_font_metrics(reference_ufo)
    source_image = Path(args.source_image) if args.source_image else None

    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    render_specimen(
        db=db,
        output=output,
        reference=reference,
        traced=traced,
        font_metrics=font_metrics,
        metrics=metrics,
        source_image=source_image,
        reference_ufo=reference_ufo,
        title=args.title,
        input_text=args.input_text if args.input_text is not None else display_input_text(args.glyph),
    )
    print(output)
    return 0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--reference-ufo", required=True)
    parser.add_argument("--traced-ufo", required=True)
    parser.add_argument("--glyph", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--source-image")
    parser.add_argument("--metrics")
    parser.add_argument("--title", default="img2bez Eval Harness Specimen")
    parser.add_argument("--input-text")
    return parser.parse_args()


def render_specimen(
    db,
    output: Path,
    reference: Glyph,
    traced: Glyph,
    font_metrics: FontMetrics,
    metrics: dict[str, str],
    source_image: Path | None,
    reference_ufo: Path,
    title: str,
    input_text: str,
) -> None:
    db.newDrawing()
    db.newPage(PAGE, PAGE)

    fill(db, BG)
    db.rect(0, 0, PAGE, PAGE)

    header, footer, row_boxes = layout()
    draw_header(db, header, title, input_text)
    viewport = glyph_bounds([reference, traced], font_metrics)

    draw_row(
        db,
        row_boxes[0],
        row_label="Input / Reference",
        glyph=reference,
        peer=traced,
        overlay_underlay=reference,
        viewport=viewport,
        font_metrics=font_metrics,
        source_image=source_image,
        overlay_mode="source-or-fill",
    )
    draw_row(
        db,
        row_boxes[1],
        row_label="Output / Trace",
        glyph=traced,
        peer=reference,
        overlay_underlay=reference,
        viewport=viewport,
        font_metrics=font_metrics,
        source_image=source_image,
        overlay_mode="fill",
    )

    draw_footer(db, footer, reference, traced, metrics, source_image, reference_ufo, font_metrics)

    db.saveImage(str(output))
    db.endDrawing()


def layout() -> tuple[Box, Box, list[list[Box]]]:
    header = Box(MARGIN, PAGE - MARGIN - HEADER_H, PAGE - MARGIN * 2, HEADER_H)
    footer = Box(MARGIN, MARGIN, PAGE - MARGIN * 2, FOOTER_H)
    content_y = footer.y1 + GAP
    content_h = header.y - GAP - content_y
    row_h = (content_h - GAP) / 2
    col_w = (PAGE - MARGIN * 2 - GAP * 2) / 3
    xs = [MARGIN + (col_w + GAP) * index for index in range(3)]
    bottom_y = content_y
    top_y = content_y + row_h + GAP
    rows = [
        [Box(x, top_y, col_w, row_h) for x in xs],
        [Box(x, bottom_y, col_w, row_h) for x in xs],
    ]
    return header, footer, rows


def draw_header(db, box: Box, title: str, input_text: str) -> None:
    draw_panel(db, box, PANEL)
    bottom_baseline = box.y + HEADER_INSET + HEADER_TEXT_SIZE * 0.22
    top_baseline = bottom_baseline + HEADER_LINE_GAP
    left_x = box.x + HEADER_INSET
    right_x = box.x1 - HEADER_INSET
    draw_text(db, title, (left_x, top_baseline), HEADLINE_BOLD_FONT, HEADER_TEXT_SIZE, fallback="Helvetica")
    draw_text(
        db,
        f"{REPO_URL}  @  {git_short_hash()}",
        (left_x, bottom_baseline),
        HEADLINE_FONT,
        HEADER_TEXT_SIZE,
        fallback="Helvetica",
    )
    draw_text(
        db,
        f"Input: {input_text}",
        (right_x, top_baseline),
        HEADLINE_FONT,
        HEADER_TEXT_SIZE,
        fallback="Helvetica",
        align="right",
    )
    draw_text(
        db,
        f"Unicode: {unicode_sequence(input_text)}",
        (right_x, bottom_baseline),
        HEADLINE_FONT,
        HEADER_TEXT_SIZE,
        fallback="Helvetica",
        align="right",
    )


def draw_row(
    db,
    boxes: list[Box],
    row_label: str,
    glyph: Glyph,
    peer: Glyph,
    overlay_underlay: Glyph,
    viewport: tuple[float, float, float, float],
    font_metrics: FontMetrics,
    source_image: Path | None,
    overlay_mode: str,
) -> None:
    titles = ["Outline Structure", "Rendered Preview", "Debug Overlay"]
    for box, title in zip(boxes, titles):
        draw_panel(db, box, PANEL_DARK)
        draw_cell_label(db, box, row_label, title)

    display_viewport = glyph_bounds([glyph], font_metrics)
    draw_structure_cell(db, boxes[0].content_area(), glyph, font_metrics, display_viewport)
    draw_preview_cell(db, boxes[1].content_area(), glyph, font_metrics, display_viewport)
    draw_overlay_cell(
        db,
        boxes[2].content_area(),
        glyph,
        overlay_underlay,
        font_metrics,
        viewport,
        source_image,
        overlay_mode,
    )


def draw_cell_label(db, box: Box, row_label: str, title: str) -> None:
    label = box.label_area()
    draw_text(db, row_label, (label.x + 22, label.y + 18), MONO_FONT, TECH_SIZE, fallback="Courier")
    draw_text(
        db,
        title,
        (label.x1 - 22, label.y + 18),
        MONO_FONT,
        TECH_SIZE,
        fallback="Courier",
        align="right",
    )


def draw_structure_cell(
    db,
    box: Box,
    glyph: Glyph,
    font_metrics: FontMetrics,
    viewport: tuple[float, float, float, float],
) -> None:
    content = box
    draw_cell_background(db, content)
    transform = make_transform(viewport, content, padding=GLYPH_PADDING)
    if SHOW_METRIC_LINES:
        draw_metric_lines(db, content, transform, font_metrics)
    draw_filled_glyph(db, glyph, transform, RASTER_GRAY, alpha=0.12)
    draw_glyph_outline(db, glyph, transform, WHITE_LINE, width=STRUCTURE_OUTLINE_WIDTH, alpha=0.95)
    draw_glyph_handles(db, glyph, transform)
    draw_glyph_points(db, glyph, transform)


def draw_preview_cell(
    db,
    box: Box,
    glyph: Glyph,
    font_metrics: FontMetrics,
    union: tuple[float, float, float, float],
) -> None:
    content = box
    stroke(db, None)
    draw_cell_background(db, content)
    transform = make_transform(union, content, padding=GLYPH_PADDING)
    if SHOW_METRIC_LINES:
        draw_metric_lines(db, content, transform, font_metrics, alpha=0.45)
    draw_filled_glyph(db, glyph, transform, YELLOW, alpha=1.0)


def draw_overlay_cell(
    db,
    box: Box,
    glyph: Glyph,
    underlay: Glyph,
    font_metrics: FontMetrics,
    union: tuple[float, float, float, float],
    source_image: Path | None,
    overlay_mode: str,
) -> None:
    content = box
    stroke(db, None)
    draw_cell_background(db, content)
    underlay_bounds = glyph_bounds([underlay], font_metrics)
    glyph_bounds_value = glyph_bounds([glyph], font_metrics)
    overlay_transform = make_transform(underlay_bounds, content, padding=GLYPH_PADDING)
    draw_filled_glyph(db, underlay, overlay_transform, RASTER_GRAY, alpha=0.72)
    normalized_glyph = normalize_glyph_to_bounds(glyph, glyph_bounds_value, underlay_bounds)
    draw_filled_glyph(db, normalized_glyph, overlay_transform, GREEN, alpha=0.48)
    draw_error_overlay(db, underlay, normalized_glyph, overlay_transform, content)


def draw_footer(
    db,
    box: Box,
    reference: Glyph,
    traced: Glyph,
    metrics: dict[str, str],
    source_image: Path | None,
    reference_ufo: Path,
    font_metrics: FontMetrics,
) -> None:
    draw_panel(db, box, PANEL)
    ref_stats = glyph_stats(reference)
    trace_stats = glyph_stats(traced)
    columns = [
        [
            ("input", display_input_text(reference.name)),
            ("unicode", unicode_sequence(display_input_text(reference.name))),
            ("source", source_image.name if source_image else "--"),
            ("ufo", reference_ufo.name),
            ("upm", format_number(font_metrics.units_per_em)),
            ("renderer", "Drawbot Skia"),
        ],
        [
            ("reference contours", str(ref_stats["contours"])),
            ("reference oncurves", str(ref_stats["oncurves"])),
            ("reference offcurves", str(ref_stats["offcurves"])),
            ("reference curves", str(ref_stats["curves"])),
            ("reference lines", str(ref_stats["lines"])),
            ("reference width", format_number(reference.width)),
        ],
        [
            ("trace contours", str(trace_stats["contours"])),
            ("trace oncurves", str(trace_stats["oncurves"])),
            ("trace offcurves", str(trace_stats["offcurves"])),
            ("trace curves", str(trace_stats["curves"])),
            ("trace lines", str(trace_stats["lines"])),
            ("trace width", format_number(traced.width)),
        ],
        [
            ("score", metric(metrics, "score")),
            ("eval score", metric(metrics, "eval_score")),
            ("raster iou", percent_metric(metrics, "raster_iou")),
            ("mean ref dist", metric(metrics, "mean_ref_distance")),
            ("matched ref pts", metric(metrics, "ref_oncurves_matched")),
            ("hv score", metric(metrics, "hv")),
        ],
        [
            ("traced extras", metric(metrics, "traced_unmatched")),
            ("point delta", signed_int(trace_stats["oncurves"] - ref_stats["oncurves"])),
            ("extrema fixes", metric(metrics, "missed_extrema_fixed")),
            ("deviation splits", metric(metrics, "high_deviation_splits")),
            ("tangent overrides", tangent_override_summary(metrics)),
            ("final repairs", metric(metrics, "final_outline_repairs")),
        ],
    ]
    col_w = (box.w - 64) / len(columns)
    for col_index, rows in enumerate(columns):
        x = box.x + 32 + col_index * col_w
        y = box.y1 - 64
        for label, value in rows:
            draw_key_value(db, titlecase_label(label), value, (x, y), col_w - 24)
            y -= FOOTER_LINE_GAP


def draw_panel(db, box: Box, color: str) -> None:
    stroke(db, None)
    fill(db, color)
    db.rect(box.x, box.y, box.w, box.h)
    stroke(db, STROKE)
    db.strokeWidth(1.5)
    db.rect(box.x, box.y, box.w, box.h)


def draw_cell_background(db, box: Box) -> None:
    stroke(db, None)
    fill(db, PANEL_DARK)
    db.rect(box.x, box.y, box.w, box.h)


def draw_page_grid(db) -> None:
    with db.savedState():
        db.fill(None)
        for step, color, width, alpha in (
            (24, GRID_MINOR, 0.65, 0.42),
            (96, GRID_MAJOR, 1.0, 0.52),
        ):
            stroke(db, color, alpha)
            db.strokeWidth(width)
            for value in range(0, PAGE + 1, step):
                db.line((value, 0), (value, PAGE))
                db.line((0, value), (PAGE, value))


def draw_editor_grid(db, box: Box, minor: int = 18, major: int = 90, alpha: float = 0.62) -> None:
    with db.savedState():
        db.fill(None)
        stroke(db, GRID_MINOR, alpha)
        db.strokeWidth(0.7)
        for x in frange(box.x, box.x1, minor):
            db.line((x, box.y), (x, box.y1))
        for y in frange(box.y, box.y1, minor):
            db.line((box.x, y), (box.x1, y))
        stroke(db, GRID_MAJOR, alpha)
        db.strokeWidth(1.1)
        for x in frange(box.x, box.x1, major):
            db.line((x, box.y), (x, box.y1))
        for y in frange(box.y, box.y1, major):
            db.line((box.x, y), (box.x1, y))


def draw_overlay_grid(db, box: Box) -> None:
    with db.savedState():
        db.fill(None)
        stroke(db, "#ffffff", 0.12)
        db.strokeWidth(0.8)
        for x in frange(box.x, box.x1, 18):
            db.line((x, box.y), (x, box.y1))
        for y in frange(box.y, box.y1, 18):
            db.line((box.x, y), (box.x1, y))


def draw_metric_lines(
    db,
    box: Box,
    transform: Transform,
    metrics: FontMetrics,
    alpha: float = 0.75,
) -> None:
    guide_values = [metrics.descender, 0, metrics.x_height, metrics.cap_height, metrics.ascender]
    stroke(db, GRID_GREEN, alpha)
    db.strokeWidth(1.6)
    for value in guide_values:
        _, y = transform(Point(0, value, "line", False))
        if box.y <= y <= box.y1:
            db.line((box.x, y), (box.x1, y))


def draw_glyph_outline(
    db,
    glyph: Glyph,
    transform: Transform,
    color: str,
    width: float,
    alpha: float = 1.0,
) -> None:
    path = glyph_path(db, glyph, transform)
    with db.savedState():
        db.fill(None)
        stroke(db, color, alpha)
        db.strokeWidth(width)
        db.drawPath(path)


def draw_filled_glyph(db, glyph: Glyph, transform: Transform, color: str, alpha: float) -> None:
    path = glyph_path(db, glyph, transform)
    with db.savedState():
        fill(db, color, alpha)
        db.stroke(None)
        db.drawPath(path)


def draw_error_overlay(db, reference: Glyph, traced: Glyph, transform: Transform, box: Box) -> None:
    error_png = build_error_overlay_png(reference, traced, transform, box)
    if error_png is None:
        return
    try:
        with db.savedState():
            db.image(str(error_png), (box.x, box.y))
    finally:
        error_png.unlink(missing_ok=True)


def build_error_overlay_png(
    reference: Glyph,
    traced: Glyph,
    transform: Transform,
    box: Box,
) -> Path | None:
    try:
        from PIL import Image, ImageChops
    except ImportError:
        return None

    mask_scale = 2
    reference_mask = rasterize_glyph_mask(reference, transform, box, scale=mask_scale)
    traced_mask = rasterize_glyph_mask(traced, transform, box, scale=mask_scale)
    if reference_mask is None or traced_mask is None:
        return None

    diff = ImageChops.difference(reference_mask, traced_mask)
    diff = diff.point(lambda value: 0 if value < 16 else min(255, round(value * 1.65)))
    if diff.getbbox() is None:
        return None

    width = max(1, round(box.w))
    height = max(1, round(box.h))
    if mask_scale != 1:
        diff = diff.resize((width, height), Image.Resampling.LANCZOS)
    red, green, blue = hex_to_rgb(RED)
    overlay = Image.new("RGBA", (width, height), (red, green, blue, 0))
    overlay.putalpha(diff.point(lambda value: round(value * 0.82)))
    with tempfile.NamedTemporaryFile(prefix="img2bez-specimen-error-", suffix=".png", delete=False) as file:
        overlay_path = Path(file.name)
    overlay.save(overlay_path)
    return overlay_path


def rasterize_glyph_mask(
    glyph: Glyph,
    transform: Transform,
    box: Box,
    scale: int,
):
    try:
        from PIL import Image
        import skia
    except ImportError:
        return None

    width = max(1, round(box.w * scale))
    height = max(1, round(box.h * scale))
    surface = skia.Surface(width, height)
    canvas = surface.getCanvas()
    canvas.clear(skia.ColorTRANSPARENT)
    path = skia.Path()

    def to_image(point: Point) -> tuple[float, float]:
        page_x, page_y = transform(point)
        return (page_x - box.x) * scale, (box.h - (page_y - box.y)) * scale

    for contour in glyph.contours:
        segments = contour_segments(contour.points)
        if not segments:
            continue
        path.moveTo(*to_image(segments[0][0]))
        for segment in segments:
            if len(segment) == 2:
                path.lineTo(*to_image(segment[1]))
            elif len(segment) == 3:
                path.quadTo(*to_image(segment[1]), *to_image(segment[2]))
            elif len(segment) == 4:
                path.cubicTo(*to_image(segment[1]), *to_image(segment[2]), *to_image(segment[3]))
        path.close()

    paint = skia.Paint(AntiAlias=True, Color=skia.ColorWHITE, Style=skia.Paint.kFill_Style)
    canvas.drawPath(path, paint)
    snapshot = surface.makeImageSnapshot()
    image = Image.frombytes("RGBA", (width, height), snapshot.tobytes())
    return image.getchannel("A")


def draw_glyph_handles(db, glyph: Glyph, transform: Transform) -> None:
    with db.savedState():
        stroke(db, HANDLE_LINE, 0.9)
        db.strokeWidth(HANDLE_LINE_WIDTH)
        for contour in glyph.contours:
            for segment in contour_segments(contour.points):
                if len(segment) == 4:
                    p0, c1, c2, p3 = segment
                    db.line(transform(p0), transform(c1))
                    db.line(transform(p3), transform(c2))


def draw_glyph_points(db, glyph: Glyph, transform: Transform) -> None:
    for contour in glyph.contours:
        for point in contour.points:
            x, y = transform(point)
            if not point.oncurve:
                stroke(db, PURPLE)
                fill(db, BG)
                db.strokeWidth(2.7)
                db.oval(x - 6.5, y - 6.5, 13, 13)
            elif point.smooth:
                stroke(db, GREEN_DARK)
                fill(db, BG)
                db.strokeWidth(2.9)
                db.oval(x - 7.1, y - 7.1, 14.2, 14.2)
            else:
                stroke(db, ORANGE)
                fill(db, BG)
                db.strokeWidth(2.7)
                db.rect(x - 6.2, y - 6.2, 12.4, 12.4)


def draw_image_fit(
    db,
    image_path: Path,
    box: Box,
    alpha: float,
    crop_to_ink: bool = False,
    tint: str | None = None,
) -> None:
    try:
        from PIL import Image
    except ImportError:
        return
    draw_path = image_path
    cleanup_path: Path | None = None
    if crop_to_ink:
        cleanup_path = crop_image_to_ink(image_path, tint=tint)
        if cleanup_path:
            draw_path = cleanup_path
    try:
        with Image.open(draw_path) as image:
            image_w, image_h = image.size
        if image_w <= 0 or image_h <= 0:
            return
        scale = min(box.w / image_w, box.h / image_h)
        x = box.x + (box.w - image_w * scale) / 2
        y = box.y + (box.h - image_h * scale) / 2
        with db.savedState():
            db.translate(x, y)
            db.scale(scale, scale)
            try:
                db.image(str(draw_path), (0, 0), alpha=alpha)
            except TypeError:
                draw_image_fit_skiacanvas_fallback(db, draw_path, image_w, image_h, alpha)
    finally:
        if cleanup_path:
            cleanup_path.unlink(missing_ok=True)


def crop_image_to_ink(image_path: Path, tint: str | None = None) -> Path | None:
    try:
        from PIL import Image, ImageChops
    except ImportError:
        return None
    with Image.open(image_path) as source:
        image = source.convert("RGBA")
    alpha = image.getchannel("A")
    if alpha.getextrema()[0] < 255:
        mask = alpha.point(lambda value: 255 if value > 8 else 0)
        bbox = mask.getbbox()
    else:
        background = Image.new("RGBA", image.size, image.getpixel((0, 0)))
        diff = ImageChops.difference(image, background).convert("L")
        mask = diff.point(lambda value: 255 if value > 8 else 0)
        bbox = mask.getbbox()
    if not bbox:
        return None
    x0, y0, x1, y1 = bbox
    pad = round(max(x1 - x0, y1 - y0) * 0.04)
    x0 = max(0, x0 - pad)
    y0 = max(0, y0 - pad)
    x1 = min(image.width, x1 + pad)
    y1 = min(image.height, y1 + pad)
    crop = image.crop((x0, y0, x1, y1))
    if tint:
        mask = mask.crop((x0, y0, x1, y1))
        red, green, blue = hex_to_rgb(tint)
        tinted = Image.new("RGBA", crop.size, (red, green, blue, 0))
        tinted.putalpha(mask)
        crop = tinted
    with tempfile.NamedTemporaryFile(prefix="img2bez-specimen-crop-", suffix=".png", delete=False) as file:
        crop_path = Path(file.name)
    crop.save(crop_path)
    return crop_path


def hex_to_rgb(color: str) -> tuple[int, int, int]:
    clean = color.removeprefix("#")
    if len(clean) != 6:
        return (255, 255, 255)
    return int(clean[0:2], 16), int(clean[2:4], 16), int(clean[4:6], 16)


def draw_image_fit_skiacanvas_fallback(
    db,
    image_path: Path,
    image_w: int,
    image_h: int,
    alpha: float,
) -> None:
    """Fallback for older drawbot-skia checkouts with a stale drawImage call.

    The repo-local venv installs eliheuer/drawbot-skia, but this keeps the
    specimen editable on machines that still have an older local checkout.
    """
    try:
        import skia
    except ImportError:
        return
    image = skia.Image.open(str(image_path))
    if image is None:
        return
    paint = skia.Paint()
    if alpha != 1.0:
        paint.setAlpha(round(alpha * 255))
    canvas = getattr(db, "_canvas", None)
    if canvas is None:
        return
    with db.savedState():
        db.translate(0, image_h)
        db.scale(1, -1)
        canvas.drawImage(image, 0, 0, skia.SamplingOptions(), paint)


def glyph_path(db, glyph: Glyph, transform: Transform):
    path = db.BezierPath()
    for contour in glyph.contours:
        segments = contour_segments(contour.points)
        if not segments:
            continue
        start = segments[0][0]
        path.moveTo(transform(start))
        for segment in segments:
            if len(segment) == 2:
                path.lineTo(transform(segment[1]))
            elif len(segment) == 3:
                path.qCurveTo(transform(segment[1]), transform(segment[2]))
            elif len(segment) == 4:
                path.curveTo(transform(segment[1]), transform(segment[2]), transform(segment[3]))
        path.closePath()
    return path


def contour_segments(points: list[Point]) -> list[list[Point]]:
    first_idx = next((index for index, point in enumerate(points) if point.oncurve), None)
    if first_idx is None:
        return []
    current = points[first_idx]
    offcurves: list[Point] = []
    segments: list[list[Point]] = []
    count = len(points)
    for step in range(1, count + 1):
        point = points[(first_idx + step) % count]
        if not point.oncurve:
            offcurves.append(point)
            continue
        if len(offcurves) >= 2:
            segments.append([current, offcurves[-2], offcurves[-1], point])
        elif len(offcurves) == 1:
            segments.append([current, offcurves[0], point])
        else:
            segments.append([current, point])
        current = point
        offcurves = []
    return segments


def make_transform(
    bounds: tuple[float, float, float, float],
    box: Box,
    padding: float,
) -> Transform:
    min_x, min_y, max_x, max_y = bounds
    width = max(max_x - min_x, 1.0)
    height = max(max_y - min_y, 1.0)
    scale = min((box.w - padding * 2) / width, (box.h - padding * 2) / height)
    dx = box.x + (box.w - width * scale) / 2 - min_x * scale
    dy = box.y + (box.h - height * scale) / 2 - min_y * scale

    def transform(point: Point) -> tuple[float, float]:
        return point.x * scale + dx, point.y * scale + dy

    return transform


def normalize_glyph_to_bounds(
    glyph: Glyph,
    source_bounds: tuple[float, float, float, float],
    target_bounds: tuple[float, float, float, float],
) -> Glyph:
    source_min_x, source_min_y, source_max_x, source_max_y = source_bounds
    target_min_x, target_min_y, target_max_x, target_max_y = target_bounds
    source_width = max(source_max_x - source_min_x, 1.0)
    source_height = max(source_max_y - source_min_y, 1.0)
    target_width = max(target_max_x - target_min_x, 1.0)
    target_height = max(target_max_y - target_min_y, 1.0)
    scale = min(target_width / source_width, target_height / source_height)
    source_center_x = (source_min_x + source_max_x) / 2
    source_center_y = (source_min_y + source_max_y) / 2
    target_center_x = (target_min_x + target_max_x) / 2
    target_center_y = (target_min_y + target_max_y) / 2

    contours = []
    for contour in glyph.contours:
        contours.append(
            Contour(
                [
                    Point(
                        x=(point.x - source_center_x) * scale + target_center_x,
                        y=(point.y - source_center_y) * scale + target_center_y,
                        typ=point.typ,
                        smooth=point.smooth,
                    )
                    for point in contour.points
                ]
            )
        )
    return Glyph(name=f"{glyph.name}.normalized", contours=contours, width=target_width)


def glyph_bounds(glyphs: Iterable[Glyph], metrics: FontMetrics) -> tuple[float, float, float, float]:
    points = [point for glyph in glyphs for contour in glyph.contours for point in contour.points]
    if not points:
        return (0.0, metrics.descender, 600.0, metrics.ascender)
    min_x = min(point.x for point in points)
    max_x = max(point.x for point in points)
    min_y = min(point.y for point in points)
    max_y = max(point.y for point in points)
    pad_x = max((max_x - min_x) * 0.04, 18.0)
    pad_y = max((max_y - min_y) * 0.04, 18.0)
    return min_x - pad_x, min_y - pad_y, max_x + pad_x, max_y + pad_y


def load_glyph(ufo: Path, glyph_name: str) -> Glyph:
    glif_path = resolve_glif(ufo, glyph_name)
    root = ET.parse(glif_path).getroot()
    advance = root.find("advance")
    width = float(advance.attrib.get("width", "0")) if advance is not None else 0.0
    contours = []
    outline = root.find("outline")
    if outline is not None:
        for contour_el in outline.findall("contour"):
            points = [
                Point(
                    x=float(point.attrib["x"]),
                    y=float(point.attrib["y"]),
                    typ=point.attrib.get("type"),
                    smooth=point.attrib.get("smooth") == "yes",
                )
                for point in contour_el.findall("point")
            ]
            if points:
                contours.append(Contour(points))
    return Glyph(name=glyph_name, contours=contours, width=width)


def resolve_glif(ufo: Path, glyph_name: str) -> Path:
    for glyphs_dir_name in ("glyphs", "glyphs.public.default"):
        contents_path = ufo / glyphs_dir_name / "contents.plist"
        if not contents_path.exists():
            continue
        with contents_path.open("rb") as file:
            contents = plistlib.load(file)
        filename = contents.get(glyph_name)
        if filename:
            return ufo / glyphs_dir_name / filename
    raise FileNotFoundError(f"glyph {glyph_name!r} not found in {ufo}")


def load_font_metrics(ufo: Path) -> FontMetrics:
    info_path = ufo / "fontinfo.plist"
    if not info_path.exists():
        return FontMetrics(1000.0, 800.0, -200.0, 700.0, 500.0)
    with info_path.open("rb") as file:
        info = plistlib.load(file)
    return FontMetrics(
        units_per_em=float(info.get("unitsPerEm", 1000)),
        ascender=float(info.get("ascender", 800)),
        descender=float(info.get("descender", -200)),
        cap_height=float(info.get("capHeight", info.get("ascender", 700))),
        x_height=float(info.get("xHeight", 500)),
    )


def read_metrics(path: Path) -> dict[str, str]:
    if not path.exists():
        return {}
    with path.open("r", encoding="utf-8") as file:
        rows = list(csv.DictReader(file, delimiter="\t"))
    return rows[0] if rows else {}


def glyph_stats(glyph: Glyph) -> dict[str, int]:
    stats = {
        "contours": len(glyph.contours),
        "oncurves": 0,
        "offcurves": 0,
        "lines": 0,
        "curves": 0,
    }
    for contour in glyph.contours:
        for point in contour.points:
            if point.oncurve:
                stats["oncurves"] += 1
                if point.typ == "line":
                    stats["lines"] += 1
                elif point.typ in {"curve", "qcurve"}:
                    stats["curves"] += 1
            else:
                stats["offcurves"] += 1
    return stats


def stats_line(stats: dict[str, int]) -> str:
    return (
        f"{stats['contours']} contours  {stats['oncurves']} on  "
        f"{stats['offcurves']} off  {stats['curves']} curves  {stats['lines']} lines"
    )


def stats_compact(stats: dict[str, int]) -> str:
    return (
        f"c{stats['contours']} on{stats['oncurves']} off{stats['offcurves']} "
        f"cu{stats['curves']} ln{stats['lines']}"
    )


def signed_int(value: int) -> str:
    return f"{value:+d}"


def metric(metrics: dict[str, str], key: str) -> str:
    value = metrics.get(key)
    return value if value not in {None, ""} else "--"


def percent_metric(metrics: dict[str, str], key: str) -> str:
    value = metric(metrics, key)
    if value == "--":
        return value
    try:
        return f"{float(value):.1f}%"
    except ValueError:
        return value


def tangent_override_summary(metrics: dict[str, str]) -> str:
    return "/".join(
        metric(metrics, key)
        for key in (
            "strong_tangent_overrides",
            "clean_tangent_overrides",
            "visible_tangent_overrides",
        )
    )


def format_number(value: float) -> str:
    return str(round(value)) if math.isclose(value, round(value)) else f"{value:.1f}"


def wrap_value(value: str, width: int) -> list[str]:
    return textwrap.wrap(value, width=width, break_long_words=False) or ["--"]


def titlecase_label(value: str) -> str:
    text = value.title()
    for source, replacement in {
        "Iou": "IoU",
        "Ufo": "UFO",
        "Pts": "Pts",
        "Ref": "Ref",
        "Upm": "UPM",
    }.items():
        text = text.replace(source, replacement)
    return text


def unicode_sequence(value: str) -> str:
    if not value:
        return "--"
    return " ".join(f"U+{ord(char):04X}" for char in value)


def display_input_text(glyph_name: str) -> str:
    if not glyph_name.startswith("stress_"):
        return glyph_name
    key = glyph_name.removeprefix("stress_")
    if not key:
        return glyph_name
    parts = key.split("_")
    if all(part.startswith("u") and len(part) >= 5 for part in parts):
        try:
            return "".join(chr(int(part[1:], 16)) for part in parts)
        except ValueError:
            return key
    return key


def git_short_hash() -> str:
    try:
        result = subprocess.run(
            ["git", "rev-parse", "--short=12", "HEAD"],
            cwd=REPO_DIR,
            text=True,
            capture_output=True,
            check=False,
        )
    except OSError:
        return "unknown"
    value = result.stdout.strip()
    return value if result.returncode == 0 and value else "unknown"


def set_font(db, names: list[str], size: float, fallback: str) -> None:
    for name in names:
        try:
            db.font(name, size)
            return
        except Exception:
            continue
    db.font(fallback, size)


def draw_text(
    db,
    text: str,
    position: tuple[float, float],
    fonts: list[str],
    size: float,
    fallback: str,
    align: str | None = None,
    color: str = TEXT,
) -> None:
    set_font(db, fonts, size, fallback=fallback)
    fill(db, color)
    if align is None:
        db.text(text, position)
    else:
        db.text(text, position, align=align)


def draw_key_value(db, key: str, value: str, position: tuple[float, float], max_width: float) -> None:
    x, y = position
    key_text = f"{key}:"
    draw_text(db, key_text, (x, y), MONO_FONT, FOOTER_SIZE, fallback="Courier")
    key_width = text_width(db, f"{key_text} ", MONO_FONT, FOOTER_SIZE, fallback="Courier")
    value_x = x + key_width
    value_width = max_width - key_width
    clipped_value = truncate_to_width(db, value, value_width, MONO_FONT, FOOTER_SIZE, fallback="Courier")
    draw_text(db, clipped_value, (value_x, y), MONO_FONT, FOOTER_SIZE, fallback="Courier", color=GREEN)


def truncate_to_width(db, text: str, max_width: float, fonts: list[str], size: float, fallback: str) -> str:
    if max_width <= 0:
        return ""
    if text_width(db, text, fonts, size, fallback=fallback) <= max_width:
        return text
    marker = "..."
    if text_width(db, marker, fonts, size, fallback=fallback) > max_width:
        return ""
    clipped = text
    while clipped:
        candidate = f"{clipped}{marker}"
        if text_width(db, candidate, fonts, size, fallback=fallback) <= max_width:
            return candidate
        clipped = clipped[:-1]
    return marker


def text_width(db, text: str, fonts: list[str], size: float, fallback: str) -> float:
    set_font(db, fonts, size, fallback=fallback)
    try:
        return float(db.textSize(text)[0])
    except Exception:
        return len(text) * size * 0.56


def fill(db, color: str | None, alpha: float = 1.0) -> None:
    if color is None:
        db.fill(None)
        return
    db.fill(*rgba(color, alpha))


def stroke(db, color: str | None, alpha: float = 1.0) -> None:
    if color is None:
        db.stroke(None)
        return
    db.stroke(*rgba(color, alpha))


def rgba(color: str, alpha: float = 1.0) -> tuple[float, float, float, float]:
    color = color.lstrip("#")
    return (
        int(color[0:2], 16) / 255,
        int(color[2:4], 16) / 255,
        int(color[4:6], 16) / 255,
        alpha,
    )


def frange(start: float, stop: float, step: float) -> Iterable[float]:
    value = start
    while value <= stop + 0.001:
        yield value
        value += step


if __name__ == "__main__":
    raise SystemExit(main())
