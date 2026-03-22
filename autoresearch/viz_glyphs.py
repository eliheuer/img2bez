#!/usr/bin/env python3
"""
viz_glyphs.py — render bezier point-structure comparison images.

Usage:
    python3 viz_glyphs.py <traced.ufo> <reference.ufo> <work_dir> [glyph ...]

If no glyphs are listed, renders all glyphs present in both UFOs.
Output: <work_dir>/viz_<name>.png  (one image per glyph)

Each image is split left/right:
  left  — reference glyph (gray)
  right — traced glyph    (blue outline, red on-curve dots, cyan off-curve squares)
"""

import sys
import os
import ufoLib2
import drawBot as db


def glyph_bbox(glyph):
    xs, ys = [], []
    for contour in glyph.contours:
        for p in contour:
            xs.append(p.x)
            ys.append(p.y)
    if not xs:
        return (0, 0, 100, 100)
    return min(xs), min(ys), max(xs), max(ys)


def draw_structure(glyph, x_off, y_off, scale, stroke_col, on_col, off_col):
    db.save()
    db.translate(x_off, y_off)
    db.scale(scale)

    # Outline
    bp = db.BezierPath()
    for contour in glyph.contours:
        pts = list(contour)
        if not pts:
            continue
        si = next((i for i, p in enumerate(pts) if p.type != "offcurve"), 0)
        bp.moveTo((pts[si].x, pts[si].y))
        i = (si + 1) % len(pts)
        while i != si:
            p = pts[i]
            if p.type == "line":
                bp.lineTo((p.x, p.y))
                i = (i + 1) % len(pts)
            elif p.type == "curve":
                p1 = pts[(i - 2) % len(pts)]
                p2 = pts[(i - 1) % len(pts)]
                bp.curveTo((p1.x, p1.y), (p2.x, p2.y), (p.x, p.y))
                i = (i + 1) % len(pts)
            else:
                i = (i + 1) % len(pts)
        bp.closePath()
    db.fill(None)
    db.stroke(*stroke_col)
    db.strokeWidth(2 / scale)
    db.drawPath(bp)

    # Handle lines
    for contour in glyph.contours:
        pts = list(contour)
        for j, p in enumerate(pts):
            if p.type == "curve":
                p2 = pts[(j - 1) % len(pts)]
                p1 = pts[(j - 2) % len(pts)]
                prv = pts[(j - 3) % len(pts)]
                db.stroke(*off_col)
                db.fill(None)
                db.strokeWidth(1 / scale)
                db.line((p.x, p.y), (p2.x, p2.y))
                if prv.type != "offcurve":
                    db.line((prv.x, prv.y), (p1.x, p1.y))

    # Points
    R_on = 16 / scale
    R_off = 10 / scale
    for contour in glyph.contours:
        for p in contour:
            if p.type != "offcurve":
                db.fill(*on_col)
                db.stroke(1, 1, 1, 1)
                db.strokeWidth(1.5 / scale)
                db.oval(p.x - R_on / 2, p.y - R_on / 2, R_on, R_on)
            else:
                db.fill(*off_col)
                db.stroke(1, 1, 1, 1)
                db.strokeWidth(1.5 / scale)
                db.rect(p.x - R_off / 2, p.y - R_off / 2, R_off, R_off)

    db.restore()


def render_glyph_viz(name, traced_glyph, ref_glyph, out_path):
    x0, y0, x1, y1 = glyph_bbox(traced_glyph)
    gw = max(x1 - x0, 1)
    gh = max(y1 - y0, 1)

    W, H = 900, 500
    PAD = 40
    scale = min((W / 2 - 2 * PAD) / gw, (H - 2 * PAD) / gh)

    # Count segments for subtitle
    def seg_counts(g):
        curves = lines = 0
        for c in g.contours:
            for p in c:
                if p.type == "curve":
                    curves += 1
                elif p.type == "line":
                    lines += 1
        return curves, lines

    tc, tl = seg_counts(traced_glyph)
    rc, rl = seg_counts(ref_glyph)

    db.newDrawing()
    db.newPage(W, H)
    db.fill(0.97)
    db.rect(0, 0, W, H)

    # Title
    db.font("Helvetica-Bold", 22)
    db.fill(0)
    db.text(f"{name}", (20, H - 35))
    db.font("Helvetica", 16)
    db.fill(0.4)
    db.text(f"ref {rc}c+{rl}l   traced {tc}c+{tl}l", (70, H - 35))

    gy = PAD - y0

    # Left panel — reference
    db.fill(0.93)
    db.stroke(None)
    db.rect(0, 0, W / 2, H)
    db.font("Helvetica", 13)
    db.fill(0.5)
    db.text(f"reference  ({rc}c+{rl}l)", (W / 4, 16), align="center")
    draw_structure(
        ref_glyph,
        W / 4 - gw * scale / 2 + PAD / 2,
        gy,
        scale,
        (0.6, 0.6, 0.6),
        (0.55, 0.55, 0.55),
        (0.75, 0.75, 0.75),
    )

    # Right panel — traced
    db.font("Helvetica", 13)
    db.fill(0.35)
    db.text(f"traced  ({tc}c+{tl}l)", (3 * W / 4, 16), align="center")
    draw_structure(
        traced_glyph,
        W / 2 + W / 4 - gw * scale / 2 + PAD / 2,
        gy,
        scale,
        (0.1, 0.2, 0.85),
        (0.85, 0.15, 0.1),
        (0.1, 0.5, 0.9),
    )

    db.saveImage(out_path)
    db.endDrawing()


def main():
    if len(sys.argv) < 4:
        print(__doc__)
        sys.exit(1)

    traced_ufo_path = sys.argv[1]
    ref_ufo_path = sys.argv[2]
    work_dir = sys.argv[3]
    glyph_filter = sys.argv[4:] or None

    traced_font = ufoLib2.Font.open(traced_ufo_path)
    ref_font = ufoLib2.Font.open(ref_ufo_path)

    names = glyph_filter or sorted(
        set(g.name for g in traced_font) & set(g.name for g in ref_font)
    )

    for name in names:
        if name not in traced_font or name not in ref_font:
            print(f"  skip {name} (missing from one UFO)")
            continue
        # Use unicode codepoint filename to avoid macOS case-collision
        file_key = f"uni{ord(name):04X}" if len(name) == 1 else name
        out_path = os.path.join(work_dir, f"viz_{file_key}.png")
        render_glyph_viz(name, traced_font[name], ref_font[name], out_path)
        print(f"  viz {name} → {out_path}")


if __name__ == "__main__":
    main()
