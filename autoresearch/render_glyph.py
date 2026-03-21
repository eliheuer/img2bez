#!/usr/bin/env python3
"""Render a UFO glyph to a PNG using drawbot-skia.

The output image represents the full ascender-to-descender height of the font,
with the glyph drawn in black on a white background. This matches the coordinate
space that img2bez expects when called with:

    --target-height <ascender - descender>
    --y-offset <descender>

Usage:
    python render_glyph.py <ufo_path> <glyph_name> <output_png> [--height 700]

Prints to stdout (for shell script capture):
    target_height=<float>
    y_offset=<float>
    advance_width=<int>
"""

import argparse
import os
import plistlib
import sys


def get_font_metrics(ufo_path):
    """Read ascender/descender/UPM from fontinfo.plist."""
    fontinfo_path = os.path.join(ufo_path, "fontinfo.plist")
    if not os.path.exists(fontinfo_path):
        return {"unitsPerEm": 1000, "ascender": 800, "descender": -200}
    with open(fontinfo_path, "rb") as f:
        info = plistlib.load(f)
    return {
        "unitsPerEm": info.get("unitsPerEm", 1000),
        "ascender": info.get("ascender", 800),
        "descender": info.get("descender", -200),
    }


def get_glif_path(ufo_path, glyph_name):
    """Resolve glyph name to its .glif file path via contents.plist."""
    for glyphs_dir in ["glyphs", "glyphs.public.default"]:
        contents_path = os.path.join(ufo_path, glyphs_dir, "contents.plist")
        if not os.path.exists(contents_path):
            continue
        with open(contents_path, "rb") as f:
            contents = plistlib.load(f)
        if glyph_name in contents:
            return os.path.join(ufo_path, glyphs_dir, contents[glyph_name])
    return None


def render_glyph(ufo_path, glyph_name, output_png, px_height=700):
    """Render a single glyph to PNG."""
    try:
        import drawbot_skia.drawbot as db
    except ImportError:
        sys.exit("Error: drawbot-skia not installed. Run: pip install drawbot-skia ufoLib2")

    try:
        import ufoLib2
    except ImportError:
        sys.exit("Error: ufoLib2 not installed. Run: pip install ufoLib2")

    metrics = get_font_metrics(ufo_path)
    ascender = metrics["ascender"]
    descender = metrics["descender"]
    upm = metrics["unitsPerEm"]

    # Total font height in units (ascender to descender)
    total_height = ascender - descender  # e.g. 1088 for ascender=832, descender=-256

    # Load glyph
    ufo = ufoLib2.Font.open(ufo_path)
    if glyph_name not in ufo:
        sys.exit(f"Error: glyph '{glyph_name}' not found in {ufo_path}")
    glyph = ufo[glyph_name]

    advance_width = glyph.width if glyph.width else upm
    scale = px_height / total_height

    # Compute glyph bounds to detect negative-LSB glyphs (e.g. j, f).
    # If the glyph extends left of x=0, extend the canvas and shift the
    # drawing so the left edge is at x=0 in the PNG.
    glyph_bounds = glyph.getBounds() if hasattr(glyph, "getBounds") else None
    if glyph_bounds is not None:
        glyph_min_x = glyph_bounds[0]
    else:
        # Fallback: compute bounds from contour points
        xs = [pt.x for contour in glyph.contours for pt in contour.points]
        glyph_min_x = min(xs) if xs else 0.0

    # x_offset: extra pixels added to the left of the canvas so negative-LSB
    # glyphs are fully visible. In font units this is max(0, -glyph_min_x).
    x_shift_font = max(0.0, -glyph_min_x)
    px_x_offset = int(x_shift_font * scale + 0.5)
    px_width = max(int(advance_width * scale) + px_x_offset, 1)

    db.newDrawing()
    db.newPage(px_width, px_height)

    # White background
    db.fill(1, 1, 1, 1)
    db.rect(0, 0, px_width, px_height)

    # Draw glyph in black.
    # Coordinate transform:
    #   - Font space: y=0 at baseline, y=descender at bottom, y=ascender at top
    #   - Drawbot/CG space: y=0 at page bottom, y=px_height at page top
    #   - We want font y=descender → page y=0, font y=ascender → page y=px_height
    #   - Transform: page_y = (font_y - descender) * scale
    #              = font_y * scale - descender * scale
    #   - As drawbot ops: translate(x_shift, -descender*scale) then scale(scale, scale)
    with db.savedState():
        db.translate(x_shift_font * scale, -descender * scale)
        db.scale(scale, scale)
        db.fill(0, 0, 0, 1)

        pen = db.BezierPath()
        glyph.draw(pen)
        db.drawPath(pen)

    db.saveImage(output_png)
    db.endDrawing()

    return {
        "target_height": total_height,
        "y_offset": descender,
        "advance_width": advance_width,
        "px_height": px_height,
        "px_width": px_width,
        "x_shift_font": x_shift_font,
    }


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Render a UFO glyph to PNG")
    parser.add_argument("ufo_path", help="Path to the UFO font")
    parser.add_argument("glyph_name", help="Glyph name (e.g. 'A')")
    parser.add_argument("output_png", help="Output PNG file path")
    parser.add_argument("--height", type=int, default=700, help="Output image height in pixels")
    args = parser.parse_args()

    result = render_glyph(args.ufo_path, args.glyph_name, args.output_png, args.height)

    # Print key=value pairs for shell script to capture
    print(f"target_height={result['target_height']}")
    print(f"y_offset={result['y_offset']}")
    print(f"advance_width={result['advance_width']}")
