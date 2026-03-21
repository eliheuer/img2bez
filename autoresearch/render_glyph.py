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
import struct
import zlib


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


def _clip_png_rows(png_path, clip_top, clip_bottom, px_height, px_width):
    """Whiten pixel rows outside [clip_top, clip_bottom] in a grayscale/RGB PNG.

    Uses only stdlib (struct + zlib) — no Pillow needed. Works on 8-bit RGB or
    RGBA PNGs as produced by drawbot-skia. Rows with index < clip_top or
    > clip_bottom are set to all-white (0xFF bytes), eliminating phantom pixels
    from anti-aliasing bleed outside the glyph's actual y-bounding-box.
    """
    with open(png_path, "rb") as f:
        data = f.read()

    # Parse PNG: signature (8) + chunks
    sig = data[:8]
    assert sig == b"\x89PNG\r\n\x1a\n", "Not a PNG file"

    # Collect chunks
    chunks = []
    pos = 8
    while pos < len(data):
        length = struct.unpack(">I", data[pos:pos+4])[0]
        ctype = data[pos+4:pos+8]
        cdata = data[pos+8:pos+8+length]
        chunks.append((ctype, cdata))
        pos += 12 + length

    # Find IHDR to get bit depth and color type
    ihdr_data = next(cd for ct, cd in chunks if ct == b"IHDR")
    width, height, bit_depth, color_type = struct.unpack(">IIBB", ihdr_data[:10])
    # color_type: 2=RGB, 6=RGBA, 0=Grayscale
    channels = {0: 1, 2: 3, 6: 4}.get(color_type, 3)
    bytes_per_pixel = channels  # assumes 8-bit

    # Decompress IDAT data
    idat_data = b"".join(cd for ct, cd in chunks if ct == b"IDAT")
    raw = zlib.decompress(idat_data)

    # PNG raw data: each row has 1 filter byte + width*bytes_per_pixel bytes
    stride = 1 + width * bytes_per_pixel
    assert len(raw) == height * stride, f"PNG decode size mismatch: {len(raw)} vs {height * stride}"

    raw = bytearray(raw)
    for row in range(height):
        if row < clip_top or row > clip_bottom:
            start = row * stride
            raw[start] = 0  # filter type None
            raw[start+1:start+stride] = b"\xff" * (stride - 1)

    # Re-compress and rebuild PNG
    new_idat = zlib.compress(bytes(raw), 9)
    new_chunks = []
    for ctype, cdata in chunks:
        if ctype == b"IDAT":
            if not any(ct == b"IDAT" for ct, _ in new_chunks):
                new_chunks.append((b"IDAT", new_idat))
        else:
            new_chunks.append((ctype, cdata))

    out = bytearray(sig)
    for ctype, cdata in new_chunks:
        out += struct.pack(">I", len(cdata))
        out += ctype
        out += cdata
        crc = zlib.crc32(ctype + cdata) & 0xFFFFFFFF
        out += struct.pack(">I", crc)

    with open(png_path, "wb") as f:
        f.write(out)


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
        glyph_min_y = glyph_bounds[1]
        glyph_max_y = glyph_bounds[3]
    else:
        # Fallback: compute bounds from contour points
        pts = [(pt.x, pt.y) for contour in glyph.contours for pt in contour.points]
        glyph_min_x = min(p[0] for p in pts) if pts else 0.0
        glyph_min_y = min(p[1] for p in pts) if pts else descender
        glyph_max_y = max(p[1] for p in pts) if pts else ascender

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

    # Clip phantom anti-aliased pixels outside the glyph's y-bounding-box.
    # Anti-aliasing can bleed 1-2 pixels below glyph_min_y (e.g. T, l, h, n at
    # baseline) creating spurious CURVE sections in img2bez. We whiten any pixel
    # rows strictly below glyph_min_y or above glyph_max_y in font space.
    #
    # Coordinate mapping (drawbot page_y=0 is bottom, PNG row=0 is top):
    #   pixel_row = px_height - page_y
    #   page_y    = (font_y - descender) * scale
    # So:
    #   pixel_row = px_height - (font_y - descender) * scale
    #
    # Rows to blank below glyph: pixel_row > clip_bottom (i.e. font_y < glyph_min_y)
    # We add a 2px margin so we don't erase the legitimate glyph bottom edge.
    clip_margin_px = 2
    clip_bottom = int(px_height - (glyph_min_y - descender) * scale) + clip_margin_px
    clip_top = int(px_height - (glyph_max_y - descender) * scale) - clip_margin_px
    if clip_bottom < px_height or clip_top > 0:
        _clip_png_rows(output_png, clip_top, clip_bottom, px_height, px_width)

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
