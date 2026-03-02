#!/bin/bash
#
# Quick test script for tracing a glyph image into test.ufo
#
# Usage:
#   ./test.sh <image> <glyph-name> <unicode-hex>
#
# Example:
#   ./test.sh ~/Desktop/a-U+0061.png a 0061
#
# Flags:
#
#   --target-height    Image height in font units. Should equal the font's
#                      full vertical range: ascender minus descender.
#
#   --y-offset         Vertical shift applied after scaling. Typically the
#                      descender value (negative) so the image bottom aligns
#                      with the descender line and the baseline lands at y=0.
#
#   --grid             Coordinate grid size. All output points are snapped to
#                      multiples of this value. 0 = no snapping.
#
#   --alphamax         Corner detection threshold. Range is 0.0 to ~1.34.
#                      Each polygon vertex gets a smoothness score (alpha):
#                        0.0  = sharp right-angle corner
#                        ~1.0 = moderately smooth
#                        ~1.34 = perfectly smooth (collinear)
#                      Vertices with alpha >= alphamax become corner points.
#                      Lower = more corners detected (aggressive).
#                      Higher = fewer corners, smoother curves.
#
# Output:
#   test.ufo/glyphs/<name>.glif   The traced glyph outline
#   <name>_comparison.png         3-panel comparison (source | traced | overlay)
#
# Debug (set env vars before running):
#   IMG2BEZ_DEBUG_SPLITS=1      Show split point analysis and line/curve decisions
#   IMG2BEZ_DEBUG_PIXELDIFF=1   Generate pixel-level diff image
#   IMG2BEZ_DEBUG_NO_CLEANUP=1  Skip cleanup pipeline (raw curve output)

set -e

if [ "$1" = "-h" ] || [ "$1" = "--help" ]; then
  sed -n '2,/^[^#]/{ s/^# //p; s/^#$//p; }' "$0"
  exit 0
fi

IMAGE="${1:?Usage: ./test.sh <image> <glyph-name> <unicode-hex>}"
NAME="${2:?Usage: ./test.sh <image> <glyph-name> <unicode-hex>}"
UNICODE="${3:?Usage: ./test.sh <image> <glyph-name> <unicode-hex>}"

cargo run --release -- \
  -i "$IMAGE" \
  -o test.ufo \
  -n "$NAME" \
  -u "$UNICODE" \
  --target-height 1088 \
  --y-offset=-256 \
  --grid 2 \
  --alphamax 1.0
