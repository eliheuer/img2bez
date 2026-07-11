#!/usr/bin/env bash
# Inspect one glyph through the tracing pipeline.
#
# Writes a small, persistent artifact folder with the rendered PNG, split debug
# log, traced UFO, structural report, and SVG overlay. Use this when a focused
# loop identifies a weak glyph and we need to step through one pipeline decision
# at a time.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
# Create/activate the repo-local Python venv for drawbot-skia + ufoLib2.
# shellcheck disable=SC1091
source "$SCRIPT_DIR/ensure_venv.sh"
REFERENCE_UFO="${REFERENCE_UFO:-$SCRIPT_DIR/reference.ufo}"
GLYPH="${1:?Usage: inspect_glyph.sh <glyph-name> [label] [img2bez flags...]}"
LABEL="${2:-$GLYPH-default}"
shift || true
shift || true

OUT_DIR="$SCRIPT_DIR/inspect/$LABEL"
PNG_PATH="$OUT_DIR/$GLYPH.png"
OUTPUT_UFO="$OUT_DIR/output.ufo"

mkdir -p "$OUT_DIR"

cd "$REPO_DIR"
cargo build --release

if ! render_out=$(python3 "$SCRIPT_DIR/render_glyph.py" \
    "$REFERENCE_UFO" "$GLYPH" "$PNG_PATH" \
    --height "${RENDER_HEIGHT:-700}" 2>&1); then
    echo "ERROR: render failed"
    echo "$render_out"
    exit 1
fi

target_height=$(echo "$render_out" | grep "^target_height=" | cut -d= -f2)
y_offset=$(echo "$render_out" | grep "^y_offset=" | cut -d= -f2)
advance_width=$(echo "$render_out" | grep "^advance_width=" | cut -d= -f2)

if [ -z "$target_height" ] || [ -z "$y_offset" ]; then
    echo "ERROR: render produced no target metrics"
    echo "$render_out"
    exit 1
fi

rm -rf "$OUTPUT_UFO"
cp -r "$REFERENCE_UFO" "$OUTPUT_UFO"
rm -f "$OUTPUT_UFO/glyphs"/*.glif
python3 - "$OUTPUT_UFO" <<'PY'
import os
import plistlib
import sys

ufo = sys.argv[1]
path = os.path.join(ufo, "glyphs", "contents.plist")
with open(path, "wb") as f:
    plistlib.dump({}, f)
PY

ref_glif=$(python3 - "$REFERENCE_UFO" "$GLYPH" <<'PY'
import os
import plistlib
import sys

ufo, glyph = sys.argv[1], sys.argv[2]
for glyphs_dir in ("glyphs", "glyphs.public.default"):
    contents_path = os.path.join(ufo, glyphs_dir, "contents.plist")
    if not os.path.exists(contents_path):
        continue
    with open(contents_path, "rb") as f:
        contents = plistlib.load(f)
    filename = contents.get(glyph)
    if filename:
        print(os.path.join(ufo, glyphs_dir, filename))
        break
PY
)

if [ -z "$ref_glif" ]; then
    echo "ERROR: reference glif not found for $GLYPH"
    exit 1
fi

IMG2BEZ_DEBUG_SPLITS=1 "$REPO_DIR/target/release/img2bez" \
    --input "$PNG_PATH" \
    --output "$OUTPUT_UFO" \
    --name "$GLYPH" \
    --width "$advance_width" \
    --target-height "$target_height" \
    --y-offset "$y_offset" \
    --reference "$ref_glif" \
    "$@" \
    > "$OUT_DIR/split_debug.log" 2>&1

python3 "$SCRIPT_DIR/structural_report.py" \
    --reference-ufo "$REFERENCE_UFO" \
    --traced-ufo "$OUTPUT_UFO" \
    --glyphs "$GLYPH" \
    --output-dir "$OUT_DIR"

printf '%s\n' "$*" > "$OUT_DIR/args.txt"
printf '%s\n' "$render_out" > "$OUT_DIR/render_metrics.txt"

echo ""
echo "Inspection written to: $OUT_DIR"
echo "Split debug: $OUT_DIR/split_debug.log"
echo "Overlay: $OUT_DIR/structure_$GLYPH.svg"
