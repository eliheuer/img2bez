#!/usr/bin/env bash
# Focused structural tracing loop.
#
# This wraps the existing raster/reference experiment but limits it to the
# glyphs we want to inspect together, then writes structural metrics and SVG
# overlays comparing traced point structure against the reference UFO.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
GLYPH_FILE="${GLYPH_FILE:-$SCRIPT_DIR/focused_glyphs.txt}"
REFERENCE_UFO="${REFERENCE_UFO:-$SCRIPT_DIR/reference.ufo}"

if [ ! -f "$GLYPH_FILE" ]; then
    echo "ERROR: glyph file not found: $GLYPH_FILE"
    exit 1
fi

glyphs="$(grep -v '^[[:space:]]*#' "$GLYPH_FILE" | tr '\n' ' ' | xargs)"
if [ -z "$glyphs" ]; then
    echo "ERROR: glyph file is empty: $GLYPH_FILE"
    exit 1
fi

echo "Focused glyphs: $glyphs"
echo "Reference UFO: $REFERENCE_UFO"
echo ""

cd "$REPO_DIR"
if ! GLYPHS_FILTER="$glyphs" REFERENCE_UFO="$REFERENCE_UFO" "$SCRIPT_DIR/run_experiment.sh" "$@"; then
    if [ ! -d "$SCRIPT_DIR/work/output.ufo" ]; then
        echo "ERROR: focused experiment failed before writing output.ufo"
        exit 1
    fi
    echo "WARN: focused experiment returned nonzero after writing output.ufo; continuing with structural report"
fi

python3 "$SCRIPT_DIR/structural_report.py" \
    --reference-ufo "$REFERENCE_UFO" \
    --traced-ufo "$SCRIPT_DIR/work/output.ufo" \
    --glyph-file "$GLYPH_FILE" \
    --output-dir "$SCRIPT_DIR/work"

if [ -n "${RUN_LABEL:-}" ]; then
    archive_dir="$SCRIPT_DIR/runs/$RUN_LABEL"
    mkdir -p "$archive_dir"
    cp "$SCRIPT_DIR/work/structural_report.tsv" "$archive_dir/"
    cp "$SCRIPT_DIR/work/structural_summary.txt" "$archive_dir/"
    cp "$SCRIPT_DIR/work"/*.log "$archive_dir/" 2>/dev/null || true
    cp "$SCRIPT_DIR/work"/structure_*.svg "$archive_dir/" 2>/dev/null || true
    printf '%s\n' "$*" > "$archive_dir/args.txt"
    echo "Archived structural run: $archive_dir"
fi

echo ""
echo "Structural report: $SCRIPT_DIR/work/structural_report.tsv"
echo "Structural summary: $SCRIPT_DIR/work/structural_summary.txt"
echo "Overlays: $SCRIPT_DIR/work/structure_<glyph>.svg"
