#!/usr/bin/env bash
# run_experiment.sh — img2bez autoresearch experiment runner
#
# Renders each reference glyph to PNG using drawbot-skia, traces it with
# img2bez, and compares the result to the original .glif. Reports raster IoU
# and the weighted vector quality score.
#
# Usage:
#   ./run_experiment.sh [extra img2bez flags...]
#
# Example:
#   ./run_experiment.sh --alphamax 0.8 --accuracy 3.0
#
# Outputs (parseable by agent):
#   mean_iou: XX.XX%
#   mean_score: 0.XXX
#   glyphs_ok: N
#   glyphs_failed: N
#
# Exit codes:
#   0 = success (all glyphs evaluated)
#   1 = build failed or no glyphs evaluated

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

# Activate Python venv that has drawbot-skia + ufoLib2
# shellcheck disable=SC1090
source ~/Py/venvs/basic-fonts/bin/activate
BINARY="$REPO_DIR/target/release/img2bez"
RENDER="$SCRIPT_DIR/render_glyph.py"
WORK_DIR="$SCRIPT_DIR/work"

# ── Configuration ─────────────────────────────────────────────────────────────

# Reference UFO: hand-drawn glyphs used as ground truth.
# Override via env var: REFERENCE_UFO=/path/to/your.ufo ./run_experiment.sh
# Or update the reference.ufo symlink: ln -sf /path/to/your.ufo autoresearch/reference.ufo
REFERENCE_UFO="${REFERENCE_UFO:-$SCRIPT_DIR/reference.ufo}"

# Glyph filter: space-separated list of glyph names to test.
# Empty string = test all glyphs in the reference UFO.
# Example: GLYPHS_FILTER="A B C O S" ./run_experiment.sh  (focused run)
GLYPHS_FILTER="${GLYPHS_FILTER:-A B C D E F G H I J K L M N O P Q R S T U V W X Y Z a b c d e f g h i j k l m n o p q r s t u v w x y z}"

# Render height (pixels). Higher = more detail but slower tracing.
RENDER_HEIGHT="${RENDER_HEIGHT:-700}"

# Extra img2bez flags passed in from command line (or empty for defaults)
EXTRA_PARAMS=()
if [ "$#" -gt 0 ]; then
    EXTRA_PARAMS=("$@")
fi

# ── Validate prerequisites ─────────────────────────────────────────────────────

if [ ! -d "$REFERENCE_UFO" ]; then
    echo "ERROR: Reference UFO not found at $REFERENCE_UFO"
    echo "  Set REFERENCE_UFO=/path/to/your.ufo or symlink reference.ufo"
    exit 1
fi

# ── Build img2bez ─────────────────────────────────────────────────────────────

echo "Building img2bez..."
cd "$REPO_DIR"
if ! cargo build --release 2>&1; then
    echo "ERROR: Build failed"
    exit 1
fi
echo "Build OK"
echo ""

# ── Discover reference glyphs ─────────────────────────────────────────────────

# Read contents.plist to map glyph names → glif filenames
GLYPHS_DIR="$REFERENCE_UFO/glyphs"
CONTENTS="$GLYPHS_DIR/contents.plist"

if [ ! -f "$CONTENTS" ]; then
    echo "ERROR: No contents.plist in $GLYPHS_DIR"
    exit 1
fi

# Build the list of glyph names to test
GLYPH_NAMES=()
if [ -n "$GLYPHS_FILTER" ]; then
    # Use the filter list, keeping only glyphs that exist in the UFO
    for name in $GLYPHS_FILTER; do
        glif=$(python3 - "$CONTENTS" "$name" <<'EOF'
import plistlib, sys
with open(sys.argv[1], "rb") as f:
    contents = plistlib.load(f)
print("yes" if sys.argv[2] in contents else "no")
EOF
        )
        if [ "$glif" = "yes" ]; then
            GLYPH_NAMES+=("$name")
        fi
    done
else
    # No filter: use all glyphs from contents.plist
    while IFS= read -r line; do
        GLYPH_NAMES+=("$line")
    done < <(python3 - "$CONTENTS" <<'EOF'
import plistlib, sys
with open(sys.argv[1], "rb") as f:
    contents = plistlib.load(f)
for name in sorted(contents.keys()):
    print(name)
EOF
    )
fi

echo "Glyphs to evaluate: ${#GLYPH_NAMES[@]} (${GLYPH_NAMES[*]})"
echo ""

# ── Set up work directory ─────────────────────────────────────────────────────

rm -rf "$WORK_DIR"
mkdir -p "$WORK_DIR"

# Create a minimal output UFO (copy structure, strip glyphs — img2bez adds them)
OUTPUT_UFO="$WORK_DIR/output.ufo"
cp -r "$REFERENCE_UFO" "$OUTPUT_UFO"
# Remove existing glyphs so we start fresh each run
rm -f "$OUTPUT_UFO/glyphs"/*.glif
# Clear the contents.plist
python3 -c "
import plistlib, os
path = os.path.join('$OUTPUT_UFO', 'glyphs', 'contents.plist')
with open(path, 'wb') as f:
    plistlib.dump({}, f)
"

# ── Run per-glyph experiments ─────────────────────────────────────────────────

total_iou=0
total_score=0
count_ok=0
count_fail=0

for glyph_name in "${GLYPH_NAMES[@]}"; do
    # Resolve .glif path via contents.plist
    glif_path=$(python3 - "$REFERENCE_UFO" "$glyph_name" <<'EOF'
import plistlib, os, sys
ufo_path, name = sys.argv[1], sys.argv[2]
for glyphs_dir in ["glyphs", "glyphs.public.default"]:
    cp = os.path.join(ufo_path, glyphs_dir, "contents.plist")
    if not os.path.exists(cp):
        continue
    with open(cp, "rb") as f:
        contents = plistlib.load(f)
    if name in contents:
        print(os.path.join(ufo_path, glyphs_dir, contents[name]))
        break
EOF
    )

    if [ -z "$glif_path" ] || [ ! -f "$glif_path" ]; then
        echo "  SKIP $glyph_name (glif not found)"
        count_fail=$((count_fail + 1))
        continue
    fi

    # Use unicode codepoint as filename to avoid case collisions on macOS
    # (case-insensitive filesystem treats B.png and b.png as the same file)
    file_key=$(python3 -c "print(f'uni{ord(\"$glyph_name\"):04X}' if len('$glyph_name') == 1 else '$glyph_name')")
    png_path="$WORK_DIR/${file_key}.png"
    log_path="$WORK_DIR/${file_key}.log"

    # Render reference glyph to PNG
    render_out=$(python3 "$RENDER" \
        "$REFERENCE_UFO" "$glyph_name" "$png_path" \
        --height "$RENDER_HEIGHT" 2>&1) || {
        echo "  FAIL $glyph_name (render error: $render_out)"
        count_fail=$((count_fail + 1))
        continue
    }

    # Parse render metrics
    target_height=$(echo "$render_out" | grep "^target_height=" | cut -d= -f2)
    y_offset=$(echo "$render_out" | grep "^y_offset=" | cut -d= -f2)

    if [ -z "$target_height" ]; then
        echo "  FAIL $glyph_name (render produced no metrics)"
        count_fail=$((count_fail + 1))
        continue
    fi

    # Run img2bez: trace the rendered PNG and compare to original .glif
    "$BINARY" \
        --input "$png_path" \
        --output "$OUTPUT_UFO" \
        --name "$glyph_name" \
        --target-height "$target_height" \
        --y-offset "$y_offset" \
        --reference "$glif_path" \
        ${EXTRA_PARAMS[@]+"${EXTRA_PARAMS[@]}"} \
        > "$log_path" 2>&1 || true

    # Parse raster IoU (e.g. "  Raster IoU  87.3%  (...)")
    iou=$(grep "Raster IoU" "$log_path" \
        | grep -oE '[0-9]+\.[0-9]+' | head -1 || echo "")

    # Parse weighted overall vector score (e.g. "  Overall       0.847")
    score=$(grep "Overall" "$log_path" \
        | grep -oE '[0-9]+\.[0-9]+' | tail -1 || echo "")

    if [ -n "$iou" ] && [ -n "$score" ]; then
        printf "  %-4s  IoU=%6.2f%%  score=%.3f\n" "$glyph_name" "$iou" "$score"
        total_iou=$(python3 -c "print($total_iou + $iou)")
        total_score=$(python3 -c "print($total_score + $score)")
        count_ok=$((count_ok + 1))
    else
        echo "  FAIL $glyph_name (no metrics — see $log_path)"
        count_fail=$((count_fail + 1))
    fi
done

# ── Summary ───────────────────────────────────────────────────────────────────

echo ""
if [ "$count_ok" -gt 0 ]; then
    mean_iou=$(python3 -c "print(f'{$total_iou / $count_ok:.2f}')")
    mean_score=$(python3 -c "print(f'{$total_score / $count_ok:.3f}')")
    echo "mean_iou: ${mean_iou}%"
    echo "mean_score: ${mean_score}"
    echo "glyphs_ok: ${count_ok}"
    echo "glyphs_failed: ${count_fail}"
else
    echo "ERROR: No glyphs evaluated successfully"
    echo "mean_iou: 0.00%"
    echo "mean_score: 0.000"
    echo "glyphs_ok: 0"
    echo "glyphs_failed: ${count_fail}"
    exit 1
fi

# ── Bezier structure visualizations (bottom 8 glyphs by IoU) ──────────────────

VIZ_SCRIPT="$SCRIPT_DIR/viz_glyphs.rs"
if command -v designbot &>/dev/null && [ -f "$VIZ_SCRIPT" ] && [ -f "$OUTPUT_UFO/glyphs/contents.plist" ]; then
    # Collect per-glyph IoU from log files; sort ascending; take bottom 8
    weakest=$(
        for glyph_name in "${GLYPH_NAMES[@]}"; do
            file_key=$(python3 -c "print(f'uni{ord(\"$glyph_name\"):04X}' if len('$glyph_name') == 1 else '$glyph_name')")
            log="$WORK_DIR/${file_key}.log"
            iou=$(grep "Raster IoU" "$log" 2>/dev/null | grep -oE '[0-9]+\.[0-9]+' | head -1)
            [ -n "$iou" ] && echo "$iou $glyph_name"
        done | sort -n | head -8 | awk '{print $2}'
    )
    if [ -n "$weakest" ]; then
        echo ""
        echo "Rendering bezier viz for weakest glyphs: $(echo $weakest | tr '\n' ' ')"
        # shellcheck disable=SC2086
        designbot --render "$VIZ_SCRIPT" --output /dev/null \
            -- "$OUTPUT_UFO" "$REFERENCE_UFO" "$WORK_DIR" $weakest 2>/dev/null \
            && echo "Viz saved to $WORK_DIR/viz_uni*.png"
    fi
fi
