#!/usr/bin/env bash
# setup.sh — One-time setup for img2bez autoresearch
#
# Run this once before starting an overnight session.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

# Activate Python venv
# shellcheck disable=SC1090
source ~/Py/venvs/basic-fonts/bin/activate

echo "=== img2bez autoresearch setup ==="
echo ""

# ── Check Python dependencies ─────────────────────────────────────────────────

echo "Checking Python dependencies..."
python3 -c "import ufoLib2" 2>/dev/null && echo "  ufoLib2       OK" || echo "  ufoLib2       MISSING — pip install ufoLib2"
python3 -c "import drawbot_skia" 2>/dev/null && echo "  drawbot-skia  OK" || echo "  drawbot-skia  MISSING — pip install drawbot-skia"
echo ""

# ── Reference UFO ─────────────────────────────────────────────────────────────

if [ ! -d "$SCRIPT_DIR/reference.ufo" ]; then
    echo "Reference UFO not found at autoresearch/reference.ufo"
    echo ""
    echo "You need a UFO with hand-drawn reference glyphs. Options:"
    echo ""
    echo "  1. Symlink your own UFO:"
    echo "       ln -s /path/to/your/handdrawn.ufo $SCRIPT_DIR/reference.ufo"
    echo ""
    echo "  2. Use the existing test.ufo (traced glyphs, not hand-drawn):"
    echo "       ln -s $REPO_DIR/test.ufo $SCRIPT_DIR/reference.ufo"
    echo "       # Note: test.ufo glyphs were produced by img2bez, so baseline"
    echo "       # IoU will already be high. Better for testing the harness."
    echo ""
    echo "  3. Use a public open-source font (e.g. Recursive, Source Serif, etc.)"
    echo ""
    read -p "Symlink test.ufo as a starting point? [y/N] " yn
    if [[ "$yn" =~ ^[Yy]$ ]]; then
        ln -s "$REPO_DIR/test.ufo" "$SCRIPT_DIR/reference.ufo"
        echo "  Linked test.ufo → autoresearch/reference.ufo"
    else
        echo "  Skipped. Set up reference.ufo before running experiments."
    fi
    echo ""
fi

# ── Initial build ─────────────────────────────────────────────────────────────

echo "Building img2bez (release)..."
cd "$REPO_DIR"
cargo build --release
echo "  Build OK"
echo ""

# ── Smoke test ────────────────────────────────────────────────────────────────

if [ -d "$SCRIPT_DIR/reference.ufo" ]; then
    echo "Running smoke test on first available glyph..."
    first_glyph=$(python3 - "$SCRIPT_DIR/reference.ufo" <<'EOF'
import plistlib, os, sys
cp = os.path.join(sys.argv[1], "glyphs", "contents.plist")
with open(cp, "rb") as f:
    contents = plistlib.load(f)
print(sorted(contents.keys())[0])
EOF
    )
    echo "  Testing glyph: $first_glyph"
    mkdir -p "$SCRIPT_DIR/work"
    python3 "$SCRIPT_DIR/render_glyph.py" \
        "$SCRIPT_DIR/reference.ufo" "$first_glyph" \
        "$SCRIPT_DIR/work/smoke_test.png" \
        --height 300 2>&1
    echo "  Render OK → autoresearch/work/smoke_test.png"
    echo ""
fi

echo "=== Setup complete ==="
echo ""
echo "Next steps:"
echo "  1. Start a new git branch:  git checkout -b autoresearch/$(date +%b%d | tr A-Z a-z)"
echo "  2. Run baseline:            ./autoresearch/run_experiment.sh > run.log 2>&1"
echo "  3. Launch agent:            Tell Claude to follow autoresearch/program.md"
echo ""
