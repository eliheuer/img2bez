#!/usr/bin/env bash
# Wild-source trace scorer, run inside the eval-harness venv.
#
# Scores a trace of an unknown-source raster image (no ground-truth UFO) on
# reproduction fidelity + reference-free structural regularizers. See
# score_wild.py for the metric definitions.
#
# Usage:
#   ./score_wild.sh sources/wild/a.png --glyph a
#   ./score_wild.sh --dir sources/wild
#   ./score_wild.sh sources/wild/a.png --glyph a -- --accuracy 4.0   # tune img2bez

set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Activate (creating if needed) the repo-local venv before running anything.
# shellcheck disable=SC1091
source "$SCRIPT_DIR/ensure_venv.sh"

exec python "$SCRIPT_DIR/score_wild.py" "$@"
