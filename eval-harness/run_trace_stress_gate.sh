#!/usr/bin/env bash
# Trace composite reference images and fail obvious point explosions.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Create/activate the repo-local Python venv for drawbot-skia, ufoLib2, Pillow.
# shellcheck disable=SC1091
source "$SCRIPT_DIR/ensure_venv.sh"

python3 "$SCRIPT_DIR/trace_stress_gate.py" "$@"
