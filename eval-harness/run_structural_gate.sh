#!/usr/bin/env bash
# Run the focused structural loop on the current checkout, then gate the exact
# archived result against the baseline.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RUN_LABEL="${RUN_LABEL:-current-gate}"
BASELINE="${BASELINE:-$SCRIPT_DIR/baselines/current.tsv}"

RUN_LABEL="$RUN_LABEL" "$SCRIPT_DIR/run_structural_loop.sh" "$@"

python3 "$SCRIPT_DIR/check_structural_gate.py" "$RUN_LABEL" \
    --baseline "$BASELINE" \
    --min-mean-delta -0.001 \
    --min-improved 0 \
    --max-worsened 0 \
    --require-not-worse ampersand:0.002 \
    --require-not-worse s:0.002 \
    --require-not-worse n \
    --require-not-worse O \
    --max-oncurve-ratio n:1.20 \
    --allow-raster-improved-regression ampersand:0.030:1.0

"$SCRIPT_DIR/run_trace_stress_gate.sh" \
    --no-build \
    --profile lowres2x \
    --min-eval-score 0.70 \
    -- "$@"
