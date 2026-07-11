#!/usr/bin/env bash
# Create and activate the repo-local Python environment used by eval-harness.
#
# Source this file from eval-harness scripts:
#   source "$SCRIPT_DIR/ensure_venv.sh"
#
# The default venv lives at the repo root so the tooling is portable across
# machines and does not depend on a user's private ~/Py paths.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
VENV_DIR="${IMG2BEZ_EVAL_HARNESS_VENV:-$REPO_DIR/.venv-eval-harness}"
REQUIREMENTS="$SCRIPT_DIR/requirements.txt"
STAMP="$VENV_DIR/.requirements-installed"

if [ -n "${VIRTUAL_ENV:-}" ] && [ "$VIRTUAL_ENV" != "$VENV_DIR" ]; then
    echo "warning: replacing active Python venv '$VIRTUAL_ENV' with '$VENV_DIR'" >&2
fi

if [ ! -f "$VENV_DIR/bin/activate" ]; then
    echo "Creating eval-harness venv: $VENV_DIR" >&2
    python3 -m venv "$VENV_DIR"
fi

# shellcheck disable=SC1091
source "$VENV_DIR/bin/activate"

if [ -z "${IMG2BEZ_SKIP_EVAL_HARNESS_PIP_INSTALL:-}" ] \
    && { [ ! -f "$STAMP" ] || [ "$REQUIREMENTS" -nt "$STAMP" ]; }; then
    echo "Installing eval-harness Python dependencies..." >&2
    python3 -m pip install --upgrade pip
    python3 -m pip install -r "$REQUIREMENTS"
    touch "$STAMP"
fi
