#!/usr/bin/env bash
# Render a 2048x2048 img2bez eval-harness specimen from the repo root.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

TEXT="a"
WORK_DIR="eval-harness/stress-work"
PROFILE="clean"
SOURCE_IMAGE=""
REFERENCE_UFO=""
RENDER_HEIGHT=""
OPEN_RESULT=1
STRESS_ARGS=()
TRACE_ARGS=()

usage() {
    cat <<'EOF'
Usage:
  eval-harness/render-specimen.sh [TEXT]
  eval-harness/render-specimen.sh --text "aRE"
  eval-harness/render-specimen.sh --chars "ش"
  eval-harness/render-specimen.sh --text "aRE" -- --accuracy 4 --alphamax 0.8

Options:
  -t, --text TEXT          Characters to render and trace. Default: a
  -c, --chars TEXT         Alias for --text.
  -w, --work-dir DIR       Output work directory. Default: eval-harness/stress-work
  -p, --profile PROFILE    Generated source profile: clean, screen-gray,
                           lowres2x, lowres4x, soft-gray. Default: clean
  -i, --source-image PNG   Trace an existing source image instead of rendering TEXT.
  -r, --reference-ufo UFO  Reference UFO. Default: eval-harness/reference.ufo
  --render-height PX       Glyph render height before composing the source image.
  --no-build               Do not rebuild target/release/img2bez.
  --open                   Open the specimen image after rendering. Default.
  --no-open                Do not open the specimen image after rendering.
  -h, --help               Show this help.

Everything after -- is passed to img2bez:
  eval-harness/render-specimen.sh --text "&" -- --grid 2 --accuracy 3 --alphamax 0.8

Outputs:
  Prints and opens the generated specimen PNG, for example:
  eval-harness/stress-work/specimen_stress_aRE.png
EOF
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        -t|--text|-c|--chars)
            TEXT="${2:?missing value for $1}"
            shift 2
            ;;
        -w|--work-dir)
            WORK_DIR="${2:?missing value for $1}"
            shift 2
            ;;
        -p|--profile)
            PROFILE="${2:?missing value for $1}"
            shift 2
            ;;
        -i|--source-image)
            SOURCE_IMAGE="${2:?missing value for $1}"
            shift 2
            ;;
        -r|--reference-ufo)
            REFERENCE_UFO="${2:?missing value for $1}"
            shift 2
            ;;
        --render-height)
            RENDER_HEIGHT="${2:?missing value for $1}"
            shift 2
            ;;
        --no-build)
            STRESS_ARGS+=(--no-build)
            shift
            ;;
        --open)
            OPEN_RESULT=1
            shift
            ;;
        --no-open)
            OPEN_RESULT=0
            shift
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        --)
            shift
            TRACE_ARGS=("$@")
            break
            ;;
        -*)
            echo "error: unknown option: $1" >&2
            echo >&2
            usage >&2
            exit 2
            ;;
        *)
            TEXT="$1"
            shift
            ;;
    esac
done

if [[ -z "$TEXT" ]]; then
    echo "error: text must not be empty" >&2
    exit 2
fi

cd "$REPO_ROOT"

CMD=(
    "$SCRIPT_DIR/run_trace_stress_gate.sh"
    --work-dir "$WORK_DIR"
    --text "$TEXT"
    --profile "$PROFILE"
)

if [[ -n "$SOURCE_IMAGE" ]]; then
    CMD+=(--source-image "$SOURCE_IMAGE")
fi
if [[ -n "$REFERENCE_UFO" ]]; then
    CMD+=(--reference-ufo "$REFERENCE_UFO")
fi
if [[ -n "$RENDER_HEIGHT" ]]; then
    CMD+=(--render-height "$RENDER_HEIGHT")
fi
if [[ ${#STRESS_ARGS[@]} -gt 0 ]]; then
    CMD+=("${STRESS_ARGS[@]}")
fi
if [[ ${#TRACE_ARGS[@]} -gt 0 ]]; then
    CMD+=(-- "${TRACE_ARGS[@]}")
fi

LOG_FILE="$(mktemp "${TMPDIR:-/tmp}/img2bez-specimen.XXXXXX")"
trap 'rm -f "$LOG_FILE"' EXIT

set +e
"${CMD[@]}" 2>&1 | tee "$LOG_FILE"
STATUS=${PIPESTATUS[0]}
set -e

SPECIMEN_IMAGE="$(awk -F': ' '/^specimen_image:/ {print $2}' "$LOG_FILE" | tail -n 1)"

if [[ -n "$SPECIMEN_IMAGE" ]]; then
    echo
    echo "Specimen image: $SPECIMEN_IMAGE"
    if [[ "$OPEN_RESULT" -eq 1 ]]; then
        if command -v open >/dev/null 2>&1; then
            open "$SPECIMEN_IMAGE"
        elif command -v xdg-open >/dev/null 2>&1; then
            xdg-open "$SPECIMEN_IMAGE" >/dev/null 2>&1 &
        else
            echo "No opener found; open the PNG manually."
        fi
    fi
fi

exit "$STATUS"
