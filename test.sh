#!/bin/bash

set -e

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
  --accuracy 2.0 \
  --corner-threshold 30
