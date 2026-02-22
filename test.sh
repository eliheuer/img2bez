#!/bin/bash
# Quick test: trace an image into the scratch UFO
#
# Usage:
#   ./test.sh <image> <glyph-name> <unicode-hex>
#
# Examples:
#   ./test.sh /tmp/test_A.png A 0041
#   ./test.sh /tmp/question.png question 003F

set -e

IMAGE="${1:?Usage: ./test.sh <image> <glyph-name> <unicode-hex>}"
NAME="${2:?Usage: ./test.sh <image> <glyph-name> <unicode-hex>}"
UNICODE="${3:?Usage: ./test.sh <image> <glyph-name> <unicode-hex>}"

cargo run -- \
    -i "$IMAGE" \
    -o test.ufo \
    -n "$NAME" \
    -u "$UNICODE" \
    --target-height 1088 \
    --y-offset=-256 \
    --grid 2 \
    --accuracy 8.0 \
    --corner-threshold 30
