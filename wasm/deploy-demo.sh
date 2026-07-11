#!/bin/sh
# Rebuild the wasm bindings and deploy them into the elih.net blog demo.
# The Astro dev server (localhost:1234) picks the new module up on reload.
set -e
cd "$(dirname "$0")/.."
wasm-pack build wasm --target web --release
cp wasm/pkg/img2bez_wasm.js \
   wasm/pkg/img2bez_wasm.d.ts \
   wasm/pkg/img2bez_wasm_bg.wasm \
   "$HOME/GH/repos/elih.net/src/lib/img2bez-wasm/"
echo "deployed: reload http://localhost:1234/blog/img2bez (hard-refresh if stale)"
