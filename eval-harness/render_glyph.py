#!/usr/bin/env python3
"""Render a UFO glyph to a PNG.

Thin Python shim over the Rust `render-glyph` tool in ``render-glyph/`` (norad +
tiny-skia). It replaced the old drawbot-skia renderer: same output space, same
metrics, no Python image dependencies. The Rust binary is built on first use.

The output image spans the full ascender-to-descender height of the font, with
the glyph drawn black on white. This matches an img2bez trace invoked with:

    --target-height <ascender - descender>
    --y-offset <descender>

Usage:
    python render_glyph.py <ufo_path> <glyph_name> <output_png> [--height 700]

Prints to stdout (for shell script capture):
    target_height=<float>
    y_offset=<float>
    advance_width=<float>
"""

from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path

_HERE = Path(__file__).resolve().parent
_CRATE = _HERE / "render-glyph"
_MANIFEST = _CRATE / "Cargo.toml"
_BINARY = _CRATE / "target" / "release" / "render-glyph"

_built = False


def _ensure_built() -> Path:
    """Build the Rust tool once per process; return the binary path.

    Cargo output goes to stderr so it never corrupts the key=value stdout that
    callers parse. A fresh `cargo build -q` is a fast no-op.
    """
    global _built
    if not _built:
        try:
            subprocess.run(
                ["cargo", "build", "--release", "-q", "--manifest-path", str(_MANIFEST)],
                check=True,
                stdout=sys.stderr,
            )
        except FileNotFoundError:
            sys.exit("Error: cargo not found. Install Rust to build render-glyph.")
        except subprocess.CalledProcessError as exc:
            sys.exit(f"Error: failed to build render-glyph (exit {exc.returncode}).")
        _built = True
    return _BINARY


def render_glyph(ufo_path, glyph_name, output_png, px_height=700):
    """Render a single glyph to PNG and return its metrics dict."""
    binary = _ensure_built()
    proc = subprocess.run(
        [str(binary), str(ufo_path), str(glyph_name), str(output_png),
         "--height", str(px_height)],
        check=True,
        capture_output=True,
        text=True,
    )

    values = {}
    for line in proc.stdout.splitlines():
        if "=" in line:
            key, val = line.split("=", 1)
            values[key.strip()] = val.strip()

    return {
        "target_height": float(values["target_height"]),
        "y_offset": float(values["y_offset"]),
        "advance_width": float(values["advance_width"]),
        "px_height": int(float(values.get("px_height", px_height))),
        "px_width": int(float(values["px_width"])),
        "x_shift_font": float(values.get("x_shift_font", 0.0)),
    }


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Render a UFO glyph to PNG")
    parser.add_argument("ufo_path", help="Path to the UFO font")
    parser.add_argument("glyph_name", help="Glyph name (e.g. 'A')")
    parser.add_argument("output_png", help="Output PNG file path")
    parser.add_argument("--height", type=int, default=700, help="Output image height in pixels")
    args = parser.parse_args()

    result = render_glyph(args.ufo_path, args.glyph_name, args.output_png, args.height)

    # key=value pairs for shell scripts to capture.
    print(f"target_height={result['target_height']}")
    print(f"y_offset={result['y_offset']}")
    print(f"advance_width={result['advance_width']}")
