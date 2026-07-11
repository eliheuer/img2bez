#!/usr/bin/env python3
"""Degrade a clean glyph render into a realistic "wild" input image.

The eval harness renders a known glyph to a clean bitmap and traces it back,
scoring against the known outline. This script sits in between: it pushes a
clean render through controlled degradations (downscale, blur, JPEG, noise,
bilevel, dither, nearest-neighbor upscale), producing a degraded image whose
*correct* outline is still known — the reference glyph it came from.

That gives (degraded image -> known-correct outline) pairs across the space of
input quality, which is what lets a settings-selector be tuned and validated
(see the "input adaptation" plan). Degradations are applied in a realistic
capture order; each is off unless its flag is given.

Usage:
    python degrade.py in.png out.png [--blur R] [--noise S] [--jpeg Q] \
        [--downscale EXTENT] [--bilevel[=T] | --dither] [--nn-upscale F]

Example (a blurry, noisy, JPEG'd, low-res capture):
    python degrade.py a-clean.png a-wild.png --downscale 96 --blur 1.2 \
        --noise 10 --jpeg 45
"""
import argparse
import io
import sys

from PIL import Image, ImageChops, ImageFilter


def downscale(img, extent):
    """Resize so the larger side is `extent` px (Lanczos), keeping aspect."""
    w, h = img.size
    s = extent / max(w, h)
    if s >= 1.0:
        return img
    return img.resize(
        (max(1, round(w * s)), max(1, round(h * s))), Image.LANCZOS
    )


def blur(img, radius):
    """Optical blur / out-of-focus."""
    return img.filter(ImageFilter.GaussianBlur(radius))


def add_noise(img, sigma):
    """Additive Gaussian sensor noise (PIL-native, no numpy)."""
    g = img.convert("L")
    # effect_noise is zero-mean gaussian in [0,255] centered at 128.
    n = Image.effect_noise(g.size, sigma)
    n = n.point(lambda p: p - 128)
    return ImageChops.add(g, n, scale=1.0).convert(img.mode)


def contrast(img, lo, hi):
    """Compress the tonal range into [lo, hi] — a washed-out, low-contrast scan
    (dark-gray ink on light-gray paper) rather than pure black on white. This is
    what drops `bilevelness` and marks the soft-photographic class (os-001)."""
    return img.convert("L").point(lambda p: round(lo + (hi - lo) * p / 255.0))


def jpeg(img, quality):
    """Round-trip through JPEG at `quality` to add blocking artifacts."""
    buf = io.BytesIO()
    img.convert("RGB").save(buf, format="JPEG", quality=quality)
    buf.seek(0)
    return Image.open(buf).convert(img.mode)


def bilevel(img, threshold):
    """Hard threshold to pure black/white — destroys all anti-aliasing."""
    return img.convert("L").point(lambda p: 0 if p < threshold else 255)


def dither(img):
    """Floyd-Steinberg 1-bit dither — a halftone-ish degradation."""
    return img.convert("1").convert("L")


def nn_upscale(img, factor):
    """Nearest-neighbor upscale — quantizes AA into uniform pixel blocks."""
    w, h = img.size
    return img.resize((w * factor, h * factor), Image.NEAREST)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("input")
    p.add_argument("output")
    p.add_argument("--downscale", type=int, metavar="EXTENT",
                   help="resize so the larger side is EXTENT px")
    p.add_argument("--blur", type=float, metavar="RADIUS",
                   help="Gaussian blur radius (px)")
    p.add_argument("--noise", type=float, metavar="SIGMA",
                   help="additive Gaussian noise sigma")
    p.add_argument("--contrast", type=str, metavar="LO:HI",
                   help="compress tones into [LO,HI] (e.g. 70:185) for a "
                        "washed-out low-contrast scan")
    p.add_argument("--jpeg", type=int, metavar="Q",
                   help="JPEG quality 1-95")
    p.add_argument("--bilevel", nargs="?", type=int, const=128, default=None,
                   metavar="T", help="hard threshold at T (default 128)")
    p.add_argument("--dither", action="store_true",
                   help="Floyd-Steinberg 1-bit dither")
    p.add_argument("--nn-upscale", type=int, metavar="F",
                   help="nearest-neighbor upscale by integer factor F")
    args = p.parse_args()

    img = Image.open(args.input).convert("L")

    # Realistic capture order: geometry -> optics -> sensor -> codec ->
    # quantization -> display upscale.
    if args.downscale:
        img = downscale(img, args.downscale)
    if args.blur:
        img = blur(img, args.blur)
    if args.contrast:
        lo, hi = (int(v) for v in args.contrast.split(":"))
        img = contrast(img, lo, hi)
    if args.noise:
        img = add_noise(img, args.noise)
    if args.jpeg:
        img = jpeg(img, args.jpeg)
    if args.bilevel is not None:
        img = bilevel(img, args.bilevel)
    if args.dither:
        img = dither(img)
    if args.nn_upscale:
        img = nn_upscale(img, args.nn_upscale)

    img.save(args.output)
    print(f"{args.input} -> {args.output}  {img.size[0]}x{img.size[1]}",
          file=sys.stderr)


if __name__ == "__main__":
    main()
