# img2bez (WebAssembly)

Trace bitmap glyph images into font-ready bezier outlines, entirely in the
browser. WebAssembly bindings for [img2bez](https://github.com/eliheuer/img2bez).

Give it the bytes of a PNG/JPEG/BMP glyph image; get back the outline as a
structured JSON object, an SVG path, or UFO GLIF XML.

## Usage

```js
import init, { traceToJson, traceToSvg, traceToGlif } from "img2bez-wasm";

await init(); // load the .wasm

const bytes = new Uint8Array(await file.arrayBuffer());
const config = JSON.stringify({ glyph: "a", unicode: "0061" });

// Structured outline — the easiest to consume in an editor:
const glyph = JSON.parse(traceToJson(bytes, config));
// {
//   name: "a",
//   unicodes: ["0061"],
//   advance: { width: 640 },
//   unitsPerEm: 1024,
//   outline: {
//     contours: [
//       { points: [ { x, y, type: "curve"|"line"|"qcurve", smooth? }, … ] },
//       …
//     ]
//   }
// }

// Or an SVG path string (y-up; flip for a y-down viewport):
const d = traceToSvg(bytes, "{}");

// Or UFO GLIF XML:
const glif = traceToGlif(bytes, config);
```

### Coordinates

Outlines are **y-up** (font convention), placed into a font using the metrics
in `config` (`targetHeight` / `yOffset`). `contours` is the canonical img2bez
point model: off-curve control points precede the on-curve point they lead
into, off-curve points omit `type`, and the first point of each contour carries
the closing segment's type — i.e. the UFO `.glif` point order, so it maps
losslessly to UFO/GLIF.

## Config

All fields optional; defaults mirror the img2bez CLI's `wild` profile.

| Field          | Default  | Meaning                                            |
|----------------|----------|----------------------------------------------------|
| `glyph`        | `"a"`    | Glyph name                                         |
| `unicode`      | —        | Hex codepoint, e.g. `"0061"`                        |
| `profile`      | `"wild"` | `"wild"` (unknown raster) or `"clean"` (font render) |
| `accuracy`     | profile  | Curve-fit accuracy in font units (smaller = more points) |
| `grid`         | `2`      | Coordinate snap grid (0 = off)                     |
| `chamfer`      | `0`      | Automatic line-corner chamfer size in font units   |
| `chamferMinEdge` | `40`   | Minimum edge length eligible for chamfering         |
| `refine`       | `true`   | Enable raster-loss refinement                       |
| `targetHeight` | `1088`   | Em the tracer normalizes to (ascender − descender) |
| `yOffset`      | `-256`   | Descender (baseline placement)                     |
| `width`        | auto     | Advance width (auto from bounds + sidebearings)    |
| `invert`       | `false`  | Swap foreground/background                          |
| `threshold`    | Otsu     | Fixed brightness threshold 0–255                   |

## Build & publish

```sh
wasm-pack build wasm --target web --release   # → wasm/pkg
npm publish wasm/pkg                           # or: npm publish --access public wasm/pkg
```

TypeScript types (`.d.ts`) are generated automatically by wasm-bindgen.

## License

Apache-2.0 OR MIT
