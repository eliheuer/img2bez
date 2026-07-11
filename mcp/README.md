# img2bez-mcp

An [MCP](https://modelcontextprotocol.io) server that exposes
[img2bez](https://github.com/eliheuer/img2bez) glyph tracing as a tool an AI
agent can call: point it at a bitmap glyph image and get back a font-ready
bezier outline.

It speaks JSON-RPC 2.0 over stdio (the MCP stdio transport) and depends only on
the img2bez tracer — no UFO/render/CLI deps.

## Tool

### `trace_glyph`

Trace a PNG/JPEG/BMP glyph image into a bezier outline.

| Argument   | Required | Description                                                   |
|------------|----------|---------------------------------------------------------------|
| `path`     | yes      | Path to the glyph image file                                  |
| `glyph`    | no       | Glyph name (default `a`)                                       |
| `unicode`  | no       | Hex codepoint, e.g. `0061`                                     |
| `profile`  | no       | `wild` (default), `clean` (font render), or `photo` (soft scan)      |
| `accuracy` | no       | Curve-fit accuracy in font units (smaller = more points)       |
| `grid`     | no       | Coordinate snap grid (default `2`; `0` = off)                  |
| `chamfer`  | no       | Automatic line-corner chamfer size (default `0`)               |
| `chamferMinEdge` | no | Minimum edge length eligible for chamfering                    |
| `refine`   | no       | Enable raster-loss refinement (default `true`)                 |
| `width`    | no       | Explicit advance width                                         |
| `targetHeight` | no   | Ascender - descender in font units (default `1088`)            |
| `yOffset`  | no       | Vertical placement offset, usually descender (default `-256`)  |
| `invert`   | no       | Invert the image before tracing                                |
| `threshold` | no      | Fixed brightness threshold 0-255; omitted uses Otsu            |
| `format`   | no       | `json` (default), `svg`, or `glif`                            |

The `json` format returns the canonical outline:
`{ name, unicodes, advance: { width }, unitsPerEm, outline: { contours: [{ points: [{ x, y, type?, smooth? }] }] } }`.
Off-curve points omit `type`, matching UFO GLIF point semantics.

## Build

```sh
cargo build --release --manifest-path mcp/Cargo.toml
# binary at mcp/target/release/img2bez-mcp
```

## Register with a client

**Claude Code:**

```sh
claude mcp add img2bez -- /absolute/path/to/img2bez-mcp
```

**Claude Desktop** (`claude_desktop_config.json`):

```json
{
  "mcpServers": {
    "img2bez": {
      "command": "/absolute/path/to/img2bez-mcp"
    }
  }
}
```

Then ask the agent to trace an image, e.g. *"trace ~/Desktop/Q.png as glyph Q
and show me the outline."*

## License

Apache-2.0 OR MIT
