// Copyright 2026 the img2bez Authors
// SPDX-License-Identifier: Apache-2.0 OR MIT

//! img2bez MCP server.
//!
//! Exposes glyph tracing as Model Context Protocol tools an AI agent can call.
//! Speaks JSON-RPC 2.0 over stdio with newline-delimited messages (the MCP
//! stdio transport): one JSON object per line on stdin, one response per line
//! on stdout. Diagnostics go to stderr so they never corrupt the protocol.

use std::io::{BufRead, Write};

use serde_json::{Value, json};

/// The MCP protocol version this server implements.
const PROTOCOL_VERSION: &str = "2024-11-05";

fn main() {
    let stdin = std::io::stdin();
    let stdout = std::io::stdout();
    let mut out = stdout.lock();

    for line in stdin.lock().lines() {
        let Ok(line) = line else { break };
        let line = line.trim();
        if line.is_empty() {
            continue;
        }
        let Ok(msg) = serde_json::from_str::<Value>(line) else {
            eprintln!("img2bez-mcp: ignoring non-JSON line");
            continue;
        };
        // Notifications (and any message without an id) get no reply.
        let Some(id) = msg.get("id").cloned() else {
            continue;
        };
        let method = msg.get("method").and_then(Value::as_str).unwrap_or("");
        let params = msg.get("params").cloned().unwrap_or(Value::Null);

        let response = match handle(method, &params) {
            Ok(result) => json!({"jsonrpc": "2.0", "id": id, "result": result}),
            Err((code, message)) => json!({
                "jsonrpc": "2.0",
                "id": id,
                "error": {"code": code, "message": message},
            }),
        };
        if writeln!(out, "{response}").is_err() {
            break;
        }
        let _ = out.flush();
    }
}

/// Dispatch a request method to its result, or a JSON-RPC `(code, message)`.
fn handle(method: &str, params: &Value) -> Result<Value, (i64, String)> {
    match method {
        "initialize" => Ok(json!({
            "protocolVersion": PROTOCOL_VERSION,
            "capabilities": {"tools": {}},
            "serverInfo": {"name": "img2bez", "version": env!("CARGO_PKG_VERSION")},
        })),
        "ping" => Ok(json!({})),
        "tools/list" => Ok(json!({"tools": [trace_tool_schema()]})),
        "tools/call" => call_tool(params),
        other => Err((-32601, format!("method not found: {other}"))),
    }
}

/// The `trace_glyph` tool definition advertised to the agent.
fn trace_tool_schema() -> Value {
    json!({
        "name": "trace_glyph",
        "description": "Trace a bitmap glyph image (PNG/JPEG/BMP) into a \
            font-ready bezier outline. Returns the outline as JSON (the \
            canonical point model), an SVG path, or UFO GLIF XML.",
        "inputSchema": {
            "type": "object",
            "properties": {
                "path": {
                    "type": "string",
                    "description": "Path to the glyph image file (PNG/JPEG/BMP)."
                },
                "glyph": {
                    "type": "string",
                    "description": "Glyph name (default \"a\")."
                },
                "unicode": {
                    "type": "string",
                    "description": "Hex codepoint, e.g. \"0061\" for 'a'."
                },
                "profile": {
                    "type": "string",
                    "enum": ["wild", "clean", "photo"],
                    "description": "Source class: \"wild\" for an unknown raster \
                        (default), \"clean\" for a high-res font render, \
                        \"photo\" for a soft scan of printed type."
                },
                "accuracy": {
                    "type": "number",
                    "description": "Curve-fit accuracy in font units (smaller = \
                        more points). Overrides the profile default."
                },
                "grid": {
                    "type": "integer",
                    "description": "Coordinate snap grid (default 2; 0 = off)."
                },
                "chamfer": {
                    "type": "number",
                    "description": "Automatic line-corner chamfer size in font units (default 0)."
                },
                "chamferMinEdge": {
                    "type": "number",
                    "description": "Minimum edge length eligible for chamfering."
                },
                "refine": {
                    "type": "boolean",
                    "description": "Enable raster-loss refinement (default true)."
                },
                "width": {
                    "type": "number",
                    "description": "Explicit advance width. Auto-computed when omitted."
                },
                "targetHeight": {
                    "type": "number",
                    "description": "Ascender - descender in font units (default 1088)."
                },
                "yOffset": {
                    "type": "number",
                    "description": "Vertical placement offset, usually the descender (default -256)."
                },
                "invert": {
                    "type": "boolean",
                    "description": "Invert the image before tracing."
                },
                "threshold": {
                    "type": "integer",
                    "minimum": 0,
                    "maximum": 255,
                    "description": "Fixed brightness threshold. Omitted uses Otsu."
                },
                "fitSource": {
                    "type": "string",
                    "enum": ["canvas", "ink"],
                    "description": "Placement source box: \"canvas\" frames the \
                        whole image to the vertical extent (default); \"ink\" \
                        detects the ink and fits it to [fitYMin, fitYMax] — use \
                        for generated/padded images. With \"ink\", json output \
                        also returns a placement report."
                },
                "fitYMin": {
                    "type": "number",
                    "description": "Target band bottom in font units (e.g. -256 \
                        for the descender). Required with fitSource \"ink\"."
                },
                "fitYMax": {
                    "type": "number",
                    "description": "Target band top in font units (e.g. 768 for \
                        cap height). Required with fitSource \"ink\"."
                },
                "lsb": {
                    "type": "number",
                    "description": "Left sidebearing in font units (default 64; \
                        used with fitSource \"ink\")."
                },
                "rsb": {
                    "type": "number",
                    "description": "Right sidebearing in font units (default 64; \
                        used with fitSource \"ink\")."
                },
                "format": {
                    "type": "string",
                    "enum": ["json", "svg", "glif"],
                    "description": "Output format (default \"json\")."
                }
            },
            "required": ["path"]
        }
    })
}

/// Handle a `tools/call` request: run the named tool and wrap its output.
fn call_tool(params: &Value) -> Result<Value, (i64, String)> {
    let name = params.get("name").and_then(Value::as_str).unwrap_or("");
    if name != "trace_glyph" {
        return Err((-32602, format!("unknown tool: {name}")));
    }
    let args = params.get("arguments").cloned().unwrap_or(Value::Null);
    match trace_glyph(&args) {
        Ok(text) => Ok(json!({
            "content": [{"type": "text", "text": text}],
            "isError": false,
        })),
        // Tool-level failures are reported in the result (isError), not as a
        // protocol error, so the agent can read and react to the message.
        Err(message) => Ok(json!({
            "content": [{"type": "text", "text": message}],
            "isError": true,
        })),
    }
}

/// Trace the image at `args.path` and serialize it in the requested format.
fn trace_glyph(args: &Value) -> Result<String, String> {
    let path = args
        .get("path")
        .and_then(Value::as_str)
        .ok_or("missing required argument: path")?;
    let bytes =
        std::fs::read(path).map_err(|e| format!("reading {path}: {e}"))?;

    let profile = img2bez::Profile::from_name_lossy(
        args.get("profile")
            .and_then(Value::as_str)
            .unwrap_or("wild"),
    );
    let mut opts = img2bez::TraceOptions::for_profile(profile);
    // RTL start-point rule from the codepoint, mirroring the CLI and wasm
    // surfaces (byte-identical output across all three).
    opts.rtl_start = args
        .get("unicode")
        .and_then(Value::as_str)
        .and_then(|h| {
            u32::from_str_radix(
                h.trim_start_matches("U+").trim_start_matches("u+").trim(),
                16,
            )
            .ok()
        })
        .and_then(char::from_u32)
        .is_some_and(img2bez::is_rtl_codepoint);
    if let Some(a) = args.get("accuracy").and_then(Value::as_f64) {
        opts = opts.with_accuracy(a);
    }
    if let Some(g) = args.get("grid").and_then(Value::as_i64) {
        opts = opts.with_grid(g as i32);
    }
    if let Some(chamfer) = args.get("chamfer").and_then(Value::as_f64) {
        opts.chamfer_size = chamfer.max(0.0);
    }
    if let Some(min_edge) = args.get("chamferMinEdge").and_then(Value::as_f64) {
        opts.chamfer_min_edge = min_edge.max(0.0);
    }
    if let Some(refine) = args.get("refine").and_then(Value::as_bool) {
        opts.refine_raster = refine;
    }
    if let Some(invert) = args.get("invert").and_then(Value::as_bool) {
        opts.invert = invert;
    }
    if let Some(threshold) = args.get("threshold").and_then(Value::as_u64) {
        let threshold = u8::try_from(threshold.min(255)).unwrap_or(255);
        opts.threshold = img2bez::ThresholdMethod::Fixed(threshold);
    }
    if let Some(target_height) =
        args.get("targetHeight").and_then(Value::as_f64)
    {
        opts.em_height = target_height.max(1.0);
    }

    let target_height = args
        .get("targetHeight")
        .and_then(Value::as_f64)
        .unwrap_or(1088.0)
        .max(1.0);
    let y_offset = args
        .get("yOffset")
        .and_then(Value::as_f64)
        .unwrap_or(-256.0);
    let mut metrics =
        img2bez::FontMetrics::from_target_height(target_height, y_offset);
    metrics.advance_width = args.get("width").and_then(Value::as_f64);

    let name = args.get("glyph").and_then(Value::as_str).unwrap_or("a");
    let codepoints = parse_codepoints(args);
    let format = args.get("format").and_then(Value::as_str).unwrap_or("json");

    // Ink-fit placement path: detect the ink, fit it to the target band, and
    // (for json) return a placement report alongside the glyph.
    if args.get("fitSource").and_then(Value::as_str) == Some("ink") {
        let placement = build_placement(args, opts.grid)?;
        let (placed, report) = img2bez::trace_place(&bytes, &opts, &placement)
            .map_err(|e| format!("trace: {e}"))?;
        let glyph =
            img2bez::glyph_from_placed(name, &codepoints, &placed, &metrics);
        return match format {
            "svg" => Ok(glyph.outline.to_svg_path()),
            "glif" => Ok(glyph.to_glif()),
            "json" => serde_json::to_string_pretty(
                &json!({ "glyph": glyph, "report": report }),
            )
            .map_err(|e| format!("json: {e}")),
            other => Err(format!("unknown format: {other}")),
        };
    }

    let glyph =
        img2bez::trace_glyph(&bytes, name, &codepoints, &opts, &metrics)
            .map_err(|e| format!("trace: {e}"))?;

    match format {
        "svg" => Ok(glyph.outline.to_svg_path()),
        "glif" => Ok(glyph.to_glif()),
        "json" => serde_json::to_string_pretty(&glyph)
            .map_err(|e| format!("json: {e}")),
        other => Err(format!("unknown format: {other}")),
    }
}

/// Build `PlacementOptions` for the ink-fit path from the tool arguments.
/// Requires `fitYMin`/`fitYMax`; `lsb`/`rsb` default to 64.
fn build_placement(
    args: &Value,
    grid: i32,
) -> Result<img2bez::PlacementOptions, String> {
    use img2bez::{Sidebearings, SourceBox, TargetBand};
    let y_min = args
        .get("fitYMin")
        .and_then(Value::as_f64)
        .ok_or("fitSource \"ink\" requires fitYMin")?;
    let y_max = args
        .get("fitYMax")
        .and_then(Value::as_f64)
        .ok_or("fitSource \"ink\" requires fitYMax")?;
    let lsb = args.get("lsb").and_then(Value::as_f64).unwrap_or(64.0);
    let rsb = args.get("rsb").and_then(Value::as_f64).unwrap_or(64.0);

    let mut placement = img2bez::PlacementOptions::ink_fit(
        TargetBand::Custom { y_min, y_max },
        Sidebearings::Explicit { lsb, rsb },
    );
    placement.source = SourceBox::InkBounds;
    placement.grid = grid;
    Ok(placement)
}

/// Parse the `unicode` hex argument into a codepoint list (empty if absent).
fn parse_codepoints(args: &Value) -> Vec<char> {
    args.get("unicode")
        .and_then(Value::as_str)
        .and_then(|h| u32::from_str_radix(h.trim(), 16).ok())
        .and_then(char::from_u32)
        .into_iter()
        .collect()
}
