// Copyright 2026 the img2bez Authors
// SPDX-License-Identifier: Apache-2.0 OR MIT

//! Surface-parity + JSON-schema fixture.
//!
//! The CLI, WASM, and MCP surfaces all assemble their output through the same
//! library helpers (`trace_glyph` / `glyph_from_outline`). This test pins that
//! shared path: the high-level helper and the decomposed `trace` + place +
//! assemble path must produce the identical [`img2bez::Glyph`], and that
//! glyph's JSON must match the stable UFO-faithful schema (see `Glyph` docs).

use image::{GrayImage, Luma};
use img2bez::{
    FontMetrics, TraceOptions, glyph_from_outline, trace, trace_glyph,
};

/// A tiny deterministic raster: a dark rectangle on white (ink = dark).
fn fixture_png() -> Vec<u8> {
    let mut img = GrayImage::from_pixel(128, 160, Luma([255]));
    for y in 32..128 {
        for x in 24..104 {
            img.put_pixel(x, y, Luma([0]));
        }
    }
    let mut bytes = Vec::new();
    image::DynamicImage::ImageLuma8(img)
        .write_to(
            &mut std::io::Cursor::new(&mut bytes),
            image::ImageFormat::Png,
        )
        .expect("encode png");
    bytes
}

#[test]
fn library_entry_points_agree() {
    let png = fixture_png();
    let opts = TraceOptions::default();
    let metrics = FontMetrics::default();

    // The one-shot helper the CLI/WASM/MCP surfaces use.
    let high =
        trace_glyph(&png, "A", &['A'], &opts, &metrics).expect("trace_glyph");
    // The decomposed path: trace to a neutral outline, then place + assemble.
    let outline = trace(&png, &opts).expect("trace");
    let low = glyph_from_outline("A", &['A'], &outline, &metrics);

    assert_eq!(high, low, "the high-level and decomposed paths must agree");
}

#[test]
fn glyph_json_matches_stable_schema() {
    let png = fixture_png();
    let glyph = trace_glyph(
        &png,
        "A",
        &['A'],
        &TraceOptions::default(),
        &FontMetrics::default(),
    )
    .expect("trace_glyph");
    let json: serde_json::Value = serde_json::to_value(&glyph).unwrap();

    // UFO-faithful shape: name, unicodes[], advance.width, unitsPerEm, outline.contours[].
    assert_eq!(json["name"], "A");
    assert_eq!(json["unicodes"], serde_json::json!(["0041"]));
    assert!(json["advance"]["width"].is_number());
    assert_eq!(json["unitsPerEm"], 1024.0);
    let contours = json["outline"]["contours"]
        .as_array()
        .expect("contours array");
    assert!(!contours.is_empty());

    // Points serialize like UFO <point>: on-curve carry `type`, off-curve omit it.
    let points = contours[0]["points"].as_array().expect("points array");
    assert!(
        points
            .iter()
            .all(|p| p["x"].is_number() && p["y"].is_number())
    );
    assert!(
        points.iter().any(|p| p.get("type").is_some()),
        "at least one on-curve point carries a type"
    );
}
