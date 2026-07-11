// Copyright 2026 the img2bez Authors
// SPDX-License-Identifier: Apache-2.0 OR MIT

//! Multi-master end-to-end fixture: `trace_masters` traces several images and
//! returns an interpolation-compatible set (see [`img2bez::compat`]).

use image::{GrayImage, Luma};
use img2bez::{TraceOptions, compat, trace_masters};

/// A dark rectangle on white at `[x0,x1] × [y0,y1]` in a `w × h` image.
fn rect_png(w: u32, h: u32, x0: u32, y0: u32, x1: u32, y1: u32) -> Vec<u8> {
    let mut img = GrayImage::from_pixel(w, h, Luma([255]));
    for y in y0..y1 {
        for x in x0..x1 {
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
fn trace_masters_returns_a_compatible_set() {
    // Two "weights" of a square: different size and padding.
    let light = rect_png(200, 300, 40, 50, 150, 250);
    let bold = rect_png(240, 320, 30, 40, 200, 280);

    let (outlines, report) =
        trace_masters(&[&light, &bold], &TraceOptions::default())
            .expect("trace_masters");

    assert_eq!(outlines.len(), 2);
    assert!(report.compatible);
    // The returned set must interpolate: same contour/point/kind structure.
    compat::check(&outlines).expect("masters interpolate");
    // Each master keeps its own geometry (different sizes → different bounds).
    assert_ne!(outlines[0].bounds(), outlines[1].bounds());
}
