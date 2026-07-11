// Copyright 2026 the img2bez Authors
// SPDX-License-Identifier: Apache-2.0 OR MIT

//! Deterministic-placement fixtures.
//!
//! Placement is the headless half of the pipeline (`trace` answers "what
//! contours?", `place`/`trace_place` answer "where in the font?"). These tests
//! pin the two properties agents depend on:
//!
//!   1. `position` is exact, deterministic geometry — ink fills the target band,
//!      the requested sidebearings/advance come out, and the report is honest.
//!   2. `trace_place` with `InkBounds` is *padding-invariant*: the same glyph
//!      with different image margins places to the identical outline. This is
//!      the germandbls failure mode the model exists to fix.

use image::{GrayImage, Luma};
use img2bez::kurbo::Rect;
use img2bez::{
    Contour, Outline, OutlinePoint, PlacementOptions, PointKind, Sidebearings,
    TargetBand, TraceOptions,
};

/// A closed rectangular contour `[x0,x1] × [y0,y1]` of straight segments.
fn rect_outline(x0: f64, y0: f64, x1: f64, y1: f64) -> Outline {
    let corner = |x: f64, y: f64| OutlinePoint {
        x,
        y,
        kind: PointKind::Line,
        smooth: false,
    };
    Outline {
        contours: vec![Contour {
            points: vec![
                corner(x0, y0),
                corner(x1, y0),
                corner(x1, y1),
                corner(x0, y1),
            ],
        }],
    }
}

#[test]
fn position_fills_band_and_honors_sidebearings() {
    // A 200×400 outline already scaled to the band height; place it into
    // y=[0,700] with lsb 50 / rsb 60.
    let outline = rect_outline(0.0, 0.0, 200.0, 400.0);
    let opts = PlacementOptions::ink_fit(
        TargetBand::Custom {
            y_min: 0.0,
            y_max: 700.0,
        },
        Sidebearings::Explicit {
            lsb: 50.0,
            rsb: 60.0,
        },
    );
    let (placed, report) = img2bez::placement::position(
        &outline,
        &opts,
        (200, 400),
        Rect::new(0.0, 0.0, 200.0, 400.0),
        1.0,
    );

    // Vertical: ink bottom sits on the band floor (the scale already sized it).
    assert_eq!(report.final_bounds.y0, 0.0, "ink bottom on the band floor");
    // Horizontal: leftmost ink at lsb, advance = rightmost ink + rsb.
    assert_eq!(report.lsb, 50.0);
    assert_eq!(report.advance_width, 50.0 + 200.0 + 60.0);
    assert_eq!(report.rsb, 60.0);
    assert_eq!(placed.advance_width, report.advance_width);
    assert!(!report.out_of_target, "ink within the band is in-target");
}

#[test]
fn position_center_and_fixed_advance() {
    let outline = rect_outline(0.0, 0.0, 200.0, 400.0);
    let band = TargetBand::Custom {
        y_min: 0.0,
        y_max: 400.0,
    };

    // Center: ink centered in a 600-unit advance → equal sidebearings of 200.
    let (_, centered) = img2bez::placement::position(
        &outline,
        &PlacementOptions::ink_fit(
            band,
            Sidebearings::Center { advance: 600.0 },
        ),
        (200, 400),
        Rect::new(0.0, 0.0, 200.0, 400.0),
        1.0,
    );
    assert_eq!(centered.advance_width, 600.0);
    assert_eq!(centered.lsb, 200.0);
    assert_eq!(centered.rsb, 200.0);

    // FixedAdvance: advance is pinned regardless of ink width.
    let (_, fixed) = img2bez::placement::position(
        &outline,
        &PlacementOptions::ink_fit(
            band,
            Sidebearings::FixedAdvance {
                width: 520.0,
                lsb: 40.0,
            },
        ),
        (200, 400),
        Rect::new(0.0, 0.0, 200.0, 400.0),
        1.0,
    );
    assert_eq!(fixed.advance_width, 520.0);
    assert_eq!(fixed.lsb, 40.0);
}

#[test]
fn position_flags_out_of_target() {
    // Outline reaches y=[-50, 450] but the band is only [0, 400]: the ink
    // bottom is placed on the floor, so the top (450) pokes out.
    let outline = rect_outline(0.0, -50.0, 100.0, 450.0);
    let (_, report) = img2bez::placement::position(
        &outline,
        &PlacementOptions::ink_fit(
            TargetBand::Custom {
                y_min: 0.0,
                y_max: 400.0,
            },
            Sidebearings::Explicit { lsb: 0.0, rsb: 0.0 },
        ),
        (100, 500),
        Rect::new(0.0, 0.0, 100.0, 500.0),
        1.0,
    );
    assert!(
        report.out_of_target,
        "outline taller than the band is flagged"
    );
}

/// A dark glyph rectangle of size `ink_w × ink_h` placed at `(ox, oy)` on a
/// white `w × h` canvas — i.e. the same ink with arbitrary padding.
fn padded_png(
    w: u32,
    h: u32,
    ox: u32,
    oy: u32,
    ink_w: u32,
    ink_h: u32,
) -> Vec<u8> {
    let mut img = GrayImage::from_pixel(w, h, Luma([255]));
    for y in oy..oy + ink_h {
        for x in ox..ox + ink_w {
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
fn ink_fit_places_to_band_regardless_of_padding() {
    let opts = TraceOptions::default();
    let placement = PlacementOptions::ink_fit(
        TargetBand::Custom {
            y_min: 0.0,
            y_max: 700.0,
        },
        Sidebearings::Explicit {
            lsb: 64.0,
            rsb: 64.0,
        },
    );

    // Same 160×280 ink, two very different canvases / offsets.
    let a = padded_png(400, 600, 40, 30, 160, 280);
    let b = padded_png(900, 1200, 500, 700, 160, 280);

    let (_placed_a, report_a) =
        img2bez::trace_place(&a, &opts, &placement).expect("place a");
    let (_placed_b, report_b) =
        img2bez::trace_place(&b, &opts, &placement).expect("place b");

    // Padding-invariant placement: the glyph lands in the same font-space box
    // and gets the same advance, whatever the margins. (The traced outlines are
    // not bit-identical — a larger canvas traces at a higher internal
    // resolution — but the placement they feed is.)
    let close = |a: f64, b: f64, tol: f64, what: &str| {
        assert!((a - b).abs() <= tol, "{what}: {a} vs {b}");
    };
    close(
        report_a.advance_width,
        report_b.advance_width,
        2.0,
        "advance is padding-invariant",
    );
    for i in 0..4 {
        let (fa, fb) = (report_a.final_bounds, report_b.final_bounds);
        let (va, vb) = (
            [fa.x0, fa.y0, fa.x1, fa.y1][i],
            [fb.x0, fb.y0, fb.x1, fb.y1][i],
        );
        close(va, vb, 2.0, "final bounds are padding-invariant");
    }

    // And it actually fills the requested band: ink bottom on 0, top at 700,
    // left ink at the 64-unit sidebearing.
    let fin = report_a.final_bounds;
    assert_eq!(fin.y0, 0.0, "ink bottom on the baseline");
    assert!(
        (fin.y1 - 700.0).abs() <= 2.0,
        "ink top fills the band (got {})",
        fin.y1
    );
    assert_eq!(fin.x0, 64.0, "left ink at the lsb");
    assert!(!report_a.out_of_target);
    assert_eq!(report_a.image_size, (400, 600));
    assert_eq!(report_b.image_size, (900, 1200));
}

#[cfg(feature = "ufo")]
#[test]
fn reference_metrics_round_trip() {
    use img2bez::{FontMetrics, read_reference};

    // Write a synthetic glyph to a real UFO, then read its metrics back.
    let dir = std::env::temp_dir().join("img2bez-ref-test");
    let _ = std::fs::remove_dir_all(&dir);
    std::fs::create_dir_all(&dir).unwrap();
    let png_path = dir.join("rect.png");
    std::fs::write(&png_path, padded_png(200, 300, 20, 30, 120, 200)).unwrap();
    let ufo_path = dir.join("Test.ufo");

    img2bez::trace_into_ufo(
        &png_path,
        "rect",
        &['A'],
        &ufo_path,
        &TraceOptions::default(),
        &FontMetrics::default(),
    )
    .expect("write ufo");

    let m = read_reference(&ufo_path, "rect").expect("read back");
    let ink = m.bounds.expect("glyph has ink");

    // The reader's derived metrics are internally consistent with the bounds
    // and advance it reports.
    assert_eq!(m.vertical_band(), Some((ink.y0, ink.y1)));
    let (lsb, rsb) = m.sidebearings().unwrap();
    assert_eq!(lsb, ink.x0, "lsb is the left ink edge");
    assert!(
        (rsb - (m.advance_width - ink.x1)).abs() < 1e-9,
        "rsb is advance minus the right ink edge"
    );
    assert!(m.advance_width > 0.0);

    // A missing glyph is a clear, typed error.
    let err = read_reference(&ufo_path, "nope").unwrap_err();
    assert!(
        matches!(err, img2bez::TraceError::ReferenceGlyphNotFound { .. }),
        "missing reference glyph is reported, got {err:?}"
    );

    let _ = std::fs::remove_dir_all(&dir);
}

#[test]
fn ink_bounds_strips_padding() {
    let png = padded_png(400, 600, 40, 30, 160, 280);
    let img = image::load_from_memory(&png).unwrap().into_luma8();
    let bounds =
        img2bez::placement::ink_bounds(&img, 128, false).expect("ink present");
    // (x0, y0, x1+1, y1+1) spanning the full extent of the ink rectangle.
    assert_eq!(bounds, Rect::new(40.0, 30.0, 200.0, 310.0));

    // A blank canvas has no ink.
    let blank = GrayImage::from_pixel(10, 10, Luma([255]));
    assert!(img2bez::placement::ink_bounds(&blank, 128, false).is_none());
}
