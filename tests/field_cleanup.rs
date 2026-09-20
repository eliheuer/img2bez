// Copyright 2026 the img2bez Authors
// SPDX-License-Identifier: Apache-2.0 OR MIT

//! Cleanup budgets apply to signed fields as well as image contours.

use img2bez::kurbo::{BezPath, ParamCurve, ParamCurveNearest};
use img2bez::{Profile, TraceOptions, trace_sdf};

fn distance(from: &BezPath, to: &BezPath) -> f64 {
    from.segments()
        .flat_map(|s| (0..=100).map(move |i| s.eval(f64::from(i) / 100.0)))
        .map(|p| {
            to.segments()
                .map(|s| s.nearest(p, 1e-6).distance_sq)
                .fold(f64::INFINITY, f64::min)
                .sqrt()
        })
        .fold(0.0, f64::max)
}

#[test]
fn diagonal_field_cleanup_respects_the_requested_displacement() {
    // A shallow diagonal blend used to move over two units during cleanup,
    // despite a requested fitting accuracy of 0.25 units.
    let size = 1000;
    let values: Vec<_> = (0..size * size)
        .map(|i| {
            let x = (i % size) as f64 * 0.5 - 40.0;
            let y = (size - 1 - i / size) as f64 * 0.5 - 40.0;
            let field: f64 = [(150.0, 150.0, 180.0), (290.0, 300.0, 140.0)]
                .into_iter()
                .map(|(cx, cy, r)| {
                    2.0 * (1.0
                        - ((x - cx).powi(2) + (y - cy).powi(2)) / (r * r))
                        .max(0.0)
                        .powi(3)
                })
                .sum();
            (field - 0.5) as f32
        })
        .collect();
    let mut options = TraceOptions::for_profile(Profile::Clean)
        .with_grid(0)
        .with_em_height(500.0)
        .with_accuracy(0.25);
    options.smoothing = 0.0;
    options.min_contour_area = 0.0;
    options.faithful = true;
    let raw = trace_sdf(size, size, &values, 1, &options)
        .unwrap()
        .to_bezpaths();
    options.faithful = false;
    let unbounded = trace_sdf(size, size, &values, 1, &options)
        .unwrap()
        .to_bezpaths();
    options.cleanup_max_deviation = Some(0.25);
    let bounded = trace_sdf(size, size, &values, 1, &options)
        .unwrap()
        .to_bezpaths();
    assert_eq!(bounded.len(), raw.len());
    assert!(
        distance(&unbounded[0], &raw[0]) > 1.0,
        "fixture must exercise cleanup movement"
    );
    let error =
        distance(&bounded[0], &raw[0]).max(distance(&raw[0], &bounded[0]));
    assert!(error <= 0.25, "sampled cleanup movement {error}");
}
