// Copyright 2026 the img2bez Authors
// SPDX-License-Identifier: Apache-2.0 OR MIT

//! Smooth analytic boundaries retain design structure without forced inflection nodes.

use img2bez::kurbo::{
    BezPath, CubicBez, ParamCurve, ParamCurveDeriv, ParamCurveNearest, Shape,
};
use img2bez::{BoundaryFeature, BoundarySample, fit_smooth_contours};

fn source() -> Vec<CubicBez> {
    // An asymmetric two-lobed contour. Each join is a true axis extremum;
    // four connecting cubics carry an inflection inside the segment.
    [
        [(100.0, 300.0), (45.0, 300.0), (0.0, 255.0), (0.0, 200.0)],
        [(0.0, 200.0), (0.0, 150.0), (50.0, 170.0), (50.0, 120.0)],
        [(50.0, 120.0), (50.0, 60.0), (-100.0, 60.0), (-100.0, 0.0)],
        [
            (-100.0, 0.0),
            (-100.0, -55.0),
            (-55.0, -100.0),
            (0.0, -100.0),
        ],
        [(0.0, -100.0), (55.0, -100.0), (100.0, -55.0), (100.0, 0.0)],
        [(100.0, 0.0), (100.0, 50.0), (50.0, 30.0), (50.0, 80.0)],
        [(50.0, 80.0), (50.0, 140.0), (200.0, 140.0), (200.0, 200.0)],
        [
            (200.0, 200.0),
            (200.0, 255.0),
            (155.0, 300.0),
            (100.0, 300.0),
        ],
    ]
    .map(|p| CubicBez::new(p[0], p[1], p[2], p[3]))
    .to_vec()
}

fn samples(curves: &[CubicBez]) -> Vec<BoundarySample> {
    let mut samples = Vec::new();
    for (index, curve) in curves.iter().enumerate() {
        let inflections = curve.inflections();
        let mut parameters: Vec<_> = (0..128)
            .map(|i| f64::from(i) / 128.0)
            .chain(inflections.iter().copied())
            .collect();
        parameters.sort_by(f64::total_cmp);
        parameters.dedup_by(|a, b| (*a - *b).abs() < 1e-12);
        for t in parameters {
            let position = curve.eval(t);
            let tangent = curve.deriv().eval(t);
            let feature = if t == 0.0 {
                Some(if index == 0 || index == 4 {
                    BoundaryFeature::ExtremumY
                } else {
                    BoundaryFeature::ExtremumX
                })
            } else if inflections.iter().any(|i| (t - i).abs() < 1e-12) {
                Some(BoundaryFeature::Inflection)
            } else {
                None
            };
            samples.push(BoundarySample {
                position: [position.x, position.y],
                tangent: [tangent.x, tangent.y],
                feature,
            });
        }
    }
    samples
}

fn path(curves: &[CubicBez]) -> BezPath {
    let mut path = BezPath::new();
    path.move_to(curves[0].p0);
    for c in curves {
        path.curve_to(c.p1, c.p2, c.p3);
    }
    path.close_path();
    path
}

fn directed_error(from: &BezPath, to: &BezPath) -> f64 {
    from.segments()
        .flat_map(|s| (0..=256).map(move |i| s.eval(f64::from(i) / 256.0)))
        .map(|p| {
            to.segments()
                .map(|s| s.nearest(p, 1e-8).distance_sq)
                .fold(f64::INFINITY, f64::min)
                .sqrt()
        })
        .fold(0.0, f64::max)
}

#[test]
fn one_cubic_can_carry_an_inflection_without_an_extra_node() {
    let curves = source();
    let samples = samples(&curves);
    assert_eq!(
        samples
            .iter()
            .filter(|s| s.feature == Some(BoundaryFeature::Inflection))
            .count(),
        4,
        "fixture must contain internal inflections"
    );
    let paths = fit_smooth_contours(&[samples], 0.1).unwrap().to_bezpaths();
    assert_eq!(paths.len(), 1);
    assert_eq!(paths[0].segments().count(), curves.len());
    let reference = path(&curves);
    let error = directed_error(&reference, &paths[0])
        .max(directed_error(&paths[0], &reference));
    assert!(error <= 0.1, "sampled symmetric source error {error}");
    for segment in paths[0].segments() {
        let c = segment.to_cubic();
        for handle in [c.p1 - c.p0, c.p3 - c.p2] {
            assert!(handle.x == 0.0 || handle.y == 0.0);
        }
        assert!(curves.iter().any(|source| source.p0 == c.p0));
    }
}

#[test]
fn bottom_start_is_independent_of_input_rotation_and_winding() {
    let curves = source();
    let original = samples(&curves);
    let area = path(&curves).area();
    for reverse in [false, true] {
        for offset in [0, 73, 500] {
            let mut input = original.clone();
            input.rotate_left(offset);
            if reverse {
                input.reverse();
                for sample in &mut input {
                    sample.tangent[0] = -sample.tangent[0];
                    sample.tangent[1] = -sample.tangent[1];
                }
            }
            let paths =
                fit_smooth_contours(&[input], 0.1).unwrap().to_bezpaths();
            let first = paths[0].segments().next().unwrap().start();
            assert_eq!(first.x, 0.0);
            assert_eq!(first.y, -100.0);
            assert_eq!(
                paths[0].area().signum(),
                if reverse {
                    -area.signum()
                } else {
                    area.signum()
                }
            );
            assert_eq!(paths[0].segments().count(), curves.len());
        }
    }
}
