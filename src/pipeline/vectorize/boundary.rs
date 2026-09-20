// Copyright 2026 the img2bez Authors
// SPDX-License-Identifier: Apache-2.0 OR MIT

//! Fit smooth boundaries whose source supplies exact tangents and structural features.

use kurbo::{BezPath, Point, Vec2};

use crate::{Outline, TraceError};

/// A structural node retained when fitting a smooth boundary.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub enum BoundaryFeature {
    /// Minimum or maximum x; both handles are exactly vertical.
    ExtremumX,
    /// Minimum or maximum y; both handles are exactly horizontal.
    ExtremumY,
    /// Curvature sign change; retain the supplied tangent direction.
    Inflection,
}

/// One ordered sample of a closed smooth boundary, in output coordinates.
///
/// The tangent points in the direction of travel along the contour.
/// Its magnitude does not matter, but it must be finite and nonzero.
/// Analytic sources can supply exact derivatives and structural locations
/// instead of having these estimated from pixels.
#[derive(Clone, Copy, Debug)]
pub struct BoundarySample {
    /// Position in font units, with y pointing up.
    pub position: [f64; 2],
    /// Forward tangent vector in the same coordinate system.
    pub tangent: [f64; 2],
    /// A retained node, or None for an ordinary boundary sample.
    pub feature: Option<BoundaryFeature>,
}

/// Fit ordered, closed smooth contours with source-supplied feature nodes.
///
/// Each contour must contain at least three samples, at least two marked
/// features, and distinct consecutive positions (including the closing pair).
/// Do not repeat the first sample at the end.
/// Positions, tangents, and `accuracy` must be finite; accuracy must be positive.
/// The caller supplies enough samples to capture the shape and every feature.
/// Winding, feature positions and fractional coordinates are preserved.
/// No image cleanup, grid snapping, placement or direction normalization runs.
///
/// Kurbo fits each span independently, retaining endpoint positions and tangents.
/// Accuracy is relative to the cubic Hermite interpolant of the samples;
/// it is not an error bound against the source's underlying analytic curve.
/// This entry point complements [`crate::trace_sdf`] when a source can provide
/// more information than scalar grid samples.
///
/// # Errors
///
/// Returns [`TraceError::InvalidBoundary`] for invalid samples or accuracy,
/// and [`TraceError::NoContours`] for an empty contour list.
pub fn fit_smooth_contours(
    contours: &[Vec<BoundarySample>],
    accuracy: f64,
) -> Result<Outline, TraceError> {
    if !accuracy.is_finite() || accuracy <= 0.0 {
        return Err(TraceError::InvalidBoundary(
            "accuracy must be finite and positive",
        ));
    }
    if contours.is_empty() {
        return Err(TraceError::NoContours);
    }
    let paths = contours
        .iter()
        .map(|c| fit(c, accuracy))
        .collect::<Result<Vec<_>, _>>()?;
    Ok(Outline::from_bezpaths(&paths))
}

fn position(sample: BoundarySample) -> Point {
    Point::new(sample.position[0], sample.position[1])
}

fn direction(sample: BoundarySample) -> Vec2 {
    let mut direction = Vec2::new(sample.tangent[0], sample.tangent[1]);
    match sample.feature {
        Some(BoundaryFeature::ExtremumX) => direction.x = 0.0,
        Some(BoundaryFeature::ExtremumY) => direction.y = 0.0,
        _ => (),
    }
    direction
}

fn align(handle: &mut Point, sample: BoundarySample) {
    match sample.feature {
        Some(BoundaryFeature::ExtremumX) => handle.x = sample.position[0],
        Some(BoundaryFeature::ExtremumY) => handle.y = sample.position[1],
        _ => (),
    }
}

fn fit(
    samples: &[BoundarySample],
    accuracy: f64,
) -> Result<BezPath, TraceError> {
    if samples.len() < 3
        || samples.iter().filter(|s| s.feature.is_some()).count() < 2
    {
        return Err(TraceError::InvalidBoundary(
            "need three samples and two features per contour",
        ));
    }
    for (i, sample) in samples.iter().enumerate() {
        let edge_length = position(*sample)
            .distance(position(samples[(i + 1) % samples.len()]));
        if !sample
            .position
            .iter()
            .chain(&sample.tangent)
            .all(|x| x.is_finite())
            || !direction(*sample).hypot().is_finite()
            || direction(*sample).hypot() < 1e-12
            || !edge_length.is_finite()
            || edge_length <= 1e-8
        {
            return Err(TraceError::InvalidBoundary(
                "nonfinite, coincident or degenerate boundary samples",
            ));
        }
    }
    let mut samples = samples.to_vec();
    let first = samples
        .iter()
        .position(|s| s.feature.is_some())
        .expect("features validated above");
    samples.rotate_left(first);
    let mut stops: Vec<_> = samples
        .iter()
        .enumerate()
        .filter_map(|(i, s)| s.feature.map(|_| i))
        .collect();
    stops.push(samples.len());
    let mut output = BezPath::new();
    output.move_to(position(samples[0]));
    for pair in stops.windows(2) {
        let start = samples[pair[0]];
        let end = samples[pair[1] % samples.len()];
        let mut span = BezPath::new();
        span.move_to(position(start));
        for i in pair[0]..pair[1] {
            let a = samples[i];
            let b = samples[(i + 1) % samples.len()];
            let length = position(a).distance(position(b)) / 3.0;
            span.curve_to(
                position(a) + direction(a).normalize() * length,
                position(b) - direction(b).normalize() * length,
                position(b),
            );
        }
        if !span.is_finite() {
            return Err(TraceError::InvalidBoundary(
                "boundary interpolation overflow",
            ));
        }
        let fitted = kurbo::simplify::simplify_bezpath(
            span,
            accuracy,
            &kurbo::simplify::SimplifyOptions::default(),
        );
        let mut cubics: Vec<_> =
            fitted.segments().map(|s| s.to_cubic()).collect();
        // Restore exact endpoint constraints after Kurbo's floating-point angle arithmetic.
        if let Some(first) = cubics.first_mut() {
            first.p1 += position(start) - first.p0;
            first.p0 = position(start);
            align(&mut first.p1, start);
        }
        if let Some(last) = cubics.last_mut() {
            last.p2 += position(end) - last.p3;
            last.p3 = position(end);
            align(&mut last.p2, end);
        }
        for cubic in cubics {
            output.curve_to(cubic.p1, cubic.p2, cubic.p3);
        }
    }
    output.close_path();
    if !output.is_finite() {
        return Err(TraceError::InvalidBoundary(
            "fitting produced nonfinite coordinates",
        ));
    }
    Ok(output)
}

#[cfg(test)]
mod tests {
    use super::*;
    use kurbo::{ParamCurve, Shape};

    fn circle(radius: f64) -> Vec<BoundarySample> {
        (0..256)
            .map(|i| {
                let angle = f64::from(i) * std::f64::consts::TAU / 256.0;
                BoundarySample {
                    position: [
                        12.25 + radius * angle.cos(),
                        -23.5 + radius * angle.sin(),
                    ],
                    tangent: [-angle.sin(), angle.cos()],
                    feature: match i {
                        0 | 128 => Some(BoundaryFeature::ExtremumX),
                        64 | 192 => Some(BoundaryFeature::ExtremumY),
                        _ => None,
                    },
                }
            })
            .collect()
    }

    #[test]
    fn circle_has_four_exact_axis_nodes_and_preserves_fractional_position() {
        let outline = fit_smooth_contours(&[circle(100.0)], 0.25).unwrap();
        let paths = outline.to_bezpaths();
        assert_eq!(paths[0].segments().count(), 4);
        for segment in paths[0].segments() {
            let c = segment.to_cubic();
            assert!((c.p0.x - 12.25).abs().min((c.p0.y + 23.5).abs()) < 1e-10);
            for handle in [c.p1 - c.p0, c.p3 - c.p2] {
                assert!(handle.x == 0.0 || handle.y == 0.0);
            }
            for i in 0..=100 {
                assert!(
                    (c.eval(f64::from(i) / 100.0)
                        .distance(Point::new(12.25, -23.5))
                        - 100.0)
                        .abs()
                        < 0.03
                );
            }
        }
    }

    #[test]
    fn holes_keep_their_opposite_winding() {
        let mut hole = circle(40.0);
        hole.reverse();
        for sample in &mut hole {
            sample.tangent[0] *= -1.0;
            sample.tangent[1] *= -1.0;
        }
        let paths = fit_smooth_contours(&[circle(100.0), hole], 0.25)
            .unwrap()
            .to_bezpaths();
        assert_eq!(paths.len(), 2);
        assert!(paths[0].area() > 0.0 && paths[1].area() < 0.0);
    }

    #[test]
    fn invalid_boundaries_fail_without_partial_output() {
        let valid = circle(100.0);
        let mut invalid = valid.clone();
        invalid[1].tangent = [0.0, 0.0];
        assert!(fit_smooth_contours(&[valid.clone(), invalid], 0.25).is_err());
        assert!(
            fit_smooth_contours(std::slice::from_ref(&valid), f64::NAN)
                .is_err()
        );
        let mut missing_features = valid.clone();
        for point in &mut missing_features {
            point.feature = None;
        }
        assert!(fit_smooth_contours(&[missing_features], 0.25).is_err());
        let mut duplicate = valid;
        duplicate.push(duplicate[0]);
        assert!(fit_smooth_contours(&[duplicate], 0.25).is_err());
    }
}
