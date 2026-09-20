// Copyright 2026 the img2bez Authors
// SPDX-License-Identifier: Apache-2.0 OR MIT

//! Keep optional design cleanup within a sampled displacement budget.

use kurbo::{BezPath, ParamCurve, ParamCurveNearest};

pub(super) fn constrain(
    reference: &[BezPath],
    previous: &[BezPath],
    candidates: Vec<BezPath>,
    limit: Option<f64>,
) -> Vec<BezPath> {
    let Some(limit) = limit else {
        return candidates;
    };
    if !limit.is_finite() || limit < 0.0 || candidates.len() != reference.len()
    {
        return previous.to_vec();
    }
    candidates
        .into_iter()
        .zip(reference)
        .zip(previous)
        .map(|((candidate, reference), previous)| {
            if candidate == *previous || within(reference, &candidate, limit) {
                candidate
            } else {
                previous.clone()
            }
        })
        .collect()
}

fn within(a: &BezPath, b: &BezPath, limit: f64) -> bool {
    if a.is_empty() || b.is_empty() || !a.is_finite() || !b.is_finite() {
        return false;
    }
    let directed = |from: &BezPath, to: &BezPath| {
        from.segments().all(|segment| {
            (0..=32).all(|i| {
                let point = segment.eval(f64::from(i) / 32.0);
                let distance2 = to
                    .segments()
                    .map(|target| target.nearest(point, 1e-6).distance_sq)
                    .fold(f64::INFINITY, f64::min);
                distance2 <= (limit + 1e-9).powi(2)
            })
        })
    };
    directed(a, b) && directed(b, a)
}

#[cfg(test)]
mod tests {
    use super::*;
    use kurbo::{Affine, Circle, Shape};

    #[test]
    fn accepted_passes_cannot_accumulate_drift() {
        let original = Circle::new((0.0, 0.0), 100.0).to_path(0.01);
        let first = Affine::translate((0.2, 0.0)) * &original;
        let second = Affine::translate((0.4, 0.0)) * &original;
        let kept = constrain(
            std::slice::from_ref(&original),
            std::slice::from_ref(&original),
            vec![first.clone()],
            Some(0.25),
        );
        assert_eq!(kept, vec![first.clone()]);
        assert_eq!(
            constrain(&[original], &kept, vec![second], Some(0.25)),
            vec![first]
        );
    }

    #[test]
    fn checks_both_directions_and_preserves_previous_contours() {
        let source = Circle::new((0.0, 0.0), 100.0).to_path(0.01);
        let short = BezPath::from_vec(source.elements()[..2].to_vec());
        assert!(
            !within(&source, &short, 0.25),
            "dropping most of a contour must fail"
        );
        assert!(!within(&short, &source, 0.25));
        let kept = constrain(
            std::slice::from_ref(&source),
            std::slice::from_ref(&source),
            vec![short],
            Some(0.25),
        );
        assert_eq!(kept, vec![source]);
    }
}
