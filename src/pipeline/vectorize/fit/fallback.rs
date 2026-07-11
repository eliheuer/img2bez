// Copyright 2026 the img2bez Authors
// SPDX-License-Identifier: Apache-2.0 OR MIT

//! Fallback subdivision fit for sections one constrained cubic cannot hold.

use kurbo::{Point, Vec2};

use crate::model::geom::line_seg;

use super::sections::constrained_cubic_fit;
use super::{
    OPEN_FIT_MAX_SEGMENTS, SHORT_STRAIGHT_MAX_DEVIATION,
    SHORT_STRAIGHT_MAX_SAMPLES, chord_deviation_pts, dist_pt,
};

/// Fallback fit for sections no single constrained cubic can hold:
/// minimal-count subdivision at equal arc length, shared window tangents
/// at the cuts (tangent-continuous joints). Returns the smallest segment
/// count within `accuracy`, capped at OPEN_FIT_MAX_SEGMENTS.
pub(super) fn fit_open_samples(
    samples: &[Point],
    accuracy: f64,
) -> Vec<[Point; 4]> {
    if samples.len() < 3
        || dist_pt(samples[0], samples[samples.len() - 1]) < 1.5
    {
        return vec![line_seg(samples[0], samples[samples.len() - 1])];
    }
    if let Some(dir) = std::env::var_os("IMG2BEZ_DEBUG_OPTFIT_DUMP") {
        let dump: String = samples
            .iter()
            .map(|p| format!("{},{}\n", p.x, p.y))
            .collect();
        let path = std::path::Path::new(&dir)
            .join(format!("optfit-{}.csv", samples.len()));
        let _ = std::fs::write(path, dump);
    }
    // Own subdivision instead of kurbo's fitters: kurbo 0.13's
    // fit_to_bezpath_opt can fail to terminate on clean polyline sources,
    // and its plain sibling spends extra segments. Equal-arc spans keep
    // the count guarantee, are bounded by construction, and share window
    // tangents at cuts (tangent-continuous joints). Error-adaptive cut
    // placement was rejected: cuts at the worst sample land where window
    // tangents are noisiest and produce reversed-handle cubics.
    let n = samples.len();
    let mut cum = Vec::with_capacity(n);
    let mut acc = 0.0;
    cum.push(0.0);
    for i in 1..n {
        acc += (samples[i] - samples[i - 1]).hypot();
        cum.push(acc);
    }
    let total = acc.max(1e-12);
    let tangent = |i: usize| -> Vec2 {
        let a = i.saturating_sub(1);
        let b = (i + 1).min(n - 1);
        let d = samples[b] - samples[a];
        let len = d.hypot();
        if len < 1e-9 {
            Vec2::new(1.0, 0.0)
        } else {
            d * (1.0 / len)
        }
    };
    let fit_span = |a: usize, b: usize| -> ([Point; 4], f64) {
        let span = &samples[a..=b];
        // A short isolated span may be a designed straight: emit the line,
        // not a near-degenerate cubic. Long spans stay cubic — a long
        // shallow bow must keep its curvature.
        if span.len() <= SHORT_STRAIGHT_MAX_SAMPLES
            && chord_deviation_pts(span) <= SHORT_STRAIGHT_MAX_DEVIATION
        {
            return (line_seg(span[0], span[span.len() - 1]), 0.0);
        }
        constrained_cubic_fit(span, tangent(a), -tangent(b))
    };
    let max_k = OPEN_FIT_MAX_SEGMENTS.min(n / 2).max(1);
    let mut best: Option<(f64, Vec<[Point; 4]>)> = None;
    for k in 1..=max_k {
        // Span boundaries at even arc length, strictly increasing.
        let mut idx = Vec::with_capacity(k + 1);
        idx.push(0usize);
        let mut cursor = 0usize;
        for j in 1..k {
            let target = total * j as f64 / k as f64;
            while cursor + 1 < n - 1 && cum[cursor + 1] < target {
                cursor += 1;
            }
            let pick = if cursor + 1 < n - 1
                && (cum[cursor + 1] - target).abs()
                    < (target - cum[cursor]).abs()
            {
                cursor + 1
            } else {
                cursor
            };
            let prev = *idx.last().unwrap();
            idx.push(pick.max(prev + 1).min(n - 1 - (k - j)));
        }
        idx.push(n - 1);
        if idx.windows(2).any(|w| w[1] <= w[0]) {
            break; // not enough samples to split further
        }
        let mut segs = Vec::with_capacity(k);
        let mut worst: f64 = 0.0;
        for w in idx.windows(2) {
            let (seg, err) = fit_span(w[0], w[1]);
            worst = worst.max(err);
            segs.push(seg);
        }
        if best.as_ref().is_none_or(|(e, _)| worst < *e) {
            best = Some((worst, segs));
        }
        if worst <= accuracy {
            break;
        }
    }
    let (_, segs) = best.expect("k=1 always fits something");
    segs
}
