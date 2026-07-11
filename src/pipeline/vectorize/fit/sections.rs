// Copyright 2026 the img2bez Authors
// SPDX-License-Identifier: Apache-2.0 OR MIT

//! Stage 4: fit each piece between splits as a line or constrained cubic.

use kurbo::{Point, Vec2};

use crate::model::geom::line_seg;

use super::fallback::fit_open_samples;
use super::{
    CONSTRAINED_FIT_TOLERANCE_FACTOR, CORNER_AXIS_SNAP_DEG,
    FREE_DIR_AXIS_SNAP_DEG, FittedContour, SHORT_STRAIGHT_MAX_DEVIATION,
    SHORT_STRAIGHT_MAX_SAMPLES, SPLIT_SNAP_SAMPLES, Split, SplitKind,
    chord_deviation_pts,
};

// ── Stage 4: section fitting ─────────────────────────────────────────

/// Fit every piece between adjacent splits: straight runs and short
/// near-straight sections become lines; curved sections try one
/// constrained cubic, then an inflection-split pair, then
/// `fit_open_samples` subdivision.
pub(super) fn fit_sections(
    smoothed: &[(f64, f64)],
    splits: &[Split],
    line_sections: &[(usize, usize)],
    n: usize,
    accuracy: f64,
) -> FittedContour {
    let ns = splits.len();
    let mut segs: Vec<[Point; 4]> = Vec::new();
    let mut is_line: Vec<bool> = Vec::new();
    let mut joint_kind: Vec<SplitKind> = Vec::new();
    for si in 0..ns {
        let a = splits[si];
        let b = splits[(si + 1) % ns];
        let len = {
            let d = (b.idx + n - a.idx) % n;
            if d == 0 { n } else { d }
        };
        let samples: Vec<Point> = (0..=len)
            .map(|k| {
                let (x, y) = smoothed[(a.idx + k) % n];
                Point::new(x, y)
            })
            .collect();
        let line = line_sections
            .iter()
            .any(|&(ra, rb)| ra == a.idx && rb == b.idx);
        // Short straight sections are lines; run detection cannot see
        // them (the rounding zones leave no clean interior to grow from).
        let short_straight = samples.len() <= SHORT_STRAIGHT_MAX_SAMPLES
            && chord_deviation_pts(&samples) <= SHORT_STRAIGHT_MAX_DEVIATION;
        if line || short_straight || samples.len() < 3 {
            segs.push(line_seg(samples[0], samples[samples.len() - 1]));
            is_line.push(true);
            joint_kind.push(a.kind);
        } else {
            let start_tangent = constrained_end_tangent(&samples, a.kind, true);
            let end_tangent = constrained_end_tangent(&samples, b.kind, false);
            let tol = accuracy * CONSTRAINED_FIT_TOLERANCE_FACTOR;
            let (single, err) =
                constrained_cubic_fit(&samples, start_tangent, end_tangent);
            if err <= tol {
                segs.push(single);
                is_line.push(false);
                joint_kind.push(a.kind);
            } else if let Some((pair, pair_lines)) = fit_split_at_inflection(
                &samples,
                start_tangent,
                end_tangent,
                tol,
            ) {
                // The section spans a curvature-sign change one cubic
                // can't bridge: split at the inflection like a designer
                // would; a flank too short for its own cubic is a line.
                for (k, seg) in pair.iter().enumerate() {
                    segs.push(*seg);
                    is_line.push(pair_lines[k]);
                    joint_kind.push(if k == 0 {
                        a.kind
                    } else {
                        SplitKind::FitterJoint
                    });
                }
            } else {
                let fitted = fit_open_samples(&samples, accuracy);
                if std::env::var("IMG2BEZ_DEBUG_FIT").is_ok() {
                    eprintln!(
                        "    section {}..{} ({} samples): \
                         constrained err {:.2}px -> {} fallback cubics",
                        a.idx,
                        b.idx,
                        samples.len(),
                        err,
                        fitted.len(),
                    );
                }
                for (k, seg) in fitted.iter().enumerate() {
                    segs.push(*seg);
                    is_line.push(false);
                    joint_kind.push(if k == 0 {
                        a.kind
                    } else {
                        SplitKind::FitterJoint
                    });
                }
            }
        }
    }
    FittedContour {
        segs,
        is_line,
        joint_kind,
    }
}

/// Tangent direction at a section end, honoring the split's constraint.
/// `at_start = true` returns the travel direction at the first sample;
/// `at_start = false` the BACKWARD direction at the last (matching
/// `constrained_cubic_fit`'s end_tangent).
pub(in crate::pipeline::vectorize) fn constrained_end_tangent(
    samples: &[Point],
    kind: SplitKind,
    at_start: bool,
) -> Vec2 {
    let n = samples.len();
    let w = 6.min(n - 1);
    let raw = if at_start {
        samples[w] - samples[0]
    } else {
        samples[n - 1 - w] - samples[n - 1]
    };
    let raw = if raw.hypot() < 1e-9 {
        Vec2::new(1.0, 0.0)
    } else {
        raw.normalize()
    };
    match kind {
        // An extremum's tangent is forced onto its axis; the sign keeps
        // the measured heading.
        SplitKind::ExtremumX => Vec2::new(0.0, raw.y.signum()), // vertical
        SplitKind::ExtremumY => Vec2::new(raw.x.signum(), 0.0), // horizontal
        _ => {
            // Free directions (corner/inflection) snap onto an axis
            // within the band; corners get the tighter band plus the
            // beyond-zone override (see CORNER_AXIS_SNAP_DEG).
            let corner = kind == SplitKind::Corner;
            let band = if corner {
                CORNER_AXIS_SNAP_DEG
            } else {
                FREE_DIR_AXIS_SNAP_DEG
            };
            let slope = |deg: f64| deg.to_radians().tan();
            let near_h = |d: Vec2, deg: f64| {
                d.x.abs() > d.y.abs() && d.y.abs() <= d.x.abs() * slope(deg)
            };
            let near_v = |d: Vec2, deg: f64| {
                d.y.abs() > d.x.abs() && d.x.abs() <= d.y.abs() * slope(deg)
            };
            let beyond = || -> Option<Vec2> {
                let skip = SPLIT_SNAP_SAMPLES.min(n.saturating_sub(8) / 2);
                let bw = 6.min(n.saturating_sub(1 + skip));
                if skip == 0 || bw < 3 {
                    return None;
                }
                let d = if at_start {
                    samples[skip + bw] - samples[skip]
                } else {
                    samples[n - 1 - skip - bw] - samples[n - 1 - skip]
                };
                (d.hypot() > 1e-9).then(|| d.normalize())
            };
            if corner && std::env::var("IMG2BEZ_DEBUG_TANGENT").is_ok() {
                eprintln!(
                    "    corner-tangent at_start={} raw={:.1}° beyond={}",
                    at_start,
                    raw.atan2().to_degrees(),
                    beyond()
                        .map(|d| format!("{:.1}°", d.atan2().to_degrees()))
                        .unwrap_or("-".into()),
                );
            }
            if near_h(raw, band) {
                Vec2::new(raw.x.signum(), 0.0)
            } else if near_v(raw, band) {
                Vec2::new(0.0, raw.y.signum())
            } else if corner
                && (near_h(raw, FREE_DIR_AXIS_SNAP_DEG)
                    || near_v(raw, FREE_DIR_AXIS_SNAP_DEG))
            {
                // Ambiguous band: defer to the bias-free direction beyond
                // the rounding zone — near-axis there means the axis was
                // real, clearly diagonal there means the stroke is.
                let d = beyond().unwrap_or(raw);
                if near_h(d, CORNER_AXIS_SNAP_DEG) {
                    Vec2::new(d.x.signum(), 0.0)
                } else if near_v(d, CORNER_AXIS_SNAP_DEG) {
                    Vec2::new(0.0, d.y.signum())
                } else {
                    d
                }
            } else {
                raw
            }
        }
    }
}

/// Constrained cubic fit, graphics-gems style: tangent DIRECTIONS are
/// fixed (`start_tangent` = travel at start, `end_tangent` = backward at
/// end), only the two handle lengths are solved (least squares + Newton
/// reparameterization). Fixed tangents make the fit robust for type work
/// and give exactly H/V handles at extrema by construction.
pub(in crate::pipeline::vectorize) fn constrained_cubic_fit(
    samples: &[Point],
    start_tangent: Vec2,
    end_tangent: Vec2,
) -> ([Point; 4], f64) {
    let n = samples.len();
    let p0 = samples[0];
    let p3 = samples[n - 1];
    let chord = (p3 - p0).hypot();

    // Chord-length parameterization.
    let mut ts = Vec::with_capacity(n);
    let mut acc = 0.0;
    ts.push(0.0);
    for i in 1..n {
        acc += (samples[i] - samples[i - 1]).hypot();
        ts.push(acc);
    }
    let total = acc.max(1e-12);
    for t in &mut ts {
        *t /= total;
    }

    let bez = |a: f64, b: f64, t: f64| -> Point {
        let mt = 1.0 - t;
        let b0 = mt * mt * mt;
        let b1 = 3.0 * mt * mt * t;
        let b2 = 3.0 * mt * t * t;
        let b3 = t * t * t;
        let p1 = p0 + start_tangent * a;
        let p2 = p3 + end_tangent * b;
        Point::new(
            b0 * p0.x + b1 * p1.x + b2 * p2.x + b3 * p3.x,
            b0 * p0.y + b1 * p1.y + b2 * p2.y + b3 * p3.y,
        )
    };

    let mut alpha = chord / 3.0;
    let mut beta = chord / 3.0;
    for _round in 0..3 {
        // Linear least squares for (alpha, beta) at fixed parameters.
        let (mut xx, mut xy, mut yy, mut xd, mut yd) =
            (0.0, 0.0, 0.0, 0.0, 0.0);
        for (i, &t) in ts.iter().enumerate() {
            let mt = 1.0 - t;
            let b1 = 3.0 * mt * mt * t;
            let b2 = 3.0 * mt * t * t;
            let base = Point::new(
                (mt * mt * mt + b1) * p0.x + (b2 + t * t * t) * p3.x,
                (mt * mt * mt + b1) * p0.y + (b2 + t * t * t) * p3.y,
            );
            let x = start_tangent * b1;
            let y = end_tangent * b2;
            let d = samples[i] - base;
            xx += x.dot(x);
            xy += x.dot(y);
            yy += y.dot(y);
            xd += x.dot(d);
            yd += y.dot(d);
        }
        let det = xx * yy - xy * xy;
        if det.abs() > 1e-12 {
            alpha = (xd * yy - yd * xy) / det;
            beta = (xx * yd - xy * xd) / det;
        }
        let min_len = (chord * 0.02).max(1e-6);
        let max_len = chord * 1.2;
        alpha = alpha.clamp(min_len, max_len);
        beta = beta.clamp(min_len, max_len);

        // Newton reparameterization toward nearest points.
        for (i, t) in ts.iter_mut().enumerate() {
            let tt = *t;
            let p = bez(alpha, beta, tt);
            let eps = 1e-4;
            let dp = (bez(alpha, beta, (tt + eps).min(1.0))
                - bez(alpha, beta, (tt - eps).max(0.0)))
                * (1.0 / (2.0 * eps));
            let diff = p - samples[i];
            let denom = dp.dot(dp);
            if denom > 1e-12 {
                *t = (tt - diff.dot(dp) / denom).clamp(0.0, 1.0);
            }
        }
        ts[0] = 0.0;
        let last = ts.len() - 1;
        ts[last] = 1.0;
    }

    let p1 = p0 + start_tangent * alpha;
    let p2 = p3 + end_tangent * beta;
    let max_err = ts
        .iter()
        .enumerate()
        .map(|(i, &t)| (bez(alpha, beta, t) - samples[i]).hypot())
        .fold(0.0, f64::max);
    ([p0, p1, p2, p3], max_err)
}

/// Try to fit a failing section as two constrained cubics split at its
/// dominant curvature-sign change. Returns None when no usable inflection
/// exists or the halves still miss the tolerance.
pub(in crate::pipeline::vectorize) fn fit_split_at_inflection(
    samples: &[Point],
    start_tangent: Vec2,
    end_tangent: Vec2,
    tol: f64,
) -> Option<([[Point; 4]; 2], [bool; 2])> {
    let n = samples.len();
    if n < 24 {
        return None;
    }
    // Smoothed signed turn per sample (curvature proxy).
    let radius = 6usize;
    let kappa: Vec<f64> = (0..n)
        .map(|i| {
            let lo = i.saturating_sub(radius);
            let hi = (i + radius).min(n - 1);
            let a = samples[lo];
            let m = samples[i];
            let b = samples[hi];
            let u = m - a;
            let w = b - m;
            u.cross(w).atan2(u.dot(w))
        })
        .collect();
    // Best zero crossing: maximize the smaller |kappa| swing on each side.
    let margin = 8usize;
    let mut best: Option<(usize, f64)> = None;
    for k in margin..n - margin {
        if kappa[k] * kappa[k + 1] < 0.0 {
            let left = kappa[..=k].iter().map(|v| v.abs()).fold(0.0, f64::max);
            let right =
                kappa[k + 1..].iter().map(|v| v.abs()).fold(0.0, f64::max);
            let strength = left.min(right);
            if best.is_none() || strength > best.unwrap().1 {
                best = Some((k, strength));
            }
        }
    }
    let (k, _) = best?;
    // Free tangent at the split, shared by both halves for G1.
    let w = 6.min(k).min(n - 1 - k);
    let dir = (samples[k + w] - samples[k - w]).normalize();

    // A flank too short for a meaningful cubic becomes a LINE, the long
    // piece's tangent constrained to it — the corner + short line + curve
    // structure a designer draws at a crotch.
    let min_piece = 18usize;
    if k < min_piece
        && chord_deviation_pts(&samples[..=k]) <= SHORT_STRAIGHT_MAX_DEVIATION
    {
        let line_dir = (samples[k] - samples[0]).normalize();
        let (right_seg, right_err) =
            constrained_cubic_fit(&samples[k..], line_dir, end_tangent);
        if right_err <= tol {
            return Some((
                [line_seg(samples[0], samples[k]), right_seg],
                [true, false],
            ));
        }
    }
    if n - 1 - k < min_piece
        && chord_deviation_pts(&samples[k..]) <= SHORT_STRAIGHT_MAX_DEVIATION
    {
        let line_dir = (samples[n - 1] - samples[k]).normalize();
        let (left_seg, left_err) =
            constrained_cubic_fit(&samples[..=k], start_tangent, -line_dir);
        if left_err <= tol {
            return Some((
                [left_seg, line_seg(samples[k], samples[n - 1])],
                [false, true],
            ));
        }
    }

    let (left_seg, left_err) =
        constrained_cubic_fit(&samples[..=k], start_tangent, -dir);
    let (right_seg, right_err) =
        constrained_cubic_fit(&samples[k..], dir, end_tangent);
    if left_err.max(right_err) <= tol {
        Some(([left_seg, right_seg], [false, false]))
    } else {
        None
    }
}
