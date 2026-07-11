// Copyright 2026 the img2bez Authors
// SPDX-License-Identifier: Apache-2.0 OR MIT

//! Stages 1-3 of tracing: resampling, smoothing, and structure planning.

use crate::pipeline::vectorize::subpixel::SubpixelContour;

use super::corners::{
    count_turn_spikes, detect_corners, detect_round_corner_brackets,
};
use super::runs::{detect_straight_runs, drop_arc_chain_runs};
use super::{
    CURVATURE_SIGMA, ContourPlan, EXTREMUM_PROMINENCE,
    EXTREMUM_PROMINENCE_FRAC, EXTREMUM_REFINE_WINDOW, EXTREMUM_SPACING_FRAC,
    EXTREMUM_STANDOFF, INFLECTION_BOUNDARY_MARGIN, INFLECTION_MAX_RADIUS,
    INFLECTION_MIN_SIDE_TURN_DEG, INFLECTION_REACH, INFLECTION_STANDOFF,
    INFLECTION_STANDOFF_FRAC, MAX_MEAN_ABS_TURN, MIN_SAMPLES_PER_CORNER,
    PlanOutcome, SAMPLE_SPACING, SMOOTH_SIGMA, SPLIT_SNAP_SAMPLES, Split,
    SplitKind,
};

/// Stages 1-3 of `trace_contour`: resample, adaptively smooth, and decide the
/// structure (corners, straight runs, extrema, inflections).
pub(in crate::pipeline::vectorize) fn plan_contour(
    contour: &SubpixelContour,
    smoothing: f64,
    corner_deg: f64,
    corner_smear: bool,
    soft_source: bool,
    raster: Option<&crate::pipeline::vectorize::refine::RasterTarget>,
) -> PlanOutcome {
    let resampled = resample_closed(&contour.points, SAMPLE_SPACING);
    let n = resampled.len();
    if n < 12 {
        return PlanOutcome::TooSmall;
    }
    // Stage 1: adaptive smoothing — escalate sigma until the boundary
    // reads as clean type geometry. Clamp: `smoothing` is a public field;
    // zero would NaN the kernel, huge would overflow it.
    let mut sigma = (SMOOTH_SIGMA * smoothing).clamp(0.05, 50.0);
    let mut smoothed = gaussian_smooth_closed(&resampled, sigma);
    let mut turns = vertex_turns(&smoothed);
    for _ in 0..4 {
        let mean_turn = turns.iter().map(|t| t.abs()).sum::<f64>() / n as f64;
        if mean_turn <= MAX_MEAN_ABS_TURN
            && count_turn_spikes(&turns, n, corner_deg)
                <= n / MIN_SAMPLES_PER_CORNER
        {
            break;
        }
        sigma *= 1.8;
        smoothed = gaussian_smooth_closed(&resampled, sigma);
        turns = vertex_turns(&smoothed);
    }
    let corners =
        detect_corners(&turns, &smoothed, n, corner_deg, sigma, corner_smear);
    // Soft photographic sources keep their tight ink rounds smooth: the
    // rounding is wear, not a designed vertex.
    let brackets = if soft_source {
        Vec::new()
    } else {
        detect_round_corner_brackets(
            &turns, &smoothed, &corners, n, corner_deg, sigma, raster,
        )
    };
    let runs = detect_straight_runs(&smoothed, &turns, &corners, n);
    let runs = drop_arc_chain_runs(runs, &smoothed, &turns, n);

    // Stage 3: assemble splits. Run endpoints snap onto nearby existing
    // splits so corner-rounding zones don't produce curve slivers.
    let mut splits: Vec<Split> = corners
        .iter()
        .map(|&i| Split {
            idx: i,
            kind: SplitKind::Corner,
        })
        .collect();
    splits.extend(brackets);
    let mut line_sections: Vec<(usize, usize)> = Vec::new();
    for &(a, b) in &runs {
        let a = snap_to_existing(a, &splits, n);
        let b = snap_to_existing(b, &splits, n);
        if a == b {
            continue;
        }
        for idx in [a, b] {
            if !splits.iter().any(|s| s.idx == idx) {
                splits.push(Split {
                    idx,
                    kind: SplitKind::Tangent,
                });
            }
        }
        line_sections.push((a, b));
    }
    splits.sort_by_key(|s| s.idx);
    splits.dedup_by_key(|s| s.idx);

    // Stage 3 (continued): extrema + inflections inside curved pieces.
    let base = splits.clone();
    let mut extra: Vec<Split> = Vec::new();
    let pieces: Vec<(usize, usize)> = if base.is_empty() {
        // Whole contour: scan twice with a half-rotation so an extremum
        // at the contour origin is not lost in the scan margins; the
        // standoff merge dedupes.
        vec![(0, n), (n / 2, n)]
    } else {
        (0..base.len())
            .map(|i| {
                let a = base[i].idx;
                let b = base[(i + 1) % base.len()].idx;
                let len = (b + n - a) % n;
                (a, if len == 0 { n } else { len })
            })
            .collect()
    };
    for &(start, len) in &pieces {
        if line_sections
            .iter()
            .any(|&(a, b)| a == start && (b + n - a) % n == len % n)
        {
            continue; // straight piece: no interior points
        }
        extra.extend(find_extrema(&smoothed, &turns, start, len, n));
        extra.extend(find_inflections(&turns, &smoothed, start, len, n));
    }
    for s in extra {
        let clear = !splits.iter().any(|q| {
            let d = (s.idx + n - q.idx) % n;
            let d = d.min(n - d);
            let standoff = match (s.kind, q.kind) {
                (SplitKind::Inflection, _) => INFLECTION_STANDOFF
                    .max((n as f64 * INFLECTION_STANDOFF_FRAC) as usize),
                // An "extremum" next to a straight run is line edge
                // noise; one next to a corner is real structure.
                (_, SplitKind::Tangent) => SPLIT_SNAP_SAMPLES,
                // Extremum vs extremum: scale the spacing with the ring.
                (
                    SplitKind::ExtremumX | SplitKind::ExtremumY,
                    SplitKind::ExtremumX | SplitKind::ExtremumY,
                ) => EXTREMUM_STANDOFF
                    .max((n as f64 * EXTREMUM_SPACING_FRAC) as usize),
                _ => EXTREMUM_STANDOFF,
            };
            d <= standoff
        });
        if clear {
            splits.push(s);
        }
    }
    splits.sort_by_key(|s| s.idx);

    if std::env::var("IMG2BEZ_DEBUG_FIT").is_ok() {
        let mean_turn = turns.iter().map(|t| t.abs()).sum::<f64>() / n as f64;
        eprintln!(
            "  fit: contour n={} splits={} ({} corners, {} runs) \
             sigma={:.1} mean_turn={:.3}",
            n,
            splits.len(),
            splits
                .iter()
                .filter(|s| s.kind == SplitKind::Corner)
                .count(),
            line_sections.len(),
            sigma,
            mean_turn,
        );
        for s in &splits {
            eprintln!(
                "    split[{}] = ({:.1}, {:.1}) {:?}",
                s.idx, smoothed[s.idx].0, smoothed[s.idx].1, s.kind
            );
        }
        for &(a, b) in &line_sections {
            eprintln!("    line_section {}..{}", a, b);
        }
    }

    if splits.is_empty() {
        return PlanOutcome::NoSplits { smoothed };
    }

    PlanOutcome::Plan(ContourPlan {
        smoothed,
        splits,
        line_sections,
    })
}

// ── Stage 1: resampling and smoothing ────────────────────────────────

/// Resample a closed polyline at uniform arc-length spacing.
fn resample_closed(points: &[(f64, f64)], spacing: f64) -> Vec<(f64, f64)> {
    let n = points.len();
    let mut cumulative = Vec::with_capacity(n + 1);
    cumulative.push(0.0);
    let mut total = 0.0;
    for i in 0..n {
        let (x0, y0) = points[i];
        let (x1, y1) = points[(i + 1) % n];
        total += ((x1 - x0).powi(2) + (y1 - y0).powi(2)).sqrt();
        cumulative.push(total);
    }
    if total < spacing * 4.0 {
        return points.to_vec();
    }
    let count = (total / spacing).round() as usize;
    let step = total / count as f64;
    let mut out = Vec::with_capacity(count);
    let mut seg = 0usize;
    for k in 0..count {
        let target = k as f64 * step;
        while cumulative[seg + 1] < target && seg + 1 < n {
            seg += 1;
        }
        let seg_len = cumulative[seg + 1] - cumulative[seg];
        let t = if seg_len > 1e-12 {
            (target - cumulative[seg]) / seg_len
        } else {
            0.0
        };
        let (x0, y0) = points[seg];
        let (x1, y1) = points[(seg + 1) % n];
        out.push((x0 + t * (x1 - x0), y0 + t * (y1 - y0)));
    }
    out
}

/// Gaussian-smooth a closed polyline (sigma in samples, cyclic kernel).
fn gaussian_smooth_closed(
    points: &[(f64, f64)],
    sigma: f64,
) -> Vec<(f64, f64)> {
    let n = points.len();
    let radius = (sigma * 3.0).ceil() as i64;
    let mut kernel = Vec::with_capacity((2 * radius + 1) as usize);
    let mut sum = 0.0;
    for k in -radius..=radius {
        let w = (-(k as f64).powi(2) / (2.0 * sigma * sigma)).exp();
        kernel.push(w);
        sum += w;
    }
    for w in &mut kernel {
        *w /= sum;
    }
    (0..n)
        .map(|i| {
            let mut x = 0.0;
            let mut y = 0.0;
            for (ki, k) in (-radius..=radius).enumerate() {
                let j = ((i as i64 + k).rem_euclid(n as i64)) as usize;
                x += kernel[ki] * points[j].0;
                y += kernel[ki] * points[j].1;
            }
            (x, y)
        })
        .collect()
}

/// Per-vertex turn angles (radians) of a closed polyline.
pub(in crate::pipeline::vectorize) fn vertex_turns(
    smoothed: &[(f64, f64)],
) -> Vec<f64> {
    let n = smoothed.len();
    (0..n)
        .map(|i| {
            let p = smoothed[(i + n - 1) % n];
            let c = smoothed[i];
            let nx = smoothed[(i + 1) % n];
            let u = (c.0 - p.0, c.1 - p.1);
            let w = (nx.0 - c.0, nx.1 - c.1);
            (u.0 * w.1 - u.1 * w.0).atan2(u.0 * w.0 + u.1 * w.1)
        })
        .collect()
}

/// Snap `idx` to the nearest existing split within SPLIT_SNAP_SAMPLES,
/// or return it unchanged.
fn snap_to_existing(idx: usize, splits: &[Split], n: usize) -> usize {
    splits
        .iter()
        .map(|s| {
            let d = (idx + n - s.idx) % n;
            (d.min(n - d), s.idx)
        })
        .filter(|&(d, _)| d <= SPLIT_SNAP_SAMPLES)
        .min_by_key(|&(d, _)| d)
        .map(|(_, idx)| idx)
        .unwrap_or(idx)
}

/// Local x/y extrema with sufficient prominence inside a cyclic piece.
fn find_extrema(
    smoothed: &[(f64, f64)],
    turns: &[f64],
    start: usize,
    len: usize,
    n: usize,
) -> Vec<Split> {
    let mut out = Vec::new();
    if len < 2 * SPLIT_SNAP_SAMPLES + 2 {
        return out;
    }
    // Scale-relative prominence: the noise-vs-structure separator.
    let _ = turns; // (kept for future gating experiments)
    let (mut minx, mut miny, mut maxx, mut maxy) = (
        f64::INFINITY,
        f64::INFINITY,
        f64::NEG_INFINITY,
        f64::NEG_INFINITY,
    );
    for &(x, y) in smoothed {
        minx = minx.min(x);
        miny = miny.min(y);
        maxx = maxx.max(x);
        maxy = maxy.max(y);
    }
    let diag = ((maxx - minx).powi(2) + (maxy - miny).powi(2)).sqrt();
    let prominence = (diag * EXTREMUM_PROMINENCE_FRAC).max(EXTREMUM_PROMINENCE);
    for axis in 0..2 {
        let coord = |k: usize| -> f64 {
            let p = smoothed[(start + k) % n];
            if axis == 0 { p.0 } else { p.1 }
        };
        // Candidates: derivative sign flips, plateaus collapsed to their
        // midpoint.
        let mut prev_sign = 0.0f64;
        let mut prev_end = 0usize; // sample index ending the last nonzero diff
        for k in 1..len {
            let d = coord(k) - coord(k - 1);
            let s = d.signum();
            if s == 0.0 {
                continue;
            }
            if prev_sign != 0.0 && s != prev_sign {
                let candidate = (prev_end + k - 1) / 2;
                if candidate >= EXTREMUM_STANDOFF
                    && candidate + EXTREMUM_STANDOFF < len
                {
                    let v = coord(candidate);
                    let is_max = prev_sign > 0.0;
                    let recede =
                        |delta: f64| if is_max { delta } else { -delta };
                    let side_ok =
                        |indices: &mut dyn Iterator<Item = usize>| -> bool {
                            for q in indices {
                                let drop = recede(v - coord(q));
                                if drop < -0.05 {
                                    return false; // more extreme point first
                                }
                                if drop >= prominence {
                                    return true;
                                }
                            }
                            // Receded monotonically into the piece
                            // boundary: real even if the scan was cut short.
                            true
                        };
                    if side_ok(&mut (0..candidate).rev())
                        && side_ok(&mut ((candidate + 1)..len))
                    {
                        // Parabola-fit localization: robust on locally
                        // flat extremes, centered as a designer would.
                        let lo =
                            candidate.saturating_sub(EXTREMUM_REFINE_WINDOW);
                        let hi =
                            (candidate + EXTREMUM_REFINE_WINDOW).min(len - 1);
                        // Clamp clear of the piece ends: refinement must
                        // not push a real extremum into the standoff zone.
                        let lo_clear = lo.max(EXTREMUM_STANDOFF + 1);
                        let hi_clear =
                            hi.min(len.saturating_sub(EXTREMUM_STANDOFF + 2));
                        let refined = parabola_vertex(&|q| coord(q), lo, hi)
                            .map(|s| {
                                (s.round() as usize)
                                    .clamp(lo_clear, hi_clear.max(lo_clear))
                            })
                            .filter(|&s| s >= lo && s <= hi)
                            .unwrap_or(candidate);
                        out.push(Split {
                            idx: (start + refined) % n,
                            kind: if axis == 0 {
                                SplitKind::ExtremumX
                            } else {
                                SplitKind::ExtremumY
                            },
                        });
                    }
                }
            }
            prev_sign = s;
            prev_end = k;
        }
    }
    out
}

/// Inflection points: persistent curvature sign changes.
fn find_inflections(
    turns: &[f64],
    smoothed: &[(f64, f64)],
    start: usize,
    len: usize,
    n: usize,
) -> Vec<Split> {
    if len < 2 * INFLECTION_REACH {
        return Vec::new();
    }
    // Smoothed curvature (turn per sample) over the piece.
    let radius = (CURVATURE_SIGMA * 3.0).ceil() as i64;
    let kappa: Vec<f64> = (0..len)
        .map(|k| {
            let mut s = 0.0;
            let mut wsum = 0.0;
            for q in -radius..=radius {
                let w = (-(q as f64).powi(2)
                    / (2.0 * CURVATURE_SIGMA * CURVATURE_SIGMA))
                    .exp();
                let idx =
                    (start as i64 + k as i64 + q).rem_euclid(n as i64) as usize;
                s += w * turns[idx];
                wsum += w;
            }
            s / wsum
        })
        .collect();
    let threshold = SAMPLE_SPACING / INFLECTION_MAX_RADIUS;
    // Keep the reach scans away from the piece ends: corner-rounding
    // tails carry curvature that would satisfy the reach test for a
    // crossing with no genuine curve behind it.
    let interior_lo = INFLECTION_BOUNDARY_MARGIN.min(len / 4);
    let interior_hi =
        len.saturating_sub(INFLECTION_BOUNDARY_MARGIN.min(len / 4));
    let mut out = Vec::new();
    let mut k = SPLIT_SNAP_SAMPLES;
    while k + 1 + SPLIT_SNAP_SAMPLES < len {
        if kappa[k] * kappa[k + 1] < 0.0 {
            let back_start =
                k.saturating_sub(INFLECTION_REACH).max(interior_lo);
            let reach_back = back_start <= k
                && kappa[back_start..=k].iter().any(|&v| {
                    v.abs() >= threshold && v.signum() == kappa[k].signum()
                });
            let fwd_end =
                (k + 1 + INFLECTION_REACH).min(len - 1).min(interior_hi);
            let reach_fwd = k < fwd_end
                && kappa[k + 1..=fwd_end].iter().any(|&v| {
                    v.abs() >= threshold && v.signum() == kappa[k + 1].signum()
                });
            // Tightness alone is fakeable by a small bump; integrated
            // turn is not — each side must accumulate real degrees.
            let min_side = INFLECTION_MIN_SIDE_TURN_DEG.to_radians();
            let net_back: f64 = if back_start <= k {
                kappa[back_start..=k].iter().sum()
            } else {
                0.0
            };
            let net_fwd: f64 = if k < fwd_end {
                kappa[k + 1..=fwd_end].iter().sum()
            } else {
                0.0
            };
            let integrated = net_back.signum() == kappa[k].signum()
                && net_fwd.signum() == kappa[k + 1].signum()
                && net_back.abs() >= min_side
                && net_fwd.abs() >= min_side;
            if reach_back
                && reach_fwd
                && std::env::var("IMG2BEZ_DEBUG_FIT").is_ok()
            {
                eprintln!(
                    "    infl-cand k={k} net_back={:.1}° net_fwd={:.1}° -> {}",
                    net_back.to_degrees(),
                    net_fwd.to_degrees(),
                    if integrated { "KEEP" } else { "drop" }
                );
            }
            if reach_back && reach_fwd && crate::ml::mldata::enabled() {
                // Peak same-sign curvature per side, as a multiple of the
                // reach threshold.
                let peak = |lo: usize, hi: usize, sign: f64| -> f64 {
                    kappa[lo..=hi]
                        .iter()
                        .filter(|v| v.signum() == sign)
                        .fold(0.0f64, |m, v| m.max(v.abs()))
                        / threshold
                };
                crate::ml::mldata::log(
                    "inflection",
                    Some(smoothed[(start + k) % n]),
                    &[
                        ("net_back_deg", net_back.to_degrees()),
                        ("net_fwd_deg", net_fwd.to_degrees()),
                        ("peak_back", peak(back_start, k, kappa[k].signum())),
                        (
                            "peak_fwd",
                            peak(k + 1, fwd_end, kappa[k + 1].signum()),
                        ),
                        ("piece_len", len as f64),
                        ("pos_frac", k as f64 / len.max(1) as f64),
                    ],
                    if integrated { "point" } else { "none" },
                );
            }
            if reach_back && reach_fwd && integrated {
                out.push(Split {
                    idx: (start + k) % n,
                    kind: SplitKind::Inflection,
                });
                k += INFLECTION_REACH;
                continue;
            }
        }
        k += 1;
    }
    out
}

/// Least-squares parabola vertex of f(q) over q in [lo, hi]; used to
/// localize extrema (`find_extrema`) robustly on locally flat curves.
fn parabola_vertex(
    f: &dyn Fn(usize) -> f64,
    lo: usize,
    hi: usize,
) -> Option<f64> {
    let m = hi - lo + 1;
    if m < 5 {
        return None;
    }
    // Fit y = a q^2 + b q + c with q centered for conditioning.
    let mid = (lo + hi) as f64 / 2.0;
    let (mut s0, mut s1, mut s2, mut s3, mut s4) = (0.0, 0.0, 0.0, 0.0, 0.0);
    let (mut t0, mut t1, mut t2) = (0.0, 0.0, 0.0);
    for q in lo..=hi {
        let x = q as f64 - mid;
        let y = f(q);
        let x2 = x * x;
        s0 += 1.0;
        s1 += x;
        s2 += x2;
        s3 += x2 * x;
        s4 += x2 * x2;
        t0 += y;
        t1 += x * y;
        t2 += x2 * y;
    }
    // Solve the 3x3 normal equations via Cramer's rule.
    let det = s4 * (s2 * s0 - s1 * s1) - s3 * (s3 * s0 - s1 * s2)
        + s2 * (s3 * s1 - s2 * s2);
    if det.abs() < 1e-12 {
        return None;
    }
    let a = (t2 * (s2 * s0 - s1 * s1) - s3 * (t1 * s0 - s1 * t0)
        + s2 * (t1 * s1 - s2 * t0))
        / det;
    let b = (s4 * (t1 * s0 - t0 * s1) - t2 * (s3 * s0 - s1 * s2)
        + s2 * (s3 * t0 - s2 * t1))
        / det;
    if a.abs() < 1e-12 {
        return None;
    }
    Some(-b / (2.0 * a) + mid)
}
