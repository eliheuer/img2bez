// Copyright 2026 the img2bez Authors
// SPDX-License-Identifier: Apache-2.0 OR MIT

//! Stage 2: corner and tight-vertex (bracket) detection.

use super::{
    CORNER_CONCENTRATION, CORNER_CUSP_TURN_DEG, CORNER_MIN_TOTAL_TURN_DEG,
    CORNER_SHALLOW_FLANK_MAX_DEG, CORNER_SHALLOW_MIN_TOTAL_TURN_DEG,
    CORNER_WINDOW, Split, SplitKind,
};

// ── Stage 2: feature detection ───────────────────────────────────────

/// `i - off` on the cyclic contour: sigma-scaled windows can exceed `n`
/// on a small contour and underflow `(i + n - off) % n`, so reduce first.
fn cyc_sub(i: usize, off: usize, n: usize) -> usize {
    (i + n - off % n) % n
}

/// A TIGHT VERTEX is a designed corner's worth of turn packed into a
/// compact arc: the spike passes the central threshold but concentration
/// fails, and the fitter smooths straight through, deflating the shape.
/// References draw ONE point there (turn-weighted apex).
const BRACKET_MIN_TOTAL_TURN_DEG: f64 = 45.0;
const BRACKET_MAX_ZONE_SAMPLES: usize = 140;
const BRACKET_CAND_MERGE_GAP: usize = 12;
/// The zone's effective radius (arc / total turn) must be genuinely
/// tight: a noise spike borrows a bowl's turn but implies a large radius
/// (~30px+), a real dot vertex or counter cusp reads under ~14px.
const BRACKET_MAX_RADIUS_PX: f64 = 18.0;

/// Detect tight-vertex zones the concentration gate missed and emit one
/// corner split at each zone's turn-weighted apex.
pub(super) fn detect_round_corner_brackets(
    turns: &[f64],
    smoothed: &[(f64, f64)],
    corners: &[usize],
    n: usize,
    corner_deg: f64,
    sigma: f64,
    raster: Option<&crate::pipeline::vectorize::refine::RasterTarget>,
) -> Vec<Split> {
    let central_min = corner_deg.to_radians();
    let c_half = 1usize.max((sigma / 1.2).round() as usize);
    let half = CORNER_WINDOW.max((3.0 * sigma).ceil() as usize);
    let span = |i: usize, h: usize| -> f64 {
        let mut s = 0.0;
        for k in 0..(2 * h + 1) {
            s += turns[cyc_sub(i + k, h, n)];
        }
        s
    };
    let cyc = |a: usize, b: usize| {
        let d = (a + n - b) % n;
        d.min(n - d)
    };
    // Candidates: central spike, near no kept corner.
    let cands: Vec<usize> = (0..n)
        .filter(|&i| {
            span(i, c_half).abs() >= central_min
                && corners.iter().all(|&c| cyc(i, c) > 2 * half)
        })
        .collect();
    if cands.is_empty() {
        return Vec::new();
    }
    let debug = std::env::var("IMG2BEZ_DEBUG_CORNERS").is_ok();
    // Convex features turn the same way as the contour's net 360; convex
    // tight vertices are always pointed, concave ones differ by style.
    let net: f64 = turns.iter().sum();
    let mut out = Vec::new();
    let mut zi = 0usize;
    while zi < cands.len() {
        let start = cands[zi];
        let mut end = start;
        let mut zj = zi + 1;
        while zj < cands.len()
            && (cands[zj] + n - end) % n <= BRACKET_CAND_MERGE_GAP
        {
            end = cands[zj];
            zj += 1;
        }
        let zone_len = (end + n - start) % n;
        // Total signed turn across the zone plus its smoothing tails.
        let mut total = 0.0f64;
        let mut k = cyc_sub(start, half, n);
        let steps = zone_len + 2 * half + 1;
        for _ in 0..steps.min(n) {
            total += turns[k];
            k = (k + 1) % n;
        }
        // All spikes must turn the same way as the zone total: wobble
        // alternates, a designed vertex does not.
        let sign_ok =
            (zi..zj).all(|q| span(cands[q], c_half).signum() == total.signum());
        let arc = (zone_len + 2 * half) as f64;
        let r_eff = arc / total.abs().max(1e-9);
        // A notch and a spike are locally identical in turn sequence;
        // the raster tells them apart at the BASE: ink at the chord
        // midpoint means a convex vertex (always pointed), background a
        // white wedge (style-dependent, left smooth).
        let (base_ink, probe_dbg) = {
            // Walls well beyond the arc so the midpoint sits inside the
            // feature, not the apex rounding.
            let woff = 3 * half;
            let (sx, sy) = smoothed[cyc_sub(start, woff, n)];
            let (ex, ey) = smoothed[(end + woff) % n];
            let (cx, cy) = ((sx + ex) / 2.0, (sy + ey) / 2.0);
            match raster {
                Some(rt) => {
                    let cov = rt.coverage(cx, cy);
                    (cov >= 0.5, format!("base=({cx:.0},{cy:.0}) cov={cov:.2}"))
                }
                None => (false, "no-raster".to_string()),
            }
        };
        let qualifies = zone_len <= BRACKET_MAX_ZONE_SAMPLES
            && total.abs() >= BRACKET_MIN_TOTAL_TURN_DEG.to_radians()
            && sign_ok
            && r_eff <= BRACKET_MAX_RADIUS_PX
            && total.signum() == net.signum()
            && base_ink;
        if debug {
            let (x, y) = smoothed[(start + zone_len / 2) % n];
            eprintln!(
                "    bracket-zone at ({x:.1},{y:.1}) len={zone_len} total={:.1}° r={:.1}px sign_ok={} {} ink={} -> {}",
                total.to_degrees(),
                r_eff,
                sign_ok,
                probe_dbg,
                base_ink,
                if qualifies { "VERTEX" } else { "skip" }
            );
        }
        if qualifies {
            // Turn-weighted apex of the zone.
            let mut acc = 0.0f64;
            let mut wsum = 0.0f64;
            for off in 0..=zone_len {
                let idx = (start + off) % n;
                let w = turns[idx].abs();
                acc += off as f64 * w;
                wsum += w;
            }
            let apex = (start + (acc / wsum.max(1e-9)).round() as usize) % n;
            out.push(Split {
                idx: apex,
                kind: SplitKind::Corner,
            });
            if crate::ml::mldata::enabled() {
                let (x, y) = smoothed[(start + zone_len / 2) % n];
                crate::ml::mldata::log(
                    "bracket",
                    Some((x, y)),
                    &[
                        ("zone_len", zone_len as f64),
                        ("total_deg", total.to_degrees()),
                        ("sigma", sigma),
                    ],
                    "bracket",
                );
            }
        }
        zi = zj;
    }
    out
}

/// Count turn spikes ignoring the designed-corner gates: the NOISE probe
/// for adaptive smoothing (a stair-stepped boundary is dense with spikes;
/// real glyphs have a corner every 50+ samples).
pub(super) fn count_turn_spikes(
    turns: &[f64],
    n: usize,
    corner_deg: f64,
) -> usize {
    let central_min = corner_deg.to_radians();
    // Count clusters, with a tight merge gap so wobble noise counts each
    // spike and drives escalation.
    const SPIKE_MERGE_GAP: usize = 2;
    let mut clusters = 0usize;
    let mut last: Option<usize> = None;
    let mut first: Option<usize> = None;
    for i in 0..n {
        let c = turns[(i + n - 1) % n] + turns[i] + turns[(i + 1) % n];
        if c.abs() < central_min {
            continue;
        }
        if first.is_none() {
            first = Some(i);
        }
        if match last {
            Some(l) => i - l > SPIKE_MERGE_GAP,
            None => true,
        } {
            clusters += 1;
        }
        last = Some(i);
    }
    // Cyclic wrap: the first and last clusters may be the same feature.
    if clusters >= 2
        && let (Some(f), Some(l)) = (first, last)
        && (f + n - l) % n <= 2
    {
        clusters -= 1;
    }
    clusters
}

/// Detect corners: central turn over threshold, window turn at a
/// designed-corner angle, central turn dominating the window; one vertex
/// per cluster by non-max suppression. The window widens with sigma so
/// heavier smoothing still collects the full turn.
pub(super) fn detect_corners(
    turns: &[f64],
    smoothed: &[(f64, f64)],
    n: usize,
    corner_deg: f64,
    sigma: f64,
    corner_smear: bool,
) -> Vec<usize> {
    let central_min = corner_deg.to_radians();
    let total_min = CORNER_MIN_TOTAL_TURN_DEG.to_radians();
    // Sigma-scaled spans make the concentration test scale-invariant: a
    // smoothed corner keeps ~68% of its turn within ±sigma, a smooth
    // arc's central share stays near 1/3 at every sigma.
    let c_half = 1usize.max((sigma / 1.2).round() as usize);
    let half = CORNER_WINDOW.max((3.0 * sigma).ceil() as usize);
    let span = |i: usize, h: usize| -> f64 {
        let mut s = 0.0;
        for k in 0..(2 * h + 1) {
            s += turns[cyc_sub(i + k, h, n)];
        }
        s
    };
    let central = |i: usize| -> f64 { span(i, c_half) };
    let window = |i: usize| -> f64 { span(i, half) };
    // Signed turn of the outer band on each side of the window: quiet
    // (near-zero) flanks mean the break sits between straights.
    let flank = |i: usize, back: bool| -> f64 {
        (half + 1..=2 * half)
            .map(|q| {
                let idx = if back { cyc_sub(i, q, n) } else { (i + q) % n };
                turns[idx]
            })
            .sum::<f64>()
    };
    let debug = std::env::var("IMG2BEZ_DEBUG_CORNERS").is_ok();
    // Learned SITE head: scores each curvature zone against five classes
    // (corner / corner2 / bracket / point / smooth); corner+corner2 emit
    // Corner splits, point defers to the procedural gate, bracket flows
    // through the bracket pass. One row per zone, apex = largest central
    // turn, mirroring the trainer.
    if crate::ml::heads::site_head_enabled() {
        let mut cands: Vec<(usize, f64, f64)> = Vec::new();
        for i in 0..n {
            let c = central(i);
            if c.abs() >= central_min {
                cands.push((i, c, window(i)));
            }
        }
        let mut out: Vec<usize> = Vec::new();
        if cands.is_empty() {
            return out;
        }
        // Cluster candidates whose sample indices sit within the corner
        // window of their predecessor (cyclic wrap handled at the end).
        let mut zones: Vec<Vec<(usize, f64, f64)>> = vec![vec![cands[0]]];
        for &cd in &cands[1..] {
            let last = zones.last().unwrap().last().unwrap().0;
            if cd.0 - last <= CORNER_WINDOW {
                zones.last_mut().unwrap().push(cd);
            } else {
                zones.push(vec![cd]);
            }
        }
        if zones.len() >= 2 {
            let first_i = zones[0][0].0;
            let last_i = zones.last().unwrap().last().unwrap().0;
            if (first_i + n - last_i) % n <= CORNER_WINDOW {
                let first = zones.remove(0);
                zones.last_mut().unwrap().extend(first);
            }
        }
        for zone in &zones {
            let quiet = CORNER_SHALLOW_FLANK_MAX_DEG.to_radians();
            let mut span: f64 = 0.0;
            for a in zone {
                for b in zone {
                    let (ax, ay) = smoothed[a.0];
                    let (bx, by) = smoothed[b.0];
                    span = span
                        .max(((ax - bx).powi(2) + (ay - by).powi(2)).sqrt());
                }
            }
            let any_cusp = corner_smear
                && zone
                    .iter()
                    .any(|z| z.2.abs() >= CORNER_CUSP_TURN_DEG.to_radians());
            let any_shallow = zone.iter().any(|z| {
                z.2.abs() >= CORNER_SHALLOW_MIN_TOTAL_TURN_DEG.to_radians()
                    && flank(z.0, true).abs() <= quiet
                    && flank(z.0, false).abs() <= quiet
            });
            let feats = [
                zone.len() as f64,
                zone.iter()
                    .map(|z| z.1.abs())
                    .fold(0.0, f64::max)
                    .to_degrees(),
                zone.iter()
                    .map(|z| z.2.abs())
                    .fold(0.0, f64::max)
                    .to_degrees(),
                zone.iter()
                    .map(|z| z.1.abs() / z.2.abs().max(1e-9))
                    .fold(0.0, f64::max),
                span,
                sigma,
                if corner_smear { 1.0 } else { 0.0 },
                if any_cusp { 1.0 } else { 0.0 },
                if any_shallow { 1.0 } else { 0.0 },
            ];
            let scores = crate::ml::heads::site_scores(&feats);
            if debug {
                eprintln!(
                    "    site-zone at ({:.1},{:.1}) feats={:?} scores={:?}",
                    smoothed[zone[0].0].0, smoothed[zone[0].0].1, feats, scores
                );
            }
            let best = (0..5).max_by(|&a, &b| scores[a].total_cmp(&scores[b]));
            match best {
                Some(0) => {
                    let apex = zone
                        .iter()
                        .max_by(|a, b| a.1.abs().total_cmp(&b.1.abs()))
                        .unwrap()
                        .0;
                    out.push(apex);
                }
                Some(1) => {
                    // corner2: a designed corner PAIR in one zone. Place
                    // at the two strongest turn apexes (geometric
                    // extremes sit past their corners and get merged
                    // away); a one-bump zone falls back to one apex.
                    let apex1 = *zone
                        .iter()
                        .max_by(|a, b| a.1.abs().total_cmp(&b.1.abs()))
                        .unwrap();
                    let (ax, ay) = smoothed[apex1.0];
                    let min_d2 = (span * 0.4).powi(2);
                    let apex2 = zone
                        .iter()
                        .filter(|z| {
                            let (zx, zy) = smoothed[z.0];
                            (zx - ax).powi(2) + (zy - ay).powi(2) >= min_d2
                        })
                        .max_by(|a, b| a.1.abs().total_cmp(&b.1.abs()));
                    out.push(apex1.0);
                    if let Some(a2) = apex2
                        && a2.0 != apex1.0
                    {
                        out.push(a2.0);
                    }
                }
                Some(3) => {
                    // point: structure, just not a corner. The head has
                    // no smooth-split channel and emitting nothing
                    // deletes the structure, so defer to the procedural
                    // gate; downstream smoothing classifies it.
                    let quiet = CORNER_SHALLOW_FLANK_MAX_DEG.to_radians();
                    let apex = zone
                        .iter()
                        .filter(|z| {
                            let shallow_designed = z.2.abs()
                                >= CORNER_SHALLOW_MIN_TOTAL_TURN_DEG
                                    .to_radians()
                                && flank(z.0, true).abs() <= quiet
                                && flank(z.0, false).abs() <= quiet;
                            let cusp = corner_smear
                                && z.2.abs()
                                    >= CORNER_CUSP_TURN_DEG.to_radians();
                            (z.2.abs() >= total_min || shallow_designed)
                                && (cusp
                                    || z.1.abs()
                                        >= CORNER_CONCENTRATION * z.2.abs())
                        })
                        .max_by(|a, b| a.1.abs().total_cmp(&b.1.abs()));
                    if let Some(a) = apex {
                        out.push(a.0);
                    }
                }
                _ => {}
            }
        }
        out.sort_unstable();
        return out;
    }

    let mut candidates: Vec<usize> = (0..n)
        .filter(|&i| {
            let c = central(i);
            let w = window(i);
            let cusp = corner_smear
                && w.abs() >= CORNER_CUSP_TURN_DEG.to_radians();
            let quiet = CORNER_SHALLOW_FLANK_MAX_DEG.to_radians();
            let shallow_designed = w.abs()
                >= CORNER_SHALLOW_MIN_TOTAL_TURN_DEG.to_radians()
                && flank(i, true).abs() <= quiet
                && flank(i, false).abs() <= quiet;
            let pass = if crate::ml::heads::corner_head_enabled() {
                // The head only saw candidates above the minimal central
                // turn in training, so the procedural floor stays.
                c.abs() >= central_min
                    && crate::ml::heads::corner_score(&[
                        c.to_degrees().abs(),
                        w.to_degrees().abs(),
                        c.abs() / w.abs().max(1e-9),
                        sigma,
                        if corner_smear { 1.0 } else { 0.0 },
                        if cusp { 1.0 } else { 0.0 },
                        if shallow_designed { 1.0 } else { 0.0 },
                    ]) >= 0.0
            } else {
                c.abs() >= central_min
                    && (w.abs() >= total_min || shallow_designed)
                    && (cusp || c.abs() >= CORNER_CONCENTRATION * w.abs())
            };
            if debug && c.abs() >= central_min {
                eprintln!(
                    "    corner-cand i={i} central={:.1}° window={:.1}° conc={:.2} -> {}",
                    c.to_degrees(),
                    w.to_degrees(),
                    c.abs() / w.abs().max(1e-9),
                    if pass { "PASS" } else { "fail" }
                );
            }
            if crate::ml::mldata::enabled() && c.abs() >= central_min {
                crate::ml::mldata::log(
                    "corner",
                    Some(smoothed[i]),
                    &[
                        ("central_deg", c.to_degrees()),
                        ("window_deg", w.to_degrees()),
                        ("conc", c.abs() / w.abs().max(1e-9)),
                        ("sigma", sigma),
                        ("smear", if corner_smear { 1.0 } else { 0.0 }),
                        ("cusp", if cusp { 1.0 } else { 0.0 }),
                        (
                            "shallow_flanked",
                            if shallow_designed { 1.0 } else { 0.0 },
                        ),
                    ],
                    if pass { "corner" } else { "smooth" },
                );
            }
            pass
        })
        .collect();
    // Non-max suppression: within each cluster of nearby candidates keep
    // the sharpest vertex.
    candidates.sort_unstable();
    let mut keep: Vec<usize> = Vec::new();
    for &i in &candidates {
        if let Some(&last) = keep.last()
            && (i + n - last) % n <= CORNER_WINDOW
        {
            if turns[i].abs() > turns[last].abs() {
                *keep.last_mut().unwrap() = i;
            }
            continue;
        }
        keep.push(i);
    }
    // Cyclic wrap: first and last may be the same cluster.
    if keep.len() >= 2 {
        let first = keep[0];
        let last = *keep.last().unwrap();
        if (first + n - last) % n <= CORNER_WINDOW {
            if turns[first].abs() >= turns[last].abs() {
                keep.pop();
            } else {
                keep.remove(0);
            }
        }
    }
    keep
}
