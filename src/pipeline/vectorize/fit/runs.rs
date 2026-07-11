// Copyright 2026 the img2bez Authors
// SPDX-License-Identifier: Apache-2.0 OR MIT

//! Stage 2: straight-run detection and its anchoring filters.

use super::{
    ARC_FLAT_FLANK_WINDOW, ARC_FLAT_MAX_CHORD_FRAC, ARC_FLAT_MAX_FLANK_DEG,
    ARC_FLAT_MIN_FLANK_DEG, CORNER_FLAT_DEV_FLOOR, RUN_AXIS_MAX_DEG,
    RUN_EXTEND_MAX_SAMPLES, SPLIT_SNAP_SAMPLES, STRAIGHT_DEV_FLOOR,
    STRAIGHT_DEV_SLOPE, STRAIGHT_MIN_CHORD, STRAIGHT_MIN_CHORD_AT_CORNER,
    chord_deviation, dist,
};

/// Arc-chain veto: a gentle sweep passes the straightness envelope in
/// pieces, yielding a polyline of "straight" runs stepping same-sign a
/// few degrees per join — one continuous curve that kinks at every join.
/// Real straights meet neighbors through corners (corner-sized steps) and
/// wobble-fragmented straights step with alternating signs; so a chain of
/// MIN_RUNS+ nearby runs whose steps never oppose and accumulate real
/// turn is dropped whole. Steps below NEUTRAL neither set nor break the
/// chain's direction.
const ARC_CHAIN_MAX_STEP_DEG: f64 = 15.0;
const ARC_CHAIN_NEUTRAL_STEP_DEG: f64 = 1.5;
const ARC_CHAIN_MIN_TOTAL_DEG: f64 = 2.5;
const ARC_CHAIN_MIN_RUNS: usize = 3;

/// Drop chains of detected runs that together trace one continuous curve.
pub(super) fn drop_arc_chain_runs(
    runs: Vec<(usize, usize)>,
    smoothed: &[(f64, f64)],
    turns: &[f64],
    n: usize,
) -> Vec<(usize, usize)> {
    let _ = turns; // kept for future gating experiments, like line_anchor's
    let m = runs.len();
    if m < ARC_CHAIN_MIN_RUNS {
        return runs;
    }
    let dir = |r: (usize, usize)| -> f64 {
        let (ax, ay) = smoothed[r.0];
        let (bx, by) = smoothed[r.1];
        (by - ay).atan2(bx - ax)
    };
    let len = |r: (usize, usize)| (r.1 + n - r.0) % n;
    let gap = |a: (usize, usize), b: (usize, usize)| (b.0 + n - a.1) % n;
    // Signed join step to the next run; None when not chainable.
    let step: Vec<Option<f64>> = (0..m)
        .map(|i| {
            let a = runs[i];
            let b = runs[(i + 1) % m];
            let mut d = dir(b) - dir(a);
            while d > std::f64::consts::PI {
                d -= 2.0 * std::f64::consts::PI;
            }
            while d < -std::f64::consts::PI {
                d += 2.0 * std::f64::consts::PI;
            }
            let close = gap(a, b) <= (len(a).min(len(b)) * 3) / 2;
            (close && d.abs() <= ARC_CHAIN_MAX_STEP_DEG.to_radians())
                .then_some(d)
        })
        .collect();
    let neutral = ARC_CHAIN_NEUTRAL_STEP_DEG.to_radians();
    let dbg = std::env::var("IMG2BEZ_DEBUG_ARCCHAIN").is_ok();
    let mut drop = vec![false; m];
    let mut i = 0usize;
    while i < m {
        if step[i].is_none() {
            i += 1;
            continue;
        }
        // Grow a chain from run i: steps may be neutral or share one sign.
        let mut sign = 0.0f64;
        let mut total = 0.0f64;
        let mut count = 1usize; // runs in chain
        let mut j = i;
        while count <= m {
            let Some(d) = step[j % m] else { break };
            if d.abs() >= neutral {
                if sign != 0.0 && d.signum() != sign {
                    break;
                }
                sign = d.signum();
            }
            total += d;
            count += 1;
            j += 1;
        }
        let qualifies = count >= ARC_CHAIN_MIN_RUNS
            && total.abs() >= ARC_CHAIN_MIN_TOTAL_DEG.to_radians();
        if dbg && count >= 2 {
            eprintln!(
                "    arc-chain {} runs from sample {}: total={:.1}° -> {}",
                count,
                runs[i].0,
                total.to_degrees(),
                if qualifies { "DROP" } else { "keep" }
            );
        }
        if crate::ml::mldata::enabled() && count >= 2 {
            for k in i..i + count {
                let r = runs[k % m];
                let mid = smoothed[(r.0 + len(r) / 2) % n];
                crate::ml::mldata::log(
                    "line_arcchain",
                    Some(mid),
                    &[
                        ("run_len", len(r) as f64),
                        ("chain_runs", count as f64),
                        ("chain_total_deg", total.to_degrees()),
                        (
                            "own_step_deg",
                            step[k % m].map(|d| d.to_degrees()).unwrap_or(0.0),
                        ),
                    ],
                    if qualifies { "drop" } else { "keep" },
                );
            }
        }
        if qualifies {
            for k in i..i + count {
                drop[k % m] = true;
            }
        }
        i = (j + 1).max(i + 1);
    }
    runs.into_iter()
        .enumerate()
        .filter(|(k, _)| !drop[*k])
        .map(|(_, r)| r)
        .collect()
}

/// Detect maximal straight runs (cyclic (start, end) sample indices):
/// spans whose chord deviation stays under the linear envelope. Runs
/// never start inside a corner's rounding zone; accepted runs extend with
/// a relaxed floor only if the extension reaches a corner; diagonal runs
/// are kept only when anchored (`keep_anchored_runs`).
pub(super) fn detect_straight_runs(
    smoothed: &[(f64, f64)],
    turns: &[f64],
    corners: &[usize],
    n: usize,
) -> Vec<(usize, usize)> {
    let near_corner = |i: usize| -> bool {
        corners.iter().any(|&c| {
            (i + n - c) % n <= SPLIT_SNAP_SAMPLES
                || (c + n - i) % n <= SPLIT_SNAP_SAMPLES
        })
    };
    let mut runs = Vec::new();
    // Start scanning where no straight run can be (a corner, else the
    // most curved sample), so a seam-wrapping run is not split in two.
    let origin = corners.first().copied().unwrap_or_else(|| {
        (0..n)
            .max_by(|&a, &b| {
                let da = chord_deviation(&nine_point_window(smoothed, a, n));
                let db = chord_deviation(&nine_point_window(smoothed, b, n));
                da.partial_cmp(&db).unwrap_or(std::cmp::Ordering::Equal)
            })
            .unwrap_or(0)
    });
    let on_corner_rounding = |idx: usize| -> bool {
        corners.iter().any(|&c| {
            let d = (idx + n - c) % n;
            d.min(n - d) <= 3
        })
    };
    let mut i = origin;
    let mut consumed = 0usize;
    while consumed < n {
        // Never start growth inside a corner's rounding zone: a run
        // seeded on the rounded vertex can wander across the corner, and
        // its failure jump then skips where a clean run could begin.
        if on_corner_rounding(i % n) {
            i += 1;
            consumed += 1;
            continue;
        }
        // Grow [i, j] while straight under the envelope; corner-adjacent
        // starts get the looser floor (terminal flats sag mid-flat).
        let floor = if near_corner(i % n) {
            CORNER_FLAT_DEV_FLOOR
        } else {
            STRAIGHT_DEV_FLOOR
        };
        let mut j = i + 1;
        let mut buf: Vec<(f64, f64)> =
            vec![smoothed[i % n], smoothed[(i + 1) % n]];
        while j - i < n - 1 {
            buf.push(smoothed[(j + 1) % n]);
            let chord = dist(buf[0], buf[buf.len() - 1]);
            let allowed = floor.max(chord * STRAIGHT_DEV_SLOPE);
            if chord_deviation(&buf) > allowed {
                buf.pop();
                break;
            }
            j += 1;
        }
        let chord = dist(smoothed[i % n], smoothed[j % n]);
        let len = j - i;
        if len >= 2 {
            let min_chord = if near_corner(i % n) && near_corner(j % n) {
                STRAIGHT_MIN_CHORD_AT_CORNER
            } else {
                STRAIGHT_MIN_CHORD
            };
            if chord >= min_chord {
                // Extend both ends with the relaxed floor: growth may
                // have started well past a corner's rounding tail,
                // leaving a gap that would become a degenerate collinear
                // cubic between the corner and the line.
                let relaxed_ok = |buf: &[(f64, f64)]| -> bool {
                    let c = dist(buf[0], buf[buf.len() - 1]);
                    chord_deviation(buf)
                        <= CORNER_FLAT_DEV_FLOOR.max(c * STRAIGHT_DEV_SLOPE)
                };
                let mut start = i as i64;
                let mut ext: Vec<(f64, f64)> = buf.clone();
                while (i as i64) - start < RUN_EXTEND_MAX_SAMPLES as i64
                    && (j as i64) - start < n as i64 - 1
                {
                    let prev = (start - 1).rem_euclid(n as i64) as usize;
                    let mut candidate = vec![smoothed[prev]];
                    candidate.extend_from_slice(&ext);
                    if !relaxed_ok(&candidate) {
                        break;
                    }
                    ext = candidate;
                    start -= 1;
                }
                let mut end = j as i64;
                while end - (j as i64) < RUN_EXTEND_MAX_SAMPLES as i64
                    && end - start < n as i64 - 1
                {
                    let next = (end + 1).rem_euclid(n as i64) as usize;
                    let mut candidate = ext.clone();
                    candidate.push(smoothed[next]);
                    if !relaxed_ok(&candidate) {
                        break;
                    }
                    ext = candidate;
                    end += 1;
                }
                // Keep an extension only when it reaches a corner;
                // otherwise it creeps into gentle curves and drags the
                // tangent point off its true position.
                let start_idx = start.rem_euclid(n as i64) as usize;
                let end_idx = end.rem_euclid(n as i64) as usize;
                let final_start =
                    if start_idx != i % n && near_corner(start_idx) {
                        start_idx
                    } else {
                        i % n
                    };
                let final_end = if end_idx != j % n && near_corner(end_idx) {
                    end_idx
                } else {
                    j % n
                };
                runs.push((final_start, final_end));
            }
        }
        consumed += (j - i).max(1);
        i = j.max(i + 1);
    }

    keep_anchored_runs(runs, smoothed, turns, corners, n)
}

/// Filter runs down to the anchored ones: axis-aligned, corner-
/// terminated, or chained to a run that is. An unanchored diagonal
/// blending into curves at both ends is a flat spot inside a deliberate
/// curve and must stay part of the curve.
fn keep_anchored_runs(
    runs: Vec<(usize, usize)>,
    smoothed: &[(f64, f64)],
    turns: &[f64],
    corners: &[usize],
    n: usize,
) -> Vec<(usize, usize)> {
    let cyc = |a: usize, b: usize| {
        let d = (a + n - b) % n;
        d.min(n - d)
    };
    let max_off = RUN_AXIS_MAX_DEG.to_radians().tan();
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
    let flank = |from: usize, back: bool| -> f64 {
        (1..=ARC_FLAT_FLANK_WINDOW)
            .map(|q| {
                let idx = if back {
                    (from + n - q) % n
                } else {
                    (from + q) % n
                };
                turns[idx]
            })
            .sum::<f64>()
            .to_degrees()
    };
    let mut keep: Vec<bool> = runs
        .iter()
        .map(|&(s, e)| {
            let a = smoothed[s];
            let b = smoothed[e];
            let (dx, dy) = ((b.0 - a.0).abs(), (b.1 - a.1).abs());
            let dir_ok = (dx > dy && dy <= dx * max_off)
                || (dy > dx && dx <= dy * max_off);
            let corner_anchored = corners.iter().any(|&c| {
                cyc(s, c) <= 2 * SPLIT_SNAP_SAMPLES
                    || cyc(e, c) <= 2 * SPLIT_SNAP_SAMPLES
            });
            // Arc-flat veto: same-sign curving flanks + a short,
            // corner-free run = the flat of an arc, not a line.
            if !corner_anchored {
                let before = flank(s, true);
                let after = flank(e, false);
                let chord = ((b.0 - a.0).powi(2) + (b.1 - a.1).powi(2)).sqrt();
                if before.signum() == after.signum()
                    && before.abs() >= ARC_FLAT_MIN_FLANK_DEG
                    && after.abs() >= ARC_FLAT_MIN_FLANK_DEG
                    && before.abs() <= ARC_FLAT_MAX_FLANK_DEG
                    && after.abs() <= ARC_FLAT_MAX_FLANK_DEG
                    && chord < diag * ARC_FLAT_MAX_CHORD_FRAC
                {
                    return false;
                }
            }
            dir_ok || corner_anchored
        })
        .collect();
    let anchored_direct = keep.clone();
    // Propagate anchoring through endpoint-adjacent runs (line chains).
    loop {
        let mut changed = false;
        for idx in 0..runs.len() {
            if keep[idx] {
                continue;
            }
            let (s, e) = runs[idx];
            let linked = runs.iter().enumerate().any(|(k, &(s2, e2))| {
                k != idx
                    && keep[k]
                    && [cyc(s, s2), cyc(s, e2), cyc(e, s2), cyc(e, e2)]
                        .into_iter()
                        .any(|d| d <= SPLIT_SNAP_SAMPLES)
            });
            if linked {
                keep[idx] = true;
                changed = true;
            }
        }
        if !changed {
            break;
        }
    }
    if crate::ml::mldata::enabled() {
        for (idx, &(rs, re)) in runs.iter().enumerate() {
            let a = smoothed[rs];
            let b = smoothed[re];
            let (dx, dy) = ((b.0 - a.0).abs(), (b.1 - a.1).abs());
            let chord = (dx * dx + dy * dy).sqrt();
            let mid = smoothed[(rs + (re + n - rs) % n / 2) % n];
            let corner_anchored = corners.iter().any(|&c| {
                cyc(rs, c) <= 2 * SPLIT_SNAP_SAMPLES
                    || cyc(re, c) <= 2 * SPLIT_SNAP_SAMPLES
            });
            crate::ml::mldata::log(
                "line_anchor",
                Some(mid),
                &[
                    ("run_len", ((re + n - rs) % n) as f64),
                    ("axis_off", dx.min(dy) / dx.max(dy).max(1e-9)),
                    ("corner_anchored", f64::from(corner_anchored)),
                    ("flank_before_deg", flank(rs, true)),
                    ("flank_after_deg", flank(re, false)),
                    ("chord_frac", chord / diag.max(1e-9)),
                    ("linked", f64::from(keep[idx] && !anchored_direct[idx])),
                ],
                if keep[idx] { "keep" } else { "drop" },
            );
        }
    }
    runs.into_iter()
        .zip(keep)
        .filter_map(|(r, k)| k.then_some(r))
        .collect()
}

/// The 9 samples centered on `center` (cyclic).
fn nine_point_window(
    smoothed: &[(f64, f64)],
    center: usize,
    n: usize,
) -> Vec<(f64, f64)> {
    (0..9).map(|k| smoothed[(center + n + k - 4) % n]).collect()
}
