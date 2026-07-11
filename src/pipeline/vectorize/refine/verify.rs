// Copyright 2026 the img2bez Authors
// SPDX-License-Identifier: Apache-2.0 OR MIT

use kurbo::{Point, Vec2};

use crate::pipeline::vectorize::fit::{FittedContour, SplitKind};

use super::{
    RasterTarget, all_polylines, band_loss, collect_band, optimize_handles,
    sample_cubic,
};

/// Coverage along a fitted curve should sit near 0.5 (the iso edge); a
/// sustained run beyond the drift bound means the outline is buried or
/// floating — the failure averaging metrics miss. Correction may ROTATE
/// the handle at a corner-anchored end (no tangent constraint there);
/// length-only passes can never fix an aim error.
const VERIFY_COV_DRIFT: f64 = 0.35;
const VERIFY_MIN_RUN: usize = 15;
/// A fix must CURE the drift (residual run <= this), not merely reduce
/// it: on thin strokes the probe can be half-satisfied by dragging the
/// deviation elsewhere. Genuine aim fixes leave runs of 1-8.
const VERIFY_MAX_RESIDUAL: usize = 8;
/// Zero rotation first: the unrotated candidate wins ties, so clean
/// axis-aligned corner handles stay exactly axis-aligned.
const VERIFY_ANGLE_STEPS: [f64; 5] = [0.0, -8.0, 8.0, -16.0, 16.0];

/// Worst-case coverage check: "does any stretch of the final outline sit
/// off the ink edge", not "is the average loss acceptable".
pub(super) fn verify_coverage(
    mut contour: FittedContour,
    rt: &RasterTarget,
    ink_left: bool,
    debug: bool,
) -> FittedContour {
    let n = contour.segs.len();
    // Corner-rounding-zone samples are excluded: rendered ink is
    // legitimately rounded next to a sharp corner, and chasing that drift
    // drags the outline onto the rounding.
    let drift_run =
        |seg: &[Point; 4], skip_start: bool, skip_end: bool| -> usize {
            let pts = sample_cubic(seg);
            let m = pts.len();
            let zone = (m * 3) / 20; // ~15% of the samples at each masked end
            let lo = if skip_start { zone } else { 0 };
            let hi = if skip_end { m - zone.min(m) } else { m };
            let mut worst = 0usize;
            let mut run = 0usize;
            for p in pts.iter().take(hi).skip(lo) {
                let cov = rt.coverage(p.x, p.y);
                if (cov - 0.5).abs() >= VERIFY_COV_DRIFT {
                    run += 1;
                    worst = worst.max(run);
                } else {
                    run = 0;
                }
            }
            worst
        };
    let polys = all_polylines(&contour);
    for i in 0..n {
        if contour.is_line[i] {
            continue;
        }
        // Angular freedom only at Corner joints; smooth joints, extrema,
        // and tangent points keep their directions exactly.
        let start_free = contour.joint_kind[i] == SplitKind::Corner;
        let end_free = contour.joint_kind[(i + 1) % n] == SplitKind::Corner;
        let run = drift_run(&contour.segs[i], start_free, end_free);
        if run < VERIFY_MIN_RUN {
            continue;
        }
        let seg = contour.segs[i];
        let u0 = seg[1] - seg[0];
        let u1 = seg[2] - seg[3];
        if u0.hypot() < 1e-9 || u1.hypot() < 1e-9 {
            continue;
        }
        if !start_free && !end_free {
            continue;
        }
        let fixed = [&polys[(i + n - 1) % n], &polys[(i + 1) % n]];
        let band = collect_band(rt, &polys[i], &fixed);
        if band.len() < 16 {
            continue;
        }
        let base_loss = band_loss(&band, &polys[i], ink_left);
        let rot = |v: Vec2, deg: f64| -> Vec2 {
            let (s, c) = deg.to_radians().sin_cos();
            Vec2::new(v.x * c - v.y * s, v.x * s + v.y * c)
        };
        let mut best: Option<([Point; 4], f64, usize)> = None;
        for &da in VERIFY_ANGLE_STEPS.iter() {
            if da != 0.0 && !start_free {
                continue;
            }
            for &db in VERIFY_ANGLE_STEPS.iter() {
                if db != 0.0 && !end_free {
                    continue;
                }
                let (cand, loss) = optimize_handles(
                    seg[0],
                    rot(u0.normalize(), da),
                    rot(u1.normalize(), db),
                    seg[3],
                    (u0.hypot(), u1.hypot()),
                    &band,
                    ink_left,
                    &[],
                    &[],
                );
                let cand_run = drift_run(&cand, start_free, end_free);
                match &best {
                    Some((_, _, r)) if cand_run >= *r => {}
                    _ => best = Some((cand, loss, cand_run)),
                }
                // Strict `>=`: ties keep the less-rotated candidate.
            }
        }
        if let Some((cand, cand_loss, cand_run)) = best
            && cand_run * 2 <= run
            && cand_run <= VERIFY_MAX_RESIDUAL
            && cand_loss <= base_loss
        {
            if debug {
                eprintln!(
                    "    refine: VERIFY fix at ({:.0},{:.0}) drift run {} -> {}",
                    seg[0].x, seg[0].y, run, cand_run
                );
            }
            contour.segs[i] = cand;
        }
    }
    contour
}
