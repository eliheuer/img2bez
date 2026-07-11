// Copyright 2026 the img2bez Authors
// SPDX-License-Identifier: Apache-2.0 OR MIT

use kurbo::Point;

use crate::pipeline::vectorize::fit::{FittedContour, SplitKind};

use super::{
    MAX_HANDLE_FRAC, MIN_HANDLE_FRAC, RasterTarget, all_polylines, band_loss,
    collect_band, optimize_handles, sample_cubic,
};

/// Polish dead-band: leave a cubic alone unless base loss >=
/// MIN_BASE_LOSS, optimization reaches MIN_GAIN of it, AND the result
/// fits (<= RESULT_MAX) — skips well-placed handles and mis-segmented
/// sections no handle length can fix.
const POLISH_MIN_BASE_LOSS: f64 = 0.02;
const POLISH_MIN_GAIN: f64 = 0.7;
const POLISH_RESULT_MAX: f64 = 0.02;
/// Rescue tier: a segment this badly off the raster (e.g. a crotch wall
/// left by a vertex collapse) accepts any optimization that at least
/// ~halves its loss, even when RESULT_MAX is unreachable.
const POLISH_RESCUE_BASE: f64 = 0.10;
const POLISH_RESCUE_GAIN: f64 = 0.30;

/// Span-normalized handle ratio that triggers re-evening (drawn pairs sit
/// near 1.1; inverted pairs read 1.5+).
const BALANCE_LOPSIDED_RATIO: f64 = 1.35;
/// Extra band loss the evening may spend, scaled by turn: gentle curves
/// barely move when evened (GENTLE budget); tight quarter-arcs bulge off
/// the ink (TIGHT). Linear in turn angle up to TURN_REF.
const BALANCE_LOPSIDED_SLACK_GENTLE: f64 = 0.040;
const BALANCE_LOPSIDED_SLACK_TIGHT: f64 = 0.012;
const BALANCE_LOPSIDED_TURN_REF: f64 = std::f64::consts::FRAC_PI_2;
/// Skip curves already fitting this poorly: their shape is wrong, and the
/// within-budget search would reshape rather than even them.
const BALANCE_LOPSIDED_MAX_LOPT: f64 = 0.06;
/// Grid resolution for the lopsided-handle raster search.
const BALANCE_LOPSIDED_GRID: usize = 28;

/// A joint-side handle shorter than this (px) at a constrained line↔curve
/// joint marks a misplaced BOUNDARY (the line swallowed the arc's
/// nearly-straight tail); the joint itself must move.
const SLIDE_TRIGGER_HANDLE: f64 = 8.0;
/// The boundary may retreat at most this fraction of the line's length.
const SLIDE_MAX_LINE_FRAC: f64 = 0.5;
/// Candidate boundary positions tried along the line.
const SLIDE_GRID: usize = 10;
/// The re-fit pair may lose at most this factor of the current pair's
/// loss plus a small floor (the true boundary fits at least as well; the
/// slack absorbs grid quantization).
const SLIDE_MAX_LOSS_FACTOR: f64 = 1.02;
const SLIDE_MAX_LOSS_ABS: f64 = 0.002;
/// Ignore lines too short to plausibly have swallowed a curve tail.
const SLIDE_MIN_LINE: f64 = 24.0;
/// Lateral pivot steps per side and max perpendicular step (px): the
/// blend that swallowed the tail also drags the joint sideways, tilting
/// the line off the ink edge, and the slide must be able to undo that.
const SLIDE_LATERAL: usize = 4;
const SLIDE_LATERAL_MAX: f64 = 3.0;

/// Slide a constrained line↔curve boundary back out of the line where the
/// curve's joint-side handle is degenerate (Pass 8); tangent directions
/// are preserved, so the joint stays G1 by construction.
pub(super) fn slide_pass(
    mut contour: FittedContour,
    rt: &RasterTarget,
    ink_left: bool,
    debug: bool,
) -> FittedContour {
    let n = contour.segs.len();
    if n < 3 {
        return contour;
    }
    for i in 0..n {
        if contour.is_line[i] {
            continue;
        }
        // Start joint: line (i-1) -> curve i.
        let j = (i + n - 1) % n;
        if contour.is_line[j]
            && slide_qualifies(contour.joint_kind[i])
            && (contour.segs[i][1] - contour.segs[i][0]).hypot()
                < SLIDE_TRIGGER_HANDLE
        {
            try_slide(&mut contour, rt, ink_left, j, i, true, debug);
        }
        // End joint: curve i -> line (i+1).
        let k = (i + 1) % n;
        if contour.is_line[k]
            && slide_qualifies(contour.joint_kind[k])
            && (contour.segs[i][2] - contour.segs[i][3]).hypot()
                < SLIDE_TRIGGER_HANDLE
        {
            try_slide(&mut contour, rt, ink_left, k, i, false, debug);
        }
    }
    contour
}

/// Joint kinds whose tangent is constrained to a fixed direction — where
/// a misplaced boundary manifests as a degenerate handle. Corners have no
/// tangent constraint; inflections are real structure.
fn slide_qualifies(kind: SplitKind) -> bool {
    matches!(
        kind,
        SplitKind::Tangent | SplitKind::ExtremumX | SplitKind::ExtremumY
    )
}

/// Attempt one boundary slide between line `j` and curve `i` (`at_start`:
/// the joint is at the curve's start); applies the winner in place.
fn try_slide(
    contour: &mut FittedContour,
    rt: &RasterTarget,
    ink_left: bool,
    j: usize,
    i: usize,
    at_start: bool,
    debug: bool,
) {
    let n = contour.segs.len();
    let polys = all_polylines(contour);
    let line = contour.segs[j];
    let seg = contour.segs[i];
    // Far end of the line (the end NOT at the joint) and the joint point.
    let (far, joint) = if at_start {
        (line[0], seg[0])
    } else {
        (line[3], seg[3])
    };
    let line_v = joint - far;
    let line_len = line_v.hypot();
    if line_len < SLIDE_MIN_LINE {
        return;
    }
    let dir = line_v / line_len; // points from the line INTO the joint
    // The curve's far-end tangent direction stays fixed.
    let far_tan = if at_start {
        seg[2] - seg[3]
    } else {
        seg[1] - seg[0]
    };
    if far_tan.hypot() < 1e-9 {
        return;
    }
    let far_tan = far_tan.normalize();
    let far_hlen = if at_start {
        (seg[2] - seg[3]).hypot()
    } else {
        (seg[1] - seg[0]).hypot()
    };
    // Band over the line+curve pair, outer neighbors fixed.
    let (before, after) = if at_start {
        ((j + n - 1) % n, (i + 1) % n)
    } else {
        ((i + n - 1) % n, (j + 1) % n)
    };
    let fixed = [&polys[before], &polys[after]];
    let d_max = line_len * SLIDE_MAX_LINE_FRAC;
    // Judge on a LOCAL band (slidable stretch + curve): the line's far
    // half contributes the same error to every candidate and would drown
    // the local improvement. Candidates still use the true final geometry.
    let mid = joint - dir * d_max;
    let region: Vec<Point> = if at_start {
        [mid, joint]
            .into_iter()
            .chain(polys[i].iter().copied())
            .collect()
    } else {
        polys[i].iter().copied().chain([joint, mid]).collect()
    };
    let band = collect_band(rt, &region, &fixed);
    if band.len() < 16 {
        return;
    }
    let base_poly: Vec<Point> = if at_start {
        [far, joint]
            .into_iter()
            .chain(polys[i].iter().copied())
            .collect()
    } else {
        polys[i].iter().copied().chain([joint, far]).collect()
    };
    let base = band_loss(&band, &base_poly, ink_left);
    let limit = base * SLIDE_MAX_LOSS_FACTOR + SLIDE_MAX_LOSS_ABS;
    // Each candidate also pivots the line about its far end (the joint
    // steps perpendicular to the line, tangent following, staying G1):
    // the swallowed tail drags the joint laterally too, tilting the whole
    // line off the ink edge.
    let perp = kurbo::Vec2::new(-dir.y, dir.x);
    let mut best: Option<(f64, Point, [Point; 4])> = None;
    let mut dbg_min = (f64::INFINITY, 0.0f64, 0.0f64);
    for step in 1..=SLIDE_GRID {
        let d = d_max * step as f64 / SLIDE_GRID as f64;
        for lat_step in -(SLIDE_LATERAL as i32)..=(SLIDE_LATERAL as i32) {
            let lat =
                lat_step as f64 * SLIDE_LATERAL_MAX / SLIDE_LATERAL as f64;
            let p2 = joint - dir * d + perp * lat;
            let line_dir = (p2 - far).normalize();
            let (cand, loss) = if at_start {
                // Pivoted line is the loss prefix; the cubic's start
                // tangent lies ON the line.
                optimize_handles(
                    p2,
                    line_dir,
                    far_tan,
                    seg[3],
                    (d.max(SLIDE_TRIGGER_HANDLE), far_hlen),
                    &band,
                    ink_left,
                    &[far, p2],
                    &[],
                )
            } else {
                optimize_handles(
                    seg[0],
                    far_tan,
                    -line_dir,
                    p2,
                    (far_hlen, d.max(SLIDE_TRIGGER_HANDLE)),
                    &band,
                    ink_left,
                    &[],
                    &[p2, far],
                )
            };
            if loss < dbg_min.0 {
                dbg_min = (loss, d, lat);
            }
            if loss > limit {
                continue;
            }
            // The slide must actually heal the joint-side handle.
            let joint_h = if at_start {
                (cand[1] - cand[0]).hypot()
            } else {
                (cand[2] - cand[3]).hypot()
            };
            if joint_h < SLIDE_TRIGGER_HANDLE {
                continue;
            }
            if best.as_ref().is_none_or(|(l, _, _)| loss < *l) {
                best = Some((loss, p2, cand));
            }
        }
    }
    let Some((loss, p2, cand)) = best else {
        if debug {
            eprintln!(
                "    refine: slide at ({:.0},{:.0}) NO CANDIDATE base={:.4} limit={:.4} line_len={:.0} min={:.4}@d={:.0},lat={:.1}",
                joint.x,
                joint.y,
                base,
                limit,
                line_len,
                dbg_min.0,
                dbg_min.1,
                dbg_min.2
            );
        }
        return;
    };
    if debug {
        eprintln!(
            "    refine: slide joint ({:.0},{:.0}) -> ({:.0},{:.0})  loss {:.4} -> {:.4}",
            joint.x, joint.y, p2.x, p2.y, base, loss
        );
    }
    contour.segs[j] = if at_start {
        [far, far.lerp(p2, 1.0 / 3.0), far.lerp(p2, 2.0 / 3.0), p2]
    } else {
        [p2, p2.lerp(far, 1.0 / 3.0), p2.lerp(far, 2.0 / 3.0), far]
    };
    contour.segs[i] = cand;
}

/// Polish handle lengths of every remaining cubic (Pass 6; see module
/// docs). Segments near a `touched` joint are eligible for the rescue
/// tier.
pub(super) fn polish_pass(
    mut contour: FittedContour,
    rt: &RasterTarget,
    ink_left: bool,
    touched: &[Point],
    debug: bool,
) -> FittedContour {
    let n = contour.segs.len();
    let polys = all_polylines(&contour);
    for i in 0..n {
        if contour.is_line[i] {
            continue;
        }
        let seg = contour.segs[i];
        let u0 = seg[1] - seg[0];
        let u1 = seg[2] - seg[3];
        if u0.hypot() < 1e-9 || u1.hypot() < 1e-9 {
            continue;
        }
        let fixed = [&polys[(i + n - 1) % n], &polys[(i + 1) % n]];
        let band = collect_band(rt, &polys[i], &fixed);
        if band.len() < 16 {
            continue;
        }
        let base = band_loss(&band, &polys[i], ink_left);
        if base < POLISH_MIN_BASE_LOSS {
            continue; // already fits the raster well: leave it alone
        }
        let (opt, loss) = optimize_handles(
            seg[0],
            u0.normalize(),
            u1.normalize(),
            seg[3],
            (u0.hypot(), u1.hypot()),
            &band,
            ink_left,
            &[],
            &[],
        );
        let near_touched = touched.iter().any(|t| {
            (*t - seg[0]).hypot() < 3.0 || (*t - seg[3]).hypot() < 3.0
        });
        let accept = (loss <= POLISH_MIN_GAIN * base
            && loss <= POLISH_RESULT_MAX)
            || (near_touched
                && base >= POLISH_RESCUE_BASE
                && loss <= base * POLISH_RESCUE_GAIN);
        if accept {
            if debug {
                eprintln!(
                    "    refine: polish seg at ({:.1}, {:.1}) loss {:.5} -> {:.5}",
                    seg[0].x, seg[0].y, base, loss
                );
            }
            contour.segs[i] = opt;
        }
    }
    contour
}

/// Re-even lopsided handles against the raster (Pass 7): polish skips
/// well-fit curves, so fitter-inherited asymmetry survives it; this pass
/// evens those judged against the ink rather than the traced curve.
pub(super) fn rebalance_pass(
    mut contour: FittedContour,
    rt: &RasterTarget,
    ink_left: bool,
    debug: bool,
) -> FittedContour {
    let n = contour.segs.len();
    let polys = all_polylines(&contour);
    for i in 0..n {
        if contour.is_line[i] {
            continue;
        }
        let seg = contour.segs[i];
        let (u0, u1) = (seg[1] - seg[0], seg[2] - seg[3]);
        let (l0, l1) = (u0.hypot(), u1.hypot());
        // Sub-8-unit handles sit below integer-rounding resolution: both
        // down there is noise territory, left alone; a single degenerate
        // handle is the one-side-collapsed fit this pass fixes.
        if l0 < 8.0 && l1 < 8.0 {
            continue;
        }
        // A zero-length handle cannot be normalized (NaN direction).
        if l0 < 1e-9 || l1 < 1e-9 {
            continue;
        }
        let (u0n, u1n) = (u0.normalize(), u1.normalize());
        // Balance is judged span-normalized (see geom::handle_spans): an
        // asymmetric quadrant's proportionally-lopsided pair is already
        // balanced.
        let spans = crate::model::geom::handle_spans(seg[0], u0n, seg[3], u1n);
        let norm_ratio = |a: f64, b: f64| -> f64 {
            let (na, nb) = match spans {
                Some((s0, s1)) => (a / s0, b / s1),
                None => (a, b),
            };
            na.max(nb) / na.min(nb)
        };
        if norm_ratio(l0, l1) < BALANCE_LOPSIDED_RATIO {
            continue;
        }
        let fixed = [&polys[(i + n - 1) % n], &polys[(i + 1) % n]];
        let band = collect_band(rt, &polys[i], &fixed);
        if band.len() < 16 {
            continue;
        }
        let eval = |a: f64, b: f64| -> f64 {
            let cand = [seg[0], seg[0] + u0n * a, seg[3] + u1n * b, seg[3]];
            band_loss(&band, &sample_cubic(&cand), ink_left)
        };
        let l_opt = eval(l0, l1);
        let turn = (-(u0n.dot(u1n)).clamp(-1.0, 1.0)).acos();
        let gentleness =
            (1.0 - turn / BALANCE_LOPSIDED_TURN_REF).clamp(0.0, 1.0);
        if l_opt > BALANCE_LOPSIDED_MAX_LOPT {
            continue;
        }
        let slack = BALANCE_LOPSIDED_SLACK_TIGHT
            + (BALANCE_LOPSIDED_SLACK_GENTLE - BALANCE_LOPSIDED_SLACK_TIGHT)
                * gentleness;
        // Budget is relative to the current fit; among pairs within
        // `target`, take the most even (ties: closest to the originals).
        let target = l_opt + slack;
        let chord = (seg[3] - seg[0]).hypot();
        let (lo, hi) =
            ((chord * MIN_HANDLE_FRAC).max(8.0), chord * MAX_HANDLE_FRAC);
        if hi <= lo {
            continue;
        }
        let mut best = (l0, l1);
        let mut best_ratio = norm_ratio(l0, l1);
        let mut best_spread = 0.0;
        for ai in 0..=BALANCE_LOPSIDED_GRID {
            let a = lo + (hi - lo) * ai as f64 / BALANCE_LOPSIDED_GRID as f64;
            for bi in 0..=BALANCE_LOPSIDED_GRID {
                let b =
                    lo + (hi - lo) * bi as f64 / BALANCE_LOPSIDED_GRID as f64;
                if eval(a, b) > target {
                    continue;
                }
                let ratio = norm_ratio(a, b);
                let spread = (a - l0).abs() + (b - l1).abs();
                if ratio < best_ratio - 1e-6
                    || (ratio <= best_ratio + 1e-6 && spread < best_spread)
                {
                    best = (a, b);
                    best_ratio = ratio;
                    best_spread = spread;
                }
            }
        }
        if debug {
            eprintln!(
                "    refine: rebalance ({:.0},{:.0}) {:.0}/{:.0} -> {:.0}/{:.0}  turn={:.0} l_opt={:.4} target={:.4}",
                seg[0].x,
                seg[0].y,
                l0,
                l1,
                best.0,
                best.1,
                turn.to_degrees(),
                l_opt,
                target
            );
        }
        contour.segs[i] =
            [seg[0], seg[0] + u0n * best.0, seg[3] + u1n * best.1, seg[3]];
    }
    contour
}
