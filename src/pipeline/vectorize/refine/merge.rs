// Copyright 2026 the img2bez Authors
// SPDX-License-Identifier: Apache-2.0 OR MIT

use crate::pipeline::vectorize::fit::{FittedContour, SplitKind};

use super::{
    RasterTarget, all_polylines, band_loss, collect_band, optimize_handles,
    poly_turn, sample_cubic,
};

/// Merge acceptance: the single cubic must fit well absolutely (good
/// merges measure 0.01-0.04, bad 0.06+) or lose almost nothing vs the
/// RE-OPTIMIZED pair — re-optimizing keeps the relative test fair.
const MERGE_ABS_OK: f64 = 0.04;
const MERGE_LOSS_FACTOR: f64 = 1.3;
const MERGE_LOSS_FLOOR: f64 = 1e-3;

/// Never merge a pair whose combined turn exceeds this (radians, ~100°):
/// one cubic per quadrant is the unit type designers draw in.
const MERGE_MAX_TURN: f64 = 1.75;

/// Segments turning less than this (radians) are direction-ambiguous;
/// they may merge with either turn sign.
const TURN_SIGN_MIN: f64 = 0.05;

/// A Corner joint may merge away only when its tangent break is shallow
/// (cosine >= this, ~30°): the fitter over-splits smooth curves with
/// faint corners; a genuine corner breaks far sharper.
const MERGE_CORNER_MIN_COS: f64 = 0.866;

/// Merge fitter-created joints where one cubic suffices (Pass 5; see
/// module docs).
pub(super) fn merge_pass(
    mut contour: FittedContour,
    rt: &RasterTarget,
    ink_left: bool,
    debug: bool,
) -> FittedContour {
    loop {
        let n = contour.segs.len();
        if n < 3 {
            break;
        }
        let polys = all_polylines(&contour);
        let mut merged_any = false;
        for i in 0..n {
            let p = (i + n - 1) % n;
            if contour.is_line[p] || contour.is_line[i] {
                continue;
            }
            // Merge over inflection/fitter joints and shallow corners;
            // sharp corners are excluded by the break test, with the
            // band-loss gate as backstop.
            let mergeable = match contour.joint_kind[i] {
                SplitKind::Inflection | SplitKind::FitterJoint => true,
                SplitKind::Corner => {
                    joint_break_cos(&contour, i) >= MERGE_CORNER_MIN_COS
                }
                _ => false,
            };
            if !mergeable {
                continue;
            }
            // Same turn direction required (an S-merge erases structure,
            // and band loss under-reports the misfit on thin strokes);
            // near-straight segments may merge either way. This applies
            // to FitterJoints too.
            let tp = poly_turn(&polys[p]);
            let ti = poly_turn(&polys[i]);
            if tp * ti < 0.0
                && tp.abs() > TURN_SIGN_MIN
                && ti.abs() > TURN_SIGN_MIN
            {
                continue;
            }
            if (tp + ti).abs() > MERGE_MAX_TURN {
                continue;
            }
            let a = contour.segs[p][0];
            let b = contour.segs[i][3];
            let u0 = contour.segs[p][1] - a;
            let u1 = contour.segs[i][2] - b;
            if u0.hypot() < 1e-9 || u1.hypot() < 1e-9 {
                continue;
            }
            let mut region = polys[p].clone();
            region.extend_from_slice(&polys[i][1..]);
            let fixed = [&polys[(p + n - 1) % n], &polys[(i + 1) % n]];
            let band = collect_band(rt, &region, &fixed);
            if band.len() < 16 {
                continue;
            }
            let loss_pair = band_loss(&band, &region, ink_left);
            let chord = (b - a).hypot();
            let init = chord / 3.0;
            let (merged, loss_merged) = optimize_handles(
                a,
                u0.normalize(),
                u1.normalize(),
                b,
                (init, init),
                &band,
                ink_left,
                &[],
                &[],
            );
            // Fair baseline: give the pair the same optimization budget
            // (handle lengths only, shared joint and directions fixed).
            let joint = contour.segs[i][0];
            let u1p = contour.segs[p][2] - joint;
            let u0i = contour.segs[i][1] - joint;
            let loss_pair = if u1p.hypot() < 1e-9 || u0i.hypot() < 1e-9 {
                loss_pair
            } else {
                let (sp, _) = optimize_handles(
                    a,
                    u0.normalize(),
                    u1p.normalize(),
                    joint,
                    (u0.hypot(), u1p.hypot()),
                    &band,
                    ink_left,
                    &[],
                    &polys[i],
                );
                let poly_p = sample_cubic(&sp);
                let (_, loss_pair_opt) = optimize_handles(
                    joint,
                    u0i.normalize(),
                    u1.normalize(),
                    b,
                    (u0i.hypot(), u1.hypot()),
                    &band,
                    ink_left,
                    &poly_p,
                    &[],
                );
                loss_pair.min(loss_pair_opt)
            };
            let accept = loss_merged <= MERGE_ABS_OK
                || loss_merged
                    <= MERGE_LOSS_FACTOR * loss_pair + MERGE_LOSS_FLOOR;
            if crate::ml::mldata::enabled() {
                let joint = contour.segs[i][0];
                crate::ml::mldata::log(
                    "point_merge",
                    Some((joint.x, joint.y)),
                    &[
                        (
                            "joint_kind",
                            match contour.joint_kind[i] {
                                SplitKind::Corner => 1.0,
                                SplitKind::Inflection => 2.0,
                                _ => 3.0,
                            },
                        ),
                        ("break_cos", joint_break_cos(&contour, i)),
                        ("turn_prev_deg", tp.to_degrees()),
                        ("turn_seg_deg", ti.to_degrees()),
                        ("chord", chord),
                        ("loss_pair", loss_pair),
                        ("loss_merged", loss_merged),
                        ("band_len", band.len() as f64),
                    ],
                    if accept { "merge" } else { "keep" },
                );
            }
            if debug {
                eprintln!(
                    "    refine: {} merge at ({:.1}, {:.1}) pair_opt={:.5} merged={:.5}",
                    if accept { "ACCEPT" } else { "reject" },
                    contour.segs[i][0].x,
                    contour.segs[i][0].y,
                    loss_pair,
                    loss_merged
                );
            }
            if accept {
                contour.segs[p] = merged;
                contour.segs.remove(i);
                contour.is_line.remove(i);
                contour.joint_kind.remove(i);
                merged_any = true;
                break;
            }
        }
        if !merged_any {
            break;
        }
    }
    contour
}

/// Cosine of the tangent break at the joint starting segment `i` (= end of
/// segment `i-1`). 1.0 = perfectly smooth, lower = sharper corner.
fn joint_break_cos(contour: &FittedContour, i: usize) -> f64 {
    let n = contour.segs.len();
    let p = (i + n - 1) % n;
    let t_in = contour.segs[p][3] - contour.segs[p][2]; // end tangent of the previous seg
    let t_out = contour.segs[i][1] - contour.segs[i][0]; // start tangent of this seg
    if t_in.hypot() < 1e-9 || t_out.hypot() < 1e-9 {
        return -1.0;
    }
    t_in.normalize().dot(t_out.normalize())
}
