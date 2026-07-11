// Copyright 2026 the img2bez Authors
// SPDX-License-Identifier: Apache-2.0 OR MIT

//! Stage 5: post-fit cleanup, G1 join alignment, and G2 harmonization.

use kurbo::{Point, Vec2};

use crate::model::geom::line_seg;

use super::{
    CORNER_SLIVER_MAX_CHORD, CORNER_SLIVER_MAX_REACH, FittedContour,
    HARMONIZE_MAX_SHIFT, MICRO_LINE_MAX_CHORD, MICRO_LINE_MAX_TURN,
    SHORT_CUBIC_MAX_CHORD, SHORT_CUBIC_MAX_HANDLE,
    SHORT_STRAIGHT_MAX_DEVIATION, SMOOTH_JOIN_MAX_DEG, SplitKind,
    angle_between, dist_pt, line_intersection,
};

// ── Stage 5a: post-fit cleanup ───────────────────────────────────────

/// Merge adjacent line segments whose shared non-corner joint barely
/// deviates from the merged chord: a sub-pixel bump mid-flat can split
/// one designed line into two.
pub(super) fn merge_collinear_lines(
    mut contour: FittedContour,
) -> FittedContour {
    loop {
        let n = contour.segs.len();
        if n < 3 {
            return contour;
        }
        let mut merge_at: Option<usize> = None;
        for i in 0..n {
            let p = (i + n - 1) % n;
            if !contour.is_line[p]
                || !contour.is_line[i]
                || contour.joint_kind[i] == SplitKind::Corner
            {
                continue;
            }
            let a = contour.segs[p][0];
            let joint = contour.segs[i][0];
            let b = contour.segs[i][3];
            let chord = b - a;
            let len = chord.hypot();
            if len < 1e-9 {
                continue;
            }
            let dev = (chord.cross(joint - a) / len).abs();
            if dev <= SHORT_STRAIGHT_MAX_DEVIATION {
                merge_at = Some(i);
                break;
            }
        }
        let Some(i) = merge_at else {
            return contour;
        };
        let p = (i + contour.segs.len() - 1) % contour.segs.len();
        let merged = line_seg(contour.segs[p][0], contour.segs[i][3]);
        contour.segs[p] = merged;
        contour.segs.remove(i);
        contour.is_line.remove(i);
        contour.joint_kind.remove(i);
    }
}

/// Replace a tiny curve squeezed between two lines with their sharp
/// intersection: rasterization rounds sharp corners by a few pixels, and
/// the source design has a corner there.
pub(super) fn collapse_corner_slivers(contour: FittedContour) -> FittedContour {
    let n = contour.segs.len();
    if n < 3 {
        return contour;
    }
    let mut segs = Vec::with_capacity(n);
    let mut is_line = Vec::with_capacity(n);
    let mut joint_kind = Vec::with_capacity(n);
    let mut skip = vec![false; n];
    let mut replace_end: Vec<Option<Point>> = vec![None; n];
    #[allow(clippy::needless_range_loop)]
    // cyclic neighbor access across several arrays
    for i in 0..n {
        let p = (i + n - 1) % n;
        let nx = (i + 1) % n;
        if contour.is_line[i] || !contour.is_line[p] || !contour.is_line[nx] {
            continue;
        }
        let chord = (contour.segs[i][3] - contour.segs[i][0]).hypot();
        if chord > CORNER_SLIVER_MAX_CHORD {
            continue;
        }
        let d = match line_intersection(
            contour.segs[p][0],
            contour.segs[p][3],
            contour.segs[nx][3],
            contour.segs[nx][0],
        ) {
            Some(d) => d,
            None => continue,
        };
        let mid = contour.segs[i][0].midpoint(contour.segs[i][3]);
        if (d - mid).hypot() > CORNER_SLIVER_MAX_REACH {
            continue;
        }
        skip[i] = true;
        replace_end[p] = Some(d);
    }
    for i in 0..n {
        if skip[i] {
            continue;
        }
        let mut seg = contour.segs[i];
        if let Some(d) = replace_end[i] {
            seg = line_seg(seg[0], d);
        }
        // If the previous segment was collapsed, start from the corner.
        let p = (i + n - 1) % n;
        if skip[p]
            && let Some(d) = replace_end[(p + n - 1) % n]
        {
            if contour.is_line[i] {
                seg = line_seg(d, seg[3]);
            } else {
                seg[0] = d;
            }
        }
        segs.push(seg);
        is_line.push(contour.is_line[i]);
        joint_kind.push(if skip[p] {
            SplitKind::Corner
        } else {
            contour.joint_kind[i]
        });
    }
    FittedContour {
        segs,
        is_line,
        joint_kind,
    }
}

/// Remove a vestigial micro-line wedged between two curves that stay
/// tangent-continuous across it (a 1-2px stub on a gently flattening
/// bowl), welding the curves at its midpoint. The turn gate preserves
/// intentional flats at corners, which turn sharply across their flat.
pub(super) fn collapse_micro_lines(
    mut contour: FittedContour,
) -> FittedContour {
    let max_turn = MICRO_LINE_MAX_TURN.to_radians();
    loop {
        let n = contour.segs.len();
        if n < 3 {
            return contour;
        }
        let mut collapse_at: Option<usize> = None;
        for i in 0..n {
            let p = (i + n - 1) % n;
            let nx = (i + 1) % n;
            // a line flanked by two curves
            if !contour.is_line[i] || contour.is_line[p] || contour.is_line[nx]
            {
                continue;
            }
            if (contour.segs[i][3] - contour.segs[i][0]).hypot()
                > MICRO_LINE_MAX_CHORD
            {
                continue;
            }
            // turn between the flanking curves' tangents at the joint
            // (tin = prev curve's end tangent, tout = next curve's start)
            let tin = contour.segs[p][3] - contour.segs[p][2];
            let tout = contour.segs[nx][1] - contour.segs[nx][0];
            if tin.hypot() < 1e-9 || tout.hypot() < 1e-9 {
                continue;
            }
            let turn = tin.cross(tout).atan2(tin.dot(tout)).abs();
            if turn <= max_turn {
                collapse_at = Some(i);
                break;
            }
        }
        let Some(i) = collapse_at else {
            return contour;
        };
        let n = contour.segs.len();
        let p = (i + n - 1) % n;
        let nx = (i + 1) % n;
        let weld = contour.segs[i][0].midpoint(contour.segs[i][3]);
        contour.segs[p][3] = weld;
        contour.segs[nx][0] = weld;
        contour.segs.remove(i);
        contour.is_line.remove(i);
        contour.joint_kind.remove(i);
    }
}

/// Clamp handle lengths on very short cubics (crotch fillets, corner
/// rounds): least-squares fitting on a dozen samples can overshoot into a
/// visible micro-bump; a clean fillet's handles stay near a third.
pub(in crate::pipeline::vectorize) fn tame_short_cubic_handles(
    mut contour: FittedContour,
) -> FittedContour {
    for (i, seg) in contour.segs.iter_mut().enumerate() {
        if contour.is_line[i] {
            continue;
        }
        let chord = (seg[3] - seg[0]).hypot();
        if chord > SHORT_CUBIC_MAX_CHORD {
            continue;
        }
        let max_len = chord * SHORT_CUBIC_MAX_HANDLE;
        let h1 = seg[1] - seg[0];
        if h1.hypot() > max_len {
            seg[1] = seg[0] + h1 * (max_len / h1.hypot());
        }
        let h2 = seg[2] - seg[3];
        if h2.hypot() > max_len {
            seg[2] = seg[3] + h2 * (max_len / h2.hypot());
        }
    }
    contour
}

/// Same limit as `refine::HANDLE_REACH_MAX` (see its doc for why 0.9).
const HANDLE_REACH_MAX: f64 = 0.9;

/// Enforce well-drawn handle invariants on the FINAL outline, whatever
/// path produced each segment: (1) the magic triangle — a handle past the
/// tangent intersection bulges or cusps the curve; (2) no crossing — the
/// off-curves' chord projections must not reach past each other (covers
/// the near-parallel case where the triangle degenerates).
pub(in crate::pipeline::vectorize) fn cap_handle_reach(
    mut contour: FittedContour,
) -> FittedContour {
    for (i, seg) in contour.segs.iter_mut().enumerate() {
        if contour.is_line[i] {
            continue;
        }
        let chord_v = seg[3] - seg[0];
        let chord = chord_v.hypot();
        if chord < 1e-9 {
            continue;
        }
        // Magic triangle: clamp each handle to the tangent intersection.
        let h1 = seg[1] - seg[0];
        let h2 = seg[2] - seg[3];
        let (l1, l2) = (h1.hypot(), h2.hypot());
        if l1 > 1e-9
            && l2 > 1e-9
            && let Some((t, s)) = crate::model::geom::handle_triangle(
                seg[0],
                h1 * (1.0 / l1),
                seg[3],
                h2 * (1.0 / l2),
            )
        {
            if l1 > t {
                seg[1] = seg[0] + h1 * (t / l1);
            }
            if l2 > s {
                seg[2] = seg[3] + h2 * (s / l2);
            }
        }
        // No crossing: signed reach of both handles along the chord.
        let e = chord_v * (1.0 / chord);
        let reach = (seg[1] - seg[0]).dot(e) + (seg[3] - seg[2]).dot(e);
        let limit = chord * HANDLE_REACH_MAX;
        if reach > limit {
            let s = limit / reach;
            seg[1] = seg[0] + (seg[1] - seg[0]) * s;
            seg[2] = seg[3] + (seg[2] - seg[3]) * s;
        }
    }
    contour
}

// ── Stage 5b: G1 alignment and G2 harmonization ──────────────────────

/// Align handle directions across smooth joins for G1 continuity:
/// extrema get exactly axis-aligned handles, joins involving a line take
/// the line's direction, free joins take the bisector. Corners and joins
/// turning more than `SMOOTH_JOIN_MAX_DEG` are left alone.
pub(in crate::pipeline::vectorize) fn smooth_joins(
    mut contour: FittedContour,
) -> FittedContour {
    let n = contour.segs.len();
    let max_angle = SMOOTH_JOIN_MAX_DEG.to_radians();
    for i in 0..n {
        // Joint at start of segment i, between segment p = i-1 and i.
        let p = (i + n - 1) % n;
        let kind = contour.joint_kind[i];
        if kind == SplitKind::Corner {
            continue;
        }
        let joint = contour.segs[i][0];
        let in_vec = joint - contour.segs[p][2];
        let out_vec = contour.segs[i][1] - joint;
        if in_vec.hypot() < 1e-9 || out_vec.hypot() < 1e-9 {
            continue;
        }
        let angle = angle_between(in_vec, out_vec);
        if angle > max_angle {
            continue;
        }
        let dir = match kind {
            SplitKind::ExtremumX => {
                Vec2::new(0.0, if out_vec.y >= 0.0 { 1.0 } else { -1.0 })
            }
            SplitKind::ExtremumY => {
                Vec2::new(if out_vec.x >= 0.0 { 1.0 } else { -1.0 }, 0.0)
            }
            _ => {
                let sum = in_vec.normalize() + out_vec.normalize();
                if sum.hypot() < 1e-9 {
                    continue;
                }
                sum.normalize()
            }
        };
        // Lines own their direction: align the curve handle to the line.
        if contour.is_line[p] && contour.is_line[i] {
            continue;
        }
        let dir = if contour.is_line[p] {
            in_vec.normalize()
        } else if contour.is_line[i] {
            out_vec.normalize()
        } else {
            dir
        };
        if !contour.is_line[p] {
            contour.segs[p][2] = joint - dir * in_vec.hypot();
        }
        if !contour.is_line[i] {
            contour.segs[i][1] = joint + dir * out_vec.hypot();
        }
    }
    contour
}

/// Move smooth-join on-curve points to the G2 (equal-curvature) position
/// between the neighboring handles: t = sqrt(p0*p1) / (sqrt(p0*p1) + 1)
/// along a2->b1, where p0 = |a1a2|/|a2 d|, p1 = |d b1|/|b1 b2| and d is
/// the intersection of the handle lines.
pub(in crate::pipeline::vectorize) fn harmonize(
    mut contour: FittedContour,
) -> FittedContour {
    let n = contour.segs.len();
    for _pass in 0..2 {
        for i in 0..n {
            let p = (i + n - 1) % n;
            let kind = contour.joint_kind[i];
            if kind == SplitKind::Corner
                || contour.is_line[p]
                || contour.is_line[i]
            {
                continue;
            }
            let a1 = contour.segs[p][1];
            let a2 = contour.segs[p][2];
            let b0 = contour.segs[i][0];
            let b1 = contour.segs[i][1];
            let b2 = contour.segs[i][2];
            // Skip degenerate or inflection-like joins where the far
            // handles sit on opposite sides.
            let join_dir = b1 - a2;
            if join_dir.hypot() < 1e-9 {
                continue;
            }
            let side_a = join_dir.cross(a1 - a2);
            let side_b = join_dir.cross(b2 - b1);
            if side_a * side_b <= 0.0 {
                continue;
            }
            let d = match line_intersection(a1, a2, b1, b2) {
                Some(d) => d,
                None => continue,
            };
            let la = dist_pt(a1, a2);
            let lad = dist_pt(a2, d);
            let ldb = dist_pt(d, b1);
            let lb = dist_pt(b1, b2);
            if lad < 1e-9 || lb < 1e-9 {
                continue;
            }
            let p0 = la / lad;
            let p1 = ldb / lb;
            if !(p0.is_finite() && p1.is_finite()) || p0 <= 0.0 || p1 <= 0.0 {
                continue;
            }
            let ratio = (p0 * p1).sqrt();
            let t = ratio / (ratio + 1.0);
            let target = a2 + (b1 - a2) * t;
            let shift = target - b0;
            let shift = if shift.hypot() > HARMONIZE_MAX_SHIFT {
                shift * (HARMONIZE_MAX_SHIFT / shift.hypot())
            } else {
                shift
            };
            let moved = b0 + shift;
            contour.segs[p][3] = moved;
            contour.segs[i][0] = moved;
        }
    }
    contour
}
