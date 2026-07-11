// Copyright 2026 the img2bez Authors
// SPDX-License-Identifier: Apache-2.0 OR MIT

use crate::model::geom::line_seg;
use crate::pipeline::vectorize::fit::{FittedContour, SplitKind};

use super::RasterTarget;

/// A convex tip's corner pair closer than this (font units) collapses to
/// the vertex where its edges meet (the pair only exists because the
/// iso-contour rounds the apex); designed chamfers sit above the limit.
const TIP_WELD_MAX_CHORD_UNITS: f64 = 12.0;
/// Max distance (font units) of the welded vertex beyond the pair: a
/// needle taper's true apex lies well past the traced pair; the cap
/// excludes near-parallel flanks, which must not collapse to a point.
const TIP_WELD_MAX_REACH_UNITS: f64 = 45.0;
/// Both joins must break at least this much (degrees): a needle tip
/// breaks ~80-90 per side, a designed facet pair 20-30 and must not weld.
const TIP_WELD_MIN_BREAK_DEG: f64 = 45.0;
/// Ink-continuation probe (fraction toward the apex, min coverage): a cut
/// tip has blank paper past its flat, a needle's spine still carries ink.
const TIP_WELD_INK_PROBE_T: f64 = 0.35;
const TIP_WELD_INK_MIN_COVERAGE: f64 = 0.2;

pub(super) fn weld_convex_tips(
    mut contour: FittedContour,
    rt: &RasterTarget,
    ink_left: bool,
    debug: bool,
) -> FittedContour {
    let px_per_unit = rt.px_per_unit;
    loop {
        let n = contour.segs.len();
        if n < 3 {
            return contour;
        }
        let mut welded = false;
        for i in 0..n {
            if !contour.is_line[i] {
                continue;
            }
            let a = contour.segs[i][0];
            let b = contour.segs[i][3];
            if (b - a).hypot() > TIP_WELD_MAX_CHORD_UNITS * px_per_unit {
                continue;
            }
            let p = (i + n - 1) % n;
            let nx = (i + 1) % n;
            if p == i || nx == i {
                continue;
            }
            let dbg_cand = debug && std::env::var("IMG2BEZ_DEBUG_WELD").is_ok();
            let v_in = if contour.is_line[p] {
                contour.segs[p][3] - contour.segs[p][0]
            } else {
                contour.segs[p][3] - contour.segs[p][2]
            };
            let v_out = if contour.is_line[nx] {
                contour.segs[nx][3] - contour.segs[nx][0]
            } else {
                contour.segs[nx][1] - contour.segs[nx][0]
            };
            if v_in.hypot() < 1e-9 || v_out.hypot() < 1e-9 {
                continue;
            }
            let vi = v_in.normalize();
            let vo = v_out.normalize();
            // Both ends must actually break (a doubled corner, not a smooth
            // passage over a stub the micro-line pass handles).
            let flat_dir = (b - a).normalize();
            let min_cos = TIP_WELD_MIN_BREAK_DEG.to_radians().cos();
            if dbg_cand {
                eprintln!(
                    "    weld-cand ({:.1},{:.1})-({:.1},{:.1}) breaks {:.0}°/{:.0}° cross={:.2} ink_left={}",
                    a.x,
                    a.y,
                    b.x,
                    b.y,
                    vi.dot(flat_dir).clamp(-1.0, 1.0).acos().to_degrees(),
                    flat_dir.dot(vo).clamp(-1.0, 1.0).acos().to_degrees(),
                    vi.cross(vo),
                    ink_left
                );
            }
            let log_weld = |decision: &str, cov: f64| {
                if crate::ml::mldata::enabled() {
                    crate::ml::mldata::log(
                        "tip_weld",
                        Some((a.midpoint(b).x, a.midpoint(b).y)),
                        &[
                            ("chord", (b - a).hypot()),
                            (
                                "break_in_deg",
                                vi.dot(flat_dir)
                                    .clamp(-1.0, 1.0)
                                    .acos()
                                    .to_degrees(),
                            ),
                            (
                                "break_out_deg",
                                flat_dir
                                    .dot(vo)
                                    .clamp(-1.0, 1.0)
                                    .acos()
                                    .to_degrees(),
                            ),
                            ("cross", vi.cross(vo)),
                            ("ink_cov", cov),
                            ("px_per_unit", px_per_unit),
                        ],
                        decision,
                    );
                }
            };
            if vi.dot(flat_dir) > min_cos && flat_dir.dot(vo) > min_cos {
                log_weld("keep_flat_smooth", f64::NAN);
                continue;
            }
            // Convex only: with ink on the left the boundary turns left
            // (positive cross) around features that stick OUT of the ink.
            let cross = vi.cross(vo);
            let convex = if ink_left { cross > 0.0 } else { cross < 0.0 };
            if !convex {
                continue;
            }
            // The apex: intersection of the two flank tangent lines.
            let denom = vi.cross(vo);
            if denom.abs() < 1e-9 {
                continue;
            }
            let t = (b - a).cross(vo) / denom;
            let d = a + vi * t;
            let mid = a.midpoint(b);
            if t < 0.0
                || (d - mid).hypot() > TIP_WELD_MAX_REACH_UNITS * px_per_unit
            {
                continue;
            }
            // The ink must actually continue toward the apex: a cut tip
            // has nothing past its flat, and welding would push the
            // outline into blank paper. Ink evidence, so the raster calls it.
            let probe = mid.lerp(d, TIP_WELD_INK_PROBE_T);
            let cov = rt.coverage(probe.x, probe.y);
            if cov < TIP_WELD_INK_MIN_COVERAGE {
                if debug {
                    eprintln!(
                        "    refine: weld skipped at ({:.1},{:.1}): no ink toward apex (cov {:.2})",
                        mid.x, mid.y, cov
                    );
                }
                log_weld("keep_flat_cut", cov);
                continue;
            }
            if debug {
                eprintln!(
                    "    refine: WELD tip ({:.1},{:.1})+({:.1},{:.1}) -> ({:.1},{:.1})",
                    a.x, a.y, b.x, b.y, d.x, d.y
                );
            }
            log_weld("weld_vertex", cov);
            if contour.is_line[p] {
                contour.segs[p] = line_seg(contour.segs[p][0], d);
            } else {
                contour.segs[p][3] = d;
            }
            if contour.is_line[nx] {
                contour.segs[nx] = line_seg(d, contour.segs[nx][3]);
            } else {
                contour.segs[nx][0] = d;
            }
            contour.segs.remove(i);
            contour.is_line.remove(i);
            contour.joint_kind.remove(i);
            let j = if i < nx { i } else { 0 };
            contour.joint_kind[j] = SplitKind::Corner;
            welded = true;
            break;
        }
        if !welded {
            return contour;
        }
    }
}
