// Copyright 2026 the img2bez Authors
// SPDX-License-Identifier: Apache-2.0 OR MIT

use kurbo::Point;

use crate::pipeline::vectorize::fit::{FittedContour, SplitKind};

use super::{
    RasterTarget, all_polylines, band_loss, collect_band, optimize_handles,
    poly_turn, ray_intersection, sample_cubic,
};

/// Tighter reach cap for inflection-collapse curves: gentle curves
/// over-fit into bunched handles; ~2/3 reach spaces controls near thirds.
const INFLECTION_REACH_MAX: f64 = 0.67;

/// Minimum turn (radians) each flanking curve must sweep for a bridging
/// line to read as a real inflection.
const INFLECTION_LINE_MIN_TURN: f64 = 0.12;
/// Only collapse a line whose straight fit is at least this poor (the ink
/// is genuinely curved there); a real flat fits its line well and stays
/// below this, which keeps the pass off intentional flats.
const INFLECTION_LINE_MIN_BASE: f64 = 0.06;
/// Accept the rebuilt pair within this factor of the original loss (plus
/// floor): generous — the point structure is the goal — but not so loose
/// it bulges a genuinely straight section.
const INFLECTION_LINE_FACTOR: f64 = 1.8;
const INFLECTION_LINE_FLOOR: f64 = 5e-3;
/// Extremum-line branch: a SAME-bending axis-aligned flat at a curve's
/// extremum fits the ink well locally (no MIN_BASE gate); it collapses
/// only when a curve fits at least as well. Must point within this sine
/// of an axis.
const EXTREMUM_LINE_AXIS_SIN: f64 = 0.26; // sin 15°
/// Extremum collapse only fires on a line short relative to its flanking
/// arcs: leftover crown flats run ratio <= ~2.3, designed rounded-rect
/// edges >= ~4.5 and must never collapse.
const EXTREMUM_LINE_MAX_FLANK_RATIO: f64 = 3.0;
/// Absolute backstop in FONT units (ultra-bold designs can read ratio
/// ~1.4): a leftover crown flat is a few dozen units at most; anything
/// longer is a designed edge.
const EXTREMUM_LINE_MAX_UNITS: f64 = 60.0;
const EXTREMUM_LINE_FLOOR: f64 = 4e-3;

/// Corner micro-line gates: the stub's chord must be under this many px
/// (absolute) and under CORNER_MICRO_MAX_FRAC of each flanking line —
/// longer means a real flat, left alone.
const CORNER_MICRO_MAX_CHORD: f64 = 18.0;
const CORNER_MICRO_MAX_FRAC: f64 = 0.35;
/// The recovered vertex must sit within this many px of the micro-line —
/// near-parallel flanks whose intersection shoots far away are no corner.
const CORNER_MICRO_MAX_REACH: f64 = 24.0;
/// Collapse only when the sharp corner fits within this factor of the
/// micro-line's loss (plus floor); a genuine flat fits its line far
/// better and survives.
const CORNER_MICRO_FACTOR: f64 = 1.3;
const CORNER_MICRO_FLOOR: f64 = 2e-3;

/// Re-express a near-straight line bridging an inflection as two curves
/// meeting at one smooth inflection point, the way a designer draws a
/// spine; accepted only when the rebuild reproduces the ink about as
/// well as the original three segments.
pub(super) fn smooth_inflection_lines(
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
        let mut changed = false;
        for li in 0..n {
            let (p, ni) = ((li + n - 1) % n, (li + 1) % n);
            if !contour.is_line[li] || contour.is_line[p] || contour.is_line[ni]
            {
                continue;
            }
            // Must be a fitter straight run (Tangent joints both ends).
            if contour.joint_kind[li] != SplitKind::Tangent
                || contour.joint_kind[ni] != SplitKind::Tangent
            {
                continue;
            }
            let (tp, tn) = (poly_turn(&polys[p]), poly_turn(&polys[ni]));
            let s0 = contour.segs[p][0];
            let s1 = contour.segs[ni][3];
            let (a_line, b_line) = (contour.segs[li][0], contour.segs[li][3]);
            let line_vec = b_line - a_line;
            let u0 = contour.segs[p][1] - s0; // handle direction leaving S0
            let u1 = contour.segs[ni][2] - s1; // handle direction entering S1
            if line_vec.hypot() < 1e-6 || u0.hypot() < 1e-9 || u1.hypot() < 1e-9
            {
                continue;
            }
            let line_dir = line_vec.normalize();

            // Opposite curvature both sides → an inflection bridge; an
            // axis-aligned line with one bending side is an extremum
            // flat; a same-bending diagonal is a genuine flat — skip.
            let opposite = tp * tn < 0.0
                && tp.abs() >= INFLECTION_LINE_MIN_TURN
                && tn.abs() >= INFLECTION_LINE_MIN_TURN;
            // Flank-ratio gate applies to BOTH branches: an italic stem
            // has opposite-curving flanks like a spine bridge, and must
            // never collapse to a point.
            let arc_len = |poly: &[Point]| -> f64 {
                poly.windows(2).map(|w| (w[1] - w[0]).hypot()).sum()
            };
            let flank = arc_len(&polys[p]).min(arc_len(&polys[ni]));
            if debug {
                eprintln!(
                    "    refine: infl-line cand ({:.0},{:.0})-({:.0},{:.0}) line={:.0} flank={:.0} ratio={:.2}",
                    a_line.x,
                    a_line.y,
                    b_line.x,
                    b_line.y,
                    line_vec.hypot(),
                    flank,
                    line_vec.hypot() / flank.max(1e-9)
                );
            }
            if line_vec.hypot() > flank * EXTREMUM_LINE_MAX_FLANK_RATIO {
                continue;
            }
            let extremum = if opposite {
                None
            } else if tp.abs().max(tn.abs()) >= INFLECTION_LINE_MIN_TURN {
                if line_vec.hypot() > EXTREMUM_LINE_MAX_UNITS * rt.px_per_unit {
                    continue; // designed edge, whatever its flank ratio
                }
                if line_dir.x.abs() < EXTREMUM_LINE_AXIS_SIN {
                    Some(SplitKind::ExtremumX) // vertical line → vertical tangent
                } else if line_dir.y.abs() < EXTREMUM_LINE_AXIS_SIN {
                    Some(SplitKind::ExtremumY) // horizontal line → horizontal tangent
                } else {
                    continue;
                }
            } else {
                continue;
            };

            // Band over the three-segment region, immediate neighbours fixed.
            let mut region = polys[p].clone();
            region.extend_from_slice(&polys[li][1..]);
            region.extend_from_slice(&polys[ni][1..]);
            let band = collect_band(
                rt,
                &region,
                &[&polys[(p + n - 1) % n], &polys[(ni + 1) % n]],
            );
            if band.len() < 16 {
                continue;
            }
            let base = band_loss(&band, &region, ink_left);
            // Inflection lines must fit the ink poorly (a genuine flat
            // fits well and is left alone); extremum lines fit well
            // locally, so no base gate there.
            if extremum.is_none() && base < INFLECTION_LINE_MIN_BASE {
                continue;
            }

            // Place the inflection point along the line; optimize each
            // flanking cubic's handles with the shared tangent fixed.
            let mut best: Option<(f64, [Point; 4], [Point; 4])> = None;
            for frac in [0.35, 0.5, 0.65] {
                let pt = a_line.lerp(b_line, frac);
                let (chord0, chord1) = ((pt - s0).hypot(), (s1 - pt).hypot());
                let (c1, _) = optimize_handles(
                    s0,
                    u0.normalize(),
                    -line_dir,
                    pt,
                    (chord0 / 3.0, chord0 / 3.0),
                    &band,
                    ink_left,
                    &[],
                    &[],
                );
                let poly1 = sample_cubic(&c1);
                let (c2, loss) = optimize_handles(
                    pt,
                    line_dir,
                    u1.normalize(),
                    s1,
                    (chord1 / 3.0, chord1 / 3.0),
                    &band,
                    ink_left,
                    &poly1,
                    &[],
                );
                if best.as_ref().is_none_or(|(bl, _, _)| loss < *bl) {
                    best = Some((loss, c1, c2));
                }
            }
            let Some((loss, c1, c2)) = best else { continue };
            // Inflection: trade a little raster fit for structure.
            // Extremum: only if the curve fits at least as well as the line.
            let accept = match extremum {
                None => {
                    loss <= INFLECTION_LINE_FACTOR * base
                        + INFLECTION_LINE_FLOOR
                }
                Some(_) => loss <= base + EXTREMUM_LINE_FLOOR,
            };
            if debug {
                eprintln!(
                    "    refine: {} inflection-line ({:.0},{:.0})-({:.0},{:.0}) base={:.5} rebuilt={:.5}",
                    if accept { "ACCEPT" } else { "reject" },
                    a_line.x,
                    a_line.y,
                    b_line.x,
                    b_line.y,
                    base,
                    loss
                );
            }
            if !accept {
                continue;
            }

            // Replace [curve p, line li, curve ni] with [c1, c2]; cap each
            // curve's reach so the controls space out near thirds.
            contour.segs[p] = cap_reach(c1, INFLECTION_REACH_MAX);
            contour.is_line[p] = false;
            contour.segs[li] = cap_reach(c2, INFLECTION_REACH_MAX);
            contour.is_line[li] = false;
            contour.joint_kind[li] = extremum.unwrap_or(SplitKind::Inflection);
            contour.segs.remove(ni);
            contour.is_line.remove(ni);
            contour.joint_kind.remove(ni);
            changed = true;
            break;
        }
        if !changed {
            break;
        }
    }
    contour
}

/// Collapse a doubled corner pair joined by a tiny stub into the single
/// true vertex when at least one flank is straight: line+line meet at
/// their intersection; curve+line extends the curve onto the edge.
/// Raster-gated so a genuine short flat survives; the convex counterpart
/// of `junction_flats`.
pub(super) fn collapse_corner_micro_lines(
    mut contour: FittedContour,
    rt: &RasterTarget,
    ink_left: bool,
    debug: bool,
) -> FittedContour {
    loop {
        let n = contour.segs.len();
        if n < 4 {
            break;
        }
        let mut collapsed = false;
        for i in 0..n {
            let p = (i + n - 1) % n;
            let nx = (i + 1) % n;
            // A short straight stub with at least one straight neighbour.
            if !contour.is_line[i]
                || (!contour.is_line[p] && !contour.is_line[nx])
            {
                continue;
            }
            let micro = contour.segs[i];
            let chord = (micro[3] - micro[0]).hypot();
            if chord > CORNER_MICRO_MAX_CHORD {
                continue;
            }
            let (lp, ln) =
                (seg_chord(&contour.segs[p]), seg_chord(&contour.segs[nx]));
            if lp < 1e-6 || ln < 1e-6 {
                continue;
            }
            if chord > CORNER_MICRO_MAX_FRAC * lp.min(ln) {
                continue;
            }
            // Rebuild the flanking segments onto one vertex; `None` if no
            // clean vertex is available.
            let Some((new_p, new_nx)) = collapse_targets(&contour, p, i, nx)
            else {
                continue;
            };
            // Score the original three segments against the collapsed pair.
            let mut region = poly_of(&contour, p);
            region.extend_from_slice(&poly_of(&contour, i)[1..]);
            region.extend_from_slice(&poly_of(&contour, nx)[1..]);
            let mut cand = sample_seg(&new_p, contour.is_line[p]);
            cand.extend_from_slice(
                &sample_seg(&new_nx, contour.is_line[nx])[1..],
            );
            let before = poly_of(&contour, (p + n - 1) % n);
            let after = poly_of(&contour, (nx + 1) % n);
            let band = collect_band(rt, &region, &[&before, &after]);
            if band.len() < 12 {
                continue;
            }
            let loss_orig = band_loss(&band, &region, ink_left);
            let loss_cand = band_loss(&band, &cand, ink_left);
            let accept = loss_cand
                <= CORNER_MICRO_FACTOR * loss_orig + CORNER_MICRO_FLOOR;
            if debug {
                let j = new_nx[0];
                eprintln!(
                    "    refine: {} corner-micro ({:.1},{:.1}) chord={:.1} doubled={:.5} vertex={:.5}",
                    if accept { "ACCEPT" } else { "reject" },
                    j.x,
                    j.y,
                    chord,
                    loss_orig,
                    loss_cand
                );
            }
            if !accept {
                continue;
            }
            contour.segs[p] = new_p;
            contour.segs[nx] = new_nx;
            contour.joint_kind[nx] = SplitKind::Corner;
            contour.segs.remove(i);
            contour.is_line.remove(i);
            contour.joint_kind.remove(i);
            collapsed = true;
            break;
        }
        if !collapsed {
            break;
        }
    }
    contour
}

/// Rebuilt flanking segments that collapse the micro-line at `i` onto one
/// vertex, or `None` if no clean vertex exists; the caller drops the micro.
fn collapse_targets(
    contour: &FittedContour,
    p: usize,
    i: usize,
    nx: usize,
) -> Option<([Point; 4], [Point; 4])> {
    let micro = contour.segs[i];
    let (line_p, line_nx) = (contour.is_line[p], contour.is_line[nx]);
    if line_p && line_nx {
        // Both straight: meet at the two lines' extended intersection.
        let a = contour.segs[p][0];
        let b = contour.segs[nx][3];
        let corner = ray_intersection(a, micro[0] - a, b, micro[3] - b)?;
        if (corner - micro[0].midpoint(micro[3])).hypot()
            > CORNER_MICRO_MAX_REACH
        {
            return None;
        }
        Some(([a, a, corner, corner], [corner, corner, b, b]))
    } else if line_nx {
        // Curve before, straight stem after: extend the curve onto the
        // stem's near end, co-translating its end handle.
        let j = micro[3];
        let delta = j - micro[0];
        let mut np = contour.segs[p];
        np[2] += delta;
        np[3] = j;
        Some((np, contour.segs[nx]))
    } else {
        // Straight stem before, curve after: mirror of the above.
        let j = micro[0];
        let delta = j - micro[3];
        let mut nn = contour.segs[nx];
        nn[1] += delta;
        nn[0] = j;
        Some((contour.segs[p], nn))
    }
}

/// Chord length of a segment (endpoint to endpoint).
fn seg_chord(seg: &[Point; 4]) -> f64 {
    (seg[3] - seg[0]).hypot()
}

/// Sampled polyline of a segment given whether it is a line.
fn sample_seg(seg: &[Point; 4], is_line: bool) -> Vec<Point> {
    if is_line {
        vec![seg[0], seg[3]]
    } else {
        sample_cubic(seg)
    }
}

/// Sampled polyline of a single segment (2 points for a line).
fn poly_of(contour: &FittedContour, i: usize) -> Vec<Point> {
    if contour.is_line[i] {
        vec![contour.segs[i][0], contour.segs[i][3]]
    } else {
        sample_cubic(&contour.segs[i])
    }
}

/// Scale a cubic's handles down equally so their combined chord-projected
/// reach is at most `max_frac` of the chord; endpoints stay fixed.
fn cap_reach(seg: [Point; 4], max_frac: f64) -> [Point; 4] {
    let (a, b) = (seg[0], seg[3]);
    let chord_vec = b - a;
    let chord = chord_vec.hypot();
    if chord < 1e-9 {
        return seg;
    }
    let e = chord_vec / chord;
    let h0 = seg[1] - a; // handle leaving the start
    let h1 = seg[2] - b; // handle entering the end
    let reach = h0.dot(e) - h1.dot(e);
    let limit = chord * max_frac;
    if reach > limit {
        let s = limit / reach;
        return [a, a + h0 * s, b + h1 * s, b];
    }
    seg
}
