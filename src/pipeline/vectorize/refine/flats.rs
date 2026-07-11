// Copyright 2026 the img2bez Authors
// SPDX-License-Identifier: Apache-2.0 OR MIT

use kurbo::{Point, Vec2};

use crate::model::geom::line_seg;
use crate::pipeline::vectorize::fit::{FittedContour, SplitKind};

use super::{
    BandPt, RasterTarget, all_polylines, band_loss, collect_band, golden_min,
    ray_intersection,
};

/// Junction-flat gates (px unless noted). A junction cluster is a corner
/// joint or one/two short segments (the rounded tip), flanked by two
/// longer "wall" segments descending into the valley.
const FLAT_CLUSTER_MAX_LEN: f64 = 26.0;
/// Walls shorter than this are junction debris, not walls.
const FLAT_WALL_MIN_CHORD: f64 = 8.0;
/// Minimum wall-to-wall turn (degrees); gentler is ordinary curvature.
const FLAT_MIN_TURN_DEG: f64 = 50.0;
/// The valley's depth axis must lie within this of H/V (degrees):
/// reference junction flats are axis-aligned.
const FLAT_AXIS_TOL_DEG: f64 = 35.0;
/// Acceptable flat widths in FONT units (references run ~2-8 units);
/// scaled to px via px_per_unit so resolution doesn't change the gate.
const FLAT_MIN_WIDTH: f64 = 2.0;
const FLAT_MAX_WIDTH: f64 = 12.0;
/// Prefer the narrowest flat within this factor of the best: the ramp
/// model under-penalizes wide flats and references draw the minimal one.
const FLAT_NARROW_TIE: f64 = 1.08;
/// Only the near portion of each wall joins the judged band; the far
/// part barely moves when the wall is re-anchored.
const FLAT_WALL_NEAR_LEN: f64 = 30.0;
/// Candidate depth range around the current tip: past it (AA + smoothing
/// recede narrow wedge tips) and back toward the opening.
const FLAT_DEEP_SLACK: f64 = 40.0;
const FLAT_SHALLOW_SLACK: f64 = 12.0;
/// Leave junctions alone when the current outline already reproduces
/// the raster this well.
const FLAT_MIN_BASE_LOSS: f64 = 0.003;
/// Accept only when the flat beats BOTH the current structure and the
/// sharp-vertex alternative (a genuinely sharp corner ties with the
/// sharp candidate, so it can never pass).
const FLAT_ACCEPT_FACTOR: f64 = 0.8;
const FLAT_ABS_MAX: f64 = 0.03;
/// Max wall tilt (degrees, pivoting about the far endpoint) for bare
/// joints to reach the true valley tip, with a 3px displacement floor so
/// short walls can still reach the flat.
const FLAT_MAX_TILT_DEG: f64 = 3.0;
const FLAT_TILT_FLOOR: f64 = 3.0;

// ── Junction flats ───────────────────────────────────────────────────

/// Rebuild sharp valleys as tiny axis-aligned flats where the raster
/// prefers them (see module docs). One junction is rebuilt per scan;
/// the scan restarts until no candidate wins.
pub(super) fn junction_flats(
    mut contour: FittedContour,
    rt: &RasterTarget,
    ink_left: bool,
    debug: bool,
) -> FittedContour {
    if debug && std::env::var("IMG2BEZ_DEBUG_FLATS").is_ok() {
        for (j, seg) in contour.segs.iter().enumerate() {
            eprintln!(
                "      seg[{j}] ({:.1},{:.1})->({:.1},{:.1}) chord={:.1} line={} kind={:?}",
                seg[0].x,
                seg[0].y,
                seg[3].x,
                seg[3].y,
                (seg[3] - seg[0]).hypot(),
                contour.is_line[j],
                contour.joint_kind[j]
            );
        }
    }
    loop {
        let n = contour.segs.len();
        if n < 6 {
            return contour;
        }
        let polys = all_polylines(&contour);
        // Evaluate every cluster expression of each junction, apply the
        // single strongest accept, rescan. Rank structural parsimony
        // first (a wider cluster spends fewer points), then ratio.
        let mut best: Option<(FittedContour, (i32, f64))> = None;
        for i in 0..n {
            for m in 0..=2usize {
                if let Some((next, ratio)) =
                    try_flat(&contour, rt, ink_left, i, m, &polys, debug)
                {
                    let key = (1 - m as i32, ratio);
                    if best.as_ref().is_none_or(|(_, bk)| key < *bk) {
                        best = Some((next, key));
                    }
                }
            }
        }
        match best {
            Some((next, _)) => contour = next,
            None => return contour,
        }
    }
}

/// Judge the junction cluster of `m` segments starting at segment `i`
/// (m = 0: the bare joint at its start); when the flat wins on the
/// raster, returns the rebuilt contour and its improvement ratio.
fn try_flat(
    contour: &FittedContour,
    rt: &RasterTarget,
    ink_left: bool,
    i: usize,
    m: usize,
    polys: &[Vec<Point>],
    debug: bool,
) -> Option<(FittedContour, f64)> {
    let n = contour.segs.len();
    if n < m + 4 {
        return None;
    }
    let p = (i + n - 1) % n; // wall into the valley
    let k = (i + m) % n; // wall out of the valley
    let in_cluster = |j: usize| (0..m).any(|s| (i + s) % n == j);
    // Debug aid: report which gate rejected a candidate.
    let verbose = debug && std::env::var("IMG2BEZ_DEBUG_FLATS").is_ok();
    let why = |stage: u32, reason: &str| {
        if verbose || (debug && m > 0 && stage > 0) {
            eprintln!(
                "    refine: flat-cand i={} m={} near ({:.1}, {:.1}) skipped: {}",
                i, m, contour.segs[i][0].x, contour.segs[i][0].y, reason
            );
        }
    };

    // Cheap structural gates first. Cluster size uses chords — junk
    // micro-segments carry inflated arc length from degenerate handles.
    if m > 0 {
        let len: f64 = (0..m)
            .map(|s| {
                let seg = &contour.segs[(i + s) % n];
                (seg[3] - seg[0]).hypot()
            })
            .sum();
        if len > FLAT_CLUSTER_MAX_LEN {
            why(0, "cluster too long");
            return None;
        }
        // An axis-aligned tiny line IS already a junction flat.
        if m == 1 && contour.is_line[i] {
            let v = contour.segs[i][3] - contour.segs[i][0];
            if v.x.abs() < 1e-6 || v.y.abs() < 1e-6 {
                return None;
            }
        }
    }
    for wall in [p, k] {
        if (contour.segs[wall][3] - contour.segs[wall][0]).hypot()
            < FLAT_WALL_MIN_CHORD
        {
            why(0, "wall too short");
            return None;
        }
    }
    let sec_in = approach_dir(&polys[p], false)?;
    let sec_out = approach_dir(&polys[k], true)?;
    let turn = sec_in.dot(sec_out).clamp(-1.0, 1.0).acos();
    if turn < FLAT_MIN_TURN_DEG.to_radians() {
        why(1, "turn too gentle");
        return None;
    }
    // Junction flats live in CONCAVE valleys (the boundary dents into the
    // ink); a convex turn is a terminal tip and gets a vertex, not a flat.
    let cross = sec_in.cross(sec_out);
    let concave = if ink_left { cross < 0.0 } else { cross > 0.0 };
    if !concave {
        why(1, "convex tip, not a junction");
        return None;
    }

    // Depth axis: into the valley, snapped exactly onto x or y.
    let w = sec_in - sec_out;
    if w.hypot() < 0.2 {
        why(1, "walls antiparallel");
        return None; // depth direction is ill-defined
    }
    let w = w.normalize();
    let depth = if w.x.abs() >= w.y.abs() {
        Vec2::new(w.x.signum(), 0.0)
    } else {
        Vec2::new(0.0, w.y.signum())
    };
    if w.dot(depth) < (FLAT_AXIS_TOL_DEG.to_radians()).cos() {
        why(1, "valley is diagonal");
        return None; // reference flats are H/V
    }
    let d = |q: Point| q.to_vec2().dot(depth);
    // Wall ray direction: whichever of the end tangent and approach
    // secant descends more steeply (G1 alignment can rotate the tangent
    // parallel to the eventual flat; the secant flares at the very end).
    let tangent =
        |seg: &[Point; 4], is_line: bool, at_start: bool| -> Option<Vec2> {
            let v = if is_line {
                seg[3] - seg[0]
            } else if at_start {
                seg[1] - seg[0]
            } else {
                seg[3] - seg[2]
            };
            (v.hypot() > 1e-9).then(|| v.normalize())
        };
    let dir_in = match tangent(&contour.segs[p], contour.is_line[p], false) {
        Some(t) if t.dot(depth) > sec_in.dot(depth) => t,
        _ => sec_in,
    };
    let dir_out = match tangent(&contour.segs[k], contour.is_line[k], true) {
        Some(t) if t.dot(depth) < sec_out.dot(depth) => t,
        _ => sec_out,
    };
    // Walls must actually descend into / ascend out of the valley, if
    // only slightly: an arch can leave a stem crotch almost flat.
    if dir_in.dot(depth) < 0.05 || dir_out.dot(depth) > -0.05 {
        why(1, "walls do not descend");
        return None;
    }
    // Expected travel direction along the flat.
    let tang = (sec_in - depth * sec_in.dot(depth))
        + (sec_out - depth * sec_out.dot(depth));
    let tang = if tang.hypot() < 1e-6 {
        return None;
    } else {
        tang.normalize()
    };

    let a_in = contour.segs[p][3]; // cluster start (end of wall_in)
    let a_out = contour.segs[k][0]; // cluster end (start of wall_out)
    let cluster: Vec<Point> = if m == 0 {
        vec![contour.segs[i][0]]
    } else {
        let mut pts = Vec::new();
        for s in 0..m {
            pts.extend_from_slice(&polys[(i + s) % n]);
        }
        pts
    };
    let d_deep = cluster
        .iter()
        .map(|&q| d(q))
        .fold(d(a_in).max(d(a_out)), f64::max);
    let deepest = cluster
        .iter()
        .copied()
        .max_by(|x, y| d(*x).total_cmp(&d(*y)))
        .unwrap_or(a_in);

    // Find the wedge's true tip by walking the bisector ray until
    // coverage flips: a fixed slack could punch out a thin stroke's far
    // side, drowning the comparison in phantom errors.
    let interior = {
        let q = deepest - depth * 4.0; // safely on the wedge's open side
        rt.coverage(q.x, q.y)
    };
    let mut reach = FLAT_DEEP_SLACK;
    let mut s = -4.0;
    while s < FLAT_DEEP_SLACK {
        s += 1.0;
        let q = deepest + depth * s;
        if (rt.coverage(q.x, q.y) - interior).abs() > 0.5 {
            reach = (s + 3.0).max(4.0);
            break;
        }
    }

    // Candidate depth range; a bare joint has no usable intersection, so
    // its candidates tilt the walls instead.
    let sharp_v = if m > 0 {
        ray_intersection(a_in, dir_in, a_out, -dir_out)
    } else {
        None
    };
    let deep_lim = match sharp_v {
        Some(v) => (d(v) + 1.0).min(d_deep + reach),
        None => d_deep + reach,
    };
    let shallow_lim = d_deep - FLAT_SHALLOW_SLACK;
    if deep_lim <= shallow_lim {
        why(1, "no depth range");
        return None;
    }

    // Judged geometry: near wall portions, densified, plus extension rays
    // past the anchors. The walls' far remainders join the FIXED set so
    // band pixels near the trim point are not misattributed.
    let (win_near, win_rest) = split_wall_near(&polys[p], false);
    let (wout_near, wout_rest) = split_wall_near(&polys[k], true);
    let win_near = densify(&win_near);
    let wout_near = densify(&wout_near);
    let ext_in = ray_points(a_in, dir_in, (deep_lim - d(a_in)).max(0.0) + 2.0);
    let ext_out =
        ray_points(a_out, -dir_out, (deep_lim - d(a_out)).max(0.0) + 2.0);

    let mut region = win_near.clone();
    region.extend_from_slice(&cluster);
    region.extend_from_slice(&wout_near);
    region.extend_from_slice(&ext_in);
    region.extend_from_slice(&ext_out);
    // Bisector rays so band membership also covers the strip between the
    // wall extensions, where the candidate flats live.
    for off in [-6.0, 0.0, 6.0] {
        region.extend(ray_points(
            deepest + tang * off,
            depth,
            (deep_lim - d_deep).max(0.0) + 2.0,
        ));
    }
    let fixed = [&win_rest, &wout_rest];
    let full_band = collect_band(rt, &region, &fixed);
    // Judge only near the junction: wall pixels are near-identical across
    // candidates and merely dilute the comparison.
    let center = a_in.midpoint(a_out);
    let tip = deepest + depth * (deep_lim - d_deep).max(0.0);
    let judge_r = (a_in - center).hypot().max((tip - center).hypot()) + 6.0;
    let band: Vec<BandPt> = full_band
        .iter()
        .map(|bp| BandPt {
            p: bp.p,
            src: bp.src,
            fixed_sd: bp.fixed_sd,
        })
        .filter(|bp| (bp.p - center).hypot() <= judge_r)
        .collect();
    if band.len() < 16 {
        why(1, "band too small");
        return None;
    }

    // The incumbent: walls plus the cluster as fitted.
    let mut base_poly = win_near.clone();
    base_poly.extend_from_slice(&cluster);
    base_poly.extend_from_slice(&wout_near);
    let loss_base = band_loss(&band, &base_poly, ink_left);
    if loss_base < FLAT_MIN_BASE_LOSS {
        if debug {
            eprintln!(
                "    refine: flat-cand at ({:.1}, {:.1}) m={} base={:.5} below floor",
                a_in.x, a_in.y, m, loss_base
            );
        }
        return None;
    }

    const STEPS: usize = 24;
    let (loss_sharp, loss_flat, fa, fb) = if m == 0 {
        // Bare joint: candidates pivot each wall about its far endpoint
        // onto a flat of width `wdt` at depth t.
        let vertex = contour.segs[i][0];
        let far_in = contour.segs[p][0];
        let far_out = contour.segs[k][3];
        let max_tilt = FLAT_MAX_TILT_DEG.to_radians().tan();
        let tilt_ok = |q: Point, far: Point, near: Point| {
            let old = near - far;
            let len = old.hypot();
            if len < 1e-9 {
                return false;
            }
            let perp = (old * (1.0 / len)).cross(q - far).abs();
            perp <= (len * max_tilt).max(FLAT_TILT_FLOOR)
        };
        // Tilted walls are modeled by shearing the fitted wall samples,
        // preserving the wall's curvature so the comparison stays fair.
        let d0 = d(vertex);
        // Sharp competitor: the same vertex pushed to its best depth,
        // else a shallow-but-sharp corner would take a flat purely for
        // the depth correction.
        let eval_sharp = |s: f64| {
            let v2 = vertex + depth * s;
            if !tilt_ok(v2, far_in, a_in) || !tilt_ok(v2, far_out, a_out) {
                return f64::INFINITY;
            }
            let mut poly = shear(&win_near, v2 - vertex, true);
            poly.extend(shear(&wout_near, v2 - vertex, false));
            band_loss(&band, &poly, ink_left)
        };
        let s = golden_min(eval_sharp, shallow_lim - d0, deep_lim - d0);
        let loss_sharp = eval_sharp(s);
        let eval0 = |t: f64, wdt: f64| {
            let center = vertex + depth * (t - d0);
            let qa = center - tang * (wdt * 0.5);
            let qb = center + tang * (wdt * 0.5);
            if !tilt_ok(qa, far_in, a_in) || !tilt_ok(qb, far_out, a_out) {
                return f64::INFINITY;
            }
            let mut poly = shear(&win_near, qa - vertex, true);
            poly.extend(shear(&wout_near, qb - vertex, false));
            band_loss(&band, &poly, ink_left)
        };
        // Candidate widths in FONT units, scaled to pixels per source.
        const WIDTHS: [f64; 8] = [2.0, 3.0, 4.0, 5.0, 6.5, 8.0, 10.0, 12.0];
        let widths: Vec<f64> =
            WIDTHS.iter().map(|w| w * rt.px_per_unit).collect();
        let mut per_w = [(f64::INFINITY, f64::NAN); WIDTHS.len()]; // (loss, t)
        for (wi, &wdt) in widths.iter().enumerate() {
            for s in 0..=STEPS {
                let t = shallow_lim
                    + (deep_lim - shallow_lim) * s as f64 / STEPS as f64;
                let loss = eval0(t, wdt);
                if loss < per_w[wi].0 {
                    per_w[wi] = (loss, t);
                }
            }
        }
        let global = per_w.iter().fold(f64::INFINITY, |a, &(l, _)| a.min(l));
        if !global.is_finite() {
            why(1, "no finite flat candidate");
            return None;
        }
        let wi = per_w
            .iter()
            .position(|&(l, _)| l <= FLAT_NARROW_TIE * global)
            .unwrap();
        let wdt = WIDTHS[wi];
        let step = (deep_lim - shallow_lim) / STEPS as f64;
        let t = golden_min(
            |t| eval0(t, wdt),
            (per_w[wi].1 - step).max(shallow_lim),
            (per_w[wi].1 + step).min(deep_lim),
        );
        let loss_flat = eval0(t, wdt);
        if !loss_flat.is_finite() {
            return None;
        }
        let center = vertex + depth * (t - d0);
        (
            loss_sharp,
            loss_flat,
            center - tang * (wdt * 0.5),
            center + tang * (wdt * 0.5),
        )
    } else {
        // Cluster: candidates cut both walls at depth t and join the
        // crossings with an axis-aligned line; the sharp competitor is
        // the walls extended to their intersection.
        let loss_sharp = match sharp_v {
            Some(v) => {
                let mut poly = win_near.clone();
                poly.push(v);
                poly.extend_from_slice(&wout_near);
                band_loss(&band, &poly, ink_left)
            }
            None => f64::INFINITY,
        };
        let mut wall_in_ext = win_near.clone();
        wall_in_ext.extend_from_slice(&ext_in);
        let mut wall_out_ext = ext_out.clone();
        wall_out_ext.reverse();
        wall_out_ext.extend_from_slice(&wout_near);
        let eval_flat = |t: f64| -> (f64, Option<(Point, Point)>) {
            let fa = match cross_at_depth(&wall_in_ext, depth, t, true) {
                Some(q) => q,
                None => return (f64::INFINITY, None),
            };
            let fb = match cross_at_depth(&wall_out_ext, depth, t, false) {
                Some(q) => q,
                None => return (f64::INFINITY, None),
            };
            let width = (fb - fa).hypot();
            let (min_w, max_w) = (
                FLAT_MIN_WIDTH * rt.px_per_unit,
                FLAT_MAX_WIDTH * rt.px_per_unit,
            );
            if !(min_w..=max_w).contains(&width)
                || (fb - fa).dot(tang) < min_w * 0.5
            {
                return (f64::INFINITY, None);
            }
            let mut poly: Vec<Point> =
                win_near.iter().copied().filter(|&q| d(q) < t).collect();
            poly.push(fa);
            poly.push(fb);
            poly.extend(wout_near.iter().copied().filter(|&q| d(q) < t));
            (band_loss(&band, &poly, ink_left), Some((fa, fb)))
        };
        let mut best_t = f64::NAN;
        let mut best = f64::INFINITY;
        for s in 0..=STEPS {
            let t = shallow_lim
                + (deep_lim - shallow_lim) * s as f64 / STEPS as f64;
            let (loss, _) = eval_flat(t);
            if loss < best {
                best = loss;
                best_t = t;
            }
        }
        if !best.is_finite() {
            why(1, "no finite flat candidate");
            return None;
        }
        let step = (deep_lim - shallow_lim) / STEPS as f64;
        let t = golden_min(
            |t| eval_flat(t).0,
            (best_t - step).max(shallow_lim),
            (best_t + step).min(deep_lim),
        );
        let (loss_flat, endpoints) = eval_flat(t);
        let (fa, fb) = endpoints?;
        (loss_sharp, loss_flat, fa, fb)
    };
    let incumbent = loss_base.min(loss_sharp);
    let accept = loss_flat <= FLAT_ACCEPT_FACTOR * incumbent
        && loss_flat <= FLAT_ABS_MAX;
    if debug {
        eprintln!(
            "    refine: {} flat at ({:.1}, {:.1}) m={} base={:.5} sharp={:.5} flat={:.5} width={:.1}",
            if accept { "ACCEPT" } else { "reject" },
            a_in.x,
            a_in.y,
            m,
            loss_base,
            loss_sharp,
            loss_flat,
            (fb - fa).hypot(),
        );
    }
    if !accept {
        return None;
    }

    // Rebuild: wall_in re-anchored to end at `fa`, the flat, wall_out
    // re-anchored to start at `fb`.
    let win_seg = reanchor(&contour.segs[p], contour.is_line[p], fa, false);
    let wout_seg = reanchor(&contour.segs[k], contour.is_line[k], fb, true);
    let flat_seg = line_seg(fa, fb);

    let mut segs = Vec::with_capacity(n + 1 - m);
    let mut is_line = Vec::with_capacity(n + 1 - m);
    let mut joint_kind = Vec::with_capacity(n + 1 - m);
    for j in 0..n {
        if j == p {
            segs.push(win_seg);
            is_line.push(contour.is_line[p]);
            joint_kind.push(contour.joint_kind[p]);
        } else if m == 0 && j == i {
            segs.push(flat_seg);
            is_line.push(true);
            joint_kind.push(SplitKind::Corner);
            segs.push(wout_seg);
            is_line.push(contour.is_line[k]);
            joint_kind.push(SplitKind::Corner);
        } else if m > 0 && j == i {
            segs.push(flat_seg);
            is_line.push(true);
            joint_kind.push(SplitKind::Corner);
        } else if m > 0 && j == k {
            segs.push(wout_seg);
            is_line.push(contour.is_line[k]);
            joint_kind.push(SplitKind::Corner);
        } else if !in_cluster(j) {
            segs.push(contour.segs[j]);
            is_line.push(contour.is_line[j]);
            joint_kind.push(contour.joint_kind[j]);
        }
    }
    Some((
        FittedContour {
            segs,
            is_line,
            joint_kind,
        },
        loss_flat / incumbent.max(1e-9),
    ))
}

/// Travel direction where a wall meets a valley: the secant over its last
/// FLAT_DIR_SPAN of arc (handle tangents are wrong here — G1 alignment
/// can rotate them parallel to the eventual flat).
fn approach_dir(poly: &[Point], at_start: bool) -> Option<Vec2> {
    const FLAT_DIR_SPAN: f64 = 8.0;
    let mut fwd: Vec<Point> = poly.to_vec();
    if at_start {
        fwd.reverse(); // make the valley end last in both cases
    }
    let near = *fwd.last()?;
    let mut acc = 0.0;
    let mut far = fwd[0];
    for j in (0..fwd.len() - 1).rev() {
        acc += (fwd[j + 1] - fwd[j]).hypot();
        if acc >= FLAT_DIR_SPAN {
            far = fwd[j];
            break;
        }
    }
    let v = if at_start { far - near } else { near - far };
    if v.hypot() < 1e-9 {
        None
    } else {
        Some(v.normalize())
    }
}

/// Re-anchor a wall segment onto a new valley-side endpoint; cubics keep
/// handle directions and scale lengths by the chord ratio (free
/// re-optimization is unsafe: the judged band only covers the junction).
fn reanchor(
    seg: &[Point; 4],
    is_line: bool,
    end: Point,
    at_start: bool,
) -> [Point; 4] {
    if is_line {
        return if at_start {
            line_seg(end, seg[3])
        } else {
            line_seg(seg[0], end)
        };
    }
    let (a, b) = if at_start {
        (end, seg[3])
    } else {
        (seg[0], end)
    };
    let u0 = seg[1] - seg[0];
    let u1 = seg[2] - seg[3];
    if u0.hypot() < 1e-9 || u1.hypot() < 1e-9 {
        return [a, a.lerp(b, 1.0 / 3.0), a.lerp(b, 2.0 / 3.0), b];
    }
    let scale = (b - a).hypot() / (seg[3] - seg[0]).hypot().max(1e-9);
    [a, a + u0 * scale, b + u1 * scale, b]
}

/// Split a wall polyline into its near-the-valley portion (arc length <=
/// FLAT_WALL_NEAR_LEN) and the remainder; both keep travel order and
/// share the interpolated cut point.
fn split_wall_near(
    poly: &[Point],
    near_is_start: bool,
) -> (Vec<Point>, Vec<Point>) {
    let mut fwd: Vec<Point> = poly.to_vec();
    if near_is_start {
        fwd.reverse(); // make the valley end last
    }
    let mut near_rev = vec![fwd[fwd.len() - 1]];
    let mut rest = Vec::new();
    let mut acc = 0.0;
    for j in (0..fwd.len() - 1).rev() {
        let step = (fwd[j + 1] - fwd[j]).hypot();
        if acc + step <= FLAT_WALL_NEAR_LEN {
            near_rev.push(fwd[j]);
            acc += step;
        } else {
            let cut = fwd[j + 1]
                .lerp(fwd[j], (FLAT_WALL_NEAR_LEN - acc) / step.max(1e-9));
            near_rev.push(cut);
            rest = fwd[..=j].to_vec();
            rest.push(cut);
            break;
        }
    }
    near_rev.reverse();
    let mut near = near_rev;
    if near_is_start {
        near.reverse();
        rest.reverse();
    }
    (near, rest)
}

/// Displace a wall polyline's valley end by `delta`, fading to zero (by
/// arc length) at the far end.
fn shear(poly: &[Point], delta: Vec2, near_is_last: bool) -> Vec<Point> {
    let total: f64 = poly.windows(2).map(|w| (w[1] - w[0]).hypot()).sum();
    if total < 1e-9 {
        return poly.iter().map(|&q| q + delta).collect();
    }
    let mut out = Vec::with_capacity(poly.len());
    let mut acc = 0.0;
    for (j, &q) in poly.iter().enumerate() {
        if j > 0 {
            acc += (poly[j] - poly[j - 1]).hypot();
        }
        let f = acc / total;
        let f = if near_is_last { f } else { 1.0 - f };
        out.push(q + delta * f);
    }
    out
}

/// Insert points so consecutive samples are at most ~2 px apart (the
/// band's stamp prefilter needs dense region points to see every pixel).
fn densify(poly: &[Point]) -> Vec<Point> {
    let mut out = Vec::with_capacity(poly.len());
    for w in poly.windows(2) {
        out.push(w[0]);
        let len = (w[1] - w[0]).hypot();
        let extra = (len / 2.0).floor() as usize;
        for e in 1..=extra {
            out.push(w[0].lerp(w[1], e as f64 / (extra + 1) as f64));
        }
    }
    if let Some(&last) = poly.last() {
        out.push(last);
    }
    out
}

/// Points every 2 px along a ray (excluding the origin).
fn ray_points(from: Point, dir: Vec2, len: f64) -> Vec<Point> {
    let steps = (len / 2.0).ceil() as usize;
    (1..=steps.max(1))
        .map(|s| from + dir * (s as f64 * 2.0).min(len))
        .collect()
}

/// Where a polyline crosses depth level `t` (projection on `axis`);
/// `last` picks the crossing nearest the end (valley tip side).
fn cross_at_depth(
    poly: &[Point],
    axis: Vec2,
    t: f64,
    last: bool,
) -> Option<Point> {
    let d = |q: Point| q.to_vec2().dot(axis);
    let mut found = None;
    for j in 0..poly.len().saturating_sub(1) {
        let (d0, d1) = (d(poly[j]) - t, d(poly[j + 1]) - t);
        if d0 * d1 > 0.0 {
            continue;
        }
        let span = d1 - d0;
        if span.abs() < 1e-12 {
            continue;
        }
        let q = poly[j].lerp(poly[j + 1], -d0 / span);
        found = Some(q);
        if !last {
            break;
        }
    }
    // Pin the depth coordinate exactly so the flat is exactly H/V.
    found.map(|q| q + axis * (t - d(q)))
}
