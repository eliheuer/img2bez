// Copyright 2026 the img2bez Authors
// SPDX-License-Identifier: Apache-2.0 OR MIT

//! Multi-master interpolation compatibility.
//!
//! Masters must match in contour count, point count/order, and segment
//! types. [`check`] verifies that; [`make_compatible`] coerces a mismatched
//! set (match contours, normalize winding/start, insert missing points,
//! unify segment types). Guessed correspondences are flagged
//! `low_confidence`; differing contour counts report `compatible: false`.

use kurbo::{CubicBez, ParamCurve, ParamCurveArclen, ParamCurveNearest, Point};
use serde::Serialize;

use crate::model::outline::{Contour, Outline, OutlinePoint, PointKind};

/// Perimeter fraction within which two masters' on-curve points count as the
/// same feature: covers weight drift without merging distinct features.
const CLUSTER_TOL: f64 = 0.04;
/// Arc-length accuracy (font units) for cubic length / inverse-length queries.
const ARCLEN_ACC: f64 = 0.05;
/// Vertex travel above this multiple of the median travel marks a
/// mis-correspondence (a real jump is 5-6x; weight motion is not).
const TRAVEL_OUTLIER_MULT: f64 = 3.5;
/// Bbox-diagonal-relative floor for the outlier test, so a tiny median can't
/// make the threshold meaningless.
const TRAVEL_OUTLIER_FLOOR_FRAC: f64 = 0.06;
/// A one-sided feature is kept (counterpart inserted) only if it shapes the
/// outline by this fraction of the bbox diagonal; shallower ones are dropped.
const MINOR_FEATURE_DEV_FRAC: f64 = 0.03;
/// Matcher cost (fraction of bbox diagonal) for pairing vertices whose
/// outgoing edges run different ways (e.g. vertical vs horizontal).
const MATCH_DIR_PENALTY: f64 = 0.35;
/// Cosine below which corresponding edges count as flipped (~70 degrees).
/// Checked separately because the travel test misses flips whose endpoints
/// barely move.
const DIRECTION_FLIP_COS: f64 = 0.35;

/// What [`make_compatible`] did to one contour across the master set.
#[derive(Debug, Clone, PartialEq, Serialize)]
#[serde(rename_all = "camelCase")]
#[non_exhaustive]
pub struct ContourReport {
    /// On-curve point count every master's copy of this contour ended with.
    pub final_on_curve: usize,
    /// On-curve points inserted into each master to reach that count.
    pub inserted_per_master: Vec<usize>,
    /// True when correspondence was guessed, so the result is worth review.
    pub low_confidence: bool,
}

/// Outcome of [`make_compatible`] over a set of masters.
#[derive(Debug, Clone, PartialEq, Serialize)]
#[serde(rename_all = "camelCase")]
#[non_exhaustive]
pub struct CompatibilityReport {
    /// Number of masters.
    pub master_count: usize,
    /// Contours per master (0 when incompatible).
    pub contour_count: usize,
    /// Whether the returned outlines interpolate.
    pub compatible: bool,
    /// Total on-curve points inserted across all masters and contours.
    pub inserted_points: usize,
    /// True if any contour was reconciled by a guessed correspondence.
    pub low_confidence: bool,
    /// Per-contour detail.
    pub contours: Vec<ContourReport>,
    /// Why the set is incompatible, when it is.
    pub note: Option<String>,
}

/// A `compatible: false` report carrying `note`.
fn incompatible(master_count: usize, note: String) -> CompatibilityReport {
    CompatibilityReport {
        master_count,
        contour_count: 0,
        compatible: false,
        inserted_points: 0,
        low_confidence: false,
        contours: Vec::new(),
        note: Some(note),
    }
}

/// Verify every outline is interpolation-compatible with the first.
///
/// # Errors
///
/// [`TraceError::MastersIncompatible`](crate::TraceError::MastersIncompatible),
/// describing the first mismatch.
pub fn check(outlines: &[Outline]) -> Result<(), crate::TraceError> {
    let Some(first) = outlines.first() else {
        return Ok(());
    };
    for (m, o) in outlines.iter().enumerate().skip(1) {
        if o.contours.len() != first.contours.len() {
            return Err(crate::TraceError::MastersIncompatible(format!(
                "master {m} has {} contours, expected {}",
                o.contours.len(),
                first.contours.len()
            )));
        }
        for (ci, (a, b)) in first.contours.iter().zip(&o.contours).enumerate() {
            if a.points.len() != b.points.len() {
                return Err(crate::TraceError::MastersIncompatible(format!(
                    "master {m} contour {ci} has {} points, expected {}",
                    b.points.len(),
                    a.points.len()
                )));
            }
            for (pi, (pa, pb)) in a.points.iter().zip(&b.points).enumerate() {
                if pa.kind != pb.kind {
                    return Err(crate::TraceError::MastersIncompatible(
                        format!(
                            "master {m} contour {ci} point {pi}: {:?}, expected {:?}",
                            pb.kind, pa.kind
                        ),
                    ));
                }
            }
        }
    }
    Ok(())
}

/// Coerce `masters` into interpolation-compatible outlines plus a
/// [`CompatibilityReport`]. Fewer than two masters is a no-op; a differing
/// contour count reports `compatible: false` with inputs unchanged.
pub fn make_compatible(
    masters: &[Outline],
) -> (Vec<Outline>, CompatibilityReport) {
    let n = masters.len();
    if n < 2 {
        return (
            masters.to_vec(),
            CompatibilityReport {
                master_count: n,
                contour_count: masters.first().map_or(0, |m| m.contours.len()),
                compatible: true,
                inserted_points: 0,
                low_confidence: false,
                contours: Vec::new(),
                note: None,
            },
        );
    }

    let c0 = masters[0].contours.len();
    if masters.iter().any(|m| m.contours.len() != c0) {
        let counts: Vec<usize> =
            masters.iter().map(|m| m.contours.len()).collect();
        return (
            masters.to_vec(),
            incompatible(
                n,
                format!(
                    "contour counts differ across masters: {counts:?}; cannot \
                     reconcile without fabricating a counter — regenerate the \
                     outlier image"
                ),
            ),
        );
    }

    // Canonical order (descending area, then centroid) makes contour `ci`
    // correspond in every master.
    let mut out = masters.to_vec();
    for o in out.iter_mut() {
        o.contours.sort_by(|a, b| {
            canonical_key(b)
                .partial_cmp(&canonical_key(a))
                .unwrap_or(std::cmp::Ordering::Equal)
        });
    }

    let mut contours = Vec::with_capacity(c0);
    for ci in 0..c0 {
        let mut group: Vec<Vec<Seg>> = out
            .iter()
            .map(|o| contour_to_segs(&o.contours[ci]))
            .collect();
        match reconcile_group(&mut group) {
            Ok(rep) => {
                for (o, segs) in out.iter_mut().zip(&group) {
                    o.contours[ci] = segs_to_contour(segs);
                }
                contours.push(rep);
            }
            Err(e) => {
                return (
                    masters.to_vec(),
                    incompatible(n, format!("contour {ci}: {e}")),
                );
            }
        }
    }

    let inserted_points = contours
        .iter()
        .map(|r| r.inserted_per_master.iter().sum::<usize>())
        .sum();
    let low_confidence = contours.iter().any(|r| r.low_confidence);
    (
        out,
        CompatibilityReport {
            master_count: n,
            contour_count: c0,
            compatible: true,
            inserted_points,
            low_confidence,
            contours,
            note: None,
        },
    )
}

/// Sort key for contour matching: (area, centroid.x, centroid.y).
fn canonical_key(c: &Contour) -> (f64, f64, f64) {
    let segs = contour_to_segs(c);
    let area = signed_area(&segs).abs();
    let (mut cx, mut cy) = (0.0, 0.0);
    let n = segs.len().max(1) as f64;
    for s in &segs {
        cx += s.start().x;
        cy += s.start().y;
    }
    (area, cx / n, cy / n)
}

/// Reconcile a group of corresponding contours (one per master) so they share
/// one vertex count and segment-type sequence, returning what it did.
fn reconcile_group(masters: &mut [Vec<Seg>]) -> Result<ContourReport, String> {
    // Winding: match the first master.
    let target = signed_area(&masters[0]).signum();
    for m in masters.iter_mut() {
        if signed_area(m).signum() != target {
            reverse_loop(m);
        }
    }
    // Start point: rotate each ring to its bottom-left vertex.
    for m in masters.iter_mut() {
        let start = canonical_start(m);
        m.rotate_left(start);
    }
    let n = masters.len();
    let mut inserted_per_master = vec![0usize; n];
    let mut low_confidence = false;
    let before: Vec<usize> = masters.iter().map(|m| m.len()).collect();
    if !masters.iter().all(|m| m.len() == masters[0].len()) {
        // Counts differ: insert each master's missing vertices by arc-length.
        align(masters);
        low_confidence = true;
    } else if travel_has_outlier(masters) || direction_has_flip(masters) {
        // Counts match but some vertex is paired with the wrong feature;
        // re-derive the correspondence and insert/drop as needed.
        repair_by_travel(masters);
        low_confidence = true;
    }
    for (i, &b) in before.iter().enumerate() {
        inserted_per_master[i] = masters[i].len().saturating_sub(b);
    }
    let k = masters[0].len();
    if !masters.iter().all(|m| m.len() == k) {
        return Err(format!(
            "could not align vertex counts ({:?}); the masters differ too \
             much in structure to reconcile — regenerate the outlier image",
            masters.iter().map(|m| m.len()).collect::<Vec<_>>()
        ));
    }
    // Segment types: if any master draws a segment as a curve, all do.
    for si in 0..k {
        if masters.iter().any(|m| matches!(m[si], Seg::Cubic(..))) {
            for m in masters.iter_mut() {
                m[si] = m[si].as_cubic();
            }
        }
    }
    Ok(ContourReport {
        final_on_curve: k,
        inserted_per_master,
        low_confidence,
    })
}

/// True if some vertex's travel between masters is a gross outlier versus the
/// median — the signature of a vertex matched to the wrong feature.
fn travel_has_outlier(masters: &[Vec<Seg>]) -> bool {
    let k = masters[0].len();
    if k < 6 {
        return false; // too few vertices to judge robustly
    }
    let mut travel = vec![0.0f64; k];
    let (mut minx, mut miny, mut maxx, mut maxy) = (
        f64::INFINITY,
        f64::INFINITY,
        f64::NEG_INFINITY,
        f64::NEG_INFINITY,
    );
    for i in 0..k {
        let p0 = masters[0][i].start();
        minx = minx.min(p0.x);
        miny = miny.min(p0.y);
        maxx = maxx.max(p0.x);
        maxy = maxy.max(p0.y);
        for m in masters.iter().skip(1) {
            let p = m[i].start();
            travel[i] = travel[i]
                .max(((p.x - p0.x).powi(2) + (p.y - p0.y).powi(2)).sqrt());
        }
    }
    let mut sorted = travel.clone();
    sorted
        .sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    let median = sorted[sorted.len() / 2];
    let diag = ((maxx - minx).powi(2) + (maxy - miny).powi(2)).sqrt();
    let threshold =
        (median * TRAVEL_OUTLIER_MULT).max(TRAVEL_OUTLIER_FLOOR_FRAC * diag);
    travel.iter().any(|&t| t > threshold)
}

/// True if a corresponding segment flips direction across masters (e.g.
/// vertical to horizontal) — it would swing under interpolation.
fn direction_has_flip(masters: &[Vec<Seg>]) -> bool {
    let k = masters[0].len();
    if k < 3 {
        return false;
    }
    for i in 0..k {
        let d0 = masters[0][i].end() - masters[0][i].start();
        let l0 = d0.hypot();
        if l0 < 1e-6 {
            continue;
        }
        for m in masters.iter().skip(1) {
            let dm = m[i].end() - m[i].start();
            let lm = dm.hypot();
            if lm > 1e-6
                && (d0.x * dm.x + d0.y * dm.y) / (l0 * lm) < DIRECTION_FLIP_COS
            {
                return true;
            }
        }
    }
    false
}

/// Repair a mis-corresponded equal-count pair: re-derive the correspondence
/// by minimal-travel DTW, then insert/drop vertices to realize it.
/// Two-master case only; larger sets keep the arc-length `align`.
fn repair_by_travel(masters: &mut [Vec<Seg>]) {
    if masters.len() != 2 {
        return;
    }
    let va: Vec<Point> = masters[0].iter().map(|s| s.start()).collect();
    let vb: Vec<Point> = masters[1].iter().map(|s| s.start()).collect();
    let fa = vertex_fractions(&masters[0]);
    let fb = vertex_fractions(&masters[1]);
    let path = dtw_path(&va, &vb);
    let minor = MINOR_FEATURE_DEV_FRAC * bbox_diag(&va).max(bbox_diag(&vb));
    // Where the path advances one master alone, that master has a one-sided
    // feature: insert its counterpart if it shapes the outline, else drop it.
    let mut drop_a: Vec<usize> = Vec::new();
    let mut drop_b: Vec<usize> = Vec::new();
    let mut ins_a: Vec<f64> = Vec::new();
    let mut ins_b: Vec<f64> = Vec::new();
    for w in 1..path.len() {
        let (pi, pj) = path[w - 1];
        let (i, j) = path[w];
        match (i > pi, j > pj) {
            (false, true) => {
                let jv = j % vb.len();
                if merge_error(&masters[1], jv) < minor {
                    drop_b.push(jv);
                } else {
                    ins_a.push(fb[jv]);
                }
            }
            (true, false) => {
                let iv = i % va.len();
                if merge_error(&masters[0], iv) < minor {
                    drop_a.push(iv);
                } else {
                    ins_b.push(fa[iv]);
                }
            }
            _ => {}
        }
    }
    let a2 = remove_vertices(&masters[0], &drop_a);
    masters[0] = if ins_a.is_empty() {
        a2
    } else {
        insert_at(&a2, &ins_a)
    };
    let b2 = remove_vertices(&masters[1], &drop_b);
    masters[1] = if ins_b.is_empty() {
        b2
    } else {
        insert_at(&b2, &ins_b)
    };
}

/// One cubic replacing the two segments around a vertex, keeping the outer
/// tangents (mirrors what `remove_vertices` builds).
fn merged_cubic(seg_in: Seg, seg_out: Seg) -> CubicBez {
    let p0 = seg_in.start();
    let p3 = seg_out.end();
    let p1 = match seg_in {
        Seg::Cubic(_, b, _, _) => b,
        Seg::Line(a, _) => a + (p3 - a) / 3.0,
    };
    let p2 = match seg_out {
        Seg::Cubic(_, _, c, _) => c,
        Seg::Line(_, b) => b + (p0 - b) / 3.0,
    };
    CubicBez::new(p0, p1, p2, p3)
}

/// How far dropping vertex `v` would pull the outline off its original path
/// (max sample distance to the replacement cubic). Small = redundant vertex.
fn merge_error(segs: &[Seg], v: usize) -> f64 {
    let n = segs.len();
    if n < 3 {
        return f64::INFINITY;
    }
    let seg_in = segs[(v + n - 1) % n];
    let seg_out = segs[v];
    let merged = merged_cubic(seg_in, seg_out);
    let mut maxd = 0.0f64;
    for k in 1..8 {
        let t = k as f64 / 8.0;
        let p = if t < 0.5 {
            seg_in.eval(t * 2.0)
        } else {
            seg_out.eval((t - 0.5) * 2.0)
        };
        maxd = maxd.max(merged.nearest(p, 1e-3).distance_sq.sqrt());
    }
    maxd
}

/// Diagonal of the bounding box of a vertex ring.
fn bbox_diag(pts: &[Point]) -> f64 {
    let (mut minx, mut miny, mut maxx, mut maxy) = (
        f64::INFINITY,
        f64::INFINITY,
        f64::NEG_INFINITY,
        f64::NEG_INFINITY,
    );
    for p in pts {
        minx = minx.min(p.x);
        miny = miny.min(p.y);
        maxx = maxx.max(p.x);
        maxy = maxy.max(p.y);
    }
    ((maxx - minx).powi(2) + (maxy - miny).powi(2)).sqrt()
}

/// Remove the given on-curve vertices from a closed contour, merging each gap
/// into one cubic that keeps the surrounding tangents. No-op below a triangle.
fn remove_vertices(segs: &[Seg], remove: &[usize]) -> Vec<Seg> {
    if remove.is_empty() {
        return segs.to_vec();
    }
    let rm: std::collections::HashSet<usize> = remove.iter().copied().collect();
    let n = segs.len();
    let kept: Vec<usize> = (0..n).filter(|i| !rm.contains(i)).collect();
    if kept.len() < 3 {
        return segs.to_vec();
    }
    let mut out = Vec::with_capacity(kept.len());
    for k in 0..kept.len() {
        let start_v = kept[k];
        let end_v = kept[(k + 1) % kept.len()];
        let last_seg = (end_v + n - 1) % n;
        if start_v == last_seg {
            out.push(segs[start_v]);
            continue;
        }
        let p0 = segs[start_v].start();
        let p3 = segs[last_seg].end();
        // A straight run stays a line; otherwise merge into one cubic
        // keeping the surrounding tangents.
        if matches!(segs[start_v], Seg::Line(..))
            && matches!(segs[last_seg], Seg::Line(..))
        {
            out.push(Seg::Line(p0, p3));
            continue;
        }
        let out_handle = match segs[start_v] {
            Seg::Cubic(_, b, _, _) => b,
            Seg::Line(a, _) => a + (p3 - a) / 3.0,
        };
        let in_handle = match segs[last_seg] {
            Seg::Cubic(_, _, c, _) => c,
            Seg::Line(_, b) => b + (p0 - b) / 3.0,
        };
        out.push(Seg::Cubic(p0, out_handle, in_handle, p3));
    }
    out
}

/// Minimal-travel, order-preserving (DTW) alignment of two vertex rings:
/// the warping path as (i, j) pairs from (0, 0) to (a.len(), b.len()).
fn dtw_path(a: &[Point], b: &[Point]) -> Vec<(usize, usize)> {
    let av: Vec<Point> =
        a.iter().copied().chain(std::iter::once(a[0])).collect();
    let bv: Vec<Point> =
        b.iter().copied().chain(std::iter::once(b[0])).collect();
    let (ra, rb) = (av.len(), bv.len());
    let diag = bbox_diag(a);
    let dir = |v: &[Point], i: usize| -> (f64, f64) {
        let n = v.len();
        let d = v[(i + 1) % n] - v[i];
        let l = d.hypot();
        if l > 1e-6 {
            (d.x / l, d.y / l)
        } else {
            (0.0, 0.0)
        }
    };
    // Cost = distance plus a penalty for mismatched outgoing-edge directions.
    let cost = |i: usize, j: usize| -> f64 {
        let d = (av[i] - bv[j]).hypot();
        let (ax, ay) = dir(&av, i);
        let (bx, by) = dir(&bv, j);
        if ax * ax + ay * ay > 0.5 && bx * bx + by * by > 0.5 {
            d + (1.0 - (ax * bx + ay * by)) * MATCH_DIR_PENALTY * diag
        } else {
            d
        }
    };
    let inf = f64::INFINITY;
    let mut dp = vec![vec![inf; rb]; ra];
    let mut bk = vec![vec![(0usize, 0usize); rb]; ra];
    dp[0][0] = cost(0, 0);
    for i in 0..ra {
        for j in 0..rb {
            if i == 0 && j == 0 {
                continue;
            }
            let mut best = inf;
            let mut from = (0, 0);
            for (oi, oj) in [
                (i.checked_sub(1), j.checked_sub(1)),
                (i.checked_sub(1), Some(j)),
                (Some(i), j.checked_sub(1)),
            ] {
                if let (Some(pi), Some(pj)) = (oi, oj)
                    && dp[pi][pj] < best
                {
                    best = dp[pi][pj];
                    from = (pi, pj);
                }
            }
            dp[i][j] = best + cost(i, j);
            bk[i][j] = from;
        }
    }
    let mut path = Vec::new();
    let (mut i, mut j) = (ra - 1, rb - 1);
    while (i, j) != (0, 0) {
        path.push((i, j));
        let (pi, pj) = bk[i][j];
        i = pi;
        j = pj;
    }
    path.push((0, 0));
    path.reverse();
    path
}

/// Align differing-length masters by an ordered merge of vertices by
/// perimeter fraction, inserting each master's missing vertices.
fn align(masters: &mut [Vec<Seg>]) {
    let n = masters.len();
    let fracs: Vec<Vec<f64>> =
        masters.iter().map(|m| vertex_fractions(m)).collect();
    let counts: Vec<usize> = masters.iter().map(|m| m.len()).collect();
    let mut idx = vec![0usize; n];
    // Fractions each master must have a vertex inserted at.
    let mut inserts: Vec<Vec<f64>> = vec![Vec::new(); n];
    loop {
        let next: Vec<Option<f64>> = (0..n)
            .map(|m| (idx[m] < counts[m]).then(|| fracs[m][idx[m]]))
            .collect();
        if next.iter().all(Option::is_none) {
            break;
        }
        let min_f =
            next.iter().filter_map(|x| *x).fold(f64::INFINITY, f64::min);
        for m in 0..n {
            match next[m] {
                Some(f) if (f - min_f).abs() <= CLUSTER_TOL => idx[m] += 1,
                _ => inserts[m].push(min_f),
            }
        }
    }
    for m in 0..n {
        if !inserts[m].is_empty() {
            masters[m] = insert_at(&masters[m], &inserts[m]);
        }
    }
}

/// Split `segs` so a vertex lands at each global perimeter fraction in `cuts`
/// (each assumed to fall strictly inside one segment).
fn insert_at(segs: &[Seg], cuts: &[f64]) -> Vec<Seg> {
    let fracs = vertex_fractions(segs);
    let n = segs.len();
    let mut out = Vec::new();
    for i in 0..n {
        let a = fracs[i];
        let b = if i + 1 < n { fracs[i + 1] } else { 1.0 };
        let span = (b - a).max(1e-12);
        let mut local: Vec<f64> = cuts
            .iter()
            .filter(|&&c| c > a + 1e-9 && c < b - 1e-9)
            .map(|&c| ((c - a) / span).clamp(0.0, 1.0))
            .collect();
        local.sort_by(|x, y| x.partial_cmp(y).unwrap());
        if local.is_empty() {
            out.push(segs[i]);
            continue;
        }
        let mut prev = 0.0;
        for t in local {
            out.push(segs[i].subsegment(prev, t));
            prev = t;
        }
        out.push(segs[i].subsegment(prev, 1.0));
    }
    out
}

// ── Segment representation ──────────────────────────────────────────────────

/// One segment of a contour between two on-curve points.
#[derive(Debug, Clone, Copy)]
enum Seg {
    Line(Point, Point),
    Cubic(Point, Point, Point, Point),
}

impl Seg {
    fn start(&self) -> Point {
        match *self {
            Seg::Line(a, _) => a,
            Seg::Cubic(a, _, _, _) => a,
        }
    }
    fn end(&self) -> Point {
        match *self {
            Seg::Line(_, b) => b,
            Seg::Cubic(_, _, _, d) => d,
        }
    }
    fn arclen(&self) -> f64 {
        match *self {
            Seg::Line(a, b) => (b - a).hypot(),
            Seg::Cubic(a, b, c, d) => {
                CubicBez::new(a, b, c, d).arclen(ARCLEN_ACC)
            }
        }
    }
    fn eval(&self, t: f64) -> Point {
        match *self {
            Seg::Line(a, b) => a.lerp(b, t),
            Seg::Cubic(a, b, c, d) => CubicBez::new(a, b, c, d).eval(t),
        }
    }
    fn reversed(&self) -> Seg {
        match *self {
            Seg::Line(a, b) => Seg::Line(b, a),
            Seg::Cubic(a, b, c, d) => Seg::Cubic(d, c, b, a),
        }
    }
    /// As a cubic (a line gets handles at 1/3 and 2/3, staying straight).
    fn as_cubic(&self) -> Seg {
        match *self {
            Seg::Cubic(..) => *self,
            Seg::Line(a, b) => {
                Seg::Cubic(a, a.lerp(b, 1.0 / 3.0), a.lerp(b, 2.0 / 3.0), b)
            }
        }
    }
    /// Sub-segment between arc-length fractions `f0` and `f1` (`0..=1`).
    fn subsegment(&self, f0: f64, f1: f64) -> Seg {
        match *self {
            Seg::Line(a, b) => Seg::Line(a.lerp(b, f0), a.lerp(b, f1)),
            Seg::Cubic(a, b, c, d) => {
                let cb = CubicBez::new(a, b, c, d);
                let t0 = inv_arclen_frac(&cb, f0);
                let t1 = inv_arclen_frac(&cb, f1);
                let s = cb.subsegment(t0..t1);
                Seg::Cubic(s.p0, s.p1, s.p2, s.p3)
            }
        }
    }
}

/// Parameter `t` at arc-length fraction `f` of a cubic.
fn inv_arclen_frac(cb: &CubicBez, f: f64) -> f64 {
    if f <= 0.0 {
        return 0.0;
    }
    if f >= 1.0 {
        return 1.0;
    }
    let target = f * cb.arclen(ARCLEN_ACC);
    cb.inv_arclen(target, ARCLEN_ACC)
}

// ── Contour ↔ segments ──────────────────────────────────────────────────────

/// Convert a UFO-ordered contour to its closed loop of segments.
fn contour_to_segs(c: &Contour) -> Vec<Seg> {
    let pts = &c.points;
    if pts.is_empty() {
        return Vec::new();
    }
    let start = Point::new(pts[0].x, pts[0].y);
    let mut segs = Vec::new();
    let mut cur = start;
    let mut pending: Vec<Point> = Vec::new();
    for p in &pts[1..] {
        if p.kind == PointKind::OffCurve {
            pending.push(Point::new(p.x, p.y));
        } else {
            let end = Point::new(p.x, p.y);
            segs.push(make_seg(cur, &pending, end));
            pending.clear();
            cur = end;
        }
    }
    // Closing segment back to the start, using any trailing off-curves.
    segs.push(make_seg(cur, &pending, start));
    segs
}

/// One segment from a start point, its pending off-curve handles, and its end.
fn make_seg(p0: Point, pending: &[Point], p3: Point) -> Seg {
    match pending {
        [] => Seg::Line(p0, p3),
        [a] => Seg::Cubic(p0, *a, *a, p3), // quad → cubic (coincident handles)
        [a, b, ..] => Seg::Cubic(p0, *a, *b, p3),
    }
}

/// Rebuild a UFO-ordered contour from a closed loop of segments (first point
/// carries the closing segment's type; off-curves precede their on-curve).
fn segs_to_contour(segs: &[Seg]) -> Contour {
    let mut points: Vec<OutlinePoint> = Vec::new();
    let n = segs.len();
    if n == 0 {
        return Contour { points };
    }
    let closing_kind = match segs[n - 1] {
        Seg::Line(..) => PointKind::Line,
        Seg::Cubic(..) => PointKind::Curve,
    };
    let v0 = segs[0].start();
    points.push(on(v0, closing_kind));
    for s in &segs[..n - 1] {
        if let Seg::Cubic(_, b, c, _) = *s {
            points.push(off(b));
            points.push(off(c));
        }
        let kind = match s {
            Seg::Line(..) => PointKind::Line,
            Seg::Cubic(..) => PointKind::Curve,
        };
        points.push(on(s.end(), kind));
    }
    // The closing segment's handles precede point 0 (the wrap-around).
    if let Seg::Cubic(_, b, c, _) = segs[n - 1] {
        points.push(off(b));
        points.push(off(c));
    }
    compute_smooth(&mut points);
    Contour { points }
}

// ── Geometry helpers ────────────────────────────────────────────────────────

/// Twice the loop's signed area (shoelace); the sign gives the winding.
fn signed_area(segs: &[Seg]) -> f64 {
    let mut sum = 0.0;
    for s in segs {
        let a = s.start();
        let b = s.end();
        sum += a.x * b.y - b.x * a.y;
    }
    sum
}

/// Reverse a closed loop's direction in place.
fn reverse_loop(segs: &mut [Seg]) {
    segs.reverse();
    for s in segs.iter_mut() {
        *s = s.reversed();
    }
}

/// Index of the bottom-most, then left-most vertex — a corner that
/// corresponds across masters. Returns 0 for an empty list (must not panic).
fn canonical_start(segs: &[Seg]) -> usize {
    if segs.is_empty() {
        return 0;
    }
    let mut best = 0;
    let mut best_pt = segs[0].start();
    for (i, s) in segs.iter().enumerate() {
        let p = s.start();
        if p.y < best_pt.y - 1e-9
            || (p.y < best_pt.y + 1e-9 && p.x < best_pt.x - 1e-9)
        {
            best = i;
            best_pt = p;
        }
    }
    best
}

/// Each vertex's perimeter fraction along the loop (vertex 0 at 0).
fn vertex_fractions(segs: &[Seg]) -> Vec<f64> {
    let lens: Vec<f64> = segs.iter().map(|s| s.arclen()).collect();
    let total: f64 = lens.iter().sum();
    let mut fracs = Vec::with_capacity(segs.len());
    let mut acc = 0.0;
    for &l in &lens {
        fracs.push(if total > 0.0 { acc / total } else { 0.0 });
        acc += l;
    }
    fracs
}

/// Mark on-curve points between near-collinear neighbours as smooth (~10°),
/// mirroring `outline::compute_smooth` so reconciled output reads the same.
fn compute_smooth(points: &mut [OutlinePoint]) {
    let n = points.len();
    if n < 3 {
        return;
    }
    for i in 0..n {
        if points[i].kind == PointKind::OffCurve {
            continue;
        }
        let prev = if i == 0 { n - 1 } else { i - 1 };
        let next = (i + 1) % n;
        let (idx, idy) =
            (points[i].x - points[prev].x, points[i].y - points[prev].y);
        let (odx, ody) =
            (points[next].x - points[i].x, points[next].y - points[i].y);
        let il = (idx * idx + idy * idy).sqrt();
        let ol = (odx * odx + ody * ody).sqrt();
        if il < 0.01 || ol < 0.01 {
            continue;
        }
        let cross = (idx / il) * (ody / ol) - (idy / il) * (odx / ol);
        let dot = (idx / il) * (odx / ol) + (idy / il) * (ody / ol);
        if cross.abs() < 0.174 && dot > 0.0 {
            points[i].smooth = true;
        }
    }
}

fn on(p: Point, kind: PointKind) -> OutlinePoint {
    OutlinePoint {
        x: p.x,
        y: p.y,
        kind,
        smooth: false,
    }
}
fn off(p: Point) -> OutlinePoint {
    OutlinePoint {
        x: p.x,
        y: p.y,
        kind: PointKind::OffCurve,
        smooth: false,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn square() -> Outline {
        let mut p = kurbo::BezPath::new();
        p.move_to((0.0, 0.0));
        p.line_to((100.0, 0.0));
        p.line_to((100.0, 100.0));
        p.line_to((0.0, 100.0));
        p.close_path();
        Outline::from_bezpaths(&[p])
    }

    /// A square with an extra on-curve point midway up the right edge.
    fn square_extra_point() -> Outline {
        let mut p = kurbo::BezPath::new();
        p.move_to((0.0, 0.0));
        p.line_to((100.0, 0.0));
        p.line_to((100.0, 50.0)); // extra
        p.line_to((100.0, 100.0));
        p.line_to((0.0, 100.0));
        p.close_path();
        Outline::from_bezpaths(&[p])
    }

    fn shift_a() -> Outline {
        // Tall shape, long right edge, extra detail along the top.
        let mut p = kurbo::BezPath::new();
        p.move_to((0.0, 0.0));
        p.line_to((100.0, 0.0));
        p.line_to((100.0, 300.0));
        p.line_to((60.0, 300.0));
        p.line_to((20.0, 300.0));
        p.line_to((0.0, 150.0));
        p.close_path();
        Outline::from_bezpaths(&[p])
    }

    fn shift_b() -> Outline {
        // Same vertex count, extra detail near the bottom, so naive
        // vertex-for-vertex alignment mis-pairs across the long right edge.
        let mut p = kurbo::BezPath::new();
        p.move_to((0.0, 0.0));
        p.line_to((80.0, 0.0));
        p.line_to((100.0, 0.0));
        p.line_to((100.0, 300.0));
        p.line_to((60.0, 300.0));
        p.line_to((0.0, 150.0));
        p.close_path();
        Outline::from_bezpaths(&[p])
    }

    fn max_vertex_travel(a: &Outline, b: &Outline) -> f64 {
        let (pa, pb) = (&a.contours[0].points, &b.contours[0].points);
        assert_eq!(pa.len(), pb.len());
        pa.iter()
            .zip(pb)
            .map(|(p, q)| ((p.x - q.x).powi(2) + (p.y - q.y).powi(2)).sqrt())
            .fold(0.0_f64, f64::max)
    }

    #[test]
    fn make_compatible_repairs_miscorresponded_equal_counts() {
        let before = max_vertex_travel(&shift_a(), &shift_b());
        let (set, report) = make_compatible(&[shift_a(), shift_b()]);
        assert!(report.compatible);
        check(&set).expect("compatible after repair");
        assert_eq!(
            set[0].contours[0].points.len(),
            set[1].contours[0].points.len()
        );
        assert!(report.low_confidence, "repair should flag low confidence");
        let after = max_vertex_travel(&set[0], &set[1]);
        assert!(
            after < before,
            "repair should reduce travel: {after} !< {before}"
        );
    }

    #[test]
    fn check_passes_for_identical() {
        assert!(check(&[square(), square()]).is_ok());
    }

    #[test]
    fn check_catches_point_count_mismatch() {
        let err = check(&[square(), square_extra_point()]).unwrap_err();
        assert!(err.to_string().contains("points"), "{err}");
    }

    #[test]
    fn make_compatible_fixes_mismatched_squares() {
        let (set, report) = make_compatible(&[square(), square_extra_point()]);
        assert!(report.compatible);
        check(&set).expect("compatible after make_compatible");
        assert_eq!(
            set[0].contours[0].points.len(),
            set[1].contours[0].points.len()
        );
        assert!(report.inserted_points > 0);
        assert!(report.low_confidence);
    }

    #[test]
    fn make_compatible_is_confident_when_already_matched() {
        let (set, report) = make_compatible(&[square(), square()]);
        assert!(report.compatible);
        assert_eq!(report.inserted_points, 0);
        assert!(!report.low_confidence);
        assert_eq!(set, vec![square(), square()]);
    }

    #[test]
    fn make_compatible_rejects_topology_mismatch() {
        let one = square();
        let mut two = square();
        two.contours.push(square().contours[0].clone()); // 2 contours
        let (set, report) = make_compatible(&[one.clone(), two.clone()]);
        assert!(!report.compatible);
        assert!(report.note.unwrap().contains("contour counts"));
        // Inputs are returned unchanged.
        assert_eq!(set, vec![one, two]);
    }
}
