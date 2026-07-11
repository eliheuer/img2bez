// Copyright 2026 the img2bez Authors
// SPDX-License-Identifier: Apache-2.0 OR MIT

//! Joint master tracing: decide outline structure once, across the master
//! set, on the dense sub-pixel boundaries — then fit each master to the
//! shared plan (independently traced masters diverge structurally).
//! A successful [`trace_masters_joint`] guarantees identical contour
//! order and per-contour segment count/kinds in every master, so the
//! outlines interpolate by construction; on any [`JointError`] the caller
//! falls back to independent tracing plus reconciliation.

use image::GrayImage;
use kurbo::{Affine, BezPath, Point, Vec2};

use super::fit::{
    self, ContourPlan, FittedContour, PlanOutcome, Split, SplitKind,
};
use super::subpixel::SubpixelContour;
use crate::model::config::TraceOptions;
use crate::model::geom::line_seg;

/// Per-master realized split lists (same column order in every master),
/// inserted-feature counts per master, and whether resolution was lossy.
type AlignedSplits = (Vec<Vec<Split>>, Vec<usize>, bool);
/// One master's inflection-pair fit: the two cubics and their line-ness.
type InflectionPair = ([[Point; 4]; 2], [bool; 2]);

/// An unmatched corner turning at least this much (degrees, windowed) is
/// real and gets a counterpart realized in every other master; a
/// shallower one is a threshold artifact and is dropped everywhere.
const MINOR_CORNER_DEG: f64 = 50.0;
/// Samples of slack when testing whether a section lies inside one of a
/// master's planned straight runs.
const LINE_CONTAIN_SLACK: usize = 8;
/// Window (samples each side) for accumulating a corner's turn.
const CORNER_TURN_WINDOW: usize = 5;

/// One master's input to the joint trace (per-master `em_height` is
/// respected).
pub(crate) struct JointMaster<'a> {
    pub gray: &'a GrayImage,
    pub threshold: u8,
    pub config: &'a TraceOptions,
}

/// Per-master paths in em space (same canonical contour order in every
/// master), plus what the alignment had to do.
pub(crate) struct JointOutcome {
    pub paths: Vec<Vec<BezPath>>,
    pub low_confidence: bool,
    pub inserted_per_master: Vec<usize>,
}

/// Why the joint path could not run; the caller falls back to independent
/// tracing plus `compat::make_compatible`.
#[derive(Debug, Clone)]
pub(crate) enum JointError {
    NoContours(usize),
    ContourCount(Vec<usize>),
    Degenerate,
    Alignment(String),
    /// A fitted contour drifted off its own source boundary (master,
    /// contour, deviation px, threshold px) — the mangled-fit safety net.
    SourceDeviation(usize, usize, f64, f64),
}

impl std::fmt::Display for JointError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            JointError::NoContours(m) => {
                write!(f, "master {m} produced no contours")
            }
            JointError::ContourCount(c) => {
                write!(f, "contour counts differ across masters: {c:?}")
            }
            JointError::Degenerate => {
                write!(f, "a contour was too small or featureless to plan")
            }
            JointError::Alignment(s) => write!(f, "feature alignment: {s}"),
            JointError::SourceDeviation(m, c, dev, thr) => write!(
                f,
                "master {m} contour {c} drifted {dev:.0}px off its source \
                 boundary (threshold {thr:.0}px)"
            ),
        }
    }
}

/// Trace a master set jointly. See the module docs.
pub(crate) fn trace_masters_joint(
    masters: &[JointMaster],
) -> Result<JointOutcome, JointError> {
    let nm = masters.len();
    debug_assert!(nm >= 2);

    // 1. Extract each master's glyph contours.
    let mut sets: Vec<Vec<SubpixelContour>> = Vec::with_capacity(nm);
    let mut scales: Vec<f64> = Vec::with_capacity(nm);
    for (mi, m) in masters.iter().enumerate() {
        let h = m.gray.height() as f64;
        let scale = m.config.em_height / h;
        let min_area = (m.config.min_contour_area / (scale * scale)).max(2.0);
        let cs = super::glyph_contours(m.gray, m.threshold, m.config, min_area);
        if cs.is_empty() {
            return Err(JointError::NoContours(mi));
        }
        sets.push(cs);
        scales.push(scale);
    }
    let c0 = sets[0].len();
    if sets.iter().any(|s| s.len() != c0) {
        return Err(JointError::ContourCount(
            sets.iter().map(|s| s.len()).collect(),
        ));
    }

    // 2. Canonical contour order (em units), winding aligned to master 0.
    for (set, &s) in sets.iter_mut().zip(&scales) {
        set.sort_by(|a, b| {
            canon_key(b, s)
                .partial_cmp(&canon_key(a, s))
                .unwrap_or(std::cmp::Ordering::Equal)
        });
    }
    for ci in 0..c0 {
        let want = sets[0][ci].signed_area2() >= 0.0;
        for set in sets.iter_mut().skip(1) {
            if (set[ci].signed_area2() >= 0.0) != want {
                set[ci].points.reverse();
            }
        }
    }

    // 3. Per contour: plan each master, align the plans, fit jointly.
    let mut paths: Vec<Vec<BezPath>> = vec![Vec::with_capacity(c0); nm];
    let mut low_confidence = false;
    let mut inserted_per_master = vec![0usize; nm];
    #[allow(clippy::needless_range_loop)] // ci indexes parallel per-master sets
    for ci in 0..c0 {
        let mut plans: Vec<ContourPlan> = Vec::with_capacity(nm);
        for (mi, m) in masters.iter().enumerate() {
            match fit::plan_contour(
                &sets[mi][ci],
                m.config.smoothing,
                m.config.corner_threshold_deg,
                m.config.corner_smear,
                m.config.soft_source,
                None,
            ) {
                PlanOutcome::Plan(p) => plans.push(p),
                _ => return Err(JointError::Degenerate),
            }
        }
        if std::env::var("IMG2BEZ_DEBUG_JOINT").is_ok() {
            for (mi, plan) in plans.iter().enumerate() {
                let fs = feats_of(plan);
                eprintln!(
                    "  joint: contour {ci} master {mi}: {} features",
                    fs.len()
                );
                for f in &fs {
                    eprintln!(
                        "    {:?} sig{:+} frac {:.3} idx {} turn {:.0}",
                        f.kind, f.sig, f.frac, f.idx, f.turn_deg
                    );
                }
            }
        }
        let (aligned, ins, lc) = align_plans(&plans)?;
        for (a, b) in inserted_per_master.iter_mut().zip(&ins) {
            *a += b;
        }
        low_confidence |= lc;

        let fitted = fit_joint(&plans, &aligned, masters)?;
        for (mi, fc) in fitted.into_iter().enumerate() {
            let fc = fit::tame_short_cubic_handles(fc);
            let fc = fit::smooth_joins(fc);
            let fc = fit::harmonize(fc);
            // Safety net: a mangled fit (folded handle, wrong
            // correspondence) drifts an order of magnitude further off the
            // source boundary than a deliberately loose single cubic.
            let ring = &plans[mi].smoothed;
            let dev = max_source_deviation(&fc, ring);
            let threshold =
                (ring_diag(ring) * SOURCE_DEV_FRAC).max(SOURCE_DEV_MIN_PX);
            if dev > threshold {
                return Err(JointError::SourceDeviation(
                    mi, ci, dev, threshold,
                ));
            }
            let mut p = fit::contour_to_bezpath(&fc);
            p.apply_affine(Affine::scale(scales[mi]));
            paths[mi].push(p);
        }
    }

    Ok(JointOutcome {
        paths,
        low_confidence,
        inserted_per_master,
    })
}

/// Maximum distance from the fitted contour to the dense source ring
/// (~1px-spaced, so nearest-vertex distance is good to ~half a pixel).
fn max_source_deviation(fc: &FittedContour, ring: &[(f64, f64)]) -> f64 {
    let mut max_dev = 0.0f64;
    for seg in &fc.segs {
        let cubic = kurbo::CubicBez::new(seg[0], seg[1], seg[2], seg[3]);
        for k in 1..8 {
            let p = kurbo::ParamCurve::eval(&cubic, k as f64 / 8.0);
            let mut best = f64::INFINITY;
            for &(x, y) in ring {
                let d = (p.x - x).powi(2) + (p.y - y).powi(2);
                if d < best {
                    best = d;
                }
            }
            max_dev = max_dev.max(best.sqrt());
        }
    }
    max_dev
}

/// Diagonal of a ring's bounding box.
fn ring_diag(ring: &[(f64, f64)]) -> f64 {
    let (mut minx, mut miny, mut maxx, mut maxy) = (
        f64::INFINITY,
        f64::INFINITY,
        f64::NEG_INFINITY,
        f64::NEG_INFINITY,
    );
    for &(x, y) in ring {
        minx = minx.min(x);
        miny = miny.min(y);
        maxx = maxx.max(x);
        maxy = maxy.max(y);
    }
    ((maxx - minx).powi(2) + (maxy - miny).powi(2)).sqrt()
}

/// Contour matching key in em units: (area, centroid x, centroid y).
fn canon_key(c: &SubpixelContour, scale: f64) -> (f64, f64, f64) {
    let area = c.signed_area2().abs() * 0.5 * scale * scale;
    let n = c.points.len().max(1) as f64;
    let (mut cx, mut cy) = (0.0, 0.0);
    for &(x, y) in &c.points {
        cx += x;
        cy += y;
    }
    (area, cx / n * scale, cy / n * scale)
}

// ── Feature alignment ────────────────────────────────────────────────────────

/// A planned split as a matchable feature. Only *primary* landmarks
/// match — corners (including corner-sized run endpoints) and H/V
/// extrema; inflections and gentle run endpoints are threshold-dependent
/// per master and excluded.
#[derive(Debug, Clone, Copy)]
struct Feat {
    idx: usize,
    frac: f64,
    /// Normalized kind (a corner-sized `Tangent` becomes `Corner`).
    kind: SplitKind,
    /// Turn sign / extremum travel direction — stable across weight.
    sig: i8,
    /// Windowed turn magnitude (degrees) — corner significance.
    turn_deg: f64,
}

/// A run endpoint turning at least this much (degrees, windowed) is the same
/// physical feature as a corner and is matched as one.
const CORNERISH_TURN_DEG: f64 = 40.0;

/// Minimum turn (degrees, scale-relative window) around a tangent-sign
/// change to count as a real extremum rather than flat-run edge noise; a
/// large gentle arc turns well under a degree per 10px.
const MIN_EXTREMUM_TURN_DEG: f64 = 1.0;
/// Maximum turn over the same window: corner-scale turn at the crossing
/// means a corner's rounding zone, not an extremum.
const MAX_EXTREMUM_TURN_DEG: f64 = 45.0;
/// Up to this much total turn a section is always one cubic (or an
/// inflection pair on a genuine S) — raster fidelity never justifies an
/// extra structural point. Beyond it, equal-arc subdivision kicks in.
const SINGLE_CUBIC_MAX_TURN_DEG: f64 = 135.0;
/// A section is genuinely S-curved only when its running signed turn's
/// range exceeds the net turn by this margin (degrees) in every master.
const GENUINE_INFLECTION_DEG: f64 = 20.0;
/// Safety net: deviation beyond this fraction of the bbox diagonal (with
/// a px floor) means a mangled fit — well above single-cubic looseness
/// (~1-1.5%).
const SOURCE_DEV_FRAC: f64 = 0.03;
const SOURCE_DEV_MIN_PX: f64 = 8.0;
/// Flatness-test window as a fraction of the ring (clamped to >= 8 samples).
const EXTREMUM_FLAT_WINDOW_FRAC: f64 = 1.0 / 300.0;
/// Standoff (samples) between an extremum and a corner, and the non-max
/// suppression window between same-kind extrema.
const EXTREMUM_CORNER_STANDOFF: usize = 10;
const EXTREMUM_NMS: usize = 30;

/// The primary (matchable) features of a plan, in contour order: corners
/// from the plan plus H/V extrema detected on the dense ring itself (the
/// plan's piece-scoped extremum splits can miss one near a boundary).
fn feats_of(plan: &ContourPlan) -> Vec<Feat> {
    let n = plan.smoothed.len();
    let turns = fit::vertex_turns(&plan.smoothed);
    let fracs = arc_fracs(&plan.smoothed);
    let mut feats: Vec<Feat> = plan
        .splits
        .iter()
        .filter_map(|s| {
            let turn_deg = windowed_turn_deg(&turns, s.idx);
            let kind = match s.kind {
                SplitKind::Corner => SplitKind::Corner,
                SplitKind::Tangent if turn_deg.abs() >= CORNERISH_TURN_DEG => {
                    SplitKind::Corner
                }
                _ => return None,
            };
            let probe = Split { idx: s.idx, kind };
            Some(Feat {
                idx: s.idx,
                frac: fracs[s.idx],
                kind,
                sig: sig_of(&probe, &plan.smoothed, turn_deg, n),
                turn_deg,
            })
        })
        .collect();
    let corner_idxs: Vec<usize> = feats.iter().map(|f| f.idx).collect();
    feats.extend(dense_extrema(&plan.smoothed, &turns, &fracs, &corner_idxs));
    feats.sort_by_key(|f| f.idx);
    feats
}

/// H/V extrema found on the dense smoothed ring: a tangent component's sign
/// change at a genuinely curved sample, away from corners, one per window.
fn dense_extrema(
    smoothed: &[(f64, f64)],
    turns: &[f64],
    fracs: &[f64],
    corner_idxs: &[usize],
) -> Vec<Feat> {
    let n = smoothed.len();
    let comp = |i: usize, axis: usize| -> f64 {
        let a = smoothed[(i + n - 2) % n];
        let b = smoothed[(i + 2) % n];
        if axis == 0 { b.0 - a.0 } else { b.1 - a.1 }
    };
    let near_corner = |i: usize| {
        corner_idxs.iter().any(|&c| {
            let d = (i + n - c) % n;
            d.min(n - d) < EXTREMUM_CORNER_STANDOFF
        })
    };
    // Scale-relative flatness window: a gentle large-radius extremum still
    // reads as curved while a straight run reads flat.
    let flat_w = ((n as f64 * EXTREMUM_FLAT_WINDOW_FRAC) as usize).max(8);
    let wide_turn = |i: usize| -> f64 {
        let sum: f64 = (0..=2 * flat_w)
            .map(|k| turns[(i + n - flat_w + k) % n])
            .sum();
        sum.to_degrees()
    };
    let mut out: Vec<Feat> = Vec::new();
    // axis 0: tx sign change = x-extremum; axis 1: y-extremum.
    for (axis, kind) in [(0, SplitKind::ExtremumX), (1, SplitKind::ExtremumY)] {
        let mut cands: Vec<Feat> = Vec::new();
        // A crossing is a sign change between cyclically consecutive
        // nonzero samples, placed at the gap's midpoint — so a crossing
        // through an exact-zero plateau lands at the plateau's middle.
        let nz: Vec<(usize, f64)> = (0..n)
            .filter_map(|i| {
                let v = comp(i, axis);
                (v != 0.0).then(|| (i, v.signum()))
            })
            .collect();
        let dbg = std::env::var("IMG2BEZ_DEBUG_JOINT_EXTREMA").is_ok();
        for w in 0..nz.len() {
            let (i0, s0) = nz[w];
            let (i1, s1) = nz[(w + 1) % nz.len()];
            if s0 == s1 {
                continue;
            }
            let gap = (i1 + n - i0) % n;
            let mid = (i0 + gap / 2) % n;
            if near_corner(mid) {
                if dbg {
                    eprintln!(
                        "    ext-cand axis{axis} idx {mid} REJECT near-corner"
                    );
                }
                continue;
            }
            let turn_deg = wide_turn(mid);
            if turn_deg.abs() < MIN_EXTREMUM_TURN_DEG
                || turn_deg.abs() > MAX_EXTREMUM_TURN_DEG
            {
                if dbg {
                    eprintln!(
                        "    ext-cand axis{axis} idx {mid} REJECT \
                         (turn {turn_deg:.2})"
                    );
                }
                continue;
            }
            let other = comp(mid, 1 - axis);
            cands.push(Feat {
                idx: mid,
                frac: fracs[mid],
                kind,
                sig: if other >= 0.0 { 1 } else { -1 },
                turn_deg,
            });
        }
        // Non-max suppression, strongest first: a near-straight stem's
        // tangent oscillates across zero — keep the most curved crossing
        // per window.
        cands.sort_by(|a, b| {
            b.turn_deg
                .abs()
                .partial_cmp(&a.turn_deg.abs())
                .unwrap_or(std::cmp::Ordering::Equal)
        });
        let mut kept: Vec<Feat> = Vec::new();
        for c in cands {
            let clear = kept.iter().all(|k| {
                let d = (c.idx + n - k.idx) % n;
                d.min(n - d) >= EXTREMUM_NMS
            });
            if clear {
                kept.push(c);
            }
        }
        out.extend(kept);
    }
    out
}

fn windowed_turn_deg(turns: &[f64], idx: usize) -> f64 {
    let n = turns.len();
    let w = CORNER_TURN_WINDOW;
    let sum: f64 = (0..=2 * w).map(|k| turns[(idx + n - w + k) % n]).sum();
    sum.to_degrees()
}

fn sig_of(s: &Split, smoothed: &[(f64, f64)], turn_deg: f64, n: usize) -> i8 {
    match s.kind {
        SplitKind::Corner => {
            if turn_deg >= 0.0 {
                1
            } else {
                -1
            }
        }
        SplitKind::ExtremumX => {
            // Travel direction in y tells left from right side of the shape.
            let a = smoothed[(s.idx + n - 2) % n];
            let b = smoothed[(s.idx + 2) % n];
            if b.1 - a.1 >= 0.0 { 1 } else { -1 }
        }
        SplitKind::ExtremumY => {
            let a = smoothed[(s.idx + n - 2) % n];
            let b = smoothed[(s.idx + 2) % n];
            if b.0 - a.0 >= 0.0 { 1 } else { -1 }
        }
        SplitKind::Tangent | SplitKind::Inflection | SplitKind::FitterJoint => {
            0
        }
    }
}

/// Arc-length fraction of every sample along the closed ring.
fn arc_fracs(pts: &[(f64, f64)]) -> Vec<f64> {
    let n = pts.len();
    let mut cum = Vec::with_capacity(n);
    let mut acc = 0.0;
    for i in 0..n {
        cum.push(acc);
        let (x0, y0) = pts[i];
        let (x1, y1) = pts[(i + 1) % n];
        acc += ((x1 - x0).powi(2) + (y1 - y0).powi(2)).sqrt();
    }
    let total = acc.max(1e-12);
    cum.iter().map(|d| d / total).collect()
}

/// Sample index nearest an arc-length fraction.
fn idx_at_frac(fracs: &[f64], target: f64) -> usize {
    let t = target.rem_euclid(1.0);
    let mut best = 0;
    let mut bd = f64::INFINITY;
    for (i, &f) in fracs.iter().enumerate() {
        let d = cyc_dist(f, t);
        if d < bd {
            bd = d;
            best = i;
        }
    }
    best
}

fn cyc_dist(a: f64, b: f64) -> f64 {
    let d = (a - b).rem_euclid(1.0);
    d.min(1.0 - d)
}

fn same_sig(a: &Feat, b: &Feat) -> bool {
    a.kind == b.kind && a.sig == b.sig
}

/// Align every master's plan to a shared feature sequence, returning the
/// realized splits per master (same count, kinds, order).
fn align_plans(plans: &[ContourPlan]) -> Result<AlignedSplits, JointError> {
    let nm = plans.len();
    let feats: Vec<Vec<Feat>> = plans.iter().map(feats_of).collect();

    // Exact cyclic signature match against master 0: the lossless case.
    let f0 = &feats[0];
    let mut rots: Vec<usize> = vec![0; nm];
    let mut exact = !f0.is_empty();
    for (k, fk) in feats.iter().enumerate().skip(1) {
        match cyclic_exact(f0, fk) {
            Some(r) => rots[k] = r,
            None => {
                exact = false;
                break;
            }
        }
    }
    if exact {
        let ncols = f0.len();
        let mut out: Vec<Vec<Split>> = vec![Vec::with_capacity(ncols); nm];
        for i in 0..ncols {
            for (k, fk) in feats.iter().enumerate() {
                let f = fk[(i + rots[k]) % ncols];
                out[k].push(Split {
                    idx: f.idx,
                    kind: f.kind,
                });
            }
        }
        // Do NOT re-rotate per master — column i corresponds across masters
        // and the outline starts at column 0 everywhere. A master's idx
        // list may wrap once mid-list; the section sampler is cyclic.
        return Ok((out, vec![0; nm], false));
    }

    // Gapped alignment (two masters only).
    if nm != 2 {
        return Err(JointError::Alignment(format!(
            "feature sequences differ across {nm} masters; joint gapped \
             alignment supports two"
        )));
    }
    let cols = gapped_align(&feats[0], &feats[1])?;
    resolve_columns(cols, plans)
}

/// Rotation r such that fb[(i+r) % n] matches fa[i] by signature for all
/// i, minimizing total position distance; None if impossible.
fn cyclic_exact(fa: &[Feat], fb: &[Feat]) -> Option<usize> {
    if fa.len() != fb.len() || fa.is_empty() {
        return None;
    }
    let n = fa.len();
    let mut best: Option<(usize, f64)> = None;
    for r in 0..n {
        if !(0..n).all(|i| same_sig(&fa[i], &fb[(i + r) % n])) {
            continue;
        }
        let cost: f64 = (0..n)
            .map(|i| cyc_dist(fa[i].frac, fb[(i + r) % n].frac))
            .sum();
        if best.is_none_or(|(_, c)| cost < c) {
            best = Some((r, cost));
        }
    }
    best.map(|(r, _)| r)
}

/// One entry of the gapped alignment between two masters.
enum Pairing {
    Match(Feat, Feat),
    OnlyA(Feat),
    OnlyB(Feat),
}

/// Order-preserving cyclic alignment with gaps: anchor at the best
/// uniquely matching signature pair, then edit-distance-style DP over the
/// rotated sequences.
fn gapped_align(fa: &[Feat], fb: &[Feat]) -> Result<Vec<Pairing>, JointError> {
    if fa.is_empty() || fb.is_empty() {
        return Err(JointError::Alignment("a master has no features".into()));
    }
    // Anchor: the closest-positioned pair with equal signature, preferring
    // signatures unique in both sequences.
    let mut anchor: Option<(usize, usize, f64, bool)> = None;
    for (i, a) in fa.iter().enumerate() {
        let unique_a = fa.iter().filter(|f| same_sig(f, a)).count() == 1;
        for (j, b) in fb.iter().enumerate() {
            if !same_sig(a, b) {
                continue;
            }
            let unique =
                unique_a && fb.iter().filter(|f| same_sig(f, b)).count() == 1;
            let d = cyc_dist(a.frac, b.frac);
            let better = match anchor {
                None => true,
                Some((_, _, bd, bu)) => {
                    (unique && !bu) || (unique == bu && d < bd)
                }
            };
            if better {
                anchor = Some((i, j, d, unique));
            }
        }
    }
    let Some((ai, bj, _, _)) = anchor else {
        return Err(JointError::Alignment(
            "no feature signature is shared between the masters".into(),
        ));
    };
    // Rotate both to start at the anchor.
    let ra: Vec<Feat> =
        (0..fa.len()).map(|k| fa[(ai + k) % fa.len()]).collect();
    let rb: Vec<Feat> =
        (0..fb.len()).map(|k| fb[(bj + k) % fb.len()]).collect();
    // Positions relative to the anchor, so fracs are comparable.
    let rel = |f: &Feat, base: f64| (f.frac - base).rem_euclid(1.0);
    let base_a = ra[0].frac;
    let base_b = rb[0].frac;

    let (la, lb) = (ra.len(), rb.len());
    let gap = |f: &Feat| -> f64 {
        match f.kind {
            SplitKind::Corner => {
                0.04 + (f.turn_deg.abs() / 90.0).min(2.0) * 0.10
            }
            SplitKind::ExtremumX | SplitKind::ExtremumY => 0.08,
            SplitKind::Tangent => 0.03,
            SplitKind::Inflection | SplitKind::FitterJoint => 0.02,
        }
    };
    let inf = f64::INFINITY;
    let mut dp = vec![vec![inf; lb + 1]; la + 1];
    let mut bk = vec![vec![0u8; lb + 1]; la + 1]; // 1=match 2=gapA 3=gapB
    dp[0][0] = 0.0;
    for i in 0..=la {
        for j in 0..=lb {
            if i == 0 && j == 0 {
                continue;
            }
            let mut best = inf;
            let mut how = 0u8;
            if i > 0 && j > 0 && same_sig(&ra[i - 1], &rb[j - 1]) {
                let c = dp[i - 1][j - 1]
                    + (rel(&ra[i - 1], base_a) - rel(&rb[j - 1], base_b)).abs();
                if c < best {
                    best = c;
                    how = 1;
                }
            }
            if i > 0 {
                let c = dp[i - 1][j] + gap(&ra[i - 1]);
                if c < best {
                    best = c;
                    how = 2;
                }
            }
            if j > 0 {
                let c = dp[i][j - 1] + gap(&rb[j - 1]);
                if c < best {
                    best = c;
                    how = 3;
                }
            }
            dp[i][j] = best;
            bk[i][j] = how;
        }
    }
    if !dp[la][lb].is_finite() {
        return Err(JointError::Alignment(
            "no order-preserving alignment of the feature sequences".into(),
        ));
    }
    let mut cols = Vec::new();
    let (mut i, mut j) = (la, lb);
    while i > 0 || j > 0 {
        match bk[i][j] {
            1 => {
                cols.push(Pairing::Match(ra[i - 1], rb[j - 1]));
                i -= 1;
                j -= 1;
            }
            2 => {
                cols.push(Pairing::OnlyA(ra[i - 1]));
                i -= 1;
            }
            3 => {
                cols.push(Pairing::OnlyB(rb[j - 1]));
                j -= 1;
            }
            _ => unreachable!(),
        }
    }
    cols.reverse();
    if std::env::var("IMG2BEZ_DEBUG_JOINT").is_ok() {
        eprintln!("  joint: gapped alignment columns:");
        for c in &cols {
            match c {
                Pairing::Match(a, b) => eprintln!(
                    "    match  {:?}{:+} A@{:.3} <-> B@{:.3}",
                    a.kind, a.sig, a.frac, b.frac
                ),
                Pairing::OnlyA(f) => eprintln!(
                    "    onlyA  {:?}{:+} @{:.3} turn {:.0}",
                    f.kind, f.sig, f.frac, f.turn_deg
                ),
                Pairing::OnlyB(f) => eprintln!(
                    "    onlyB  {:?}{:+} @{:.3} turn {:.0}",
                    f.kind, f.sig, f.frac, f.turn_deg
                ),
            }
        }
    }
    Ok(cols)
}

/// Resolve gapped columns into realized splits: matched features keep
/// their own positions; a significant unmatched feature gets a
/// counterpart in the other master; a minor one is dropped from both.
fn resolve_columns(
    cols: Vec<Pairing>,
    plans: &[ContourPlan],
) -> Result<AlignedSplits, JointError> {
    let fracs_a = arc_fracs(&plans[0].smoothed);
    let fracs_b = arc_fracs(&plans[1].smoothed);

    // Matched pairs frame the interpolation of inserted positions.
    let matched: Vec<(f64, f64)> = cols
        .iter()
        .filter_map(|c| match c {
            Pairing::Match(a, b) => Some((a.frac, b.frac)),
            _ => None,
        })
        .collect();
    if matched.len() < 2 {
        return Err(JointError::Alignment(
            "fewer than two matched features; cannot frame insertions".into(),
        ));
    }
    // Map a frac in A to B by proportional position between the enclosing
    // matched pair (cyclic).
    let map_frac = |f: f64, forward: bool| -> f64 {
        let (from, to): (Vec<f64>, Vec<f64>) = if forward {
            (
                matched.iter().map(|m| m.0).collect(),
                matched.iter().map(|m| m.1).collect(),
            )
        } else {
            (
                matched.iter().map(|m| m.1).collect(),
                matched.iter().map(|m| m.0).collect(),
            )
        };
        let n = from.len();
        // Find the matched pair bracketing f in cyclic order of `from`.
        let mut lo = 0;
        let mut best = f64::INFINITY;
        for (k, &s) in from.iter().enumerate() {
            let d = (f - s).rem_euclid(1.0);
            if d < best {
                best = d;
                lo = k;
            }
        }
        let hi = (lo + 1) % n;
        let span_from = (from[hi] - from[lo]).rem_euclid(1.0).max(1e-9);
        let span_to = (to[hi] - to[lo]).rem_euclid(1.0);
        let t = ((f - from[lo]).rem_euclid(1.0) / span_from).clamp(0.0, 1.0);
        (to[lo] + t * span_to).rem_euclid(1.0)
    };

    // An unmatched extremum inserts with confidence (the H/V tangent is
    // right whether the other master curves or runs flat there); an
    // unmatched corner is a structural disagreement — a big one is
    // realized in both but flagged, a minor one dropped as noise.
    let mut out_a: Vec<Split> = Vec::new();
    let mut out_b: Vec<Split> = Vec::new();
    let mut inserted = vec![0usize; 2];
    let mut lossy = false;
    for c in &cols {
        match c {
            Pairing::Match(a, b) => {
                out_a.push(Split {
                    idx: a.idx,
                    kind: a.kind,
                });
                out_b.push(Split {
                    idx: b.idx,
                    kind: a.kind,
                });
            }
            Pairing::OnlyA(f) => {
                let keep = match f.kind {
                    SplitKind::Corner => {
                        lossy = true;
                        f.turn_deg.abs() >= MINOR_CORNER_DEG
                    }
                    _ => true,
                };
                if keep {
                    let fb = map_frac(f.frac, true);
                    out_a.push(Split {
                        idx: f.idx,
                        kind: f.kind,
                    });
                    out_b.push(Split {
                        idx: idx_at_frac(&fracs_b, fb),
                        kind: f.kind,
                    });
                    inserted[1] += 1;
                }
            }
            Pairing::OnlyB(f) => {
                let keep = match f.kind {
                    SplitKind::Corner => {
                        lossy = true;
                        f.turn_deg.abs() >= MINOR_CORNER_DEG
                    }
                    _ => true,
                };
                if keep {
                    let fa = map_frac(f.frac, false);
                    out_a.push(Split {
                        idx: idx_at_frac(&fracs_a, fa),
                        kind: f.kind,
                    });
                    out_b.push(Split {
                        idx: f.idx,
                        kind: f.kind,
                    });
                    inserted[0] += 1;
                }
            }
        }
    }

    // Columns correspond by list position; do NOT re-rotate either list.
    // Each list must be cyclically ordered (ascending, at most one wrap)
    // and collision-free, or the plan is unusable.
    for m in [&out_a, &out_b] {
        let len = m.len();
        if len < 2 {
            return Err(JointError::Alignment(
                "fewer than two resolved features".into(),
            ));
        }
        let descents = (0..len)
            .filter(|&i| m[(i + 1) % len].idx <= m[i].idx)
            .count();
        if descents > 1 {
            return Err(JointError::Alignment(
                "resolved feature positions collide or fold back".into(),
            ));
        }
    }
    Ok((vec![out_a, out_b], inserted, lossy))
}

// ── Joint fitting ────────────────────────────────────────────────────────────

/// Fit every master to the shared plan: one unified segment structure per
/// section column, fitted to each master's own dense samples.
fn fit_joint(
    plans: &[ContourPlan],
    aligned: &[Vec<Split>],
    masters: &[JointMaster],
) -> Result<Vec<FittedContour>, JointError> {
    let nm = plans.len();
    let ncols = aligned[0].len();
    if aligned.iter().any(|a| a.len() != ncols) || ncols < 2 {
        return Err(JointError::Alignment(
            "aligned split lists disagree in length".into(),
        ));
    }
    let accuracy: Vec<f64> = masters
        .iter()
        .map(|m| {
            let scale = m.config.em_height / m.gray.height() as f64;
            (m.config.fit_accuracy / scale).clamp(0.5, 3.0)
        })
        .collect();

    let mut out: Vec<FittedContour> = (0..nm)
        .map(|_| FittedContour {
            segs: Vec::new(),
            is_line: Vec::new(),
            joint_kind: Vec::new(),
        })
        .collect();

    for si in 0..ncols {
        let a_kind = aligned[0][si].kind;
        let b_kind = aligned[0][(si + 1) % ncols].kind;
        // Section samples per master.
        let mut samples: Vec<Vec<Point>> = Vec::with_capacity(nm);
        for (mi, plan) in plans.iter().enumerate() {
            let n = plan.smoothed.len();
            let a = aligned[mi][si].idx;
            let b = aligned[mi][(si + 1) % ncols].idx;
            let len = {
                let d = (b + n - a) % n;
                if d == 0 { n } else { d }
            };
            samples.push(
                (0..=len)
                    .map(|k| {
                        let (x, y) = plan.smoothed[(a + k) % n];
                        Point::new(x, y)
                    })
                    .collect(),
            );
        }

        // A section is a line only when every master reads it as one.
        let all_line = (0..nm).all(|mi| {
            let n = plans[mi].smoothed.len();
            let a = aligned[mi][si].idx;
            let b = aligned[mi][(si + 1) % ncols].idx;
            in_line_section(&plans[mi], a, b, n)
                || (samples[mi].len() <= fit::SHORT_STRAIGHT_MAX_SAMPLES
                    && fit::chord_deviation_pts(&samples[mi])
                        <= fit::SHORT_STRAIGHT_MAX_DEVIATION)
        });
        if all_line || samples.iter().all(|s| s.len() < 3) {
            for (mi, s) in samples.iter().enumerate() {
                out[mi].segs.push(line_seg(s[0], s[s.len() - 1]));
                out[mi].is_line.push(true);
                out[mi].joint_kind.push(a_kind);
            }
            continue;
        }

        // Unified cascade: single cubic everywhere, else inflection pair
        // everywhere, else the same equal-arc subdivision everywhere.
        let tangents: Vec<(Vec2, Vec2)> = samples
            .iter()
            .map(|s| {
                (
                    fit::constrained_end_tangent(s, a_kind, true),
                    fit::constrained_end_tangent(s, b_kind, false),
                )
            })
            .collect();
        let singles: Vec<([Point; 4], f64)> = samples
            .iter()
            .zip(&tangents)
            .map(|(s, (t0, t1))| {
                let (mut seg, err) = fit::constrained_cubic_fit(s, *t0, *t1);
                enforce_tangents(&mut seg, *t0, *t1);
                (seg, err)
            })
            .collect();
        let all_single =
            singles.iter().zip(&accuracy).all(|((_, err), acc)| {
                *err <= acc * fit::CONSTRAINED_FIT_TOLERANCE_FACTOR
            });
        let section_turn = |mi: usize| -> f64 {
            let plan = &plans[mi];
            let n = plan.smoothed.len();
            let a = aligned[mi][si].idx;
            let b = aligned[mi][(si + 1) % ncols].idx;
            let len = {
                let d = (b + n - a) % n;
                if d == 0 { n } else { d }
            };
            let turns = fit::vertex_turns(&plan.smoothed);
            (0..len)
                .map(|k| turns[(a + k) % n])
                .sum::<f64>()
                .to_degrees()
                .abs()
        };
        if all_single {
            for (mi, (seg, _)) in singles.iter().enumerate() {
                out[mi].segs.push(*seg);
                out[mi].is_line.push(false);
                out[mi].joint_kind.push(a_kind);
            }
            continue;
        }

        // Single cubic missed tolerance. If the boundary genuinely
        // S-curves in every master, split at the inflection; a monotone
        // arc that merely varies in curvature earns no extra point.
        let section_sness = |mi: usize| -> f64 {
            let plan = &plans[mi];
            let n = plan.smoothed.len();
            let a = aligned[mi][si].idx;
            let b = aligned[mi][(si + 1) % ncols].idx;
            let len = {
                let d = (b + n - a) % n;
                if d == 0 { n } else { d }
            };
            let turns = fit::vertex_turns(&plan.smoothed);
            let (mut cum, mut lo, mut hi) = (0.0f64, 0.0f64, 0.0f64);
            for k in 0..len {
                cum += turns[(a + k) % n];
                lo = lo.min(cum);
                hi = hi.max(cum);
            }
            ((hi - lo) - cum.abs()).to_degrees()
        };
        let genuine_s = (0..plans.len())
            .all(|mi| section_sness(mi) > GENUINE_INFLECTION_DEG);
        let pairs: Vec<Option<InflectionPair>> = if genuine_s {
            samples
                .iter()
                .zip(&tangents)
                .zip(&accuracy)
                .map(|((s, (t0, t1)), acc)| {
                    fit::fit_split_at_inflection(
                        s,
                        *t0,
                        *t1,
                        acc * fit::CONSTRAINED_FIT_TOLERANCE_FACTOR,
                    )
                })
                .collect()
        } else {
            vec![None; samples.len()]
        };
        if pairs.iter().all(Option::is_some) {
            // A half is a line only if every master reads it as one.
            let mut half_line = [true, true];
            for p in pairs.iter().flatten() {
                half_line[0] &= p.1[0];
                half_line[1] &= p.1[1];
            }
            for (mi, p) in pairs.iter().enumerate() {
                let (segs, _) = p.as_ref().unwrap();
                for (h, seg) in segs.iter().enumerate() {
                    out[mi].segs.push(*seg);
                    out[mi].is_line.push(half_line[h]);
                    out[mi].joint_kind.push(if h == 0 {
                        a_kind
                    } else {
                        SplitKind::FitterJoint
                    });
                }
            }
            continue;
        }

        // Otherwise the single cubic IS the answer up to the turn cap —
        // raster deviation never justifies an extra structural point.
        if (0..plans.len())
            .all(|mi| section_turn(mi) <= SINGLE_CUBIC_MAX_TURN_DEG)
        {
            if std::env::var("IMG2BEZ_DEBUG_JOINT").is_ok() {
                for (mi, single) in singles.iter().enumerate() {
                    eprintln!(
                        "  joint: section {si} master {mi}: single by turn \
                         ({:.0} deg, err {:.1})",
                        section_turn(mi),
                        single.1,
                    );
                }
            }
            for (mi, (seg, _)) in singles.iter().enumerate() {
                out[mi].segs.push(*seg);
                out[mi].is_line.push(false);
                out[mi].joint_kind.push(a_kind);
            }
            continue;
        }

        // Equal-arc K-subdivision, the same K for every master: one cubic
        // per quadrant of total turn, measured on the dense samples.
        let k = plans
            .iter()
            .zip(aligned)
            .map(|(plan, spl)| {
                let n = plan.smoothed.len();
                let a = spl[si].idx;
                let b = spl[(si + 1) % ncols].idx;
                let len = {
                    let d = (b + n - a) % n;
                    if d == 0 { n } else { d }
                };
                let turns = fit::vertex_turns(&plan.smoothed);
                let total: f64 = (0..len)
                    .map(|k| turns[(a + k) % n])
                    .sum::<f64>()
                    .to_degrees()
                    .abs();
                ((total / 80.0).ceil() as usize).clamp(2, 8)
            })
            .max()
            .unwrap_or(2);
        for (mi, s) in samples.iter().enumerate() {
            let (spans, bounds) = split_equal_arc(s, k);
            // One tangent per interior boundary, shared by both adjacent
            // spans: joints are G1 by construction. End tangents point
            // backward, hence the negation.
            let boundary_t: Vec<Vec2> =
                bounds[1..k].iter().map(|&b| tangent_at(s, b)).collect();
            for (h, span) in spans.iter().enumerate() {
                let t0 = if h == 0 {
                    tangents[mi].0
                } else {
                    boundary_t[h - 1]
                };
                let t1 = if h == k - 1 {
                    tangents[mi].1
                } else {
                    -boundary_t[h]
                };
                let (mut seg, _) = fit::constrained_cubic_fit(span, t0, t1);
                enforce_tangents(&mut seg, t0, t1);
                out[mi].segs.push(seg);
                out[mi].is_line.push(false);
                out[mi].joint_kind.push(if h == 0 {
                    a_kind
                } else {
                    SplitKind::Tangent
                });
            }
        }
    }
    Ok(out)
}

/// Does the section from `a` to `b` (sample indices, cyclic) lie inside one of
/// the plan's straight runs, with a little slack at the ends?
fn in_line_section(plan: &ContourPlan, a: usize, b: usize, n: usize) -> bool {
    plan.line_sections.iter().any(|&(ra, rb)| {
        let run = (rb + n - ra) % n;
        let run = if run == 0 { n } else { run };
        let off_a = (a + n - ra) % n;
        let off_b = (b + n - ra) % n;
        let inside = |o: usize| {
            o <= run + LINE_CONTAIN_SLACK || o + LINE_CONTAIN_SLACK >= n
        };
        inside(off_a) && inside(off_b) && {
            let oa = if off_a + LINE_CONTAIN_SLACK >= n {
                0
            } else {
                off_a
            };
            let ob = off_b.min(run);
            oa <= ob
        }
    })
}

/// Split section samples into `k` equal-arc spans (adjacent spans share
/// their boundary sample); also returns the boundary indices.
fn split_equal_arc(
    samples: &[Point],
    k: usize,
) -> (Vec<Vec<Point>>, Vec<usize>) {
    let n = samples.len();
    // k comes from the max turn across masters but n is this master's
    // sample count; a span needs 2 samples, so k must not exceed n-1 or
    // the bounds slice out of range.
    let k = k.clamp(1, n.saturating_sub(1).max(1));
    let mut cum = Vec::with_capacity(n);
    let mut acc = 0.0;
    cum.push(0.0);
    for i in 1..n {
        acc += (samples[i] - samples[i - 1]).hypot();
        cum.push(acc);
    }
    let total = acc.max(1e-12);
    let mut bounds = vec![0usize];
    for h in 1..k {
        let target = total * h as f64 / k as f64;
        let mut j = bounds[h - 1] + 1;
        while j + 1 < n && cum[j] < target {
            j += 1;
        }
        bounds.push(j.min(n - 2).max(bounds[h - 1] + 1));
    }
    bounds.push(n - 1);
    let spans = (0..k)
        .map(|h| samples[bounds[h]..=bounds[h + 1]].to_vec())
        .collect();
    (spans, bounds)
}

/// Enforce prescribed end tangents: project each handle onto its tangent,
/// re-deriving degenerate handles at chord/3 (keeps joints exactly G1).
/// Conventions match `fit::constrained_cubic_fit`: `t0` forward
/// (p1 = p0 + t0*a), `t1` backward (p2 = p3 + t1*b).
fn enforce_tangents(seg: &mut [Point; 4], t0: Vec2, t1: Vec2) {
    let chord = (seg[3] - seg[0]).hypot();
    if chord < 1e-9 {
        return;
    }
    let min_len = chord * 0.02;
    let p0 = seg[0];
    let p3 = seg[3];
    let out_len = (seg[1] - p0).dot(t0);
    seg[1] = p0
        + t0 * if out_len > min_len {
            out_len
        } else {
            chord / 3.0
        };
    let in_len = (seg[2] - p3).dot(t1);
    seg[2] = p3
        + t1 * if in_len > min_len {
            in_len
        } else {
            chord / 3.0
        };
}

/// Tangent direction at sample `i`, by central difference over a small reach.
fn tangent_at(samples: &[Point], i: usize) -> Vec2 {
    let n = samples.len();
    let a = i.saturating_sub(3);
    let b = (i + 3).min(n - 1);
    let v = samples[b] - samples[a];
    let len = v.hypot();
    if len > 1e-9 {
        v / len
    } else {
        Vec2::new(1.0, 0.0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::model::geom::line_seg;

    /// A dense unit-spaced square ring of side `s`, starting at the origin.
    fn square_ring(s: usize) -> Vec<(f64, f64)> {
        let s = s as f64;
        let mut ring = Vec::new();
        let n = s as usize;
        for i in 0..n {
            ring.push((i as f64, 0.0));
        }
        for i in 0..n {
            ring.push((s, i as f64));
        }
        for i in 0..n {
            ring.push((s - i as f64, s));
        }
        for i in 0..n {
            ring.push((0.0, s - i as f64));
        }
        ring
    }

    fn square_fit(s: f64, offset: f64) -> FittedContour {
        let p = |x: f64, y: f64| Point::new(x + offset, y + offset);
        FittedContour {
            segs: vec![
                line_seg(p(0.0, 0.0), p(s, 0.0)),
                line_seg(p(s, 0.0), p(s, s)),
                line_seg(p(s, s), p(0.0, s)),
                line_seg(p(0.0, s), p(0.0, 0.0)),
            ],
            is_line: vec![true; 4],
            joint_kind: vec![SplitKind::Corner; 4],
        }
    }

    #[test]
    fn faithful_fit_has_near_zero_source_deviation() {
        let ring = square_ring(100);
        let dev = max_source_deviation(&square_fit(100.0, 0.0), &ring);
        assert!(dev < 1.0, "faithful fit deviates {dev}");
    }

    #[test]
    fn displaced_fit_trips_the_source_deviation_net() {
        // The mangled-outline guard must catch a fit shifted well past the
        // threshold.
        let ring = square_ring(100);
        let dev = max_source_deviation(&square_fit(100.0, 25.0), &ring);
        let threshold =
            (ring_diag(&ring) * SOURCE_DEV_FRAC).max(SOURCE_DEV_MIN_PX);
        assert!(
            dev > threshold,
            "displaced fit should trip the net: dev {dev} <= {threshold}"
        );
    }
}
