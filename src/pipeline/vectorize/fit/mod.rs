// Copyright 2026 the img2bez Authors
// SPDX-License-Identifier: Apache-2.0 OR MIT

//! Type-design-quality curve fitting from sub-pixel boundaries.
//! `trace_contour` stages (submodules in the same order): resample and
//! smooth; detect corners/runs/extrema/inflections from curvature;
//! assemble splits (`plan_contour`) — exactly the on-curve points; fit
//! each section as a line or one constrained cubic; clean up + G1 + G2.

use kurbo::{BezPath, PathEl, Point, Vec2};

use super::subpixel::SubpixelContour;

mod corners;
mod fallback;
mod finish;
mod plan;
mod runs;
mod sections;

pub(super) use finish::{
    cap_handle_reach, harmonize, smooth_joins, tame_short_cubic_handles,
};
pub(super) use plan::{plan_contour, vertex_turns};
pub(super) use sections::{
    constrained_cubic_fit, constrained_end_tangent, fit_split_at_inflection,
};

use fallback::fit_open_samples;
use finish::{
    collapse_corner_slivers, collapse_micro_lines, merge_collinear_lines,
};
use sections::fit_sections;

// ── Tuning constants (units: source-image pixels / degrees) ─────────

// Resampling and smoothing.

/// Arc-length spacing of resampled boundary points.
const SAMPLE_SPACING: f64 = 1.0;

/// Gaussian smoothing sigma (samples) applied before feature detection.
const SMOOTH_SIGMA: f64 = 1.2;

/// Mean |turn| per sample above which the smoothing sigma escalates
/// (clean boundaries sit well under 0.02).
const MAX_MEAN_ABS_TURN: f64 = 0.10;

/// Minimum boundary samples per detected corner; denser means
/// quantization stairs, not type features.
const MIN_SAMPLES_PER_CORNER: usize = 30;

// Corner detection.

// A vertex is a corner when the turn within +-1 sample exceeds
// `corner_deg` (caller-supplied, default 12), the window turn reaches a
// designed-corner angle, and the central turn dominates the window.
const CORNER_WINDOW: usize = 5;
const CORNER_CONCENTRATION: f64 = 0.55;

/// Minimum window turn (degrees) for a designed corner: scale-tolerant
/// under smearing; below 45 so exact 45-degree chamfers survive noise
/// (fake corners top out near 28).
const CORNER_MIN_TOTAL_TURN_DEG: f64 = 35.0;

/// Above this window turn (degrees) the vertex is a corner regardless of
/// concentration — concentration only disambiguates the 45-90 band, and a
/// corner smeared by low-res upscaling can read concentration ~0.3.
const CORNER_CUSP_TURN_DEG: f64 = 90.0;

/// A shallower break still counts as a corner when both outer flanks are
/// straight (turn <= the quiet limit): a 20-30 degree line-line break is a
/// deliberate chamfer/facet joint; the same turn inside curvature is not.
const CORNER_SHALLOW_MIN_TOTAL_TURN_DEG: f64 = 18.0;
const CORNER_SHALLOW_FLANK_MAX_DEG: f64 = 5.0;

// Straight-run detection.

/// A run is straight while its chord deviation stays under
/// max(floor, slope * chord); the linear envelope separates flats from
/// gentle arcs at every scale (slope 0.005 rejects r < ~750px at L=30px).
const STRAIGHT_DEV_FLOOR: f64 = 0.15;
const STRAIGHT_DEV_SLOPE: f64 = 0.005;

/// Relaxed deviation floor for runs starting next to a corner: terminal
/// and serif flats carry render sag that breaks the strict floor
/// mid-flat; round-shape flats have no corners nearby and keep it.
const CORNER_FLAT_DEV_FLOOR: f64 = 0.45;

/// Max samples an accepted run's ends may extend with the relaxed floor,
/// absorbing the corner-rounding tail.
const RUN_EXTEND_MAX_SAMPLES: usize = 20;

/// A run is axis-aligned (stem/crossbar candidate even without corners)
/// when its chord is within this many degrees of horizontal or vertical.
const RUN_AXIS_MAX_DEG: f64 = 20.0;

/// Minimum chord for a free-standing straight run, and the (smaller)
/// minimum for runs that end at corners (serif/terminal flats).
const STRAIGHT_MIN_CHORD: f64 = 30.0;
/// Arc-flat veto: a short corner-free run with same-direction curving
/// flanks is the flat of an arc, not structure. Flank turn is bounded
/// above too — a large one means a blur-rounded corner (real flat).
const ARC_FLAT_FLANK_WINDOW: usize = 24;
const ARC_FLAT_MIN_FLANK_DEG: f64 = 5.0;
const ARC_FLAT_MAX_FLANK_DEG: f64 = 30.0;
const ARC_FLAT_MAX_CHORD_FRAC: f64 = 0.08;
const STRAIGHT_MIN_CHORD_AT_CORNER: f64 = 7.0;

// Split assembly.

/// Snap run ends and extrema onto existing splits closer than this; must
/// cover the corner-rounding zone (~±4 samples) or curve slivers appear.
const SPLIT_SNAP_SAMPLES: usize = 8;

// Extremum detection.

/// Minimum prominence (px) for an x/y extremum to earn an on-curve point.
const EXTREMUM_PROMINENCE: f64 = 1.0;

/// Scale-relative prominence (fraction of bbox diagonal): real extrema
/// recede a large fraction of the shape, wobble a few px at any size.
const EXTREMUM_PROMINENCE_FRAC: f64 = 0.012;

/// Minimum sample distance from an extremum to other splits. Tighter than
/// SPLIT_SNAP_SAMPLES: designs place bowl extrema ~10px from corners.
const EXTREMUM_STANDOFF: usize = 5;

/// Half-width (samples) of the parabola-fit extremum-localization window.
const EXTREMUM_REFINE_WINDOW: usize = 14;

// Section fitting.

/// Accept a constrained single-cubic section fit when its max deviation
/// is within this multiple of the base accuracy; otherwise subdivide.
pub(super) const CONSTRAINED_FIT_TOLERANCE_FACTOR: f64 = 1.8;

// Inflection detection.

/// Curvature must reach 1/INFLECTION_MAX_RADIUS on both sides of the sign
/// change within INFLECTION_REACH samples: bowl flats wobble around zero,
/// a genuine S-spine swings well past 1/150 on both sides.
const INFLECTION_MAX_RADIUS: f64 = 150.0;
const INFLECTION_REACH: usize = 60;
/// Minimum summed turn (degrees) per side of the crossing: an S-spine
/// integrates real turn, ink-grain meander integrates to nearly nothing.
const INFLECTION_MIN_SIDE_TURN_DEG: f64 = 9.0;
/// Inflections are optional structure: keep them clear of other splits by
/// this many samples, and on large contours by a fraction of the ring.
const INFLECTION_STANDOFF: usize = 12;
const INFLECTION_STANDOFF_FRAC: f64 = 1.0 / 64.0;
/// Same-feature extrema need room between them too; against corners the
/// fixed EXTREMUM_STANDOFF stays.
const EXTREMUM_SPACING_FRAC: f64 = 1.0 / 128.0;
/// Reach scans stay this many samples from piece boundaries
/// (corner-rounding curvature is not evidence).
const INFLECTION_BOUNDARY_MARGIN: usize = 12;
/// Curvature smoothing sigma (samples) for inflection detection.
const CURVATURE_SIGMA: f64 = 3.0;

// Join smoothing and harmonization.

/// Max angle between handle directions for a join to count as smooth.
const SMOOTH_JOIN_MAX_DEG: f64 = 30.0;

/// Cap on how far harmonization may move an on-curve point (px).
const HARMONIZE_MAX_SHIFT: f64 = 2.0;

/// Free (corner/inflection) tangent directions within this many degrees
/// of an axis snap to the axis — type convention keeps handles H/V
/// wherever the shape allows.
const FREE_DIR_AXIS_SNAP_DEG: f64 = 15.0;

/// Tighter snap band for corner tangents: the rounding zone biases
/// shallow diagonals toward the axis (axis corners read 3-11 degrees off,
/// designed diagonals ~13); in the band up to FREE_DIR_AXIS_SNAP_DEG the
/// bias-free direction beyond the rounding zone decides.
const CORNER_AXIS_SNAP_DEG: f64 = 12.0;

// Post-fit cleanup.

/// A curve between two lines shorter than this chord (px) is a
/// rasterization-rounded corner; reconstruct the sharp corner if the
/// line intersection is within the reach (guards curved joins).
const CORNER_SLIVER_MAX_CHORD: f64 = 32.0;
const CORNER_SLIVER_MAX_REACH: f64 = 15.0;

/// Cubics with chords up to this length (px) get their handle lengths
/// clamped to SHORT_CUBIC_MAX_HANDLE of the chord.
const SHORT_CUBIC_MAX_CHORD: f64 = 15.0;
const SHORT_CUBIC_MAX_HANDLE: f64 = 0.45;

/// A section up to this many samples with chord deviation below
/// SHORT_STRAIGHT_MAX_DEVIATION is emitted as a line regardless of end
/// kinds: a cubic on a near-degenerate straight produces junk handles.
pub(super) const SHORT_STRAIGHT_MAX_SAMPLES: usize = 16;

/// Hard cap on the fallback subdivision search (`fit_open_samples`);
/// beyond it the best fit so far ships.
pub(super) const OPEN_FIT_MAX_SEGMENTS: usize = 24;

pub(super) const SHORT_STRAIGHT_MAX_DEVIATION: f64 = 1.2;

/// A line shorter than this (px) between two curves tangent-continuous
/// across it (turn < MICRO_LINE_MAX_TURN) is a fitting stub;
/// `collapse_micro_lines` welds the curves across it.
const MICRO_LINE_MAX_CHORD: f64 = 14.0;
const MICRO_LINE_MAX_TURN: f64 = 25.0;

// ── Structure types ──────────────────────────────────────────────────

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) enum SplitKind {
    Corner,
    /// Straight-run endpoint: tangent constrained to the line direction.
    Tangent,
    /// x-extremum: tangent is vertical.
    ExtremumX,
    /// y-extremum: tangent is horizontal.
    ExtremumY,
    Inflection,
    /// Interior joint the fitter created for error tolerance, not a
    /// structure decision; refinement may merge across it.
    FitterJoint,
}

#[derive(Debug, Clone, Copy)]
pub(super) struct Split {
    pub(super) idx: usize,
    pub(super) kind: SplitKind,
}

/// One contour's worth of fitted segments plus per-joint metadata.
pub(super) struct FittedContour {
    /// Cubic segments; lines are degenerate cubics with handles at 1/3
    /// and 2/3 of the chord, and `is_line[i]` remembers which is which.
    pub(super) segs: Vec<[Point; 4]>,
    pub(super) is_line: Vec<bool>,
    /// Kind of the joint at the START of segment i (= end of segment i-1).
    pub(super) joint_kind: Vec<SplitKind>,
}

/// Trace one sub-pixel contour into a type-quality BezPath (px space).
/// `raster` enables the optional raster-loss refinement; `None` skips it.
pub fn trace_contour(
    contour: &SubpixelContour,
    accuracy: f64,
    smoothing: f64,
    corner_deg: f64,
    corner_smear: bool,
    soft_source: bool,
    raster: Option<&super::refine::RasterTarget>,
) -> BezPath {
    let plan = match plan_contour(
        contour,
        smoothing,
        corner_deg,
        corner_smear,
        soft_source,
        raster,
    ) {
        PlanOutcome::TooSmall => return polyline_path(&contour.points),
        PlanOutcome::NoSplits { smoothed } => {
            // Degenerate: fit the whole closed loop.
            return fit_samples_closed(&smoothed, accuracy);
        }
        PlanOutcome::Plan(plan) => plan,
    };
    let n = plan.smoothed.len();

    // Raster refinement runs after G1 alignment (handle directions final)
    // but before G2 harmonization (style keeps the last word).
    let fitted = fit_sections(
        &plan.smoothed,
        &plan.splits,
        &plan.line_sections,
        n,
        accuracy,
    );
    let fitted = merge_collinear_lines(fitted);
    let fitted = collapse_corner_slivers(fitted);
    let fitted = collapse_micro_lines(fitted);
    let fitted = tame_short_cubic_handles(fitted);
    let fitted = smooth_joins(fitted);
    let fitted = match raster {
        Some(rt) => super::refine::refine_contour(fitted, rt),
        None => fitted,
    };
    let fitted = harmonize(fitted);
    let fitted = cap_handle_reach(fitted);
    contour_to_bezpath(&fitted)
}

/// The structure plan for one contour — every structural decision the
/// tracer makes, separated from fitting so `super::joint` can decide
/// structure once across a master set.
pub(super) struct ContourPlan {
    pub(super) smoothed: Vec<(f64, f64)>,
    pub(super) splits: Vec<Split>,
    pub(super) line_sections: Vec<(usize, usize)>,
}

pub(super) enum PlanOutcome {
    /// Too few samples to plan; emit the raw polyline.
    TooSmall,
    /// No splits found (an O with no detected extrema, a blob); fit closed.
    NoSplits {
        smoothed: Vec<(f64, f64)>,
    },
    Plan(ContourPlan),
}

// ── Assembly ─────────────────────────────────────────────────────────

/// Emit the fitted segments as a closed BezPath (lines as LineTo).
pub(super) fn contour_to_bezpath(contour: &FittedContour) -> BezPath {
    let mut path = BezPath::new();
    if contour.segs.is_empty() {
        return path;
    }
    path.move_to(contour.segs[0][0]);
    for (i, seg) in contour.segs.iter().enumerate() {
        if contour.is_line[i] {
            path.line_to(seg[3]);
        } else {
            path.curve_to(seg[1], seg[2], seg[3]);
        }
    }
    path.push(PathEl::ClosePath);
    path
}

/// Fit a whole closed loop with no structural splits.
fn fit_samples_closed(smoothed: &[(f64, f64)], accuracy: f64) -> BezPath {
    let mut samples: Vec<Point> =
        smoothed.iter().map(|&(x, y)| Point::new(x, y)).collect();
    samples.push(samples[0]);
    let segs = fit_open_samples(&samples, accuracy);
    let c = FittedContour {
        joint_kind: vec![SplitKind::FitterJoint; segs.len()],
        is_line: vec![false; segs.len()],
        segs,
    };
    contour_to_bezpath(&c)
}

// ── Small helpers ────────────────────────────────────────────────────

/// A closed BezPath connecting the points with straight lines.
fn polyline_path(points: &[(f64, f64)]) -> BezPath {
    let mut path = BezPath::new();
    if points.is_empty() {
        return path;
    }
    path.move_to(Point::new(points[0].0, points[0].1));
    for &(x, y) in &points[1..] {
        path.line_to(Point::new(x, y));
    }
    path.push(PathEl::ClosePath);
    path
}

fn dist(a: (f64, f64), b: (f64, f64)) -> f64 {
    ((b.0 - a.0).powi(2) + (b.1 - a.1).powi(2)).sqrt()
}

fn dist_pt(a: Point, b: Point) -> f64 {
    (b - a).hypot()
}

fn angle_between(u: Vec2, v: Vec2) -> f64 {
    u.cross(v).atan2(u.dot(v)).abs()
}

/// Maximum perpendicular deviation of interior points from the chord.
pub(super) fn chord_deviation_pts(points: &[Point]) -> f64 {
    let tuples: Vec<(f64, f64)> = points.iter().map(|p| (p.x, p.y)).collect();
    chord_deviation(&tuples)
}

fn chord_deviation(points: &[(f64, f64)]) -> f64 {
    let n = points.len();
    if n < 3 {
        return 0.0;
    }
    let (ax, ay) = points[0];
    let (bx, by) = points[n - 1];
    let dx = bx - ax;
    let dy = by - ay;
    let len = (dx * dx + dy * dy).sqrt();
    if len < 1e-12 {
        return 0.0;
    }
    points[1..n - 1]
        .iter()
        .map(|&(px, py)| ((px - ax) * dy - (py - ay) * dx).abs() / len)
        .fold(0.0, f64::max)
}

/// Intersection of the infinite lines a1-a2 and b1-b2 (None if parallel).
fn line_intersection(
    a1: Point,
    a2: Point,
    b1: Point,
    b2: Point,
) -> Option<Point> {
    let da = a2 - a1;
    let db = b2 - b1;
    let denom = da.cross(db);
    if denom.abs() < 1e-9 {
        return None;
    }
    let t = (b1 - a1).cross(db) / denom;
    Some(a1 + da * t)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::pipeline::vectorize::subpixel::SubpixelContour;
    use kurbo::{CubicBez, ParamCurve};

    #[test]
    fn circle_fits_to_four_smooth_points() {
        // 200px-diameter circle sampled densely.
        let points: Vec<(f64, f64)> = (0..1024)
            .map(|i| {
                let a = i as f64 / 1024.0 * std::f64::consts::TAU;
                (300.0 + 100.0 * a.cos(), 300.0 + 100.0 * a.sin())
            })
            .collect();
        let contour = SubpixelContour { points };
        let path = trace_contour(&contour, 0.5, 1.0, 12.0, false, false, None);
        let oncurves = path
            .elements()
            .iter()
            .filter(|el| matches!(el, PathEl::CurveTo(..) | PathEl::LineTo(..)))
            .count();
        assert!(
            (4..=6).contains(&oncurves),
            "circle should fit with ~4 points, got {oncurves}"
        );
        let lines = path
            .elements()
            .iter()
            .filter(|el| matches!(el, PathEl::LineTo(..)))
            .count();
        assert_eq!(lines, 0, "circle must contain no line segments");
    }

    #[test]
    fn rounded_rectangle_has_lines_and_corner_free_joins() {
        // Rectangle 300x200 with r=40 rounded corners, dense sampling.
        let mut points = Vec::new();
        let (w, h, r) = (300.0, 200.0, 40.0);
        let arc = |cx: f64, cy: f64, a0: f64, points: &mut Vec<(f64, f64)>| {
            for i in 0..64 {
                let a = a0 + i as f64 / 64.0 * std::f64::consts::FRAC_PI_2;
                points.push((cx + r * a.cos(), cy + r * a.sin()));
            }
        };
        // Walk CCW starting on the bottom edge.
        for i in 0..128 {
            points.push((r + (w - 2.0 * r) * i as f64 / 128.0, 0.0));
        }
        arc(w - r, r, -std::f64::consts::FRAC_PI_2, &mut points);
        for i in 0..96 {
            points.push((w, r + (h - 2.0 * r) * i as f64 / 96.0));
        }
        arc(w - r, h - r, 0.0, &mut points);
        for i in 0..128 {
            points.push((w - r - (w - 2.0 * r) * i as f64 / 128.0, h));
        }
        arc(r, h - r, std::f64::consts::FRAC_PI_2, &mut points);
        for i in 0..96 {
            points.push((0.0, h - r - (h - 2.0 * r) * i as f64 / 96.0));
        }
        arc(r, r, std::f64::consts::PI, &mut points);
        let contour = SubpixelContour { points };
        let path = trace_contour(&contour, 0.5, 1.0, 12.0, false, false, None);
        let lines = path
            .elements()
            .iter()
            .filter(|el| matches!(el, PathEl::LineTo(..)))
            .count();
        assert_eq!(
            lines, 4,
            "rounded rect should have exactly 4 straight sides"
        );
    }

    #[test]
    fn quarter_circle_fits_one_cubic() {
        let samples: Vec<Point> = (0..=157)
            .map(|i| {
                let a = i as f64 / 157.0 * std::f64::consts::FRAC_PI_2;
                Point::new(100.0 * a.cos(), 100.0 * a.sin())
            })
            .collect();
        let segs = fit_open_samples(&samples, 1.5);
        assert_eq!(
            segs.len(),
            1,
            "quarter circle should fit one cubic, got {}",
            segs.len()
        );
    }

    #[test]
    fn reference_o_quadrant_fits_one_cubic() {
        // Bottom-right quadrant of the reference 'o' (squarish, high tension).
        let c = CubicBez::new(
            Point::new(316.0, -16.0),
            Point::new(482.0, -16.0),
            Point::new(598.0, 110.0),
            Point::new(600.0, 288.0),
        );
        let samples: Vec<Point> =
            (0..=400).map(|i| c.eval(i as f64 / 400.0)).collect();
        let segs = fit_open_samples(&samples, 1.5);
        assert_eq!(
            segs.len(),
            1,
            "reference quadrant should fit one cubic, got {}",
            segs.len()
        );
    }
}
