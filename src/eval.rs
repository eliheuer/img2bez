//! Reference-comparison evaluation for traced glyph quality.
//!
//! Compares a traced output against a hand-drawn reference `.glif` file
//! across multiple quality metrics: scale match, shape distance, point
//! count, segment types, handle alignment, grid alignment, and contour count.

use std::fmt;
use std::path::Path;

use kurbo::{BezPath, ParamCurve, ParamCurveArclen, PathEl, PathSeg, Point, Rect, Shape};

use crate::error::TraceError;
use crate::TraceResult;

/// Number of sample points per path set for shape distance computation.
/// Higher values give more accurate Hausdorff/mean distance but cost O(n²).
const SHAPE_SAMPLE_COUNT: usize = 256;

/// Hausdorff distance at which the shape score drops to zero. A glyph
/// whose worst-case point-to-point distance exceeds this is scored 0.0.
const HAUSDORFF_ZERO_SCORE: f64 = 40.0;

/// Collected bezier paths for evaluation.
pub struct GlyphPaths {
    pub paths: Vec<BezPath>,
}

/// Full evaluation report comparing traced output to a reference.
pub struct EvalReport {
    pub scale: ScaleMetric,
    pub shape: ShapeMetric,
    pub points: PointMetric,
    pub segments: SegmentMetric,
    pub hv_handles: HVHandleMetric,
    pub grid: GridMetric,
    pub contours: ContourMetric,
    pub reference_path: String,
}

pub struct ScaleMetric {
    pub ratio: f64,
    pub traced_size: (f64, f64),
    pub ref_size: (f64, f64),
    pub score: f64,
}

pub struct ShapeMetric {
    pub hausdorff: f64,
    pub mean: f64,
    pub bbox_diagonal: f64,
    pub score: f64,
}

pub struct PointMetric {
    pub traced: usize,
    pub reference: usize,
    pub ratio: f64,
    pub score: f64,
}

pub struct SegmentMetric {
    pub traced_curves: usize,
    pub traced_lines: usize,
    pub ref_curves: usize,
    pub ref_lines: usize,
    pub traced_line_frac: f64,
    pub ref_line_frac: f64,
    pub score: f64,
}

pub struct HVHandleMetric {
    pub aligned: usize,
    pub total: usize,
    pub score: f64,
}

pub struct GridMetric {
    pub grid_size: i32,
    pub on_grid: usize,
    pub total: usize,
    pub score: f64,
}

pub struct ContourMetric {
    pub traced: usize,
    pub reference: usize,
    pub score: f64,
}

impl EvalReport {
    /// Weighted overall score (0.0–1.0).
    ///
    /// Weight rationale:
    /// - **Shape (0.30)**: visual similarity is the primary goal.
    /// - **Scale (0.15)**: wrong scale makes everything wrong.
    /// - **H/V handles (0.15)**: critical for font editor usability.
    /// - **Points, segments, grid, contours (0.10 each)**: structural
    ///   metrics that matter but are secondary to visual match.
    pub fn overall(&self) -> f64 {
        self.scale.score * 0.15
            + self.shape.score * 0.30
            + self.points.score * 0.10
            + self.segments.score * 0.10
            + self.hv_handles.score * 0.15
            + self.grid.score * 0.10
            + self.contours.score * 0.10
    }
}

// ── Loading ──────────────────────────────────────────────────────

/// Load a reference `.glif` file into `GlyphPaths`.
pub fn load_glif(path: &Path) -> Result<GlyphPaths, TraceError> {
    let glyph = norad::Glyph::load(path)?;
    let mut paths = Vec::new();
    for contour in &glyph.contours {
        let bez = contour
            .to_kurbo()
            .map_err(|e| TraceError::ContourConvert(e.to_string()))?;
        paths.push(bez);
    }
    Ok(GlyphPaths { paths })
}

/// Build `GlyphPaths` from a `TraceResult`.
pub fn from_trace_result(result: &TraceResult) -> GlyphPaths {
    GlyphPaths {
        paths: result.paths.clone(),
    }
}

// ── Path statistics ──────────────────────────────────────────────

/// Segment and point counts extracted from a set of BezPaths in a single pass.
struct PathStats {
    on_curve: usize,
    curves: usize,
    lines: usize,
}

/// Count on-curve points, curves, and lines across all paths.
fn count_path_stats(paths: &[BezPath]) -> PathStats {
    let mut on_curve = 0;
    let mut curves = 0;
    let mut lines = 0;
    for path in paths {
        for el in path.elements() {
            match el {
                PathEl::MoveTo(_) => on_curve += 1,
                PathEl::LineTo(_) => {
                    on_curve += 1;
                    lines += 1;
                }
                PathEl::CurveTo(..) => {
                    on_curve += 1;
                    curves += 1;
                }
                PathEl::QuadTo(..) => {
                    on_curve += 1;
                    curves += 1;
                }
                PathEl::ClosePath => {}
            }
        }
    }
    PathStats {
        on_curve,
        curves,
        lines,
    }
}

/// Fraction of segments that are lines (0.0–1.0).
fn line_fraction(stats: &PathStats) -> f64 {
    let total = stats.curves + stats.lines;
    if total == 0 {
        0.0
    } else {
        stats.lines as f64 / total as f64
    }
}

// ── Evaluation ───────────────────────────────────────────────────

/// Compare traced output against a reference, producing an `EvalReport`.
pub fn evaluate(
    traced: &GlyphPaths,
    reference: &GlyphPaths,
    grid: i32,
    ref_path: &str,
) -> EvalReport {
    let t_bbox = paths_bbox(&traced.paths);
    let r_bbox = paths_bbox(&reference.paths);
    let t_stats = count_path_stats(&traced.paths);
    let r_stats = count_path_stats(&reference.paths);

    EvalReport {
        scale: eval_scale(&t_bbox, &r_bbox),
        shape: eval_shape(traced, reference, &t_bbox, &r_bbox),
        points: eval_points(&t_stats, &r_stats),
        segments: eval_segments(&t_stats, &r_stats),
        hv_handles: eval_hv_handles(traced),
        grid: eval_grid(traced, grid),
        contours: eval_contours(traced, reference),
        reference_path: ref_path.to_string(),
    }
}

// ── Scale match ─────────────────────────────────────────────────

fn eval_scale(t_bbox: &Rect, r_bbox: &Rect) -> ScaleMetric {
    let t_diag = bbox_diagonal(t_bbox).max(1.0);
    let r_diag = bbox_diagonal(r_bbox).max(1.0);
    let ratio = t_diag / r_diag;
    let score = (1.0 - (ratio - 1.0).abs() * 2.0).max(0.0);

    ScaleMetric {
        ratio,
        traced_size: (t_bbox.width(), t_bbox.height()),
        ref_size: (r_bbox.width(), r_bbox.height()),
        score,
    }
}

// ── Shape distance ───────────────────────────────────────────────

/// Sample points along all path segments, proportional to arc length.
fn sample_points(paths: &[BezPath], num_samples: usize) -> Vec<Point> {
    let mut total_len = 0.0;
    let mut seg_lens: Vec<(PathSeg, f64)> = Vec::new();
    for path in paths {
        for seg in path.segments() {
            let len = seg.arclen(0.5);
            total_len += len;
            seg_lens.push((seg, len));
        }
    }

    if total_len == 0.0 || seg_lens.is_empty() {
        return Vec::new();
    }

    let mut samples = Vec::with_capacity(num_samples);
    for (seg, len) in &seg_lens {
        let n = ((*len / total_len) * num_samples as f64).ceil().max(1.0) as usize;
        for i in 0..n {
            let t = (i as f64 + 0.5) / n as f64;
            samples.push(seg.eval(t));
        }
    }
    samples
}

/// Union bounding box of all path segments.
fn paths_bbox(paths: &[BezPath]) -> Rect {
    let mut bbox = Rect::ZERO;
    let mut first = true;
    for path in paths {
        for seg in path.segments() {
            let b = seg.bounding_box();
            if first {
                bbox = b;
                first = false;
            } else {
                bbox = bbox.union(b);
            }
        }
    }
    bbox
}

fn bbox_diagonal(bbox: &Rect) -> f64 {
    (bbox.width().powi(2) + bbox.height().powi(2)).sqrt()
}

/// Directed Hausdorff: max over `from` of min distance to `to`.
fn directed_hausdorff(from: &[Point], to: &[Point]) -> f64 {
    if from.is_empty() || to.is_empty() {
        return f64::INFINITY;
    }
    from.iter()
        .map(|a| {
            to.iter()
                .map(|b| a.distance(*b))
                .fold(f64::INFINITY, f64::min)
        })
        .fold(0.0f64, f64::max)
}

/// Mean nearest-point distance from `from` to `to`.
fn mean_nearest(from: &[Point], to: &[Point]) -> f64 {
    if from.is_empty() || to.is_empty() {
        return f64::INFINITY;
    }
    let sum: f64 = from
        .iter()
        .map(|a| {
            to.iter()
                .map(|b| a.distance(*b))
                .fold(f64::INFINITY, f64::min)
        })
        .sum();
    sum / from.len() as f64
}

fn eval_shape(
    traced: &GlyphPaths,
    reference: &GlyphPaths,
    t_bbox: &Rect,
    r_bbox: &Rect,
) -> ShapeMetric {
    let mut t_samples = sample_points(&traced.paths, SHAPE_SAMPLE_COUNT);
    let r_samples = sample_points(&reference.paths, SHAPE_SAMPLE_COUNT);

    // Normalize traced samples: translate+scale to match reference bbox.
    let t_diag = bbox_diagonal(t_bbox).max(1.0);
    let r_diag = bbox_diagonal(r_bbox).max(1.0);
    let scale = r_diag / t_diag;

    let t_center = Point::new(
        t_bbox.x0 + t_bbox.width() / 2.0,
        t_bbox.y0 + t_bbox.height() / 2.0,
    );
    let r_center = Point::new(
        r_bbox.x0 + r_bbox.width() / 2.0,
        r_bbox.y0 + r_bbox.height() / 2.0,
    );

    for p in &mut t_samples {
        *p = Point::new(
            (p.x - t_center.x) * scale + r_center.x,
            (p.y - t_center.y) * scale + r_center.y,
        );
    }

    let h_tr = directed_hausdorff(&t_samples, &r_samples);
    let h_rt = directed_hausdorff(&r_samples, &t_samples);
    let hausdorff = h_tr.max(h_rt);

    let m_tr = mean_nearest(&t_samples, &r_samples);
    let m_rt = mean_nearest(&r_samples, &t_samples);
    let mean = (m_tr + m_rt) / 2.0;

    let score = (1.0 - hausdorff / HAUSDORFF_ZERO_SCORE).max(0.0);

    ShapeMetric {
        hausdorff,
        mean,
        bbox_diagonal: r_diag,
        score,
    }
}

// ── Point count ──────────────────────────────────────────────────

fn eval_points(traced: &PathStats, reference: &PathStats) -> PointMetric {
    let ratio = if reference.on_curve > 0 {
        traced.on_curve as f64 / reference.on_curve as f64
    } else {
        1.0
    };
    let score = (1.0 - (ratio - 1.0).abs()).max(0.0);

    PointMetric {
        traced: traced.on_curve,
        reference: reference.on_curve,
        ratio,
        score,
    }
}

// ── Segment types ────────────────────────────────────────────────

fn eval_segments(traced: &PathStats, reference: &PathStats) -> SegmentMetric {
    let t_frac = line_fraction(traced);
    let r_frac = line_fraction(reference);
    let score = (1.0 - (t_frac - r_frac).abs()).max(0.0);

    SegmentMetric {
        traced_curves: traced.curves,
        traced_lines: traced.lines,
        ref_curves: reference.curves,
        ref_lines: reference.lines,
        traced_line_frac: t_frac,
        ref_line_frac: r_frac,
        score,
    }
}

// ── H/V handle alignment ────────────────────────────────────────

fn eval_hv_handles(traced: &GlyphPaths) -> HVHandleMetric {
    let mut aligned = 0usize;
    let mut total = 0usize;
    let mut prev = Point::ZERO;

    for path in &traced.paths {
        for el in path.elements() {
            match *el {
                PathEl::MoveTo(p) | PathEl::LineTo(p) => prev = p,
                PathEl::CurveTo(a, b, p) => {
                    total += 2;
                    if is_hv(prev, a) {
                        aligned += 1;
                    }
                    if is_hv(p, b) {
                        aligned += 1;
                    }
                    prev = p;
                }
                PathEl::QuadTo(a, p) => {
                    total += 1;
                    if is_hv(prev, a) || is_hv(p, a) {
                        aligned += 1;
                    }
                    prev = p;
                }
                PathEl::ClosePath => {}
            }
        }
    }

    let score = if total > 0 {
        aligned as f64 / total as f64
    } else {
        1.0
    };
    HVHandleMetric {
        aligned,
        total,
        score,
    }
}

fn is_hv(oncurve: Point, offcurve: Point) -> bool {
    (oncurve.x - offcurve.x).abs() < 0.5 || (oncurve.y - offcurve.y).abs() < 0.5
}

// ── Grid alignment ───────────────────────────────────────────────

fn eval_grid(traced: &GlyphPaths, grid: i32) -> GridMetric {
    if grid <= 0 {
        return GridMetric {
            grid_size: grid,
            on_grid: 0,
            total: 0,
            score: 1.0,
        };
    }

    let g = grid as f64;
    let mut on_grid = 0usize;
    let mut total = 0usize;

    for path in &traced.paths {
        for el in path.elements() {
            let p = match *el {
                PathEl::MoveTo(p) | PathEl::LineTo(p) => p,
                PathEl::CurveTo(_, _, p) | PathEl::QuadTo(_, p) => p,
                PathEl::ClosePath => continue,
            };
            total += 1;
            if (p.x % g).abs() < 0.5 && (p.y % g).abs() < 0.5 {
                on_grid += 1;
            }
        }
    }

    let score = if total > 0 {
        on_grid as f64 / total as f64
    } else {
        1.0
    };
    GridMetric {
        grid_size: grid,
        on_grid,
        total,
        score,
    }
}

// ── Contour count ────────────────────────────────────────────────

fn eval_contours(traced: &GlyphPaths, reference: &GlyphPaths) -> ContourMetric {
    let t = traced.paths.len();
    let r = reference.paths.len();
    ContourMetric {
        traced: t,
        reference: r,
        score: if t == r { 1.0 } else { 0.0 },
    }
}

// ── Display ──────────────────────────────────────────────────────

impl fmt::Display for EvalReport {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        writeln!(f)?;
        writeln!(f, "  Eval vs {}", self.reference_path)?;
        writeln!(f)?;
        writeln!(
            f,
            "  Scale         {:.0}x{:.0} vs {:.0}x{:.0}  ({:.2}x)                  {:.3}",
            self.scale.traced_size.0,
            self.scale.traced_size.1,
            self.scale.ref_size.0,
            self.scale.ref_size.1,
            self.scale.ratio,
            self.scale.score
        )?;
        writeln!(
            f,
            "  Shape         Hausdorff {:<5.1}  mean {:<5.1}  (normalized)      {:.3}",
            self.shape.hausdorff, self.shape.mean, self.shape.score
        )?;
        writeln!(
            f,
            "  Points        {} vs {} on-curve  (\u{00d7}{:.2})                   {:.3}",
            self.points.traced, self.points.reference, self.points.ratio, self.points.score
        )?;
        writeln!(
            f,
            "  Segments      {}c+{}l vs {}c+{}l  ({:.2} vs {:.2})            {:.3}",
            self.segments.traced_curves,
            self.segments.traced_lines,
            self.segments.ref_curves,
            self.segments.ref_lines,
            self.segments.traced_line_frac,
            self.segments.ref_line_frac,
            self.segments.score
        )?;
        writeln!(
            f,
            "  H/V handles   {}/{}  ({:.0}%)                                 {:.3}",
            self.hv_handles.aligned,
            self.hv_handles.total,
            self.hv_handles.score * 100.0,
            self.hv_handles.score
        )?;
        if self.grid.grid_size > 0 {
            writeln!(
                f,
                "  Grid ({})      {}/{}  ({:.0}%)                                 {:.3}",
                self.grid.grid_size,
                self.grid.on_grid,
                self.grid.total,
                self.grid.score * 100.0,
                self.grid.score
            )?;
        } else {
            writeln!(
                f,
                "  Grid          (off)                                        1.000"
            )?;
        }
        writeln!(
            f,
            "  Contours      {} vs {}                                       {:.3}",
            self.contours.traced, self.contours.reference, self.contours.score
        )?;
        writeln!(f)?;
        writeln!(f, "  Overall       {:.3}", self.overall())?;
        Ok(())
    }
}
