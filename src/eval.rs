//! Reference-comparison evaluation for traced glyph quality.
//!
//! Compares a traced output against a hand-drawn reference `.glif` file
//! across multiple quality metrics: scale match, shape distance, point
//! count, segment types, handle alignment, grid alignment, and contour count.

use std::fmt;
use std::path::Path;

use kurbo::{BezPath, ParamCurve, ParamCurveArclen, PathSeg, Point, Rect, Shape};

use crate::error::TraceError;
use crate::TraceResult;

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
    /// Ratio of traced bbox diagonal to reference bbox diagonal.
    pub ratio: f64,
    /// Traced dimensions (w x h).
    pub traced_size: (f64, f64),
    /// Reference dimensions (w x h).
    pub ref_size: (f64, f64),
    pub score: f64,
}

pub struct ShapeMetric {
    /// Hausdorff distance after normalizing traced to reference scale/position.
    pub hausdorff: f64,
    /// Mean nearest-point distance after normalization.
    pub mean: f64,
    /// Reference bbox diagonal (for context).
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
    /// Weighted overall score.
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

// ── Evaluation ───────────────────────────────────────────────────

/// Compare traced output against a reference, producing an `EvalReport`.
pub fn evaluate(traced: &GlyphPaths, reference: &GlyphPaths, grid: i32, ref_path: &str) -> EvalReport {
    let t_bbox = paths_bbox(&traced.paths);
    let r_bbox = paths_bbox(&reference.paths);

    let scale = eval_scale(&t_bbox, &r_bbox);
    let shape = eval_shape(traced, reference, &t_bbox, &r_bbox);
    let points = eval_points(traced, reference);
    let segments = eval_segments(traced, reference);
    let hv_handles = eval_hv_handles(traced);
    let grid_metric = eval_grid(traced, grid);
    let contours = eval_contours(traced, reference);

    EvalReport {
        scale,
        shape,
        points,
        segments,
        hv_handles,
        grid: grid_metric,
        contours,
        reference_path: ref_path.to_string(),
    }
}

// ── Scale match ─────────────────────────────────────────────────

fn eval_scale(t_bbox: &Rect, r_bbox: &Rect) -> ScaleMetric {
    let t_diag = bbox_diagonal(t_bbox).max(1.0);
    let r_diag = bbox_diagonal(r_bbox).max(1.0);
    let ratio = t_diag / r_diag;

    // Score: 1.0 at ratio=1.0, drops quickly for mismatch.
    // |ratio - 1| of 0.05 → score 0.95, 0.5 → score 0.0.
    let score = (1.0 - (ratio - 1.0).abs() * 2.0).max(0.0);

    ScaleMetric {
        ratio,
        traced_size: (t_bbox.width(), t_bbox.height()),
        ref_size: (r_bbox.width(), r_bbox.height()),
        score,
    }
}

// ── Shape distance ───────────────────────────────────────────────

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
    let n = 256;
    let mut t_samples = sample_points(&traced.paths, n);
    let r_samples = sample_points(&reference.paths, n);

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

    // Score based on Hausdorff in font units (after normalization).
    // At 1024 UPM: <5u = excellent, 5-15u = good, 15-30u = fair, >30u = poor.
    let score = (1.0 - hausdorff / 40.0).max(0.0);

    ShapeMetric {
        hausdorff,
        mean,
        bbox_diagonal: r_diag,
        score,
    }
}

// ── Point count ──────────────────────────────────────────────────

fn count_on_curve(paths: &[BezPath]) -> usize {
    let mut count = 0;
    for path in paths {
        for el in path.elements() {
            match el {
                kurbo::PathEl::MoveTo(_) => count += 1,
                kurbo::PathEl::LineTo(_) => count += 1,
                kurbo::PathEl::CurveTo(_, _, _) => count += 1,
                kurbo::PathEl::QuadTo(_, _) => count += 1,
                kurbo::PathEl::ClosePath => {}
            }
        }
    }
    count
}

fn eval_points(traced: &GlyphPaths, reference: &GlyphPaths) -> PointMetric {
    let t = count_on_curve(&traced.paths);
    let r = count_on_curve(&reference.paths);
    let ratio = if r > 0 { t as f64 / r as f64 } else { 1.0 };
    let score = (1.0 - (ratio - 1.0).abs()).max(0.0);

    PointMetric {
        traced: t,
        reference: r,
        ratio,
        score,
    }
}

// ── Segment types ────────────────────────────────────────────────

fn count_curve_line(paths: &[BezPath]) -> (usize, usize) {
    let mut curves = 0;
    let mut lines = 0;
    for path in paths {
        for el in path.elements() {
            match el {
                kurbo::PathEl::CurveTo(..) | kurbo::PathEl::QuadTo(..) => curves += 1,
                kurbo::PathEl::LineTo(_) => lines += 1,
                _ => {}
            }
        }
    }
    (curves, lines)
}

fn line_fraction(curves: usize, lines: usize) -> f64 {
    let total = curves + lines;
    if total == 0 {
        0.0
    } else {
        lines as f64 / total as f64
    }
}

fn eval_segments(traced: &GlyphPaths, reference: &GlyphPaths) -> SegmentMetric {
    let (tc, tl) = count_curve_line(&traced.paths);
    let (rc, rl) = count_curve_line(&reference.paths);
    let t_frac = line_fraction(tc, tl);
    let r_frac = line_fraction(rc, rl);
    let score = (1.0 - (t_frac - r_frac).abs()).max(0.0);

    SegmentMetric {
        traced_curves: tc,
        traced_lines: tl,
        ref_curves: rc,
        ref_lines: rl,
        traced_line_frac: t_frac,
        ref_line_frac: r_frac,
        score,
    }
}

// ── H/V handle alignment ────────────────────────────────────────

fn eval_hv_handles(traced: &GlyphPaths) -> HVHandleMetric {
    let mut aligned = 0usize;
    let mut total = 0usize;

    for path in &traced.paths {
        let elements = path.elements();
        let mut prev_oncurve = Point::ZERO;

        for el in elements {
            match *el {
                kurbo::PathEl::MoveTo(p) => {
                    prev_oncurve = p;
                }
                kurbo::PathEl::LineTo(p) => {
                    prev_oncurve = p;
                }
                kurbo::PathEl::CurveTo(a, b, p) => {
                    total += 1;
                    if is_hv(prev_oncurve, a) {
                        aligned += 1;
                    }
                    total += 1;
                    if is_hv(p, b) {
                        aligned += 1;
                    }
                    prev_oncurve = p;
                }
                kurbo::PathEl::QuadTo(a, p) => {
                    total += 1;
                    if is_hv(prev_oncurve, a) || is_hv(p, a) {
                        aligned += 1;
                    }
                    prev_oncurve = p;
                }
                kurbo::PathEl::ClosePath => {}
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
    let dx = (oncurve.x - offcurve.x).abs();
    let dy = (oncurve.y - offcurve.y).abs();
    dx < 0.5 || dy < 0.5
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
            let pts: Vec<Point> = match *el {
                kurbo::PathEl::MoveTo(p) => vec![p],
                kurbo::PathEl::LineTo(p) => vec![p],
                kurbo::PathEl::CurveTo(_, _, p) => vec![p],
                kurbo::PathEl::QuadTo(_, p) => vec![p],
                kurbo::PathEl::ClosePath => vec![],
            };
            for p in pts {
                total += 1;
                if (p.x % g).abs() < 0.5 && (p.y % g).abs() < 0.5 {
                    on_grid += 1;
                }
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
    let score = if t == r { 1.0 } else { 0.0 };

    ContourMetric {
        traced: t,
        reference: r,
        score,
    }
}

// ── Display ──────────────────────────────────────────────────────

impl fmt::Display for EvalReport {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        writeln!(f)?;
        writeln!(f, "  Eval vs {}", self.reference_path)?;
        writeln!(f)?;

        // Scale
        writeln!(
            f,
            "  Scale         {:.0}x{:.0} vs {:.0}x{:.0}  ({:.2}x)                  {:.3}",
            self.scale.traced_size.0, self.scale.traced_size.1,
            self.scale.ref_size.0, self.scale.ref_size.1,
            self.scale.ratio, self.scale.score,
        )?;

        // Shape (after normalization)
        writeln!(
            f,
            "  Shape         Hausdorff {:<5.1}  mean {:<5.1}  (normalized)      {:.3}",
            self.shape.hausdorff, self.shape.mean, self.shape.score,
        )?;

        // Points
        writeln!(
            f,
            "  Points        {} vs {} on-curve  (\u{00d7}{:.2})                   {:.3}",
            self.points.traced, self.points.reference, self.points.ratio, self.points.score,
        )?;

        // Segments
        writeln!(
            f,
            "  Segments      {}c+{}l vs {}c+{}l  ({:.2} vs {:.2})            {:.3}",
            self.segments.traced_curves,
            self.segments.traced_lines,
            self.segments.ref_curves,
            self.segments.ref_lines,
            self.segments.traced_line_frac,
            self.segments.ref_line_frac,
            self.segments.score,
        )?;

        // H/V handles
        writeln!(
            f,
            "  H/V handles   {}/{}  ({:.0}%)                                 {:.3}",
            self.hv_handles.aligned,
            self.hv_handles.total,
            self.hv_handles.score * 100.0,
            self.hv_handles.score,
        )?;

        // Grid
        if self.grid.grid_size > 0 {
            writeln!(
                f,
                "  Grid ({})      {}/{}  ({:.0}%)                                 {:.3}",
                self.grid.grid_size,
                self.grid.on_grid,
                self.grid.total,
                self.grid.score * 100.0,
                self.grid.score,
            )?;
        } else {
            writeln!(f, "  Grid          (off)                                        1.000")?;
        }

        // Contours
        writeln!(
            f,
            "  Contours      {} vs {}                                       {:.3}",
            self.contours.traced, self.contours.reference, self.contours.score,
        )?;

        writeln!(f)?;
        writeln!(f, "  Overall       {:.3}", self.overall())?;

        Ok(())
    }
}
