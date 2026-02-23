//! Post-processing pipeline for traced bezier contours.
//!
//! Transforms raw fitted curves into font-ready outlines:
//! contour direction, extrema insertion, grid snapping,
//! H/V handle correction, and optional chamfers.

use std::f64::consts::{FRAC_PI_2, PI};

use kurbo::{BezPath, CubicBez, ParamCurve, PathEl, Point, Vec2};

use crate::config::TracingConfig;

/// Apply all post-processing steps to traced contours.
pub fn process(paths: &[BezPath], config: &TracingConfig) -> Vec<BezPath> {
    let mut result = paths.to_vec();

    if config.fix_direction {
        result = fix_directions(&result);
    }

    if config.add_extrema {
        let depth = config.min_extrema_depth;
        result = result.iter().map(|p| insert_extrema(p, depth)).collect();
    }

    if config.grid > 0 {
        let g = config.grid as f64;
        result = result.iter().map(|p| snap_to_grid(p, g)).collect();
    }

    // Snap handles within 15° of H/V to exact H/V.
    result = result.iter().map(|p| snap_hv_handles(p, 15.0)).collect();

    // Convert near-straight curves to lines.
    let line_tol = if config.grid > 0 {
        (config.grid as f64 * 3.0).max(6.0)
    } else {
        6.0
    };
    result = result
        .iter()
        .map(|p| curves_to_lines(p, line_tol))
        .collect();

    // Merge collinear lines, remove tiny segments.
    let tight = if config.grid > 0 {
        config.grid as f64
    } else {
        1.0
    };
    result = result
        .iter()
        .map(|p| {
            let p = merge_collinear(p, tight);
            remove_tiny(&p, tight)
        })
        .collect();

    if config.chamfer_size > 0.0 {
        let size = config.chamfer_size;
        let min = config.chamfer_min_edge;
        result = result.iter().map(|p| chamfer(p, size, min)).collect();
    }

    result
}

// ── Contour direction ────────────────────────────────────

/// Ensure outer contours are CCW, counters are CW.
fn fix_directions(paths: &[BezPath]) -> Vec<BezPath> {
    if paths.is_empty() {
        return vec![];
    }

    let areas: Vec<f64> = paths.iter().map(signed_area).collect();

    let outer = areas
        .iter()
        .enumerate()
        .max_by(|(_, a), (_, b)| a.abs().partial_cmp(&b.abs()).unwrap())
        .map(|(i, _)| i)
        .unwrap_or(0);

    paths
        .iter()
        .enumerate()
        .map(|(i, path)| {
            let should_flip = if i == outer {
                areas[i] < 0.0
            } else {
                areas[i] > 0.0
            };
            if should_flip {
                reverse_path(path)
            } else {
                path.clone()
            }
        })
        .collect()
}

/// Signed area via shoelace (on-curve points only).
fn signed_area(path: &BezPath) -> f64 {
    let mut area = 0.0;
    let mut first = Point::ZERO;
    let mut cur = Point::ZERO;

    for el in path.elements() {
        match *el {
            PathEl::MoveTo(p) => {
                first = p;
                cur = p;
            }
            PathEl::LineTo(p) | PathEl::CurveTo(_, _, p) | PathEl::QuadTo(_, p) => {
                area += cur.x * p.y - p.x * cur.y;
                cur = p;
            }
            PathEl::ClosePath => {
                area += cur.x * first.y - first.x * cur.y;
            }
        }
    }
    area / 2.0
}

/// Reverse a BezPath's winding direction.
fn reverse_path(path: &BezPath) -> BezPath {
    let mut first = Point::ZERO;
    let mut segs: Vec<(Point, Seg)> = Vec::new();
    let mut closed = false;

    for el in path.elements() {
        match *el {
            PathEl::MoveTo(p) => first = p,
            PathEl::LineTo(p) => {
                segs.push((p, Seg::Line));
            }
            PathEl::CurveTo(a, b, p) => {
                segs.push((p, Seg::Curve(a, b)));
            }
            PathEl::QuadTo(a, p) => {
                segs.push((p, Seg::Quad(a)));
            }
            PathEl::ClosePath => closed = true,
        }
    }

    let mut out = BezPath::new();
    if segs.is_empty() {
        out.move_to(first);
        if closed {
            out.push(PathEl::ClosePath);
        }
        return out;
    }

    out.move_to(segs.last().unwrap().0);

    for i in (0..segs.len()).rev() {
        let target = if i == 0 { first } else { segs[i - 1].0 };
        match segs[i].1 {
            Seg::Line => out.line_to(target),
            Seg::Curve(a, b) => {
                out.push(PathEl::CurveTo(b, a, target));
            }
            Seg::Quad(a) => {
                out.push(PathEl::QuadTo(a, target));
            }
        }
    }

    if closed {
        out.push(PathEl::ClosePath);
    }
    out
}

#[derive(Clone, Copy)]
enum Seg {
    Line,
    Curve(Point, Point),
    Quad(Point),
}

// ── H/V handle snapping ─────────────────────────────────

/// Snap handles within `deg` of H/V to exact H/V.
fn snap_hv_handles(path: &BezPath, threshold_deg: f64) -> BezPath {
    let thresh = threshold_deg.to_radians();
    let mut els: Vec<PathEl> = path.elements().to_vec();
    let mut prev = Point::ZERO;

    for el in &mut els {
        match el {
            PathEl::MoveTo(p) | PathEl::LineTo(p) => {
                prev = *p;
            }
            PathEl::CurveTo(c1, c2, p3) => {
                *c1 = snap_handle(*c1, prev, thresh);
                *c2 = snap_handle(*c2, *p3, thresh);
                prev = *p3;
            }
            PathEl::QuadTo(_, p) => prev = *p,
            PathEl::ClosePath => {}
        }
    }

    BezPath::from_vec(els)
}

/// Snap a handle to exact H/V if it's close enough.
fn snap_handle(handle: Point, anchor: Point, threshold: f64) -> Point {
    let dx = handle.x - anchor.x;
    let dy = handle.y - anchor.y;
    if dx * dx + dy * dy < 1e-12 {
        return handle;
    }

    let angle = dy.atan2(dx).abs(); // 0..PI

    if angle < threshold || (PI - angle) < threshold {
        return Point::new(handle.x, anchor.y);
    }
    if (angle - FRAC_PI_2).abs() < threshold {
        return Point::new(anchor.x, handle.y);
    }
    handle
}

// ── Grid snapping ────────────────────────────────────────

fn snap_to_grid(path: &BezPath, g: f64) -> BezPath {
    BezPath::from_vec(
        path.elements()
            .iter()
            .map(|el| match *el {
                PathEl::MoveTo(p) => PathEl::MoveTo(snap(p, g)),
                PathEl::LineTo(p) => PathEl::LineTo(snap(p, g)),
                PathEl::CurveTo(a, b, p) => PathEl::CurveTo(snap(a, g), snap(b, g), snap(p, g)),
                PathEl::QuadTo(a, p) => PathEl::QuadTo(snap(a, g), snap(p, g)),
                PathEl::ClosePath => PathEl::ClosePath,
            })
            .collect(),
    )
}

fn snap(p: Point, g: f64) -> Point {
    Point::new((p.x / g).round() * g, (p.y / g).round() * g)
}

// ── Extrema insertion ────────────────────────────────────

/// Insert on-curve points at H/V extrema of cubic curves.
///
/// Only inserts when the extremum extends beyond the
/// endpoints by at least `min_depth` font units.
fn insert_extrema(path: &BezPath, min_depth: f64) -> BezPath {
    let mut out = BezPath::new();
    let mut cur = Point::ZERO;

    for el in path.elements() {
        match *el {
            PathEl::MoveTo(p) => {
                out.move_to(p);
                cur = p;
            }
            PathEl::LineTo(p) => {
                out.line_to(p);
                cur = p;
            }
            PathEl::CurveTo(a, b, p) => {
                let c = CubicBez::new(cur, a, b, p);
                let ts = extrema_t_values(&c, min_depth);
                if ts.is_empty() {
                    out.push(PathEl::CurveTo(a, b, p));
                } else {
                    split_at_ts(&mut out, &c, &ts);
                }
                cur = p;
            }
            PathEl::QuadTo(a, p) => {
                out.push(PathEl::QuadTo(a, p));
                cur = p;
            }
            PathEl::ClosePath => {
                out.push(PathEl::ClosePath);
            }
        }
    }
    out
}

/// Find t-values of H/V extrema on a cubic.
fn extrema_t_values(c: &CubicBez, min_depth: f64) -> Vec<f64> {
    let mut ts = Vec::new();
    let margin = 0.01;

    for axis in 0..2 {
        let (v0, v1, v2, v3) = if axis == 0 {
            (c.p0.x, c.p1.x, c.p2.x, c.p3.x)
        } else {
            (c.p0.y, c.p1.y, c.p2.y, c.p3.y)
        };

        // Derivative: At² + Bt + C = 0
        let a = -3.0 * v0 + 9.0 * v1 - 9.0 * v2 + 3.0 * v3;
        let b = 6.0 * v0 - 12.0 * v1 + 6.0 * v2;
        let cc = -3.0 * v0 + 3.0 * v1;

        for t in solve_quadratic(a, b, cc) {
            if t <= margin || t >= 1.0 - margin {
                continue;
            }
            if extremum_depth(c, t, axis) >= min_depth {
                ts.push(t);
            }
        }
    }

    ts.sort_by(|a, b| a.partial_cmp(b).unwrap());
    ts.dedup_by(|a, b| (*a - *b).abs() < 0.001);
    ts
}

/// How far an extremum extends beyond the endpoints.
fn extremum_depth(c: &CubicBez, t: f64, axis: usize) -> f64 {
    let pt = c.eval(t);
    let vt = if axis == 0 { pt.x } else { pt.y };
    let v0 = if axis == 0 { c.p0.x } else { c.p0.y };
    let v3 = if axis == 0 { c.p3.x } else { c.p3.y };
    let (lo, hi) = min_max(v0, v3);

    if vt > hi {
        vt - hi
    } else if vt < lo {
        lo - vt
    } else {
        0.0
    }
}

fn min_max(a: f64, b: f64) -> (f64, f64) {
    if a < b {
        (a, b)
    } else {
        (b, a)
    }
}

/// Solve At² + Bt + C = 0, returning real roots.
fn solve_quadratic(a: f64, b: f64, c: f64) -> Vec<f64> {
    if a.abs() < 1e-10 {
        if b.abs() > 1e-10 {
            return vec![-c / b];
        }
        return vec![];
    }
    let disc = b * b - 4.0 * a * c;
    if disc < 0.0 {
        return vec![];
    }
    let sq = disc.sqrt();
    vec![(-b + sq) / (2.0 * a), (-b - sq) / (2.0 * a)]
}

/// Split a cubic at sorted t-values, appending to `out`.
fn split_at_ts(out: &mut BezPath, cubic: &CubicBez, ts: &[f64]) {
    let mut rest = *cubic;
    let mut prev_t = 0.0;

    for &t in ts {
        let local = (t - prev_t) / (1.0 - prev_t);
        let (left, right) = subdivide(rest, local);
        out.push(PathEl::CurveTo(left.p1, left.p2, left.p3));
        rest = right;
        prev_t = t;
    }
    out.push(PathEl::CurveTo(rest.p1, rest.p2, rest.p3));
}

/// De Casteljau subdivision at parameter t.
fn subdivide(c: CubicBez, t: f64) -> (CubicBez, CubicBez) {
    let ab = lerp(c.p0, c.p1, t);
    let bc = lerp(c.p1, c.p2, t);
    let cd = lerp(c.p2, c.p3, t);
    let abc = lerp(ab, bc, t);
    let bcd = lerp(bc, cd, t);
    let mid = lerp(abc, bcd, t);
    (
        CubicBez::new(c.p0, ab, abc, mid),
        CubicBez::new(mid, bcd, cd, c.p3),
    )
}

fn lerp(a: Point, b: Point, t: f64) -> Point {
    Point::new(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t)
}

// ── Cleanup ──────────────────────────────────────────────

/// Convert curves where both handles hug the chord.
fn curves_to_lines(path: &BezPath, tol: f64) -> BezPath {
    let mut out = BezPath::new();
    let mut cur = Point::ZERO;

    for el in path.elements() {
        match *el {
            PathEl::MoveTo(p) => {
                out.move_to(p);
                cur = p;
            }
            PathEl::CurveTo(a, b, p) => {
                let d1 = point_to_line_dist(a, cur, p);
                let d2 = point_to_line_dist(b, cur, p);
                if d1 < tol && d2 < tol {
                    out.line_to(p);
                } else {
                    out.push(PathEl::CurveTo(a, b, p));
                }
                cur = p;
            }
            PathEl::LineTo(p) => {
                out.line_to(p);
                cur = p;
            }
            other => out.push(other),
        }
    }
    out
}

/// Distance from point P to line through A→B.
fn point_to_line_dist(p: Point, a: Point, b: Point) -> f64 {
    let ab = Vec2::new(b.x - a.x, b.y - a.y);
    let ap = Vec2::new(p.x - a.x, p.y - a.y);
    let len_sq = ab.x * ab.x + ab.y * ab.y;
    if len_sq < 1e-10 {
        return ap.hypot();
    }
    (ab.x * ap.y - ab.y * ap.x).abs() / len_sq.sqrt()
}

/// Merge consecutive collinear line segments.
fn merge_collinear(path: &BezPath, tol: f64) -> BezPath {
    let mut out = BezPath::new();
    let mut anchor = Point::ZERO;
    let mut pending: Option<Point> = None;

    for el in path.elements() {
        match *el {
            PathEl::MoveTo(p) => {
                flush(&mut out, &mut pending);
                out.move_to(p);
                anchor = p;
            }
            PathEl::LineTo(p) => match pending {
                Some(mid) if point_to_line_dist(mid, anchor, p) < tol => {
                    pending = Some(p);
                }
                Some(mid) => {
                    out.line_to(mid);
                    anchor = mid;
                    pending = Some(p);
                }
                None => {
                    pending = Some(p);
                }
            },
            PathEl::CurveTo(a, b, p) => {
                flush(&mut out, &mut pending);
                out.push(PathEl::CurveTo(a, b, p));
                anchor = p;
            }
            PathEl::QuadTo(a, p) => {
                flush(&mut out, &mut pending);
                out.push(PathEl::QuadTo(a, p));
                anchor = p;
            }
            PathEl::ClosePath => {
                flush(&mut out, &mut pending);
                out.push(PathEl::ClosePath);
            }
        }
    }
    flush(&mut out, &mut pending);
    out
}

fn flush(out: &mut BezPath, pending: &mut Option<Point>) {
    if let Some(p) = pending.take() {
        out.line_to(p);
    }
}

/// Remove segments shorter than `tol`.
fn remove_tiny(path: &BezPath, tol: f64) -> BezPath {
    let mut out = BezPath::new();
    let mut cur = Point::ZERO;
    let tol_sq = tol * tol;

    for el in path.elements() {
        match *el {
            PathEl::MoveTo(p) => {
                out.move_to(p);
                cur = p;
            }
            PathEl::LineTo(p) => {
                if dist_sq(cur, p) >= tol_sq {
                    out.line_to(p);
                    cur = p;
                }
            }
            PathEl::CurveTo(a, b, p) => {
                if dist_sq(cur, p) >= tol_sq {
                    out.push(PathEl::CurveTo(a, b, p));
                    cur = p;
                }
            }
            PathEl::QuadTo(a, p) => {
                if dist_sq(cur, p) >= tol_sq {
                    out.push(PathEl::QuadTo(a, p));
                    cur = p;
                }
            }
            PathEl::ClosePath => {
                out.push(PathEl::ClosePath);
            }
        }
    }
    out
}

fn dist_sq(a: Point, b: Point) -> f64 {
    (a.x - b.x).powi(2) + (a.y - b.y).powi(2)
}

// ── Chamfers ─────────────────────────────────────────────

/// Add 45° chamfers at line-line corners.
fn chamfer(path: &BezPath, size: f64, min_edge: f64) -> BezPath {
    let mut first = Point::ZERO;
    let mut pts: Vec<(Point, bool)> = Vec::new();
    let mut closed = false;

    for el in path.elements() {
        match *el {
            PathEl::MoveTo(p) => first = p,
            PathEl::LineTo(p) => pts.push((p, true)),
            PathEl::CurveTo(_, _, p) | PathEl::QuadTo(_, p) => pts.push((p, false)),
            PathEl::ClosePath => closed = true,
        }
    }

    if pts.is_empty() {
        return path.clone();
    }

    let mut all = vec![(first, true)];
    all.extend_from_slice(&pts);
    let n = all.len();

    let mut new_pts: Vec<Point> = Vec::new();
    for i in 0..n {
        let (pt, is_line) = all[i];
        let (prev, _) = all[(i + n - 1) % n];
        let (next, next_line) = all[(i + 1) % n];

        if !is_line || !next_line {
            new_pts.push(pt);
            continue;
        }

        let v_in = Vec2::new(pt.x - prev.x, pt.y - prev.y);
        let v_out = Vec2::new(next.x - pt.x, next.y - pt.y);
        let len_in = v_in.hypot();
        let len_out = v_out.hypot();

        if len_in < min_edge || len_out < min_edge {
            new_pts.push(pt);
            continue;
        }

        let dot = v_in.x * v_out.x + v_in.y * v_out.y;
        let cos = dot / (len_in * len_out);
        if cos.abs() > 0.95 {
            new_pts.push(pt);
            continue;
        }

        new_pts.push(Point::new(
            pt.x - size * v_in.x / len_in,
            pt.y - size * v_in.y / len_in,
        ));
        new_pts.push(Point::new(
            pt.x + size * v_out.x / len_out,
            pt.y + size * v_out.y / len_out,
        ));
    }

    let mut out = BezPath::new();
    if let Some(&p) = new_pts.first() {
        out.move_to(p);
        for &p in &new_pts[1..] {
            out.line_to(p);
        }
        if closed {
            out.push(PathEl::ClosePath);
        }
    }
    out
}
