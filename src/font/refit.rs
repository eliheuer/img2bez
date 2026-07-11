// Copyright 2026 the img2bez Authors
// SPDX-License-Identifier: Apache-2.0 OR MIT

//! Refit existing bezier outlines onto a raster image: move points to match
//! the raster without changing point count or type, for interpolation-
//! compatible variable-font masters.
//!
//! **Experimental — not production-ready.** Gated behind `experimental-refit`.
//! Normal projection distorts some glyphs badly and has no regression tests;
//! the API may change or be removed.
//!
//! Algorithm: trace the raster to a target, match contours by centroid,
//! project each on-curve point along its outward normal onto the target
//! boundary, and move off-curves by a per-segment similarity transform.

use kurbo::{BezPath, PathEl, PathSeg, Point, Shape, Vec2};
use std::path::Path;

use crate::model::config::{FontMetrics, TraceOptions};
use crate::model::error::TraceError;

/// Flattening tolerance for target curves (font units).
const FLATTEN_TOLERANCE: f64 = 0.5;

/// Result of refitting existing outlines onto a raster image.
#[derive(Debug, Clone)]
pub struct RefitResult {
    /// Refitted paths — same structure as input, only positions changed.
    pub paths: Vec<BezPath>,
    /// Advance width computed from the traced image.
    pub advance_width: f64,
}

/// Refit existing bezier outlines onto a raster image, preserving point
/// count, type, and winding. `existing_paths` must be in placed-trace space
/// (font units, baseline at y=0). Experimental: see the [module docs](self).
///
/// # Errors
///
/// Returns any [`TraceError`] from tracing `image_path`.
pub fn refit(
    image_path: &Path,
    existing_paths: &[BezPath],
    opts: &TraceOptions,
    metrics: &FontMetrics,
) -> Result<RefitResult, TraceError> {
    let outline = crate::trace_file(image_path, opts)?;
    let target = crate::place(&outline, metrics);
    let target_paths = target.outline.to_bezpaths();

    let matches = match_contours(existing_paths, &target_paths);

    let paths: Vec<BezPath> = existing_paths
        .iter()
        .enumerate()
        .map(|(i, existing)| {
            if let Some(target_idx) = matches[i] {
                refit_path_normal(existing, &target_paths[target_idx])
            } else {
                existing.clone()
            }
        })
        .collect();

    Ok(RefitResult {
        paths,
        advance_width: target.advance_width,
    })
}

// ── Contour matching ─────────────────────────────────────────────────────

/// Match each existing contour to the closest target contour by centroid.
fn match_contours(
    existing: &[BezPath],
    targets: &[BezPath],
) -> Vec<Option<usize>> {
    if targets.is_empty() {
        return vec![None; existing.len()];
    }

    let target_centroids: Vec<Point> =
        targets.iter().map(path_centroid).collect();

    existing
        .iter()
        .map(|ep| {
            let ec = path_centroid(ep);
            target_centroids
                .iter()
                .enumerate()
                .min_by(|(_, a), (_, b)| {
                    let da = (ec - **a).hypot2();
                    let db = (ec - **b).hypot2();
                    da.partial_cmp(&db).unwrap_or(std::cmp::Ordering::Equal)
                })
                .map(|(idx, _)| idx)
        })
        .collect()
}

/// Centroid of a BezPath (average of on-curve points).
fn path_centroid(path: &BezPath) -> Point {
    let mut sum = Vec2::ZERO;
    let mut count = 0;
    for el in path.elements() {
        let p = match *el {
            PathEl::MoveTo(p)
            | PathEl::LineTo(p)
            | PathEl::CurveTo(_, _, p)
            | PathEl::QuadTo(_, p) => p,
            PathEl::ClosePath => continue,
        };
        sum += p.to_vec2();
        count += 1;
    }
    if count == 0 {
        Point::ZERO
    } else {
        (sum / count as f64).to_point()
    }
}

// ── Normal-based refitting ───────────────────────────────────────────────

/// Refit a single path: ray-cast each on-curve point along its outward
/// normal (both directions, closer hit wins; nearest-point fallback), then
/// move off-curves by a per-segment similarity transform.
fn refit_path_normal(existing: &BezPath, target: &BezPath) -> BezPath {
    let elements = existing.elements();
    if elements.is_empty() {
        return BezPath::new();
    }

    let existing_segs: Vec<PathSeg> = existing.segments().collect();
    if existing_segs.is_empty() {
        return existing.clone();
    }

    let target_polyline = flatten_to_segments(target, FLATTEN_TOLERANCE);
    if target_polyline.is_empty() {
        return existing.clone();
    }

    let n_segs = existing_segs.len();
    let is_closed = elements.iter().any(|el| matches!(el, PathEl::ClosePath));
    let is_ccw = existing.area() >= 0.0;

    let mut on_curve_old: Vec<Point> = Vec::new();
    let mut on_curve_normals: Vec<Vec2> = Vec::new();

    for el in elements {
        let point = match *el {
            PathEl::MoveTo(p)
            | PathEl::LineTo(p)
            | PathEl::CurveTo(_, _, p)
            | PathEl::QuadTo(_, p) => p,
            PathEl::ClosePath => continue,
        };

        let oc_idx = on_curve_old.len();
        on_curve_old.push(point);

        let normal = compute_outward_normal(
            oc_idx,
            n_segs,
            is_closed,
            is_ccw,
            &existing_segs,
        );
        on_curve_normals.push(normal);
    }

    let on_curve_new: Vec<Point> = on_curve_old
        .iter()
        .zip(on_curve_normals.iter())
        .map(|(&point, &normal)| {
            project_onto_boundary(point, normal, &target_polyline)
        })
        .collect();

    let on_curve_map: Vec<(Point, Point)> =
        on_curve_old.into_iter().zip(on_curve_new).collect();

    rebuild_path(elements, &on_curve_map)
}

/// Outward normal at on-curve point `oc_idx`: perpendicular to the average
/// of the incoming and outgoing tangents.
fn compute_outward_normal(
    oc_idx: usize,
    n_segs: usize,
    is_closed: bool,
    is_ccw: bool,
    segments: &[PathSeg],
) -> Vec2 {
    let (in_tang, out_tang) = if is_closed {
        let in_seg = &segments[(oc_idx + n_segs - 1) % n_segs];
        let out_seg = &segments[oc_idx % n_segs];
        (segment_end_tangent(in_seg), segment_start_tangent(out_seg))
    } else if oc_idx == 0 {
        let t = segment_start_tangent(&segments[0]);
        (t, t)
    } else if oc_idx >= n_segs {
        let t = segment_end_tangent(&segments[n_segs - 1]);
        (t, t)
    } else {
        (
            segment_end_tangent(&segments[oc_idx - 1]),
            segment_start_tangent(&segments[oc_idx]),
        )
    };

    // Normalize before averaging so corners get a true bisector.
    let in_len = in_tang.hypot();
    let out_len = out_tang.hypot();

    let in_norm = if in_len > 1e-10 {
        in_tang / in_len
    } else {
        in_tang
    };
    let out_norm = if out_len > 1e-10 {
        out_tang / out_len
    } else {
        out_tang
    };

    let avg_tangent = in_norm + out_norm;

    // Y-up coords: CCW interior is left of the tangent, so outward is a
    // 90° CW rotation (y, -x); CW winding is the opposite.
    let normal = if is_ccw {
        Vec2::new(avg_tangent.y, -avg_tangent.x)
    } else {
        Vec2::new(-avg_tangent.y, avg_tangent.x)
    };

    let len = normal.hypot();
    if len > 1e-10 {
        normal / len
    } else {
        // Degenerate (e.g. cusp) — fall back to incoming normal.
        let fallback = if is_ccw {
            Vec2::new(in_norm.y, -in_norm.x)
        } else {
            Vec2::new(-in_norm.y, in_norm.x)
        };
        let flen = fallback.hypot();
        if flen > 1e-10 {
            fallback / flen
        } else {
            Vec2::ZERO
        }
    }
}

/// Project a point onto the target boundary by ray-casting along its normal
/// (both directions, closer hit wins).
fn project_onto_boundary(
    point: Point,
    normal: Vec2,
    target_polyline: &[(Point, Point)],
) -> Point {
    if normal.hypot2() < 1e-10 {
        // Zero normal — fall back to nearest point.
        return nearest_on_polyline(point, target_polyline);
    }

    let hit_fwd = raycast(target_polyline, point, normal);
    let hit_rev = raycast(target_polyline, point, -normal);

    match (hit_fwd, hit_rev) {
        (Some(fwd), Some(rev)) => {
            let d_fwd = (fwd - point).hypot2();
            let d_rev = (rev - point).hypot2();
            if d_fwd <= d_rev { fwd } else { rev }
        }
        (Some(fwd), None) => fwd,
        (None, Some(rev)) => rev,
        (None, None) => {
            // No ray hit — fall back to nearest point.
            nearest_on_polyline(point, target_polyline)
        }
    }
}

// ── Path rebuilding ──────────────────────────────────────────────────────

/// Rebuild a BezPath with new on-curve positions; off-curves move by the
/// per-segment similarity transform.
fn rebuild_path(
    elements: &[PathEl],
    on_curve_map: &[(Point, Point)],
) -> BezPath {
    let mut result = BezPath::new();
    let mut oc_idx: usize = 0;

    for el in elements {
        match *el {
            PathEl::MoveTo(_) => {
                result.move_to(on_curve_map[oc_idx].1);
                oc_idx += 1;
            }
            PathEl::LineTo(_) => {
                result.line_to(on_curve_map[oc_idx].1);
                oc_idx += 1;
            }
            PathEl::CurveTo(cp1, cp2, _) => {
                let (start_old, start_new) = on_curve_map[oc_idx - 1];
                let (end_old, end_new) = on_curve_map[oc_idx];

                let cp1_new = transform_control_point(
                    cp1, start_old, end_old, start_new, end_new,
                );
                let cp2_new = transform_control_point(
                    cp2, start_old, end_old, start_new, end_new,
                );

                result.curve_to(cp1_new, cp2_new, end_new);
                oc_idx += 1;
            }
            PathEl::QuadTo(cp, _) => {
                let (start_old, start_new) = on_curve_map[oc_idx - 1];
                let (end_old, end_new) = on_curve_map[oc_idx];

                let cp_new = transform_control_point(
                    cp, start_old, end_old, start_new, end_new,
                );

                result.quad_to(cp_new, end_new);
                oc_idx += 1;
            }
            PathEl::ClosePath => {
                result.close_path();
            }
        }
    }

    result
}

// ── Ray casting ──────────────────────────────────────────────────────────

/// Closest intersection of the ray from `origin` with the polyline.
fn raycast(
    polyline: &[(Point, Point)],
    origin: Point,
    direction: Vec2,
) -> Option<Point> {
    let mut closest_t = f64::INFINITY;
    let mut closest_point = None;

    for &(a, b) in polyline {
        if let Some((t, _s)) = ray_segment_intersection(origin, direction, a, b)
            && t > 1e-6
            && t < closest_t
        {
            closest_t = t;
            closest_point = Some(origin + t * direction);
        }
    }

    closest_point
}

/// 2D ray–segment intersection: ray `origin + t*direction` (`t >= 0`) vs
/// segment `a + s*(b - a)` (`s` in `0..=1`); `Some((t, s))` on hit.
fn ray_segment_intersection(
    origin: Point,
    direction: Vec2,
    a: Point,
    b: Point,
) -> Option<(f64, f64)> {
    let u = b - a;
    let v = origin - a;

    let det = direction.x * u.y - direction.y * u.x;
    if det.abs() < 1e-10 {
        return None; // parallel
    }

    // Cramer's rule
    let s = (direction.x * v.y - direction.y * v.x) / det;
    let t = (u.x * v.y - u.y * v.x) / det;

    if t >= 0.0 && (0.0..=1.0).contains(&s) {
        Some((t, s))
    } else {
        None
    }
}

/// Find the nearest point on any polyline segment to `point`.
fn nearest_on_polyline(point: Point, polyline: &[(Point, Point)]) -> Point {
    let mut best = point;
    let mut best_dist_sq = f64::INFINITY;

    for &(a, b) in polyline {
        let ab = b - a;
        let len_sq = ab.hypot2();
        let t = if len_sq < 1e-10 {
            0.0
        } else {
            ((point - a).dot(ab) / len_sq).clamp(0.0, 1.0)
        };
        let nearest = a + t * ab;
        let dist_sq = (point - nearest).hypot2();
        if dist_sq < best_dist_sq {
            best_dist_sq = dist_sq;
            best = nearest;
        }
    }

    best
}

// ── Polyline flattening ──────────────────────────────────────────────────

/// Flatten a `BezPath` to line segments within `tolerance`.
fn flatten_to_segments(path: &BezPath, tolerance: f64) -> Vec<(Point, Point)> {
    let mut segments = Vec::new();
    let mut current = Point::ZERO;
    let mut start = Point::ZERO;

    kurbo::flatten(path.iter(), tolerance, |el| match el {
        PathEl::MoveTo(p) => {
            start = p;
            current = p;
        }
        PathEl::LineTo(p) => {
            segments.push((current, p));
            current = p;
        }
        PathEl::ClosePath if (current - start).hypot() > 1e-10 => {
            segments.push((current, start));
        }
        _ => {}
    });

    segments
}

// ── Tangent computation ──────────────────────────────────────────────────

/// Tangent direction at the start of a segment (t = 0).
fn segment_start_tangent(seg: &PathSeg) -> Vec2 {
    match *seg {
        PathSeg::Line(l) => l.p1 - l.p0,
        PathSeg::Quad(q) => {
            let d = q.p1 - q.p0;
            if d.hypot2() > 1e-10 { d } else { q.p2 - q.p0 }
        }
        PathSeg::Cubic(c) => {
            let d = c.p1 - c.p0;
            if d.hypot2() > 1e-10 {
                d
            } else {
                let d = c.p2 - c.p0;
                if d.hypot2() > 1e-10 { d } else { c.p3 - c.p0 }
            }
        }
    }
}

/// Tangent direction at the end of a segment (t = 1).
fn segment_end_tangent(seg: &PathSeg) -> Vec2 {
    match *seg {
        PathSeg::Line(l) => l.p1 - l.p0,
        PathSeg::Quad(q) => {
            let d = q.p2 - q.p1;
            if d.hypot2() > 1e-10 { d } else { q.p2 - q.p0 }
        }
        PathSeg::Cubic(c) => {
            let d = c.p3 - c.p2;
            if d.hypot2() > 1e-10 {
                d
            } else {
                let d = c.p3 - c.p1;
                if d.hypot2() > 1e-10 { d } else { c.p3 - c.p0 }
            }
        }
    }
}

// ── Control-point transform ──────────────────────────────────────────────

/// Map a control point by the similarity transform from the old segment
/// frame to the new one: handles scale and rotate with the segment.
fn transform_control_point(
    cp: Point,
    start_old: Point,
    end_old: Point,
    start_new: Point,
    end_new: Point,
) -> Point {
    let u_old: Vec2 = end_old - start_old;
    let len_sq_old = u_old.hypot2();

    // Degenerate case: old endpoints coincide.
    if len_sq_old < 1e-10 {
        let delta = start_new - start_old;
        return cp + delta;
    }

    let v_old = Vec2::new(-u_old.y, u_old.x);

    let rel: Vec2 = cp - start_old;
    let a = rel.dot(u_old) / len_sq_old;
    let b = rel.dot(v_old) / len_sq_old;

    let u_new: Vec2 = end_new - start_new;
    let v_new = Vec2::new(-u_new.y, u_new.x);

    start_new + a * u_new + b * v_new
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn transform_preserves_midpoint() {
        let cp_new = transform_control_point(
            Point::new(50.0, 0.0),
            Point::new(0.0, 0.0),
            Point::new(100.0, 0.0),
            Point::new(0.0, 0.0),
            Point::new(200.0, 0.0),
        );
        assert!((cp_new.x - 100.0).abs() < 1e-10);
        assert!(cp_new.y.abs() < 1e-10);
    }

    #[test]
    fn transform_preserves_perpendicular_offset() {
        let cp_new = transform_control_point(
            Point::new(50.0, 30.0),
            Point::new(0.0, 0.0),
            Point::new(100.0, 0.0),
            Point::new(10.0, 10.0),
            Point::new(210.0, 10.0),
        );
        assert!((cp_new.x - 110.0).abs() < 1e-10);
        assert!((cp_new.y - 70.0).abs() < 1e-10);
    }

    #[test]
    fn transform_degenerate_segment() {
        let cp_new = transform_control_point(
            Point::new(60.0, 70.0),
            Point::new(50.0, 50.0),
            Point::new(50.0, 50.0),
            Point::new(100.0, 100.0),
            Point::new(100.0, 100.0),
        );
        assert!((cp_new.x - 110.0).abs() < 1e-10);
        assert!((cp_new.y - 120.0).abs() < 1e-10);
    }

    #[test]
    fn ray_hits_vertical_segment() {
        // Ray going right from origin, segment is vertical at x=50.
        let hit = ray_segment_intersection(
            Point::new(0.0, 0.0),
            Vec2::new(1.0, 0.0),
            Point::new(50.0, -10.0),
            Point::new(50.0, 10.0),
        );
        assert!(hit.is_some());
        let (t, s) = hit.unwrap();
        assert!((t - 50.0).abs() < 1e-10);
        assert!((s - 0.5).abs() < 1e-10);
    }

    #[test]
    fn ray_misses_behind() {
        // Ray going right, segment is behind at x=-50.
        let hit = ray_segment_intersection(
            Point::new(0.0, 0.0),
            Vec2::new(1.0, 0.0),
            Point::new(-50.0, -10.0),
            Point::new(-50.0, 10.0),
        );
        assert!(hit.is_none());
    }

    #[test]
    fn nearest_on_polyline_correct() {
        let poly = vec![
            (Point::new(0.0, 0.0), Point::new(100.0, 0.0)),
            (Point::new(100.0, 0.0), Point::new(100.0, 100.0)),
        ];
        let nearest = nearest_on_polyline(Point::new(50.0, 10.0), &poly);
        // Closest to the bottom edge at (50, 0)
        assert!((nearest.x - 50.0).abs() < 1e-10);
        assert!(nearest.y.abs() < 1e-10);
    }

    #[test]
    fn outward_normal_ccw_square() {
        // CCW square: (0,0) → (100,0) → (100,100) → (0,100)
        let mut path = BezPath::new();
        path.move_to(Point::new(0.0, 0.0));
        path.line_to(Point::new(100.0, 0.0));
        path.line_to(Point::new(100.0, 100.0));
        path.line_to(Point::new(0.0, 100.0));
        path.close_path();

        let segs: Vec<PathSeg> = path.segments().collect();
        let n_segs = segs.len();
        let is_ccw = path.area() >= 0.0;

        // Outward at the bottom-left corner of a CCW square points down.
        let n0 = compute_outward_normal(0, n_segs, true, is_ccw, &segs);
        assert!(
            n0.y < -0.1,
            "expected outward normal to point down at (0,0), got {:?}",
            n0
        );
    }

    #[test]
    fn flatten_produces_segments() {
        let mut path = BezPath::new();
        path.move_to(Point::new(0.0, 0.0));
        path.line_to(Point::new(100.0, 0.0));
        path.line_to(Point::new(100.0, 100.0));
        path.close_path();

        let segs = flatten_to_segments(&path, 1.0);
        assert!(
            segs.len() >= 3,
            "expected at least 3 segments, got {}",
            segs.len()
        );
    }
}
