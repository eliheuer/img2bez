//! Refit existing bezier outlines onto a raster image.
//!
//! Given existing outlines (e.g. from a regular-weight master) and a
//! raster image of a different weight, this module moves the existing
//! points to match the raster shape without changing the number or
//! type of points. This produces interpolation-compatible outlines
//! for variable font workflows.
//!
//! # Algorithm
//!
//! 1. Trace the raster image using the normal img2bez pipeline to get
//!    a "target" set of bezier contours.
//! 2. Match each existing contour to the closest target contour (by
//!    centroid proximity).
//! 3. For each on-curve point, compute the contour's outward normal
//!    at that point, then ray-cast along the normal to find the
//!    intersection with the target boundary. This preserves the
//!    geometric role of each point (a point on the left side stays on
//!    the left side, etc.).
//! 4. Off-curve control points are adjusted via a similarity
//!    transform that preserves their position relative to adjacent
//!    on-curve endpoints.

use kurbo::{BezPath, PathEl, PathSeg, Point, Shape, Vec2};
use std::path::Path;

use crate::config::TracingConfig;
use crate::error::TraceError;

/// Tolerance for flattening target bezier curves into polylines
/// (font units). Lower = more accurate but more segments.
const FLATTEN_TOLERANCE: f64 = 0.5;

/// Result of refitting existing outlines onto a raster image.
#[derive(Debug, Clone)]
pub struct RefitResult {
    /// Refitted paths — same structure as input, only positions changed.
    pub paths: Vec<BezPath>,
    /// Advance width computed from the traced image.
    pub advance_width: f64,
}

/// Refit existing bezier outlines onto a raster image.
///
/// Traces the image to get a target shape, matches each existing
/// contour to the closest target contour, then projects each
/// on-curve point along its contour normal onto the target boundary.
/// The number, type, and winding direction of points are preserved.
///
/// `existing_paths` must be in the same coordinate space as the trace
/// output (font units, repositioned to baseline). The caller is
/// responsible for any coordinate alignment before and after.
pub fn refit(
    image_path: &Path,
    existing_paths: &[BezPath],
    config: &TracingConfig,
) -> Result<RefitResult, TraceError> {
    let target = crate::trace(image_path, config)?;

    // Match each existing contour to its closest target contour.
    let matches = match_contours(existing_paths, &target.paths);

    let paths: Vec<BezPath> = existing_paths
        .iter()
        .enumerate()
        .map(|(i, existing)| {
            if let Some(target_idx) = matches[i] {
                refit_path_normal(existing, &target.paths[target_idx])
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

// ============================================================================
// CONTOUR MATCHING
// ============================================================================

/// Match each existing contour to the closest target contour by
/// centroid proximity.
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
                    da.partial_cmp(&db)
                        .unwrap_or(std::cmp::Ordering::Equal)
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

// ============================================================================
// NORMAL-BASED REFITTING
// ============================================================================

/// Refit a single path by projecting each on-curve point along its
/// contour normal onto the target boundary.
///
/// For each on-curve point:
/// 1. Compute the outward normal from the adjacent segment tangents.
/// 2. Ray-cast in both the normal and reverse-normal directions
///    against the flattened target polyline.
/// 3. Pick the closer hit (handles both expansion and contraction).
/// 4. If no ray hit, fall back to nearest point on the target.
///
/// Off-curve control points are adjusted via a similarity transform
/// that preserves their position relative to the segment endpoints.
fn refit_path_normal(
    existing: &BezPath,
    target: &BezPath,
) -> BezPath {
    let elements = existing.elements();
    if elements.is_empty() {
        return BezPath::new();
    }

    let existing_segs: Vec<PathSeg> = existing.segments().collect();
    if existing_segs.is_empty() {
        return existing.clone();
    }

    // Flatten target to a polyline for ray intersection.
    let target_polyline = flatten_to_segments(target, FLATTEN_TOLERANCE);
    if target_polyline.is_empty() {
        return existing.clone();
    }

    let n_segs = existing_segs.len();
    let is_closed = elements
        .iter()
        .any(|el| matches!(el, PathEl::ClosePath));

    // Determine winding direction for outward normal computation.
    let is_ccw = existing.area() >= 0.0;

    // Collect on-curve points and compute their outward normals.
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

        // Compute tangent from adjacent segments.
        let normal = compute_outward_normal(
            oc_idx,
            n_segs,
            is_closed,
            is_ccw,
            &existing_segs,
        );
        on_curve_normals.push(normal);
    }

    // Project each on-curve point onto the target boundary.
    let on_curve_new: Vec<Point> = on_curve_old
        .iter()
        .zip(on_curve_normals.iter())
        .map(|(&point, &normal)| {
            project_onto_boundary(point, normal, &target_polyline)
        })
        .collect();

    let on_curve_map: Vec<(Point, Point)> = on_curve_old
        .into_iter()
        .zip(on_curve_new)
        .collect();

    // Rebuild path with new positions.
    rebuild_path(elements, &on_curve_map)
}

/// Compute the outward normal at on-curve point `oc_idx`.
///
/// The normal is perpendicular to the average of the incoming and
/// outgoing tangent directions, pointing outward from the contour.
fn compute_outward_normal(
    oc_idx: usize,
    n_segs: usize,
    is_closed: bool,
    is_ccw: bool,
    segments: &[PathSeg],
) -> Vec2 {
    let (in_tang, out_tang) = if is_closed {
        let in_seg =
            &segments[(oc_idx + n_segs - 1) % n_segs];
        let out_seg = &segments[oc_idx % n_segs];
        (
            segment_end_tangent(in_seg),
            segment_start_tangent(out_seg),
        )
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

    // Normalize tangents before averaging so corners get a proper
    // bisector direction rather than being biased by segment length.
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

    // Rotate 90° to get outward normal.
    // In Y-up (font) coordinates, CCW interior is to the LEFT of
    // the tangent, so outward = RIGHT = rotate 90° CW = (y, -x).
    // CW interior is to the RIGHT, so outward = LEFT = (-y, x).
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

/// Project a point onto the target boundary by ray-casting along
/// its normal. Tries both directions and picks the closer hit.
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

// ============================================================================
// PATH REBUILDING
// ============================================================================

/// Rebuild a BezPath using new on-curve positions, transforming
/// off-curve control points via similarity transform.
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
                let (start_old, start_new) =
                    on_curve_map[oc_idx - 1];
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
                let (start_old, start_new) =
                    on_curve_map[oc_idx - 1];
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

// ============================================================================
// RAY-CASTING
// ============================================================================

/// Cast a ray from `origin` in `direction` and find the closest
/// intersection with the polyline segments.
fn raycast(
    polyline: &[(Point, Point)],
    origin: Point,
    direction: Vec2,
) -> Option<Point> {
    let mut closest_t = f64::INFINITY;
    let mut closest_point = None;

    for &(a, b) in polyline {
        if let Some((t, _s)) =
            ray_segment_intersection(origin, direction, a, b)
        {
            if t > 1e-6 && t < closest_t {
                closest_t = t;
                closest_point = Some(origin + t * direction);
            }
        }
    }

    closest_point
}

/// 2D ray–line-segment intersection.
///
/// Ray: origin + t * direction  (t >= 0)
/// Segment: a + s * (b - a)     (s ∈ [0, 1])
///
/// Returns (t, s) if the ray hits the segment, None otherwise.
fn ray_segment_intersection(
    origin: Point,
    direction: Vec2,
    a: Point,
    b: Point,
) -> Option<(f64, f64)> {
    let u = b - a;
    let v = origin - a;

    // 2D cross product: direction × u
    let det = direction.x * u.y - direction.y * u.x;

    if det.abs() < 1e-10 {
        return None; // parallel
    }

    // Cramer's rule
    let s = (direction.x * v.y - direction.y * v.x) / det;
    let t = (u.x * v.y - u.y * v.x) / det;

    if t >= 0.0 && s >= 0.0 && s <= 1.0 {
        Some((t, s))
    } else {
        None
    }
}

/// Find the nearest point on any polyline segment to `point`.
fn nearest_on_polyline(
    point: Point,
    polyline: &[(Point, Point)],
) -> Point {
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

// ============================================================================
// POLYLINE FLATTENING
// ============================================================================

/// Flatten a BezPath to a list of line segments (polyline).
///
/// Uses kurbo's adaptive flattening to convert bezier curves into
/// line segments within the specified tolerance.
fn flatten_to_segments(
    path: &BezPath,
    tolerance: f64,
) -> Vec<(Point, Point)> {
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
        PathEl::ClosePath => {
            if (current - start).hypot() > 1e-10 {
                segments.push((current, start));
            }
        }
        _ => {}
    });

    segments
}

// ============================================================================
// TANGENT COMPUTATION
// ============================================================================

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
                if d.hypot2() > 1e-10 {
                    d
                } else {
                    c.p3 - c.p0
                }
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
                if d.hypot2() > 1e-10 {
                    d
                } else {
                    c.p3 - c.p0
                }
            }
        }
    }
}

// ============================================================================
// CONTROL POINT TRANSFORM
// ============================================================================

/// Transform a control point using the similarity transform that maps
/// the old segment frame (start_old→end_old) to the new segment
/// frame (start_new→end_new).
///
/// The control point's position relative to the segment (expressed in
/// the segment's local coordinate system) is preserved. If the
/// segment gets longer the handles scale proportionally; if it
/// rotates the handles rotate with it.
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

        // Point 0 at (0,0): between seg[3] (going left) and
        // seg[0] (going right). Normal should point downward
        // (outward from the square for Y-up coords).
        let n0 = compute_outward_normal(0, n_segs, true, is_ccw, &segs);
        // For CCW in standard math coords, outward at bottom-left
        // corner should have negative y component.
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
