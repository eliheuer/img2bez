use kurbo::{BezPath, CubicBez, PathEl, Point, Vec2};

use crate::config::TracingConfig;

/// Apply all post-processing steps to a set of traced contours.
pub fn process(paths: &[BezPath], config: &TracingConfig) -> Vec<BezPath> {
    let mut result = paths.to_vec();

    if config.add_extrema {
        result = result.iter().map(|p| insert_extrema(p)).collect();
    }

    if config.grid > 0 {
        result = result.iter().map(|p| snap_to_grid(p, config.grid)).collect();
    }

    if config.fix_direction {
        result = fix_contour_directions(&result);
    }

    if config.chamfer_size > 0.0 {
        result = result
            .iter()
            .map(|p| add_chamfers(p, config.chamfer_size, config.chamfer_min_edge))
            .collect();
    }

    result
}

// ---------------------------------------------------------------------------
// Grid snapping
// ---------------------------------------------------------------------------

fn snap_to_grid(path: &BezPath, grid: i32) -> BezPath {
    let g = grid as f64;
    BezPath::from_vec(
        path.elements()
            .iter()
            .map(|el| match *el {
                PathEl::MoveTo(p) => PathEl::MoveTo(snap_pt(p, g)),
                PathEl::LineTo(p) => PathEl::LineTo(snap_pt(p, g)),
                PathEl::CurveTo(p1, p2, p3) => {
                    PathEl::CurveTo(snap_pt(p1, g), snap_pt(p2, g), snap_pt(p3, g))
                }
                PathEl::QuadTo(p1, p2) => PathEl::QuadTo(snap_pt(p1, g), snap_pt(p2, g)),
                PathEl::ClosePath => PathEl::ClosePath,
            })
            .collect(),
    )
}

fn snap_pt(p: Point, grid: f64) -> Point {
    Point::new((p.x / grid).round() * grid, (p.y / grid).round() * grid)
}

// ---------------------------------------------------------------------------
// Contour direction
// ---------------------------------------------------------------------------

/// Signed area of a BezPath (using on-curve points only for speed).
/// Positive = CCW, negative = CW.
fn path_signed_area(path: &BezPath) -> f64 {
    let mut area = 0.0;
    let mut first = Point::ZERO;
    let mut current = Point::ZERO;

    for el in path.elements() {
        match *el {
            PathEl::MoveTo(p) => {
                first = p;
                current = p;
            }
            PathEl::LineTo(p) => {
                area += current.x * p.y - p.x * current.y;
                current = p;
            }
            PathEl::CurveTo(_, _, p) => {
                area += current.x * p.y - p.x * current.y;
                current = p;
            }
            PathEl::QuadTo(_, p) => {
                area += current.x * p.y - p.x * current.y;
                current = p;
            }
            PathEl::ClosePath => {
                area += current.x * first.y - first.x * current.y;
            }
        }
    }
    area / 2.0
}

fn fix_contour_directions(paths: &[BezPath]) -> Vec<BezPath> {
    if paths.is_empty() {
        return vec![];
    }

    // Find the contour with the largest absolute area — that's the outer.
    let areas: Vec<f64> = paths.iter().map(|p| path_signed_area(p)).collect();
    let outer_idx = areas
        .iter()
        .enumerate()
        .max_by(|(_, a), (_, b)| a.abs().partial_cmp(&b.abs()).unwrap())
        .map(|(i, _)| i)
        .unwrap_or(0);

    paths
        .iter()
        .enumerate()
        .map(|(i, path)| {
            let area = areas[i];
            let is_outer = i == outer_idx;

            // Outer should be CCW (positive area), counters CW (negative)
            if is_outer && area < 0.0 {
                reverse_bezpath(path)
            } else if !is_outer && area > 0.0 {
                reverse_bezpath(path)
            } else {
                path.clone()
            }
        })
        .collect()
}

fn reverse_bezpath(path: &BezPath) -> BezPath {
    // Collect on-curve points and segment types in reverse
    let elements = path.elements();

    // Extract all points from the path
    let mut segments: Vec<PathEl> = Vec::new();
    let mut first_pt = Point::ZERO;

    for el in elements {
        match *el {
            PathEl::MoveTo(p) => {
                first_pt = p;
            }
            other => segments.push(other),
        }
    }

    // Remove trailing ClosePath if present
    let had_close = matches!(segments.last(), Some(PathEl::ClosePath));
    if had_close {
        segments.pop();
    }

    // Reverse the segments and swap control point order
    let mut reversed = BezPath::new();

    // The new start is the last on-curve point
    let last_oncurve = segments
        .iter()
        .rev()
        .find_map(|el| match *el {
            PathEl::LineTo(p) | PathEl::CurveTo(_, _, p) | PathEl::QuadTo(_, p) => Some(p),
            _ => None,
        })
        .unwrap_or(first_pt);

    reversed.move_to(last_oncurve);

    for seg in segments.iter().rev() {
        match *seg {
            PathEl::LineTo(_) => {
                // The target of this reversed segment is the previous segment's endpoint
                // (or first_pt for the first segment)
            }
            _ => {}
        }
    }

    // Simpler approach: flatten to points, reverse, rebuild
    // This works for line-only and mixed paths
    let mut points_and_types: Vec<(Point, SegType)> = Vec::new();

    for seg in &segments {
        match *seg {
            PathEl::LineTo(p) => {
                points_and_types.push((p, SegType::Line));
            }
            PathEl::CurveTo(c1, c2, p) => {
                points_and_types.push((p, SegType::Curve(c1, c2)));
            }
            PathEl::QuadTo(c1, p) => {
                points_and_types.push((p, SegType::Quad(c1)));
            }
            _ => {}
        }
    }

    // Build reversed path
    let mut rev = BezPath::new();
    if points_and_types.is_empty() {
        rev.move_to(first_pt);
        if had_close {
            rev.push(PathEl::ClosePath);
        }
        return rev;
    }

    // Start at the last point
    let last = points_and_types.last().unwrap().0;
    rev.move_to(last);

    // Walk backwards
    for i in (0..points_and_types.len()).rev() {
        let target = if i == 0 {
            first_pt
        } else {
            points_and_types[i - 1].0
        };

        match points_and_types[i].1 {
            SegType::Line => {
                rev.line_to(target);
            }
            SegType::Curve(c1, c2) => {
                // Reverse control point order
                rev.push(PathEl::CurveTo(c2, c1, target));
            }
            SegType::Quad(c1) => {
                rev.push(PathEl::QuadTo(c1, target));
            }
        }
    }

    if had_close {
        rev.push(PathEl::ClosePath);
    }

    rev
}

#[derive(Clone, Copy)]
enum SegType {
    Line,
    Curve(Point, Point),
    Quad(Point),
}

// ---------------------------------------------------------------------------
// Extrema insertion
// ---------------------------------------------------------------------------

fn insert_extrema(path: &BezPath) -> BezPath {
    let mut result = BezPath::new();
    let mut current = Point::ZERO;

    for el in path.elements() {
        match *el {
            PathEl::MoveTo(p) => {
                result.move_to(p);
                current = p;
            }
            PathEl::LineTo(p) => {
                result.line_to(p);
                current = p;
            }
            PathEl::CurveTo(p1, p2, p3) => {
                let cubic = CubicBez::new(current, p1, p2, p3);
                let extrema_t = find_extrema_t(&cubic);

                if extrema_t.is_empty() {
                    result.push(PathEl::CurveTo(p1, p2, p3));
                } else {
                    // Split at all extrema t values
                    let mut remaining = cubic;
                    let mut prev_t = 0.0;

                    for &t in &extrema_t {
                        // Remap t to the remaining portion
                        let local_t = (t - prev_t) / (1.0 - prev_t);
                        let (left, right) = subdivide_cubic(&remaining, local_t);

                        result.push(PathEl::CurveTo(left.p1, left.p2, left.p3));
                        remaining = right;
                        prev_t = t;
                    }
                    result.push(PathEl::CurveTo(remaining.p1, remaining.p2, remaining.p3));
                }
                current = p3;
            }
            PathEl::QuadTo(p1, p2) => {
                result.push(PathEl::QuadTo(p1, p2));
                current = p2;
            }
            PathEl::ClosePath => {
                result.push(PathEl::ClosePath);
            }
        }
    }
    result
}

/// Find t values where a cubic bezier has horizontal or vertical extrema.
fn find_extrema_t(cubic: &CubicBez) -> Vec<f64> {
    let mut ts = Vec::new();
    let threshold = 0.01;

    for axis in 0..2 {
        let (a, b, c, d) = if axis == 0 {
            (cubic.p0.x, cubic.p1.x, cubic.p2.x, cubic.p3.x)
        } else {
            (cubic.p0.y, cubic.p1.y, cubic.p2.y, cubic.p3.y)
        };

        // Derivative: At^2 + Bt + C = 0
        let da = -3.0 * a + 9.0 * b - 9.0 * c + 3.0 * d;
        let db = 6.0 * a - 12.0 * b + 6.0 * c;
        let dc = -3.0 * a + 3.0 * b;

        if da.abs() < 1e-10 {
            if db.abs() > 1e-10 {
                let t = -dc / db;
                if t > threshold && t < 1.0 - threshold {
                    ts.push(t);
                }
            }
        } else {
            let disc = db * db - 4.0 * da * dc;
            if disc >= 0.0 {
                let sq = disc.sqrt();
                for t in [(-db + sq) / (2.0 * da), (-db - sq) / (2.0 * da)] {
                    if t > threshold && t < 1.0 - threshold {
                        ts.push(t);
                    }
                }
            }
        }
    }

    ts.sort_by(|a, b| a.partial_cmp(b).unwrap());
    ts.dedup_by(|a, b| (*a - *b).abs() < 0.001);
    ts
}

fn subdivide_cubic(cubic: &CubicBez, t: f64) -> (CubicBez, CubicBez) {
    // De Casteljau subdivision
    let p01 = lerp(cubic.p0, cubic.p1, t);
    let p12 = lerp(cubic.p1, cubic.p2, t);
    let p23 = lerp(cubic.p2, cubic.p3, t);
    let p012 = lerp(p01, p12, t);
    let p123 = lerp(p12, p23, t);
    let p0123 = lerp(p012, p123, t);

    (
        CubicBez::new(cubic.p0, p01, p012, p0123),
        CubicBez::new(p0123, p123, p23, cubic.p3),
    )
}

fn lerp(a: Point, b: Point, t: f64) -> Point {
    Point::new(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t)
}

// ---------------------------------------------------------------------------
// Chamfer insertion
// ---------------------------------------------------------------------------

fn add_chamfers(path: &BezPath, size: f64, min_edge: f64) -> BezPath {
    let elements = path.elements();

    // Collect on-curve points with their types
    let mut points: Vec<(Point, bool)> = Vec::new(); // (point, is_line_segment)
    let mut first = Point::ZERO;
    let mut has_close = false;

    for el in elements {
        match *el {
            PathEl::MoveTo(p) => {
                first = p;
            }
            PathEl::LineTo(p) => {
                points.push((p, true));
            }
            PathEl::CurveTo(_, _, p) | PathEl::QuadTo(_, p) => {
                points.push((p, false));
            }
            PathEl::ClosePath => {
                has_close = true;
            }
        }
    }

    if points.is_empty() {
        return path.clone();
    }

    // Insert first_pt at the beginning (it's the MoveTo target)
    // For closed paths, the closing segment goes from last point to first_pt
    let mut all_pts: Vec<(Point, bool)> = vec![(first, true)]; // assume line for the moveto
    all_pts.extend_from_slice(&points);

    let n = all_pts.len();
    let mut result = BezPath::new();
    let mut new_points: Vec<Point> = Vec::new();

    for i in 0..n {
        let (pt, is_line) = all_pts[i];
        let (prev_pt, _) = all_pts[(i + n - 1) % n];
        let (next_pt, next_is_line) = all_pts[(i + 1) % n];

        // Only chamfer line-line corners
        if !is_line || !next_is_line {
            new_points.push(pt);
            continue;
        }

        let v_in = Vec2::new(pt.x - prev_pt.x, pt.y - prev_pt.y);
        let v_out = Vec2::new(next_pt.x - pt.x, next_pt.y - pt.y);
        let len_in = v_in.hypot();
        let len_out = v_out.hypot();

        if len_in < min_edge || len_out < min_edge {
            new_points.push(pt);
            continue;
        }

        // Check if there's a significant angle
        let dot = v_in.x * v_out.x + v_in.y * v_out.y;
        let cos_angle = dot / (len_in * len_out);
        if cos_angle.abs() > 0.95 {
            // Nearly parallel, skip chamfer
            new_points.push(pt);
            continue;
        }

        // Compute chamfer cut points
        let before = Point::new(
            pt.x - size * v_in.x / len_in,
            pt.y - size * v_in.y / len_in,
        );
        let after = Point::new(
            pt.x + size * v_out.x / len_out,
            pt.y + size * v_out.y / len_out,
        );

        new_points.push(before);
        new_points.push(after);
    }

    // Rebuild BezPath from points (all lines for chamfered paths)
    if let Some(&first) = new_points.first() {
        result.move_to(first);
        for &p in &new_points[1..] {
            result.line_to(p);
        }
        if has_close {
            result.push(PathEl::ClosePath);
        }
    }

    result
}
