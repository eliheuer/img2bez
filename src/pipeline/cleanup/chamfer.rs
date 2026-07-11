// Copyright 2026 the img2bez Authors
// SPDX-License-Identifier: Apache-2.0 OR MIT

//! Chamfer insertion: replace sharp line-line corners with bevels — the
//! defining visual feature of some grotesque designs.

use kurbo::{BezPath, Line, ParamCurve, PathEl, PathSeg, Point, QuadBez};

/// Skip corners within ~18° of straight (cos > 0.95) — too shallow for a
/// visible bevel.
const COLLINEAR_THRESHOLD: f64 = 0.95;

/// Add chamfers at line-line corners: a bevel with endpoints `size` font
/// units back along each edge. Skipped when an adjacent edge is shorter than
/// `min_edge` or `2*size`, the edges are near-collinear, or a neighbor is a
/// curve (curves pass through unchanged).
pub fn chamfer(path: &BezPath, size: f64, min_edge: f64) -> BezPath {
    let mut out = BezPath::new();
    for (segs, closed) in split_contours(path) {
        let segs = chamfer_contour(segs, size, min_edge, closed);
        append_contour(&mut out, &segs, closed);
    }
    out
}

/// Insert bevels into one contour's segment list.
fn chamfer_contour(
    segs: Vec<PathSeg>,
    size: f64,
    min_edge: f64,
    closed: bool,
) -> Vec<PathSeg> {
    let n = segs.len();
    if n < 2 {
        return segs;
    }
    // Joint j sits at the start of segment j (= end of segment j-1,
    // cyclically). An open contour has no joint before its first segment.
    let mut bevel = vec![false; n];
    for j in 0..n {
        if !closed && j == 0 {
            continue;
        }
        let prev = (j + n - 1) % n;
        let (PathSeg::Line(a), PathSeg::Line(b)) = (segs[prev], segs[j]) else {
            continue;
        };
        let v_in = a.p1 - a.p0;
        let v_out = b.p1 - b.p0;
        let (len_in, len_out) = (v_in.hypot(), v_out.hypot());
        if len_in < min_edge.max(2.0 * size)
            || len_out < min_edge.max(2.0 * size)
        {
            continue;
        }
        let cos = v_in.dot(v_out) / (len_in * len_out);
        if cos.abs() > COLLINEAR_THRESHOLD {
            continue;
        }
        bevel[j] = true;
    }
    let mut out: Vec<PathSeg> = Vec::with_capacity(n + bevel.len());
    for j in 0..n {
        let mut seg = segs[j];
        if let PathSeg::Line(mut l) = seg {
            // Trim the ends that receive a bevel.
            let u = (l.p1 - l.p0) / (l.p1 - l.p0).hypot();
            if bevel[j] {
                l.p0 += u * size;
            }
            if bevel[(j + 1) % n] && (closed || j + 1 < n) {
                l.p1 -= u * size;
            }
            seg = PathSeg::Line(l);
        }
        if bevel[j] {
            // Bevel: previous segment's trimmed end → this trimmed start.
            // If nothing is emitted yet (closed contour), compute the last
            // segment's trimmed end directly; bevel[j] guarantees a line.
            let start = out.last().map(|s| s.end()).unwrap_or_else(|| {
                let trim = match segs[n - 1] {
                    PathSeg::Line(l) => {
                        (l.p1 - l.p0) / (l.p1 - l.p0).hypot() * size
                    }
                    _ => kurbo::Vec2::ZERO,
                };
                segs[n - 1].end() - trim
            });
            out.push(PathSeg::Line(Line::new(start, seg.start())));
        }
        out.push(seg);
    }
    out
}

/// Split a path into per-contour segment lists, materializing each closing
/// line, and remember whether the contour was closed.
fn split_contours(path: &BezPath) -> Vec<(Vec<PathSeg>, bool)> {
    let mut contours = Vec::new();
    let mut cur: Vec<PathSeg> = Vec::new();
    let mut closed = false;
    let mut start = Point::ZERO;
    let mut at = Point::ZERO;
    for el in path.elements() {
        match *el {
            PathEl::MoveTo(p) => {
                if !cur.is_empty() {
                    contours.push((std::mem::take(&mut cur), closed));
                }
                closed = false;
                start = p;
                at = p;
            }
            PathEl::LineTo(p) => {
                cur.push(PathSeg::Line(Line::new(at, p)));
                at = p;
            }
            PathEl::QuadTo(c, p) => {
                cur.push(PathSeg::Quad(QuadBez::new(at, c, p)));
                at = p;
            }
            PathEl::CurveTo(c1, c2, p) => {
                cur.push(PathSeg::Cubic(kurbo::CubicBez::new(at, c1, c2, p)));
                at = p;
            }
            PathEl::ClosePath => {
                if (at - start).hypot() > 1e-9 {
                    cur.push(PathSeg::Line(Line::new(at, start)));
                    at = start;
                }
                closed = true;
            }
        }
    }
    if !cur.is_empty() {
        contours.push((cur, closed));
    }
    contours
}

/// Append a contour (segment list) to `out`, closing it if it was closed.
fn append_contour(out: &mut BezPath, segs: &[PathSeg], closed: bool) {
    let Some(first) = segs.first() else { return };
    out.move_to(first.start());
    for seg in segs {
        match seg {
            PathSeg::Line(l) => out.line_to(l.p1),
            PathSeg::Quad(q) => out.quad_to(q.p1, q.p2),
            PathSeg::Cubic(c) => out.curve_to(c.p1, c.p2, c.p3),
        }
    }
    if closed {
        out.close_path();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use kurbo::{BezPath, PathEl, Point};

    fn lines(path: &BezPath) -> usize {
        path.elements()
            .iter()
            .filter(|el| matches!(el, PathEl::LineTo(_)))
            .count()
    }

    fn curves(path: &BezPath) -> usize {
        path.elements()
            .iter()
            .filter(|el| matches!(el, PathEl::CurveTo(..)))
            .count()
    }

    #[test]
    fn right_angle_gets_chamfered() {
        let mut path = BezPath::new();
        path.move_to(Point::new(0.0, 0.0));
        path.line_to(Point::new(100.0, 0.0));
        path.line_to(Point::new(100.0, 100.0));
        path.line_to(Point::new(0.0, 100.0));
        path.push(PathEl::ClosePath);
        let result = chamfer(&path, 10.0, 5.0);
        // Four corners, each gaining one bevel line: 4 (trimmed) + 4 bevels.
        assert_eq!(lines(&result), 8, "square should gain 4 bevel lines");
    }

    #[test]
    fn curves_pass_through_unchanged() {
        // Stadium shape: no line-line corner, so nothing may change.
        let mut path = BezPath::new();
        path.move_to(Point::new(0.0, 0.0));
        path.line_to(Point::new(100.0, 0.0));
        path.curve_to(
            Point::new(130.0, 0.0),
            Point::new(130.0, 50.0),
            Point::new(100.0, 50.0),
        );
        path.line_to(Point::new(0.0, 50.0));
        path.curve_to(
            Point::new(-30.0, 50.0),
            Point::new(-30.0, 0.0),
            Point::new(0.0, 0.0),
        );
        path.push(PathEl::ClosePath);
        let result = chamfer(&path, 10.0, 5.0);
        assert_eq!(curves(&result), 2, "curves must be preserved");
        assert_eq!(lines(&result), 2, "no bevel at line-curve joints");
    }

    #[test]
    fn contours_stay_separate() {
        let mut path = BezPath::new();
        for off in [0.0, 300.0] {
            path.move_to(Point::new(off, 0.0));
            path.line_to(Point::new(off + 100.0, 0.0));
            path.line_to(Point::new(off + 100.0, 100.0));
            path.line_to(Point::new(off, 100.0));
            path.push(PathEl::ClosePath);
        }
        let result = chamfer(&path, 10.0, 5.0);
        let moves = result
            .elements()
            .iter()
            .filter(|el| matches!(el, PathEl::MoveTo(_)))
            .count();
        assert_eq!(moves, 2, "each contour keeps its own MoveTo");
        assert_eq!(lines(&result), 16, "both squares chamfer independently");
    }
}
