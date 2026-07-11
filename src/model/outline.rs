// Copyright 2026 the img2bez Authors
// SPDX-License-Identifier: Apache-2.0 OR MIT

//! The canonical outline model: a language-neutral, serde-serializable
//! description of a traced glyph's contours.
//!
//! The hub of img2bez's output: the tracer produces an [`Outline`], and
//! everything else is a view onto it — [`kurbo::BezPath`], GLIF, JSON. Unlike
//! a bare `BezPath`, the model carries the UFO point semantics: each on-curve
//! point's type (`line`/`curve`/`qcurve`) and its `smooth` flag.
//!
//! ## Point convention
//!
//! Each [`Contour`] is a closed ring of [`OutlinePoint`]s in the UFO point
//! order: off-curve control points precede the on-curve point they lead into,
//! and the first point carries the type of the segment that closes the ring.
//! This is exactly the UFO `.glif` point stream, so conversion to GLIF/UFO is
//! lossless. Coordinates are y-up (font convention).

use kurbo::{BezPath, PathEl, Point};
use serde::{Deserialize, Serialize};

/// The role of a point in a contour. These are exactly the UFO point types:
/// `move`/`line`/`curve`/`qcurve` are on-curve, and an off-curve control point
/// carries no `type` in a `.glif` — so it serializes with the `type` field
/// omitted (see [`OutlinePoint`]), matching UFO.
#[derive(
    Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize,
)]
#[serde(rename_all = "lowercase")]
pub enum PointKind {
    /// The start of an open contour. Traced glyphs are closed, so this is
    /// rare; closed contours carry the closing segment's type on point 0.
    Move,
    /// An on-curve point ending a straight segment.
    Line,
    /// An on-curve point ending a cubic segment (two preceding off-curves).
    Curve,
    /// An on-curve point ending a quadratic segment (one preceding off-curve).
    QCurve,
    /// An off-curve control point (bezier handle). The default — a UFO point
    /// with no `type` is off-curve.
    #[default]
    OffCurve,
}

impl PointKind {
    /// Whether this is an off-curve control point.
    pub fn is_off_curve(&self) -> bool {
        matches!(self, PointKind::OffCurve)
    }
}

/// A single point in a contour. Serializes like a UFO `<point>`: `x`, `y`, an
/// optional `type` (omitted for off-curve points), and `smooth` only when set.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct OutlinePoint {
    /// X coordinate (font units, y-up space).
    pub x: f64,
    /// Y coordinate (font units, y-up space).
    pub y: f64,
    /// The point's role in the contour. Off-curve omits this in JSON (UFO).
    #[serde(
        rename = "type",
        default,
        skip_serializing_if = "PointKind::is_off_curve"
    )]
    pub kind: PointKind,
    /// True when the two segments meeting at this on-curve point are tangent
    /// (a smooth join). Never set on off-curve points.
    #[serde(default, skip_serializing_if = "is_false")]
    pub smooth: bool,
}

fn is_false(b: &bool) -> bool {
    !*b
}

/// A single closed contour: a ring of points in UFO point order.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Contour {
    /// The contour's points (off-curves precede their on-curve).
    pub points: Vec<OutlinePoint>,
}

impl Contour {
    /// Rotate the point list so the contour starts at its bottom-most
    /// on-curve point; ties on y resolve to the leftmost (`rtl` false)
    /// or rightmost (`rtl` true) point, matching writing direction.
    /// Off-curves that preceded the new start wrap to the end, which is
    /// the standard UFO representation. No-op on all-off-curve contours.
    pub fn normalize_start(&mut self, rtl: bool) {
        let mut best: Option<(usize, f64, f64)> = None;
        for (i, p) in self.points.iter().enumerate() {
            if p.kind == PointKind::OffCurve {
                continue;
            }
            let better = match best {
                None => true,
                Some((_, by, bx)) => {
                    p.y < by - 1e-9
                        || (p.y < by + 1e-9
                            && if rtl {
                                p.x > bx + 1e-9
                            } else {
                                p.x < bx - 1e-9
                            })
                }
            };
            if better {
                best = Some((i, p.y, p.x));
            }
        }
        if let Some((i, _, _)) = best {
            self.points.rotate_left(i);
        }
    }
}

/// A traced glyph outline: one or more closed contours.
///
/// This is the canonical output of [`crate::trace`]. Convert it to the
/// representation a given consumer wants with the adapter methods, or
/// serialize it to JSON directly.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Outline {
    /// The glyph's contours (outer boundaries and counters).
    pub contours: Vec<Contour>,
}

/// Coordinates within this distance (font units) of an integer print
/// without a decimal point in SVG output (see [`num`]).
const COINCIDENT_EPS: f64 = 1e-6;

impl Outline {
    /// Normalize every contour's start point (see
    /// [`Contour::normalize_start`]). Called by the tracer with
    /// [`TraceOptions::rtl_start`](crate::TraceOptions::rtl_start);
    /// exposed for callers that build or edit outlines directly.
    pub fn normalize_starts(&mut self, rtl: bool) {
        for c in &mut self.contours {
            c.normalize_start(rtl);
        }
    }

    /// Build an outline from traced `kurbo` contours.
    ///
    /// Converts each `BezPath` into the UFO point stream — splitting cubics and
    /// quadratics into their off-curve/on-curve points, choosing the closing
    /// segment's type, dropping a redundant closing point, and computing the
    /// `smooth` flag once here so every downstream view agrees on it.
    pub fn from_bezpaths(paths: &[BezPath]) -> Self {
        let contours = paths.iter().filter_map(contour_from_bezpath).collect();
        Outline { contours }
    }

    /// Convert back to one `kurbo::BezPath` per contour.
    pub fn to_bezpaths(&self) -> Vec<BezPath> {
        self.contours.iter().map(contour_to_bezpath).collect()
    }

    /// Return a copy with every coordinate multiplied by `factor`. Use this to
    /// rescale a trace (done at the default em) to a different UPM before
    /// placing it — e.g. `outline.scaled(2048.0 / 1088.0)`.
    pub fn scaled(&self, factor: f64) -> Outline {
        let contours = self
            .contours
            .iter()
            .map(|c| Contour {
                points: c
                    .points
                    .iter()
                    .map(|p| OutlinePoint {
                        x: p.x * factor,
                        y: p.y * factor,
                        ..*p
                    })
                    .collect(),
            })
            .collect();
        Outline { contours }
    }

    /// Return a copy translated by `(dx, dy)`.
    pub fn translated(&self, dx: f64, dy: f64) -> Outline {
        let contours = self
            .contours
            .iter()
            .map(|c| Contour {
                points: c
                    .points
                    .iter()
                    .map(|p| OutlinePoint {
                        x: p.x + dx,
                        y: p.y + dy,
                        ..*p
                    })
                    .collect(),
            })
            .collect();
        Outline { contours }
    }

    /// Tight bounds of the outline (accounting for curve extents), or `None`
    /// if it has no drawable points.
    pub fn bounds(&self) -> Option<kurbo::Rect> {
        use kurbo::Shape;
        self.to_bezpaths()
            .iter()
            .map(|p| p.bounding_box())
            .reduce(|a, b| a.union(b))
    }

    /// Serialize as an SVG `<path>` `d` attribute string (one subpath per
    /// contour). Coordinates are emitted as-is (y-up); flip vertically when
    /// embedding in a y-down SVG viewport.
    pub fn to_svg_path(&self) -> String {
        let mut d = String::new();
        for contour in &self.contours {
            append_svg_contour(&mut d, contour);
        }
        d.trim_end().to_string()
    }
}

/// Convert one `BezPath` to a UFO-ordered `Contour`, or `None` if it carries no
/// drawable points.
fn contour_from_bezpath(path: &BezPath) -> Option<Contour> {
    let elements = path.elements();
    let first = match elements.first() {
        Some(PathEl::MoveTo(p)) => *p,
        _ => return None,
    };

    let mut points: Vec<OutlinePoint> = Vec::new();
    for el in elements.iter().skip(1) {
        match *el {
            PathEl::LineTo(p) => points.push(on(p, PointKind::Line)),
            PathEl::CurveTo(a, b, p) => {
                points.push(off(a));
                points.push(off(b));
                points.push(on(p, PointKind::Curve));
            }
            PathEl::QuadTo(a, p) => {
                points.push(off(a));
                points.push(on(p, PointKind::QCurve));
            }
            PathEl::ClosePath => {}
            PathEl::MoveTo(_) => return None,
        }
    }

    // The first point of a closed ring carries the type of the segment that
    // closes it (the last real segment).
    let closing_kind = elements
        .iter()
        .rev()
        .find(|e| !matches!(e, PathEl::ClosePath))
        .map(|e| match e {
            PathEl::CurveTo(..) => PointKind::Curve,
            PathEl::QuadTo(..) => PointKind::QCurve,
            _ => PointKind::Line,
        })
        .unwrap_or(PointKind::Line);

    // Drop a trailing on-curve coincident with the start (duplicate close).
    if let Some(idx) =
        points.iter().rposition(|p| p.kind != PointKind::OffCurve)
    {
        let last = points[idx];
        let eps = 0.5;
        if (last.x - first.x).abs() < eps && (last.y - first.y).abs() < eps {
            points.remove(idx);
        }
    }

    points.insert(0, on(first, closing_kind));
    if points.is_empty() {
        return None;
    }
    compute_smooth(&mut points);
    Some(Contour { points })
}

/// Reconstruct a closed `BezPath` from a UFO-ordered contour.
fn contour_to_bezpath(contour: &Contour) -> BezPath {
    let mut bez = BezPath::new();
    let pts = &contour.points;
    let Some(start) = pts.first() else {
        return bez;
    };
    let start_pt = Point::new(start.x, start.y);
    bez.move_to(start_pt);

    let mut pending: Vec<Point> = Vec::new();
    for p in &pts[1..] {
        if p.kind == PointKind::OffCurve {
            pending.push(Point::new(p.x, p.y));
        } else {
            emit_segment(&mut bez, &pending, p);
            pending.clear();
        }
    }
    // Closing segment back to the start, whose kind names the segment type.
    let close = OutlinePoint {
        x: start.x,
        y: start.y,
        kind: start.kind,
        smooth: false,
    };
    if !pending.is_empty() || close.kind != PointKind::Line {
        emit_segment(&mut bez, &pending, &close);
    }
    bez.close_path();
    bez
}

/// Emit one segment ending at `end` using the pending off-curve handles.
fn emit_segment(bez: &mut BezPath, pending: &[Point], end: &OutlinePoint) {
    let p = Point::new(end.x, end.y);
    match end.kind {
        PointKind::Line | PointKind::Move => {
            // A "line" with stray handles is degenerate; treat as a line.
            if bez.elements().last().map(|e| e.end_point()) != Some(Some(p)) {
                bez.line_to(p);
            }
        }
        PointKind::Curve => match pending {
            [a, b, ..] => bez.curve_to(*a, *b, p),
            [a] => bez.quad_to(*a, p),
            [] => bez.line_to(p),
        },
        PointKind::QCurve => match pending {
            [a, ..] => bez.quad_to(*a, p),
            [] => bez.line_to(p),
        },
        PointKind::OffCurve => {}
    }
}

/// Append one contour to an SVG path `d` string.
fn append_svg_contour(d: &mut String, contour: &Contour) {
    let pts = &contour.points;
    let Some(start) = pts.first() else { return };
    d.push_str(&format!("M{} {} ", num(start.x), num(start.y)));

    let mut pending: Vec<&OutlinePoint> = Vec::new();
    let emit =
        |d: &mut String, pending: &[&OutlinePoint], p: &OutlinePoint| match p
            .kind
        {
            PointKind::Line | PointKind::Move => {
                d.push_str(&format!("L{} {} ", num(p.x), num(p.y)))
            }
            PointKind::Curve => match pending {
                [a, b, ..] => d.push_str(&format!(
                    "C{} {} {} {} {} {} ",
                    num(a.x),
                    num(a.y),
                    num(b.x),
                    num(b.y),
                    num(p.x),
                    num(p.y)
                )),
                [a] => d.push_str(&format!(
                    "Q{} {} {} {} ",
                    num(a.x),
                    num(a.y),
                    num(p.x),
                    num(p.y)
                )),
                [] => d.push_str(&format!("L{} {} ", num(p.x), num(p.y))),
            },
            PointKind::QCurve => match pending {
                [a, ..] => d.push_str(&format!(
                    "Q{} {} {} {} ",
                    num(a.x),
                    num(a.y),
                    num(p.x),
                    num(p.y)
                )),
                [] => d.push_str(&format!("L{} {} ", num(p.x), num(p.y))),
            },
            PointKind::OffCurve => {}
        };

    for p in &pts[1..] {
        if p.kind == PointKind::OffCurve {
            pending.push(p);
        } else {
            emit(d, &pending, p);
            pending.clear();
        }
    }
    let close = OutlinePoint {
        kind: start.kind,
        ..*start
    };
    if !pending.is_empty() || close.kind != PointKind::Line {
        emit(d, &pending, &close);
    }
    d.push_str("Z ");
}

/// Mark on-curve points whose adjacent neighbours are nearly collinear as
/// smooth (~10° tolerance). The single source of truth for the smooth flag.
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
        let (in_dx, in_dy) =
            (points[i].x - points[prev].x, points[i].y - points[prev].y);
        let (out_dx, out_dy) =
            (points[next].x - points[i].x, points[next].y - points[i].y);
        let in_len = (in_dx * in_dx + in_dy * in_dy).sqrt();
        let out_len = (out_dx * out_dx + out_dy * out_dy).sqrt();
        if in_len < 0.01 || out_len < 0.01 {
            continue;
        }
        let cross = (in_dx / in_len) * (out_dy / out_len)
            - (in_dy / in_len) * (out_dx / out_len);
        let dot = (in_dx / in_len) * (out_dx / out_len)
            + (in_dy / in_len) * (out_dy / out_len);
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

/// Format a coordinate for SVG: integers print without a decimal point.
///
/// BYTE-IDENTICAL INVARIANT: output formatting here must stay in lockstep
/// with `glif::format_coord` — downstream workflows diff emitted files
/// byte-for-byte. Do not change the formatting rules.
pub(crate) fn num(v: f64) -> String {
    let r = v.round();
    if (v - r).abs() < COINCIDENT_EPS {
        format!("{}", r as i64)
    } else {
        let mut s = format!("{v:.3}");
        while s.contains('.') && s.ends_with('0') {
            s.pop();
        }
        if s.ends_with('.') {
            s.pop();
        }
        s
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use kurbo::Shape;

    fn square() -> BezPath {
        let mut p = BezPath::new();
        p.move_to((0.0, 0.0));
        p.line_to((100.0, 0.0));
        p.line_to((100.0, 100.0));
        p.line_to((0.0, 100.0));
        p.close_path();
        p
    }

    #[test]
    fn square_round_trips_through_outline() {
        let o = Outline::from_bezpaths(&[square()]);
        assert_eq!(o.contours.len(), 1);
        // 4 corners, all lines.
        assert_eq!(o.contours[0].points.len(), 4);
        assert!(
            o.contours[0]
                .points
                .iter()
                .all(|p| p.kind == PointKind::Line)
        );
        // Reconstructs to a closed path with the same corner set.
        let back = o.to_bezpaths();
        assert_eq!(back.len(), 1);
        let area_a = square().area().abs();
        let area_b = back[0].area().abs();
        assert!((area_a - area_b).abs() < 1.0, "{area_a} vs {area_b}");
    }

    #[test]
    fn json_round_trips() {
        let o = Outline::from_bezpaths(&[square()]);
        let json = serde_json::to_string(&o).unwrap();
        let back: Outline = serde_json::from_str(&json).unwrap();
        assert_eq!(o, back);
    }

    #[test]
    fn svg_path_has_move_and_close() {
        let o = Outline::from_bezpaths(&[square()]);
        let d = o.to_svg_path();
        assert!(d.starts_with('M'), "{d}");
        assert!(d.ends_with('Z'), "{d}");
    }
}
