//! Grid snapping and H/V handle correction.
//!
//! Snaps coordinates to a grid and corrects bezier handles
//! that are nearly horizontal or vertical to be exact.

use std::f64::consts::{FRAC_PI_2, PI};

use kurbo::{BezPath, PathEl, Point};

/// Snap all coordinates to a grid.
pub fn to_grid(path: &BezPath, grid: f64) -> BezPath {
    let snap = |p: Point| -> Point {
        Point::new(
            (p.x / grid).round() * grid,
            (p.y / grid).round() * grid,
        )
    };
    BezPath::from_vec(
        path.elements()
            .iter()
            .map(|el| match *el {
                PathEl::MoveTo(p) => PathEl::MoveTo(snap(p)),
                PathEl::LineTo(p) => PathEl::LineTo(snap(p)),
                PathEl::CurveTo(a, b, p) => {
                    PathEl::CurveTo(snap(a), snap(b), snap(p))
                }
                PathEl::QuadTo(a, p) => PathEl::QuadTo(snap(a), snap(p)),
                PathEl::ClosePath => PathEl::ClosePath,
            })
            .collect(),
    )
}

/// Snap handles within `threshold_deg` of H/V to exact H/V.
pub fn hv_handles(path: &BezPath, threshold_deg: f64) -> BezPath {
    let threshold = threshold_deg.to_radians();
    let mut elements: Vec<PathEl> = path.elements().to_vec();
    let mut prev = Point::ZERO;

    for el in &mut elements {
        match el {
            PathEl::MoveTo(p) | PathEl::LineTo(p) => {
                prev = *p;
            }
            PathEl::CurveTo(c1, c2, p3) => {
                *c1 = snap_handle(*c1, prev, threshold);
                *c2 = snap_handle(*c2, *p3, threshold);
                prev = *p3;
            }
            PathEl::QuadTo(_, p) => prev = *p,
            PathEl::ClosePath => {}
        }
    }

    BezPath::from_vec(elements)
}

/// Snap nearly-horizontal/vertical line and curve endpoints to exact H/V.
///
/// For each segment, if the endpoint is within `threshold` font units of
/// being horizontally or vertically aligned with the previous on-curve point,
/// snap it to exact alignment.
pub fn hv_lines(path: &BezPath, threshold: f64) -> BezPath {
    let mut elements: Vec<PathEl> = path.elements().to_vec();
    let mut prev = Point::ZERO;

    for el in &mut elements {
        match el {
            PathEl::MoveTo(p) => {
                prev = *p;
            }
            PathEl::LineTo(p) => {
                if (p.y - prev.y).abs() < threshold {
                    p.y = prev.y;
                }
                if (p.x - prev.x).abs() < threshold {
                    p.x = prev.x;
                }
                prev = *p;
            }
            PathEl::CurveTo(_c1, _c2, p) => {
                if (p.y - prev.y).abs() < threshold {
                    p.y = prev.y;
                }
                if (p.x - prev.x).abs() < threshold {
                    p.x = prev.x;
                }
                prev = *p;
            }
            PathEl::QuadTo(_, p) => {
                if (p.y - prev.y).abs() < threshold {
                    p.y = prev.y;
                }
                if (p.x - prev.x).abs() < threshold {
                    p.x = prev.x;
                }
                prev = *p;
            }
            PathEl::ClosePath => {}
        }
    }

    BezPath::from_vec(elements)
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
