//! Grid snapping and H/V handle correction.
//!
//! Snaps coordinates to a grid and corrects bezier handles
//! that are nearly horizontal or vertical to be exact.

use std::f64::consts::{FRAC_PI_2, PI};

use kurbo::{BezPath, PathEl, Point};

/// Snap all coordinates to a grid.
///
/// On-curve points (MoveTo, LineTo, and the endpoint of CurveTo/QuadTo)
/// are snapped to the grid. Off-curve control points are NOT snapped,
/// preserving curve shape accuracy while keeping on-curve points on-grid.
pub fn to_grid(path: &BezPath, grid: f64) -> BezPath {
    let snap = |p: Point| -> Point {
        Point::new((p.x / grid).round() * grid, (p.y / grid).round() * grid)
    };
    BezPath::from_vec(
        path.elements()
            .iter()
            .map(|el| match *el {
                PathEl::MoveTo(p) => PathEl::MoveTo(snap(p)),
                PathEl::LineTo(p) => PathEl::LineTo(snap(p)),
                PathEl::CurveTo(a, b, p) => PathEl::CurveTo(a, b, snap(p)),
                PathEl::QuadTo(a, p) => PathEl::QuadTo(a, snap(p)),
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

/// Snap a handle to exact H/V if its angle from the anchor is close enough.
///
/// Computes the absolute angle of the handle-anchor vector. If the angle
/// is within `threshold` radians of 0 or 180 deg (horizontal), the handle's
/// y-coordinate is set to the anchor's y. If within `threshold` of 90 deg
/// (vertical), the handle's x is set to the anchor's x.
fn snap_handle(handle: Point, anchor: Point, threshold: f64) -> Point {
    let dx = handle.x - anchor.x;
    let dy = handle.y - anchor.y;
    if dx * dx + dy * dy < 1e-12 {
        return handle; // coincident — nothing to snap
    }

    let angle = dy.atan2(dx).abs(); // range: 0..PI

    // Near horizontal (0 deg or 180 deg): lock y to anchor.
    if angle < threshold || (PI - angle) < threshold {
        return Point::new(handle.x, anchor.y);
    }
    // Near vertical (90 deg): lock x to anchor.
    if (angle - FRAC_PI_2).abs() < threshold {
        return Point::new(anchor.x, handle.y);
    }
    handle
}
