//! Grid snapping and H/V handle correction.
//!
//! Snaps coordinates to a grid and corrects bezier handles
//! that are nearly horizontal or vertical to be exact.

use std::f64::consts::{FRAC_PI_2, PI};

use kurbo::{BezPath, CubicBez, ParamCurve, PathEl, Point};

/// Number of sample points along a curve for deviation checks.
const CURVE_SAMPLES: usize = 8;

/// Threshold for considering a handle already H/V aligned (font units).
const ALREADY_HV_THRESHOLD: f64 = 0.5;

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
                if (p.y - prev.y).abs() <= threshold {
                    p.y = prev.y;
                }
                if (p.x - prev.x).abs() <= threshold {
                    p.x = prev.x;
                }
                prev = *p;
            }
            PathEl::CurveTo(_c1, _c2, p) => {
                if (p.y - prev.y).abs() <= threshold {
                    p.y = prev.y;
                }
                if (p.x - prev.x).abs() <= threshold {
                    p.x = prev.x;
                }
                prev = *p;
            }
            PathEl::QuadTo(_, p) => {
                if (p.y - prev.y).abs() <= threshold {
                    p.y = prev.y;
                }
                if (p.x - prev.x).abs() <= threshold {
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

/// Force remaining non-H/V handles to the nearest axis, keeping the
/// result only when the curve doesn't move more than `max_deviation`.
///
/// Unlike `hv_handles` (angle-threshold gated), this tries every handle
/// and uses a shape-deviation guard instead.
pub fn force_hv_handles(path: &BezPath, max_deviation: f64) -> BezPath {
    let mut elements: Vec<PathEl> = path.elements().to_vec();
    let mut prev = Point::ZERO;

    #[allow(clippy::needless_range_loop)]
    for idx in 0..elements.len() {
        match elements[idx] {
            PathEl::MoveTo(p) | PathEl::LineTo(p) => {
                prev = p;
            }
            PathEl::CurveTo(c1, c2, p3) => {
                let orig = CubicBez::new(prev, c1, c2, p3);

                // Try snapping c1 relative to prev anchor
                let c1_h = Point::new(c1.x, prev.y);
                let c1_v = Point::new(prev.x, c1.y);
                let new_c1 = if is_already_hv(c1, prev) {
                    c1
                } else {
                    pick_best_snap(c1, c1_h, c1_v, |c| {
                        curve_deviation(&orig, &CubicBez::new(prev, c, c2, p3))
                    }, max_deviation)
                };

                // Try snapping c2 relative to p3 anchor
                let c2_h = Point::new(c2.x, p3.y);
                let c2_v = Point::new(p3.x, c2.y);
                let new_c2 = if is_already_hv(c2, p3) {
                    c2
                } else {
                    pick_best_snap(c2, c2_h, c2_v, |c| {
                        curve_deviation(&orig, &CubicBez::new(prev, new_c1, c, p3))
                    }, max_deviation)
                };

                elements[idx] = PathEl::CurveTo(new_c1, new_c2, p3);
                prev = p3;
            }
            PathEl::QuadTo(_, p) => prev = p,
            PathEl::ClosePath => {}
        }
    }

    BezPath::from_vec(elements)
}

fn is_already_hv(handle: Point, anchor: Point) -> bool {
    (handle.x - anchor.x).abs() < ALREADY_HV_THRESHOLD
        || (handle.y - anchor.y).abs() < ALREADY_HV_THRESHOLD
}

/// Pick horizontal or vertical snap — whichever produces less deviation.
/// Returns the original handle if both exceed `max_deviation`.
fn pick_best_snap(
    original: Point,
    h_snap: Point,
    v_snap: Point,
    deviation_fn: impl Fn(Point) -> f64,
    max_deviation: f64,
) -> Point {
    let d_h = deviation_fn(h_snap);
    let d_v = deviation_fn(v_snap);

    if d_h <= d_v && d_h <= max_deviation {
        h_snap
    } else if d_v <= max_deviation {
        v_snap
    } else {
        original
    }
}

/// Max deviation between two cubics, sampled at evenly-spaced points.
fn curve_deviation(a: &CubicBez, b: &CubicBez) -> f64 {
    let n = CURVE_SAMPLES;
    let mut max_d = 0.0f64;
    for i in 0..=n {
        let t = i as f64 / n as f64;
        let d = a.eval(t).distance(b.eval(t));
        max_d = max_d.max(d);
    }
    max_d
}
