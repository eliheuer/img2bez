//! Extrema insertion for cubic bezier curves.
//!
//! Inserts on-curve points at horizontal and vertical extrema
//! so that bounding boxes are tight and hinting works correctly.

use kurbo::{BezPath, CubicBez, ParamCurve, PathEl, Point};

/// Margin from t=0 and t=1 — extrema closer to endpoints than this are skipped.
const ENDPOINT_MARGIN: f64 = 0.01;

/// Insert on-curve points at H/V extrema of cubic curves.
///
/// Only inserts when the extremum extends beyond the
/// endpoints by at least `min_depth` font units.
pub fn insert_extrema(path: &BezPath, min_depth: f64) -> BezPath {
    let mut output = BezPath::new();
    let mut current = Point::ZERO;

    for el in path.elements() {
        match *el {
            PathEl::MoveTo(p) => {
                output.move_to(p);
                current = p;
            }
            PathEl::LineTo(p) => {
                output.line_to(p);
                current = p;
            }
            PathEl::CurveTo(a, b, p) => {
                let cubic = CubicBez::new(current, a, b, p);
                let splits = extrema_t_values(&cubic, min_depth);
                if splits.is_empty() {
                    output.push(PathEl::CurveTo(a, b, p));
                } else {
                    split_at_ts(&mut output, &cubic, &splits);
                }
                current = p;
            }
            PathEl::QuadTo(a, p) => {
                output.push(PathEl::QuadTo(a, p));
                current = p;
            }
            PathEl::ClosePath => {
                output.push(PathEl::ClosePath);
            }
        }
    }
    output
}

/// Find t-values of H/V extrema on a cubic.
fn extrema_t_values(cubic: &CubicBez, min_depth: f64) -> Vec<f64> {
    let mut t_values = Vec::new();
    for axis in 0..2 {
        let (v0, v1, v2, v3) = if axis == 0 {
            (cubic.p0.x, cubic.p1.x, cubic.p2.x, cubic.p3.x)
        } else {
            (cubic.p0.y, cubic.p1.y, cubic.p2.y, cubic.p3.y)
        };

        // Derivative coefficients: At² + Bt + C = 0
        let coeff_a = -3.0 * v0 + 9.0 * v1 - 9.0 * v2 + 3.0 * v3;
        let coeff_b = 6.0 * v0 - 12.0 * v1 + 6.0 * v2;
        let coeff_c = -3.0 * v0 + 3.0 * v1;

        for t in solve_quadratic(coeff_a, coeff_b, coeff_c) {
            if t <= ENDPOINT_MARGIN || t >= 1.0 - ENDPOINT_MARGIN {
                continue;
            }
            if extremum_depth(cubic, t, axis) >= min_depth {
                t_values.push(t);
            }
        }
    }

    t_values.sort_by(|a, b| a.partial_cmp(b).unwrap());
    t_values.dedup_by(|a, b| (*a - *b).abs() < 0.001);
    t_values
}

/// How far an extremum extends beyond the endpoints.
fn extremum_depth(cubic: &CubicBez, t: f64, axis: usize) -> f64 {
    let point = cubic.eval(t);
    let vt = if axis == 0 { point.x } else { point.y };
    let v0 = if axis == 0 { cubic.p0.x } else { cubic.p0.y };
    let v3 = if axis == 0 { cubic.p3.x } else { cubic.p3.y };
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
    let sqrt_disc = disc.sqrt();
    vec![(-b + sqrt_disc) / (2.0 * a), (-b - sqrt_disc) / (2.0 * a)]
}

/// Split a cubic at sorted t-values, appending to output.
fn split_at_ts(output: &mut BezPath, cubic: &CubicBez, ts: &[f64]) {
    let mut rest = *cubic;
    let mut prev_t = 0.0;

    for &t in ts {
        let local = (t - prev_t) / (1.0 - prev_t);
        let (left, right) = subdivide(rest, local);
        output.push(PathEl::CurveTo(left.p1, left.p2, left.p3));
        rest = right;
        prev_t = t;
    }
    output.push(PathEl::CurveTo(rest.p1, rest.p2, rest.p3));
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
