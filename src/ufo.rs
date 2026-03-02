//! Convert traced bezier paths to UFO glyph format.

use kurbo::{BezPath, PathEl};
use norad::{Contour, ContourPoint, Glyph, PointType};

use crate::config::TracingConfig;
use crate::error::TraceError;
use crate::TraceResult;

/// Convert a `TraceResult` to a `norad::Glyph`.
pub fn to_glyph(
    name: &str,
    result: &TraceResult,
    config: &TracingConfig,
) -> Result<Glyph, TraceError> {
    let mut glyph = Glyph::new(name);
    glyph.width = result.advance_width;

    for &codepoint in &config.codepoints {
        glyph.codepoints.insert(codepoint);
    }
    for path in &result.paths {
        glyph.contours.push(to_contour(path)?);
    }

    Ok(glyph)
}

/// Convert a `kurbo::BezPath` to a `norad::Contour`.
pub fn to_contour(path: &BezPath) -> Result<Contour, TraceError> {
    let elements = path.elements();
    if elements.is_empty() {
        return Err(TraceError::EmptyContour);
    }

    let first = match elements.first() {
        Some(PathEl::MoveTo(p)) => *p,
        _ => {
            return Err(TraceError::InvalidPath(
                "path must start with MoveTo".into(),
            ))
        }
    };

    let mut points: Vec<ContourPoint> = Vec::new();

    for el in elements.iter().skip(1) {
        match *el {
            PathEl::LineTo(p) => {
                points.push(contour_point(p, PointType::Line, false));
            }
            PathEl::CurveTo(a, b, p) => {
                points.push(contour_point(a, PointType::OffCurve, false));
                points.push(contour_point(b, PointType::OffCurve, false));
                // smooth is computed below after all points are collected.
                points.push(contour_point(p, PointType::Curve, false));
            }
            PathEl::QuadTo(a, p) => {
                points.push(contour_point(a, PointType::OffCurve, false));
                points.push(contour_point(p, PointType::QCurve, false));
            }
            PathEl::ClosePath => {}
            PathEl::MoveTo(_) => {
                return Err(TraceError::InvalidPath("unexpected MoveTo mid-path".into()))
            }
        }
    }

    // First point type comes from the closing segment.
    let closing_type = elements
        .iter()
        .rev()
        .find(|e| !matches!(e, PathEl::ClosePath))
        .map(|e| match e {
            PathEl::CurveTo(..) => PointType::Curve,
            PathEl::QuadTo(..) => PointType::QCurve,
            _ => PointType::Line,
        })
        .unwrap_or(PointType::Line);

    // If the last on-curve point duplicates the MoveTo (closing segment
    // returns to start), remove it — UFO contours are cyclic and the
    // first point implicitly closes the loop.
    let last_oncurve = points.iter().rposition(|p| {
        matches!(
            p.typ,
            PointType::Curve | PointType::Line | PointType::QCurve
        )
    });
    if let Some(idx) = last_oncurve {
        let last = &points[idx];
        // Tolerance for detecting duplicate closing points. 0.5 font units
        // accounts for grid-snapping rounding (grid=2 → max rounding = 1.0,
        // but the closing point and MoveTo are snapped independently so
        // they may differ by up to 1 unit; 0.5 is conservative).
        let eps = 0.5;
        if (last.x - first.x).abs() < eps && (last.y - first.y).abs() < eps {
            points.remove(idx);
        }
    }

    points.insert(0, contour_point(first, closing_type, false));

    // Compute smooth attribute: a curve/qcurve point is smooth only
    // if the incoming and outgoing tangent directions are collinear.
    // Per type design convention, smooth means the off-curve handles
    // on both sides and the on-curve point form a straight line.
    compute_smooth(&mut points);

    Ok(Contour::new(points, None))
}

/// Set smooth=true on on-curve points where tangent is continuous.
///
/// For each on-curve point (curve, qcurve, or line), check if the
/// incoming and outgoing tangent directions are collinear. Only
/// collinear tangents get smooth=true; tangent discontinuities are
/// corners. Line points are included so that line→curve junctions
/// with collinear handles (e.g. a vertical stem flowing into a curve
/// with a vertical first handle) are correctly marked smooth.
fn compute_smooth(points: &mut [ContourPoint]) {
    let n = points.len();
    if n < 3 {
        return;
    }

    for i in 0..n {
        if !matches!(
            points[i].typ,
            PointType::Curve | PointType::QCurve | PointType::Line
        ) {
            continue;
        }

        // Incoming tangent: from previous point toward this point.
        let prev = if i == 0 { n - 1 } else { i - 1 };
        let in_dx = points[i].x - points[prev].x;
        let in_dy = points[i].y - points[prev].y;

        // Outgoing tangent: from this point toward next point.
        let next = (i + 1) % n;
        let out_dx = points[next].x - points[i].x;
        let out_dy = points[next].y - points[i].y;

        let in_len = (in_dx * in_dx + in_dy * in_dy).sqrt();
        let out_len = (out_dx * out_dx + out_dy * out_dy).sqrt();

        if in_len < 0.01 || out_len < 0.01 {
            continue; // degenerate, leave as corner
        }

        // Cross product of unit tangent vectors.
        // Smooth if |cross| < sin(10°) ≈ 0.174, meaning tangents
        // are within ~10° of being collinear.
        let cross = (in_dx / in_len) * (out_dy / out_len) - (in_dy / in_len) * (out_dx / out_len);

        // Also check they point the same direction (dot > 0),
        // not opposite (which would be a cusp, not smooth).
        let dot = (in_dx / in_len) * (out_dx / out_len) + (in_dy / in_len) * (out_dy / out_len);

        if cross.abs() < 0.174 && dot > 0.0 {
            points[i].smooth = true;
        }
    }
}

fn contour_point(p: kurbo::Point, typ: PointType, smooth: bool) -> ContourPoint {
    ContourPoint::new(p.x, p.y, typ, smooth, None, None)
}
