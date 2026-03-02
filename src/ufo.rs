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
                points.push(contour_point(p, PointType::Curve, true));
            }
            PathEl::QuadTo(a, p) => {
                points.push(contour_point(a, PointType::OffCurve, false));
                points.push(contour_point(p, PointType::QCurve, true));
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
        matches!(p.typ, PointType::Curve | PointType::Line | PointType::QCurve)
    });
    let closing_smooth = if let Some(idx) = last_oncurve {
        let last = &points[idx];
        let eps = 0.5;
        if (last.x - first.x).abs() < eps && (last.y - first.y).abs() < eps {
            let smooth = last.smooth;
            points.remove(idx);
            smooth
        } else {
            false
        }
    } else {
        false
    };

    points.insert(0, contour_point(first, closing_type, closing_smooth));

    Ok(Contour::new(points, None))
}

fn contour_point(p: kurbo::Point, typ: PointType, smooth: bool) -> ContourPoint {
    ContourPoint::new(p.x, p.y, typ, smooth, None, None)
}
