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

    for &cp in &config.codepoints {
        glyph.codepoints.insert(cp);
    }
    for path in &result.paths {
        glyph.contours.push(to_contour(path)?);
    }

    Ok(glyph)
}

/// Convert a `kurbo::BezPath` to a `norad::Contour`.
pub fn to_contour(path: &BezPath) -> Result<Contour, TraceError> {
    let els = path.elements();
    if els.is_empty() {
        return Err(TraceError::EmptyContour);
    }

    let first = match els.first() {
        Some(PathEl::MoveTo(p)) => *p,
        _ => {
            return Err(TraceError::InvalidPath(
                "path must start with MoveTo".into(),
            ))
        }
    };

    let mut points: Vec<ContourPoint> = Vec::new();

    for el in els.iter().skip(1) {
        match *el {
            PathEl::LineTo(p) => {
                points.push(pt(p, PointType::Line, false));
            }
            PathEl::CurveTo(a, b, p) => {
                points.push(pt(a, PointType::OffCurve, false));
                points.push(pt(b, PointType::OffCurve, false));
                points.push(pt(p, PointType::Curve, true));
            }
            PathEl::QuadTo(a, p) => {
                points.push(pt(a, PointType::OffCurve, false));
                points.push(pt(p, PointType::QCurve, true));
            }
            PathEl::ClosePath => {}
            PathEl::MoveTo(_) => {
                return Err(TraceError::InvalidPath("unexpected MoveTo mid-path".into()))
            }
        }
    }

    // First point type comes from the closing segment.
    let closing_type = els
        .iter()
        .rev()
        .find(|e| !matches!(e, PathEl::ClosePath))
        .map(|e| match e {
            PathEl::CurveTo(..) => PointType::Curve,
            PathEl::QuadTo(..) => PointType::QCurve,
            _ => PointType::Line,
        })
        .unwrap_or(PointType::Line);

    points.insert(0, pt(first, closing_type, false));

    Ok(Contour::new(points, None))
}

fn pt(p: kurbo::Point, typ: PointType, smooth: bool) -> ContourPoint {
    ContourPoint::new(p.x, p.y, typ, smooth, None, None)
}
