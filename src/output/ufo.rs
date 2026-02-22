use kurbo::{BezPath, PathEl};
use norad::{Contour, ContourPoint, Glyph, PointType};

use crate::config::TracingConfig;
use crate::error::TraceError;
use crate::TraceResult;

/// Convert a `TraceResult` to a `norad::Glyph` ready for insertion into a UFO.
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
        let contour = bezpath_to_contour(path)?;
        glyph.contours.push(contour);
    }

    Ok(glyph)
}

/// Convert a `kurbo::BezPath` to a `norad::Contour`.
///
/// Handles the UFO convention where the first point's type indicates
/// the closing segment type (not an explicit MoveTo).
pub fn bezpath_to_contour(path: &BezPath) -> Result<Contour, TraceError> {
    let elements = path.elements();
    if elements.is_empty() {
        return Err(TraceError::EmptyContour);
    }

    let mut points: Vec<ContourPoint> = Vec::new();

    // Skip the initial MoveTo — we'll add the first point at the end
    // once we know the closing segment type.
    let first_pt = match elements.first() {
        Some(PathEl::MoveTo(p)) => *p,
        _ => return Err(TraceError::InvalidPath("path must start with MoveTo".into())),
    };

    for el in elements.iter().skip(1) {
        match *el {
            PathEl::LineTo(p) => {
                points.push(ContourPoint::new(
                    p.x,
                    p.y,
                    PointType::Line,
                    false,
                    None,
                    None,
                ));
            }
            PathEl::CurveTo(p1, p2, p3) => {
                points.push(ContourPoint::new(
                    p1.x,
                    p1.y,
                    PointType::OffCurve,
                    false,
                    None,
                    None,
                ));
                points.push(ContourPoint::new(
                    p2.x,
                    p2.y,
                    PointType::OffCurve,
                    false,
                    None,
                    None,
                ));
                points.push(ContourPoint::new(
                    p3.x,
                    p3.y,
                    PointType::Curve,
                    true,
                    None,
                    None,
                ));
            }
            PathEl::QuadTo(p1, p2) => {
                points.push(ContourPoint::new(
                    p1.x,
                    p1.y,
                    PointType::OffCurve,
                    false,
                    None,
                    None,
                ));
                points.push(ContourPoint::new(
                    p2.x,
                    p2.y,
                    PointType::QCurve,
                    true,
                    None,
                    None,
                ));
            }
            PathEl::ClosePath => {}
            PathEl::MoveTo(_) => {
                return Err(TraceError::InvalidPath(
                    "unexpected MoveTo in middle of path".into(),
                ));
            }
        }
    }

    // Determine the first point's type from the closing segment.
    // The closing segment goes from the last on-curve point back to first_pt.
    // Its type is determined by the last explicit segment in the path.
    let last_seg = elements
        .iter()
        .rev()
        .find(|e| !matches!(e, PathEl::ClosePath));

    let first_type = match last_seg {
        Some(PathEl::LineTo(_)) | Some(PathEl::MoveTo(_)) | Some(PathEl::ClosePath) | None => {
            PointType::Line
        }
        Some(PathEl::CurveTo(..)) => PointType::Curve,
        Some(PathEl::QuadTo(..)) => PointType::QCurve,
    };

    // Insert the first point at the beginning
    points.insert(
        0,
        ContourPoint::new(first_pt.x, first_pt.y, first_type, false, None, None),
    );

    Ok(Contour::new(points, None))
}
