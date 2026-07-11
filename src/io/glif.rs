// Copyright 2026 the img2bez Authors
// SPDX-License-Identifier: Apache-2.0 OR MIT

//! Browser-safe GLIF serialization for traced glyph outlines.
//!
//! This module deliberately does not depend on `norad`, so applications can
//! compile `img2bez` into WASM with `default-features = false`. It is a thin
//! serializer over the canonical [`crate::Outline`]: the point stream and the
//! `smooth` flag are produced there, so GLIF, UFO, and the JSON model always
//! agree.

use crate::model::glyph::Glyph;
use crate::model::outline::PointKind;

/// Serialize a [`Glyph`] to UFO GLIF XML.
pub fn write_glif(glyph: &Glyph) -> String {
    let outline = &glyph.outline;
    let mut out = String::new();
    out.push_str("<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n");
    out.push_str(&format!(
        "<glyph name=\"{}\" format=\"2\">\n",
        escape_xml(&glyph.name)
    ));

    for hex in &glyph.unicodes {
        out.push_str(&format!("\t<unicode hex=\"{}\"/>\n", escape_xml(hex)));
    }

    out.push_str(&format!(
        "\t<advance width=\"{}\"/>\n",
        format_coord(glyph.advance.width)
    ));

    let has_points = outline.contours.iter().any(|c| !c.points.is_empty());
    if has_points {
        out.push_str("\t<outline>\n");
        for contour in &outline.contours {
            if contour.points.is_empty() {
                continue;
            }
            out.push_str("\t\t<contour>\n");
            for point in &contour.points {
                out.push_str("\t\t\t<point");
                out.push_str(&format!(
                    " x=\"{}\" y=\"{}\"",
                    format_coord(point.x),
                    format_coord(point.y)
                ));
                match point.kind {
                    PointKind::OffCurve => {}
                    PointKind::Line => out.push_str(" type=\"line\""),
                    PointKind::Curve => out.push_str(" type=\"curve\""),
                    PointKind::QCurve => out.push_str(" type=\"qcurve\""),
                    PointKind::Move => out.push_str(" type=\"move\""),
                }
                if point.smooth {
                    out.push_str(" smooth=\"yes\"");
                }
                out.push_str("/>\n");
            }
            out.push_str("\t\t</contour>\n");
        }
        out.push_str("\t</outline>\n");
    }

    out.push_str("</glyph>\n");
    out
}

/// Format a coordinate: integers print bare (`120`), other values keep at
/// most three decimals with trailing zeros trimmed.
///
/// This formatting is load-bearing: traced output is compared byte-for-byte
/// across surfaces (CLI, wasm, MCP) and against committed baselines, and it
/// must stay in lockstep with `num()` in `outline.rs`. Do not change the
/// rounding or trimming behavior.
pub(crate) fn format_coord(value: f64) -> String {
    let rounded = value.round();
    if (value - rounded).abs() < 1e-6 {
        return format!("{}", rounded as i64);
    }
    let mut s = format!("{value:.3}");
    while s.contains('.') && s.ends_with('0') {
        s.pop();
    }
    if s.ends_with('.') {
        s.pop();
    }
    s
}

/// Escape XML special characters for element and attribute content.
fn escape_xml(value: &str) -> String {
    value
        .replace('&', "&amp;")
        .replace('"', "&quot;")
        .replace('<', "&lt;")
        .replace('>', "&gt;")
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Outline;
    use crate::model::glyph::Glyph;
    use kurbo::BezPath;

    #[test]
    fn serializes_minimal_glif_without_norad() {
        let mut path = BezPath::new();
        path.move_to((0.0, 0.0));
        path.line_to((100.0, 0.0));
        path.line_to((100.0, 100.0));
        path.line_to((0.0, 100.0));
        path.close_path();

        let outline = Outline::from_bezpaths(&[path]);
        let glyph = Glyph::new("A", &['A'], 120.0, 1088.0, outline);
        let glif = write_glif(&glyph);
        assert!(glif.contains("<glyph name=\"A\" format=\"2\">"));
        assert!(glif.contains("<unicode hex=\"0041\"/>"));
        assert!(glif.contains("<advance width=\"120\"/>"));
        assert!(glif.contains("<contour>"));
        assert!(glif.contains("type=\"line\""));
    }
}

#[cfg(test)]
mod format_parity {
    /// `glif::format_coord` and `outline::num` are two deliberately
    /// hand-synced copies of the byte-identical output formatting rule.
    /// This test is the lockstep guard: if either drifts, downstream
    /// byte-identical invariants break silently.
    #[test]
    fn coord_formatters_agree() {
        let cases = [
            0.0,
            -0.0,
            1.0,
            -1.0,
            0.5,
            -0.5,
            2.25,
            1088.0,
            -256.0,
            123.456789,
            -987.654321,
            0.1 + 0.2,
            1e-9,
            -1e-9,
            1e6,
            33.333333333333336,
            0.30000000000000004,
        ];
        for v in cases {
            assert_eq!(
                super::format_coord(v),
                crate::model::outline::num(v),
                "formatters disagree for {v}"
            );
        }
    }
}
