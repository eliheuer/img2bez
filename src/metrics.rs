//! Glyph positioning and advance width computation.

use kurbo::{Affine, BezPath, Shape, Vec2};

/// Shift paths so bottom sits on baseline, left at LSB.
///
/// When `y_offset` is non-zero the caller has already placed the glyph
/// in font-unit coordinates with an explicit baseline; only the x
/// (LSB) shift is applied.  When `y_offset` is zero the glyph is
/// assumed to start at the image bottom and needs shifting up to y=0.
///
/// When `grid > 0`, shifts are rounded to the grid so coordinates
/// that were on-grid before repositioning stay on-grid after.
///
/// Returns `(repositioned_paths, (dx, dy))` where (dx, dy) is the
/// translation that was applied.
pub fn reposition(
    paths: &[BezPath],
    lsb: f64,
    grid: i32,
    y_offset: f64,
) -> (Vec<BezPath>, (f64, f64)) {
    let mut min_x = f64::MAX;
    let mut min_y = f64::MAX;
    // Use on-curve point extremes (not tight bbox) to avoid fractional shifts.
    for path in paths {
        for el in path.elements() {
            let pts: &[kurbo::Point] = match el {
                kurbo::PathEl::MoveTo(p) | kurbo::PathEl::LineTo(p) => std::slice::from_ref(p),
                kurbo::PathEl::CurveTo(_, _, p) => std::slice::from_ref(p),
                kurbo::PathEl::QuadTo(_, p) => std::slice::from_ref(p),
                kurbo::PathEl::ClosePath => &[],
            };
            for p in pts {
                min_x = min_x.min(p.x);
                min_y = min_y.min(p.y);
            }
        }
    }
    if min_x == f64::MAX {
        return (paths.to_vec(), (0.0, 0.0));
    }
    let mut dx = lsb - min_x;
    // Only auto-shift y when no explicit y_offset was given.
    // With y_offset set, the glyph is already in the correct font position.
    let mut dy = if y_offset == 0.0 { -min_y } else { 0.0 };
    if grid > 0 {
        let g = grid as f64;
        dx = (dx / g).round() * g;
        dy = (dy / g).round() * g;
    }
    (
        paths.iter().map(|path| translate(path, dx, dy)).collect(),
        (dx, dy),
    )
}

/// Compute advance width from bounding box + right sidebearing.
pub fn advance_from_bounds(paths: &[BezPath], rsb: f64) -> f64 {
    let mut max_x = f64::MIN;
    for path in paths {
        max_x = max_x.max(path.bounding_box().x1);
    }
    if max_x == f64::MIN { 0.0 } else { max_x + rsb }
}

/// Translate all points in a BezPath by (dx, dy).
fn translate(path: &BezPath, dx: f64, dy: f64) -> BezPath {
    let mut translated = path.clone();
    translated.apply_affine(Affine::translate(Vec2::new(dx, dy)));
    translated
}
