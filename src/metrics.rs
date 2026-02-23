//! Glyph positioning and advance width computation.

use kurbo::{Affine, BezPath, Shape, Vec2};

/// Shift paths so bottom sits on baseline, left at LSB.
pub fn reposition(paths: &[BezPath], lsb: f64) -> Vec<BezPath> {
    let mut min_x = f64::MAX;
    let mut min_y = f64::MAX;
    for path in paths {
        let bounds = path.bounding_box();
        min_x = min_x.min(bounds.x0);
        min_y = min_y.min(bounds.y0);
    }
    if min_x == f64::MAX {
        return paths.to_vec();
    }
    let dx = lsb - min_x;
    let dy = -min_y;
    paths.iter().map(|path| translate(path, dx, dy)).collect()
}

/// Compute advance width from bounding box + right sidebearing.
pub fn advance_from_bounds(paths: &[BezPath], rsb: f64) -> f64 {
    let mut max_x = f64::MIN;
    for path in paths {
        max_x = max_x.max(path.bounding_box().x1);
    }
    if max_x == f64::MIN {
        0.0
    } else {
        max_x + rsb
    }
}

fn translate(path: &BezPath, dx: f64, dy: f64) -> BezPath {
    let mut translated = path.clone();
    translated.apply_affine(Affine::translate(Vec2::new(dx, dy)));
    translated
}
