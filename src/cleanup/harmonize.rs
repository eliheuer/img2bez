//! G2 curvature continuity harmonization.
//!
//! At smooth joins between cubic Beziers, the tangent direction is
//! continuous (G1) but the curvature magnitude may jump. This creates
//! visible "kinks" where the curve seems to change speed. G2 harmonization
//! adjusts handle lengths so curvature matches on both sides of each
//! smooth join.
//!
//! Math: For a cubic P0,P1,P2,P3, curvature at P0 is:
//!   κ = (2/3) * |cross(P1-P0, P2-P1)| / |P1-P0|³
//! Since cross(u,u) = 0, this simplifies to:
//!   κ = (2/3) * |cross(unit_handle, next_control - on_curve)| / handle_len²
//!
//! To achieve a target curvature κ_t at a join, we solve:
//!   handle_len = sqrt(C / κ_t)  where C = (2/3)*|cross(unit_handle, next_control - on_curve)|

use kurbo::{BezPath, PathEl, Point, Vec2};

/// Maximum angle (radians) between opposing handles to consider a join "smooth".
const SMOOTH_ANGLE_THRESHOLD: f64 = 30.0 * std::f64::consts::PI / 180.0;

/// Minimum handle length (font units) below which we skip harmonization.
const MIN_HANDLE_LEN: f64 = 2.0;

/// Minimum curvature constant C below which harmonization is skipped
/// (nearly-straight segments where curvature is ill-defined).
const MIN_CURVATURE_CONST: f64 = 1.0;

/// Maximum ratio by which a handle length can change in one iteration.
/// Prevents over-adjustment that distorts the curve shape.
const MAX_ADJUSTMENT_RATIO: f64 = 1.5;

/// Harmonize curvature at smooth joins in the path.
///
/// For each pair of consecutive cubics sharing a smooth on-curve point,
/// adjusts the shorter handle to reduce the curvature mismatch.
/// Runs `iterations` passes to propagate adjustments.
/// Adjusted positions are snapped to `grid` (0 = no snap).
pub fn harmonize(path: &BezPath, iterations: usize, grid: f64) -> BezPath {
    let mut elements: Vec<PathEl> = path.elements().to_vec();

    for _ in 0..iterations {
        harmonize_pass(&mut elements, grid);
    }

    BezPath::from_vec(elements)
}

/// Snap a coordinate to the grid (0 = no snapping).
fn snap(v: f64, grid: f64) -> f64 {
    if grid > 0.0 {
        (v / grid).round() * grid
    } else {
        v
    }
}

/// One harmonization pass over all smooth joins.
fn harmonize_pass(elements: &mut [PathEl], grid: f64) {
    let n = elements.len();
    if n < 3 {
        return;
    }

    // Build index: for each element, what's the on-curve point?
    let mut on_curves: Vec<Point> = vec![Point::ZERO; n];
    for (i, el) in elements.iter().enumerate() {
        match el {
            PathEl::MoveTo(p) | PathEl::LineTo(p) => on_curves[i] = *p,
            PathEl::CurveTo(_, _, p) => on_curves[i] = *p,
            PathEl::QuadTo(_, p) => on_curves[i] = *p,
            PathEl::ClosePath => {
                // Find the MoveTo for this contour
                for j in (0..i).rev() {
                    if let PathEl::MoveTo(p) = elements[j] {
                        on_curves[i] = p;
                        break;
                    }
                }
            }
        }
    }

    // For each pair of consecutive cubics, check if it's a smooth join.
    // We need to handle ClosePath wrapping: when a contour closes,
    // the last element connects to the first element after MoveTo.
    for i in 1..n {
        // Find the "previous" cubic and "current" cubic at this join.
        let (prev_idx, curr_idx) = if let PathEl::ClosePath = elements[i] {
            // ClosePath: the "current" cubic is the first element after MoveTo
            let move_idx = find_move_to(elements, i);
            if move_idx + 1 < n {
                (i - 1, move_idx + 1)
            } else {
                continue;
            }
        } else {
            (i - 1, i)
        };

        // Both must be CurveTo
        let (prev_c1, prev_c2, prev_p3) = match elements[prev_idx] {
            PathEl::CurveTo(c1, c2, p3) => (c1, c2, p3),
            _ => continue,
        };
        let (curr_c1, curr_c2, _curr_p3) = match elements[curr_idx] {
            PathEl::CurveTo(c1, c2, _p3) => (c1, c2, _p3),
            _ => continue,
        };

        // The shared on-curve point
        let join_pt = prev_p3;

        // Incoming handle: from join_pt toward prev_c2
        let handle_in = Vec2::new(prev_c2.x - join_pt.x, prev_c2.y - join_pt.y);
        let h_in = handle_in.length();

        // Outgoing handle: from join_pt toward curr_c1
        let handle_out = Vec2::new(curr_c1.x - join_pt.x, curr_c1.y - join_pt.y);
        let h_out = handle_out.length();

        if h_in < MIN_HANDLE_LEN || h_out < MIN_HANDLE_LEN {
            continue; // Degenerate handles, skip
        }

        // Check if this is a smooth join (handles roughly opposite)
        let unit_in = Vec2::new(handle_in.x / h_in, handle_in.y / h_in);
        let unit_out = Vec2::new(handle_out.x / h_out, handle_out.y / h_out);
        let dot = unit_in.x * unit_out.x + unit_in.y * unit_out.y;

        // For a smooth join, handles should point in opposite directions
        // (dot product close to -1). Allow some tolerance.
        if dot > -SMOOTH_ANGLE_THRESHOLD.cos() {
            continue; // Not a smooth join (corner)
        }

        // Compute curvature constants:
        // C_in = (2/3) * |cross(unit_in, prev_c1 - join_pt)|
        // C_out = (2/3) * |cross(unit_out, curr_c2 - join_pt)|
        let v_in = Vec2::new(prev_c1.x - join_pt.x, prev_c1.y - join_pt.y);
        let v_out = Vec2::new(curr_c2.x - join_pt.x, curr_c2.y - join_pt.y);

        let cross_in = (unit_in.x * v_in.y - unit_in.y * v_in.x).abs();
        let cross_out = (unit_out.x * v_out.y - unit_out.y * v_out.x).abs();

        let c_in = (2.0 / 3.0) * cross_in;
        let c_out = (2.0 / 3.0) * cross_out;

        if c_in < MIN_CURVATURE_CONST || c_out < MIN_CURVATURE_CONST {
            continue; // Nearly-straight segment, curvature ill-defined
        }

        // Curvature on each side
        let kappa_in = c_in / (h_in * h_in);
        let kappa_out = c_out / (h_out * h_out);

        // Skip if already well-matched (within 10%)
        let ratio = kappa_in / kappa_out;
        if ratio > 0.9 && ratio < 1.1 {
            continue;
        }

        // Balanced approach: target the geometric mean curvature and
        // blend toward harmonized positions at BLEND_FACTOR strength.
        // This preserves most of the fitted shape while reducing kinks.
        const BLEND_FACTOR: f64 = 0.3;

        let kappa_target = (kappa_in * kappa_out).sqrt();

        let h_in_ideal = (c_in / kappa_target).sqrt();
        let h_out_ideal = (c_out / kappa_target).sqrt();

        // Blend: lerp between original and harmonized handle lengths
        let h_in_new = h_in + BLEND_FACTOR * (h_in_ideal - h_in);
        let h_out_new = h_out + BLEND_FACTOR * (h_out_ideal - h_out);

        // Clamp to prevent extreme adjustments
        let h_in_clamped = h_in_new.clamp(
            h_in / MAX_ADJUSTMENT_RATIO,
            h_in * MAX_ADJUSTMENT_RATIO,
        );
        let h_out_clamped = h_out_new.clamp(
            h_out / MAX_ADJUSTMENT_RATIO,
            h_out * MAX_ADJUSTMENT_RATIO,
        );

        // Apply: move control points along existing handle directions
        let new_prev_c2 = Point::new(
            snap(join_pt.x + unit_in.x * h_in_clamped, grid),
            snap(join_pt.y + unit_in.y * h_in_clamped, grid),
        );
        let new_curr_c1 = Point::new(
            snap(join_pt.x + unit_out.x * h_out_clamped, grid),
            snap(join_pt.y + unit_out.y * h_out_clamped, grid),
        );

        elements[prev_idx] = PathEl::CurveTo(prev_c1, new_prev_c2, prev_p3);
        elements[curr_idx] = PathEl::CurveTo(new_curr_c1, curr_c2, _curr_p3);
    }
}

/// Find the MoveTo that starts the contour containing element at `idx`.
fn find_move_to(elements: &[PathEl], idx: usize) -> usize {
    for j in (0..idx).rev() {
        if let PathEl::MoveTo(_) = elements[j] {
            return j;
        }
    }
    0
}
