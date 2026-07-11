// Copyright 2026 the img2bez Authors
// SPDX-License-Identifier: Apache-2.0 OR MIT

//! Multi-master orchestration: trace a master set jointly (the joint masters
//! pipeline), with the independent-trace + reconcile fallback, and the
//! placed-per-master variant.

use crate::compat::{
    self, CompatibilityReport, ContourReport, make_compatible,
};
use crate::io::bitmap;
use crate::model::config::TraceOptions;
use crate::model::error::TraceError;
use crate::model::outline::{Outline, PointKind};
use crate::pipeline::preprocess::preprocess_luma;
use crate::pipeline::vectorize;
use crate::placement::{self, PlacedGlyph, PlacementOptions, PlacementReport};
use crate::{
    PlacementGeom, normalize_master_starts, placement_prework, trace,
    trace_place,
};

/// Trace several master images of the same glyph into an
/// interpolation-compatible set.
///
/// Masters are traced *jointly*: outline structure is decided once across the
/// set, then each master is fitted to that shared plan, so the set
/// interpolates by construction. When the joint path cannot run, each image
/// is traced independently and [`compat::make_compatible`] reconciles the
/// result, flagged `low_confidence`.
///
/// ```no_run
/// use img2bez::{trace_masters, TraceOptions};
///
/// let regular = std::fs::read("G-regular.png")?;
/// let bold = std::fs::read("G-bold.png")?;
/// let (outlines, report) =
///     trace_masters(&[&regular, &bold], &TraceOptions::default())?;
/// assert!(report.compatible);
/// assert_eq!(outlines.len(), 2);
/// # Ok::<(), Box<dyn std::error::Error>>(())
/// ```
///
/// # Errors
///
/// As [`trace`], for whichever image fails first. Master sets that cannot be
/// made compatible are NOT an error: the returned
/// [`CompatibilityReport::compatible`] is `false` and its `note` says why.
pub fn trace_masters(
    images: &[&[u8]],
    opts: &TraceOptions,
) -> Result<(Vec<Outline>, CompatibilityReport), TraceError> {
    if images.len() >= 2 {
        let raws = images
            .iter()
            .map(|bytes| bitmap::load_luma_bytes(bytes))
            .collect::<Result<Vec<_>, _>>()?;
        let prepped: Vec<(image::GrayImage, u8, (bool, bool))> = raws
            .into_iter()
            .map(|raw| preprocess_luma(raw, opts))
            .collect();
        let configs: Vec<TraceOptions> = prepped
            .iter()
            .map(|&(_, _, (upscaled, soft))| {
                let mut c = opts.clone();
                c.corner_smear = upscaled;
                c.soft_source = soft;
                c
            })
            .collect();
        let pre: Vec<(image::GrayImage, u8)> = prepped
            .into_iter()
            .map(|(img, thr, _)| (img, thr))
            .collect();
        if let Some(result) = joint_outlines(&pre, &configs) {
            return Ok(result);
        }
    }
    // Fallback: independent traces reconciled after the fact.
    let outlines = images
        .iter()
        .map(|bytes| trace(bytes, opts))
        .collect::<Result<Vec<_>, _>>()?;
    Ok(make_compatible(&outlines))
}

/// Run the joint trace over preprocessed masters; `None` means the joint
/// path declined and the caller falls back to independent trace + reconcile.
fn joint_outlines(
    pre: &[(image::GrayImage, u8)],
    configs: &[TraceOptions],
) -> Option<(Vec<Outline>, CompatibilityReport)> {
    let jm: Vec<vectorize::joint::JointMaster> = pre
        .iter()
        .zip(configs)
        .map(
            |((gray, threshold), config)| vectorize::joint::JointMaster {
                gray,
                threshold: *threshold,
                config,
            },
        )
        .collect();
    let out = match vectorize::joint::trace_masters_joint(&jm) {
        Ok(out) => out,
        Err(e) => {
            if configs[0].verbose
                || std::env::var("IMG2BEZ_DEBUG_JOINT").is_ok()
            {
                eprintln!("  Joint       declined ({e}); falling back");
            }
            return None;
        }
    };
    let mut outlines: Vec<Outline> = out
        .paths
        .iter()
        .map(|paths| Outline::from_bezpaths(paths))
        .collect();
    normalize_master_starts(&mut outlines, configs[0].rtl_start);
    if outlines.iter().any(|o| o.contours.is_empty())
        || compat::check(&outlines).is_err()
    {
        // The joint invariant did not hold; fall back to independent tracing.
        return None;
    }
    let contour_count = outlines[0].contours.len();
    let contours = outlines[0]
        .contours
        .iter()
        .map(|c| ContourReport {
            final_on_curve: c
                .points
                .iter()
                .filter(|p| p.kind != PointKind::OffCurve)
                .count(),
            inserted_per_master: vec![0; outlines.len()],
            low_confidence: false,
        })
        .collect();
    let report = CompatibilityReport {
        master_count: outlines.len(),
        contour_count,
        compatible: true,
        inserted_points: out.inserted_per_master.iter().sum(),
        low_confidence: out.low_confidence,
        contours,
        note: None,
    };
    Some((outlines, report))
}

/// [`trace_place`] over a whole master set, traced jointly (see
/// [`trace_masters`]) and then placed per master. Returns the placed masters
/// in input order plus the compatibility report.
///
/// # Errors
///
/// As [`trace_place`], for whichever master fails first.
pub fn trace_place_masters(
    inputs: &[(&[u8], &PlacementOptions)],
    opts: &TraceOptions,
) -> Result<
    (Vec<(PlacedGlyph, PlacementReport)>, CompatibilityReport),
    TraceError,
> {
    // Same per-master `placement_prework` as `trace_place`, plus the
    // preprocessing joint tracing needs up front.
    let mut pre: Vec<(image::GrayImage, u8)> = Vec::with_capacity(inputs.len());
    let mut configs: Vec<TraceOptions> = Vec::with_capacity(inputs.len());
    let mut geom: Vec<PlacementGeom> = Vec::with_capacity(inputs.len());
    for (bytes, placement) in inputs {
        let raw = bitmap::load_luma_bytes(bytes)?;
        let (mut topts, g) = placement_prework(&raw, opts, placement)?;
        let (img, thr, (upscaled, soft)) = preprocess_luma(raw, &topts);
        topts.corner_smear = upscaled;
        topts.soft_source = soft;
        pre.push((img, thr));
        configs.push(topts);
        geom.push(g);
    }

    // Joint tracing needs at least two masters; 0/1 inputs take the
    // independent fallback below.
    if inputs.len() >= 2
        && let Some((outlines, report)) = joint_outlines(&pre, &configs)
    {
        let placed = outlines
            .iter()
            .zip(inputs)
            .zip(&geom)
            .map(|((outline, (_, placement)), &(dims, ink_px, scale))| {
                placement::position(outline, placement, dims, ink_px, scale)
            })
            .collect();
        return Ok((placed, report));
    }

    // Fallback: trace each independently, place, then reconcile the placed
    // outlines.
    let mut placed: Vec<(PlacedGlyph, PlacementReport)> = inputs
        .iter()
        .map(|(bytes, placement)| trace_place(bytes, opts, placement))
        .collect::<Result<Vec<_>, _>>()?;
    let traced: Vec<Outline> =
        placed.iter().map(|(p, _)| p.outline.clone()).collect();
    let (mut outlines, report) = make_compatible(&traced);
    normalize_master_starts(&mut outlines, opts.rtl_start);
    for ((p, _), o) in placed.iter_mut().zip(outlines) {
        p.outline = o;
    }
    Ok((placed, report))
}
