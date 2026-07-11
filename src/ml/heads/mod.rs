// Copyright 2026 the img2bez Authors
// SPDX-License-Identifier: Apache-2.0 OR MIT

//! Learned decision heads: tiny gradient-boosted tree ensembles replacing
//! hand-tuned thresholds at pipeline gates, trained offline (img2bez-data)
//! and exported as const arrays — deterministic, dependency-free tree-walks.
//! Each head is opt-in via env var while under evaluation; default behavior
//! is byte-identical with heads off.

mod corner_v1 {
    include!("corner_v1.rs");
}
mod site_v1 {
    include!("site_v1.rs");
}

/// Corner keep/drop head. `IMG2BEZ_CORNER_HEAD=1` enables it inside
/// `detect_corners`, refining only candidates past the minimal central turn
/// — the training distribution; it must not judge candidates below it.
pub(crate) fn corner_head_enabled() -> bool {
    match FORCE.load(std::sync::atomic::Ordering::Relaxed) {
        1 => false,
        2 => true,
        _ => {
            static ON: std::sync::OnceLock<bool> = std::sync::OnceLock::new();
            *ON.get_or_init(|| {
                std::env::var("IMG2BEZ_CORNER_HEAD").is_ok_and(|v| v != "0")
            })
        }
    }
}

/// 0 = follow the env var, 1 = forced off, 2 = forced on.
static FORCE: std::sync::atomic::AtomicU8 = std::sync::atomic::AtomicU8::new(0);

/// Programmatic override for hosts without an environment (wasm demos).
/// **Process-global**: applies to every trace on every thread until changed;
/// concurrent callers cannot use different values (per-call option planned
/// for 0.2). `None` restores env-var behavior. Experimental A/B API; may
/// change or vanish.
pub fn set_corner_head(enabled: Option<bool>) {
    FORCE.store(
        match enabled {
            None => 0,
            Some(false) => 1,
            Some(true) => 2,
        },
        std::sync::atomic::Ordering::Relaxed,
    );
}

/// Raw ensemble score for a corner candidate; `>= 0.0` means corner.
/// Features (order fixed by the exporter): |central_deg|, |window_deg|,
/// conc, sigma, smear, cusp, shallow_flanked.
pub(crate) fn corner_score(features: &[f64; 7]) -> f64 {
    let mut score = corner_v1::PRIOR;
    for &root in corner_v1::ROOTS.iter() {
        let mut i = root as usize;
        loop {
            let (feat, thresh, left, right, value) = corner_v1::NODES[i];
            if feat < 0 {
                score += value;
                break;
            }
            i = if features[feat as usize] <= thresh {
                left as usize
            } else {
                right as usize
            };
        }
    }
    score
}

/// Site head (5-class zone classifier: corner/corner2/bracket/point/smooth).
/// `IMG2BEZ_SITE_HEAD=1` replaces the per-candidate corner gate with a
/// per-zone decision in `detect_corners`.
pub(crate) fn site_head_enabled() -> bool {
    match SITE_FORCE.load(std::sync::atomic::Ordering::Relaxed) {
        1 => false,
        2 => true,
        _ => {
            static ON: std::sync::OnceLock<bool> = std::sync::OnceLock::new();
            *ON.get_or_init(|| {
                std::env::var("IMG2BEZ_SITE_HEAD").is_ok_and(|v| v != "0")
            })
        }
    }
}

static SITE_FORCE: std::sync::atomic::AtomicU8 =
    std::sync::atomic::AtomicU8::new(0);

/// Programmatic override for hosts without an environment (wasm demos).
/// **Process-global**: applies to every trace on every thread until changed;
/// concurrent callers cannot use different values (per-call option planned
/// for 0.2). `None` restores env-var behavior. Experimental, like
/// [`set_corner_head`](crate::set_corner_head).
pub fn set_site_head(enabled: Option<bool>) {
    SITE_FORCE.store(
        match enabled {
            None => 0,
            Some(false) => 1,
            Some(true) => 2,
        },
        std::sync::atomic::Ordering::Relaxed,
    );
}

/// Raw per-class scores for a curvature zone; take the argmax. Class
/// order: corner, corner2 (a designed corner PAIR: serif feet, flat
/// terminals), bracket, point, smooth. Features (exporter order):
/// n_cands, max|central|deg, max|window|deg, max_conc, span_px, sigma,
/// smear, any_cusp, any_shallow.
pub(crate) fn site_scores(features: &[f64; 9]) -> [f64; 5] {
    let walk = |roots: &[u32]| -> f64 {
        let mut sum = 0.0;
        for &root in roots {
            let mut i = root as usize;
            loop {
                let (feat, thresh, left, right, value) = site_v1::NODES[i];
                if feat < 0 {
                    sum += value;
                    break;
                }
                i = if features[feat as usize] <= thresh {
                    left as usize
                } else {
                    right as usize
                };
            }
        }
        sum
    };
    [
        site_v1::PRIORS[0] + walk(&site_v1::ROOTS_CORNER),
        site_v1::PRIORS[1] + walk(&site_v1::ROOTS_CORNER2),
        site_v1::PRIORS[2] + walk(&site_v1::ROOTS_BRACKET),
        site_v1::PRIORS[3] + walk(&site_v1::ROOTS_POINT),
        site_v1::PRIORS[4] + walk(&site_v1::ROOTS_SMOOTH),
    ]
}

#[cfg(test)]
mod tests {
    /// Site-head golden parity: 5-class raw scores match sklearn's
    /// decision_function on the exported fixture.
    #[test]
    fn site_head_matches_sklearn() {
        let raw = include_str!("../../../tests/data/site_v1_parity.json");
        let strip = |s: &str| -> Vec<f64> {
            s.split(|c: char| {
                !(c.is_ascii_digit()
                    || c == '.'
                    || c == '-'
                    || c == 'e'
                    || c == 'E'
                    || c == '+')
            })
            .filter(|t| !t.is_empty() && *t != "-" && *t != "+")
            .filter_map(|t| t.parse().ok())
            .collect()
        };
        let feat_part = raw
            .split("\"features\":")
            .nth(1)
            .unwrap()
            .split("\"scores\":")
            .next()
            .unwrap();
        let score_part = raw.split("\"scores\":").nth(1).unwrap();
        let fv = strip(feat_part);
        let sv = strip(score_part);
        assert_eq!(fv.len() / 9, sv.len() / 5, "fixture shape");
        for k in 0..(sv.len() / 5) {
            let mut f = [0.0; 9];
            f.copy_from_slice(&fv[k * 9..k * 9 + 9]);
            let got = super::site_scores(&f);
            let want = &sv[k * 5..k * 5 + 5];
            for j in 0..5 {
                assert!(
                    (got[j] - want[j]).abs() < 1e-6,
                    "row {k} class {j}: got {}, want {}",
                    got[j],
                    want[j]
                );
            }
            let am = |v: &[f64]| {
                (0..5)
                    .max_by(|&a, &b| v[a].partial_cmp(&v[b]).unwrap())
                    .unwrap()
            };
            assert_eq!(am(&got), am(want), "row {k} argmax");
        }
    }

    /// Golden parity with the training-side sklearn model: identical
    /// decisions on the exported fixture, scores within float noise.
    #[test]
    fn corner_head_matches_sklearn() {
        let raw = include_str!("../../../tests/data/corner_v1_parity.json");
        let strip = |s: &str| -> Vec<f64> {
            s.split(|c: char| {
                !(c.is_ascii_digit()
                    || c == '.'
                    || c == '-'
                    || c == 'e'
                    || c == 'E'
                    || c == '+')
            })
            .filter(|t| !t.is_empty() && *t != "-" && *t != "+")
            .filter_map(|t| t.parse().ok())
            .collect()
        };
        let feat_part = raw
            .split("\"features\":")
            .nth(1)
            .unwrap()
            .split("\"scores\":")
            .next()
            .unwrap();
        let score_part = raw.split("\"scores\":").nth(1).unwrap();
        let fv = strip(feat_part);
        let scores = strip(score_part);
        assert_eq!(fv.len(), scores.len() * 7, "fixture shape");
        for (k, expect) in scores.iter().enumerate() {
            let mut f = [0.0; 7];
            f.copy_from_slice(&fv[k * 7..k * 7 + 7]);
            let got = super::corner_score(&f);
            assert!(
                (got - expect).abs() < 1e-6,
                "row {k}: got {got}, expected {expect}"
            );
            assert_eq!(got >= 0.0, *expect >= 0.0, "row {k} decision");
        }
    }
}
