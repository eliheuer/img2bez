// Copyright 2026 the img2bez Authors
// SPDX-License-Identifier: Apache-2.0 OR MIT

//! Decision logging for ML training data. `IMG2BEZ_LOG_DECISIONS=<path>`
//! makes each instrumented gate append one JSON line per candidate
//! (features + decision); `IMG2BEZ_LOG_TAG` names the source image for
//! per-glyph joins. Off by default and free when off (one atomic lookup).
//! Hand-formatted records — the library does not carry serde_json.

use std::fs::{File, OpenOptions};
use std::io::Write;
use std::sync::{Mutex, OnceLock};

static SINK: OnceLock<Option<Mutex<File>>> = OnceLock::new();

fn sink() -> Option<&'static Mutex<File>> {
    SINK.get_or_init(|| {
        std::env::var("IMG2BEZ_LOG_DECISIONS").ok().and_then(|p| {
            OpenOptions::new()
                .create(true)
                .append(true)
                .open(p)
                .ok()
                .map(Mutex::new)
        })
    })
    .as_ref()
}

/// Cheap guard so call sites can skip feature marshalling entirely.
pub(crate) fn enabled() -> bool {
    sink().is_some()
}

fn esc(s: &str) -> String {
    s.replace('\\', "\\\\").replace('"', "\\\"")
}

fn num(v: f64) -> String {
    if v.is_finite() {
        format!("{v:.4}")
    } else {
        "null".to_string()
    }
}

/// Append one decision record. `pos` is in the gate's own coordinate space
/// (pixels for fit/refine gates, font units for cleanup gates — implied by
/// `kind`). `fields` are the features the gate decided on; `decision` is
/// what it chose.
pub(crate) fn log(
    kind: &str,
    pos: Option<(f64, f64)>,
    fields: &[(&str, f64)],
    decision: &str,
) {
    let Some(m) = sink() else { return };
    let tag = std::env::var("IMG2BEZ_LOG_TAG").unwrap_or_default();
    let mut s = String::with_capacity(256);
    s.push_str("{\"tag\":\"");
    s.push_str(&esc(&tag));
    s.push_str("\",\"kind\":\"");
    s.push_str(kind);
    s.push('"');
    if let Some((x, y)) = pos {
        s.push_str(&format!(",\"x\":{},\"y\":{}", num(x), num(y)));
    }
    for (k, v) in fields {
        s.push_str(&format!(",\"{k}\":{}", num(*v)));
    }
    s.push_str(",\"decision\":\"");
    s.push_str(decision);
    s.push_str("\"}\n");
    if let Ok(mut f) = m.lock() {
        let _ = f.write_all(s.as_bytes());
    }
}
