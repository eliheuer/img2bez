// Copyright 2026 the img2bez Authors
// SPDX-License-Identifier: Apache-2.0 OR MIT

//! Core data model: outlines, glyphs, options, and errors. The types
//! external users need are re-exported from the crate root.

pub(crate) mod config;
pub(crate) mod error;
pub(crate) mod geom;
pub(crate) mod glyph;
pub(crate) mod outline;
pub(crate) mod trace_mode;
