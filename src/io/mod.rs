// Copyright 2026 the img2bez Authors
// SPDX-License-Identifier: Apache-2.0 OR MIT

//! Input/output: image decoding, GLIF/UFO writing, and rendering.

pub(crate) mod bitmap;
pub(crate) mod glif;

#[cfg(feature = "ufo")]
pub mod eval;
#[cfg(feature = "render")]
pub mod render;
#[cfg(feature = "ufo")]
pub mod ufo;
