// Copyright 2026 the img2bez Authors
// SPDX-License-Identifier: Apache-2.0 OR MIT

//! Multi-master and font-level concerns: master sets, interpolation
//! compatibility, and the experimental refit pass.

pub mod compat;
#[cfg(feature = "ufo")]
pub mod masters;
pub(crate) mod masters_flow;
#[cfg(feature = "experimental-refit")]
pub mod refit;
