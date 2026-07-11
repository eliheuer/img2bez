// Copyright 2026 the img2bez Authors
// SPDX-License-Identifier: Apache-2.0 OR MIT

//! The trace pipeline: preprocessing, vectorization, cleanup, and
//! placement.

pub(crate) mod cleanup;
pub mod placement;
pub(crate) mod preprocess;
pub(crate) mod vectorize;
