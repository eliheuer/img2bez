// Copyright 2026 the img2bez Authors
// SPDX-License-Identifier: Apache-2.0 OR MIT

//! Measurement and learned decisions: image statistics, output judging,
//! decision heads, and ML data logging.

pub(crate) mod heads;
pub(crate) mod image_stats;
pub mod judge;
pub(crate) mod mldata;
