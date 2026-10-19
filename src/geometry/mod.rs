// SPDX-FileCopyrightText: 2024 Topola contributors
//
// SPDX-License-Identifier: MIT

#[macro_use]
// `crate::geometry::geometry` is a deliberate layout: the inner module
// holds the module's central type alongside sibling submodules.
#[allow(clippy::module_inception)]
mod geometry;
pub mod compound;
pub mod edit;
mod polygon;
pub mod primitive;
pub mod recording_with_rtree;
pub mod shape;
pub mod with_rtree;

pub use geometry::*;
