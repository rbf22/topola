// SPDX-FileCopyrightText: 2024 Topola contributors
//
// SPDX-License-Identifier: MIT

#[macro_use]
pub mod graph;
pub mod band;
pub mod bend;
mod cane;
pub mod dot;
// `crate::drawing::drawing` is a deliberate layout: the inner module
// holds the module's central type alongside sibling submodules.
#[allow(clippy::module_inception)]
mod drawing;
pub mod gear;
pub mod guide;
pub mod head;
pub mod loose;
pub mod primitive;
mod query;
pub use specctra_core::rules;
pub mod seg;

pub use cane::Cane;
pub use drawing::*;
