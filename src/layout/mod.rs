// SPDX-FileCopyrightText: 2024 Topola contributors
//
// SPDX-License-Identifier: MIT

//! Layout module for handling board geometry.

mod collect_bands;
// `crate::layout::layout` is a deliberate layout: the inner module
// holds the module's central type alongside sibling submodules.
#[allow(clippy::module_inception)]
mod layout;
pub mod poly;
pub mod query;
pub mod via;

pub use layout::*;
