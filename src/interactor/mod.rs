// SPDX-FileCopyrightText: 2024 Topola contributors
//
// SPDX-License-Identifier: MIT

pub mod activity;
pub mod interaction;
// `crate::interactor::interactor` is a deliberate layout: the inner module
// holds the module's central type alongside sibling submodules.
#[allow(clippy::module_inception)]
mod interactor;
pub mod route_plan;

pub use interactor::*;
