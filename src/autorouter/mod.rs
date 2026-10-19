// SPDX-FileCopyrightText: 2024 Topola contributors
//
// SPDX-License-Identifier: MIT

pub mod anneal;
pub mod anterouter;
// `crate::autorouter::autorouter` is a deliberate layout: the inner module
// holds the module's central type alongside sibling submodules.
#[allow(clippy::module_inception)]
mod autorouter;
pub mod carve;
pub mod compass_direction;
pub mod connected_components;
pub mod critical_autoroute;
pub mod criticality;
pub mod escape_stubs;
pub mod execution;
pub mod history;
pub mod invoker;
pub mod measure_length;
pub mod multilayer_autoroute;
pub mod multilayer_preconfigurer;
pub mod multilayer_reconfigurator;
pub mod multilayer_reconfigurer;
pub mod pad_entry;
pub mod place_via;
pub mod planar_autoroute;
pub mod planar_preconfigurer;
pub mod planar_reconfigurator;
pub mod planar_reconfigurer;
pub mod pointroute;
pub mod probe;
pub mod ratline;
pub mod ratsnest;
pub mod ratsnests;
pub mod remove_bands;
pub mod scc;
pub mod selection;
pub mod shift_components;

pub use autorouter::*;
