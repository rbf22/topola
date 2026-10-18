// SPDX-FileCopyrightText: 2026 Topola contributors
//
// SPDX-License-Identifier: MIT

//! Lightweight global counters and timers for performance analysis.
//!
//! All instrumentation compiles to no-ops unless the `metrics` feature is
//! enabled (`cargo build --features metrics`). When enabled, counters are
//! `Relaxed` atomics and timers are nanosecond accumulators, so the
//! overhead stays negligible relative to the microsecond-scale operations
//! being measured.

use std::fmt;
use std::sync::atomic::{AtomicU64, Ordering};
use std::time::Instant;

macro_rules! metrics_struct {
    ($($f:ident),* $(,)?) => {
        #[derive(Default)]
        pub struct Metrics {
            $(pub $f: AtomicU64,)*
        }

        impl Metrics {
            pub const fn new() -> Self {
                Self { $($f: AtomicU64::new(0),)* }
            }

            pub fn reset(&self) {
                $(self.$f.store(0, Ordering::Relaxed);)*
            }
        }

        impl fmt::Display for Metrics {
            fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
                let ms = |nanos: u64| nanos as f64 / 1e6;
                writeln!(f, "== topola metrics ==")?;
                writeln!(
                    f,
                    "construction: prenavmesh {}x ({:.1}ms), bases {}x ({:.1}ms) + from-base {}x ({:.1}ms) + constrained {}x ({:.1}ms, deltas {}x, hits {}), navmesh {}x ({:.1}ms, hits {}), navnodes {} edges {}",
                    self.prenavmesh_builds.load(Ordering::Relaxed),
                    ms(self.prenavmesh_nanos.load(Ordering::Relaxed)),
                    self.prenavmesh_base_builds.load(Ordering::Relaxed),
                    ms(self.prenavmesh_base_nanos.load(Ordering::Relaxed)),
                    self.prenavmesh_from_base_builds.load(Ordering::Relaxed),
                    ms(self.prenavmesh_from_base_nanos.load(Ordering::Relaxed)),
                    self.prenavmesh_constrained_base_builds
                        .load(Ordering::Relaxed),
                    ms(self
                        .prenavmesh_constrained_base_nanos
                        .load(Ordering::Relaxed)),
                    self.prenavmesh_delta_constraint_adds
                        .load(Ordering::Relaxed),
                    self.prenavmesh_constrained_base_hits
                        .load(Ordering::Relaxed),
                    self.navmesh_builds.load(Ordering::Relaxed),
                    ms(self.navmesh_nanos.load(Ordering::Relaxed)),
                    self.navmesh_cache_hits.load(Ordering::Relaxed),
                    self.navmesh_nodes_total.load(Ordering::Relaxed),
                    self.navmesh_edges_total.load(Ordering::Relaxed),
                )?;
                writeln!(
                    f,
                    "search: pops {} (stale {}), visit_navnode {}x, path_reconstruct {}x",
                    self.frontier_pops.load(Ordering::Relaxed),
                    self.stale_frontier_pops.load(Ordering::Relaxed),
                    self.visit_navnode_calls.load(Ordering::Relaxed),
                    self.path_reconstructions.load(Ordering::Relaxed),
                )?;
                writeln!(
                    f,
                    "reconfiguration: surgical_ripups {}, multilayer_surgical_reconfigures {}, search_states stored {} hits {}",
                    self.surgical_ripups.load(Ordering::Relaxed),
                    self.multilayer_surgical_reconfigures.load(Ordering::Relaxed),
                    self.search_state_stores.load(Ordering::Relaxed),
                    self.search_state_hits.load(Ordering::Relaxed),
                )?;
                writeln!(
                    f,
                    "probes: {} (memoized {}, ghost {}, los_ok {}, failed {}, backtracked {})",
                    self.probes.load(Ordering::Relaxed),
                    self.probes_memoized.load(Ordering::Relaxed),
                    self.ghost_probes.load(Ordering::Relaxed),
                    self.probes_los_ok.load(Ordering::Relaxed),
                    self.probes_failed.load(Ordering::Relaxed),
                    self.probes_backtracked.load(Ordering::Relaxed),
                )?;
                writeln!(
                    f,
                    "draw: step_to ok {} err {} trusted {} memo {} ({:.1}ms), step_back {} ({:.1}ms)",
                    self.step_to_ok.load(Ordering::Relaxed),
                    self.step_to_err.load(Ordering::Relaxed),
                    self.step_to_trusted.load(Ordering::Relaxed),
                    self.step_memo_hits.load(Ordering::Relaxed),
                    ms(self.step_to_nanos.load(Ordering::Relaxed)),
                    self.step_backs.load(Ordering::Relaxed),
                    ms(self.step_back_nanos.load(Ordering::Relaxed)),
                )?;
                writeln!(
                    f,
                    "rework_path: {}x, undid {} steps, redrew {} steps ({:.1}ms)",
                    self.rework_path_calls.load(Ordering::Relaxed),
                    self.rework_steps_undone.load(Ordering::Relaxed),
                    self.rework_steps_redrawn.load(Ordering::Relaxed),
                    ms(self.rework_path_nanos.load(Ordering::Relaxed)),
                )?;
                write!(
                    f,
                    "step_to internals: guide {}x ({:.1}ms), infringe {}x ({:.1}ms, cands {} @ {:.1}ms), collide {}x ({:.1}ms)\n  inserts {} ({:.1}ms), removes {} ({:.1}ms), move_dot {} ({:.1}ms), bend_mod {} ({:.1}ms), bow_upd {} ({:.1}ms)",
                    self.guide_calls.load(Ordering::Relaxed),
                    ms(self.guide_nanos.load(Ordering::Relaxed)),
                    self.infringement_queries.load(Ordering::Relaxed),
                    ms(self.infringement_nanos.load(Ordering::Relaxed)),
                    self.infringement_candidates.load(Ordering::Relaxed),
                    ms(self.infringement_candidate_nanos.load(Ordering::Relaxed)),
                    self.collision_queries.load(Ordering::Relaxed),
                    ms(self.collision_nanos.load(Ordering::Relaxed)),
                    self.geometry_inserts.load(Ordering::Relaxed),
                    ms(self.insert_nanos.load(Ordering::Relaxed)),
                    self.geometry_removes.load(Ordering::Relaxed),
                    ms(self.remove_nanos.load(Ordering::Relaxed)),
                    self.move_dot_updates.load(Ordering::Relaxed),
                    ms(self.move_dot_update_nanos.load(Ordering::Relaxed)),
                    self.bend_modifies.load(Ordering::Relaxed),
                    ms(self.bend_modify_nanos.load(Ordering::Relaxed)),
                    self.bow_updates.load(Ordering::Relaxed),
                    ms(self.bow_update_nanos.load(Ordering::Relaxed)),
                )
            }
        }
    };
}

metrics_struct!(
    // Construction
    prenavmesh_builds,
    prenavmesh_nanos,
    prenavmesh_base_builds,
    prenavmesh_base_nanos,
    prenavmesh_from_base_builds,
    prenavmesh_from_base_nanos,
    prenavmesh_constrained_base_builds,
    prenavmesh_constrained_base_nanos,
    prenavmesh_constrained_base_hits,
    prenavmesh_delta_constraint_adds,
    navmesh_cache_hits,
    navmesh_builds,
    navmesh_nanos,
    navmesh_nodes_total,
    navmesh_edges_total,
    // Search
    frontier_pops,
    stale_frontier_pops,
    visit_navnode_calls,
    path_reconstructions,
    probes,
    probes_memoized,
    ghost_probes,
    probes_los_ok,
    probes_failed,
    probes_backtracked,
    // Reconfiguration
    surgical_ripups,
    multilayer_surgical_reconfigures,
    search_state_stores,
    search_state_hits,
    // Geometry draw/undraw — the suspected hot path
    step_to_ok,
    step_to_err,
    step_to_trusted,
    step_memo_hits,
    step_to_nanos,
    step_backs,
    step_back_nanos,
    rework_path_calls,
    rework_steps_undone,
    rework_steps_redrawn,
    rework_path_nanos,
    // step_to internals
    guide_calls,
    guide_nanos,
    infringement_queries,
    infringement_nanos,
    infringement_candidates,
    infringement_candidate_nanos,
    collision_queries,
    collision_nanos,
    geometry_inserts,
    insert_nanos,
    geometry_removes,
    remove_nanos,
    move_dot_updates,
    move_dot_update_nanos,
    bend_modifies,
    bend_modify_nanos,
    bow_updates,
    bow_update_nanos,
);

pub static METRICS: Metrics = Metrics::new();

/// RAII nanosecond timer. With the `metrics` feature off this is a no-op.
pub struct Timer {
    #[allow(dead_code)]
    start: Option<Instant>,
    #[allow(dead_code)]
    sink: &'static AtomicU64,
}

impl Timer {
    #[inline(always)]
    pub fn start(sink: &'static AtomicU64) -> Self {
        Self {
            #[cfg(feature = "metrics")]
            start: Some(Instant::now()),
            #[cfg(not(feature = "metrics"))]
            start: None,
            sink,
        }
    }
}

impl Drop for Timer {
    #[inline(always)]
    fn drop(&mut self) {
        #[cfg(feature = "metrics")]
        if let Some(start) = self.start.take() {
            self.sink
                .fetch_add(start.elapsed().as_nanos() as u64, Ordering::Relaxed);
        }
    }
}

#[inline(always)]
pub fn incr(counter: &AtomicU64) {
    #[cfg(feature = "metrics")]
    counter.fetch_add(1, Ordering::Relaxed);
    #[cfg(not(feature = "metrics"))]
    let _ = counter;
}

#[inline(always)]
pub fn add(counter: &AtomicU64, n: usize) {
    #[cfg(feature = "metrics")]
    counter.fetch_add(n as u64, Ordering::Relaxed);
    #[cfg(not(feature = "metrics"))]
    let _ = (counter, n);
}
