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
                    "steps: {} (route {}, routed {}, skipped {}, planar-reconf {}, layer-reconf {}) — {:.1}ms total, route {:.1}ms, reconf {:.1}ms, max {:.1}ms",
                    self.steps_total.load(Ordering::Relaxed),
                    self.steps_route.load(Ordering::Relaxed),
                    self.steps_routed.load(Ordering::Relaxed),
                    self.steps_skipped.load(Ordering::Relaxed),
                    self.steps_planar_reconfigured.load(Ordering::Relaxed),
                    self.steps_layer_reconfigured.load(Ordering::Relaxed),
                    ms(self.step_nanos_total.load(Ordering::Relaxed)),
                    ms(self.step_nanos_route.load(Ordering::Relaxed)),
                    ms(self.step_nanos_reconfigured.load(Ordering::Relaxed)),
                    ms(self.step_nanos_max.load(Ordering::Relaxed)),
                )?;
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
                    "  navmesh overlapees: {}x ({:.1}ms)",
                    self.navmesh_overlapee_queries.load(Ordering::Relaxed),
                    ms(self.navmesh_overlapee_nanos.load(Ordering::Relaxed)),
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
                    "reconfiguration: planar {} layer {} ({:.1}ms), surgical_ripups {}, multilayer_surgical_reconfigures {}, emit candidates {}, backtraced {}, blockers {}, relayered {} refanout-fail {} kept-wrapped {}, search_states stored {} hits {}",
                    self.planar_reconfigurations.load(Ordering::Relaxed),
                    self.layer_reconfigurations.load(Ordering::Relaxed),
                    ms(self.reconfigure_nanos.load(Ordering::Relaxed)),
                    self.surgical_ripups.load(Ordering::Relaxed),
                    self.multilayer_surgical_reconfigures.load(Ordering::Relaxed),
                    self.emit_loop_candidates.load(Ordering::Relaxed),
                    self.backtraced_ratlines.load(Ordering::Relaxed),
                    self.blockers_recorded.load(Ordering::Relaxed),
                    self.relayered_ratlines.load(Ordering::Relaxed),
                    self.refanout_failures.load(Ordering::Relaxed),
                    self.reconfig_kept_wrapped.load(Ordering::Relaxed),
                    self.search_state_stores.load(Ordering::Relaxed),
                    self.search_state_hits.load(Ordering::Relaxed),
                )?;
                writeln!(
                    f,
                    "critical: probes {}x, identified {}, reprobe rounds {}, promotions {}, protected_rip_skips {}, desperations {}",
                    self.critical_probes.load(Ordering::Relaxed),
                    self.critical_identified.load(Ordering::Relaxed),
                    self.critical_reprobe_rounds.load(Ordering::Relaxed),
                    self.critical_promotions.load(Ordering::Relaxed),
                    self.protected_rip_skips.load(Ordering::Relaxed),
                    self.protection_desperations.load(Ordering::Relaxed),
                )?;
                writeln!(
                    f,
                    "carve: segs placed {} failed {}, removed {}, kept-wrapped {}",
                    self.carve_segs_placed.load(Ordering::Relaxed),
                    self.carve_segs_failed.load(Ordering::Relaxed),
                    self.carves_removed.load(Ordering::Relaxed),
                    self.carves_kept_wrapped.load(Ordering::Relaxed),
                )?;
                writeln!(
                    f,
                    "geometry: pending joins recorded {}, pending-wrapped holds {}, dedup truncations {}",
                    self.pending_joins_recorded.load(Ordering::Relaxed),
                    self.pending_joiners_held.load(Ordering::Relaxed),
                    self.reconfig_dedup_truncations.load(Ordering::Relaxed),
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
    // Step accounting (Invoker step loop)
    steps_total,
    steps_route,
    steps_routed,
    steps_skipped,
    steps_planar_reconfigured,
    steps_layer_reconfigured,
    step_nanos_total,
    step_nanos_route,
    step_nanos_reconfigured,
    step_nanos_max,
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
    navmesh_overlapee_queries,
    navmesh_overlapee_nanos,
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
    planar_reconfigurations,
    layer_reconfigurations,
    reconfigure_nanos,
    surgical_ripups,
    multilayer_surgical_reconfigures,
    emit_loop_candidates,
    backtraced_ratlines,
    blockers_recorded,
    relayered_ratlines,
    refanout_failures,
    reconfig_kept_wrapped,
    search_state_stores,
    search_state_hits,
    // Critical-first routing
    critical_probes,
    critical_identified,
    critical_reprobe_rounds,
    critical_promotions,
    carve_segs_placed,
    carve_segs_failed,
    carves_removed,
    carves_kept_wrapped,
    pending_joins_recorded,
    pending_joiners_held,
    reconfig_dedup_truncations,
    protected_rip_skips,
    protection_desperations,
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

/// Records one stepped `Continue` of the top-level execution loop:
/// step count and wall-clock nanos by [`crate::autorouter::execution::StepKind`].
/// Compiles to a no-op without the `metrics` feature.
#[inline(always)]
pub fn record_step(kind: crate::autorouter::execution::StepKind, duration: std::time::Duration) {
    #[cfg(feature = "metrics")]
    {
        use crate::autorouter::execution::StepKind;

        incr(&METRICS.steps_total);
        add(&METRICS.step_nanos_total, duration.as_nanos() as usize);
        METRICS
            .step_nanos_max
            .fetch_max(duration.as_nanos() as u64, Ordering::Relaxed);
        match kind {
            StepKind::Route => incr(&METRICS.steps_route),
            StepKind::Routed => incr(&METRICS.steps_routed),
            StepKind::Skipped => incr(&METRICS.steps_skipped),
            StepKind::PlanarReconfigured => incr(&METRICS.steps_planar_reconfigured),
            StepKind::LayerReconfigured => incr(&METRICS.steps_layer_reconfigured),
            StepKind::Other => (),
        }
        match kind {
            StepKind::PlanarReconfigured | StepKind::LayerReconfigured => add(
                &METRICS.step_nanos_reconfigured,
                duration.as_nanos() as usize,
            ),
            StepKind::Route | StepKind::Routed | StepKind::Skipped => {
                add(&METRICS.step_nanos_route, duration.as_nanos() as usize)
            }
            StepKind::Other => (),
        }
    }
    #[cfg(not(feature = "metrics"))]
    let _ = (kind, duration);
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
