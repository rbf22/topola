// SPDX-FileCopyrightText: 2026 Topola contributors
//
// SPDX-License-Identifier: MIT

//! Continuous-3D simulated-annealing global pre-planning.
//!
//! Each selected ratline becomes a parametric polyline in a continuous
//! `(x, y, z)` volume — `z ∈ [0, layer_count−1]` a fluid layer axis —
//! and the whole ensemble is optimized at once under a global energy
//! `E = w_len·E_len + w_rep·E_rep + w_obs·E_obs + w_via·E_via + w_plan·E_plan`.
//! Repulsion is a *bounded soft-core* potential: crossings cost finite
//! energy, so at high temperature curves pass through one another (and
//! through obstacles) to discover homotopy classes globally; as `T → 0`
//! the barrier hardens, `z` quantizes onto integer layers, and vertical
//! excursions consolidate into via sites.
//!
//! The output is not geometry but an [`AnnealPlan`] consumed by
//! [`handoff`]: a per-ratline dominant layer, planned mid-route layer
//! hops as continuation ratlines, a contention-sorted routing order,
//! a per-obstacle congestion seed, and per-layer demand corridors for
//! carving. The existing multilayer machinery then legalizes
//! everything.

pub mod continuous;
pub mod curve;
pub mod discrete;
pub mod energy;
pub mod handoff;
pub mod obstacles;
pub mod quantize;
pub mod rng;

use std::collections::{BTreeMap, BTreeSet};

use geo::Point;
use serde::{Deserialize, Serialize};
use specctra_core::mesadata::AccessMesadata;

use crate::{
    autorouter::{ratline::RatlineUid, Autorouter},
    drawing::graph::PrimitiveIndex,
};

/// Which annealer formulation produced the plan.
#[derive(Clone, Copy, Debug, Default, Deserialize, Eq, PartialEq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum AnnealMode {
    /// The paper's formulation: parametric 3D curves annealed under
    /// the 4-term soft-core energy.
    #[default]
    Continuous,
    /// Comparison baseline: SA directly over
    /// (layer assignment, routing order) with a corridor-conflict
    /// energy — no continuous curves.
    Discrete,
}

fn default_iterations() -> usize {
    300
}

fn default_control_points() -> usize {
    8
}

fn default_w_length() -> f64 {
    1.0
}

fn default_w_repulsion() -> f64 {
    30.0
}

fn default_w_via() -> f64 {
    10.0
}

fn default_w_planar() -> f64 {
    100.0
}

fn default_w_obstacle() -> f64 {
    50.0
}

fn default_min_run_fraction() -> f64 {
    0.15
}

fn default_true() -> bool {
    true
}

/// Simulated-annealing pre-plan options, embedded in
/// [`crate::autorouter::multilayer_autoroute::MultilayerAutorouteOptions`].
/// Kept POD so the enclosing options stay `Copy`.
#[derive(Clone, Copy, Debug, Deserialize, Serialize)]
pub struct AnnealOptions {
    /// `continuous` runs the 3D-curve formulation; `discrete` the
    /// corridor-level comparison baseline.
    #[serde(default)]
    pub mode: AnnealMode,
    /// Deterministic seed; the anneal is a pure function of it.
    #[serde(default)]
    pub seed: u64,
    /// Number of cooling sweeps (each ~one proposal per curve). The
    /// schedule derives `α` from `t_initial → t_final` over this many
    /// sweeps. Never wall-clock bounded — replay determinism.
    #[serde(default = "default_iterations")]
    pub iterations: usize,
    /// Interior control points per curve.
    #[serde(default = "default_control_points")]
    pub control_points: usize,
    /// Weight on stretch ratio (3D length / endpoint distance).
    #[serde(default = "default_w_length")]
    pub w_length: f64,
    /// Soft-core repulsion barrier height — the crossing cost.
    /// `t_initial` should sit near it so `exp(−w_rep/T₀)` is a
    /// moderate acceptance probability.
    #[serde(default = "default_w_repulsion")]
    pub w_repulsion: f64,
    /// Weight on Σ|Δz| z-tortuosity — consolidates layer hops.
    #[serde(default = "default_w_via")]
    pub w_via: f64,
    /// Peak weight of the planarity well `sin²(πz)`; ramps 0→this over
    /// the schedule.
    #[serde(default = "default_w_planar")]
    pub w_planar: f64,
    /// Peak weight of obstacle soft-repulsion.
    #[serde(default = "default_w_obstacle")]
    pub w_obstacle: f64,
    /// Soft-core radius in XY units; `None` → `2 × routed_band_width`.
    #[serde(default)]
    pub soft_core_sigma: Option<f64>,
    /// XY units per unit of z (one layer step); `None` →
    /// `4 × routed_band_width`. Without it the 3D length term cannot
    /// compare against detours.
    #[serde(default)]
    pub z_weight: Option<f64>,
    /// Annealing start temperature; `None` → `w_repulsion` (crossing
    /// acceptance ≈ e⁻¹).
    #[serde(default)]
    pub t_initial: Option<f64>,
    /// Annealing stop temperature; `None` → `0.002 × w_repulsion`.
    #[serde(default)]
    pub t_final: Option<f64>,
    /// Carve demand corridors for this many most-contested ratlines;
    /// `0` disables carving.
    #[serde(default)]
    pub carve_top_k: usize,
    /// z-runs shorter than this fraction of a curve's samples merge
    /// into neighbors instead of becoming via hops.
    #[serde(default = "default_min_run_fraction")]
    pub min_run_fraction: f64,
    /// Ramp the obstacle weight 10%→100% over the schedule (soft
    /// obstacles early) instead of holding it constant.
    #[serde(default = "default_true")]
    pub obstacle_weight_ramp: bool,
}

impl Default for AnnealOptions {
    fn default() -> Self {
        Self {
            mode: AnnealMode::default(),
            seed: 0,
            iterations: default_iterations(),
            control_points: default_control_points(),
            w_length: default_w_length(),
            w_repulsion: default_w_repulsion(),
            w_via: default_w_via(),
            w_planar: default_w_planar(),
            w_obstacle: default_w_obstacle(),
            soft_core_sigma: None,
            z_weight: None,
            t_initial: None,
            t_final: None,
            carve_top_k: 0,
            min_run_fraction: default_min_run_fraction(),
            obstacle_weight_ramp: true,
        }
    }
}

impl AnnealOptions {
    /// Effective soft-core radius.
    pub fn sigma(&self, band_width: f64) -> f64 {
        self.soft_core_sigma.unwrap_or(2.0 * band_width)
    }

    /// Effective z scaling.
    pub fn z_scale(&self, band_width: f64) -> f64 {
        self.z_weight.unwrap_or(4.0 * band_width)
    }

    /// Effective initial temperature.
    pub fn t_start(&self) -> f64 {
        self.t_initial.unwrap_or(self.w_repulsion)
    }

    /// Effective final temperature.
    pub fn t_stop(&self) -> f64 {
        self.t_final.unwrap_or(0.002 * self.w_repulsion)
    }

    /// Geometric cooling factor per sweep.
    pub fn cooling(&self) -> f64 {
        (self.t_stop() / self.t_start()).powf(1.0 / self.iterations.max(1) as f64)
    }
}

/// One ratline whose quantized curve hops layers mid-route: ordered
/// `(layer, polyline)` runs, with via sites at run boundaries.
#[derive(Clone, Debug, PartialEq)]
pub struct AnnealSplit {
    /// The original ratline covering `runs[0]`; subsequent runs become
    /// continuation ratlines created at handoff.
    pub parent: RatlineUid,
    /// Quantized `(layer, xy-polyline)` runs in path order.
    pub runs: Vec<(usize, Vec<Point>)>,
    /// Via site candidates at each run boundary (`runs.len() − 1`).
    pub via_sites: Vec<Point>,
}

/// What an annealing run produces for the routing machinery.
#[derive(Clone, Debug, Default)]
pub struct AnnealPlan {
    /// Dominant layer per single-run ratline (split parents and
    /// continuations get their entries at handoff).
    pub layer_map: BTreeMap<RatlineUid, usize>,
    /// Ratlines whose curves hop layers mid-route.
    pub splits: Vec<AnnealSplit>,
    /// Suggested routing order, most-contested first.
    pub order: Vec<RatlineUid>,
    /// Per-fixed-primitive near-miss counts at freeze — congestion
    /// seed for the negotiated-congestion reconfigurer.
    pub congestion: BTreeMap<PrimitiveIndex, usize>,
    /// Quantized demand corridor per ratline as `(point, layer)`
    /// waypoints — `carve_corridor`'s input shape.
    pub corridors: BTreeMap<RatlineUid, Vec<(Point, usize)>>,
    /// System energy of the initial straight-line ensemble, for the
    /// improvement-over-initialization check.
    pub initial_energy: f64,
    /// Final system energy, for the metrics/benchmark report.
    pub final_energy: f64,
    /// Accepted proposals that changed a curve's repulsion-neighbor
    /// set — the soft-core pass-through observability metric.
    pub crossings: usize,
}

/// Runs the annealer selected by `options.mode` over `ratlines` and
/// returns the pre-routing plan. Pure function of the board and
/// `options.seed`; performs no board mutation. `band_width` scales the
/// repulsion core and z axis against design units.
pub fn run(
    autorouter: &Autorouter<impl AccessMesadata>,
    ratlines: &BTreeSet<RatlineUid>,
    options: &AnnealOptions,
    band_width: f64,
) -> AnnealPlan {
    let _timer = crate::metrics::Timer::start(&crate::metrics::METRICS.anneal_nanos);
    let plan = match options.mode {
        AnnealMode::Continuous => continuous::anneal(autorouter, ratlines, options, band_width),
        AnnealMode::Discrete => discrete::anneal(autorouter, ratlines, options, band_width),
    };
    crate::metrics::add(
        &crate::metrics::METRICS.anneal_energy_milliunits,
        (plan.final_energy * 1000.0) as usize,
    );
    crate::metrics::add(&crate::metrics::METRICS.anneal_splits, plan.splits.len());
    crate::metrics::add(&crate::metrics::METRICS.anneal_crossings, plan.crossings);
    plan
}

#[cfg(test)]
mod tests {
    use std::{fs::File, io::BufReader};

    use specctra_core::mesadata::SpecctraMesadata;

    use super::*;
    use crate::{board::edit::BoardEdit, specctra::design::SpecctraDesign};

    fn astable() -> Autorouter<SpecctraMesadata> {
        let design_file =
            File::open("tests/multilayer/astable_multivibrator/astable_multivibrator.dsn").unwrap();
        let design = SpecctraDesign::load(BufReader::new(design_file)).unwrap();
        Autorouter::new(design.make_board(&mut BoardEdit::new())).unwrap()
    }

    /// Every ratline on every principal layer.
    fn input(autorouter: &Autorouter<SpecctraMesadata>) -> BTreeSet<RatlineUid> {
        let mut ratlines = BTreeSet::new();
        for layer in 0..autorouter.board().layout().drawing().layer_count() {
            ratlines.extend(
                autorouter
                    .ratsnests()
                    .on_principal_layer(layer)
                    .graph()
                    .edge_indices()
                    .map(|index| RatlineUid {
                        principal_layer: layer,
                        index,
                    }),
            );
        }
        ratlines
    }

    fn assert_same_plan(first: &AnnealPlan, second: &AnnealPlan) {
        assert_eq!(first.layer_map, second.layer_map);
        assert_eq!(first.order, second.order);
        assert_eq!(first.splits, second.splits);
        assert_eq!(first.corridors, second.corridors);
        assert_eq!(first.final_energy, second.final_energy);
        assert_eq!(first.crossings, second.crossings);
    }

    #[test]
    fn continuous_anneal_is_deterministic() {
        let autorouter = astable();
        let ratlines = input(&autorouter);
        let options = AnnealOptions {
            seed: 7,
            iterations: 60,
            ..AnnealOptions::default()
        };
        let first = run(&autorouter, &ratlines, &options, 100.0);
        let second = run(&autorouter, &ratlines, &options, 100.0);
        assert_same_plan(&first, &second);
    }

    #[test]
    fn different_seeds_change_the_plan() {
        let autorouter = astable();
        let ratlines = input(&autorouter);
        let first = run(
            &autorouter,
            &ratlines,
            &AnnealOptions {
                seed: 7,
                iterations: 60,
                ..AnnealOptions::default()
            },
            100.0,
        );
        let second = run(
            &autorouter,
            &ratlines,
            &AnnealOptions {
                seed: 8,
                iterations: 60,
                ..AnnealOptions::default()
            },
            100.0,
        );
        // Continuous coordinates make a byte-identical corridor set
        // across seeds essentially impossible.
        assert_ne!(first.corridors, second.corridors);
    }

    #[test]
    fn continuous_anneal_improves_energy_and_crosses() {
        let autorouter = astable();
        let ratlines = input(&autorouter);
        let plan = run(
            &autorouter,
            &ratlines,
            &AnnealOptions {
                seed: 7,
                iterations: 60,
                ..AnnealOptions::default()
            },
            100.0,
        );
        assert!(plan.final_energy < plan.initial_energy);
        // Soft-core pass-throughs must actually occur — otherwise the
        // repulsion field is effectively hard-core and the annealer
        // cannot explore homotopy classes.
        assert!(plan.crossings > 0);
    }

    #[test]
    fn discrete_anneal_is_bounded_and_deterministic() {
        let autorouter = astable();
        let ratlines = input(&autorouter);
        let layer_count = autorouter.board().layout().drawing().layer_count();
        let options = AnnealOptions {
            mode: AnnealMode::Discrete,
            seed: 7,
            iterations: 60,
            ..AnnealOptions::default()
        };
        let first = run(&autorouter, &ratlines, &options, 100.0);
        let second = run(&autorouter, &ratlines, &options, 100.0);
        assert_same_plan(&first, &second);
        assert!(first.layer_map.values().all(|layer| *layer < layer_count));
    }
}
