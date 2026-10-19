// SPDX-FileCopyrightText: 2026 Topola contributors
//
// SPDX-License-Identifier: MIT

//! Critical-first iterative routing with corridor carving.
//!
//! Where the ordinary reconfigurators discover contention only by
//! failing, this outer stepper measures it up front: relaxed-rule
//! channel probes reveal each selected ratline's demand path, its
//! real-clearance blockers, and the paths it conflicts with
//! ([`criticality::analyze`]). The hardest connections — the *critical
//! set* — are then committed before ordinary routing fills in around
//! them:
//!
//! - **scheduled first**: the injected ordering puts criticals ahead
//!   of everything else, and [`PlanarAutorouteReconfigurator`] keeps
//!   them pinned to the front of every emitted ordering,
//! - **physically reserved**: each critical ratline's demand corridor
//!   is carved onto the board as same-net fixed geometry
//!   ([`carve::carve_corridor`]) — invisible to its own net, constraint
//!   edges to everyone else,
//! - **protected from rip-up**: committed critical bands are filtered
//!   out of `rip_up` lists (with a desperation escape so a protected
//!   band can never cause a deadlock).
//!
//! Routing then proceeds inside the ordinary planar or multilayer
//! machinery. Every `reprobe_every` reconfigurations the still-unrouted
//! ratlines are re-probed — geometry has changed, so has what is hard —
//! newly critical ratlines are promoted, carved, and protected, and
//! carves that went stale on multilayer layer rotation are re-carved on
//! the new layer. The loop terminates when the board completes, when no
//! progress is left to make (`max_reprobe_rounds`), or when the inner
//! stepper breaks.
//!
//! All probing happens on autorouter clones: the live board never sees
//! relaxed rules or probe-band residue. Carves are real copper placed
//! outside the inner stepper's per-ratline edits (like escape stubs and
//! fanout vias); dead ones — nothing else depends on them — are removed
//! when the command breaks, while wrapped ones are kept as valid
//! same-net copper.

use std::{
    collections::{BTreeMap, BTreeSet},
    ops::ControlFlow,
};

use geo::Point;
use serde::{Deserialize, Serialize};
use specctra_core::mesadata::AccessMesadata;

use crate::{
    autorouter::{
        carve,
        criticality::{self, CriticalityReport},
        invoker::GetDebugOverlayData,
        multilayer_autoroute::MultilayerAutorouteOptions,
        multilayer_preconfigurer::MultilayerAutoroutePreconfigurerInput,
        multilayer_reconfigurator::{MultilayerAutorouteReconfigurator, MultilayerReconfigKind},
        planar_autoroute::{PlanarAutorouteConfiguration, PlanarAutorouteContinueStatus},
        planar_preconfigurer::PlanarAutoroutePreconfigurerInput,
        planar_reconfigurator::PlanarAutorouteReconfigurator,
        ratline::RatlineUid,
        Autorouter, AutorouterError, PlanarAutorouteOptions,
    },
    board::edit::BoardEdit,
    drawing::{dot::FixedDotIndex, primitive::GetWeight},
    geometry::{edit::Edit, primitive::PrimitiveShape, GetLayer, GetSetPos},
    graph::MakeRef,
    metrics::{add, incr, METRICS},
    router::{navcord::Navcord, navmesh::Navmesh, thetastar::ThetastarStepper, RouterOptions},
    stepper::{
        Abort, EstimateProgress, GetTimeoutProgress, LinearScale, ReconfiguratorStatus, Step,
    },
};

fn default_quantile() -> f64 {
    0.75
}

fn default_true() -> bool {
    true
}

fn default_reprobe_every() -> usize {
    4
}

fn default_max_reprobe_rounds() -> usize {
    8
}

fn default_max_carves() -> usize {
    24
}

/// Options for [`Command::CriticalAutoroute`][crate::autorouter::execution::Command].
#[derive(Clone, Copy, Debug, Deserialize, Serialize)]
pub struct CriticalAutorouteOptions {
    /// Inner planar routing options, used directly unless `multilayer`
    /// is set.
    pub planar: PlanarAutorouteOptions,
    /// `Some` routes through the multilayer inner stepper; its
    /// `.planar` field then takes precedence over `planar` above.
    pub multilayer: Option<MultilayerAutorouteOptions>,
    /// Score quantile cut for the critical set: ratlines scoring in
    /// the top `1 - quantile` of positive scores join every relaxed
    /// probe failure.
    #[serde(default = "default_quantile")]
    pub critical_quantile: f64,
    /// Physically carve each critical ratline's demand corridor as
    /// same-net fixed geometry.
    #[serde(default = "default_true")]
    pub carve: bool,
    /// Protect committed critical bands from rip-up and permutation
    /// reordering.
    #[serde(default = "default_true")]
    pub protect: bool,
    /// Re-probe the unresolved ratlines after every this many
    /// reconfigurations.
    #[serde(default = "default_reprobe_every")]
    pub reprobe_every: usize,
    /// Cap on re-identification rounds; exhaustion leaves routing to
    /// the inner machinery alone.
    #[serde(default = "default_max_reprobe_rounds")]
    pub max_reprobe_rounds: usize,
    /// Cap on how many corridors are physically carved — the
    /// top-scoring criticals. Protection and critical-first
    /// scheduling apply to every critical ratline regardless; carves
    /// are real geometry in every subsequent navmesh, so unbounded
    /// carving would strangle routing on dense boards.
    #[serde(default = "default_max_carves")]
    pub max_carves: usize,
    /// Keep every carve in the output instead of removing dead ones at
    /// break time.
    #[serde(default)]
    pub keep_carves: bool,
}

/// What the inner stepper reported this step.
#[derive(Clone, Copy, Debug, PartialEq)]
enum InnerStatus {
    Running(PlanarAutorouteContinueStatus),
    Reconfigured,
    LayerReconfigured,
}

/// The status reported by the outer stepper.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum CriticalAutorouteStatus {
    Running(PlanarAutorouteContinueStatus),
    /// A re-identification round ran or the inner planar machinery
    /// reconfigured.
    Reconfigured,
    /// The inner multilayer machinery reconfigured (rotation, weave).
    LayerReconfigured,
}

/// Planar or multilayer routing machinery, driven by the outer stepper.
enum InnerRouter {
    Planar(Box<PlanarAutorouteReconfigurator>),
    Multilayer(Box<MultilayerAutorouteReconfigurator>),
}

impl InnerRouter {
    fn planar(&self) -> &PlanarAutorouteReconfigurator {
        match self {
            Self::Planar(inner) => inner,
            Self::Multilayer(inner) => inner.planar(),
        }
    }

    fn planar_mut(&mut self) -> &mut PlanarAutorouteReconfigurator {
        match self {
            Self::Planar(inner) => inner,
            Self::Multilayer(inner) => inner.planar_mut(),
        }
    }

    /// The live layer assignments — empty for the planar inner router.
    fn layer_map(&self) -> BTreeMap<RatlineUid, usize> {
        match self {
            Self::Planar(_) => BTreeMap::new(),
            Self::Multilayer(inner) => inner.layer_map().clone(),
        }
    }

    fn step(
        &mut self,
        autorouter: &mut Autorouter<impl AccessMesadata + Clone>,
    ) -> Result<ControlFlow<Option<BoardEdit>, InnerStatus>, AutorouterError> {
        match self {
            Self::Planar(inner) => Ok(match inner.step(autorouter)? {
                ControlFlow::Break(edit) => ControlFlow::Break(edit),
                ControlFlow::Continue(ReconfiguratorStatus::Running(status)) => {
                    ControlFlow::Continue(InnerStatus::Running(status))
                }
                ControlFlow::Continue(ReconfiguratorStatus::Reconfigured(_)) => {
                    ControlFlow::Continue(InnerStatus::Reconfigured)
                }
            }),
            Self::Multilayer(inner) => Ok(match inner.step(autorouter)? {
                ControlFlow::Break(edit) => ControlFlow::Break(edit),
                ControlFlow::Continue(ReconfiguratorStatus::Running(inner_status)) => {
                    match inner_status {
                        ReconfiguratorStatus::Running(status) => {
                            ControlFlow::Continue(InnerStatus::Running(status))
                        }
                        ReconfiguratorStatus::Reconfigured(_) => {
                            ControlFlow::Continue(InnerStatus::Reconfigured)
                        }
                    }
                }
                ControlFlow::Continue(ReconfiguratorStatus::Reconfigured(kind)) => {
                    ControlFlow::Continue(match kind {
                        MultilayerReconfigKind::Planar => InnerStatus::Reconfigured,
                        MultilayerReconfigKind::Weave | MultilayerReconfigKind::LayerRotation => {
                            InnerStatus::LayerReconfigured
                        }
                    })
                }
            }),
        }
    }
}

/// The critical-first outer stepper: probes, scores, carves, and
/// protects the hardest ratlines, drives the inner router, and
/// re-identifies critical ratlines on stalls.
pub struct CriticalAutorouteExecutionStepper {
    inner: InnerRouter,
    /// Every ratline ever identified as critical — protected,
    /// carve-eligible, and scheduled first in injected orderings.
    critical: BTreeSet<RatlineUid>,
    /// Latest criticality score per probed ratline, for ordering.
    scores: BTreeMap<RatlineUid, f64>,
    /// Chronological merge of every carve edit applied to the board
    /// (placements and selective removals). Reversed wholesale on
    /// abort; merged into the command's output edit on break.
    carve_edit: BoardEdit,
    /// Per-ratline carve records currently on the board, for selective
    /// removal of dead or stale carves.
    carve_edits: BTreeMap<RatlineUid, BoardEdit>,
    /// Layer assignments at the last re-identification round — diffed
    /// to find ratlines whose carves went stale on rotation.
    last_layer_map: BTreeMap<RatlineUid, usize>,
    /// Reconfiguration events seen from the inner stepper.
    seen_reconfigurations: usize,
    /// Re-identification rounds run so far.
    reprobe_rounds: usize,
    options: CriticalAutorouteOptions,
}

impl CriticalAutorouteExecutionStepper {
    /// Probes `ratlines` on an autorouter clone under relaxed rules,
    /// seeds the inner reconfigurers' congestion with the observed
    /// blockers, then builds the inner router — injecting the
    /// critical-first ordering, carves, and protected set into its
    /// initial configuration.
    pub fn new(
        autorouter: &mut Autorouter<impl AccessMesadata + Clone>,
        ratlines: BTreeSet<RatlineUid>,
        options: CriticalAutorouteOptions,
    ) -> Result<Self, AutorouterError> {
        let router_options = options
            .multilayer
            .map(|options| options.planar.router)
            .unwrap_or(options.planar.router);

        // The probe pass runs on a clone: the live board never sees
        // relaxed rules or probe-band residue.
        let relaxed = criticality::relaxed_probe_clearance(
            autorouter.board().layout().drawing().rules(),
            router_options.routed_band_width,
        );
        let report = {
            let mut probe_autorouter = autorouter.clone();
            criticality::analyze(
                &mut probe_autorouter,
                &ratlines,
                &BTreeMap::new(),
                router_options,
                options.critical_quantile,
                |rules| rules.override_clearances(relaxed),
            )
        };
        log::debug!(
            "critical-first: {} ratlines probed, {} critical",
            ratlines.len(),
            report.critical.len()
        );

        // Seed the negotiated-congestion record with the observed
        // blockers so foreign bands avoid the demanded corridors
        // before a single failure is routed.
        autorouter.seed_congestion(
            report
                .blockers
                .values()
                .flatten()
                .copied()
                .collect::<Vec<_>>(),
        );

        let scores = report.scores.clone();
        let mut overridden: Option<(BTreeSet<RatlineUid>, BTreeMap<RatlineUid, BoardEdit>)> = None;

        let mut inner = if let Some(multilayer_options) = options.multilayer {
            InnerRouter::Multilayer(Box::new(
                MultilayerAutorouteReconfigurator::new_with_planar_override(
                    autorouter,
                    MultilayerAutoroutePreconfigurerInput {
                        ratlines: ratlines.clone(),
                    },
                    multilayer_options,
                    |autorouter, configuration| {
                        // The multilayer plan's own order encodes
                        // fanout/layer structure — congestion seeding and
                        // re-probing apply, but neither the critical-first
                        // reorder, protected-set scheduling, nor carving:
                        // carved corridors starve the via-weaving the
                        // fanout plan depends on.
                        overridden = Some(Self::install_critical(
                            autorouter,
                            configuration,
                            &report,
                            router_options,
                            &CriticalAutorouteOptions {
                                carve: false,
                                ..options
                            },
                            false,
                        ));
                    },
                )?,
            ))
        } else {
            InnerRouter::Planar(Box::new(
                PlanarAutorouteReconfigurator::new_with_configuration_override(
                    autorouter,
                    PlanarAutoroutePreconfigurerInput {
                        ratlines: ratlines.clone(),
                        terminating_dot_map: BTreeMap::new(),
                    },
                    options.planar,
                    |autorouter, configuration| {
                        overridden = Some(Self::install_critical(
                            autorouter,
                            configuration,
                            &report,
                            router_options,
                            &options,
                            true,
                        ));
                    },
                )?,
            ))
        };

        let (critical, carve_edits) = overridden.unwrap_or_default();
        let mut carve_edit = BoardEdit::new();
        for edit in carve_edits.values() {
            carve_edit.merge(edit.clone());
        }

        let last_layer_map = inner.layer_map();
        // Protected-set scheduling stays a planar mechanism: under
        // multilayer its urgent re-scheduling starves the plan's own
        // fanout ordering.
        if options.protect && options.multilayer.is_none() {
            inner
                .planar_mut()
                .set_protected(critical.iter().copied().collect());
        }

        Ok(Self {
            inner,
            critical,
            scores,
            carve_edit,
            carve_edits,
            last_layer_map,
            seen_reconfigurations: 0,
            reprobe_rounds: 0,
            options,
        })
    }

    /// The configuration-override shared by both inner routers:
    /// re-probes the critical set between the configuration's live
    /// terminating dots (fanout vias on assigned layers, or escape
    /// stubs) so demand paths land where routing will actually run,
    /// carves those corridors onto the board, installs the
    /// critical-first ordering, and marks the critical set protected.
    ///
    /// Returns `(critical ∩ live ratlines, per-ratline carve edits)`.
    fn install_critical(
        autorouter: &mut Autorouter<impl AccessMesadata + Clone>,
        configuration: &mut PlanarAutorouteConfiguration,
        report: &CriticalityReport,
        router_options: RouterOptions,
        options: &CriticalAutorouteOptions,
        reorder: bool,
    ) -> (BTreeSet<RatlineUid>, BTreeMap<RatlineUid, BoardEdit>) {
        let live: BTreeSet<RatlineUid> = configuration.ratlines.iter().copied().collect();
        let mut critical: BTreeSet<RatlineUid> = report
            .critical
            .iter()
            .copied()
            .filter(|ratline| live.contains(ratline))
            .collect();

        // Bound the critical set: once a large fraction of the board
        // is "critical", hard-first scheduling only starves the easy
        // work. Keep the top-scored quarter (or eight, whichever is
        // more).
        let max_total = (configuration.ratlines.len() / 4).max(8);
        if critical.len() > max_total {
            let mut by_score: Vec<RatlineUid> = critical.iter().copied().collect();
            by_score.sort_by(|a, b| {
                report
                    .scores
                    .get(b)
                    .copied()
                    .unwrap_or(0.0)
                    .total_cmp(&report.scores.get(a).copied().unwrap_or(0.0))
            });
            critical = by_score.into_iter().take(max_total).collect();
        }

        let mut paths: BTreeMap<RatlineUid, Vec<(Point, usize)>> = BTreeMap::new();
        let mut carve_edits = BTreeMap::new();

        if !critical.is_empty() {
            // Re-probe criticals whose terminating dots were remapped
            // (fanout vias or escape stubs) so the demand path lands on
            // the layer/endpoint the router will actually see. Probing
            // runs on a clone, as always.
            let remapped: Vec<(RatlineUid, FixedDotIndex, FixedDotIndex)> = critical
                .iter()
                .filter_map(|&ratline| {
                    let endpoints = Self::terminating_dots(autorouter, configuration, ratline);
                    (endpoints != ratline.ref_(autorouter).endpoint_dots()).then_some((
                        ratline,
                        endpoints.0,
                        endpoints.1,
                    ))
                })
                .collect();
            if !remapped.is_empty() {
                let relaxed = criticality::relaxed_probe_clearance(
                    autorouter.board().layout().drawing().rules(),
                    router_options.routed_band_width,
                );
                let mut probe_autorouter = autorouter.clone();
                let probes =
                    probe_autorouter.probe_ratlines_dots(&remapped, router_options, |rules| {
                        rules.override_clearances(relaxed)
                    });
                for (probe, (ratline, _, _)) in probes.iter().zip(remapped.iter()) {
                    if !probe.path.is_empty() {
                        paths.insert(*ratline, probe.path.clone());
                    }
                }
            }

            if options.carve {
                // Carve the highest-scoring corridors first, up to
                // `max_carves` — carving is real geometry in every
                // subsequent navmesh, so it is bounded; protection and
                // scheduling cover the rest of the critical set.
                let mut by_score: Vec<RatlineUid> = critical.iter().copied().collect();
                by_score.sort_by(|a, b| {
                    report
                        .scores
                        .get(b)
                        .copied()
                        .unwrap_or(0.0)
                        .total_cmp(&report.scores.get(a).copied().unwrap_or(0.0))
                });
                for &ratline in by_score.iter().take(options.max_carves) {
                    // A relaxed-probe failure has no demand path; fall back
                    // to the straight chord between its live terminating
                    // dots so the corridor is still reserved. Dots that
                    // would land on foreign geometry simply fail to place.
                    let chord = || {
                        let (origin, destination) =
                            Self::terminating_dots(autorouter, configuration, ratline);
                        let drawing = autorouter.board().layout().drawing();
                        let (o, d) = (
                            drawing.primitive_ref(origin),
                            drawing.primitive_ref(destination),
                        );
                        (o.layer() == d.layer()).then(|| {
                            vec![(o.weight().pos(), o.layer()), (d.weight().pos(), d.layer())]
                        })
                    };
                    let path = paths
                        .get(&ratline)
                        .or_else(|| report.paths.get(&ratline))
                        .cloned()
                        .or_else(chord);
                    if let Some(path) = path {
                        if let Some(edit) = carve::carve_corridor(
                            autorouter,
                            ratline,
                            &path,
                            router_options.routed_band_width,
                        ) {
                            carve_edits.insert(ratline, edit);
                        }
                    }
                }
            }

            // Critical-first ordering: criticals by score descending,
            // then everything else in the preconfigured order. (The
            // multilayer inner skips this — its plan's order encodes
            // fanout/layer structure a probe-driven reorder would
            // wreck.)
            if reorder {
                let mut criticals: Vec<RatlineUid> = critical.iter().copied().collect();
                criticals.sort_by(|a, b| {
                    report
                        .scores
                        .get(b)
                        .copied()
                        .unwrap_or(0.0)
                        .total_cmp(&report.scores.get(a).copied().unwrap_or(0.0))
                });
                let criticals_set: BTreeSet<RatlineUid> = criticals.iter().copied().collect();
                configuration.ratlines = criticals
                    .into_iter()
                    .chain(
                        configuration
                            .ratlines
                            .iter()
                            .copied()
                            .filter(|ratline| !criticals_set.contains(ratline)),
                    )
                    .collect();
            }

            if options.protect && reorder {
                configuration.protected = critical.clone();
            }
        }

        (critical, carve_edits)
    }

    /// The terminating dots `configuration` routes `ratline` to —
    /// the remapped fanout/stub dots when present, else the pads.
    fn terminating_dots(
        autorouter: &Autorouter<impl AccessMesadata>,
        configuration: &PlanarAutorouteConfiguration,
        ratline: RatlineUid,
    ) -> (FixedDotIndex, FixedDotIndex) {
        let endpoints = ratline.ref_(autorouter).endpoint_dots();
        (
            configuration
                .terminating_dot_map
                .get(&(ratline, endpoints.0))
                .copied()
                .unwrap_or(endpoints.0),
            configuration
                .terminating_dot_map
                .get(&(ratline, endpoints.1))
                .copied()
                .unwrap_or(endpoints.1),
        )
    }

    /// One re-identification round: dissolve the in-flight route,
    /// re-probe the still-unrouted ratlines on a clone, promote newly
    /// critical ones (carve + protect + splice right after the current
    /// position), re-carve rotated stale carves, and resume.
    fn reidentify(
        &mut self,
        autorouter: &mut Autorouter<impl AccessMesadata + Clone>,
    ) -> Result<(), AutorouterError> {
        let router_options = self
            .options
            .multilayer
            .map(|options| options.planar.router)
            .unwrap_or(self.options.planar.router);

        let configuration = self.inner.planar().configuration().clone();
        let curr = self.inner.planar().curr_ratline_index();

        // Unrouted = no band between the ratline's live terminating dots.
        let unrouted: BTreeSet<RatlineUid> = configuration
            .ratlines
            .iter()
            .copied()
            .filter(|&ratline| {
                let (origin, destination) =
                    Self::terminating_dots(autorouter, &configuration, ratline);
                autorouter
                    .board
                    .band_between_nodes(origin, destination)
                    .is_none()
            })
            .collect();
        if unrouted.is_empty() {
            return Ok(());
        }

        incr(&METRICS.critical_reprobe_rounds);
        self.reprobe_rounds += 1;
        self.inner
            .planar_mut()
            .begin_surgical_reconfiguration(autorouter, &[])?;

        // Carves whose ratline's layer assignment rotated away are
        // stale: remove them (unless load-bearing) before probing —
        // the re-probe below sees the board they will carve on.
        let layer_map = self.inner.layer_map();
        let relayered: Vec<RatlineUid> = self
            .carve_edits
            .keys()
            .copied()
            .filter(|ratline| layer_map.get(ratline) != self.last_layer_map.get(ratline))
            .collect();
        for ratline in relayered {
            if let Some(edit) = self.carve_edits.get(&ratline) {
                if carve::edit_is_wrapped(autorouter.board().layout().drawing(), &edit.layout_edit)
                {
                    incr(&METRICS.carves_kept_wrapped);
                    continue;
                }
                let reverse = edit.reverse();
                autorouter.board.apply_edit(&reverse);
                self.carve_edit.merge(reverse);
            }
            self.carve_edits.remove(&ratline);
        }
        self.last_layer_map = layer_map;

        // Probe the unresolved ratlines on a clone, between the
        // terminating dots routing actually targets.
        let endpoints: BTreeMap<RatlineUid, (FixedDotIndex, FixedDotIndex)> = unrouted
            .iter()
            .map(|&ratline| {
                (
                    ratline,
                    Self::terminating_dots(autorouter, &configuration, ratline),
                )
            })
            .collect();
        let report = {
            let relaxed = criticality::relaxed_probe_clearance(
                autorouter.board().layout().drawing().rules(),
                router_options.routed_band_width,
            );
            let mut probe_autorouter = autorouter.clone();
            criticality::analyze(
                &mut probe_autorouter,
                &unrouted,
                &endpoints,
                router_options,
                self.options.critical_quantile,
                |rules| rules.override_clearances(relaxed),
            )
        };

        // Newly critical among the unresolved, plus stale-carve
        // replacements for relayered criticals.
        let relayered_candidates = relayered_carve_candidates(
            &self.critical,
            &self.carve_edits,
            &self.last_layer_map,
            &report,
        );

        // Promote the newly critical among the unresolved — but
        // bounded: the re-probe's quantile cut would otherwise promote
        // a fresh fraction of whatever is left every round, and once
        // half the board is "critical" the hard-first ordering only
        // starves the easy work. Cap each round and the set overall.
        let max_total = (configuration.ratlines.len() / 4).max(8);
        let remaining = max_total.saturating_sub(self.critical.len());
        let mut promoted: Vec<RatlineUid> = unrouted
            .intersection(&report.critical)
            .copied()
            .filter(|ratline| !self.critical.contains(ratline))
            .collect();
        promoted.sort_by(|a, b| {
            report
                .scores
                .get(b)
                .copied()
                .unwrap_or(0.0)
                .total_cmp(&report.scores.get(a).copied().unwrap_or(0.0))
        });
        promoted.truncate(remaining.min(8));
        add(&METRICS.critical_promotions, promoted.len());
        self.critical.extend(promoted.iter().copied());
        for (ratline, score) in report.scores.iter() {
            self.scores.insert(*ratline, *score);
        }

        let mut carve_candidates: Vec<RatlineUid> = promoted
            .iter()
            .copied()
            .chain(relayered_candidates)
            .collect();
        carve_candidates.sort_by(|a, b| {
            self.scores
                .get(b)
                .copied()
                .unwrap_or(0.0)
                .total_cmp(&self.scores.get(a).copied().unwrap_or(0.0))
        });
        if self.options.carve {
            for ratline in carve_candidates {
                if self.carve_edits.len() >= self.options.max_carves {
                    break;
                }
                // As in `install_critical`, relaxed-probe failures get
                // the endpoint chord as their demand path.
                let chord = || {
                    let (origin, destination) = *endpoints.get(&ratline)?;
                    let drawing = autorouter.board().layout().drawing();
                    let (o, d) = (
                        drawing.primitive_ref(origin),
                        drawing.primitive_ref(destination),
                    );
                    (o.layer() == d.layer())
                        .then(|| vec![(o.weight().pos(), o.layer()), (d.weight().pos(), d.layer())])
                };
                let path = report.paths.get(&ratline).cloned().or_else(chord);
                if let Some(path) = path {
                    if let Some(edit) = carve::carve_corridor(
                        autorouter,
                        ratline,
                        &path,
                        router_options.routed_band_width,
                    ) {
                        self.carve_edits.insert(ratline, edit.clone());
                        self.carve_edit.merge(edit);
                    }
                }
            }
        }

        let planar = self.inner.planar_mut();
        planar.seed_congestion(report.blockers.values().flatten().copied());
        if self.options.protect {
            planar.set_protected(self.critical.iter().copied().collect());
        }

        // Resume at the attempted ratline with the promoted ones
        // spliced right after it — they are routed next, while their
        // corridors are still warm.
        promoted.sort_by(|a, b| {
            self.scores
                .get(b)
                .copied()
                .unwrap_or(0.0)
                .total_cmp(&self.scores.get(a).copied().unwrap_or(0.0))
        });
        let promoted_set: BTreeSet<RatlineUid> = promoted.iter().copied().collect();
        let mut order: Vec<RatlineUid> = configuration
            .ratlines
            .iter()
            .copied()
            .filter(|ratline| !promoted_set.contains(ratline))
            .collect();
        let insert_at = configuration
            .ratlines
            .get(curr)
            .and_then(|attempted| order.iter().position(|ratline| ratline == attempted))
            .map(|index| index + 1)
            .unwrap_or(order.len());
        order.splice(insert_at..insert_at, promoted.iter().copied());

        let new_configuration = PlanarAutorouteConfiguration {
            ratlines: order,
            rip_up: vec![],
            protected: self.critical.clone(),
            ..self.inner.planar().configuration().clone()
        };
        self.inner
            .planar_mut()
            .commit_surgical_reconfiguration(autorouter, new_configuration)?;

        Ok(())
    }
}

/// Criticals whose carve was just removed as stale (relayered) and
/// which the fresh probe still demands — eligible for a re-carve even
/// though they are already in the critical set. Restricted to
/// multilayer (`layer_map` nonempty) so a planar critical whose carve
/// could not be placed is not retried every round.
fn relayered_carve_candidates(
    critical: &BTreeSet<RatlineUid>,
    carve_edits: &BTreeMap<RatlineUid, BoardEdit>,
    layer_map: &BTreeMap<RatlineUid, usize>,
    report: &CriticalityReport,
) -> Vec<RatlineUid> {
    report
        .paths
        .keys()
        .copied()
        .filter(|ratline| {
            critical.contains(ratline)
                && !carve_edits.contains_key(ratline)
                && layer_map.contains_key(ratline)
        })
        .collect()
}

impl<M: AccessMesadata + Clone> Step<Autorouter<M>, Option<BoardEdit>, CriticalAutorouteStatus>
    for CriticalAutorouteExecutionStepper
{
    type Error = AutorouterError;

    fn step(
        &mut self,
        autorouter: &mut Autorouter<M>,
    ) -> Result<ControlFlow<Option<BoardEdit>, CriticalAutorouteStatus>, AutorouterError> {
        match self.inner.step(autorouter)? {
            ControlFlow::Break(maybe_edit) => {
                if !self.options.keep_carves {
                    let removal = carve::remove_dead_carves(autorouter, &mut self.carve_edits);
                    self.carve_edit.merge(removal);
                }
                let mut edit = maybe_edit.unwrap_or_default();
                edit.merge(std::mem::replace(&mut self.carve_edit, BoardEdit::new()));
                Ok(ControlFlow::Break(Some(edit)))
            }
            ControlFlow::Continue(InnerStatus::Running(status)) => Ok(ControlFlow::Continue(
                CriticalAutorouteStatus::Running(status),
            )),
            ControlFlow::Continue(
                reconfigured @ (InnerStatus::Reconfigured | InnerStatus::LayerReconfigured),
            ) => {
                self.seen_reconfigurations += 1;
                // Under multilayer re-identification only ever reseeds
                // congestion — ordering, carving, and protection are
                // planar mechanisms — and the probe pass is too
                // expensive to spend on that alone.
                if self.options.multilayer.is_none()
                    && self.reprobe_rounds < self.options.max_reprobe_rounds
                    && self.options.reprobe_every > 0
                    && self
                        .seen_reconfigurations
                        .is_multiple_of(self.options.reprobe_every)
                {
                    self.reidentify(autorouter)?;
                }
                Ok(ControlFlow::Continue(match reconfigured {
                    InnerStatus::Reconfigured => CriticalAutorouteStatus::Reconfigured,
                    _ => CriticalAutorouteStatus::LayerReconfigured,
                }))
            }
        }
    }
}

impl CriticalAutorouteExecutionStepper {
    /// Finishes routing early, keeping already routed bands and carves.
    /// The next `step()` produces the accumulated edit.
    pub fn finish_early(&mut self, autorouter: &mut Autorouter<impl AccessMesadata + Clone>) {
        match &mut self.inner {
            InnerRouter::Planar(inner) => inner.finish_early(autorouter),
            InnerRouter::Multilayer(inner) => inner.finish_early(autorouter),
        }
    }
}

impl<M: AccessMesadata + Clone> Abort<Autorouter<M>> for CriticalAutorouteExecutionStepper {
    fn abort(&mut self, autorouter: &mut Autorouter<M>) {
        match &mut self.inner {
            InnerRouter::Planar(inner) => inner.abort(autorouter),
            InnerRouter::Multilayer(inner) => inner.abort(autorouter),
        }
        autorouter.board.apply_edit(&self.carve_edit.reverse());
    }
}

impl EstimateProgress for CriticalAutorouteExecutionStepper {
    type Value = usize;
    type Subscale = LinearScale<f64>;

    fn estimate_progress(&self) -> LinearScale<usize, LinearScale<f64>> {
        match &self.inner {
            InnerRouter::Planar(inner) => inner.estimate_progress(),
            InnerRouter::Multilayer(inner) => inner.estimate_progress(),
        }
    }
}

impl GetTimeoutProgress for CriticalAutorouteExecutionStepper {
    type Subscale = LinearScale<f64>;

    fn timeout_progress(&self) -> Option<LinearScale<f64, LinearScale<f64>>> {
        match &self.inner {
            // The planar inner's timeout progress is a bare scale —
            // matching `ExecutionStepper::timeout_progress`, which
            // reports none for planar autorouting.
            InnerRouter::Planar(_) => None,
            InnerRouter::Multilayer(inner) => inner.timeout_progress(),
        }
    }
}

impl GetDebugOverlayData for CriticalAutorouteExecutionStepper {
    fn maybe_thetastar(&self) -> Option<&ThetastarStepper<Navmesh, f64>> {
        match &self.inner {
            InnerRouter::Planar(inner) => inner.maybe_thetastar(),
            InnerRouter::Multilayer(inner) => inner.maybe_thetastar(),
        }
    }

    fn maybe_navcord(&self) -> Option<&Navcord> {
        match &self.inner {
            InnerRouter::Planar(inner) => inner.maybe_navcord(),
            InnerRouter::Multilayer(inner) => inner.maybe_navcord(),
        }
    }

    fn ghosts(&self) -> &[PrimitiveShape] {
        match &self.inner {
            InnerRouter::Planar(inner) => inner.ghosts(),
            InnerRouter::Multilayer(inner) => inner.ghosts(),
        }
    }

    fn obstacles(&self) -> &[crate::drawing::graph::PrimitiveIndex] {
        match &self.inner {
            InnerRouter::Planar(inner) => inner.obstacles(),
            InnerRouter::Multilayer(inner) => inner.obstacles(),
        }
    }
}
