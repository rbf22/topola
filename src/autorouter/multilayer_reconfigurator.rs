// SPDX-FileCopyrightText: 2025 Topola contributors
//
// SPDX-License-Identifier: MIT

use std::{collections::BTreeMap, ops::ControlFlow};

use specctra_core::mesadata::AccessMesadata;

use crate::{
    autorouter::{
        invoker::GetDebugOverlayData,
        multilayer_autoroute::{
            MultilayerAutorouteConfiguration, MultilayerAutorouteExecutionStepper,
            MultilayerAutorouteOptions,
        },
        multilayer_preconfigurer::{
            MultilayerAutoroutePreconfigurerInput, MultilayerPreconfigurer,
        },
        multilayer_reconfigurer::{
            IncrementFailedRatlineLayersMultilayerAutorouteReconfigurer,
            MakeNextMultilayerAutorouteConfiguration, MultilayerAutorouteReconfigurer,
        },
        planar_autoroute::{
            PlanarAutorouteConfiguration, PlanarAutorouteConfigurationStatus, PlanarAutorouteCosts,
        },
        planar_preconfigurer::PlanarAutoroutePreconfigurerInput,
        planar_reconfigurator::{
            PlanarAutorouteReconfigurator, PlanarAutorouteReconfiguratorStatus,
        },
        ratline::RatlineUid,
        Autorouter, AutorouterError,
    },
    board::edit::BoardEdit,
    drawing::graph::PrimitiveIndex,
    geometry::primitive::PrimitiveShape,
    router::{navcord::Navcord, navmesh::Navmesh, thetastar::ThetastarStepper},
    stepper::{
        Abort, EstimateProgress, GetTimeoutProgress, LinearScale, ReconfiguratorStatus,
        Reconfigure, Step, TimeVsProgressAccumulatorTimeout,
    },
};

/// What a multilayer reconfiguration step actually changed — the bare
/// `Reconfigured` status alone cannot distinguish a pass-through of an
/// inner planar reconfiguration from a real layer-level move.
#[derive(Clone, Copy, Debug)]
pub enum MultilayerReconfigKind {
    /// Only the inner planar machinery reconfigured (ordering change or
    /// backtrace); no layer-level geometry changed.
    Planar,
    /// A mid-route via weave was applied or broken weaves were undone.
    Weave,
    /// A layer rotation / layer-map reassignment was applied.
    LayerRotation,
}

pub type MultilayerReconfiguratorStatus =
    ReconfiguratorStatus<MultilayerReconfigKind, PlanarAutorouteReconfiguratorStatus>;

pub struct MultilayerAutorouteReconfigurator {
    stepper: MultilayerAutorouteExecutionStepper,
    timeout: TimeVsProgressAccumulatorTimeout,
    reconfigurer: MultilayerAutorouteReconfigurer,
    options: MultilayerAutorouteOptions,
    /// Set by [`MultilayerAutorouteReconfigurator::finish_early`] to prevent
    /// any further reconfiguration once early finishing is requested.
    finished: bool,
    reconfiguration_count: usize,
    /// Furthest ratline index reached by any planar configuration —
    /// the frontier layer rotations are meant to advance.
    furthest_index: usize,
    /// Planar reconfigurations since the frontier last advanced. Once
    /// it stalls, escalate: pop a layer-rotated configuration instead
    /// of letting orderings permute forever under the same congestion.
    stalls: usize,
}

impl MultilayerAutorouteReconfigurator {
    pub fn new(
        autorouter: &mut Autorouter<impl AccessMesadata>,
        input: MultilayerAutoroutePreconfigurerInput,
        options: MultilayerAutorouteOptions,
    ) -> Result<Self, AutorouterError> {
        Self::new_with_planar_override(autorouter, input, options, |_, _| {})
    }

    /// Like [`MultilayerAutorouteReconfigurator::new`], but
    /// `planar_override` runs on the inner planar reconfigurator's
    /// freshly built preconfiguration (after anterouting, so
    /// terminating dots name the fanout vias). The critical-first
    /// outer stepper uses it to inject its ordering, carve corridors
    /// on the assigned layers, and seed the protected set.
    pub fn new_with_planar_override<M: AccessMesadata>(
        autorouter: &mut Autorouter<M>,
        mut input: MultilayerAutoroutePreconfigurerInput,
        options: MultilayerAutorouteOptions,
        planar_override: impl FnOnce(&mut Autorouter<M>, &mut PlanarAutorouteConfiguration),
    ) -> Result<Self, AutorouterError> {
        // Optional simulated-annealing pre-plan: a global annealer
        // decides layer assignments, planned mid-route layer hops
        // (committed as ordinary weaves), the contestedness order, the
        // congestion seed, and carved corridors. Everything it
        // produces is validated before it lands — an unplaceable via
        // degrades its split to a plain layer assignment.
        let mut annealed = None;
        let preconfigurer = if let Some(anneal_options) = options.anneal {
            let plan = crate::autorouter::anneal::run(
                autorouter,
                &input.ratlines,
                &anneal_options,
                options.planar.router.routed_band_width,
            );
            let handoff = crate::autorouter::anneal::handoff::apply(
                autorouter,
                &mut input.ratlines,
                plan,
                &options,
            );
            let preconfigurer = MultilayerPreconfigurer::new_from_layer_map(
                autorouter,
                &input.ratlines,
                handoff.layer_map.clone(),
            );
            annealed = Some(handoff);
            preconfigurer
        } else {
            // One band width: demand corridors closer than that cannot
            // both fit on one layer without a detour.
            MultilayerPreconfigurer::new_seeded(
                autorouter,
                input.clone(),
                options.layer_seed,
                options.planar.router.routed_band_width,
            )
        };

        let preconfiguration = MultilayerAutorouteConfiguration {
            plan: preconfigurer.plan().clone(),
            planar: PlanarAutoroutePreconfigurerInput {
                ratlines: input.ratlines.clone(),
                terminating_dot_map: BTreeMap::new(),
            },
        };
        let reconfigurer = MultilayerAutorouteReconfigurer::UniformRandomLayers(
            IncrementFailedRatlineLayersMultilayerAutorouteReconfigurer::new(
                autorouter,
                preconfiguration.clone(),
                &options,
            ),
        );

        let order = annealed.as_ref().map(|handoff| handoff.order.clone());
        let mut stepper = MultilayerAutorouteExecutionStepper::new_with_planar_override(
            autorouter,
            preconfiguration,
            options,
            |autorouter, configuration| {
                if let Some(order) = &order {
                    // The annealed contestedness order: every ratline
                    // the plan did not rank keeps its preset position.
                    let rank: BTreeMap<RatlineUid, usize> = order
                        .iter()
                        .enumerate()
                        .map(|(rank, ratline)| (*ratline, rank))
                        .collect();
                    configuration
                        .ratlines
                        .sort_by_key(|ratline| rank.get(ratline).copied().unwrap_or(usize::MAX));
                }
                planar_override(autorouter, configuration);
            },
        )?;
        if let Some(handoff) = annealed {
            stepper.adopt_planned_weaves(autorouter, handoff.edit, handoff.weaves);
        }

        Ok(Self {
            stepper,
            timeout: TimeVsProgressAccumulatorTimeout::new_from_options(options.timeout),
            reconfigurer,
            options,
            finished: false,
            reconfiguration_count: 0,
            furthest_index: 0,
            stalls: 0,
        })
    }

    /// Finishes routing early, keeping already routed bands. The next
    /// `step()` produces the accumulated edit without triggering another
    /// reconfiguration.
    pub fn finish_early(&mut self, autorouter: &mut Autorouter<impl AccessMesadata>) {
        self.stepper.finish_early(autorouter);
        self.finished = true;
    }

    /// The inner planar reconfigurator.
    pub fn planar(&self) -> &PlanarAutorouteReconfigurator {
        self.stepper.planar()
    }

    /// Mutable access to the inner planar reconfigurator — for the
    /// critical-first outer stepper's surgical windows and protected
    /// bookkeeping.
    pub fn planar_mut(&mut self) -> &mut PlanarAutorouteReconfigurator {
        self.stepper.planar_mut()
    }

    /// The layer assignments the placed fanout vias correspond to —
    /// diffed between critical-first re-identification rounds to find
    /// ratlines whose carves went stale on rotation.
    pub fn layer_map(&self) -> &BTreeMap<RatlineUid, usize> {
        self.stepper.layer_map()
    }

    /// Whether the number of layer reassignments has reached
    /// `options.planar.max_reconfigurations`. Always `false` when no limit
    /// is set.
    fn reconfiguration_budget_exhausted(&self) -> bool {
        self.options
            .planar
            .max_reconfigurations
            .is_some_and(|max| self.reconfiguration_count >= max)
    }

    fn reconfigure(
        &mut self,
        autorouter: &mut Autorouter<impl AccessMesadata>,
    ) -> Result<ControlFlow<Option<BoardEdit>, MultilayerReconfiguratorStatus>, AutorouterError>
    {
        self.reconfiguration_count += 1;

        crate::metrics::incr(&crate::metrics::METRICS.layer_reconfigurations);
        let _reconfigure_timer =
            crate::metrics::Timer::start(&crate::metrics::METRICS.reconfigure_nanos);

        // Reset the reconfiguration trigger.
        self.timeout = TimeVsProgressAccumulatorTimeout::new_from_options(self.options.timeout);

        loop {
            let Some(mut configuration) = self.reconfigurer.next_configuration(autorouter) else {
                return Ok(ControlFlow::Break(None));
            };

            // Ratlines declared dead by the planar reconfigurator are
            // unroutable: keep layer rotations from re-attempting them
            // so the frontier advances instead of cycling the same
            // sealed ratline through every layer forever.
            let dead = self.stepper.planar().dead_ratlines();
            if !dead.is_empty() {
                configuration
                    .planar
                    .ratlines
                    .retain(|ratline| !dead.contains(ratline));
                configuration
                    .planar
                    .terminating_dot_map
                    .retain(|(ratline, _), _| !dead.contains(ratline));
            }

            match self.stepper.reconfigure(autorouter, configuration) {
                Ok(_) => {
                    return Ok(ControlFlow::Continue(ReconfiguratorStatus::Reconfigured(
                        MultilayerReconfigKind::LayerRotation,
                    )))
                }
                Err(AutorouterError::NothingToUndoForReconfiguration) => continue,
                Err(err) => return Err(err),
            }
        }
    }
}

impl<M: AccessMesadata> Step<Autorouter<M>, Option<BoardEdit>, MultilayerReconfiguratorStatus>
    for MultilayerAutorouteReconfigurator
{
    type Error = AutorouterError;

    fn step(
        &mut self,
        autorouter: &mut Autorouter<M>,
    ) -> Result<ControlFlow<Option<BoardEdit>, MultilayerReconfiguratorStatus>, AutorouterError>
    {
        if !self.finished
            && !self
                .timeout
                .update(*self.estimate_progress().value() as f64)
        {
            if self.reconfiguration_budget_exhausted() {
                self.finish_early(autorouter);
            } else {
                return self.reconfigure(autorouter);
            }
        }

        match self.stepper.step(autorouter) {
            Ok(ControlFlow::Break(maybe_edit)) => {
                // Planar stopped — either its ordering space
                // exhausted mid-route (Break(None)) or it finished
                // early on a reconfiguration budget/timeout with
                // bands still missing (Break(Some)). A ratline
                // sealed on its assigned layer may route on another,
                // so give the layer search its turn before ending the
                // command: the frontier names a ratline that failed
                // to route, which the search rotates first.
                let frontier = if maybe_edit.is_none() {
                    Some(self.stepper.planar().curr_ratline_index())
                } else {
                    let configuration = self.stepper.planar().configuration();
                    (0..configuration.ratlines.len()).find(|&index| {
                        let (origin, destination) =
                            configuration.ratline_terminating_dots(autorouter, index);
                        autorouter
                            .board()
                            .band_between_nodes(origin, destination)
                            .is_none()
                    })
                };

                if let (Some(frontier), false, false) = (
                    frontier,
                    self.finished,
                    self.reconfiguration_budget_exhausted(),
                ) {
                    self.reconfigurer.process_planar_result(
                        autorouter,
                        Ok(PlanarAutorouteConfigurationStatus {
                            configuration: self.stepper.planar().configuration().clone(),
                            costs: PlanarAutorouteCosts {
                                lengths: vec![0.0; frontier],
                            },
                        }),
                        self.stepper.planar().blockers(),
                    );
                    if let ret @ (ControlFlow::Continue(..) | ControlFlow::Break(Some(..))) =
                        self.reconfigure(autorouter)?
                    {
                        return Ok(ret);
                    }
                }
                // The command is really done — surviving weave
                // continuations leave the ratsnest before history
                // and undo can see their dead-end vertices.
                self.stepper.remove_continuations(autorouter);
                Ok(ControlFlow::Break(maybe_edit))
            }
            Ok(ControlFlow::Continue(ReconfiguratorStatus::Running(status))) => {
                Ok(ControlFlow::Continue(ReconfiguratorStatus::Running(
                    ReconfiguratorStatus::Running(status),
                )))
            }
            Ok(ControlFlow::Continue(ReconfiguratorStatus::Reconfigured(status))) => {
                // The planar frontier is the index of the ratline that
                // failed this round; ordering retries that never get
                // past it mean the contention is layer-local — the
                // congestion may dissolve entirely on another layer.
                let reached = status.costs.lengths.len();
                let stalled_ratline = status.configuration.ratlines.get(reached).copied();
                self.reconfigurer.process_planar_result(
                    autorouter,
                    Ok(status),
                    self.stepper.planar().blockers(),
                );
                if reached > self.furthest_index {
                    self.furthest_index = reached;
                    self.stalls = 0;
                } else {
                    self.stalls += 1;
                }
                // A few ordering rounds give negotiated congestion its
                // chance; after that, broader moves are tried:
                // 1. A mid-route via weave on the stalled ratline —
                //    the connection hops layers partway through rather
                //    than needing a whole corridor on one layer.
                // 2. A layer rotation popped from the search queue.
                let mut kind = MultilayerReconfigKind::Planar;
                const STALL_LIMIT: usize = 4;
                if self.stalls >= STALL_LIMIT && !self.finished {
                    self.stalls = 0;
                    // Undo weaves whose chain is broken — either half
                    // unrouted — so their parents regain rotation
                    // freedom instead of staying anchored to a
                    // connection that cannot complete.
                    let broken = self.stepper.broken_weaves(autorouter);
                    if !broken.is_empty() {
                        for parent in broken {
                            self.stepper.unweave_ratline(autorouter, parent)?;
                        }
                        kind = MultilayerReconfigKind::Weave;
                    } else {
                        let wove = stalled_ratline
                            .map(|ratline| self.stepper.weave_ratline(autorouter, ratline))
                            .transpose()?
                            .unwrap_or(false);
                        if !wove {
                            match self.reconfigure(autorouter)? {
                                // No rotated configurations queued — keep
                                // stepping rather than ending the command.
                                ControlFlow::Break(None) => {}
                                ret => return Ok(ret),
                            }
                        } else {
                            kind = MultilayerReconfigKind::Weave;
                        }
                    }
                }
                Ok(ControlFlow::Continue(ReconfiguratorStatus::Reconfigured(
                    kind,
                )))
            }
            Err(err) => {
                if self.finished {
                    return Err(err);
                }
                if self.reconfiguration_budget_exhausted() {
                    // Out of reconfiguration attempts: finish early, keeping
                    // the bands routed so far instead of failing the whole
                    // command.
                    self.finish_early(autorouter);
                    return self.step(autorouter);
                }
                self.reconfigurer.process_planar_result(
                    autorouter,
                    Err(err.clone()),
                    self.stepper.planar().blockers(),
                );
                // A failed route attempt can leave a weave's chain
                // broken — the continuation half could not complete —
                // while the parent half keeps a dangling band that
                // reports the ratline routed but connects nothing.
                // Rip it now so the parent regains rotation freedom;
                // the stalled-round check only reaches broken weaves
                // via the Reconfigured branch, which errors bypass.
                let broken = self.stepper.broken_weaves(autorouter);
                for parent in broken {
                    self.stepper.unweave_ratline(autorouter, parent)?;
                }
                self.reconfigure(autorouter)
            }
        }
    }
}

impl<M: AccessMesadata> Abort<Autorouter<M>> for MultilayerAutorouteReconfigurator {
    fn abort(&mut self, autorouter: &mut Autorouter<M>) {
        self.stepper.abort(autorouter)
    }
}

impl EstimateProgress for MultilayerAutorouteReconfigurator {
    type Value = usize;
    type Subscale = LinearScale<f64>;

    fn estimate_progress(&self) -> LinearScale<usize, LinearScale<f64>> {
        self.stepper.estimate_progress()
    }
}

impl GetTimeoutProgress for MultilayerAutorouteReconfigurator {
    type Subscale = LinearScale<f64>;

    fn timeout_progress(&self) -> Option<LinearScale<f64, LinearScale<f64>>> {
        Some(LinearScale::new(
            self.timeout.start_instant().elapsed().as_secs_f64(),
            *self.timeout.progress_accumulator(),
            self.stepper.planar().timeout_progress()?,
        ))
    }
}

impl GetDebugOverlayData for MultilayerAutorouteReconfigurator {
    fn maybe_thetastar(&self) -> Option<&ThetastarStepper<Navmesh, f64>> {
        self.stepper.maybe_thetastar()
    }

    fn maybe_navcord(&self) -> Option<&Navcord> {
        self.stepper.maybe_navcord()
    }

    fn ghosts(&self) -> &[PrimitiveShape] {
        self.stepper.ghosts()
    }

    fn obstacles(&self) -> &[PrimitiveIndex] {
        self.stepper.obstacles()
    }
}
