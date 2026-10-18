// SPDX-FileCopyrightText: 2025 Topola contributors
//
// SPDX-License-Identifier: MIT

use std::{collections::BTreeMap, ops::ControlFlow};

use specctra_core::mesadata::AccessMesadata;

use crate::{
    autorouter::{
        escape_stubs,
        invoker::GetDebugOverlayData,
        planar_autoroute::{
            PlanarAutorouteConfiguration, PlanarAutorouteConfigurationStatus,
            PlanarAutorouteContinueStatus, PlanarAutorouteExecutionStepper,
        },
        planar_preconfigurer::{
            PlanarAutoroutePreconfigurerInput, PreconfigurePlanarAutoroute, PresortParams,
            SccIntersectionsAndLengthRatlinePlanarAutoroutePreconfigurer,
        },
        planar_reconfigurer::{MakeNextPlanarAutorouteConfiguration, PlanarAutorouteReconfigurer},
        ratline::RatlineUid,
        Autorouter, AutorouterError, PlanarAutorouteOptions,
    },
    board::edit::{BoardDataEdit, BoardEdit},
    drawing::{dot::FixedDotIndex, graph::PrimitiveIndex},
    geometry::{edit::Edit, primitive::PrimitiveShape},
    layout::LayoutEdit,
    router::{navcord::Navcord, navmesh::Navmesh, thetastar::ThetastarStepper},
    stepper::{
        Abort, EstimateProgress, GetTimeoutProgress, LinearScale, ReconfiguratorStatus,
        Reconfigure, Step, TimeVsProgressAccumulatorTimeout,
    },
};

pub type PlanarAutorouteReconfiguratorStatus =
    ReconfiguratorStatus<PlanarAutorouteConfigurationStatus, PlanarAutorouteContinueStatus>;

pub struct PlanarAutorouteReconfigurator {
    stepper: PlanarAutorouteExecutionStepper,
    timeout: TimeVsProgressAccumulatorTimeout,
    reconfigurer: PlanarAutorouteReconfigurer,
    options: PlanarAutorouteOptions,
    /// Geometry of the escape stubs laid before routing started. Kept
    /// out of the stepper's per-ratline edits so backtracing never
    /// removes stubs that `terminating_dot_map` still points at; merged
    /// into the command's edit when it is produced, reversed on abort.
    escape_stub_edit: LayoutEdit,
    /// Set by [`PlanarAutorouteReconfigurator::finish_early`] to prevent any
    /// further reconfiguration once early finishing is requested.
    finished: bool,
    reconfiguration_count: usize,
}

impl PlanarAutorouteReconfigurator {
    pub fn new(
        autorouter: &mut Autorouter<impl AccessMesadata>,
        input: PlanarAutoroutePreconfigurerInput,
        options: PlanarAutorouteOptions,
    ) -> Result<Self, AutorouterError> {
        let preconfigurer = SccIntersectionsAndLengthRatlinePlanarAutoroutePreconfigurer::new(
            autorouter,
            input.clone(),
            &PresortParams {
                intersector_count_weight: 1.0,
                length_weight: 0.001,
            },
            &options,
        );
        let mut preconfiguration = preconfigurer.preconfigure(autorouter, input);

        let mut escape_stub_edit = LayoutEdit::new();
        if options.escape_stubs {
            let stats = escape_stubs::anteroute_escape_stubs(
                autorouter,
                &mut escape_stub_edit,
                &preconfiguration.ratlines,
                &mut preconfiguration.terminating_dot_map,
                &options,
            );
            log::debug!("escape stubs: {stats:?}");
        }

        let reconfigurer = PlanarAutorouteReconfigurer::new(
            autorouter,
            preconfiguration.clone(),
            preconfigurer,
            &options,
        );

        Ok(Self {
            stepper: PlanarAutorouteExecutionStepper::new(autorouter, preconfiguration, options)?,
            timeout: TimeVsProgressAccumulatorTimeout::new_from_options(options.timeout),
            // Note: I assume here that the first permutation is the same as the original order.
            reconfigurer,
            options,
            escape_stub_edit,
            finished: false,
            reconfiguration_count: 0,
        })
    }

    /// Begins a surgical reconfiguration driven from outside the
    /// reconfigurer: rips up the bands of `rip_up` ratlines (resolved
    /// under the live configuration's terminating-dot map) and folds
    /// their edit slots into the stepper's pending edit. Leaves no
    /// route in flight.
    ///
    /// Must be followed by [`Self::commit_surgical_reconfiguration`]
    /// before stepping resumes. Between the two calls the caller may
    /// modify the board — e.g. swap the ripped ratlines' fanout vias
    /// for multilayer re-layering — and call
    /// [`Self::remap_terminating_dots`] to update the remappings the
    /// requeued ratlines will route to.
    pub fn begin_surgical_reconfiguration(
        &mut self,
        autorouter: &mut Autorouter<impl AccessMesadata>,
        rip_up: &[RatlineUid],
    ) -> Result<(), AutorouterError> {
        self.stepper.surgical_rip_up(autorouter, rip_up)
    }

    /// Installs `new_configuration` after
    /// [`Self::begin_surgical_reconfiguration`] and resumes routing.
    /// Its `rip_up` list is informational only — the bands are already
    /// gone — but is kept for the resume-position bookkeeping.
    pub fn commit_surgical_reconfiguration(
        &mut self,
        autorouter: &mut Autorouter<impl AccessMesadata>,
        new_configuration: PlanarAutorouteConfiguration,
    ) -> Result<PlanarAutorouteConfigurationStatus, AutorouterError> {
        self.reconfiguration_count += 1;
        self.timeout = TimeVsProgressAccumulatorTimeout::new_from_options(self.options.timeout);
        self.stepper
            .install_configuration(autorouter, new_configuration)
    }

    /// Extends the terminating-dot remappings in both the stepper's
    /// live configuration and the reconfigurer's stored
    /// preconfiguration, so emitted configurations keep them.
    pub fn remap_terminating_dots(
        &mut self,
        entries: &BTreeMap<(RatlineUid, FixedDotIndex), FixedDotIndex>,
    ) {
        self.stepper.remap_terminating_dots(entries);
        self.reconfigurer.remap_terminating_dots(entries);
    }

    /// The live configuration of the inner stepper — the ratline order
    /// and terminating-dot map routing is currently working against.
    pub fn configuration(&self) -> &PlanarAutorouteConfiguration {
        self.stepper.configuration()
    }

    /// Index of the ratline currently being attempted in the live
    /// configuration's ordering.
    pub fn curr_ratline_index(&self) -> usize {
        *self.stepper.curr_ratline_index()
    }

    /// Blocking ratlines recorded for the most recent routing
    /// failure, in their original routing order.
    pub fn blockers(&self) -> &[RatlineUid] {
        self.reconfigurer.blockers()
    }

    /// Finishes routing early, keeping already routed bands. The next
    /// `step()` produces the accumulated edit without triggering another
    /// reconfiguration.
    pub fn finish_early(&mut self, autorouter: &mut Autorouter<impl AccessMesadata>) {
        self.stepper.finish_early(autorouter);
        self.finished = true;
    }

    /// Whether the number of reconfigurations has reached
    /// [`PlanarAutorouteOptions::max_reconfigurations`]. Always `false` when
    /// no limit is set.
    fn reconfiguration_budget_exhausted(&self) -> bool {
        self.options
            .max_reconfigurations
            .is_some_and(|max| self.reconfiguration_count >= max)
    }

    fn reconfigure(
        &mut self,
        autorouter: &mut Autorouter<impl AccessMesadata>,
    ) -> Result<ControlFlow<Option<BoardEdit>, PlanarAutorouteReconfiguratorStatus>, AutorouterError>
    {
        self.reconfiguration_count += 1;
        self.timeout = TimeVsProgressAccumulatorTimeout::new_from_options(self.options.timeout);

        loop {
            let Some(configuration) = self
                .reconfigurer
                .next_configuration(autorouter, &self.stepper)
            else {
                return Ok(ControlFlow::Break(None));
            };

            match self.stepper.reconfigure(autorouter, configuration) {
                Ok(result) => {
                    return Ok(ControlFlow::Continue(ReconfiguratorStatus::Reconfigured(
                        result,
                    )))
                }
                Err(AutorouterError::NothingToUndoForReconfiguration) => continue,
                Err(err) => return Err(err),
            }
        }
    }
}

impl<M: AccessMesadata> Step<Autorouter<M>, Option<BoardEdit>, PlanarAutorouteReconfiguratorStatus>
    for PlanarAutorouteReconfigurator
{
    type Error = AutorouterError;

    fn step(
        &mut self,
        autorouter: &mut Autorouter<M>,
    ) -> Result<ControlFlow<Option<BoardEdit>, PlanarAutorouteReconfiguratorStatus>, AutorouterError>
    {
        // Once the stepper has routed every ratline it must not be
        // reconfigured: the next `step()` produces the merged edit, and
        // reconfiguring here would backtrace away a completed
        // configuration — the in-flight route is gone by then, so the
        // timeout's subscale no longer accrues budget.
        let stepper_done =
            *self.stepper.curr_ratline_index() >= self.stepper.configuration().ratlines.len();

        if !self.finished
            && !stepper_done
            && !self
                .timeout
                .update(*self.estimate_progress().subscale().value() as f64)
        {
            if self.reconfiguration_budget_exhausted() {
                self.finish_early(autorouter);
            } else {
                return self.reconfigure(autorouter);
            }
        }

        match self.stepper.step(autorouter) {
            Ok(ControlFlow::Break(maybe_edit)) => {
                Ok(ControlFlow::Break(maybe_edit.map(|edit| {
                    let mut merged = BoardEdit::new_from_edits(
                        BoardDataEdit::new(),
                        self.escape_stub_edit.clone(),
                    );
                    merged.merge(edit);
                    merged
                })))
            }
            Ok(ControlFlow::Continue(status)) => {
                Ok(ControlFlow::Continue(ReconfiguratorStatus::Running(status)))
            }
            Err(err) => {
                if self.finished || !self.options.permutate {
                    return Err(err);
                }
                if self.reconfiguration_budget_exhausted() {
                    // Out of reconfiguration attempts: finish early, keeping
                    // the bands routed so far instead of failing the whole
                    // command.
                    self.finish_early(autorouter);
                    return self.step(autorouter);
                }
                self.reconfigure(autorouter)
            }
        }
    }
}

impl<M: AccessMesadata> Abort<Autorouter<M>> for PlanarAutorouteReconfigurator {
    fn abort(&mut self, autorouter: &mut Autorouter<M>) {
        self.stepper.abort(autorouter);
        autorouter.board.apply_edit(
            &BoardEdit::new_from_edits(
                BoardDataEdit::new(),
                self.escape_stub_edit.clone(),
            )
            .reverse(),
        );
    }
}

impl EstimateProgress for PlanarAutorouteReconfigurator {
    type Value = usize;
    type Subscale = LinearScale<f64>;

    fn estimate_progress(&self) -> LinearScale<usize, LinearScale<f64>> {
        self.stepper.estimate_progress()
    }
}

impl GetTimeoutProgress for PlanarAutorouteReconfigurator {
    type Subscale = ();

    fn timeout_progress(&self) -> Option<LinearScale<f64>> {
        Some(LinearScale::new(
            self.timeout.start_instant().elapsed().as_secs_f64(),
            *self.timeout.progress_accumulator(),
            (),
        ))
    }
}

impl GetDebugOverlayData for PlanarAutorouteReconfigurator {
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
