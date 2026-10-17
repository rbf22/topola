// SPDX-FileCopyrightText: 2025 Topola contributors
//
// SPDX-License-Identifier: MIT

use std::ops::ControlFlow;

use specctra_core::mesadata::AccessMesadata;

use crate::{
    autorouter::{
        invoker::GetDebugOverlayData,
        planar_autoroute::{
            PlanarAutorouteConfigurationStatus, PlanarAutorouteContinueStatus,
            PlanarAutorouteExecutionStepper,
        },
        planar_preconfigurer::{
            PlanarAutoroutePreconfigurerInput, PreconfigurePlanarAutoroute, PresortParams,
            SccIntersectionsAndLengthRatlinePlanarAutoroutePreconfigurer,
        },
        planar_reconfigurer::{MakeNextPlanarAutorouteConfiguration, PlanarAutorouteReconfigurer},
        Autorouter, AutorouterError, PlanarAutorouteOptions,
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

pub type PlanarAutorouteReconfiguratorStatus =
    ReconfiguratorStatus<PlanarAutorouteConfigurationStatus, PlanarAutorouteContinueStatus>;

pub struct PlanarAutorouteReconfigurator {
    stepper: PlanarAutorouteExecutionStepper,
    timeout: TimeVsProgressAccumulatorTimeout,
    reconfigurer: PlanarAutorouteReconfigurer,
    options: PlanarAutorouteOptions,
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
        let preconfiguration = preconfigurer.preconfigure(autorouter, input);
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
            finished: false,
            reconfiguration_count: 0,
        })
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
        if !self.finished
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
            Ok(ControlFlow::Break(maybe_edit)) => Ok(ControlFlow::Break(maybe_edit)),
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
