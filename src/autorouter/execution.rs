// SPDX-FileCopyrightText: 2024 Topola contributors
//
// SPDX-License-Identifier: MIT

use std::{collections::BTreeSet, ops::ControlFlow};

use enum_dispatch::enum_dispatch;
use serde::{Deserialize, Serialize};

use crate::{
    autorouter::{
        critical_autoroute::{
            CriticalAutorouteExecutionStepper, CriticalAutorouteOptions, CriticalAutorouteStatus,
        },
        multilayer_autoroute::MultilayerAutorouteOptions,
        multilayer_reconfigurator::{
            MultilayerAutorouteReconfigurator, MultilayerReconfigKind,
            MultilayerReconfiguratorStatus,
        },
        planar_autoroute::PlanarAutorouteContinueStatus,
        planar_reconfigurator::{
            PlanarAutorouteReconfigurator, PlanarAutorouteReconfiguratorStatus,
        },
    },
    board::{edit::BoardEdit, AccessMesadata},
    layout::via::ViaWeight,
    router::ng,
    stepper::{
        Abort, EstimateProgress, GetTimeoutProgress, LinearScale, ReconfiguratorStatus, Step,
    },
};

use super::{
    invoker::{Invoker, InvokerError},
    measure_length::MeasureLengthExecutionStepper,
    place_via::PlaceViaExecutionStepper,
    remove_bands::RemoveBandsExecutionStepper,
    selection::{BandSelection, PinSelection},
    shift_components::{ComponentShift, ShiftComponentsExecutionStepper},
    Autorouter, PlanarAutorouteOptions,
};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum Command {
    Autoroute(PinSelection, PlanarAutorouteOptions), // TODO: Rename to PlanarAutoroute.
    MultilayerAutoroute(PinSelection, MultilayerAutorouteOptions),
    CriticalAutoroute(PinSelection, CriticalAutorouteOptions),
    TopoAutoroute {
        selection: PinSelection,
        #[serde(default, skip_serializing_if = "BTreeSet::is_empty")]
        allowed_edges: BTreeSet<ng::PieEdgeIndex>,
        active_layer: String,
        routed_band_width: f64,
    },
    PlaceVia(ViaWeight),
    ShiftComponents(Vec<ComponentShift>),
    RemoveBands(BandSelection),
    MeasureLength(BandSelection),
}

/// Classification of what an [`ExecutionStepper`] step did. Returned in
/// the `Continue` payload of [`Step`] so callers can attribute progress
/// and time to routing phases without peeking at stepper internals.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum StepKind {
    /// Incremental drawing of a band (no ratline completed this step).
    Route,
    /// A band was committed this step.
    Routed,
    /// The ratline was already routed and was skipped.
    Skipped,
    /// Planar-level reconfiguration: ordering change, backtrace, or
    /// surgical rip-up.
    PlanarReconfigured,
    /// Multilayer-level reconfiguration: layer rotation, weave, or
    /// unweave.
    LayerReconfigured,
    /// Non-autoroute command work (PlaceVia, ShiftComponents, ...).
    Other,
}

fn continue_status_kind(status: PlanarAutorouteContinueStatus) -> StepKind {
    match status {
        PlanarAutorouteContinueStatus::Running => StepKind::Route,
        PlanarAutorouteContinueStatus::Routed(..) => StepKind::Routed,
        PlanarAutorouteContinueStatus::Skipped(..) => StepKind::Skipped,
    }
}

fn planar_reconfigurator_kind(status: PlanarAutorouteReconfiguratorStatus) -> StepKind {
    match status {
        ReconfiguratorStatus::Running(inner) => continue_status_kind(inner),
        ReconfiguratorStatus::Reconfigured(..) => StepKind::PlanarReconfigured,
    }
}

fn multilayer_reconfigurator_kind(status: MultilayerReconfiguratorStatus) -> StepKind {
    match status {
        ReconfiguratorStatus::Running(inner) => planar_reconfigurator_kind(inner),
        ReconfiguratorStatus::Reconfigured(kind) => match kind {
            MultilayerReconfigKind::Planar => StepKind::PlanarReconfigured,
            MultilayerReconfigKind::Weave | MultilayerReconfigKind::LayerRotation => {
                StepKind::LayerReconfigured
            }
        },
    }
}

fn critical_autoroute_kind(status: CriticalAutorouteStatus) -> StepKind {
    match status {
        CriticalAutorouteStatus::Running(inner) => continue_status_kind(inner),
        CriticalAutorouteStatus::Reconfigured => StepKind::PlanarReconfigured,
        CriticalAutorouteStatus::LayerReconfigured => StepKind::LayerReconfigured,
    }
}

#[enum_dispatch(GetDebugOverlayData)]
pub enum ExecutionStepper<M> {
    MultilayerAutoroute(Box<MultilayerAutorouteReconfigurator>),
    PlanarAutoroute(Box<PlanarAutorouteReconfigurator>),
    CriticalAutoroute(Box<CriticalAutorouteExecutionStepper>),
    TopoAutoroute(Box<ng::AutorouteExecutionStepper<M>>),
    PlaceVia(PlaceViaExecutionStepper),
    ShiftComponents(ShiftComponentsExecutionStepper),
    RemoveBands(RemoveBandsExecutionStepper),
    MeasureLength(MeasureLengthExecutionStepper),
}

impl<M: AccessMesadata + Clone> ExecutionStepper<M> {
    fn step_catch_err(
        &mut self,
        autorouter: &mut Autorouter<M>,
    ) -> Result<ControlFlow<(Option<BoardEdit>, String), StepKind>, InvokerError> {
        Ok(match self {
            ExecutionStepper::MultilayerAutoroute(autoroute) => match autoroute.step(autorouter)? {
                ControlFlow::Continue(status) => {
                    ControlFlow::Continue(multilayer_reconfigurator_kind(status))
                }
                ControlFlow::Break(edit) => {
                    ControlFlow::Break((edit, "finished multilayer autorouting".to_string()))
                }
            },
            ExecutionStepper::PlanarAutoroute(autoroute) => match autoroute.step(autorouter)? {
                ControlFlow::Continue(status) => {
                    ControlFlow::Continue(planar_reconfigurator_kind(status))
                }
                ControlFlow::Break(edit) => {
                    ControlFlow::Break((edit, "finished planar autorouting".to_string()))
                }
            },
            ExecutionStepper::CriticalAutoroute(autoroute) => match autoroute.step(autorouter)? {
                ControlFlow::Continue(status) => {
                    ControlFlow::Continue(critical_autoroute_kind(status))
                }
                ControlFlow::Break(edit) => {
                    ControlFlow::Break((edit, "finished critical autorouting".to_string()))
                }
            },
            ExecutionStepper::TopoAutoroute(autoroute) => {
                let ret = match autoroute.step() {
                    ControlFlow::Continue(()) => ControlFlow::Continue(StepKind::Other),
                    ControlFlow::Break(false) => {
                        ControlFlow::Break((None, "topo-autorouting failed".to_string()))
                    }
                    ControlFlow::Break(true) => {
                        for (ep, band) in &autoroute.last_bands {
                            let (source, target) = ep.end_points.into();
                            autorouter.board.try_set_band_between_nodes(
                                &mut autoroute.last_recorder.board_data_edit,
                                source,
                                target,
                                *band,
                            );
                        }

                        ControlFlow::Break((
                            Some(autoroute.last_recorder.clone()),
                            "finished topo-autorouting".to_string(),
                        ))
                    }
                };
                // TODO: maintain topo-navmesh just like layout
                *autorouter.board.layout_mut() = autoroute.last_layout.clone();
                ret
            }
            ExecutionStepper::PlaceVia(place_via) => {
                let edit = place_via.doit(autorouter)?;
                ControlFlow::Break((edit, "finished placing via".to_string()))
            }
            ExecutionStepper::ShiftComponents(shift_components) => {
                let edit = shift_components.doit(autorouter)?;
                ControlFlow::Break((edit, "finished shifting components".to_string()))
            }
            ExecutionStepper::RemoveBands(remove_bands) => {
                let edit = remove_bands.doit(autorouter)?;
                ControlFlow::Break((edit, "finished removing bands".to_string()))
            }
            ExecutionStepper::MeasureLength(measure_length) => {
                let length = measure_length.doit(autorouter)?;
                ControlFlow::Break((None, format!("Total length of selected bands: {}", length)))
            }
        })
    }
}

impl<M: AccessMesadata + Clone> Step<Invoker<M>, String, StepKind> for ExecutionStepper<M> {
    type Error = InvokerError;

    fn step(
        &mut self,
        invoker: &mut Invoker<M>,
    ) -> Result<ControlFlow<String, StepKind>, InvokerError> {
        match self.step_catch_err(&mut invoker.autorouter) {
            Ok(ControlFlow::Continue(kind)) => Ok(ControlFlow::Continue(kind)),
            Ok(ControlFlow::Break((maybe_edit, msg))) => {
                if let (Some(command), Some(edit)) = (invoker.ongoing_command.take(), maybe_edit) {
                    invoker.history.do_(command, Some(edit));
                }

                Ok(ControlFlow::Break(msg))
            }
            Err(err) => {
                invoker.ongoing_command = None;
                Err(err)
            }
        }
    }
}

impl<M: AccessMesadata + Clone> ExecutionStepper<M> {
    /// Finishes the executed command early, keeping already routed bands and
    /// undoing only in-flight partial work. The stepper must be stepped once
    /// more so that it records the accumulated edit.
    ///
    /// Unlike [`Abort::abort`], which rolls the command back entirely, this
    /// retains completed work. Commands that cannot finish early cleanly are
    /// aborted instead.
    pub fn finish_early(&mut self, autorouter: &mut Autorouter<M>) {
        match self {
            ExecutionStepper::MultilayerAutoroute(autoroute) => autoroute.finish_early(autorouter),
            ExecutionStepper::PlanarAutoroute(autoroute) => autoroute.finish_early(autorouter),
            ExecutionStepper::CriticalAutoroute(autoroute) => autoroute.finish_early(autorouter),
            ExecutionStepper::TopoAutoroute(autoroute) => {
                autoroute.abort(&mut ());
            }
            ExecutionStepper::PlaceVia(..)
            | ExecutionStepper::ShiftComponents(..)
            | ExecutionStepper::RemoveBands(..)
            | ExecutionStepper::MeasureLength(..) => (),
        }
    }
}

impl<M: AccessMesadata + Clone> Abort<Invoker<M>> for ExecutionStepper<M> {
    fn abort(&mut self, invoker: &mut Invoker<M>) {
        match self {
            ExecutionStepper::MultilayerAutoroute(autoroute) => {
                autoroute.abort(&mut invoker.autorouter)
            }
            ExecutionStepper::PlanarAutoroute(autoroute) => {
                autoroute.abort(&mut invoker.autorouter)
            }
            ExecutionStepper::CriticalAutoroute(autoroute) => {
                autoroute.abort(&mut invoker.autorouter)
            }
            ExecutionStepper::TopoAutoroute(autoroute) => {
                autoroute.abort(&mut ());
                // TODO: maintain topo-navmesh just like layout
                *invoker.autorouter.board.layout_mut() = autoroute.last_layout.clone();
            }
            ExecutionStepper::PlaceVia(_place_via) => (), //place_via.abort(),
            ExecutionStepper::ShiftComponents(_shift_components) => (),
            ExecutionStepper::RemoveBands(_remove_bands) => (), //remove_bands.abort(),
            ExecutionStepper::MeasureLength(_measure_length) => (), //measure_length.abort(),
        }
    }
}

// Since enum_dispatch does not really support generics, we implement this the
// long way by using `match`.
impl<M> EstimateProgress for ExecutionStepper<M> {
    type Value = usize;
    type Subscale = LinearScale<f64>;

    fn estimate_progress(&self) -> LinearScale<usize, LinearScale<f64>> {
        match self {
            ExecutionStepper::MultilayerAutoroute(autoroute) => autoroute.estimate_progress(),
            ExecutionStepper::PlanarAutoroute(autoroute) => autoroute.estimate_progress(),
            ExecutionStepper::CriticalAutoroute(autoroute) => autoroute.estimate_progress(),
            ExecutionStepper::TopoAutoroute(..) => {
                LinearScale::new(0, 0, LinearScale::new(0.0, 0.0, ()))
            }
            ExecutionStepper::PlaceVia(..) => {
                LinearScale::new(0, 0, LinearScale::new(0.0, 0.0, ()))
            }
            ExecutionStepper::ShiftComponents(..) => {
                LinearScale::new(0, 0, LinearScale::new(0.0, 0.0, ()))
            }
            ExecutionStepper::RemoveBands(..) => {
                LinearScale::new(0, 0, LinearScale::new(0.0, 0.0, ()))
            }
            ExecutionStepper::MeasureLength(..) => {
                LinearScale::new(0, 0, LinearScale::new(0.0, 0.0, ()))
            }
        }
    }
}

// Since enum_dispatch does not really support generics, we implement this the
// long way by using `match`.
impl<M> GetTimeoutProgress for ExecutionStepper<M> {
    type Subscale = LinearScale<f64>;

    fn timeout_progress(&self) -> Option<LinearScale<f64, LinearScale<f64>>> {
        match self {
            ExecutionStepper::MultilayerAutoroute(autoroute) => autoroute.timeout_progress(),
            ExecutionStepper::CriticalAutoroute(autoroute) => autoroute.timeout_progress(),
            ExecutionStepper::PlanarAutoroute(_autoroute) => None,
            ExecutionStepper::TopoAutoroute(..) => None,
            ExecutionStepper::PlaceVia(..) => None,
            ExecutionStepper::ShiftComponents(..) => None,
            ExecutionStepper::RemoveBands(..) => None,
            ExecutionStepper::MeasureLength(..) => None,
        }
    }
}
