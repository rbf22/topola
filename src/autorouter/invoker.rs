// SPDX-FileCopyrightText: 2024 Topola contributors
//
// SPDX-License-Identifier: MIT

//! Manages the execution of routing commands within the autorouting system.

use std::ops::ControlFlow;
use std::time::{Duration, Instant};

use derive_getters::{Dissolve, Getters};
use enum_dispatch::enum_dispatch;
use geo::geometry::LineString;
use thiserror::Error;

use crate::{
    board::AccessMesadata,
    drawing::graph::PrimitiveIndex,
    geometry::{edit::Edit, primitive::PrimitiveShape},
    graph::GenericIndex,
    layout::poly::PolyWeight,
    router::{
        navcord::Navcord,
        navmesh::{Navmesh, NavnodeIndex},
        ng,
        thetastar::ThetastarStepper,
    },
    stepper::Step,
};

use super::{
    critical_autoroute::CriticalAutorouteExecutionStepper,
    execution::{Command, ExecutionStepper, StepKind},
    history::{History, HistoryError},
    measure_length::MeasureLengthExecutionStepper,
    multilayer_reconfigurator::MultilayerAutorouteReconfigurator,
    place_via::PlaceViaExecutionStepper,
    planar_reconfigurator::PlanarAutorouteReconfigurator,
    remove_bands::RemoveBandsExecutionStepper,
    shift_components::ShiftComponentsExecutionStepper,
    Autorouter, AutorouterError,
};

/// Trait for getting the information to display on the debug overlay,
#[enum_dispatch]
pub trait GetDebugOverlayData {
    /// Get the Theta* stepper. Most importantly, this gives us the access to
    /// the navmesh.
    fn maybe_thetastar(&self) -> Option<&ThetastarStepper<Navmesh, f64>> {
        None
    }

    /// Obtain Topological/Planar Navigation Mesh, if present.
    fn maybe_topo_navmesh(&self) -> Option<ng::pie::navmesh::NavmeshRef<'_, ng::PieNavmeshBase>> {
        None
    }

    /// Get the navcord. This is useful for coloring the currently visited path
    /// on the navmesh.
    fn maybe_navcord(&self) -> Option<&Navcord> {
        None
    }

    fn active_polygons(&self) -> &[GenericIndex<PolyWeight>] {
        &[]
    }

    /// Get ghosts. Ghosts are the shapes that Topola attempted to create but
    /// failed due to them infringing on other shapes.
    fn ghosts(&self) -> &[PrimitiveShape] {
        &[]
    }

    /// Get the obstacles that prevented Topola from creating new objects (the
    /// shapes of these objects can be obtained from the above `.ghosts(...)`)
    /// method. This allows to highlight what prevented Topola's algorithm from
    /// going some way.
    fn obstacles(&self) -> &[PrimitiveIndex] {
        &[]
    }

    fn polygonal_blockers(&self) -> &[LineString] {
        &[]
    }

    /// Get a text string to show on the debug overlay near to a navnode.
    /// Usually returns None, this method exists only for quick and dirty
    /// debugging.
    fn navnode_debug_text(&self, _navnode: NavnodeIndex) -> Option<&str> {
        None
    }

    /// Get a text string to show on the debug overlay near to a navedge.
    /// Usually returns None, this method exists only for quick and dirty
    /// debugging.
    fn navedge_debug_text(&self, _navedge: (NavnodeIndex, NavnodeIndex)) -> Option<&str> {
        None
    }
}

/// `ExecutionStepper` and `ActivityStepper` box their large variants,
/// so the trait must delegate through `Box` for `enum_dispatch` to
/// see them.
impl<T: GetDebugOverlayData + ?Sized> GetDebugOverlayData for Box<T> {
    fn maybe_thetastar(&self) -> Option<&ThetastarStepper<Navmesh, f64>> {
        (**self).maybe_thetastar()
    }

    fn maybe_topo_navmesh(&self) -> Option<ng::pie::navmesh::NavmeshRef<'_, ng::PieNavmeshBase>> {
        (**self).maybe_topo_navmesh()
    }

    fn maybe_navcord(&self) -> Option<&Navcord> {
        (**self).maybe_navcord()
    }

    fn active_polygons(&self) -> &[GenericIndex<PolyWeight>] {
        (**self).active_polygons()
    }

    fn ghosts(&self) -> &[PrimitiveShape] {
        (**self).ghosts()
    }

    fn obstacles(&self) -> &[PrimitiveIndex] {
        (**self).obstacles()
    }

    fn polygonal_blockers(&self) -> &[LineString] {
        (**self).polygonal_blockers()
    }

    fn navnode_debug_text(&self, navnode: NavnodeIndex) -> Option<&str> {
        (**self).navnode_debug_text(navnode)
    }

    fn navedge_debug_text(&self, navedge: (NavnodeIndex, NavnodeIndex)) -> Option<&str> {
        (**self).navedge_debug_text(navedge)
    }
}

/// Error types that can occur during the invocation of commands.
#[derive(Error, Debug, Clone)]
pub enum InvokerError {
    /// Wraps errors related to command history operations.
    #[error(transparent)]
    History(#[from] HistoryError),

    /// Wraps errors related to autorouter operations.
    #[error(transparent)]
    Autorouter(#[from] AutorouterError),
}

/// Per-step report passed to progress callbacks of
/// [`Invoker::execute_with_progress`] and [`Invoker::execute_with_timeout`].
pub struct StepReport<'a, M> {
    /// The invoker executing the command.
    pub invoker: &'a Invoker<M>,
    /// The ongoing stepper, from which a progress estimate may be
    /// obtained via [`EstimateProgress::estimate_progress`]
    /// ([`crate::stepper::EstimateProgress`]).
    pub stepper: &'a ExecutionStepper<M>,
    /// Number of steps taken so far, this step included (1-based).
    pub index: usize,
    /// Wall-clock duration of this step.
    pub duration: Duration,
    /// What this step did.
    pub kind: StepKind,
}

#[derive(Getters, Dissolve)]
pub struct Invoker<M> {
    /// Instance for executing desired autorouting commands.
    pub(super) autorouter: Autorouter<M>,
    /// History of executed commands.
    pub(super) history: History,
    /// Currently ongoing command type.
    pub(super) ongoing_command: Option<Command>,
}

impl<M: AccessMesadata + Clone> Invoker<M> {
    /// Creates a new instance of Invoker with the given autorouter instance
    pub fn new(autorouter: Autorouter<M>) -> Self {
        Self::new_with_history(autorouter, History::new())
    }

    /// Creates a new instance of Invoker with the given autorouter and history
    pub fn new_with_history(autorouter: Autorouter<M>, history: History) -> Self {
        Self {
            autorouter,
            history,
            ongoing_command: None,
        }
    }

    /// Returns a mutable reference to the autorouter, for commands run
    /// outside the stepper machinery such as channel probes.
    pub fn autorouter_mut(&mut self) -> &mut Autorouter<M> {
        &mut self.autorouter
    }

    //#[debug_requires(self.ongoing_command.is_none())]
    /// Executes a command, managing the command status and history.
    ///
    /// This function is used to pass the [`Command`] to [`Invoker::execute_stepper`]
    /// function, and control its execution status.
    pub fn execute(&mut self, command: Command) -> Result<(), InvokerError> {
        self.execute_with_progress(command, |_| {})
    }

    /// Executes a command, invoking `on_progress` after each step.
    ///
    /// The callback receives a [`StepReport`] describing the step: its
    /// duration, what it did ([`StepKind`]), and the ongoing
    /// [`ExecutionStepper`], from which a progress estimate may be
    /// obtained via [`EstimateProgress::estimate_progress`]
    /// ([`crate::stepper::EstimateProgress`]).
    pub fn execute_with_progress(
        &mut self,
        command: Command,
        mut on_progress: impl FnMut(&StepReport<M>),
    ) -> Result<(), InvokerError> {
        let mut execute = self.execute_stepper(command)?;
        let mut index = 0usize;

        loop {
            let started = Instant::now();
            let status = execute.step(self)?;
            let duration = started.elapsed();
            index += 1;

            let kind = match status {
                ControlFlow::Break(..) => {
                    self.history.set_undone(std::iter::empty());
                    return Ok(());
                }
                ControlFlow::Continue(kind) => kind,
            };

            crate::metrics::record_step(kind, duration);
            on_progress(&StepReport {
                invoker: self,
                stepper: &execute,
                index,
                duration,
                kind,
            });
        }
    }

    /// Executes a command like [`Invoker::execute_with_progress`], but
    /// finishes it early once `timeout` has elapsed, `max_steps` steps
    /// have been taken, or `should_stop` reports true. Early finishing
    /// keeps already routed bands and undoes only in-flight partial
    /// work; a panicking step is salvaged the same way.
    ///
    /// Returns `true` when the command was finished early due to the
    /// timeout, step budget, stop condition, or a salvaged panic.
    pub fn execute_with_timeout(
        &mut self,
        command: Command,
        timeout: Option<Duration>,
        max_steps: Option<usize>,
        mut should_stop: impl FnMut() -> bool,
        mut on_progress: impl FnMut(&StepReport<M>),
    ) -> Result<bool, InvokerError> {
        let deadline = timeout.map(|timeout| Instant::now() + timeout);
        let mut execute = self.execute_stepper(command)?;
        let mut timed_out = false;
        let mut index = 0usize;

        loop {
            // Some geometry bugs manifest as panics deep in the
            // router; salvage already routed bands the same way an
            // early finish does instead of losing all the work.
            let started = Instant::now();
            let status =
                match std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| execute.step(self)))
                {
                    Ok(status) => status?,
                    Err(payload) => {
                        if std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                            execute.finish_early(&mut self.autorouter)
                        }))
                        .is_err()
                        {
                            std::panic::resume_unwind(payload);
                        }
                        self.history.set_undone(std::iter::empty());
                        return Ok(true);
                    }
                };
            let duration = started.elapsed();
            index += 1;

            let kind = match status {
                ControlFlow::Break(..) => {
                    self.history.set_undone(std::iter::empty());
                    return Ok(timed_out);
                }
                ControlFlow::Continue(kind) => kind,
            };

            crate::metrics::record_step(kind, duration);

            if !timed_out
                && (deadline.is_some_and(|deadline| Instant::now() >= deadline)
                    || max_steps.is_some_and(|max_steps| index >= max_steps)
                    || should_stop())
            {
                execute.finish_early(&mut self.autorouter);
                timed_out = true;
            }

            on_progress(&StepReport {
                invoker: self,
                stepper: &execute,
                index,
                duration,
                kind,
            });
        }
    }

    /// Pass given command to be executed.
    ///
    /// Function used to set given [`Command`] to ongoing state, dispatch and execute it.
    pub fn execute_stepper(
        &mut self,
        command: Command,
    ) -> Result<ExecutionStepper<M>, InvokerError> {
        debug_assert!(self.ongoing_command.is_none());
        let execute = self.dispatch_command(&command)?;
        self.ongoing_command = Some(command);
        Ok(execute)
    }

    fn dispatch_command(&mut self, command: &Command) -> Result<ExecutionStepper<M>, InvokerError> {
        debug_assert!(self.ongoing_command.is_none());
        Ok(match command {
            Command::Autoroute(selection, options) => ExecutionStepper::PlanarAutoroute(Box::new(
                self.autorouter.planar_autoroute(selection, *options)?,
            )),
            Command::MultilayerAutoroute(selection, options) => {
                ExecutionStepper::MultilayerAutoroute(Box::new(
                    self.autorouter.multilayer_autoroute(selection, *options)?,
                ))
            }
            Command::CriticalAutoroute(selection, options) => ExecutionStepper::CriticalAutoroute(
                Box::new(self.autorouter.critical_autoroute(selection, *options)?),
            ),
            Command::TopoAutoroute {
                selection,
                allowed_edges,
                active_layer: active_layer_name,
                routed_band_width,
            } => {
                let active_layer = self
                    .autorouter
                    .board
                    .layout()
                    .rules()
                    .layername_layer(active_layer_name)
                    .unwrap();
                let ratlines = self.autorouter.selected_ratlines(selection, active_layer);

                // TODO: consider "presort by pairwise detours"

                ExecutionStepper::TopoAutoroute(Box::new(self.autorouter.topo_autoroute_ratlines(
                    ratlines.into_iter().collect(),
                    allowed_edges.clone(),
                    active_layer,
                    *routed_band_width,
                    None,
                )?))
            }
            Command::PlaceVia(weight) => {
                ExecutionStepper::PlaceVia(self.autorouter.place_via(*weight)?)
            }
            Command::ShiftComponents(shifts) => ExecutionStepper::ShiftComponents(
                ShiftComponentsExecutionStepper::new(shifts.clone())?,
            ),
            Command::RemoveBands(selection) => {
                ExecutionStepper::RemoveBands(self.autorouter.remove_bands(selection)?)
            }
            Command::MeasureLength(selection) => {
                ExecutionStepper::MeasureLength(self.autorouter.measure_length(selection)?)
            }
        })
    }

    /// Undo last command.
    pub fn undo(&mut self) -> Result<(), InvokerError> {
        debug_assert!(self.ongoing_command.is_none());
        let last_done = self.history.last_done()?;

        if let Some(edit) = last_done.edit() {
            self.autorouter.board.apply_edit(&edit.reverse());
        }

        Ok(self.history.undo()?)
    }

    //#[debug_requires(self.ongoing_command.is_none())]
    /// Redo last command.
    pub fn redo(&mut self) -> Result<(), InvokerError> {
        let last_undone = self.history.last_undone()?;

        if let Some(edit) = last_undone.edit() {
            self.autorouter.board.apply_edit(edit);
        }

        Ok(self.history.redo()?)
    }

    /// Replay last command.
    pub fn replay(&mut self, history: History) {
        debug_assert!(self.ongoing_command.is_none());
        let (done, undone) = history.dissolve();

        for entry in done {
            self.execute(entry.command().clone());
        }

        self.history.set_undone(undone);
    }
}
