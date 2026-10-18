// SPDX-FileCopyrightText: 2024 Topola contributors
//
// SPDX-License-Identifier: MIT

//! Manages autorouting of ratlines in a layout, tracking status and processed
//! routing steps.

use std::{collections::BTreeMap, ops::ControlFlow};

use derive_getters::Getters;

use crate::{
    board::{
        edit::{BoardDataEdit, BoardEdit},
        AccessMesadata,
    },
    drawing::{
        band::BandTermsegIndex,
        dot::FixedDotIndex,
        graph::PrimitiveIndex,
    },
    geometry::{edit::Edit, primitive::PrimitiveShape},
    graph::MakeRef,
    layout::LayoutEdit,
    router::{
        navcord::Navcord,
        navmesh::Navmesh,
        prenavmesh::PrenavmeshBases,
        thetastar::ThetastarStepper,
        RouteStepper, Router,
    },
    stepper::{Abort, EstimateProgress, LinearScale, Reconfigure, Step},
};

use super::{
    invoker::GetDebugOverlayData, pad_entry, ratline::RatlineUid, Autorouter, AutorouterError,
    PlanarAutorouteOptions,
};

#[derive(Clone, Debug)]
pub struct PlanarAutorouteConfiguration {
    pub ratlines: Vec<RatlineUid>,
    pub terminating_dot_map: BTreeMap<(RatlineUid, FixedDotIndex), FixedDotIndex>,
    /// Extra path cost for wrapping each primitive, accumulated from
    /// recorded blockers (negotiated congestion). Inflating these
    /// obstacles' costs makes retries prefer alternate corridors.
    pub obstacle_penalties: BTreeMap<PrimitiveIndex, f64>,
    /// Ratlines whose bands are to be ripped up surgically when this
    /// configuration is applied, then requeued right after the failed
    /// ratline. Empty on orderings that only permute ratlines, which
    /// take the usual backtrace path instead.
    pub rip_up: Vec<RatlineUid>,
}

impl PlanarAutorouteConfiguration {
    pub fn ratline_terminating_dots(
        &self,
        autorouter: &Autorouter<impl AccessMesadata>,
        ratline_index: usize,
    ) -> (FixedDotIndex, FixedDotIndex) {
        let ratline = self.ratlines[ratline_index];
        let endpoint_dots = ratline.ref_(autorouter).endpoint_dots();
        (
            *self
                .terminating_dot_map
                .get(&(ratline, endpoint_dots.0))
                .unwrap_or(&endpoint_dots.0),
            *self
                .terminating_dot_map
                .get(&(ratline, endpoint_dots.1))
                .unwrap_or(&endpoint_dots.1),
        )
    }
}

#[derive(Clone, Debug)]
pub struct PlanarAutorouteCosts {
    pub lengths: Vec<f64>,
}

#[derive(Clone, Debug)]
pub struct PlanarAutorouteConfigurationStatus {
    pub configuration: PlanarAutorouteConfiguration,
    pub costs: PlanarAutorouteCosts,
}

/// The current status of the autoroute operation.
#[derive(Clone, Copy, Debug)]
pub enum PlanarAutorouteContinueStatus {
    /// The autoroute is currently running and in progress.
    Running,
    /// A band has been successfully routed.
    Routed(BandTermsegIndex),
    /// A band had been already routed and has been skipped.
    Skipped(BandTermsegIndex),
}

/// Manages the autorouting process across multiple ratlines.
#[derive(Getters)]
pub struct PlanarAutorouteExecutionStepper {
    /// The stepper configuration, including the ratlines which we are routing.
    configuration: PlanarAutorouteConfiguration,
    /// Keeps track of the current ratline being routed, if one is active.
    curr_ratline_index: usize,
    /// Stores the current route being processed, if any.
    route: Option<RouteStepper>,
    /// Records the changes to the layout, one routed band per item.
    layout_edits: Vec<LayoutEdit>,
    /// Records the changes to the board data, one routed band per item.
    board_data_edits: Vec<BoardDataEdit>,
    /// The options for the autorouting process, defining how routing should be carried out.
    options: PlanarAutorouteOptions,
    /// Edits applied outside any ratline's route — board data updates
    /// for a just-routed band and surgically ripped-up blocker bands.
    /// Merged into the next pushed edit slot so `layout_edits` and
    /// `board_data_edits` stay index-aligned for backtracing.
    pending_edit: BoardEdit,
    /// Per-`(net, layer)` prenavmesh bases. The vertex triangulation
    /// depends only on the ratline's net and the board's fixed
    /// primitives, which do not change while routing, so it is built
    /// once per net and cloned per ratline. The constrained base adds
    /// constraint edges; it is invalidated by rip-ups and extended
    /// incrementally as more bands commit.
    prenavmesh_bases: PrenavmeshBases,
}

impl PlanarAutorouteExecutionStepper {
    pub fn new(
        autorouter: &mut Autorouter<impl AccessMesadata>,
        configuration: PlanarAutorouteConfiguration,
        options: PlanarAutorouteOptions,
    ) -> Result<Self, AutorouterError> {
        if configuration.ratlines.is_empty() {
            return Err(AutorouterError::NothingToRoute);
        };

        let mut this = Self {
            configuration,
            curr_ratline_index: 0,
            route: None,
            layout_edits: vec![],
            board_data_edits: vec![],
            options,
            pending_edit: BoardEdit::new(),
            prenavmesh_bases: PrenavmeshBases::default(),
        };
        this.start_routing_ratline(autorouter)?;
        Ok(this)
    }

    fn backtrace_to_index(
        &mut self,
        autorouter: &mut Autorouter<impl AccessMesadata>,
        index: usize,
    ) -> Result<(), AutorouterError> {
        if index >= self.board_data_edits.len() {
            return Err(AutorouterError::NothingToUndoForReconfiguration);
        }

        self.dissolve_route_stepper_and_push_edits();

        let board_edit = BoardEdit::new_from_edits(
            BoardDataEdit::merge_iter(self.board_data_edits.split_off(index)),
            LayoutEdit::merge_iter(self.layout_edits.split_off(index)),
        );

        autorouter.board.apply_edit(&board_edit.reverse());

        self.curr_ratline_index = index;
        Ok(self.start_routing_ratline(autorouter)?)
    }

    /// Builds the ratline's navmesh, reusing a cached per-net
    /// prenavmesh vertex triangulation when one exists. The triangulation
    /// is the dominant cost of navmesh construction, and its vertex set
    /// only depends on the ratline's net and layer, not on routed bands.
    fn make_navmesh(
        &mut self,
        layout: &crate::layout::Layout<impl AccessMesadata>,
        origin: FixedDotIndex,
        destination: FixedDotIndex,
    ) -> Result<Navmesh, AutorouterError> {
        Ok(self.prenavmesh_bases.navmesh(
            layout,
            origin,
            destination,
            self.options.router,
        )?)
    }

    fn start_routing_ratline(
        &mut self,
        autorouter: &mut Autorouter<impl AccessMesadata>,
    ) -> Result<(), AutorouterError> {
        let (origin, destination) = self
            .configuration
            .ratline_terminating_dots(autorouter, self.curr_ratline_index);
        let navmesh = self.make_navmesh(autorouter.board.layout(), origin, destination)?;

        let mut router = Router::new(autorouter.board.layout_mut(), self.options.router);
        let mut route = RouteStepper::new_from_navmesh(
            &mut router,
            LayoutEdit::new(),
            navmesh,
            self.options.router.routed_band_width,
        );
        route.set_obstacle_penalties(self.configuration.obstacle_penalties.clone());
        self.route = Some(route);

        Ok(())
    }

    /// Dissolves the in-flight route and pushes one aligned entry onto
    /// both edit vecs: any pending work (band registry updates,
    /// surgical rip-ups) applied since the last boundary, plus the
    /// route's own edit. Pushed unconditionally so the vecs stay
    /// index-aligned for `backtrace_to_index`, even for skipped
    /// ratlines that produce no board data edit.
    fn dissolve_route_stepper_and_push_edits(&mut self) {
        if let Some(taken_route) = self.route.take() {
            let (_thetastar, navcord, ..) = taken_route.dissolve();
            self.pending_edit.layout_edit.merge(navcord.recorder);
        }

        let pending = std::mem::replace(&mut self.pending_edit, BoardEdit::new());
        self.layout_edits.push(pending.layout_edit);
        self.board_data_edits.push(pending.board_data_edit);
    }

    /// Finishes routing early: the in-flight ratline's partial band is
    /// undone, while already routed bands are kept. The stepper must be
    /// stepped once more so that it produces the accumulated edit.
    pub fn finish_early(&mut self, autorouter: &mut Autorouter<impl AccessMesadata>) {
        if self.curr_ratline_index >= self.configuration.ratlines.len() {
            return;
        }

        if let Some(taken_route) = self.route.take() {
            // Undo the in-flight ratline's partially drawn band, along
            // with any uncommitted pending work such as surgical
            // rip-ups.
            let (_thetastar, navcord, ..) = taken_route.dissolve();
            self.pending_edit.layout_edit.merge(navcord.recorder);
        }

        let pending = std::mem::replace(&mut self.pending_edit, BoardEdit::new());
        autorouter.board.apply_edit(&pending.reverse());

        self.curr_ratline_index = self.configuration.ratlines.len();
    }

    /// Surgical rip-up, phase one: reverts the in-flight route's
    /// partial work, removes the bands of `rip_up` ratlines — resolved
    /// under the *current* configuration's terminating-dot map, which
    /// is where the bands were registered — and folds their edit slots
    /// into `pending_edit` so each creation and removal record
    /// coalesces instead of resurrecting the band on undo.
    ///
    /// Must be followed by [`Self::install_configuration`] (which may
    /// change the terminating-dot map) before stepping resumes.
    /// Between the two calls the board holds no bands for the ripped
    /// ratlines and no route is in flight, which is where multilayer
    /// re-layering swaps the affected fanout vias.
    pub fn surgical_rip_up(
        &mut self,
        autorouter: &mut Autorouter<impl AccessMesadata>,
        rip_up: &[RatlineUid],
    ) -> Result<(), AutorouterError> {
        log::debug!(
            "surgical rip-up: removing {} blocker bands at ratline {}",
            rip_up.len(),
            self.curr_ratline_index,
        );
        crate::metrics::incr(&crate::metrics::METRICS.surgical_ripups);

        if let Some(taken_route) = self.route.take() {
            let (_thetastar, navcord, ..) = taken_route.dissolve();
            let edit = BoardEdit::new_from_edits(BoardDataEdit::new(), navcord.recorder);
            autorouter.board.apply_edit(&edit.reverse());
        }

        // Drop the ripped-up ratlines' edit slots, preserving the
        // per-position alignment between the edit vecs and the
        // configuration's ratlines. The dropped entries merge into
        // `pending_edit` before the surgical removals so each ripped
        // band's creation and removal records coalesce into a no-op
        // instead of resurrecting the band on undo.
        let mut indices: Vec<_> = rip_up
            .iter()
            .filter_map(|ratline| {
                self.configuration
                    .ratlines
                    .iter()
                    .position(|uid| uid == ratline)
            })
            .filter(|index| *index < self.layout_edits.len())
            .collect();
        indices.sort_unstable();
        indices.dedup();

        let mut ripped_edits = BoardEdit::new();
        for index in indices.iter().rev() {
            ripped_edits
                .layout_edit
                .merge(self.layout_edits.remove(*index));
            ripped_edits
                .board_data_edit
                .merge(self.board_data_edits.remove(*index));
        }

        // Bands are looked up under the current configuration:
        // reconfigurations that change `terminating_dot_map`
        // (multilayer re-layering) must still find each band where it
        // was registered, under the old terminating dots.
        let mut removal_edit = BoardEdit::new();
        for ratline in rip_up {
            let index = self
                .configuration
                .ratlines
                .iter()
                .position(|uid| uid == ratline)
                .unwrap();
            let (origin, destination) =
                self.configuration.ratline_terminating_dots(autorouter, index);

            let Some(band) = autorouter.board.band_between_nodes(origin, destination)
            else {
                continue;
            };

            autorouter
                .board
                .remove_band_between_nodes(&mut removal_edit, origin, destination)
                .map_err(|_| AutorouterError::CouldNotRemoveBand(band[false]))?;
        }

        // Chronological merge: the ripped bands' original edits
        // precede their surgical removals, so each pair coalesces to a
        // no-op when the pending edit later merges into a slot.
        self.pending_edit.merge(ripped_edits);
        self.pending_edit.merge(removal_edit);
        Ok(())
    }

    /// Surgical rip-up, phase two: installs `new_configuration` and
    /// resumes routing at the ratline that was being attempted, which
    /// keeps its position relative to the requeued `rip_up` ratlines
    /// by the caller's ordering. When routing had already run off the
    /// end of the ratline list, resumes at the first requeued ratline
    /// instead.
    pub fn install_configuration(
        &mut self,
        autorouter: &mut Autorouter<impl AccessMesadata>,
        new_configuration: PlanarAutorouteConfiguration,
    ) -> Result<PlanarAutorouteConfigurationStatus, AutorouterError> {
        let attempted = self
            .configuration
            .ratlines
            .get(self.curr_ratline_index)
            .copied();
        let first_requeued = new_configuration.rip_up.first().copied();
        let rip_up_count = new_configuration.rip_up.len();

        let result = PlanarAutorouteConfigurationStatus {
            configuration: std::mem::replace(&mut self.configuration, new_configuration),
            costs: PlanarAutorouteCosts {
                lengths: (0..self.curr_ratline_index).map(|_| 0.0).collect(),
            },
        };

        self.curr_ratline_index = if let Some(attempted) = attempted {
            self.configuration
                .ratlines
                .iter()
                .position(|uid| *uid == attempted)
                .unwrap()
        } else {
            // The queue was exhausted: the requeued ratlines sit at the
            // end of the new ordering.
            first_requeued
                .and_then(|ratline| {
                    self.configuration
                        .ratlines
                        .iter()
                        .position(|uid| *uid == ratline)
                })
                .unwrap_or_else(|| self.configuration.ratlines.len().saturating_sub(rip_up_count))
        };

        if self.curr_ratline_index < self.configuration.ratlines.len() {
            self.start_routing_ratline(autorouter)?;
        }
        Ok(result)
    }

    /// Extends the live configuration's terminating-dot remappings.
    /// Multilayer re-layering calls this between
    /// [`Self::surgical_rip_up`] and [`Self::install_configuration`]
    /// after placing the new fanout vias; planar reconfigurations
    /// leave the map untouched.
    pub fn remap_terminating_dots(
        &mut self,
        entries: &BTreeMap<(RatlineUid, FixedDotIndex), FixedDotIndex>,
    ) {
        self.configuration.terminating_dot_map.extend(entries.iter());
    }
}

impl<M: AccessMesadata> Step<Autorouter<M>, Option<BoardEdit>, PlanarAutorouteContinueStatus>
    for PlanarAutorouteExecutionStepper
{
    type Error = AutorouterError;

    fn step(
        &mut self,
        autorouter: &mut Autorouter<M>,
    ) -> Result<ControlFlow<Option<BoardEdit>, PlanarAutorouteContinueStatus>, AutorouterError>
    {
        // TODO: Use a proper state machine here for better readability?

        if self.curr_ratline_index >= self.configuration().ratlines.len() {
            self.dissolve_route_stepper_and_push_edits();

            if self.options.improve_pad_entry {
                let mut layout_edit = LayoutEdit::new();
                let mut board_data_edit = BoardDataEdit::new();
                let stats = pad_entry::improve_pad_entries(
                    autorouter,
                    &mut layout_edit,
                    &mut board_data_edit,
                    self.options.principal_layer,
                    self.options.pad_entry_min_angle,
                );
                log::debug!("pad-entry post-processing: {stats:?}");
                self.layout_edits.push(layout_edit);
                self.board_data_edits.push(board_data_edit);
            }

            return Ok(ControlFlow::Break(Some(BoardEdit::new_from_edits(
                BoardDataEdit::merge_iter(self.board_data_edits.iter().cloned()),
                // FIXME: This is a large clone. We probably need some
                // high-level, say `AutorouteEdit`, struct to store sequences
                // of edits.
                LayoutEdit::merge_iter(self.layout_edits.iter().cloned()),
            ))));
        }

        let (origin, destination) = self
            .configuration
            .ratline_terminating_dots(autorouter, self.curr_ratline_index);

        let Some(ref mut route) = self.route else {
            // May happen if stepper was aborted.
            return Ok(ControlFlow::Break(None));
        };

        let ret = if let Some(band_termseg) =
            autorouter.board.band_between_nodes(origin, destination)
        {
            PlanarAutorouteContinueStatus::Skipped(band_termseg[false])
        } else {
            let band_termseg = {
                let mut router = Router::new(autorouter.board.layout_mut(), self.options.router);

                let ControlFlow::Break(band_termseg) = route.step(&mut router)? else {
                    return Ok(ControlFlow::Continue(
                        PlanarAutorouteContinueStatus::Running,
                    ));
                };
                band_termseg
            };

            let band = autorouter
                .board
                .layout()
                .drawing()
                .find_loose_band_uid(band_termseg.into())
                .expect("a completely routed band should've Seg's as ends");

            autorouter
                .ratsnests
                .on_principal_layer_mut(self.options.principal_layer)
                .assign_band_termseg_to_ratline(
                    self.configuration().ratlines[self.curr_ratline_index].index,
                    band_termseg,
                );

            let mut board_data_edit = BoardDataEdit::new();

            autorouter.board.try_set_band_between_nodes(
                &mut board_data_edit,
                origin,
                destination,
                band,
            );

            self.pending_edit.board_data_edit.merge(board_data_edit);

            PlanarAutorouteContinueStatus::Routed(band_termseg)
        };

        self.curr_ratline_index += 1;

        if self
            .configuration
            .ratlines
            .get(self.curr_ratline_index)
            .is_some()
        {
            self.dissolve_route_stepper_and_push_edits();
            self.start_routing_ratline(autorouter);
        }

        Ok(ControlFlow::Continue(ret))
    }
}

impl<M: AccessMesadata> Abort<Autorouter<M>> for PlanarAutorouteExecutionStepper {
    fn abort(&mut self, autorouter: &mut Autorouter<M>) {
        self.backtrace_to_index(autorouter, 0);
        self.curr_ratline_index = self.configuration.ratlines.len();
    }
}

impl<M: AccessMesadata> Reconfigure<Autorouter<M>> for PlanarAutorouteExecutionStepper {
    type Configuration = PlanarAutorouteConfiguration;
    type Output = Result<PlanarAutorouteConfigurationStatus, AutorouterError>;

    fn reconfigure(
        &mut self,
        autorouter: &mut Autorouter<M>,
        new_configuration: PlanarAutorouteConfiguration,
    ) -> Result<PlanarAutorouteConfigurationStatus, AutorouterError> {
        if !new_configuration.rip_up.is_empty() {
            self.surgical_rip_up(autorouter, &new_configuration.rip_up)?;
            return self.install_configuration(autorouter, new_configuration);
        }

        let Some(new_index) = new_configuration
            .ratlines
            .iter()
            .zip(self.configuration.ratlines.iter())
            .position(|(permuted, original)| *permuted != *original)
        else {
            return Err(AutorouterError::NothingToUndoForReconfiguration);
        };

        let result = PlanarAutorouteConfigurationStatus {
            configuration: std::mem::replace(&mut self.configuration, new_configuration),
            costs: PlanarAutorouteCosts {
                lengths: (0..*self.curr_ratline_index()).map(|_| 0.0).collect(),
            },
        };

        self.backtrace_to_index(autorouter, new_index)?;
        Ok(result)
    }
}

impl EstimateProgress for PlanarAutorouteExecutionStepper {
    type Value = usize;
    type Subscale = LinearScale<f64>;

    fn estimate_progress(&self) -> LinearScale<usize, LinearScale<f64>> {
        LinearScale::new(
            self.curr_ratline_index,
            self.configuration().ratlines.len(),
            LinearScale::new(
                self.route
                    .as_ref()
                    .map_or(0.0, |route| *route.estimate_progress().value()),
                self.route
                    .as_ref()
                    .map_or(0.0, |route| *route.estimate_progress().maximum()),
                (),
            ),
        )
    }
}

impl GetDebugOverlayData for PlanarAutorouteExecutionStepper {
    fn maybe_thetastar(&self) -> Option<&ThetastarStepper<Navmesh, f64>> {
        self.route.as_ref().map(|route| route.thetastar())
    }

    fn maybe_navcord(&self) -> Option<&Navcord> {
        self.route.as_ref().map(|route| route.navcord())
    }

    fn ghosts(&self) -> &[PrimitiveShape] {
        self.route.as_ref().map_or(&[], |route| route.ghosts())
    }

    fn obstacles(&self) -> &[PrimitiveIndex] {
        self.route.as_ref().map_or(&[], |route| route.obstacles())
    }
}
