// SPDX-FileCopyrightText: 2024 Topola contributors
//
// SPDX-License-Identifier: MIT

//! Manages autorouting of ratlines in a layout, tracking status and processed
//! routing steps.

use std::{
    collections::{BTreeMap, BTreeSet},
    ops::ControlFlow,
};

use derive_getters::Getters;

use crate::{
    board::{
        edit::{BoardDataEdit, BoardEdit},
        AccessMesadata,
    },
    drawing::{
        band::BandTermsegIndex,
        dot::FixedDotIndex,
        graph::{MakePrimitiveRef, PrimitiveIndex},
    },
    geometry::{edit::Edit, primitive::PrimitiveShape, GetLayer},
    graph::MakeRef,
    layout::LayoutEdit,
    router::{
        navcord::{Navcord, NavcordSearchState},
        navmesh::{Navmesh, NavmeshError},
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
    /// Critical-first ratlines whose committed bands must survive
    /// reconfiguration: orderings keep them first, and they are
    /// filtered out of `rip_up` lists (the reconfigurator's
    /// desperation escape may still revoke protection when a
    /// protected band is the *only* way forward). Empty when
    /// critical-first routing is not in use.
    pub protected: BTreeSet<RatlineUid>,
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
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
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
    /// Debug: the ratline whose attempt produced each edit slot, used to
    /// detect drift between slot positions and `configuration.ratlines`.
    slot_ratlines: Vec<Option<RatlineUid>>,
    /// Debug: the ratline currently in flight or just dissolved.
    attempt_ratline: Option<RatlineUid>,
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
            slot_ratlines: vec![],
            attempt_ratline: None,
            options,
            pending_edit: BoardEdit::new(),
            prenavmesh_bases: PrenavmeshBases::default(),
        };
        this.start_routing_ratline(autorouter)?;
        Ok(this)
    }

    /// Mutable access to the live configuration — for protected-set
    /// bookkeeping, which lives in the configuration but is managed
    /// externally by the critical-first machinery.
    pub fn configuration_mut(&mut self) -> &mut PlanarAutorouteConfiguration {
        &mut self.configuration
    }

    fn backtrace_to_index(
        &mut self,
        autorouter: &mut Autorouter<impl AccessMesadata>,
        index: usize,
    ) -> Result<(), AutorouterError> {
        if index >= self.board_data_edits.len() {
            return Err(AutorouterError::NothingToUndoForReconfiguration);
        }

        let extracted = self.dissolve_route_stepper_and_push_edits();

        crate::metrics::add(
            &crate::metrics::METRICS.backtraced_ratlines,
            self.board_data_edits.len() - index,
        );

        let board_edit = BoardEdit::new_from_edits(
            BoardDataEdit::merge_iter(self.board_data_edits.split_off(index)),
            LayoutEdit::merge_iter(self.layout_edits.split_off(index)),
        );
        let _ = self.slot_ratlines.split_off(index);

        autorouter.board.apply_edit(&board_edit.reverse());

        // Store the search state only after the reversion so its
        // snapshot keys by the board a retry will actually see.
        if let Some((origin, destination, state)) = extracted {
            self.store_search_state(autorouter, origin, destination, state);
        }

        self.curr_ratline_index = index;
        Ok(self.start_routing_ratline(autorouter)?)
    }

    /// Builds the ratline's navmesh, reusing a cached per-net
    /// prenavmesh vertex triangulation when one exists. The triangulation
    /// is the dominant cost of navmesh construction, and its vertex set
    /// only depends on the ratline's net and layer, not on routed bands.
    /// On a whole-navmesh cache hit the retained search state of a
    /// previous attempt on the same ratline rides along.
    fn make_navmesh(
        &mut self,
        layout: &crate::layout::Layout<impl AccessMesadata>,
        origin: FixedDotIndex,
        destination: FixedDotIndex,
    ) -> Result<(Navmesh, Option<NavcordSearchState>), AutorouterError> {
        Ok(self.prenavmesh_bases.navmesh_and_search_state(
            layout,
            origin,
            destination,
            self.options.router,
        )?)
    }

    /// Retains a dissolved route attempt's search state (probe and
    /// step memos plus the path interner they key into) for the next
    /// attempt on the same ratline under provably identical board
    /// geometry. Callers hand it to the prenavmesh-bases store *after*
    /// any pending board reversion so the stored snapshot matches what
    /// a retry sees.
    fn store_search_state(
        &mut self,
        autorouter: &Autorouter<impl AccessMesadata>,
        origin: FixedDotIndex,
        destination: FixedDotIndex,
        state: NavcordSearchState,
    ) {
        self.prenavmesh_bases.store_search_state(
            autorouter.board.layout(),
            origin,
            destination,
            self.options.router,
            state,
        );
    }

    fn start_routing_ratline(
        &mut self,
        autorouter: &mut Autorouter<impl AccessMesadata>,
    ) -> Result<(), AutorouterError> {
        while self.curr_ratline_index < self.configuration.ratlines.len() {
            self.attempt_ratline = Some(self.configuration.ratlines[self.curr_ratline_index]);
            let (origin, destination) = self
                .configuration
                .ratline_terminating_dots(autorouter, self.curr_ratline_index);
            let (navmesh, search_state) =
                match self.make_navmesh(autorouter.board.layout(), origin, destination) {
                    Ok(ok) => ok,
                    Err(AutorouterError::Navmesh(
                        NavmeshError::UnreachableOrigin | NavmeshError::UnreachableDestination,
                    )) => {
                        // The endpoint is sealed off — e.g. a pad
                        // ringed by foreign geometry. Leave the ratline
                        // unrouted and move on: erroring here retries
                        // the same ratline at the head of every
                        // reconfiguration and aborts the command.
                        // An empty edit slot keeps `layout_edits`
                        // index-aligned with the ratline list.
                        log::debug!(
                            "ratline {:?} endpoint unreachable in navmesh — skipping",
                            self.configuration.ratlines[self.curr_ratline_index]
                        );
                        self.dissolve_route_stepper_and_push_edits();
                        self.curr_ratline_index += 1;
                        continue;
                    }
                    Err(err) => return Err(err),
                };

            log::debug!(
                "ratline {:?} attempt on layer {}: {} navnodes",
                self.configuration.ratlines[self.curr_ratline_index],
                origin
                    .primitive_ref(autorouter.board().layout().drawing())
                    .layer(),
                navmesh.graph().node_count()
            );

            let mut router = Router::new(autorouter.board.layout_mut(), self.options.router);
            let mut route = RouteStepper::new_from_navmesh(
                &mut router,
                LayoutEdit::new(),
                navmesh,
                self.options.router.routed_band_width,
            );
            if let Some(state) = search_state {
                route.restore_search_state(state);
            }
            route.set_obstacle_penalties(self.configuration.obstacle_penalties.clone());
            self.route = Some(route);

            return Ok(());
        }

        Ok(())
    }

    /// Dissolves the in-flight route and pushes one aligned entry onto
    /// both edit vecs: any pending work (band registry updates,
    /// surgical rip-ups) applied since the last boundary, plus the
    /// route's own edit. Pushed unconditionally so the vecs stay
    /// index-aligned for `backtrace_to_index`, even for skipped
    /// ratlines that produce no board data edit.
    ///
    /// Returns the dissolved route's retained search state with its
    /// ratline's terminating dots, so the caller can store it against
    /// the right board snapshot — after any board reversion that
    /// immediately follows the dissolve.
    fn dissolve_route_stepper_and_push_edits(
        &mut self,
    ) -> Option<(FixedDotIndex, FixedDotIndex, NavcordSearchState)> {
        let extracted = self.route.take().map(|taken_route| {
            let (thetastar, mut navcord, ..) = taken_route.dissolve();
            let state = navcord.take_search_state();
            self.pending_edit.layout_edit.merge(navcord.recorder);
            let navmesh = thetastar.graph();
            (navmesh.origin(), navmesh.destination(), state)
        });

        let pending = std::mem::replace(&mut self.pending_edit, BoardEdit::new());
        self.layout_edits.push(pending.layout_edit);
        self.board_data_edits.push(pending.board_data_edit);
        self.slot_ratlines.push(self.attempt_ratline);
        extracted
    }

    /// Finishes routing early: the in-flight ratline's partial band is
    /// undone, while already routed bands are kept. The stepper must be
    /// stepped once more so that it produces the accumulated edit.
    pub fn finish_early(&mut self, autorouter: &mut Autorouter<impl AccessMesadata>) {
        if self.curr_ratline_index >= self.configuration.ratlines.len() {
            return;
        }

        let extracted = self.route.take().map(|taken_route| {
            // Undo the in-flight ratline's partially drawn band, along
            // with any uncommitted pending work such as surgical
            // rip-ups.
            let (thetastar, mut navcord, ..) = taken_route.dissolve();
            let state = navcord.take_search_state();
            self.pending_edit.layout_edit.merge(navcord.recorder);
            let navmesh = thetastar.graph();
            (navmesh.origin(), navmesh.destination(), state)
        });

        let pending = std::mem::replace(&mut self.pending_edit, BoardEdit::new());
        autorouter.board.apply_edit(&pending.reverse());

        if let Some((origin, destination, state)) = extracted {
            self.store_search_state(autorouter, origin, destination, state);
        }

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

        let extracted = self.route.take().map(|taken_route| {
            let (thetastar, mut navcord, ..) = taken_route.dissolve();
            let state = navcord.take_search_state();
            let edit = BoardEdit::new_from_edits(BoardDataEdit::new(), navcord.recorder);
            autorouter.board.apply_edit(&edit.reverse());
            let navmesh = thetastar.graph();
            (navmesh.origin(), navmesh.destination(), state)
        });

        // The stored snapshot reflects the board right after the
        // in-flight route's own reversion but before the rip-up
        // removals below — a retry only reuses it if that exact state
        // recurs.
        if let Some((origin, destination, state)) = extracted {
            self.store_search_state(autorouter, origin, destination, state);
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
                    .map(|index| (index, *ratline))
            })
            .filter(|(index, _)| *index < self.layout_edits.len())
            .collect();
        indices.sort_unstable_by_key(|(index, _)| *index);
        indices.dedup();

        let mut ripped_edits = BoardEdit::new();
        for (index, ratline) in indices.iter().rev() {
            debug_assert_eq!(
                self.slot_ratlines.remove(*index),
                Some(*ratline),
                "edit slot {index} must hold the ripped ratline's edits",
            );
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
            let (origin, destination) = self
                .configuration
                .ratline_terminating_dots(autorouter, index);

            let Some(band) = autorouter.board.band_between_nodes(origin, destination) else {
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
                // Unweaving can remove the attempted ratline itself
                // (a weave continuation); resume at the position it
                // occupied, which now holds the next ratline.
                .unwrap_or(self.curr_ratline_index)
                .min(self.configuration.ratlines.len())
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
                .unwrap_or_else(|| {
                    self.configuration
                        .ratlines
                        .len()
                        .saturating_sub(rip_up_count)
                })
        };

        if self.curr_ratline_index < self.configuration.ratlines.len() {
            self.start_routing_ratline(autorouter)?;
        }
        Ok(result)
    }

    /// Installs an ordering whose first difference from the live one
    /// lies past every recorded edit slot — a permutation of the
    /// un-attempted tail only. No routed band needs undoing, so the
    /// configuration is adopted and routing resumes at the first
    /// changed position; every reordered ratline is still attempted.
    /// The emit loop falls back to this after skipping a bounded
    /// number of tail-only candidates, which keeps a pathological
    /// emitter from draining its permutation space inside a single
    /// `reconfigure()` call.
    pub fn install_tail_configuration(
        &mut self,
        autorouter: &mut Autorouter<impl AccessMesadata>,
        new_configuration: PlanarAutorouteConfiguration,
    ) -> Result<PlanarAutorouteConfigurationStatus, AutorouterError> {
        let new_index = new_configuration
            .ratlines
            .iter()
            .zip(self.configuration.ratlines.iter())
            .position(|(permuted, original)| *permuted != *original)
            .unwrap_or(usize::MAX);

        let result = PlanarAutorouteConfigurationStatus {
            configuration: std::mem::replace(&mut self.configuration, new_configuration),
            costs: PlanarAutorouteCosts {
                lengths: (0..self.curr_ratline_index).map(|_| 0.0).collect(),
            },
        };

        if self.route.is_some() {
            self.dissolve_route_stepper_and_push_edits();
        }
        if new_index >= self.board_data_edits.len() {
            self.curr_ratline_index = new_index.min(self.configuration.ratlines.len());
        } else {
            // The just-dissolved in-flight slot now covers the first
            // changed position — that position holds a different
            // ratline under the new ordering, so drop the slot and
            // resume there.
            self.backtrace_to_index(autorouter, new_index)?;
        }

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
        self.configuration
            .terminating_dot_map
            .extend(entries.iter());
    }

    /// Removes every terminating-dot remapping belonging to `ratlines`.
    /// Multilayer re-layering calls this for ratlines whose fanout vias
    /// were removed: without it the map keeps `FixedDotIndex`es of dead
    /// dots whose node slots may be recycled as compound nodes, and the
    /// next `ratline_terminating_dots` lookup dereferences a stale
    /// index.
    pub fn remove_terminating_dots(&mut self, ratlines: &[RatlineUid]) {
        self.configuration
            .terminating_dot_map
            .retain(|(ratline, _), _| !ratlines.contains(ratline));
    }

    /// Removes specific terminating-dot remapping keys from the live
    /// configuration — for unweaving a single endpoint's remap without
    /// dropping the ratline's other entries.
    pub fn remove_terminating_dot_entries(&mut self, keys: &[(RatlineUid, FixedDotIndex)]) {
        self.configuration
            .terminating_dot_map
            .retain(|key, _| !keys.contains(key));
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
            // Ratlines here completed (or were skipped with their band
            // already in place): the extracted search state is keyed
            // by a board still holding the ratline's own band, a state
            // no retry can see — requeues only happen after rip-ups.
            // Dropping it keeps the store's MRU budget for
            // failed/in-flight attempts.
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
                self.slot_ratlines.push(None);
            }

            self.slot_ratlines.clear();
            // The produced edit is committed: its slots are consumed
            // so that a later round (the multilayer reconfigurator may
            // restart this stepper after `Break`) cannot backtrace or
            // rip them up — records already handed out must not be
            // reverted and dropped, which would leave dead creations
            // in the committed edit.
            return Ok(ControlFlow::Break(Some(BoardEdit::new_from_edits(
                BoardDataEdit::merge_iter(self.board_data_edits.drain(..)),
                // FIXME: This is a large clone. We probably need some
                // high-level, say `AutorouteEdit`, struct to store sequences
                // of edits.
                LayoutEdit::merge_iter(self.layout_edits.drain(..)),
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
            // Same as the completion path above: the dissolved route's
            // ratline has its band in place, so the search state can
            // never be handed out.
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

#[cfg(test)]
mod tests {
    use std::{collections::BTreeMap, fs::File, io::BufReader};

    use specctra_core::mesadata::SpecctraMesadata;

    use super::*;
    use crate::{
        autorouter::{PresortBy, ReconfigureBy},
        board::edit::BoardEdit,
        drawing::dot::FixedDotIndex,
        router::RouterOptions,
        specctra::design::SpecctraDesign,
        stepper::TimeoutOptions,
    };

    fn autorouter_0603() -> Autorouter<SpecctraMesadata> {
        let design_file = File::open("tests/unilayer/0603_breakout/0603_breakout.dsn").unwrap();
        let design = SpecctraDesign::load(BufReader::new(design_file)).unwrap();
        Autorouter::new(design.make_board(&mut BoardEdit::new())).unwrap()
    }

    /// A terminating-dot entry for a via that no longer exists must not
    /// outlive the via: `remove_terminating_dots` purges it so the
    /// ratline falls back to its endpoint dots instead of dereferencing
    /// a node slot that a later `add_via` may have recycled as a
    /// compound.
    #[test]
    fn remove_terminating_dots_drops_dead_via_entries() {
        let mut autorouter = autorouter_0603();

        let ratline = autorouter
            .ratsnests()
            .on_principal_layer(0)
            .graph()
            .edge_indices()
            .next()
            .map(|index| RatlineUid {
                principal_layer: 0,
                index,
            })
            .unwrap();
        let endpoints = ratline.ref_(&autorouter).endpoint_dots();

        let mut stepper = PlanarAutorouteExecutionStepper::new(
            &mut autorouter,
            PlanarAutorouteConfiguration {
                ratlines: vec![ratline],
                terminating_dot_map: BTreeMap::new(),
                obstacle_penalties: BTreeMap::new(),
                rip_up: vec![],
                protected: BTreeSet::new(),
            },
            PlanarAutorouteOptions {
                principal_layer: 0,
                presort_by: PresortBy::RatlineIntersectionCountAndLength,
                permutate: true,
                reconfigure_by: ReconfigureBy::default(),
                congestion_penalty: 500.0,
                router: RouterOptions {
                    wrap_around_bands: true,
                    squeeze_through_under_bends: false,
                    routed_band_width: 100.0,
                },
                timeout: TimeoutOptions {
                    initial: 1.0,
                    progress_bonus: 0.005,
                },
                max_reconfigurations: None,
                improve_pad_entry: false,
                pad_entry_min_angle: std::f64::consts::FRAC_PI_4,
                escape_stubs: false,
                surgical_ripup: false,
                shuffle_seed: 0,
            },
        )
        .unwrap();

        // Simulate a fanout-via dot that has since been removed: an
        // out-of-range index stands in for the recycled node slot.
        let dead_dot = FixedDotIndex::new(1_000_000);
        stepper.remap_terminating_dots(&BTreeMap::from([((ratline, endpoints.0), dead_dot)]));

        assert_eq!(
            stepper
                .configuration()
                .ratline_terminating_dots(&autorouter, 0),
            (dead_dot, endpoints.1),
        );

        stepper.remove_terminating_dots(&[ratline]);

        assert_eq!(
            stepper
                .configuration()
                .ratline_terminating_dots(&autorouter, 0),
            endpoints,
        );
    }
}
