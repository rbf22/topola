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
        primitive::GetLimbs,
    },
    geometry::{
        edit::{ApplyGeometryEdit, Edit},
        primitive::PrimitiveShape,
        GetLayer,
    },
    graph::MakeRef,
    layout::LayoutEdit,
    router::{
        navcord::{Navcord, NavcordSearchState},
        navmesh::{Navmesh, NavmeshError},
        prenavmesh::PrenavmeshBases,
        thetastar::{ThetastarError, ThetastarStepper},
        RouteStepper, Router,
    },
    stepper::{Abort, EstimateProgress, LinearScale, Reconfigure, Step},
};

use super::{
    escape_stubs, invoker::GetDebugOverlayData, pad_entry, ratline::RatlineUid, Autorouter,
    AutorouterError, PlanarAutorouteOptions,
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
    /// The ratline whose band the last `Routed` status committed —
    /// unlike `attempt_ratline` (which already names the *next*
    /// attempt when a status is returned), this stays with the routed
    /// ratline so bookkeeping attributes progress correctly.
    last_routed_ratline: Option<RatlineUid>,
    /// Ratlines skipped because an endpoint was navmesh-unreachable —
    /// the sealed-pad failure class. They produce no `Err` (the
    /// frontier must keep moving), so the reconfigurator drains this
    /// queue each step to count them against the failure deadline;
    /// otherwise a ratline whose endpoint is permanently ringed could
    /// neither accumulate failures nor be respanned onto reachable
    /// endpoints.
    unreachable_failed: Vec<RatlineUid>,
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
    /// Lazy escape stubs: the crowded-pad survey, populated when the
    /// option is set. No stub is laid until an attempt on a crowded
    /// pad fails or finds its endpoint navmesh-unreachable.
    lazy_stubs: Option<escape_stubs::PadEscapeAnalysis>,
    /// Pads already stubbed: pad dot → stub dot. Re-asserted into each
    /// installed configuration's `terminating_dot_map`, since emitted
    /// orderings reset that map.
    stubbed: BTreeMap<FixedDotIndex, FixedDotIndex>,
    /// Pads a lazy stub was attempted for but could not be placed —
    /// keeps `stubbed` for successes alone and still bounds retries to
    /// one per pad.
    stub_failed: BTreeSet<FixedDotIndex>,
    /// Geometry of lazily laid stubs, one edit per pad so a stub that
    /// no band ever used can be retracted alone. Kept out of the
    /// per-ratline slots so backtracing never drops a stub that
    /// `terminating_dot_map` still points at; live stubs merge into
    /// the produced edit, all remaining stubs reverse on abort.
    stub_edits: BTreeMap<FixedDotIndex, LayoutEdit>,
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

        // Eager stubbing wins when both flags are set — the survey is
        // only worth its cost when stubs can actually be laid lazily.
        let lazy_stubs = (options.lazy_escape_stubs && !options.escape_stubs)
            .then(|| escape_stubs::analyze_pads(autorouter, &configuration.ratlines, &options));

        let mut this = Self {
            configuration,
            curr_ratline_index: 0,
            route: None,
            layout_edits: vec![],
            board_data_edits: vec![],
            slot_ratlines: vec![],
            attempt_ratline: None,
            last_routed_ratline: None,
            unreachable_failed: Vec::new(),
            lazy_stubs,
            stubbed: BTreeMap::new(),
            stub_failed: BTreeSet::new(),
            stub_edits: BTreeMap::new(),
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
        self.start_routing_ratline(autorouter)
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
                        unreach @ (NavmeshError::UnreachableOrigin
                        | NavmeshError::UnreachableDestination),
                    )) => {
                        // A sealed endpoint is exactly what lazy escape
                        // stubs are for: if the failing endpoint is a
                        // crowded pad, lay its stub and retry the same
                        // position against the retargeted dot.
                        let (source, target) = self.configuration.ratlines[self.curr_ratline_index]
                            .ref_(autorouter)
                            .endpoint_dots();
                        let candidates = if matches!(unreach, NavmeshError::UnreachableOrigin) {
                            [source, target]
                        } else {
                            [target, source]
                        };
                        if self.try_lazy_stub(autorouter, candidates)? {
                            continue;
                        }

                        // The endpoint is sealed off — e.g. a pad
                        // ringed by foreign geometry. Leave the ratline
                        // unrouted and move on: erroring here retries
                        // the same ratline at the head of every
                        // reconfiguration and aborts the command.
                        // An empty edit slot keeps `layout_edits`
                        // index-aligned with the ratline list. The
                        // skip still counts as a failure for the
                        // reconfigurator's deadline bookkeeping.
                        log::debug!(
                            "ratline {:?} endpoint unreachable in navmesh — skipping",
                            self.configuration.ratlines[self.curr_ratline_index]
                        );
                        self.unreachable_failed
                            .push(self.configuration.ratlines[self.curr_ratline_index]);
                        self.dissolve_route_stepper_and_push_edits();
                        self.curr_ratline_index += 1;
                        continue;
                    }
                    Err(err) => return Err(err),
                };

            let navnodes = navmesh.graph().node_count();
            log::debug!(
                "ratline {:?} attempt on layer {}: {} navnodes",
                self.configuration.ratlines[self.curr_ratline_index],
                origin
                    .primitive_ref(autorouter.board().layout().drawing())
                    .layer(),
                navnodes
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
            route.set_step_limit(self.options.step_limit_for(navnodes));
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
    /// Folds the in-flight attempt's recorded geometry into
    /// `pending_edit` and drops the stepper, returning its endpoints
    /// and reusable search state. Pushes no slot — use this when the
    /// current position is being retried in place (lazy stubs), so the
    /// position still produces exactly one slot when it finishes.
    fn dissolve_route_stepper(
        &mut self,
    ) -> Option<(FixedDotIndex, FixedDotIndex, NavcordSearchState)> {
        self.route.take().map(|taken_route| {
            let (thetastar, mut navcord, ..) = taken_route.dissolve();
            let state = navcord.take_search_state();
            self.pending_edit.layout_edit.merge(navcord.recorder);
            let navmesh = thetastar.graph();
            (navmesh.origin(), navmesh.destination(), state)
        })
    }

    fn dissolve_route_stepper_and_push_edits(
        &mut self,
    ) -> Option<(FixedDotIndex, FixedDotIndex, NavcordSearchState)> {
        let extracted = self.dissolve_route_stepper();

        let pending = std::mem::replace(&mut self.pending_edit, BoardEdit::new());
        self.layout_edits.push(pending.layout_edit);
        self.board_data_edits.push(pending.board_data_edit);
        self.slot_ratlines.push(self.attempt_ratline);
        extracted
    }

    /// Lazy escape stubs: when the in-flight ratline terminates at a
    /// crowded pad that has no stub yet, lays the stub now and
    /// retargets the ratline (and every other ratline on the pad) at
    /// the stub dot, so the retry only has to reach open space. The
    /// stub goes into a per-pad `stub_edits` entry — kept out of the
    /// per-ratline slots so backtracing never drops a stub the
    /// terminating-dot map still points at, and retractable if no band
    /// ever reaches it — and the remap is re-asserted on every
    /// installed ordering by `restore_stub_remaps`. Returns `true`
    /// when a stub was laid; one attempt per pad bounds retries.
    fn try_lazy_stub(
        &mut self,
        autorouter: &mut Autorouter<impl AccessMesadata>,
        candidates: [FixedDotIndex; 2],
    ) -> Result<bool, AutorouterError> {
        if self.lazy_stubs.is_none() {
            return Ok(false);
        }
        let width = self.options.router.routed_band_width;

        for pad in candidates {
            let Some(analysis) = &self.lazy_stubs else {
                break;
            };
            let Some(directions) = analysis.crowded.get(&pad) else {
                continue;
            };
            if self.stubbed.contains_key(&pad) || self.stub_failed.contains(&pad) {
                continue;
            }

            let mut stub_edit = LayoutEdit::new();
            let Some(stub) = escape_stubs::stub_pad(
                autorouter,
                &mut stub_edit,
                pad,
                analysis.positions[&pad],
                analysis.radius,
                directions,
                width,
                true,
            ) else {
                self.stub_failed.insert(pad);
                continue;
            };

            self.stubbed.insert(pad, stub);
            self.stub_edits.insert(pad, stub_edit);
            for ratline in &self.lazy_stubs.as_ref().unwrap().pad_ratlines[&pad] {
                // A ratline whose band already exists keeps resolving
                // to the pad — its band is registered under the pad
                // dot, so remapping it would orphan the band from
                // rip-up resolution.
                let (source, target) = ratline.ref_(autorouter).endpoint_dots();
                let map = &self.configuration.terminating_dot_map;
                let source = map.get(&(*ratline, source)).copied().unwrap_or(source);
                let target = map.get(&(*ratline, target)).copied().unwrap_or(target);
                if autorouter
                    .board
                    .band_between_nodes(source, target)
                    .is_some()
                {
                    continue;
                }
                self.configuration
                    .terminating_dot_map
                    .insert((*ratline, pad), stub);
            }
            return Ok(true);
        }

        Ok(false)
    }

    /// Re-asserts stub remaps on a freshly installed ordering: emitted
    /// configurations carry their own `terminating_dot_map`, which
    /// drops mid-run stub remaps — without this a requeued stubbed
    /// ratline would re-attempt the sealed pad it just failed on, and
    /// rip-ups would miss its band (registered under the stub dot).
    /// Existing entries win: a multilayer via remap supersedes a stub,
    /// and a ratline whose band is registered under the un-remapped
    /// pad is left alone so rip-ups still find it.
    fn restore_stub_remaps(
        &self,
        autorouter: &Autorouter<impl AccessMesadata>,
        configuration: &mut PlanarAutorouteConfiguration,
    ) {
        if self.stubbed.is_empty() {
            return;
        }
        let Some(analysis) = &self.lazy_stubs else {
            return;
        };
        for (pad, stub) in &self.stubbed {
            for ratline in analysis.pad_ratlines.get(pad).into_iter().flatten() {
                let (source, target) = ratline.ref_(autorouter).endpoint_dots();
                let source = configuration
                    .terminating_dot_map
                    .get(&(*ratline, source))
                    .copied()
                    .unwrap_or(source);
                let target = configuration
                    .terminating_dot_map
                    .get(&(*ratline, target))
                    .copied()
                    .unwrap_or(target);
                if autorouter
                    .board
                    .band_between_nodes(source, target)
                    .is_some()
                {
                    continue;
                }
                configuration
                    .terminating_dot_map
                    .entry((*ratline, *pad))
                    .or_insert(*stub);
            }
        }
    }

    /// Retracts stubs no routed band ever reached: a stub that carries
    /// no band limb is dead fixed-geometry debris — an unnecessary
    /// foreign-net obstacle in the produced board. The stub's edit is
    /// reverse-applied and its remap dropped so a post-`Break` restart
    /// (the multilayer reconfigurator reuses this stepper) cannot
    /// re-assert a mapping onto a removed dot.
    fn retract_dead_stubs(&mut self, autorouter: &mut Autorouter<impl AccessMesadata>) {
        let dead: Vec<FixedDotIndex> = self
            .stubbed
            .iter()
            .filter(|(_, stub)| {
                stub.primitive_ref(autorouter.board().layout().drawing())
                    .limbs()
                    .len()
                    <= 1
            })
            .map(|(&pad, _)| pad)
            .collect();

        for pad in dead {
            self.stubbed.remove(&pad);
            if let Some(edit) = self.stub_edits.remove(&pad) {
                autorouter.board.layout_mut().apply(&edit.reverse());
            }
            self.configuration
                .terminating_dot_map
                .retain(|&(_, dot), _| dot != pad);
        }
    }

    /// Drains ratlines skipped for a navmesh-unreachable endpoint since
    /// the last drain — the reconfigurator counts each against the
    /// failing ratline's deadline streak.
    pub fn take_unreachable_failed(&mut self) -> Vec<RatlineUid> {
        std::mem::take(&mut self.unreachable_failed)
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

        // Drop the ripped-up ratlines' edit slots. The dropped entries
        // merge into `pending_edit` before the surgical removals so
        // each ripped band's creation and removal records coalesce into
        // a no-op instead of resurrecting the band on undo.
        //
        // Slots are located via `slot_ratlines` — NOT by position in
        // `configuration.ratlines`. The edit vecs are append-only in
        // attempt order, while `install_configuration` swaps in a
        // reordered ratline list on every reconfiguration, so a
        // ratline's current position no longer matches the slot its
        // edits were pushed into.
        let mut indices: Vec<usize> = rip_up
            .iter()
            .flat_map(|ratline| {
                self.slot_ratlines
                    .iter()
                    .enumerate()
                    .filter_map(move |(index, slot)| (*slot == Some(*ratline)).then_some(index))
            })
            .collect();
        indices.sort_unstable();
        indices.dedup();

        debug_assert_eq!(self.slot_ratlines.len(), self.layout_edits.len());
        debug_assert_eq!(self.slot_ratlines.len(), self.board_data_edits.len());

        let mut ripped_edits = BoardEdit::new();
        for index in indices.iter().rev() {
            self.slot_ratlines.remove(*index);
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
                .position(|uid| uid == ratline);
            let (origin, destination) = if let Some(index) = index {
                self.configuration
                    .ratline_terminating_dots(autorouter, index)
            } else {
                // A ratline already dropped from the live ordering —
                // e.g. declared dead and scrubbed — can still hold a
                // band that must be ripped. Fall back to its own
                // endpoints through the terminating-dot map.
                let (origin, destination) = ratline.ref_(autorouter).endpoint_dots();
                let map = &self.configuration.terminating_dot_map;
                (
                    map.get(&(*ratline, origin)).copied().unwrap_or(origin),
                    map.get(&(*ratline, destination))
                        .copied()
                        .unwrap_or(destination),
                )
            };

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
        mut new_configuration: PlanarAutorouteConfiguration,
    ) -> Result<PlanarAutorouteConfigurationStatus, AutorouterError> {
        let attempted = self
            .configuration
            .ratlines
            .get(self.curr_ratline_index)
            .copied();
        let first_requeued = new_configuration.rip_up.first().copied();
        let rip_up_count = new_configuration.rip_up.len();
        self.restore_stub_remaps(autorouter, &mut new_configuration);

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

        self.curr_ratline_index = self.first_unattempted_position(autorouter);

        if self.curr_ratline_index < self.configuration.ratlines.len() {
            self.start_routing_ratline(autorouter)?;
        }
        Ok(result)
    }

    /// The earliest ratline-list position that still needs an attempt:
    /// no recorded attempt slot and no routed band. Resuming any later
    /// would silently skip it — it would never be attempted — which is
    /// how slot/position alignment and routing completeness both break
    /// when an emitted ordering lands a continuation or requeued
    /// ratline ahead of the nominal resume point. When the emit
    /// contract holds (survivors keep their prefix, requeued go after
    /// the resume point) every pre-resume position has a slot or a
    /// band, so this equals the nominal resume point.
    fn first_unattempted_position(&self, autorouter: &Autorouter<impl AccessMesadata>) -> usize {
        let slotted: BTreeSet<RatlineUid> = self.slot_ratlines.iter().flatten().copied().collect();
        (0..self
            .curr_ratline_index
            .min(self.configuration.ratlines.len()))
            .find(|&index| {
                if slotted.contains(&self.configuration.ratlines[index]) {
                    return false;
                }
                let (origin, destination) = self
                    .configuration
                    .ratline_terminating_dots(autorouter, index);
                autorouter
                    .board
                    .band_between_nodes(origin, destination)
                    .is_none()
            })
            .unwrap_or(self.curr_ratline_index)
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
        mut new_configuration: PlanarAutorouteConfiguration,
    ) -> Result<PlanarAutorouteConfigurationStatus, AutorouterError> {
        let new_index = new_configuration
            .ratlines
            .iter()
            .zip(self.configuration.ratlines.iter())
            .position(|(permuted, original)| *permuted != *original)
            .unwrap_or(usize::MAX);
        self.restore_stub_remaps(autorouter, &mut new_configuration);

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
            // Positions between the walked prefix and `new_index` are
            // unchanged but unattempted — a tail permutation that
            // shares a long prefix must not jump past them.
            self.curr_ratline_index = self.first_unattempted_position(autorouter);
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
            // Retract stubs whose ratlines never routed a band to them:
            // dead fixed-geometry debris otherwise committed into the
            // produced edit.
            self.retract_dead_stubs(autorouter);
            let mut layout_edit = LayoutEdit::merge_iter(self.layout_edits.drain(..));
            layout_edit.merge(LayoutEdit::merge_iter(
                std::mem::take(&mut self.stub_edits).into_values(),
            ));
            return Ok(ControlFlow::Break(Some(BoardEdit::new_from_edits(
                BoardDataEdit::merge_iter(self.board_data_edits.drain(..)),
                // FIXME: This is a large clone. We probably need some
                // high-level, say `AutorouteEdit`, struct to store sequences
                // of edits.
                layout_edit,
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
                route.step(&mut router)
            };

            let band_termseg = match band_termseg {
                Ok(ControlFlow::Break(band_termseg)) => band_termseg,
                Ok(ControlFlow::Continue(())) => {
                    return Ok(ControlFlow::Continue(
                        PlanarAutorouteContinueStatus::Running,
                    ));
                }
                Err(err) => {
                    // A stub on a crowded endpoint may unblock the
                    // retry: dissolve the failed attempt and re-arm at
                    // the same position. Only `NotFound` is worth
                    // retrying — other errors describe geometry or
                    // state no stub changes.
                    let candidates = self
                        .attempt_ratline
                        .and_then(|ratline| {
                            let (source, target) = ratline.ref_(autorouter).endpoint_dots();
                            Some([source, target])
                        })
                        .unwrap_or([origin, destination]);
                    if matches!(err, ThetastarError::NotFound)
                        && self.try_lazy_stub(autorouter, candidates)?
                    {
                        // Fold the failed attempt's debris into
                        // `pending_edit` without pushing a slot: the
                        // position produces its single slot only when
                        // it finishes.
                        self.dissolve_route_stepper();
                        self.start_routing_ratline(autorouter)?;
                        return Ok(ControlFlow::Continue(
                            PlanarAutorouteContinueStatus::Running,
                        ));
                    }
                    return Err(err.into());
                }
            };

            let band = autorouter
                .board
                .layout()
                .drawing()
                .find_loose_band_uid(band_termseg.into())
                .expect("a completely routed band should've Seg's as ends");

            let ratline = self.configuration().ratlines[self.curr_ratline_index];
            autorouter
                .ratsnests
                .on_principal_layer_mut(self.options.principal_layer)
                .assign_band_termseg_to_ratline(ratline.index, band_termseg);
            self.last_routed_ratline = Some(ratline);

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
        for edit in std::mem::take(&mut self.stub_edits).into_values() {
            autorouter.board.layout_mut().apply(&edit.reverse());
        }
        self.curr_ratline_index = self.configuration.ratlines.len();
    }
}

impl<M: AccessMesadata> Reconfigure<Autorouter<M>> for PlanarAutorouteExecutionStepper {
    type Configuration = PlanarAutorouteConfiguration;
    type Output = Result<PlanarAutorouteConfigurationStatus, AutorouterError>;

    fn reconfigure(
        &mut self,
        autorouter: &mut Autorouter<M>,
        mut new_configuration: PlanarAutorouteConfiguration,
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
        self.restore_stub_remaps(autorouter, &mut new_configuration);

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
        autorouter::{
            planar_reconfigurator::DEADLINE_FAILURES_PER_LAYER, PresortBy, ReconfigureBy,
        },
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

    fn options_0603() -> PlanarAutorouteOptions {
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
            lazy_escape_stubs: false,
            per_ratline_step_limit: None,
            navmesh_step_factor: None,
            surgical_ripup: false,
            shuffle_seed: 0,
            deadline_failures_per_layer: DEADLINE_FAILURES_PER_LAYER,
        }
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
            options_0603(),
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

    fn stepper_for(
        autorouter: &mut Autorouter<SpecctraMesadata>,
        ratlines: Vec<RatlineUid>,
    ) -> PlanarAutorouteExecutionStepper {
        stepper_for_options(autorouter, ratlines, options_0603())
    }

    fn stepper_for_options(
        autorouter: &mut Autorouter<SpecctraMesadata>,
        ratlines: Vec<RatlineUid>,
        options: PlanarAutorouteOptions,
    ) -> PlanarAutorouteExecutionStepper {
        PlanarAutorouteExecutionStepper::new(
            autorouter,
            PlanarAutorouteConfiguration {
                ratlines,
                terminating_dot_map: BTreeMap::new(),
                obstacle_penalties: BTreeMap::new(),
                rip_up: vec![],
                protected: BTreeSet::new(),
            },
            options,
        )
        .unwrap()
    }

    fn board_ratlines(autorouter: &Autorouter<SpecctraMesadata>, count: usize) -> Vec<RatlineUid> {
        autorouter
            .ratsnests()
            .on_principal_layer(0)
            .graph()
            .edge_indices()
            .take(count)
            .map(|index| RatlineUid {
                principal_layer: 0,
                index,
            })
            .collect()
    }

    /// An emitted ordering may land an unattempted ratline ahead of
    /// the attempted ratline's new position — a weave continuation
    /// inserted before it, or a requeue to the front. Resuming at the
    /// attempted ratline's position would skip it forever: it has no
    /// recorded attempt slot and no band, so `install_configuration`
    /// must resume at the first such position instead.
    #[test]
    fn install_configuration_resumes_at_first_unattempted_position() {
        let mut autorouter = {
            let design_file = File::open(
                "tests/unilayer/4x4_1206_led_matrix_breakout/4x4_1206_led_matrix_breakout.dsn",
            )
            .unwrap();
            let design = SpecctraDesign::load(BufReader::new(design_file)).unwrap();
            Autorouter::new(design.make_board(&mut BoardEdit::new())).unwrap()
        };
        let ratlines = board_ratlines(&autorouter, 3);
        let [a, b, c] = [ratlines[0], ratlines[1], ratlines[2]];

        let mut stepper = stepper_for(&mut autorouter, vec![a, b, c]);
        // `a` was attempted; `b`'s in-flight attempt was dissolved and
        // reverted (as `surgical_rip_up` does before installing).
        stepper.slot_ratlines = vec![Some(a), Some(b)];
        stepper.curr_ratline_index = 1;

        stepper
            .install_configuration(
                &mut autorouter,
                PlanarAutorouteConfiguration {
                    ratlines: vec![c, a, b],
                    terminating_dot_map: BTreeMap::new(),
                    obstacle_penalties: BTreeMap::new(),
                    rip_up: vec![],
                    protected: BTreeSet::new(),
                },
            )
            .unwrap();

        // `b`'s new position is 2, but `c` at position 0 has neither a
        // slot nor a band — resuming at 2 would skip it permanently.
        assert_eq!(stepper.curr_ratline_index, 0);
    }

    /// Same skip on the tail-permutation path: when the first
    /// difference lands past the walked prefix, the positions in
    /// between are unchanged but still unattempted — jumping straight
    /// to `new_index` would never route them.
    #[test]
    fn install_tail_configuration_resumes_at_first_unattempted_position() {
        let mut autorouter = {
            let design_file = File::open(
                "tests/unilayer/4x4_1206_led_matrix_breakout/4x4_1206_led_matrix_breakout.dsn",
            )
            .unwrap();
            let design = SpecctraDesign::load(BufReader::new(design_file)).unwrap();
            Autorouter::new(design.make_board(&mut BoardEdit::new())).unwrap()
        };
        let ratlines = board_ratlines(&autorouter, 4);
        let [a, b, c, d] = [ratlines[0], ratlines[1], ratlines[2], ratlines[3]];

        let mut stepper = stepper_for(&mut autorouter, vec![a, b, c, d]);
        stepper.slot_ratlines = vec![Some(a), Some(b)];
        stepper.curr_ratline_index = 1;

        // Ordering keeps the whole walked prefix and `c`, drops `d`:
        // the first difference is past every recorded slot.
        stepper
            .install_tail_configuration(
                &mut autorouter,
                PlanarAutorouteConfiguration {
                    ratlines: vec![a, b, c],
                    terminating_dot_map: BTreeMap::new(),
                    obstacle_penalties: BTreeMap::new(),
                    rip_up: vec![],
                    protected: BTreeSet::new(),
                },
            )
            .unwrap();

        // Without the gap check `curr_ratline_index` would land at 3,
        // leaving `c` (position 2, unattempted) forever unrouted.
        assert_eq!(stepper.curr_ratline_index, 2);
    }

    /// A lazy stub's terminating-dot remap lives only in the installed
    /// configuration — emitted orderings carry their own map, which
    /// drops it. Without re-assertion a requeued stubbed ratline would
    /// re-attempt the sealed pad it just failed on, and rip-ups would
    /// miss its band (registered under the stub dot).
    #[test]
    fn lazy_stub_remap_survives_configuration_install() {
        let mut autorouter = {
            let design_file = File::open(
                "tests/unilayer/4x4_1206_led_matrix_breakout/4x4_1206_led_matrix_breakout.dsn",
            )
            .unwrap();
            let design = SpecctraDesign::load(BufReader::new(design_file)).unwrap();
            Autorouter::new(design.make_board(&mut BoardEdit::new())).unwrap()
        };
        let ratlines = board_ratlines(&autorouter, 12);

        let mut options = options_0603();
        options.lazy_escape_stubs = true;
        let mut stepper = stepper_for_options(&mut autorouter, ratlines.clone(), options);

        // Lay a stub directly on a crowded pad from the survey, the
        // same call `step()` makes when an attempt fails.
        let (pad, pad_ratlines) = {
            let analysis = stepper.lazy_stubs.as_ref().unwrap();
            let pad = *analysis.crowded.keys().next().unwrap();
            (pad, analysis.pad_ratlines[&pad].clone())
        };
        assert!(stepper.try_lazy_stub(&mut autorouter, [pad, pad]).unwrap());
        let stub = stepper.stubbed[&pad];

        // An emitted ordering arrives with its own
        // terminating-dot map — the mid-run stub remap is absent.
        let mut emitted = ratlines;
        emitted.rotate_left(1);
        stepper
            .install_configuration(
                &mut autorouter,
                PlanarAutorouteConfiguration {
                    ratlines: emitted,
                    terminating_dot_map: BTreeMap::new(),
                    obstacle_penalties: BTreeMap::new(),
                    rip_up: vec![],
                    protected: BTreeSet::new(),
                },
            )
            .unwrap();

        for ratline in pad_ratlines {
            assert_eq!(
                stepper.configuration().terminating_dot_map[&(ratline, pad)],
                stub
            );
        }
    }

    /// A stub no band ever reaches is dead fixed-geometry debris. At
    /// `Break` it is retracted: reverse-applied from the board, absent
    /// from the produced edit, and its pad's remap dropped so a
    /// post-`Break` restart cannot resurrect a mapping onto a removed
    /// dot.
    #[test]
    fn dead_lazy_stub_retracts_at_break() {
        let mut autorouter = {
            let design_file = File::open(
                "tests/unilayer/4x4_1206_led_matrix_breakout/4x4_1206_led_matrix_breakout.dsn",
            )
            .unwrap();
            let design = SpecctraDesign::load(BufReader::new(design_file)).unwrap();
            Autorouter::new(design.make_board(&mut BoardEdit::new())).unwrap()
        };
        let ratlines = board_ratlines(&autorouter, 12);

        let mut options = options_0603();
        options.lazy_escape_stubs = true;
        let mut stepper = stepper_for_options(&mut autorouter, ratlines, options);

        let pad = *stepper
            .lazy_stubs
            .as_ref()
            .unwrap()
            .crowded
            .keys()
            .next()
            .unwrap();
        assert!(stepper.try_lazy_stub(&mut autorouter, [pad, pad]).unwrap());
        assert!(stepper.stub_edits.contains_key(&pad));

        // Nothing routes a band to the stub: finishing early produces
        // the edit with the stub still unused.
        stepper.finish_early(&mut autorouter);
        let edit = match stepper.step(&mut autorouter).unwrap() {
            ControlFlow::Break(edit) => edit.unwrap(),
            ControlFlow::Continue(_) => panic!("a finished stepper must produce its edit"),
        };

        assert!(!stepper.stubbed.contains_key(&pad));
        assert!(!stepper.stub_edits.contains_key(&pad));
        assert!(
            stepper
                .configuration()
                .terminating_dot_map
                .keys()
                .all(|&(_, dot)| dot != pad),
            "the retracted pad's remaps must be dropped"
        );
        assert!(
            edit.layout_edit.created_segs().next().is_none()
                && edit.layout_edit.created_dots().next().is_none(),
            "the retracted stub must not land in the produced edit"
        );
    }
}
