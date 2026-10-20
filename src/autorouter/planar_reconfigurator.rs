// SPDX-FileCopyrightText: 2025 Topola contributors
//
// SPDX-License-Identifier: MIT

use std::{
    collections::{BTreeMap, BTreeSet},
    ops::ControlFlow,
};

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
    /// Critical-first ratlines whose committed bands must survive
    /// reconfiguration: every emitted ordering keeps them first and
    /// never lists them for rip-up — unless a protected band is the
    /// only way forward, when desperation revokes its protection.
    protected: BTreeSet<RatlineUid>,
    /// Ratlines whose protection was revoked by desperation. Kept for
    /// introspection; protection is monotone-shrinking so a revoked
    /// ratline is never re-protected by this reconfigurator.
    desperated: BTreeSet<RatlineUid>,
}

impl PlanarAutorouteReconfigurator {
    pub fn new(
        autorouter: &mut Autorouter<impl AccessMesadata>,
        input: PlanarAutoroutePreconfigurerInput,
        options: PlanarAutorouteOptions,
    ) -> Result<Self, AutorouterError> {
        Self::new_with_configuration_override(autorouter, input, options, |_, _| {})
    }

    /// Like [`PlanarAutorouteReconfigurator::new`], but
    /// `configuration_override` runs on the freshly built
    /// preconfiguration (after escape-stub placement, which may remap
    /// terminating dots) before the inner stepper and reconfigurer
    /// consume it. The critical-first outer stepper uses it to inject
    /// its ordering, carve corridors onto the board, and seed the
    /// protected set — so the routing history starts with the
    /// critical-first order already installed, no surgical dance.
    pub fn new_with_configuration_override<M: AccessMesadata>(
        autorouter: &mut Autorouter<M>,
        input: PlanarAutoroutePreconfigurerInput,
        options: PlanarAutorouteOptions,
        configuration_override: impl FnOnce(&mut Autorouter<M>, &mut PlanarAutorouteConfiguration),
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

        // The lazy escape-stub survey lives on the inner stepper: it
        // alone sees endpoint-unreachable skips, which never surface
        // as errors here.
        configuration_override(autorouter, &mut preconfiguration);

        let protected = preconfiguration.protected.clone();
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
            protected,
            desperated: BTreeSet::new(),
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

    /// Removes every terminating-dot remapping belonging to `ratlines`
    /// from both the stepper's live configuration and the reconfigurer's
    /// stored preconfiguration. Multilayer re-layering calls this for
    /// ratlines whose fanout vias were removed — a retained entry would
    /// point at a freed node slot that can be recycled as a compound,
    /// and dereferencing it panics.
    pub fn remove_terminating_dots(&mut self, ratlines: &[RatlineUid]) {
        self.stepper.remove_terminating_dots(ratlines);
        self.reconfigurer.remove_terminating_dots(ratlines);
    }

    /// Registers a mid-run-created ratline — a via-weave
    /// continuation — in the reconfigurer's stored preconfigurations
    /// so subsequently emitted orderings keep it. The stepper's live
    /// configuration is updated separately by the caller's surgical
    /// commit.
    pub fn insert_ratline_after(&mut self, ratline: RatlineUid, after: RatlineUid) {
        self.reconfigurer.insert_ratline_after(ratline, after);
    }

    /// Drops a mid-run-created ratline from the reconfigurer's stored
    /// preconfigurations — the mirror of [`Self::insert_ratline_after`],
    /// for unweaving.
    pub fn remove_ratline(&mut self, ratline: RatlineUid) {
        self.reconfigurer.remove_ratline(ratline);
    }

    /// Removes specific terminating-dot remapping keys from both the
    /// stepper's live configuration and the reconfigurer's stored
    /// preconfigurations — for unweaving a single endpoint's remap
    /// without dropping the ratline's other entries.
    pub fn remove_terminating_dot_entries(&mut self, keys: &[(RatlineUid, FixedDotIndex)]) {
        self.stepper.remove_terminating_dot_entries(keys);
        self.reconfigurer.remove_terminating_dot_entries(keys);
    }

    /// Replaces the critical-first protected set: the stepper's live
    /// configuration and the reconfigurer's stored preconfigurations
    /// are updated so subsequently emitted orderings keep it.
    pub fn set_protected(&mut self, protected: BTreeSet<RatlineUid>) {
        self.stepper.configuration_mut().protected = protected.clone();
        self.reconfigurer.set_protected(&protected);
        self.protected = protected;
    }

    /// The ratlines whose bands are currently protected from rip-up
    /// and permutation reordering.
    pub fn protected(&self) -> &BTreeSet<RatlineUid> {
        &self.protected
    }

    /// Feeds externally observed blockers (e.g. relaxed channel probes
    /// run by the critical-first outer stepper) into the
    /// reconfigurer's congestion record so emitted orderings penalize
    /// paths through them.
    pub fn seed_congestion(&mut self, obstacles: impl IntoIterator<Item = PrimitiveIndex>) {
        self.reconfigurer
            .add_congestion(obstacles, self.options.congestion_penalty);
    }

    /// The live configuration of the inner stepper — the ratline order
    /// and terminating-dot map routing is currently working against.
    pub fn configuration(&self) -> &PlanarAutorouteConfiguration {
        self.stepper.configuration()
    }

    /// Mutable access to the live configuration — the anneal handoff
    /// uses it to degrade a planned split whose parent half could not
    /// reach the assumed layer.
    pub fn configuration_mut(&mut self) -> &mut PlanarAutorouteConfiguration {
        self.stepper.configuration_mut()
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

        crate::metrics::incr(&crate::metrics::METRICS.planar_reconfigurations);
        let _reconfigure_timer =
            crate::metrics::Timer::start(&crate::metrics::METRICS.reconfigure_nanos);

        // Bound the emit-and-apply cycle: an ordering that differs
        // from all previously emitted ones can still fail to install
        // (e.g. nothing left to undo), and an unbounded loop here
        // would spin inside a single `step()` where no timeout can
        // fire. `tail_candidate` keeps the newest ordering that
        // permutes only the un-attempted tail — installable without
        // undoing anything — so a search that never produces an
        // ordering touching routed bands still makes progress instead
        // of ending the command.
        let mut tail_candidate = None;
        for _ in 0..256 {
            crate::metrics::incr(&crate::metrics::METRICS.emit_loop_candidates);
            let Some(mut configuration) = self
                .reconfigurer
                .next_configuration(autorouter, &self.stepper)
            else {
                break;
            };

            self.apply_protection(autorouter, &mut configuration);

            let tail_only = configuration.rip_up.is_empty()
                && configuration
                    .ratlines
                    .iter()
                    .zip(self.stepper.configuration().ratlines.iter())
                    .position(|(permuted, original)| *permuted != *original)
                    .is_some_and(|index| index >= self.stepper.board_data_edits().len());
            if tail_only {
                tail_candidate = Some(configuration);
                continue;
            }

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

        if let Some(configuration) = tail_candidate {
            let result = self
                .stepper
                .install_tail_configuration(autorouter, configuration)?;
            return Ok(ControlFlow::Continue(ReconfiguratorStatus::Reconfigured(
                result,
            )));
        }

        Ok(ControlFlow::Break(None))
    }

    /// Enforces the critical-first protected set on an emitted
    /// configuration. Protected ratlines are filtered out of `rip_up`;
    /// desperation escape: when *every* listed rip is protected, those
    /// bands are the only way forward — protection is revoked and the
    /// rip proceeds. Protection only ever shrinks, so this cannot
    /// deadlock.
    ///
    /// The ordering transform keeps critical-first scheduling without
    /// churn: a protected ratline *with a committed band* is pinned at
    /// its live position — the first-difference index lands after it,
    /// so backtracing can never undo its band — while a protected one
    /// *without* a band is scheduled into free slots right after the
    /// current ratline, to be attempted next without ripping the
    /// routed bands ahead of it.
    fn apply_protection(
        &mut self,
        autorouter: &Autorouter<impl AccessMesadata>,
        configuration: &mut PlanarAutorouteConfiguration,
    ) {
        if self.protected.is_empty() {
            return;
        }

        if !configuration.rip_up.is_empty()
            && configuration
                .rip_up
                .iter()
                .all(|ratline| self.protected.contains(ratline))
        {
            for ratline in &configuration.rip_up {
                self.protected.remove(ratline);
                self.desperated.insert(*ratline);
            }
            crate::metrics::incr(&crate::metrics::METRICS.protection_desperations);
            self.stepper.configuration_mut().protected = self.protected.clone();
            self.reconfigurer.set_protected(&self.protected);
        } else {
            let skipped = configuration
                .rip_up
                .iter()
                .filter(|ratline| self.protected.contains(ratline))
                .count();
            if skipped > 0 {
                crate::metrics::add(&crate::metrics::METRICS.protected_rip_skips, skipped);
                configuration
                    .rip_up
                    .retain(|ratline| !self.protected.contains(ratline));
            }
        }

        configuration.protected = self.protected.clone();

        let live = &self.stepper.configuration().ratlines;
        let live_pos: BTreeMap<RatlineUid, usize> = live
            .iter()
            .enumerate()
            .map(|(index, ratline)| (*ratline, index))
            .collect();
        let curr = *self.stepper.curr_ratline_index();

        // Routed protected ratlines hold the front of the ordering in
        // live order: every transformed ordering then shares that
        // prefix with the live one, so the first-difference index
        // always lands after it and backtracing can never undo a
        // protected band. (A protected band later than the first
        // difference would be undone despite being pinned.)
        let mut slots: Vec<Option<RatlineUid>> =
            vec![None; live.len().max(configuration.ratlines.len())];
        let mut pinned = BTreeSet::new();
        let mut front = 0;
        // Front prefix is in *live* order so it stays identical to the
        // live configuration's own protected prefix.
        for ratline in live.iter().copied() {
            if !self.protected.contains(&ratline) || !configuration.ratlines.contains(&ratline) {
                continue;
            }
            let index = live_pos[&ratline];
            let (origin, destination) = self
                .stepper
                .configuration()
                .ratline_terminating_dots(autorouter, index);
            if autorouter
                .board
                .band_between_nodes(origin, destination)
                .is_some()
            {
                slots[front] = Some(ratline);
                front += 1;
                pinned.insert(ratline);
            }
        }

        // Unrouted protected ratlines fill free slots right after the
        // current position — attempted next, without ripping the
        // routed bands ahead of them.
        let urgent_set: BTreeSet<RatlineUid> = self
            .protected
            .iter()
            .copied()
            .filter(|ratline| configuration.ratlines.contains(ratline) && !pinned.contains(ratline))
            .collect();
        let mut urgent = urgent_set.iter().copied();
        for slot in slots.iter_mut().skip(curr + 1) {
            if urgent.len() == 0 {
                break;
            }
            if slot.is_none() {
                *slot = urgent.next();
            }
        }

        let mut rest = configuration
            .ratlines
            .iter()
            .copied()
            .filter(|ratline| !pinned.contains(ratline) && !urgent_set.contains(ratline));
        let mut order = Vec::with_capacity(configuration.ratlines.len());
        for slot in slots {
            match slot {
                Some(ratline) => order.push(ratline),
                None => {
                    if let Some(ratline) = rest.next() {
                        order.push(ratline);
                    }
                }
            }
        }
        order.extend(urgent);
        order.extend(rest);
        configuration.ratlines = order;
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
                .update(*self.estimate_progress().subscale().value())
        {
            if self.reconfiguration_budget_exhausted() {
                self.finish_early(autorouter);
            } else {
                return self.reconfigure(autorouter);
            }
        }

        match self.stepper.step(autorouter) {
            Ok(ControlFlow::Break(maybe_edit)) => Ok(ControlFlow::Break(maybe_edit.map(|edit| {
                let mut merged =
                    BoardEdit::new_from_edits(BoardDataEdit::new(), self.escape_stub_edit.clone());
                merged.merge(edit);
                merged
            }))),
            Ok(ControlFlow::Continue(status)) => {
                Ok(ControlFlow::Continue(ReconfiguratorStatus::Running(status)))
            }
            Err(err) => {
                if self.finished {
                    return Err(err);
                }
                if !self.options.permutate {
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
            &BoardEdit::new_from_edits(BoardDataEdit::new(), self.escape_stub_edit.clone())
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

#[cfg(test)]
mod tests {
    use std::{fs::File, io::BufReader, ops::ControlFlow};

    use specctra_core::mesadata::SpecctraMesadata;

    use super::*;
    use crate::{
        autorouter::{selection::PinSelection, PresortBy, ReconfigureBy},
        router::RouterOptions,
        specctra::design::SpecctraDesign,
        stepper::TimeoutOptions,
    };

    fn options(
        per_ratline_step_limit: Option<usize>,
        lazy_escape_stubs: bool,
    ) -> PlanarAutorouteOptions {
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
                initial: 10.0,
                progress_bonus: 0.0,
            },
            max_reconfigurations: Some(0),
            improve_pad_entry: false,
            pad_entry_min_angle: std::f64::consts::FRAC_PI_4,
            escape_stubs: false,
            lazy_escape_stubs,
            per_ratline_step_limit,
            navmesh_step_factor: None,
            surgical_ripup: false,
            shuffle_seed: 0,
        }
    }

    fn autorouter_led_matrix() -> Autorouter<SpecctraMesadata> {
        let design_file = File::open(
            "tests/unilayer/4x4_1206_led_matrix_breakout/4x4_1206_led_matrix_breakout.dsn",
        )
        .unwrap();
        let design = SpecctraDesign::load(BufReader::new(design_file)).unwrap();
        Autorouter::new(design.make_board(&mut BoardEdit::new())).unwrap()
    }

    /// Lazy escape stubs fire on demand: with every attempt capped at
    /// one router step the first ratline's crowded pad gets stubbed
    /// and the ratline re-armed against the stub dot — and when the
    /// retries also fail the command still finishes early instead of
    /// erroring, with the stub geometry folded into the produced edit.
    /// Since no attempt completes, every seg in the edit is a stub.
    #[test]
    fn lazy_escape_stubs_fire_on_route_failure() {
        let mut autorouter = autorouter_led_matrix();
        let selection = PinSelection::new_select_layer(autorouter.board(), 0);

        let mut reconfigurator = autorouter
            .planar_autoroute(&selection, options(Some(1), true))
            .unwrap();

        let mut maybe_edit = None;
        for _ in 0..10_000 {
            if let ControlFlow::Break(edit) = reconfigurator.step(&mut autorouter).unwrap() {
                maybe_edit = Some(edit);
                break;
            }
        }

        let edit = maybe_edit
            .expect("the command should finish early")
            .expect("a finished command yields its edit");
        assert!(
            edit.layout_edit.created_segs().next().is_some(),
            "the produced edit should hold the lazily laid stub segs"
        );
    }

    /// With lazy stubbing off the same forced failure must not lay
    /// anything: the produced edit carries no geometry at all.
    #[test]
    fn lazy_escape_stubs_off_lays_nothing() {
        let mut autorouter = autorouter_led_matrix();
        let selection = PinSelection::new_select_layer(autorouter.board(), 0);

        let mut reconfigurator = autorouter
            .planar_autoroute(&selection, options(Some(1), false))
            .unwrap();

        let mut maybe_edit = None;
        for _ in 0..10_000 {
            if let ControlFlow::Break(edit) = reconfigurator.step(&mut autorouter).unwrap() {
                maybe_edit = Some(edit);
                break;
            }
        }

        let edit = maybe_edit
            .expect("the command should finish early")
            .expect("a finished command yields its edit");
        assert!(edit.layout_edit.created_segs().next().is_none());
    }
}
