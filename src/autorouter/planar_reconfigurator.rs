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
        connected_components::ConnectedComponents,
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
    drawing::{
        dot::FixedDotIndex,
        graph::{GetMaybeNet, MakePrimitiveRef, PrimitiveIndex},
    },
    geometry::{edit::Edit, primitive::PrimitiveShape},
    graph::{GetIndex, MakeRef},
    layout::LayoutEdit,
    router::{navcord::Navcord, navmesh::Navmesh, thetastar::ThetastarStepper},
    stepper::{
        Abort, EstimateProgress, GetTimeoutProgress, LinearScale, ReconfiguratorStatus,
        Reconfigure, Step, TimeVsProgressAccumulatorTimeout,
    },
    triangulation::GetTrianvertexNodeIndex,
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
    /// Route-attempt failures per ratline since it last routed
    /// successfully — its own success is the only evidence that it
    /// remains viable, so only that resets the count. Without a bound,
    /// a permanently sealed ratline reconfigures forever: each emitted
    /// ordering retries it, it fails, and the cycle repeats until the
    /// step budget is gone (under `--deterministic` the timeout never
    /// fires, so this loop is unbounded there).
    failure_counts: BTreeMap<RatlineUid, usize>,
    /// Ratlines that exhausted their failure budget — declared
    /// unroutable for this command and dropped from every emitted
    /// configuration so the frontier can advance past them.
    dead: BTreeSet<RatlineUid>,
    /// Endpoint pairs already tried to death, per ratsnest
    /// (`principal_layer`). Before a ratline is declared dead the
    /// reconfigurator tries to respan it — rewire its edge to the
    /// shortest same-net vertex pair bridging two still-disjoint
    /// components — and dead pairs are never proposed again, so the
    /// respan→fail→respan chain terminates once a net's alternates
    /// are exhausted.
    dead_pairs: BTreeMap<
        usize,
        BTreeSet<(
            petgraph::graph::NodeIndex<usize>,
            petgraph::graph::NodeIndex<usize>,
        )>,
    >,
    /// Respans already granted per ratline. Uncapped, a uid could walk
    /// a large net's pair space one deadline-cycle at a time — the
    /// same unbounded stall dead-marking exists to prevent — so a uid
    /// dies after [`MAX_RESPANS_PER_RATLINE`] rewires even while the
    /// net still has untried pairs.
    respan_counts: BTreeMap<RatlineUid, usize>,
}

/// Alternate endpoint pairs a ratline may try before it is declared
/// dead. Candidates are proposed shortest-first, so the first few are
/// the plausible bridges; the long tail of a big net's pair space is
/// geometrically unlikely to route and not worth a full failure budget
/// each.
const MAX_RESPANS_PER_RATLINE: usize = 3;

/// Default for
/// [`crate::autorouter::PlanarAutorouteOptions::deadline_failures_per_layer`]:
/// consecutive failures (with no success of the same ratline in
/// between) a ratline may accumulate per routing layer before it is
/// respanned or declared dead. Layered boards legitimately retry a
/// ratline under several layer assignments, so the budget scales with
/// layer count. Measured: astable's hardest ratline needs ~54 stalled
/// failures before it routes — 64/layer leaves that headroom while
/// still bounding the candE-style deadlock (sealed ratlines stall at
/// thousands of cycles).
pub const DEADLINE_FAILURES_PER_LAYER: usize = 64;

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
            failure_counts: BTreeMap::new(),
            dead: BTreeSet::new(),
            dead_pairs: BTreeMap::new(),
            respan_counts: BTreeMap::new(),
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

    /// Ratlines declared unroutable after exhausting their failure
    /// budget — the multilayer reconfigurator consults this to scrub
    /// them from its emitted configurations too.
    pub fn dead_ratlines(&self) -> &BTreeSet<RatlineUid> {
        &self.dead
    }

    /// Counts a route-attempt failure for the ratline at the stepper's
    /// current position; once the streak reaches
    /// [`crate::autorouter::PlanarAutorouteOptions::deadline_failures_per_layer`]
    /// per board layer the ratline is respanned onto an alternate
    /// same-net pair, and declared dead once those are exhausted.
    fn note_route_failure(&mut self, autorouter: &mut Autorouter<impl AccessMesadata>) {
        let index = *self.stepper.curr_ratline_index();
        let Some(&ratline) = self.stepper.configuration().ratlines.get(index) else {
            return;
        };
        // The streak counts this ratline's failures since it last
        // routed successfully — its own success is the only evidence
        // it remains viable, so nothing else may reset it. A ratline
        // that keeps failing while the rest of the board advances is
        // still converging on dead; this is what lets sealed ratlines
        // be detected on slowly-progressing boards at all.
        let streak = {
            let entry = self.failure_counts.entry(ratline).or_insert(0);
            *entry += 1;
            *entry
        };
        let layers = autorouter.board.layout().drawing().layer_count().max(1);
        if streak >= self.options.deadline_failures_per_layer * layers {
            // The ratsnest fixes which pad pairs become ratlines — a
            // sealed pair retries the same endpoints forever even when
            // the same net could be bridged elsewhere. Respan first:
            // rewire the edge to the best untried bridging pair.
            let respans = *self.respan_counts.get(&ratline).unwrap_or(&0);
            if respans < MAX_RESPANS_PER_RATLINE && self.try_respan_ratline(autorouter, ratline) {
                self.respan_counts.insert(ratline, respans + 1);
                self.failure_counts.remove(&ratline);
                log::debug!("ratline {ratline:?} respanned after {streak} stalled attempts");
            } else if self.dead.insert(ratline) {
                log::debug!("ratline {ratline:?} declared dead after {streak} stalled attempts",);
            }
        }
    }

    /// Tries to respan `ratline`: rewire its ratsnest edge to the
    /// shortest same-net vertex pair that bridges two components the
    /// board's union-find still keeps apart. The freed `EdgeIndex` is
    /// reused, so the `RatlineUid` — and with it the ordering slots,
    /// layer assignment and bookkeeping — survives; only the endpoint
    /// pair changes. Pairs already tried to death are never proposed
    /// again (`dead_pairs`), so repeated respans walk the net's
    /// alternates once and then fall through to dead-marking. Returns
    /// false when the net has no remaining candidate — including weave
    /// continuations, whose weave bookkeeping is bound to their
    /// endpoints.
    fn try_respan_ratline(
        &mut self,
        autorouter: &mut Autorouter<impl AccessMesadata>,
        ratline: RatlineUid,
    ) -> bool {
        let ratsnest = autorouter
            .ratsnests
            .on_principal_layer(ratline.principal_layer);
        if ratsnest
            .graph()
            .edge_weight(ratline.index)
            .is_none_or(|weight| weight.continuation || weight.band_termseg.is_some())
        {
            return false;
        }
        let Some((origin, destination)) = ratsnest.graph().edge_endpoints(ratline.index) else {
            return false;
        };
        let dead_pair = (origin.min(destination), origin.max(destination));
        let Some(net) = ratline
            .ref_(autorouter)
            .endpoint_dots()
            .0
            .primitive_ref(autorouter.board.layout().drawing())
            .maybe_net()
        else {
            return false;
        };

        let conncomps = ConnectedComponents::new(autorouter.board());
        let vertex_info = |vertex: petgraph::graph::NodeIndex<usize>| {
            let weight = ratsnest.graph().node_weight(vertex).unwrap();
            let dot = match weight.node_index() {
                crate::autorouter::ratsnest::RatvertexNodeIndex::FixedDot(dot) => dot,
                crate::autorouter::ratsnest::RatvertexNodeIndex::Poly(poly) => {
                    poly.ref_(autorouter.board.layout()).apex()
                }
            };
            (conncomps.unionfind().find(dot.index()), weight.pos, dot)
        };

        // Same-net vertices grouped by their connected component. A
        // respan candidate must bridge two different components — a
        // pair inside one component adds a redundant edge.
        let mut per_component: BTreeMap<usize, Vec<_>> = BTreeMap::new();
        for vertex in ratsnest.graph().node_indices() {
            let (component, pos, dot) = vertex_info(vertex);
            let vertex_net = dot
                .primitive_ref(autorouter.board.layout().drawing())
                .maybe_net();
            if vertex_net == Some(net) {
                per_component
                    .entry(component)
                    .or_default()
                    .push((vertex, pos));
            }
        }

        let dead_pairs = self.dead_pairs.entry(ratline.principal_layer).or_default();
        dead_pairs.insert(dead_pair);

        let mut best: Option<(f64, _, _)> = None;
        let components: Vec<_> = per_component.into_values().collect();
        for (i, side_a) in components.iter().enumerate() {
            for side_b in &components[i + 1..] {
                for (a, pos_a) in side_a {
                    for (b, pos_b) in side_b {
                        let pair = ((*a).min(*b), (*a).max(*b));
                        if dead_pairs.contains(&pair)
                            || ratsnest.graph().find_edge(*a, *b).is_some()
                        {
                            continue;
                        }
                        let distance = (pos_a.x() - pos_b.x()).hypot(pos_a.y() - pos_b.y());
                        if best.is_none_or(|(best_distance, _, _)| distance < best_distance) {
                            best = Some((distance, *a, *b));
                        }
                    }
                }
            }
        }
        let Some((_, a, b)) = best else {
            return false;
        };

        autorouter
            .ratsnests
            .on_principal_layer_mut(ratline.principal_layer)
            .rewire_edge(ratline.index, a, b);
        true
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

            // Dead ratlines are unroutable: omit them so the search
            // advances instead of re-permuting a sealed ratline
            // forever. Their terminating-dot map entries go too — the
            // ratline will never be attempted again.
            if !self.dead.is_empty() {
                configuration
                    .ratlines
                    .retain(|ratline| !self.dead.contains(ratline));
                configuration
                    .rip_up
                    .retain(|ratline| !self.dead.contains(ratline));
                configuration
                    .protected
                    .retain(|ratline| !self.dead.contains(ratline));
                configuration
                    .terminating_dot_map
                    .retain(|(ratline, _), _| !self.dead.contains(ratline));
            }

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
                if matches!(status, PlanarAutorouteContinueStatus::Routed(_)) {
                    // A successful (re-)route is the only reset for the
                    // ratline's failure streak: it just proved it can
                    // still route in the current board state.
                    // `attempt_ratline` already names the next attempt
                    // when this status returns, so use the stepper's
                    // record of which ratline actually routed.
                    if let Some(ratline) = self.stepper.last_routed_ratline() {
                        self.failure_counts.remove(ratline);
                    }
                }
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
                self.note_route_failure(autorouter);
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
            deadline_failures_per_layer: DEADLINE_FAILURES_PER_LAYER,
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
    /// erroring. Since no band ever reaches a stub, every one of them
    /// is dead at `Break` and retracted: the produced edit carries no
    /// stub geometry at all.
    #[test]
    fn lazy_escape_stubs_fire_on_route_failure() {
        let mut autorouter = autorouter_led_matrix();
        let selection = PinSelection::new_select_layer(autorouter.board(), 0);

        let mut reconfigurator = autorouter
            .planar_autoroute(&selection, options(Some(1), true))
            .unwrap();

        let mut stub_fired = false;
        let mut maybe_edit = None;
        for _ in 0..10_000 {
            if let ControlFlow::Break(edit) = reconfigurator.step(&mut autorouter).unwrap() {
                maybe_edit = Some(edit);
                break;
            }
            stub_fired |= !reconfigurator.stepper.stubbed().is_empty();
        }

        assert!(
            stub_fired,
            "a crowded pad's failed attempt should have laid a stub"
        );
        let _edit = maybe_edit
            .expect("the command should finish early")
            .expect("a finished command yields its edit");
        assert!(
            reconfigurator.stepper.stubbed().is_empty()
                && reconfigurator.stepper.stub_edits().is_empty(),
            "dead stubs retract at Break — none may remain claimed"
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

    /// A ratline that keeps failing isn't necessarily unroutable — only
    /// its endpoint pair is. Respanning must rewire the edge in place
    /// (same `EdgeIndex`, so the `RatlineUid` and its ordering slot
    /// survive) to a different vertex pair of the same net, and never
    /// propose the just-died pair again.
    #[test]
    fn dead_end_ratline_respans_to_alternate_pair_of_same_net() {
        let mut autorouter = autorouter_led_matrix();
        let selection = PinSelection::new_select_layer(autorouter.board(), 0);

        let mut reconfigurator = autorouter
            .planar_autoroute(&selection, options(None, false))
            .unwrap();

        // Any ratline of a net with at least three vertices — the
        // fixture's ROW/COLUMN nets have five — has alternate pairs.
        let ratline = autorouter
            .ratsnests()
            .on_principal_layer(0)
            .graph()
            .edge_indices()
            .find(|index| {
                let uid = RatlineUid {
                    principal_layer: 0,
                    index: *index,
                };
                let net = uid.ref_(&autorouter).endpoint_dots().0;
                let net = net
                    .primitive_ref(autorouter.board.layout().drawing())
                    .maybe_net();
                autorouter
                    .ratsnests()
                    .on_principal_layer(0)
                    .graph()
                    .node_indices()
                    .filter(|vertex| {
                        let weight = autorouter
                            .ratsnests()
                            .on_principal_layer(0)
                            .graph()
                            .node_weight(*vertex)
                            .unwrap();
                        let dot = match weight.node_index() {
                            crate::autorouter::ratsnest::RatvertexNodeIndex::FixedDot(dot) => dot,
                            crate::autorouter::ratsnest::RatvertexNodeIndex::Poly(poly) => {
                                poly.ref_(autorouter.board.layout()).apex()
                            }
                        };
                        dot.primitive_ref(autorouter.board.layout().drawing())
                            .maybe_net()
                            == net
                    })
                    .count()
                    >= 3
            })
            .map(|index| RatlineUid {
                principal_layer: 0,
                index,
            })
            .expect("fixture should have a multi-pin net");
        let before = ratline.ref_(&autorouter).endpoint_indices();

        assert!(reconfigurator.try_respan_ratline(&mut autorouter, ratline));

        let after = ratline.ref_(&autorouter).endpoint_indices();
        assert_ne!(
            before, after,
            "respan should move at least one endpoint to a different vertex"
        );
        assert!(
            autorouter
                .ratsnests()
                .on_principal_layer(0)
                .graph()
                .edge_endpoints(ratline.index)
                .is_some(),
            "the rewired edge keeps its index and stays in the graph"
        );
        assert!(
            !reconfigurator.dead.contains(&ratline),
            "a respanned ratline must not be dead-marked"
        );

        // The exhausted pair is remembered: another respan of the same
        // uid must pick yet another pair, never the original one.
        let second = ratline.ref_(&autorouter).endpoint_indices();
        let second = (second.0.min(second.1), second.0.max(second.1));
        let before = (before.0.min(before.1), before.0.max(before.1));
        assert_ne!(before, second);
    }

    /// A ratline that keeps failing route attempts must not re-permute
    /// forever. Once its cumulative failures reach the deadline it is
    /// declared dead and dropped from every emitted configuration — the
    /// frontier then advances instead of burning the step budget on a
    /// sealed ratline (this loop is unbounded under `--deterministic`,
    /// where the timeout never fires).
    #[test]
    fn sealed_ratline_is_declared_dead_and_command_terminates() {
        let mut autorouter = autorouter_led_matrix();
        let selection = PinSelection::new_select_layer(autorouter.board(), 0);

        // A bounded per-attempt step limit keeps each failure cheap;
        // an unbounded reconfiguration budget lets the loop run. A
        // short failure deadline keeps the respan chain (deadline ×
        // MAX_RESPANS attempts per ratline) within the iteration bound.
        // The fixture's unroutable ratlines must hit the deadline and
        // drop out of the emitted orderings, letting the command end
        // instead of permuting them forever.
        let mut opts = options(Some(500), false);
        opts.max_reconfigurations = None;
        opts.deadline_failures_per_layer = 4;
        let mut reconfigurator = autorouter.planar_autoroute(&selection, opts).unwrap();

        let mut maybe_edit = None;
        for _ in 0..5_000_000 {
            if let ControlFlow::Break(edit) = reconfigurator.step(&mut autorouter).unwrap() {
                maybe_edit = Some(edit);
                break;
            }
        }

        maybe_edit.expect("the command must terminate, not permute forever");
        assert!(
            !reconfigurator.dead.is_empty(),
            "permanently failing ratlines should be declared dead"
        );
        for ratline in &reconfigurator.dead {
            assert!(
                !reconfigurator
                    .stepper
                    .configuration()
                    .ratlines
                    .contains(ratline),
                "a dead ratline must not remain in the live ordering"
            );
        }
    }
}
