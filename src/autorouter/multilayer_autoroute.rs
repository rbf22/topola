// SPDX-FileCopyrightText: 2025 Topola contributors
//
// SPDX-License-Identifier: MIT

use std::{
    collections::{BTreeMap, BTreeSet},
    ops::ControlFlow,
};

use derive_getters::Getters;
use serde::{Deserialize, Serialize};
use specctra_core::mesadata::AccessMesadata;

use crate::{
    autorouter::{
        anterouter::{Anterouter, AnterouterOptions, AnterouterPlan},
        invoker::GetDebugOverlayData,
        planar_autoroute::PlanarAutorouteConfiguration,
        planar_preconfigurer::PlanarAutoroutePreconfigurerInput,
        planar_reconfigurator::{
            PlanarAutorouteReconfigurator, PlanarAutorouteReconfiguratorStatus,
        },
        ratline::RatlineUid,
        ratsnest::RatvertexNodeIndex,
        Autorouter, AutorouterError, PlanarAutorouteOptions,
    },
    board::edit::BoardEdit,
    drawing::{
        dot::FixedDotIndex,
        graph::{GetMaybeNet, MakePrimitiveRef, PrimitiveIndex},
        primitive::MakePrimitiveShape,
    },
    geometry::{edit::Edit, primitive::PrimitiveShape, shape::AccessShape, GetLayer},
    graph::MakeRef,
    layout::via::ViaWeight,
    math::Circle,
    router::{navcord::Navcord, navmesh::Navmesh, thetastar::ThetastarStepper},
    stepper::{
        Abort, EstimateProgress, LinearScale, ReconfiguratorStatus, Reconfigure, Step,
        TimeoutOptions,
    },
};

#[derive(Clone, Debug, Eq, Ord, PartialEq, PartialOrd)]
pub struct MultilayerAutorouteConfiguration {
    pub plan: AnterouterPlan,
    pub planar: PlanarAutoroutePreconfigurerInput,
}

#[derive(Clone, Copy, Debug, Deserialize, Serialize)]
pub struct MultilayerAutorouteOptions {
    pub anterouter: AnterouterOptions,
    pub planar: PlanarAutorouteOptions,
    pub timeout: TimeoutOptions,
    /// Seed for perturbing the initial layer assignment: when nonzero,
    /// each ratline starts on a deterministic pseudo-random layer
    /// instead of its preferred layer, falling back to the preferred
    /// layer when fanout placement fails. Parallel workers pass
    /// different seeds to explore different layer assignments.
    #[serde(default)]
    pub layer_seed: u64,
}

/// Bookkeeping for one mid-route via weave, kept so the weave can be
/// fully undone when either half cannot complete.
struct WeaveRecord {
    continuation: RatlineUid,
    via_vertex: petgraph::stable_graph::NodeIndex<usize>,
    via_dot_source: FixedDotIndex,
    via_dot_target: FixedDotIndex,
    target_layer: usize,
    /// The parent's destination-endpoint remapping before the weave
    /// retargeted it at the via — `None` when it terminated directly
    /// on its endpoint pad.
    parent_destination_entry: Option<FixedDotIndex>,
    destination_dot: FixedDotIndex,
}

#[derive(Getters)]
pub struct MultilayerAutorouteExecutionStepper {
    planar: PlanarAutorouteReconfigurator,
    /// Chronological record of every fanout-related edit, merged into
    /// the command's output. Surgical reconfigurations append each
    /// changed ratline's old-edit reversal and new edit here.
    #[getter(skip)]
    anteroute_edit: BoardEdit,
    /// Per-ratline fanout edits, so re-layering can reverse only the
    /// changed ratlines' vias instead of the whole anteroute pass.
    #[getter(skip)]
    anteroute_edits: BTreeMap<RatlineUid, BoardEdit>,
    /// The layer plan the current vias were placed for.
    #[getter(skip)]
    plan: AnterouterPlan,
    /// Ratlines involved in a mid-route via weave, as either half.
    /// They are anchored to their layers: rotating one would reverse
    /// the weave via or rewrite the terminating-dot link that chains
    /// the halves, dangling a board reference either way.
    #[getter(skip)]
    woven: BTreeSet<RatlineUid>,
    /// Bookkeeping for each live weave, keyed by the parent ratline,
    /// so a weave whose halves cannot complete can be fully undone.
    #[getter(skip)]
    weaves: BTreeMap<RatlineUid, WeaveRecord>,
    /// Parents whose weave was undone: the via site sampling is
    /// deterministic, so a second weave would place the same via and
    /// fail identically — they go straight to layer rotation.
    #[getter(skip)]
    weave_exhausted: BTreeSet<RatlineUid>,
    #[getter(skip)]
    options: MultilayerAutorouteOptions,
}

impl MultilayerAutorouteExecutionStepper {
    pub fn new(
        autorouter: &mut Autorouter<impl AccessMesadata>,
        configuration: MultilayerAutorouteConfiguration,
        options: MultilayerAutorouteOptions,
    ) -> Result<Self, AutorouterError> {
        Self::new_with_planar_override(autorouter, configuration, options, |_, _| {})
    }

    /// Like [`MultilayerAutorouteExecutionStepper::new`], but
    /// `planar_override` runs on the inner planar reconfigurator's
    /// freshly built preconfiguration — after anterouting, so the
    /// terminating-dot map already names each ratline's fanout dots.
    /// The critical-first outer stepper uses it to inject its
    /// ordering, carve corridors on the assigned layers, and seed the
    /// protected set.
    pub fn new_with_planar_override<M: AccessMesadata>(
        autorouter: &mut Autorouter<M>,
        configuration: MultilayerAutorouteConfiguration,
        options: MultilayerAutorouteOptions,
        planar_override: impl FnOnce(&mut Autorouter<M>, &mut PlanarAutorouteConfiguration),
    ) -> Result<Self, AutorouterError> {
        let mut anterouter = Anterouter::new(configuration.plan.clone());
        let mut anteroute_edit = BoardEdit::new();
        let mut anteroute_edits = BTreeMap::new();
        let mut terminating_dot_map = BTreeMap::new();

        let mut unplaceable = BTreeSet::new();
        for (ratline, layer) in configuration.plan.layer_map.clone() {
            let mut ratline_edit = BoardEdit::new();
            // `anteroute_ratline` writes terminating-dot entries during
            // placement, so a failed attempt can leave entries pointing
            // at vias the reversal just removed — restore the map along
            // with the board edit.
            let entries_before = terminating_dot_map.clone();
            let placed = anterouter
                .anteroute_ratline(
                    autorouter,
                    &mut ratline_edit,
                    ratline,
                    layer,
                    &options.anterouter,
                    &mut terminating_dot_map,
                )
                .map_err(|_| {
                    // Seeded layer assignments may ask for a layer the
                    // pads cannot reach. Reverse any partial placement and
                    // fall back to the preferred layer.
                    autorouter.board.apply_edit(&ratline_edit.reverse());
                    anteroute_edit.merge(ratline_edit.clone());
                    anteroute_edit.merge(ratline_edit.reverse());
                    ratline_edit = BoardEdit::new();
                    terminating_dot_map = entries_before.clone();
                    anterouter.anteroute_ratline(
                        autorouter,
                        &mut ratline_edit,
                        ratline,
                        ratline.ref_(autorouter).preferred_layer(),
                        &options.anterouter,
                        &mut terminating_dot_map,
                    )
                });
            if matches!(placed, Err(Err(()))) {
                // No room for a fanout via on either layer — drop the
                // ratline from this configuration's work list instead of
                // panicking the whole run.
                autorouter.board.apply_edit(&ratline_edit.reverse());
                anteroute_edit.merge(ratline_edit.clone());
                anteroute_edit.merge(ratline_edit.reverse());
                terminating_dot_map = entries_before;
                unplaceable.insert(ratline);
                continue;
            }
            anteroute_edit.merge(ratline_edit.clone());
            anteroute_edits.insert(ratline, ratline_edit);
        }

        Ok(Self {
            planar: PlanarAutorouteReconfigurator::new_with_configuration_override(
                autorouter,
                PlanarAutoroutePreconfigurerInput {
                    terminating_dot_map,
                    ratlines: configuration
                        .planar
                        .ratlines
                        .iter()
                        .copied()
                        .filter(|ratline| !unplaceable.contains(ratline))
                        .collect(),
                },
                options.planar,
                planar_override,
            )?,
            anteroute_edit,
            anteroute_edits,
            plan: configuration.plan,
            woven: BTreeSet::new(),
            weaves: BTreeMap::new(),
            weave_exhausted: BTreeSet::new(),
            options: options.clone(),
        })
    }

    /// Mutable access to the inner planar reconfigurator — for the
    /// critical-first outer stepper's surgical windows and protected
    /// bookkeeping.
    pub fn planar_mut(&mut self) -> &mut PlanarAutorouteReconfigurator {
        &mut self.planar
    }

    /// The layer assignments the currently placed fanout vias were
    /// made for. Critical-first diffs this between re-identification
    /// rounds to find ratlines whose carves went stale on rotation.
    pub fn layer_map(&self) -> &BTreeMap<RatlineUid, usize> {
        &self.plan.layer_map
    }

    /// Finishes routing early, keeping already routed bands. See
    /// [`PlanarAutorouteReconfigurator::finish_early`].
    pub fn finish_early(&mut self, autorouter: &mut Autorouter<impl AccessMesadata>) {
        self.planar.finish_early(autorouter);
    }

    /// Decomposes `ratline` at a mid-route via: places a via at a
    /// sampled position between the ratline's current terminating
    /// dots, retargets the ratline's destination endpoint at the via's
    /// dot on its current layer, and enqueues a continuation ratline
    /// running from the via's dot on another layer to the original
    /// destination (fanned out to that layer like any other endpoint).
    ///
    /// This is the mid-route layer hop: a connection whose corridor is
    /// blocked on every layer it is tried on can still complete by
    /// ducking to a freer layer partway. Returns `true` when a via was
    /// placed and the continuation was enqueued.
    pub fn weave_ratline(
        &mut self,
        autorouter: &mut Autorouter<impl AccessMesadata>,
        ratline: RatlineUid,
    ) -> Result<bool, AutorouterError> {
        if self.woven.contains(&ratline) || self.weave_exhausted.contains(&ratline) {
            return Ok(false);
        }

        let source_layer = self.plan.layer_map[&ratline];
        let layer_count = autorouter.board().layout().drawing().layer_count();
        if layer_count < 2 {
            return Ok(false);
        }

        let Some(index) = self
            .planar
            .configuration()
            .ratlines
            .iter()
            .position(|uid| *uid == ratline)
        else {
            return Ok(false);
        };

        let (term_source, term_destination) = self
            .planar
            .configuration()
            .ratline_terminating_dots(autorouter, index);
        let pos_of = |dot: FixedDotIndex| {
            dot.primitive_ref(autorouter.board().layout().drawing())
                .shape()
                .center()
        };
        let pos_source = pos_of(term_source);
        let pos_destination = pos_of(term_destination);

        // Via sites hug the source end of the failed corridor: the
        // parent's first half must stay short to survive the
        // congestion that stalled it, while the continuation takes
        // the long hop on the freer layer.
        let delta = pos_destination - pos_source;
        let span = delta.x().hypot(delta.y()).max(1.0);
        let perp = geo::Point::new(-delta.y(), delta.x()) * (150.0 / span);
        let candidates = [
            pos_source + delta * (1.0 / 8.0),
            pos_source + delta * (1.0 / 4.0),
            pos_source + delta * (1.0 / 8.0) + perp,
            pos_source + delta * (1.0 / 8.0) - perp,
            pos_source + delta * (3.0 / 8.0),
            pos_source + delta * (1.0 / 4.0) + perp,
            pos_source + delta * (1.0 / 4.0) - perp,
            pos_source + delta * 0.5,
        ];

        let maybe_net = term_source
            .primitive_ref(autorouter.board().layout().drawing())
            .maybe_net();

        let mut weave_edit = BoardEdit::new();
        let mut placed_via = None;
        'sites: for pos in candidates {
            for offset in 1..layer_count {
                let target_layer = (source_layer + offset) % layer_count;
                if let Ok((_via, dots)) = autorouter.board.add_via(
                    &mut weave_edit,
                    ViaWeight {
                        from_layer: source_layer.min(target_layer),
                        to_layer: source_layer.max(target_layer),
                        circle: Circle {
                            pos,
                            r: autorouter.board().mesadata().default_via_radius(),
                        },
                        maybe_net,
                    },
                    None,
                ) {
                    placed_via = Some((pos, dots, target_layer));
                    break 'sites;
                }
            }
        }
        let Some((via_pos, via_dots, target_layer)) = placed_via else {
            // The failed attempts each recorded their own reversal;
            // keep the edit so undo history stays faithful.
            self.anteroute_edit.merge(weave_edit);
            return Ok(false);
        };

        let dot_on_layer = |layer: usize| {
            via_dots.iter().copied().find(|dot| {
                dot.primitive_ref(autorouter.board().layout().drawing())
                    .layer()
                    == layer
            })
        };
        let via_dot_source =
            dot_on_layer(source_layer).expect("via spans the source layer by construction");
        let via_dot_target =
            dot_on_layer(target_layer).expect("via spans the target layer by construction");

        let destination_vertex = ratline.ref_(autorouter).endpoint_indices().1;
        let destination_dot = ratline.ref_(autorouter).endpoint_dots().1;

        // The continuation ratline: via's far-side dot → the original
        // destination. Only the parent's ratsnest holds the edge —
        // ratline lookups are per-principal-layer anyway.
        let (via_vertex, continuation_edge) = autorouter
            .ratsnests
            .on_principal_layer_mut(ratline.principal_layer)
            .add_continuation(
                RatvertexNodeIndex::FixedDot(via_dot_target),
                via_pos,
                destination_vertex,
            );
        let continuation = RatlineUid {
            principal_layer: ratline.principal_layer,
            index: continuation_edge,
        };

        self.plan.layer_map.insert(continuation, target_layer);
        // The via-side endpoint terminates on itself — only the
        // destination pad gets a fanout via to the target layer.
        self.plan
            .static_terminating_dot_map
            .insert((continuation, via_dot_target, target_layer), via_dot_target);

        let mut anterouter = Anterouter::new(self.plan.clone());
        let mut new_entries = BTreeMap::new();
        if anterouter
            .anteroute_ratline(
                autorouter,
                &mut weave_edit,
                continuation,
                target_layer,
                &self.options.anterouter,
                &mut new_entries,
            )
            .is_err()
        {
            autorouter.board.apply_edit(&weave_edit.reverse());
            self.anteroute_edit.merge(weave_edit.clone());
            self.anteroute_edit.merge(weave_edit.reverse());
            autorouter
                .ratsnests
                .on_principal_layer_mut(ratline.principal_layer)
                .remove_continuation(via_vertex, continuation_edge);
            self.plan.layer_map.remove(&continuation);
            self.plan.static_terminating_dot_map.remove(&(
                continuation,
                via_dot_target,
                target_layer,
            ));
            return Ok(false);
        }

        // Commit the weave edit under the continuation: it owns the
        // via and its destination fanout, so a later reversal of the
        // parent's fanout never touches them.
        // The parent's destination endpoint now terminates at the
        // via's source-layer dot; the continuation's entries carry its
        // destination fanout. Its prior remapping is kept so unweaving
        // restores exactly what was there.
        let parent_destination_entry = self
            .planar
            .configuration()
            .terminating_dot_map
            .get(&(ratline, destination_dot))
            .copied();
        new_entries.insert((ratline, destination_dot), via_dot_source);
        self.planar.remap_terminating_dots(&new_entries);

        self.anteroute_edit.merge(weave_edit.clone());
        self.anteroute_edits.insert(continuation, weave_edit);
        self.woven.insert(ratline);
        self.woven.insert(continuation);
        self.weaves.insert(
            ratline,
            WeaveRecord {
                continuation,
                via_vertex,
                via_dot_source,
                via_dot_target,
                target_layer,
                parent_destination_entry,
                destination_dot,
            },
        );
        // The continuation must survive orderings emitted by the
        // reconfigurers' stored preconfigurations, not just the live
        // list spliced below.
        self.planar.insert_ratline_after(continuation, ratline);

        // Restart at the parent with the continuation queued right
        // after it, so the second half is routed while the first
        // half's corridor is still warm. No bands are ripped: the
        // parent's last attempt failed and nothing else moved.
        let mut order = self.planar.configuration().ratlines.clone();
        order.insert(index + 1, continuation);
        self.planar
            .begin_surgical_reconfiguration(autorouter, &[])?;
        self.planar.commit_surgical_reconfiguration(
            autorouter,
            PlanarAutorouteConfiguration {
                ratlines: order,
                rip_up: vec![],
                ..self.planar.configuration().clone()
            },
        )?;

        log::debug!(
            "via weave: ratline {:?} split at ({:.0}, {:.0}), \
             continuation on layer {}",
            ratline.index,
            via_pos.x(),
            via_pos.y(),
            target_layer,
        );
        log::debug!(
            "via weave: continuation {:?}, order {:?}, resume at {}",
            continuation,
            self.planar
                .configuration()
                .ratlines
                .iter()
                .map(|r| r.index)
                .collect::<Vec<_>>(),
            self.planar.curr_ratline_index(),
        );
        Ok(true)
    }

    /// Installs `plan`, then re-injects the weave-registered layer
    /// assignments and static terminating dots. Popped search
    /// configurations carry plan snapshots that predate any weaves —
    /// adopting one wholesale would strand continuations without a
    /// layer or terminating dot.
    fn adopt_plan_preserving_woven(&mut self, plan: AnterouterPlan) {
        let old_plan = std::mem::replace(&mut self.plan, plan);
        for ratline in &self.woven {
            if let Some(layer) = old_plan.layer_map.get(ratline) {
                self.plan.layer_map.insert(*ratline, *layer);
            }
            for (key, dot) in old_plan.static_terminating_dot_map.iter() {
                if key.0 == *ratline {
                    self.plan.static_terminating_dot_map.insert(*key, *dot);
                }
            }
        }
    }

    /// Returns the weave parent of `ratline`: itself when it is a
    /// woven parent, or its parent when it is a weave continuation.
    pub fn weave_parent_of(&self, ratline: RatlineUid) -> Option<RatlineUid> {
        if self.weaves.contains_key(&ratline) {
            return Some(ratline);
        }
        self.weaves
            .iter()
            .find(|(_, record)| record.continuation == ratline)
            .map(|(parent, _)| *parent)
    }

    /// Weave parents whose chain is broken — either half's band is
    /// absent. Checked at escalation time: a weave's re-attempt
    /// completes before the next escalation, so an absent band means
    /// that half genuinely failed rather than still being in flight.
    pub fn broken_weaves(&self, autorouter: &Autorouter<impl AccessMesadata>) -> Vec<RatlineUid> {
        let terminating = |ratline: RatlineUid, endpoint: FixedDotIndex| {
            self.planar
                .configuration()
                .terminating_dot_map
                .get(&(ratline, endpoint))
                .copied()
                .unwrap_or(endpoint)
        };
        self.weaves
            .iter()
            .filter(|(parent, record)| {
                let (parent_origin, _) = parent.ref_(autorouter).endpoint_dots();
                let (_, child_destination) = record.continuation.ref_(autorouter).endpoint_dots();
                let parent_half = (terminating(**parent, parent_origin), record.via_dot_source);
                let child_half = (
                    record.via_dot_target,
                    terminating(record.continuation, child_destination),
                );
                autorouter
                    .board
                    .band_between_nodes(parent_half.0, parent_half.1)
                    .is_none()
                    || autorouter
                        .board
                        .band_between_nodes(child_half.0, child_half.1)
                        .is_none()
            })
            .map(|(parent, _)| *parent)
            .collect()
    }

    /// Undoes a weave whose halves could not complete: rips whichever
    /// half-bands exist, removes the via and continuation edge, and
    /// restores the parent's pre-weave terminating-dot remap so the
    /// ratline regains its rotation freedom.
    pub fn unweave_ratline(
        &mut self,
        autorouter: &mut Autorouter<impl AccessMesadata>,
        parent: RatlineUid,
    ) -> Result<(), AutorouterError> {
        let Some(record) = self.weaves.remove(&parent) else {
            return Ok(());
        };
        let child = record.continuation;

        // Rip whichever half-bands exist and fold their edit slots
        // into the pending edit. The parent's band, if it routed,
        // terminates at the via dot that is about to be removed.
        self.planar
            .begin_surgical_reconfiguration(autorouter, &[parent, child])?;

        // The weave's via and the continuation's fanout stay on the
        // board as dead-end same-net copper. Other bands may have
        // bent around the via dots, holding them as bend cores —
        // removing the dots would orphan those bends.
        self.anteroute_edits.remove(&child);

        autorouter
            .ratsnests
            .on_principal_layer_mut(parent.principal_layer)
            .remove_continuation(record.via_vertex, child.index);
        self.plan.layer_map.remove(&child);
        self.plan.static_terminating_dot_map.remove(&(
            child,
            record.via_dot_target,
            record.target_layer,
        ));
        self.woven.remove(&parent);
        self.woven.remove(&child);
        // The site sampling is deterministic — a second weave of this
        // ratline would place the same via and fail identically.
        self.weave_exhausted.insert(parent);

        // Stored preconfigurations drop the continuation and its
        // via-dot remaps so emitted orderings never resurrect them.
        self.planar.remove_ratline(child);
        self.planar.remove_terminating_dots(&[child]);
        match record.parent_destination_entry {
            Some(dot) => self.planar.remap_terminating_dots(
                &[((parent, record.destination_dot), dot)]
                    .into_iter()
                    .collect(),
            ),
            None => self
                .planar
                .remove_terminating_dot_entries(&[(parent, record.destination_dot)]),
        }

        // The live configuration carries the same revert: the
        // continuation leaves the order and the parent's destination
        // endpoint returns to its pre-weave terminating dot.
        let mut new_configuration = PlanarAutorouteConfiguration {
            ratlines: self
                .planar
                .configuration()
                .ratlines
                .iter()
                .copied()
                .filter(|uid| *uid != child)
                .collect(),
            rip_up: vec![],
            ..self.planar.configuration().clone()
        };
        new_configuration
            .terminating_dot_map
            .retain(|(ratline, _), _| *ratline != child);
        match record.parent_destination_entry {
            Some(dot) => {
                new_configuration
                    .terminating_dot_map
                    .insert((parent, record.destination_dot), dot);
            }
            None => {
                new_configuration
                    .terminating_dot_map
                    .remove(&(parent, record.destination_dot));
            }
        }
        self.planar
            .commit_surgical_reconfiguration(autorouter, new_configuration)?;

        log::debug!("unweave: ratline {:?} restored", parent.index);
        Ok(())
    }
}

impl<M: AccessMesadata> Step<Autorouter<M>, Option<BoardEdit>, PlanarAutorouteReconfiguratorStatus>
    for MultilayerAutorouteExecutionStepper
{
    type Error = AutorouterError;

    fn step(
        &mut self,
        autorouter: &mut Autorouter<M>,
    ) -> Result<ControlFlow<Option<BoardEdit>, PlanarAutorouteReconfiguratorStatus>, AutorouterError>
    {
        match self.planar.step(autorouter) {
            Ok(ControlFlow::Break(Some(edit))) => {
                self.anteroute_edit.merge(edit);
                // FIXME: Unnecessary large clone.
                Ok(ControlFlow::Break(Some(self.anteroute_edit.clone())))
            }
            Ok(ControlFlow::Continue(ReconfiguratorStatus::Reconfigured(result))) => Ok(
                ControlFlow::Continue(ReconfiguratorStatus::Reconfigured(result)),
            ),
            x => x,
        }
    }
}

impl<M: AccessMesadata> Abort<Autorouter<M>> for MultilayerAutorouteExecutionStepper {
    fn abort(&mut self, autorouter: &mut Autorouter<M>) {
        self.planar.abort(autorouter)
    }
}

impl<M: AccessMesadata> Reconfigure<Autorouter<M>> for MultilayerAutorouteExecutionStepper {
    type Configuration = MultilayerAutorouteConfiguration;
    type Output = Result<(), AutorouterError>;

    fn reconfigure(
        &mut self,
        autorouter: &mut Autorouter<M>,
        new_configuration: MultilayerAutorouteConfiguration,
    ) -> Result<(), AutorouterError> {
        // Surgical re-layering: only the ratlines whose layer
        // assignment changed need their bands ripped and their fanout
        // vias swapped; every other routed band survives.
        // Woven ratlines are anchored: rotating one would either
        // reverse the weave via (dangling the continuation's endpoint)
        // or re-anteroute the parent's destination endpoint (severing
        // the terminating-dot link into the via). They keep their
        // layers; the recorded plan is restored below.
        let (anchored, changed): (Vec<RatlineUid>, Vec<RatlineUid>) = new_configuration
            .plan
            .layer_map
            .iter()
            .filter(|(ratline, layer)| self.plan.layer_map.get(*ratline) != Some(*layer))
            .map(|(ratline, _)| *ratline)
            .partition(|ratline| self.woven.contains(ratline));

        if changed.is_empty() {
            self.adopt_plan_preserving_woven(new_configuration.plan);
            return Ok(());
        }
        crate::metrics::incr(&crate::metrics::METRICS.multilayer_surgical_reconfigures);

        // Ratlines already attempted hold bands that must be ripped and
        // requeued; the rest pick up their new layer when reached.
        let curr = self.planar.curr_ratline_index();
        let attempted: BTreeSet<RatlineUid> = self.planar.configuration().ratlines
            [..curr.min(self.planar.configuration().ratlines.len())]
            .iter()
            .copied()
            .collect();
        let routed_changed: Vec<RatlineUid> = changed
            .iter()
            .copied()
            .filter(|ratline| attempted.contains(ratline))
            .collect();

        // Phase 1: rip the changed ratlines' bands — recorded inside
        // the planar edit stream, resolved under the live
        // terminating-dot map — freeing the fanout via dots. Always
        // run, even with an empty rip-up list, so the in-flight route
        // is dissolved and cannot materialize bends around a via that
        // phase 2 is about to remove.
        self.planar
            .begin_surgical_reconfiguration(autorouter, &routed_changed)?;

        // Phase 2: swap each changed ratline's fanout vias.
        let mut anterouter = Anterouter::new(new_configuration.plan.clone());
        let mut new_entries = BTreeMap::new();
        let mut layer_fixes = vec![];
        // Anchored (woven) ratlines keep their old layers — restored
        // into the new plan by the layer_fixes pass below.
        for ratline in &anchored {
            layer_fixes.push((*ratline, self.plan.layer_map[ratline]));
        }
        // Ratlines whose old fanout vias are removed below. Their
        // terminating-dot map entries must be purged: a retained
        // `FixedDotIndex` points at a freed node slot that a later
        // `add_via` may recycle as a compound — dereferencing it
        // panics in `Geometry::primitive_weight`.
        let mut relayered = vec![];
        let mut wrapped_count = 0usize;
        for ratline in &changed {
            // The gears the old edit created — and that its reversal
            // therefore removes — must be free of foreign band
            // attachments before removal: a band may bend around a via
            // dot (a wrap) or terminate at it through a seg joint (a
            // limb — lone termsegs wrap nothing). Limbs the same edit
            // itself created, like its fanout seg, don't count.
            // Enumerating the edit's own creations also covers vias the
            // terminating-dot map does not reference.
            let wrapped = self.anteroute_edits.get(ratline).is_some_and(|old_edit| {
                crate::autorouter::carve::edit_is_wrapped(
                    autorouter.board.layout().drawing(),
                    &old_edit.layout_edit,
                )
            });

            if wrapped {
                // Keep the old fanout and the old layer: its gears are
                // load-bearing for another band.
                wrapped_count += 1;
                layer_fixes.push((*ratline, self.plan.layer_map[ratline]));
                continue;
            }

            relayered.push(*ratline);

            if let Some(old_edit) = self.anteroute_edits.remove(ratline) {
                autorouter.board.apply_edit(&old_edit.reverse());
                self.anteroute_edit.merge(old_edit.clone());
                self.anteroute_edit.merge(old_edit.reverse());
            }

            let mut ratline_edit = BoardEdit::new();
            let entries_before = new_entries.clone();
            let placed = anterouter.anteroute_ratline(
                autorouter,
                &mut ratline_edit,
                *ratline,
                new_configuration.plan.layer_map[ratline],
                &self.options.anterouter,
                &mut new_entries,
            );

            if placed.is_err() {
                // Re-fanout to the new layer found no room — the board
                // is now crowded where the initial pass worked. A
                // partially placed fanout must be reversed before
                // restoring: an orphan via would remain in this
                // ratline's recorded edit unmapped, and a later swap
                // could remove it while a foreign band wraps it.
                autorouter.board.apply_edit(&ratline_edit.reverse());
                self.anteroute_edit.merge(ratline_edit.clone());
                self.anteroute_edit.merge(ratline_edit.reverse());
                ratline_edit = BoardEdit::new();
                new_entries = entries_before;

                // Restore the old layer's fanout so the ratline keeps
                // terminating dots it can route to. Reversed likewise
                // on partial failure.
                let old_layer = self.plan.layer_map[ratline];
                let entries_before_restore = new_entries.clone();
                if anterouter
                    .anteroute_ratline(
                        autorouter,
                        &mut ratline_edit,
                        *ratline,
                        old_layer,
                        &self.options.anterouter,
                        &mut new_entries,
                    )
                    .is_err()
                {
                    autorouter.board.apply_edit(&ratline_edit.reverse());
                    self.anteroute_edit.merge(ratline_edit.clone());
                    self.anteroute_edit.merge(ratline_edit.reverse());
                    ratline_edit = BoardEdit::new();
                    new_entries = entries_before_restore;
                }
                layer_fixes.push((*ratline, old_layer));
            }

            self.anteroute_edit.merge(ratline_edit.clone());
            self.anteroute_edits.insert(*ratline, ratline_edit);
        }

        // Phase 3: drop the dead dots' remappings, then point live and
        // future configurations at the new terminating dots, then
        // requeue the ripped ratlines right after the currently
        // attempted one.
        self.planar.remove_terminating_dots(&relayered);
        self.planar.remap_terminating_dots(&new_entries);

        {
            let ripped: BTreeSet<RatlineUid> = routed_changed.iter().copied().collect();
            let mut order: Vec<RatlineUid> = self
                .planar
                .configuration()
                .ratlines
                .iter()
                .copied()
                .filter(|ratline| !ripped.contains(ratline))
                .collect();
            let insert_at = self
                .planar
                .configuration()
                .ratlines
                .get(curr)
                .and_then(|attempted| {
                    order
                        .iter()
                        .position(|ratline| *ratline == *attempted)
                        .map(|index| index + 1)
                })
                .unwrap_or(order.len());
            order.splice(insert_at..insert_at, routed_changed.iter().copied());

            self.planar.commit_surgical_reconfiguration(
                autorouter,
                PlanarAutorouteConfiguration {
                    ratlines: order,
                    rip_up: routed_changed,
                    ..self.planar.configuration().clone()
                },
            )?;
        }

        self.adopt_plan_preserving_woven(new_configuration.plan);
        let refanout_failures = layer_fixes.len() - wrapped_count;
        // Ratlines whose re-fanout failed keep their old layer, so the
        // recorded plan matches the vias actually on the board.
        for (ratline, layer) in layer_fixes {
            self.plan.layer_map.insert(ratline, layer);
        }
        log::debug!(
            "multilayer reconfigure: {} changed, {} relayered, {} kept wrapped, {} re-fanout failed",
            changed.len(),
            relayered.len(),
            wrapped_count,
            refanout_failures
        );
        Ok(())
    }
}

impl EstimateProgress for MultilayerAutorouteExecutionStepper {
    type Value = usize;
    type Subscale = LinearScale<f64>;

    fn estimate_progress(&self) -> LinearScale<usize, LinearScale<f64>> {
        self.planar.estimate_progress()
    }
}

impl GetDebugOverlayData for MultilayerAutorouteExecutionStepper {
    fn maybe_thetastar(&self) -> Option<&ThetastarStepper<Navmesh, f64>> {
        self.planar.maybe_thetastar()
    }

    fn maybe_navcord(&self) -> Option<&Navcord> {
        self.planar.maybe_navcord()
    }

    fn ghosts(&self) -> &[PrimitiveShape] {
        self.planar.ghosts()
    }

    fn obstacles(&self) -> &[PrimitiveIndex] {
        self.planar.obstacles()
    }
}

#[cfg(test)]
mod tests {
    use std::{collections::BTreeSet, fs::File, io::BufReader};

    use specctra_core::mesadata::SpecctraMesadata;

    use super::*;
    use crate::{
        autorouter::{
            multilayer_preconfigurer::{
                MultilayerAutoroutePreconfigurerInput, MultilayerPreconfigurer,
            },
            PresortBy, ReconfigureBy,
        },
        drawing::graph::MakePrimitiveRef,
        geometry::GetLayer,
        router::RouterOptions,
        specctra::design::SpecctraDesign,
    };

    fn astable() -> Autorouter<SpecctraMesadata> {
        let design_file =
            File::open("tests/multilayer/astable_multivibrator/astable_multivibrator.dsn").unwrap();
        let design = SpecctraDesign::load(BufReader::new(design_file)).unwrap();
        Autorouter::new(design.make_board(&mut BoardEdit::new())).unwrap()
    }

    /// Moving a ratline to another layer removes its old fanout vias
    /// and places new ones. Every `terminating_dot_map` value must
    /// afterwards resolve to a live dot on the layer the stepper's
    /// recorded plan claims — before the purge fix a dead
    /// `FixedDotIndex` survived in the map until a later `add_via`
    /// recycled its node slot into a compound and `primitive_weight`
    /// panicked.
    #[test]
    fn reconfigure_refreshes_terminating_dot_map() {
        let mut autorouter = astable();
        let layer_count = autorouter.board().layout().drawing().layer_count();

        let ratlines: BTreeSet<RatlineUid> = (0..layer_count)
            .flat_map(|layer| {
                autorouter
                    .ratsnests()
                    .on_principal_layer(layer)
                    .graph()
                    .edge_indices()
                    .take(2)
                    .map(move |index| RatlineUid {
                        principal_layer: layer,
                        index,
                    })
            })
            .take(3)
            .collect();
        let rotated = *ratlines.iter().next().unwrap();

        let options = MultilayerAutorouteOptions {
            anterouter: AnterouterOptions {
                fanout_clearance: 200.0,
            },
            planar: PlanarAutorouteOptions {
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
            timeout: TimeoutOptions {
                initial: 10.0,
                progress_bonus: 0.0,
            },
            layer_seed: 0,
        };

        let plan = MultilayerPreconfigurer::new(
            &autorouter,
            MultilayerAutoroutePreconfigurerInput {
                ratlines: ratlines.clone(),
            },
        )
        .plan()
        .clone();

        let make_configuration = |plan: &AnterouterPlan| MultilayerAutorouteConfiguration {
            plan: plan.clone(),
            planar: PlanarAutoroutePreconfigurerInput {
                ratlines: ratlines.clone(),
                terminating_dot_map: BTreeMap::new(),
            },
        };

        let mut stepper = MultilayerAutorouteExecutionStepper::new(
            &mut autorouter,
            make_configuration(&plan),
            options,
        )
        .unwrap();

        let assert_map_consistent =
            |stepper: &MultilayerAutorouteExecutionStepper,
             autorouter: &Autorouter<SpecctraMesadata>| {
                let drawing = autorouter.board().layout().drawing();
                for ((ratline, _), dot) in &stepper.planar().configuration().terminating_dot_map {
                    // A stale index panics here once its freed slot has
                    // been recycled into another node variant.
                    let dot_ref = dot.primitive_ref(drawing);
                    assert_eq!(dot_ref.layer(), stepper.plan.layer_map[ratline]);
                }
            };
        assert_map_consistent(&stepper, &autorouter);

        // Cycle the ratline across layers twice: each reconfigure
        // removes the previous fanout via and installs fresh
        // terminating dots, so both the purge and the remap run.
        let mut new_plan = plan;
        for _ in 0..2 {
            *new_plan.layer_map.get_mut(&rotated).unwrap() =
                (new_plan.layer_map[&rotated] + 1) % layer_count;
            stepper
                .reconfigure(&mut autorouter, make_configuration(&new_plan))
                .unwrap();
            assert_map_consistent(&stepper, &autorouter);
            // The swap must have actually run: a relayered ratline
            // without fanout dots would mean the test exercised
            // nothing.
            assert!(
                stepper
                    .planar()
                    .configuration()
                    .terminating_dot_map
                    .keys()
                    .any(|(ratline, _)| *ratline == rotated),
                "rotated ratline has no terminating-dot map entries"
            );
        }
    }
}
