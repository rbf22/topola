// SPDX-FileCopyrightText: 2025 Topola contributors
//
// SPDX-License-Identifier: MIT

use std::{
    cmp::Ordering,
    collections::{BTreeMap, BTreeSet},
    iter::{Skip, Take},
};

use derive_getters::Getters;
use enum_dispatch::enum_dispatch;
use itertools::{Itertools, Permutations};
use petgraph::visit::Walker;
use specctra_core::mesadata::AccessMesadata;

use crate::{
    astar::Astar,
    autorouter::{
        invoker::GetDebugOverlayData,
        planar_autoroute::{PlanarAutorouteConfiguration, PlanarAutorouteExecutionStepper},
        planar_preconfigurer::SccIntersectionsAndLengthRatlinePlanarAutoroutePreconfigurer,
        ratline::RatlineUid,
        scc::Scc,
        Autorouter, PlanarAutorouteOptions, ReconfigureBy,
    },
    drawing::{
        gear::WalkOutwards,
        graph::{GetMaybeNet, MakePrimitiveRef, PrimitiveIndex},
        loose::{GetPrevNextLoose, LooseIndex},
    },
    graph::MakeRef,
};

#[enum_dispatch]
pub trait MakeNextPlanarAutorouteConfiguration {
    fn next_configuration(
        &mut self,
        autorouter: &Autorouter<impl AccessMesadata>,
        stepper: &PlanarAutorouteExecutionStepper,
    ) -> Option<PlanarAutorouteConfiguration>;
}

#[enum_dispatch(MakeNextPlanarAutorouteConfiguration)]
pub enum PlanarAutorouteReconfigurer {
    //RatlineCuts(RatlineCutsPlanarAutorouteReconfigurer),
    SccPermutations(SccPermutationsPlanarAutorouteReconfigurer),
    NegotiatedCongestion(NegotiatedCongestionPlanarAutorouteReconfigurer),
}

impl PlanarAutorouteReconfigurer {
    pub fn new(
        autorouter: &mut Autorouter<impl AccessMesadata>,
        preconfiguration: PlanarAutorouteConfiguration,
        presorter: SccIntersectionsAndLengthRatlinePlanarAutoroutePreconfigurer,
        options: &PlanarAutorouteOptions,
    ) -> Self {
        match options.reconfigure_by {
            ReconfigureBy::SccPermutations => PlanarAutorouteReconfigurer::SccPermutations(
                SccPermutationsPlanarAutorouteReconfigurer::new(
                    autorouter,
                    preconfiguration,
                    presorter,
                    options,
                ),
            ),
            ReconfigureBy::NegotiatedCongestion => {
                PlanarAutorouteReconfigurer::NegotiatedCongestion(
                    NegotiatedCongestionPlanarAutorouteReconfigurer::new(
                        autorouter,
                        preconfiguration,
                        presorter,
                        options,
                    ),
                )
            }
        }
    }
}

#[derive(Clone, Debug, Getters)]
struct SccSearchNode {
    permutation: Vec<Scc>,
    #[getter(skip)]
    permutations_iter: Skip<Permutations<Take<std::vec::IntoIter<Scc>>>>,
    #[getter(skip)]
    length: usize,
}

impl Ord for SccSearchNode {
    fn cmp(&self, other: &Self) -> Ordering {
        self.permutation
            .cmp(&other.permutation)
            .then(self.length.cmp(&other.length))
    }
}

impl PartialOrd for SccSearchNode {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl Eq for SccSearchNode {}

impl PartialEq for SccSearchNode {
    fn eq(&self, other: &Self) -> bool {
        self.permutation == other.permutation && self.length == other.length
    }
}

impl SccSearchNode {
    pub fn new(sccs: Vec<Scc>) -> Self {
        let len = sccs.len();

        Self {
            permutation: sccs.clone(),
            permutations_iter: sccs.into_iter().take(len).permutations(0).skip(0),
            length: 0,
        }
    }

    pub fn expand(&self, length: usize) -> Vec<(f64, f64, Self)> {
        let mut expanded_nodes = vec![];

        if let Some(resized) = self.clone().resize(length) {
            if let Some((permuted_resized, changed_count)) = resized.permute() {
                expanded_nodes.push((
                    changed_count as f64 / 100.0,
                    (self.permutation.len() - permuted_resized.length) as f64,
                    permuted_resized,
                ));
            }
        }

        if let Some((permuted, changed_count)) = self.clone().permute() {
            expanded_nodes.push((
                changed_count as f64 / 100.0,
                (self.permutation.len() - permuted.length) as f64,
                permuted,
            ));
        }

        expanded_nodes
    }

    fn resize(self, length: usize) -> Option<Self> {
        if length == self.length {
            return None;
        }

        Some(Self {
            permutation: self.permutation.clone(),
            permutations_iter: self
                .permutation
                .into_iter()
                .take(length)
                .permutations(length)
                .skip(1),
            length,
        })
    }

    fn permute(mut self) -> Option<(Self, usize)> {
        let mut changed_count = 0;

        for (i, element) in self.permutations_iter.next()?.iter().enumerate() {
            if self.permutation[i] != *element {
                self.permutation[i] = element.clone();
                changed_count += 1;
            }
        }

        Some((self, changed_count))
    }
}

pub struct SccPermutationsPlanarAutorouteReconfigurer {
    configuration_search: Astar<SccSearchNode, f64>,
    preconfiguration: PlanarAutorouteConfiguration,
}

impl SccPermutationsPlanarAutorouteReconfigurer {
    pub fn new(
        _autorouter: &mut Autorouter<impl AccessMesadata>,
        preconfiguration: PlanarAutorouteConfiguration,
        presorter: SccIntersectionsAndLengthRatlinePlanarAutoroutePreconfigurer,
        _options: &PlanarAutorouteOptions,
    ) -> Self {
        // TODO: Instead of instantiating presorter again here, get it from
        // an argument.
        let sccs = presorter.dissolve();

        Self {
            configuration_search: Astar::new(SccSearchNode::new(sccs)),
            preconfiguration,
        }
    }
}

impl MakeNextPlanarAutorouteConfiguration for SccPermutationsPlanarAutorouteReconfigurer {
    fn next_configuration(
        &mut self,
        autorouter: &Autorouter<impl AccessMesadata>,
        stepper: &PlanarAutorouteExecutionStepper,
    ) -> Option<PlanarAutorouteConfiguration> {
        let scc_index = self
            .configuration_search
            .curr_node()
            .permutation()
            .iter()
            .position(|scc| {
                scc.scc_ref(autorouter)
                    .contains(self.preconfiguration.ratlines[*stepper.curr_ratline_index()])
            })
            .unwrap();

        let next_search_node = self
            .configuration_search
            .expand(&self.configuration_search.curr_node().expand(scc_index + 1))?;
        let next_permutation = next_search_node.permutation();
        let mut ratlines = vec![];

        for scc in next_permutation {
            for ratline in self.preconfiguration.ratlines.iter() {
                if scc.node_indices().contains(
                    &autorouter
                        .ratsnests()
                        .on_principal_layer(ratline.principal_layer)
                        .graph()
                        .edge_endpoints(ratline.index)
                        .unwrap()
                        .0,
                ) && scc.node_indices().contains(
                    &autorouter
                        .ratsnests()
                        .on_principal_layer(ratline.principal_layer)
                        .graph()
                        .edge_endpoints(ratline.index)
                        .unwrap()
                        .1,
                ) {
                    ratlines.push(*ratline);
                }
            }
        }

        Some(PlanarAutorouteConfiguration {
            ratlines,
            ..self.preconfiguration.clone()
        })
    }
}

/// Blocker-directed reconfigurer: a negotiated-congestion loop adapted
/// to the sequential router.
///
/// Two mechanisms are fed by the same recorded history:
///
/// 1. When a ratline fails because a band routed earlier blocks its
///    corridor, the failed ratline's SCC is ordered before the blocking
///    band's SCC in the next configuration. The failed net then claims
///    the channel first, and the blocking band is ripped up and
///    rerouted afterwards, negotiating around it. This is a directed
///    version of the ordering permutations: instead of enumerating the
///    whole permutation space, we jump straight to an ordering that
///    resolves the recorded conflict.
///
/// 2. Every primitive that blocks a route attempt accumulates a
///    congestion count. Counts on fixed primitives (pads, keepouts —
///    obstacles no reordering can remove) are emitted as
///    [`PlanarAutorouteConfiguration::obstacle_penalties`], inflating
///    the path cost of wrapping them so that retries prefer alternate
///    corridors.
pub struct NegotiatedCongestionPlanarAutorouteReconfigurer {
    fallback: SccPermutationsPlanarAutorouteReconfigurer,
    preconfiguration: PlanarAutorouteConfiguration,
    sccs: Vec<Scc>,
    /// "Earlier SCC must precede later SCC" precedence constraints, in
    /// the order they were learned.
    precedence: Vec<(Scc, Scc)>,
    /// Precedence constraints that produced cycles; never re-added.
    dead_precedence: BTreeSet<(Scc, Scc)>,
    /// Times each primitive blocked a route attempt.
    congestion: BTreeMap<PrimitiveIndex, usize>,
    /// Ratline orderings already returned, including the
    /// preconfiguration.
    emitted: BTreeSet<Vec<RatlineUid>>,
    /// The last `(ratline index, obstacles)` recorded, to avoid
    /// recounting when `reconfigure` loops back without new data.
    last_recorded: Option<(usize, Vec<PrimitiveIndex>)>,
    options: PlanarAutorouteOptions,
}

impl NegotiatedCongestionPlanarAutorouteReconfigurer {
    pub fn new(
        autorouter: &mut Autorouter<impl AccessMesadata>,
        preconfiguration: PlanarAutorouteConfiguration,
        presorter: SccIntersectionsAndLengthRatlinePlanarAutoroutePreconfigurer,
        options: &PlanarAutorouteOptions,
    ) -> Self {
        let sccs = presorter.sccs().clone();
        let mut emitted = BTreeSet::new();
        emitted.insert(preconfiguration.ratlines.clone());

        Self {
            fallback: SccPermutationsPlanarAutorouteReconfigurer::new(
                autorouter,
                preconfiguration.clone(),
                presorter,
                options,
            ),
            preconfiguration,
            sccs,
            precedence: vec![],
            dead_precedence: BTreeSet::new(),
            congestion: BTreeMap::new(),
            emitted,
            last_recorded: None,
            options: *options,
        }
    }

    /// Records the failing ratline's blocking obstacles: bumps each
    /// obstacle's congestion count and, for obstacles that are
    /// primitives of a band routed for another ratline, learns a
    /// precedence constraint ordering the failed ratline's SCC before
    /// the blocking ratline's SCC.
    fn record_blockers(
        &mut self,
        autorouter: &Autorouter<impl AccessMesadata>,
        stepper: &PlanarAutorouteExecutionStepper,
    ) {
        let curr_ratline_index = *stepper.curr_ratline_index();

        let Some(failed) = stepper
            .configuration()
            .ratlines
            .get(curr_ratline_index)
            .copied()
        else {
            return;
        };

        let drawing = autorouter.board().layout().drawing();
        let ratsnest = autorouter
            .ratsnests()
            .on_principal_layer(self.options.principal_layer);

        // Obstacles reported by the failed probes, plus clinch
        // detection: a foreign-net band wrapping an endpoint pad's gear
        // can clinch the ratline without any probe reporting the
        // clinching bend — a probe can die on `NoBitangents` or far
        // from the pad — so the endpoints' outward wrap chains are
        // checked directly.
        let mut obstacles: BTreeSet<PrimitiveIndex> =
            stepper.obstacles().iter().copied().collect();

        let (origin, destination) = failed.ref_(autorouter).endpoint_dots();

        for dot in [origin, destination] {
            let dot_ref = drawing.primitive_ref(dot);
            let pad_net = dot_ref.maybe_net();
            let mut outwards = dot_ref.outwards();

            while let Some(bend) = outwards.walk_next(drawing) {
                // A same-net wrap does not block the ratline.
                if bend.primitive_ref(drawing).maybe_net() == pad_net {
                    continue;
                }
                obstacles.insert(PrimitiveIndex::LooseBend(bend));
            }
        }

        let obstacles: Vec<PrimitiveIndex> = obstacles.into_iter().collect();

        if self
            .last_recorded
            .as_ref()
            .is_some_and(|last| *last == (curr_ratline_index, obstacles.clone()))
        {
            return;
        }
        self.last_recorded = Some((curr_ratline_index, obstacles.clone()));

        // Loose primitive -> ratline it was routed for. Bands are
        // resolved through the board's band registry, which stays
        // consistent under undo, rather than through the ratlines'
        // `band_termseg` weights: those go stale when a band is ripped
        // up during reconfiguration, after which they resolve to
        // unrelated or malformed loose chains.
        let mut band_of: BTreeMap<PrimitiveIndex, RatlineUid> = BTreeMap::new();
        for edge in ratsnest.graph().edge_indices() {
            let ratline = RatlineUid {
                principal_layer: self.options.principal_layer,
                index: edge,
            };
            let (origin, destination) = ratline.ref_(autorouter).endpoint_dots();

            let Some(band) = autorouter.board.band_between_nodes(origin, destination) else {
                continue;
            };

            let mut maybe_loose: Option<LooseIndex> = Some(band[false].into());
            let mut prev = None;

            while let Some(loose) = maybe_loose {
                band_of.insert(loose.into(), ratline);

                let prev_prev = prev;
                prev = maybe_loose;
                maybe_loose = drawing.loose(loose).next_loose(prev_prev);
            }
        }

        for obstacle in obstacles {
            *self.congestion.entry(obstacle).or_insert(0) += 1;

            let Some(blocker) = band_of.get(&obstacle).copied() else {
                continue;
            };

            let (Some(failed_scc), Some(blocker_scc)) = (
                self.scc_of(autorouter, failed),
                self.scc_of(autorouter, blocker),
            ) else {
                continue;
            };

            // A same-net band cannot infringe, so same-SCC blockers
            // shouldn't occur; guard anyway.
            if failed_scc == blocker_scc {
                continue;
            }

            let constraint = (failed_scc, blocker_scc);
            if !self.dead_precedence.contains(&constraint)
                && !self.precedence.contains(&constraint)
            {
                self.precedence.push(constraint);
            }
        }
    }

    fn scc_of(
        &self,
        autorouter: &Autorouter<impl AccessMesadata>,
        ratline: RatlineUid,
    ) -> Option<Scc> {
        self.sccs
            .iter()
            .find(|scc| scc.scc_ref(autorouter).contains(ratline))
            .cloned()
    }

    /// The SCC ordering of the stepper's current configuration, so
    /// directed reorders perturb as few routed bands as possible.
    fn base_scc_order(
        &self,
        autorouter: &Autorouter<impl AccessMesadata>,
        stepper: &PlanarAutorouteExecutionStepper,
    ) -> Vec<Scc> {
        let mut order = vec![];

        for ratline in &stepper.configuration().ratlines {
            if let Some(scc) = self.scc_of(autorouter, *ratline) {
                if !order.contains(&scc) {
                    order.push(scc);
                }
            }
        }

        for scc in &self.sccs {
            if !order.contains(scc) {
                order.push(scc.clone());
            }
        }

        order
    }

    /// Applies all live precedence constraints to `base` via
    /// move-before operations. Constraints found to be cyclic are moved
    /// to `dead_precedence` (newest first). Returns `None` when there
    /// are no constraints to apply.
    fn constrained_order(&mut self, base: &[Scc]) -> Option<Vec<Scc>> {
        if self.precedence.is_empty() {
            return None;
        }

        loop {
            let mut order = base.to_vec();

            for (earlier, later) in self.precedence.iter() {
                let (Some(pos_earlier), Some(pos_later)) = (
                    order.iter().position(|scc| scc == earlier),
                    order.iter().position(|scc| scc == later),
                ) else {
                    continue;
                };

                if pos_earlier > pos_later {
                    let scc = order.remove(pos_earlier);
                    let pos_later = order.iter().position(|s| s == later).unwrap();
                    order.insert(pos_later, scc);
                }
            }

            // A constraint still violated must participate in a cycle.
            // Kill the newest such constraint and rebuild.
            if let Some(dead) = self
                .precedence
                .iter()
                .rposition(|(earlier, later)| {
                    order.iter().position(|s| s == earlier)
                        > order.iter().position(|s| s == later)
                })
                .map(|index| self.precedence.remove(index))
            {
                self.dead_precedence.insert(dead);
                continue;
            }

            return Some(order);
        }
    }

    fn ratlines_in_scc_order(
        &self,
        autorouter: &Autorouter<impl AccessMesadata>,
        order: &[Scc],
    ) -> Vec<RatlineUid> {
        let mut ratlines = vec![];

        for scc in order {
            for ratline in self.preconfiguration.ratlines.iter() {
                if scc.scc_ref(autorouter).contains(*ratline) {
                    ratlines.push(*ratline);
                }
            }
        }

        ratlines
    }

    /// Congestion costs translated to path penalties. Only fixed
    /// primitives are penalized: loose indices are recycled as bands
    /// are undone and redrawn, so penalizing them could hit unrelated
    /// geometry.
    fn obstacle_penalties(&self) -> BTreeMap<PrimitiveIndex, f64> {
        self.congestion
            .iter()
            .filter(|(primitive, _)| {
                matches!(
                    primitive,
                    PrimitiveIndex::FixedDot(_)
                        | PrimitiveIndex::FixedSeg(_)
                        | PrimitiveIndex::FixedBend(_)
                )
            })
            .map(|(primitive, count)| {
                (*primitive, *count as f64 * self.options.congestion_penalty)
            })
            .collect()
    }
}

impl MakeNextPlanarAutorouteConfiguration for NegotiatedCongestionPlanarAutorouteReconfigurer {
    fn next_configuration(
        &mut self,
        autorouter: &Autorouter<impl AccessMesadata>,
        stepper: &PlanarAutorouteExecutionStepper,
    ) -> Option<PlanarAutorouteConfiguration> {
        self.record_blockers(autorouter, stepper);

        // Directed attempt: ordering satisfying all learned precedence
        // constraints, perturbed minimally from the current ordering.
        if let Some(order) = self.constrained_order(&self.base_scc_order(autorouter, stepper)) {
            let ratlines = self.ratlines_in_scc_order(autorouter, &order);

            if ratlines != stepper.configuration().ratlines
                && self.emitted.insert(ratlines.clone())
            {
                return Some(PlanarAutorouteConfiguration {
                    ratlines,
                    obstacle_penalties: self.obstacle_penalties(),
                    ..self.preconfiguration.clone()
                });
            }
        }

        // Fallback: enumerate orderings, carrying the accumulated
        // congestion penalties either way.
        loop {
            let mut configuration = self.fallback.next_configuration(autorouter, stepper)?;
            configuration.obstacle_penalties = self.obstacle_penalties();

            if self.emitted.insert(configuration.ratlines.clone()) {
                return Some(configuration);
            }
        }
    }
}
