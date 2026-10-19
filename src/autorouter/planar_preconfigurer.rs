// SPDX-FileCopyrightText: 2025 Topola contributors
//
// SPDX-License-Identifier: MIT

use std::collections::{BTreeMap, BTreeSet};

use derive_getters::{Dissolve, Getters};
use enum_dispatch::enum_dispatch;
use geo::{
    algorithm::line_measures::{Distance, Euclidean},
    Line,
};
use petgraph::algo::tarjan_scc;
use rstar::{
    primitives::{GeomWithData, Rectangle},
    RTree, RTreeObject,
};
use specctra_core::mesadata::AccessMesadata;

use crate::{
    autorouter::{
        planar_autoroute::PlanarAutorouteConfiguration, ratline::RatlineUid, scc::Scc, Autorouter,
        PlanarAutorouteOptions, PresortBy,
    },
    drawing::{
        dot::FixedDotIndex,
        graph::{GetMaybeNet, MakePrimitiveRef},
        primitive::{GetWeight, MakePrimitiveShape},
    },
    geometry::{
        primitive::{PrimitiveShape, SegShape},
        shape::AccessShape,
        GetSetPos,
    },
    graph::MakeRef,
};

#[derive(Clone, Debug, Eq, Ord, PartialEq, PartialOrd)]
pub struct PlanarAutoroutePreconfigurerInput {
    pub ratlines: BTreeSet<RatlineUid>,
    pub terminating_dot_map: BTreeMap<(RatlineUid, FixedDotIndex), FixedDotIndex>,
}

pub struct PresortParams {
    pub intersector_count_weight: f64,
    pub length_weight: f64,
}

#[enum_dispatch]
pub trait PreconfigurePlanarAutoroute {
    fn preconfigure(
        &self,
        autorouter: &mut Autorouter<impl AccessMesadata>,
        input: PlanarAutoroutePreconfigurerInput,
    ) -> PlanarAutorouteConfiguration;
}

#[enum_dispatch(PresortRatlines)]
pub enum PlanarAutoroutePreconfigurer {
    SccIntersectionsLength(SccIntersectionsAndLengthRatlinePlanarAutoroutePreconfigurer),
}

#[derive(Getters, Dissolve)]
pub struct SccIntersectionsAndLengthRatlinePlanarAutoroutePreconfigurer {
    sccs: Vec<Scc>,
    /// Per-ratline escape-difficulty scores (endpoint crowding),
    /// ordering ratlines within each SCC when
    /// [`PresortBy::RatlineEscapeDensity`] is selected: ratlines
    /// touching the densest regions route first, before corridors
    /// clog. Empty when a different presort is selected.
    escape_scores: BTreeMap<RatlineUid, f64>,
    /// Per-ratline corridor obstruction counts (primitives infringing
    /// the straight origin→destination seg at routing clearances),
    /// ordering ratlines within each SCC when
    /// [`PresortBy::RatlineCorridorCongestion`] is selected: the most
    /// constrained ratlines route first, claiming contested channels
    /// before other bands wall them in. Empty otherwise.
    corridor_scores: BTreeMap<RatlineUid, f64>,
    /// Per-ratline demand-corridor conflict counts: how many other
    /// ratlines' corridors pass within one band width. Among equally
    /// obstructed ratlines the more-conflicted one claims the channel
    /// first. Computed together with `corridor_scores` — empty
    /// otherwise.
    conflict_scores: BTreeMap<RatlineUid, f64>,
    /// Seed for a deterministic within-SCC ratline shuffle; `0` keeps
    /// the deterministic ordering. Parallel exploration runs give each
    /// attempt a different seed so they traverse different ordering
    /// trajectories.
    shuffle_seed: u64,
}

impl SccIntersectionsAndLengthRatlinePlanarAutoroutePreconfigurer {
    pub fn new(
        autorouter: &mut Autorouter<impl AccessMesadata>,
        input: PlanarAutoroutePreconfigurerInput,
        params: &PresortParams,
        options: &PlanarAutorouteOptions,
    ) -> Self {
        // FIXME: Unnecessary copy.
        let mut filtered_ratsnest = autorouter
            .ratsnests()
            .on_principal_layer(options.principal_layer)
            .graph()
            .clone();
        filtered_ratsnest
            .retain_edges(|_g, i| input.ratlines.iter().any(|ratline| ratline.index == i));

        let mut sccs: Vec<_> = tarjan_scc(&filtered_ratsnest)
            .into_iter()
            // Drop SCCs containing no selected ratline — singleton
            // pin-nodes left edgeless by the edge filter above. They
            // cannot affect the emitted ratline ordering but would
            // otherwise inflate the permutation space searched by the
            // reconfigurer's fallback from (#nets)! to (#pins)!, making
            // nearly every emitted ordering a duplicate.
            .filter(|node_indices| {
                input.ratlines.iter().any(|ratline| {
                    let (from, to) = filtered_ratsnest.edge_endpoints(ratline.index).unwrap();
                    node_indices.contains(&from) && node_indices.contains(&to)
                })
            })
            .map(|node_indices| {
                Scc::new(
                    autorouter,
                    &input.ratlines,
                    &filtered_ratsnest,
                    node_indices,
                )
            })
            .collect();

        sccs.sort_unstable_by(|a, b| {
            Self::scc_score(params, a).total_cmp(&Self::scc_score(params, b))
        });

        let escape_scores = if options.presort_by == PresortBy::RatlineEscapeDensity {
            Self::compute_escape_scores(autorouter, &input, 20.0 * options.router.routed_band_width)
        } else {
            BTreeMap::new()
        };

        let (corridor_scores, conflict_scores) =
            if options.presort_by == PresortBy::RatlineCorridorCongestion {
                Self::compute_corridor_scores(
                    autorouter,
                    &input,
                    options.principal_layer,
                    options.router.routed_band_width,
                )
            } else {
                (BTreeMap::new(), BTreeMap::new())
            };

        Self {
            sccs,
            escape_scores,
            corridor_scores,
            conflict_scores,
            shuffle_seed: options.shuffle_seed,
        }
    }

    fn scc_score(params: &PresortParams, scc: &Scc) -> f64 {
        params.intersector_count_weight * *scc.intersector_count() as f64
            + params.length_weight * scc.length()
    }

    /// Counts, for each ratline, the primitives that would infringe the
    /// straight origin→destination corridor at routing clearances: a
    /// direct measure of how blocked the ratline's shortest path is —
    /// plus how many other ratlines' demand corridors come within one
    /// band width of it, a conflict signal for scheduling contested
    /// channels first. Same-net primitives and loose dots are already
    /// excluded by the infringement pipeline, so a ratline's own
    /// endpoints don't count; same-net corridor pairs are excluded
    /// from conflicts because their bands merge rather than compete.
    fn compute_corridor_scores(
        autorouter: &Autorouter<impl AccessMesadata>,
        input: &PlanarAutoroutePreconfigurerInput,
        layer: usize,
        width: f64,
    ) -> (BTreeMap<RatlineUid, f64>, BTreeMap<RatlineUid, f64>) {
        let drawing = autorouter.board().layout().drawing();
        let corridors: Vec<_> = input
            .ratlines
            .iter()
            .map(|ratline| {
                let (origin, destination) = ratline.ref_(autorouter).endpoint_dots();
                let from = origin.primitive_ref(drawing).shape().center();
                let to = destination.primitive_ref(drawing).shape().center();
                (
                    *ratline,
                    Line::new(from, to),
                    origin.primitive_ref(drawing).maybe_net(),
                )
            })
            .collect();

        let scores = corridors
            .iter()
            .map(|(ratline, line, maybe_net)| {
                let corridor = PrimitiveShape::Seg(SegShape {
                    from: line.start_point(),
                    to: line.end_point(),
                    width,
                });
                (
                    *ratline,
                    drawing
                        .infringements_of_shape(corridor, layer, *maybe_net, &|_, _| true)
                        .count() as f64,
                )
            })
            .collect();

        // Envelope-pruned corridor pairs conflict when the lines come
        // within one band width: two bands cannot both fit there.
        let margin = width;
        let envelope_of = |line: &Line<f64>| {
            let envelope = rstar::AABB::from_points([&line.start_point(), &line.end_point()]);
            let (l, u) = (envelope.lower(), envelope.upper());
            Rectangle::from_corners(
                geo::Point::new(l.x() - margin, l.y() - margin),
                geo::Point::new(u.x() + margin, u.y() + margin),
            )
        };
        let envelopes: Vec<Rectangle<geo::Point<f64>>> = corridors
            .iter()
            .map(|(_, line, _)| envelope_of(line))
            .collect();
        let tree = RTree::bulk_load(
            envelopes
                .iter()
                .enumerate()
                .map(|(i, envelope)| GeomWithData::new(*envelope, i))
                .collect(),
        );

        let mut conflicts = BTreeMap::new();
        for (i, (ratline, line, maybe_net)) in corridors.iter().enumerate() {
            let mut count = 0.0;
            for hit in tree.locate_in_envelope_intersecting(&envelopes[i].envelope()) {
                let j = hit.data;
                let same_net = maybe_net.is_some() && *maybe_net == corridors[j].2;
                if j != i && !same_net && Euclidean::distance(&*line, &corridors[j].1) < margin {
                    count += 1.0;
                }
            }
            conflicts.insert(*ratline, count);
        }

        (scores, conflicts)
    }

    /// Counts, for each ratline, the number of ratline endpoints
    /// within `radius` of its own endpoints: endpoints compete for the
    /// same escape corridors, so the count is a congestion proxy. A
    /// ratline's score is the larger of its two endpoints' counts.
    fn compute_escape_scores(
        autorouter: &mut Autorouter<impl AccessMesadata>,
        input: &PlanarAutoroutePreconfigurerInput,
        radius: f64,
    ) -> BTreeMap<RatlineUid, f64> {
        let dot_pos = |dot: FixedDotIndex| {
            autorouter
                .board()
                .layout()
                .drawing()
                .primitive_ref(dot)
                .weight()
                .pos()
        };

        let endpoint_positions: Vec<_> = input
            .ratlines
            .iter()
            .flat_map(|ratline| {
                let (origin, destination) = ratline.ref_(autorouter).endpoint_dots();
                [dot_pos(origin), dot_pos(destination)]
            })
            .collect();

        input
            .ratlines
            .iter()
            .map(|ratline| {
                let (origin, destination) = ratline.ref_(autorouter).endpoint_dots();
                let score = [origin, destination]
                    .into_iter()
                    .map(|dot| {
                        let pos = dot_pos(dot);
                        endpoint_positions
                            .iter()
                            .filter(|other| Euclidean::distance(pos, **other) <= radius)
                            .count() as f64
                    })
                    .fold(0.0, f64::max);
                (*ratline, score)
            })
            .collect()
    }
}

impl PreconfigurePlanarAutoroute for SccIntersectionsAndLengthRatlinePlanarAutoroutePreconfigurer {
    fn preconfigure(
        &self,
        autorouter: &mut Autorouter<impl AccessMesadata>,
        input: PlanarAutoroutePreconfigurerInput,
    ) -> PlanarAutorouteConfiguration {
        let mut ratlines: Vec<_> = input.ratlines.iter().copied().collect();
        if self.shuffle_seed != 0 {
            // Deterministic within-SCC diversity for parallel
            // exploration: shuffling the input only permutes the order
            // ratlines are collected *inside* each SCC below — the
            // SCC-level congestion ordering is preserved.
            let mut state = self.shuffle_seed.wrapping_mul(0x9e3779b97f4a7c15).max(1);
            for i in (1..ratlines.len()).rev() {
                state ^= state << 13;
                state ^= state >> 7;
                state ^= state << 17;
                let j = (state % (i as u64 + 1)) as usize;
                ratlines.swap(i, j);
            }
        }
        let mut presorted_ratlines = vec![];

        for scc in self.sccs.iter() {
            let mut scc_ratlines: Vec<_> = ratlines
                .iter()
                .filter(|ratline| scc.scc_ref(autorouter).contains(**ratline))
                .copied()
                .collect();

            // Constraint-first ordering: the ratline whose corridor is
            // most blocked (or, for escape ordering, whose most-crowded
            // endpoint is densest) routes first, while its corridors
            // are still open. Corridor congestion dominates; among
            // equally blocked ratlines the one whose corridor conflicts
            // with more rivals claims the contested channel first;
            // escape density breaks remaining ties.
            scc_ratlines.sort_by(|a, b| {
                self.corridor_scores
                    .get(b)
                    .copied()
                    .unwrap_or(0.0)
                    .total_cmp(&self.corridor_scores.get(a).copied().unwrap_or(0.0))
                    .then(
                        self.conflict_scores
                            .get(b)
                            .copied()
                            .unwrap_or(0.0)
                            .total_cmp(&self.conflict_scores.get(a).copied().unwrap_or(0.0)),
                    )
                    .then(
                        self.escape_scores
                            .get(b)
                            .copied()
                            .unwrap_or(0.0)
                            .total_cmp(&self.escape_scores.get(a).copied().unwrap_or(0.0)),
                    )
            });

            presorted_ratlines.extend(scc_ratlines);
        }

        PlanarAutorouteConfiguration {
            ratlines: presorted_ratlines,
            terminating_dot_map: input.terminating_dot_map,
            obstacle_penalties: BTreeMap::new(),
            rip_up: vec![],
            protected: BTreeSet::new(),
        }
    }
}
