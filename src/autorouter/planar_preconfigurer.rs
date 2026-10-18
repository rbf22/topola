// SPDX-FileCopyrightText: 2025 Topola contributors
//
// SPDX-License-Identifier: MIT

use std::collections::{BTreeMap, BTreeSet};

use derive_getters::{Dissolve, Getters};
use enum_dispatch::enum_dispatch;
use geo::algorithm::line_measures::{Distance, Euclidean};
use petgraph::algo::tarjan_scc;
use specctra_core::mesadata::AccessMesadata;

use crate::{
    autorouter::{
        planar_autoroute::PlanarAutorouteConfiguration, ratline::RatlineUid, scc::Scc, Autorouter,
        PlanarAutorouteOptions, PresortBy,
    },
    drawing::{dot::FixedDotIndex, primitive::GetWeight},
    geometry::GetSetPos,
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
            Self::compute_escape_scores(
                autorouter,
                &input,
                20.0 * options.router.routed_band_width,
            )
        } else {
            BTreeMap::new()
        };

        Self { sccs, escape_scores }
    }

    fn scc_score(params: &PresortParams, scc: &Scc) -> f64 {
        params.intersector_count_weight * *scc.intersector_count() as f64
            + params.length_weight * scc.length()
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
        let mut presorted_ratlines = vec![];

        for scc in self.sccs.iter() {
            let mut scc_ratlines: Vec<_> = input
                .ratlines
                .iter()
                .filter(|ratline| scc.scc_ref(autorouter).contains(**ratline))
                .copied()
                .collect();

            // Escape-first ordering: the ratline whose most-crowded
            // endpoint is densest routes first, while its corridors
            // are still open.
            scc_ratlines.sort_by(|a, b| {
                self.escape_scores
                    .get(b)
                    .copied()
                    .unwrap_or(0.0)
                    .total_cmp(&self.escape_scores.get(a).copied().unwrap_or(0.0))
            });

            presorted_ratlines.extend(scc_ratlines);
        }

        PlanarAutorouteConfiguration {
            ratlines: presorted_ratlines,
            terminating_dot_map: input.terminating_dot_map,
            obstacle_penalties: BTreeMap::new(),
            rip_up: vec![],
        }
    }
}
