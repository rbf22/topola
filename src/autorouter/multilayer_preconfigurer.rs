// SPDX-FileCopyrightText: 2025 Topola contributors
//
// SPDX-License-Identifier: MIT

use std::collections::{BTreeMap, BTreeSet};

use derive_getters::Getters;
use geo::{Distance, Euclidean, Line};
use rstar::{
    primitives::{GeomWithData, Rectangle},
    RTree, RTreeObject, AABB,
};
use specctra_core::mesadata::AccessMesadata;

use crate::{
    autorouter::{anterouter::AnterouterPlan, ratline::RatlineUid, Autorouter},
    drawing::{
        dot::FixedDotIndex,
        graph::{MakePrimitiveRef, PrimitiveIndex},
        primitive::MakePrimitiveShape,
    },
    geometry::{shape::AccessShape, GenericNode, GetLayer},
    graph::MakeRef,
};

#[derive(Clone, Debug)]
pub struct MultilayerAutoroutePreconfigurerInput {
    pub ratlines: BTreeSet<RatlineUid>,
}

/// splitmix64 finalizer — deterministic, cheap, well-distributed.
fn splitmix64(mut x: u64) -> u64 {
    x = x.wrapping_add(0x9e3779b97f4a7c15);
    x = (x ^ (x >> 30)).wrapping_mul(0xbf58476d1ce4e5b9);
    x = (x ^ (x >> 27)).wrapping_mul(0x94d049bb133111eb);
    x ^ (x >> 31)
}

#[derive(Getters)]
pub struct MultilayerPreconfigurer {
    plan: AnterouterPlan,
}

impl MultilayerPreconfigurer {
    /// Mutable access to the plan — the annealing handoff extends
    /// `static_terminating_dot_map` with via-dot entries the pinname
    /// scan cannot find.
    pub fn plan_mut(&mut self) -> &mut AnterouterPlan {
        &mut self.plan
    }
}

impl MultilayerPreconfigurer {
    pub fn new(
        autorouter: &Autorouter<impl AccessMesadata>,
        input: MultilayerAutoroutePreconfigurerInput,
    ) -> Self {
        // The corridor margin is only read by the nonzero-seed branch.
        Self::new_seeded(autorouter, input, 0, 0.0)
    }

    /// `layer_seed == 0` keeps every ratline's preferred layer. A
    /// nonzero seed assigns layers conflict-directedly: ratlines whose
    /// straight-line demand corridors approach within
    /// `corridor_margin` compete for the same channel, so conflicting
    /// ratlines are spread over distinct layers where a free layer
    /// exists — the seed only rotates each ratline's scan order, so
    /// parallel workers resolve conflicts differently while
    /// conflict-free ratlines keep their preferred layer (blind
    /// random assignment showed that preferred layers are often
    /// near-optimal). Assignments that fail fanout placement fall back
    /// to the preferred layer.
    pub fn new_seeded(
        autorouter: &Autorouter<impl AccessMesadata>,
        input: MultilayerAutoroutePreconfigurerInput,
        layer_seed: u64,
        corridor_margin: f64,
    ) -> Self {
        let layer_count = autorouter.board().layout().drawing().layer_count();
        let layer_map = if layer_seed == 0 {
            input
                .ratlines
                .iter()
                .map(|ratline| (*ratline, ratline.ref_(autorouter).preferred_layer()))
                .collect()
        } else {
            Self::conflict_directed_layer_map(
                autorouter,
                &input.ratlines,
                layer_seed,
                layer_count,
                corridor_margin,
            )
        };
        Self::new_from_layer_map(autorouter, &input.ratlines, layer_map)
    }

    /// The straight-line corridor a ratline demands between its
    /// endpoint dots.
    fn corridor(autorouter: &Autorouter<impl AccessMesadata>, ratline: RatlineUid) -> Line<f64> {
        let drawing = autorouter.board().layout().drawing();
        let pos = |dot: FixedDotIndex| dot.primitive_ref(drawing).shape().center();
        let (origin, destination) = ratline.ref_(autorouter).endpoint_dots();
        Line::new(pos(origin), pos(destination))
    }

    /// Conflict-directed layer assignment: two ratlines whose demand
    /// corridors come within `corridor_margin` cannot both follow
    /// their demand paths on one layer, so the second gets an
    /// alternate channel. Most-conflicted ratlines assign first; each
    /// keeps its preferred layer when unclaimed by a conflicting
    /// already-assigned corridor, else takes the first free layer in a
    /// seed-rotated scan. Falls back to the preferred layer when every
    /// layer is claimed.
    fn conflict_directed_layer_map(
        autorouter: &Autorouter<impl AccessMesadata>,
        ratlines: &BTreeSet<RatlineUid>,
        layer_seed: u64,
        layer_count: usize,
        corridor_margin: f64,
    ) -> BTreeMap<RatlineUid, usize> {
        let corridors: Vec<(RatlineUid, Line<f64>)> = ratlines
            .iter()
            .map(|ratline| (*ratline, Self::corridor(autorouter, *ratline)))
            .collect();

        // Envelope-pruned candidate pairs; corridors conflict when
        // their segments actually come within the margin.
        let envelope_of = |line: &Line<f64>| {
            let envelope = AABB::from_points([&line.start_point(), &line.end_point()]);
            let (l, u) = (envelope.lower(), envelope.upper());
            Rectangle::from_corners(
                geo::Point::new(l.x() - corridor_margin, l.y() - corridor_margin),
                geo::Point::new(u.x() + corridor_margin, u.y() + corridor_margin),
            )
        };
        let envelopes: Vec<Rectangle<geo::Point<f64>>> = corridors
            .iter()
            .map(|(_, line)| envelope_of(line))
            .collect();
        let tree = RTree::bulk_load(
            envelopes
                .iter()
                .enumerate()
                .map(|(i, envelope)| GeomWithData::new(*envelope, i))
                .collect(),
        );

        let mut conflicts: Vec<BTreeSet<usize>> = vec![BTreeSet::new(); corridors.len()];
        for (i, (_, line)) in corridors.iter().enumerate() {
            for hit in tree.locate_in_envelope_intersecting(&envelopes[i].envelope()) {
                let j = hit.data;
                if j != i && Euclidean::distance(line, &corridors[j].1) < corridor_margin {
                    conflicts[i].insert(j);
                    conflicts[j].insert(i);
                }
            }
        }

        // Most-conflicted ratlines assign first — they have the least
        // freedom left later.
        let mut order: Vec<usize> = (0..corridors.len()).collect();
        order.sort_by_key(|&i| (std::cmp::Reverse(conflicts[i].len()), i));

        let mut layers = vec![usize::MAX; corridors.len()];
        let mut map = BTreeMap::new();
        for &i in &order {
            let (ratline, _) = corridors[i];
            let preferred = ratline.ref_(autorouter).preferred_layer();
            let chosen = if conflicts[i].is_empty() {
                preferred
            } else {
                let claimed: BTreeSet<usize> = conflicts[i]
                    .iter()
                    .filter(|&&j| layers[j] != usize::MAX)
                    .map(|&j| layers[j])
                    .collect();
                if !claimed.contains(&preferred) {
                    preferred
                } else {
                    let start = splitmix64(layer_seed ^ i as u64) as usize % layer_count;
                    (0..layer_count)
                        .map(|k| (start + k) % layer_count)
                        .find(|layer| !claimed.contains(layer))
                        .unwrap_or(preferred)
                }
            };
            layers[i] = chosen;
            map.insert(ratline, chosen);
        }
        map
    }

    pub fn new_from_layer_map(
        autorouter: &Autorouter<impl AccessMesadata>,
        ratlines: &BTreeSet<RatlineUid>,
        layer_map: BTreeMap<RatlineUid, usize>,
    ) -> Self {
        let mut plan = AnterouterPlan {
            layer_map,
            static_terminating_dot_map: BTreeMap::new(),
        };

        for ratline in ratlines {
            for layer in 0..autorouter.board().layout().drawing().layer_count() {
                if let Some(static_terminating_dot) = Self::find_static_terminating_dot(
                    autorouter,
                    ratline.ref_(autorouter).endpoint_dots().0,
                    layer,
                ) {
                    plan.static_terminating_dot_map.insert(
                        (*ratline, ratline.ref_(autorouter).endpoint_dots().0, layer),
                        static_terminating_dot,
                    );
                }

                if let Some(static_terminating_dot) = Self::find_static_terminating_dot(
                    autorouter,
                    ratline.ref_(autorouter).endpoint_dots().1,
                    layer,
                ) {
                    plan.static_terminating_dot_map.insert(
                        (*ratline, ratline.ref_(autorouter).endpoint_dots().1, layer),
                        static_terminating_dot,
                    );
                }
            }
        }

        Self { plan }
    }

    fn find_static_terminating_dot(
        autorouter: &Autorouter<impl AccessMesadata>,
        ratline_endpoint_dot: FixedDotIndex,
        layer: usize,
    ) -> Option<FixedDotIndex> {
        if layer
            == ratline_endpoint_dot
                .primitive_ref(autorouter.board().layout().drawing())
                .layer()
        {
            return Some(ratline_endpoint_dot);
        }

        // Anonymous endpoint dots (e.g. wire-end dots on partially routed
        // boards) have no pin name, so no same-pin terminating dot exists on
        // other layers.
        let pinname = autorouter
            .board()
            .node_pinname(&GenericNode::Primitive(ratline_endpoint_dot.into()))?;

        autorouter.board().pinname_nodes(pinname).find_map(|node| {
            if let GenericNode::Primitive(PrimitiveIndex::FixedDot(dot)) = node {
                (layer
                    == dot
                        .primitive_ref(autorouter.board().layout().drawing())
                        .layer())
                .then_some(dot)
            } else {
                None
            }
        })
    }
}

#[cfg(test)]
mod tests {
    use std::{fs::File, io::BufReader};

    use super::*;
    use crate::{
        board::edit::BoardEdit,
        specctra::{design::SpecctraDesign, mesadata::SpecctraMesadata},
    };

    fn astable() -> Autorouter<SpecctraMesadata> {
        let design_file =
            File::open("tests/multilayer/astable_multivibrator/astable_multivibrator.dsn").unwrap();
        let design = SpecctraDesign::load(BufReader::new(design_file)).unwrap();
        Autorouter::new(design.make_board(&mut BoardEdit::new())).unwrap()
    }

    /// Every ratline on every principal layer.
    fn input(autorouter: &Autorouter<SpecctraMesadata>) -> MultilayerAutoroutePreconfigurerInput {
        let mut ratlines = BTreeSet::new();
        for layer in 0..autorouter.board().layout().drawing().layer_count() {
            ratlines.extend(
                autorouter
                    .ratsnests()
                    .on_principal_layer(layer)
                    .graph()
                    .edge_indices()
                    .map(|index| RatlineUid {
                        principal_layer: layer,
                        index,
                    }),
            );
        }
        MultilayerAutoroutePreconfigurerInput { ratlines }
    }

    #[test]
    fn seeded_assignment_is_deterministic_and_bounded() {
        let autorouter = astable();
        let layer_count = autorouter.board().layout().drawing().layer_count();
        let input = input(&autorouter);

        let first = MultilayerPreconfigurer::new_seeded(&autorouter, input.clone(), 7, 100.0);
        let second = MultilayerPreconfigurer::new_seeded(&autorouter, input, 7, 100.0);
        assert_eq!(first.plan().layer_map, second.plan().layer_map);
        assert!(first
            .plan()
            .layer_map
            .values()
            .all(|layer| *layer < layer_count));
    }

    #[test]
    fn zero_seed_keeps_preferred_layers() {
        let autorouter = astable();
        let input = input(&autorouter);

        let preferred = MultilayerPreconfigurer::new(&autorouter, input.clone());
        let seeded = MultilayerPreconfigurer::new_seeded(&autorouter, input, 0, 100.0);
        assert_eq!(preferred.plan().layer_map, seeded.plan().layer_map);
    }

    /// Conflict-directed assignment only departs from the preferred
    /// layer where corridors contend, and seeds may resolve conflicts
    /// differently.
    #[test]
    fn conflict_seeded_assignment_spreads_conflicts() {
        let autorouter = astable();
        let layer_count = autorouter.board().layout().drawing().layer_count();
        let input = input(&autorouter);
        let preferred = MultilayerPreconfigurer::new(&autorouter, input.clone());
        let preferred = &preferred.plan().layer_map;

        // A large margin makes every nearby corridor conflict; a zero
        // margin (pointwise overlap only) should leave almost every
        // ratline preferred.
        let tight = MultilayerPreconfigurer::new_seeded(&autorouter, input.clone(), 7, 1.0);
        let wide = MultilayerPreconfigurer::new_seeded(&autorouter, input.clone(), 7, 300.0);
        for layer in tight.plan().layer_map.values() {
            assert!(*layer < layer_count);
        }
        for layer in wide.plan().layer_map.values() {
            assert!(*layer < layer_count);
        }
        // Wider margins can only add conflicts, so the wide map may
        // diverge more from preferred than the tight one — assert it
        // is at least as divergent.
        let diverge = |map: &BTreeMap<RatlineUid, usize>| {
            map.iter()
                .filter(|(ratline, layer)| preferred.get(*ratline) != Some(layer))
                .count()
        };
        assert!(diverge(&wide.plan().layer_map) >= diverge(&tight.plan().layer_map));
    }

    #[test]
    fn nonzero_seed_changes_assignment() {
        let autorouter = astable();
        let input = input(&autorouter);

        let preferred = MultilayerPreconfigurer::new(&autorouter, input.clone());
        let seeded = MultilayerPreconfigurer::new_seeded(&autorouter, input, 7, 300.0);
        assert_ne!(preferred.plan().layer_map, seeded.plan().layer_map);
    }
}
