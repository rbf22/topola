// SPDX-FileCopyrightText: 2025 Topola contributors
//
// SPDX-License-Identifier: MIT

use std::collections::{BTreeMap, BTreeSet};

use derive_getters::Getters;
use specctra_core::mesadata::AccessMesadata;

use crate::{
    autorouter::{anterouter::AnterouterPlan, ratline::RatlineUid, Autorouter},
    drawing::{
        dot::FixedDotIndex,
        graph::{MakePrimitiveRef, PrimitiveIndex},
    },
    geometry::{GenericNode, GetLayer},
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
    pub fn new(
        autorouter: &Autorouter<impl AccessMesadata>,
        input: MultilayerAutoroutePreconfigurerInput,
    ) -> Self {
        Self::new_seeded(autorouter, input, 0)
    }

    /// `layer_seed == 0` keeps every ratline's preferred layer. A
    /// nonzero seed assigns each ratline a deterministic
    /// pseudo-random layer, so parallel routing workers explore
    /// different initial layer assignments. Assignments that fail
    /// fanout placement fall back to the preferred layer.
    pub fn new_seeded(
        autorouter: &Autorouter<impl AccessMesadata>,
        input: MultilayerAutoroutePreconfigurerInput,
        layer_seed: u64,
    ) -> Self {
        let layer_count = autorouter.board().layout().drawing().layer_count();
        Self::new_from_layer_map(
            autorouter,
            &input.ratlines,
            input
                .ratlines
                .iter()
                .enumerate()
                .map(|(i, ratline)| {
                    let layer = if layer_seed == 0 {
                        ratline.ref_(autorouter).preferred_layer()
                    } else {
                        splitmix64(layer_seed ^ i as u64) as usize % layer_count
                    };
                    (*ratline, layer)
                })
                .collect(),
        )
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
        let design_file = File::open(
            "tests/multilayer/astable_multivibrator/astable_multivibrator.dsn",
        )
        .unwrap();
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

        let first = MultilayerPreconfigurer::new_seeded(&autorouter, input.clone(), 7);
        let second = MultilayerPreconfigurer::new_seeded(&autorouter, input, 7);
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
        let seeded = MultilayerPreconfigurer::new_seeded(&autorouter, input, 0);
        assert_eq!(preferred.plan().layer_map, seeded.plan().layer_map);
    }

    #[test]
    fn nonzero_seed_changes_assignment() {
        let autorouter = astable();
        let input = input(&autorouter);

        let preferred = MultilayerPreconfigurer::new(&autorouter, input.clone());
        let seeded = MultilayerPreconfigurer::new_seeded(&autorouter, input, 7);
        assert_ne!(preferred.plan().layer_map, seeded.plan().layer_map);
    }
}
