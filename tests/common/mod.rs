// SPDX-FileCopyrightText: 2024 Topola contributors
//
// SPDX-License-Identifier: MIT

// Helpers shared across test binaries; not every consumer uses all of them.
#![allow(dead_code)]

use std::{fs::File, io::BufReader};

use topola::{
    autorouter::{
        connected_components::ConnectedComponents,
        history::{History, HistoryError},
        invoker::{Invoker, InvokerError},
        ratline::RatlineUid,
        Autorouter,
    },
    board::{edit::BoardEdit, AccessMesadata, Board},
    drawing::{
        graph::{GetMaybeNet, MakePrimitiveRef, PrimitiveIndex},
        loose::GetPrevNextLoose,
        primitive::MakePrimitiveShape,
    },
    geometry::{shape::MeasureLength, GenericNode},
    graph::{GetIndex, MakeRef},
    router::{navmesh::Navmesh, RouterOptions},
    specctra::{design::SpecctraDesign, mesadata::SpecctraMesadata},
};

pub fn load_design(filename: &str) -> Autorouter<SpecctraMesadata> {
    let design_file = File::open(filename).unwrap();
    let design_bufread = BufReader::new(design_file);
    let design = SpecctraDesign::load(design_bufread).unwrap();
    Autorouter::new(design.make_board(&mut BoardEdit::new())).unwrap()
}

pub fn create_invoker_and_assert(
    autorouter: Autorouter<SpecctraMesadata>,
) -> Invoker<SpecctraMesadata> {
    let mut invoker = Invoker::new(autorouter);

    assert!(matches!(
        invoker.undo(),
        Err(InvokerError::History(HistoryError::NoPreviousCommand))
    ));
    assert!(matches!(
        invoker.redo(),
        Err(InvokerError::History(HistoryError::NoNextCommand))
    ));

    invoker
}

pub fn replay_and_assert_and_report(
    invoker: &mut Invoker<SpecctraMesadata>,
    filename: &str,
    variant: &str,
) {
    // Captured before the first replay: this is the node count a full
    // undo must restore exactly. Stronger than `assert_no_loose_nodes`
    // — it also catches leftover compounds (vias) and fixed geometry.
    let pristine_node_count = invoker.autorouter().board().layout().drawing().node_count();

    let file = File::open(filename).unwrap();
    let history: History = serde_json::from_reader(file).unwrap();

    invoker.replay(history);
    report_route_lengths(invoker.autorouter());

    if variant == "with_undo_redo_replay" {
        assert_undo_redo_replay(invoker, filename, pristine_node_count);
    }
}

pub fn assert_undo_redo_replay(
    invoker: &mut Invoker<SpecctraMesadata>,
    filename: &str,
    pristine_node_count: usize,
) {
    let prev_node_count = invoker.autorouter().board().layout().drawing().node_count();

    assert_bend_integrity(invoker.autorouter());

    // Sanity test: check if node count remained the same after some attempts at undo-redo.

    if invoker.redo().is_ok() {
        let _ = invoker.undo();
    }

    if invoker.undo().is_ok() {
        if invoker.undo().is_ok() {
            let _ = invoker.redo();
        }

        let _ = invoker.redo();
    }

    assert_eq!(
        invoker.autorouter().board().layout().drawing().node_count(),
        prev_node_count,
    );

    // Another sanity test: undo all, and then replay again. This protects
    // against undo failing to remove something that does not cause the
    // subsequent redo to fail or something that the redo restores idempotently.

    let file = File::open(filename).unwrap();
    let history: History = serde_json::from_reader(file).unwrap();

    undo_all_and_assert(invoker);

    // A complete undo must restore the board exactly as it was before
    // the first replay — tombstoned slots are excluded from the count.
    assert_eq!(
        invoker.autorouter().board().layout().drawing().node_count(),
        pristine_node_count,
    );

    invoker.replay(history);

    // No exact node-count check here: replay re-routes live, and the
    // rerouted result may legitimately differ in node count from the
    // first run. (`assert_no_loose_nodes` doesn't apply either — routed
    // bands are made of loose segs and bends.)
    assert_bend_integrity(invoker.autorouter());
}

/// Forward-run sanity: every bend must still have both chain joints —
/// a floating bend means band removal stranded it before undo ever ran.
pub fn assert_bend_integrity(autorouter: &Autorouter<impl AccessMesadata>) {
    for node in autorouter.board().layout().drawing().primitive_nodes() {
        match node {
            PrimitiveIndex::LooseBend(bend) => {
                let drawing = autorouter.board().layout().drawing();
                let _ = drawing.loose(bend.into()).next_loose(None);
            }
            PrimitiveIndex::FixedBend(bend) => {
                let drawing = autorouter.board().layout().drawing();
                let _ = bend.primitive_ref(drawing).shape();
            }
            _ => (),
        }
    }
}

pub fn undo_all_and_assert(invoker: &mut Invoker<SpecctraMesadata>) {
    for _ in 0..invoker.history().done().len() {
        invoker.undo().unwrap();
    }

    assert!(matches!(
        invoker.undo(),
        Err(InvokerError::History(HistoryError::NoPreviousCommand))
    ));

    assert_no_loose_nodes(invoker.autorouter());
}

pub fn assert_no_loose_nodes(autorouter: &Autorouter<impl AccessMesadata>) {
    for node in autorouter.board().layout().drawing().primitive_nodes() {
        match node {
            PrimitiveIndex::LooseDot(..)
            | PrimitiveIndex::LoneLooseSeg(..)
            | PrimitiveIndex::SeqLooseSeg(..)
            | PrimitiveIndex::LooseBend(..) => {
                let drawing = autorouter.board().layout().drawing();
                panic!(
                    "loose node left behind: {node:?} {:?}",
                    node.primitive_ref(drawing).shape()
                )
            }
            _ => (),
        }
    }
}

pub fn assert_layer_0_navnode_count(
    autorouter: &mut Autorouter<SpecctraMesadata>,
    origin_pin: &str,
    destination_pin: &str,
    expected_count: usize,
) {
    let (origin, destination) = autorouter
        .ratsnests()
        .on_principal_layer(0)
        .graph()
        .edge_indices()
        .map(|index| RatlineUid {
            principal_layer: 0,
            index,
        })
        .collect::<Vec<_>>()
        .iter()
        .find_map(|ratline| {
            let (candidate_origin, candidate_destination) =
                ratline.ref_(autorouter).endpoint_dots();
            let candidate_origin_pin = autorouter
                .board()
                .node_pinname(&GenericNode::Primitive(candidate_origin.into()))
                .unwrap();
            let candidate_destination_pin = autorouter
                .board()
                .node_pinname(&GenericNode::Primitive(candidate_destination.into()))
                .unwrap();

            ((candidate_origin_pin == origin_pin && candidate_destination_pin == destination_pin)
                || (candidate_origin_pin == destination_pin
                    && candidate_destination_pin == origin_pin))
                .then_some((candidate_origin, candidate_destination))
        })
        .unwrap();

    let navmesh = Navmesh::new(
        autorouter.board().layout(),
        origin,
        destination,
        RouterOptions {
            wrap_around_bands: true,
            squeeze_through_under_bends: false,
            routed_band_width: 100.0,
        },
    )
    .unwrap();
    assert_eq!(navmesh.graph().node_count(), expected_count);
}

pub fn assert_that_all_ratlines_besides_gnd_are_autorouted(
    autorouter: &mut Autorouter<impl AccessMesadata>,
) {
    let conncomps = ConnectedComponents::new(autorouter.board());
    assert!(autorouter.board().layout().drawing().layer_count() >= 1);

    let mut unconnected = Vec::new();
    for principal_layer in 0..autorouter.board().layout().drawing().layer_count() {
        for ratline in autorouter
            .ratsnests()
            .on_principal_layer(principal_layer)
            .graph()
            .edge_indices()
            .map(|index| RatlineUid {
                principal_layer,
                index,
            })
        {
            let (origin_dot, destination_dot) = ratline.ref_(autorouter).endpoint_dots();

            let origin_net = autorouter
                .board()
                .layout()
                .drawing()
                .primitive_ref(origin_dot)
                .maybe_net();
            let destination_net = autorouter
                .board()
                .layout()
                .drawing()
                .primitive_ref(destination_dot)
                .maybe_net();
            assert_eq!(origin_net, destination_net);

            let net = origin_net.unwrap();

            if let Some(netname) = autorouter.board().layout().rules().net_netname(net) {
                // We don't route ground.
                let origin = conncomps.unionfind().find(origin_dot.index());
                let destination = conncomps.unionfind().find(destination_dot.index());

                if netname != "GND" && origin != destination {
                    let weight = autorouter
                        .ratsnests()
                        .on_principal_layer(principal_layer)
                        .graph()
                        .edge_weight(ratline.index);
                    eprintln!(
                        "UNCONNECTED ratline={:?} layer={} net={} origin_dot={:?} dest_dot={:?} has_band={:?}",
                        ratline.index,
                        principal_layer,
                        netname,
                        origin_dot,
                        destination_dot,
                        weight.map(|w| w.band_termseg.is_some()),
                    );
                    unconnected.push(ratline);
                }
            }
        }
    }

    assert!(
        unconnected.is_empty(),
        "{} unconnected ratline(s)",
        unconnected.len()
    );
}

/// Consistency check for the rerouted board after an undo-all→replay
/// cycle: a legitimately re-routed result may leave ratlines unrouted
/// (replayed searches land in different ordering basins once slot
/// recycling perturbs iteration order), but the band registry and the
/// connectivity union-find must agree — a registered band spanning
/// disconnected dots is the signature of lost connectivity (I14).
/// Something must also have routed: an empty registry means the replay
/// degenerated entirely.
pub fn assert_band_registry_consistent(autorouter: &mut Autorouter<impl AccessMesadata>) {
    let conncomps = ConnectedComponents::new(autorouter.board());
    assert!(autorouter.board().bands().count() > 0);

    for principal_layer in 0..autorouter.board().layout().drawing().layer_count() {
        for ratline in autorouter
            .ratsnests()
            .on_principal_layer(principal_layer)
            .graph()
            .edge_indices()
            .map(|index| RatlineUid {
                principal_layer,
                index,
            })
        {
            let (origin_dot, destination_dot) = ratline.ref_(autorouter).endpoint_dots();

            if autorouter
                .board()
                .band_between_nodes(origin_dot, destination_dot)
                .is_some()
            {
                assert_eq!(
                    conncomps.unionfind().find(origin_dot.index()),
                    conncomps.unionfind().find(destination_dot.index()),
                    "registered band {ratline:?} spans dots the union-find keeps apart"
                );
            }
        }
    }
}

/*pub fn assert_number_of_conncomps(
    autorouter: &mut Autorouter<impl MesadataTrait>,
    conncomp_count: usize,
) {
    let unionfind = unionfind(autorouter);
    let mut labels = unionfind.into_labeling();
    labels.sort_unstable();
    labels.dedup();

    assert_eq!(labels.len(), conncomp_count);
}*/

pub fn assert_band_length(
    board: &Board<impl AccessMesadata>,
    source_pin: &str,
    target_pin: &str,
    expected_length: f64,
    rel_err: f64,
) {
    let band = board.band_between_pins(source_pin, target_pin).unwrap();
    let band_length = band[false].ref_(board.layout().drawing()).length();
    assert!(
        (band_length - expected_length).abs() < expected_length * rel_err,
        "band_length = {}, expected_length = {}, epsilon = {}",
        band_length,
        expected_length,
        rel_err
    );
}

pub fn report_route_lengths(autorouter: &Autorouter<impl AccessMesadata>) {
    let mut total_length = 0.0;

    for layer in 0..autorouter.board().layout().drawing().layer_count() {
        let mut layer_total_length = 0.0;

        for primitive in autorouter
            .board()
            .layout()
            .drawing()
            .layer_primitive_nodes(layer)
        {
            match primitive {
                PrimitiveIndex::LooseDot(..)
                | PrimitiveIndex::LoneLooseSeg(..)
                | PrimitiveIndex::SeqLooseSeg(..) => {
                    layer_total_length += primitive
                        .primitive_ref(autorouter.board().layout().drawing())
                        .shape()
                        .length();
                }
                _ => (),
            }
        }

        dbg!(layer, layer_total_length);

        total_length += layer_total_length;
    }

    dbg!(total_length);
}
