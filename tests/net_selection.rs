// SPDX-FileCopyrightText: 2024 Topola contributors
//
// SPDX-License-Identifier: MIT

use std::collections::BTreeSet;

use topola::{
    autorouter::selection::PinSelection,
    board::AccessMesadata,
    drawing::graph::{GetMaybeNet, MakePrimitiveRef},
    geometry::GenericNode,
    triangulation::GetTrianvertexNodeIndex,
};

mod common;

#[test]
fn select_pins_by_netname() {
    let autorouter = common::load_design("tests/unilayer/0603_breakout/0603_breakout.dsn");
    let board = autorouter.board();

    let named = BTreeSet::from(["Net-(J1-Pin_1)".to_string()]);
    assert!(PinSelection::new_select_nets(board, &named)
        .selectors()
        .next()
        .is_some());

    let absent = BTreeSet::from(["NoSuchNet".to_string()]);
    assert!(PinSelection::new_select_nets(board, &absent)
        .selectors()
        .next()
        .is_none());
}

#[test]
fn exclude_pins_by_netname() {
    let autorouter = common::load_design("tests/unilayer/0603_breakout/0603_breakout.dsn");
    let board = autorouter.board();

    let all = PinSelection::new_select_all_except_nets(board, &BTreeSet::new());
    let without_named = PinSelection::new_select_all_except_nets(
        board,
        &BTreeSet::from(["Net-(J1-Pin_1)".to_string()]),
    );

    assert!(without_named.selectors().count() < all.selectors().count());
}

/// On partially routed boards ratline endpoints are anonymous wire-end dots
/// without pin names. Net selections must still match them via their net.
#[test]
fn select_anonymous_nodes_by_netname() {
    let autorouter = common::load_design(
        "tests/multilayer/prerouted_lm317_breakout/prerouted_lm317_breakout.dsn",
    );
    let board = autorouter.board();

    let selection = PinSelection::new_select_nets(
        board,
        &BTreeSet::from(["/wejście".to_string(), "/wyjście".to_string()]),
    );

    let anonymous_node = board
        .layout()
        .drawing()
        .primitive_nodes()
        .find(|node| {
            node.primitive_ref(board.layout().drawing())
                .maybe_net()
                .and_then(|net| board.mesadata().net_netname(net))
                .is_some_and(|netname| netname == "/wejście")
                && board.node_pinname(&GenericNode::Primitive(*node)).is_none()
        })
        .expect("fixture has an anonymous node on net /wejście");

    assert!(selection.contains_node(board, GenericNode::Primitive(anonymous_node)));
}

/// Net selections must produce ratlines on boards whose ratsnest endpoints
/// are anonymous dots, rather than ending up with nothing to route.
#[test]
fn select_ratlines_with_anonymous_endpoints() {
    let autorouter = common::load_design(
        "tests/multilayer/prerouted_lm317_breakout/prerouted_lm317_breakout.dsn",
    );
    let board = autorouter.board();

    let selection = PinSelection::new_select_nets(
        board,
        &BTreeSet::from([
            "/adj".to_string(),
            "/wejście".to_string(),
            "/wyjście".to_string(),
        ]),
    );

    let ratsnest = autorouter.ratsnests().on_principal_layer(0);
    let mut selected_ratlines = 0;
    let mut selected_with_anonymous_endpoint = 0;

    for edge in ratsnest.graph().edge_indices() {
        let (source, target) = ratsnest.graph().edge_endpoints(edge).unwrap();
        let source_node: topola::layout::NodeIndex = ratsnest
            .graph()
            .node_weight(source)
            .unwrap()
            .node_index()
            .into();
        let target_node: topola::layout::NodeIndex = ratsnest
            .graph()
            .node_weight(target)
            .unwrap()
            .node_index()
            .into();

        if !(selection.contains_node(board, source_node)
            && selection.contains_node(board, target_node))
        {
            continue;
        }
        selected_ratlines += 1;

        if board.node_pinname(&source_node).is_none() || board.node_pinname(&target_node).is_none()
        {
            selected_with_anonymous_endpoint += 1;
        }
    }

    assert!(selected_ratlines > 0, "expected selected ratlines");
    assert!(
        selected_with_anonymous_endpoint > 0,
        "expected at least one selected ratline with an anonymous endpoint"
    );
}

#[test]
fn deserialize_legacy_pin_selection() {
    let selection: PinSelection =
        serde_json::from_str(r#"[{"pin": "R1-2", "layer": "F.Cu"}]"#).unwrap();

    assert_eq!(selection.selectors().count(), 1);
    assert!(selection.net_names.is_empty());
    assert!(!selection.nets_excluded);
}

#[test]
fn deserialize_extended_pin_selection() {
    let selection: PinSelection = serde_json::from_str(
        r#"{"pins": [{"pin": "R1-2", "layer": "F.Cu"}], "net_names": ["GND"], "nets_excluded": true}"#,
    )
    .unwrap();

    assert_eq!(selection.selectors().count(), 1);
    assert_eq!(selection.net_names, BTreeSet::from(["GND".to_string()]));
    assert!(selection.nets_excluded);
}

/// Pin-only selections keep serializing as the legacy bare array so `.cmd`
/// files stay compatible with older versions.
#[test]
fn serialize_pin_selection() {
    let autorouter = common::load_design("tests/unilayer/0603_breakout/0603_breakout.dsn");
    let pin_only = PinSelection::new_select_nets(
        autorouter.board(),
        &BTreeSet::from(["Net-(J1-Pin_1)".to_string()]),
    );
    let mut pin_only = pin_only;
    pin_only.net_names.clear();

    assert!(serde_json::to_value(&pin_only).unwrap().is_array());

    let with_nets = PinSelection::new_select_nets(
        autorouter.board(),
        &BTreeSet::from(["Net-(J1-Pin_1)".to_string()]),
    );
    let value = serde_json::to_value(&with_nets).unwrap();
    assert_eq!(value["net_names"], serde_json::json!(["Net-(J1-Pin_1)"]));
}
