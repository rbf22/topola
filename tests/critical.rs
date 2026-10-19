// SPDX-FileCopyrightText: 2025 Topola contributors
//
// SPDX-License-Identifier: MIT

use topola::{
    autorouter::{connected_components::ConnectedComponents, ratline::RatlineUid, Autorouter},
    board::AccessMesadata,
    drawing::graph::GetMaybeNet,
    graph::{GetIndex, MakeRef},
};

mod common;

/// Counts non-GND ratlines whose endpoints are not connected — the
/// unrouted remainder. GND is excluded for the same reason as in
/// `assert_that_all_ratlines_besides_gnd_are_autorouted`: the fixture's
/// GND wires pre-connect their net.
fn count_unrouted_ratlines(autorouter: &Autorouter<impl AccessMesadata>) -> usize {
    let conncomps = ConnectedComponents::new(autorouter.board());
    let mut unconnected = 0;

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
            let net = autorouter
                .board()
                .layout()
                .drawing()
                .primitive_ref(origin_dot)
                .maybe_net()
                .unwrap();

            if let Some(netname) = autorouter.board().layout().rules().net_netname(net) {
                if netname == "GND" {
                    continue;
                }

                let origin = conncomps.unionfind().find(origin_dot.index());
                let destination = conncomps.unionfind().find(destination_dot.index());
                if origin != destination {
                    unconnected += 1;
                }
            }
        }
    }

    unconnected
}

/// Baseline: on the critical_corridor board, ordinary routing leaves at
/// least one selected ratline unrouted — the contention the
/// critical-first machinery is designed to fix. This assertion locks in
/// the "hard board" property the fixture exists to provide.
///
/// Single-shot only: the oscillating board's stopping point depends on
/// the wall-clock timeout, so undo/redo/replay is not node-count
/// idempotent here.
#[test]
fn autoroute_critical_corridor_baseline_leaves_unrouted() {
    let autorouter = common::load_design("tests/unilayer/critical_corridor/critical_corridor.dsn");
    let mut invoker = common::create_invoker_and_assert(autorouter);
    common::replay_and_assert_and_report(
        &mut invoker,
        "tests/unilayer/critical_corridor/autoroute_all.cmd",
        "",
    );

    let (autorouter, ..) = invoker.dissolve();
    let unrouted = count_unrouted_ratlines(&autorouter);
    assert!(
        unrouted >= 1,
        "baseline unexpectedly routed the whole board ({unrouted} unrouted)"
    );
}
