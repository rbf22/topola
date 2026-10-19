// SPDX-FileCopyrightText: 2026 Topola contributors
//
// SPDX-License-Identifier: MIT

//! Tests over third-party board fixtures (see `tests/external/README.md`
//! for provenance and regeneration instructions).

use rstest::rstest;
use rstest_reuse::{self, *};
use topola::{
    autorouter::Autorouter,
    autorouter::{connected_components::ConnectedComponents, ratline::RatlineUid},
    board::AccessMesadata,
    graph::{GetIndex, MakeRef},
};

mod common;

#[template]
#[rstest]
#[case::plain("")]
#[case::with_undo_redo_replay("with_undo_redo_replay")]
fn test_master(#[case] variant: &str) {}

/// Counts ratlines whose endpoint dots are in different connected
/// components — the unrouted remainder. Unlike
/// `assert_that_all_ratlines_besides_gnd_are_autorouted` this exempts
/// nothing, but external boards may name their grounds something other
/// than "GND" anyway.
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
            let origin = conncomps.unionfind().find(origin_dot.index());
            let destination = conncomps.unionfind().find(destination_dot.index());
            if origin != destination {
                unconnected += 1;
            }
        }
    }

    unconnected
}

/// PCBench board `4N35-TTL-Serial-Optoisolator`: 14 nets, all
/// through-hole, ~40x23mm, 2 layers. A human-routed reference solution
/// ships alongside as `optoisolator_4n35_solved.dsn` /
/// `optoisolator_4n35_solved.kicad_pcb`. Topola routes all 20 layer-0
/// ratlines without vias.
#[apply(test_master)]
fn autoroute_external_optoisolator_4n35(variant: &str) {
    let autorouter = common::load_design("tests/external/optoisolator_4n35/optoisolator_4n35.dsn");
    let mut invoker = common::create_invoker_and_assert(autorouter);
    common::replay_and_assert_and_report(
        &mut invoker,
        "tests/external/optoisolator_4n35/autoroute_all.cmd",
        variant,
    );

    let (mut autorouter, ..) = invoker.dissolve();
    common::assert_that_all_ratlines_besides_gnd_are_autorouted(&mut autorouter);
}

/// Freerouting fixture `Issue313-FastTest`: a single ~150-pin component
/// with ~20 nets on 2 layers, known to be routable (it ships fully
/// routed upstream — `fasttest_solved.dsn` is the reference, with the
/// `(wiring …)` scope emptied to form the problem).
///
/// Topola does not complete this board within a bounded budget, which
/// makes it a real-world contention fixture for the critical-first
/// work. The command is bounded (small timeouts, few reconfigurations)
/// and the test is `#[ignore]`d so it runs on demand rather than in the
/// default suite: `cargo test --test external -- --ignored`.
#[apply(test_master)]
#[ignore]
fn autoroute_external_fasttest_bounded_leaves_unrouted(variant: &str) {
    let autorouter = common::load_design("tests/external/fasttest/fasttest.dsn");
    let mut invoker = common::create_invoker_and_assert(autorouter);
    common::replay_and_assert_and_report(
        &mut invoker,
        "tests/external/fasttest/autoroute_bounded.cmd",
        variant,
    );

    let (autorouter, ..) = invoker.dissolve();
    let unrouted = count_unrouted_ratlines(&autorouter);
    assert!(
        unrouted >= 1,
        "bounded run unexpectedly routed the whole board ({unrouted} unrouted)"
    );
}
