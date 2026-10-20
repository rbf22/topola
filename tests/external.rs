// SPDX-FileCopyrightText: 2026 Topola contributors
//
// SPDX-License-Identifier: MIT

//! Tests over third-party board fixtures (see `tests/external/README.md`
//! for provenance and regeneration instructions).

use std::time::Duration;

use rstest::rstest;
use rstest_reuse::{self, *};
use topola::{
    autorouter::Autorouter,
    autorouter::{
        connected_components::ConnectedComponents, execution::Command, ratline::RatlineUid,
        selection::PinSelection, PlanarAutorouteOptions, PresortBy,
    },
    board::AccessMesadata,
    graph::{GetIndex, MakeRef},
    router::RouterOptions,
    stepper::{Step, TimeoutOptions},
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

/// Regression test for the lvds2tmds prenavmesh panic
/// (`bench/FINDINGS.md` I1): `intersects_constraint` resolved its
/// endpoints by *position*, and spade's `locate_with_hint_option_core`
/// could walk forever on this board's float-noise pad coordinates
/// ("Failed to locate position"). The invoker catches that panic and
/// reports it as `timed_out`, so completing without timing out is the
/// pass condition. Fixed by driving the intersection iterator with
/// vertex handles (`can_add_constraint`), which never locates.
#[test]
fn autoroute_external_lvds2tmds_does_not_panic() {
    let autorouter = common::load_design("tests/external/pcbench/lvds2tmds/lvds2tmds.dsn");
    let mut invoker = common::create_invoker_and_assert(autorouter);

    let command = Command::Autoroute(
        PinSelection::new_select_layer(invoker.autorouter().board(), 0),
        PlanarAutorouteOptions {
            principal_layer: 0,
            presort_by: PresortBy::RatlineIntersectionCountAndLength,
            permutate: true,
            reconfigure_by: Default::default(),
            congestion_penalty: 500.0,
            router: RouterOptions {
                wrap_around_bands: true,
                squeeze_through_under_bends: false,
                routed_band_width: 100.0,
            },
            timeout: TimeoutOptions {
                initial: f64::INFINITY,
                progress_bonus: 0.0,
            },
            max_reconfigurations: None,
            improve_pad_entry: false,
            pad_entry_min_angle: std::f64::consts::FRAC_PI_4,
            escape_stubs: false,
            lazy_escape_stubs: false,
            per_ratline_step_limit: None,
            navmesh_step_factor: None,
            surgical_ripup: false,
            shuffle_seed: 0,
        },
    );

    let timed_out = invoker
        .execute_with_timeout(
            command,
            Some(Duration::from_secs(120)),
            Some(30_000),
            || false,
            |_| {},
        )
        .unwrap();
    assert!(!timed_out);
}

/// Regression test for the surgical-rip-up tombstone panic
/// (`bench/FINDINGS.md` I11): `pending_edit` merges a ripped band's
/// creation and removal records into a no-op so undo does not
/// resurrect it — but a surviving bend's `old.inner` could still
/// reference the ripped bend's slot, and reversed replay then tried
/// to reattach to a removal tombstone ("stale edit: reattaching
/// bend … which holds a removal tombstone"). The panic fires inside
/// `finish_early` when the step cap is hit, so this drives
/// `execute_stepper` manually without `catch_unwind`: any panic in
/// `step` or `finish_early` propagates and fails the test. Fixed by
/// skipping restore reattaches whose inner the merged edit leaves
/// dead.
#[test]
fn autoroute_external_relaymodule_surgical_ripup_does_not_panic() {
    let autorouter = common::load_design("tests/external/freerouting/relaymodule/relaymodule.dsn");
    let mut invoker = common::create_invoker_and_assert(autorouter);

    // Mirror `topola --multilayer --surgical-ripup --deterministic
    // --seed 0 --max-steps 30000`: all-pins selection, rule-derived
    // band width, and multilayer's bend-squeeze.
    let band_width = {
        let declared = invoker
            .autorouter()
            .board()
            .mesadata()
            .structure_rule()
            .width;
        if declared > 0.0 {
            declared
        } else {
            100.0
        }
    };
    let command = Command::MultilayerAutoroute(
        PinSelection::new_select_all_except_nets(
            invoker.autorouter().board(),
            &std::collections::BTreeSet::new(),
        ),
        topola::autorouter::multilayer_autoroute::MultilayerAutorouteOptions {
            anterouter: topola::autorouter::anterouter::AnterouterOptions {
                fanout_clearance: 200.0,
                fanout_rings: 0,
            },
            planar: PlanarAutorouteOptions {
                principal_layer: 0,
                presort_by: PresortBy::RatlineIntersectionCountAndLength,
                permutate: true,
                reconfigure_by: Default::default(),
                congestion_penalty: 500.0,
                router: RouterOptions {
                    wrap_around_bands: true,
                    squeeze_through_under_bends: true,
                    routed_band_width: band_width,
                },
                timeout: TimeoutOptions {
                    initial: f64::INFINITY,
                    progress_bonus: 0.0,
                },
                max_reconfigurations: None,
                improve_pad_entry: false,
                pad_entry_min_angle: std::f64::consts::FRAC_PI_4,
                escape_stubs: false,
                lazy_escape_stubs: false,
                per_ratline_step_limit: None,
                navmesh_step_factor: None,
                surgical_ripup: true,
                shuffle_seed: 0,
            },
            timeout: TimeoutOptions {
                initial: f64::INFINITY,
                progress_bonus: 0.0,
            },
            layer_seed: 0,
            anneal: None,
        },
    );

    let mut stepper = invoker.execute_stepper(command).unwrap();
    let mut broke = false;

    for _ in 0..30_000 {
        if stepper.step(&mut invoker).unwrap().is_break() {
            broke = true;
            break;
        }
    }
    // The board must still have live work at the cap, otherwise the
    // `finish_early` path below is never exercised.
    assert!(!broke, "board drained before the step cap");
    // `finish_early` while a surgical rip-up's merged pending edit is
    // still live is where the tombstone reattach used to panic.
    stepper.finish_early(invoker.autorouter_mut());
    loop {
        if stepper.step(&mut invoker).unwrap().is_break() {
            break;
        }
    }
}

/// Regression test for the stale `layer_map` panic
/// (`bench/FINDINGS.md` I11): when a ratline's planned-layer fanout
/// fails, the initial anteroute falls back to the preferred layer —
/// but `plan.layer_map` kept the planned assignment. A later
/// `weave_ratline` then anchored the parent on the phantom layer and
/// retargeted its destination at a via dot on that layer while the
/// origin still resolved to a pad dot on another, tripping the
/// navmesh same-layer assert. Fixed by recording the layer the
/// fanout actually achieved into `layer_map`. Driven manually
/// without `catch_unwind` so the assert propagates.
#[test]
fn autoroute_external_arf154_multilayer_anneal_does_not_panic() {
    let autorouter = common::load_design("tests/external/pcbench/arf154/arf154.dsn");
    let mut invoker = common::create_invoker_and_assert(autorouter);

    // Mirror `topola --multilayer --anneal --anneal-mode discrete
    // --surgical-ripup --deterministic --seed 0 --max-steps 30000`.
    let band_width = {
        let declared = invoker
            .autorouter()
            .board()
            .mesadata()
            .structure_rule()
            .width;
        if declared > 0.0 {
            declared
        } else {
            100.0
        }
    };
    let command = Command::MultilayerAutoroute(
        PinSelection::new_select_all_except_nets(
            invoker.autorouter().board(),
            &std::collections::BTreeSet::new(),
        ),
        topola::autorouter::multilayer_autoroute::MultilayerAutorouteOptions {
            anterouter: topola::autorouter::anterouter::AnterouterOptions {
                fanout_clearance: 200.0,
                fanout_rings: 0,
            },
            planar: PlanarAutorouteOptions {
                principal_layer: 0,
                presort_by: PresortBy::RatlineIntersectionCountAndLength,
                permutate: true,
                reconfigure_by: Default::default(),
                congestion_penalty: 500.0,
                router: RouterOptions {
                    wrap_around_bands: true,
                    squeeze_through_under_bends: true,
                    routed_band_width: band_width,
                },
                timeout: TimeoutOptions {
                    initial: f64::INFINITY,
                    progress_bonus: 0.0,
                },
                max_reconfigurations: None,
                improve_pad_entry: false,
                pad_entry_min_angle: std::f64::consts::FRAC_PI_4,
                escape_stubs: false,
                lazy_escape_stubs: false,
                per_ratline_step_limit: None,
                navmesh_step_factor: None,
                surgical_ripup: true,
                shuffle_seed: 0,
            },
            timeout: TimeoutOptions {
                initial: f64::INFINITY,
                progress_bonus: 0.0,
            },
            layer_seed: 0,
            anneal: Some(topola::autorouter::anneal::AnnealOptions {
                mode: topola::autorouter::anneal::AnnealMode::Discrete,
                seed: 0,
                iterations: 300,
                carve_top_k: 0,
                ..Default::default()
            }),
        },
    );

    let mut stepper = invoker.execute_stepper(command).unwrap();

    for _ in 0..30_000 {
        if stepper.step(&mut invoker).unwrap().is_break() {
            break;
        }
    }
    stepper.finish_early(invoker.autorouter_mut());
    loop {
        if stepper.step(&mut invoker).unwrap().is_break() {
            break;
        }
    }
}

/// Regression test for the load-time rstar panic on EasyEDA exports
/// (`bench/FINDINGS.md` I7 + I12): this file quotes `host_cad`
/// (`"EasyEDA Pro"`) without declaring `string_quote`, and contains
/// zero-length fixed segs whose `corners()` normalization divided by a
/// zero tangent norm — producing NaN envelopes that poisoned the R-tree
/// (`rstar.rs` `partial_cmp` unwrap) during board load. Loading the
/// board is the assertion: both defects live before routing starts.
#[test]
fn load_external_issue684_easyeda_export_does_not_panic() {
    let autorouter = common::load_design("tests/external/freerouting/issue684/issue684.dsn");

    // The board must produce a routable ratline set — proves the
    // geometry graph built fully rather than silently dropping the
    // degenerate segs.
    assert!(autorouter
        .ratsnests()
        .on_principal_layer(0)
        .graph()
        .edge_indices()
        .next()
        .is_some());
}
