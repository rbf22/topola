// SPDX-FileCopyrightText: 2026 Topola contributors
//
// SPDX-License-Identifier: MIT

use rstest::rstest;
use rstest_reuse::{self, *};
use topola::autorouter::{
    anneal::{AnnealMode, AnnealOptions},
    multilayer_autoroute::MultilayerAutorouteOptions,
};

mod common;

#[template]
#[rstest]
#[case::plain("")]
#[case::with_undo_redo_replay("with_undo_redo_replay")]
fn test_master(#[case] variant: &str) {}

/// Continuous-mode annealing end-to-end: the pre-planner commits
/// layer assignments, order, congestion, and any planned mid-route
/// vias; the existing router legalizes them. The undo/redo variant
/// also checks planned vias and carved corridors reverse and restore
/// with the command's history.
#[apply(test_master)]
fn test_astable_multivibrator_anneal_continuous(variant: &str) {
    let mut invoker = common::create_invoker_and_assert(common::load_design(
        "tests/multilayer/astable_multivibrator/astable_multivibrator.dsn",
    ));

    common::replay_and_assert_and_report(
        &mut invoker,
        "tests/multilayer/astable_multivibrator/autoroute_all_anneal_continuous.cmd",
        variant,
    );

    let (mut autorouter, ..) = invoker.dissolve();
    common::assert_that_all_ratlines_besides_gnd_are_autorouted(&mut autorouter);
}

/// The discrete comparison baseline: same handoff shape, no
/// continuous curves or planned splits.
#[apply(test_master)]
fn test_astable_multivibrator_anneal_discrete(variant: &str) {
    let mut invoker = common::create_invoker_and_assert(common::load_design(
        "tests/multilayer/astable_multivibrator/astable_multivibrator.dsn",
    ));

    common::replay_and_assert_and_report(
        &mut invoker,
        "tests/multilayer/astable_multivibrator/autoroute_all_anneal_discrete.cmd",
        variant,
    );

    let (mut autorouter, ..) = invoker.dissolve();
    common::assert_that_all_ratlines_besides_gnd_are_autorouted(&mut autorouter);
}

/// Same command executed on two identical boards produces identical
/// geometry — the annealer is a pure function of its seed.
#[test]
fn annealed_routing_is_deterministic() {
    let node_count_of = |command_path: &str| -> usize {
        let mut invoker = common::create_invoker_and_assert(common::load_design(
            "tests/multilayer/astable_multivibrator/astable_multivibrator.dsn",
        ));
        common::replay_and_assert_and_report(&mut invoker, command_path, "");
        invoker.autorouter().board().layout().drawing().node_count()
    };
    let first =
        node_count_of("tests/multilayer/astable_multivibrator/autoroute_all_anneal_continuous.cmd");
    let second =
        node_count_of("tests/multilayer/astable_multivibrator/autoroute_all_anneal_continuous.cmd");
    assert_eq!(first, second);
}

/// Older `.cmd` files carry no `anneal` field — they must keep
/// deserializing with annealing off.
#[test]
fn multilayer_options_deserialize_without_anneal() {
    let options: MultilayerAutorouteOptions = serde_json::from_str(
        r#"{
            "anterouter": {"fanout_clearance": 200.0},
            "planar": {
                "principal_layer": 0,
                "presort_by": "RatlineIntersectionCountAndLength",
                "permutate": true,
                "router": {
                    "routed_band_width": 100.0,
                    "wrap_around_bands": true,
                    "squeeze_through_under_bends": true
                },
                "timeout": {"initial": 1.0, "progress_bonus": 0.005}
            },
            "timeout": {"initial": 5.0, "progress_bonus": 0.5}
        }"#,
    )
    .unwrap();

    assert!(options.anneal.is_none());
}

/// A sparse anneal block fills in the calibrated defaults.
#[test]
fn anneal_options_deserialize_with_defaults() {
    let options: MultilayerAutorouteOptions = serde_json::from_str(
        r#"{
            "anterouter": {"fanout_clearance": 200.0},
            "planar": {
                "principal_layer": 0,
                "presort_by": "RatlineIntersectionCountAndLength",
                "permutate": true,
                "router": {
                    "routed_band_width": 100.0,
                    "wrap_around_bands": true,
                    "squeeze_through_under_bends": true
                },
                "timeout": {"initial": 1.0, "progress_bonus": 0.005}
            },
            "timeout": {"initial": 5.0, "progress_bonus": 0.5},
            "anneal": {"seed": 42}
        }"#,
    )
    .unwrap();

    let anneal = options.anneal.unwrap();
    assert_eq!(anneal.mode, AnnealMode::Continuous);
    assert_eq!(anneal.seed, 42);
    assert_eq!(anneal.iterations, AnnealOptions::default().iterations);
    assert_eq!(anneal.w_repulsion, AnnealOptions::default().w_repulsion);
}
