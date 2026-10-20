// SPDX-FileCopyrightText: 2024 Topola contributors
//
// SPDX-License-Identifier: MIT

use rstest::rstest;
use rstest_reuse::{self, *};
use topola::{
    autorouter::{execution::Command, invoker::InvokerError, AutorouterError},
    board::AccessMesadata,
    layout::via::ViaWeight,
    math::Circle,
};

mod common;

#[template]
#[rstest]
#[case::plain("")]
#[case::with_undo_redo_replay("with_undo_redo_replay")]
fn test_master(#[case] variant: &str) {}

#[test]
fn test_unrouted_lm317_breakout() {
    let mut invoker = common::create_invoker_and_assert(common::load_design(
        "tests/multilayer/prerouted_lm317_breakout/unrouted_lm317_breakout.dsn",
    ));

    let result = invoker.execute(Command::PlaceVia(ViaWeight {
        from_layer: 0,
        to_layer: 1,
        circle: Circle {
            pos: [125000.0, -84000.0].into(),
            r: 1000.0,
        },
        maybe_net: Some(1234),
    }));
    assert!(matches!(
        result,
        Err(InvokerError::Autorouter(AutorouterError::CouldNotPlaceVia(
            ..
        )))
    ));
}

/// The second undo's compound removal goes through
/// `ApplyGeometryEdit::apply`, which must reinsert the compound's
/// R-tree envelope when it restores the members — otherwise the
/// removal fails the envelope lookup. Regression test: this used to
/// panic in `remove_compound`.
#[test]
fn test_place_via_undo_redo_undo() {
    let mut invoker = common::create_invoker_and_assert(common::load_design(
        "tests/multilayer/prerouted_lm317_breakout/unrouted_lm317_breakout.dsn",
    ));

    invoker
        .execute(Command::PlaceVia(ViaWeight {
            from_layer: 0,
            to_layer: 1,
            circle: Circle {
                pos: [-2500.0, -1250.0].into(),
                r: 400.0,
            },
            maybe_net: None,
        }))
        .unwrap();

    invoker.undo().unwrap();
    invoker.redo().unwrap();
    invoker.undo().unwrap();

    common::assert_no_loose_nodes(invoker.autorouter());
}

/// Diagnostic for I14: does undo-all restore the ratsnest graph?
/// Continuation vertices/edges added during weaving mutate the ratsnest
/// graph but are not part of the recorded LayoutEdit — check whether
/// they survive a full undo.
#[test]
/// I14 regression (`bench/FINDINGS.md`): after autorouting and undoing
/// everything, the board must be restored exactly — same live
/// primitives slot-for-slot, same compound memberships, same pinname
/// registrations, no leaked bands or stale R-tree entries. (What undo
/// does *not* restore is the ratsnest edge weights: `band_termseg`
/// marks persist, which is why `unrouted_ratlines` reads the board
/// registry instead — see test_probe_channels_after_undo….) Note this
/// says nothing about reroute outcomes: slot recycling perturbs
/// insertion order, so a second replay may legitimately route
/// differently; exact restore is the invariant.
fn test_astable_board_state_restored_exactly_after_undo_all() {
    use std::{fs::File, io::BufReader};
    use topola::autorouter::history::History;

    let mut invoker = common::create_invoker_and_assert(common::load_design(
        "tests/multilayer/astable_multivibrator/astable_multivibrator.dsn",
    ));

    let counts = |invoker: &topola::autorouter::invoker::Invoker<
        topola::specctra::mesadata::SpecctraMesadata,
    >|
     -> Vec<(usize, usize)> {
        (0..invoker
            .autorouter()
            .board()
            .layout()
            .drawing()
            .layer_count())
            .map(|pl| {
                let g = invoker
                    .autorouter()
                    .ratsnests()
                    .on_principal_layer(pl)
                    .graph();
                (g.node_count(), g.edge_count())
            })
            .collect()
    };

    // Count primitive→compound membership edges per primitive —
    // the via link that `fixed_dot_via` reads.
    let compound_membership_count = |invoker: &topola::autorouter::invoker::Invoker<
        topola::specctra::mesadata::SpecctraMesadata,
    >|
     -> usize {
        use topola::graph::GetIndex;
        let drawing = invoker.autorouter().board().layout().drawing();
        drawing
            .primitive_nodes()
            .map(|node| {
                drawing
                    .compounds(topola::graph::GenericIndex::<()>::new(node.index()))
                    .count()
            })
            .sum::<usize>()
    };

    let pin_memberships = |invoker: &topola::autorouter::invoker::Invoker<
        topola::specctra::mesadata::SpecctraMesadata,
    >|
     -> usize {
        invoker
            .autorouter()
            .board()
            .pinnames()
            .map(|p| invoker.autorouter().board().pinname_nodes(p).count())
            .sum()
    };

    let rtree_size = |invoker: &topola::autorouter::invoker::Invoker<
        topola::specctra::mesadata::SpecctraMesadata,
    >|
     -> usize {
        invoker
            .autorouter()
            .board()
            .layout()
            .drawing()
            .rtree()
            .size()
    };
    let live_primitives = |invoker: &topola::autorouter::invoker::Invoker<
        topola::specctra::mesadata::SpecctraMesadata,
    >|
     -> usize {
        invoker
            .autorouter()
            .board()
            .layout()
            .drawing()
            .primitive_nodes()
            .count()
    };

    // Slot-for-slot snapshot: kind + shape + joints + net per live
    // primitive, keyed by graph index. Equal counts can hide a restore
    // that wrote the right node count but wrong geometry.
    let snapshot = |invoker: &topola::autorouter::invoker::Invoker<
        topola::specctra::mesadata::SpecctraMesadata,
    >|
     -> Vec<String> {
        use topola::{
            drawing::graph::{GetMaybeNet, MakePrimitiveRef},
            drawing::primitive::MakePrimitiveShape,
            graph::GetIndex,
        };
        let drawing = invoker.autorouter().board().layout().drawing();
        let mut rows: Vec<String> = drawing
            .primitive_nodes()
            .map(|node| {
                let primitive = node.primitive_ref(drawing);
                format!(
                    "{:?} kind={:?} shape={:?} net={:?}",
                    node.index(),
                    node,
                    primitive.shape(),
                    primitive.maybe_net(),
                )
            })
            .collect();
        rows.sort();
        rows
    };

    let pristine = counts(&invoker);
    let pristine_members = compound_membership_count(&invoker);
    let pristine_pins = pin_memberships(&invoker);
    let pristine_rtree = rtree_size(&invoker);
    let pristine_snapshot = snapshot(&invoker);
    eprintln!("pristine ratsnest (nodes, edges): {pristine:?}");
    eprintln!("pristine compound memberships: {pristine_members}");
    eprintln!("pristine pinname node memberships: {pristine_pins}");
    eprintln!(
        "pristine rtree {} vs live primitives {}",
        rtree_size(&invoker),
        live_primitives(&invoker)
    );

    let file = File::open("tests/multilayer/astable_multivibrator/autoroute_all.cmd").unwrap();
    let history: History = serde_json::from_reader(BufReader::new(file)).unwrap();
    invoker.replay(history);
    eprintln!("routed ratsnest (nodes, edges):   {:?}", counts(&invoker));

    common::undo_all_and_assert(&mut invoker);
    let after = counts(&invoker);
    let after_members = compound_membership_count(&invoker);
    eprintln!("post-undo ratsnest (nodes, edges): {after:?}");
    eprintln!("post-undo compound memberships: {after_members}");
    eprintln!(
        "post-undo bands registered: {}",
        invoker.autorouter().board().bands().count()
    );
    eprintln!(
        "post-undo stale band_termseg marks: {}",
        (0..invoker
            .autorouter()
            .board()
            .layout()
            .drawing()
            .layer_count())
            .map(|pl| invoker
                .autorouter()
                .ratsnests()
                .on_principal_layer(pl)
                .graph()
                .edge_weights()
                .filter(|w| w.band_termseg.is_some())
                .count())
            .sum::<usize>()
    );
    let after_pins = pin_memberships(&invoker);
    eprintln!("post-undo pinname node memberships: {after_pins}");
    eprintln!(
        "post-undo rtree {} vs live primitives {}",
        rtree_size(&invoker),
        live_primitives(&invoker)
    );

    let after_snapshot = snapshot(&invoker);
    let snapshot_diffs: Vec<String> = pristine_snapshot
        .iter()
        .zip(after_snapshot.iter())
        .enumerate()
        .filter_map(|(i, (a, b))| (a != b).then(|| format!("row {i}: {a}  =>  {b}")))
        .take(20)
        .collect();
    eprintln!("post-undo snapshot diffs (first 20): {snapshot_diffs:?}");
    eprintln!(
        "snapshot lengths: {} vs {}",
        pristine_snapshot.len(),
        after_snapshot.len()
    );

    assert_eq!(pristine, after, "ratsnest graph not restored by undo-all");
    assert_eq!(
        pristine_pins, after_pins,
        "pinname memberships lost across undo-all"
    );
    assert_eq!(
        pristine_members, after_members,
        "compound memberships lost across undo-all"
    );
    assert_eq!(
        invoker.autorouter().board().bands().count(),
        0,
        "band registry not cleared by undo-all"
    );
    assert_eq!(
        rtree_size(&invoker),
        pristine_rtree,
        "R-tree not restored by undo-all"
    );
    assert_eq!(
        pristine_snapshot, after_snapshot,
        "live primitives not restored slot-for-slot by undo-all"
    );
}

/// Regression test for stale `band_termseg` marks (`bench/FINDINGS.md`
/// I14 family): the mark lives on the ratsnest edge, outside the edit
/// record, so undoing an autoroute leaves every ratline marked routed.
/// `unrouted_ratlines` must consult the board's band registry — the
/// undo-consistent source `planar_autoroute` itself uses — otherwise
/// post-undo channel probes find nothing to probe.
#[test]
fn test_probe_channels_after_undo_still_finds_ratlines() {
    use topola::{
        autorouter::{
            planar_reconfigurator::DEADLINE_FAILURES_PER_LAYER, selection::PinSelection,
            PlanarAutorouteOptions, PresortBy,
        },
        router::RouterOptions,
        stepper::TimeoutOptions,
    };

    let mut invoker = common::create_invoker_and_assert(common::load_design(
        "tests/multilayer/prerouted_lm317_breakout/unrouted_lm317_breakout.dsn",
    ));

    let total_ratlines = invoker
        .autorouter()
        .ratsnests()
        .on_principal_layer(0)
        .graph()
        .edge_count();

    invoker
        .execute(Command::Autoroute(
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
                deadline_failures_per_layer: DEADLINE_FAILURES_PER_LAYER,
            },
        ))
        .unwrap();
    assert!(
        invoker.autorouter().board().bands().count() > 0,
        "fixture must route at least one band for this test to mean anything"
    );

    invoker.undo().unwrap();
    assert_eq!(invoker.autorouter().board().bands().count(), 0);

    let probes = invoker.autorouter_mut().probe_channels(
        RouterOptions {
            wrap_around_bands: true,
            squeeze_through_under_bends: false,
            routed_band_width: 100.0,
        },
        |rules| rules.override_clearances(0.0),
    );

    assert_eq!(
        probes.len(),
        total_ratlines,
        "post-undo probes must cover every ratline — stale band_termseg marks hid them"
    );
}

#[apply(test_master)]
fn test_astable_multivibrator(variant: &str) {
    let mut invoker = common::create_invoker_and_assert(common::load_design(
        "tests/multilayer/astable_multivibrator/astable_multivibrator.dsn",
    ));

    common::replay_and_assert_and_report(
        &mut invoker,
        "tests/multilayer/astable_multivibrator/autoroute_all.cmd",
        variant,
    );

    let (mut autorouter, ..) = invoker.dissolve();
    common::assert_that_all_ratlines_besides_gnd_are_autorouted(&mut autorouter);
}

#[test]
fn test_signal_integrity_test() {
    let invoker = common::create_invoker_and_assert(common::load_design(
        "tests/multilayer/signal_integrity_test/signal_integrity_test.dsn",
    ));

    assert_eq!(
        invoker
            .autorouter()
            .board()
            .layout()
            .drawing()
            .layer_count(),
        4
    );

    for layer in 0..invoker
        .autorouter()
        .board()
        .layout()
        .drawing()
        .layer_count()
    {
        let layername = invoker
            .autorouter()
            .board()
            .mesadata()
            .layer_layername(layer);

        if layer == 0 {
            assert_eq!(layername, Some("F.Cu"));
        } else if layer == 1 {
            assert_eq!(layername, Some("In1.Cu"));
        } else if layer == 2 {
            assert_eq!(layername, Some("In2.Cu"));
        } else if layer == 3 {
            assert_eq!(layername, Some("B.Cu"));
        }
    }
}
