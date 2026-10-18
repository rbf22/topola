// SPDX-FileCopyrightText: 2024 Topola contributors
//
// SPDX-License-Identifier: MIT

use std::time::Duration;

use topola::{
    autorouter::{execution::Command, selection::PinSelection, PlanarAutorouteOptions, PresortBy},
    board::Board,
    router::RouterOptions,
    specctra::mesadata::SpecctraMesadata,
    stepper::TimeoutOptions,
};

mod common;

fn autoroute_command(board: &Board<SpecctraMesadata>) -> Command {
    Command::Autoroute(
        PinSelection::new_select_layer(board, 0),
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
                initial: 1.0,
                progress_bonus: 0.005,
            },
            max_reconfigurations: None,
            improve_pad_entry: false,
            pad_entry_min_angle: std::f64::consts::FRAC_PI_4,
        },
    )
}

#[test]
fn zero_timeout_reverts_in_flight_work() {
    let autorouter = common::load_design("tests/unilayer/0603_breakout/0603_breakout.dsn");
    let mut invoker = common::create_invoker_and_assert(autorouter);

    let initial_node_count = invoker.autorouter().board().layout().drawing().node_count();
    let command = autoroute_command(invoker.autorouter().board());

    let timed_out = invoker
        .execute_with_timeout(command, Some(Duration::ZERO), |_| {})
        .unwrap();
    assert!(timed_out);

    // The in-flight partial band must have been fully reverted.
    assert_eq!(
        invoker.autorouter().board().layout().drawing().node_count(),
        initial_node_count,
    );
    common::assert_no_loose_nodes(invoker.autorouter());

    // The recorded command must undo cleanly.
    invoker.undo().unwrap();
}

#[test]
fn generous_timeout_completes_normally() {
    let autorouter = common::load_design("tests/unilayer/0603_breakout/0603_breakout.dsn");
    let mut invoker = common::create_invoker_and_assert(autorouter);
    let command = autoroute_command(invoker.autorouter().board());

    let timed_out = invoker
        .execute_with_timeout(command, Some(Duration::from_secs(120)), |_| {})
        .unwrap();
    assert!(!timed_out);

    let (mut autorouter, ..) = invoker.dissolve();
    common::assert_that_all_ratlines_besides_gnd_are_autorouted(&mut autorouter);
}

#[test]
fn zero_reconfigurations_completes_normally() {
    let autorouter = common::load_design("tests/unilayer/0603_breakout/0603_breakout.dsn");
    let mut invoker = common::create_invoker_and_assert(autorouter);
    let Command::Autoroute(selection, mut options) =
        autoroute_command(invoker.autorouter().board())
    else {
        unreachable!()
    };
    options.max_reconfigurations = Some(0);

    let timed_out = invoker
        .execute_with_timeout(
            Command::Autoroute(selection, options),
            Some(Duration::from_secs(120)),
            |_| {},
        )
        .unwrap();
    assert!(!timed_out);

    let (mut autorouter, ..) = invoker.dissolve();
    common::assert_that_all_ratlines_besides_gnd_are_autorouted(&mut autorouter);
}

#[test]
fn planar_autoroute_options_deserialize_without_max_reconfigurations() {
    // Older `.cmd` files do not carry `max_reconfigurations`.
    let options: PlanarAutorouteOptions = serde_json::from_str(
        r#"{
            "principal_layer": 0,
            "presort_by": "RatlineIntersectionCountAndLength",
            "permutate": true,
            "router": {
                "wrap_around_bands": true,
                "squeeze_through_under_bends": false,
                "routed_band_width": 100.0
            },
            "timeout": {"initial": 1.0, "progress_bonus": 0.005}
        }"#,
    )
    .unwrap();

    assert_eq!(options.max_reconfigurations, None);
}
