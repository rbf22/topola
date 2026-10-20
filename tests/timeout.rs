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
            escape_stubs: false,
            per_ratline_step_limit: None,
            navmesh_step_factor: None,
            surgical_ripup: false,
            shuffle_seed: 0,
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
        .execute_with_timeout(command, Some(Duration::ZERO), None, || false, |_| {})
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
        .execute_with_timeout(
            command,
            Some(Duration::from_secs(120)),
            None,
            || false,
            |_| {},
        )
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
            None,
            || false,
            |_| {},
        )
        .unwrap();
    assert!(!timed_out);

    let (mut autorouter, ..) = invoker.dissolve();
    common::assert_that_all_ratlines_besides_gnd_are_autorouted(&mut autorouter);
}

#[test]
fn max_steps_terminates_early_and_keeps_geometry_clean() {
    let autorouter = common::load_design("tests/unilayer/0603_breakout/0603_breakout.dsn");
    let mut invoker = common::create_invoker_and_assert(autorouter);

    let initial_node_count = invoker.autorouter().board().layout().drawing().node_count();
    let command = autoroute_command(invoker.autorouter().board());

    let mut steps = 0usize;
    let timed_out = invoker
        .execute_with_timeout(command, None, Some(1), || false, |_| steps += 1)
        .unwrap();
    assert!(timed_out);
    assert_eq!(steps, 1);

    // Early finishing must have undone in-flight partial work.
    assert_eq!(
        invoker.autorouter().board().layout().drawing().node_count(),
        initial_node_count,
    );
    common::assert_no_loose_nodes(invoker.autorouter());
    invoker.undo().unwrap();
}

#[test]
fn per_ratline_step_limit_reaches_autoroute_and_keeps_geometry_clean() {
    // End-to-end plumbing check for `PlanarAutorouteOptions::
    // per_ratline_step_limit`: with a 1-step cap no search can ever
    // finish, so a completed run must leave the board untouched. If
    // the option stopped being wired into `RouteStepper` this would
    // route bands instead.
    let autorouter = common::load_design("tests/unilayer/0603_breakout/0603_breakout.dsn");
    let mut invoker = common::create_invoker_and_assert(autorouter);

    let initial_node_count = invoker.autorouter().board().layout().drawing().node_count();
    let Command::Autoroute(selection, mut options) =
        autoroute_command(invoker.autorouter().board())
    else {
        unreachable!()
    };
    options.per_ratline_step_limit = Some(1);
    options.timeout = topola::stepper::TimeoutOptions {
        initial: f64::INFINITY,
        progress_bonus: 0.0,
    };

    let timed_out = invoker
        .execute_with_timeout(
            Command::Autoroute(selection, options),
            Some(Duration::from_secs(120)),
            Some(1_000_000),
            || false,
            |_| {},
        )
        .unwrap();
    assert!(!timed_out);

    assert_eq!(
        invoker.autorouter().board().layout().drawing().node_count(),
        initial_node_count,
    );
    // Nothing was ever routed, so the command produced no history
    // entry to undo.
}

#[test]
fn navmesh_step_factor_scales_cap_and_defers_to_explicit_limit() {
    // `navmesh_step_factor` adapts the cap to each search's size but
    // must never override an explicit `per_ratline_step_limit`.
    let autorouter = common::load_design("tests/unilayer/0603_breakout/0603_breakout.dsn");
    let invoker = common::create_invoker_and_assert(autorouter);
    let Command::Autoroute(_, mut options) = autoroute_command(invoker.autorouter().board()) else {
        unreachable!()
    };

    options.per_ratline_step_limit = None;
    options.navmesh_step_factor = None;
    assert_eq!(options.step_limit_for(1000), None);

    options.navmesh_step_factor = Some(2.5);
    assert_eq!(options.step_limit_for(1000), Some(2500));
    assert_eq!(options.step_limit_for(4), Some(10));

    options.per_ratline_step_limit = Some(7);
    assert_eq!(options.step_limit_for(1000), Some(7));
}

#[test]
fn navmesh_step_factor_reaches_autoroute_and_keeps_geometry_clean() {
    // Same plumbing check as the fixed cap: a zero factor caps every
    // search at zero steps, so a completed run leaves the board
    // untouched — proving the factor is wired into `RouteStepper`.
    let autorouter = common::load_design("tests/unilayer/0603_breakout/0603_breakout.dsn");
    let mut invoker = common::create_invoker_and_assert(autorouter);

    let initial_node_count = invoker.autorouter().board().layout().drawing().node_count();
    let Command::Autoroute(selection, mut options) =
        autoroute_command(invoker.autorouter().board())
    else {
        unreachable!()
    };
    options.navmesh_step_factor = Some(0.0);
    options.timeout = topola::stepper::TimeoutOptions {
        initial: f64::INFINITY,
        progress_bonus: 0.0,
    };

    let timed_out = invoker
        .execute_with_timeout(
            Command::Autoroute(selection, options),
            Some(Duration::from_secs(120)),
            Some(1_000_000),
            || false,
            |_| {},
        )
        .unwrap();
    assert!(!timed_out);

    assert_eq!(
        invoker.autorouter().board().layout().drawing().node_count(),
        initial_node_count,
    );
}

#[test]
fn seeded_run_is_deterministic_across_invokers() {
    // The step-kind sequence and resulting geometry must be a pure
    // function of board + options when the timeout accumulator is
    // frozen, regardless of wall-clock speed.
    use topola::stepper::TimeoutOptions as TO;

    let run = || {
        let autorouter = common::load_design("tests/unilayer/0603_breakout/0603_breakout.dsn");
        let mut invoker = common::create_invoker_and_assert(autorouter);
        let Command::Autoroute(selection, mut options) =
            autoroute_command(invoker.autorouter().board())
        else {
            unreachable!()
        };
        options.timeout = TO {
            initial: f64::INFINITY,
            progress_bonus: 0.0,
        };
        options.shuffle_seed = 7;

        let mut kinds = Vec::new();
        invoker
            .execute_with_timeout(
                Command::Autoroute(selection, options),
                None,
                Some(1000),
                || false,
                |report| kinds.push(report.kind),
            )
            .unwrap();

        (
            kinds,
            invoker.autorouter().board().layout().drawing().node_count(),
        )
    };

    let (kinds_a, nodes_a) = run();
    let (kinds_b, nodes_b) = run();
    assert_eq!(kinds_a, kinds_b);
    assert_eq!(nodes_a, nodes_b);
}

#[test]
fn should_stop_predicate_terminates_early() {
    // Models the --jobs sibling-stop flag: an external predicate ends
    // the run without any timeout or step budget.
    let autorouter = common::load_design("tests/unilayer/0603_breakout/0603_breakout.dsn");
    let mut invoker = common::create_invoker_and_assert(autorouter);

    let initial_node_count = invoker.autorouter().board().layout().drawing().node_count();
    let command = autoroute_command(invoker.autorouter().board());

    let mut polls = 0usize;
    let mut steps = 0usize;
    let timed_out = invoker
        .execute_with_timeout(
            command,
            None,
            None,
            || {
                polls += 1;
                polls >= 3
            },
            |_| steps += 1,
        )
        .unwrap();
    assert!(timed_out);
    // Polled once per step: the predicate trips after step 3.
    assert_eq!(steps, 3);

    assert_eq!(
        invoker.autorouter().board().layout().drawing().node_count(),
        initial_node_count,
    );
    common::assert_no_loose_nodes(invoker.autorouter());
    invoker.undo().unwrap();
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
