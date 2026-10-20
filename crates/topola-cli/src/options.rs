// SPDX-FileCopyrightText: 2024 Topola contributors
//
// SPDX-License-Identifier: MIT

//! Command construction: translates [`Cli`] flags into the selection,
//! planar/multilayer options, and the [`Command`] the invoker runs.
//! Single-run and `--jobs` workers both build through here so the two
//! paths cannot drift apart.

use std::collections::BTreeSet;

use topola::autorouter::anneal::{AnnealMode, AnnealOptions};
use topola::autorouter::anterouter::AnterouterOptions;
use topola::autorouter::critical_autoroute::CriticalAutorouteOptions;
use topola::autorouter::execution::Command;
use topola::autorouter::multilayer_autoroute::MultilayerAutorouteOptions;
use topola::autorouter::planar_reconfigurator::DEADLINE_FAILURES_PER_LAYER;
use topola::autorouter::selection::PinSelection;
use topola::autorouter::{PlanarAutorouteOptions, PresortBy};
use topola::board::AccessMesadata;
use topola::board::Board;
use topola::router::RouterOptions;
use topola::specctra::mesadata::SpecctraMesadata;
use topola::stepper::TimeoutOptions;

use crate::cli::{AnnealModeArg, Cli};

pub fn make_selection(
    board: &Board<SpecctraMesadata>,
    nets: &[String],
    exclude_nets: &[String],
    multilayer: bool,
) -> PinSelection {
    if !nets.is_empty() {
        let nets: BTreeSet<String> = nets.iter().cloned().collect();
        for net in &nets {
            if board.mesadata().netname_net(net).is_none() {
                eprintln!("[topola] warning: net '{}' not found", net);
            }
        }
        let selection = PinSelection::new_select_nets(board, &nets);
        eprintln!(
            "[topola] selected {} pins across {} requested nets",
            selection.selectors().count(),
            nets.len()
        );
        selection
    } else if !exclude_nets.is_empty() {
        let nets: BTreeSet<String> = exclude_nets.iter().cloned().collect();
        let selection = PinSelection::new_select_all_except_nets(board, &nets);
        eprintln!(
            "[topola] selected {} pins excluding {} nets",
            selection.selectors().count(),
            nets.len()
        );
        selection
    } else if multilayer {
        // Multilayer routing can reach pads on any layer — including
        // compound pads and unnamed net-bearing primitives that carry
        // no pin selector. "All except no nets" selects them through
        // the net-membership path in `PinSelection::contains_node`.
        let selection = PinSelection::new_select_all_except_nets(board, &BTreeSet::new());
        eprintln!(
            "[topola] selected {} pins across all layers",
            selection.selectors().count()
        );
        selection
    } else {
        PinSelection::new_select_layer(board, 0)
    }
}

/// Routed band width: the `--band-width` override, else the trace
/// width the design's structure rule declares, else the historical
/// fixed default. The rule-derived default keeps the option in the
/// design's own units — a fixed value is meaningless across boards
/// with different resolutions.
pub fn band_width(args: &Cli, board: &Board<SpecctraMesadata>) -> f64 {
    if let Some(width) = args.band_width {
        return width;
    }

    let declared = board.mesadata().structure_rule().width;
    if declared > 0.0 {
        declared
    } else {
        100.0
    }
}

/// Under `--deterministic` the reconfiguration trigger never fires on
/// elapsed time — routing decisions become a pure function of the
/// board, options, and seeds.
fn timeout_options(deterministic: bool, initial: f64, progress_bonus: f64) -> TimeoutOptions {
    if deterministic {
        TimeoutOptions {
            initial: f64::INFINITY,
            progress_bonus: 0.0,
        }
    } else {
        TimeoutOptions {
            initial,
            progress_bonus,
        }
    }
}

pub fn make_planar_options(args: &Cli, board: &Board<SpecctraMesadata>) -> PlanarAutorouteOptions {
    PlanarAutorouteOptions {
        principal_layer: 0,
        presort_by: if args.corridor_presort {
            PresortBy::RatlineCorridorCongestion
        } else if args.escape_presort {
            PresortBy::RatlineEscapeDensity
        } else {
            PresortBy::RatlineIntersectionCountAndLength
        },
        permutate: true,
        reconfigure_by: Default::default(),
        congestion_penalty: 500.0,
        router: RouterOptions {
            wrap_around_bands: true,
            squeeze_through_under_bends: args.multilayer,
            routed_band_width: band_width(args, board),
        },
        timeout: timeout_options(args.deterministic, 1.0, 0.005),
        max_reconfigurations: args.max_reconfigurations.map(|n| n as usize),
        improve_pad_entry: args.pad_entry,
        pad_entry_min_angle: std::f64::consts::FRAC_PI_4,
        escape_stubs: args.escape_stubs,
        lazy_escape_stubs: args.lazy_escape_stubs,
        per_ratline_step_limit: args.per_ratline_steps.map(|n| n as usize),
        navmesh_step_factor: args.navmesh_step_factor,
        surgical_ripup: args.surgical_ripup,
        shuffle_seed: args.seed.unwrap_or(0),
        deadline_failures_per_layer: DEADLINE_FAILURES_PER_LAYER,
    }
}

/// Fanout clearance: how far past the pad's bounding-box edge the via
/// center is placed. The minimum legal distance scales with the
/// design's via size and clearance — a fixed value is meaningless
/// across boards with different resolutions.
fn fanout_clearance(board: &Board<SpecctraMesadata>) -> f64 {
    let mesadata = board.mesadata();
    mesadata.via_radius() + 1.05 * mesadata.structure_rule().clearance
}

/// `--anneal` flag → `AnnealOptions`: the CLI exposes mode, seed,
/// sweep count, corridor carving, and the quality weights (`w_length`
/// doubles as the layer-deviation penalty in discrete mode — the
/// via-pressure lever — while `w_via`/`min_run_fraction` tune
/// continuous-mode hop cost and hop merging); the rest stay at the
/// calibrated defaults.
pub fn make_anneal_options(args: &Cli) -> Option<AnnealOptions> {
    args.anneal.then(|| AnnealOptions {
        mode: match args.anneal_mode.unwrap_or_default() {
            AnnealModeArg::Continuous => AnnealMode::Continuous,
            AnnealModeArg::Discrete => AnnealMode::Discrete,
        },
        seed: args.anneal_seed.unwrap_or(0),
        iterations: args.anneal_iterations.unwrap_or(300),
        carve_top_k: args.anneal_carve_top_k.unwrap_or(0),
        w_length: args.anneal_w_length.unwrap_or(1.0),
        w_via: args.anneal_w_via.unwrap_or(10.0),
        min_run_fraction: args.anneal_min_run_fraction.unwrap_or(0.15),
        ..AnnealOptions::default()
    })
}

fn make_multilayer_options(
    deterministic: bool,
    planar_options: PlanarAutorouteOptions,
    layer_seed: u64,
    fanout_rings: usize,
    fanout_angles: usize,
    anneal: Option<AnnealOptions>,
    board: &Board<SpecctraMesadata>,
) -> MultilayerAutorouteOptions {
    MultilayerAutorouteOptions {
        anterouter: AnterouterOptions {
            fanout_clearance: fanout_clearance(board),
            fanout_rings,
            fanout_angles,
        },
        planar: planar_options,
        timeout: timeout_options(
            deterministic,
            // Bonus applies per newly-reached ratline index —
            // 0.5s each banks ~2min of budget by mid-list on
            // big boards, starving layer rotation entirely.
            5.0,
            0.05,
        ),
        layer_seed,
        anneal,
    }
}

#[allow(clippy::too_many_arguments)]
pub fn make_command(
    critical_first: bool,
    multilayer: bool,
    deterministic: bool,
    selection: PinSelection,
    planar_options: PlanarAutorouteOptions,
    layer_seed: u64,
    fanout_rings: usize,
    fanout_angles: usize,
    anneal: Option<AnnealOptions>,
    board: &Board<SpecctraMesadata>,
) -> Command {
    if critical_first {
        Command::CriticalAutoroute(
            selection,
            CriticalAutorouteOptions {
                planar: planar_options,
                multilayer: multilayer.then(|| {
                    make_multilayer_options(
                        deterministic,
                        planar_options,
                        layer_seed,
                        fanout_rings,
                        fanout_angles,
                        anneal,
                        board,
                    )
                }),
                critical_quantile: 0.75,
                carve: true,
                protect: true,
                reprobe_every: 4,
                max_reprobe_rounds: 8,
                max_carves: 24,
                keep_carves: false,
            },
        )
    } else if multilayer {
        Command::MultilayerAutoroute(
            selection,
            make_multilayer_options(
                deterministic,
                planar_options,
                layer_seed,
                fanout_rings,
                fanout_angles,
                anneal,
                board,
            ),
        )
    } else {
        Command::Autoroute(selection, planar_options)
    }
}
