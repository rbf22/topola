// SPDX-FileCopyrightText: 2024 Topola contributors
//
// SPDX-License-Identifier: MIT

use clap::Parser;
use std::collections::{BTreeMap, BTreeSet};
use std::fs::File;
use std::io::BufReader;
use std::io::Write;
use std::time::{Duration, Instant};
use topola::autorouter::execution::Command;
use topola::autorouter::history::History;
use topola::autorouter::invoker::Invoker;
use topola::autorouter::ratline::RatlineUid;
use topola::autorouter::selection::PinSelection;
use topola::autorouter::Autorouter;
use topola::autorouter::PlanarAutorouteOptions;
use topola::autorouter::PresortBy;
use topola::board::edit::BoardEdit;
use topola::board::AccessMesadata;
use topola::drawing::graph::{GetMaybeNet, MakePrimitiveRef};

use topola::router::RouterOptions;
use topola::specctra::design::SpecctraDesign;
use topola::stepper::{EstimateProgress, TimeoutOptions};

pub mod cli;
use cli::Cli;

fn main() -> Result<(), std::io::Error> {
    let args = Cli::parse();
    let design_file = File::open(&args.input)?;
    let design_bufread = BufReader::new(design_file);

    let design =
        SpecctraDesign::load(design_bufread).expect("File failed to parse as Specctra DSN");

    let board = design.make_board(&mut BoardEdit::new());

    let history = if let Some(commands_filename) = args.commands {
        let command_file = File::open(commands_filename)?;
        let commands_bufread = BufReader::new(command_file);
        serde_json::from_reader(commands_bufread)?
    } else {
        let mut history = History::new();
        let selection = if !args.nets.is_empty() {
            let nets: BTreeSet<String> = args.nets.iter().cloned().collect();
            for net in &nets {
                if board.mesadata().netname_net(net).is_none() {
                    eprintln!("[topola] warning: net '{}' not found", net);
                }
            }
            let selection = PinSelection::new_select_nets(&board, &nets);
            eprintln!(
                "[topola] selected {} pins across {} requested nets",
                selection.selectors().count(),
                nets.len()
            );
            selection
        } else if !args.exclude_nets.is_empty() {
            let nets: BTreeSet<String> = args.exclude_nets.iter().cloned().collect();
            let selection = PinSelection::new_select_all_except_nets(&board, &nets);
            eprintln!(
                "[topola] selected {} pins excluding {} nets",
                selection.selectors().count(),
                nets.len()
            );
            selection
        } else {
            PinSelection::new_select_layer(&board, 0)
        };
        history.do_(
            Command::Autoroute(
                selection,
                PlanarAutorouteOptions {
                    principal_layer: 0,
                    presort_by: PresortBy::RatlineIntersectionCountAndLength,
                    permutate: true,
                    router: RouterOptions {
                        wrap_around_bands: true,
                        squeeze_through_under_bends: false,
                        routed_band_width: 100.0,
                    },
                    timeout: TimeoutOptions {
                        initial: 1.0,
                        progress_bonus: 0.005,
                    },
                },
            ),
            None,
        );
        history
    };

    let mut invoker = Invoker::new(Autorouter::new(board).unwrap());

    let (done, undone) = history.dissolve();
    for entry in done {
        let mut last_report = Instant::now();
        let start = Instant::now();
        let status = invoker.execute_with_progress(entry.command().clone(), |execute| {
            if last_report.elapsed() >= Duration::from_secs(2) {
                let progress = execute.estimate_progress();
                let value = *progress.value();
                let maximum = *progress.maximum();
                let pct = if maximum > 0 {
                    100.0 * value as f64 / maximum as f64
                } else {
                    0.0
                };
                eprintln!(
                    "[topola] {} elapsed, routed {}/{} ({:.1}%)",
                    humantime(start.elapsed()),
                    value,
                    maximum,
                    pct
                );
                let _ = std::io::stderr().flush();
                last_report = Instant::now();
            }
        });
        match status {
            Ok(()) => eprintln!(
                "[topola] command finished in {}",
                humantime(start.elapsed())
            ),
            Err(err) => eprintln!("[topola] command failed: {:?}", err),
        }
    }
    let _ = undone; // replay drops undone history entries

    {
        use topola::graph::MakeRef;

        let autorouter = invoker.autorouter();
        let ratsnest = autorouter.ratsnests().on_principal_layer(0);
        let mut unrouted: BTreeMap<String, usize> = BTreeMap::new();
        let mut total = 0usize;

        for edge in ratsnest.graph().edge_indices() {
            total += 1;
            let unrouted_ratline = ratsnest
                .graph()
                .edge_weight(edge)
                .is_none_or(|weight| weight.band_termseg.is_none());

            if unrouted_ratline {
                let uid = RatlineUid {
                    principal_layer: 0,
                    index: edge,
                };
                let source_dot = uid.ref_(autorouter).endpoint_dots().0;
                let maybe_netname = source_dot
                    .primitive_ref(autorouter.board().layout().drawing())
                    .maybe_net()
                    .and_then(|net| autorouter.board().mesadata().net_netname(net))
                    .map(str::to_owned);

                *unrouted
                    .entry(maybe_netname.unwrap_or_else(|| "<netless>".to_string()))
                    .or_default() += 1;
            }
        }

        let unrouted_count: usize = unrouted.values().sum();
        eprintln!(
            "[topola] finished: {} ratlines total, {} unrouted",
            total, unrouted_count
        );
        for (netname, count) in &unrouted {
            eprintln!("[topola]   unrouted: {} ({} ratlines)", netname, count);
        }

        if !args.nets.is_empty() {
            let still_open: Vec<_> = args
                .nets
                .iter()
                .filter(|net| unrouted.contains_key(*net))
                .collect();
            if still_open.is_empty() {
                eprintln!("[topola] all requested nets routed");
            } else {
                eprintln!(
                    "[topola] requested nets still unrouted: {}",
                    still_open
                        .iter()
                        .map(|net| net.as_str())
                        .collect::<Vec<_>>()
                        .join(", ")
                );
            }
        }
    }

    let output_filename = args
        .output
        .unwrap_or_else(|| args.input.clone().with_extension("ses"));
    let mut file = File::create(output_filename).unwrap();
    design
        .write_ses(invoker.autorouter().board(), &mut file)
        .expect("Failed to write Specctra Session file");

    Ok(())
}

fn humantime(d: Duration) -> String {
    let s = d.as_secs();
    format!("{}m{:02}s", s / 60, s % 60)
}
