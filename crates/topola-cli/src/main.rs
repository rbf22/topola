// SPDX-FileCopyrightText: 2024 Topola contributors
//
// SPDX-License-Identifier: MIT

use clap::Parser;
use std::collections::{BTreeMap, BTreeSet};
use std::fs::File;
use std::io::BufReader;
use std::io::Write;
use std::ops::ControlFlow;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};
use topola::autorouter::execution::Command;
use topola::autorouter::history::History;
use topola::autorouter::invoker::Invoker;
use topola::autorouter::ratline::RatlineUid;
use topola::autorouter::selection::PinSelection;
use topola::board::Board;
use topola::autorouter::anterouter::AnterouterOptions;
use topola::autorouter::multilayer_autoroute::MultilayerAutorouteOptions;
use topola::autorouter::Autorouter;
use topola::autorouter::PlanarAutorouteOptions;
use topola::autorouter::PresortBy;
use topola::board::edit::BoardEdit;
use topola::board::AccessMesadata;
use topola::drawing::graph::{GetMaybeNet, MakePrimitiveRef};

use topola::router::RouterOptions;
use topola::specctra::design::SpecctraDesign;
use topola::specctra::mesadata::SpecctraMesadata;
use topola::stepper::{EstimateProgress, Step, TimeoutOptions};

pub mod cli;
pub mod suggest;
use cli::Cli;
use serde::Deserialize;

/// Applies the per-owner displacements of a `--suggest-shifts` report
/// to the board before routing ("component shifting" for congestion
/// relief). Only reference-designator owners are components;
/// `wire:<net>` blockers are routed geometry and `<board>` owners are
/// netless shapes, neither of which can be translated.
fn apply_shift_report(
    board: &mut topola::board::Board<topola::specctra::mesadata::SpecctraMesadata>,
    report_filename: &std::path::Path,
) -> Result<(), std::io::Error> {
    #[derive(Deserialize)]
    struct ShiftEntry {
        dx: f64,
        dy: f64,
    }

    #[derive(Deserialize)]
    struct ShiftReport {
        shifts: BTreeMap<String, ShiftEntry>,
    }

    let report: ShiftReport =
        serde_json::from_reader(BufReader::new(File::open(report_filename)?))?;

    let mut edit = BoardEdit::new();
    for (refdes, shift) in report.shifts {
        if refdes.starts_with("wire:") || refdes == "<board>" {
            continue;
        }

        match board.shift_component(&mut edit, &refdes, (shift.dx, shift.dy)) {
            Some(moved) => eprintln!(
                "[topola] shifted {} ({} dots) by ({:.0}, {:.0})",
                refdes, moved, shift.dx, shift.dy
            ),
            None => eprintln!("[topola] skipped shift of {}", refdes),
        }
    }
    Ok(())
}

fn main() -> Result<(), std::io::Error> {
    env_logger::init();
    let args = Cli::parse();
    let design_file = File::open(&args.input)?;
    let design_bufread = BufReader::new(design_file);

    let design =
        SpecctraDesign::load(design_bufread).expect("File failed to parse as Specctra DSN");

    let mut invoker = if args.jobs.unwrap_or(1) > 1 && args.commands.is_none() {
        run_parallel_jobs(&args)?
    } else {
        let mut board = design.make_board(&mut BoardEdit::new());

        if let Some(report_filename) = &args.apply_shifts {
            apply_shift_report(&mut board, report_filename)?;
        }

        let history = if let Some(commands_filename) = &args.commands {
            let command_file = File::open(commands_filename)?;
            let commands_bufread = BufReader::new(command_file);
            serde_json::from_reader(commands_bufread)?
        } else {
            let mut history = History::new();
            let selection = make_selection(&board, &args);
            let command = make_command(&args, selection, make_planar_options(&args));
            history.do_(command, None);
            history
        };

        let mut invoker = Invoker::new(Autorouter::new(board).unwrap());

        let (done, undone) = history.dissolve();
        for entry in done {
        let mut last_report = Instant::now();
        let start = Instant::now();
        let status = invoker.execute_with_timeout(
            entry.command().clone(),
            args.max_seconds.map(Duration::from_secs_f64),
            |execute| {
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
            },
        );
        match status {
            Ok(true) => eprintln!(
                "[topola] timed out after {}, keeping routed bands",
                humantime(start.elapsed())
            ),
            Ok(false) => eprintln!(
                "[topola] command finished in {}",
                humantime(start.elapsed())
            ),
            Err(err) => eprintln!("[topola] command failed: {:?}", err),
        }
        }
        let _ = undone; // replay drops undone history entries
        invoker
    };

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

    if let Some(suggest_shifts_filename) = &args.suggest_shifts {
        let file = File::create(suggest_shifts_filename)?;
        suggest::write_suggestions(
            invoker.autorouter_mut(),
            RouterOptions {
                wrap_around_bands: true,
                squeeze_through_under_bends: false,
                routed_band_width: 100.0,
            },
            file,
        )?;
        eprintln!(
            "[topola] wrote shift suggestions to {}",
            suggest_shifts_filename.display()
        );
    }

    #[cfg(feature = "metrics")]
    eprintln!("{}", topola::metrics::METRICS);

    let output_filename = args
        .output
        .unwrap_or_else(|| args.input.clone().with_extension("ses"));
    let mut file = File::create(output_filename).unwrap();
    design
        .write_ses(invoker.autorouter().board(), &mut file)
        .expect("Failed to write Specctra Session file");

    Ok(())
}

fn make_selection(board: &Board<SpecctraMesadata>, args: &Cli) -> PinSelection {
    if !args.nets.is_empty() {
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
    }
}

fn make_planar_options(args: &Cli) -> PlanarAutorouteOptions {
    PlanarAutorouteOptions {
        principal_layer: 0,
        presort_by: if args.escape_presort {
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
            routed_band_width: 100.0,
        },
        timeout: TimeoutOptions {
            initial: 1.0,
            progress_bonus: 0.005,
        },
        max_reconfigurations: args.max_reconfigurations.map(|n| n as usize),
        improve_pad_entry: args.pad_entry,
        pad_entry_min_angle: std::f64::consts::FRAC_PI_4,
        escape_stubs: args.escape_stubs,
        surgical_ripup: args.surgical_ripup,
        shuffle_seed: 0,
    }
}

fn make_command(
    args: &Cli,
    selection: PinSelection,
    planar_options: PlanarAutorouteOptions,
) -> Command {
    if args.multilayer {
        Command::MultilayerAutoroute(
            selection,
            MultilayerAutorouteOptions {
                anterouter: AnterouterOptions {
                    fanout_clearance: 200.0,
                },
                planar: planar_options,
                timeout: TimeoutOptions {
                    initial: 5.0,
                    progress_bonus: 0.5,
                },
                layer_seed: 0,
            },
        )
    } else {
        Command::Autoroute(selection, planar_options)
    }
}

/// Counts (routed, total) ratlines on the principal-layer ratsnest.
fn routed_counts(autorouter: &Autorouter<SpecctraMesadata>) -> (usize, usize) {
    let ratsnest = autorouter.ratsnests().on_principal_layer(0);
    let total = ratsnest.graph().edge_indices().count();
    let routed = ratsnest
        .graph()
        .edge_indices()
        .filter(|edge| {
            ratsnest
                .graph()
                .edge_weight(*edge)
                .is_some_and(|weight| weight.band_termseg.is_some())
        })
        .count();
    (routed, total)
}

/// Runs `args.jobs` independent routing attempts in parallel, each on
/// its own board copy and with a different preconfiguration shuffle
/// seed, then keeps the invoker that routed the most ratlines. The
/// first attempt to route everything flags the others to finish early.
fn run_parallel_jobs(args: &Cli) -> Result<Invoker<SpecctraMesadata>, std::io::Error> {
    let jobs = args.jobs.unwrap_or(1) as usize;
    let stop = Arc::new(AtomicBool::new(false));
    let template = make_planar_options(args);

    eprintln!(
        "[topola] running {} parallel routing attempts (best of {} kept)",
        jobs, jobs
    );

    let mut handles = Vec::with_capacity(jobs);
    for i in 0..jobs {
        let input = args.input.clone();
        let apply_shifts = args.apply_shifts.clone();
        let nets = args.nets.clone();
        let exclude_nets = args.exclude_nets.clone();
        let multilayer = args.multilayer;
        let max_seconds = args.max_seconds;
        let stop = Arc::clone(&stop);
        let mut planar_options = template;
        planar_options.shuffle_seed = i as u64;

        handles.push(std::thread::spawn(
            move || -> Result<(usize, Invoker<SpecctraMesadata>), String> {
                let design_file = File::open(&input).map_err(|e| e.to_string())?;
                let design = SpecctraDesign::load(BufReader::new(design_file))
                    .map_err(|e| format!("{:?}", e))?;
                let mut board = design.make_board(&mut BoardEdit::new());
                if let Some(report_filename) = &apply_shifts {
                    apply_shift_report(&mut board, report_filename).map_err(|e| e.to_string())?;
                }

                let selection = if !nets.is_empty() {
                    let net_set: BTreeSet<String> = nets.iter().cloned().collect();
                    PinSelection::new_select_nets(&board, &net_set)
                } else if !exclude_nets.is_empty() {
                    let net_set: BTreeSet<String> = exclude_nets.iter().cloned().collect();
                    PinSelection::new_select_all_except_nets(&board, &net_set)
                } else {
                    PinSelection::new_select_layer(&board, 0)
                };
                let command = if multilayer {
                    Command::MultilayerAutoroute(
                        selection,
                        MultilayerAutorouteOptions {
                            anterouter: AnterouterOptions {
                                fanout_clearance: 200.0,
                            },
                            planar: planar_options,
                            timeout: TimeoutOptions {
                                initial: 5.0,
                                progress_bonus: 0.5,
                            },
                            layer_seed: i as u64,
                        },
                    )
                } else {
                    Command::Autoroute(selection, planar_options)
                };

                let mut invoker =
                    Invoker::new(Autorouter::new(board).map_err(|e| format!("{:?}", e))?);
                let mut execute = invoker
                    .execute_stepper(command)
                    .map_err(|e| format!("{:?}", e))?;

                let start = Instant::now();
                let deadline = max_seconds.map(|s| start + Duration::from_secs_f64(s));
                let mut last_report = Instant::now();
                let mut finished_early = false;

                loop {
                    let status = execute.step(&mut invoker).map_err(|e| format!("{:?}", e))?;
                    if let ControlFlow::Break(..) = status {
                        break;
                    }

                    if !finished_early
                        && (deadline.is_some_and(|d| Instant::now() >= d)
                            || stop.load(Ordering::Relaxed))
                    {
                        execute.finish_early(invoker.autorouter_mut());
                        finished_early = true;
                    }

                    if last_report.elapsed() >= Duration::from_secs(2) {
                        let progress = execute.estimate_progress();
                        let value = *progress.value();
                        let maximum = *progress.maximum();
                        eprintln!(
                            "[topola j{}] {} elapsed, routed {}/{} ({:.1}%)",
                            i,
                            humantime(start.elapsed()),
                            value,
                            maximum,
                            if maximum > 0 {
                                100.0 * value as f64 / maximum as f64
                            } else {
                                0.0
                            }
                        );
                        last_report = Instant::now();
                    }
                }

                // A full completion (no early finish) means this
                // attempt routed everything — stop the sibling
                // attempts, the winner is already decided.
                if !finished_early {
                    stop.store(true, Ordering::Relaxed);
                }

                let (routed, total) = routed_counts(invoker.autorouter());
                eprintln!(
                    "[topola j{}] done in {}: routed {}/{}",
                    i,
                    humantime(start.elapsed()),
                    routed,
                    total
                );
                Ok((routed, invoker))
            },
        ));
    }

    let mut best: Option<(usize, Invoker<SpecctraMesadata>)> = None;
    for handle in handles {
        match handle.join() {
            Ok(Ok(result)) => {
                if best.as_ref().is_none_or(|(routed, _)| result.0 > *routed) {
                    best = Some(result);
                }
            }
            Ok(Err(err)) => eprintln!("[topola] parallel job failed: {}", err),
            Err(_) => eprintln!("[topola] parallel job panicked"),
        }
    }

    best.map(|(_, invoker)| invoker).ok_or_else(|| {
        std::io::Error::new(std::io::ErrorKind::Other, "all parallel routing jobs failed")
    })
}

fn humantime(d: Duration) -> String {
    let s = d.as_secs();
    format!("{}m{:02}s", s / 60, s % 60)
}
