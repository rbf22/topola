// SPDX-FileCopyrightText: 2025 Topola contributors
//
// SPDX-License-Identifier: MIT

//! `--jobs` parallel routing: N independent attempts on their own
//! board copies, each with a different preconfiguration shuffle seed.
//! The first attempt to route everything flags the others to finish
//! early; the invoker that routed the most ratlines wins.

use std::fs::File;
use std::io::BufReader;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use topola::autorouter::invoker::Invoker;
use topola::autorouter::Autorouter;
use topola::board::edit::BoardEdit;
use topola::specctra::design::SpecctraDesign;
use topola::specctra::mesadata::SpecctraMesadata;
use topola::stepper::EstimateProgress;

use crate::cli::Cli;
use crate::options::{make_anneal_options, make_command, make_planar_options, make_selection};
use crate::preprobe::preprobe_blockers;
use crate::report::{humantime, routed_counts, ProgressCsv};
use crate::suggest;

pub fn run_parallel_jobs(args: &Cli) -> Result<Invoker<SpecctraMesadata>, std::io::Error> {
    let jobs = args.jobs.unwrap_or(1) as usize;
    let stop = Arc::new(AtomicBool::new(false));

    // The template needs the design's rule width for the band-width
    // default; the same board serves the up-front probe below.
    let template_board = {
        let design_file = File::open(&args.input)?;
        let design = SpecctraDesign::load(BufReader::new(design_file))
            .map_err(|e| std::io::Error::other(format!("{e:?}")))?;
        design.make_board(&mut BoardEdit::new())
    };
    let template = make_planar_options(args, &template_board);

    // Probe the pristine board once up front; every worker seeds its
    // congestion map from the shared blockers.
    let preprobe_seed = if args.preprobe {
        let mut board = template_board;
        if let Some(report_filename) = &args.apply_shifts {
            suggest::apply_report(&mut board, report_filename)?;
        }
        let selection = make_selection(&board, &args.nets, &args.exclude_nets, args.multilayer);
        let autorouter =
            Autorouter::new(board).map_err(|e| std::io::Error::other(format!("{e:?}")))?;
        Some(preprobe_blockers(
            &autorouter,
            Some(&selection),
            template.router,
            jobs,
        ))
    } else {
        None
    };

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
        let critical_first = args.critical_first;
        let fanout_rings = args.fanout_rings;
        let max_seconds = args.max_seconds;
        let max_steps = args.max_steps;
        let deterministic = args.deterministic;
        let progress_csv = args.progress_csv.clone();
        let stop = Arc::clone(&stop);
        let preprobe_seed = preprobe_seed.clone();
        let mut planar_options = template;
        planar_options.shuffle_seed = i as u64;
        // Each worker explores a different annealing trajectory —
        // same derivation convention as `shuffle_seed`.
        let mut anneal = make_anneal_options(args);
        if let Some(anneal) = &mut anneal {
            anneal.seed = anneal.seed.wrapping_add(i as u64);
        }

        handles.push(std::thread::spawn(
            move || -> Result<(usize, Invoker<SpecctraMesadata>), String> {
                let design_file = File::open(&input).map_err(|e| e.to_string())?;
                let design = SpecctraDesign::load(BufReader::new(design_file))
                    .map_err(|e| format!("{:?}", e))?;
                let mut board = design.make_board(&mut BoardEdit::new());
                if let Some(report_filename) = &apply_shifts {
                    suggest::apply_report(&mut board, report_filename)
                        .map_err(|e| e.to_string())?;
                }

                let selection = make_selection(&board, &nets, &exclude_nets, multilayer);
                let command = make_command(
                    critical_first,
                    multilayer,
                    deterministic,
                    selection,
                    planar_options,
                    i as u64,
                    fanout_rings,
                    anneal,
                    &board,
                );

                let mut invoker =
                    Invoker::new(Autorouter::new(board).map_err(|e| format!("{:?}", e))?);
                if let Some(seed) = &preprobe_seed {
                    invoker
                        .autorouter_mut()
                        .seed_congestion(seed.iter().copied());
                }
                let start = Instant::now();
                let mut last_report = Instant::now();
                let mut csv = progress_csv
                    .as_deref()
                    .map(|path| ProgressCsv::new(&ProgressCsv::job_path(path, i)))
                    .transpose()
                    .map_err(|e| format!("{:?}", e))?;

                let timed_out = invoker
                    .execute_with_timeout(
                        command,
                        max_seconds.map(Duration::from_secs_f64),
                        max_steps,
                        || stop.load(Ordering::Relaxed),
                        |report| {
                            if let Some(csv) = csv.as_mut() {
                                csv.write(report, start);
                            }
                            if last_report.elapsed() >= Duration::from_secs(2) {
                                let progress = report.stepper.estimate_progress();
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
                        },
                    )
                    .map_err(|e| format!("{:?}", e))?;

                // A full completion (no early finish) means this
                // attempt routed everything — stop the sibling
                // attempts, the winner is already decided.
                if !timed_out {
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

    best.map(|(_, invoker)| invoker)
        .ok_or_else(|| std::io::Error::other("all parallel routing jobs failed"))
}
