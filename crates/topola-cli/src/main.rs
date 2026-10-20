// SPDX-FileCopyrightText: 2024 Topola contributors
//
// SPDX-License-Identifier: MIT

use std::fs::File;
use std::io::BufReader;
use std::io::Write;
use std::time::{Duration, Instant};

use clap::Parser;
use topola::autorouter::history::History;
use topola::autorouter::invoker::Invoker;
use topola::autorouter::Autorouter;
use topola::board::edit::BoardEdit;
use topola::router::RouterOptions;
use topola::specctra::design::SpecctraDesign;
use topola::stepper::EstimateProgress;

mod cli;
mod jobs;
mod logger;
mod options;
mod preprobe;
mod report;
pub mod suggest;

use cli::Cli;
use jobs::run_parallel_jobs;
use options::{band_width, make_anneal_options, make_command, make_planar_options, make_selection};
use preprobe::preprobe_blockers;
use report::{humantime, write_blockers_csv, ProgressCsv};

fn main() -> Result<(), std::io::Error> {
    logger::init();
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
            suggest::apply_report(&mut board, report_filename)?;
        }

        let (history, selection) = if let Some(commands_filename) = &args.commands {
            let command_file = File::open(commands_filename)?;
            let commands_bufread = BufReader::new(command_file);
            (serde_json::from_reader(commands_bufread)?, None)
        } else {
            let mut history = History::new();
            let selection = make_selection(&board, &args.nets, &args.exclude_nets, args.multilayer);
            let command = make_command(
                args.critical_first,
                args.multilayer,
                args.deterministic,
                selection.clone(),
                make_planar_options(&args, &board),
                args.layer_seed.unwrap_or(0),
                args.fanout_rings,
                make_anneal_options(&args),
                &board,
            );
            history.do_(command, None);
            (history, Some(selection))
        };

        let mut invoker = Invoker::new(Autorouter::new(board).unwrap());

        if args.preprobe {
            let router_options = make_planar_options(&args, invoker.autorouter().board()).router;
            let blockers = preprobe_blockers(
                invoker.autorouter_mut(),
                selection.as_ref(),
                router_options,
                args.jobs.unwrap_or(1) as usize,
            );
            invoker.autorouter_mut().seed_congestion(blockers);
        }

        let mut progress_csv = args
            .progress_csv
            .as_deref()
            .map(ProgressCsv::new)
            .transpose()?;

        let (done, undone) = history.dissolve();
        for entry in done {
            let mut last_report = Instant::now();
            let start = Instant::now();
            let status = invoker.execute_with_timeout(
                entry.command().clone(),
                args.max_seconds.map(Duration::from_secs_f64),
                args.max_steps,
                || false,
                |report| {
                    if let Some(csv) = progress_csv.as_mut() {
                        csv.write(report, start);
                    }
                    if last_report.elapsed() >= Duration::from_secs(2) {
                        let progress = report.stepper.estimate_progress();
                        let value = *progress.value();
                        let maximum = *progress.maximum();
                        let pct = if maximum > 0 {
                            100.0 * value as f64 / maximum as f64
                        } else {
                            0.0
                        };
                        eprintln!(
                            "[topola] {} elapsed, processed {}/{} ratlines ({:.1}%)",
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

    report::print_final_summary(invoker.autorouter(), &args.nets);

    if let Some(suggest_shifts_filename) = &args.suggest_shifts {
        let file = File::create(suggest_shifts_filename)?;
        let router_options = RouterOptions {
            wrap_around_bands: true,
            squeeze_through_under_bends: false,
            routed_band_width: band_width(&args, invoker.autorouter().board()),
        };
        suggest::write_suggestions(
            invoker.autorouter_mut(),
            router_options,
            args.jobs.unwrap_or(1) as usize,
            file,
        )?;
        eprintln!(
            "[topola] wrote shift suggestions to {}",
            suggest_shifts_filename.display()
        );
    }

    #[cfg(feature = "metrics")]
    eprintln!("{}", topola::metrics::METRICS);

    if let Some(path) = &args.blockers_csv {
        write_blockers_csv(invoker.autorouter(), path)?;
        eprintln!("[topola] wrote blocker attributions to {}", path.display());
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
