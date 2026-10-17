// SPDX-FileCopyrightText: 2024 Topola contributors
//
// SPDX-License-Identifier: MIT

use clap::Parser;
use std::fs::File;
use std::io::BufReader;
use topola::autorouter::execution::Command;
use topola::autorouter::history::History;
use topola::autorouter::invoker::Invoker;
use topola::autorouter::selection::PinSelection;
use topola::autorouter::Autorouter;
use topola::autorouter::PlanarAutorouteOptions;
use topola::autorouter::PresortBy;
use topola::board::edit::BoardEdit;
use std::io::Write;
use std::time::{Duration, Instant};
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
        history.do_(
            Command::Autoroute(
                PinSelection::new_select_layer(&board, 0),
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
                let pct = if maximum > 0 { 100.0 * value as f64 / maximum as f64 } else { 0.0 };
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
            Ok(()) => eprintln!("[topola] command finished in {}", humantime(start.elapsed())),
            Err(err) => eprintln!("[topola] command failed: {:?}", err),
        }
    }
    let _ = undone; // replay drops undone history entries

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
