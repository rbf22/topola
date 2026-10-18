// SPDX-FileCopyrightText: 2024 Topola contributors
//
// SPDX-License-Identifier: MIT

use clap::Parser;
use std::path::PathBuf;

#[derive(Parser, Debug, Default)]
#[command(about, version)]
pub struct Cli {
    #[arg(
        value_name = "SPECCTRA DESIGN FILE",
        help = "Specify the Specctra Design (*.dsn) input file for the Topola autorouter"
    )]
    pub input: PathBuf,
    #[arg(
        short,
        long,
        value_name = "SPECCTRA SESSION FILE",
        help = "Specify the output session file in Specctra-compatible format (*.ses). The input filename is used by default, with the extension changed to Specctra Session File extension"
    )]
    pub output: Option<PathBuf>,
    #[arg(
        short,
        long,
        value_name = "COMMAND FILE",
        help = "JSON-like file with .cmd extension, containing sequence of available commands "
    )]
    pub commands: Option<PathBuf>,
    #[arg(
        long,
        value_name = "NETS",
        value_delimiter = ',',
        help = "Route only the named nets (comma-separated). By default, all pads on the first layer are selected"
    )]
    pub nets: Vec<String>,
    #[arg(
        long,
        value_name = "NETS",
        value_delimiter = ',',
        conflicts_with = "nets",
        help = "Route all nets except the named nets (comma-separated)"
    )]
    pub exclude_nets: Vec<String>,
    #[arg(
        long,
        value_name = "SECONDS",
        help = "Finish routing early after this many seconds, keeping already routed bands"
    )]
    pub max_seconds: Option<f64>,
    #[arg(
        long,
        value_name = "N",
        help = "Stop trying new route orderings after this many reconfigurations, keeping already routed bands"
    )]
    pub max_reconfigurations: Option<u64>,
    #[arg(
        long,
        value_name = "FILE",
        help = "Write a JSON report of channel blockers and per-component shift suggestions for unrouted ratlines"
    )]
    pub suggest_shifts: Option<std::path::PathBuf>,
    #[arg(
        long,
        value_name = "WIDTH",
        help = "Half-width of the wish channel used for shift suggestions (defaults to routed band half-width plus largest clearance)"
    )]
    pub channel_width: Option<f64>,
}
