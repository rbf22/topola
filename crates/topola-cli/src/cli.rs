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
        value_name = "N",
        help = "Run N parallel routing attempts with different ratline orderings and keep the best result"
    )]
    pub jobs: Option<u16>,
    #[arg(
        long,
        value_name = "WIDTH",
        help = "Width of the routed band in the design's units. Defaults to the trace width declared by the design's rule, or 100.0 when it declares none"
    )]
    pub band_width: Option<f64>,
    #[arg(
        long,
        value_name = "FILE",
        help = "Write a JSON report of channel blockers and per-component shift suggestions for unrouted ratlines"
    )]
    pub suggest_shifts: Option<std::path::PathBuf>,
    #[arg(
        long,
        value_name = "FILE",
        help = "Apply per-component shifts from a --suggest-shifts report before routing"
    )]
    pub apply_shifts: Option<std::path::PathBuf>,
    #[arg(
        long,
        help = "Order ratlines by endpoint crowding, most-crowded first (BGA escape heuristic)"
    )]
    pub escape_presort: bool,
    #[arg(
        long,
        help = "Order ratlines by corridor obstruction, most-blocked first (counts primitives infringing each ratline's direct corridor)"
    )]
    pub corridor_presort: bool,
    #[arg(
        long,
        help = "Probe every unrouted ratline under relaxed clearances before routing and seed obstacle-congestion penalties from the blockers found (G3)"
    )]
    pub preprobe: bool,
    #[arg(
        long,
        help = "Lay escape stubs from surrounded pads through the widest local gap before routing (BGA escape)"
    )]
    pub escape_stubs: bool,
    #[arg(
        long,
        help = "Rip up only blocking bands on failure instead of backtracing (negotiated congestion)"
    )]
    pub surgical_ripup: bool,
    #[arg(
        long,
        help = "Probe-selected ratlines identify the hardest connections up front; they are scheduled first, their demand corridors carved as same-net fixed geometry, and their bands protected from rip-up, with re-identification on stalls (combines with --multilayer)"
    )]
    pub critical_first: bool,
    #[arg(
        long,
        help = "Route across all layers, fanning out to vias (multilayer autorouting)"
    )]
    pub multilayer: bool,
    #[arg(
        long,
        help = "Post-process routed bands to swing shallow polygonal pad entries to the last cane's alternate tangent"
    )]
    pub pad_entry: bool,
}
