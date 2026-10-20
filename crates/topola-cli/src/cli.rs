// SPDX-FileCopyrightText: 2024 Topola contributors
//
// SPDX-License-Identifier: MIT

use clap::{Parser, ValueEnum};
use std::path::PathBuf;

/// Which annealer formulation `--anneal` runs (mirrors
/// `topola::autorouter::anneal::AnnealMode`; clap's `ValueEnum` cannot
/// be derived on a foreign type).
#[derive(Clone, Copy, Debug, Default, ValueEnum)]
pub enum AnnealModeArg {
    /// Parametric 3D curves annealed under soft-core repulsion — the
    /// full continuous-3D formulation.
    #[default]
    Continuous,
    /// SA directly over (layer assignment, route order) — the
    /// comparison baseline.
    Discrete,
}

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
        help = "Survey crowded pads up front but lay an escape stub only when a ratline terminating at one actually fails, then retry it against the stub (lazy BGA escape)"
    )]
    pub lazy_escape_stubs: bool,
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
        requires = "multilayer",
        help = "Run a simulated-annealing global pre-planner before multilayer routing: it anneals layer assignments, route order, congestion seeding, and (in continuous mode) mid-route layer hops under a bounded soft-core potential"
    )]
    pub anneal: bool,
    #[arg(
        long,
        value_enum,
        value_name = "MODE",
        requires = "anneal",
        help = "Annealer formulation: continuous 3D curves, or the discrete layer/order comparison baseline"
    )]
    pub anneal_mode: Option<AnnealModeArg>,
    #[arg(
        long,
        value_name = "SEED",
        requires = "anneal",
        help = "Deterministic seed for the annealer (single-run only; --jobs derives seeds per worker)"
    )]
    pub anneal_seed: Option<u64>,
    #[arg(
        long,
        value_name = "N",
        requires = "anneal",
        help = "Number of cooling sweeps for the annealer (default 300)"
    )]
    pub anneal_iterations: Option<usize>,
    #[arg(
        long,
        value_name = "K",
        requires = "anneal",
        help = "Carve demand corridors for the K most-contested ratlines after annealing (default 0, off)"
    )]
    pub anneal_carve_top_k: Option<usize>,
    #[arg(
        long,
        help = "Post-process routed bands to swing shallow polygonal pad entries to the last cane's alternate tangent"
    )]
    pub pad_entry: bool,
    #[arg(
        long,
        value_name = "N",
        help = "Finish routing early after this many execution steps, keeping already routed bands. Unlike --max-seconds this is machine-independent"
    )]
    pub max_steps: Option<usize>,
    #[arg(
        long,
        value_name = "N",
        help = "Cap router steps a single ratline's search may consume before the attempt fails (as if its frontier exhausted). Without a cap, one expanding search can starve every later ratline of budget"
    )]
    pub per_ratline_steps: Option<u64>,
    #[arg(
        long,
        value_name = "F",
        help = "Adaptive per-ratline step cap: a search fails after consuming F times its own navmesh node count in steps — sized per ratline, unlike the constant --per-ratline-steps. Ignored if --per-ratline-steps is set"
    )]
    pub navmesh_step_factor: Option<f64>,
    #[arg(
        long,
        value_name = "N",
        help = "Seed for the preconfiguration ratline shuffle (single-run only; --jobs derives seeds per worker)"
    )]
    pub seed: Option<u64>,
    #[arg(
        long,
        help = "Disable wall-clock-driven reconfiguration: per-ratline timeouts never fire, so reconfiguration is purely failure-driven. Combine with --max-steps and --seed for reproducible benchmark runs"
    )]
    pub deterministic: bool,
    #[arg(
        long,
        value_name = "FILE",
        help = "Write a CSV time series of per-step progress (elapsed time, step kind, ratline position, unrouted/unconnected counts, geometry ledger sizes). With --jobs, one file per worker named FILE-j<N>.csv"
    )]
    pub progress_csv: Option<PathBuf>,
    #[arg(
        long,
        value_name = "FILE",
        help = "Write a CSV of (failing ratline, blocking ratline/obstacle) pairs recorded at each routing failure"
    )]
    pub blockers_csv: Option<PathBuf>,
}
