// SPDX-FileCopyrightText: 2024 Topola contributors
//
// SPDX-License-Identifier: MIT

use clap::Parser;
use std::collections::{BTreeMap, BTreeSet};
use std::fs::File;
use std::io::BufReader;
use std::io::BufWriter;
use std::io::Write;
use std::ops::ControlFlow;
use std::path::Path;
use std::path::PathBuf;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};
use topola::autorouter::anterouter::AnterouterOptions;
use topola::autorouter::connected_components::ConnectedComponents;
use topola::autorouter::critical_autoroute::CriticalAutorouteOptions;
use topola::autorouter::execution::{Command, StepKind};
use topola::autorouter::history::History;
use topola::autorouter::invoker::{Invoker, StepReport};
use topola::autorouter::multilayer_autoroute::MultilayerAutorouteOptions;
use topola::autorouter::ratline::RatlineUid;
use topola::autorouter::selection::PinSelection;
use topola::autorouter::Autorouter;
use topola::autorouter::PlanarAutorouteOptions;
use topola::autorouter::PresortBy;
use topola::board::edit::BoardEdit;
use topola::board::AccessMesadata;
use topola::board::Board;
use topola::drawing::graph::{GetMaybeNet, MakePrimitiveRef, PrimitiveIndex};

use topola::router::RouterOptions;
use topola::specctra::design::SpecctraDesign;
use topola::specctra::mesadata::SpecctraMesadata;
use topola::stepper::{EstimateProgress, Step, TimeoutOptions};

pub mod cli;
pub mod logger;
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
            apply_shift_report(&mut board, report_filename)?;
        }

        let (history, selection) = if let Some(commands_filename) = &args.commands {
            let command_file = File::open(commands_filename)?;
            let commands_bufread = BufReader::new(command_file);
            (serde_json::from_reader(commands_bufread)?, None)
        } else {
            let mut history = History::new();
            let selection = make_selection(&board, &args.nets, &args.exclude_nets, args.multilayer);
            let command = make_command(
                &args,
                selection.clone(),
                make_planar_options(&args, &board),
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
        let unconnected = unconnected_count(autorouter);
        eprintln!(
            "[topola] finished: {} ratlines total, {} unrouted, {} unconnected",
            total, unrouted_count, unconnected
        );

        // DIAG: union-find connectivity check
        {
            use topola::autorouter::connected_components::ConnectedComponents;
            use topola::graph::GetIndex;
            use topola::graph::MakeRef as _;
            let conncomps = ConnectedComponents::new(autorouter.board());
            let mut unconn = 0;
            for pl in 0..autorouter.board().layout().drawing().layer_count() {
                for edge in autorouter
                    .ratsnests()
                    .on_principal_layer(pl)
                    .graph()
                    .edge_indices()
                {
                    let uid = RatlineUid {
                        principal_layer: pl,
                        index: edge,
                    };
                    let (o, d) = uid.ref_(autorouter).endpoint_dots();
                    let net = o
                        .primitive_ref(autorouter.board().layout().drawing())
                        .maybe_net();
                    let name = net.and_then(|n| autorouter.board().mesadata().net_netname(n));
                    if name == Some("GND") {
                        continue;
                    }
                    if conncomps.unionfind().find(o.index())
                        != conncomps.unionfind().find(d.index())
                    {
                        eprintln!("[topola] UNCONNECTED {:?} pl{} {:?}", edge, pl, name);
                        unconn += 1;
                    }
                }
            }
            if unconn > 0 {
                eprintln!("[topola] connectivity: {} unconnected", unconn);
            }
        }
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

fn make_selection(
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
        let selection = PinSelection::new_select_nets(&board, &nets);
        eprintln!(
            "[topola] selected {} pins across {} requested nets",
            selection.selectors().count(),
            nets.len()
        );
        selection
    } else if !exclude_nets.is_empty() {
        let nets: BTreeSet<String> = exclude_nets.iter().cloned().collect();
        let selection = PinSelection::new_select_all_except_nets(&board, &nets);
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
        let selection = PinSelection::new_select_all_except_nets(&board, &BTreeSet::new());
        eprintln!(
            "[topola] selected {} pins across all layers",
            selection.selectors().count()
        );
        selection
    } else {
        PinSelection::new_select_layer(&board, 0)
    }
}

/// Routed band width: the `--band-width` override, else the trace
/// width the design's structure rule declares, else the historical
/// fixed default. The rule-derived default keeps the option in the
/// design's own units — a fixed value is meaningless across boards
/// with different resolutions.
fn band_width(args: &Cli, board: &Board<SpecctraMesadata>) -> f64 {
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
fn timeout_options(args: &Cli, initial: f64, progress_bonus: f64) -> TimeoutOptions {
    if args.deterministic {
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

fn make_planar_options(args: &Cli, board: &Board<SpecctraMesadata>) -> PlanarAutorouteOptions {
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
        timeout: timeout_options(args, 1.0, 0.005),
        max_reconfigurations: args.max_reconfigurations.map(|n| n as usize),
        improve_pad_entry: args.pad_entry,
        pad_entry_min_angle: std::f64::consts::FRAC_PI_4,
        escape_stubs: args.escape_stubs,
        surgical_ripup: args.surgical_ripup,
        shuffle_seed: args.seed.unwrap_or(0),
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

fn make_multilayer_options(
    args: &Cli,
    planar_options: PlanarAutorouteOptions,
    layer_seed: u64,
    board: &Board<SpecctraMesadata>,
) -> MultilayerAutorouteOptions {
    MultilayerAutorouteOptions {
        anterouter: AnterouterOptions {
            fanout_clearance: fanout_clearance(board),
        },
        planar: planar_options,
        timeout: timeout_options(
            args,
            // Bonus applies per newly-reached ratline index —
            // 0.5s each banks ~2min of budget by mid-list on
            // big boards, starving layer rotation entirely.
            5.0, 0.05,
        ),
        layer_seed,
    }
}

fn make_command(
    args: &Cli,
    selection: PinSelection,
    planar_options: PlanarAutorouteOptions,
    board: &Board<SpecctraMesadata>,
) -> Command {
    if args.critical_first {
        Command::CriticalAutoroute(
            selection,
            CriticalAutorouteOptions {
                planar: planar_options,
                multilayer: args
                    .multilayer
                    .then(|| make_multilayer_options(args, planar_options, 0, board)),
                critical_quantile: 0.75,
                carve: true,
                protect: true,
                reprobe_every: 4,
                max_reprobe_rounds: 8,
                max_carves: 24,
                keep_carves: false,
            },
        )
    } else if args.multilayer {
        Command::MultilayerAutoroute(
            selection,
            make_multilayer_options(args, planar_options, 0, board),
        )
    } else {
        Command::Autoroute(selection, planar_options)
    }
}

/// Counts (routed, total) ratlines on the principal-layer ratsnest.
/// Runs relaxed-clearance channel probes on a clone of `autorouter`
/// (leaving the routed board untouched) and returns the corridor
/// blockers found, as a flat list to feed `Autorouter::seed_congestion`.
fn preprobe_blockers(
    autorouter: &Autorouter<SpecctraMesadata>,
    selection: Option<&PinSelection>,
    router_options: RouterOptions,
    jobs: usize,
) -> Vec<PrimitiveIndex> {
    let mut probe_autorouter = autorouter.clone();
    // When a selection is given, probe only its ratlines so demand that
    // won't be routed doesn't seed congestion.
    let relax = |rules: &mut SpecctraMesadata| rules.override_clearances(0.0);
    let probes = if jobs > 1 {
        match selection {
            Some(selection) => {
                probe_autorouter.probe_selection_par(selection, router_options, &relax, jobs)
            }
            None => probe_autorouter.probe_channels_par(router_options, &relax, jobs),
        }
    } else {
        match selection {
            Some(selection) => probe_autorouter.probe_selection(selection, router_options, relax),
            None => probe_autorouter.probe_channels(router_options, relax),
        }
    };
    let band_half_width = router_options.routed_band_width / 2.0;
    let drawing = probe_autorouter.board().layout().drawing();
    let blockers: Vec<PrimitiveIndex> = probes
        .iter()
        .flat_map(|probe| probe.blockers(drawing, band_half_width))
        .map(|blocker| blocker.obstacle)
        .collect();
    eprintln!(
        "[topola] preprobe: {} channel probes found {} blockers",
        probes.len(),
        blockers.len()
    );
    blockers
}

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

/// Union-find connectivity count of ratlines across all layers —
/// the truth behind `routed_counts`' band-presence check: a ratline
/// can carry a band yet be electrically open (e.g. a dangling weave
/// half). GND is exempted, matching the final report.
fn unconnected_count(autorouter: &Autorouter<SpecctraMesadata>) -> usize {
    use topola::graph::{GetIndex, MakeRef};

    let conncomps = ConnectedComponents::new(autorouter.board());
    let mut unconnected = 0;

    for principal_layer in 0..autorouter.board().layout().drawing().layer_count() {
        for edge in autorouter
            .ratsnests()
            .on_principal_layer(principal_layer)
            .graph()
            .edge_indices()
        {
            let uid = RatlineUid {
                principal_layer,
                index: edge,
            };
            let (origin, destination) = uid.ref_(autorouter).endpoint_dots();
            let net = origin
                .primitive_ref(autorouter.board().layout().drawing())
                .maybe_net();
            if net.and_then(|n| autorouter.board().mesadata().net_netname(n)) == Some("GND") {
                continue;
            }
            if conncomps.unionfind().find(origin.index())
                != conncomps.unionfind().find(destination.index())
            {
                unconnected += 1;
            }
        }
    }

    unconnected
}

/// Per-step progress CSV writer. Emits a row when the ratline position
/// or step kind changed, plus heartbeat rows at most every 500 ms.
/// `unconnected` is a full-board union-find scan, so it is recomputed
/// at most every 2 s and the last-known value is written in between.
struct ProgressCsv {
    out: BufWriter<File>,
    last_row: Option<(usize, StepKind)>,
    last_emit: Instant,
    last_unconn_at: Option<Instant>,
    unconnected: Option<usize>,
}

impl ProgressCsv {
    fn new(path: &Path) -> std::io::Result<Self> {
        let mut out = BufWriter::new(File::create(path)?);
        writeln!(
            out,
            "elapsed_s,step,kind,step_ms,position,total,unrouted,unconnected,tombstones,pending_joiners"
        )?;
        Ok(Self {
            out,
            last_row: None,
            last_emit: Instant::now(),
            last_unconn_at: None,
            unconnected: None,
        })
    }

    /// `path-j<I>.csv` for per-worker outputs in `--jobs` mode.
    fn job_path(path: &Path, job: usize) -> PathBuf {
        let stem = path
            .file_stem()
            .map(|stem| stem.to_string_lossy().into_owned())
            .unwrap_or_default();
        path.with_file_name(format!("{stem}-j{job}.csv"))
    }

    fn write(&mut self, report: &StepReport<SpecctraMesadata>, start: Instant) {
        let autorouter = report.invoker.autorouter();
        let progress = report.stepper.estimate_progress();
        let position = *progress.value();
        let total = *progress.maximum();

        let state_changed = self.last_row != Some((position, report.kind));
        let heartbeat_due = self.last_emit.elapsed() >= Duration::from_millis(500);
        if !state_changed && !heartbeat_due {
            return;
        }

        if self
            .last_unconn_at
            .is_none_or(|at| at.elapsed() >= Duration::from_secs(2))
        {
            self.unconnected = Some(unconnected_count(autorouter));
            self.last_unconn_at = Some(Instant::now());
        }

        let (routed, _) = routed_counts(autorouter);
        let geometry = autorouter.board().layout().drawing().geometry();
        let _ = writeln!(
            self.out,
            "{:.3},{},{:?},{:.3},{},{},{},{},{},{}",
            start.elapsed().as_secs_f64(),
            report.index,
            report.kind,
            report.duration.as_secs_f64() * 1e3,
            position,
            total,
            total.saturating_sub(routed),
            self.unconnected.map(|n| n.to_string()).unwrap_or_default(),
            geometry.tombstone_count(),
            geometry.pending_joiner_count(),
        );
        self.last_row = Some((position, report.kind));
        self.last_emit = Instant::now();
    }
}

/// Net name and `layer:index` label for a ratline, for reports.
fn ratline_label(autorouter: &Autorouter<SpecctraMesadata>, uid: RatlineUid) -> (String, String) {
    use topola::graph::MakeRef;

    let (origin, _) = uid.ref_(autorouter).endpoint_dots();
    let net = origin
        .primitive_ref(autorouter.board().layout().drawing())
        .maybe_net()
        .and_then(|net| autorouter.board().mesadata().net_netname(net))
        .unwrap_or("<netless>")
        .to_owned();
    (
        net,
        format!("{}:{}", uid.principal_layer, uid.index.index()),
    )
}

/// Drains the failure-attribution log — one row per (failing ratline,
/// blocking obstacle) pair recorded at each routing failure.
fn write_blockers_csv(
    autorouter: &Autorouter<SpecctraMesadata>,
    path: &Path,
) -> std::io::Result<()> {
    let mut out = BufWriter::new(File::create(path)?);
    writeln!(
        out,
        "seq,position,failed_net,failed_uid,blocker_net,blocker_uid,obstacle"
    )?;
    for rec in autorouter.blocker_log() {
        let (failed_net, failed_uid) = ratline_label(autorouter, rec.failed);
        let (blocker_net, blocker_uid) = rec
            .blocker
            .map(|blocker| ratline_label(autorouter, blocker))
            .unwrap_or_default();
        writeln!(
            out,
            "{},{},{},{},{},{},{:?}",
            rec.seq, rec.position, failed_net, failed_uid, blocker_net, blocker_uid, rec.obstacle,
        )?;
    }
    Ok(())
}

/// Runs `args.jobs` independent routing attempts in parallel, each on
/// its own board copy and with a different preconfiguration shuffle
/// seed, then keeps the invoker that routed the most ratlines. The
/// first attempt to route everything flags the others to finish early.
fn run_parallel_jobs(args: &Cli) -> Result<Invoker<SpecctraMesadata>, std::io::Error> {
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
            apply_shift_report(&mut board, report_filename)?;
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
        let max_seconds = args.max_seconds;
        let max_steps = args.max_steps;
        let deterministic = args.deterministic;
        let progress_csv = args.progress_csv.clone();
        let stop = Arc::clone(&stop);
        let preprobe_seed = preprobe_seed.clone();
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

                let selection = make_selection(&board, &nets, &exclude_nets, multilayer);
                let multilayer_timeout = if deterministic {
                    TimeoutOptions {
                        initial: f64::INFINITY,
                        progress_bonus: 0.0,
                    }
                } else {
                    TimeoutOptions {
                        initial: 5.0,
                        progress_bonus: 0.5,
                    }
                };
                let command = if critical_first {
                    Command::CriticalAutoroute(
                        selection,
                        CriticalAutorouteOptions {
                            planar: planar_options,
                            multilayer: multilayer.then(|| MultilayerAutorouteOptions {
                                anterouter: AnterouterOptions {
                                    fanout_clearance: fanout_clearance(&board),
                                },
                                planar: planar_options,
                                timeout: multilayer_timeout,
                                layer_seed: i as u64,
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
                        MultilayerAutorouteOptions {
                            anterouter: AnterouterOptions {
                                fanout_clearance: fanout_clearance(&board),
                            },
                            planar: planar_options,
                            timeout: multilayer_timeout,
                            layer_seed: i as u64,
                        },
                    )
                } else {
                    Command::Autoroute(selection, planar_options)
                };

                let mut invoker =
                    Invoker::new(Autorouter::new(board).map_err(|e| format!("{:?}", e))?);
                if let Some(seed) = &preprobe_seed {
                    invoker
                        .autorouter_mut()
                        .seed_congestion(seed.iter().copied());
                }
                let mut execute = invoker
                    .execute_stepper(command)
                    .map_err(|e| format!("{:?}", e))?;

                let start = Instant::now();
                let deadline = max_seconds.map(|s| start + Duration::from_secs_f64(s));
                let mut last_report = Instant::now();
                let mut finished_early = false;
                let mut steps = 0usize;
                let mut csv = progress_csv
                    .as_deref()
                    .map(|path| ProgressCsv::new(&ProgressCsv::job_path(path, i)))
                    .transpose()
                    .map_err(|e| format!("{:?}", e))?;

                loop {
                    let step_start = Instant::now();
                    let status = execute.step(&mut invoker).map_err(|e| format!("{:?}", e))?;
                    let duration = step_start.elapsed();
                    steps += 1;

                    let kind = match status {
                        ControlFlow::Break(..) => break,
                        ControlFlow::Continue(kind) => kind,
                    };

                    topola::metrics::record_step(kind, duration);

                    if !finished_early
                        && (deadline.is_some_and(|d| Instant::now() >= d)
                            || max_steps.is_some_and(|max| steps >= max)
                            || stop.load(Ordering::Relaxed))
                    {
                        execute.finish_early(invoker.autorouter_mut());
                        finished_early = true;
                    }

                    if let Some(csv) = csv.as_mut() {
                        csv.write(
                            &StepReport {
                                invoker: &invoker,
                                stepper: &execute,
                                index: steps,
                                duration,
                                kind,
                            },
                            start,
                        );
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
        std::io::Error::new(
            std::io::ErrorKind::Other,
            "all parallel routing jobs failed",
        )
    })
}

fn humantime(d: Duration) -> String {
    let s = d.as_secs();
    format!("{}m{:02}s", s / 60, s % 60)
}
