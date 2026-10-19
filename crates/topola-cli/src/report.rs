// SPDX-FileCopyrightText: 2025 Topola contributors
//
// SPDX-License-Identifier: MIT

//! Run telemetry and result reporting: the per-step progress CSV, the
//! final unrouted/unconnected summary, and the blocker-attribution
//! dump.

use std::collections::BTreeMap;
use std::fs::File;
use std::io::{BufWriter, Write};
use std::path::{Path, PathBuf};
use std::time::{Duration, Instant};

use topola::autorouter::connected_components::ConnectedComponents;
use topola::autorouter::execution::StepKind;
use topola::autorouter::invoker::StepReport;
use topola::autorouter::ratline::RatlineUid;
use topola::autorouter::Autorouter;
use topola::board::AccessMesadata;
use topola::drawing::graph::{GetMaybeNet, MakePrimitiveRef};
use topola::graph::{GetIndex, MakeRef};
use topola::specctra::mesadata::SpecctraMesadata;
use topola::stepper::EstimateProgress;

pub fn humantime(d: Duration) -> String {
    let s = d.as_secs();
    format!("{}m{:02}s", s / 60, s % 60)
}

/// Counts (routed, total) ratlines on the principal-layer ratsnest.
pub fn routed_counts(autorouter: &Autorouter<SpecctraMesadata>) -> (usize, usize) {
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

/// Union-find connectivity check of ratlines across all layers —
/// the truth behind `routed_counts`' band-presence check: a ratline
/// can carry a band yet be electrically open (e.g. a dangling weave
/// half). GND is exempted, matching the final report. Returns the
/// open ratlines with their net names.
fn unconnected_ratlines(
    autorouter: &Autorouter<SpecctraMesadata>,
) -> Vec<(RatlineUid, Option<String>)> {
    let conncomps = ConnectedComponents::new(autorouter.board());
    let mut unconnected = Vec::new();

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
            let name = net.and_then(|n| autorouter.board().mesadata().net_netname(n));
            if name == Some("GND") {
                continue;
            }
            if conncomps.unionfind().find(origin.index())
                != conncomps.unionfind().find(destination.index())
            {
                unconnected.push((uid, name.map(str::to_owned)));
            }
        }
    }

    unconnected
}

/// Net name and `layer:index` label for a ratline, for reports.
fn ratline_label(autorouter: &Autorouter<SpecctraMesadata>, uid: RatlineUid) -> (String, String) {
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

/// The `[topola] finished:` line plus the unconnected-ratline
/// listing, per-net unrouted breakdown, and requested-nets verdict.
/// Printed on both the single-run and `--jobs` paths.
pub fn print_final_summary(autorouter: &Autorouter<SpecctraMesadata>, requested_nets: &[String]) {
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
            let (netname, _) = ratline_label(autorouter, uid);
            *unrouted.entry(netname).or_default() += 1;
        }
    }

    let unrouted_count: usize = unrouted.values().sum();
    let unconnected = unconnected_ratlines(autorouter);
    eprintln!(
        "[topola] finished: {} ratlines total, {} unrouted, {} unconnected",
        total,
        unrouted_count,
        unconnected.len()
    );

    for (uid, name) in &unconnected {
        eprintln!(
            "[topola] UNCONNECTED {:?} pl{} {:?}",
            uid.index, uid.principal_layer, name
        );
    }
    if !unconnected.is_empty() {
        eprintln!("[topola] connectivity: {} unconnected", unconnected.len());
    }
    for (netname, count) in &unrouted {
        eprintln!("[topola]   unrouted: {} ({} ratlines)", netname, count);
    }

    if !requested_nets.is_empty() {
        let still_open: Vec<_> = requested_nets
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

/// Per-step progress CSV writer. Emits a row when the ratline position
/// or step kind changed, plus heartbeat rows at most every 500 ms.
/// The unconnected column is a full-board union-find scan, so it is
/// recomputed at most every 2 s and the last-known value is written
/// in between.
pub struct ProgressCsv {
    out: BufWriter<File>,
    last_row: Option<(usize, StepKind)>,
    last_emit: Instant,
    last_unconn_at: Option<Instant>,
    unconnected: Option<usize>,
}

impl ProgressCsv {
    pub fn new(path: &Path) -> std::io::Result<Self> {
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
    pub fn job_path(path: &Path, job: usize) -> PathBuf {
        let stem = path
            .file_stem()
            .map(|stem| stem.to_string_lossy().into_owned())
            .unwrap_or_default();
        path.with_file_name(format!("{stem}-j{job}.csv"))
    }

    pub fn write(&mut self, report: &StepReport<SpecctraMesadata>, start: Instant) {
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
            self.unconnected = Some(unconnected_ratlines(autorouter).len());
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

/// Drains the failure-attribution log — one row per (failing ratline,
/// blocking obstacle) pair recorded at each routing failure.
pub fn write_blockers_csv(
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
