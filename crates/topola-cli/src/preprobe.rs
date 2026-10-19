// SPDX-FileCopyrightText: 2025 Topola contributors
//
// SPDX-License-Identifier: MIT

//! Up-front congestion probing (`--preprobe`): relaxed-clearance
//! channel probes on a clone of the board, whose blockers seed the
//! autorouter's congestion map before routing starts.

use topola::autorouter::selection::PinSelection;
use topola::autorouter::Autorouter;
use topola::drawing::graph::PrimitiveIndex;
use topola::router::RouterOptions;
use topola::specctra::mesadata::SpecctraMesadata;

/// Runs relaxed-clearance channel probes on a clone of `autorouter`
/// (leaving the routed board untouched) and returns the corridor
/// blockers found, as a flat list to feed `Autorouter::seed_congestion`.
/// When a selection is given, only its ratlines are probed so demand
/// that won't be routed doesn't seed congestion.
pub fn preprobe_blockers(
    autorouter: &Autorouter<SpecctraMesadata>,
    selection: Option<&PinSelection>,
    router_options: RouterOptions,
    jobs: usize,
) -> Vec<PrimitiveIndex> {
    let mut probe_autorouter = autorouter.clone();
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
