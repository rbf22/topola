// SPDX-FileCopyrightText: 2024 Topola contributors
//
// SPDX-License-Identifier: MIT

//! Blocker analysis and component shift suggestions.
//!
//! Every unrouted ratline is probed under relaxed clearances
//! ([`Autorouter::probe_channels`]). A probe that succeeds yields the
//! demand path the connection would take; each primitive lying inside
//! the corridor at the real clearance is a blocker, pushed by the
//! smallest perpendicular displacement that moves it outside. A probe
//! that fails anyway reports the terminal infringement's obstacle,
//! pushed away from where the band wanted to be. Forces aggregate by
//! owner — a component's reference designator, or `wire:<net>` for
//! wire geometry — and are meant to be applied by the board generator
//! before re-exporting and routing again.

use std::collections::BTreeMap;
use std::fs::File;
use std::io::{BufReader, Write};
use std::path::Path;

use serde::{Deserialize, Serialize};

use topola::{
    autorouter::Autorouter,
    board::{edit::BoardEdit, AccessMesadata, Board},
    drawing::{
        graph::{GetMaybeNet, MakePrimitiveRef},
        primitive::MakePrimitiveShape,
    },
    geometry::{shape::AccessShape, GenericNode},
    router::RouterOptions,
    specctra::mesadata::SpecctraMesadata,
};

/// Applies the per-owner displacements of a `--suggest-shifts` report
/// to the board before routing ("component shifting" for congestion
/// relief). Only reference-designator owners are components;
/// `wire:<net>` blockers are routed geometry and `<board>` owners are
/// netless shapes, neither of which can be translated.
pub fn apply_report(
    board: &mut Board<SpecctraMesadata>,
    report_filename: &Path,
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

#[derive(Serialize)]
struct ShiftSuggestions {
    /// Coordinate units match the input Specctra file.
    units: &'static str,
    /// Routed band half-width used for corridor measurements.
    band_half_width: f64,
    channels: Vec<ChannelReport>,
    /// Suggested displacement per owner, summed over all blockers.
    shifts: BTreeMap<String, Shift>,
}

#[derive(Serialize)]
struct ChannelReport {
    net: Option<String>,
    from: String,
    to: String,
    /// Waypoints of the relaxed demand path, or empty when the probe
    /// could not route even with clearances disabled.
    path: Vec<[f64; 2]>,
    blockers: Vec<BlockerReport>,
}

#[derive(Serialize)]
struct BlockerReport {
    /// Reference designator for pins, `wire:<net>` for wire geometry, or
    /// `<board>` for netless primitives.
    owner: String,
    primitive: String,
    center: [f64; 2],
    shift: [f64; 2],
}

#[derive(Serialize)]
struct Shift {
    dx: f64,
    dy: f64,
    blocker_count: usize,
}

pub fn write_suggestions<W: Write>(
    autorouter: &mut Autorouter<SpecctraMesadata>,
    router_options: RouterOptions,
    jobs: usize,
    mut writer: W,
) -> Result<(), std::io::Error> {
    let band_half_width = router_options.routed_band_width / 2.0;
    let probes = if jobs > 1 {
        autorouter.probe_channels_par(
            router_options,
            &|rules: &mut SpecctraMesadata| rules.override_clearances(0.0),
            jobs,
        )
    } else {
        autorouter.probe_channels(router_options, |rules| rules.override_clearances(0.0))
    };

    // Rules are restored by now; blockers are measured at real
    // clearances inside `ChannelProbe::blockers`.
    let mesadata = autorouter.board().mesadata();
    let netname = |net| mesadata.net_netname(net);

    let mut channels = vec![];
    let mut shifts: BTreeMap<String, (f64, f64, usize)> = BTreeMap::new();

    for probe in &probes {
        let board = autorouter.board();
        let drawing = board.layout().drawing();

        let mut blockers = vec![];
        for blocker in probe.blockers(drawing, band_half_width) {
            let obstacle = blocker.obstacle;
            let shift = blocker.shift;
            let blocker_primitive = obstacle.primitive_ref(drawing);

            let node = GenericNode::Primitive(obstacle);
            let pinname = board.node_pinname(&node);
            let netname_of_blocker = blocker_primitive.maybe_net().and_then(netname);
            let owner = pinname
                .and_then(|pin| pin.rsplit_once('-').map(|(refdes, _)| refdes.to_string()))
                .or_else(|| netname_of_blocker.map(|net| format!("wire:{net}")))
                .unwrap_or_else(|| "<board>".to_string());

            let entry = shifts.entry(owner.clone()).or_default();
            entry.0 += shift.0;
            entry.1 += shift.1;
            entry.2 += 1;

            let center = blocker_primitive.shape().center();
            blockers.push(BlockerReport {
                owner,
                primitive: pinname
                    .cloned()
                    .or_else(|| netname_of_blocker.map(str::to_owned))
                    .unwrap_or_else(|| format!("{obstacle:?}")),
                center: [center.x(), center.y()],
                shift: [shift.0, shift.1],
            });
        }

        let node_name = |dot: topola::drawing::dot::FixedDotIndex| {
            board
                .node_pinname(&GenericNode::Primitive(dot.into()))
                .cloned()
        };
        channels.push(ChannelReport {
            net: probe.net.and_then(netname).map(str::to_owned),
            from: node_name(probe.endpoints.0)
                .unwrap_or_else(|| format!("{:?}", probe.endpoints.0)),
            to: node_name(probe.endpoints.1).unwrap_or_else(|| format!("{:?}", probe.endpoints.1)),
            path: probe
                .path
                .iter()
                .map(|(point, _)| [point.x(), point.y()])
                .collect(),
            blockers,
        });
    }

    serde_json::to_writer_pretty(
        &mut writer,
        &ShiftSuggestions {
            units: "dsn",
            band_half_width,
            channels,
            shifts: shifts
                .into_iter()
                .map(|(owner, (dx, dy, blocker_count))| {
                    (
                        owner,
                        Shift {
                            dx,
                            dy,
                            blocker_count,
                        },
                    )
                })
                .collect(),
        },
    )?;
    writeln!(writer)
}
