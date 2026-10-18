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
use std::io::Write;

use geo::{Line, Point};
use serde::Serialize;

use topola::{
    autorouter::Autorouter,
    board::AccessMesadata,
    drawing::{
        graph::{GetMaybeNet, MakePrimitiveRef, PrimitiveIndex},
        primitive::MakePrimitiveShape,
        rules::{AccessRules, Conditions},
    },
    geometry::{primitive::PrimitiveShape, shape::AccessShape, GenericNode},
    router::RouterOptions,
    specctra::mesadata::SpecctraMesadata,
};

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

/// Projects `(x, y)` onto the unit normal `n` relative to `origin`.
fn project(origin: Point, n: (f64, f64), x: f64, y: f64) -> f64 {
    (x - origin.x()) * n.0 + (y - origin.y()) * n.1
}

/// Interval of a shape's bounding box along the unit vector `dir`,
/// relative to `origin`.
fn extent_along(origin: Point, dir: (f64, f64), shape: &PrimitiveShape) -> (f64, f64) {
    let lo = shape.bbox_without_margin().lower();
    let hi = shape.bbox_without_margin().upper();
    [(lo[0], lo[1]), (hi[0], lo[1]), (hi[0], hi[1]), (lo[0], hi[1])]
        .into_iter()
        .fold((f64::INFINITY, f64::NEG_INFINITY), |(lo_s, hi_s), (x, y)| {
            (lo_s.min(project(origin, dir, x, y)), hi_s.max(project(origin, dir, x, y)))
        })
}

/// The smallest displacement along unit vector `n` that moves `shape`'s
/// bounding box outside `[-half_width, half_width]` around `origin`, on
/// the side the shape's center already occupies. Returns `None` when the
/// box already clears.
fn clearing_shift(
    origin: Point,
    n: (f64, f64),
    half_width: f64,
    center: Point,
    shape: &PrimitiveShape,
) -> Option<(f64, f64)> {
    let (s_min, s_max) = extent_along(origin, n, shape);
    let magnitude = if project(origin, n, center.x(), center.y()) >= 0.0 {
        half_width - s_min
    } else {
        s_max + half_width
    };
    if magnitude <= 0.0 {
        return None;
    }
    let sign = if project(origin, n, center.x(), center.y()) >= 0.0 {
        1.0
    } else {
        -1.0
    };
    Some((n.0 * sign * magnitude, n.1 * sign * magnitude))
}

pub fn write_suggestions<W: Write>(
    autorouter: &mut Autorouter<SpecctraMesadata>,
    router_options: RouterOptions,
    mut writer: W,
) -> Result<(), std::io::Error> {
    let band_half_width = router_options.routed_band_width / 2.0;
    let probes = autorouter.probe_channels(router_options, |rules| {
        rules.override_clearances(0.0)
    });

    // Rules are restored by now; use them for per-net-pair clearances.
    let rules = autorouter.board().layout().drawing().rules().clone();
    let mesadata = autorouter.board().mesadata();
    let netname = |net| mesadata.net_netname(net);

    let mut channels = vec![];
    let mut shifts: BTreeMap<String, (f64, f64, usize)> = BTreeMap::new();

    for probe in &probes {
        let board = autorouter.board();
        let drawing = board.layout().drawing();
        let probe_net = probe.net.unwrap_or(0);

        let mut blockers = vec![];
        let mut push = |obstacle: PrimitiveIndex, shift: (f64, f64)| {
            let blocker_primitive = obstacle.primitive_ref(drawing);
            if probe.net.is_some() && blocker_primitive.maybe_net() == probe.net {
                return; // Same-net geometry is connectable, not an obstacle.
            }

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
        };

        if probe.path.is_empty() {
            // Relaxed route failed: the terminal infringement names the
            // last obstacle. Push it away from where the band wanted to
            // be, by enough to separate the clearance-inflated ghosts.
            if let Some((infringer, infringee, obstacle)) = probe.terminal_blocker {
                let delta = infringee.center() - infringer.center();
                let len = (delta.x() * delta.x() + delta.y() * delta.y()).sqrt();
                if len > 0.0 {
                    let dir = (delta.x() / len, delta.y() / len);
                    let (o_lo, _) = extent_along(infringer.center(), dir, &infringee);
                    let (_, g_hi) = extent_along(infringer.center(), dir, &infringer);
                    let needed = g_hi - o_lo;
                    if needed > 0.0 {
                        push(obstacle, (dir.0 * needed, dir.1 * needed));
                    }
                }
            }
        } else {
            // Corridor along each demand path segment.
            for window in probe.path.windows(2) {
                let (a, layer_a) = window[0];
                let (b, layer_b) = window[1];
                let delta = b - a;
                let len = (delta.x() * delta.x() + delta.y() * delta.y()).sqrt();
                if len == 0.0 || layer_a != layer_b {
                    continue; // Degenerate or a layer transition (via).
                }
                let n = (-delta.y() / len, delta.x() / len);
                let corridor_width =
                    band_half_width * 2.0 + 2.0 * rules.largest_clearance(None);

                for obstacle in drawing.cut(Line::new(a, b), corridor_width, layer_a) {
                    let blocker_net = obstacle
                        .primitive_ref(drawing)
                        .maybe_net()
                        .unwrap_or(0);
                    let layer = Some(layer_a);
                    let clearance = rules
                        .clearance(
                            &Conditions {
                                net: probe_net,
                                maybe_region: None,
                                maybe_layer: layer,
                            },
                            &Conditions {
                                net: blocker_net,
                                maybe_region: None,
                                maybe_layer: layer,
                            },
                        )
                        .unwrap_or(0.0);
                    let half_width = band_half_width + clearance;
                    let blocker_primitive = obstacle.primitive_ref(drawing);
                    let shape = blocker_primitive.shape();
                    if let Some(shift) =
                        clearing_shift(a, n, half_width, shape.center(), &shape)
                    {
                        push(obstacle, shift);
                    }
                }
            }
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
            to: node_name(probe.endpoints.1)
                .unwrap_or_else(|| format!("{:?}", probe.endpoints.1)),
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
                    (owner, Shift { dx, dy, blocker_count })
                })
                .collect(),
        },
    )?;
    writeln!(writer)
}
