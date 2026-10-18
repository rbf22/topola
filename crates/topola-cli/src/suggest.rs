// SPDX-FileCopyrightText: 2024 Topola contributors
//
// SPDX-License-Identifier: MIT

//! Blocker analysis and component shift suggestions.
//!
//! For every unrouted ratline we intersect a straight "wish channel"
//! between its endpoints with the board's primitives and compute, per
//! blocking primitive, the smallest perpendicular displacement of the
//! blocker's owner (a component's reference designator, or a wire's net)
//! that would move its bounding box outside the channel. Suggestions are
//! meant to be applied by the board generator, after which the board is
//! re-exported and routed again.

use std::collections::BTreeMap;
use std::io::Write;

use geo::Line;
use serde::Serialize;

use topola::{
    autorouter::{ratline::RatlineUid, Autorouter},
    board::AccessMesadata,
    drawing::{
        graph::{GetMaybeNet, MakePrimitiveRef},
        primitive::MakePrimitiveShape,
    },
    geometry::{shape::AccessShape, GenericNode, GetLayer},
    graph::MakeRef,
};

#[derive(Serialize)]
struct ShiftSuggestions {
    /// Coordinate units match the input Specctra file.
    units: &'static str,
    channel_half_width: f64,
    channels: Vec<ChannelReport>,
    /// Suggested displacement per owner, summed over all blockers.
    shifts: BTreeMap<String, Shift>,
}

#[derive(Serialize)]
struct ChannelReport {
    net: Option<String>,
    from: String,
    to: String,
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
    autorouter: &Autorouter<impl AccessMesadata>,
    channel_half_width: f64,
    mut writer: W,
) -> Result<(), std::io::Error> {
    let board = autorouter.board();
    let drawing = board.layout().drawing();
    let ratsnest = autorouter.ratsnests().on_principal_layer(0);

    let mut channels = vec![];
    let mut shifts: BTreeMap<String, (f64, f64, usize)> = BTreeMap::new();

    for edge in ratsnest.graph().edge_indices() {
        if ratsnest
            .graph()
            .edge_weight(edge)
            .is_some_and(|weight| weight.band_termseg.is_some())
        {
            continue; // Already routed.
        }

        let uid = RatlineUid {
            principal_layer: 0,
            index: edge,
        };
        let (from_dot, to_dot) = uid.ref_(autorouter).endpoint_dots();
        let from = from_dot.primitive_ref(drawing).shape().center();
        let to = to_dot.primitive_ref(drawing).shape().center();

        let delta = to - from;
        let len = (delta.x() * delta.x() + delta.y() * delta.y()).sqrt();
        if len == 0.0 {
            continue;
        }
        // Left-hand unit normal of the wish channel.
        let (nx, ny) = (-delta.y() / len, delta.x() / len);
        let layer = from_dot.primitive_ref(drawing).layer();
        let ratline_net = from_dot.primitive_ref(drawing).maybe_net();
        let netname = ratline_net.and_then(|net| board.mesadata().net_netname(net));

        let mut blockers = vec![];

        for blocker in drawing.cut(Line::new(from, to), 2.0 * channel_half_width, layer) {
            let blocker_primitive = blocker.primitive_ref(drawing);
            if ratline_net.is_some() && blocker_primitive.maybe_net() == ratline_net {
                continue; // Same-net geometry is connectable, not an obstacle.
            }

            let shape = blocker_primitive.shape();
            let center = shape.center();
            let lo = shape.bbox_without_margin().lower();
            let hi = shape.bbox_without_margin().upper();

            // Project the blocker's bbox onto the channel normal to find its
            // extent toward the channel's centerline.
            let project = |x: f64, y: f64| (x - from.x()) * nx + (y - from.y()) * ny;
            let (s_min, s_max) = [
                (lo[0], lo[1]),
                (hi[0], lo[1]),
                (hi[0], hi[1]),
                (lo[0], hi[1]),
            ]
            .into_iter()
            .fold(
                (f64::INFINITY, f64::NEG_INFINITY),
                |(lo_s, hi_s), (x, y)| (lo_s.min(project(x, y)), hi_s.max(project(x, y))),
            );
            let s_center = project(center.x(), center.y());

            // Displacement perpendicular to the channel that moves the
            // blocker's nearest bbox edge just outside the channel.
            let (dx, dy, shift_magnitude) = if s_center >= 0.0 {
                (nx, ny, channel_half_width - s_min)
            } else {
                (-nx, -ny, channel_half_width + s_max)
            };
            if shift_magnitude <= 0.0 {
                continue; // Already clears the channel.
            }
            let (shift_x, shift_y) = (dx * shift_magnitude, dy * shift_magnitude);

            let node = GenericNode::Primitive(blocker);
            let pinname = board.node_pinname(&node);
            let netname_of_blocker = blocker_primitive
                .maybe_net()
                .and_then(|net| board.mesadata().net_netname(net));
            let owner = pinname
                .and_then(|pin| pin.rsplit_once('-').map(|(refdes, _)| refdes.to_string()))
                .or_else(|| netname_of_blocker.map(|net| format!("wire:{net}")))
                .unwrap_or_else(|| "<board>".to_string());

            let entry = shifts.entry(owner.clone()).or_default();
            entry.0 += shift_x;
            entry.1 += shift_y;
            entry.2 += 1;

            blockers.push(BlockerReport {
                owner,
                primitive: pinname
                    .cloned()
                    .or_else(|| netname_of_blocker.map(str::to_owned))
                    .unwrap_or_else(|| format!("{blocker:?}")),
                center: [center.x(), center.y()],
                shift: [shift_x, shift_y],
            });
        }

        if !blockers.is_empty() {
            let node_name = |dot: topola::drawing::dot::FixedDotIndex| {
                board
                    .node_pinname(&GenericNode::Primitive(dot.into()))
                    .cloned()
            };
            channels.push(ChannelReport {
                net: netname.map(str::to_owned),
                from: node_name(from_dot).unwrap_or_else(|| format!("{from_dot:?}")),
                to: node_name(to_dot).unwrap_or_else(|| format!("{to_dot:?}")),
                blockers,
            });
        }
    }

    serde_json::to_writer_pretty(
        &mut writer,
        &ShiftSuggestions {
            units: "dsn",
            channel_half_width,
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
