// SPDX-FileCopyrightText: 2025 Topola contributors
//
// SPDX-License-Identifier: MIT

//! Escape stub pre-routing for dense pad fields ("BGA escape").
//!
//! Before the main routing pass, endpoint pads that are surrounded by
//! other ratline endpoints get a fixed seg laid from the pad through
//! the widest local gap to a fresh stub dot outside the crowded
//! neighborhood, and the ratlines terminating at the pad are remapped
//! to the stub dot via
//! [`PlanarAutorouteConfiguration::terminating_dot_map`]. The actual
//! band then only has to reach open space, where routing is easy.
//!
//! Stubs are laid most-crowded-first so interior pads claim their
//! escape corridors before outer pads' stubs can wall them in. A stub
//! seg is invisible to navmeshes of its own net (same-net fixed segs
//! are not constrained) but appears as a constraint edge to every
//! foreign net, so corridor reservation survives later reroutes.

use std::collections::BTreeMap;

use geo::{Distance, Euclidean, Point};
use specctra_core::mesadata::AccessMesadata;

use crate::{
    autorouter::{ratline::RatlineUid, Autorouter, PlanarAutorouteOptions},
    drawing::{
        dot::{FixedDotIndex, FixedDotWeight, GeneralDotWeight},
        graph::{GetMaybeNet, MakePrimitiveRef},
        primitive::MakePrimitiveShape,
        seg::{FixedSegWeight, GeneralSegWeight},
    },
    geometry::{
        edit::{ApplyGeometryEdit, Edit},
        shape::AccessShape,
        GetLayer,
    },
    graph::MakeRef,
    layout::LayoutEdit,
    math::Circle,
};

/// A pad needs an escape stub when its neighbors cover more than half
/// of the circle around it, i.e. the widest bearing gap to a neighbor
/// is below this angle.
const INTERIOR_MAX_GAP: f64 = std::f64::consts::PI;

/// How many gap directions are tried per pad, each at three lengths.
const MAX_GAP_CANDIDATES: usize = 4;

/// Outcome of the escape-stub pre-routing pass.
#[derive(Debug, Default, Clone, Copy)]
pub struct EscapeStubStats {
    /// Endpoint pads found to be surrounded.
    pub crowded_pads: usize,
    /// Pads that got an escape stub.
    pub stubbed_pads: usize,
    /// Surrounded pads where no direction admitted a stub.
    pub failed_pads: usize,
}

/// Lays escape stubs for the surrounded endpoint pads of `ratlines`,
/// recording drawn geometry into `recorder` and remapping the affected
/// ratlines' terminating dots in `terminating_dot_map`.
pub fn anteroute_escape_stubs(
    autorouter: &mut Autorouter<impl AccessMesadata>,
    recorder: &mut LayoutEdit,
    ratlines: &[RatlineUid],
    terminating_dot_map: &mut BTreeMap<(RatlineUid, FixedDotIndex), FixedDotIndex>,
    options: &PlanarAutorouteOptions,
) -> EscapeStubStats {
    let width = options.router.routed_band_width;

    // Endpoint dots of the selected ratlines on the principal layer,
    // each paired with the ratlines touching it.
    let mut pad_ratlines: BTreeMap<FixedDotIndex, Vec<RatlineUid>> = BTreeMap::new();
    let mut positions: BTreeMap<FixedDotIndex, Point> = BTreeMap::new();

    for ratline in ratlines {
        let (source, target) = ratline.ref_(autorouter).endpoint_dots();

        for dot in [source, target] {
            if dot
                .primitive_ref(autorouter.board().layout().drawing())
                .layer()
                != options.principal_layer
            {
                continue;
            }

            pad_ratlines.entry(dot).or_default().push(*ratline);
            positions.entry(dot).or_insert_with(|| {
                dot.primitive_ref(autorouter.board().layout().drawing())
                    .shape()
                    .center()
            });
        }
    }

    // The neighborhood radius adapts to the board's pad pitch: a fixed
    // multiple of the band width would miscount neighbors on boards
    // whose pads sit many wire widths apart. Two and a half median
    // pitches covers the nearest two rings of endpoints.
    let mut pitches: Vec<_> = positions
        .values()
        .filter_map(|pos| {
            positions
                .iter()
                .filter(|(_, other)| *other != pos)
                .map(|(_, other)| Euclidean::distance(*pos, *other))
                .min_by(|a, b| a.total_cmp(b))
        })
        .collect();

    if pitches.is_empty() {
        return EscapeStubStats::default();
    }

    pitches.sort_by(|a, b| a.total_cmp(b));
    let pitch = pitches[pitches.len() / 2];

    let radius = 2.5 * pitch;
    let mut stats = EscapeStubStats::default();

    // Crowded-first: interior pads escape before outer pads' stubs can
    // wall them in.
    let mut pads: Vec<_> = pad_ratlines.keys().copied().collect();
    pads.sort_by(|a, b| {
        crowding(&positions, *b, radius)
            .cmp(&crowding(&positions, *a, radius))
            .then(a.cmp(b))
    });

    for pad in pads {
        let pos = positions[&pad];

        let mut neighbors: Vec<(f64, f64)> = positions
            .iter()
            .filter(|(other, _)| **other != pad)
            .filter_map(|(_other, other_pos)| {
                let distance = Euclidean::distance(pos, *other_pos);
                (distance <= radius).then(|| {
                    (
                        distance,
                        (other_pos.y() - pos.y()).atan2(other_pos.x() - pos.x()),
                    )
                })
            })
            .collect();

        if neighbors.len() < 2 {
            continue;
        }

        neighbors.sort_by(|a, b| a.1.total_cmp(&b.1));

        // Bearing gaps between consecutive neighbors, wrap-around
        // included. A pad is surrounded when even its widest gap is
        // below INTERIOR_MAX_GAP.
        let mut gaps: Vec<(f64, f64)> = neighbors
            .iter()
            .enumerate()
            .map(|(i, (_, bearing))| {
                let next_bearing = neighbors
                    .get(i + 1)
                    .map_or(neighbors[0].1 + 2.0 * std::f64::consts::PI, |next| next.1);
                let gap = next_bearing - bearing;
                (gap, bearing + gap / 2.0)
            })
            .collect();

        gaps.sort_by(|a, b| b.0.total_cmp(&a.0));

        if gaps[0].0 >= INTERIOR_MAX_GAP {
            continue;
        }

        stats.crowded_pads += 1;

        let Some((_, stub)) = gaps
            .iter()
            .take(MAX_GAP_CANDIDATES)
            .flat_map(|(_, direction)| {
                [radius, radius * 0.5, radius * 0.25].map(move |length| (*direction, length))
            })
            .find_map(|(direction, length)| {
                try_stub(autorouter, recorder, pad, pos, direction, length, width)
            })
        else {
            stats.failed_pads += 1;
            continue;
        };

        stats.stubbed_pads += 1;

        for ratline in &pad_ratlines[&pad] {
            terminating_dot_map.insert((*ratline, pad), stub);
        }
    }

    stats
}

fn crowding(positions: &BTreeMap<FixedDotIndex, Point>, dot: FixedDotIndex, radius: f64) -> usize {
    let pos = positions[&dot];
    positions
        .values()
        .filter(|other| Euclidean::distance(pos, **other) <= radius)
        .count()
}

/// Tries to place a stub dot at `pos + direction * length` and connect
/// it to `pad` with a fixed seg. On success the scratch edit is merged
/// into `recorder` and the stub dot is returned; on failure the
/// scratch edit is reversed.
fn try_stub(
    autorouter: &mut Autorouter<impl AccessMesadata>,
    recorder: &mut LayoutEdit,
    pad: FixedDotIndex,
    pos: Point,
    direction: f64,
    length: f64,
    width: f64,
) -> Option<((), FixedDotIndex)> {
    let stub_pos = pos
        + Point::new(direction.cos(), direction.sin()) * length;

    let primitive = pad.primitive_ref(autorouter.board().layout().drawing());
    let layer = primitive.layer();
    let maybe_net = primitive.maybe_net();

    let layout = autorouter.board.layout_mut();
    let mut scratch = LayoutEdit::new();

    let placed = layout
        .add_fixed_dot(
            &mut scratch,
            FixedDotWeight(GeneralDotWeight {
                circle: Circle {
                    pos: stub_pos,
                    r: width,
                },
                layer,
                maybe_net,
            }),
        )
        .ok()
        .and_then(|stub| {
            layout
                .add_fixed_seg(
                    &mut scratch,
                    pad,
                    stub,
                    FixedSegWeight(GeneralSegWeight {
                        width,
                        layer,
                        maybe_net,
                    }),
                )
                .ok()
                .map(|_| stub)
        });

    if let Some(stub) = placed {
        recorder.merge(scratch);
        Some(((), stub))
    } else {
        layout.apply(&scratch.reverse());
        None
    }
}
