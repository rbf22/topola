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

/// Compass fallback directions tried after the endpoint-gap bearings —
/// the gap survey only sees endpoint pads, so a corridor that misses
/// every gap center (e.g. aligned off-axis or ringed by non-endpoint
/// copper) is only reachable this way.
const COMPASS_DIRECTIONS: usize = 8;

/// Minimum angular separation between tried directions — deduplicates
/// compass bearings that land on an already-tried gap center.
const MIN_DIRECTION_SEPARATION: f64 = 0.1;

/// The crowded-pad analysis shared by eager and lazy escape stubbing.
/// Computed once at preconfigure; laying geometry is deferred in lazy
/// mode so stubs are only paid for pads whose ratlines actually fail.
pub struct PadEscapeAnalysis {
    /// Every endpoint dot's center on the principal layer.
    pub positions: BTreeMap<FixedDotIndex, Point>,
    /// Endpoint dot → ratlines terminating at it.
    pub pad_ratlines: BTreeMap<FixedDotIndex, Vec<RatlineUid>>,
    /// 2.5 × median endpoint pitch — the longest stub length tried.
    pub radius: f64,
    /// Crowded pad → escape bearings, widest gap first.
    pub crowded: BTreeMap<FixedDotIndex, Vec<f64>>,
}

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

/// Surveys `ratlines`' endpoint pads on the principal layer: positions,
/// per-pad ratline lists, the pitch-adaptive neighborhood radius, and
/// for every surrounded pad the ranked escape bearings. Pure analysis —
/// places no geometry.
pub fn analyze_pads(
    autorouter: &Autorouter<impl AccessMesadata>,
    ratlines: &[RatlineUid],
    options: &PlanarAutorouteOptions,
) -> PadEscapeAnalysis {
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

    pitches.sort_by(|a, b| a.total_cmp(b));
    let radius = pitches.get(pitches.len() / 2).copied().unwrap_or(0.0) * 2.5;

    let mut crowded = BTreeMap::new();

    for (&pad, &pos) in &positions {
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

        crowded.insert(
            pad,
            gaps.into_iter().map(|(_, direction)| direction).collect(),
        );
    }

    PadEscapeAnalysis {
        positions,
        pad_ratlines,
        radius,
        crowded,
    }
}

/// Tries to lay a stub for `pad` along `directions`, each at the three
/// candidate lengths derived from `radius`. When `compass_fallback` is
/// set, the 8 compass bearings are appended after the gap directions —
/// worthwhile on demand (a failing ratline has nothing to lose) but
/// harmful in the eager pass, where each extra stub is more un-rippable
/// foreign-net geometry (see FINDINGS I17).
///
/// On success the drawn dot and seg are merged into `recorder` and the
/// stub dot is returned.
pub fn stub_pad(
    autorouter: &mut Autorouter<impl AccessMesadata>,
    recorder: &mut LayoutEdit,
    pad: FixedDotIndex,
    pos: Point,
    radius: f64,
    directions: &[f64],
    width: f64,
    compass_fallback: bool,
) -> Option<FixedDotIndex> {
    stub_direction_candidates(directions, compass_fallback)
        .into_iter()
        .flat_map(|direction| {
            [radius, radius * 0.5, radius * 0.25].map(move |length| (direction, length))
        })
        .find_map(|(direction, length)| {
            try_stub(autorouter, recorder, pad, pos, direction, length, width)
        })
        .map(|(_, stub)| stub)
}

/// The ordered bearings tried for a stub: the first
/// `MAX_GAP_CANDIDATES` gap centers, then — only with
/// `compass_fallback` — the 8 compass directions, deduplicated to
/// `MIN_DIRECTION_SEPARATION`.
fn stub_direction_candidates(directions: &[f64], compass_fallback: bool) -> Vec<f64> {
    let mut tried: Vec<f64> = Vec::new();
    let mut push = |direction: f64| {
        let direction = direction.rem_euclid(std::f64::consts::TAU);
        let novel = tried
            .iter()
            .all(|&t| angular_distance(t, direction) >= MIN_DIRECTION_SEPARATION);
        if novel {
            tried.push(direction);
        }
    };

    directions
        .iter()
        .take(MAX_GAP_CANDIDATES)
        .for_each(|&d| push(d));
    if compass_fallback {
        (0..COMPASS_DIRECTIONS).for_each(|i| push(i as f64 * std::f64::consts::FRAC_PI_4));
    }
    tried
}

/// Absolute angular separation between two bearings, in `[0, π]`.
fn angular_distance(a: f64, b: f64) -> f64 {
    let diff = (a - b).rem_euclid(std::f64::consts::TAU);
    diff.min(std::f64::consts::TAU - diff)
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
    let analysis = analyze_pads(autorouter, ratlines, options);
    let mut stats = EscapeStubStats::default();

    // Crowded-first: interior pads escape before outer pads' stubs can
    // wall them in.
    let mut pads: Vec<_> = analysis.crowded.keys().copied().collect();
    pads.sort_by(|a, b| {
        crowding(&analysis.positions, *b, analysis.radius)
            .cmp(&crowding(&analysis.positions, *a, analysis.radius))
            .then(a.cmp(b))
    });

    for pad in pads {
        stats.crowded_pads += 1;

        let Some(stub) = stub_pad(
            autorouter,
            recorder,
            pad,
            analysis.positions[&pad],
            analysis.radius,
            &analysis.crowded[&pad],
            width,
            false,
        ) else {
            stats.failed_pads += 1;
            continue;
        };

        stats.stubbed_pads += 1;

        for ratline in &analysis.pad_ratlines[&pad] {
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
    let stub_pos = pos + Point::new(direction.cos(), direction.sin()) * length;

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

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::{FRAC_PI_2, FRAC_PI_4, PI};

    #[test]
    fn angular_distance_wraps_around() {
        assert!(angular_distance(0.0, std::f64::consts::TAU - 0.05) < 0.06);
        assert!((angular_distance(0.0, PI) - PI).abs() < 1e-9);
    }

    /// With the fallback on, gap directions come first and compass
    /// bearings that don't collide with them are appended — a corridor
    /// off every gap center is still reachable.
    #[test]
    fn stub_candidates_include_compass_fallbacks() {
        let gaps = [0.13, 1.7, 3.9];
        let candidates = stub_direction_candidates(&gaps, true);

        assert_eq!(&candidates[..3], &[0.13, 1.7, 3.9]);
        assert!(candidates.len() >= 8);
        for i in 1..8 {
            let compass = i as f64 * FRAC_PI_4;
            let near_gap = gaps
                .iter()
                .any(|&g| angular_distance(g, compass) < MIN_DIRECTION_SEPARATION);
            assert_eq!(candidates.contains(&compass), !near_gap);
        }
    }

    /// Without the fallback only the gap bearings are tried — the
    /// eager pass must not sprawl into un-surveyed directions.
    #[test]
    fn stub_candidates_skip_compass_when_fallback_off() {
        let gaps = [0.13, 1.7];
        assert_eq!(stub_direction_candidates(&gaps, false), vec![0.13, 1.7]);
    }

    /// A gap bearing landing on a compass direction suppresses the
    /// duplicate — each bearing is tried at most once.
    #[test]
    fn stub_candidates_dedup_near_identical_bearings() {
        let gaps = [FRAC_PI_2 + 0.02, FRAC_PI_2 - 0.02];
        let candidates = stub_direction_candidates(&gaps, true);

        let near_vertical = candidates
            .iter()
            .filter(|&&d| angular_distance(d, FRAC_PI_2) < 0.2)
            .count();
        assert_eq!(near_vertical, 1, "the two near-vertical gaps collapse");
    }
}
