// SPDX-FileCopyrightText: 2025 Topola contributors
//
// SPDX-License-Identifier: MIT

//! Critical-first difficulty analysis.
//!
//! Relaxed-rule channel probes reveal each ratline's demand path (or its
//! terminal blocker). From those probes we derive a *criticality score*
//! per ratline combining four signals:
//!
//! - whether the probe failed even under relaxed rules (the strongest
//!   signal — the connection is fighting geometry itself),
//! - how many primitives block the demand corridor at real clearances,
//! - how many *other* ratlines' demand paths come within one band width
//!   (demand-polyline conflicts — the upgrade over the straight-line
//!   corridor conflict count used by the presort, since a real path may
//!   detour far from its chord),
//! - how crowded the ratline's endpoints are (escape density).
//!
//! The hardest ratlines — every relaxed-failure plus the top quantile by
//! score — form the *critical set*: connections that should be routed
//! and reserved before ordinary routing fills in around them.
//!
//! Probing is expected to happen on a *clone* of the live autorouter so
//! the probe pass never perturbs real routing state.

use std::collections::{BTreeMap, BTreeSet};

use geo::algorithm::line_measures::{Distance, Euclidean};
use geo::{Line, Point};
use rstar::{
    primitives::{GeomWithData, Rectangle},
    RTree, RTreeObject,
};

use crate::{
    autorouter::{ratline::RatlineUid, Autorouter},
    board::AccessMesadata,
    drawing::{dot::FixedDotIndex, graph::PrimitiveIndex, primitive::GetWeight},
    geometry::GetSetPos,
    graph::MakeRef,
    metrics::{incr, METRICS},
    router::RouterOptions,
};

/// The outcome of probing and scoring one batch of ratlines.
#[derive(Clone, Debug, Default)]
pub struct CriticalityReport {
    /// Combined difficulty score per probed ratline.
    pub scores: BTreeMap<RatlineUid, f64>,
    /// The ratlines that should route before everything else.
    pub critical: BTreeSet<RatlineUid>,
    /// Demand polylines for probes that produced one: `(center, layer)`
    /// waypoints in path order. Feeds corridor carving.
    pub paths: BTreeMap<RatlineUid, Vec<(Point, usize)>>,
    /// Corridors' blockers at real clearance, per ratline.
    pub blockers: BTreeMap<RatlineUid, Vec<PrimitiveIndex>>,
    /// How many other ratlines' demand paths conflict with each
    /// ratline's own.
    pub conflicts: BTreeMap<RatlineUid, usize>,
}

/// Score weight for failing the relaxed probe outright.
const FAIL_WEIGHT: f64 = 100.0;
/// Score weight per real-clearance corridor blocker.
const BLOCKER_WEIGHT: f64 = 4.0;
/// Score weight per conflicting rival demand path.
const CONFLICT_WEIGHT: f64 = 2.0;
/// Score weight per crowded endpoint (escape density).
const CROWD_WEIGHT: f64 = 0.25;

/// The clearance demand-path probes relax to: a quarter of the larger
/// of the board's largest clearance and the routed band width.
/// Probing at clearance zero produces degenerate navmesh geometry —
/// spurious "no path" failures on boards that are perfectly routable —
/// so probes relax only down to this small positive value, still far
/// looser than the board's real clearances.
pub fn relaxed_probe_clearance(
    rules: &impl crate::drawing::rules::AccessRules,
    band_width: f64,
) -> f64 {
    f64::max(rules.largest_clearance(None), band_width) / 4.0
}

/// Probes `ratlines` between the given endpoint dots (falling back to
/// each ratline's pad endpoints when absent from `endpoints`) and scores
/// them, returning the critical set plus the per-ratline demand paths,
/// blockers, and conflict counts the caller needs for carving and
/// congestion seeding.
///
/// `quantile` selects the score cut within (0, 1]: ratlines scoring in
/// the top `1 - quantile` of positive scores join every relaxed-failure
/// in the critical set.
///
/// `relax_rules` is the probe relaxation (typically
/// `override_clearances(0.0)`), matching [`Autorouter::probe_ratlines`].
pub fn analyze<M: AccessMesadata + Clone>(
    autorouter: &mut Autorouter<M>,
    ratlines: &BTreeSet<RatlineUid>,
    endpoints: &BTreeMap<RatlineUid, (FixedDotIndex, FixedDotIndex)>,
    router_options: RouterOptions,
    quantile: f64,
    relax_rules: impl FnOnce(&mut M),
) -> CriticalityReport {
    let mut report = CriticalityReport::default();
    if ratlines.is_empty() {
        return report;
    }

    let pairs: Vec<_> = ratlines
        .iter()
        .map(|&ratline| {
            let (origin, destination) = endpoints
                .get(&ratline)
                .copied()
                .unwrap_or_else(|| ratline.ref_(autorouter).endpoint_dots());
            (ratline, origin, destination)
        })
        .collect();

    let probes = autorouter.probe_ratlines_dots(&pairs, router_options, relax_rules);
    incr(&METRICS.critical_probes);

    let drawing = autorouter.board().layout().drawing();
    let band_half_width = router_options.routed_band_width / 2.0;

    // Per-ratline: net, failure flag, blockers, demand path segments.
    let mut nets = BTreeMap::new();
    let mut demand_segs: BTreeMap<RatlineUid, Vec<(Line, usize)>> = BTreeMap::new();

    for (probe, (ratline, _, _)) in probes.iter().zip(pairs.iter()) {
        nets.insert(*ratline, probe.net);
        log::debug!(
            "criticality: {:?} path_len={} blockers={}",
            ratline.index,
            probe.path.len(),
            probe.blockers(drawing, band_half_width).len(),
        );

        let blocker_list: Vec<_> = probe
            .blockers(drawing, band_half_width)
            .into_iter()
            .map(|blocker| blocker.obstacle)
            .collect();
        report.blockers.insert(*ratline, blocker_list);

        if probe.path.is_empty() {
            // Relaxed-rule failure is decisive on its own; no demand
            // path exists to carve or conflict with.
            report.scores.insert(*ratline, FAIL_WEIGHT);
            report.critical.insert(*ratline);
            continue;
        }

        report.paths.insert(*ratline, probe.path.clone());
        let segs: Vec<_> = probe
            .path
            .windows(2)
            .filter_map(|window| {
                let (a, layer_a) = window[0];
                let (b, layer_b) = window[1];
                (layer_a == layer_b && a != b).then(|| (Line::new(a, b), layer_a))
            })
            .collect();
        demand_segs.insert(*ratline, segs);
    }

    // Demand-path conflicts: pairs of same-layer demand segments from
    // different nets within one band width cannot both host a band.
    // Envelopes prune the O(n²) seg-pair search, as in
    // `compute_corridor_scores`.
    let margin = router_options.routed_band_width;
    let mut seg_list: Vec<(RatlineUid, Line, usize)> = vec![];
    for (ratline, segs) in &demand_segs {
        for &(seg, layer) in segs {
            seg_list.push((*ratline, seg, layer));
        }
    }
    let envelope_of = |line: &Line<f64>| {
        let envelope = rstar::AABB::from_points([&line.start_point(), &line.end_point()]);
        let (l, u) = (envelope.lower(), envelope.upper());
        Rectangle::from_corners(
            Point::new(l.x() - margin, l.y() - margin),
            Point::new(u.x() + margin, u.y() + margin),
        )
    };
    let envelopes: Vec<_> = seg_list
        .iter()
        .map(|(_, seg, _)| envelope_of(seg))
        .collect();
    let tree = RTree::bulk_load(
        envelopes
            .iter()
            .enumerate()
            .map(|(i, envelope)| GeomWithData::new(*envelope, i))
            .collect(),
    );

    let mut conflict_partners: BTreeMap<RatlineUid, BTreeSet<RatlineUid>> = BTreeMap::new();
    for (i, (ratline, seg, layer)) in seg_list.iter().enumerate() {
        for hit in tree.locate_in_envelope_intersecting(&envelopes[i].envelope()) {
            let j = hit.data;
            let (other_ratline, other_seg, other_layer) = seg_list[j];
            if *ratline == other_ratline || *layer != other_layer {
                continue;
            }
            let same_net = nets.get(ratline).copied().flatten().is_some()
                && nets.get(ratline).copied().flatten()
                    == nets.get(&other_ratline).copied().flatten();
            if same_net {
                continue;
            }
            if Euclidean::distance(seg, &other_seg) < margin {
                conflict_partners
                    .entry(*ratline)
                    .or_default()
                    .insert(other_ratline);
            }
        }
    }
    for (ratline, partners) in &conflict_partners {
        report.conflicts.insert(*ratline, partners.len());
    }

    // Escape density: number of endpoints within the crowding radius of
    // this ratline's own endpoints (score = the worse endpoint's count).
    let crowd_radius = 20.0 * router_options.routed_band_width;
    let dot_pos = |dot: FixedDotIndex| drawing.primitive_ref(dot).weight().pos();
    let endpoint_positions: Vec<_> = pairs
        .iter()
        .flat_map(|(_, origin, destination)| [dot_pos(*origin), dot_pos(*destination)])
        .collect();
    let escape_scores: BTreeMap<RatlineUid, f64> = pairs
        .iter()
        .map(|(ratline, origin, destination)| {
            let score = [*origin, *destination]
                .into_iter()
                .map(|dot| {
                    let pos = dot_pos(dot);
                    endpoint_positions
                        .iter()
                        .filter(|other| Euclidean::distance(pos, **other) <= crowd_radius)
                        .count() as f64
                })
                .fold(0.0, f64::max);
            (*ratline, score)
        })
        .collect();

    // Combine signals. Relaxed failures already got FAIL_WEIGHT.
    for (ratline, _, _) in &pairs {
        let entry = report.scores.entry(*ratline).or_insert(0.0);
        *entry += BLOCKER_WEIGHT * report.blockers[ratline].len() as f64
            + CONFLICT_WEIGHT * report.conflicts.get(ratline).copied().unwrap_or(0) as f64
            + CROWD_WEIGHT * escape_scores[ratline];
    }

    // Critical threshold: the quantile cut over positive scores —
    // ratlines with zero measurable difficulty are never critical.
    let mut positive: Vec<_> = report
        .scores
        .values()
        .copied()
        .filter(|score| *score > 0.0)
        .collect();
    positive.sort_by(f64::total_cmp);
    if !positive.is_empty() {
        let cut_index = ((positive.len() as f64) * quantile.clamp(0.0, 1.0)) as usize;
        let cut = positive[cut_index.min(positive.len() - 1)];
        for (ratline, score) in &report.scores {
            if *score >= cut && *score > 0.0 {
                report.critical.insert(*ratline);
            }
        }
    }

    incr(&METRICS.critical_identified);
    report
}
