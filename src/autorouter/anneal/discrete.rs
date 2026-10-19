// SPDX-FileCopyrightText: 2026 Topola contributors
//
// SPDX-License-Identifier: MIT

//! Comparison baseline: simulated annealing directly over the
//! handoff's own degrees of freedom — per-ratline layer assignment and
//! routing order — scored by a corridor-conflict energy. No
//! continuous curves, no mid-route layer hops: the same Metropolis
//! machinery answers whether the paper's 3D formulation actually
//! beats a discrete search over the same decisions.

use std::collections::{BTreeMap, BTreeSet};

use geo::{Distance, Euclidean, Line, Point};
use rstar::{
    primitives::{GeomWithData, Rectangle},
    RTree, RTreeObject, AABB,
};
use specctra_core::mesadata::AccessMesadata;

use crate::{
    autorouter::{ratline::RatlineUid, Autorouter},
    drawing::{
        dot::FixedDotIndex,
        graph::{GetMaybeNet, MakePrimitiveRef, PrimitiveIndex},
        primitive::MakePrimitiveShape,
    },
    geometry::{
        primitive::{PrimitiveShape, SegShape},
        shape::AccessShape,
    },
    graph::MakeRef,
};

use super::{rng::AnnealRng, AnnealOptions, AnnealPlan};

/// One ratline's fixed corridor context.
struct Corridor {
    ratline: RatlineUid,
    line: Line<f64>,
    maybe_net: Option<usize>,
    preferred_layer: usize,
    /// Other-net obstruction count of this corridor on each layer —
    /// a corridor with nowhere legal to go is expensive everywhere.
    obstructions: Vec<usize>,
}

/// Corridor pair indices contending for the same channel.
type Conflicts = Vec<BTreeSet<usize>>;

/// Runs the discrete simulated annealer.
pub fn anneal(
    autorouter: &Autorouter<impl AccessMesadata>,
    ratlines: &BTreeSet<RatlineUid>,
    options: &AnnealOptions,
    band_width: f64,
) -> AnnealPlan {
    let layer_count = autorouter.board().layout().drawing().layer_count();
    let corridors = build_corridors(autorouter, ratlines, band_width, layer_count);
    if corridors.is_empty() {
        return AnnealPlan::default();
    }
    let n = corridors.len();
    let conflicts = build_conflicts(&corridors, band_width);

    let mut rng = AnnealRng::new(options.seed);
    // Initial state: preferred layers, order by descending conflict
    // count — the annealer's own contestedness heuristic.
    let mut assign: Vec<usize> = corridors.iter().map(|c| c.preferred_layer).collect();
    let mut order: Vec<usize> = (0..n).collect();
    order.sort_by_key(|&i| (std::cmp::Reverse(conflicts[i].len()), i));

    // pos[i] = order position of ratline i; inverse of `order`.
    let mut pos = vec![0usize; n];
    for (p, &i) in order.iter().enumerate() {
        pos[i] = p;
    }

    // Energy terms:
    //   E = w_rep · Σ_{same-layer conflicting pairs} 1
    //     + w_obs · Σ_i obstructions[i][assign[i]]
    //     + w_len · Σ_i [assign[i] ≠ preferred]
    //     + w_ord · Σ_{conflicting pairs} [pos(harder) > pos(easier)]
    // "Harder" = higher obstruction count on the assigned layer.
    let w_order = 0.5;

    let mut energy = total_energy(&corridors, &conflicts, &assign, &pos, options, w_order);
    let initial_energy = energy;
    let (mut temperature, cooling) = (options.t_start(), options.cooling());
    let mut crossings = 0usize;

    for _ in 0..options.iterations {
        for _ in 0..n {
            if rng.uniform() < 0.6 && layer_count > 1 {
                // Relayer move: ratline i takes a different layer.
                let i = rng.below(n);
                let old_layer = assign[i];
                let new_layer = rng.below(layer_count);
                if new_layer == old_layer {
                    continue;
                }
                let de = layer_delta(&corridors, &conflicts, &assign, i, new_layer, options);
                if rng.accept(de, temperature) {
                    assign[i] = new_layer;
                    energy += de;
                    crossings += 1;
                }
            } else {
                // Order swap: exchange two ratlines' positions.
                let (i, j) = (rng.below(n), rng.below(n));
                if i == j {
                    continue;
                }
                let de = order_delta(&corridors, &conflicts, &assign, &pos, i, j, w_order);
                if rng.accept(de, temperature) {
                    order.swap(pos[i], pos[j]);
                    pos.swap(i, j);
                    energy += de;
                }
            }
        }
        temperature *= cooling;
    }

    // Congestion seed: count each assigned corridor's infringements
    // per primitive — same semantics as the continuous mode's
    // near-miss accumulator.
    let mut congestion: BTreeMap<PrimitiveIndex, usize> = BTreeMap::new();
    let drawing = autorouter.board().layout().drawing();
    for (i, corridor) in corridors.iter().enumerate() {
        let shape = PrimitiveShape::Seg(SegShape {
            from: corridor.line.start_point(),
            to: corridor.line.end_point(),
            width: band_width,
        });
        for infringement in
            drawing.infringements_of_shape(shape, assign[i], corridor.maybe_net, &|_, _| true)
        {
            *congestion.entry(infringement.2).or_insert(0) += 1;
        }
    }

    AnnealPlan {
        layer_map: corridors
            .iter()
            .enumerate()
            .map(|(i, c)| (c.ratline, assign[i]))
            .collect(),
        splits: vec![],
        order: order.iter().map(|&i| corridors[i].ratline).collect(),
        congestion,
        corridors: corridors
            .iter()
            .enumerate()
            .map(|(i, c)| {
                (
                    c.ratline,
                    vec![
                        (c.line.start_point(), assign[i]),
                        (c.line.end_point(), assign[i]),
                    ],
                )
            })
            .collect(),
        initial_energy,
        final_energy: energy,
        crossings,
    }
}

/// Straight-line demand corridors plus per-layer obstruction counts.
fn build_corridors(
    autorouter: &Autorouter<impl AccessMesadata>,
    ratlines: &BTreeSet<RatlineUid>,
    band_width: f64,
    layer_count: usize,
) -> Vec<Corridor> {
    let drawing = autorouter.board().layout().drawing();
    let center = |dot: FixedDotIndex| dot.primitive_ref(drawing).shape().center();
    ratlines
        .iter()
        .map(|ratline| {
            let (origin, destination) = ratline.ref_(autorouter).endpoint_dots();
            let line = Line::new(center(origin), center(destination));
            let maybe_net = origin.primitive_ref(drawing).maybe_net();
            let shape = PrimitiveShape::Seg(SegShape {
                from: line.start_point(),
                to: line.end_point(),
                width: band_width,
            });
            let obstructions = (0..layer_count)
                .map(|layer| {
                    drawing
                        .infringements_of_shape(shape, layer, maybe_net, &|_, _| true)
                        .count()
                })
                .collect();
            Corridor {
                ratline: *ratline,
                line,
                maybe_net,
                preferred_layer: ratline.ref_(autorouter).preferred_layer(),
                obstructions,
            }
        })
        .collect()
}

/// Envelope-pruned corridor conflicts, same margin rule as
/// `MultilayerPreconfigurer::conflict_directed_layer_map`.
fn build_conflicts(corridors: &[Corridor], margin: f64) -> Conflicts {
    let envelope_of = |line: &Line<f64>| {
        let envelope = AABB::from_points([&line.start_point(), &line.end_point()]);
        let (l, u) = (envelope.lower(), envelope.upper());
        Rectangle::from_corners(
            Point::new(l.x() - margin, l.y() - margin),
            Point::new(u.x() + margin, u.y() + margin),
        )
    };
    let envelopes: Vec<_> = corridors.iter().map(|c| envelope_of(&c.line)).collect();
    let tree = RTree::bulk_load(
        envelopes
            .iter()
            .enumerate()
            .map(|(i, e)| GeomWithData::new(*e, i))
            .collect(),
    );

    let mut conflicts: Conflicts = vec![BTreeSet::new(); corridors.len()];
    for (i, corridor) in corridors.iter().enumerate() {
        for hit in tree.locate_in_envelope_intersecting(&envelopes[i].envelope()) {
            let j = hit.data;
            // Corridors of one net merge rather than conflict.
            if j == i
                || (corridor.maybe_net.is_some() && corridors[j].maybe_net == corridor.maybe_net)
            {
                continue;
            }
            if Euclidean::distance(&corridor.line, &corridors[j].line) < margin {
                conflicts[i].insert(j);
                conflicts[j].insert(i);
            }
        }
    }
    conflicts
}

/// The harder half of a conflicting pair routes first: "harder" =
/// more fixed-geometry obstruction on its assigned layer.
fn harder(i: usize, j: usize, corridors: &[Corridor], assign: &[usize]) -> (usize, usize) {
    let obstruction = |k: usize| corridors[k].obstructions[assign[k]];
    if obstruction(i) >= obstruction(j) {
        (i, j)
    } else {
        (j, i)
    }
}

fn total_energy(
    corridors: &[Corridor],
    conflicts: &Conflicts,
    assign: &[usize],
    pos: &[usize],
    options: &AnnealOptions,
    w_order: f64,
) -> f64 {
    let mut energy = 0.0;
    for i in 0..corridors.len() {
        energy += options.w_obstacle * corridors[i].obstructions[assign[i]] as f64;
        if assign[i] != corridors[i].preferred_layer {
            energy += options.w_length;
        }
        for &j in &conflicts[i] {
            if j > i && assign[i] == assign[j] {
                energy += options.w_repulsion;
                let (hard, easy) = harder(i, j, corridors, assign);
                if pos[hard] > pos[easy] {
                    energy += w_order;
                }
            }
        }
    }
    energy
}

/// ΔE of moving ratline `i` from its assigned layer to `new_layer`:
/// obstruction + preference terms plus the same-layer conflict
/// contributions of every pair containing `i`.
fn layer_delta(
    corridors: &[Corridor],
    conflicts: &Conflicts,
    assign: &[usize],
    i: usize,
    new_layer: usize,
    options: &AnnealOptions,
) -> f64 {
    let old_layer = assign[i];
    let mut de = options.w_obstacle
        * (corridors[i].obstructions[new_layer] as f64
            - corridors[i].obstructions[old_layer] as f64);
    let preferred = corridors[i].preferred_layer;
    de += options.w_length
        * ((new_layer != preferred) as i64 - (old_layer != preferred) as i64) as f64;
    for &j in &conflicts[i] {
        de += options.w_repulsion
            * ((assign[j] == new_layer) as i64 - (assign[j] == old_layer) as i64) as f64;
    }
    de
}

/// ΔE of swapping the order positions of ratlines `i` and `j`: only
/// conflicting pairs containing either can change their
/// harder-first verdict.
fn order_delta(
    corridors: &[Corridor],
    conflicts: &Conflicts,
    assign: &[usize],
    pos: &[usize],
    i: usize,
    j: usize,
    w_order: f64,
) -> f64 {
    let mut affected: BTreeSet<(usize, usize)> = BTreeSet::new();
    for &k in [i, j].iter() {
        for &l in &conflicts[k] {
            if assign[k] == assign[l] {
                affected.insert((k.min(l), k.max(l)));
            }
        }
    }

    let misordered = |a: usize, b: usize, pos: &[usize]| {
        let (hard, easy) = harder(a, b, corridors, assign);
        pos[hard] > pos[easy]
    };

    let old: f64 = affected
        .iter()
        .map(|&(a, b)| misordered(a, b, pos) as u8 as f64)
        .sum();
    let mut swapped: Vec<usize> = pos.to_vec();
    swapped.swap(i, j);
    let new: f64 = affected
        .iter()
        .map(|&(a, b)| misordered(a, b, &swapped) as u8 as f64)
        .sum();
    w_order * (new - old)
}
